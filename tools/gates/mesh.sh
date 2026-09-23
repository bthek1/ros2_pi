#!/usr/bin/env bash
#
# P6 gate: a triangle surface out of the volume, without stalling the integrator.
#
# Five claims, and the first is the one the whole design of `mesh_node` is about.
#
#  1. **No dip at mesh time — measured against a control, not against a ratio.**
#     Extracting a surface costs seconds; integration has a 20 ms budget. If the
#     mesher held the volume's lock while it worked, the integrator would stop
#     dead for the duration — and a *rate* averaged over five seconds is blind to
#     that, because losing 60 frames once in ten seconds barely moves a mean. So
#     what is measured is the **maximum interval between two consecutive
#     integrations**, which a single stall cannot hide from.
#
#     **P6 asks for `max <= 2 x median` and the input does not meet that.**
#     Measured 2026-09-16: bags/desk1 produces a ~400 ms gap about 35 s in — in the
#     seventh five-second window of every run, at 13.6, 14.4 and 14.6 Hz — and it
#     is there in runs recorded *before mesh_node existed at all*, with depth_node's
#     own per-frame cost unchanged at 55.9 ms throughout. It is the clip or the
#     player, not the mesher. A gate asserting a 2x ratio would have failed on it
#     and the finger would have pointed at the wrong node.
#
#     So the second run is a **control**: the same clip, the same container, one
#     parameter apart — `remesh_period_s` set past the length of the clip, so
#     nothing meshes while the integrator is measured. The assertion is that the
#     worst gap with meshing is no worse than the worst gap without it. That is
#     the claim P6 actually wants, and unlike a ratio it cannot be satisfied or
#     broken by how steady the bag happens to be.
#  2. **The published Marker is under the cap**, read off the topic itself rather
#     than off what the node said about it.
#  3. **No pinholes, and the frontier still open.** Two assertions that pull in
#     opposite directions and are both required: the boundary-loop count has to
#     *fall* across the fill (holes were closed) and it has to stay **above zero**
#     (the edge of what the camera saw was not). A mesh with no boundary at all is
#     a sealed box, and a sealed box looks *more* finished than a correct scan —
#     it is the most seductive false positive in this project.
#  4. **The saved file is full detail.** `/world/save_mesh` writes the surface
#     before the Marker's decimation, so the PLY must have *more* triangles than
#     the topic. Saving the decimated one would quietly throw away the detail the
#     volume paid sixty seconds to accumulate, and nothing would look wrong.
#  5. **Three renders exist and have a surface in them.** `tools/eval/mesh-views.sh`
#     writes them offscreen and reports what fraction of each frame the surface
#     covers. Asserted rather than merely written: an empty mesh, a camera inside
#     the geometry and a sign error in the projection all produce a black
#     rectangle, and a gate that only checked the file existed would pass over all
#     three.
#
# **Those PNGs are the evidence and the RViz window is not.** `just view-mesh` is
# for a person.
#
# It replays bags/desk1, like every phase from P3 on. The Pi is not involved.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-mesh =="

BAG_NAME=${1:-desk1}

# How much worse the worst gap may be with the mesher running than without it.
# Not 1.0: both runs see the clip's own ~400 ms stall, and which five-second window
# it lands in shifts by a frame or two between runs. A mesher holding the volume's
# lock would be several times worse than this, not a quarter.
MAX_GAP_RATIO=1.25
# And an absolute ceiling, so a control run that was itself terrible cannot license
# a terrible measurement run. ~9 frame intervals.
MAX_GAP_MS=500
# The Marker cap, from config/pimesh.yaml. Asserted with a little slack above,
# because decimation stops at the first collapse that takes it under the cap and
# can land a triangle or two either side.
MAX_MARKER_TRIANGLES=120010
# A mesh has to actually exist. Not a floor anybody should tune: it is here so
# that "the surface is empty" fails rather than passing every other assertion
# vacuously.
MIN_TRIANGLES=5000
# What fraction of a render has to be surface rather than background.
MIN_COVERAGE=0.02
# How long to wait after the clip for one more extraction, so what is measured is
# a surface built from the whole clip rather than from most of it.
SETTLE_S=25

assert_no_session "bash tools/gates/mesh.sh"

arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || {
    echo "FAIL: no model at ${MODEL}"
    echo "  bash tools/fetch-model.sh"
    exit 1
}

BAG=
for cand in "$BAG_NAME" "$PIMESH_WS/bags/$BAG_NAME"; do
    [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
done
[[ -n $BAG ]] || {
    echo "FAIL: no bag at '${BAG_NAME}' (looked for metadata.yaml there and under bags/)"
    echo
    echo "The reference clip is recorded once, with:"
    echo "  bash tools/record-clip.sh desk1 60"
    echo
    echo "bags/ is git-ignored, so a fresh clone has none — this gate cannot be"
    echo "run without one and does not pretend otherwise."
    exit 1
}

work=$(mktemp -d)
log="$work/run.log"
ply="$work/gate_mesh.ply"

CLIP_SECONDS=$(/usr/bin/python3 - "$BAG/metadata.yaml" <<'META'
import sys
import yaml

with open(sys.argv[1]) as handle:
    info = yaml.safe_load(handle)['rosbag2_bagfile_information']
print(f"{info['duration']['nanoseconds'] / 1e9:.0f}")
META
)
echo "clip : ${BAG}  (${CLIP_SECONDS}s)"

# --- One run of the real pipeline --------------------------------------------
timeout -s INT $(( CLIP_SECONDS + SETTLE_S + 60 )) \
    ros2 launch pimesh_bringup pimesh.launch.py >"$log" 2>&1 &

ready=0
for _ in $(seq 240); do
    if grep -q "mesh_node up:" "$log" 2>/dev/null; then ready=1; break; fi
    sleep 0.5
done
(( ready == 1 )) || {
    echo "FAIL: mesh_node never came up within 120 s"
    tail -40 "$log"
    echo "FAIL gate-mesh"
    exit 1
}

# Once, not --loop: a looping bag replays header stamps a minute into the past at
# every wrap, and fusion_node's pose lookup at the frame's own stamp then fails
# for the rest of the run — so there would be nothing to mesh. See tools/view/replay.sh
# for the mechanism. The redirect is for the same reason it is everywhere else: a
# backgrounded player that can read its terminal is stopped by SIGTTIN, silently.
timeout -s INT $(( CLIP_SECONDS + 20 )) ros2 bag play "$BAG" \
    --disable-keyboard-controls </dev/null >"$log.play" 2>&1 &

sleep $(( CLIP_SECONDS + 4 ))

# One more extraction after the clip, so the surface measured is the whole room
# rather than whatever had accumulated when the last timer happened to fire.
echo "-- clip finished; waiting ${SETTLE_S}s for a final extraction --"
sleep "$SETTLE_S"

# --- Claim 2, from the topic rather than from the node's own opinion ----------
#
# `--no-arr` prints each array as its length instead of its contents, which is the
# difference between a number and 360 000 lines of points. It renders as
# `points: '<sequence type: geometry_msgs/msg/Point, length: 360000>'` — **a colon
# and a space, not an equals sign**, which the first version of this line guessed
# wrong. The gate then reported "nothing was published on /world/mesh" over a run
# that had published eight surfaces, which is the same shape as the prefix bug
# tools/gates/depth.sh records: a pattern that matches nothing reads as a
# measurement of zero, and zero looks like a finding rather than like a broken
# gate. This is one message read
# once, not a rate — so a Python subscriber is honest here in a way `ros2 topic hz`
# would not be: there is no scheduling error to land in a count.
# **`--qos-durability transient_local`, and without it this reads nothing.** The
# publisher is latched precisely so a late subscriber gets the current surface, and
# a *volatile* reader against a transient-local writer is compatible — it simply
# does not get the stored message, and then waits for the next one. Measured: the
# echo returned empty and the gate reported "nothing was published on /world/mesh"
# over a run that had published eight surfaces. A QoS mismatch that is legal is
# worse than one that is not, because nothing anywhere says the two disagree.
marker_points=$(timeout 30 ros2 topic echo --once --no-arr \
    --qos-durability transient_local --qos-reliability reliable /world/mesh 2>/dev/null |
    sed -n "s/.*points:.*length: \\([0-9]*\\).*/\\1/p" | head -1)

# --- Claim 4: save the full-detail surface ------------------------------------
save_out=$(timeout 60 ros2 service call /world/save_mesh pimesh_msgs/srv/SaveMesh \
    "{path: '${ply}'}" 2>&1 || true)
saved_triangles=$(sed -n 's/.*triangles=\([0-9]*\).*/\1/p' <<<"$save_out" | head -1)

kill_local
sleep 1

# --- The node's own log, by node name ----------------------------------------
#
# Grepped by node name and taking the last *complete* extraction, for the reason
# tools/gates/keypoints.sh paid for: four nodes in this container log a summary,
# and `grep 'stats' | tail -1` silently started reading a different node's idle
# window the day one was added.
last_mesh=$(grep -h 'mesh_node' "$log" | grep -o 'mesh blocks=.*' | tail -1)
field() { sed -n "s/.*[^a-z_]$1=\([0-9.-]*\).*/\1/p" <<<"$last_mesh" | head -1; }

blocks=$(sed -n 's/^mesh blocks=\([0-9]*\).*/\1/p' <<<"$last_mesh")
voxels_meshed=$(field voxels_meshed)
raw_tri=$(field raw_tri)
pruned=$(field pruned)
filled=$(sed -n 's/.*filled=\([0-9]*\)\/.*/\1/p' <<<"$last_mesh")
loops_before=$(sed -n 's/.*loops=\([0-9]*\)->.*/\1/p' <<<"$last_mesh")
loops_after=$(sed -n 's/.*loops=[0-9]*->\([0-9]*\).*/\1/p' <<<"$last_mesh")
collapses=$(field collapses)
final_tri=$(field tri)
final_vert=$(field vert)
mesh_total_ms=$(field total)
extractions=$(grep -hc 'mesh blocks=' "$log" || true)

if [[ -z ${last_mesh:-} ]]; then
    note "mesh_node never completed an extraction — nothing below can be read"
    grep -iE 'error|died|refused|waiting' "$log" | head -10
    tail -30 "$log"
    echo "FAIL gate-mesh"
    exit 1
fi

# --- Claim 1: the integrator did not dip -------------------------------------
#
# Read off fusion_node's windows, and only those where the clip was actually
# playing: the worst gap over the run and the median interval it sits against.
# The floor is a rate rather than "more than no frames" for the reason
# tools/gates/fusion.sh spells out at length — the last window of every run is the
# idle tail after the clip, and its numbers are about nothing.
RUNNING_RATE_HZ=5
worst_gap_of() {         # $1 = log; prints "max median ratio windows"
    grep -h 'fusion_node' "$1" | grep -o 'stats rate=.*' |
        awk -v floor="$RUNNING_RATE_HZ" '{
            delete v
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); v[kv[1]] = kv[2] }
            if (v["rate"] + 0 < floor || v["interval_p50"] + 0 <= 0) { next }
            n++
            median += v["interval_p50"]
            if (v["interval_max"] + 0 > mx) { mx = v["interval_max"] + 0 }
        }
        END {
            if (n == 0) { print "0 0 0 0"; exit }
            printf "%.1f %.1f %.2f %d\n", mx, median / n, mx / (median / n), n
        }'
}

read -r worst_max worst_median worst_ratio windows < <(worst_gap_of "$log")

# --- Claim 2 -----------------------------------------------------------------
if [[ -z ${marker_points:-} ]]; then
    note "nothing was published on /world/mesh — RViz would show an empty 3D view, \
which looks exactly like a dead topic"
    marker_triangles=0
else
    marker_triangles=$(( marker_points / 3 ))
    (( marker_triangles <= MAX_MARKER_TRIANGLES )) ||
        note "the published Marker carries ${marker_triangles} triangles, over the \
${MAX_MARKER_TRIANGLES} cap — decimation is not reaching it"
    (( marker_triangles >= MIN_TRIANGLES )) ||
        note "the published Marker carries only ${marker_triangles} triangles"
fi

# --- Claim 3: pinholes closed, frontier open ---------------------------------
(( ${loops_after:-0} < ${loops_before:-0} )) ||
    note "boundary loops went ${loops_before:-?} -> ${loops_after:-?} — the fill closed \
nothing, so every pinhole marching cubes left is still there"
(( ${loops_after:-0} > 0 )) ||
    note "**the surface has no boundary at all after filling.** Every component's \
largest loop is the edge of what the camera saw and must stay open; a sealed box \
is unseen space invented, and it looks more finished than a correct scan"
(( ${filled:-0} > 0 )) || note "no interior holes were filled at all"

# --- The weight threshold is doing something ---------------------------------
voxels_allocated=$(( ${blocks:-0} * 512 ))
(( ${voxels_meshed:-0} > 0 )) || note "no voxel passed the meshing weight threshold"
(( ${voxels_meshed:-0} < voxels_allocated )) ||
    note "every allocated voxel passed the meshing threshold, so the threshold is \
doing nothing and the surface is being built out of the noise floor"

# --- Claim 4: the saved file is the full-detail one ---------------------------
if [[ ! -r $ply ]]; then
    note "/world/save_mesh wrote nothing at ${ply} — service said: $(tr '\n' ' ' <<<"$save_out" | cut -c1-200)"
    saved_triangles=0
else
    (( ${saved_triangles:-0} > marker_triangles )) ||
        note "the saved PLY has ${saved_triangles:-0} triangles against the Marker's \
${marker_triangles} — the file is supposed to be the surface *before* decimation, and \
saving the capped one throws away the detail the volume spent a minute accumulating"
fi

# --- Claim 5: the renders ------------------------------------------------------
echo
echo "-- offscreen renders --"
renders=0
worst_coverage=1.0
if [[ -r $ply ]]; then
    render_out=$(bash "$PIMESH_WS/tools/eval/mesh-views.sh" "$ply" "$work/views" desk 2>&1) || true
    echo "$render_out"
    while read -r path coverage; do
        [[ -n $path ]] || continue
        renders=$(( renders + 1 ))
        awk -v a="$coverage" -v b="$worst_coverage" 'BEGIN { exit !(a < b) }' &&
            worst_coverage=$coverage
    done < <(sed -n 's/^render [a-z]* \(.*\) coverage=\(.*\)$/\1 \2/p' <<<"$render_out")

    (( renders == 3 )) || note "mesh-views.sh wrote ${renders} renders, not 3"
    awk -v c="$worst_coverage" -v m="$MIN_COVERAGE" 'BEGIN { exit !(c >= m) }' ||
        note "the emptiest render is ${worst_coverage} surface (floor ${MIN_COVERAGE}) — \
an empty mesh, a camera inside the geometry and a sign error in the projection all \
produce a black rectangle, and this is what tells them from a picture"
else
    note "no saved mesh to render"
fi

# =============================================================================
# Run 2 — the control: the same clip with nothing meshing
# =============================================================================
#
# Without this the gap above is a number with nothing to compare it to, and the
# clip's own 400 ms stall reads as the mesher's doing. One parameter apart.
echo
echo "-- run 2: the control, remesh_period_s past the clip, so nothing meshes --"
control_log="$work/control.log"
timeout -s INT $(( CLIP_SECONDS + 60 )) \
    ros2 launch pimesh_bringup pimesh.launch.py remesh_period_s:=600.0 \
    >"$control_log" 2>&1 &

ready=0
for _ in $(seq 240); do
    if grep -q "mesh_node up:" "$control_log" 2>/dev/null; then ready=1; break; fi
    sleep 0.5
done
(( ready == 1 )) || { echo "FAIL: the control run never came up"; echo "FAIL gate-mesh"; exit 1; }

timeout -s INT $(( CLIP_SECONDS + 20 )) ros2 bag play "$BAG" \
    --disable-keyboard-controls </dev/null >"$control_log.play" 2>&1 &
sleep $(( CLIP_SECONDS + 4 ))
kill_local
sleep 1

read -r c_max c_median c_ratio c_windows < <(worst_gap_of "$control_log")
control_extractions=$(grep -hc 'mesh blocks=' "$control_log" || true)

(( ${control_extractions:-0} == 0 )) ||
    note "the control run meshed ${control_extractions} times — remesh_period_s did not reach the node, so it is not a control"

if (( ${windows:-0} < 3 || ${c_windows:-0} < 3 )); then
    note "too few usable windows (${windows:-0} measured, ${c_windows:-0} control) for the dip comparison"
else
    awk -v a="$worst_max" -v b="$c_max" -v r="$MAX_GAP_RATIO" \
        'BEGIN { exit !(a <= b * r) }' ||
        note "the worst gap between two integrations was ${worst_max} ms with the mesher running against ${c_max} ms without it — more than ${MAX_GAP_RATIO}x worse, so extraction is stalling the integrator"
    awk -v a="$worst_max" -v lim="$MAX_GAP_MS" 'BEGIN { exit !(a <= lim) }' ||
        note "the worst gap was ${worst_max} ms, over the ${MAX_GAP_MS} ms ceiling — a control that is equally bad does not make this acceptable"
fi

echo
echo "clip                : $(basename "$BAG")  (${CLIP_SECONDS}s)"
echo "extractions         : ${extractions}  (one every 10 s, plus one after the clip)"
echo
echo "integrator, at mesh time — measured against a control"
echo "  ... meshing       : worst gap ${worst_max} ms, median interval ${worst_median} ms = ${worst_ratio}x  (${windows} windows)"
echo "  ... control       : worst gap ${c_max} ms, median interval ${c_median} ms = ${c_ratio}x  (${c_windows} windows, ${control_extractions} extractions)"
echo "  ... assert        : ${worst_max} <= ${c_max} x ${MAX_GAP_RATIO}, and <= ${MAX_GAP_MS} ms"
echo "                      P6 asks for max <= 2x median; the *input* does not meet"
echo "                      that. bags/desk1 stalls ~400 ms about 35 s in, in runs"
echo "                      recorded before mesh_node existed. Hence a control."
echo
echo "the last extraction"
echo "  ... volume        : ${blocks} blocks = ${voxels_allocated} voxels, ${voxels_meshed} above the meshing weight  (assert 0 < meshed < allocated)"
echo "  ... marching cubes: ${raw_tri} triangles"
echo "  ... debris pruned : ${pruned} components under 30 triangles"
echo "  ... holes filled  : ${filled} of ${loops_before} boundary loops"
echo "  ... loops left    : ${loops_after}  (assert < ${loops_before} and > 0: pinholes closed, frontier open)"
echo "  ... decimation    : ${collapses} edge collapses -> ${final_tri} triangles, ${final_vert} vertices"
echo "  ... cost          : ${mesh_total_ms} ms, off the integration path entirely"
echo
echo "published / saved"
echo "  ... /world/mesh   : ${marker_triangles} triangles  (assert <= ${MAX_MARKER_TRIANGLES}, read off the topic)"
echo "  ... saved PLY     : ${saved_triangles:-0} triangles at ${ply}  (assert > the Marker's: full detail)"
echo "  ... renders       : ${renders} of 3, emptiest ${worst_coverage} surface  (assert >= ${MIN_COVERAGE})"
if (( renders > 0 )); then
    echo
    echo "**These are the evidence, not the RViz window:**"
    ls -1 "$work/views"/*.png 2>/dev/null | sed 's/^/  /'
fi
echo
echo "  What they will *not* show is a room. odometry_node publishes rotation"
echo "  only, so a hand-held sweep's ~0.9 m of real arm arc is modelled as zero"
echo "  and the same wall is integrated at a different distance every time the"
echo "  camera moves — the surface is a shell at roughly constant radius rather"
echo "  than a desk with a wall behind it. P7 is what fixes that, and the"
echo "  scale is arbitrary until a tape measure pins depth_scale. This gate"
echo "  asserts that a surface is extracted correctly and cheaply from whatever"
echo "  the volume holds; it does not assert that the volume is a room."

(( fail == 0 )) || { echo "FAIL gate-mesh"; exit 1; }
echo "PASS gate-mesh"
