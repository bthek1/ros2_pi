#!/usr/bin/env bash
#
# P3 gate: ORB keeps up with the clip, costs what it was budgeted, and finds the
# same corners the predecessor did.
#
# Three claims, measured three different ways on purpose.
#
#  1. **Rate**, from a C++ subscriber's own steady clock. Not from `header.stamp`:
#     those are the Pi's capture times, and under `ros2 bag play` they say nothing
#     whatsoever about how fast anything is running now. Not from `ros2 topic hz`
#     either — Python subscribing to 500-feature messages at 50 Hz has scheduling
#     error the size of the thing being measured.
#  2. **Per-frame cost**, from the node's own log line. A per-frame cost *is* the
#     interval between entering and leaving the work, on one clock, and the only
#     clock that sees both ends is the node's. Nothing outside it can measure this
#     without measuring the queue in front of it as well.
#  3. **Matched-keypoint fraction**, against the predecessor's algorithm over the
#     same clip — tools/orb_reference.py, a separate implementation in a different
#     language. This is the one claim that is about whether the corners *mean*
#     anything: a tracker can run at 50 Hz inside its budget while matching
#     nothing, and rate and cost both look perfect while it does.
#
# It replays a bag rather than using the camera, and that is the point of the bag:
# every phase after this one measures against the same seconds of room, so the
# numbers compare like for like. The Pi is not involved at all.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-keypoints =="

BAG_NAME=${1:-desk1}
MIN_RATE_HZ=30
MAX_COST_MS=8.0
# Five percentage points, which is the tolerance P3 was written with. The two
# implementations share OpenCV's ORB and nothing else — different language,
# different binding, the matching written from the predecessor's description rather
# than from the C++ — so agreement to five points is a statement about the pooled
# matching and the Hamming threshold, which is what it is meant to be.
MAX_MATCH_DELTA_PCT=5.0
MEASURE_S=20
# Frames the probe ignores before it starts averaging. The window takes ten frames
# to fill, so a matched fraction that includes the warm-up is partly a measurement
# of the warm-up. tools/orb_reference.py skips the same number.
WARMUP_FRAMES=15

arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

BAG=
for cand in "$BAG_NAME" "$PIMESH_WS/bags/$BAG_NAME"; do
    [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
done
[[ -n $BAG ]] || {
    echo "FAIL: no bag at '${BAG_NAME}' (looked for metadata.yaml there and under bags/)"
    echo
    echo "P3's reference clip is recorded once, with:"
    echo "  bash tools/record-clip.sh desk1 60"
    echo
    echo "bags/ is git-ignored, so a fresh clone has none — this gate cannot be"
    echo "run without one and does not pretend otherwise."
    exit 1
}
echo "clip: ${BAG}"

work=$(mktemp -d)

# --- The pipeline, fed from the clip -----------------------------------------
#
# The real launch file, so what is measured is the configuration that runs: one
# container, intra-process comms on, decode_node and keypoint_node composed. The
# static transforms come with it, and keypoint_node needs them — the optical-to-body
# basis comes from TF, not from a quaternion written into the node.
timeout -s INT $(( MEASURE_S + 40 )) ros2 launch pimesh_bringup pimesh.launch.py \
    >"$work/launch.log" 2>&1 &

for _ in $(seq 60); do
    if ros2 node list 2>/dev/null | grep -qx /keypoint_node; then break; fi
    sleep 0.5
done
ros2 node list 2>/dev/null | grep -qx /keypoint_node || {
    echo "FAIL: /keypoint_node never appeared within 30 s"
    tail -30 "$work/launch.log"
    exit 1
}

# --loop, because the measurement window may be shorter than the clip and must not
# end with it. Both halves of the terminal problem are here for the reason
# tools/replay.sh documents at length: a backgrounded `ros2 bag play` that can read
# its controlling TTY is sent SIGTTIN and stops, silently, publishing nothing.
timeout -s INT $(( MEASURE_S + 25 )) ros2 bag play "$BAG" --loop \
    --disable-keyboard-controls </dev/null >"$work/play.log" 2>&1 &

# Claim 1 and 3, from the messages themselves.
run_for $(( MEASURE_S + 30 )) ros2 run pimesh_perception keypoint_probe --ros-args \
    -p duration_s:="${MEASURE_S}.0" -p warmup_frames:="$WARMUP_FRAMES" \
    >"$work/probe.out" 2>"$work/probe.err" || true

# Claim 2, from the node's last summary line — after the window, so it covers it.
stats=$(grep -h 'stats rate=' "$work/launch.log" | tail -1 | sed 's/.*keypoint_node]: //')

kill_local
sleep 1

probe_value() {         # $1 = key
    awk -F= -v key="$1" '$1 == "probe " key {print $2}' "$work/probe.out"
}
stat_value() {          # $1 = key -> the value of key=… in the stats line
    sed -n "s/.*[[:space:]]$1=\([0-9.]*\).*/\1/p" <<<"$stats"
}

frames=$(probe_value frames)
rate=$(probe_value rate_hz)
interval_p95=$(probe_value interval_p95_ms)
keypoints=$(probe_value keypoints_mean)
matched=$(probe_value matched_fraction)
matched_p05=$(probe_value matched_p05)
malformed=$(probe_value malformed)
descriptor_bytes=$(probe_value descriptor_bytes)

if [[ -z ${frames:-} || ${frames:-0} -lt 50 ]]; then
    note "the probe saw ${frames:-0} messages on /keypoints — nothing below can be read"
    tail -5 "$work/probe.err"
    tail -20 "$work/launch.log"
    echo "FAIL gate-keypoints"
    exit 1
fi

cost=$(stat_value cost_mean)
detect=$(stat_value detect)
match_ms=$(stat_value match)
preview=$(stat_value preview)
reject_rate=$(stat_value reject_rate)
residual=$(stat_value residual)
dropped=$(stat_value dropped)

# --- Claim 1: sustained rate -------------------------------------------------
in_range "$rate" "$MIN_RATE_HZ" 1000 ||
    note "sustained ${rate} Hz on /keypoints, floor is ${MIN_RATE_HZ} Hz"

# --- Claim 2: per-frame cost, on the node's own clock ------------------------
if [[ -z ${cost:-} ]]; then
    note "keypoint_node logged no stats line — cost cannot be read from anywhere else"
else
    in_range "$cost" 0 "$MAX_COST_MS" ||
        note "mean per-frame cost ${cost} ms, budget is ${MAX_COST_MS} ms"
fi

# --- Claim 3: the corners mean something -------------------------------------
#
# The predecessor's algorithm over the *same* clip, every frame of it. Slow — this
# is Python decoding and matching a minute of 720p — and it is the only part of this
# gate that has an outside opinion about whether the tracker works.
echo
echo "-- the reference implementation over the same clip (this takes a minute) --"
ref_start=$SECONDS
/usr/bin/python3 "$PIMESH_WS/tools/orb_reference.py" "$BAG" \
    --warmup-frames "$WARMUP_FRAMES" >"$work/ref.out" 2>"$work/ref.err" || {
    note "tools/orb_reference.py failed"
    tail -5 "$work/ref.err"
}
ref_seconds=$(( SECONDS - ref_start ))
ref_value() { awk -F= -v key="$1" '$1 == "ref " key {print $2}' "$work/ref.out"; }
ref_matched=$(ref_value matched_fraction)
ref_frames=$(ref_value frames)
ref_keypoints=$(ref_value keypoints_mean)

delta_pct=
if [[ -z ${ref_matched:-} ]]; then
    note "the reference produced no matched fraction, so claim 3 was not measured"
else
    delta_pct=$(awk -v a="$matched" -v b="$ref_matched" \
        'BEGIN { d = (a - b) * 100; printf "%.2f", (d < 0) ? -d : d }')
    in_range "$delta_pct" 0 "$MAX_MATCH_DELTA_PCT" ||
        note "matched fraction ${matched} vs the reference's ${ref_matched} — \
${delta_pct} points apart, tolerance is ${MAX_MATCH_DELTA_PCT}"
fi

# --- Structural, and cheap to assert while everything is up -------------------
(( ${malformed:-1} == 0 )) ||
    note "${malformed} /keypoints messages had inconsistent array lengths"
[[ ${descriptor_bytes:-0} == 32 ]] ||
    note "descriptor_bytes is ${descriptor_bytes}, expected 32 — ORB is a 256-bit descriptor"

echo
echo "clip                : $(basename "$BAG")  ($(du -sh "$BAG" | cut -f1))"
echo "messages measured   : ${frames} over ${MEASURE_S}s, ${WARMUP_FRAMES} warm-up frames skipped"
echo "sustained rate      : ${rate} Hz  (assert >= ${MIN_RATE_HZ})"
echo "worst interval      : ${interval_p95} ms at p95  (printed: a mean hides a stall)"
echo "keypoints per frame : ${keypoints}  (cap is 500)"
echo "per-frame cost      : ${cost} ms  (assert <= ${MAX_COST_MS}, node's own clock)"
echo "  ... detection     : ${detect} ms"
echo "  ... matching      : ${match_ms} ms  (500 features against a 10-frame window)"
echo "  ... preview       : ${preview} ms  (not in the cost above: ~10 Hz, for a person)"
echo "frames dropped      : ${dropped} in the last stats window  (by design: newest wins)"
echo "matched fraction    : ${matched}  (p05 ${matched_p05})"
echo "  ... reference     : ${ref_matched:-none} over ${ref_frames:-0} frames, ${ref_keypoints:-?} kp, ${ref_seconds}s to run"
echo "  ... difference    : ${delta_pct:-not measured} points  (assert <= ${MAX_MATCH_DELTA_PCT})"
echo "pose gate           : reject rate ${reject_rate}, mean residual ${residual} rad"
echo "descriptor bytes    : ${descriptor_bytes}  (assert 32)"

(( fail == 0 )) || { echo "FAIL gate-keypoints"; exit 1; }
echo "PASS gate-keypoints"
