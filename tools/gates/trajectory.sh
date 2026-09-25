#!/usr/bin/env bash
#
# P11 gate: the first number in this project about the pose that does not come
# out of this project.
#
# Everything P0-P10 measured about the trajectory was measured by the thing that
# produced it — a PnP reprojection residual, a paired-surface gap, a step size.
# P7 is the demonstration of how far that gets you: the rotation had been composed
# inverted since P3, for six days, and *every* internal number describing it was
# correct. So this gate replays a public sequence with motion-capture ground truth
# through the real container and hands the result to `evo`, which is somebody
# else's ATE and is the point.
#
# --- The unit problem, and why two ATEs are printed --------------------------
#
# Monocular depth is scale-ambiguous and `depth_scale` is 10.0 because somebody
# typed it, so an **SE(3)**-aligned ATE on a metric dataset is very largely a
# measurement of that constant. `evo_ape --align --correct_scale` fits a Sim(3)
# and reports the trajectory's *shape*, which is the thing this pipeline can
# currently be right about. Both are printed; the Sim(3) one is asserted.
#
# The fitted scale is printed too, and it is worth more than the assertion: on a
# dataset with metric ground truth **it is `depth_scale`'s answer, obtained
# without a tape measure**. It does not transfer to the C922 in this room — a
# different camera, a different scene, and Depth Anything's scale is per-image —
# but it bounds it, and it turns P12 from the only source of that number into a
# check on one that already exists.
#
# --- The three false greens this phase can produce, and what each cost --------
#
#  1. **`--correct_scale` hides scale drift.** It fits *one* scale over the whole
#     trajectory, so a system whose scale walks smoothly still aligns to a small
#     ATE. RPE over a 1 s window is reported beside it, because a relative error
#     is sensitive to exactly what an ATE-after-Sim(3) absorbs.
#
#  2. **`camera_info` not swapped with the frames.** fr1/desk is 640x480 with
#     Freiburg's intrinsics; serving the C922's fx=953.4 over it makes every
#     unprojection wrong by ~1.5x and the resulting ATE a measurement of our
#     calibration against somebody else's room. This is asserted twice over: the
#     intrinsics are read **off the wire** and compared against the dataset's own
#     YAML, and a *control* run starts `dataset_node` with the C922's calibration
#     and requires it to refuse — the size mismatch is a startup failure, not a
#     number.
#
#  3. **A second publisher on `/image_raw/compressed`.** `dataset_node` is a third
#     source on the topic the Pi's camera and `ros2 bag play` already use, which is
#     the one-session rule seen from the producing end: one `decode_node`
#     interleaving two sources produces a pose whose stamps jump minutes back and
#     forth, with *neither* session doing anything wrong. `assert_no_session`
#     refuses to start beside one, and `source:=` is a name rather than a flag so
#     two sources cannot be loaded at once.
#
# --- And one this gate found ---------------------------------------------------
#
# **The written trajectory is the camera's optical pose, not base_link's.**
# `/odom` is the REP-103 body frame; TUM's ground truth is the colour camera's
# optical frame. The two differ by a constant rotation and no translation, so the
# ATE is *identical* either way and the RPE is not. Measured 2026-09-25 by
# rotating TUM's own ground truth by that constant and scoring it against itself:
# **0.654 m RPE over a 1 s window for a trajectory that is exactly right.** The
# first run of this pipeline reported 0.768 m in the body frame and 0.150 m in the
# optical one — five times, essentially all of it the convention. An ATE would
# never have shown it.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-trajectory =="

# --- The budgets -------------------------------------------------------------
#
# **The ceiling sits between two measurements**, which is what makes it an
# assertion rather than a round number. Sim(3)-aligned ATE RMSE on fr1/desk,
# 2026-09-25: 0.2958 m and 0.3475 m over two runs of the code as it stands. The
# rotation_only control cannot be aligned at all — with translation identically
# zero the estimate's covariance is rank-deficient and Umeyama has nothing to fit
# — but the number it *would* have is computable from the ground truth alone: an
# estimate collapsed to a point aligns to its centroid, so the ATE is the RMS
# distance of the truth from its own centroid, **0.8559 m** over fr1/desk's 2335
# poses. 0.60 sits between 0.35 and 0.86 with room on both sides for a clip this
# short.
MAX_ATE_M=0.60
# Printed, not asserted, and the header says why: one run of a number nobody has
# a second opinion on. Measured 0.1503 m optical-frame, against 0.7679 m for the
# same trajectory in the body frame — which is the pair that made it worth
# reporting at all.
RPE_WINDOW_S=1.0
# The trajectory has to be a trajectory. fr1/desk is 613 frames over 20.4 s and
# depth runs at ~18 Hz on 640x480, so ~360 poses is the whole clip; 250 leaves
# room for a slow start without admitting a run that posed a third of it.
MIN_POSES=250
# `evo` associates our stamps against the dataset's own, and it should find all of
# them: our stamps *are* the dataset's. A shortfall means the stamps have been
# through a float somewhere.
MIN_ASSOCIATED_PCT=99
# Depth frames that got a pose rather than holding the last one, as in
# gates/odom.sh. Lower than that gate's 55% would mean a clip this estimator
# cannot track.
MIN_POSED_PCT=70
# Mean inlier reprojection, in pixels. Same budget as gates/odom.sh, and the same
# reason for the unit: the metres here are arbitrary until P12.
MAX_REPROJ_PX=2.0

# Before arm_cleanup, always: the cleanup handler kills this workspace's
# processes, so a refusal after the trap is armed would tear down the session it
# is refusing to disturb.
assert_no_session "bash tools/gates/trajectory.sh"

arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

PY=/usr/bin/python3
EVO_APE=${EVO_APE:-$HOME/.local/bin/evo_ape}
EVO_RPE=${EVO_RPE:-$HOME/.local/bin/evo_rpe}

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || {
    echo "FAIL: no model at ${MODEL}"
    echo
    echo "models/ is git-ignored, so a fresh clone has none. Fetch it with:"
    echo "  bash tools/fetch-model.sh"
    exit 1
}

SEQ=$(bash "$PIMESH_WS/tools/fetch-dataset.sh" --print-path)
GROUND_TRUTH="$SEQ/groundtruth.txt"
[[ -r $GROUND_TRUTH ]] || {
    echo "FAIL: no dataset at ${SEQ}"
    echo
    echo "It lives outside the workspace and is not committed. Fetch it with:"
    echo "  bash tools/fetch-dataset.sh"
    echo
    echo "This gate cannot be run without it and does not pretend otherwise —"
    echo "there is no internal number it could fall back to, which is the whole"
    echo "point of the phase."
    exit 1
}

[[ -x $EVO_APE && -x $EVO_RPE ]] || {
    echo "FAIL: no evo at ${EVO_APE}"
    echo
    echo "The ATE is computed by somebody else's tool on purpose — writing our own"
    echo "would be writing the instrument and the thing it measures in the same"
    echo "afternoon. Install it the way shellcheck is installed:"
    echo "  uv tool install evo"
    exit 1
}

# The two calibrations: the dataset's, and the camera's, which is the control.
DATASET_CAL="$PIMESH_WS/src/pimesh_bringup/config/camera_info/tum_freiburg1.yaml"
CAMERA_CAL=package://pimesh_bringup/config/camera_info/c922_720p.yaml

work=$(mktemp -d)

read -r SEQ_FRAMES SEQ_SECONDS < <("$PY" - "$SEQ/rgb.txt" <<'META'
import sys
stamps = []
with open(sys.argv[1]) as handle:
    for line in handle:
        line = line.strip()
        if line and not line.startswith('#'):
            stamps.append(float(line.split()[0]))
print(f"{len(stamps)} {stamps[-1] - stamps[0]:.1f}")
META
)
MEASURE_S=$(awk -v s="$SEQ_SECONDS" 'BEGIN { printf "%d", s + 8 }')

# **What the ceiling has to exclude, computed from the ground truth and nothing
# else.** The rotation_only control publishes translation identically zero, so
# Umeyama has a rank-deficient covariance and `evo` refuses to align it at all —
# a hard refusal, and a stronger outcome than a large number, but not a *number*.
# This is the number: an estimate collapsed to a point aligns to the truth's
# centroid, so its ATE is the RMS distance of the truth from that centroid. It is
# derived here rather than written down so that it re-derives for any sequence
# somebody points this gate at, and so that MAX_ATE_M being below it is an
# assertion rather than a sentence in a comment.
STATIONARY_ATE_M=$("$PY" - "$GROUND_TRUTH" <<'SPREAD'
import sys
rows = [line.split() for line in open(sys.argv[1]) if not line.startswith('#')]
n = len(rows)
xs = [[float(r[i]) for r in rows] for i in (1, 2, 3)]
mean = [sum(c) / n for c in xs]
print("%.4f" % ((sum((v - m) ** 2 for c, m in zip(xs, mean) for v in c) / n) ** 0.5))
SPREAD
)

echo "sequence : ${SEQ}"
echo "           ${SEQ_FRAMES} frames, ${SEQ_SECONDS}s, ground truth $(grep -cv '^#' "$GROUND_TRUTH") poses"
echo "model    : ${MODEL}"
echo "evo      : ${EVO_APE}"

# =============================================================================
# Control 0 — the camera's own calibration over the dataset's frames must refuse
# =============================================================================
#
# **A budget nobody has seen fail is not an assertion, and neither is a refusal.**
# This is the cheapest control in the workspace: no container, no GPU, one node
# started standalone for a second with the wrong `camera_info_url`. If it ever
# starts, the protection against #10's second false green has gone and the ATE
# below would silently become a measurement of our calibration against Freiburg's
# room.
echo
echo "-- control 0: the C922's calibration over 640x480 frames must be refused --"
set +e
timeout --foreground -s INT 20 ros2 run pimesh_dataset dataset_node --ros-args \
    -p dataset_dir:="$SEQ" -p max_frames:=2 \
    -p camera_info_url:="$CAMERA_CAL" >"$work/wrongcal.log" 2>&1
wrongcal_status=$?
set -e
if (( wrongcal_status == 0 )); then
    note "dataset_node started with the C922's 1280x720 calibration over 640x480 frames"
elif ! grep -qiE '1280|720|width|height|resolution' "$work/wrongcal.log"; then
    note "dataset_node refused the wrong calibration but not for the resolution — the refusal may be about something else entirely"
    tail -5 "$work/wrongcal.log"
else
    echo "   refused, exit ${wrongcal_status}: $(grep -oE 'is unusable:.*|refusing to start:.*' "$work/wrongcal.log" | head -1 | cut -c1-140)"
fi

# --- One run of the real pipeline in one regime ------------------------------
#
# The real launch file, so what is measured is the configuration that runs, with
# `dataset_node` and `odom_probe` loaded into the same container. No `ros2 bag
# play` anywhere: the source *is* a component, which is what lets it serve the
# dataset's intrinsics beside the dataset's frames.
run_regime() {           # $1 = log path, $2 = regime, $3 = trajectory path
    local log=$1 regime=$2 traj=$3

    timeout -s INT $(( MEASURE_S + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" \
        odom_regime:="$regime" probe:=odom_probe \
        probe_duration_s:="$(awk -v s="$MEASURE_S" 'BEGIN { printf "%.1f", s + 4 }')" \
        trajectory_path:="$traj" \
        >"$log" 2>&1 &

    # Wait on the launcher's own log line, not on `ros2 node list`: the daemon
    # caches discovery state and a stale cache has already made one gate in this
    # repo fail while the log beside it said the node was loaded.
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset finished" "$log" 2>/dev/null; then ready=2; break; fi
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready > 0 )) || {
        echo "FAIL: dataset_node never came up within 60 s"
        tail -40 "$log"
        return 1
    }

    # **The intrinsics, off the wire, while the run is happening.** Not off
    # dataset_node's own startup line, which is the node agreeing with itself:
    # `/camera_info` is what every consumer in the container actually reads, and
    # transient_local is what it is published with — a volatile reader here would
    # simply wait forever and the gate would report nothing rather than a
    # mismatch.
    if [[ ! -s $work/served_k.txt ]]; then
        timeout --foreground 20 ros2 topic echo --once \
            --qos-durability transient_local --qos-reliability reliable \
            --field k /camera_info >"$work/served_k.txt" 2>/dev/null || true
        timeout --foreground 20 ros2 topic echo --once \
            --qos-durability transient_local --qos-reliability reliable \
            --field width /camera_info >"$work/served_w.txt" 2>/dev/null || true
    fi

    sleep $(( MEASURE_S + 12 ))
    kill_local
    sleep 1
}

probe_value() {          # $1 = log, $2 = key
    grep -h 'odom_probe' "$1" | grep -o 'odom_probe result .*' |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { last = kv[2] } }
        } END { print (last == "") ? "" : last }'
}

# The last odometry_node window that had frames in it — `rate > 0` is not enough,
# because the last window of a run covers the idle seconds after the clip ended.
# The correction gates/fusion.sh paid for.
odom_value() {           # $1 = log, $2 = key
    grep -h 'odometry_node' "$1" | grep -o 'stats regime=.*' |
        awk -v key="$2" '{
            delete v
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); v[kv[1]] = kv[2] }
            rate = v["rate"]; sub(/Hz$/, "", rate)
            if (rate + 0 < 5) { next }
            last = v[key]
        } END { print (last == "") ? "" : last }'
}

# --- evo, and the parsing of it ----------------------------------------------
#
# Three numbers out of two tools. `-v` is what makes `evo_ape` print the fitted
# scale and the association count, both of which are assertions here rather than
# decoration: a run that associated a fifth of its poses would otherwise report a
# small ATE over the fifth it managed.
ate_of() {               # $1 = trajectory, $2 = "sim3" | "se3"; prints "rmse scale compared"
    local traj=$1 mode=$2 extra=()
    [[ $mode == sim3 ]] && extra=(-s)
    "$EVO_APE" tum "$GROUND_TRUTH" "$traj" -a "${extra[@]}" -v 2>&1 |
        awk '
            /Scale correction:/ { scale = $3 }
            /absolute pose pairs/ { compared = $2 }
            /Degenerate covariance/ { degenerate = 1 }
            $1 == "rmse" { rmse = $2 }
            END {
                printf "%s %s %s %s\n",
                    (rmse == "") ? "none" : rmse,
                    (scale == "") ? "1.0" : scale,
                    (compared == "") ? "0" : compared,
                    (degenerate == 1) ? "degenerate" : "ok"
            }'
}

rpe_of() {               # $1 = trajectory, $2 = delta in frames
    "$EVO_RPE" tum "$GROUND_TRUTH" "$1" -a -s --delta "$2" --delta_unit f --all_pairs 2>&1 |
        awk '$1 == "rmse" { rmse = $2 } END { print (rmse == "") ? "none" : rmse }'
}

# =============================================================================
# Run 1 — 6-DoF, which is what is being measured
# =============================================================================
echo
echo "-- run 1: odom_regime:=sixdof over the whole sequence --"
run_regime "$work/six.log" sixdof "$work/six.tum" || { echo "FAIL gate-trajectory"; exit 1; }

six_frames=$(probe_value "$work/six.log" frames)
six_rate=$(probe_value "$work/six.log" rate)
six_path=$(probe_value "$work/six.log" path_m)
six_speed_max=$(probe_value "$work/six.log" speed_max)
six_reproj=$(odom_value "$work/six.log" reproj_px)
six_inliers=$(odom_value "$work/six.log" inliers)
six_shift_ok=$(odom_value "$work/six.log" shift_ok)
six_shift_held=$(odom_value "$work/six.log" shift_held)
six_implausible=$(odom_value "$work/six.log" implausible)
six_published=$(grep -oE 'dataset finished: [0-9]+ frames' "$work/six.log" | grep -oE '[0-9]+' | head -1)
# Poses odom_probe saw but could not put in the optical frame, because the static
# edge was not in its TF buffer yet. A shortfall can only be a leading one — the
# edge is static and the rotation is cached the first time it resolves — so this
# is "the trajectory silently starts later than the run did", which is a smaller
# ATE over less of the clip and looks like an improvement.
six_skipped=$(grep -oE 'odom_probe trajectory .*skipped=[0-9]+' "$work/six.log" |
    grep -oE 'skipped=[0-9]+' | cut -d= -f2 | tail -1)
six_late=$(grep -oE 'dataset finished: [0-9]+ frames published, [0-9]+ behind' "$work/six.log" |
    awk '{print $(NF-1)}')

if [[ ! -s $work/six.tum ]]; then
    note "no trajectory was written — odom_probe refused it or saw no poses"
    grep -iE 'odom_probe|refus|error' "$work/six.log" | tail -10
    echo "FAIL gate-trajectory"
    exit 1
fi
six_poses=$(grep -cv '^#' "$work/six.tum")

read -r six_ate_se3 _ _ _ < <(ate_of "$work/six.tum" se3)
read -r six_ate_sim3 six_scale six_cmp_sim3 six_degen < <(ate_of "$work/six.tum" sim3)
# The RPE window, in frames, from the pose rate this run actually achieved —
# `evo_rpe` has no seconds unit, and a constant frame count would be a different
# window every time the pipeline's rate moved.
rpe_delta=$(awk -v r="$six_rate" -v w="$RPE_WINDOW_S" 'BEGIN { d = int(r * w + 0.5); print (d < 2) ? 2 : d }')
six_rpe=$(rpe_of "$work/six.tum" "$rpe_delta")

# --- Claim 1: the intrinsics on the wire are the dataset's --------------------
#
# **#10's second false green.** Read off /camera_info rather than off the node's
# own log, and compared against the YAML by a reader that has never heard of
# either — the same argument for reading both files as text that
# test_dashboard_contract makes. Checked here, between the runs, because it is a
# fact about run 1 and reading it after run 2 would leave a reader pairing an
# assertion with the wrong run.
echo
echo "-- the intrinsics that reached the pipeline --"
if [[ ! -s $work/served_k.txt ]]; then
    note "nothing was served on /camera_info — the assertion about which intrinsics reached the pipeline could not be made at all, which is not the same as it passing"
else
    if ! "$PY" - "$work/served_k.txt" "$DATASET_CAL" <<'CHECK'
import sys
import re
import yaml

served = [float(x) for x in re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', open(sys.argv[1]).read())]
want = yaml.safe_load(open(sys.argv[2]))['camera_matrix']['data']
if len(served) != 9:
    print(f"   /camera_info K had {len(served)} numbers in it, not 9")
    sys.exit(1)
bad = [(i, a, b) for i, (a, b) in enumerate(zip(served, want)) if abs(a - b) > 1e-6]
for i, a, b in bad:
    print(f"   K[{i}] served {a}, {sys.argv[2]} says {b}")
print(f"   served fx={served[0]:.4f} fy={served[4]:.4f} "
      f"cx={served[2]:.4f} cy={served[5]:.4f}")
sys.exit(1 if bad else 0)
CHECK
    then
        note "the intrinsics on /camera_info are not the dataset's — every unprojection in the run above was wrong by a constant factor and the ATE is a measurement of that"
    fi
fi

# =============================================================================
# Run 2 — rotation only, the control
# =============================================================================
echo
echo "-- run 2: odom_regime:=rotation_only, the control --"
run_regime "$work/rot.log" rotation_only "$work/rot.tum" || { echo "FAIL gate-trajectory"; exit 1; }

rot_path=$(probe_value "$work/rot.log" path_m)
rot_poses=0
[[ -s $work/rot.tum ]] && rot_poses=$(grep -cv '^#' "$work/rot.tum")
rot_ate_sim3=none
rot_degen=ok
if [[ -s $work/rot.tum ]]; then
    read -r rot_ate_sim3 _ _ rot_degen < <(ate_of "$work/rot.tum" sim3)
fi

# =============================================================================
# The assertions
# =============================================================================

# --- Claim 2: the replay was real time, and all of it ------------------------
#
# A source that cannot keep up publishes a 30 Hz sequence at some other rate, and
# the trajectory is then measured against a clip the pipeline never saw at its own
# speed — a slower replay gives depth more time per frame and makes every number
# below better for a reason that has nothing to do with the code.
(( ${six_published:-0} == SEQ_FRAMES )) ||
    note "dataset_node published ${six_published:-0} of ${SEQ_FRAMES} frames"
(( ${six_late:-0} == 0 )) ||
    note "${six_late} frames went out behind schedule — this replay was not real time, so every rate below is about a clip the pipeline never saw at its own speed"

# --- Claim 3: the trajectory is a trajectory, and evo saw all of it -----------
(( ${six_skipped:-0} == 0 )) ||
    note "${six_skipped} poses were left out of the trajectory because camera_optical_frame was not in odom_probe's TF buffer yet — the file describes a shorter run than the summary line beside it, which is a smaller ATE over less of the clip"
(( six_poses >= MIN_POSES )) ||
    note "the trajectory has ${six_poses} poses in it, floor ${MIN_POSES} — a clip of ${SEQ_FRAMES} frames at ~18 Hz of depth should give ~360"
associated_pct=$(awk -v c="$six_cmp_sim3" -v p="$six_poses" \
    'BEGIN { printf "%.1f", (p > 0) ? 100 * c / p : 0 }')
in_range "$associated_pct" "$MIN_ASSOCIATED_PCT" 100 ||
    note "evo associated ${associated_pct}% of our poses against the ground truth, floor ${MIN_ASSOCIATED_PCT}% — our stamps *are* the dataset's, so a shortfall means they have been through a float"

# --- Claim 4: the pose solve is solving ---------------------------------------
posed_pct=$(awk -v ok="$six_shift_ok" -v held="$six_shift_held" \
    'BEGIN { t = ok + held; printf "%.1f", (t > 0) ? 100 * ok / t : 0 }')
in_range "$six_reproj" 0.01 "$MAX_REPROJ_PX" ||
    note "mean inlier reprojection ${six_reproj} px, budget ${MAX_REPROJ_PX} px (0 would mean it never ran)"
in_range "$posed_pct" "$MIN_POSED_PCT" 100 ||
    note "only ${posed_pct}% of depth frames got a pose rather than holding the last one, floor ${MIN_POSED_PCT}%"

# --- Claim 5: the ATE, Sim(3)-aligned, under a ceiling the control fails ------
if [[ $six_ate_sim3 == none ]]; then
    note "evo could not align the sixdof trajectory at all (${six_degen})"
else
    in_range "$six_ate_sim3" 0 "$MAX_ATE_M" ||
        note "Sim(3)-aligned ATE RMSE ${six_ate_sim3} m, ceiling ${MAX_ATE_M} m"
fi

# --- Claim 6: the control does not pass the same ceiling ----------------------
#
# **And it fails before the metric is computed, which is a stronger statement
# than a large number.** rotation_only publishes translation identically zero, so
# the estimate's covariance is rank-deficient and Umeyama has nothing to fit —
# `evo` refuses to align it. Two things are asserted so that an `evo` failure for
# any *other* reason cannot be mistaken for this one: the control's published path
# is exactly zero (which is what makes the degeneracy the expected outcome), and
# no ATE under the ceiling comes back.
# And the ceiling has to be *below* what a motionless estimate scores, which is a
# figure derived from the ground truth above rather than typed here. Without this
# the previous two checks are satisfied by a ceiling of 10 m.
in_range "$MAX_ATE_M" 0 "$(awk -v s="$STATIONARY_ATE_M" 'BEGIN { printf "%.4f", s * 0.95 }')" ||
    note "the ${MAX_ATE_M} m ceiling is not meaningfully below the ${STATIONARY_ATE_M} m an estimate that never moved would score on this sequence — it would admit a pipeline that publishes nothing"
if [[ $(awk -v p="${rot_path:-1}" 'BEGIN { print (p + 0 == 0) ? 1 : 0 }') -ne 1 ]]; then
    note "the rotation_only control reported a ${rot_path} m path — it publishes translation identically zero by construction, so this control is no longer the thing it claims to be"
fi
if [[ $rot_ate_sim3 != none ]] && in_range "$rot_ate_sim3" 0 "$MAX_ATE_M"; then
    note "the rotation_only control scored ${rot_ate_sim3} m against the same ${MAX_ATE_M} m ceiling — a ceiling that admits a trajectory which never moves is not excluding anything"
fi

# =============================================================================
echo
echo "============================ gate-trajectory ============================"
echo "sequence       : $(basename "$SEQ")"
echo "               : ${SEQ_FRAMES} frames over ${SEQ_SECONDS}s, ${six_published:-?} published, ${six_late:-?} behind schedule"
echo "poses on /odom : ${six_frames} at ${six_rate} Hz, path ${six_path} m in the map's own units"
echo "poses written  : ${six_poses}, ${six_skipped:-?} skipped for want of the optical frame,"
echo "                 ${six_cmp_sim3} associated with ground truth (${associated_pct}%)"
echo "---"
printf '%-30s %12s\n' "ATE RMSE, SE(3) aligned (m)" "$six_ate_se3"
printf '%-30s %12s\n' "ATE RMSE, Sim(3) aligned (m)" "$six_ate_sim3"
printf '%-30s %12s\n' "RPE RMSE over ${RPE_WINDOW_S}s (m)" "$six_rpe"
printf '%-30s %12s\n' "  (window, frames)" "$rpe_delta"
printf '%-30s %12s\n' "fitted scale s" "$six_scale"
echo "---"
echo "solve          : ${six_reproj} px over ${six_inliers} inliers, ${posed_pct}% of depth frames posed"
echo "                 ${six_implausible} poses refused as implausible motion, fastest ${six_speed_max} m/s"
echo "control        : rotation_only, path ${rot_path} m over ${rot_poses} poses, ATE ${rot_ate_sim3} (${rot_degen})"
echo "                 an estimate that never moves scores ${STATIONARY_ATE_M} m on this"
echo "                 sequence — the RMS spread of the truth about its own centroid,"
echo "                 derived from groundtruth.txt and not typed here. The ceiling"
echo "                 sits between that and the ${six_ate_sim3} m measured above."
echo "assert         : Sim(3) ATE <= ${MAX_ATE_M} m, itself below ${STATIONARY_ATE_M} m, and the"
echo "                 control does not reach it; the replay was real time and complete;"
echo "                 >= ${MIN_POSES} poses, >= ${MIN_ASSOCIATED_PCT}% associated, >= ${MIN_POSED_PCT}% posed,"
echo "                 reprojection <= ${MAX_REPROJ_PX} px; /camera_info == the dataset's;"
echo "                 dataset_node refuses the C922's calibration over these frames"
echo
echo "The SE(3) number is printed and NOT asserted, and the difference between the"
echo "two is the whole of what is unpinned here: depth_scale is 10.0 because"
echo "somebody typed it, so an SE(3) ATE on a metric dataset is very largely a"
echo "measurement of that constant. The Sim(3) figure is the trajectory's *shape*."
echo
echo "  depth_scale implied by this sequence: $(awk -v s="${six_scale:-1}" 'BEGIN { printf "%.2f", 10.0 * s }')"
echo
echo "  — 10.0 times the fitted scale, and it is the answer P12's tape measure is"
echo "    going to give, obtained without one. It does **not** transfer: different"
echo "    camera, different scene, and Depth Anything's scale is per-image. What it"
echo "    does is bound the number, so P12 becomes a check on a figure that already"
echo "    exists rather than the only source of it. The predecessor's room came out"
echo "    at 2.69."
echo
echo "RPE is printed rather than asserted: it is one run of a figure this project"
echo "has no second opinion on, and it has already moved once for a reason that was"
echo "not the pipeline — 0.7679 m with the trajectory written in the body frame"
echo "against 0.1503 m in the optical one, where rotating TUM's own ground truth by"
echo "that same constant scores 0.654 m against itself. An ATE cannot see it."
echo "========================================================================="

if (( fail )); then echo "FAIL gate-trajectory"; exit 1; fi
echo "PASS gate-trajectory"
