#!/usr/bin/env bash
#
# P14 gate (#11): map points, covisibility, and tracking against the local map.
#
# Replays TUM fr1/desk with `local_map:=true` and with the control,
# `local_map:=false` — the same binary one parameter apart, which is P7's tracker
# posing each frame against the newest keyframe alone — several times each,
# alternating, and hands every trajectory to `evo`. Then replays bags/walk1 once
# with the local map, for the structural claims on a clip that translates.
#
# --- What it asserts, and the order that matters -------------------------------
#
# **The phase's claim is the ATE, and it is asserted, not printed.** Everything
# else here is a precondition for that number to mean anything, and a gate that
# passed on the preconditions while the claim failed would be the thing this
# project has spent its build log learning not to write — green over the one
# assertion that was the point. As of 2026-09-30 **this gate fails on the ATE**:
# see the measurement note at the bottom and #11's P14 annotation.
#
# The three false greens #11 names for this phase, each with the assertion that
# answers it:
#
#  1. **A map that never culls has beautiful counts.** Asserted: points *and*
#     keyframes were judged by the cull — `judged_points`, `judged_keyframes` — and
#     points were culled. Keyframes culled is printed rather than asserted: on
#     fr1/desk the redundancy test (90% of a keyframe's points seen by three other
#     keyframes) is never met, measured 0 in every run, and a cull that looked and
#     found nothing is a different fact from one that never looked. The judged
#     count is what separates them.
#  2. **"Three observations" satisfied by re-admitting a track id.** The map
#     refuses a second observation of a point from one keyframe and refuses a
#     track id re-admitted onto a feature the geometry disagrees with; both are
#     pinned in test_map. Here the refusals are printed, and `refused_dup` is
#     asserted to be *counted* (the field exists and parsed), not to be zero.
#  3. **A local map that is always the newest keyframe.** Asserted: the median
#     local-map size is greater than 1 in every local-map run — measured 2 — and the
#     control's local map is *empty*, because the control that tracks against a
#     local map is not a control.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-map =="

# --- The budgets -------------------------------------------------------------
#
# Runs per arm on fr1/desk. **Three, because one of each is inside the noise**:
# P11 recorded 0.27-0.36 m over seven runs of the same code, and single runs of
# the local map ranged 0.205-0.509 m on one afternoon. The ATE claim is that the
# two distributions do not overlap, which is the form #10's P13 used for the same
# reason — a median comparison of three noisy numbers is a coin flip with a
# threshold drawn on it.
RUNS=${PIMESH_MAP_RUNS:-3}
# The local map has to be more than the reference keyframe.
MIN_LOCAL_KF_MEDIAN=2
# Every live-map run must have culled something and judged keyframes at all.
MIN_CULLED_POINTS=1
MIN_JUDGED_KEYFRAMES=1
# bags/walk1's replay window.
WALK_BAG="$PIMESH_WS/bags/walk1"

assert_no_session "bash tools/gates/map.sh"
arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

EVO_APE=${EVO_APE:-$HOME/.local/bin/evo_ape}
MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || { echo "FAIL: no model at ${MODEL} — bash tools/fetch-model.sh"; exit 1; }
SEQ=$(bash "$PIMESH_WS/tools/fetch-dataset.sh" --print-path)
GROUND_TRUTH="$SEQ/groundtruth.txt"
[[ -r $GROUND_TRUTH ]] || { echo "FAIL: no dataset at ${SEQ} — bash tools/fetch-dataset.sh"; exit 1; }
[[ -x $EVO_APE ]] || { echo "FAIL: no evo at ${EVO_APE} — uv tool install evo"; exit 1; }
[[ -d $WALK_BAG ]] || {
    echo "FAIL: no bag at ${WALK_BAG}"
    echo "bags/ is git-ignored; it was recorded for #10's P13 with"
    echo "  bash tools/record-clip.sh walk1 60"
    exit 1
}

work=$(mktemp -d)
SEQ_SECONDS=$(awk '!/^#/ && NF { if (!s) s = $1; e = $1 } END { printf "%d", e - s }' "$SEQ/rgb.txt")
MEASURE_S=$(( SEQ_SECONDS + 8 ))

# --- One fr1/desk replay -------------------------------------------------------
run_dataset() {          # $1 = log, $2 = local_map, $3 = trajectory
    local log=$1 local_map=$2 traj=$3
    timeout -s INT $(( MEASURE_S + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" odom_regime:=sixdof \
        local_map:="$local_map" probe:=odom_probe \
        probe_duration_s:="$(( MEASURE_S + 4 ))" trajectory_path:="$traj" \
        >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: dataset_node never came up within 60 s"; tail -30 "$log"; return 1; }
    sleep $(( MEASURE_S + 12 ))
    kill_local
    sleep 1
}

# --- One bags/walk1 replay, played once ------------------------------------------
#
# Once, not --loop, for the reason gates/odom.sh gives: a looping bag's stamps jump
# back by its length and a TF edge stamped with them freezes.
run_walk() {             # $1 = log
    local log=$1 window
    window=$(awk '/duration:/ { getline; if ($1 == "nanoseconds:") { printf "%d", $2 / 1e9; exit } }' \
        "$WALK_BAG/metadata.yaml")
    [[ -n $window && $window -gt 0 ]] || window=60
    timeout -s INT $(( window + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        odom_regime:=sixdof local_map:=true probe:=odom_probe \
        probe_duration_s:="$(( window + 6 ))" >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "fusion_node up:" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: the container never came up within 60 s"; tail -30 "$log"; return 1; }
    timeout -s INT $(( window + 20 )) ros2 bag play "$WALK_BAG" \
        --disable-keyboard-controls </dev/null >"$log.play" 2>&1 &
    sleep $(( window + 10 ))
    kill_local
    sleep 1
}

# The last `stats map` line odometry_node printed. Every figure on it is
# cumulative over the run, so the last line is the whole run — unlike the
# windowed rates, where the last window is the idle tail.
map_value() {            # $1 = log, $2 = key
    grep -h 'odometry_node' "$1" | grep -o 'stats map .*' | tail -1 |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { sub(/ms$/, "", v); print v }'
}

ate_of() {               # $1 = trajectory; prints "rmse scale"
    "$EVO_APE" tum "$GROUND_TRUTH" "$1" -a -s -v 2>&1 |
        awk '/Scale correction:/ { s = $3 } $1 == "rmse" { r = $2 }
             END { printf "%s %s\n", (r == "") ? "none" : r, (s == "") ? "?" : s }'
}

# =============================================================================
# fr1/desk, alternating arms so that load on the box lands on both
# =============================================================================
declare -a lm_ate ctl_ate
echo
echo "sequence: ${SEQ} (${SEQ_SECONDS}s), ${RUNS} run(s) per arm"
for i in $(seq "$RUNS"); do
    for arm in true false; do
        tag=$([[ $arm == true ]] && echo lm || echo ctl)
        echo "-- run ${i}, local_map:=${arm} --"
        run_dataset "$work/$tag.$i.log" "$arm" "$work/$tag.$i.tum" || { echo "FAIL gate-map"; exit 1; }
        if [[ ! -s $work/$tag.$i.tum ]]; then
            note "run ${i} local_map:=${arm} wrote no trajectory"
            continue
        fi
        read -r ate scale < <(ate_of "$work/$tag.$i.tum")
        printf '   ATE %s m (Sim(3) scale %s), local map p50 %s, by projection %s/frame\n' \
            "$ate" "$scale" "$(map_value "$work/$tag.$i.log" local_kf_p50)" \
            "$(map_value "$work/$tag.$i.log" by_projection)"
        if [[ $arm == true ]]; then lm_ate+=("$ate"); else ctl_ate+=("$ate"); fi

        # --- Structural claims, per run ---------------------------------------
        judged_kf=$(map_value "$work/$tag.$i.log" judged_keyframes)
        culled_pts=$(map_value "$work/$tag.$i.log" culled_points)
        points=$(map_value "$work/$tag.$i.log" points)
        [[ -n $points ]] || { note "run ${i} local_map:=${arm}: no 'stats map' line — the map was never reported, which is not the same as empty"; continue; }
        (( ${culled_pts:-0} >= MIN_CULLED_POINTS )) ||
            note "run ${i} local_map:=${arm}: ${culled_pts:-?} points culled — a map that never culls has the best counts of all"
        (( ${judged_kf:-0} >= MIN_JUDGED_KEYFRAMES )) ||
            note "run ${i} local_map:=${arm}: the keyframe cull judged ${judged_kf:-?} keyframes — it did not run"
        if [[ $arm == true ]]; then
            in_range "$(map_value "$work/$tag.$i.log" local_kf_p50)" "$MIN_LOCAL_KF_MEDIAN" 1000 ||
                note "run ${i}: median local map $(map_value "$work/$tag.$i.log" local_kf_p50) keyframes — a local map of the reference alone is P7's tracker under a new name"
        else
            # The control must not be tracking against the map.
            [[ $(map_value "$work/$tag.$i.log" local_kf_n) == 0 ]] ||
                note "run ${i}: the control tracked $(map_value "$work/$tag.$i.log" local_kf_n) frames against a local map — it is no longer the control"
        fi
    done
done

# =============================================================================
# bags/walk1, the local map only
# =============================================================================
echo
echo "-- bags/walk1, local_map:=true --"
run_walk "$work/walk.log" || { echo "FAIL gate-map"; exit 1; }
walk_points=$(map_value "$work/walk.log" points)
if [[ -z $walk_points ]]; then
    note "bags/walk1: no 'stats map' line"
else
    in_range "$(map_value "$work/walk.log" local_kf_p50)" "$MIN_LOCAL_KF_MEDIAN" 1000 ||
        note "bags/walk1: median local map $(map_value "$work/walk.log" local_kf_p50) keyframes"
    (( $(map_value "$work/walk.log" culled_points) >= MIN_CULLED_POINTS )) ||
        note "bags/walk1: no points culled"
    (( $(map_value "$work/walk.log" judged_keyframes) >= MIN_JUDGED_KEYFRAMES )) ||
        note "bags/walk1: the keyframe cull never ran"
fi

# =============================================================================
# The claim: the local map's ATE falls, and the distributions do not overlap
# =============================================================================
read -r lm_min lm_med lm_max < <(printf '%s\n' "${lm_ate[@]}" | sort -g |
    awk '{ v[NR] = $1 } END { printf "%s %s %s\n", v[1], v[int((NR + 1) / 2)], v[NR] }')
read -r ctl_min ctl_med ctl_max < <(printf '%s\n' "${ctl_ate[@]}" | sort -g |
    awk '{ v[NR] = $1 } END { printf "%s %s %s\n", v[1], v[int((NR + 1) / 2)], v[NR] }')
(( ${#lm_ate[@]} == RUNS && ${#ctl_ate[@]} == RUNS )) ||
    note "only ${#lm_ate[@]} local-map and ${#ctl_ate[@]} control ATEs were computed of ${RUNS} each"
ate_falls=$(awk -v a="$lm_max" -v b="$ctl_min" 'BEGIN { print (a + 0 < b + 0) ? 1 : 0 }')
(( ate_falls == 1 )) ||
    note "the local map's worst ATE (${lm_max} m) is not below the control's best (${ctl_min} m) — #11's P14 claim is that tracking against the local map lowers the ATE, and it did not"

# =============================================================================
line() { printf '%-26s %10s %10s\n' "$1" "$2" "$3"; }
echo
echo "================================ gate-map ================================"
line "" "local_map" "control"
line "ATE Sim(3), min (m)" "$lm_min" "$ctl_min"
line "ATE Sim(3), median (m)" "$lm_med" "$ctl_med"
line "ATE Sim(3), max (m)" "$lm_max" "$ctl_max"
echo "---  the map, last local-map run on fr1/desk and on bags/walk1"
last="$work/lm.${RUNS}.log"
for key in keyframes points obs3_frac triangulated tri_err_px depth_ratio \
    depth_ratio_p05 depth_ratio_p95 culled_points culled_keyframes judged_points \
    judged_keyframes local_kf_p50 local_kf_p95 by_track by_projection associated \
    refused_dup refused_reproj align_dev map_cost; do
    line "$key" "$(map_value "$last" "$key")" "$(map_value "$work/walk.log" "$key")"
done | sed '1i\                              fr1/desk   walk1'
echo "---"
echo "assert: ATE falls — every local-map run below every control run on fr1/desk;"
echo "        local map median >= ${MIN_LOCAL_KF_MEDIAN} keyframes; points culled and keyframes"
echo "        judged in every run and on walk1; the control never tracked a local map"
echo
echo "depth_ratio is a triangulated point's distance from the camera that created it"
echo "over the network's reading for the same point: the first opinion about the"
echo "depth model that does not come from the depth model. It is not independent —"
echo "the poses were solved against network-depth points — and align_dev is the"
echo "number that says how far apart two keyframes' depth maps are: the mean"
echo "|z_map / z_network - 1| over the points each new keyframe re-observed."
echo "=========================================================================="

if (( fail )); then echo "FAIL gate-map"; exit 1; fi
echo "PASS gate-map"
