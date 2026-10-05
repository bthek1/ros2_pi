#!/usr/bin/env bash
#
# P20 gate (#13): relocalise from a persisted map.
#
#   TUM fr1/desk frames [0, SPLIT)        save a map (map_save_path)
#   TUM fr1/desk frames [SPLIT+GAP, end)  a new container loads it and relocalises
#   bags/desk1, whole clip                the same map, another room: the control
#
# **The positive case is on fr1/desk, not walk1 as #13 was written**, for the reason
# gates/loop.sh moved its ATE there: walk1 has no ground truth, so "recovered within
# a tolerance of the saved trajectory" could only be judged against this pipeline's
# own drifted odometry. On fr1/desk every relocalised pose is scored against motion
# capture by tools/eval/reloc_truth.py, through the Sim(3) that carries the saved
# map's frame onto it.
#
# **The control is a different room, which walk1-against-desk1 might not be** — both
# were recorded in the same room, where relocalising would be the right answer. TUM's
# office in Munich and this room are certainly two places, seen by two cameras.
#
# --- What is asserted, and the false green each answers ---------------------------
#
#  1. **The map that was saved is the map that was loaded** — same keyframe count. A
#     loader that read half a file relocalises less and looks like a smaller room.
#  2. **A relocalisation happened, within MAX_LOST_FRAMES depth frames** of the
#     loading session's start (it starts LOST: its odom has no relation to the map).
#  3. **Every relocalised pose within TOL_M of motion capture**, in TUM's metres, and
#     none unjudged. Printed beside the saved map's own error against motion capture,
#     because a relocalisation can be no better than the map it lands in.
#  4. **fusion_node refused frames while LOST and integrated after** — the state the
#     relocalisation ends is the one P19 made fusion read.
#  5. **The control: queries > MIN_CONTROL_QUERIES and 0 accepted.** A relocaliser
#     that cannot refuse is not a relocaliser; and one that never searched has a
#     perfect refusal record, which is why the count of searches is asserted first.
#
# Printed: load time, map size on disk and per keyframe (the plan expected ~40 kB),
# query cost, recovery latency, and the recovered pose's error.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-relocalise =="

SPLIT=${PIMESH_RELOC_SPLIT:-300}
GAP=${PIMESH_RELOC_GAP:-30}
MAX_LOST_FRAMES=${PIMESH_RELOC_MAX_LOST:-30}
TOL_M=${PIMESH_RELOC_TOL_M:-0.30}
MIN_CONTROL_QUERIES=50

assert_no_session "bash tools/gates/relocalise.sh"
arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || { echo "FAIL: no model at ${MODEL} — bash tools/fetch-model.sh"; exit 1; }
SEQ=$(bash "$PIMESH_WS/tools/fetch-dataset.sh" --print-path)
GROUND_TRUTH="$SEQ/groundtruth.txt"
[[ -r $GROUND_TRUTH ]] || { echo "FAIL: no dataset at ${SEQ} — bash tools/fetch-dataset.sh"; exit 1; }
BAG=$PIMESH_WS/bags/desk1
[[ -r $BAG/metadata.yaml ]] || { echo "FAIL: no bag at ${BAG} — just record desk1 60"; exit 1; }

work=$(mktemp -d)
MAP="$work/fr1_desk_first.map"
SAVED_TUM="$work/saved.tum"

# Seconds of the sequence between two frame indices of rgb.txt.
seq_seconds() {          # $1 = first index, $2 = last index (exclusive; 0 = end)
    awk -v a="$1" -v b="$2" '!/^#/ && NF { s[n++] = $1 }
        END { if (b == 0 || b > n) b = n; printf "%d", s[b - 1] - s[a] + 1 }' "$SEQ/rgb.txt"
}
A_SECONDS=$(seq_seconds 0 "$SPLIT")
B_SECONDS=$(seq_seconds $(( SPLIT + GAP )) 0)
BAG_SECONDS=$(/usr/bin/python3 - "$BAG/metadata.yaml" <<'META'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
print(int(m['duration']['nanoseconds'] / 1e9) + 1)
META
)
echo "sequence: ${SEQ}; save frames [0, ${SPLIT}) (${A_SECONDS}s), relocalise from ${SPLIT}+${GAP} (${B_SECONDS}s)"

stat_of() {              # $1 = log, $2 = node, $3 = line, $4 = key
    grep -h "$2" "$1" | grep -o "stats $3 .*" | tail -1 |
        awk -v key="$4" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}

run_dataset() {          # $1 = log, $2 = seconds, then extra launch arguments
    local log=$1 secs=$2; shift 2
    timeout -s INT $(( secs + 60 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" odom_regime:=sixdof "$@" >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: dataset_node never came up within 60 s"; tail -30 "$log"; return 1; }
    # Past the clip and one more stats window, so the keyframe trajectory is written
    # after the last keyframe.
    sleep $(( secs + 8 ))
    kill_local
    sleep 1
}

declare -A R

# --- A: save ---------------------------------------------------------------------
echo "-- A: fr1/desk [0, ${SPLIT}), saving --"
run_dataset "$work/a.log" "$A_SECONDS" dataset_max_frames:="$SPLIT" \
    map_save_path:="$MAP" keyframe_trajectory_path:="$SAVED_TUM" || { echo "FAIL gate-relocalise"; exit 1; }
[[ -s $MAP ]] || { note "A wrote no map to ${MAP}"; }
R[saved]=$(grep -o 'map saved (shutdown): keyframes=[0-9]*' "$work/a.log" | tail -1 | grep -o '[0-9]*$')
[[ -n ${R[saved]} ]] || {
    R[saved]=$(grep -o 'map saved ([a-z]*): keyframes=[0-9]*' "$work/a.log" | tail -1 | grep -o '[0-9]*$')
    note "A never logged its shutdown save — the destructor did not run, or failed"
}
R[saved_mb]=$(grep -o 'map saved ([a-z]*): keyframes=[0-9]* file_mb=[0-9.]*' "$work/a.log" | tail -1 | grep -o '[0-9.]*$')
R[save_ms]=$(stat_of "$work/a.log" odometry_node reloc save_ms)
(( ${R[saved]:-0} >= 5 )) || note "A saved ${R[saved]:-no} keyframes — too few to call a map"
[[ -s $SAVED_TUM ]] || note "A wrote no keyframe trajectory"

# --- B: load, relocalise -----------------------------------------------------------
echo "-- B: fr1/desk from $(( SPLIT + GAP )), loading --"
run_dataset "$work/b.log" "$B_SECONDS" dataset_skip_frames:=$(( SPLIT + GAP )) \
    map_load_path:="$MAP" || { echo "FAIL gate-relocalise"; exit 1; }
R[loaded]=$(grep -o 'map loaded: keyframes=[0-9]*' "$work/b.log" | head -1 | grep -o '[0-9]*$')
R[load_ms]=$(grep -o 'map loaded: .* load_ms=[0-9.]*' "$work/b.log" | head -1 | grep -o '[0-9.]*$')
R[first_lost]=$(grep -o 'relocalised .*lost_frames=[0-9]*' "$work/b.log" | head -1 | grep -o '[0-9]*$')
for key in queries accepted applied ignored query_ms query_ms_p95 submitted; do
    R[b.$key]=$(stat_of "$work/b.log" odometry_node reloc "$key")
done
for key in refused_lost integrated; do
    R[b.$key]=$(stat_of "$work/b.log" fusion_node lost "$key")
done
while IFS='=' read -r key value; do
    R[t.$key]=$value
done < <(cd "$PIMESH_WS/tools/eval" && /usr/bin/python3 reloc_truth.py \
    --groundtruth "$GROUND_TRUTH" --saved "$SAVED_TUM" --log "$work/b.log")

[[ ${R[loaded]:-x} == "${R[saved]:-y}" ]] ||
    note "B loaded ${R[loaded]:-no} keyframes where A saved ${R[saved]:-?}"
(( ${R[b.applied]:-0} >= 1 )) || note "B never relocalised (${R[b.queries]:-0} queries)"
if [[ -n ${R[first_lost]} ]] && (( R[first_lost] > MAX_LOST_FRAMES )); then
    note "the first relocalisation came after ${R[first_lost]} LOST depth frames, against ${MAX_LOST_FRAMES}"
fi
[[ ${R[t.unjudged]:-x} == 0 ]] || note "${R[t.unjudged]:-?} relocalisations motion capture could not judge"
if (( ${R[t.judged]:-0} > 0 )); then
    awk -v e="${R[t.error_max_m]}" -v t="$TOL_M" 'BEGIN { exit !(e + 0 <= t + 0) }' ||
        note "a relocalised pose is ${R[t.error_max_m]} m from motion capture, against ${TOL_M}"
fi
(( ${R[b.refused_lost]:-0} > 0 )) || note "fusion refused nothing while B was LOST — did it read the state?"
(( ${R[b.integrated]:-0} > 0 )) || note "fusion integrated nothing after B relocalised"

# --- Control: another room -----------------------------------------------------------
echo "-- control: bags/desk1 against fr1/desk's map --"
log="$work/control.log"
timeout -s INT $(( BAG_SECONDS + 60 )) ros2 launch pimesh_bringup pimesh.launch.py \
    odom_regime:=sixdof map_load_path:="$MAP" >"$log" 2>&1 &
ready=0
for _ in $(seq 120); do
    if grep -q "fusion_node up:" "$log" 2>/dev/null; then ready=1; break; fi
    sleep 0.5
done
(( ready == 1 )) || { echo "FAIL: the container never came up within 60 s"; tail -30 "$log"; exit 1; }
timeout -s INT $(( BAG_SECONDS + 20 )) ros2 bag play "$BAG" \
    --disable-keyboard-controls </dev/null >"$log.play" 2>&1
sleep 6
kill_local
sleep 1
for key in queries accepted applied query_ms; do
    R[c.$key]=$(stat_of "$log" odometry_node reloc "$key")
done
R[c.integrated]=$(stat_of "$log" fusion_node lost integrated)
(( ${R[c.queries]:-0} >= MIN_CONTROL_QUERIES )) ||
    note "the control searched ${R[c.queries]:-0} times — too few for its zero to mean anything"
[[ ${R[c.accepted]:-x} == 0 && ${R[c.applied]:-x} == 0 ]] ||
    note "the control relocalised into another room: ${R[c.accepted]:-?} accepted, ${R[c.applied]:-?} applied"
[[ ${R[c.integrated]:-x} == 0 ]] || note "the control fused ${R[c.integrated]:-?} frames while never localised"

# =============================================================================
kb_per=$(awk -v mb="${R[saved_mb]:-0}" -v n="${R[saved]:-0}" 'BEGIN { if (n > 0) printf "%.1f", mb * 1000 / n; else print "-" }')
echo
echo "============================= gate-relocalise ============================="
echo "map  : ${R[saved]:-?} keyframes saved, ${R[loaded]:-?} loaded; ${R[saved_mb]:-?} MB on disk (${kb_per} kB a keyframe)"
echo "       load ${R[load_ms]:-?} ms; last save ${R[save_ms]:-?} ms"
echo "B    : ${R[b.applied]:-?} relocalisation(s) applied, ${R[b.ignored]:-?} ignored; first after ${R[first_lost]:-?} LOST depth frames"
echo "       ${R[b.queries]:-?} queries (${R[b.submitted]:-?} submitted), ${R[b.query_ms]:-?} ms median, p95 ${R[b.query_ms_p95]:-?}"
echo "       vs motion capture: first ${R[t.error_first_m]:-?} m, median ${R[t.error_median_m]:-?} m, max ${R[t.error_max_m]:-?} m;"
echo "       rotation first ${R[t.rot_first_deg]:-?} deg (the saved map's own: ${R[t.saved_rot_median_deg]:-?} deg)"
echo "       saved map vs motion capture: ATE ${R[t.saved_ate_m]:-?} m over ${R[t.saved_associated]:-?} keyframes, scale ${R[t.scale]:-?}"
echo "       fusion refused ${R[b.refused_lost]:-?} while LOST, integrated ${R[b.integrated]:-?}"
echo "ctrl : ${R[c.queries]:-?} queries into fr1/desk's map from bags/desk1, ${R[c.accepted]:-?} accepted, ${R[c.query_ms]:-?} ms median; fusion integrated ${R[c.integrated]:-?}"
echo "assert: saved == loaded; >= 1 relocalisation within ${MAX_LOST_FRAMES} frames; every one within"
echo "        ${TOL_M} m of motion capture, none unjudged; fusion refused while LOST and"
echo "        integrated after; control >= ${MIN_CONTROL_QUERIES} queries, 0 accepted, 0 fused"
echo "==========================================================================="

if (( fail )); then echo "FAIL gate-relocalise"; exit 1; fi
echo "PASS gate-relocalise"
