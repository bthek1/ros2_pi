#!/usr/bin/env bash
#
# P17 gate (#12): a pose graph over every keyframe, and `map -> odom` published
# for real.
#
#   TUM fr1/desk  x RUNS per arm, alternating   loop_closure:=true against :=false
#   bags/walk1    once, loop_closure:=true      the TF tree survives a real closure
#
# **The ATE is on fr1/desk, not on walk1 as #12 was written.** walk1 has no ground
# truth, so an ATE on it cannot be computed; fr1/desk has motion capture and — P16
# measured — ten or more confirmed loops a run. walk1 keeps what it can answer:
# whether the tree is still whole after the correction it was recorded to cause.
#
# --- What is asserted, and the false green each answers -------------------------
#
#  1. **Every loop-closure run's keyframe ATE below every control run's** (Sim(3),
#     evo, against motion capture). The control is the same binary with
#     loop_closure:=false: the same search, the graph with no loop edge, which is
#     the odometry chain exactly (test_pose_graph pins that it is exact).
#  2. **The graph ran**: loop edges added and solves run in every closing run, none
#     in any control run. A graph that never solves passes (1) only if the control
#     happens to lose, and the counters say which.
#  3. **The tree survives**, asked during the replay with tf2_echo — `/tf` is not
#     latched, so a question asked after the clip ends hears nothing and would fail
#     a tree that was fine: `map -> odom` resolves and is **not** identity (a
#     correction was published), `map -> base_link` resolves through it, and
#     `base_link -> camera_optical_frame` still resolves (tf_static intact — the
#     check that would have caught the `--clock` failure CLAUDE.md records).
#  4. **Zero TF_OLD_DATA, TF_REPEATED_DATA or time-jump warnings** in either arm: a
#     correction stamped anything but the frame's own stamp floods every listener
#     from inside tf2's lock.
#
# Printed: graph solve cost, the last correction's size, closures the solved graph
# could not reconcile. **That last number is not a safety check** at the odometry
# noise model measured on fr1/desk — test_pose_graph pins that a 5 m false closure
# on a 16 m loop is absorbed silently — so the defence against a wrong-place closure
# is P16's, and gates/place.sh is where it is asserted.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-loop =="

RUNS=${PIMESH_LOOP_RUNS:-3}
# tf2_echo is started this many seconds before the replay ends and listens this long.
TF_LEAD_S=5
TF_LISTEN_S=3
# `map -> odom` further than this from identity counts as a published correction.
MIN_CORRECTION_M=0.01

assert_no_session "bash tools/gates/loop.sh"
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

work=$(mktemp -d)
SEQ_SECONDS=$(awk '!/^#/ && NF { if (!s) s = $1; e = $1 } END { printf "%d", e - s }' "$SEQ/rgb.txt")

# Ask tf2_echo for one edge for a few seconds; print its last translation, or nothing.
tf_echo() {              # $1 = parent, $2 = child, $3 = output file
    timeout -s INT "$TF_LISTEN_S" stdbuf -oL ros2 run tf2_ros tf2_echo "$1" "$2" \
        </dev/null >"$3" 2>&1 || true
}
last_translation() {     # $1 = tf2_echo output; "x y z" or empty
    grep -oE 'Translation: \[[^]]*\]' "$1" | tail -1 | tr -d '[],' | awk '{ print $2, $3, $4 }'
}
norm3() { awk '{ printf "%.4f", sqrt($1 * $1 + $2 * $2 + $3 * $3) }' <<<"$1"; }

# The three tree questions, asked while frames are still arriving.
check_tree() {           # $1 = file prefix, $2 = label, $3 = "closing" or "control"
    local p=$1 label=$2 arm=$3
    tf_echo map odom "$p.map_odom" &
    tf_echo map base_link "$p.map_base" &
    tf_echo base_link camera_optical_frame "$p.static" &
    wait
    local mo mb st
    mo=$(last_translation "$p.map_odom")
    mb=$(last_translation "$p.map_base")
    st=$(last_translation "$p.static")
    [[ -n $mo ]] || note "${label}: map -> odom never resolved"
    [[ -n $mb ]] || note "${label}: map -> base_link never resolved"
    [[ -n $st ]] || note "${label}: base_link -> camera_optical_frame never resolved — tf_static lost"
    if [[ -n $mo ]]; then
        local size
        size=$(norm3 "$mo")
        echo "   map -> odom translation ${size} m"
        if [[ $arm == closing ]]; then
            (( $(awk -v s="$size" -v m="$MIN_CORRECTION_M" 'BEGIN { print (s > m) }') )) ||
                note "${label}: map -> odom is identity (${size} m) — no correction was published"
        else
            [[ $size == 0.0000 ]] || note "${label}: the control published a correction of ${size} m"
        fi
    fi
}

tf_warnings() {          # $1 = log
    grep -cE 'TF_OLD_DATA|TF_REPEATED_DATA|[Jj]ump back in time' "$1" || true
}

place_value() {          # $1 = log, $2 = key — the last cumulative `stats place` line
    grep -h 'odometry_node' "$1" | grep -o 'stats place .*' | tail -1 |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}

run_dataset() {          # $1 = log, $2 = loop_closure, $3 = trajectory, $4 = tree prefix, $5 = label
    local log=$1 lc=$2 traj=$3 prefix=$4 label=$5 window=$(( SEQ_SECONDS + 8 ))
    timeout -s INT $(( window + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" odom_regime:=sixdof \
        loop_closure:="$lc" keyframe_trajectory_path:="$traj" >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: dataset_node never came up within 60 s"; tail -30 "$log"; return 1; }
    sleep $(( SEQ_SECONDS - TF_LEAD_S ))
    check_tree "$prefix" "$label" "$([[ $lc == true ]] && echo closing || echo control)"
    sleep $(( window - SEQ_SECONDS + TF_LEAD_S - TF_LISTEN_S + 12 ))
    kill_local
    sleep 1
}

ate_of() {
    "$EVO_APE" tum "$GROUND_TRUTH" "$1" -a -s 2>&1 | awk '$1 == "rmse" { print $2 }'
}

# --- fr1/desk, alternating --------------------------------------------------------
echo
echo "sequence: ${SEQ} (${SEQ_SECONDS}s), ${RUNS} run(s) per arm"
declare -A ate
declare -a rows
for i in $(seq "$RUNS"); do
    for arm in closing control; do
        lc=$([[ $arm == closing ]] && echo true || echo false)
        log="$work/$arm.$i.log"
        traj="$work/$arm.$i.tum"
        echo "-- run ${i}, ${arm} (loop_closure:=${lc}) --"
        run_dataset "$log" "$lc" "$traj" "$work/$arm.$i.tree" "run ${i} ${arm}" || { echo "FAIL gate-loop"; exit 1; }
        [[ -s $traj ]] || { note "run ${i} ${arm}: no keyframe trajectory written"; continue; }
        ate[$arm.$i]=$(ate_of "$traj")
        [[ -n ${ate[$arm.$i]} ]] || note "run ${i} ${arm}: evo returned no ATE"
        loops=$(place_value "$log" loops)
        solves=$(place_value "$log" solves)
        warn=$(tf_warnings "$log")
        printf '   ATE %s m over %s keyframes; %s loop edge(s), %s solve(s), %s ms (p95 %s), last correction %s m / %s deg, %s inconsistent; %s TF warning(s)\n' \
            "${ate[$arm.$i]}" "$(grep -vc '^#' "$traj")" "$loops" "$solves" \
            "$(place_value "$log" solve_ms)" "$(place_value "$log" solve_ms_p95)" \
            "$(place_value "$log" correction_m)" "$(place_value "$log" correction_deg)" \
            "$(place_value "$log" inconsistent)" "$warn"
        [[ $warn == 0 ]] || note "run ${i} ${arm}: ${warn} TF warning(s) — a correction stamped off the frame's own stamp"
        if [[ $arm == closing ]]; then
            [[ $(place_value "$log" loop_closure) == true ]] || note "run ${i}: loop_closure was not on"
            (( ${loops:-0} > 0 && ${solves:-0} > 0 )) ||
                note "run ${i}: ${loops:-none} loop edges and ${solves:-none} solves — the graph never ran"
        else
            [[ ${loops:-x} == 0 && ${solves:-x} == 0 ]] ||
                note "run ${i} control: ${loops:-none} loop edges, ${solves:-none} solves — it is not the control"
        fi
        rows+=("$(printf '%-6s %-8s %10s %6s %6s' "$i" "$arm" "${ate[$arm.$i]}" "$loops" "$solves")")
    done
done

stats_of() {             # $1 = arm; prints "min median max"
    local arm=$1 i
    for i in $(seq "$RUNS"); do echo "${ate[$arm.$i]}"; done | awk 'NF' | sort -g |
        awk '{ v[NR] = $1 } END { if (!NR) { print "none none none"; exit }
            printf "%s %s %s\n", v[1], v[int((NR + 1) / 2)], v[NR] }'
}
read -r on_min on_med on_max < <(stats_of closing)
read -r off_min off_med off_max < <(stats_of control)
if [[ $on_max == none || $off_min == none ]]; then
    note "no ATE for one of the arms"
else
    (( $(awk -v a="$on_max" -v b="$off_min" 'BEGIN { print (a + 0 < b + 0) }') )) ||
        note "loop closure's worst ATE (${on_max} m) is not below the control's best (${off_min} m)"
fi

# --- walk1: the tree through a real closure --------------------------------------
walk_line="(no bags/walk1)"
if [[ -r $PIMESH_WS/bags/walk1/metadata.yaml ]]; then
    echo "-- bags/walk1 (loop_closure:=true) --"
    bag=$PIMESH_WS/bags/walk1
    secs=$(/usr/bin/python3 - "$bag/metadata.yaml" <<'META'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
print(int(m['duration']['nanoseconds'] / 1e9) + 1)
META
)
    log="$work/walk1.log"
    timeout -s INT $(( secs + 60 )) ros2 launch pimesh_bringup pimesh.launch.py \
        odom_regime:=sixdof loop_closure:=true >"$log" 2>&1 &
    ready=0
    for _ in $(seq 120); do
        if grep -q "fusion_node up:" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: the container never came up within 60 s"; tail -30 "$log"; exit 1; }
    timeout -s INT $(( secs + 20 )) ros2 bag play "$bag" \
        --disable-keyboard-controls </dev/null >"$log.play" 2>&1 &
    # walk1's return is found in its last seconds, so the tree is asked about at
    # the very end of the clip, while its frames are still arriving.
    sleep $(( secs - 2 ))
    check_tree "$work/walk1.tree" walk1 closing
    sleep 10
    kill_local
    sleep 1
    warn=$(tf_warnings "$log")
    [[ $warn == 0 ]] || note "walk1: ${warn} TF warning(s)"
    (( $(place_value "$log" solves) > 0 )) || note "walk1: the graph never solved — no closure reached it"
    walk_line="$(place_value "$log" loops) loop edge(s), $(place_value "$log" solves) solve(s), last correction $(place_value "$log" correction_m) m / $(place_value "$log" correction_deg) deg; ${warn} TF warning(s)"
fi

# =============================================================================
echo
echo "================================ gate-loop ================================"
printf '%-6s %-8s %10s %6s %6s\n' run arm "ATE (m)" loops solves
printf '%s\n' "${rows[@]}"
echo "---"
printf '%-24s %10s %10s\n' "keyframe ATE Sim(3) (m)" "closing" "control"
printf '%-24s %10s %10s\n' "  min" "$on_min" "$off_min"
printf '%-24s %10s %10s\n' "  median" "$on_med" "$off_med"
printf '%-24s %10s %10s\n' "  max" "$on_max" "$off_max"
echo "walk1 : ${walk_line}"
echo "assert: every closing run's ATE below every control run's; the graph solved in"
echo "        every closing run and never in the control; map -> odom resolves and is"
echo "        non-identity when closing, identity in the control; map -> base_link"
echo "        and tf_static resolve during the replay; 0 TF warnings"
echo "==========================================================================="

if (( fail )); then echo "FAIL gate-loop"; exit 1; fi
echo "PASS gate-loop"
