#!/usr/bin/env bash
#
# P19 gate (#13): a tracking state, and refusing to fuse while LOST.
#
#   bags/desk1, a lens cap injected mid-clip   x2: the real run, and the control
#
# decode_node publishes black for BLACKOUT_S seconds of stamp starting
# BLACKOUT_START_S after the clip's first frame (`blackout_s:=`), which is what a
# covered lens delivers: ORB finds nothing, and the depth network still returns a
# confident depth map that fusion_node would integrate at whatever pose it was
# handed. odometry_node's held pose *is* still published — TF has to stay
# continuous — so the only thing standing between that depth map and the volume is
# the state on /tracking/state.
#
# --- What is asserted, and the false green each answers ---------------------------
#
#  1. **LOST within lost_after_holds depth frames of the blackout's first black
#     stamp** — read off the stream by tools/eval/lost_timeline.py, against the
#     stamps decode_node logged, not off odometry_node's own counters.
#  2. **0 voxels integrated while LOST, beside refused_lost > 0.** The zero alone is
#     the zero from a check that never ran: fusion_node's counter only moves when a
#     LOST frame is integrated, so a fusion_node that never *saw* a LOST state reports
#     the same 0. refused_lost > 0 says the refusal executed.
#  3. **OK again within MAX_TO_OK depth frames of the last black stamp.**
#  4. **The OK fraction over the un-blacked remainder is at least 79.9%** — P7's
#     measured share of depth frames posed. The plan's first false green: a system
#     that goes LOST and never comes back has a perfect "nothing fused while lost"
#     record, because nothing is fused at all.
#  5. **0 frames refused for want of a state** (`no_state`), and 0 UNKNOWN states on
#     the stream. Unknown is not OK, so fusion refuses it; a non-zero here is fusion
#     and odometry disagreeing about where the state is, which is the plan's second
#     false green — the state nobody reads — arriving at runtime rather than in YAML.
#
# **The control is one run with both refusals switched off**: fuse_while_lost:=true
# and recover_after_fits:=100000000. It must FAIL (2) — voxels integrated while LOST —
# and FAIL (4) — an OK fraction under the floor, because it never recovers. A floor
# nobody has watched exclude anything is not an assertion.
#
# What it does not touch: the pose odometry_node recovers *to*. Recovery here is
# odometric — the tracker resumes from the held pose with a fresh reference, so the
# map after a blackout is offset by whatever motion the blackout hid. Relocalising
# against the map is P20's question, not this one's.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-lost =="

BAG_NAME=${1:-desk1}
# Where the tracker is holding on its own, which on desk1 is not "mid-clip": it loses
# track by itself for 17.5-20 s and 24.4-25.5 s (two runs, 2026-10-02), and a lens
# cap at 20 s found it already LOST and passed (1) over nothing. 40 s is clear of
# both and of the clip's own ~400 ms stall at ~35 s; the gate asserts the state
# before the blackout was OK rather than trusting this comment.
BLACKOUT_START_S=${PIMESH_BLACKOUT_START_S:-40}
BLACKOUT_S=${PIMESH_BLACKOUT_S:-3}
# Recovery budget, in depth frames after the last black one. A fresh reference
# needs one frame to be taken and recover_after_fits more to be trusted; the rest is
# slack for the ORB tracker re-acquiring after ten frames of nothing (~0.6 s total).
MAX_TO_OK=${PIMESH_MAX_TO_OK:-10}
OK_FLOOR=0.799
NEVER=100000000

assert_no_session "bash tools/gates/lost.sh"
arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || { echo "FAIL: no model at ${MODEL} — bash tools/fetch-model.sh"; exit 1; }
BAG=
for cand in "$BAG_NAME" "$PIMESH_WS/bags/$BAG_NAME"; do
    [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
done
[[ -n $BAG ]] || { echo "FAIL: no bag at '${BAG_NAME}' — just record desk1 60"; exit 1; }
SECS=$(/usr/bin/python3 - "$BAG/metadata.yaml" <<'META'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
print(int(m['duration']['nanoseconds'] / 1e9) + 1)
META
)
work=$(mktemp -d)
echo "clip : ${BAG} (${SECS}s); blackout ${BLACKOUT_S}s from ${BLACKOUT_START_S}s"

# Last value of `key` on the node's `stats <line>` line.
stat_of() {              # $1 = log, $2 = node, $3 = line, $4 = key
    grep -h "$2" "$1" | grep -o "stats $3 .*" | tail -1 |
        awk -v key="$4" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}

run() {                  # $1 = log, then extra launch arguments
    local log=$1; shift
    timeout -s INT $(( SECS + 60 )) ros2 launch pimesh_bringup pimesh.launch.py \
        odom_regime:=sixdof blackout_start_s:="$BLACKOUT_START_S" blackout_s:="$BLACKOUT_S" \
        "$@" >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "fusion_node up:" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: the container never came up within 60 s"; tail -30 "$log"; return 1; }
    # The state stream, from outside the container: a few bytes at the depth rate, so
    # an out-of-process reader costs nothing — unlike an image topic — and it reads
    # what crossed the transport rather than what the node meant to send.
    timeout -s INT $(( SECS + 30 )) ros2 topic echo --csv /tracking/state \
        pimesh_msgs/msg/TrackingState </dev/null >"$log.csv" 2>"$log.echo" &
    local echo_pid=$!
    sleep 3
    timeout -s INT $(( SECS + 20 )) ros2 bag play "$BAG" \
        --disable-keyboard-controls </dev/null >"$log.play" 2>&1
    sleep 4
    kill -INT "$echo_pid" 2>/dev/null || true
    wait "$echo_pid" 2>/dev/null || true
    kill_local
    sleep 1
}

# The blackout's stamps, as decode_node logged them.
blackout_stamps() {      # $1 = log; prints "first last frames"
    local first last frames
    first=$(grep -o 'blackout start stamp_ns=[0-9]*' "$1" | head -1 | cut -d= -f2)
    last=$(grep -o 'last_black_ns=[0-9]*' "$1" | head -1 | cut -d= -f2)
    frames=$(grep -o 'blackout end .*frames=[0-9]*' "$1" | head -1 | grep -o 'frames=[0-9]*' | cut -d= -f2)
    echo "${first:-} ${last:-} ${frames:-}"
}

declare -A R
measure() {              # $1 = log, $2 = arm
    local log=$1 arm=$2 first last frames
    read -r first last frames < <(blackout_stamps "$log")
    if [[ -z $first || -z $last ]]; then
        note "${arm}: decode_node never logged the blackout — nothing was injected"
        return
    fi
    R[$arm.black_frames]=$frames
    while IFS='=' read -r key value; do
        R[$arm.$key]=$value
    done < <(/usr/bin/python3 "$PIMESH_WS/tools/eval/lost_timeline.py" "$log.csv" "$first" "$last")
    for key in refused_lost no_state integrated_lost voxels_lost integrated_held integrated voxels_total; do
        R[$arm.$key]=$(stat_of "$log" fusion_node lost "$key")
    done
    for key in lost_frames entered_lost recovered longest_hold_run lost_after_holds recover_after_fits; do
        R[$arm.$key]=$(stat_of "$log" odometry_node tracking "$key")
    done
}

echo "-- the real run --"
run "$work/lost.log" || { echo "FAIL gate-lost"; exit 1; }
measure "$work/lost.log" lost
echo "-- the control (fuse_while_lost:=true, recover_after_fits:=${NEVER}) --"
run "$work/control.log" fuse_while_lost:=true recover_after_fits:="$NEVER" || { echo "FAIL gate-lost"; exit 1; }
measure "$work/control.log" control

gt() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a + 0 > b + 0) }'; }
ge() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a + 0 >= b + 0) }'; }

# --- The real run ---------------------------------------------------------------
L=lost
(( ${R[$L.frames]:-0} > 300 )) || note "only ${R[$L.frames]:-0} states read off /tracking/state"
(( ${R[$L.blackout_frames]:-0} > 0 )) || note "no depth frame fell inside the blackout"
lost_after=${R[$L.lost_after_holds]:-}
[[ -n $lost_after ]] || note "odometry_node never logged its stats tracking line"
[[ ${R[$L.state_before]:-x} == 1 ]] ||
    note "the state before the blackout was ${R[$L.state_before]:-?}, not OK (1) — LOST already, so 'frames to LOST' would measure nothing"
if [[ ${R[$L.to_lost]:-"-1"} == -1 ]]; then
    note "LOST was never declared after the blackout started"
elif [[ -n $lost_after ]] && (( R[$L.to_lost] > lost_after )); then
    note "LOST took ${R[$L.to_lost]} depth frames, against lost_after_holds=${lost_after}"
fi
(( ${R[$L.refused_lost]:-0} > 0 )) ||
    note "fusion_node refused ${R[$L.refused_lost]:-no} LOST frames — the refusal never ran, so its zero means nothing"
[[ ${R[$L.voxels_lost]:-x} == 0 && ${R[$L.integrated_lost]:-x} == 0 ]] ||
    note "${R[$L.voxels_lost]:-?} voxels from ${R[$L.integrated_lost]:-?} frames integrated while LOST"
if [[ ${R[$L.to_ok]:-"-1"} == -1 ]]; then
    note "OK never came back after the blackout"
elif (( R[$L.to_ok] > MAX_TO_OK )); then
    note "OK took ${R[$L.to_ok]} depth frames after the blackout, against ${MAX_TO_OK}"
fi
ge "${R[$L.ok_fraction]:--1}" "$OK_FLOOR" ||
    note "OK over the un-blacked remainder is ${R[$L.ok_fraction]:-?}, under ${OK_FLOOR}"
[[ ${R[$L.no_state]:-x} == 0 ]] || note "fusion_node refused ${R[$L.no_state]:-?} frames for want of a state"
[[ ${R[$L.unknown]:-x} == 0 ]] || note "${R[$L.unknown]:-?} UNKNOWN states on the stream"

# --- The control, which must fail (2) and (4) ------------------------------------
C=control
gt "${R[$C.voxels_lost]:-0}" 0 ||
    note "the control integrated ${R[$C.voxels_lost]:-no} voxels while LOST — the '0 voxels' assertion has not been seen to fail"
ge "${R[$C.ok_fraction]:--1}" "$OK_FLOOR" &&
    note "the never-recovering control scored ${R[$C.ok_fraction]} OK, over the ${OK_FLOOR} floor — the floor excludes nothing"
[[ ${R[$C.recovered]:-x} == 0 ]] || note "the control recovered ${R[$C.recovered]:-?} times — it is not the control"

# =============================================================================
row() { printf '%-34s %14s %14s\n' "$1" "${R[lost.$2]:-}" "${R[control.$2]:-}"; }
echo
echo "================================ gate-lost ================================"
printf '%-34s %14s %14s\n' "" "real" "control"
row "states read off /tracking/state" frames
row "black frames published (decode)" black_frames
row "depth frames inside the blackout" blackout_frames
row "state before it (1 = OK)" state_before
row "depth frames to LOST" to_lost
row "depth frames to OK after it" to_ok
row "OK fraction, un-blacked" ok_fraction
row "posed fraction, un-blacked" posed_fraction
row "LOST frames outside the blackout" lost_outside
row "LOST frames (odometry)" lost_frames
row "times LOST was entered" entered_lost
row "times OK came back" recovered
row "longest run of holds" longest_hold_run
row "refused while LOST (fusion)" refused_lost
row "frames integrated while LOST" integrated_lost
row "voxels integrated while LOST" voxels_lost
row "integrated on a held pose (OK)" integrated_held
row "integrated, all" integrated
row "voxels, all" voxels_total
row "refused for want of a state" no_state
row "UNKNOWN states" unknown
echo "assert: OK before the blackout; LOST within lost_after_holds (${lost_after:-?}) frames; 0 voxels while LOST"
echo "        beside refused_lost > 0; OK within ${MAX_TO_OK} frames after; OK >= ${OK_FLOOR}"
echo "        un-blacked; 0 no_state, 0 UNKNOWN. The control must integrate voxels while"
echo "        LOST and score under the floor."
echo "==========================================================================="

if (( fail )); then echo "FAIL gate-lost"; exit 1; fi
echo "PASS gate-lost"
