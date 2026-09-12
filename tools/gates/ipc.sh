#!/usr/bin/env bash
#
# P2 gate: one reader on the Wi-Fi topic, one decode, and the decoded frame
# handed downstream as a pointer.
#
# Two claims, and they are different kinds of claim.
#
#  1. **Exactly one subscriber on `/image_raw/compressed`.** This is the Wi-Fi
#     constraint the whole architecture is shaped around: five RELIABLE readers
#     each pull their own unicast copy and collapsed the predecessor's link to
#     ~2 frames/s per reader against 14.7 Hz for one. It is asserted against the
#     *real* stream from the Pi rather than a bag, because a bag replayed locally
#     cannot fail this way and so cannot test it.
#  2. **The 2.7 MB decoded frame is not copied.** decode_node logs the address of
#     the buffer it publishes; IpcProbe, loaded into the same container, logs the
#     address it was handed. Under intra-process comms these are the same object.
#
# The second claim needs a control and this is the whole design of the gate.
# Address equality on its own is not evidence: two allocations in one process can
# coincide — one did, at 1/22, in the hello-world run on 2026-09-08 — so the
# claim is not "the addresses matched" but "the addresses matched *and stopped
# matching the moment the mechanism was switched off*". Same binary, same launch
# file, one argument.
#
# Both halves of the mechanism are in the code and neither announces itself: the
# publisher moves a `unique_ptr` into `publish()` and the subscription callback
# takes a `unique_ptr`. A `const &` callback works perfectly and quietly copies,
# and nothing in any log, topic tool or rate measurement says so.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-ipc =="

MIN_FRAMES=20
MAX_CONTROL_MATCH_PCT=25
MEASURE_S=8

arm_cleanup

fail=0
note() { echo "FAIL: $*"; fail=1; }

tmp=$(mktemp -d)

# One run: the camera on the Pi, the container here, the probe loaded into it.
#
# The camera is the Pi's real one and not a bag, because claim 1 is about a
# subscriber on a topic that crosses Wi-Fi. pi_run_for puts `timeout` inside the
# login shell so the limit reaches the node rather than the shell wrapping it —
# a camera_node orphaned here holds /dev/video0 exclusively and every later
# session in this project dies with "Device or resource busy".
start_run() {           # $1 = log path, $2 = true|false (intra-process)
    pi_run_for $(( MEASURE_S + 25 )) "ros2 run pimesh_camera camera_node" \
        >"$1.camera" 2>&1 &

    # probe:=true loads IpcProbe through the container's load service;
    # log_payloads:=true makes decode_node print the publisher half of the
    # evidence. `ros2 launch` has no way to override one node's parameter from the
    # command line, which is why that is a launch argument at all.
    timeout -s INT $(( MEASURE_S + 22 )) ros2 launch pimesh_bringup pimesh.launch.py \
        probe:=true log_payloads:=true intra_process:="$2" >"$1" 2>&1 &

    for _ in $(seq 60); do
        if ros2 node list 2>/dev/null | grep -qx /ipc_probe; then return 0; fi
        sleep 0.5
    done
    echo "FAIL: /ipc_probe never appeared within 30 s"
    tail -30 "$1"
    return 1
}

# How many of the addresses the probe logged were addresses decode_node logged.
# Under intra-process that is every one of them, because it is the same object;
# under serialisation it is whatever malloc happens to hand back, which is not
# guaranteed to be zero — hence the control run.
match_rate() {          # $1 = log -> prints "<hits> <total>"
    awk '
        /decode seq=/ {
            if (match($0, /payload=0x[0-9a-f]+/))
                pub[substr($0, RSTART + 8, RLENGTH - 8)] = 1
        }
        /probe n=/ {
            if (match($0, /payload=0x[0-9a-f]+/)) {
                a = substr($0, RSTART + 8, RLENGTH - 8)
                total++
                if (a in pub) hits++
            }
        }
        # The trailing newline matters: `read` returns 1 at EOF without one, and
        # under `set -e` that ends the gate with no message at all.
        END { printf "%d %d\n", hits + 0, total + 0 }
    ' "$1"
}

subscriber_count() {    # $1 = topic
    # `-v` lists every endpoint; the count line is the one number that cannot be
    # got wrong by miscounting stanzas.
    ros2 topic info -v "$1" 2>/dev/null |
        awk -F': ' '/^Subscription count:/ {print $2; exit}'
}

# ---- Run 1: intra-process on, the claim -------------------------------------
start_run "$tmp/on.log" true || exit 1

# One process holding both components. A container per component would satisfy
# every address assertion below by failing to have an inside at all.
containers=$(pgrep -fc "$PIMESH_CONTAINER_PAT" || echo 0)
[[ $containers -eq 1 ]] ||
    note "expected 1 component container process, found ${containers}"

# The parameter applied, not merely present in a file: input_topic is the key
# that decides what this node reads, and a mis-keyed YAML would leave it on the
# code default with nothing saying so.
input_topic=$(ros2 param get /decode_node input_topic 2>&1 | awk '{print $NF}')
[[ $input_topic == /image_raw/compressed ]] ||
    note "decode_node.input_topic is '${input_topic}', expected /image_raw/compressed"

sleep "$MEASURE_S"

# Claim 1, measured while the stream is live and the container is up. RViz is not
# running and must not be: it is a second reader on this topic, which is the
# thing being asserted against.
subs=$(subscriber_count /image_raw/compressed)
[[ -n $subs ]] || subs=0
(( subs == 1 )) ||
    note "/image_raw/compressed has ${subs} subscriber(s), expected exactly 1 — \
a second reader on the Wi-Fi topic is what this architecture is shaped to prevent"

# **At least two consumers on the decoded topic, and this assertion exists because
# its absence made this gate report a false green.**
#
# On 2026-09-12 this gate passed at 429/429 with one consumer — the probe — and the
# very next run, with keypoint_node added beside it, reported **0/574**. rclcpp
# serves ownership-taking intra-process subscriptions by moving the buffer into the
# last one and *copying it for every other*, so a pipeline with one consumer cannot
# exhibit the failure and a gate measuring one consumer cannot see it. The fix was
# the subscriber signature (a shared const pointer, which rclcpp hands to all of
# them); the lesson is that the interesting case is the fan-out, and a gate that
# measures the easy configuration is a gate whose green means nothing the day a
# stage is added.
decoded_subs=$(subscriber_count /image_raw)
[[ -n $decoded_subs ]] || decoded_subs=0
(( decoded_subs >= 2 )) ||
    note "/image_raw has ${decoded_subs} subscriber(s); this gate needs at least 2 \
to be measuring the fan-out case at all — with one consumer the copy it looks for \
cannot happen"

stats_line=$(grep -h 'stats in=' "$tmp/on.log" | tail -1 | sed 's/.*decode_node]: //')

cleanup_both; sleep 1

read -r on_hits on_total < <(match_rate "$tmp/on.log")
if (( on_total < MIN_FRAMES )); then
    note "only ${on_total} frames reached the probe, need >= ${MIN_FRAMES} to say anything"
    tail -20 "$tmp/on.log"
    echo "--- the Pi's camera said: ---"
    tail -10 "$tmp/on.log.camera"
elif (( on_hits != on_total )); then
    note "intra-process on, only ${on_hits}/${on_total} addresses matched"
fi

on_pub=$(grep -m1 -o 'decode seq=[0-9]* payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*' || true)
on_probe=$(grep -m1 -o 'probe n=[0-9]* payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*' || true)

# ---- Run 2: intra-process off, the control ---------------------------------
#
# Without this the gate proves nothing. The same launch file, the same two
# components in the same process, one argument different — and every frame now
# goes out through the middleware and comes back as a fresh allocation.
start_run "$tmp/off.log" false || exit 1
sleep "$MEASURE_S"
cleanup_both; sleep 1

read -r off_hits off_total < <(match_rate "$tmp/off.log")
off_pct=0
if (( off_total < MIN_FRAMES )); then
    note "control run delivered ${off_total} frames, need >= ${MIN_FRAMES}"
else
    off_pct=$(( 100 * off_hits / off_total ))
    (( off_pct <= MAX_CONTROL_MATCH_PCT )) ||
        note "control matched ${off_hits}/${off_total} (${off_pct}%) — at that rate \
address equality is coincidence and run 1 proves nothing"
fi

echo
echo "decode_node.input_topic   : ${input_topic}  (assert the YAML key applied)"
echo "container processes       : ${containers}  (assert exactly 1)"
echo "/image_raw/compressed subs: ${subs}  (assert exactly 1 — the Wi-Fi constraint)"
echo "/image_raw subs           : ${decoded_subs}  (assert >= 2 — the fan-out is the case that can fail)"
echo "first published buffer    : ${on_pub}"
echo "first buffer probed       : ${on_probe}"
echo "intra-process ON          : ${on_hits}/${on_total} addresses matched (assert all)"
echo "intra-process OFF         : ${off_hits}/${off_total} = ${off_pct}% (assert <= ${MAX_CONTROL_MATCH_PCT}%)"
echo "decode throughput         : ${stats_line:-none logged}"

(( fail == 0 )) || { echo "FAIL gate-ipc"; exit 1; }
echo "PASS gate-ipc"
