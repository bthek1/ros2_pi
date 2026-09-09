#!/usr/bin/env bash
#
# P1 gate: the Pi puts 720p MJPEG on the LAN, stamped honestly, and refuses to
# idle when it cannot.
#
# Three claims, and the middle one is the reason this node exists at all.
#
#  1. **Rate.** >= 40 Hz measured *on the dev box*, not on the Pi. Capture that
#     the network cannot carry is not capture, and this is the only topic in the
#     project that crosses Wi-Fi.
#  2. **Stamps.** The stamp-vs-receipt offset must sit inside one frame interval
#     and must be the *same across two launches*. `usb_cam` 0.8.1 fails exactly
#     this: it converts the monotonic capture clock to the ROS clock through an
#     offset computed once per process, so every stamp in a session is displaced
#     by the same random sub-second amount, redrawn at each launch (measured at
#     0.223 / 0.362 / 0.979 s). One launch cannot see that bug — the offset is
#     constant within a run, so everything looks self-consistent. Two launches
#     is the smallest experiment that can.
#  3. **Failure.** A busy device exits non-zero, fast, with a message that names
#     the problem. A node that logs an error and idles is worse than one that
#     crashes, because it is discoverable, subscribed to, and publishing
#     nothing.
#
# The rate is measured with RViz closed and asserted to have exactly one
# subscriber, because a second RELIABLE reader on this topic is the thing the
# whole architecture is arranged to prevent: five of them collapsed the
# predecessor's link to ~2 frames/s each.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-capture =="

MIN_RATE_HZ=40
MAX_LAUNCH_DELTA_MS=5.0
# The dev-box offset is a *cross-machine* number: it contains the two hosts'
# NTP relationship as well as the flight time, and that relationship moves. It
# was measured at +8 ms and at -19 ms an hour apart on 2026-09-09, both with
# the node unchanged and the two-launch delta under 1.1 ms throughout. So this
# bound is not a latency budget — it is the "somebody computed an epoch once
# and got it wrong" detector, sized to sit far above NTP skew (tens of ms) and
# far below every usb_cam sample (223, 362 and 979 ms). The tight,
# single-clock claim is asserted on the Pi instead, below.
MAX_CROSS_MACHINE_OFFSET_MS=100.0
BUSY_EXIT_BUDGET_S=2.0
RATE_WINDOW_S=30
STAMP_WINDOW_S=10

arm_cleanup

fail=0
note() { echo "FAIL: $*"; fail=1; }

work=$(mktemp -d)

probe_value() {         # $1 = probe output file, $2 = key
    awk -F= -v key="$2" '$1 == "probe " key {print $2}' "$1"
}

# One launch of the node on the Pi, one measurement here. Returns with the
# probe's key=value output in $work/probe.<tag>.
#
# The node is bounded on the Pi by pi_run_for, which puts `timeout` inside the
# login shell so the signal reaches the node rather than the shell wrapping it —
# the other spelling orphans it, and an orphaned camera_node holds /dev/video0
# exclusively and fails every session after this one.
measure() {             # $1 = tag, $2 = probe window in seconds
    local tag=$1 window=$2
    pi_run_for $(( window + 15 )) "ros2 run pimesh_camera camera_node" \
        >"$work/camera.$tag" 2>&1 &
    local pi_job=$!

    # Discovery across the LAN takes a beat. The probe's own window opens at its
    # first frame, so this sleep only has to be long enough for the node to
    # exist, not long enough to be accurate.
    sleep 4

    if [[ $tag == rate ]]; then
        # Exactly one subscriber, and it is the probe. Checked while the stream
        # is live because that is the only time it is true of anything.
        ros2 topic info -v /image_raw/compressed >"$work/topicinfo" 2>&1 &
    fi

    timeout $(( window + 40 )) ros2 run pimesh_camera capture_probe \
        --ros-args -p "duration_s:=${window}.0" >"$work/probe.$tag" 2>"$work/probe.$tag.err"

    wait "$pi_job" 2>/dev/null || true
    kill_pi
    sleep 1
}

# 0. Nothing of ours already running, or every count below is somebody else's.
if pgrep -f "$PIMESH_NODE_PAT" >/dev/null 2>&1; then
    echo "FAIL: a pimesh node is already running here — close RViz and any probe first"
    exit 1
fi

# 1. Reset the camera. Not hygiene: V4L2 controls persist inside the camera
#    across processes and reboots, and exposure_dynamic_framerate costs ~10 fps
#    in indoor light. A rate measured without this step is a measurement of
#    whatever the last person left behind.
bash "$PIMESH_WS/tools/camera-reset.sh" >"$work/reset" 2>&1 || {
    echo "FAIL: camera-reset did not reach its baseline"; sed 's/^/  /' "$work/reset"; exit 1
}
exposure_mode=$(awk -F= '$1 == "camera-reset exposure_mode" {print $2}' "$work/reset")
dynamic_fps=$(awk -F= '$1 == "camera-reset exposure_dynamic_framerate" {print $2}' "$work/reset")

# 2. Launch one: the rate, over a full window.
measure rate "$RATE_WINDOW_S"

frames=$(probe_value "$work/probe.rate" frames)
unique=$(probe_value "$work/probe.rate" unique_frames)
rate=$(probe_value "$work/probe.rate" rate_hz)
bytes=$(probe_value "$work/probe.rate" bytes_mean)
offset_a=$(probe_value "$work/probe.rate" offset_median_ms)
p95_a=$(probe_value "$work/probe.rate" offset_p95_ms)

if [[ -z ${rate:-} || ${frames:-0} -eq 0 ]]; then
    echo "FAIL: no frames reached the dev box"
    sed 's/^/  /' "$work/camera.rate" | tail -20
    exit 1
fi

in_range "$rate" "$MIN_RATE_HZ" 200 ||
    note "rate was ${rate} Hz on the dev box, budget is >= ${MIN_RATE_HZ} Hz"

# A rate of repeated payloads is a rate at the DDS layer and a fiction about the
# sensor. The predecessor quoted 42-60 fps *with* "0 duplicate payloads in 634
# messages", and a frame rate here is quoted the same way or not at all.
[[ ${unique:-0} -eq ${frames:-1} ]] ||
    note "${frames} messages carried only ${unique} distinct payloads — the camera is repeating frames"

subs=$(awk '/^Subscription count:/ {print $3}' "$work/topicinfo" 2>/dev/null)
[[ ${subs:-0} -eq 1 ]] ||
    note "subscription count was ${subs:-unknown}, expected exactly 1 (is RViz open?)"

# 3. Launch two: the same measurement again, from a fresh process.
measure stamp "$STAMP_WINDOW_S"
offset_b=$(probe_value "$work/probe.stamp" offset_median_ms)
p95_b=$(probe_value "$work/probe.stamp" offset_p95_ms)
rate_b=$(probe_value "$work/probe.stamp" rate_hz)

[[ -n ${offset_b:-} ]] || { echo "FAIL: the second launch delivered no frames"; exit 1; }

frame_interval_ms=$(awk -v r="$rate" 'BEGIN {printf "%.2f", 1000.0 / r}')

# The two offsets measured here each contain three things, and only one of them
# is the camera: the true capture-to-arrival flight time, the gap between the
# two machines' NTP-disciplined system clocks, and the probe's own wakeup. So
# they get the loose bound — enough to catch a per-process epoch, not tight
# enough to be a claim about latency.
for pair in "1:$offset_a" "2:$offset_b"; do
    IFS=: read -r which value <<<"$pair"
    awk -v v="$value" -v limit="$MAX_CROSS_MACHINE_OFFSET_MS" \
        'BEGIN {exit !(v >= -limit && v <= limit)}' ||
        note "launch ${which} offset ${value} ms exceeds ${MAX_CROSS_MACHINE_OFFSET_MS} ms — that is an epoch error, not clock skew"
done

# There is deliberately no clock-skew figure printed beside those offsets.
# The obvious way to get one — bracket an `ssh pi date` between two local reads
# and take the midpoint — does not work over this link, and it fails in the
# direction that looks plausible: the remote `date` runs at the *end* of the
# exchange, after connection setup and a login shell, so the midpoint estimate
# comes back at roughly +RTT/2 regardless of the truth. Measured 2026-09-09 it
# reported 195-430 ms of skew while the offsets it was supposed to explain were
# -15 ms, which is not an imprecise answer but a contradictory one. A number
# that confident and that wrong in a gate's output is worse than no number, so
# the cross-host figures are reported for what they are — offsets carrying an
# unquantified clock term — and the claim about stamping is made on the Pi,
# where there is only one clock to carry.

# **The clean measurement.** Same machine, same clock: the probe runs on the Pi,
# so the stamp and the receipt come from one system clock and the cross-machine
# term is gone entirely. What is left is the real thing P1 claims — the interval
# between the kernel dequeuing a buffer and a subscriber holding the message
# built from it. *This* is what has to fit inside a frame interval, and it is
# the number to quote when somebody asks how honest the stamps are.
pi_run_for 30 "ros2 run pimesh_camera camera_node" >"$work/camera.local" 2>&1 &
pi_camera=$!
sleep 4
pi_run_for 25 "ros2 run pimesh_camera capture_probe --ros-args -p duration_s:=10.0" \
    >"$work/probe.local" 2>"$work/probe.local.err" || true
wait "$pi_camera" 2>/dev/null || true
kill_pi
sleep 1

offset_local=$(probe_value "$work/probe.local" offset_median_ms)
rate_local=$(probe_value "$work/probe.local" rate_hz)
if [[ -z ${offset_local:-} ]]; then
    note "the Pi-side probe delivered no frames — the single-clock stamp claim is untested"
    offset_local=unknown
else
    awk -v v="$offset_local" -v limit="$frame_interval_ms" \
        'BEGIN {exit !(v >= -limit && v <= limit)}' ||
        note "on the Pi, stamp-to-receipt was ${offset_local} ms, more than one frame interval (${frame_interval_ms} ms) — the stamp is not the capture time"
fi

# **The assertion this gate exists for.** Two launches of the same binary must
# agree, because there is nothing in this node that could differ between them:
# the stamp is `now - (monotonic_now - buffer_monotonic)`, an interval, with no
# per-process epoch to be wrong. usb_cam redraws exactly that epoch each launch
# and lands hundreds of milliseconds apart.
delta=$(awk -v a="$offset_a" -v b="$offset_b" 'BEGIN {printf "%.3f", (a > b) ? a - b : b - a}')
awk -v d="$delta" -v limit="$MAX_LAUNCH_DELTA_MS" 'BEGIN {exit !(d < limit)}' ||
    note "the two launches disagree by ${delta} ms, budget is < ${MAX_LAUNCH_DELTA_MS} ms — that is a per-process epoch"

# 4. Unplug by proxy. v4l2-ctl streaming holds the device exclusively, which is
#    the same condition a leaked camera_node from a previous session creates —
#    the failure this is really about.
pi_run "nohup timeout -s TERM 30 v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=100000 >/dev/null 2>&1 & sleep 2" >/dev/null 2>&1
busy_start=$(date +%s.%N)
busy_out=$(pi_run_for 15 "ros2 run pimesh_camera camera_node" 2>&1) && busy_rc=0 || busy_rc=$?
busy_wall=$(awk -v a="$busy_start" -v b="$(date +%s.%N)" 'BEGIN {printf "%.2f", b - a}')
pi_run "pkill -f 'v4l2-[c]tl' || true" >/dev/null 2>&1 || true

# Two different numbers, and conflating them is how this assertion first
# measured the wrong thing. The wall clock from here spans an SSH handshake, a
# login shell sourcing the Pi's ROS setup, and `ros2 run`'s Python startup —
# 2.7 s of which about 0.2 s belonged to the node. The claim in P1 is about the
# *node*: that it refuses rather than idles. So the budget is asserted against
# the node's own log timeline, first line to FATAL, and the end-to-end wall time
# is printed beside it so the slower number is visible rather than hidden.
busy_node_s=$(awk '
    match($0, /\[[0-9]+\.[0-9]+\]/) {
        ts = substr($0, RSTART + 1, RLENGTH - 2) + 0
        if (first == 0) { first = ts }
        if ($1 == "[FATAL]") { fatal = ts }
    }
    END { if (first > 0 && fatal > 0) printf "%.2f", fatal - first; else print "" }
' <<<"$busy_out")

[[ $busy_rc -ne 0 ]] || note "the node exited 0 against a busy device — it should refuse to start"
if [[ -z ${busy_node_s:-} ]]; then
    note "no FATAL line from the busy-device run — the node did not say why it stopped"
    busy_node_s=unknown
else
    awk -v e="$busy_node_s" -v limit="$BUSY_EXIT_BUDGET_S" 'BEGIN {exit !(e < limit)}' ||
        note "the node took ${busy_node_s}s to give up on a busy device, budget is < ${BUSY_EXIT_BUDGET_S}s"
fi
# The end-to-end path still has to terminate; it just is not a 2 s claim.
awk -v e="$busy_wall" 'BEGIN {exit !(e < 15.0)}' ||
    note "the busy-device run took ${busy_wall}s end to end — something hung rather than refused"
# The message matters as much as the exit code: "Device or resource busy" with
# no further help is what sends somebody looking for a software bug.
grep -q 'another process is streaming' <<<"$busy_out" ||
    note "the busy-device message does not name the cause: $(grep -m1 FATAL <<<"$busy_out")"

cleanup_both

echo
echo "exposure mode    : ${exposure_mode}  (exposure_dynamic_framerate=${dynamic_fps})"
echo "rate             : ${rate} Hz on the dev box over ${RATE_WINDOW_S}s  (assert >= ${MIN_RATE_HZ})"
echo "distinct frames  : ${unique} of ${frames}  (assert equal)"
echo "mean JPEG        : ${bytes} bytes  ($(awk -v b="$bytes" -v r="$rate" 'BEGIN {printf "%.1f", b * r / 1048576}') MB/s over wlan0)"
echo "subscribers      : ${subs}  (assert 1; RViz closed)"
echo "stamp, one clock : ${offset_local} ms measured on the Pi at ${rate_local} Hz"
echo "                   assert |offset| <= one frame interval = ${frame_interval_ms} ms"
echo "stamp, cross-host : launch1=${offset_a} ms (p95 ${p95_a})  launch2=${offset_b} ms (p95 ${p95_b})"
echo "                   assert |offset| < ${MAX_CROSS_MACHINE_OFFSET_MS} ms; carries an unmeasured dev-to-pi clock term"
echo "launch delta     : ${delta} ms  (assert < ${MAX_LAUNCH_DELTA_MS} — this is the usb_cam bug)"
echo "second launch    : ${rate_b} Hz over ${STAMP_WINDOW_S}s"
echo "busy device      : exit ${busy_rc}, node refused in ${busy_node_s}s  (assert non-zero, < ${BUSY_EXIT_BUDGET_S}s)"
echo "                   ${busy_wall}s end to end including ssh, login shell and ros2 run startup"

(( fail == 0 )) || { echo "FAIL gate-capture"; exit 1; }
echo "PASS gate-capture"
