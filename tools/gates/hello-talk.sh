#!/usr/bin/env bash
#
# P1 gate: the talker publishes at the rate it was asked for, and refuses a
# rate it was not.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-hello-talk =="

# Nothing this gate starts outlives it, including on the failure paths. The
# patterns match the node binary, never the `ros2 run` wrapper: killing the
# wrapper orphans the binary, which is how a workspace ends up with a publisher
# nobody can find on a topic somebody is still debugging.
arm_cleanup kill_local

start_node() {          # $1 = log path, rest = --ros-args ...
    local log=$1; shift
    timeout -s INT 30 ros2 run pimesh_hello hello_node "$@" >"$log" 2>&1 &
    for _ in $(seq 40); do
        if ros2 topic list 2>/dev/null | grep -qx /hello_node/hello; then return 0; fi
        sleep 0.25
    done
    echo "FAIL: /hello_node/hello never appeared within 10 s"; cat "$log"; return 1
}

# Sample for long enough that the --window 10 average is over ten real intervals
# rather than the two the tool starts with, and read the *last* line for the
# same reason.
#
# Through a file rather than a pipe, deliberately: `timeout ... | awk` puts awk
# in the signal's blast radius, so the timeout kills the parser along with the
# thing being parsed and END never runs. Measured — it reported every rate as
# empty. Two commands and a temp file have no such coupling.
#
# run_for rather than a bare `timeout` for the same reason every foreground one
# in this repo is: a plain `timeout` puts its child in another process group, so
# Ctrl-C during a 13 s sample reaches nothing and the gate cannot be interrupted
# until the sample ends.
measure_rate() {        # $1 = seconds to sample -> prints Hz
    local out; out=$(mktemp)
    run_for "$1" ros2 topic hz /hello_node/hello --window 10 >"$out" 2>/dev/null || true
    awk '/average rate/ { r = $3 } END { if (r == "") exit 1; print r }' "$out"
}

log=$(mktemp -d)/hello.log

# 1. The default rate is the declared default.
start_node "$log"
rate_default=$(measure_rate 13)
in_range "$rate_default" 0.9 1.1 ||
    { echo "FAIL: default rate ${rate_default} Hz outside 0.9–1.1"; exit 1; }
kill_local; sleep 1

# 2. The payload is the declared default, and it is exactly that.
start_node "$log"
rate_echo=$(measure_rate 13)
echo_out=$(mktemp)
run_for 10 ros2 topic echo --once --field data /hello_node/hello >"$echo_out" 2>/dev/null || true
text=$(head -1 "$echo_out")
[[ $text =~ ^hello\ world$ ]] ||
    { echo "FAIL: echoed \"${text}\", expected \"hello world\""; exit 1; }
in_range "$rate_echo" 0.9 1.1 ||
    { echo "FAIL: rate ${rate_echo} Hz outside 0.9–1.1 on the echo run"; exit 1; }
kill_local; sleep 1

# 3. The parameter is wired to the timer, not decorative. A node that accepts
#    rate_hz and ignores it passes every check but this one.
start_node "$log" --ros-args -p rate_hz:=5.0
rate_five=$(measure_rate 10)
in_range "$rate_five" 4.5 5.5 ||
    { echo "FAIL: rate_hz:=5.0 gave ${rate_five} Hz, outside 4.5–5.5"; exit 1; }
kill_local; sleep 1

# 4. A value outside the descriptor's range is refused at construction.
#    Clamping and carrying on would be the friendly thing to do and the wrong
#    one: a parameter that silently means something else is the bug class this
#    whole declaration style exists to prevent.
bad_start=$(date +%s.%N)
set +e
run_for 10 ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=0.0 >"$log" 2>&1
bad_exit=$?
set -e
bad_secs=$(awk -v a="$bad_start" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')
if (( bad_exit == 0 )); then
    echo "FAIL: rate_hz:=0.0 was accepted (exit 0)"; exit 1
fi
if awk -v s="$bad_secs" 'BEGIN { exit !(s > 2.0) }'; then
    echo "FAIL: rate_hz:=0.0 took ${bad_secs}s to fail, budget is 2.0s"; exit 1
fi
grep -q "floating point range" "$log" ||
    { echo "FAIL: exited non-zero but never named the range"; tail -3 "$log"; exit 1; }

echo
echo "rate @ default   : ${rate_default} Hz   (assert 0.9–1.1)"
echo "rate @ echo run  : ${rate_echo} Hz   (assert 0.9–1.1)"
echo "rate @ 5.0       : ${rate_five} Hz   (assert 4.5–5.5)"
echo "text             : \"${text}\""
echo "rate_hz:=0.0     : exit ${bad_exit} after ${bad_secs}s (assert non-zero, < 2.0s)"
echo "PASS gate-hello-talk"
