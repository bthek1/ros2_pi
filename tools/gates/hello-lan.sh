#!/usr/bin/env bash
#
# P3 gate: one source tree, two distros, messages across the LAN.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-hello-lan =="

arm_cleanup

pi_env() { pi_run "echo \$$1"; }

# 1. Preflight. Both hosts on the same domain with the same middleware, or every
#    later number in this gate is measuring the wrong graph.
here_domain=${ROS_DOMAIN_ID:-unset}; here_rmw=${RMW_IMPLEMENTATION:-unset}
pi_domain=$(pi_env ROS_DOMAIN_ID); pi_rmw=$(pi_env RMW_IMPLEMENTATION)
for pair in "dev:$here_domain:42" "pi:$pi_domain:42" \
            "dev:$here_rmw:rmw_cyclonedds_cpp" "pi:$pi_rmw:rmw_cyclonedds_cpp"; do
    IFS=: read -r who got want <<<"$pair"
    [[ $got == "$want" ]] || { echo "FAIL: ${who} has ${got}, expected ${want}"; exit 1; }
done

# The daemon caches the discovery graph and will happily show a stale one, which
# is how a fix that worked looks like a fix that did not.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
pi_run "ros2 daemon stop; ros2 daemon start" >/dev/null 2>&1 || true

# 2. The same source builds on both, from source, on each machine's own ROS.
t0=$(date +%s.%N); bash "$PIMESH_WS/tools/build.sh" >/dev/null; t1=$(date +%s.%N)
here_build=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')
t0=$(date +%s.%N); bash "$PIMESH_WS/tools/build-pi.sh" >/dev/null; t1=$(date +%s.%N)
pi_build=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')

# 3. Different distros, or the cross-distro property was never exercised.
here_distro=$ROS_DISTRO
pi_distro=$(pi_env ROS_DISTRO)
if [[ $here_distro == "$pi_distro" ]]; then
    echo "FAIL: both hosts report ${here_distro} — nothing cross-distro was tested"
    exit 1
fi

# 4. Twenty seconds of 2 Hz across the LAN.
#
# The payload carries the Pi's hostname because `ros2 topic info -v` will not:
# it reports the publisher's *node* name, which is `hello_node` no matter which
# machine it runs on. Provenance has to be in the message.
marker="hello-from-$(pi_run hostname)"
log=$(mktemp -d)/lan.log

if pgrep -f "$PIMESH_NODE_PAT" >/dev/null 2>&1; then
    echo "FAIL: a pimesh_hello node is already running here — the count would be a lie"
    exit 1
fi

timeout -s INT 40 ros2 run pimesh_hello echo_node >"$log" 2>&1 &
sleep 3
pi_ws_run "timeout -s INT 20 ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=2.0 -p text:=$marker" >/dev/null 2>&1 &
pi_job=$!

sleep 10
info=$(ros2 topic info -v /hello_node/hello 2>&1 || true)
pub_count=$(awk '/^Publisher count:/ { print $3 }' <<<"$info")

wait "$pi_job" 2>/dev/null || true
sleep 2
cleanup_both

received=$(grep -c 'echo n=' "$log" || true)
payload=$(grep -m1 -o 'payload=0x[0-9a-f]* "[^"]*"' "$log" | sed 's/.*"\(.*\)"/\1/' || true)

if [[ ${pub_count:-0} -ne 1 ]]; then
    echo "FAIL: publisher count was ${pub_count:-none}, expected 1"; exit 1
fi
if [[ $payload != "$marker" ]]; then
    echo "FAIL: payload \"${payload}\" is not \"${marker}\" — that is not the Pi's talker"
    exit 1
fi
if [[ $received -lt 34 ]]; then
    echo "FAIL: received ${received} messages in 20 s at 2 Hz, budget is 34 of 40"
    exit 1
fi

echo
echo "distros          : dev=${here_distro}  pi=${pi_distro}  (assert different)"
echo "build times      : dev=${here_build}s  pi=${pi_build}s  (same source, both from source)"
echo "domain / rmw     : ${here_domain} / ${here_rmw} on both hosts"
echo "publisher count  : ${pub_count}  (assert 1)"
echo "payload          : \"${payload}\"  (assert \"${marker}\")"
echo "received         : ${received} of 40 in 20 s at 2 Hz  (assert >= 34)"
echo "PASS gate-hello-lan"
