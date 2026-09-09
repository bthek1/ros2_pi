#!/usr/bin/env bash
#
# P2 gate: two components, one process, and the message handed over as a
# pointer.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-hello-ipc =="

arm_cleanup kill_local

start_container() {     # $1 = log path, $2 = true|false
    timeout -s INT 40 ros2 launch pimesh_hello hello.launch.py \
        intra_process:="$2" >"$1" 2>&1 &
    for _ in $(seq 60); do
        if ros2 node list 2>/dev/null | grep -qx /echo_node; then return 0; fi
        sleep 0.25
    done
    echo "FAIL: /echo_node never appeared within 15 s"; cat "$1"; return 1
}

# How many of the addresses the subscriber logged were addresses the publisher
# logged. Under intra-process this is every one of them, because it is the same
# object; under serialisation it is whatever malloc happens to hand back, which
# is not zero — hence the control run below.
match_rate() {          # $1 = log -> prints "<hits> <total>"
    awk '
        /pub seq=/ {
            if (match($0, /payload=0x[0-9a-f]+/))
                pub[substr($0, RSTART + 8, RLENGTH - 8)] = 1
        }
        /echo n=/ {
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

tmp=$(mktemp -d)

# ---- Run 1: intra-process on, the claim ------------------------------
start_container "$tmp/on.log" true

# Both components, and exactly one process holding them. A container per
# component would pass every other assertion in this gate and none of the ones
# the architecture actually needs.
nodes=$(ros2 node list 2>/dev/null | sort | tr '\n' ' ')
for n in /hello_node /echo_node; do
    grep -q -- "$n" <<<"$nodes" || { echo "FAIL: $n missing from: $nodes"; exit 1; }
done
containers=$(pgrep -fc "$PIMESH_CONTAINER_PAT" || echo 0)
if [[ $containers -ne 1 ]]; then
    echo "FAIL: expected 1 component_container_mt process, found ${containers}"; exit 1
fi
pid=$(pgrep -f "$PIMESH_CONTAINER_PAT" | head -1)

# The YAML applied, not merely loaded. `rate_hz` is 2.0 in config/hello.yaml and
# 1.0 in the code; asking the running node which one it has is the only question
# whose answer distinguishes a keyed file from a mis-keyed one.
param_out=$(ros2 param get /hello_node rate_hz 2>&1)
rate=$(awk '{ print $NF }' <<<"$param_out")
awk -v v="$rate" 'BEGIN { exit !(v > 1.99 && v < 2.01) }' ||
    { echo "FAIL: rate_hz is ${rate}, expected 2.0 from config/hello.yaml"; exit 1; }

sleep 8
kill_local; sleep 1
read -r on_hits on_total < <(match_rate "$tmp/on.log")
if [[ $on_total -lt 5 ]]; then
    echo "FAIL: only ${on_total} messages received, need ≥ 5 to say anything"; exit 1
fi
if [[ $on_hits -ne $on_total ]]; then
    echo "FAIL: intra-process on, only ${on_hits}/${on_total} addresses matched"; exit 1
fi
on_pub=$(grep -m1 -o 'pub seq=1 payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*')
on_echo=$(grep -m1 -o 'echo n=1 .*payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*')

# ---- Run 2: intra-process off, the control ---------------------------
#
# Without this the gate proves nothing. Two allocations in one process can share
# an address by coincidence — the publisher frees, the subscriber allocates the
# same size, and malloc obliges. So the claim is not "the addresses matched", it
# is "the addresses matched *and stop matching the moment the mechanism is
# switched off*". Same binary, same launch file, one flag.
start_container "$tmp/off.log" false
sleep 8
kill_local; sleep 1
read -r off_hits off_total < <(match_rate "$tmp/off.log")
if [[ $off_total -lt 5 ]]; then
    echo "FAIL: control run received ${off_total} messages, need ≥ 5"; exit 1
fi
off_pct=$(( 100 * off_hits / off_total ))
if [[ $off_pct -gt 25 ]]; then
    echo "FAIL: control matched ${off_hits}/${off_total} (${off_pct}%) — at that"
    echo "      rate address equality is coincidence, and run 1 proves nothing"
    exit 1
fi

echo
echo "nodes             : ${nodes}"
echo "container pid     : ${pid}  (assert exactly 1 process)"
echo "rate_hz           : ${rate}  (assert 2.0, the YAML value, not 1.0)"
echo "first pub payload : ${on_pub}"
echo "first echo payload: ${on_echo}"
echo "intra-process ON  : ${on_hits}/${on_total} addresses matched (assert all)"
echo "intra-process OFF : ${off_hits}/${off_total} = ${off_pct}% (assert ≤ 25%)"
echo "PASS gate-hello-ipc"
