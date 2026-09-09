#!/usr/bin/env bash
#
# P4 gate: Ctrl-C and a closed window both leave nothing running, on either
# machine.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-hello-clean =="

log=$(mktemp -d)/clean.log
stragglers="$PIMESH_WS/tools/stragglers.sh"

# Start with a clean slate, or the gate cannot tell its own leak from somebody
# else's.
if ! bash "$stragglers" >/dev/null 2>&1; then
    echo "FAIL: something was already running before the gate started"
    bash "$stragglers" || true
    exit 1
fi

# setsid so the session gets its own process group. That is what makes the kill
# below realistic: a terminal sends Ctrl-C to the foreground *group*, not to one
# pid, and a trap that only covers the pid it was installed on would pass a
# weaker test than the one it has to survive.
#
# The script, not `just hello-lan`: the trap under test is the script's, and
# going through `just` only adds a process between the group and the trap —
# plus a dependency on `just` being on PATH inside a fresh session.
#
# `env --default-signal` is what makes this a Ctrl-C rather than a near miss,
# and it is the sharpest thing in this gate. A background command started by a
# *non-interactive* shell inherits SIGINT and SIGQUIT set to SIG_IGN, and bash
# will not install a handler for a signal that was ignored on entry — so
# `trap … INT` in the launched script silently does nothing and the session
# shrugs the signal off. Measured 2026-09-09: /proc/PID/status showed
# SigIgn 0x6 without this and 0x4 with it. A terminal's Ctrl-C reaches a
# foreground job whose SIGINT is at its default, so resetting the disposition
# is not a workaround — it is the only spelling that tests the real case. (The
# earlier version of this gate went through `just`, which reset the disposition
# for its child as a side effect, and so passed for a reason it never stated.)
# Both watchable recipes, because they fail differently and only one of them
# was ever tested here. hello-lan.sh spans the LAN and backgrounds its timeout,
# so its trap runs the moment the signal lands and the interesting question is
# whether cleanup reaches the Pi. hello-compose.sh runs its launcher in the
# *foreground*, where a plain `timeout` moves ros2 launch into a process group
# of its own — outside the one a terminal signals — and bash will not run a trap
# until its foreground child returns. Measured 2026-09-09: `just hello-compose`
# ignored six Ctrl-Cs and ended on its own when the 30 s timer expired. This
# gate said PASS throughout, because it only ever started the other script.
session_up() {              # $1 = lan or compose
    case $1 in
        # Both ends, or killing them proves nothing.
        lan)     pgrep -f "$PIMESH_NODE_PAT" >/dev/null 2>&1 &&
                 pi_run "pgrep -f '$PIMESH_NODE_PAT'" >/dev/null 2>&1 ;;
        compose) pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 ;;
    esac
}

run_and_signal() {          # $1 = INT or HUP, $2 = lan or compose
    # 45 s: longer than this gate takes, so the session never ends on its own
    # timer. A recipe that outlives the signal has to be killed *by* the signal
    # for the straggler count below to mean anything.
    setsid env --default-signal=INT,TERM,HUP \
        bash "$PIMESH_WS/tools/hello-$2.sh" 45 >"$log" 2>&1 &
    local launcher=$! pgid up=0
    pgid=$(ps -o pgid= -p "$launcher" | tr -d ' ')

    for _ in $(seq 45); do
        if session_up "$2"; then up=1; break; fi
        sleep 1
    done
    if [[ $up -ne 1 ]]; then
        echo "FAIL: hello-$2 never came up, so there was nothing to kill"
        tail -20 "$log"; return 1
    fi

    kill -"$1" -"$pgid" 2>/dev/null || true
    sleep 3
    return 0
}

counts=""
for what in lan compose; do
    for sig in INT HUP; do
        run_and_signal "$sig" "$what"
        # tools/stragglers.sh is the assertion: it prints a count per host and
        # exits non-zero if any survived.
        out=$(bash "$stragglers" 2>&1) && rc=0 || rc=$?
        echo "--- hello-${what} after SIG${sig} ---"
        echo "$out"
        if [[ $rc -ne 0 ]]; then
            echo "FAIL: SIG${sig} left processes behind after hello-${what}"
            exit 1
        fi
        counts+="${what}/SIG${sig}: $(grep -o '[0-9]*$' <<<"$out" | tr '\n' '/' | sed 's:/$::')  "
    done
done

echo
echo "survivors dev/pi : ${counts}"
echo "                   (assert 0/0 after every signal, for both recipes)"
echo "PASS gate-hello-clean"
