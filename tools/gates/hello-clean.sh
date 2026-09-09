#!/usr/bin/env bash
#
# Teardown gate: Ctrl-C and a closed window both leave nothing running, on
# either machine — for *every* recipe a person sits and watches.
#
# It was written as the hello-world plan's P4 and keeps that name, because the
# name is what the closed issue's build log refers to. Its subject was never
# hello-world though: it is the repo-wide rule that a session ends when you end
# it. So the table below grows with the justfile's `run` group, and the reason
# it must is the reason this gate exists in the first place. It passed for a
# week over a broken `hello-compose` because it only ever signalled
# `hello-lan` — a green gate over broken behaviour, which is worse than no gate,
# because it is a false claim with a script's authority behind it. Every recipe
# a person can Ctrl-C belongs here; the question to ask of this file is always
# what it does *not* touch.

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
#
# view-camera is the third, and it is the one with the most to lose. It spans
# both machines like hello-lan, runs its viewer in the foreground like
# hello-compose, and — unlike either — the thing it leaves behind on the Pi
# holds /dev/video0 *exclusively*. A leaked camera_node does not merely linger;
# it makes every later session in this project die with "Device or resource
# busy", including the ones that would have diagnosed it.

# recipe -> the script under tools/, and what "it is up" means for it.
RECIPES=(lan compose view-camera)
script_for() {              # $1 = recipe
    case $1 in
        view-camera) echo "$PIMESH_WS/tools/view-camera.sh" ;;
        *)           echo "$PIMESH_WS/tools/hello-$1.sh" ;;
    esac
}

session_up() {              # $1 = recipe
    case $1 in
        # Both ends, or killing them proves nothing.
        lan)     pgrep -f "$PIMESH_NODE_PAT" >/dev/null 2>&1 &&
                 pi_run "pgrep -f '$PIMESH_NODE_PAT'" >/dev/null 2>&1 ;;
        compose) pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 ;;
        # The camera on the Pi *and* the viewer here. Waiting on only one of
        # them would signal the session before the other had started, and a
        # process that was never running is trivially not a straggler.
        view-camera) pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 &&
                     pi_run "pgrep -f '$PIMESH_NODE_PAT'" >/dev/null 2>&1 ;;
    esac
}

run_and_signal() {          # $1 = INT or HUP, $2 = recipe
    # 45 s: longer than this gate takes, so the session never ends on its own
    # timer. A recipe that outlives the signal has to be killed *by* the signal
    # for the straggler count below to mean anything.
    # The session reports its own process group id, rather than this shell
    # deducing it from $!. `setsid` forks whenever it finds itself already a
    # process group leader — which depends on whether the *calling* shell has
    # job control on, so it happens in some contexts and not others. When it
    # does fork, the pid in $! belongs to a setsid that has already exited,
    # `ps -o pgid=` prints nothing, and the kill below becomes
    # `kill -INT -` — an error swallowed by `|| true`. The session then runs to
    # its own timer and dies of old age, and this gate reports 0 stragglers and
    # PASS having signalled nothing at all. That is the same shape of false
    # green this file already carries a paragraph about, so it does not get to
    # happen twice: after setsid, the new leader's own $$ *is* the pgid, and it
    # writes it down before exec'ing the script.
    local pgidfile; pgidfile=$(mktemp)
    setsid env --default-signal=INT,TERM,HUP \
        bash -c 'echo $$ >"$1"; exec bash "$2" 45' _ "$pgidfile" "$(script_for "$2")" \
        >"$log" 2>&1 &
    local pgid up=0
    for _ in $(seq 50); do
        pgid=$(cat "$pgidfile" 2>/dev/null || true)
        [[ -n ${pgid:-} ]] && break
        sleep 0.1
    done
    if [[ -z ${pgid:-} ]]; then
        echo "FAIL: the $2 session never reported a process group to signal"
        return 1
    fi

    for _ in $(seq 45); do
        if session_up "$2"; then up=1; break; fi
        sleep 1
    done
    if [[ $up -ne 1 ]]; then
        echo "FAIL: $2 never came up, so there was nothing to kill"
        tail -20 "$log"; return 1
    fi

    # A group kill that names no group is the failure this function was just
    # rewritten to prevent, so it is checked rather than swallowed.
    if ! kill -"$1" -"$pgid" 2>/dev/null; then
        echo "FAIL: could not send SIG$1 to process group ${pgid} for $2"
        return 1
    fi
    sleep 3
    return 0
}

counts=""
for what in "${RECIPES[@]}"; do
    for sig in INT HUP; do
        run_and_signal "$sig" "$what" || exit 1
        # tools/stragglers.sh is the assertion: it prints a count per host and
        # exits non-zero if any survived.
        out=$(bash "$stragglers" 2>&1) && rc=0 || rc=$?
        echo "--- ${what} after SIG${sig} ---"
        echo "$out"
        if [[ $rc -ne 0 ]]; then
            echo "FAIL: SIG${sig} left processes behind after ${what}"
            exit 1
        fi
        counts+="${what}/SIG${sig}: $(grep -o '[0-9]*$' <<<"$out" | tr '\n' '/' | sed 's:/$::')  "
    done
done

echo
echo "survivors dev/pi : ${counts}"
echo "                   (assert 0/0 after every signal, for all ${#RECIPES[@]} recipes)"
echo "PASS gate-hello-clean"
