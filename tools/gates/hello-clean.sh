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

# replay is the fourth, and it is here because of how its failure looks rather
# than how expensive it is. What it leaves behind is a `ros2 bag play` on
# /image_raw/compressed — a *second source* of the pipeline's input topic, and
# an unattended one. A leaked viewer is visible and a leaked camera_node
# announces itself the next time anything opens /dev/video0; a leaked player
# announces nothing and quietly mixes recorded frames into whatever the next
# measurement thinks it is measuring. On 2026-09-12 two of them ran for four
# minutes while tools/stragglers.sh reported 0/0 and meant it, because no
# pattern in just-lib.sh matched a bag player at all. That hole is closed there;
# this is the half that proves the recipe's own trap reaches one.

# recipe -> the argv to run it with, and what "it is up" means for it.
RECIPES=(lan compose view-camera replay)

# `replay` is the only recipe here that takes an argument, and the bag it takes
# has to be *this gate's own*. bags/ is git-ignored, so on a fresh clone there
# is nothing in it; picking whatever happened to be there would make a core
# teardown gate fail for reasons that have nothing to do with teardown, and
# would make what it asserts depend on which machine it ran on.
#
# So the fixture is synthesized: three seconds of a throwaway topic, recorded
# into a temp directory and deleted on the way out. `replay` never reads the
# messages — it hands the bag to `ros2 bag play` and puts RViz in front of it —
# so the payload is irrelevant and a std_msgs/String is the cheapest thing that
# makes a valid bag. What is being tested is the process tree: a player, a
# launcher, three static transform publishers and a viewer, all of which must be
# gone three seconds after the signal.
GATE_BAG=

# `wait` with a deadline. Bash's builtin has no timeout, so a process that
# cannot act on the signal it was sent hangs the script rather than failing it —
# which is how the SIGINT/SIG_IGN case below stayed invisible for a ten-minute
# run that printed one line. Polling with kill -0 is the portable spelling.
_reap() {                   # $1 = pid, $2 = seconds to allow
    local pid=$1 secs=$2
    for _ in $(seq $(( secs * 10 ))); do
        kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
        sleep 0.1
    done
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    return 1
}

make_fixture_bag() {
    local dir; dir=$(mktemp -d)
    GATE_BAG="$dir/fixture"
    # `env --default-signal=INT,TERM,HUP` on both, and it is the same trap this
    # gate carries a paragraph about forty lines above — arriving, once again,
    # by a different door. A background command started by a *non-interactive*
    # shell inherits SIGINT as SIG_IGN, and this script is exactly that shell.
    # Without the reset, `kill -INT` on the recorder lands on a process that
    # cannot receive it, the `wait` below never returns, and the gate hangs
    # before it has measured anything — silently, since its next output is the
    # line after. Measured 2026-09-12: /proc/PID/status showed
    # SigIgn 0000000001001006 (bit 0x2 = SIGINT) without this.
    #
    # The publisher goes first, or the recorder opens on a topic that does not
    # exist yet and discovers nothing. Neither of these is a pimesh process, so
    # neither is covered by the straggler patterns — they are killed by pid
    # here, deliberately, and before the first measurement is taken.
    env --default-signal=INT,TERM,HUP \
        ros2 topic pub -r 20 /gate_clean_fixture std_msgs/msg/String '{data: x}' \
        >/dev/null 2>&1 &
    local pub=$!
    sleep 2
    env --default-signal=INT,TERM,HUP \
        ros2 bag record -s mcap -o "$GATE_BAG" --topics /gate_clean_fixture \
        </dev/null >/dev/null 2>&1 &
    local rec=$!
    sleep 3
    # SIGINT, not SIGTERM: the recorder finalizes its storage on an interrupt
    # and a bag without metadata.yaml is one tools/replay.sh refuses by design.
    # Bounded, because a `wait` that does not return is how this was found.
    kill -INT "$rec" 2>/dev/null || true
    _reap "$rec" 10 || { echo "FAIL: the fixture recorder ignored SIGINT"; return 1; }
    kill -INT "$pub" 2>/dev/null || true
    _reap "$pub" 10 || true
    [[ -r "$GATE_BAG/metadata.yaml" ]] || {
        echo "FAIL: could not build a fixture bag at ${GATE_BAG}"
        return 1
    }
    GATE_BAG_DIR=$dir
}
GATE_BAG_DIR=
cleanup_fixture() { [[ -n $GATE_BAG_DIR ]] && rm -rf "$GATE_BAG_DIR"; }
trap cleanup_fixture EXIT

argv_for() {                # $1 = recipe, $2 = seconds; prints one argv word per line
    case $1 in
        replay)      printf '%s\n' "$PIMESH_WS/tools/replay.sh" "$GATE_BAG" "$2" ;;
        view-camera) printf '%s\n' "$PIMESH_WS/tools/view-camera.sh" "$2" ;;
        *)           printf '%s\n' "$PIMESH_WS/tools/hello-$1.sh" "$2" ;;
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
        # The player *and* the viewer, for the same reason as view-camera: the
        # player is up within a second and RViz takes several, so waiting on the
        # player alone would signal the session before the viewer existed — and
        # a viewer that was never running is trivially not a straggler. This
        # gate has been fooled by exactly that shape of "pass" once already.
        replay)  pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                 pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 ;;
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
    # argv as an array rather than a single word: replay takes a bag path before
    # its seconds, and a path from mktemp -d is exactly the kind of thing that
    # must not be re-split by a shell on its way through two of them.
    local -a argv; mapfile -t argv < <(argv_for "$2" 45)
    setsid env --default-signal=INT,TERM,HUP \
        bash -c 'echo $$ >"$1"; shift; exec bash "$@"' _ "$pgidfile" "${argv[@]}" \
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

# Built after the clean-slate check above, not before: the fixture starts two
# short-lived ros2 processes of its own, and a gate that cannot tell its own
# leak from somebody else's has to do its looking while nothing of its own is
# running.
make_fixture_bag || exit 1
echo "fixture bag      : ${GATE_BAG} ($(du -sh "$GATE_BAG" | cut -f1), for replay)"

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
