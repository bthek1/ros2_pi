#!/usr/bin/env bash
#
# Teardown gate: Ctrl-C and a closed window both leave nothing running, on
# either machine — for *every* recipe a person sits and watches.
#
# It was written as the hello-world plan's P4 and was called `hello-clean.sh`
# until 2026-09-23; issue #2's build log refers to it by that name. Its subject
# was never hello-world, which is why it outlived that package: it is the
# repo-wide rule that a session ends when you end it. So the table below grows
# with the justfile's `run` group, and the reason it must is the reason this
# gate exists in the first place. It passed for a week over a broken
# `hello-compose` because it only ever signalled `hello-lan` — a green gate over
# broken behaviour, which is worse than no gate, because it is a false claim
# with a script's authority behind it. Every recipe a person can Ctrl-C belongs
# here; the question to ask of this file is always what it does *not* touch.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-teardown =="

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
# **Two teardown shapes fail differently, and this gate once covered only one.**
# A recipe that backgrounds its timeout runs its trap the moment the signal
# lands, and the interesting question is whether cleanup reaches the Pi. A
# recipe that runs its launcher in the *foreground* is the harder case: a plain
# `timeout` moves ros2 launch into a process group of its own — outside the one
# a terminal signals — and bash will not run a trap until its foreground child
# returns. Measured 2026-09-09 against the since-deleted `hello-compose`: it
# ignored six Ctrl-Cs and ended on its own when the 30 s timer expired, while
# this gate said PASS throughout, because it only ever started the other script.
# That is what `run_for` exists for, and why every recipe below is started here
# rather than trusted.
#
# view-camera is the cheapest recipe that carries *both* shapes — it spans both
# machines and runs its viewer in the foreground — which is why it is the one
# that carries the extra endings below. It also has the most to lose: the thing
# it leaves behind on the Pi holds /dev/video0 *exclusively*. A leaked
# camera_node does not merely linger; it makes every later session in this
# project die with "Device or resource busy", including the ones that would
# have diagnosed it.

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

# view-keypoints is the fifth, and it is the most expensive thing in this table to
# leak: a container holding decode_node and keypoint_node, the Pi's camera, three
# static transform publishers and RViz. It is given the fixture bag rather than the
# live camera here — the session shape being tested is the same either way, and
# the bag version does not depend on the Pi's Wi-Fi being up for the gate to mean
# something. The camera path is covered by view-camera in the row above.

# recipe -> the argv to run it with, and what "it is up" means for it.
RECIPES=(view-camera replay view-keypoints view-depth view-mesh view-odom dashboard)

# `replay` is the only recipe here that takes an argument, and the bag it takes
# has to be *this gate's own*. bags/ is git-ignored, so on a fresh clone there
# is nothing in it; picking whatever happened to be there would make a core
# teardown gate fail for reasons that have nothing to do with teardown, and
# would make what it asserts depend on which machine it ran on.
#
# So the fixture is synthesized: a throwaway topic recorded into a temp directory
# and deleted on the way out. `replay` never reads the messages — it hands the bag
# to `ros2 bag play` and puts RViz in front of it — so the payload is irrelevant
# and a std_msgs/String is the cheapest thing that makes a valid bag. What is being
# tested is the process tree: a player, a launcher, three static transform
# publishers and a viewer, all of which must be gone by the time the recipe has
# returned.
#
# **Its length is load-bearing, and it has had to grow three times.** Three seconds was
# too short in 2026-09-13 (below); twenty was too short for `view-depth`, added
# 2026-09-15. That recipe waits six seconds before starting RViz rather than
# three, because depth_node loads a 99 MB model and warms a CUDA session before
# the container is of any use — so `session_up` cannot be satisfied until the
# player, the container and a fully started rviz2 overlap, and the player is the
# one with a deadline. `view-mesh`, added 2026-09-16, waits eight. The fixture has
# to outlive all of that with room to spare.
#
# `view-keypoints` plays a bag **once** rather than on a loop (a looping bag
# replays header stamps minutes into the past, and keypoint_node's pose is then
# rejected by every TF listener in the domain — tools/replay.sh's header has the
# measurement). `session_up` for that recipe requires the player, the container
# *and* the viewer to be running at the same moment, and rviz2 takes several
# seconds to exist — so a three-second clip was finished before there was
# anything to see, and the gate failed with "view-keypoints never came up"
# against a recipe that was behaving correctly. The fixture has to outlive RViz's
# startup, which is what this is: long enough to overlap by a comfortable margin,
# short enough that building it is not the slowest thing here.
GATE_BAG_SECONDS=40
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
    sleep "$GATE_BAG_SECONDS"
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
        view-keypoints) printf '%s\n' "$PIMESH_WS/tools/view-keypoints.sh" "$2" "$GATE_BAG" ;;
        view-depth)  printf '%s\n' "$PIMESH_WS/tools/view-depth.sh" "$2" "$GATE_BAG" ;;
        view-mesh)   printf '%s\n' "$PIMESH_WS/tools/view-mesh.sh" "$2" "$GATE_BAG" ;;
        view-odom)   printf '%s\n' "$PIMESH_WS/tools/view-odom.sh" "$2" "$GATE_BAG" ;;
        # A port, not a window. The third argument keeps it off 8080 so a gate run
        # cannot collide with a dashboard somebody has open.
        dashboard)   printf '%s\n' "$PIMESH_WS/tools/dashboard.sh" "$2" "$GATE_BAG" 18080 ;;
        # No fallback on purpose. A dispatch table that guesses an argv for a
        # recipe nobody taught it is how a row gets added to RECIPES above and
        # silently tested as something else.
        *)           echo "argv_for: no argv for recipe '$1'" >&2; return 1 ;;
    esac
}

session_up() {              # $1 = recipe
    case $1 in
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
        # The container as well as the player and the viewer: this recipe is the
        # only one in the table that starts all three, and the container is the
        # one that would go on decoding frames unattended.
        view-keypoints) pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                        pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 &&
                        pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 ;;
        # The same three, and the slowest of the set to reach all of them: the
        # container is not merely started here, it has a model to load and a CUDA
        # context to build before depth_node returns from its constructor.
        view-depth) pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 ;;
        # The same three again, and slower still: this container loads the model,
        # builds the CUDA context *and* waits eight seconds before starting RViz,
        # because there is nothing for a mesh viewer to show until frames have been
        # integrated. The fixture clip has to outlive all of that.
        view-mesh)  pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 ;;
        # The same three once more. view-odom is view-mesh with an extra display
        # in the config; nothing about how it starts or ends differs.
        view-odom)  pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 ;;
        # **The one recipe here with no window at all**, which is why it is worth
        # having in this list rather than assuming it behaves like the viewers: it
        # starts a player, the container, *and* a node holding a listening socket
        # in its own process. A dashboard_node left behind would keep port 8080
        # bound and the next session would refuse to start — the same failure
        # shape as a camera_node holding /dev/video0, one machine over.
        dashboard)  pgrep -f "$PIMESH_BAG_PAT" >/dev/null 2>&1 &&
                    pgrep -f "$PIMESH_CONTAINER_PAT" >/dev/null 2>&1 &&
                    pgrep -f '/lib/[p]imesh_dashboard/' >/dev/null 2>&1 ;;
    esac
}

# How each recipe is ended, and this list is the hole the gate had until
# 2026-09-14.
#
# **Every case here was a signal to the process group, and that is not how a
# session usually ends.** A person closes the RViz window far more often than
# they type Ctrl-C, and it is a completely different code path: no signal reaches
# the script at all, `wait` returns because its child exited, and the EXIT trap
# runs from an ordinary end-of-script rather than from a handler. The gate
# asserted the signalled path ten times over and had never once closed a window.
#
# It leaked, of course. On 2026-09-14 `just view-keypoints` was closed at the
# window and left a `camera_node` on the Pi holding /dev/video0, with
# `tools/stragglers.sh` reporting a clean dev box and the next session refusing
# to start. This gate was green throughout. That is the third time this file has
# been the thing that was wrong rather than the code under it, and the question
# it exists to ask has not changed: what does the gate *not* touch?
#
# CLOSE-EARLY is the second half of the same lesson. CLOSE waits for the session
# to be fully up before closing the window, which quietly excludes the case that
# actually failed — a teardown that runs while the far end is still *starting*.
# `pi_run_for` puts three processes between ssh and the node, so a pattern kill
# aimed at the node can land before the node exists, match nothing, report
# success, and leave the wrapper to exec it a moment later. Measured 2026-09-14:
# kill_pi called half a second after a remote start returned 0, and twelve
# seconds later the Pi had the whole chain running. Only view-camera carries this
# case, because it is the cheapest recipe that spans both machines and has a
# window to close.
#
# INT-TWICE is the fourth time round, and it is the one this file had no case for
# on 2026-09-18: `just view-mesh` ended with `^C^C` left a camera_node holding
# /dev/video0 on the Pi with a clean dev box beside it. `_pimesh_on_signal`
# restored the default disposition before cleaning up, so the second interrupt
# killed the script between kill_local and kill_pi — and the split is the tell,
# because kill_local always goes first. Ten INT cases over five recipes had never
# sent a *second* signal, and a verified teardown takes 7-9 s while printing
# nothing, which is long enough that pressing Ctrl-C again is the normal thing to
# do. Only view-camera carries this case, for the same reason it carries
# CLOSE-EARLY: it is the cheapest recipe that spans both machines, and a teardown
# with no far end to reach cannot exhibit the bug.
hows_for() {                # $1 = recipe
    case $1 in
        # No viewer window to close, so the CLOSE case does not apply:
        # `dashboard` ends by a signal or by its own timer. Listing CLOSE for it
        # would emulate closing an rviz2 that was never started, which is a case
        # that passes for the wrong reason.
        dashboard)   echo "INT HUP" ;;
        view-camera) echo "INT INT-TWICE HUP CLOSE CLOSE-EARLY" ;;
        *)           echo "INT HUP CLOSE" ;;
    esac
}

close_viewer() {            # emulate a closed window: end rviz2 and nothing else
    local pid
    pid=$(pgrep -f "$PIMESH_VIEWER_PAT" | head -1)
    [[ -n ${pid:-} ]] || return 1
    # SIGTERM to the viewer alone. A window manager's close button is a
    # WM_DELETE_WINDOW message rather than a signal, but both end with rviz2
    # returning of its own accord while the script that started it is untouched —
    # which is the property under test. What must *not* happen here is a signal
    # reaching the script or its group: that would be the Ctrl-C case again,
    # wearing a different name.
    kill -TERM "$pid" 2>/dev/null || return 1
    return 0
}

run_and_end() {             # $1 = INT | HUP | CLOSE | CLOSE-EARLY, $2 = recipe
    # 90 s: longer than this gate takes, so the session never ends on its own
    # timer. A recipe that outlives the way it was ended has to be ended *by* it
    # for the straggler count below to mean anything. It was 45 s while teardown
    # was a fixed sleep; verified teardown can now spend up to
    # PIMESH_TEARDOWN_SECONDS insisting, and a recipe whose own timer fired
    # mid-teardown would be measuring the timer.
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
    local -a argv; mapfile -t argv < <(argv_for "$2" 90)
    setsid env --default-signal=INT,TERM,HUP \
        bash -c 'echo $$ >"$1"; shift; exec bash "$@"' _ "$pgidfile" "${argv[@]}" \
        >"$log" 2>&1 &
    local sess=$! pgid up=0
    for _ in $(seq 50); do
        pgid=$(cat "$pgidfile" 2>/dev/null || true)
        [[ -n ${pgid:-} ]] && break
        sleep 0.1
    done
    if [[ -z ${pgid:-} ]]; then
        echo "FAIL: the $2 session never reported a process group to end"
        return 1
    fi

    if [[ $1 == CLOSE-EARLY ]]; then
        # Deliberately *not* session_up: this case exists to end the session
        # while the other half of it is still coming up.
        for _ in $(seq 60); do
            pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 && { up=1; break; }
            sleep 0.5
        done
        if [[ $up -ne 1 ]]; then
            echo "FAIL: $2 never opened a window, so there was none to close"
            tail -20 "$log"; return 1
        fi
    else
        for _ in $(seq 45); do
            if session_up "$2"; then up=1; break; fi
            sleep 1
        done
        if [[ $up -ne 1 ]]; then
            echo "FAIL: $2 never came up, so there was nothing to kill"
            tail -20 "$log"; return 1
        fi
    fi

    case $1 in
        CLOSE|CLOSE-EARLY)
            if ! close_viewer; then
                echo "FAIL: could not close the window of $2"
                return 1
            fi
            ;;
        INT-TWICE)
            if ! kill -INT -"$pgid" 2>/dev/null; then
                echo "FAIL: could not send SIGINT to process group ${pgid} for $2"
                return 1
            fi
            # **The second one has to land inside the teardown**, or this case is
            # the INT case wearing a different name and asserting nothing. The
            # aim is taken off the clock and off the group still existing:
            # cleanup starts synchronously the moment the handler is entered, and
            # by then the session has no other work left, so a session still
            # alive a moment later is a session *inside* its teardown.
            #
            # **It deliberately does not wait for the teardown's announcement
            # line, and the first version of this case did.** That line arrived
            # with the fix, so keying the aim on it made the case unable to fail
            # against the very code it exists to catch: run against the pre-fix
            # handler on 2026-09-18 it reported "never announced a teardown" and
            # proved nothing, where aimed by the clock it leaves a camera_node on
            # the Pi. A test whose trigger ships with the fix is a test of the
            # fix's presence, not of the behaviour.
            sleep 0.5
            if ! kill -INT -"$pgid" 2>/dev/null; then
                echo "FAIL: $2 finished tearing down within 0.5 s, so the window this case"
                echo "      exists to test was never open — verified teardown takes 7-9 s"
                return 1
            fi

            # **This case cannot use the sweep the others use, and finding out why
            # was worth more than the case.** Every other ending is asserted by
            # _reap_session below — wait for the session's whole process *group* to
            # empty, then run tools/stragglers.sh — and that method is structurally
            # blind to this leak. The local `ssh` client sits in the group until its
            # remote command finishes, so when the script is killed mid-teardown the
            # group does not empty until the *recipe's own* `timeout` expires on the
            # Pi, 90 s in — which is the same event that kills the leaked camera_node.
            # By the time the group is empty the evidence has reaped itself, and the
            # sweep reports 0/0 truthfully. Measured 2026-09-18: this case passed
            # against the pre-fix handler that way, which is a false green.
            #
            # So it waits for the *script* rather than its group — the pgid leader is
            # the recipe, and its exit is exactly the contract's "the recipe has
            # returned" — and looks at the Pi at once. Gap and window, measured
            # against the pre-fix handler over five runs: a second SIGINT at 0.2, 1
            # and 2 s each left 3 processes on the Pi, at 4 and 6 s none, because
            # kill_pi has finished by then. 0.5 s has margin at both ends.
            for _ in $(seq 240); do
                kill -0 "$pgid" 2>/dev/null || break
                sleep 0.25
            done
            local pileft; pileft=$(pimesh_pi_processes)
            if [[ -n $pileft ]]; then
                echo "FAIL: $2 was killed by the second SIGINT mid-teardown and left the Pi"
                echo "      holding these — a camera_node there holds /dev/video0:"
                sed 's/^/  /' <<<"$pileft"
                kill_pi >/dev/null 2>&1 || true
                tail -20 "$log"; return 1
            fi
            ;;
        *)
            # A group kill that names no group is the failure this function was
            # once rewritten to prevent, so it is checked rather than swallowed.
            if ! kill -"$1" -"$pgid" 2>/dev/null; then
                echo "FAIL: could not send SIG$1 to process group ${pgid} for $2"
                return 1
            fi
            ;;
    esac

    # **Wait for the recipe to return, rather than sleeping and hoping.** This
    # was `sleep 3`, which was a guess that happened to be longer than a fixed
    # teardown and is shorter than a verified one. Waiting is also the stronger
    # assertion: the contract is that when the recipe has returned, nothing it
    # started is running — not that things tend to be gone three seconds later.
    if ! _reap_session "$sess" "$pgid" 90; then
        echo "FAIL: the $2 session did not return after $1 — still running:"
        pgrep -a -g "$pgid" | sed 's/^/  /'
        tail -20 "$log"; return 1
    fi

    # **Nine silent seconds is what makes a second Ctrl-C tempting**, so the line
    # that breaks the silence is part of the contract rather than a nicety, and it
    # is asserted separately from the behaviour above — after the fact, where it
    # cannot become the thing the case is aimed with.
    if [[ $1 == INT-TWICE ]] && ! grep -q 'further Ctrl-C ignored' "$log"; then
        echo "FAIL: $2 tore down without saying it was doing so. A teardown that"
        echo "      prints nothing for 7-9 s is the reason this case exists."
        tail -20 "$log"; return 1
    fi
    return 0
}

# Wait for the session's whole process *group* to go away, and pick up its exit
# status on the way past if it is safe to believe.
#
# The group rather than the pid, because the pid is not reliably the session.
# `setsid` forks when it is already a process group leader, and whether it is
# depends on the job control of the shell that started it — so `$!` is sometimes
# the session and sometimes a setsid that exited a millisecond later. Waiting on
# that pid would return immediately, and this gate would then sweep for
# stragglers while the recipe was still tearing down and report a leak that was
# simply a race with its own measurement. `$$` written by the session itself is
# the one identity that is always right, and an empty process group is the one
# statement of "it is finished" that does not depend on which process is which.
#
# The status is only believed when the pid we waited on *is* the group leader,
# which is exactly the case where setsid did not fork. Otherwise it is left
# empty and the caller skips the assertion rather than asserting on a number
# that belongs to a different process.
_reap_session() {           # $1 = pid started, $2 = pgid reported, $3 = seconds
    local pid=$1 pgid=$2 secs=$3 i
    SESSION_STATUS=
    for (( i = 0; i < secs * 10; i++ )); do
        if ! pgrep -g "$pgid" >/dev/null 2>&1; then
            local st=0
            wait "$pid" 2>/dev/null || st=$?
            [[ $pid == "$pgid" ]] && SESSION_STATUS=$st
            return 0
        fi
        sleep 0.1
    done
    return 1
}
SESSION_STATUS=

# Teardown is *verified*, not fired — asserted directly, because the recipe
# table above cannot reach this case and it was the actual bug.
#
# **This function exists because the gate was green over the broken code.**
# CLOSE-EARLY was written to cover the startup race and does not: it closes the
# window as soon as one exists, and rviz2 takes five to seven seconds to exist,
# by which time the Pi's camera_node is long up and a blind pattern kill finds
# it. Run against the one-line `kill_pi` this replaced, the whole recipe table
# printed 0/0 and PASS. That is the third false green this file has carried, and
# the lesson each time is the one it opens with: ask what the gate does not
# touch.
#
# So the race is asserted where it lives, on the function whose contract changed.
# `pi_run_for` puts a login shell, a `timeout` and a `ros2 run` between ssh and
# the node, so half a second after a remote start the leaf does not exist yet. A
# pattern kill aimed at the leaf matches nothing and — this is the part that made
# it a leak rather than a retry — *reports success*, leaving the wrapper to exec
# the node a moment later. Measured 2026-09-14 with the old body: returned 0, and
# twelve seconds later the Pi had `timeout`, `ros2 run` and a live `camera_node`
# holding /dev/video0.
#
# Two sweeps, and the second one is the assertion. An immediate sweep passes
# against the broken version too — at that instant the Pi really is between
# processes. What separates a Pi that is clean from a Pi that is *about to start
# a camera* is only visible a few seconds later, which is exactly why the leak
# outlived the session that caused it.
assert_teardown_verified() {
    echo "--- kill_pi, called while the far end is still starting ---"
    pi_run_for 60 "ros2 run pimesh_camera camera_node" >/dev/null 2>&1 &
    local sess=$! rc=0 left
    sleep 0.5
    kill_pi || rc=$?
    echo "kill_pi returned ${rc}"

    left=$(pimesh_pi_processes)
    if [[ -n $left ]]; then
        echo "FAIL: kill_pi returned ${rc} with the Pi still dirty:"
        sed 's/^/  /' <<<"$left"
        kill "$sess" 2>/dev/null || true
        return 1
    fi

    # The sweep that matters. Long enough for a wrapper that survived to have
    # exec'd `ros2 run`, and for that to have exec'd the node.
    sleep 10
    left=$(pimesh_pi_processes)
    if [[ -n $left ]]; then
        echo "FAIL: the Pi was clean when kill_pi returned ${rc} and dirty 10s later:"
        sed 's/^/  /' <<<"$left"
        echo "      a wrapper outlived the kill and started a node after it."
        kill "$sess" 2>/dev/null || true
        bash "$PIMESH_WS/tools/stragglers.sh" || true
        return 1
    fi
    kill "$sess" 2>/dev/null || true
    echo "pi clean on kill_pi's return and 10s later  (assert both)"
    echo
    return 0
}

# Built after the clean-slate check above, not before: the fixture starts two
# short-lived ros2 processes of its own, and a gate that cannot tell its own
# leak from somebody else's has to do its looking while nothing of its own is
# running.
assert_teardown_verified || exit 1

make_fixture_bag || exit 1
echo "fixture bag      : ${GATE_BAG} ($(du -sh "$GATE_BAG" | cut -f1), for replay)"

counts=""
cases=0
for what in "${RECIPES[@]}"; do
    for how in $(hows_for "$what"); do
        cases=$(( cases + 1 ))
        run_and_end "$how" "$what" || exit 1
        # tools/stragglers.sh is the assertion: it prints a count per host and
        # exits non-zero if any survived.
        out=$(bash "$stragglers" 2>&1) && rc=0 || rc=$?
        echo "--- ${what} after ${how} (session exited ${SESSION_STATUS:-?}) ---"
        echo "$out"
        if [[ $rc -ne 0 ]]; then
            echo "FAIL: ${how} left processes behind after ${what}"
            exit 1
        fi
        # **The second assertion, and it is independent of the first.** A
        # verified teardown exits non-zero when it could not get both machines
        # clean, so a closed window that returns 0 is the recipe's *own* evidence
        # that it checked — where the sweep above is this gate checking after the
        # fact. The two can disagree: a teardown that gave up at its deadline
        # while the last process was on its way out would sweep clean and still
        # report 1, and that is a failure worth seeing.
        #
        # Only for the close cases. A recipe ended by SIGINT re-raises it after
        # cleaning up, so the shell that started it sees 130 — which is correct,
        # and is asserted as such rather than being folded in here.
        case ${SESSION_STATUS:+$how} in
            CLOSE|CLOSE-EARLY)
                if [[ $SESSION_STATUS -ne 0 ]]; then
                    echo "FAIL: ${what} exited ${SESSION_STATUS} after ${how} —"
                    echo "      its own teardown could not confirm both machines were clean"
                    tail -20 "$log"
                    exit 1
                fi
                ;;
            INT|INT-TWICE)
                if [[ $SESSION_STATUS -ne 130 ]]; then
                    echo "FAIL: ${what} exited ${SESSION_STATUS} after SIGINT, expected 130"
                    echo "      a handler that cleans up and returns is not an interrupt;"
                    echo "      see _pimesh_on_signal in tools/just-lib.sh"
                    tail -20 "$log"
                    exit 1
                fi
                ;;
        esac
        counts+="${what}/${how}: $(grep -o '[0-9]*$' <<<"$out" | tr '\n' '/' | sed 's:/$::')  "
    done
done

echo
echo "survivors dev/pi : ${counts}"
echo "                   (assert 0/0 after every ending, ${cases} cases over"
echo "                    ${#RECIPES[@]} recipes: Ctrl-C, Ctrl-C twice with the second"
echo "                    inside the teardown, a closed terminal, a closed window,"
echo "                    and a window closed before the far end is up)"
echo "PASS gate-teardown"
