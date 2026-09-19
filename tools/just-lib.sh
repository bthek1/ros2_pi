# shellcheck shell=bash
#
# The shell that every `just` recipe in this repo shares. Sourced, never run:
#
#   source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"            # prelude only
#   source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --ros      # ...plus /opt/ros
#   source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay  # ...plus install/
#
# It exists because a `justfile` recipe body is a script with no way to share
# code with another recipe — `just` has no file-scope shell functions. Before
# this file the alternative was copy-paste, and the copies had drifted: the SSH
# invocation was written out fourteen times and the process patterns in three
# spellings. Both are things that must never be written differently twice.
#
# It is rsynced to the Pi with the rest of tools/, so the same functions are
# available at both ends of the LAN — which is the second reason it is a file
# and not a longer justfile.

set -euo pipefail

# The workspace root, from this file's own location rather than from the
# caller's cwd: gate scripts get run from `just`, from a terminal and from
# inside a `setsid` process group, and only one of those starts anywhere known.
PIMESH_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PIMESH_WS"

# ament_cmake is not a pure-CMake buildtool: it shells out to Python at
# *configure* time (package_xml_2_cmake.py and friends), so a C++ package still
# needs an interpreter that can import catkin_pkg. CMake's FindPython3 picks the
# highest version it can see, and this machine has two 3.14s on PATH — the apt
# one in /usr/bin that ROS's dist-packages belong to, and a uv-managed one in
# ~/.local/bin that has never heard of catkin_pkg. Naming the interpreter is the
# only way to stop that coin-flip; the alternative (reordering PATH) leaves the
# choice to whichever Python sorts highest. Measured 2026-09-08: without this,
# every ament_cmake package fails at ament_package() with ModuleNotFoundError.
# /usr/bin/python3 is the right answer on both machines — both take ROS from apt.
#
# **CMAKE_BUILD_TYPE, and it was missing until 2026-09-12.** colcon sets no build
# type of its own, and an ament_cmake package that does not set one either compiles
# with *no optimisation flags at all* — not `-O0` explicitly, simply nothing. Every
# C++ cost this project had measured was therefore an unoptimised number, which was
# found by P3's per-frame budget: 7.90 ms unoptimised against an 8 ms ceiling, and
# 6.99 ms with this flag. A 1% margin that is really 13% is the kind of wrong number
# that gets quoted and then acted on.
#
# RelWithDebInfo rather than Release: `-O2 -g` against `-O3 -DNDEBUG`, which measured
# 6.99 ms against 6.87 ms — 2% apart — for the difference between a node you can put
# a debugger on and one you cannot. In a project whose point is understanding what
# the code does, that is not a close call.
#
# An array, not a string: two flags in one quoted word arrive at CMake as a single
# argument it cannot parse. The remote scripts join it with ${...[*]} on purpose,
# because there the words are re-split by the far shell.
# shellcheck disable=SC2034  # read by the scripts that source this file
PIMESH_CMAKE_ARGS=(
    -DPython3_EXECUTABLE=/usr/bin/python3
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
)

# --- The Pi -----------------------------------------------------------------

PI=${PIMESH_PI:-pi}
PI_WS=${PIMESH_PI_WS:-'~/ros2_pi'}

# BatchMode and ConnectTimeout are not tuning. The Pi's Wi-Fi link dies while
# the Pi keeps running, and a bare ssh hangs about two minutes against a dead
# link — long enough to wedge whatever trap it is sitting in. An array, not a
# string, so the words survive quoting without splitting on $IFS.
PI_SSH=(-o BatchMode=yes -o ConnectTimeout=5)

# Run a command on the Pi in a *login* shell. Equally non-negotiable:
# ROS_DOMAIN_ID and RMW_IMPLEMENTATION live in the Pi's ~/.profile, so a
# non-login shell runs on domain 0 with the wrong middleware and produces a
# result that means nothing.
#
# printf %q does the quoting. The command crosses two shells — ssh pastes its
# arguments together for the remote login shell, which then parses `bash -lc
# <this>` — so anything containing a quote or a bracketed pgrep pattern has to
# arrive intact rather than half-expanded at whichever end.
pi_run() {              # $* = command line to run on the Pi
    ssh "${PI_SSH[@]}" "$PI" "bash -lc $(printf '%q' "$*")"
}

# ...in the Pi's workspace, with its own ROS and this workspace's install/ on
# the environment. The Pi is Jazzy and this box is Lyrical, so the distro is
# discovered at the far end by tools/ros-env.sh and never named here.
pi_ws_run() {           # $* = command line to run in the Pi's workspace
    pi_run "cd $PI_WS && source tools/ros-env.sh --overlay && $*"
}

# --- Killing things ---------------------------------------------------------
#
# Bracketed, and path-qualified, and both for the same reason. `pgrep -f
# component_container` matches the shell command line that contains those
# characters — so the plain spelling reports itself as a straggler and, with
# pkill, kills the session asking. Measured, the hard way. The bracket only
# protects the pattern's own text, though: a command mentioning the bare word
# somewhere else still matches, so these anchor on installed binary paths,
# which prose does not contain.
PIMESH_NODE_PAT='/lib/[p]imesh_[a-z]*/'
PIMESH_CONTAINER_PAT='rclcpp_components/[c]omponent_container'
PIMESH_LAUNCH_PAT='ros2 [l]aunch pimesh_[a-z]*'
# The viewer counts as a straggler. It holds no device and leaks nothing
# expensive, which is exactly why it would have been left out — and then a
# `just view-camera` whose RViz outlived its Ctrl-C would have gone unnoticed
# by a gate whose whole subject is sessions ending. Scoped to *our* config, so
# somebody else's rviz2 on this machine is none of our business.
#
# **Anchored at the start of the command line**, and the `^` is the whole
# lesson. Every other pattern here is path-qualified — `/lib/[p]imesh_camera/`
# is what the *process* shows, while a shell launching it says
# `ros2 run pimesh_camera camera_node`, so the two cannot be confused. rviz2
# has no such distinction: the viewer's command line and the command line of
# whatever started it contain the same `rviz2 -d …/pimesh_bringup/…` text. The
# unanchored version therefore matched the shell that ran it and `kill_local`
# killed its own caller — measured 2026-09-09, three ad-hoc diagnostic runs
# dying at exit 144 before the output could be read. This is the same trap the
# brackets exist for, arriving by a different door: the bracket only protects
# the pattern's own text. The real process has `rviz2` at argv[0]; a launcher
# has `bash`, `timeout` or `just` there instead.
PIMESH_VIEWER_PAT='^[r]viz2 -d .*pimesh_'

# The static transform publishers pimesh_bringup's launch starts. They are
# ours, they are not ours to *name* — the binary lives in tf2_ros — and leaving
# them out of this list cost an hour of confusion on 2026-09-09.
#
# Nine of them, from three separate sessions, were found still running while
# `tools/stragglers.sh` reported 0 on both machines and `gates/hello-clean.sh`
# had been printing `view-camera/SIGINT: 0/0`. Every pattern above matches a
# path containing `pimesh_`, and these run out of `/opt/ros/*/lib/tf2_ros/`, so
# nothing looked at them. The symptom was not subtle once seen — `ros2 node
# list` warning about nodes sharing an exact name, three `/map_to_odom`s, and
# `hello-lan` failing with `publisher count was 0` off the back of a graph that
# had nine stale participants in it.
#
# **Duplicate publishers on one TF edge is the specific failure this project
# says makes a mesh smear and shows up in no single log.** So this pattern is
# deliberately broader than the rest: it matches any tf2_ros static publisher on
# the machine, not only ones whose node names we recognise. Naming ours would
# mean keeping a list here in step with STATIC_TRANSFORMS in the launch file,
# and a drifted list would silently stop matching — which is how this got missed
# the first time. A person running one by hand for unrelated work will lose it
# to a gate; that is cheap, and a corrupted frame tree is not.
PIMESH_TF_PAT='tf2_ros/[s]tatic_transform_publisher'

# The bag player, and it is here because the straggler sweep was blind to it.
# On 2026-09-12 two `ros2 bag play bags/desk1 --loop` processes sat on this
# machine for four minutes, publishing on /image_raw/compressed — the one topic
# the whole pipeline reads — while `tools/stragglers.sh` reported 0 on both
# machines and meant it, because every pattern above matches either a
# `pimesh_`-qualified path or tf2_ros, and a bag player is neither. A stale
# publisher on that topic is worse than a stale viewer: it is a *second source*
# of the pipeline's input, so a measurement taken beside one is measuring a
# mixture and nothing in the output says so.
#
# **Path-anchored, not `^`-anchored**, and the two are not interchangeable here.
# The rviz2 pattern above can use `^` because the real process has `rviz2` at
# argv[0]; this one cannot, because `ros2` is a Python script and the real
# process has `/usr/bin/python3` there with `/opt/ros/<distro>/bin/ros2 bag
# play` behind it. The `/bin/` is what separates the player from a shell whose
# command line merely contains the words `ros2 bag play` — including, every
# time, the script that started it.
#
# The breadth is deliberate, on the same reasoning as the TF pattern: this
# matches any bag player on the machine, not only one `just replay` started.
# Somebody replaying a bag by hand will lose it to a gate, which is cheap; a
# gate that reports a clean machine while a bag drives the pipeline's input
# topic is a green light over a wrong measurement, which is not.
PIMESH_BAG_PAT='/bin/[r]os2 bag play'

# image_transport's republisher, and it is here for the same reason as the bag
# player: it is a process that takes part in this pipeline's topics and no pattern
# above describes it.
#
# Found on 2026-09-12 while smoke-testing decode_node. **Three**
# `ros2 run image_transport republish compressed raw` processes had been running
# for three and a quarter hours — 14:26, 14:28 and 14:29 that afternoon, from
# trying the off-the-shelf decoder by hand — with `tools/stragglers.sh` reporting
# 0 on both machines throughout. Each one is a subscriber on
# `/image_raw/compressed`, which is the one topic that crosses Wi-Fi and the one
# topic this architecture allows exactly one reader on, so a leaked republisher is
# the *specific* failure P2 exists to assert against. It would have made
# `gates/ipc.sh`'s subscriber count wrong, and at worst it is a second RELIABLE
# reader collapsing the link.
#
# What made it survive three gates is worth stating: `republish`'s subscription is
# **lazy** — it connects only when something subscribes to its output — so
# `ros2 topic info -v /image_raw/compressed` reported `Subscription count: 0` with
# all three alive. A dormant reader that wakes up the moment a viewer appears is
# exactly the kind of straggler that is invisible until it is expensive.
#
# **Path-anchored, and the first spelling of this was not.** Written as
# `image_transport[ /][r]epublish` it also matched the `ros2 run image_transport
# republish` wrapper — and matched this very comment, so the first run of
# `tools/stragglers.sh` after adding it reported the shell that was running the
# sweep. That is the same trap the brackets exist for, arriving the same way it
# did for the viewer pattern: a bracket protects the pattern's own text and
# nothing else, and a pattern loose enough to match prose will eventually be put
# in a command line beside some. The wrapper needs no pattern of its own —
# `ros2 run` waits on its child and exits when the child is killed.
PIMESH_REPUBLISH_PAT='/image_transport/[r]epublish'

# The two wrappers a node on the Pi is started *inside*, and they are the reason
# a camera_node outlived `just view-keypoints` on 2026-09-14 while
# tools/stragglers.sh reported a clean Pi.
#
# `pi_run_for` starts `timeout -s INT <n> ros2 run pimesh_camera camera_node`
# inside a login shell, so the far end is three processes deep before the node
# exists: the login `bash -lc`, then `timeout`, then `/usr/bin/python3
# .../bin/ros2 run`, and only then the installed binary that PIMESH_NODE_PAT
# matches. **Every pattern above matches the leaf and none of them matches the
# stem**, and the stem is the half that matters, for two separate reasons.
#
#  1. **The sweep under-reports.** Measured 2026-09-14, one second after a
#     remote start: `tools/stragglers.sh` printed `stragglers on pi: 1` while
#     `pgrep` at the far end listed three of ours. During the startup window it
#     prints 0 — a clean Pi, over a chain that is about to open /dev/video0.
#  2. **Teardown loses a race it cannot see it lost.** `kill_pi` used to send one
#     SIGTERM to the leaf pattern and return 0 whether or not anything matched.
#     Fire it before the leaf exists and it matches nothing, reports success, and
#     the *wrapper it did not match* goes on to exec the node a moment later.
#     Measured 2026-09-14: `kill_pi` half a second after a remote start returned
#     0, and twelve seconds later the Pi had a full `camera_node` running with
#     its whole wrapper chain — which is the exact state the user's leaked
#     session was found in.
#
# So these two are in the kill list as much as in the sweep. Between them they
# cover the chain with no gap: before `timeout` has forked, the login shell's own
# command line still carries the whole `timeout -s INT <n> ros2 run pimesh_…`
# text and matches PIMESH_PI_WRAP_PAT; between fork and exec the child is still
# `timeout` and matches it too; after exec the python matches PIMESH_RUN_PAT; and
# after *that* the node matches PIMESH_NODE_PAT. Kill any live link and nothing
# downstream of it is ever created, which is what makes "the far end is clean"
# a terminal answer rather than a snapshot.
#
# Path-anchored on `/ros2`, per the rule the republish pattern was rewritten
# under: the process is `/opt/ros/jazzy/bin/ros2 run pimesh_camera camera_node`,
# and the leading slash is what separates it from a shell — or a comment — that
# merely contains the words `ros2 run pimesh_camera`.
PIMESH_RUN_PAT='/[r]os2 run pimesh_[a-z]*'

# `timeout` has no path to anchor on: pi_run_for writes it as a bare word so the
# Pi's own PATH resolves it. The bracket is doing all the work here, and it is
# enough — a script or a `pgrep` command line quoting this pattern contains
# `[t]imeout`, which does not match `timeout`, and that is precisely what the
# bracket is for.
#
# It matches the **local** ssh client too, whose command line carries the same
# remote text, and that is deliberate rather than a side effect: the client is
# what stays behind on this machine when a session's far end is orphaned. On
# 2026-09-14 one sat reparented to systemd --user with the dead session's ssh
# under it, invisible to every pattern in this file. kill_local now ends it, and
# because kill_local runs *before* kill_pi the connection is gone before the
# verified remote sweep opens a fresh one of its own.
PIMESH_PI_WRAP_PAT='[t]imeout -s INT [0-9]* ros2 run pimesh_[a-z]*'

# shellcheck disable=SC2034  # read by tools/stragglers.sh
PIMESH_PATTERNS=(
    "$PIMESH_NODE_PAT" "$PIMESH_CONTAINER_PAT" "$PIMESH_LAUNCH_PAT"
    "$PIMESH_VIEWER_PAT" "$PIMESH_TF_PAT" "$PIMESH_BAG_PAT"
    "$PIMESH_REPUBLISH_PAT" "$PIMESH_RUN_PAT" "$PIMESH_PI_WRAP_PAT"
)

# --- One session at a time --------------------------------------------------

# Every process on this machine matching the patterns above, except the caller's
# own process group.
#
# `pgrep -f` reads command lines, and the command line asking the question is one
# of them — a terminal command that merely *mentions* a pattern makes the caller
# report itself. Everything in the caller's process group is the caller or its
# children; a process belonging to another session is by definition in another
# group. tools/stragglers.sh reads this for the dev host, and so does
# assert_no_session below, because the question "what of ours is running" must
# not have two spellings.
pimesh_local_processes() {
    local mypgid pat hits pid rest pgid
    mypgid=$(ps -o pgid= -p $$ | tr -d ' ')
    for pat in "${PIMESH_PATTERNS[@]}"; do
        hits=$(pgrep -af "$pat" 2>/dev/null || true)
        [[ -n $hits ]] || continue
        while read -r pid rest; do
            [[ -n ${pid:-} ]] || continue
            pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
            [[ $pgid == "$mypgid" ]] || echo "$pid $rest"
        done <<<"$hits"
    done
}

# The same question at the far end, and tools/stragglers.sh reads this too.
#
# Failure-tolerant on purpose, for the guard's sake: the Pi's Wi-Fi link dies
# while the Pi keeps running, and turning an unreachable Pi into a refusal would
# stop `just replay` — which does not touch the Pi at all — from opening a bag.
#
# **The cost of that is a known soft spot in the sweep, and it is worth stating
# rather than discovering.** An unreachable Pi is indistinguishable here from a
# clean one, so `tools/stragglers.sh` reports 0 for a Pi it could not ask. That
# was already true before this function existed — its Pi loop swallowed the same
# error — so nothing changed but the number of places it is written down. The
# fix, if it is ever wanted, is to separate "asked and found nothing" from "could
# not ask" and make only the first one a pass; it is not free, because a Wi-Fi
# blip would then fail every gate that ends in a straggler sweep.
#
# **One connection, not one per pattern.** This used to loop `pi_run` over the
# pattern list, which is nine ssh handshakes over Wi-Fi for one question — about
# six seconds, paid by `assert_no_session` at the start of every recipe and
# again by every turn of kill_pi's verify loop. Worse than the latency, nine
# round trips are nine chances for the link to blink mid-answer, in a function
# whose failure mode is reporting a dirty Pi as a clean one. The patterns are
# joined into a single remote command instead.
pimesh_pi_processes() {
    local pat cmd=
    for pat in "${PIMESH_PATTERNS[@]}"; do
        cmd+="pgrep -af '$pat'; "
    done
    pi_run "{ $cmd }; true" 2>/dev/null || true
    return 0
}

# Refuse to start beside a session that is already running.
#
# **Two sessions on one ROS domain are not two independent sessions, and the way
# that fails is indistinguishable from a bug in whichever one you happen to be
# looking at.** Measured 2026-09-13: `just view-camera` and `just replay desk1`
# were up at the same time, so the Pi's live camera and a three-minute-old bag
# were both publishing `/image_raw/compressed`, one `keypoint_node` was decoding
# the interleaved mixture, and it published `odom -> base_link` with stamps that
# jumped minutes back and forth. Every TF listener in the domain — both RViz
# windows — then logged TF_OLD_DATA at the frame rate from inside tf2's own
# buffer mutex, which stalls the render loop. Both windows flickered; neither had
# anything wrong with it. Run alone, each was silent: 0 warnings over 195 s of
# replay and 75 s of view-camera.
#
# That is worth a hard refusal rather than a warning, because the symptom appears
# in the *innocent* session and points nowhere near the cause. The project
# already has one rule of this shape — one reader on the topic that crosses
# Wi-Fi — and this is the same rule from the publisher's side.
#
# **Every script that starts a session calls this, gates included**, and for the
# gates it is the sharper case: a viewer beside a gate makes the gate measure a
# mixture of two sources and print a number with nothing in it saying so. This
# file already carries two paragraphs about that exact failure — the leaked bag
# players, and the three `republish` processes — found both times *after* the
# measurements they had polluted. A gate that cannot tell whether it had the
# machine to itself is the green-over-broken shape this project keeps paying for.
#
# **Call this before arm_cleanup, always.** The EXIT handler runs kill_local,
# so a refusal after the trap is armed would tear down the very session it was
# refusing to disturb.
assert_no_session() {   # $1 = this recipe's name, for the message
    local running pi_running
    running=$(pimesh_local_processes)
    # The Pi as well as this box, and the Pi half is not optional: on 2026-09-13 a
    # `camera_node` outlived its session at the far end while this machine was
    # completely clean, and that one process is a *second publisher* on
    # /image_raw/compressed — the exact thing this guard exists to prevent, and
    # the one that also holds /dev/video0 so the next session cannot open it.
    # A local-only guard would have waved it through.
    pi_running=$(pimesh_pi_processes)
    [[ -n $running || -n $pi_running ]] || return 0
    {
        echo "${1:-this recipe} refuses to start: a session of this workspace is already running."
        echo
        # sed, not printf: printf '  %s\n' on a multi-line string indents only
        # the first line, which reads as one long wrapped entry rather than a list.
        [[ -n $running ]] && { echo "  on this machine:"; sed 's/^/    /' <<<"$running"; }
        [[ -n $pi_running ]] && { echo "  on ${PI}:"; sed 's/^/    /' <<<"$pi_running"; }
        echo
        echo "One ROS domain, one pipeline. A second session puts a second publisher on"
        echo "/image_raw/compressed — the Pi's live camera beside a bag's minutes-old"
        echo "frames — and keypoint_node decodes the mixture and publishes a pose whose"
        echo "stamps jump back and forth, which floods every TF listener and stutters"
        echo "every viewer. A gate run beside another session measures that mixture and"
        echo "prints a number with nothing in it saying so."
        echo
        echo "End the other session (Ctrl-C in its terminal), or sweep with"
        echo "  bash tools/stragglers.sh"
        echo "which prints the pid and full path of anything that outlived one, on both"
        echo "machines. A camera_node left at the far end is the expensive one: it holds"
        echo "/dev/video0 exclusively, so every later session dies on a busy device."
    } >&2
    exit 1
}

# How long teardown keeps *asking*, in seconds, before it gives up and says so.
#
# The number that matters is not how long a node takes to die — that is well
# under a second — but how long the slowest thing this workspace starts takes to
# come into existence. A `ros2 launch` that has not yet forked its container, or
# a remote `timeout` that has not yet exec'd `ros2 run`, cannot be killed by a
# pattern that describes the child. Teardown therefore kills the *parent* too
# (see PIMESH_RUN_PAT and PIMESH_PI_WRAP_PAT) and then keeps looking until two
# consecutive sweeps come back empty.
#
# 20 s is roughly four times the worst startup this project has measured, and it
# is only ever paid when something is genuinely refusing to die: the ordinary
# path is one kill and one confirming sweep.
PIMESH_TEARDOWN_SECONDS=${PIMESH_TEARDOWN_SECONDS:-20}

# What is running *here* that matches any of our patterns, without the
# process-group filter that pimesh_local_processes applies.
#
# The filter is right for the sweep and wrong for teardown, and the difference is
# the whole reason this is a second function rather than a reused one. A session's
# own launcher, container, viewer and static transform publishers are all in the
# *caller's* process group — that is what a session is — so
# pimesh_local_processes, asked from inside one, correctly reports none of them.
# A teardown loop built on it would see an empty machine on its first look and
# declare victory over a container that is still decoding frames.
#
# It is safe for the same reason `pkill -f` in kill_local is: every pattern is
# bracketed, so neither this shell's command line nor the ssh that carries a
# pattern to the Pi can match the thing it is asking about.
pimesh_live_locally() {
    local pat
    for pat in "${PIMESH_PATTERNS[@]}"; do
        pgrep -af "$pat" 2>/dev/null || true
    done
}

# Container before launcher, always: killing the launcher first orphans the
# container, which then holds the topics nobody can find a publisher for.
#
# **This used to fire once and return, and returning was the bug.** A pattern
# kill can only end a process that exists when it runs, and a session torn down
# early — a window closed while RViz was still painting, a Ctrl-C three seconds
# in — has processes that do not exist yet. They are created a moment later by
# parents the old pattern list did not describe, and nothing looked again.
kill_local() {
    local deadline=$(( SECONDS + PIMESH_TEARDOWN_SECONDS )) clean=0

    while :; do
        # SIGINT to the launcher *first*, and a moment to act on it. `ros2 launch`
        # shuts its own children down on an interrupt, which is the only way they
        # get a clean exit — TERM to the launcher tends to leave the tree behind,
        # and that is how nine static_transform_publishers accumulated. The pattern
        # kills below are the backstop for whatever that misses, not the mechanism.
        pkill -INT -f "$PIMESH_LAUNCH_PAT" 2>/dev/null || true
        pkill -INT -f "$PIMESH_VIEWER_PAT" 2>/dev/null || true
        # SIGINT to the player too, and for a reason the others do not have: a bag
        # player that has been stopped by SIGTTIN (see PIMESH_BAG_PAT) cannot run a
        # handler at all, so the plain pkill below is what actually ends that one.
        # Signalling first is still right for the ordinary case, where it closes the
        # storage cleanly instead of being cut off mid-read.
        pkill -INT -f "$PIMESH_BAG_PAT" 2>/dev/null || true
        sleep 1

        pkill -f "$PIMESH_VIEWER_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_REPUBLISH_PAT" 2>/dev/null || true
        pkill -CONT -f "$PIMESH_BAG_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_BAG_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_CONTAINER_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_NODE_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_TF_PAT" 2>/dev/null || true
        # The ssh client of this session's far end, and the local `ros2 run`
        # wrapper if a gate started one. Killing the client does *not* reach the
        # Pi — measured 2026-09-14, the remote timeout/ros2 run/camera_node chain
        # carried on after its client was killed — so this is housekeeping, not
        # teardown. kill_pi is what ends the far side, and it runs after this and
        # opens a connection of its own.
        pkill -f "$PIMESH_PI_WRAP_PAT" 2>/dev/null || true
        pkill -f "$PIMESH_RUN_PAT" 2>/dev/null || true
        sleep 0.5
        pkill -f "$PIMESH_LAUNCH_PAT" 2>/dev/null || true

        # Two empty sweeps, not one. A single empty look is exactly what the old
        # version trusted: it is also what you get in the half-second between a
        # launcher being killed and the container it already forked appearing.
        sleep 0.5
        if [[ -z $(pimesh_live_locally) ]]; then
            sleep 0.5
            [[ -z $(pimesh_live_locally) ]] && { clean=1; break; }
        fi
        (( SECONDS < deadline )) || break
    done

    if (( clean != 1 )); then
        # SIGKILL, once, and then say so either way. A process that has survived
        # a bounded loop of SIGINT and SIGTERM is not going to be talked round.
        local pat
        for pat in "${PIMESH_PATTERNS[@]}"; do
            pkill -KILL -f "$pat" 2>/dev/null || true
        done
        sleep 1
        local left; left=$(pimesh_live_locally)
        [[ -n $left ]] && {
            echo "teardown: ${PIMESH_TEARDOWN_SECONDS}s was not enough on this machine:" >&2
            sed 's/^/  /' <<<"$left" >&2
            return 1
        }
    fi
    return 0
}

# End the far side, and *check*, which is the entire point of this rewrite.
#
# **The old body was one line and it was a lie by omission:**
#
#     pi_run "pkill -f '$PIMESH_NODE_PAT' || true" >/dev/null 2>&1 || true
#
# One SIGTERM, to the leaf of a four-deep wrapper chain, over an ssh whose
# failure was discarded twice over — and it returned 0 in every case that
# matters: connection refused, Wi-Fi down, pattern matched nothing. Measured
# 2026-09-14: called half a second after `pi_run_for` started a camera_node, it
# returned 0, and twelve seconds later the Pi had `timeout`, `ros2 run` and a
# live `camera_node` holding /dev/video0. Nothing on either machine would ever
# have ended them; the session that started them had already exited, believing
# itself clean, and the next `just view-keypoints` refused to start.
#
# So it kills the whole chain rather than its leaf, and it keeps asking until the
# Pi answers with nothing — which is a terminal answer and not a snapshot,
# because with the wrappers dead there is nothing left that can create a node.
# When it cannot get that answer it prints what is still there, on stderr, with
# the pid and the full path. An unreachable Pi ends up in the same branch: the
# sweep cannot tell "asked and found nothing" from "could not ask", so a link
# that dies mid-teardown is reported as a failure to clean rather than as a
# success. That is the right way round — it is the reading that sends somebody to
# look.
kill_pi() {
    local deadline=$(( SECONDS + PIMESH_TEARDOWN_SECONDS )) left reached

    while :; do
        # Both wrappers and the node, in one remote shell: three round trips over
        # Wi-Fi to kill three processes is three chances for the link to blink.
        #
        # **The ssh's own status is the reachability probe**, and it is needed
        # because `pimesh_pi_processes` cannot tell "asked and found nothing"
        # from "could not ask" — see its note, and the reason it must stay that
        # way: turning an unreachable Pi into an error there would fail
        # `just replay`, which never touches the Pi at all. Teardown is the one
        # caller that needs the distinction, because an *unconfirmed* far end is
        # precisely the state this rewrite exists to stop reporting as clean. So
        # it comes free: ssh exits 255 when it cannot connect, and the remote
        # command is `…; true`, so any other status means the Pi answered.
        reached=1
        pi_run "pkill -INT -f '$PIMESH_PI_WRAP_PAT'; pkill -f '$PIMESH_RUN_PAT'; pkill -f '$PIMESH_NODE_PAT'; true" \
            >/dev/null 2>&1 || reached=0
        sleep 1
        if (( reached )); then
            # Two empty sweeps, not one. With the wrappers dead there is nothing
            # left that can create a node, so an empty answer is terminal rather
            # than a snapshot — but only once it is empty twice, because the
            # first look can land in the gap between a wrapper being killed and
            # the child it had already forked appearing.
            left=$(pimesh_pi_processes)
            [[ -z $left ]] && { sleep 1; left=$(pimesh_pi_processes); }
            [[ -z $left ]] && return 0
        fi
        (( SECONDS < deadline )) || break
    done

    if (( reached )); then
        pi_run "pkill -KILL -f '$PIMESH_PI_WRAP_PAT'; pkill -KILL -f '$PIMESH_RUN_PAT'; pkill -KILL -f '$PIMESH_NODE_PAT'; true" \
            >/dev/null 2>&1 || reached=0
        sleep 1
        left=$(pimesh_pi_processes)
        (( reached )) && [[ -z $left ]] && return 0
    fi

    {
        if (( reached )); then
            echo "teardown: ${PIMESH_TEARDOWN_SECONDS}s was not enough on ${PI}:"
            sed 's/^/  /' <<<"$left"
        else
            echo "teardown: ${PI} stopped answering, so what is running there is unknown."
            echo "The Pi's Wi-Fi link dies while the Pi keeps running — ping it, and read"
            echo "journalctl -b -1 after it comes back."
        fi
        echo "A camera_node left there holds /dev/video0 exclusively, so the next"
        echo "session will die on a busy device. Sweep with: bash tools/stragglers.sh"
    } >&2
    return 1
}

# Both ends, and the status says whether they are actually clean.
#
# Order is load-bearing: kill_local first, because it ends the ssh *client* of
# this session's far end, and kill_pi then opens a connection of its own to a Pi
# that nothing is still driving.
cleanup_both() {
    local rc=0
    kill_local || rc=1
    kill_pi || rc=1
    return $rc
}

# One trap for the four signals that end a session. Bash fires EXIT on Ctrl-C
# too, so EXIT alone would nearly do — the rest are there because a recipe that
# is killed by its parent's process group never runs EXIT's handler unless the
# signal is caught first.
#
# The handler must do two things beyond cleaning up, and the first version of
# this function did neither.
#
#  1. **Exit.** `trap handler INT` does not end a script: bash runs the handler
#     and then *resumes at the next line*. A viewer recipe gets away with that
#     because the next line is the end of the file, but a gate interrupted
#     halfway carries on measuring things it has just killed and reports
#     failures that are its own doing. Re-raising after restoring the default
#     disposition is the portable exit-as-if-by-signal: the caller's shell sees
#     130 for a Ctrl-C rather than a bare 0, which is what `set -e` upstream and
#     `$?` in a gate both expect.
#  2. **Run once.** With one handler on both INT and EXIT, a Ctrl-C ran the
#     cleanup twice — harmless for `pkill`, but it also paid `kill_local`'s
#     0.5 s settle twice and issued a second round of SSH to the Pi.
arm_cleanup() {         # $1 = optional cleanup function, default cleanup_both
    PIMESH_CLEANUP_FN=${1:-cleanup_both}
    PIMESH_CLEANED=
    trap '_pimesh_on_exit' EXIT
    trap '_pimesh_on_signal INT'  INT
    trap '_pimesh_on_signal TERM' TERM
    trap '_pimesh_on_signal HUP'  HUP
}

# **A second Ctrl-C must not be able to kill the teardown it is impatient with,
# and until 2026-09-18 it could.** `_pimesh_on_signal` restored the default
# disposition *before* cleaning up, so from that instant the script was killable
# by the next interrupt — and verified teardown takes 7-9 s while printing
# nothing, which is exactly long enough to look hung. Measured with a fixture of
# this handler's shape: one SIGINT ran kill_local and kill_pi to completion; two,
# three seconds apart, ran kill_local and died inside kill_pi. The real thing
# behind that number was `just view-mesh` closed with `^C^C`, which left the Pi
# holding /dev/video0 with a clean dev box beside it and no teardown message
# anywhere — the signature is always the same, because kill_local goes first.
#
# So the signals are *ignored* for the duration rather than defaulted, and the
# one thing that made a second Ctrl-C tempting is removed too: a line saying
# what is happening and how long it may take. Ignoring is safe only because the
# wait is bounded — each half gives up after PIMESH_TEARDOWN_SECONDS and says
# so — and an unbounded cleanup that cannot be interrupted would be the worse
# bug of the two.
_pimesh_cleanup_once() {
    [[ -n ${PIMESH_CLEANED:-} ]] && return "${PIMESH_CLEANUP_RC:-0}"
    PIMESH_CLEANED=1
    PIMESH_CLEANUP_RC=0
    trap '' INT TERM HUP
    echo "cleaning up and checking, up to ${PIMESH_TEARDOWN_SECONDS}s per machine — further Ctrl-C ignored" >&2
    "${PIMESH_CLEANUP_FN:-cleanup_both}" || PIMESH_CLEANUP_RC=$?
    return "$PIMESH_CLEANUP_RC"
}

# **A session that could not clean up exits non-zero, even when what it was
# doing succeeded.** That is the whole difference between this teardown and the
# one it replaces: the old one could not fail, so a leak left no trace anywhere
# except on the machine it leaked onto, where the next session found it hours
# later. A viewer is allowed to end at 0 for having shown somebody a picture;
# it is not allowed to end at 0 having left a camera_node holding /dev/video0.
#
# The script's own status wins when it is already non-zero — a gate that failed
# its assertion should report the assertion, not the tidying up.
_pimesh_on_exit() {
    local rc=$?
    _pimesh_cleanup_once || { (( rc == 0 )) && rc=1; }
    exit "$rc"
}

_pimesh_on_signal() {   # $1 = signal name
    # Ignore first, and only then drop EXIT: the window between entering this
    # handler and _pimesh_cleanup_once installing its own ignore is a window in
    # which a second Ctrl-C still has the default disposition. Bash does not
    # block a signal while its own handler for it runs.
    trap '' INT TERM HUP
    trap - EXIT
    _pimesh_cleanup_once
    # Defaults back, so the re-raise below actually ends this shell rather than
    # being discarded: exit-as-if-by-signal is what makes a Ctrl-C read as 130.
    trap - INT TERM HUP
    kill -"$1" $$
    # Only reached if the signal was inherited as SIG_IGN, which a shell cannot
    # undo — see the comment on `set -m` in gates/hello-clean.sh.
    exit $(( 128 + $(kill -l "$1") ))
}

# --- Bounding a thing you watch ---------------------------------------------

# Run a command under a time limit *without* going deaf to Ctrl-C.
#
# GNU timeout puts its child in a new process group, so that when the timer
# fires it can signal the whole tree rather than just the one pid it started.
# That is the right default for a batch job and precisely wrong for anything a
# person sits and watches, because a terminal delivers Ctrl-C to the
# *foreground* process group only. The command under a plain `timeout` is not in
# it, so it never sees the interrupt; and the shell that is waiting on `timeout`
# will not run a trap until its foreground child returns, so the trap does not
# fire either. The result is a session that ignores Ctrl-C completely and ends
# on its own when the timer runs out.
#
# Measured 2026-09-09, sending SIGINT to the process group at t=1 s the way a
# terminal does, against `timeout -s INT 8 sleep 8`:
#
#   plain `timeout ...`, foreground   trap never fired, script alive at t=3
#   `timeout --foreground ...`        trap fired immediately, group gone
#   `timeout ... &` then `wait`       trap fired immediately, group gone
#
# --foreground is the spelling for a foreground command: the child stays in the
# caller's process group, so Ctrl-C reaches it directly and ROS shuts down the
# way it does under a bare `ros2 launch`, with the trap left as the backstop for
# whatever the graceful path misses. A backgrounded `timeout ... &` followed by
# `wait` is equally fine and needs no flag — the trap runs the moment the signal
# lands — but it can only ever kill by pattern, so prefer this for the recipes
# whose output someone is reading.
#
# Giving up timeout's group-kill costs nothing here: every command run this way
# is a launcher that already shuts its own children down.
run_for() {             # $1 = seconds, rest = command line
    local secs=$1; shift
    timeout --foreground -s INT "$secs" "$@"
}

# Run a command on the Pi, in its workspace, under a time limit — and have the
# limit reach the *node* rather than the login shell wrapping it.
#
# This is the remote counterpart of run_for, and it is deliberately spelled
# differently, because the two ends want opposite things from `timeout`.
#
# Locally, run_for passes --foreground so that a person's Ctrl-C reaches a
# command they are watching. Nobody is typing at the far end of an ssh, and
# there --foreground is actively wrong: timeout then signals only its direct
# child. Written as
#
#     ssh pi 'timeout --foreground -s INT 12 bash -lc "... ros2 run ..."'
#
# the SIGINT lands on the login bash, which dies, and orphans `ros2 run` and the
# node under it — measured 2026-09-09, a camera_node still holding /dev/video0
# a minute after its 12 s limit, which is precisely the leak that makes every
# later session fail with "Device or resource busy". The fix is both halves:
# `timeout` goes *inside* the login shell, so that ros2 run and the node are its
# own children, and it keeps its default group-kill so the whole tree gets the
# signal.
#
# The pattern kill in kill_pi is still the backstop, not the mechanism. A node
# that is signalled shuts down cleanly and logs that it did; one that is pkill'd
# tells you nothing about whether it would have.
pi_run_for() {          # $1 = seconds, rest = command line to run in the Pi's workspace
    local secs=$1; shift
    pi_ws_run "timeout -s INT $secs $*"
}

# --- Assertions -------------------------------------------------------------

# awk rather than bash, because every rate this project asserts on is a float
# and [[ ]] cannot compare one.
in_range() {            # $1 = value, $2 = low, $3 = high
    awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN { exit !(v >= lo && v <= hi) }'
}

# --- ROS on the environment -------------------------------------------------

case "${1:-}" in
    --overlay) . "$PIMESH_WS/tools/ros-env.sh" --overlay ;;
    --ros)     . "$PIMESH_WS/tools/ros-env.sh" ;;
    "")        ;;
    *)         echo "just-lib: unknown option $1" >&2; exit 1 ;;
esac
