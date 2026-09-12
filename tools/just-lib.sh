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

# shellcheck disable=SC2034  # read by tools/stragglers.sh
PIMESH_PATTERNS=(
    "$PIMESH_NODE_PAT" "$PIMESH_CONTAINER_PAT" "$PIMESH_LAUNCH_PAT"
    "$PIMESH_VIEWER_PAT" "$PIMESH_TF_PAT" "$PIMESH_BAG_PAT"
    "$PIMESH_REPUBLISH_PAT"
)

# Container before launcher, always: killing the launcher first orphans the
# container, which then holds the topics nobody can find a publisher for.
kill_local() {
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
    sleep 0.5
    pkill -f "$PIMESH_LAUNCH_PAT" 2>/dev/null || true
}

kill_pi() {
    pi_run "pkill -f '$PIMESH_NODE_PAT' || true" >/dev/null 2>&1 || true
}

cleanup_both() { kill_local; kill_pi; }

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
    trap '_pimesh_cleanup_once' EXIT
    trap '_pimesh_on_signal INT'  INT
    trap '_pimesh_on_signal TERM' TERM
    trap '_pimesh_on_signal HUP'  HUP
}

_pimesh_cleanup_once() {
    [[ -n ${PIMESH_CLEANED:-} ]] && return 0
    PIMESH_CLEANED=1
    "${PIMESH_CLEANUP_FN:-cleanup_both}"
}

_pimesh_on_signal() {   # $1 = signal name
    trap - EXIT INT TERM HUP
    _pimesh_cleanup_once
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
