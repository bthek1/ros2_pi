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
# shellcheck disable=SC2034  # read by the scripts that source this file
PIMESH_CMAKE_ARGS="-DPython3_EXECUTABLE=/usr/bin/python3"

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
# shellcheck disable=SC2034  # read by tools/stragglers.sh
PIMESH_PATTERNS=("$PIMESH_NODE_PAT" "$PIMESH_CONTAINER_PAT" "$PIMESH_LAUNCH_PAT")

# Container before launcher, always: killing the launcher first orphans the
# container, which then holds the topics nobody can find a publisher for.
kill_local() {
    pkill -f "$PIMESH_CONTAINER_PAT" 2>/dev/null || true
    pkill -f "$PIMESH_NODE_PAT" 2>/dev/null || true
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
