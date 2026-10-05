#!/usr/bin/env bash
#
# Run one session — a `ros2 launch` a person watches — with the two things
# `launch` cannot do for it:
#
#   bash tools/session.sh <label> --pi|--local -- <command...>
#
# 1. **Refuse to start beside another session.** `assert_no_session` looks at
#    both machines. Two sessions on one ROS domain put two publishers on
#    /image_raw/compressed and make *both* look broken (CLAUDE.md, "One session
#    at a time").
# 2. **Tear down the Pi, and check.** `launch` owns its local children and shuts
#    them down itself, escalating SIGINT -> SIGTERM -> SIGKILL. It has no notion
#    of a second machine: the camera is an ssh away, and killing the local ssh
#    client does not reach the far end (measured 2026-09-14). `--pi` arms
#    `cleanup_both`, whose `kill_pi` keeps asking until the Pi is clean or says
#    it could not get it clean. `--local` arms `kill_local` alone, for a session
#    that never opened a connection — an ssh round trip on the way out of a bag
#    replay is latency for nothing, and an unreachable Pi would fail it.
#
# `kill_local` stays in both on purpose. `launch` is the mechanism now and the
# sweep is the backstop: a launch that was itself SIGKILLed cannot shut anything
# down, and the sweep is what says so.
#
# This is #15's P0. It replaced the top and bottom thirds of eight viewer
# scripts; the middle third became `src/pimesh_bringup/launch/view.launch.py`.

source "$(dirname "${BASH_SOURCE[0]}")/lib/just-lib.sh" --overlay

usage() { echo "usage: session.sh <label> --pi|--local -- <command...>" >&2; exit 2; }

label=${1:-}; shift || usage
case ${1:-} in
    --pi)    cleanup=cleanup_both ;;
    --local) cleanup=kill_local ;;
    *)       usage ;;
esac
shift
[[ ${1:-} == -- ]] || usage
shift
(( $# > 0 )) || usage

# An empty `name:=` is dropped so the launch file's default applies: `ros2
# launch` refuses one as malformed. See pimesh_launch_argv in just-lib.sh.
mapfile -t cmd < <(pimesh_launch_argv "$@")

# view.launch.py finds bags/, models/ and tools/ through this.
export PIMESH_WS

# Before arm_cleanup, always: the EXIT handler kills this workspace's processes,
# so a refusal after the trap is armed would tear down the session it refused
# to disturb.
assert_no_session "just ${label}"
arm_cleanup "$cleanup"

# Backgrounded and waited on, not run in the foreground: bash defers a trap until
# its foreground child returns, so a Ctrl-C would otherwise wait on launch's own
# shutdown before the Pi was even asked. With `wait`, the trap runs on arrival.
#
# **`env --default-signal`, because a background command inherits SIGINT
# ignored.** A non-interactive shell starts `&` jobs with SIGINT and SIGQUIT set
# to SIG_IGN, and an ignored signal survives exec: `ros2 launch` passed it to
# every child. C++ nodes reinstall a handler (rclcpp does) and never noticed;
# the Python CLIs launch runs do not. Measured 2026-10-05: `ros2 bag record`
# started with SIGINT ignored never stopped on its `timeout -s INT` and was
# SIGKILLed with no metadata.yaml, where the same command with SIGINT at its
# default stopped in 5 s — and `ros2 bag play` sat out launch's SIGINT for the
# full 5 s in #15's P0 for the same reason. gates/teardown.sh carries the same
# trap from the gate's side.
env --default-signal=INT,TERM,HUP "${cmd[@]}" &
rc=0
wait $! || rc=$?
exit "$rc"
