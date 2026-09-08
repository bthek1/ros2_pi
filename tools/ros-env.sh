# shellcheck shell=bash
#
# Put ROS on the environment of a `just` recipe. Sourced, never executed.
#
#   source tools/ros-env.sh              # the underlay only
#   source tools/ros-env.sh --overlay    # ...plus this workspace's install/
#
# Login shells on both machines already do the first half (~/.profile), so this
# is redundant — right up until the shell integration is the thing that is
# broken, which is exactly when a recipe has to keep working anyway.
#
# It is also what lets one justfile be correct on two distros. The dev box runs
# Lyrical and the Pi runs Jazzy, and this file is rsynced to the Pi (P3), so the
# distro is *discovered* here and never written down. Writing it down is how a
# recipe ends up silently building against the wrong ROS.
#
# The `set +u` dances are not optional: colcon's and ROS's setup scripts read
# unset variables by design, and a recipe running under `set -u` dies inside
# them with an error that names none of this.

_ros_env_fail() {
    echo "ros-env: $*" >&2
    exit 1
}

if [ -z "${ROS_DISTRO:-}" ]; then
    _ros_underlay=$(ls -1d /opt/ros/*/setup.bash 2>/dev/null | head -n1)
    [ -n "$_ros_underlay" ] ||
        _ros_env_fail "no ROS installation under /opt/ros — nothing to source"
    set +u
    # shellcheck disable=SC1090
    . "$_ros_underlay"
    set -u
    unset _ros_underlay
fi

if [ "${1:-}" = "--overlay" ]; then
    _ros_ws=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
    [ -r "$_ros_ws/install/setup.bash" ] ||
        _ros_env_fail "no install/setup.bash in $_ros_ws — build first"
    set +u
    # shellcheck disable=SC1090
    . "$_ros_ws/install/setup.bash"
    set -u
    unset _ros_ws
fi

unset -f _ros_env_fail
