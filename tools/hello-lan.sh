#!/usr/bin/env bash
#
# Talker on the Pi, listener here. $1 = how long the talker runs, default 20 s.
# A thing to watch, not an assertion — gates/hello-lan.sh is the assertion.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

seconds=${1:-20}

# One trap covers the interrupt, the normal end and the error paths, and it
# reaches across the LAN: a talker left running on the Pi is invisible from here
# and poisons the next run.
arm_cleanup

marker="hello-from-$(pi_run hostname)"
echo "listener here (${ROS_DISTRO}), talker on ${PI}, payload ${marker}"

# Listener first, so it is subscribed before the first message exists.
timeout -s INT $(( seconds + 15 )) ros2 run pimesh_hello echo_node &
sleep 2
pi_ws_run "timeout -s INT $seconds ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=2.0 -p text:=$marker"
wait || true
