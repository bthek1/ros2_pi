#!/usr/bin/env bash
#
# Record a clip from the Pi's camera into bags/<name>, counted from the first
# frame:
#
#   bash tools/record.sh <name> [seconds=60]
#
# It was `just record` until 2026-10-13, when the justfile was cut to `just run`.
# A script rather than a recipe because a clip is recorded rarely — desk1, walk1
# and scale1 are the references every gate replays — and not on a normal day.
# The steps are record.launch.py's (camera reset, first frame, recorder, the
# metadata check); this wraps it in tools/session.sh, whose kill_pi is what ends
# the camera on the Pi.

name=${1:?usage: bash tools/record.sh <name> [seconds=60]}
seconds=${2:-60}

exec bash "$(dirname "${BASH_SOURCE[0]}")/session.sh" record --pi -- \
    ros2 launch pimesh_bringup record.launch.py name:="$name" seconds:="$seconds"
