#!/usr/bin/env bash
#
# Build the workspace on the Pi, from source, under Jazzy. Syncs first: a build
# of last week's source is worse than no build, because it looks like one.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

bash "$PIMESH_WS/tools/sync-pi.sh"
pi_run "cd $PI_WS && source tools/ros-env.sh && colcon build --symlink-install --cmake-args $PIMESH_CMAKE_ARGS"
