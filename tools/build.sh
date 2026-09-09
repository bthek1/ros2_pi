#!/usr/bin/env bash
#
# Build this workspace with this machine's own ROS. Run on either machine.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --ros

colcon build --symlink-install --cmake-args "$PIMESH_CMAKE_ARGS" "$@"
