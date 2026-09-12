#!/usr/bin/env bash
#
# Run the unit tests on the Pi, from source, under Jazzy. Syncs first.
#
# Mirrors tools/build-pi.sh. The tests are worth running at both ends for the
# same reason the build is: the two machines compile different code from the
# same sources — different compilers, different architectures (x86_64 against
# aarch64) and different ROS versions — and the arithmetic these tests cover is
# integer-width and endian-sensitive by nature.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

bash "$PIMESH_WS/tools/sync-pi.sh"
pi_run "cd $PI_WS && source tools/ros-env.sh && colcon build --symlink-install --cmake-args ${PIMESH_CMAKE_ARGS[*]} && colcon test && colcon test-result --all --verbose"
