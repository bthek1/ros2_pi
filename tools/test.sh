#!/usr/bin/env bash
#
# Run this workspace's unit tests with this machine's own ROS. Either machine.
#
# The counterpart of tools/build.sh, and it exists for the same reason: one
# spelling of the command, so that the gate proves the thing a person runs
# rather than a second version of it.
#
# `colcon test` takes no --cmake-args, so the interpreter has to be pinned at
# *build* time — which is what tools/build.sh already does. Building first is
# therefore not a convenience here, it is how the tests come to exist: nothing
# under test/ is compiled until BUILD_TESTING is on, and a `colcon test` against
# an unbuilt tree reports "0 tests" and exits 0. That is a pass that means
# nothing, so this script always builds.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --ros

bash "$PIMESH_WS/tools/build.sh" "$@"
colcon test "$@"

# --all so that a package whose tests did not run at all is visible rather than
# absent. `colcon test` itself exits 0 when a test fails — it reports that the
# *test run* completed — so this line is the one that decides.
colcon test-result --all --verbose
