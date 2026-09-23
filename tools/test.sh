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

# **Old result files are deleted before the run, because `colcon test-result
# --all` reads every XML under build/ whether this run wrote it or not.** A suite
# that stops being run keeps reporting its last result — passing — for as long as
# the file survives, so a deleted suite is invisible and a moved one is counted
# twice. Measured 2026-09-23 during #14's P5: three suites moved from
# pimesh_bringup to pimesh_instruments and the count went from 433 to 490, which
# is 433 plus those three counted a second time out of bringup's stale files.
# Nothing in the output said so — the totals simply got better.
#
# This is the `cost_mean=0.00` rule for the test count: a number nobody measured
# and a number that is genuinely good must not have the same spelling.
find "$PIMESH_WS/build" -maxdepth 2 -name test_results -type d -exec rm -rf {} + 2>/dev/null || true

colcon test "$@"

# --all so that a package whose tests did not run at all is visible rather than
# absent. `colcon test` itself exits 0 when a test fails — it reports that the
# *test run* completed — so this line is the one that decides.
colcon test-result --all --verbose
