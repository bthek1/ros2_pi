#!/usr/bin/env bash
#
# Run the unit tests on the Pi, from source, under Jazzy. Syncs first.
#
# Mirrors tools/pi/build-pi.sh. The tests are worth running at both ends for the
# same reason the build is: the two machines compile different code from the
# same sources — different compilers, different architectures (x86_64 against
# aarch64) and different ROS versions — and the arithmetic these tests cover is
# integer-width and endian-sensitive by nature.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh"

bash "$PIMESH_WS/tools/pi/sync-pi.sh"
# **tools/test.sh, not a second spelling of it.** This line used to inline
# `colcon build && colcon test && colcon test-result` itself, which made it a
# rewrite of tools/test.sh that happened to agree — until it did not. Measured
# 2026-09-23 during #14's P5: tools/test.sh grew a step that deletes stale
# result files, this script never got it, and gates/test.sh printed
# `dev=433 pi=490` and PASS. tools/ is rsynced, so the Pi has the same script;
# running it is what keeps the two ends measuring the same thing.
pi_run "cd $PI_WS && bash tools/test.sh"
