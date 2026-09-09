#!/usr/bin/env bash
#
# Run the composed container here. $1 = how long to run it, default 30 s.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

arm_cleanup kill_local

# run_for, not a bare `timeout`: this recipe exists to be watched and then
# interrupted, and a plain `timeout` moves ros2 launch out of the foreground
# process group where Ctrl-C cannot reach it. See tools/just-lib.sh.
run_for "${1:-30}" ros2 launch pimesh_hello hello.launch.py
