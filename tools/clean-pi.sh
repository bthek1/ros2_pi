#!/usr/bin/env bash
#
# Delete the Pi's colcon trees.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

pi_run "cd $PI_WS && rm -rf build install log"
echo "removed $PI_WS/{build,install,log} on $PI"
