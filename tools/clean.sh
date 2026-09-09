#!/usr/bin/env bash
#
# Delete the colcon trees. They are git-ignored and derived; nothing else
# notices, and a stale one is how `ros2 pkg list` starts lying (see
# tools/check-stale.sh).

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

rm -rf build install log
echo "removed build/ install/ log/"
