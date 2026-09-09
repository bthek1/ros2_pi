#!/usr/bin/env bash
#
# P0 gate: one real package builds, and nothing stale pretends to be one.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-hello-build =="

# 1. Nothing stale.
#
# A colcon workspace is an *overlay*: install/setup.bash prepends prefixes to
# AMENT_PREFIX_PATH, and nothing ever removes a package from that tree when its
# source disappears. So `ros2 pkg list` is a statement about install/, not about
# src/, until someone makes the two agree. That is what this check is: the only
# way to keep the package list honest. It lives in tools/check-stale.sh because
# sync-pi runs the same one on the Pi over SSH, and one implementation is the
# point.
if ! stale=$(bash tools/check-stale.sh); then
    echo "FAIL: build artefact(s) with no src/:"
    # shellcheck disable=SC2086  # one line per artefact, and they cannot contain spaces
    printf '  %s\n' $stale
    echo "  fix: bash tools/clean.sh"
    exit 1
fi
echo "stale artefacts in build/ install/ log/latest_build: 0"

# 2. Build, timed. Through the same script `just build` runs, so the gate proves
#    the recipe rather than a second spelling of it.
start=$(date +%s.%N)
bash "$PIMESH_WS/tools/build.sh"
elapsed=$(awk -v a="$start" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')

# 3. The package list says exactly what src/ says.
. "$PIMESH_WS/tools/ros-env.sh" --overlay
mapfile -t pimesh < <(ros2 pkg list | grep '^pimesh_' || true)
if [[ ${#pimesh[@]} -ne 1 || ${pimesh[0]} != pimesh_hello ]]; then
    echo "FAIL: expected exactly [pimesh_hello], ros2 pkg list gave [${pimesh[*]-}]"
    exit 1
fi

echo
echo "ROS_DISTRO      : ${ROS_DISTRO}"
echo "build time      : ${elapsed}s"
echo "pimesh packages : ${pimesh[*]}"
echo "PASS gate-hello-build"
