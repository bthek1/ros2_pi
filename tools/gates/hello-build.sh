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

# 3. The package list says exactly what src/ says — no more and no fewer. The
#    set is read from src/ rather than written down here, because this gate is
#    about the *scaffolding* and a workspace that has grown a package is not a
#    reason for it to fail. What it still refuses is drift in either direction:
#    a package in install/ that src/ no longer has (step 1 above), or one in
#    src/ that never got built.
. "$PIMESH_WS/tools/ros-env.sh" --overlay
mapfile -t want < <(cd "$PIMESH_WS/src" && ls -d */ | tr -d / | sort)
mapfile -t pimesh < <(ros2 pkg list | grep '^pimesh_' | sort || true)
if [[ "${pimesh[*]-}" != "${want[*]}" ]]; then
    echo "FAIL: ros2 pkg list gave [${pimesh[*]-}], src/ has [${want[*]}]"
    exit 1
fi
if [[ ! " ${pimesh[*]} " == *" pimesh_hello "* ]]; then
    echo "FAIL: pimesh_hello is not in the package list — this gate is about it"
    exit 1
fi

echo
echo "ROS_DISTRO      : ${ROS_DISTRO}"
echo "build time      : ${elapsed}s"
echo "pimesh packages : ${pimesh[*]}"
echo "PASS gate-hello-build"
