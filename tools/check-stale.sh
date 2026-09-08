#!/usr/bin/env bash
#
# List colcon artefacts belonging to packages this workspace has no source for,
# and exit 1 if there are any. Silent and 0 when the build tree agrees with src/.
#
# Run from a workspace root, on either machine. It exists as a file rather than
# a few lines inside a recipe because both machines need it and the Pi gets it
# by rsync: a workspace whose build tree has drifted from its sources lies to
# `ros2 pkg list` in exactly the same way on Jazzy as on Lyrical.

set -euo pipefail
cd "${1:-.}"

stale=()
for tree in build install log/latest_build; do
    [[ -d "$tree" ]] || continue
    for dir in "$tree"/*/; do
        [[ -d "$dir" ]] || continue
        pkg=$(basename "$dir")
        [[ -d "src/$pkg" ]] || stale+=("$tree/$pkg")
    done
done

if (( ${#stale[@]} )); then
    printf '%s\n' "${stale[@]}"
    exit 1
fi
