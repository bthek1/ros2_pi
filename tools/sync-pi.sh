#!/usr/bin/env bash
#
# Ship source to the Pi. Source only — never a built tree.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

# src, tools and the justfile. Not build/, not install/, not log/ — and this is
# the one thing in the project that cannot be worked around. ROS 2 makes no ABI
# promise across distros, so a Lyrical .so does not load under Jazzy. The Pi
# compiles the same source; it never receives a binary.
rsync -a --delete -e "ssh ${PI_SSH[*]}" \
    --exclude '__pycache__' \
    src tools justfile "$PI:$PI_WS/"

# The sync may have just invalidated the Pi's build tree — `--delete` can remove
# a package's sources while its artefacts stay behind, and a colcon overlay
# never forgets a package on its own. Derived data, so clearing it is safe;
# saying so is not optional.
if ! stale=$(pi_run "cd $PI_WS && bash tools/check-stale.sh"); then
    echo "sync-pi: the Pi's build tree has artefacts with no source:"
    printf '  %s\n' $stale
    echo "sync-pi: clearing it — colcon will not do this for you"
    bash "$PIMESH_WS/tools/clean-pi.sh"
fi
echo "sync-pi: source is current on $PI"
