#!/usr/bin/env bash
#
# P0 gate: one source tree, two ROS distros, and the same interface definitions
# at both ends.
#
# The cross-distro build *is* the phase. ROS 2 makes no ABI promise across
# distros, so a .so built here does not load there — which means every package
# in this workspace has to compile from its own source on each machine, and the
# only way to know that is still true is to do it. A build that succeeds here is
# half a build.
#
# The interface diff is the second half, and it is the one that would rot
# quietly. rosidl generates each machine's C++, Python and typesupport from the
# .msg files at build time; the .msg is what crosses the network, the generated
# code never does. If the Pi's copy of a message drifts from ours — a stale
# build tree, a half-finished rsync, a field added on one side — DDS does not
# announce it. It fails much later, as a subscriber that never fires or a field
# read at the wrong offset. So: ask both machines to print what they think the
# type is, and diff the answers.
#
# Default is a from-scratch build on both machines, because that is the claim.
# `--incremental` skips the clean when you are iterating and want the interface
# diff without the wait; the mode is printed with the numbers so a fast run
# cannot be mistaken for the real one.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-build =="

MODE=scratch
[[ ${1:-} == --incremental ]] && MODE=incremental

INTERFACES=(
    msg/Keypoints
    msg/MeshStats
    msg/PipelineStats
    srv/ResetMap
    srv/SaveMesh
)

fail=0
note() { echo "FAIL: $*"; fail=1; }

# 1. From scratch, unless told otherwise. Both trees, because a clean build here
#    against a stale one there proves nothing about the pair.
if [[ $MODE == scratch ]]; then
    bash "$PIMESH_WS/tools/clean.sh"
    bash "$PIMESH_WS/tools/clean-pi.sh"
fi

# 2. Build here, through the same script `just build` runs — the gate proves the
#    recipe rather than a second spelling of it.
t0=$(date +%s.%N)
bash "$PIMESH_WS/tools/build.sh" >/dev/null || { echo "FAIL: build failed here"; exit 1; }
here_build=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')

# 3. Build on the Pi. build-pi.sh rsyncs source first — source only, never a
#    built tree — and compiles it there under the Pi's own ROS.
t0=$(date +%s.%N)
bash "$PIMESH_WS/tools/build-pi.sh" >/dev/null || { echo "FAIL: build failed on $PI"; exit 1; }
pi_build=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')

. "$PIMESH_WS/tools/ros-env.sh" --overlay

# 4. Different distros, or nothing cross-distro was exercised and the diff below
#    is a file compared with itself.
here_distro=$ROS_DISTRO
pi_distro=$(pi_run 'echo $ROS_DISTRO')
[[ $here_distro != "$pi_distro" ]] ||
    note "both hosts report ${here_distro} — nothing cross-distro was tested"

# 5. The package list says exactly what src/ says, on both machines. A colcon
#    workspace is an overlay and nothing ever removes a package from install/
#    when its source disappears, so `ros2 pkg list` is a statement about
#    install/ until somebody makes the two agree.
mapfile -t want < <(cd "$PIMESH_WS/src" && ls -d */ | tr -d / | sort)
here_pkgs=$(ros2 pkg list | grep '^pimesh_' | sort | tr '\n' ' ')
pi_pkgs=$(pi_ws_run "ros2 pkg list | grep '^pimesh_' | sort" | tr '\n' ' ')
want_pkgs="${want[*]} "
[[ $here_pkgs == "$want_pkgs" ]] || note "here: [${here_pkgs% }], src/ has [${want_pkgs% }]"
[[ $pi_pkgs == "$want_pkgs" ]]   || note "${PI}: [${pi_pkgs% }], src/ has [${want_pkgs% }]"

# 6. The interface definitions themselves, generated independently on each
#    machine from the same .msg, printed by each machine's own ros2 and diffed.
work=$(mktemp -d)
same=0
for iface in "${INTERFACES[@]}"; do
    ros2 interface show "pimesh_msgs/$iface" >"$work/here" 2>&1
    pi_ws_run "ros2 interface show pimesh_msgs/$iface" >"$work/pi" 2>&1
    if diff -q "$work/here" "$work/pi" >/dev/null; then
        same=$(( same + 1 ))
    else
        note "pimesh_msgs/${iface} differs between ${here_distro} and ${pi_distro}"
        diff -u "$work/here" "$work/pi" | head -20 | sed 's/^/  /'
    fi
done

# A definition that is empty on both machines diffs clean and means nothing —
# `ros2 interface show` prints an error to stdout when the type is unknown.
lines=$(wc -l <"$work/here")
(( lines > 0 )) || note "the last interface printed nothing on either machine"

echo
echo "mode             : ${MODE}"
echo "distros          : dev=${here_distro}  pi=${pi_distro}  (assert different)"
echo "build times      : dev=${here_build}s  pi=${pi_build}s  (same source, both from source)"
echo "packages         : ${want_pkgs% }  (assert identical on both)"
echo "interfaces equal : ${same} of ${#INTERFACES[@]}  (assert ${#INTERFACES[@]})"

(( fail == 0 )) || { echo "FAIL gate-build"; exit 1; }
echo "PASS gate-build"
