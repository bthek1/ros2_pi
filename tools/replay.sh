#!/usr/bin/env bash
#
# Replay a recorded bag and watch it, with the frame tree next to it.
#
# **A viewer, not evidence**, exactly like tools/view-camera.sh — this is the
# same window fed from disk instead of from the Pi, so that a bag can be looked
# at without the camera, the LAN or the Pi being involved at all. Nothing here
# asserts anything; gates/capture.sh is what passes or fails a claim about the
# stream.
#
# Two things about `ros2 bag play` are not obvious and cost an afternoon
# on 2026-09-12.
#
#  1. **A backgrounded `ros2 bag play` suspends itself, silently.** Playback
#     enables keyboard controls by default (space to pause, cursor keys to
#     step), which means it reads the controlling terminal. A process in the
#     *background* that reads its controlling TTY is sent SIGTTIN by the kernel
#     and stopped — `ps` shows state `T`, it publishes not one message, and it
#     prints no error, because being stopped is not a failure it gets to report.
#     The symptom is a grey Image panel in RViz and a topic with a publisher
#     that never publishes, which reads exactly like a QoS mismatch and is not
#     one. `--disable-keyboard-controls` plus `</dev/null` is the fix, and both
#     halves are here deliberately: the flag stops it wanting the terminal, the
#     redirect means it cannot have it even if a future default changes.
#
#  2. **A bag carries no `tf_static`.** The recordings in bags/ hold
#     /image_raw/compressed and /camera_info and nothing else, while
#     rviz/camera.rviz has base_link as its Fixed Frame. Replayed on its own the
#     image therefore has no frame to hang off and RViz shows the same grey
#     panel for a completely different reason. pimesh.launch.py supplies the
#     three static transforms; it is started here for that and nothing else.
#
# Teardown is local-only — `arm_cleanup kill_local` rather than the default
# cleanup_both — because no part of this touches the Pi, and an SSH round trip
# on the way out of a recipe that never opened one is latency for nothing.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

BAG_ARG=${1:-}
SECONDS_LIMIT=${2:-600}

usage() {
    echo "usage: just replay <bag> [seconds]"
    echo
    echo "  <bag>  a directory name under bags/, or a path to one"
    if compgen -G "$PIMESH_WS/bags/*/metadata.yaml" >/dev/null; then
        echo
        echo "available:"
        for m in "$PIMESH_WS"/bags/*/metadata.yaml; do
            printf '  %s\n' "$(basename "$(dirname "$m")")"
        done
    else
        echo
        echo "bags/ holds no recordings — record one with:"
        echo "  ros2 bag record -s mcap -o bags/<name> \\"
        echo "      --topics /image_raw/compressed /camera_info"
    fi
}

[[ -n $BAG_ARG ]] || { usage; exit 2; }

# Accept both `just replay desk1` and `just replay bags/desk1`, and insist on
# metadata.yaml rather than on the directory existing: a bag whose recorder was
# SIGKILLed leaves the .mcap behind without ever writing the metadata, and
# `ros2 bag play` on one of those fails with a storage error that says nothing
# about which of the two files is missing.
BAG=
for cand in "$BAG_ARG" "$PIMESH_WS/bags/$BAG_ARG"; do
    [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
done
[[ -n $BAG ]] || {
    echo "no bag at '${BAG_ARG}' (looked for metadata.yaml there and under bags/)"
    echo
    usage
    exit 1
}

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/camera.rviz"
[[ -r $RVIZ_CONFIG ]] || { echo "no RViz config at $RVIZ_CONFIG — build first"; exit 1; }

arm_cleanup kill_local

cat <<CHECKLIST
== replay ==

  bag        ${BAG}
  looping    yes, until Ctrl-C or ${SECONDS_LIMIT}s

What you should see:

  Image    the recorded video, at the rate it was recorded. Grey here means
           the replay is not publishing — check \`ros2 topic hz
           /image_raw/compressed\` before suspecting RViz.
  TF       base_link -> camera_link -> camera_optical_frame, from
           pimesh.launch.py rather than from the bag: a recording of these two
           topics carries no tf_static and never has.
  Fixed Frame is base_link.

The intrinsics you are looking at are whatever was true when the bag was
recorded, and a bag recorded before a calibration existed carries the nominal
placeholder. \`ros2 topic echo --once /camera_info\` during playback is the only
honest way to know which.

CHECKLIST

# The frames. Static transforms are latched, so this only has to be up before
# RViz asks — but it stays up, because a transient-local publisher that exits
# takes its data with it.
ros2 launch pimesh_bringup pimesh.launch.py >/dev/null 2>&1 &

# The bag. See note 1 at the top for why both the flag and the redirect are
# here; dropping either one is how this recipe goes quietly blank.
run_for "$SECONDS_LIMIT" \
    ros2 bag play "$BAG" --loop --disable-keyboard-controls \
    </dev/null >/dev/null 2>&1 &

sleep 2

# The session is Wayland; rviz2 renders through GLX and needs the xcb platform
# plugin. Hardware GL 4.6 on driver 595.84 as of 2026-08-31.
export QT_QPA_PLATFORM=xcb

# Backgrounded and waited on, never foreground — bash defers a trap until the
# foreground child returns, and an rviz2 signalled during its own startup never
# returns. tools/view-camera.sh carries the long version of that measurement.
run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
