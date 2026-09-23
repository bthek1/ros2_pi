#!/usr/bin/env bash
#
# Watch monocular depth come out of the GPU, as a coloured cloud of the room.
#
# **A viewer, not evidence.** `bash tools/gates/depth.sh` is what passes or fails
# P4: the execution provider, the per-frame cost against an 80 ms budget, and
# whether /depth/rgb is byte-identical to the frame each depth map was inferred
# on. This is here so a person can see the room has a shape before trusting a
# number about it — and because a cloud that comes out sideways, or flat, is
# obvious in a second here and takes a paragraph to assert.
#
# Same shape as tools/view/view-keypoints.sh, and the teardown is all in just-lib.sh:
# a handler on EXIT and on INT/TERM/HUP that kills both machines *and then
# checks*, and rviz2 **backgrounded** rather than run in the foreground, because
# bash defers a trap until its foreground child returns and an rviz2 signalled
# during its own startup never returns.
#
# Takes a bag name to replay instead of the camera: `bash tools/view/view-depth.sh 600
# desk1`. With no bag it uses the Pi's live camera.
#
# **A bag plays once here, not on a loop**, for the reason tools/view/replay.sh
# documents at length: odometry_node is in this container too and stamps
# `odom -> base_link` with the frame's own stamp, `--loop` sends those stamps
# ~60 s into the past at every wrap, and tf2 rejects any transform older than the
# newest it holds — so the pose freezes and every listener in the domain logs
# TF_OLD_DATA at the frame rate, from inside the TF buffer's own mutex. This view
# needs the frame tree to be alive, so: one pass.
#
# **Give it a few seconds before judging it.** depth_node loads a 99 MB model and
# runs a warm-up inference at startup — the first one costs 300-860 ms of CUDA
# context creation, cuBLAS handles and kernel autotuning — so the cloud appears
# noticeably after the preview does.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/depth.rviz"
[[ -r $RVIZ_CONFIG ]] || { echo "no RViz config at $RVIZ_CONFIG — build first"; exit 1; }

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || {
    echo "no model at ${MODEL}"
    echo "models/ is git-ignored. Fetch it with: bash tools/fetch-model.sh"
    exit 1
}

BAG=
if [[ -n $BAG_ARG ]]; then
    for cand in "$BAG_ARG" "$PIMESH_WS/bags/$BAG_ARG"; do
        [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
    done
    [[ -n $BAG ]] || { echo "no bag at '${BAG_ARG}' (looked for metadata.yaml there and under bags/)"; exit 1; }
fi

# Before arm_cleanup, deliberately: the EXIT handler kills this workspace's
# processes, so refusing after the trap is armed would tear down the session
# being refused.
assert_no_session "just view-depth"

# Local-only cleanup when replaying a bag: nothing here touches the Pi, and an SSH
# round trip on the way out of a recipe that never opened one is latency for
# nothing.
if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== view-depth ==

  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}

What you should see, and what each part of it tells you:

  DepthCloud a recognisable shell of the room, coloured by the camera image and
             sitting **in front of** camera_optical_frame. A desk edge should be
             nearer than the wall behind it. A flat plane at one distance means
             the reciprocal or the clip is wrong.
  Depth      the same map colour-mapped with **inferno over a fixed 0-6 m** —
             near bright yellow, mid orange and red, far black. The scale does not
             move between frames, so a colour is a distance: large black regions
             are the 6 m clip, this pipeline's "far away or don't know", and the
             two are the same answer on purpose.
  TF         camera_optical_frame with z pointing into the scene. A cloud rotated
             90 degrees from those axes means something re-derived the optical
             convention rather than naming the static edge.

  **Do not trust the scale.** Monocular depth is scale-ambiguous, so the room is
  plausibly shaped and the wrong size — the predecessor's came out 2.69x out.
  P5 is what pins it, with a tape measure. Do not adjust depth_scale by eye.

  The cloud appears a few seconds after the preview: depth_node loads a 99 MB
  model and warms the CUDA session before its first real frame.
${BAG:+
The clip plays **once** and then everything stops updating. That is the end of
the clip, not a crash. The window stays until Ctrl-C or ${SECONDS_LIMIT}s.
}
CHECKLIST

if [[ -n $BAG ]]; then
    # See tools/view/replay.sh for why both the flag and the redirect are needed: a
    # backgrounded player that can read its terminal is stopped by SIGTTIN and
    # publishes nothing, silently. **No `--loop`** — see the header.
    run_for "$SECONDS_LIMIT" \
        ros2 bag play "$BAG" --disable-keyboard-controls \
        </dev/null >/dev/null 2>&1 &
else
    # pi_run_for puts `timeout` inside the login shell, so the limit reaches the
    # node rather than the shell wrapping it. A camera_node orphaned on the Pi
    # holds /dev/video0 exclusively and every later session dies on it.
    pi_run_for "$SECONDS_LIMIT" "ros2 run pimesh_camera camera_node" &
fi

# The container: decode_node, keypoint_node and depth_node in one process with
# intra-process comms on, plus the static frame tree the cloud is placed by.
ros2 launch pimesh_bringup pimesh.launch.py >/dev/null 2>&1 &

# Longer than the other viewers wait: this container has a model to load and a
# CUDA session to warm before it publishes anything at all.
sleep 6

# Wayland session; rviz2 renders through GLX and needs the xcb platform plugin.
export QT_QPA_PLATFORM=xcb

run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
