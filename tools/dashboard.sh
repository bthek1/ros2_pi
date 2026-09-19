#!/usr/bin/env bash
#
# The pipeline with a browser tab on it: http://localhost:8080
#
# **Not evidence, and not an RViz view either.** `bash tools/gates/dashboard.sh`
# is what passes or fails P8, and its whole assertion is about what an attached
# client costs — so **opening this page while that gate runs invalidates the
# run**. This script is for a person who wants to watch.
#
# The page shows three things and computes none of them: per-stage rate, latency
# and *two* drop columns straight off `/pipeline/stats`; the annotated camera
# frame and the colour-mapped depth, as JPEG the pipeline already produces; and
# the surface with the camera's trajectory drawn through it. Every number on it
# is a number some node measured about itself, which is what makes the page and
# `ros2 topic echo` unable to disagree.
#
# Takes a bag name to replay instead of the camera: `bash tools/dashboard.sh 600 desk1`.
# With no bag it uses the Pi's live camera — and that is the one configuration
# where the `capture` row appears at all, since it is published by camera_node on
# the Pi and a bag does not carry it.
#
# **A bag plays once here, not on a loop**, for the reason tools/replay.sh
# documents: a looping bag replays every header stamp ~60 s into the past at each
# wrap, and the pose would freeze at the bag's final stamp for the rest of the run.
#
# Give it half a minute: depth_node loads a 99 MB model and warms a CUDA session,
# and the first surface appears about ten seconds after the first frame is fused.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}
PORT=${3:-8080}

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
assert_no_session "just dashboard"

if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== dashboard ==

  open       http://localhost:${PORT}   (and from a phone on the same network,
             http://$(hostname -I 2>/dev/null | awk '{print $1}'):${PORT})
  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}

What to look at, and what each part tells you:

  the table  one row per stage, straight off /pipeline/stats. **Two drop columns,
             never one**: 'by design' is a mailbox overwrite — depth dropping two
             frames in three is its single-slot mailbox working exactly as
             intended — and 'lost' is a fault. Summed into one number the healthy
             pipeline and the broken one look identical.
  STALE      a row with nothing for two seconds says so rather than freezing on
             its last value looking healthy. Measured on **receipt**, never on a
             stamp: the capture row is stamped on the Pi and read here.
  the strips the annotated camera frame and the depth map, colour-mapped against
             a fixed [0, max_range] scale in depth_node — so a colour is a
             distance across frames, not within one.
  the scene  the surface, with the trajectory drawn through it, oldest dim and
             newest bright. Drag to orbit, wheel to zoom, shift-drag to pan.
  footer     'frames dropped to slow clients' is the pacing rule reporting itself.
             It should be 0 with one browser on a LAN; it climbs if you background
             the tab, and **the pipeline's rates must not move when it does**.
${BAG:+
  The 'capture' row will not appear: it is published by camera_node on the Pi and
  a bag does not carry it. Run without a bag to see it.
}
  The clip plays once and then everything stops growing. The page stays until
  Ctrl-C or ${SECONDS_LIMIT}s.
CHECKLIST

if [[ -n $BAG ]]; then
    # See tools/replay.sh for why both the flag and the redirect are needed: a
    # backgrounded player that can read its terminal is stopped by SIGTTIN and
    # publishes nothing, silently.
    run_for "$SECONDS_LIMIT" \
        ros2 bag play "$BAG" --disable-keyboard-controls \
        </dev/null >/dev/null 2>&1 &
else
    # pi_run_for puts `timeout` inside the login shell, so the limit reaches the
    # node rather than the shell wrapping it. A camera_node orphaned on the Pi
    # holds /dev/video0 exclusively and every later session dies on it.
    pi_run_for "$SECONDS_LIMIT" "ros2 run pimesh_camera camera_node" &
fi

# The container plus dashboard_node in its own process — which is the whole
# point of it being a separate process: killing the page's server does not touch
# the map.
run_for "$SECONDS_LIMIT" \
    ros2 launch pimesh_bringup pimesh.launch.py dashboard:=true dashboard_port:="$PORT" &
wait $! || true
