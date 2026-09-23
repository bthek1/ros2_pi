#!/usr/bin/env bash
#
# Watch the camera move — the picture of P7, and it is a side-by-side.
#
# **A viewer, not evidence.** `bash tools/gates/odom.sh` is what passes or fails
# P7, and it prints both regimes' numbers beside each other. This is here because
# the difference between them is far more legible as two shapes on a screen than
# as two numbers in a table, and because there is one thing only a person can
# judge: whether the trail looks like a hand moving through a room or like a
# random walk that happens to have a small mean.
#
# **Run it twice, once in each regime, and compare:**
#
#     bash tools/view/view-odom.sh 600 desk1                      6-DoF
#     bash tools/view/view-odom.sh 600 desk1 rotation_only        the control
#
# Rotation-only piles every arrow at the origin, spinning in place: the ~0.9 m of
# real arm arc a hand sweep carries, reported as zero. 6-DoF draws an arc through
# space. That difference is the whole of P7.
#
# Same shape as tools/view/view-mesh.sh, and the teardown is all in just-lib.sh: a
# handler on EXIT and on INT/TERM/HUP that kills both machines *and then checks*,
# and rviz2 **backgrounded** rather than run in the foreground, because bash defers
# a trap until its foreground child returns and an rviz2 signalled during its own
# startup never returns.
#
# **A bag plays once here, not on a loop.** `--loop` sends every header stamp ~60 s
# into the past at each wrap; odometry_node stamps the pose with the frame's own
# stamp and tf2 refuses anything older than the newest it holds, so after the first
# wrap the pose would freeze at the bag's final stamp for the rest of the run. See
# tools/view/replay.sh for the measurement.
#
# **Give it half a minute.** depth_node loads a 99 MB model and warms a CUDA
# session, and in sixdof **no pose is published at all until depth is running** —
# the trajectory comes from depth-backed landmarks, so an empty 3D view for the
# first twenty seconds is the model loading, not a fault.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}
REGIME=${3:-sixdof}

case "$REGIME" in
    sixdof|rotation_only) ;;
    *) echo "regime must be sixdof or rotation_only, not '${REGIME}'"; exit 1 ;;
esac

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/odom.rviz"
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
assert_no_session "just view-odom"

# Local-only cleanup when replaying a bag: nothing here touches the Pi, and an SSH
# round trip on the way out of a recipe that never opened one is latency for
# nothing.
if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== view-odom ==

  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}
  regime     ${REGIME}

What you should see, and what each part of it tells you:

  Odometry   arrows on /odom, the last 500 kept, so the trail *is* the trajectory.
             In sixdof it should wander through space along an arc a hand could
             have made. In rotation_only every arrow sits at the origin and only
             turns — that is not a fault, it is P3's honest scope: bearing rays
             cannot see translation, so the pose says so rather than guessing.
  Marker     the surface on /world/mesh, which is what the trajectory is being
             judged by. It grows every ten seconds.
  DepthCloud the live frame, overlaid on the surface it is being fused into. The
             cloud sitting **on** the mesh rather than in front of or behind it is
             the thing to look at: an offset is drift.
  TF         map -> odom -> base_link -> camera_*, with base_link now *moving*
             rather than only turning.

  Fixed Frame is **map**, not base_link: the trajectory lives in the map frame,
  and viewing it from a camera-attached frame makes a moving camera look still.

**The side-by-side is the point.** Run the same clip both ways and put the two
windows next to each other. Then look back at the mesh: the doubled walls of
milestone D should have collapsed toward single surfaces.

**What this still will not be.** The scale is arbitrary — monocular depth is
scale-ambiguous, so the room is plausibly shaped and the wrong size until
depth_scale is pinned with a tape measure, and the trajectory is in those same
arbitrary units. A trail 3 units long is not a claim about metres. And on a clip
that is mostly a *pan*, the surface gap cannot tell the two regimes apart:
gates/odom.sh says so in its own output and names what would change it.
${BAG:+
The clip plays **once** and then everything stops. That is the end of the clip,
not a crash. The window stays until Ctrl-C or ${SECONDS_LIMIT}s.
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

# The container: decode, keypoints, depth, fusion and mesh in one process with
# intra-process comms on, plus the static frame tree it is all placed by.
ros2 launch pimesh_bringup pimesh.launch.py odom_regime:="$REGIME" >/dev/null 2>&1 &

# The same wait view-mesh uses: a 99 MB model, a CUDA session, and then frames
# have to be integrated before there is anything to look at.
sleep 8

# Wayland session; rviz2 renders through GLX and needs the xcb platform plugin.
export QT_QPA_PLATFORM=xcb

run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
