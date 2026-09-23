#!/usr/bin/env bash
#
# Watch the room become a triangle surface.
#
# **A viewer, not evidence — and the distinction matters more here than anywhere
# else in this project.** A mesh that looks right is the single most seductive
# false positive there is: a sealed box with no openings looks *more* finished
# than a correct scan, and a plausible-looking room can be metres out of scale
# with nothing on the screen saying so. `bash tools/gates/mesh.sh` is what passes
# or fails P6, and the three PNGs `tools/eval/mesh-views.sh` writes are what it points
# at. This is here so a person can see the thing exists.
#
# Same shape as tools/view/view-depth.sh, and the teardown is all in just-lib.sh: a
# handler on EXIT and on INT/TERM/HUP that kills both machines *and then checks*,
# and rviz2 **backgrounded** rather than run in the foreground, because bash defers
# a trap until its foreground child returns and an rviz2 signalled during its own
# startup never returns.
#
# Takes a bag name to replay instead of the camera: `just view-mesh 600 desk1`.
# With no bag it uses the Pi's live camera.
#
# **A bag plays once here, not on a loop.** `--loop` sends every header stamp ~60 s
# into the past at each wrap; fusion_node looks the pose up at the frame's own
# stamp and tf2 refuses anything older than the newest it holds, so after the first
# wrap nothing would be integrated at all and the surface would stop growing. See
# tools/view/replay.sh for the measurement.
#
# **Give it half a minute.** depth_node loads a 99 MB model and warms a CUDA
# session; fusion needs frames before there is a volume; and mesh_node extracts on
# a 10 s timer, so the first surface appears about ten seconds after the first
# frame is integrated and grows at every tick after that.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/mesh.rviz"
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
assert_no_session "just view-mesh"

# Local-only cleanup when replaying a bag: nothing here touches the Pi, and an SSH
# round trip on the way out of a recipe that never opened one is latency for
# nothing.
if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== view-mesh ==

  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}

What you should see, and what each part of it tells you:

  Mesh       a triangle surface on /world/mesh, growing every ten seconds. It is
             capped at 120 000 triangles by quadric decimation, so it will look
             coarser than the saved PLY — that is the Marker's budget, not the
             map's resolution.
  DepthCloud the live frame, overlaid on the surface it is being fused into. The
             cloud sitting **on** the mesh rather than in front of or behind it is
             the thing to look at: an offset is drift.
  TF         map -> odom -> base_link -> camera_*, with the camera turning in
             place at the centre of the volume.

  Fixed Frame is **map**, not base_link: the mesh lives in the map frame, and
  viewing it from a camera-attached frame makes a static room appear to swim.

**What this will not look like yet, and why.** odometry_node publishes rotation
only — the translation is identically zero — so a hand-held sweep's ~0.9 m of real
arm arc is modelled as no motion at all. The same wall therefore lands at a
different distance every time the camera moves, and the volume fills with layers
of it: the surface comes out as a shell at roughly constant radius rather than as
a desk with a wall behind it. **P7 is what fixes that.** The three failure shapes
P6 lists — doubled walls, a sealed box, a visible hitch at mesh time — are worth
knowing on sight, but doubled walls are the expected state today rather than a
fault.

And the scale is arbitrary. Monocular depth is scale-ambiguous, so the room is
plausibly shaped and the wrong size until depth_scale is pinned with a tape
measure. Do not adjust it by eye against a mesh that looks about right.

  Save the surface at any point, full detail, from another terminal:
    ros2 service call /world/save_mesh pimesh_msgs/srv/SaveMesh "{path: '/tmp/room.ply'}"
    bash tools/eval/mesh-views.sh /tmp/room.ply

  Throw the map away and start again:
    ros2 service call /world/reset_map pimesh_msgs/srv/ResetMap "{}"
${BAG:+
The clip plays **once** and then everything stops growing. That is the end of the
clip, not a crash. The window stays until Ctrl-C or ${SECONDS_LIMIT}s.
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
ros2 launch pimesh_bringup pimesh.launch.py >/dev/null 2>&1 &

# Longer than view-depth waits: this container has the same model to load and a
# CUDA session to warm, and then needs frames integrated before there is anything
# for the mesher to extract.
sleep 8

# Wayland session; rviz2 renders through GLX and needs the xcb platform plugin.
export QT_QPA_PLATFORM=xcb

run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
