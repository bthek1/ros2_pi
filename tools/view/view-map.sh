#!/usr/bin/env bash
#
# Watch the map — milestone G's picture: map points in space beside the camera's
# trail, with tracking against the local map and (by default) bundle adjustment on.
#
# **A viewer, not evidence.** `bash tools/gates/map.sh` and `bash tools/gates/ba.sh`
# are what pass or fail P14 and P15, against motion-capture ground truth. This is
# here for the one thing only a person can judge: whether the orange points sit on
# the surfaces of the room — walls, the desk, the objects on it — or float in a haze
# in front of and behind them.
#
#     bash tools/view/view-map.sh 600 walk1            local map + BA
#     bash tools/view/view-map.sh 600 walk1 false      local map, no BA — the control
#
# Same shape as tools/view/view-odom.sh, and the teardown is all in just-lib.sh: a
# handler on EXIT and on INT/TERM/HUP that kills both machines *and then checks*,
# and rviz2 **backgrounded** rather than run in the foreground, because bash defers
# a trap until its foreground child returns and an rviz2 signalled during its own
# startup never returns.
#
# **A bag plays once here, not on a loop**, for view-odom's reason: a looping bag's
# stamps jump back by its length and the pose, stamped with them, freezes.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}
BA=${3:-true}
case "$BA" in
    true|false) ;;
    *) echo "ba must be true or false, not '${BA}'"; exit 1 ;;
esac

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/map.rviz"
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

# Before arm_cleanup, always: the cleanup handler kills this workspace's processes,
# so a refusal after the trap is armed would tear down the session it is refusing.
assert_no_session "just view-map"
if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== view-map ==
  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}
  tracking   against the local map; bundle adjustment ${BA}

What you should see, and what each part of it tells you:

  MapPoints  orange, on /map/points: every live map point, republished at most once
             a second when a keyframe changes the map. They should lie on the
             room's surfaces. A haze in depth around each surface is the depth
             network's ~15% keyframe-to-keyframe scale disagreement, which P14
             measured and which bundle adjustment exists to pull in.
  Odometry   the last 500 poses on /odom — the camera's trail through the points.
  Marker     the TSDF surface on /world/mesh. Points *on* the mesh are a map that
             agrees with the fusion; points consistently in front of it are not.
  TF         map -> odom -> base_link -> camera_*. map -> odom is still static
             identity: no loop closure exists yet (milestone H), so the map is
             expressed in odom and nothing ever corrects it.

**What this will not be.** A SLAM map. Nothing here recognises a place it has
seen before, so drift is bounded per step and unbounded over a session, and the
map drifts with it. On TUM fr1/desk (gates/map.sh, 2026-09-30) the local map on
its own tracked *worse* than P7's newest-keyframe tracker; whether bundle
adjustment turns that round is gates/ba.sh's question, not this window's.
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

ros2 launch pimesh_bringup pimesh.launch.py local_map:=true local_ba:="$BA" >/dev/null 2>&1 &

sleep 8

export QT_QPA_PLATFORM=xcb

run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
