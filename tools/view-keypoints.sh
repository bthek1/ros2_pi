#!/usr/bin/env bash
#
# Watch ORB find corners, live, with the rotation-only pose next to it.
#
# **A viewer, not evidence.** tools/gates/ipc.sh and tools/gates/keypoints.sh are
# what pass or fail P2 and P3. This is here so a person can see that the tracker is
# alive and sane — green circles on texture, a frame that rotates when they pan —
# before trusting a number about it.
#
# Same shape as tools/view-camera.sh, and the teardown is all in just-lib.sh: a
# handler on EXIT and on INT/TERM/HUP that kills both machines *and then checks*,
# and rviz2 *backgrounded* rather than in the foreground, because bash defers a
# trap until its foreground child returns and an rviz2 signalled during its own
# startup never returns. That leak costs the Pi's camera, which holds
# /dev/video0 exclusively — and on 2026-09-14 this recipe lost it to the other
# half of the same problem: the handler ran, fired one pkill at the far end, and
# exited without ever looking. See the note on kill_pi in tools/just-lib.sh.
#
# Takes a bag name to replay instead of the camera: `bash tools/view-keypoints.sh
# 600 desk1`. With no bag it uses the Pi's live camera.
#
# **A bag plays once here, not on a loop, and that is the one thing about this
# recipe worth reading.** This view exists to show the pose moving, and a pose
# and a looping bag are mutually exclusive. `keypoint_node` stamps
# `odom -> base_link` with the frame's own stamp, as it must; `--loop` sends
# those stamps ~60 s into the past at every wrap; and `tf2::BufferCore` rejects
# any transform older than the newest it holds. Measured 2026-09-13 with
# `tf2_echo` beside a looping player: the edge froze at the bag's last stamp for
# the whole rest of the run, and every listener — RViz included — logged
# TF_OLD_DATA at the frame rate from inside the buffer's own mutex, which stalls
# the render loop and makes both panels stutter. tools/replay.sh's header has
# the long version, including why `--clock` and `use_sim_time` are measured to
# be the wrong fix. So: one pass, and the session ends when the clip does.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
BAG_ARG=${2:-}

RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/keypoints.rviz"
[[ -r $RVIZ_CONFIG ]] || { echo "no RViz config at $RVIZ_CONFIG — build first"; exit 1; }

BAG=
if [[ -n $BAG_ARG ]]; then
    for cand in "$BAG_ARG" "$PIMESH_WS/bags/$BAG_ARG"; do
        [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
    done
    [[ -n $BAG ]] || { echo "no bag at '${BAG_ARG}' (looked for metadata.yaml there and under bags/)"; exit 1; }
fi

# Local-only cleanup when replaying a bag: nothing here touches the Pi, and an SSH
# round trip on the way out of a recipe that never opened one is latency for
# nothing.
# Before arm_cleanup, deliberately: the EXIT handler kills this workspace's
# processes, so refusing after the trap is armed would tear down the session
# being refused.
assert_no_session "just view-keypoints"

if [[ -n $BAG ]]; then arm_cleanup kill_local; else arm_cleanup; fi

cat <<CHECKLIST
== view-keypoints ==

  source     ${BAG:-the live camera on the Pi}${BAG:+ — one pass, not looped}

What you should see, and what each part of it tells you:

  Keypoints  a few hundred circles on the preview. **Green was already being
             followed, yellow is new this frame.** Mostly green with a scatter of
             yellow is the tracker working; mostly yellow is it detecting corners
             and recognising none of them. They should cluster on texture and be
             absent on blank wall.
  TF         camera_optical_frame rotating as the camera pans — and **not
             translating**. P3 is rotation-only by design: these rays cannot
             recover translation, so a pose stuck at the origin is correct here.
             P7 is what makes it move.
  Fixed Frame is odom, because base_link is the frame that moves now.

When the pose gate fails — fewer than 8 matched pairs, or a mean ray residual over
0.03 rad — the node **holds** the last pose and logs the regime change. During a
fast flick you should see the axes stop rather than jump.
${BAG:+
The clip plays **once** and then everything stops updating — the preview holds its
last frame and the TF axes fade out over 30 s. That is the end of the clip, not a
crash: a looping bag replays header stamps minutes into the past, and a pose
published from them is rejected by every TF listener in the domain. The window
stays until Ctrl-C or ${SECONDS_LIMIT}s. \`bash tools/replay.sh\` is the one that
loops, and it can because it publishes no pose at all.
}
CHECKLIST

if [[ -n $BAG ]]; then
    # See tools/replay.sh for why both the flag and the redirect are needed: a
    # backgrounded player that can read its terminal is stopped by SIGTTIN and
    # publishes nothing, silently. **No `--loop`** — see the header.
    run_for "$SECONDS_LIMIT" \
        ros2 bag play "$BAG" --disable-keyboard-controls \
        </dev/null >/dev/null 2>&1 &
else
    # pi_run_for puts `timeout` inside the login shell, so the limit reaches the
    # node rather than the shell wrapping it.
    pi_run_for "$SECONDS_LIMIT" "ros2 run pimesh_camera camera_node" &
fi

# The container: decode_node and keypoint_node in one process, intra-process comms
# on, plus the static frame tree keypoint_node takes its optical-to-body basis from.
ros2 launch pimesh_bringup pimesh.launch.py >/dev/null 2>&1 &

sleep 3

# Wayland session; rviz2 renders through GLX and needs the xcb platform plugin.
export QT_QPA_PLATFORM=xcb

run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
