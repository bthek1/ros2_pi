#!/usr/bin/env bash
#
# Record a reference clip from the Pi's camera into bags/<name>.
#
# **Why a script and not one `ros2 bag record` command.** Three things have to be
# true of a clip every later phase replays, and each of them is easy to leave out:
#
#  1. **The camera's controls are reset first.** V4L2 controls persist inside the
#     C922 across processes and reboots, so a manual exposure left by somebody's
#     benchmark is baked into the recording — and a clip recorded at 20 fps because
#     `exposure_dynamic_framerate` was set cannot be un-recorded.
#  2. **`/camera_info` rides along.** The clip is replayed by phases that
#     unproject pixels, and the intrinsics that were true when it was recorded are
#     part of the clip. A bag of images alone is not a usable reference.
#  3. **It is recorded on the dev box, not the Pi.** The Pi has 8 GB and a
#     microSD; 60 s of 80 kB frames at 59 Hz is ~280 MB, and writing it there means
#     copying it back afterwards. Recording on this end also means the bag contains
#     what actually crossed the Wi-Fi, which is what the pipeline has to work with.
#
# The clip is the *reference*: every number in milestones C, D and E is measured
# against the same seconds of room, so re-recording it invalidates the comparison.
# Record it once, note its sha256, and keep it.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

NAME=${1:-}
SECONDS_LIMIT=${2:-60}

[[ -n $NAME ]] || {
    echo "usage: bash tools/record-clip.sh <name> [seconds]"
    echo
    echo "  <name>     the directory under bags/ to create, e.g. desk1"
    echo "  [seconds]  default 60"
    exit 2
}

BAG="$PIMESH_WS/bags/$NAME"
[[ -e $BAG ]] && {
    echo "bags/${NAME} already exists. A reference clip is recorded once —"
    echo "delete it deliberately if you really mean to replace it."
    exit 1
}

arm_cleanup

# The camera's own state, first. gates/capture.sh does this for the same reason:
# a rate measured without it is a measurement of whatever the last person left
# behind, and a *clip* recorded without it is that permanently.
bash "$PIMESH_WS/tools/camera-reset.sh" || exit 1

cat <<CHECKLIST

== record-clip: ${NAME}, ${SECONDS_LIMIT}s ==

This wants a person holding the camera. What makes a clip useful to the phases
that replay it:

  * **Sweep slowly.** ORB matches corners between consecutive frames; a fast
    flick blurs them and the pose gate holds through the whole turn. A sweep that
    takes the full minute to cross the room is not too slow.
  * **Texture, not blank wall.** Corners land on edges, print, clutter and
    furniture. A clip of a white wall is a clip with nothing in it to track.
  * **Come back to where you started.** Later phases close loops; a sweep that
    ends where it began gives them something to close.
  * **Keep the room still.** A person walking through the clip is a moving object
    in a map that assumes the world is static.

Recording starts in 5 seconds.

CHECKLIST

sleep 5

# The camera, on the Pi, bounded — with a few seconds of margin over the recording
# so the stream does not stop underneath the recorder.
pi_run_for $(( SECONDS_LIMIT + 12 )) "ros2 run pimesh_camera camera_node" &

# Wait for frames rather than guessing at a sleep: a recorder started before the
# publisher exists still records, and records nothing.
for _ in $(seq 40); do
    count=$(ros2 topic list 2>/dev/null | grep -c '^/image_raw/compressed$' || true)
    [[ ${count:-0} -ge 1 ]] && break
    sleep 0.5
done
[[ ${count:-0} -ge 1 ]] || { echo "FAIL: /image_raw/compressed never appeared"; exit 1; }

echo "recording to bags/${NAME} ..."

# mcap, the default storage here, and both topics. --disable-keyboard-controls for
# the same reason tools/replay.sh has it: a recorder that reads the terminal gets
# SIGTTIN the moment it is not in the foreground.
run_for "$SECONDS_LIMIT" \
    ros2 bag record -s mcap -o "$BAG" \
    --topics /image_raw/compressed /camera_info </dev/null || true

cleanup_both
sleep 1

[[ -r $BAG/metadata.yaml ]] || {
    echo "FAIL: no metadata.yaml in ${BAG} — the recorder was killed before it finished"
    exit 1
}

echo
echo "== bags/${NAME} =="
# The counts, from the bag's own metadata rather than from the recorder's output:
# this is what a later replay will actually find in it.
awk '/topic_metadata:/,0' "$BAG/metadata.yaml" |
    awk '/name:/ {t=$2} /message_count:/ {printf "  %-28s %s messages\n", t, $2}'
echo "  duration                     $(awk '/duration:/{getline; print $2/1e9 "s"}' "$BAG/metadata.yaml")"
echo "  size                         $(du -sh "$BAG" | cut -f1)"
# The sha256 of the mcap, which is how a number quoted against this clip can be
# checked to have been measured against *this* clip. bags/ is git-ignored, so the
# hash in a plan annotation is the only identity it has.
for mcap in "$BAG"/*.mcap; do
    echo "  sha256                       $(sha256sum "$mcap" | cut -d' ' -f1)"
done
echo
echo "Look at it before trusting it:  just replay ${NAME}"
