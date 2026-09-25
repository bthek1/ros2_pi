#!/usr/bin/env bash
#
# P12 gate: this project's distances stop being in an unknown unit.
#
#   bash tools/gates/scale.sh                 # assert against the recorded tape figure
#   bash tools/gates/scale.sh 1.985           # ...and what to run at the wall, first time
#
# **Everything this pipeline reports is plausibly shaped and the wrong size until
# this passes.** Monocular depth is scale-ambiguous — the model says "twice as
# far", never "three metres" — so `depth_scale` has been 10.0 since P4 because
# somebody typed it, and every distance downstream is that constant times an
# arbitrary reading. A mesh, a trajectory, a voxel size and a speed limit are all
# in it.
#
# --- This one needs a person, and here is the whole of what they must do -------
#
#   1. Find a flat surface — a wall, a door, a closed cupboard — with nothing in
#      front of it, and measure the distance from the camera's front element to it
#      with a tape. **Between 1.5 m and 2.5 m.** Not closer, because `min_range_m`
#      is 0.15 m and the near field is where the network is least like a metric
#      sensor; not further, because at the *unpinned* scale of 10.0 the map reads
#      roughly twice what it should and `max_range_m` is 6.0 — a surface at 3.5 m
#      true would read past the clip, and a clipped reading is not a distance.
#      This gate refuses on clipping rather than measuring it.
#   2. Point the camera square-on at it, filling the middle of the frame.
#      Square-on matters: a depth map carries *z*, not distance along the ray, so
#      an oblique patch reads a range of distances and the spread this gate prints
#      is how you will know.
#   3. **Check the framing before recording**, with `just view-depth`. The depth
#      preview is a fixed [0, max_range_m] inferno map with near bright and the
#      clip black, so a colour *is* a distance: **the middle of the frame must be
#      bright, not black.** Black is at or past 6 m, which is the absence of a
#      reading, and this gate refuses a clip whose patch is mostly that — after
#      you have walked back from the wall. Ten seconds of looking here is the
#      difference between one trip and two.
#   4. `bash tools/record-clip.sh scale1 20`. It resets the camera's V4L2 controls
#      first, which is not optional — a clip recorded under a stale manual
#      exposure cannot be un-recorded.
#   5. `bash tools/gates/scale.sh <the tape figure in metres>`. It will refuse,
#      because nothing is recorded in git yet, **and it will print the exact two
#      lines to paste into `config/pimesh.yaml`.**
#   6. Paste them, rebuild, and run `bash tools/gates/scale.sh` with no argument.
#      That is the phase's test.
#
# **Record the clip while you are there.** One clip of a surface at a measured
# distance turns this into a replayable test forever; a second trip to the wall
# does not. The same visit should also record `bags/walk1` — that is P13, and it
# is the other thing in this milestone that a script cannot do.
#
# --- The false greens ----------------------------------------------------------
#
#  1. **Reading `/depth` at a single pixel at the principal point.**
#     `test_tsdf_volume` already pins that a depth map reports *z* and not ray
#     length — agreeing perfectly at the principal point and 30% wrong in the
#     corners — so the centre is the *best* case in the frame and one hot pixel
#     there would set this project's unit for good. `scale_probe` takes a centred
#     patch, reports its spread, and asserts on the median.
#  2. **Counting far-clip values as distances.** `depth_to_metres` writes exactly
#     `max_range_m` wherever the model's inverse depth falls below its floor, so a
#     clipped pixel is the *absence* of a distance dressed as 6 m. Depth is linear
#     in `depth_scale` only below the clip, so an implied scale derived from a
#     patch with clip in it comes out **too small**, and nothing about the number
#     looks wrong. `centred_patch_stats` excludes and counts them; this gate
#     refuses past a small fraction.
#  3. **Passing vacuously.** A missing clip or a missing tape figure must not be a
#     pass. Either would put a made-up unit under every later measurement in this
#     project, and "the gate is green" would be the reason nobody looked again.
#
# --- What this asserts once it is set ------------------------------------------
#
# That the `depth_scale` in `config/pimesh.yaml` is still the value which makes
# `bags/scale1` read back at the distance somebody measured. That is a regression
# test on the constant *and* on everything upstream of it: change the model, the
# clip, or the arithmetic either side of the network, and this is what says the
# unit moved.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-scale =="

TAPE_ARG=${1:-}

# Agreement between the measured median and the tape, as a fraction. #10's P12
# names 3%. For scale: the camera's own `fx` is pinned to ±2.2% (P9), so a
# tighter budget here would be asserting below the calibration it rests on.
MAX_DISAGREE=0.03
# Clip in the patch. Not zero, because a few pixels on a picture frame or a light
# fitting at the edge of the patch are ordinary; well below the fraction at which
# the median starts moving toward the clip.
MAX_CLIPPED=0.02
# Spatial spread across the patch, as a fraction of the median. A flat surface
# square-on should read one distance; 10% at 2 m is 20 cm, which is a wall seen
# at a considerable angle or a patch that is not on one surface.
MAX_SPREAD=0.10
# Frames that produced a usable patch. 20 s at ~17 Hz is ~340.
MIN_FRAMES=100

# Before arm_cleanup, always: the cleanup handler kills this workspace's
# processes, so a refusal after the trap is armed would tear down the session it
# is refusing to disturb.
assert_no_session "bash tools/gates/scale.sh"

arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

PY=/usr/bin/python3
CONFIG="$PIMESH_WS/src/pimesh_bringup/config/pimesh.yaml"

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || {
    echo "FAIL: no model at ${MODEL}"
    echo
    echo "models/ is git-ignored, so a fresh clone has none. Fetch it with:"
    echo "  bash tools/fetch-model.sh"
    exit 1
}

# --- Refusal 1: the clip ------------------------------------------------------
BAG="$PIMESH_WS/bags/scale1"
[[ -r $BAG/metadata.yaml ]] || {
    echo "FAIL: no clip at bags/scale1"
    echo
    echo "**This phase needs a person, and this is the part a script cannot do.**"
    echo "Find a flat surface with nothing in front of it, measure the distance"
    echo "from the camera to it with a tape — between 1.5 m and 2.5 m — point the"
    echo "camera square-on so it fills the middle of the frame, and record:"
    echo
    echo "  bash tools/record-clip.sh scale1 20"
    echo
    echo "Then run this gate with the tape figure, which will tell you what to put"
    echo "in config/pimesh.yaml:"
    echo
    echo "  bash tools/gates/scale.sh <metres>"
    echo
    echo "Record bags/walk1 on the same visit — that is P13, and it is the other"
    echo "thing in this milestone that needs you in the room:"
    echo
    echo "  bash tools/record-clip.sh walk1 60"
    echo
    echo "This gate refuses rather than passing, because a vacuous pass here would"
    echo "put a made-up unit under every later measurement in this project."
    exit 1
}

# --- Refusal 2: the tape figure -----------------------------------------------
#
# Read out of the comment immediately above `depth_scale`. A parsed comment is an
# odd contract and it is the right one: the alternative is a second file or a
# parameter no node declares, and either puts the justification somewhere the
# number can drift away from. Adjacency is the guarantee, and
# `test_the_depth_scale_reference_marker_is_well_formed_if_it_exists` enforces it
# so the comment is a checked thing rather than prose.
read -r RECORDED_M RECORDED_CLIP RECORDED_DATE DEPTH_SCALE MAX_RANGE < <("$PY" - "$CONFIG" <<'READ'
import re
import sys

import yaml

lines = open(sys.argv[1]).read().split('\n')
marker = re.compile(r'^\s*#\s*depth_scale_reference:\s*([0-9]+(?:\.[0-9]+)?)\s*m,\s*(\S+),'
                    r'\s*measured\s+(\d{4}-\d{2}-\d{2})\s*$')
found = ('-', '-', '-')
for i, line in enumerate(lines):
    m = marker.match(line)
    if m and i + 1 < len(lines) and re.match(r'^\s*depth_scale:', lines[i + 1]):
        found = m.groups()
        break
cfg = yaml.safe_load(open(sys.argv[1]))['/**/depth_node']['ros__parameters']
print(found[0], found[1], found[2], cfg['depth_scale'], cfg['max_range_m'])
READ
)

TAPE=$RECORDED_M
if [[ $RECORDED_M == "-" ]]; then
    TAPE=$TAPE_ARG
fi

if [[ -z $TAPE ]]; then
    echo "FAIL: no measured distance recorded beside depth_scale in config/pimesh.yaml,"
    echo "      and none given on the command line."
    echo
    echo "bags/scale1 exists, so the recording has been made. What is missing is the"
    echo "number somebody read off a tape. Run:"
    echo
    echo "  bash tools/gates/scale.sh <metres>"
    echo
    echo "and it will measure the clip and print the two lines to paste."
    exit 1
fi

# Two sources of one number is the trap this whole project is about. If both are
# present they have to agree, and a disagreement is a refusal rather than a
# precedence rule — there is no reading of "the file says 2.0 and you typed 1.8"
# that this gate should resolve on its own.
if [[ $RECORDED_M != "-" && -n $TAPE_ARG ]]; then
    if [[ $(awk -v a="$RECORDED_M" -v b="$TAPE_ARG" \
        'BEGIN { print ((a - b < 0 ? b - a : a - b) < 1e-9) ? 1 : 0 }') -ne 1 ]]; then
        echo "FAIL: config/pimesh.yaml records ${RECORDED_M} m and you passed ${TAPE_ARG} m."
        echo "      Two sources of one number, disagreeing. Fix the file or drop the"
        echo "      argument; this gate will not choose."
        exit 1
    fi
fi

work=$(mktemp -d)

read -r CLIP_SECONDS CLIP_FRAMES < <("$PY" - "$BAG/metadata.yaml" <<'META'
import sys

import yaml

info = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
images = 0
for entry in info['topics_with_message_count']:
    if entry['topic_metadata']['name'].endswith('image_raw/compressed'):
        images = entry['message_count']
print(f"{info['duration']['nanoseconds'] / 1e9:.1f} {images}")
META
)
[[ ${CLIP_FRAMES:-0} -gt 100 ]] || {
    echo "FAIL: ${BAG} holds ${CLIP_FRAMES:-0} images on /image_raw/compressed"
    exit 1
}
MEASURE_S=$(awk -v s="$CLIP_SECONDS" 'BEGIN { printf "%d", s + 2 }')

echo "clip       : ${BAG}  (${CLIP_SECONDS}s, ${CLIP_FRAMES} frames)"
echo "tape       : ${TAPE} m$([[ $RECORDED_M == "-" ]] && echo "  (from the command line — nothing is recorded in git yet)" || echo "  (recorded ${RECORDED_DATE}, ${RECORDED_CLIP})")"
echo "depth_scale: ${DEPTH_SCALE}   far clip ${MAX_RANGE} m"

# --- One run of the real pipeline ---------------------------------------------
#
# The real launch file, with scale_probe loaded into the same container. `/depth`
# is 3.7 MB a frame; out of process the instrument would be a large part of the
# load on the thing it is reading.
log=$work/scale.log
timeout -s INT $(( MEASURE_S + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
    probe:=scale_probe \
    probe_duration_s:="$(awk -v s="$MEASURE_S" 'BEGIN { printf "%.1f", s + 6 }')" \
    >"$log" 2>&1 &

ready=0
for _ in $(seq 120); do
    if grep -q "fusion_node up:" "$log" 2>/dev/null; then ready=1; break; fi
    sleep 0.5
done
(( ready == 1 )) || {
    echo "FAIL: the container never came up within 60 s"
    tail -40 "$log"
    echo "FAIL gate-scale"
    exit 1
}

# Once, not --loop: a looping bag replays stamps into the past and freezes the TF
# tree. Both halves of the terminal handling are here for tools/view/replay.sh's
# reason — a backgrounded `ros2 bag play` that can read its controlling TTY is
# sent SIGTTIN and stops, silently, publishing nothing.
timeout -s INT $(( MEASURE_S + 20 )) ros2 bag play "$BAG" \
    --disable-keyboard-controls </dev/null >"$log.play" 2>&1 &

sleep $(( MEASURE_S + 10 ))
kill_local
sleep 1

probe_value() {          # $1 = key
    grep -h 'scale_probe' "$log" | grep -o 'scale_probe result .*' |
        awk -v key="$1" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { last = kv[2] } }
        } END { print (last == "") ? "" : last }'
}

frames=$(probe_value frames)
seen=$(probe_value seen)
empty=$(probe_value empty)
median=$(probe_value median_m)
q1=$(probe_value q1_m)
q3=$(probe_value q3_m)
spread_time=$(probe_value spread_time_m)
spread_space=$(probe_value spread_space_m)
clipped=$(probe_value clipped_frac)
clipped_max=$(probe_value clipped_max)

if [[ -z ${frames:-} || ${frames:-0} -lt 1 ]]; then
    note "scale_probe measured nothing"
    grep -iE 'scale_probe|error|refus' "$log" | tail -10
    echo "FAIL gate-scale"
    exit 1
fi

# **The number the person came for.** Depth is linear in `depth_scale` below the
# clip, so the scale that would make this clip read back at the tape figure is
# exactly the current one times the ratio. Derived rather than measured, and
# exact — which is why the clipping refusal below is not optional: past the clip
# the relationship is not linear and this arithmetic is quietly wrong.
implied=$(awk -v s="$DEPTH_SCALE" -v tape="$TAPE" -v got="$median" \
    'BEGIN { printf "%.4f", (got > 0) ? s * tape / got : 0 }')
disagree=$(awk -v tape="$TAPE" -v got="$median" \
    'BEGIN { d = (got - tape) / tape; printf "%.4f", (d < 0) ? -d : d }')
spread_frac=$(awk -v s="$spread_space" -v m="$median" \
    'BEGIN { printf "%.4f", (m > 0) ? s / m : 0 }')

# --- The assertions -----------------------------------------------------------
(( frames >= MIN_FRAMES )) ||
    note "only ${frames} frames had a usable centred patch out of ${seen} (${empty} were all clip or NaN) — floor ${MIN_FRAMES}"
in_range "$clipped_max" 0 "$MAX_CLIPPED" ||
    note "up to ${clipped_max} of the patch was at the far clip (ceiling ${MAX_CLIPPED}) — a clipped pixel is the *absence* of a distance written as ${MAX_RANGE} m, and depth is linear in depth_scale only below it, so every number here is wrong in the direction that makes the scale look smaller. Re-record closer to the surface"
in_range "$spread_frac" 0 "$MAX_SPREAD" ||
    note "the patch spans ${spread_space} m within a frame, ${spread_frac} of its own median (ceiling ${MAX_SPREAD}) — a depth map carries z and not ray length, so that is a surface seen at an angle, or a patch that is not on one surface"

if [[ $RECORDED_M == "-" ]]; then
    # **The first run, at the wall.** This exits non-zero either way — nothing is
    # recorded in git yet, and a green gate here would be the reason nobody came
    # back to record it. What changes is whether it hands over a number.
    echo
    echo "=============================== gate-scale =============================="
    echo "measured   : ${median} m over ${frames} of ${seen} frames (q1 ${q1}, q3 ${q3})"
    echo "tape       : ${TAPE} m"
    echo "spread     : ${spread_space} m within a frame (${spread_frac} of the median),"
    echo "             ${spread_time} m across frames"
    echo "clipped    : ${clipped} of the patch typically, ${clipped_max} at worst"
    echo
    if (( fail )); then
        # **Refused, so no number.** Printing an implied scale under a failed
        # clipping or spread check would be handing somebody a value to paste into
        # the config that this gate has just finished saying is wrong — and it is
        # wrong in a direction that looks plausible, which is the whole class of
        # failure this project keeps paying for. The recording has to be fixed
        # first.
        echo "**This clip cannot give a scale, and the checks above say why.** No"
        echo "number is printed on purpose: an implied scale computed over a patch"
        echo "that is mostly far clip, or that spans a range of distances, is wrong"
        echo "in the direction that makes it look smaller — and it would look like a"
        echo "perfectly ordinary answer written into config/pimesh.yaml."
        echo
        echo "Re-record: square-on to a flat surface with nothing in front of it,"
        echo "1.5-2.5 m away, filling the middle of the frame."
        echo
        echo "  bash tools/calib/camera-reset.sh"
        echo "  bash tools/record-clip.sh scale1 20"
        echo "========================================================================="
        echo "FAIL gate-scale"
        exit 1
    fi
    echo "Everything the clip can be checked for is in order. Nothing is recorded in"
    echo "config/pimesh.yaml yet, though, so this is still a refusal and not a pass."
    echo "Put these two lines in place of the existing depth_scale in"
    echo "src/pimesh_bringup/config/pimesh.yaml — the comment has to be the line"
    echo "immediately above, which is what a test asserts and what lets a gate read"
    echo "a comment at all:"
    echo
    echo "    # depth_scale_reference: ${TAPE} m, bags/scale1, measured $(date +%F)"
    echo "    depth_scale: ${implied}"
    echo
    echo "Then \`bash tools/build.sh\` and run this gate again with no argument."
    echo "========================================================================="
    echo "FAIL gate-scale — nothing recorded yet"
    exit 1
fi

in_range "$disagree" 0 "$MAX_DISAGREE" ||
    note "the clip reads ${median} m against a tape figure of ${TAPE} m — ${disagree} off, ceiling ${MAX_DISAGREE}. depth_scale is ${DEPTH_SCALE} and this clip implies ${implied}"

# =============================================================================
echo
echo "=============================== gate-scale =============================="
printf '%-28s %14s\n' "tape measure (m)" "$TAPE"
printf '%-28s %14s\n' "median of /depth (m)" "$median"
printf '%-28s %14s\n' "  q1 / q3 across frames" "${q1} / ${q3}"
printf '%-28s %14s\n' "disagreement" "$disagree"
echo "---"
printf '%-28s %14s\n' "spread within a frame (m)" "$spread_space"
printf '%-28s %14s\n' "  as a fraction of median" "$spread_frac"
printf '%-28s %14s\n' "spread across frames (m)" "$spread_time"
printf '%-28s %14s\n' "clipped, typical / worst" "${clipped} / ${clipped_max}"
printf '%-28s %14s\n' "frames measured / seen" "${frames} / ${seen}"
echo "---"
echo "depth_scale    : ${DEPTH_SCALE} in config/pimesh.yaml; this clip implies ${implied}"
echo "reference      : ${RECORDED_M} m, ${RECORDED_CLIP}, measured ${RECORDED_DATE}"
echo "assert         : |median - tape| / tape <= ${MAX_DISAGREE}; clip <= ${MAX_CLIPPED} of"
echo "                 the patch; within-frame spread <= ${MAX_SPREAD} of the median;"
echo "                 >= ${MIN_FRAMES} frames measured"
echo
echo "**The spread across frames is Depth Anything's scale breathing, not noise in"
echo "the measurement.** It is the same few percent a frame that fusion_node's"
echo "aligner exists for and that hits its 15% clamp on one frame in seven of"
echo "bags/desk1. A unit derived from this clip carries that spread, which is why"
echo "the agreement budget is ${MAX_DISAGREE} and not tighter — and why the number"
echo "above is a median over frames rather than any single frame's reading."
echo "========================================================================="

if (( fail )); then echo "FAIL gate-scale"; exit 1; fi
echo "PASS gate-scale"
