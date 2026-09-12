#!/usr/bin/env bash
#
# P9: calibrate the C922, and turn the result into files the gate can check.
#
# Three modes, run in this order, and the middle one is the physical session:
#
#   bash tools/calibrate.sh grab    --square 0.0245   held-out frames for the gate
#   bash tools/calibrate.sh session --square 0.0245   the checkerboard itself
#   bash tools/calibrate.sh install --square 0.0245   the result into the workspace
#
# **The board must be rigid, and the square must be measured with a ruler.** A
# printout that flexes converges happily and is wrong, and printer scaling lies
# about the size it was asked for — `--square` sets the scale of the entire model,
# so a 2% error there is a 2% error in every distance the pipeline ever reports.
# Nothing downstream can detect that; it is not a number this script can check.
#
# `grab` runs *before* `session` on purpose. The frames the gate measures must be
# ones the calibration never saw: cameracalibrator keeps its own frames inside
# /tmp/calibrationdata.tar.gz, and checking a fit on its own training data is how
# you get a number that is good and means nothing.
#
# Everything here bounds what it starts and cleans up both machines on the way
# out, including the camera on the Pi — a leaked camera_node holds /dev/video0
# exclusively and every later session dies with "Device or resource busy".

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

CAMERA=c922_720p
CALIB_DIR="$PIMESH_WS/calib/$CAMERA"
FRAMES_DIR="$CALIB_DIR/frames"
CONFIG_YAML="$PIMESH_WS/src/pimesh_bringup/config/camera_info/$CAMERA.yaml"
RAW_TOPIC=/image_raw_uncompressed
TARBALL=/tmp/calibrationdata.tar.gz

mode=${1:-session}
shift || true

# The board, as **squares**, which is the number printed on the sheet and the only
# one a person should have to type. Every other count is derived below, because
# the two conventions in play are a trap:
#
#   cameracalibrator -p charuco --size N   is SQUARES  (it goes to cv2.aruco.CharucoBoard)
#   cv2.findChessboardCorners(size)        is INTERIOR CORNERS (squares - 1)
#
# **The command printed on our own sheet gets this wrong.** docs/charuco_a4_7x9_25mm.pdf
# says `--size 6x8`, the interior-corner count, and measured against a real frame
# from bags/cam_2026_09_12 on 2026-09-12 that interpolates **0** chessboard corners
# while `--size 7x9` interpolates 42. It is inherited from piros2's
# tools/calib/make_calib_target.py. Do not copy the line off the sheet.
squares=7x9
square=
marker=
pattern=charuco
dict=4x4_250
frames=8
seconds=900
# cameracalibrator's motion rejection, which defaults to -1.0 — i.e. **off**, so a
# blurred frame from a board or camera in motion is accepted like any other, and
# sub-pixel corners off a blurred board are the one input the whole calibration is
# made of. A positive value is the maximum average corner motion in pixels between
# consecutive frames that it will still accept. Left off by default to match the
# tool rather than to endorse it; pass --speed 6 if the session is picking up
# smeared frames, and hold each pose still for a beat regardless.
speed=
while (( $# )); do
    case $1 in
        --square)  square=$2; shift 2 ;;
        --squares) squares=$2; shift 2 ;;
        --marker)  marker=$2; shift 2 ;;
        --pattern) pattern=$2; shift 2 ;;
        --dict)    dict=$2; shift 2 ;;
        --frames)  frames=$2; shift 2 ;;
        --seconds) seconds=$2; shift 2 ;;
        --speed)   speed=$2; shift 2 ;;
        --tarball) TARBALL=$2; shift 2 ;;
        *) echo "calibrate: unknown argument $1" >&2; exit 2 ;;
    esac
done

if [[ -z $square ]]; then
    cat >&2 <<'USAGE'
calibrate: --square is required, in metres, and measured with a ruler.

    bash tools/calibrate.sh grab    --square 0.025 --squares 7x9 --marker 0.018
    bash tools/calibrate.sh session --square 0.025 --squares 7x9 --marker 0.018
    bash tools/calibrate.sh install --square 0.025 --squares 7x9 --marker 0.018

--squares is the number of SQUARES (7x9 for docs/charuco_a4_7x9_25mm.pdf), not the
interior-corner count the sheet's own printed command line shows. --marker is the
ArUco marker size in metres, needed for -p charuco (18 mm on that sheet).

Our A4 sheet carries a 100 mm bar: check it with a ruler first. If it is not
exactly 100 mm the print was scaled, and then measure across several squares and
divide rather than trusting either the filename or the bar.
USAGE
    exit 2
fi

# Squares in, interior corners out. One input, both conventions derived, so the
# two can never be typed inconsistently.
sq_x=${squares%%x*}
sq_y=${squares##*x}
[[ $sq_x =~ ^[0-9]+$ && $sq_y =~ ^[0-9]+$ ]] ||
    { echo "calibrate: --squares must look like 7x9, got '$squares'" >&2; exit 2; }
corners="$(( sq_x - 1 ))x$(( sq_y - 1 ))"

if [[ $pattern == charuco && -z $marker ]]; then
    echo "calibrate: -p charuco needs --marker, the ArUco marker size in metres" >&2
    echo "           (0.018 for docs/charuco_a4_7x9_25mm.pdf)" >&2
    exit 2
fi
if [[ -n $marker ]]; then
    awk -v m="$marker" -v s="$square" 'BEGIN {exit !(m > 0 && m < s)}' ||
        { echo "calibrate: --marker $marker must be >0 and smaller than --square $square" >&2; exit 2; }
fi

# 24.5 mm is a typical A4 OpenCV chessboard square. Anything outside a few
# millimetres to a few centimetres is a unit mix-up, and a metres/millimetres
# slip is the one error here that produces a perfectly self-consistent
# calibration at the wrong scale.
awk -v s="$square" 'BEGIN {exit !(s > 0.005 && s < 0.15)}' || {
    echo "calibrate: --square $square is not a plausible size in METRES" >&2; exit 2
}

arm_cleanup

work=$(mktemp -d)

# The Pi's camera, bounded, with the timeout inside the login shell so the signal
# reaches the node rather than the shell wrapping it. Backgrounded and never
# waited on by job number: killing a background `bash -lc` wrapper orphans its
# grandchildren, so the camera is always ended by pattern (kill_pi) instead.
start_camera() {        # $1 = seconds
    pi_run_for "$1" "ros2 run pimesh_camera camera_node" >"$work/camera.log" 2>&1 &
    local waited=0
    while (( waited < 20 )); do
        grep -q 'publishing /image_raw/compressed' "$work/camera.log" 2>/dev/null && return 0
        sleep 1; waited=$(( waited + 1 ))
    done
    echo "calibrate: the Pi's camera did not start" >&2
    tail -15 "$work/camera.log" | sed 's/^/    /' >&2
    return 1
}

case $mode in

grab)
    # Held-out frames for the gate. No republish needed: the grabber decodes the
    # camera's JPEG itself and writes those same bytes back out, so nothing
    # re-encodes the image between the sensor and the gate's sub-pixel corner fit.
    echo "== calibrate grab =="
    bash "$PIMESH_WS/tools/camera-reset.sh" >"$work/reset" 2>&1 ||
        { echo "calibrate: camera-reset did not reach its baseline" >&2
          sed 's/^/  /' "$work/reset" >&2; exit 1; }

    rm -rf "$FRAMES_DIR"; mkdir -p "$FRAMES_DIR"
    start_camera "$seconds" || exit 1

    cat <<'EOF'

It saves a frame only when it can see the whole board, and it keeps asking until
the frames between them reach all four corners of the picture — that is what makes
the gate's measurement mean anything.

Two things to vary, and the second is the one people skip:

  * WHERE the board sits in the frame. Work it out to the corners and edges, not
    just the middle. Near the optical axis there is no distortion to fit.
  * The ANGLE between the board and the camera. Tilt 20-40 degrees, in both axes,
    several different ways.

If the board is fixed to a wall, that second one means moving the camera off to
the side and pointing back at the board — not just sliding it around parallel to
the wall. Square-on views alone are a poorly conditioned solve: measured on a
synthetic board, 14 square-on views gave fx=4841 against a true 905, with a
reprojection error of 0.08 px that looks excellent. A rigid board does not protect
you from this one.

Ctrl-C when it says it is done.

EOF
    run_for "$seconds" /usr/bin/python3 "$PIMESH_WS/tools/calib_grab.py" \
        --out "$FRAMES_DIR" --size "$corners" --count "$frames" || true

    kill_pi
    echo
    echo "frames           : $(find "$FRAMES_DIR" -name '*.jpg' | wc -l) in $FRAMES_DIR"
    ;;

session)
    # The calibration itself.
    echo "== calibrate session =="
    if [[ ! -d $FRAMES_DIR ]] || [[ -z $(find "$FRAMES_DIR" -name '*.jpg' -print -quit) ]]; then
        echo "calibrate: no held-out frames in $FRAMES_DIR — run 'grab' first, or the" >&2
        echo "           gate will have nothing to check this calibration against." >&2
        exit 2
    fi

    bash "$PIMESH_WS/tools/camera-reset.sh" >"$work/reset" 2>&1 ||
        { echo "calibrate: camera-reset did not reach its baseline" >&2
          sed 's/^/  /' "$work/reset" >&2; exit 1; }

    start_camera "$seconds" || exit 1

    # cameracalibrator subscribes to sensor_msgs/Image on `image`, and this project
    # only ever puts compressed frames on the wire. republish is the adapter, and
    # it runs *here* rather than on the Pi — decoding on the sensor head would put
    # a decode in the one place with no GPU and then send 2.7 MB frames over
    # Wi-Fi, which is the thing the whole architecture exists to avoid.
    ros2 run image_transport republish compressed raw \
        --ros-args -r in/compressed:=/image_raw/compressed -r out:="$RAW_TOPIC" \
        >"$work/republish.log" 2>&1 &
    sleep 3

    cat <<EOF

Cover the frame: corners and edges, not just the middle. Distortion is a radial
model and the corners are the only place it has anything to fit — frames taken
near the optical axis are already straight, and a calibration fitted only to
those cannot be told from no calibration at all.

**Tilt as well as slide, and Skew is the bar that measures it.** If the board is on
a wall, tilting means moving the camera off-axis and pointing back at it. Sliding
the camera around parallel to the wall leaves every view square-on, which is a
poorly conditioned solve: 14 square-on views of a synthetic board gave fx=4841
against a true 905, k1=+1.97 against a true +0.085, and a reprojection error of
0.08 px that looks like an excellent calibration. Being rigid does not save it.

When X, Y, Size and Skew are all green, press CALIBRATE, then SAVE. SAVE writes
$TARBALL; then run:

    bash tools/calibrate.sh install --square $square

--no-service-check is passed because camera_node deliberately offers no
set_camera_info service, and without the flag the tool refuses to start.

EOF

    # Foreground, under run_for, so Ctrl-C reaches it: a plain `timeout` would put
    # it in a process group the terminal never signals, and this is a GUI somebody
    # sits in front of for several minutes.
    speed_args=()
    [[ -n $speed ]] && speed_args=(--max-chessboard-speed "$speed")

    # SQUARES here, not corners — see the note on `squares` at the top. charuco
    # additionally needs the marker size and the dictionary; the dictionary name is
    # camera_calibration's own spelling ("4x4_250"), not OpenCV's DICT_4X4_250.
    pattern_args=(-p "$pattern" --size "$squares" --square "$square")
    [[ $pattern == charuco ]] &&
        pattern_args+=(--charuco_marker_size "$marker" --aruco_dict "$dict")

    run_for "$seconds" ros2 run camera_calibration cameracalibrator \
        "${pattern_args[@]}" \
        --camera_name "$CAMERA" --no-service-check "${speed_args[@]}" \
        --ros-args -r image:="$RAW_TOPIC" || true

    cleanup_both
    echo
    if [[ -f $TARBALL ]]; then
        echo "tarball          : $TARBALL ($(stat -c %s "$TARBALL") bytes)"
        echo "next             : bash tools/calibrate.sh install --square $square"
    else
        echo "no $TARBALL — SAVE was not pressed, so nothing was written"
        exit 1
    fi
    ;;

install)
    # The result into the workspace, plus a report.txt whose number is recomputed
    # rather than transcribed.
    echo "== calibrate install =="
    [[ -f $TARBALL ]] || { echo "calibrate: no $TARBALL — run 'session' first" >&2; exit 2; }

    tar -xzf "$TARBALL" -C "$work"
    ost=$(find "$work" -name 'ost.yaml' | head -1)
    [[ -n $ost ]] || { echo "calibrate: no ost.yaml inside $TARBALL" >&2; exit 1; }

    mkdir -p "$(dirname "$CONFIG_YAML")" "$CALIB_DIR"

    # Keep the tarball. It holds the ~40 still frames the fit was actually made
    # from, and `ros2 run camera_calibration tarfile_calibration` re-derives a
    # calibration from them offline — so the whole solve is reproducible from
    # stored images rather than only attested by a number in report.txt. That also
    # means a different model (more radial terms, a fixed principal point) can be
    # tried without another physical session. Git-ignored: ~40 PNGs at 720p is tens
    # of megabytes, while the held-out JPEGs and the YAML are small and committed.
    cp "$TARBALL" "$CALIB_DIR/calibrationdata.tar.gz"
    # Copied verbatim, never transcribed. Fourteen numbers retyped by hand is
    # fourteen chances to be silently wrong, and camera_node reads this exact
    # format on purpose so that no conversion step exists to get wrong.
    cp "$ost" "$CONFIG_YAML"
    echo "calibration      : $CONFIG_YAML"

    # The reprojection error, over the frames the calibrator itself used (in
    # sample) and over the held-out ones (out of sample). The calibrator prints a
    # number of its own on the console; this recomputes both from the stored
    # calibration, so report.txt holds figures somebody can check rather than a
    # figure somebody copied.
    in_sample_dir=$(dirname "$(find "$work" -name '*.png' | head -1)")
    report="$CALIB_DIR/report.txt"
    {
        echo "# $CAMERA — reprojection error, recomputed from $CONFIG_YAML"
        echo "# written by tools/calibrate.sh install on $(date -Is)"
        echo "# square $square m, board $squares squares = $corners interior corners"
        echo "#"
        echo "# Re-derive this calibration from the stored images with:"
        echo "#   ros2 run camera_calibration tarfile_calibration \\"
        echo "#       $CALIB_DIR/calibrationdata.tar.gz --mono -s $squares -q $square"
        echo "#"
        echo "# Recomputed rather than copied from cameracalibrator's console: a"
        echo "# transcribed number cannot be checked later. solvePnP places the board"
        echo "# where this calibration says it must be, the corners are projected back"
        echo "# through the model, and the RMS residual is the error."
        echo
        if [[ -n ${in_sample_dir:-} && -d ${in_sample_dir:-} ]]; then
            echo "## in sample — the calibrator's own frames"
            /usr/bin/python3 "$PIMESH_WS/tools/calib_straightness.py" \
                --frames "$in_sample_dir" --calibration "$CONFIG_YAML" \
                --size "$corners" --square "$square" 2>&1 | sed 's/^straightness /  /'
            echo
        fi
        echo "## held out — tools/calibrate.sh grab's frames, which the fit never saw"
        /usr/bin/python3 "$PIMESH_WS/tools/calib_straightness.py" \
            --frames "$FRAMES_DIR" --calibration "$CONFIG_YAML" \
            --size "$corners" --square "$square" 2>&1 | sed 's/^straightness /  /'
    } >"$report"

    echo "report           : $report"
    grep -E 'reproj_rms_px|max_radius_frac|calibrated_worst_px|nominal_worst_px' "$report" |
        sed 's/^/  /'
    echo
    echo "next             : bash tools/build.sh --packages-select pimesh_bringup"
    echo "                   bash tools/sync-pi.sh && bash tools/build-pi.sh"
    echo "                   bash tools/gates/calibration.sh"
    ;;

*)
    echo "calibrate: unknown mode '$mode' — grab, session or install" >&2
    exit 2
    ;;
esac
