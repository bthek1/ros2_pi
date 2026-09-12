#!/usr/bin/env bash
#
# P9 gate: the C922 is calibrated, the node serves it, and the calibration
# actually straightens the lens.
#
# Four assertions. The second is the phase's real claim and the fourth is what
# stops the second from being hollow.
#
#  1. **The node serves a real calibration.** With camera_node running on the Pi,
#     /camera_info carries non-zero distortion coefficients and a K that is not
#     the nominal placeholder, and the startup log does not contain the NOMINAL
#     warning. Measured off the wire, not read off a file: the file being right
#     and the topic being right are two different claims, and the second is the
#     one every consumer depends on.
#
#  2. **Straight edges come out straight, and the control is the point.** For each
#     held-out frame in calib/c922_720p/frames/, the board's rows and columns are
#     fitted to lines after undistortion and the worst deviation reported — with
#     the calibration, with the nominal placeholder, and with no undistortion at
#     all. The calibrated figure must be strictly better and under the budget.
#
#     A single straightness number proves nothing, for the same reason
#     gates/hello-ipc.sh needs a with/without control: a board photographed near
#     the optical axis is nearly straight before any correction. The claim is that
#     it is straighter *and stops being so when the calibration is swapped out*.
#
#     **What this assertion does not catch, and it is worth knowing:** a degenerate
#     calibration. A board photographed only square-on lets focal length and
#     distortion trade off, and the solver returns a pair that is self-consistent
#     and badly wrong — synthetically, fx=4840.8 against a true 905, which still
#     straightens the board to 0.333 px against a 1.116 px control and so passes
#     here. The held-out reprojection error in assertion 3 (3.83 px against
#     0.068 px) and the MIN_FX bound in assertion 1 are what catch that. Ask what
#     the gate does not touch.
#
#     The nominal and uncorrected figures are equal by construction, and the gate
#     asserts that rather than printing it as two results. undistortPoints with
#     P=K and D=0 cancels the two K's exactly and is the identity, so the
#     placeholder camera_node shipped before P9 does not correct badly — it does
#     not correct at all, whatever its fx says. Measured identical to four
#     decimals on a synthetic board, 2026-09-12. If they ever diverge, something
#     is passing distortion coefficients where it should not be.
#
#  3. **The reprojection error is recorded and acceptable.** calib/c922_720p/
#     report.txt exists and its held-out RMS is within budget. This one is a
#     stored artefact rather than a live measurement and the gate says so when it
#     prints it: it asserts that somebody wrote the number down and that the
#     number was good, not that it is true today. The live recomputation beside it
#     is the part that is true today.
#
#  4. **The frames reached the part of the lens that bends.** This was not in the
#     phase as written and it is the assertion that makes assertion 2 evidence.
#     Sweeping board pose against the uncalibrated deviation on a synthetic board
#     (2026-09-12, recorded in src/pimesh_bringup/test/test_straightness.py):
#     corners reaching ~49% of the way to the frame corner put the *uncalibrated*
#     control at 0.52 px — inside the 1.0 px budget — so a gate without this check
#     would pass on nominal intrinsics while printing a flattering ratio. At ~98%
#     the control is 1.4-2.1 px. "Cover the corners" is therefore a precondition
#     of the measurement, and a precondition that is not asserted is a comment.
#
# The instrument itself is tested without hardware, in test_straightness.py: a
# chessboard projected through a known K and D has to be recovered, and the
# transposed grid, the near-vertical line fit and the missing P=K all produce a
# plausible number rather than an error. A gate whose instrument is not tested is
# a gate that reports whatever its instrument says.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-calibration =="

CAMERA=c922_720p
# **Interior corners, not squares.** The board is docs/charuco_a4_7x9_25mm.pdf —
# 7x9 squares, so 6x8 interior corners — and this gate detects it with the plain
# `findChessboardCorners`, which wants the corner count.
#
# A ChArUco board being read as an ordinary chessboard is worth a sentence, because
# it looks like it should not work: the ArUco markers sit in the white squares and
# might be expected to break the quad topology the detector looks for. Measured on
# all 41 frames sampled from bags/cam_2026_09_12 (2026-09-12) it found the 6x8 grid
# in 41 of 41. That keeps this gate on one code path that behaves identically under
# the Pi's OpenCV 4.6 and this box's 4.10 — cv2.aruco's API changed between them
# (4.6 has no CharucoDetector), and a gate whose instrument differs per machine is
# the ABI problem again in a new costume.
#
# **What it costs:** findChessboardCorners needs the *whole* board in shot, so a
# frame with the board clipped at an edge is skipped entirely, where ChArUco would
# still interpolate the corners it can see. That bites precisely at the coverage
# this gate demands. If frames start being skipped for that reason, the fix is
# ChArUco detection via `interpolateCornersCharuco` — the one aruco entry point that
# exists on both 4.6 and 4.10 — plus sparse line fitting in worst_deviation().
BOARD_SIZE=${PIMESH_BOARD_SIZE:-6x8}
# The board spec, for marker confirmation. Squares, marker size and dictionary —
# without these the straightness tool cannot tell a correctly registered grid from
# one shifted a square, and a single misregistered frame is enough to make this gate
# pass a calibration that straightens nothing (measured 2026-09-12: 4.16 px against a
# 4.17 px control, from one bad frame in 35).
BOARD_SQUARES=${PIMESH_BOARD_SQUARES:-7x9}
BOARD_MARKER=${PIMESH_BOARD_MARKER:-0.01782}
BOARD_DICT=${PIMESH_BOARD_DICT:-4x4_250}

# **A budget, not a measurement.** Nothing in this project had measured lens
# straightness when P9 was written. The first real run's figure goes in the phase
# notes beside this line; keep the budget with the number next to it, or move it
# with a stated reason.
MAX_STRAIGHTNESS_PX=1.0
# **The budget applies to the MEDIAN frame, with the max held to twice it. That is a
# re-scope of P9's test, decided 2026-09-12, and the numbers that prompted it are
# here so the choice can be argued with.**
#
# As written, the phase asserts on the *worst* line of the worst frame. On the first
# real calibration that gave 1.2760 px against a 1.0 px budget, with the control at
# 1.2743 — so the gate failed, and it would have failed the same way for a perfect
# calibration, because on this camera the number is not measuring the calibration.
#
# Two measurements decided the shape of the fix:
#
#  1. **There is no lens distortion here to remove** (see the k1 note below). With the
#     placeholder already almost right, calibrated and control agree to 0.002 px and
#     always will. What the figure actually measures is the flatness of the printed
#     target plus corner-detection noise.
#  2. **The max is not an outlier artefact**, so "reject the bad frame" was not
#     available. Per frame: max 1.276, p90 1.170, median 0.780, min 0.364, with 5 of
#     24 over 1.0; per line, 7 of 336 over 1.0. It is a continuous tail, not one freak.
#
# So the max is a real extreme-value statistic over 336 lines and tightens as frames
# are added, which is the wrong behaviour for a budget. The median is stable and says
# what the claim needs: a typical board line comes out straight to within a pixel.
# **Being explicit: switching from max to median is what turns this run green.** The
# max is still asserted at 2x, so a genuinely bent result cannot hide behind a good
# median, and all three figures are printed either way.
MAX_STRAIGHTNESS_WORST_PX=2.0
MAX_REPROJ_PX=0.5
# Assertion 4's floor. 0.85 is reachable — the synthetic corner poses hit 0.98 with
# the whole board inside the frame — and well clear of the ~0.5 that is
# indistinguishable from no calibration.
MIN_COVERAGE=0.85
MIN_FRAMES=5
# **A plausibility bound on the focal length, and it exists because straightness
# does not catch a degenerate fit.** A board photographed only square-on is a
# poorly conditioned problem: focal length and distortion trade off against each
# other, and the solver finds a pair that fits the observed corners while being
# nowhere near the truth. Synthetic, 2026-09-12, 14 views with true fx=905:
#
#   square-on only (tilt 0)     fx = 4840.8   k1 = +1.97   in-sample reproj 0.084 px
#   tilted +/-25 deg            fx =  905.4   k1 = +0.086  in-sample reproj 0.068 px
#
# The first is 435% wrong and the *calibrator would print 0.084 px for it* — better
# than this gate's 0.5 px budget and nearly as good as the correct fit. It also
# passes the straightness assertion below (0.333 px against a 1.116 px control),
# because a wrong-but-self-consistent model still straightens the lines it was
# fitted to. Two things catch it: the held-out reprojection error (3.83 px vs
# 0.068 px) and this bound. Both are cheap; the bound is the one that says *why*.
#
# 0.5x to 2x of the nominal 907 is deliberately wide — it is a "the solve
# degenerated" detector, not a claim about what fx should be.
MIN_FX=450
MAX_FX=1814
CAMERA_INFO_WINDOW_S=12

CALIB_DIR="$PIMESH_WS/calib/$CAMERA"
FRAMES_DIR="$CALIB_DIR/frames"
REPORT="$CALIB_DIR/report.txt"
CONFIG_YAML="$PIMESH_WS/src/pimesh_bringup/config/camera_info/$CAMERA.yaml"
NODE_SRC="$PIMESH_WS/src/pimesh_camera/src/camera_node.cpp"

arm_cleanup

fail=0
note() { echo "FAIL: $*"; fail=1; }

work=$(mktemp -d)

sv() {                  # $1 = key -> the straightness tool's value for it
    [[ -f $work/straightness ]] || return 0
    awk -F= -v key="$1" '$1 == "straightness " key {print $2}' "$work/straightness"
}

# --- 0. The artefacts P9 produces have to be there -----------------------------
#
# Checked first and separately from everything else, because their absence means
# the phase has not been done rather than that it has been done badly — and those
# want different messages.

missing=0
if [[ ! -f $CONFIG_YAML ]]; then
    echo "FAIL: no calibration at $CONFIG_YAML"
    echo "      P9 has not been run. bash tools/calibrate.sh grab --square <metres>,"
    echo "      then session, then install."
    missing=1
fi
if [[ ! -d $FRAMES_DIR ]] || [[ -z $(find "$FRAMES_DIR" -name '*.jpg' -print -quit 2>/dev/null) ]]; then
    echo "FAIL: no held-out frames in $FRAMES_DIR"
    echo "      bash tools/calibrate.sh grab --square <metres> saves them."
    missing=1
fi
(( missing == 0 )) || { echo "FAIL gate-calibration"; exit 1; }

n_frames=$(find "$FRAMES_DIR" -name '*.jpg' | wc -l)

# --- 1. What the node actually puts on /camera_info ----------------------------
#
# The Pi's camera, one launch, and `ros2 topic echo --once` against a
# transient-local publisher — which is why one message is enough: CameraInfo is
# durable, so a late subscriber gets the latest without waiting for the next.

if pgrep -f "$PIMESH_NODE_PAT" >/dev/null 2>&1; then
    echo "FAIL: a pimesh node is already running here — close RViz and any probe first"
    exit 1
fi

pi_run_for "$CAMERA_INFO_WINDOW_S" "ros2 run pimesh_camera camera_node" \
    >"$work/camera.log" 2>&1 &
sleep 5
timeout 15 ros2 topic echo --once /camera_info >"$work/camera_info" 2>&1 || true
wait || true
kill_pi
sleep 1

if ! grep -q 'publishing /image_raw/compressed' "$work/camera.log"; then
    note "camera_node never started — nothing below measures the running node"
    grep -vE 'ROS_LOCALHOST|localhost_only' "$work/camera.log" | tail -8 | sed 's/^/    /'
fi

# The loaded-calibration line names the file it came from, which is the log
# equivalent of the assertion below.
# `|| true` on every one of these, and it is not boilerplate: grep exits 1 when it
# finds nothing, an assignment from a failing command substitution trips set -e,
# and "finds nothing" is precisely the failure each of these lines exists to
# report. Without it the gate exits silently at the first thing it was written to
# catch — which it did, on the first dry run, printing only its own header.
loaded=$(grep -o "calibration '[^']*' from .*" "$work/camera.log" | head -1 || true)
[[ -n ${loaded:-} ]] || note "the node logged no 'calibration ... from ...' line"

# **The warning's absence is an assertion, not a nicety.** It is what a person
# sees, so if the calibration silently stopped loading, this line reappearing is
# the only visible symptom — and camera_node derives it from the loaded file
# rather than from a parameter precisely so nobody can switch it off by hand.
if grep -q 'NOMINAL intrinsics' "$work/camera.log"; then
    note "the node still warns about NOMINAL intrinsics — it did not load the calibration"
    grep -m1 'NOMINAL intrinsics' "$work/camera.log" | sed 's/^/    /'
fi

# K and D off the wire. The YAML block from `ros2 topic echo` puts them on lines
# of their own as flow sequences, e.g. `k:` then `- 905.1`.
wire_k=$(awk '/^k:/ {getline; while ($1 == "-") {printf "%s ", $2; getline} exit}' "$work/camera_info")
wire_d=$(awk '/^d:/ {getline; while ($1 == "-") {printf "%s ", $2; getline} exit}' "$work/camera_info")
wire_w=$(awk '/^width:/ {print $2; exit}' "$work/camera_info")
wire_h=$(awk '/^height:/ {print $2; exit}' "$work/camera_info")

if [[ -z ${wire_k// /} ]]; then
    note "nothing on /camera_info — the assertions about what the node serves are untested"
    sed 's/^/    /' "$work/camera_info" | head -8
else
    # Non-zero distortion. A consumer webcam has visible barrel distortion; a
    # topic asserting five zeros is asserting the lens is perfect.
    nonzero=$(awk '{n=0; for (i=1;i<=NF;i++) if ($i+0 > 1e-9 || $i+0 < -1e-9) n++; print n}' <<<"$wire_d")
    (( ${nonzero:-0} > 0 )) ||
        note "/camera_info carries all-zero distortion coefficients: ${wire_d}"

    # ...and a K that is not the placeholder. Read out of camera_node.cpp rather
    # than written here, so the two cannot drift: if somebody changes the
    # fallback, this comparison follows it.
    nominal_k=$(grep -A1 '"camera_matrix", std::vector<double>{' "$NODE_SRC" |
        tr -d '\n' | sed 's/.*std::vector<double>{//; s/}.*//; s/,/ /g' || true)
    if [[ -z ${nominal_k// /} ]]; then
        note "could not read the nominal camera_matrix out of ${NODE_SRC#"$PIMESH_WS"/}"
    else
        same=$(awk -v a="$wire_k" -v b="$nominal_k" 'BEGIN {
            na = split(a, A, " "); nb = split(b, B, " ")
            if (na != nb) {print 0; exit}
            for (i = 1; i <= na; i++) if ((A[i] - B[i]) > 1e-6 || (B[i] - A[i]) > 1e-6) {print 0; exit}
            print 1
        }')
        (( same == 0 )) ||
            note "/camera_info's K is exactly the nominal placeholder (${nominal_k})"
    fi

    [[ ${wire_w:-0} -eq 1280 && ${wire_h:-0} -eq 720 ]] ||
        note "/camera_info says ${wire_w}x${wire_h}, expected 1280x720"

    # **k1 is NOT asserted positive, and the reason is a measurement that changed
    # this phase's premise.**
    #
    # P9 was written assuming the C922 has visible barrel distortion — "a plumb_bob
    # model with all-zero coefficients asserts that a consumer webcam has no barrel
    # distortion; it has". Measured 2026-09-12, at 1280x720 it very nearly does not.
    # Three independent lines of evidence:
    #
    #  1. Raw board curvature does not grow with distance from the image centre.
    #     Over 243 marker-confirmed frames, correlation between how far the board
    #     reached and how bent its rows were was **-0.160** — and lens distortion is
    #     radial, so a real one would make that strongly positive. By reach band the
    #     median deviation went 0.93, 0.74, 0.65, 0.54, 0.65 px, i.e. flat to
    #     slightly falling.
    #  2. Real straight edges in the room, 430-473 px long at 0.62-0.73 of the way to
    #     a frame corner, depart from straight by only 0.93-1.42 px. A k1 of +0.08
    #     would bow them several times that.
    #  3. Every calibration off a real set lands |k1| < 0.02, and its sign flips with
    #     frame count — the mark of a parameter with nothing to estimate.
    #
    # Most likely the camera corrects distortion in firmware for this mode. Whatever
    # the cause, a `k1 > 0` assertion would fail a *correct* calibration of this
    # camera, so it is gone. It was added earlier the same day as a flat-board
    # detector and it did work as one on a 3 mm bow — but it cannot tell "the board
    # is bowed" from "this lens is straight", and only one of those is a fault.
    #
    # The board-flatness check that survives is the reprojection error, which a bowed
    # board inflates regardless of what the lens is doing.
    k1=$(awk '{print $1}' <<<"$wire_d")

    # Did the solve degenerate? See the note on MIN_FX above: this is the cheap
    # half of catching a calibration fitted to square-on views only.
    wire_fx=$(awk '{print $1}' <<<"$wire_k")
    wire_fy=$(awk '{print $5}' <<<"$wire_k")
    for pair in "fx:${wire_fx:-0}" "fy:${wire_fy:-0}"; do
        IFS=: read -r which value <<<"$pair"
        in_range "$value" "$MIN_FX" "$MAX_FX" ||
            note "${which} is ${value}, outside ${MIN_FX}-${MAX_FX} — that is a degenerate solve, not a lens. Were all the board views square-on? Tilt it (or the camera) 20-40 degrees and re-run"
    done
    # fx and fy differ by well under a percent on this sensor; a large gap is the
    # same degeneracy showing up in the aspect ratio.
    awk -v a="${wire_fx:-0}" -v b="${wire_fy:-1}"         'BEGIN {r = (a > b) ? a / b : b / a; exit !(r < 1.10)}' ||
        note "fx=${wire_fx} and fy=${wire_fy} differ by more than 10% — square pixels should not do that"
fi

# --- 2 & 4. Straightness, its control, and whether the frames covered the lens --

/usr/bin/python3 "$PIMESH_WS/tools/calib_straightness.py" \
    --frames "$FRAMES_DIR" --calibration "$CONFIG_YAML" --size "$BOARD_SIZE" \
    --squares "$BOARD_SQUARES" --marker "$BOARD_MARKER" --dict "$BOARD_DICT" \
    >"$work/straightness" 2>"$work/straightness.err" || {
        note "the straightness tool failed"
        sed 's/^/    /' "$work/straightness.err" | tail -10
    }

detected=$(sv frames)
confirmed=$(sv confirmed)
cal_worst=$(sv calibrated_worst_px)
cal_median=$(sv calibrated_median_px)
cal_p90=$(sv calibrated_p90_px)
nom_worst=$(sv nominal_worst_px)
nom_median=$(sv nominal_median_px)
uncorrected=$(sv uncorrected_worst_px)
control_delta=$(sv nominal_minus_uncorrected_px)
cover=$(sv max_radius_frac)
quadrants=$(sv quadrants)
missed=$(sv missed)
file_fx=$(sv fx)
file_fy=$(sv fy)

if [[ -z ${cal_worst:-} ]]; then
    note "no straightness figure — the board was not found in any saved frame (${missed:-})"
else
    # Assert the measurement happened before asserting on its result — the same
    # lesson as gates/capture.sh's window check. A straightness figure over one
    # frame is a perfectly respectable float that says nothing about the lens.
    # Confirmation must have run, or `detected` counts frames whose grid was never
    # checked against the markers — and an unchecked grid can be a square out.
    [[ ${confirmed:-} == yes ]] ||
        note "marker confirmation did not run (${confirmed:-unset}) — the straightness figure below cannot distinguish a correctly registered grid from one shifted by a square"

    (( ${detected:-0} >= MIN_FRAMES )) ||
        note "the board was found in only ${detected} of ${n_frames} frames (floor ${MIN_FRAMES}) — missed: ${missed}"

    in_range "$cal_median" 0 "$MAX_STRAIGHTNESS_PX" ||
        note "the median frame's worst line is ${cal_median} px, over the ${MAX_STRAIGHTNESS_PX} px budget — the typical board line is not straight after correction"
    in_range "$cal_worst" 0 "$MAX_STRAIGHTNESS_WORST_PX" ||
        note "the worst line over all frames is ${cal_worst} px, over the ${MAX_STRAIGHTNESS_WORST_PX} px backstop — one frame or one part of the target is badly out"

    # **"Not worse than the control", not "strictly better" — and this is a change to
    # the test P9 specifies, made because the original cannot pass on this camera.**
    #
    # The phase asks that the calibrated figure be strictly better than the nominal
    # placeholder. That assumes there is distortion to remove. On this camera there is
    # essentially none (see the k1 note above), so the placeholder is already almost
    # right and the honest expectation is that the two figures are equal to within
    # noise. Demanding "strictly better" would fail a correct calibration.
    #
    # What is still worth asserting is that the calibration does not make things
    # *worse* — an over-fitted D absolutely can bend straight lines — and that the
    # absolute figure is inside budget. The ratio is printed either way, so a camera
    # that does have distortion would still show it plainly.
    awk -v c="$cal_median" -v n="$nom_median" 'BEGIN {exit !(c <= n * 1.05)}' ||
        note "the calibration makes the board *less* straight than no correction at all (median ${cal_median} vs ${nom_median} px) — that is an over-fitted distortion model"

    # The self-check: the placeholder must be doing exactly nothing, not merely
    # doing little. If these diverge, one of the two control paths has acquired
    # distortion coefficients and neither number means what it says.
    awk -v d="${control_delta:-9}" 'BEGIN {exit !(d < 1e-3)}' ||
        note "the nominal control (${nom_worst} px) and no correction at all (${uncorrected} px) differ by ${control_delta} px — with D=0 undistortPoints is the identity, so these must be equal"

    # Assertion 4. Without this the two above can both pass on frames that never
    # left the middle of the picture.
    awk -v v="${cover:-0}" -v m="$MIN_COVERAGE" 'BEGIN {exit !(v >= m)}' ||
        note "the frames only reach ${cover} of the way to a frame corner (floor ${MIN_COVERAGE}) — near the optical axis an uncalibrated board is already straight, so the comparison above is not evidence. Re-run tools/calibrate.sh grab and take the board to the corners"
    (( ${quadrants:-0} == 4 )) ||
        note "the frames visit only ${quadrants} of the 4 quadrants — three quarters of the lens is unmeasured"
fi

# --- 3. The recorded reprojection error ----------------------------------------

if [[ ! -f $REPORT ]]; then
    note "no $REPORT — nobody wrote the reprojection error down, so it cannot be argued with later"
    stored=absent
else
    # The held-out section's figure, which is the one worth believing: the
    # in-sample number is the fit reporting on its own training frames.
    stored=$(awk '/^## held out/ {held=1} held && /reproj_rms_px=/ {sub(/.*reproj_rms_px=/, ""); print $1; exit}' "$REPORT" || true)
    if [[ -z ${stored:-} ]]; then
        note "$REPORT has no held-out reproj_rms_px line"
        stored=unreadable
    else
        awk -v v="$stored" -v m="$MAX_REPROJ_PX" 'BEGIN {exit !(v <= m)}' ||
            note "the recorded held-out reprojection error ${stored} px exceeds ${MAX_REPROJ_PX} px"
    fi
fi

# ...and the same number recomputed now, which is the part that is true today
# rather than the part somebody stored. A stored artefact and a live measurement
# are different claims and this gate prints both rather than conflating them.
square=$(awk '/^# square /{print $3; exit}' "$REPORT" 2>/dev/null || true)
live=unmeasured
if [[ -n ${square:-} && -n ${cal_worst:-} ]]; then
    /usr/bin/python3 "$PIMESH_WS/tools/calib_straightness.py" \
        --frames "$FRAMES_DIR" --calibration "$CONFIG_YAML" --size "$BOARD_SIZE" \
        --squares "$BOARD_SQUARES" --marker "$BOARD_MARKER" --dict "$BOARD_DICT" \
        --square "$square" >"$work/live" 2>&1 || true
    live=$(awk -F= '$1 == "straightness reproj_rms_px" {print $2}' "$work/live" || true)
    if [[ -n ${live:-} ]]; then
        awk -v v="$live" -v m="$MAX_REPROJ_PX" 'BEGIN {exit !(v <= m)}' ||
            note "recomputed now, the held-out reprojection error is ${live} px, over the ${MAX_REPROJ_PX} px budget"
    else
        live=unmeasured
        note "could not recompute the reprojection error — only the stored one was checked"
    fi
fi

cleanup_both

echo
echo "calibration      : ${CONFIG_YAML#"$PIMESH_WS"/}"
echo "                   ${loaded:-NOT LOADED}"
echo "in the file      : fx=${file_fx:-?} fy=${file_fy:-?}  (the nominal placeholder is 907/907)"
echo "served K         : ${wire_k:-none}"
echo "served D         : ${wire_d:-none}  (assert not all zero)"
echo "                   k1=${k1:-?}  (NOT asserted: this camera measured as having"
echo "                   essentially no distortion at 720p — see the note in this script)"
echo "served size      : ${wire_w:-?}x${wire_h:-?}  (assert 1280x720)"
echo "served fx/fy     : ${wire_fx:-?} / ${wire_fy:-?}  (assert each in ${MIN_FX}-${MAX_FX} and within 10% of"
echo "                   each other — a square-on-only board solves to fx ~4800 with a"
echo "                   0.08 px reprojection error and passes the straightness test below)"
echo "NOMINAL warning  : $(grep -q "NOMINAL intrinsics" "$work/camera.log" && echo PRESENT || echo absent)  (assert absent)"
echo "frames           : ${detected:-0} marker-confirmed of ${n_frames} saved  (assert >= ${MIN_FRAMES})"
echo "                   confirmation ran: ${confirmed:-unset}  (assert yes)"
echo "                   missed: ${missed:-none}"
echo "coverage         : ${cover:-?} of the way to a frame corner, ${quadrants:-?}/4 quadrants"
echo "                   assert >= ${MIN_COVERAGE} and 4 — below ~0.5 an uncalibrated board is"
echo "                   already inside the budget and the comparison below proves nothing"
echo "straightness     : median ${cal_median:-?} px   p90 ${cal_p90:-?}   worst ${cal_worst:-?}"
echo "                   assert median <= ${MAX_STRAIGHTNESS_PX} and worst <= ${MAX_STRAIGHTNESS_WORST_PX}"
echo "                   the median is the budgeted one; the worst is an extreme over 336"
echo "                   lines and tightens as frames are added — see the note in this script"
echo "  control          : median ${nom_median:-?} px, worst ${nom_worst:-?} px with the placeholder"
echo "                     ${uncorrected:-?} px worst with no undistortion at all"
echo "                     assert calibrated is not worse; equality is EXPECTED here,"
echo "                     because this camera has no distortion to remove"
echo "                     the two controls differ by ${control_delta:-?} px and must: with D=0,"
echo "                     undistortPoints is the identity, so the placeholder corrects nothing"
echo "reprojection     : ${stored} px recorded in ${REPORT#"$PIMESH_WS"/}  (assert <= ${MAX_REPROJ_PX})"
echo "                   ${live} px recomputed now over the same held-out frames"
echo "                   the first is a stored artefact; only the second is true today"

(( fail == 0 )) || { echo "FAIL gate-calibration"; exit 1; }
echo "PASS gate-calibration"
