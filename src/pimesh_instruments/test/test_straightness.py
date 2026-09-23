"""The calibration gate's instrument, checked against a known distortion.

`tools/gates/calibration.sh` asserts that undistorting a chessboard makes its
rows and columns straighter, and prints a number in pixels. That number is only
worth something if the thing computing it is right — and the failure mode is
quiet: a transposed grid fits lines across the board instead of along it, a
y-on-x line fit degenerates on the near-vertical columns, and a forgotten `P=K`
scales every answer by 1/900. All three produce a plausible float and a passing
gate.

So the instrument is checked here, with no camera, no board and no saved frames:
a synthetic planar grid is projected through a *known* K and D with
`cv2.projectPoints`, and the measurement is required to recover it. This is the
one part of P9 that could be finished before the checkerboard was printed, and it
is the part that decides whether the physical session's number means anything.

It lives in pimesh_bringup because that is where this workspace's Python tests
live and because the code under test is a dev-box analysis tool, not a node. It
imports tools/calib_straightness.py by path — the module is in tools/ because
that is where this project's shell and one-off tools live, and it is rsynced to
the Pi, so this test runs at both ends the way gates/test.sh requires.
"""

import importlib.util
import pathlib

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


def _load_module():
    """Import tools/calib_straightness.py by path.

    Four parents up from src/pimesh_bringup/test/test_straightness.py is the
    workspace root. Resolved from __file__ rather than from the cwd because
    `colcon test` runs tests from the build tree.
    """
    root = pathlib.Path(__file__).resolve().parents[3]
    path = root / 'tools' / 'calib_straightness.py'
    assert path.is_file(), f'{path} is missing — the gate has no instrument'
    spec = importlib.util.spec_from_file_location('calib_straightness', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


straightness = _load_module()

# A plausible C922-ish lens: barrel distortion (k1 > 0) of the size a consumer
# webcam actually has. The exact values do not matter — what matters is that the
# measurement recovers *these* and not zeros.
TRUE_K = np.array([[905.0, 0.0, 641.0], [0.0, 904.0, 359.0], [0.0, 0.0, 1.0]])
TRUE_D = np.array([0.085, -0.170, 0.0008, -0.0005, 0.045])

COLS, ROWS = 9, 6
SQUARE_M = 0.025


def board_object_points():
    """A 9x6 grid of interior corners on the z=0 plane, row-major like OpenCV's."""
    pts = np.zeros((ROWS * COLS, 3), dtype=np.float64)
    pts[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE_M
    return pts


def project(rvec, tvec, k=TRUE_K, d=TRUE_D):
    """Project the board and return it as the (rows, cols, 2) grid detect_corners does."""
    image_points, _ = cv2.projectPoints(
        board_object_points(), np.asarray(rvec, dtype=np.float64),
        np.asarray(tvec, dtype=np.float64), k, d)
    return image_points.reshape(ROWS, COLS, 2).astype(float)


WIDTH, HEIGHT = 1280, 720

# Poses that put the board in each quadrant with a corner very near the frame's
# own corner, tilted, and entirely inside the image.
#
# **These numbers were searched for, and the search is the finding.** The first
# version of this file used politely-centred poses reaching about half way to the
# frame corner, and test_the_control_is_not_straight failed at 0.19 px — not
# because the instrument was wrong but because a board near the optical axis is
# already straight. Sweeping pose against the uncalibrated deviation (2026-09-12):
#
#   corners reach 45% of the way to the frame corner   control 0.53 px
#   ...                                     49%        control 0.52 px
#   ...                                     70%        control 0.18-1.10 px
#   ...                                     92%        control 1.62 px
#   ...                                     98%        control 1.36-2.14 px
#
# Against P9's 1.0 px budget that is the whole story: with centred frames the
# *uncalibrated* control comes in under budget, so the gate would pass on nominal
# intrinsics and prove nothing. "Cover the frame corners, not just the middle" is
# therefore a precondition of the measurement rather than advice about technique,
# which is why coverage() exists and why gates/calibration.sh asserts a floor on
# it instead of trusting whoever waved the board.
POSES = [
    ([0.18, -0.35, 0.05], [-0.32, -0.18, 0.46]),   # top-left, tilted
    ([0.35, 0.35, 0.05], [0.06, -0.16, 0.42]),     # top-right, tilted
    ([-0.18, -0.35, 0.05], [-0.22, 0.00, 0.34]),   # bottom-left, tilted
    ([-0.35, -0.18, 0.05], [0.12, 0.04, 0.46]),    # bottom-right, tilted
]

# The trap, kept as data so a test can assert on it: the board in the middle of
# the frame, which is what an unguided calibration session produces.
CENTRED_POSE = ([0.0, 0.0, 0.0], [-0.11, -0.07, 0.33])


def test_a_flat_board_with_no_distortion_is_already_straight():
    """The instrument's zero. A pinhole projection of a plane has straight lines.

    If this fails, every other number here is measuring the line fit rather than
    the lens — so it is the first thing to check and the reason it is separate.
    """
    grid = project([0.2, -0.15, 0.05], [-0.12, -0.07, 0.55], k=TRUE_K, d=np.zeros(5))
    assert straightness.worst_deviation(grid) < 1e-6


@pytest.mark.parametrize('rvec,tvec', POSES)
def test_undistorting_with_the_true_coefficients_recovers_straight_lines(rvec, tvec):
    """The claim itself: the right D makes the board's lines straight again."""
    distorted = project(rvec, tvec)
    corrected = straightness.undistort_grid(distorted, TRUE_K, TRUE_D)
    # Not exactly zero: undistortPoints inverts the model iteratively. A hundredth
    # of a pixel is two orders below the gate's 1.0 px budget.
    assert straightness.worst_deviation(corrected) < 0.01


@pytest.mark.parametrize('rvec,tvec', POSES)
def test_the_control_is_not_straight(rvec, tvec):
    """...and the control must fail, or the assertion above is not evidence.

    This is the half that makes the gate's with/without comparison meaningful. If
    a distorted board were already straight to within the budget, a passing gate
    would say nothing about the calibration — which is exactly the trap
    gates/hello-ipc.sh hit when two allocations happened to land at one address.
    """
    distorted = project(rvec, tvec)
    uncorrected = straightness.undistort_grid(distorted, TRUE_K, np.zeros(5))
    # Above the gate's 1.0 px budget, which is the bar that matters: a control
    # that came in *under* budget would mean the budget could be met without any
    # calibration at all. These poses give 1.36-2.14 px.
    assert straightness.worst_deviation(uncorrected) > 1.0


def test_zero_distortion_undistortion_is_the_identity():
    """undistort_grid with D=0 must return the input, not merely something close.

    **This is why the gate has one control and not two.** undistortPoints takes
    pixels to normalised coordinates through K inverse, applies the model, and
    puts them back through P; with P=K and D=0 the two K's cancel exactly, so the
    call is the identity *whatever K is*. The "nominal placeholder" control and a
    "no correction at all" control are therefore the same measurement — which is
    how they came out identical to four decimals on a synthetic board and prompted
    working this out. An earlier version of the tool reported both as independent
    controls on the reasoning that the nominal K changes fx as well as D and a
    pixel deviation scales with fx; that reasoning is wrong, because with D=0
    there is no fx in the answer to be confounded by.

    It also rules out the inflated-control failure: if this were a no-op *plus* a
    scaling — a missing P=K, say — the control would be larger than the truth and
    the ratio the gate prints would be flattering nonsense.
    """
    grid = project([0.15, 0.2, -0.05], [-0.14, -0.08, 0.56])
    same = straightness.undistort_grid(grid, TRUE_K, np.zeros(5))
    assert np.allclose(grid, same, atol=1e-3)

    # ...and with a *different* K, to show the K genuinely cancels rather than
    # happening not to matter for this one matrix.
    other = straightness.undistort_grid(grid, straightness.NOMINAL_K, np.zeros(5))
    assert np.allclose(grid, other, atol=1e-3)


def test_the_nominal_control_equals_no_correction_at_all():
    """The equality the gate asserts, pinned here where it is cheap to check.

    The placeholder camera_node published before P9 does not correct the lens
    badly; it does not correct it at all. So "straighter than the placeholder" and
    "straighter than the raw corners" are one claim, and the gate says so instead
    of printing one piece of evidence as two.
    """
    grids = [project(r, t) for r, t in POSES]
    via_placeholder = straightness.measure(grids, straightness.NOMINAL_K, straightness.NOMINAL_D)
    raw = max(straightness.worst_deviation(g) for g in grids)
    assert abs(via_placeholder['worst'] - raw) < 1e-6


def test_measure_reports_the_worst_frame_and_the_control_is_worse():
    """End to end over several poses, the way the gate runs it."""
    grids = [project(r, t) for r, t in POSES]

    calibrated = straightness.measure(grids, TRUE_K, TRUE_D)
    control = straightness.measure(grids, straightness.NOMINAL_K, straightness.NOMINAL_D)

    assert calibrated['frames'] == len(POSES)
    assert calibrated['worst'] < 0.01
    # An order of magnitude, not a hair. The gate asserts "strictly better"; this
    # asserts the separation is large enough for that to be a real measurement.
    assert control['worst'] > 10 * calibrated['worst']
    assert calibrated['mean'] <= calibrated['worst']


def test_worst_deviation_finds_a_bent_column_not_only_a_bent_row():
    """Both axes are checked, and a synthetic bend proves it.

    Radial distortion bends rows and columns by different amounts depending on
    where the board sits, so an instrument that fitted only rows would silently
    under-report for half of all frames.
    """
    straight = project([0.0, 0.0, 0.0], [-0.10, -0.06, 0.55], d=np.zeros(5))
    assert straightness.worst_deviation(straight) < 1e-6

    # Displace one interior point along x. It is in the middle of both a row and a
    # column, so a row-only instrument would also catch this — displace it and
    # then check the columns alone to be sure.
    bent = straight.copy()
    bent[3, 4, 0] += 4.0
    assert straightness.worst_deviation(bent) > 1.0

    # Columns only: bend a column by moving a point perpendicular to it. The
    # column runs roughly vertically, so the offset is in x.
    column = bent[:, 4, :]
    centred = column - column.mean(axis=0)
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    assert float(np.max(np.abs(centred @ vh[1]))) > 1.0


def test_a_near_vertical_line_is_fitted_without_degenerating():
    """Total least squares, not y-on-x — and a vertical line is where that shows.

    A board column in a 1280x720 frame is close to vertical. A least-squares fit
    of y against x has no answer for a truly vertical line and a badly
    conditioned one nearby, so it would report a large deviation for a perfectly
    straight column. Using the SVD's second singular vector has no such axis.
    """
    exactly_vertical = np.array([[640.0, y] for y in range(100, 600, 100)])
    assert straightness._worst_distance_from_line(exactly_vertical) < 1e-9

    exactly_horizontal = np.array([[x, 360.0] for x in range(100, 600, 100)])
    assert straightness._worst_distance_from_line(exactly_horizontal) < 1e-9

    # ...and a genuinely bent near-vertical line is still caught.
    bent = exactly_vertical.copy()
    bent[2, 0] += 2.5
    assert straightness._worst_distance_from_line(bent) > 1.0


def test_a_transposed_grid_would_be_caught():
    """Guard the reshape in detect_corners, which is the quiet way to be wrong.

    A (cols, rows) reshape of a (rows, cols) grid produces lines that zig-zag
    across the board rather than running along it. It yields a number, not an
    error — so the fact that it yields a *much larger* number even on an
    undistorted board is what a future reader needs to see written down.
    """
    straight = project([0.1, -0.1, 0.0], [-0.11, -0.07, 0.55], d=np.zeros(5))
    assert straightness.worst_deviation(straight) < 1e-6

    transposed = straight.reshape(COLS, ROWS, 2)
    assert straightness.worst_deviation(transposed) > 1.0


def test_detect_corners_returns_none_when_there_is_no_board():
    """A frame with the board half out of shot is a normal outcome, not an error."""
    blank = np.full((720, 1280, 3), 127, dtype=np.uint8)
    assert straightness.detect_corners(blank, (COLS, ROWS)) is None


def test_the_nominal_control_matches_the_node_default():
    """The placeholder K here must be the one camera_node falls back to.

    Two copies of the same nine numbers, one in C++ and one in Python, is exactly
    the drift this project keeps scripts for. gates/calibration.sh compares them
    against the source of truth; this pins the Python side so a change to it
    fails here, fast, rather than in a gate that needs the Pi and the camera.
    """
    assert straightness.NOMINAL_K.tolist() == [
        [907.0, 0.0, 640.0], [0.0, 907.0, 360.0], [0.0, 0.0, 1.0]]
    assert straightness.NOMINAL_D.tolist() == [0.0] * 5


# --- Coverage, which is a precondition rather than a statistic ----------------


def test_the_corner_poses_actually_reach_the_corners():
    """The poses above must cover the lens, or the tests using them are hollow."""
    grids = [project(r, t) for r, t in POSES]
    cover = straightness.coverage(grids, TRUE_K, WIDTH, HEIGHT)

    assert cover['max_radius_frac'] > 0.9, cover
    assert cover['quadrants'] == 4, cover

    # ...and entirely inside the frame, or findChessboardCorners would never see
    # them on a real image and the poses would be untestable fiction.
    for grid in grids:
        points = grid.reshape(-1, 2)
        assert points[:, 0].min() > 0 and points[:, 0].max() < WIDTH
        assert points[:, 1].min() > 0 and points[:, 1].max() < HEIGHT


def test_a_centred_board_scores_low_coverage():
    """The guard has to fire on the frames it exists to reject."""
    grid = project(*CENTRED_POSE)
    cover = straightness.coverage([grid], TRUE_K, WIDTH, HEIGHT)
    assert cover['max_radius_frac'] < 0.6, cover


def test_a_centred_board_would_pass_the_budget_uncalibrated():
    """**Why the coverage floor is an assertion and not a note in a docstring.**

    This is the failure the gate's coverage check exists to prevent, written down
    as a test so it cannot quietly stop being true. With the board in the middle
    of the frame the *uncalibrated* control deviates by about half a pixel — under
    P9's 1.0 px budget — so a gate that asserted only "calibrated is under budget"
    would pass on nominal intrinsics, with the full authority of a script behind a
    claim that had not been tested.

    The with/without ratio alone does not save it either: the ratio is still
    large, because the calibrated figure is near zero. What saves it is requiring
    the frames to have reached the part of the lens that bends.
    """
    grid = project(*CENTRED_POSE)
    uncalibrated = straightness.worst_deviation(
        straightness.undistort_grid(grid, TRUE_K, np.zeros(5)))
    assert uncalibrated < 1.0, (
        'if this ever fails, a centred board is no longer under budget and the '
        'coverage floor could be relaxed — check what changed first')

    cover = straightness.coverage([grid], TRUE_K, WIDTH, HEIGHT)
    assert cover['max_radius_frac'] < 0.6


def test_coverage_of_nothing_is_not_a_number():
    """An empty frame set must not report zero coverage as if it had measured."""
    cover = straightness.coverage([], TRUE_K, WIDTH, HEIGHT)
    assert np.isnan(cover['max_radius_frac'])
    assert cover['quadrants'] == 0


# --- The reprojection error, recomputed rather than transcribed ---------------


def test_reprojection_error_is_near_zero_for_the_true_model():
    """A synthetic board projected through K and D must reproject onto itself."""
    grids = [project(r, t) for r, t in POSES]
    rms = straightness.reprojection_rms(grids, TRUE_K, TRUE_D, SQUARE_M, (COLS, ROWS))
    # Everything here is exact apart from solvePnP's own iteration, so this is far
    # below P9's 0.5 px budget and the budget is not what is being tested.
    assert rms < 0.05, rms


def test_reprojection_error_is_large_for_the_wrong_model():
    """...and the control again: the nominal placeholder must not fit.

    Without this, `reproj_rms_px` could be a function that returns a small number
    whatever it is given, and report.txt would record it with a script's
    authority.
    """
    grids = [project(r, t) for r, t in POSES]
    rms = straightness.reprojection_rms(grids, straightness.NOMINAL_K, straightness.NOMINAL_D,
                                        SQUARE_M, (COLS, ROWS))
    assert rms > 0.5, rms


def test_reprojection_error_does_not_depend_much_on_the_square_size():
    """The scale cancels for a pinhole model, which is why an approximate ruler
    reading does not corrupt *this* number — it corrupts the calibration, which is
    a different measurement and the reason P9 insists the square is measured.

    Worth pinning: if this ever stops being true, report.txt's figure has silently
    acquired a dependency on a value somebody eyeballed.
    """
    grids = [project(r, t) for r, t in POSES]
    a = straightness.reprojection_rms(grids, TRUE_K, TRUE_D, SQUARE_M, (COLS, ROWS))
    b = straightness.reprojection_rms(grids, TRUE_K, TRUE_D, SQUARE_M * 1.1, (COLS, ROWS))
    assert abs(a - b) < 0.02, (a, b)


# --- What the straightness assertion does NOT catch ---------------------------


def _calibrate_from(tilt_max, n=14, seed=7):
    """Calibrate from n synthetic views, tilted by at most `tilt_max` radians.

    tilt_max=0 is the board flat on a wall photographed square-on, which is what
    you get by sliding a camera around in front of it without ever angling it.
    """
    rng = np.random.default_rng(seed)
    objp = np.zeros((ROWS * COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE_M

    image_points, grids, tries = [], [], 0
    while len(image_points) < n and tries < 8000:
        tries += 1
        rvec = rng.uniform(-tilt_max, tilt_max, 3) if tilt_max > 0 else np.zeros(3)
        if tilt_max > 0:
            rvec[2] = rng.uniform(-0.15, 0.15)
        tvec = np.array([rng.uniform(-0.34, 0.20), rng.uniform(-0.26, 0.16),
                         rng.uniform(0.30, 0.55)])
        pts, _ = cv2.projectPoints(objp.astype(np.float64), rvec, tvec, TRUE_K, TRUE_D)
        p = pts.reshape(-1, 2)
        if (p[:, 0].min() < 6 or p[:, 1].min() < 6
                or p[:, 0].max() > WIDTH - 6 or p[:, 1].max() > HEIGHT - 6):
            continue
        # Sub-pixel detection noise, as a real corner fit carries.
        noisy = p + rng.normal(0, 0.05, p.shape)
        image_points.append(noisy.astype(np.float32).reshape(-1, 1, 2))
        grids.append(noisy.reshape(ROWS, COLS, 2))

    assert len(image_points) == n, f'only {len(image_points)} usable views'
    _, k, d, _, _ = cv2.calibrateCamera([objp] * n, image_points, (WIDTH, HEIGHT), None, None)
    return k, d.ravel(), grids


def test_square_on_views_alone_produce_a_wildly_wrong_calibration():
    """**The trap a wall-mounted board walks straight into.**

    With every view square-on to the board, focal length and radial distortion
    trade off against each other and the solve is poorly conditioned: the solver
    returns a pair that fits the corners it was given and is nowhere near the
    truth. This is the "converges happily and is wrong" failure the phase warns
    about for a *flexing* board, arriving by a different door — and a board taped
    flat to a wall is rigid, so the usual precaution does not help.

    Measured here rather than asserted from received wisdom, because the size of
    the error is the surprising part.
    """
    k_bad, d_bad, _ = _calibrate_from(0.0)
    k_ok, d_ok, _ = _calibrate_from(0.44)

    # The good fit lands within a fraction of a percent.
    assert abs(k_ok[0, 0] - TRUE_K[0, 0]) / TRUE_K[0, 0] < 0.02, k_ok[0, 0]

    # The square-on fit is out by *hundreds* of percent — ~4841 against a true 905.
    assert k_bad[0, 0] > 2 * TRUE_K[0, 0], k_bad[0, 0]
    # ...and its distortion is nonsense too, k1 ~ +1.97 against a true +0.085.
    assert abs(d_bad[0]) > 10 * abs(TRUE_D[0]), d_bad[0]


def test_a_degenerate_calibration_passes_the_straightness_budget():
    """The gap this pins: straightness does not catch a degenerate fit.

    A wrong-but-self-consistent model still straightens the lines it was fitted
    to, so the calibrated figure comes in *under* the 1.0 px budget and *better*
    than the control, and assertion 2 of gates/calibration.sh passes. This is why
    that gate also bounds fx and asserts the held-out reprojection error — see the
    next test. Ask what the gate does not touch.
    """
    k_bad, d_bad, _ = _calibrate_from(0.0)
    _, _, held_out = _calibrate_from(0.44, seed=11)

    straight = straightness.measure(held_out, k_bad, d_bad)
    control = max(straightness.worst_deviation(g) for g in held_out)

    # Both of assertion 2's conditions are satisfied by a 435%-wrong calibration.
    assert straight['worst'] < 1.0, straight['worst']
    assert straight['worst'] < control, (straight['worst'], control)


def test_the_held_out_reprojection_error_is_what_catches_it():
    """...and this is the assertion that does the work.

    The in-sample error does not: a calibrator fitting square-on views reports
    ~0.08 px for the degenerate solution, better than the 0.5 px budget and about
    as good as the correct fit. Run against frames the fit never saw, the same
    measurement separates them by a factor of ~50. That is the whole reason
    tools/calibrate.sh grabs its frames *before* the calibration session.
    """
    k_bad, d_bad, in_sample_bad = _calibrate_from(0.0)
    k_ok, d_ok, _ = _calibrate_from(0.44)
    _, _, held_out = _calibrate_from(0.44, seed=11)

    # In sample, the degenerate fit looks excellent — inside P9's 0.5 px budget.
    in_sample = straightness.reprojection_rms(
        in_sample_bad, k_bad, d_bad, SQUARE_M, (COLS, ROWS))
    assert in_sample < 0.5, in_sample

    # Held out, it is disastrous, and the good fit is not.
    bad = straightness.reprojection_rms(held_out, k_bad, d_bad, SQUARE_M, (COLS, ROWS))
    good = straightness.reprojection_rms(held_out, k_ok, d_ok, SQUARE_M, (COLS, ROWS))
    assert bad > 1.0, bad
    assert good < 0.5, good
    assert bad > 10 * good, (bad, good)


def test_the_focal_length_bound_separates_the_two():
    """The gate's cheap check, pinned against the same two calibrations.

    MIN_FX/MAX_FX in gates/calibration.sh are 450 and 1814 — 0.5x to 2x the
    nominal 907. Deliberately wide: it is a "the solve degenerated" detector, not
    a claim about what fx should be.
    """
    k_bad, _, _ = _calibrate_from(0.0)
    k_ok, _, _ = _calibrate_from(0.44)

    assert 450 <= k_ok[0, 0] <= 1814, k_ok[0, 0]
    assert not (450 <= k_bad[0, 0] <= 1814), k_bad[0, 0]


# --- Marker confirmation: the silent misregistration guard --------------------
#
# **These use their own board constants, and the reason is a trap worth naming.**
# The synthetic tests above use a 9x6 *interior-corner* chessboard, an abstract grid
# with no physical counterpart. The real target is docs/charuco_a4_7x9_25mm.pdf:
# 7x9 SQUARES, therefore 6x8 interior corners. Reusing COLS/ROWS here silently asks
# for the wrong lattice and detect_corners simply returns None — which is exactly how
# these tests failed first time round.
CH_SQUARES_X, CH_SQUARES_Y = 7, 9
CH_COLS, CH_ROWS = CH_SQUARES_X - 1, CH_SQUARES_Y - 1
CH_SQUARE_M, CH_MARKER_M = 0.02475, 0.01782


def _render_board(squares_x=CH_SQUARES_X, squares_y=CH_SQUARES_Y, square_m=CH_SQUARE_M,
                  marker_m=CH_MARKER_M, dict_name='4x4_250', px_per_m=2600):
    """Render the ChArUco board to an image, across OpenCV versions.

    `generateImage` is the current name and `draw` the 4.6 one, so both are tried —
    the same portability rule as charuco_board() itself.
    """
    dictionary = straightness.aruco_dictionary(dict_name)
    board = straightness.charuco_board(squares_x, squares_y, square_m, marker_m, dictionary)
    size = (int(squares_x * square_m * px_per_m), int(squares_y * square_m * px_per_m))
    # generateImage is 4.8+, draw is the older name. Checked by attribute rather than
    # version here because, unlike the constructors in calib_straightness, only one of
    # these two names exists on each OpenCV — so the absent one fails loudly.
    if hasattr(board, 'generateImage'):
        img = board.generateImage(size)
    else:
        img = board.draw(size)
    # px_per_m=2600 gives a ~65 px square pitch, close to what the real frames show.
    # It is not a free parameter: findChessboardCorners fails on this board above
    # about 1000 px across (measured — OK at 773x971, FAIL at 1119x1416, and FAIL on
    # the 2481x3508 300-dpi render of the same sheet), which is a resolution effect in
    # the detector rather than anything about the board. The markers decode at every
    # size tried, so only the chessboard half constrains this.
    #
    # A quiet white border, or the outermost corners sit on the image edge and
    # findChessboardCorners will not see them.
    pad = 40
    return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255), \
        dictionary, board


def test_the_version_shims_build_a_board_on_this_opencv():
    """`charuco_board` and `aruco_dictionary` must work on 4.6 and 4.10 alike.

    This is the cv2.aruco equivalent of the `get_package_share_path` lesson: the API
    differs across the two machines, so the code picks the spelling that exists on
    both and a test proves it did. Without this the gate's instrument would work here
    and fail on the Pi, which is the ABI split in a new costume.
    """
    dictionary = straightness.aruco_dictionary('4x4_250')
    board = straightness.charuco_board(CH_SQUARES_X, CH_SQUARES_Y, CH_SQUARE_M,
                                              CH_MARKER_M, dictionary)
    assert board is not None
    with pytest.raises(ValueError):
        straightness.aruco_dictionary('not_a_dictionary')


def test_confirm_grid_accepts_a_correctly_registered_grid():
    image, dictionary, board = _render_board()
    grid = straightness.detect_corners(image, (CH_COLS, CH_ROWS))
    assert grid is not None, 'the rendered board must be detectable, or the rest is vacuous'

    ok, why, oriented, median = straightness.confirm_grid(image, grid, dictionary, board)
    assert ok, why
    assert oriented is not None
    assert oriented.shape == grid.shape
    assert median < 2.0


def test_confirm_grid_rejects_a_grid_shifted_by_one_square():
    """**The frame-13 failure, pinned.**

    `findChessboardCorners` locked onto a lattice one square out on a real frame and
    reported success — a genuine, internally consistent 6x8 grid of corners, just not
    the right one, disagreeing with the markers by 45.8 px at a 49 px square pitch.
    Nothing about the detection announced it, and with that one frame in a set of 35
    the calibration straightened nothing: 4.16 px against a 4.17 px control.

    Simulated here by shifting the grid by exactly one square pitch, which is what
    the real failure amounted to.
    """
    image, dictionary, board = _render_board()
    grid = straightness.detect_corners(image, (CH_COLS, CH_ROWS))
    assert grid is not None

    pitch = np.linalg.norm(grid[0, 1] - grid[0, 0])
    shifted = grid + np.array([pitch, 0.0])

    ok, why, _, median = straightness.confirm_grid(image, shifted, dictionary, board)
    assert not ok
    assert 'MISREGISTERED' in why, why
    # It must report roughly the shift it found, so the message is actionable.
    assert abs(median - pitch) < 0.25 * pitch, (median, pitch)


def test_confirm_grid_rejects_an_image_with_no_markers():
    """A blurred or too-oblique frame decodes no markers, and must be refused.

    The same check therefore rejects three failure modes at once — misregistration,
    motion blur, and obliquity past the point where markers resolve. On the real
    35-frame set it rejected 15: one misregistered and fourteen unconfirmable, taking
    the reprojection error from 0.7684 px to 0.4548 px.
    """
    image, dictionary, board = _render_board()
    grid = straightness.detect_corners(image, (CH_COLS, CH_ROWS))
    assert grid is not None

    blank = np.full_like(image, 127)
    ok, why, oriented, _ = straightness.confirm_grid(blank, grid, dictionary, board)
    assert not ok
    # Nothing usable comes back on a refusal, so a caller that ignores `ok` gets a
    # TypeError rather than a silently unconfirmed grid.
    assert oriented is None
    assert 'marker' in why.lower(), why


def test_confirm_grid_accepts_either_orientation_and_canonicalises():
    """**The 180 degree ambiguity, which cost a whole grab session.**

    `findChessboardCorners` cannot tell a symmetric grid from the same grid read end
    to end. Both orderings describe the same physical corners; which one comes back
    depends on how the board happens to sit in the image. The first version of
    `confirm_grid` compared only the as-detected ordering, so when the board was
    remounted the other way up every frame was reported "MISREGISTERED — off by
    249 px" and the grabber saved nothing at all.

    A flip is not a misregistration. It must be accepted *and* relabelled back, so
    that the ordering cannot differ between frames within one set — which would
    corrupt the object-point correspondence exactly as badly as a one-square shift.
    """
    image, dictionary, board = _render_board()
    grid = straightness.detect_corners(image, (CH_COLS, CH_ROWS))
    assert grid is not None

    ok_a, _, canon_a, _ = straightness.confirm_grid(image, grid, dictionary, board)
    flipped = grid[::-1, ::-1, :].copy()
    ok_b, why_b, canon_b, _ = straightness.confirm_grid(image, flipped, dictionary, board)

    assert ok_a
    assert ok_b, f'a 180 degree flip must be accepted, not rejected: {why_b}'
    # ...and both must come back in the same ordering, or the set is inconsistent.
    assert np.allclose(canon_a, canon_b, atol=1e-6)


def test_a_transposed_size_is_detected_but_refused():
    """A transposed `--size` is caught by the markers, not by the detector.

    I assumed a quarter turn was impossible because the grid is 6x8 rather than
    square — that asking for 8x6 simply would not match. **It does match:**
    findChessboardCorners happily returns the board transposed, shape (6, 8) instead
    of (8, 6), with no complaint. So `--size 8x6` on this board produces a real grid
    that is wrong, and only marker confirmation rejects it — measured at 231.9 px
    against a 49-65 px square pitch.

    That is also why confirm_grid does *not* try the transpose among its candidates
    the way it tries the 180 degree flip. A flip is the detector's unavoidable
    ambiguity about the same corners and must be absorbed; a transpose means the
    caller asked for the wrong board and must be reported, not quietly fixed.
    tools/calibrate.sh derives --size from --squares so it cannot drift, and this
    test is what says why that derivation matters.
    """
    image, dictionary, board = _render_board()

    transposed = straightness.detect_corners(image, (CH_ROWS, CH_COLS))
    assert transposed is not None, 'the detector does accept a transposed pattern size'
    assert transposed.shape == (CH_COLS, CH_ROWS, 2)

    ok, why, oriented, median = straightness.confirm_grid(image, transposed, dictionary, board)
    assert not ok, 'a transposed grid must not be confirmed'
    assert 'MISREGISTERED' in why, why
    assert oriented is None
    assert median > 50
