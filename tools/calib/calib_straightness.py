#!/usr/bin/env python3
"""Does the calibration actually straighten the lens? — P9's real assertion.

The claim a calibration makes is geometric and checkable: a chessboard's rows
and columns are straight lines in the world, so after undistortion they must be
straight lines in the image. This measures how far they are from straight, in
pixels, and it does it **twice** — once with the calibration and once without.

**The control is the point, and it is the same lesson as gates/hello-ipc.sh.**
A single straightness number proves nothing, because a board photographed near
the optical axis is nearly straight before any correction at all. The claim is
not "the lines are straight"; it is "the lines are straighter *because of this
calibration*, and stop being so when it is swapped out". So three numbers come
out of here:

  calibrated    K and D from the calibration file
  uncorrected   the detected corners as they are, with no undistortion at all
  nominal       the placeholder K (fx=fy=907, centre principal point) and zero D,
                which is what camera_node published before P9

`nominal` is the comparison P9 asks for. **`uncorrected` is the same number, and
that is a fact about the maths rather than a coincidence** — measured identical to
four decimals on a synthetic board, 2026-09-12, which is what prompted working out
why. undistortPoints takes pixels to normalised coordinates through K inverse,
applies the distortion model, and puts them back through P; with P=K and D=0 the
two K's cancel exactly and the whole call is the identity. So the placeholder does
not merely correct badly, it corrects *nothing*, whatever its fx says.

An earlier version of this tool reported `zero_d` — the calibrated K with D forced
to zero — as a second, independent control, on the reasoning that `nominal`
changes fx as well as D and a deviation in pixels scales with fx. That reasoning
is wrong for the reason just given: with D=0 there is no fx to confound anything.
Two controls with one value between them is one piece of evidence presented as
two, so `uncorrected` replaces it and the gate asserts the equality as a
self-check instead — if those two ever diverge, something is passing distortion
coefficients where it should not be.

Underscore in the filename, unlike every other script in tools/: this one is
imported by src/pimesh_bringup/test/test_straightness.py, and a hyphen is not a
legal module name. The geometry is separated from the image loading for the same
reason — undistort_grid() and worst_deviation() are pure, so the whole
measurement chain can be checked against synthetic grids with a known
distortion, on a machine with no camera and before any board has been printed.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml

# The nominal placeholder camera_node falls back to, and the control this
# measures against. Kept in step with camera_matrix's default in
# src/pimesh_camera/src/camera_node.cpp by tools/gates/calibration.sh, which
# reads both and compares them rather than trusting this copy.
NOMINAL_K = np.array([[907.0, 0.0, 640.0], [0.0, 907.0, 360.0], [0.0, 0.0, 1.0]])
NOMINAL_D = np.zeros(5)


def load_calibration(path):
    """Read a standard camera_info YAML. The C++ loader's Python counterpart.

    Deliberately *not* a reimplementation of its validation: this is an analysis
    tool, and the refusals that matter belong in the node that publishes the
    numbers (see pimesh_camera/src/calibration.cpp). It does check the two
    shapes, because a wrong one here would produce a straightness figure rather
    than an error.
    """
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    k = np.array(doc['camera_matrix']['data'], dtype=float)
    d = np.array(doc['distortion_coefficients']['data'], dtype=float)
    if k.size != 9:
        raise ValueError(f'{path}: camera_matrix has {k.size} values, expected 9')
    if d.size != 5:
        raise ValueError(f'{path}: distortion_coefficients has {d.size} values, expected 5')
    return {
        'K': k.reshape(3, 3),
        'D': d,
        'width': int(doc.get('image_width', 0)),
        'height': int(doc.get('image_height', 0)),
        'name': doc.get('camera_name', ''),
    }


def detect_corners(image, pattern_size):
    """Find the interior corners of a chessboard, as a (rows, cols, 2) grid.

    `pattern_size` is (cols, rows), which is OpenCV's order and the order
    `cameracalibrator --size 9x6` means. findChessboardCorners returns the points
    row-major with `cols` of them per row, so the reshape below is the board's
    own grid and the rows and columns of the result are rows and columns of the
    board. Getting that transposed would fit lines across the board instead of
    along it and still produce a plausible number.

    Returns None when the board is not found, which is a normal outcome for a
    frame where it is half out of shot.
    """
    grey = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(grey, pattern_size, flags=flags)
    if not found:
        return None

    # Sub-pixel refinement, and it is not optional here. The whole measurement is
    # a sub-pixel one — the budget is 1.0 px and integer corners carry ±0.5 px of
    # quantisation on their own, which would be most of the answer.
    #
    # **7x7, not OpenCV's usual 11x11, and the reason is this board.** On
    # docs/charuco_a4_7x9_25mm.pdf the ArUco marker is 18 mm inside a 24.75 mm
    # square, so its black border sits 3.375 mm from each chessboard corner — about
    # 6.7 px at the ~49 px square pitch these frames show — which is *inside* an
    # 11x11 window, and the marker edge then drags the saddle-point fit. Measured
    # over the 75 frames of calib/c922_720p/frames/ (2026-09-12), reprojection error
    # against a full calibration:
    #
    #     11x11  1.0400 px      5x5  0.8969 px
    #      9x9   1.0079 px      4x4  1.0290 px
    #      7x7   0.8707 px      3x3  1.1705 px
    #
    # 7x7 is the optimum: wide enough to fit a saddle, narrow enough to exclude the
    # marker. Below it there are too few pixels left. A plain chessboard with no
    # markers would prefer the larger window, so this is a board-specific constant
    # and is worth revisiting if the target ever changes.
    cv2.cornerSubPix(
        grey, corners, (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))

    cols, rows = pattern_size
    return corners.reshape(rows, cols, 2).astype(float)


# --- Confirming a grid against the board's own markers -----------------------
#
# **Why this exists, measured 2026-09-12.** `findChessboardCorners` can lock onto a
# grid **shifted by one square** on a ChArUco board and report success: the result is
# a real, internally consistent 6x8 lattice of corners, just not the right one. One
# frame in 35 of `calib/c922_720p/frames/` did exactly that — `frame-13.jpg`,
# disagreeing with the markers by 45.8 px median against a 49 px square pitch, while
# looking sharp and nearly square-on. Nothing about the detection announced it.
#
# A misregistered frame is poison: its corners are matched to object points one
# square out, so it pulls the whole solve. With it in, the 35-frame set gave a
# reprojection error of 0.7684 px and a straightness of 4.16 px against a 4.17 px
# control — i.e. a calibration that straightened nothing. Rejecting it and the frames
# the markers could not confirm gave **0.4548 px** and 1.09 px against 1.18 px.
#
# The markers are the only thing that can catch it, because they are *identified* —
# every ChArUco corner carries the id that says which board corner it is, and a
# chessboard corner carries nothing but its position in whatever lattice was found.
#
# It rejects three failure modes with one test, which is why it is a single check:
#  - misregistration (the grid disagrees with the ids)
#  - motion blur (the markers cannot be decoded at all)
#  - extreme obliquity (likewise — beyond ~50 deg the markers stop resolving, and
#    those frames were independently the least accurate: mean reprojection 0.933 px
#    over 50 deg against 0.429 px in the 20-35 deg band)

# camera_calibration's spellings, so one name works for the calibrator and for here.
ARUCO_DICTS = {
    'aruco_orig': 'DICT_ARUCO_ORIGINAL',
    '4x4_50': 'DICT_4X4_50', '4x4_100': 'DICT_4X4_100',
    '4x4_250': 'DICT_4X4_250', '4x4_1000': 'DICT_4X4_1000',
    '5x5_50': 'DICT_5X5_50', '5x5_100': 'DICT_5X5_100',
    '5x5_250': 'DICT_5X5_250', '5x5_1000': 'DICT_5X5_1000',
    '6x6_50': 'DICT_6X6_50', '6x6_100': 'DICT_6X6_100',
    '6x6_250': 'DICT_6X6_250', '6x6_1000': 'DICT_6X6_1000',
    '7x7_50': 'DICT_7X7_50', '7x7_100': 'DICT_7X7_100',
    '7x7_250': 'DICT_7X7_250', '7x7_1000': 'DICT_7X7_1000',
}


def aruco_dictionary(name):
    """A predefined dictionary, by camera_calibration's name for it.

    `getPredefinedDictionary` is the spelling that exists on **both** machines —
    checked 2026-09-12, the Pi's OpenCV 4.6 and this box's 4.10 — where
    `Dictionary_get` is 4.6-only and `ArucoDetector` is 4.7+. Same rule as
    get_package_share_path in the C++: pick the name that is current on both.
    """
    if name not in ARUCO_DICTS:
        raise ValueError(f'unknown aruco dictionary {name!r}; one of {sorted(ARUCO_DICTS)}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICTS[name]))


def _cv_version():
    """(major, minor) of the running OpenCV, for the two version branches below."""
    parts = cv2.__version__.split('.')
    return int(parts[0]), int(parts[1])


def charuco_board(squares_x, squares_y, square_m, marker_m, dictionary):
    """A CharucoBoard, taking **squares** — not interior corners.

    The count is squares because that is what OpenCV wants, and it is worth saying
    twice because the command printed on our own sheet passes the corner count and
    silently interpolates nothing.

    **Branched on the version explicitly, and try/except is NOT good enough here.**
    The constructor was renamed in 4.8. On the Pi's 4.6,
    `cv2.aruco.CharucoBoard((7, 9), sq, mk, d)` does not raise — it **constructs a
    default, uninitialised board**, which then *segfaults* the interpreter the first
    time anything draws with it (measured 2026-09-12: the test suite died with
    `Fatal Python error: Segmentation fault` on the Pi and passed here). A
    try/except around the modern spelling therefore silently "succeeds" on the old
    OpenCV and hands back garbage, so the version has to be asked.

    This is the same shape as the cross-distro CMake trap: an API that exists at both
    ends and means different things is worse than one that is missing at one end,
    because the missing one fails loudly.
    """
    if _cv_version() >= (4, 8):
        return cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, dictionary)
    return cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_m, marker_m, dictionary)


def _detector_params():
    """Default ArUco detector parameters, on either OpenCV.

    Version-branched for the same reason as charuco_board, and here the failure would
    have been *quieter*: `DetectorParameters()` also constructs on 4.6, giving an
    object whose thresholding fields are zeroed, so marker detection would simply
    find nothing rather than crash — and "found no markers" is a result this code
    treats as a legitimate reason to reject a frame. Every frame would have been
    rejected on the Pi, with a plausible message and no error.
    """
    if _cv_version() >= (4, 7):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def confirm_grid(grey, grid, dictionary, board, tol_px=2.0, min_corners=8):
    """Check a findChessboardCorners grid against the board's identified corners,
    and return it in the orientation the markers agree with.

    \return (ok, reason, oriented_grid, median_deviation_px). `reason` is empty when
    ok; `oriented_grid` is None on failure.

    **The 180 degree ambiguity is handled here rather than rejected, and getting that
    wrong cost a whole grab session.** findChessboardCorners cannot tell a symmetric
    grid from the same grid read end to end — both describe the same corners and
    which one comes back depends on how the board happens to sit in the image. The
    first version of this function compared only the as-detected ordering, so the
    moment the board was remounted the other way up *every* frame came back
    "MISREGISTERED — off by 249 px" and the grabber saved nothing at all. A flip is
    not a misregistration: it is the same corners relabelled, and the fix is to
    recognise it and relabel back.

    Canonicalising rather than merely accepting matters for a second reason. If the
    ordering were allowed to differ *between* frames in one set, the object-point
    correspondence would flip mid-set, which corrupts a calibration exactly as badly
    as a one-square shift. Returning the marker-agreed orientation makes every frame
    consistent by construction.

    A 90 degree ambiguity cannot arise: the grid is 6x8, so a quarter turn would be
    an 8x6 grid and would not match the requested pattern size at all.

    `interpolateCornersCharuco` is used rather than `CharucoDetector` because it is
    the one entry point present on both 4.6 and 4.10.
    """
    rows, cols, _ = grid.shape
    corners, ids, _ = cv2.aruco.detectMarkers(grey, dictionary, parameters=_detector_params())
    if ids is None or len(ids) == 0:
        return False, 'no markers decoded (blurred, or too oblique to resolve)', None, float('nan')

    count, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, grey, board)
    if not count or count < min_corners:
        return (False, f'markers confirmed only {count or 0} corners, need {min_corners}',
                None, float('nan'))

    marker_corners = ch_corners.reshape(-1, 2)
    marker_ids = ch_ids.ravel()

    def deviations_for(candidate):
        out = []
        for corner, corner_id in zip(marker_corners, marker_ids):
            col, row = int(corner_id) % cols, int(corner_id) // cols
            if row < rows:
                out.append(float(np.linalg.norm(candidate[row, col] - corner)))
        return out

    # As detected, and read end to end. Reversing both axes is the 180 degree
    # relabelling: the same physical corners, opposite ordering.
    best = None
    for candidate in (grid, grid[::-1, ::-1, :].copy()):
        devs = deviations_for(candidate)
        if not devs:
            continue
        median = float(np.median(devs))
        if best is None or median < best[0]:
            best = (median, max(devs), len(devs), candidate)

    if best is None:
        return False, 'no confirmable corners overlapped the grid', None, float('nan')

    median, worst, confirmed, oriented = best
    if worst > tol_px:
        return (False, f'MISREGISTERED — grid disagrees with the markers by {median:.1f} px '
                       f'(worst {worst:.1f}) in both orientations; findChessboardCorners '
                       f'locked onto the wrong lattice',
                None, median)
    return True, '', oriented, median


def undistort_grid(grid, k, d):
    """Map a (rows, cols, 2) grid of pixel coordinates through the lens model.

    P=k puts the result back in pixels at the same focal length, so the output is
    comparable with the input and a deviation in pixels means the same thing on
    both sides. Without it undistortPoints returns normalised coordinates and
    every number below would be ~1/900th of the size.
    """
    rows, cols, _ = grid.shape
    flat = grid.reshape(-1, 1, 2).astype(np.float64)
    out = cv2.undistortPoints(flat, np.asarray(k, dtype=np.float64),
                             np.asarray(d, dtype=np.float64), P=np.asarray(k, dtype=np.float64))
    return out.reshape(rows, cols, 2)


def _worst_distance_from_line(points):
    """Largest perpendicular distance from any point to the best-fit line.

    Total least squares, via the second singular vector of the centred points —
    not a y-on-x least-squares fit, which has no answer for a vertical line and a
    badly conditioned one for a steep one. Board columns in a 1280x720 frame are
    close to vertical, so that is not a hypothetical.
    """
    centred = points - points.mean(axis=0)
    # Rows of vh are the principal directions; the second is the line's normal.
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[1]
    return float(np.max(np.abs(centred @ normal)))


def worst_deviation(grid):
    """The worst row or column of a grid, as a distance from straight in pixels.

    Both directions, because distortion is radial: a board centred horizontally
    and offset vertically bends its rows much more than its columns, and taking
    only one axis would miss half the frames.
    """
    rows, cols, _ = grid.shape
    worst = 0.0
    for r in range(rows):
        worst = max(worst, _worst_distance_from_line(grid[r, :, :]))
    for c in range(cols):
        worst = max(worst, _worst_distance_from_line(grid[:, c, :]))
    return worst


def coverage(grids, k, width, height):
    """Where in the frame the detected corners actually are.

    **This is a precondition on the straightness measurement, not a statistic.**
    Distortion is radial: near the principal point there is almost none to
    correct, so a set of frames with the board politely in the middle gives a
    "before" that is already straight and an "after" that is no better, and the
    comparison the gate makes says nothing about the calibration.

    Measured with the synthetic board in test_straightness.py, at the 1.0 px
    budget (2026-09-12): corners reaching 49% of the way to the frame corner put
    the *uncalibrated* control at 0.52 px — comfortably inside the budget, so the
    gate would have passed on nominal intrinsics. At 98% the control is 1.4-2.1 px
    and the calibrated figure is ~0.000, which is a measurement with something in
    it. Hence the floor the gate asserts.

    `max_radius_frac` is the furthest corner from the principal point as a
    fraction of the distance to the frame's own corner, so 1.0 means a corner
    landed in the very corner of the image. `quadrants` counts how many of the
    four quadrants around the principal point were visited at all, because one
    board wedged into a single corner can score a high radius while leaving three
    quarters of the lens unmeasured.
    """
    if not grids:
        return {'max_radius_frac': float('nan'), 'quadrants': 0}
    points = np.concatenate([g.reshape(-1, 2) for g in grids])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    half_diagonal = float(np.hypot(width / 2.0, height / 2.0))
    radii = np.linalg.norm(points - np.array([cx, cy]), axis=1)
    quadrants = {(int(u > cx), int(v > cy)) for u, v in points}
    return {
        'max_radius_frac': float(radii.max() / half_diagonal),
        'quadrants': len(quadrants),
    }


def measure(grids, k, d):
    """Deviation from straight over every detected grid, as a distribution.

    The **whole** distribution, not one number, because which one is asserted on turns
    out to matter and the choice should be visible. On the 24-frame set of 2026-09-12:
    max 1.276 px, p90 1.170, median 0.780, min 0.364, with 5 of 24 frames over 1.0.
    The max is not one freak frame — it is the top of a continuous tail — so anything
    reported as "the" straightness is a choice about which end of that tail to quote.
    """
    per_frame = [worst_deviation(undistort_grid(g, k, d)) for g in grids]
    if not per_frame:
        return {'worst': float('nan'), 'p90': float('nan'), 'median': float('nan'),
                'mean': float('nan'), 'frames': 0}
    return {
        'worst': max(per_frame),
        'p90': float(np.percentile(per_frame, 90)),
        'median': float(np.median(per_frame)),
        'mean': float(np.mean(per_frame)),
        'frames': len(per_frame),
        'per_frame': per_frame,
    }


def reprojection_rms(grids, k, d, square_m, pattern_size):
    """RMS reprojection error in pixels, recomputed from the stored calibration.

    `cameracalibrator` prints a reprojection error on the console when it solves,
    and that number is the obvious thing to write into report.txt — but it is a
    number nobody can check afterwards, and a transcribed figure is the same class
    of artefact as a hand-typed K. This recomputes it: solvePnP puts the board
    where the calibration says it must be, the corners are projected back through
    the model, and the residual is the error.

    It is a *different* number from the calibrator's, and deliberately so. The
    calibrator reports its own fit over its own frames, in-sample. Run over frames
    the calibration never saw, this is the held-out version of the same
    measurement, which is the one worth believing.

    `square_m` sets the scale of the object points. It cancels out of the residual
    for a pinhole model, so an approximate value does not corrupt this the way it
    corrupts the calibration itself — but it is still asked for rather than
    assumed, because a zero or a centimetre/metre mix-up would make solvePnP
    degenerate.
    """
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_m)

    squared, count = 0.0, 0
    for grid in grids:
        image_points = grid.reshape(-1, 1, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(objp, image_points, np.asarray(k, dtype=np.float64),
                                      np.asarray(d, dtype=np.float64))
        if not ok:
            continue
        projected, _ = cv2.projectPoints(objp, rvec, tvec, np.asarray(k, dtype=np.float64),
                                         np.asarray(d, dtype=np.float64))
        residual = projected.reshape(-1, 2) - grid.reshape(-1, 2)
        squared += float(np.sum(residual ** 2))
        count += residual.shape[0]

    return float(np.sqrt(squared / count)) if count else float('nan')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--frames', required=True,
                        help='directory of saved frames containing the board')
    parser.add_argument('--calibration', required=True,
                        help='the camera_info YAML to test')
    parser.add_argument('--size', default='9x6',
                        help='interior corners as COLSxROWS, as passed to cameracalibrator')
    parser.add_argument('--square', type=float, default=None,
                        help='measured square size in metres; enables the reprojection error')
    parser.add_argument('--squares', default=None,
                        help='board SQUARES as XxY (e.g. 7x9); enables marker confirmation')
    parser.add_argument('--marker', type=float, default=None,
                        help='ArUco marker size in metres; required with --squares')
    parser.add_argument('--dict', default='4x4_250', help='ArUco dictionary name')
    args = parser.parse_args(argv)

    cols, rows = (int(v) for v in args.size.lower().split('x'))
    cal = load_calibration(args.calibration)

    paths = sorted(
        p for ext in ('png', 'jpg', 'jpeg')
        for p in glob.glob(os.path.join(args.frames, f'*.{ext}')))
    if not paths:
        print(f'straightness error=no frames in {args.frames}')
        return 1

    # Marker confirmation, when the board spec is given. Without it a frame whose
    # grid is one square out is indistinguishable from a good one — see confirm_grid.
    dictionary = board = None
    if args.squares:
        if args.marker is None:
            print('straightness error=--squares needs --marker')
            return 1
        sx, sy = (int(v) for v in args.squares.lower().split('x'))
        if (sx - 1, sy - 1) != (cols, rows):
            print(f'straightness error=--squares {sx}x{sy} implies {sx-1}x{sy-1} interior '
                  f'corners but --size says {cols}x{rows}')
            return 1
        dictionary = aruco_dictionary(args.dict)
        board = charuco_board(sx, sy, args.square or 0.025, args.marker, dictionary)

    grids, missed = [], []
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            missed.append(f'{os.path.basename(path)}:unreadable')
            continue
        # A frame calibrated at one resolution and saved at another would be
        # measured against the wrong K without anything looking wrong.
        if cal['width'] and (image.shape[1], image.shape[0]) != (cal['width'], cal['height']):
            missed.append(
                f'{os.path.basename(path)}:{image.shape[1]}x{image.shape[0]}'
                f'-not-{cal["width"]}x{cal["height"]}')
            continue
        grid = detect_corners(image, (cols, rows))
        if grid is None:
            missed.append(f'{os.path.basename(path)}:no-board')
            continue
        if board is not None:
            grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ok, why, oriented, _ = confirm_grid(grey, grid, dictionary, board)
            if not ok:
                missed.append(f'{os.path.basename(path)}:{why.split(" —")[0].split(",")[0]}')
                continue
            # The marker-agreed ordering, so every frame in the set is consistent.
            grid = oriented
        grids.append(grid)

    cover = coverage(grids, cal['K'], cal['width'] or 1280, cal['height'] or 720)
    calibrated = measure(grids, cal['K'], cal['D'])
    nominal = measure(grids, NOMINAL_K, NOMINAL_D)
    # No undistortion call at all — the corners exactly as detected. Equal to
    # `nominal` by construction (see the module docstring); reported so the gate
    # can assert that equality rather than take it on trust.
    raw = [worst_deviation(g) for g in grids]
    uncorrected = {
        'worst': max(raw) if raw else float('nan'),
        'p90': float(np.percentile(raw, 90)) if raw else float('nan'),
        'median': float(np.median(raw)) if raw else float('nan'),
        'mean': float(np.mean(raw)) if raw else float('nan'),
        'frames': len(grids),
    }

    print(f'straightness camera_name={cal["name"]}')
    print(f'straightness size={cols}x{rows}')
    print(f'straightness images={len(paths)}')
    print(f'straightness frames={calibrated["frames"]}')
    print(f'straightness confirmed={"yes" if board is not None else "NO — no --squares given"}')
    print(f'straightness missed={",".join(missed) if missed else "none"}')
    print(f'straightness max_radius_frac={cover["max_radius_frac"]:.3f}')
    print(f'straightness quadrants={cover["quadrants"]}')
    for key, value in (('calibrated', calibrated), ('nominal', nominal),
                      ('uncorrected', uncorrected)):
        print(f'straightness {key}_worst_px={value["worst"]:.4f}')
        print(f'straightness {key}_p90_px={value["p90"]:.4f}')
        print(f'straightness {key}_median_px={value["median"]:.4f}')
        print(f'straightness {key}_mean_px={value["mean"]:.4f}')
    if calibrated['frames']:
        print('straightness ratio_vs_nominal='
              f'{nominal["worst"] / calibrated["worst"]:.3f}')
        # Should be 0 to within float noise. The gate asserts on it.
        print('straightness nominal_minus_uncorrected_px='
              f'{abs(nominal["worst"] - uncorrected["worst"]):.6f}')
    for name, value in (('fx', cal['K'][0, 0]), ('fy', cal['K'][1, 1]),
                        ('cx', cal['K'][0, 2]), ('cy', cal['K'][1, 2])):
        print(f'straightness {name}={value:.3f}')
    print('straightness d=' + ','.join(f'{v:.6f}' for v in cal['D']))
    if args.square and grids:
        rms = reprojection_rms(grids, cal['K'], cal['D'], args.square, (cols, rows))
        print(f'straightness reproj_rms_px={rms:.4f}')
        print(f'straightness square_m={args.square:.5f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
