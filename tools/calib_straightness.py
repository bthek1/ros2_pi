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
    cv2.cornerSubPix(
        grey, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    cols, rows = pattern_size
    return corners.reshape(rows, cols, 2).astype(float)


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
    """Worst and mean deviation over every detected grid, in pixels."""
    per_frame = [worst_deviation(undistort_grid(g, k, d)) for g in grids]
    if not per_frame:
        return {'worst': float('nan'), 'mean': float('nan'), 'frames': 0}
    return {
        'worst': max(per_frame),
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
    args = parser.parse_args(argv)

    cols, rows = (int(v) for v in args.size.lower().split('x'))
    cal = load_calibration(args.calibration)

    paths = sorted(
        p for ext in ('png', 'jpg', 'jpeg')
        for p in glob.glob(os.path.join(args.frames, f'*.{ext}')))
    if not paths:
        print(f'straightness error=no frames in {args.frames}')
        return 1

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
        grids.append(grid)

    cover = coverage(grids, cal['K'], cal['width'] or 1280, cal['height'] or 720)
    calibrated = measure(grids, cal['K'], cal['D'])
    nominal = measure(grids, NOMINAL_K, NOMINAL_D)
    # No undistortion call at all — the corners exactly as detected. Equal to
    # `nominal` by construction (see the module docstring); reported so the gate
    # can assert that equality rather than take it on trust.
    uncorrected = {
        'worst': max(worst_deviation(g) for g in grids) if grids else float('nan'),
        'mean': float(np.mean([worst_deviation(g) for g in grids])) if grids else float('nan'),
        'frames': len(grids),
    }

    print(f'straightness camera_name={cal["name"]}')
    print(f'straightness size={cols}x{rows}')
    print(f'straightness images={len(paths)}')
    print(f'straightness frames={calibrated["frames"]}')
    print(f'straightness missed={",".join(missed) if missed else "none"}')
    print(f'straightness max_radius_frac={cover["max_radius_frac"]:.3f}')
    print(f'straightness quadrants={cover["quadrants"]}')
    for key, value in (('calibrated', calibrated), ('nominal', nominal),
                      ('uncorrected', uncorrected)):
        print(f'straightness {key}_worst_px={value["worst"]:.4f}')
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
