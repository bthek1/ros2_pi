#!/usr/bin/env python3
"""Fit the intrinsics from the selected frames and write the camera_info YAML.

**This replaces `cameracalibrator` for this project, and not only by preference.**
On Lyrical the tool does not run at all: `CalibrationNode.__init__` builds `left` and
`right` stereo subscribers unconditionally, even for a monocular calibration, and
`get_topic_qos()` logs through `RcutilsLogger.warn` when a topic has no publisher — a
method removed from rclpy. So a mono session crashes at construction with
`AttributeError: 'RcutilsLogger' object has no attribute 'warn'` before it has looked
at a single frame, because nothing ever publishes `/left`. Measured 2026-09-12.

Even where it runs, feeding it is redundant here. Its value is choosing well-spread
frames from a live stream through the X/Y/Size/Skew heuristic — and `calib_select.py`
already does that, from a bag, with marker confirmation the calibrator has no
equivalent of. What is left is `cv2.calibrateCamera`, which is what it calls anyway.

What this adds over calling OpenCV directly:

  * every frame marker-confirmed and canonically oriented, so a grid that is a square
    out or read end-to-end cannot enter the fit;
  * a **held-out** reprojection error by two-fold split, which is the number that
    catches a degenerate fit — an in-sample figure cannot (a square-on-only set
    reports 0.08 px for a focal length five times the truth);
  * the standard `camera_info` YAML written byte-compatibly with what
    `cameracalibrator` emits, so `camera_node` reads it unchanged and nothing has to
    be transcribed.
"""

import argparse
import datetime
import glob
import os
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_straightness as cs  # noqa: E402


def load_frames(frames_dir, cols, rows, dictionary, board):
    """Every frame that detects and confirms, in the marker-agreed orientation."""
    grids, used, skipped = [], [], []
    for path in sorted(glob.glob(os.path.join(frames_dir, '*.jpg'))):
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            skipped.append((os.path.basename(path), 'unreadable'))
            continue
        grid = cs.detect_corners(image, (cols, rows))
        if grid is None:
            skipped.append((os.path.basename(path), 'no board'))
            continue
        ok, why, oriented, _ = cs.confirm_grid(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), grid, dictionary, board)
        if not ok:
            skipped.append((os.path.basename(path), why.split('—')[0].strip()))
            continue
        grids.append(oriented)
        used.append(os.path.basename(path))
    return grids, used, skipped


def fit(grids, objp, size):
    points = [g.reshape(-1, 1, 2).astype(np.float32) for g in grids]
    rms, k, d, _, _ = cv2.calibrateCamera([objp] * len(points), points, size, None, None)
    return rms, k, d.ravel()[:5]


def ost_yaml(name, width, height, k, d):
    """The exact layout `cameracalibrator` writes, so nothing downstream has to change.

    P is K with a zero fourth column and R is identity: one camera, not rectified
    against another. `pimesh_camera` rebuilds both from K itself, but a consumer
    reading this file directly must not find zeros there.
    """
    def mat(values, per_row):
        rows = [', '.join(f'{v:.6f}' for v in values[i:i + per_row])
                for i in range(0, len(values), per_row)]
        return '[' + ', '.join(', '.join(r for r in rows).split(', ')) + ']'

    flat = list(np.asarray(k).ravel())
    projection = [flat[0], 0.0, flat[2], 0.0, 0.0, flat[4], flat[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    return '\n'.join([
        f'image_width: {width}',
        f'image_height: {height}',
        f'camera_name: {name}',
        'camera_matrix:',
        '  rows: 3',
        '  cols: 3',
        '  data: ' + mat(flat, 3),
        'distortion_model: plumb_bob',
        'distortion_coefficients:',
        '  rows: 1',
        '  cols: 5',
        '  data: ' + mat(list(np.asarray(d).ravel()), 5),
        'rectification_matrix:',
        '  rows: 3',
        '  cols: 3',
        '  data: ' + mat([1., 0., 0., 0., 1., 0., 0., 0., 1.], 3),
        'projection_matrix:',
        '  rows: 3',
        '  cols: 4',
        '  data: ' + mat(projection, 4),
        '',
    ])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--frames', default='calib/c922_720p/frames')
    parser.add_argument('--out', required=True, help='camera_info YAML to write')
    parser.add_argument('--report', required=True, help='report.txt to write')
    parser.add_argument('--name', default='c922_720p')
    parser.add_argument('--squares', default='7x9')
    parser.add_argument('--square', type=float, required=True)
    parser.add_argument('--marker', type=float, required=True)
    parser.add_argument('--dict', default='4x4_250')
    args = parser.parse_args(argv)

    squares_x, squares_y = (int(v) for v in args.squares.lower().split('x'))
    cols, rows = squares_x - 1, squares_y - 1
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square

    dictionary = cs.aruco_dictionary(args.dict)
    board = cs.charuco_board(squares_x, squares_y, args.square, args.marker, dictionary)
    grids, used, skipped = load_frames(args.frames, cols, rows, dictionary, board)

    print(f'solve frames={len(grids)} confirmed of {len(grids) + len(skipped)}')
    for name, why in skipped:
        print(f'solve skipped {name}={why}')
    if len(grids) < 8:
        print('solve error=fewer than 8 confirmed frames; not enough to fit')
        return 1

    sample = cv2.imread(os.path.join(args.frames, used[0]))
    height, width = sample.shape[:2]

    rms, k, d = fit(grids, objp, (width, height))
    print(f'solve reproj_in_sample_px={rms:.4f}')
    print(f'solve fx={k[0, 0]:.4f} fy={k[1, 1]:.4f} cx={k[0, 2]:.4f} cy={k[1, 2]:.4f}')
    print('solve d=' + ','.join(f'{v:.6f}' for v in d))

    # **Held out, by two-fold split.** The number that can catch a fit which is
    # self-consistent and wrong: a set of square-on-only views reports 0.08 px in
    # sample for a focal length five times the truth, and 3.8 px out of sample.
    order = np.arange(len(grids))
    held = []
    for fit_idx, val_idx in ((order[0::2], order[1::2]), (order[1::2], order[0::2])):
        _, k_half, d_half = fit([grids[i] for i in fit_idx], objp, (width, height))
        held.append(cs.reprojection_rms([grids[i] for i in val_idx], k_half, d_half,
                                        args.square, (cols, rows)))
    print(f'solve reproj_held_out_px={max(held):.4f}')

    straight = cs.measure(grids, k, d)
    control = max(cs.worst_deviation(g) for g in grids)
    cover = cs.coverage(grids, k, width, height)
    print(f'solve straightness_px={straight["worst"]:.4f}')
    print(f'solve straightness_control_px={control:.4f}')
    print(f'solve coverage={cover["max_radius_frac"]:.3f} quadrants={cover["quadrants"]}')

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(ost_yaml(args.name, width, height, k, d))
    print(f'solve wrote={out}')

    report = pathlib.Path(args.report)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text('\n'.join([
        f'# {args.name} — intrinsics fitted by tools/calib_solve.py',
        f'# {datetime.datetime.now().isoformat(timespec="seconds")}',
        f'# square {args.square} m, marker {args.marker} m, board {args.squares} '
        f'squares, dict {args.dict}',
        f'# {len(grids)} marker-confirmed frames from {args.frames}',
        '#',
        '# Not cameracalibrator: that tool builds left/right stereo subscribers even',
        '# for a mono calibration and crashes on Lyrical in RcutilsLogger.warn before',
        '# reading a frame. See tools/calib_solve.py.',
        '',
        '## in sample — the frames this was fitted to',
        f'  reproj_rms_px={rms:.4f}',
        '',
        '## held out — two-fold split, the figure worth believing',
        f'  reproj_rms_px={max(held):.4f}',
        '',
        '## straightness on the same frames',
        f'  calibrated_px={straight["worst"]:.4f}',
        f'  control_px={control:.4f}',
        f'  coverage={cover["max_radius_frac"]:.3f} quadrants={cover["quadrants"]}',
        '',
        '## frames used',
        *(f'  {n}' for n in used),
        '',
    ]))
    print(f'solve report={report}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
