#!/usr/bin/env python3
"""Pick the best calibration frames out of a recorded bag.

The alternative to `tools/calibrate.sh grab`, and the better one for anything but a
quick look. Record a minute or two of the board from varied angles, then choose the
frames offline. Three things that buys:

 1. **Selection becomes global rather than greedy.** The live grabber must decide,
    frame by frame, whether to keep what it is looking at; it cannot know that a
    better view of the same pose arrives four seconds later. Here every candidate is
    on the table at once, so the chosen set can be spread deliberately across board
    pose and image position instead of accumulated by luck.
 2. **The bag is a re-usable artefact.** A selection can be redone with different
    criteria, a different count, or a tighter obliquity bound, without going back to
    the wall. That is the same reason tools/calibrate.sh keeps the calibrator's
    tarball: a physical session nobody can repeat should leave something re-runnable.
 3. **It is a much easier thing for a person to do.** Move the camera slowly for
    ninety seconds. No negotiating with a tool that is refusing frames.

**It does not fix a board that is not flat.** Frame choice cannot undo a bulge in the
paper: a bowed target makes the solver invent pincushion distortion, and the tell is
`k1` coming out negative on a camera that has barrel. Mount the sheet on something
rigid *and flat* first — see docs/info/hardware.md#calibration-target.

Every frame is marker-confirmed on the way through, exactly as the grabber does it,
so a grid that is one square out or read the wrong way round never reaches the set.
"""

import argparse
import os
import pathlib
import sys
import warnings

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_straightness as cs  # noqa: E402

# Nominal intrinsics, used only to describe pose and coverage while choosing. The
# whole point is that the real ones are not known yet; an fx that is 7% out moves an
# obliquity estimate by a degree or so and changes no decision here.
NOMINAL_K = cs.NOMINAL_K


def read_bag(path, topic, stride):
    """Yield (index, jpeg_bytes) for every `stride`-th image message in the bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    # rosbag2 deprecates read_next() in favour of read_next_ext(), which returns the
    # send timestamp as well. The replacement does not exist on Jazzy, and tools/ is
    # rsynced to the Pi, so the old spelling is the one that works at both ends —
    # the same rule as get_package_share_path. Silenced rather than switched, so the
    # tool's key=value output is not interleaved with a stack line.
    warnings.filterwarnings('ignore', category=DeprecationWarning)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    seen = 0
    while reader.has_next():
        got_topic, data, _ = reader.read_next()
        if got_topic != topic:
            continue
        seen += 1
        if (seen - 1) % stride:
            continue
        yield seen - 1, bytes(deserialize_message(data, CompressedImage).data)


def describe(grid, objp, width, height):
    """Pose and placement of one board view, as the numbers selection spreads over.

    The four parameters are `cameracalibrator`'s own — where the board is in the
    frame, how big it is, and how skewed — because that heuristic is well tested and
    is what its X/Y/Size/Skew progress bars measure. Obliquity and sharpness are added
    because both were measured to matter here and neither is in that set.
    """
    points = grid.reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(objp, grid.reshape(-1, 1, 2), NOMINAL_K, np.zeros(5))
    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2]
    centre = rotation @ objp.mean(axis=0) + tvec.ravel()
    centre = centre / np.linalg.norm(centre)
    obliquity = float(np.degrees(np.arccos(min(1.0, abs(float(normal @ centre))))))

    rows, cols, _ = grid.shape
    across = np.linalg.norm(grid[0, -1] - grid[0, 0]) / (cols - 1)
    down = np.linalg.norm(grid[-1, 0] - grid[0, 0]) / (rows - 1)
    return {
        'x': float(points[:, 0].mean() / width),
        'y': float(points[:, 1].mean() / height),
        'size': float(np.sqrt(np.ptp(points[:, 0]) * np.ptp(points[:, 1])) /
                      np.sqrt(width * height)),
        'skew': float(min(across, down) / max(across, down)),
        'obliquity': obliquity,
        'distance_m': float(np.linalg.norm(tvec)),
    }


def sharpness(image, grid):
    """Laplacian variance over the board region only.

    Over the board only, because the background is a room whose clutter would
    dominate the number and has nothing to do with whether the corners are crisp.
    """
    grey = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    points = grid.reshape(-1, 2)
    x0, y0 = np.maximum(points.min(axis=0).astype(int), 0)
    x1, y1 = points.max(axis=0).astype(int)
    roi = grey[y0:y1, x0:x1]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0


def select(candidates, count, min_coverage, width, height):
    """Choose a spread-out subset, coverage first and then pose diversity.

    Two phases, because the gate asks for two different things and one of them is a
    hard floor:

      1. **Coverage.** Take whichever frame most increases how far the corners reach
         from the principal point, until the floor is met. Distortion is radial, so
         frames that never leave the middle of the picture cannot constrain it —
         measured, an uncalibrated board at half radius is already inside the
         straightness budget.
      2. **Diversity.** Fill the rest by max-min distance in the (x, y, size, skew)
         space, which is `cameracalibrator`'s parameterisation. Spreading here is what
         stops the set being twenty views of one pose, which is the degenerate case
         that solves to a focal length several times the truth.
    """
    chosen, remaining = [], list(range(len(candidates)))

    def coverage_of(indices):
        grids = [candidates[i]['grid'] for i in indices]
        return cs.coverage(grids, NOMINAL_K, width, height)

    while remaining and len(chosen) < count:
        current = coverage_of(chosen)['max_radius_frac'] if chosen else 0.0
        if not np.isnan(current) and current >= min_coverage:
            break
        best = max(remaining, key=lambda i: coverage_of(chosen + [i])['max_radius_frac'])
        gained = coverage_of(chosen + [best])['max_radius_frac']
        if chosen and gained <= current + 1e-6:
            break
        chosen.append(best)
        remaining.remove(best)

    keys = ('x', 'y', 'size', 'skew')
    while remaining and len(chosen) < count:
        def spread(i):
            a = np.array([candidates[i]['params'][k] for k in keys])
            return min(np.linalg.norm(a - np.array([candidates[j]['params'][k] for k in keys]))
                       for j in chosen) if chosen else 1.0
        best = max(remaining, key=spread)
        chosen.append(best)
        remaining.remove(best)
    return chosen


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--bag', required=True)
    parser.add_argument('--out', default='calib/c922_720p/frames')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--squares', default='7x9', help='board SQUARES as XxY')
    parser.add_argument('--square', type=float, required=True, help='metres, measured')
    parser.add_argument('--marker', type=float, required=True, help='metres, measured')
    parser.add_argument('--dict', default='4x4_250')
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--stride', type=int, default=3,
                        help='examine every Nth image; 3 is ~16 Hz on a 47 Hz stream')
    parser.add_argument('--min-coverage', type=float, default=0.85)
    parser.add_argument('--max-obliquity', type=float, default=45.0,
                        help='reject views more oblique than this; beyond ~50 deg the '
                             'board is foreshortened 2:1 and corner accuracy halves')
    args = parser.parse_args(argv)

    squares_x, squares_y = (int(v) for v in args.squares.lower().split('x'))
    cols, rows = squares_x - 1, squares_y - 1
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square

    dictionary = cs.aruco_dictionary(args.dict)
    board = cs.charuco_board(squares_x, squares_y, args.square, args.marker, dictionary)

    candidates, reasons, examined = [], {}, 0
    width = height = 0
    for index, payload in read_bag(args.bag, args.topic, args.stride):
        examined += 1
        image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            reasons['undecodable'] = reasons.get('undecodable', 0) + 1
            continue
        height, width = image.shape[:2]
        grid = cs.detect_corners(image, (cols, rows))
        if grid is None:
            reasons['no board'] = reasons.get('no board', 0) + 1
            continue
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, why, oriented, _ = cs.confirm_grid(grey, grid, dictionary, board)
        if not ok:
            key = why.split('—')[0].split(',')[0].strip()
            reasons[key] = reasons.get(key, 0) + 1
            continue
        params = describe(oriented, objp, width, height)
        if params['obliquity'] > args.max_obliquity:
            reasons[f'over {args.max_obliquity:.0f} deg oblique'] = \
                reasons.get(f'over {args.max_obliquity:.0f} deg oblique', 0) + 1
            continue
        candidates.append({'index': index, 'payload': payload, 'grid': oriented,
                           'params': params, 'sharpness': sharpness(grey, oriented)})

    print(f'select examined={examined}')
    print(f'select candidates={len(candidates)}')
    for key, n in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f'select rejected {key}={n}')
    if len(candidates) < 8:
        print('select error=fewer than 8 usable frames; is the whole board in shot?')
        return 1

    # Blur, judged against this bag rather than an absolute: exposure and lighting
    # move the Laplacian variance by more than motion does, so a fixed threshold
    # would be a threshold on the room.
    median_sharp = float(np.median([c['sharpness'] for c in candidates]))
    sharp = [c for c in candidates if c['sharpness'] >= 0.5 * median_sharp]
    print(f'select blurred_dropped={len(candidates) - len(sharp)} '
          f'(below half the median sharpness of {median_sharp:.0f})')

    picked = [sharp[i] for i in select(sharp, args.count, args.min_coverage, width, height)]
    grids = [c['grid'] for c in picked]
    cover = cs.coverage(grids, NOMINAL_K, width, height)
    obliquities = np.array([c['params']['obliquity'] for c in picked])
    distances = np.array([c['params']['distance_m'] for c in picked])

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob('*.jpg'):
        old.unlink()
    for n, candidate in enumerate(sorted(picked, key=lambda c: c['index'])):
        (out / f'frame-{n:02d}.jpg').write_bytes(candidate['payload'])

    print(f'select chosen={len(picked)} dir={out}')
    print(f'select coverage={cover["max_radius_frac"]:.3f} quadrants={cover["quadrants"]}')
    print(f'select obliquity_deg={obliquities.min():.1f}..{obliquities.max():.1f} '
          f'spread={np.ptp(obliquities):.1f}')
    print(f'select distance_cm={distances.min()*100:.0f}..{distances.max()*100:.0f} '
          f'spread={np.ptp(distances)*100:.0f}')
    if cover['max_radius_frac'] < args.min_coverage:
        print(f'select warning=coverage {cover["max_radius_frac"]:.3f} is under '
              f'{args.min_coverage}; the bag never took the board near a frame corner')
    if np.ptp(obliquities) < 15:
        print(f'select warning=obliquity spread is only {np.ptp(obliquities):.1f} deg; '
              'the bag needs the board tilted several different ways, not just moved')
    return 0


if __name__ == '__main__':
    sys.exit(main())
