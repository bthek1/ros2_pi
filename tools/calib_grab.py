#!/usr/bin/env python3
"""Save held-out chessboard frames from the Pi's stream, with live coverage.

These are the frames `tools/gates/calibration.sh` measures straightness on, and
the reason they are grabbed separately from the calibration is that a calibration
checked on its own training frames is checked in-sample. `cameracalibrator` keeps
the frames it solved from inside /tmp/calibrationdata.tar.gz; these are different
ones, so the gate's number is a held-out number.

Two things it does beyond writing files, and both come out of the same
measurement (see coverage() in calib_straightness.py and the sweep recorded in
test_straightness.py):

 1. **It only saves a frame when the board is actually found**, so a saved file is
    never one the gate will later skip.
 2. **It prints how much of the lens the saved frames have covered so far** and
    refuses to stop until that is enough. A board held politely in the middle of
    the frame produces an *uncalibrated* straightness of ~0.5 px — inside P9's
    1.0 px budget — which would make the gate pass without a calibration. So the
    person waving the board is told to go to the corners, by the same number the
    gate will assert on.

Subscribes to the compressed topic directly and writes the camera's own JPEG
bytes, which keeps this a one-reader path with no decode on the wire — the same
rule as everything else that touches this stream. It is a one-off tool, hence
Python; the pipeline's nodes are C++.
"""

import argparse
import os
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_straightness as cs  # noqa: E402


class Grabber(Node):

    def __init__(self, args):
        super().__init__('calib_grab')
        self.args = args
        self.out = pathlib.Path(args.out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.pattern = tuple(int(v) for v in args.size.lower().split('x'))  # (cols, rows)
        # Marker confirmation, which is what stops a silently wrong frame being
        # saved at all. findChessboardCorners can lock onto a lattice one square out
        # and report success; measured 2026-09-12, one frame in 35 did, and with it
        # in the set the calibration straightened nothing (4.16 px against a 4.17 px
        # control). Rejecting it and the frames the markers could not confirm took
        # the reprojection error from 0.768 px to 0.455 px. See confirm_grid.
        self.dictionary = cs.aruco_dictionary(args.dict)
        self.board = cs.charuco_board(
            self.pattern[0] + 1, self.pattern[1] + 1, args.square, args.marker, self.dictionary)
        self.rejected = {}
        # Counted separately, because "no board in any frame" and "a board in every
        # frame that was then refused" want completely different advice and the first
        # version of this script reported the former for both.
        self.detected = 0
        self.last_report = 0.0
        self.saved = []
        self.grids = []
        self.last_save = 0.0
        self.last_try = 0.0
        self.seen = 0
        self.k = None

        # RELIABLE KEEP_LAST(1), matching the writer. The freshest frame is the
        # only one worth having and a queue of stale ones would show the board
        # where it was a second ago, which is actively misleading when somebody is
        # moving it to a corner on this script's instructions.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(CompressedImage, args.topic, self.on_frame, qos)
        self.get_logger().info(
            f'saving up to {args.count} frames with a {self.pattern[0]}x{self.pattern[1]} '
            f'board to {self.out} — work the board out towards the frame corners, and '
            'TILT it (or the camera) 20-40 degrees; square-on views alone solve to a '
            'focal length several times the truth')

    def report(self, now, state):
        """Say what is happening, about once every three seconds.

        **A silent terminal is a bug.** The first version only logged on a save and on
        every 25th rejection of a given kind, so a session where nothing qualified
        printed one line and then nothing at all — indistinguishable from a hung node,
        and that is exactly how it was first seen. Rate-limited by time rather than by
        count so the cadence does not depend on the frame rate.
        """
        if now - self.last_report < 3.0:
            return
        self.last_report = now
        self.get_logger().info(
            f'{state}  |  saved {len(self.saved)}/{self.args.count}, '
            f'board seen in {self.detected} frames of {self.seen}')

    def on_frame(self, msg):
        self.seen += 1
        now = time.monotonic()

        # Two separate throttles, and conflating them was a mistake worth naming.
        #
        # `last_save` spaces the *saves*, because two frames 20 ms apart are the same
        # pose and would inflate the count without adding coverage. But gating the
        # decode on it means that while nothing is qualifying — which is the whole of a
        # session that is going badly — `last_save` stays old, the gate passes on every
        # frame, and a JPEG decode plus a chessboard *and* an ArUco detection run at the
        # full 47 Hz on one callback thread. `last_try` bounds that work to ~5 Hz, which
        # is far more often than a person can reposition a board.
        if now - self.last_try < 0.2:
            return
        self.last_try = now
        if now - self.last_save < self.args.interval:
            return

        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if image is None:
            return
        grid = cs.detect_corners(image, self.pattern)
        if grid is None:
            self.report(now, 'no board in shot')
            return
        self.detected += 1

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, why, oriented, _ = cs.confirm_grid(grey, grid, self.dictionary, self.board)
        if not ok:
            key = why.split('—')[0].split(',')[0].strip()
            self.rejected[key] = self.rejected.get(key, 0) + 1
            self.report(now, key)
            return
        # The marker-agreed ordering. findChessboardCorners cannot tell a symmetric
        # grid from the same grid read end to end, and letting that flip between
        # frames would corrupt the set as badly as a misregistration.
        grid = oriented

        height, width = image.shape[:2]
        if self.k is None:
            # No calibration is loaded here on purpose — this runs *before* there
            # is one. The image centre stands in for the principal point, which is
            # accurate to a few pixels on any real camera and is only used to
            # describe coverage.
            self.k = np.array([[1.0, 0, width / 2.0], [0, 1.0, height / 2.0], [0, 0, 1.0]])

        index = len(self.saved)
        path = self.out / f'frame-{index:02d}.jpg'
        # The camera's own JPEG bytes, unmodified. Re-encoding the decoded image
        # would put this tool's compression between the sensor and the gate's
        # sub-pixel corner detection.
        path.write_bytes(bytes(msg.data))
        self.saved.append(path)
        self.grids.append(grid)
        self.last_save = now

        cover = cs.coverage(self.grids, self.k, width, height)
        this = cs.coverage([grid], self.k, width, height)
        self.get_logger().info(
            f'saved {path.name}  this frame reaches {this["max_radius_frac"]:.2f} '
            f'of the way to a corner; over {len(self.saved)} frames: '
            f'{cover["max_radius_frac"]:.2f}, {cover["quadrants"]}/4 quadrants')

        if len(self.saved) >= self.args.count:
            if cover['max_radius_frac'] < self.args.min_coverage or cover['quadrants'] < 4:
                self.get_logger().warning(
                    f'{len(self.saved)} frames but coverage is '
                    f'{cover["max_radius_frac"]:.2f} over {cover["quadrants"]}/4 quadrants '
                    f'(want >= {self.args.min_coverage} and 4) — keep going, towards the '
                    'corners; the gate will reject this set')
                self.args.count += 1
                return
            raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default='calib/c922_720p/frames')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--size', default='6x8',
                        help='INTERIOR CORNERS as COLSxROWS (6x8 for our A4 sheet)')
    parser.add_argument('--square', type=float, required=True,
                        help='measured square size in metres')
    parser.add_argument('--marker', type=float, required=True,
                        help='measured ArUco marker size in metres')
    parser.add_argument('--dict', default='4x4_250', help='ArUco dictionary name')
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--interval', type=float, default=1.5,
                        help='minimum seconds between saved frames')
    parser.add_argument('--min-coverage', type=float, default=0.85,
                        help='the floor gates/calibration.sh asserts')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Grabber(args)
    status = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            # Which of the two failures it was, because they need opposite fixes.
            if node.detected == 0:
                node.get_logger().error(
                    f'saved nothing: {node.seen} frames arrived and the board was never '
                    f'detected in any of them. Is the whole board in shot (all four outer '
                    f'edges), and is --size {node.args.size} the INTERIOR CORNER count?')
            else:
                node.get_logger().error(
                    f'saved nothing: the board was detected in {node.detected} of '
                    f'{node.seen} frames but every one was refused — '
                    + '; '.join(f'{k} x{v}' for k, v in
                                sorted(node.rejected.items(), key=lambda kv: -kv[1]))
                    + '. MISREGISTERED in bulk means the markers and the chessboard '
                      f'grid disagree everywhere, which is a configuration problem rather '
                      f'than a bad frame: check --size {node.args.size} is the INTERIOR '
                      f'CORNER count the right way round, and that --squares/--marker/'
                      f'--dict match the printed sheet. A transposed --size detects fine '
                      f'and is wrong.')
            status = 1
        else:
            print(f'calib-grab saved={len(node.saved)} dir={node.out}')
        if node.rejected:
            print('calib-grab rejected: ' + '; '.join(
                f'{k} x{v}' for k, v in sorted(node.rejected.items(), key=lambda kv: -kv[1])))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
