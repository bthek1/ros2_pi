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
        self.saved = []
        self.grids = []
        self.last_save = 0.0
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

    def on_frame(self, msg):
        self.seen += 1
        now = time.monotonic()
        # Spaced in time, because two frames 20 ms apart are the same pose and
        # would inflate the frame count without adding any coverage.
        if now - self.last_save < self.args.interval:
            return

        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if image is None:
            return
        grid = cs.detect_corners(image, self.pattern)
        if grid is None:
            return

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
    parser.add_argument('--size', default='9x6')
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
            node.get_logger().error(
                f'saved nothing — {node.seen} frames arrived and the board was not found in '
                'any of them. Is it in shot, rigid, and the right --size?')
            status = 1
        else:
            print(f'calib-grab saved={len(node.saved)} dir={node.out}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
