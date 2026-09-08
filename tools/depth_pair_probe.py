#!/usr/bin/env python3
"""Check that /depth/rgb really is the frame /depth was inferred on.

WHY THIS EXISTS
---------------
`depth_node` publishes the depth map and, alongside it, the RGB frame the map
came from. The pair has to be *exact*: fusion at P5 unprojects colour through
depth, and a colour frame one or two frames out from its depth map paints the
wall behind an object onto the object.

The tempting alternative — a separate node republishing raw frames at camera
rate — cannot do this job, and the predecessor measured why. That node and the
depth node drop DIFFERENT frames (decode keeps ~14 Hz of the camera's 42-60,
inference keeps ~19), so their stamp sets rarely intersect; an exact-time
synchroniser downstream then limps at a couple of hertz with seconds of sawtooth
delay, and no queue depth fixes it. Sourcing the RGB from the frame actually
processed makes every pair exact by construction — and this probe is what turns
"by construction" into a measurement.

WHY IT IS PYTHON, AND A NODE
----------------------------
This repo is C++ for nodes. This is not a node in the pipeline: it is a test
instrument, and it has to sit OUTSIDE the container, because the claim is about
what a downstream consumer actually receives rather than about what the
publisher believes it sent. One-off tools may be Python (CLAUDE.md), and it runs
only from `just gate-depth`.

WHAT IT COMPARES
----------------
Both the compressed camera frame and `/depth/rgb` are subscribed. For each
`/depth/rgb`, the compressed frame carrying the SAME stamp is decoded and the
two byte buffers compared. Decoding here rather than trusting the node is the
point: it reproduces the pipeline's own decode independently, so a node that
published a *plausible* frame instead of the *right* one fails.
"""

import argparse
import os
import sys
from collections import OrderedDict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from depth_pairing import MIN_PAIRED, pairing

# RELIABLE to match every image publisher in this pipeline — BEST_EFFORT
# delivers zero megabyte-class frames once they fragment, and a mismatched
# profile is silently a topic with no subscribers.
#
# The DEPTH is 50 and not 1, and that difference is the whole reason the first
# version of this probe reported a false failure. The pipeline's publishers are
# KEEP_LAST(1) because a live reconstruction wants the freshest frame; a
# SUBSCRIBER with depth 1 additionally throws away anything that arrives while
# it is busy, and this probe is busy — it JPEG-decodes a 1280x720 frame per
# callback. It dropped independently on each topic, saw 9 depth and 11 rgb
# messages from different subsets, and reported the gap as "a depth message
# with no twin". That was a measurement of the probe.
BIG = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=50)

# Compressed frames arrive ~3x faster than depth maps and the depth map lags its
# input by one inference, so the frame a /depth/rgb refers to is a few frames
# back by the time it lands. Keep a short history rather than only the newest.
HISTORY = 120


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


class PairProbe(Node):

    def __init__(self, wanted):
        super().__init__('depth_pair_probe')
        self.wanted = wanted
        self.frames = OrderedDict()

        self.checked = 0
        self.identical = 0
        self.unmatched = 0        # no compressed frame with that stamp seen
        self.differing = []
        self.depth_stamps = set()
        self.rgb_stamps = set()

        self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.on_compressed, BIG)
        self.create_subscription(Image, '/depth/rgb', self.on_rgb, BIG)
        self.create_subscription(Image, '/depth', self.on_depth, BIG)

    def on_compressed(self, msg):
        self.frames[stamp_key(msg.header)] = bytes(msg.data)
        while len(self.frames) > HISTORY:
            self.frames.popitem(last=False)

    def on_depth(self, msg):
        self.depth_stamps.add(stamp_key(msg.header))

    def on_rgb(self, msg):
        key = stamp_key(msg.header)
        self.rgb_stamps.add(key)
        # Stamps keep being collected after the byte comparisons are done. The
        # comparison is capped because decoding a 1280x720 JPEG per frame is
        # expensive; the stamp bookkeeping is free, and stopping it early is
        # what made the first version report a false orphan.
        if self.checked >= self.wanted:
            return
        jpeg = self.frames.get(key)
        if jpeg is None:
            # Not a failure of the node: the probe simply never saw that
            # compressed frame (it is a separate RELIABLE KEEP_LAST(1)
            # subscription and can miss one). Counted separately so it can
            # never be mistaken for a mismatch.
            self.unmatched += 1
            return

        expected = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if expected is None:
            self.unmatched += 1
            return

        self.checked += 1
        got = np.frombuffer(bytes(msg.data), np.uint8)
        want = expected.reshape(-1)
        if got.size == want.size and bool(np.array_equal(got, want)):
            self.identical += 1
        else:
            differing = -1 if got.size != want.size else int(np.count_nonzero(got != want))
            self.differing.append(
                f'stamp {key[0]}.{key[1]:09d}: {got.size} bytes vs {want.size}, '
                f'{differing} differing')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames', type=int, default=10,
                   help='how many pairs to compare before reporting')
    p.add_argument('--timeout', type=float, default=30.0)
    a = p.parse_args()

    rclpy.init()
    node = PairProbe(a.frames)
    end = node.get_clock().now().nanoseconds + int(a.timeout * 1e9)
    # Runs for the WHOLE window, not until the comparisons are done. Stopping
    # at the tenth comparison truncated both stamp sets mid-stream and left the
    # boundary frame looking like a depth map with no twin — a measurement of
    # where the loop happened to exit, not of the node.
    try:
        while rclpy.ok():
            if node.get_clock().now().nanoseconds > end:
                break
            # A short timeout, not 0.1 s: spin_once services ONE callback per
            # call, and three topics deliver ~80 messages a second between them.
            rclpy.spin_once(node, timeout_sec=0.005)
    except KeyboardInterrupt:
        # The gate bounds this probe with `timeout -s INT` as a backstop. An
        # interrupted probe must still REPORT what it managed to compare — a
        # traceback where the numbers should be is a gate that failed to say
        # anything, which is worse than a gate that failed.
        print('  info  interrupted; reporting on what was collected so far')

    print(f'  info  {len(node.depth_stamps)} depth and {len(node.rgb_stamps)} '
          f'rgb messages seen; {len(node.depth_stamps & node.rgb_stamps)} '
          f'share a stamp')
    # Pairability, which is what P5's exact-time synchroniser will depend on.
    #
    # Asserted as a RATIO and not as "all of them", because this probe observes
    # from outside the container and its two subscriptions drop independently:
    # a stamp seen on one topic and not the other is at least as likely to be
    # the probe's own loss as the node's. A node that published depth without
    # its twin at all would score 0% and is what this catches. The exactness
    # claim is carried by the byte comparison below, which cannot be faked by
    # a lucky drop.
    seen, paired, ratio = pairing(node.depth_stamps, node.rgb_stamps)
    if seen == 0:
        print('  FAIL  no depth messages seen at all')
    elif ratio >= MIN_PAIRED:
        print(f'  ok    {paired}/{seen} depth messages ({ratio * 100:.0f}%) have '
              f'a /depth/rgb at the same stamp (>= {MIN_PAIRED * 100:.0f}%; the '
              f'shortfall is this probe dropping, not the node)')
    else:
        print(f'  FAIL  only {paired}/{seen} depth messages ({ratio * 100:.0f}%) '
              f'have a /depth/rgb at the same stamp')

    if node.checked == 0:
        print(f'  FAIL  compared 0 pairs (wanted {a.frames}) — '
              f'{node.unmatched} arrived with no compressed frame to check '
              f'against. Nothing was proved.')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f'  {"ok   " if node.identical == node.checked else "FAIL "} '
          f'{node.identical}/{node.checked} /depth/rgb frames are BYTE-IDENTICAL '
          f'to the camera frame at the same stamp '
          f'({node.unmatched} skipped, no compressed frame seen)')
    for line in node.differing[:5]:
        print(f'        {line}')

    ok = (node.identical == node.checked and seen > 0 and ratio >= MIN_PAIRED)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
