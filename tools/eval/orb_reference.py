#!/usr/bin/env python3
"""The predecessor's ORB tracker, run offline over a bag.

**Why this exists.** P3's test asserts that `keypoint_node`'s matched-keypoint
fraction is within 5 points of the predecessor's on the same clip. The
predecessor is `~/Documents/piros2`, whose `keypoint_detector.py` did pooled
matching over a 10-frame window with a Hamming threshold of 64 — and it cannot be
run to produce that number any more: it was built against Jazzy on this box, and
this box is Lyrical. Its *algorithm* is the reference, not its process, and the
algorithm is forty lines of OpenCV with no ROS in it.

So this reads the bag directly with `rosbag2_py`, decodes each frame with the same
`cv2.imdecode` the C++ node uses, and runs `cv2.ORB` with the same three
parameters through the same pooled-window matching. It prints `ref <key>=<value>`
lines for `tools/gates/keypoints.sh` to read.

**What makes the comparison worth anything** is that this implementation shares no
code with the thing it is checking — a different language, a different binding of
the same library, written from the predecessor's description rather than from the
C++. A reimplementation that shared the matching code would agree with itself and
prove nothing. What it cannot catch is an error in the *shared* dependency: if
OpenCV's ORB is wrong, both are wrong together.

It is deliberately not fast. It has no frame rate to make and no deadline, so it
decodes and matches every frame in the clip, which is the one advantage it has
over the node: it sees frames the node was right to drop.

**The tracking is a class rather than a loop body, and that is what makes it
checkable.** Every way this can be wrong produces a number the gate will happily
assert on: drop the one-to-one claim and two features inherit one track, which
moves the matched fraction *up* and makes the comparison agree better; count a
featureless frame as a matched fraction of zero and the reference drifts about
five points from the node it is the reference for, which is the whole tolerance.
Neither shows up as a failure — they show up as a gate that passes for the wrong
reason. `src/pimesh_instruments/test/test_orb_reference.py` pins the properties with
synthetic descriptors, no bag and no camera, and pins the same ones
`test_orb_tracker` pins on the C++ side, which is the only way the two numbers
are comparable at all.
"""

import argparse
from collections import deque
import pathlib
import sys

import cv2
import numpy as np


def read_compressed(bag: str, topic: str):
    """Yield each CompressedImage payload in the bag, in recorded order.

    rosbag2_py gives raw CDR bytes; deserialising properly needs the typesupport,
    which is what `rosidl_runtime_py` wraps. Importing it here rather than at the
    top keeps the import error next to the thing that needs it.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    # read_next_ext() where it exists: Lyrical deprecates read_next() in favour of
    # it (two timestamps instead of one), Jazzy has only the old name. tools/ is
    # rsynced to the Pi, so even a script only the dev box runs is written to work
    # at both ends rather than to be correct on one of them.
    read = getattr(reader, 'read_next_ext', None) or reader.read_next
    while reader.has_next():
        record = read()
        yield deserialize_message(record[1], CompressedImage)


# The three defaults below are config/pimesh.yaml's, spelled once here and passed
# to argparse from these names so the CLI and the class cannot disagree.
MATCH_WINDOW = 10
MATCH_MAX_DISTANCE = 64
# The same warm-up the C++ probe skips, and for the same reason: the first frame
# can match nothing and the window takes ten frames to fill, so a mean taken from
# frame zero is a measurement of the warm-up rather than of the tracker.
WARMUP_FRAMES = 15


def orb_detector(max_features: int):
    """ORB configured as `keypoint_node` configures it.

    The same three parameters as config/pimesh.yaml, and the same WTA_K=2 the C++
    side spells out: the 3 and 4 variants produce descriptors that Hamming
    distance does not describe, so a matcher configured for Hamming on one of
    those returns confident nonsense.
    """
    return cv2.ORB_create(
        nfeatures=max_features, scaleFactor=1.2, nlevels=8, edgeThreshold=31,
        firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31,
        fastThreshold=20)


class PooledTracker:
    """The predecessor's pooled-window matching, and the bookkeeping around it.

    A class rather than a loop body so that a test can drive it with descriptors
    it chose — see this module's docstring for why that matters more here than it
    would in a node. It holds no image and no bag: `observe` takes descriptors and
    returns who each one turned out to be.

    Deliberately *not* shared with `pimesh_frontend/orb_tracker.hpp`. The gate
    compares the two, and two implementations sharing their matching code agree
    with each other whatever either of them does.
    """

    def __init__(
        self,
        match_window: int = MATCH_WINDOW,
        match_max_distance: int = MATCH_MAX_DISTANCE,
        warmup_frames: int = WARMUP_FRAMES,
    ):
        self.match_window = match_window
        self.match_max_distance = match_max_distance
        self.warmup_frames = warmup_frames

        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.window = deque(maxlen=match_window)
        self.next_id = 0

        #: Messages read, decodable or not — see `start_frame`.
        self.frames = 0
        #: Frames past the warm-up that had no features at all.
        self.empty = 0
        #: One matched fraction per frame past the warm-up that had features.
        self.fractions = []
        #: Feature count for those same frames.
        self.counts = []

    def start_frame(self) -> None:
        """One message off the bag, before anything has been decoded.

        Separate from `observe` because a frame that fails to decode never
        reaches the tracker and yet still consumes warm-up — which is this
        script's behaviour and is pinned as such rather than tidied, since the
        alternative silently shifts which seconds of a clip the mean covers.
        """
        self.frames += 1

    def observe(self, keypoints, descriptors):
        """Match one frame's descriptors against the window; return (ids, is_new).

        `descriptors` is None when ORB found nothing, which is not the same as a
        frame that did not arrive — see the empty-frame branch below.
        """
        n = 0 if descriptors is None else len(keypoints)
        ids = [-1] * n
        is_new = [True] * n

        pooled = [d for d in self.window if d[0] is not None]
        if n and pooled:
            pooled_desc = np.vstack([d[0] for d in pooled])
            pooled_ids = [i for d in pooled for i in d[1]]

            knn = self.matcher.knnMatch(descriptors, pooled_desc, k=1)
            candidates = sorted(
                (m[0] for m in knn if m and m[0].distance <= self.match_max_distance),
                key=lambda m: m.distance)

            # One-to-one, claimed by track: the same reasoning as the C++ side. A
            # plain nearest-neighbour pass is many-to-one, and two features
            # inheriting one track id is one track in two places at once.
            claimed = set()
            for m in candidates:
                if not is_new[m.queryIdx]:
                    continue
                track = pooled_ids[m.trainIdx]
                if track in claimed:
                    continue
                claimed.add(track)
                ids[m.queryIdx] = track
                is_new[m.queryIdx] = False

        for i in range(n):
            if is_new[i]:
                ids[i] = self.next_id
                self.next_id += 1

        self.window.append((descriptors, ids))

        if self.frames > self.warmup_frames:
            if n:
                self.fractions.append(sum(1 for f in is_new if not f) / n)
                self.counts.append(n)
            else:
                # A frame with no features has no matched fraction: 0/0 is
                # undefined, not zero. It is counted and excluded, which is
                # exactly what keypoint_probe does — the two have to count the
                # same way or the difference the gate asserts on is measuring
                # their conventions rather than the tracker. On bags/desk1 this
                # convention is worth about 5 points.
                self.empty += 1

        return ids, is_new


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='a bag directory (must contain metadata.yaml)')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--max-features', type=int, default=500)
    parser.add_argument('--match-window', type=int, default=MATCH_WINDOW)
    parser.add_argument('--match-max-distance', type=int, default=MATCH_MAX_DISTANCE)
    parser.add_argument('--warmup-frames', type=int, default=WARMUP_FRAMES)
    parser.add_argument('--limit', type=int, default=0, help='0 = the whole clip')
    args = parser.parse_args()

    if not pathlib.Path(args.bag, 'metadata.yaml').is_file():
        print(f'no bag at {args.bag} (looked for metadata.yaml)', file=sys.stderr)
        return 2

    orb = orb_detector(args.max_features)
    tracker = PooledTracker(
        match_window=args.match_window,
        match_max_distance=args.match_max_distance,
        warmup_frames=args.warmup_frames)

    for msg in read_compressed(args.bag, args.topic):
        tracker.start_frame()
        if args.limit and tracker.frames > args.limit:
            break

        buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        keypoints, descriptors = orb.detectAndCompute(gray, None)
        tracker.observe(keypoints, descriptors)

    if not tracker.fractions:
        print('ref frames=0', file=sys.stdout)
        print(f'no usable frames in {args.bag}', file=sys.stderr)
        return 1

    print(f'ref bag={args.bag}')
    print(f'ref frames={len(tracker.fractions)}')
    print(f'ref skipped_warmup={min(tracker.frames, args.warmup_frames)}')
    print(f'ref keypoints_mean={np.mean(tracker.counts):.1f}')
    print(f'ref matched_fraction={np.mean(tracker.fractions):.4f}')
    print(f'ref matched_p05={np.percentile(tracker.fractions, 5):.4f}')
    print(f'ref tracks={tracker.next_id}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
