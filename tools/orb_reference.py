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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='a bag directory (must contain metadata.yaml)')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--max-features', type=int, default=500)
    parser.add_argument('--match-window', type=int, default=10)
    parser.add_argument('--match-max-distance', type=int, default=64)
    # The same warm-up the C++ probe skips, and for the same reason: the first
    # frame can match nothing and the window takes ten frames to fill, so a mean
    # taken from frame zero is a measurement of the warm-up rather than of the
    # tracker.
    parser.add_argument('--warmup-frames', type=int, default=15)
    parser.add_argument('--limit', type=int, default=0, help='0 = the whole clip')
    args = parser.parse_args()

    if not pathlib.Path(args.bag, 'metadata.yaml').is_file():
        print(f'no bag at {args.bag} (looked for metadata.yaml)', file=sys.stderr)
        return 2

    # The same three parameters as config/pimesh.yaml, and the same WTA_K=2 the C++
    # side spells out: the 3 and 4 variants produce descriptors that Hamming
    # distance does not describe, so a matcher configured for Hamming on one of
    # those returns confident nonsense.
    orb = cv2.ORB_create(
        nfeatures=args.max_features, scaleFactor=1.2, nlevels=8, edgeThreshold=31,
        firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31,
        fastThreshold=20)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    window = deque(maxlen=args.match_window)
    next_id = 0
    fractions = []
    counts = []
    frames = 0

    for msg in read_compressed(args.bag, args.topic):
        frames += 1
        if args.limit and frames > args.limit:
            break

        buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        keypoints, descriptors = orb.detectAndCompute(gray, None)
        n = 0 if descriptors is None else len(keypoints)
        ids = [-1] * n
        is_new = [True] * n

        pooled = [d for d in window if d[0] is not None]
        if n and pooled:
            pooled_desc = np.vstack([d[0] for d in pooled])
            pooled_ids = [i for d in pooled for i in d[1]]

            knn = matcher.knnMatch(descriptors, pooled_desc, k=1)
            candidates = sorted(
                (m[0] for m in knn if m and m[0].distance <= args.match_max_distance),
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
                ids[i] = next_id
                next_id += 1

        window.append((descriptors, ids))

        if frames > args.warmup_frames and n:
            fractions.append(sum(1 for f in is_new if not f) / n)
            counts.append(n)

    if not fractions:
        print('ref frames=0', file=sys.stdout)
        print(f'no usable frames in {args.bag}', file=sys.stderr)
        return 1

    print(f'ref bag={args.bag}')
    print(f'ref frames={len(fractions)}')
    print(f'ref skipped_warmup={min(frames, args.warmup_frames)}')
    print(f'ref keypoints_mean={np.mean(counts):.1f}')
    print(f'ref matched_fraction={np.mean(fractions):.4f}')
    print(f'ref matched_p05={np.percentile(fractions, 5):.4f}')
    print(f'ref tracks={next_id}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
