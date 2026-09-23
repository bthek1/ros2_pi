"""P3's outside opinion, checked against tracking whose answer is known.

`tools/gates/keypoints.sh` closes P3 by asserting that `keypoint_node`'s
matched-keypoint fraction is within 5 points of a reimplementation of the
predecessor's algorithm over the same clip — 0.9063 against 0.9065 on
`bags/desk1`. The comparison is the only part of that gate with an opinion from
outside this workspace, and it is worth exactly as much as the reimplementation
is right.

**Every way `tools/eval/orb_reference.py` can be wrong produces a number, and two of
them make the gate agree better rather than worse:**

- Drop the one-to-one claim and a plain nearest-neighbour pass is many-to-one:
  several features inherit the same track, the matched fraction rises, and the
  reference moves *towards* whatever the node reports. A tracker with one track
  in three places at once is indistinguishable, at the gate, from one that is
  working well.
- Count a featureless frame as a matched fraction of zero — 0/0 read as 0 — and
  the reference drifts about five points from the node, which is the whole
  tolerance. CLAUDE.md records this one as a measured false red that nothing in
  the output looked wrong in.
- Widen the Hamming threshold or the pooled window and the fraction rises again,
  for a reason that has nothing to do with the room.

None of those raise an exception, and none of them are visible in `ref
matched_fraction=0.87`. So the properties are pinned here with synthetic
descriptors, no bag, no camera and no clip — and they are deliberately the *same*
properties `test_orb_tracker` pins on the C++ side, because a reference
implementation that does not forgive churn, or does allow a track to be claimed
twice, is not measuring the same quantity as the thing it is the reference for,
however close the two numbers come out.

It lives in pimesh_bringup because that is where this workspace's Python tests
live, and it imports tools/eval/orb_reference.py by path for the reason
test_straightness imports its own subject that way: `tools/` is where this
project's analysis scripts live, and it is rsynced to the Pi, so this runs at
both ends as gates/test.sh requires.
"""

import importlib.util
import pathlib

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


def _load_module():
    """Import tools/eval/orb_reference.py by path.

    Three parents up from src/pimesh_bringup/test/ is the workspace root.
    Resolved from __file__ rather than from the cwd, because `colcon test` runs
    from the build tree. Importing the module also proves the top of the file has
    no ROS import in it — `rosbag2_py` is imported inside `read_compressed`, so
    the algorithm is reachable on a machine with no bag support at all.
    """
    root = pathlib.Path(__file__).resolve().parents[3]
    path = root / 'tools' / 'eval' / 'orb_reference.py'
    assert path.is_file(), f'{path} is missing — the gate has no reference'
    spec = importlib.util.spec_from_file_location('orb_reference', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


orb_reference = _load_module()

DESCRIPTOR_BYTES = 32  # ORB with WTA_K=2 is 256 bits, and the matcher assumes it


def descriptors(*rows):
    """Stack 32-byte rows the way `detectAndCompute` returns them.

    `cv2.BFMatcher` wants CV_8U with one descriptor per row; a float array or a
    single row of the wrong width is rejected by OpenCV rather than silently
    mismatched, which is why the tests can be this direct.
    """
    return np.array(rows, dtype=np.uint8).reshape(len(rows), DESCRIPTOR_BYTES)


def unrelated(count, seed=0):
    """`count` descriptors far enough apart that nothing matches anything.

    Random bytes sit around 128 bits apart — twice the threshold — so this is the
    "different room" case rather than a near miss. The seed is fixed because a
    test that fails one run in fifty is a test people learn to re-run.
    """
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(count, DESCRIPTOR_BYTES), dtype=np.uint8)


def flip_bits(descriptor, count):
    """Return `descriptor` with exactly `count` of its bits inverted.

    Hamming distance is over bits and not bytes, and that is the arithmetic the
    threshold is written in: `--match-max-distance 64` is 64 bits out of 256, a
    quarter of the descriptor. Flipping whole bytes would be off by a factor of
    eight and would make the boundary test below assert the wrong number.
    """
    out = descriptor.copy()
    for bit in range(count):
        out[bit // 8] ^= np.uint8(1 << (bit % 8))
    return out


def features(count):
    """Stand-ins for `count` keypoints.

    `observe` reads nothing but `len(keypoints)` — the positions belong to the
    node, which publishes them, and not to the reference, which only counts them.
    A tuple of ints says that plainly; `test_real_orb_output_drives_the_tracker`
    below is what checks the call shape against OpenCV's actual return.
    """
    return tuple(range(count))


def warmed(tracker):
    """Drive `tracker` past its warm-up with frames that have nothing in them.

    A featureless frame appends `(None, [])` to the window, which the pooled
    matcher filters out, so this leaves the tracker warm and its window empty —
    the state each test below wants to start from without its setup being part of
    what is measured.
    """
    for _ in range(tracker.warmup_frames):
        tracker.start_frame()
        tracker.observe(features(0), None)
    assert tracker.fractions == [], 'the warm-up recorded a measurement'
    return tracker


def feed(tracker, keypoints_and_descriptors):
    """One frame in, one (ids, is_new) out, with the message counter bumped."""
    tracker.start_frame()
    return tracker.observe(*keypoints_and_descriptors)


def frame(desc):
    return features(len(desc)), desc


# --- Matching ---------------------------------------------------------------


def test_a_feature_seen_again_inherits_its_track():
    """The base case, and the one every other property is a qualification of."""
    tracker = warmed(orb_reference.PooledTracker())
    first = unrelated(3, seed=1)

    ids_a, new_a = feed(tracker, frame(first))
    assert new_a == [True, True, True], 'a first sighting is not a match'

    ids_b, new_b = feed(tracker, frame(first.copy()))
    assert new_b == [False, False, False]
    assert ids_b == ids_a, 'the same corner came back with a different identity'
    # The seeding frame is past the warm-up too and is measured against an empty
    # window, so it contributes the 0.0 in front.
    assert tracker.fractions == [0.0, 1.0]


def test_unrelated_descriptors_are_all_new():
    """Half a descriptor apart is not a near miss, and must not be matched.

    Without this the suite could pass with a threshold of 256, which matches
    everything to everything and reports a fraction of 1.00 on any footage at
    all — including a camera pointed at a wall.
    """
    tracker = warmed(orb_reference.PooledTracker())
    feed(tracker, frame(unrelated(4, seed=2)))
    ids, is_new = feed(tracker, frame(unrelated(4, seed=3)))

    assert is_new == [True] * 4
    assert tracker.fractions[-1] == 0.0, 'a frame that matched nothing is a real zero'
    assert len(set(ids)) == 4, 'four first sightings must be four distinct tracks'


def test_the_distance_threshold_is_inclusive_and_is_in_bits():
    """Exactly `match_max_distance` matches; one bit further does not.

    Two separate things are asserted by the pair, and both are silent: that the
    comparison is `<=` rather than `<`, which moves the fraction by a hair, and
    that the threshold is counted in *bits*. A threshold read as bytes is eight
    times too tight and makes the reference disagree with the node everywhere,
    which reads as a broken node.
    """
    tracker = warmed(orb_reference.PooledTracker(match_max_distance=64))
    base = unrelated(1, seed=4)
    feed(tracker, frame(base))

    _, just_inside = feed(tracker, frame(descriptors(flip_bits(base[0], 64))))
    assert just_inside == [False], '64 bits is inside a threshold of 64'

    tracker = warmed(orb_reference.PooledTracker(match_max_distance=64))
    feed(tracker, frame(base))
    _, just_outside = feed(tracker, frame(descriptors(flip_bits(base[0], 65))))
    assert just_outside == [True], '65 bits is outside a threshold of 64'


def test_two_features_cannot_inherit_one_track():
    """The one-to-one claim, which is the property that fails *upwards*.

    Three features, two of them identical to the same corner the tracker already
    knows. A many-to-one nearest-neighbour pass gives both of those the same
    track id and reports two matches out of three; the correct answer is one out
    of three, because one track cannot be in two places at once.

    Note the direction: the wrong answer is the *higher* one. A reference that
    drops this agrees with the node more readily, so the gate goes green and the
    comparison stops meaning anything.
    """
    tracker = warmed(orb_reference.PooledTracker())
    known = unrelated(1, seed=5)
    feed(tracker, frame(known))

    twins = descriptors(known[0], known[0], unrelated(1, seed=6)[0])
    ids, is_new = feed(tracker, frame(twins))

    matched = [i for i, fresh in zip(ids, is_new) if not fresh]
    assert len(matched) == 1, f'{len(matched)} features claimed one track'
    assert tracker.fractions[-1] == pytest.approx(1 / 3)


def test_no_track_id_is_claimed_twice_in_one_frame():
    """The same rule stated over a whole frame rather than one duplicate pair.

    `test_two_features_cannot_inherit_one_track` pins the arithmetic on a case
    built to break it; this asserts the invariant over a frame with many
    duplicates, where a partial fix — a `claimed` set that is cleared in the
    wrong place, say — would still produce a plausible count.
    """
    tracker = warmed(orb_reference.PooledTracker())
    known = unrelated(3, seed=7)
    feed(tracker, frame(known))

    crowd = descriptors(*[known[i % 3] for i in range(9)])
    ids, is_new = feed(tracker, frame(crowd))

    matched = [i for i, fresh in zip(ids, is_new) if not fresh]
    assert len(matched) == len(set(matched)), 'a track was claimed twice'
    assert len(matched) == 3, 'three known corners should account for three matches'


def test_the_best_candidate_wins_the_track():
    """Candidates are consumed nearest-first, not in whatever order knnMatch returns.

    Two features compete for one track: an exact copy and something 60 bits away,
    both inside the threshold. The exact copy must win it. Without the sort the
    winner is an artefact of feature ordering, which is stable enough to look
    deliberate and has nothing to do with the image.
    """
    tracker = warmed(orb_reference.PooledTracker())
    known = unrelated(1, seed=8)
    feed(tracker, frame(known))

    # The near miss first, so that an unsorted pass would hand it the track.
    contenders = descriptors(flip_bits(known[0], 60), known[0])
    ids, is_new = feed(tracker, frame(contenders))

    assert is_new == [True, False], 'the worse candidate took the track'


# --- The window -------------------------------------------------------------


def test_the_window_forgives_a_gap_in_detection():
    """A corner ORB misses for a few frames keeps its identity when it returns.

    This is why the matching is pooled over a window at all rather than done
    frame to frame: ORB's detections churn, and a strict previous-frame matcher
    reports a collapse in the matched fraction that is an artefact of the
    detector rather than of the camera moving.
    """
    tracker = warmed(orb_reference.PooledTracker(match_window=10))
    corner = unrelated(1, seed=9)
    ids_first, _ = feed(tracker, frame(corner))

    for _ in range(5):
        feed(tracker, frame(unrelated(2, seed=10)))

    ids_again, is_new = feed(tracker, frame(corner.copy()))
    assert is_new == [False], 'a five-frame gap inside a ten-frame window broke the track'
    assert ids_again == ids_first


def test_the_window_forgets_past_its_length():
    """...and does not forgive a gap longer than the window.

    The complement of the test above, and the reason both are needed: a window
    that never evicts would match against the whole clip, which raises the
    fraction and quietly makes `match_window` do nothing.
    """
    tracker = warmed(orb_reference.PooledTracker(match_window=3))
    corner = unrelated(1, seed=11)
    feed(tracker, frame(corner))

    for _ in range(3):
        feed(tracker, frame(unrelated(2, seed=12)))

    _, is_new = feed(tracker, frame(corner.copy()))
    assert is_new == [True], 'the window kept a frame it should have evicted'


# --- What counts as a measurement -------------------------------------------


def test_a_featureless_frame_is_excluded_rather_than_scored_zero():
    """0/0 is undefined, not zero — worth about five points on bags/desk1.

    The two readings also mean different things about the room: "no corners in
    this part of it" and "corners found, none recognised" are opposite findings,
    and averaging the first into the second makes a tracker that is working look
    like one that is losing the room.

    The contrast is the point of the second half: a frame that *has* features and
    matches none of them is a real zero and must be recorded as one.
    """
    tracker = warmed(orb_reference.PooledTracker())

    feed(tracker, (features(0), None))
    assert tracker.empty == 1
    assert tracker.fractions == [], 'a frame with no features scored a fraction'
    assert tracker.counts == []

    feed(tracker, frame(unrelated(3, seed=13)))
    feed(tracker, frame(unrelated(3, seed=14)))
    assert tracker.fractions == [0.0, 0.0], 'features that matched nothing is a real zero'
    assert tracker.counts == [3, 3], 'a measured frame has to carry its feature count'
    assert tracker.empty == 1


def test_the_warmup_contributes_no_measurement():
    """The first `warmup_frames` frames fill the window and are not measured.

    The node's probe skips the same number. If the two disagree about it, the
    gate's 5-point tolerance is partly spent on the warm-up — a difference that
    grows as the clip gets shorter, which is exactly when somebody is iterating.
    """
    tracker = orb_reference.PooledTracker(warmup_frames=3)
    corner = unrelated(1, seed=15)

    for _ in range(3):
        feed(tracker, frame(corner))
    assert tracker.fractions == [], 'a warm-up frame was measured'
    assert tracker.next_id == 1, 'the warm-up still has to build the tracks'

    feed(tracker, frame(corner.copy()))
    assert tracker.fractions == [1.0], 'the first frame past the warm-up was not measured'


def test_a_frame_that_never_decoded_consumes_warmup_and_nothing_else():
    """`start_frame` without `observe`, which is what an unreadable frame is.

    Pinned rather than tidied: the counter that decides the warm-up counts
    messages read, so a corrupt frame shortens the warm-up by one instead of
    shifting the measured window later into the clip. Either convention is
    defensible; having the test and the script disagree about which one is in
    force is not, and a sweep is not uniform enough for the difference to be
    invisible — CLAUDE.md records 11 points moved by which seconds got measured.
    """
    tracker = orb_reference.PooledTracker(warmup_frames=2)
    corner = unrelated(1, seed=16)

    tracker.start_frame()          # arrived, did not decode, never observed
    assert tracker.frames == 1
    assert len(tracker.window) == 0, 'an undecodable frame entered the window'

    feed(tracker, frame(corner))   # frame 2: the last of the warm-up
    assert tracker.fractions == []
    feed(tracker, frame(corner.copy()))
    assert tracker.fractions == [1.0]


def test_the_matched_fraction_is_over_this_frame_s_features():
    """One match out of four is 0.25 — the denominator is the frame, not the window.

    A denominator taken from the pooled window instead would scale with how many
    frames were in it and would sit near a tenth of the right answer, which still
    looks like a number somebody could believe.
    """
    tracker = warmed(orb_reference.PooledTracker())
    known = unrelated(1, seed=17)
    feed(tracker, frame(known))

    mixed = descriptors(known[0], *unrelated(3, seed=18))
    feed(tracker, frame(mixed))

    assert tracker.fractions[-1] == pytest.approx(0.25)
    assert tracker.counts[-1] == 4


def test_tracks_counts_first_sightings_and_not_frames():
    """`ref tracks=` is the number of distinct corners the clip ever showed.

    A counter bumped per feature rather than per *new* feature grows at the frame
    rate and reads as a tracker that recognises nothing — the same symptom as a
    threshold that is too tight, and the gate cannot tell them apart.
    """
    tracker = warmed(orb_reference.PooledTracker())
    corner = unrelated(2, seed=19)

    feed(tracker, frame(corner))
    assert tracker.next_id == 2
    for _ in range(4):
        feed(tracker, frame(corner.copy()))
    assert tracker.next_id == 2, 'a re-sighting minted a new track'


# --- The call shape ---------------------------------------------------------


def test_real_orb_output_drives_the_tracker():
    """Drive it once with what `detectAndCompute` actually returns.

    Everything above hands `observe` a tuple and an array, which is what the
    arguments are used *as*. This is the one test that closes the loop on the
    real pair — that the detector this module configures produces descriptors the
    matcher it configures accepts, in that order. A WTA_K of 3 or 4, for
    instance, produces descriptors Hamming distance does not describe, and the
    result is confident nonsense rather than an error.
    """
    rng = np.random.default_rng(20)
    image = rng.integers(0, 256, size=(240, 320), dtype=np.uint8)

    orb = orb_reference.orb_detector(max_features=120)
    keypoints, desc = orb.detectAndCompute(image, None)
    assert desc is not None and len(keypoints) > 10, 'no corners in the noise field'
    assert desc.dtype == np.uint8 and desc.shape[1] == DESCRIPTOR_BYTES

    tracker = warmed(orb_reference.PooledTracker())
    feed(tracker, (keypoints, desc))
    ids, is_new = feed(tracker, (keypoints, desc.copy()))

    assert len(ids) == len(keypoints)
    assert is_new == [False] * len(keypoints), 'the same image did not match itself'
    assert tracker.fractions[-1] == 1.0
