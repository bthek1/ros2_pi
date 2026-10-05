"""tools/eval/reloc_truth.py decides gates/relocalise.sh, so it needs tests of its own.

The way this instrument flatters the relocaliser is by reading an error in the
wrong unit: the pipeline's metres were ~0.5 of TUM's on fr1/desk at P11's depth_scale,
so an error left in the map's own units was half the real one and passed a tolerance it
should have failed. (~0.9 since P12; the fixture keeps 0.5 because a large ratio is what
makes the mistake visible.)
The other is an alignment that quietly fits the relocalised poses too — a pose
included in its own reference has nothing to disagree with.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'eval'))
import reloc_truth as rt  # noqa: E402
from place_truth import pose  # noqa: E402


def yaw(a):
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def ground_truth(n=200, hz=100.0):
    """A camera sweeping a 2 m arc at 100 Hz: (stamps, 4x4 poses)."""
    stamps, poses = [], []
    for i in range(n):
        t = 1000.0 + i / hz
        a = 0.01 * i
        stamps.append(t)
        poses.append(pose(yaw(a), np.array([2.0 * math.sin(a), 0.1 * i / n, 2.0 * math.cos(a)])))
    return stamps, poses


# The map frame the saving session built, at S of TUM's scale — the pipeline's
# metres were ~0.5 of TUM's on fr1/desk (P11). So GT = (1/S) R p + T.
S, R, T = 0.5, yaw(0.7), np.array([0.3, -1.0, 2.0])


def into_map(g):
    """A ground-truth pose expressed in the saving session's map frame."""
    p = S * (R.T @ (g[:3, 3] - T))
    return p, R.T @ g[:3, :3]


def saved_from(stamps, poses, every=20):
    return [(stamps[i], *into_map(poses[i])) for i in range(0, len(stamps), every)]


def test_a_perfect_relocalisation_scores_zero_and_the_scale_is_recovered():
    stamps, poses = ground_truth()
    saved = saved_from(stamps, poses)
    reloc = [(stamps[i], *into_map(poses[i])) for i in (55, 130)]
    r = rt.score(stamps, poses, saved, reloc)
    assert abs(r['scale'] - 1 / S) < 1e-6   # map metres -> GT metres
    assert r['saved_ate_m'] < 1e-9
    assert r['judged'] == 2
    assert r['error_max_m'] < 1e-9
    # Not 1e-6: an angle off acos resolves only ~8e-7 deg near zero, and the Pi's
    # aarch64 rounding landed this one at 1.2e-6 — test_ground_truth's lesson after
    # milestone H, repeated here on 2026-10-02 and caught the same way.
    assert r['rot_median_deg'] < 1e-3


def test_the_error_is_in_ground_truth_metres_not_the_maps():
    # 0.1 map-metres off is 0.2 TUM metres at this scale. Read in the map's units it
    # would pass a 0.15 m tolerance it should fail.
    stamps, poses = ground_truth()
    saved = saved_from(stamps, poses)
    p, rot = into_map(poses[80])
    reloc = [(stamps[80], p + np.array([0.1, 0.0, 0.0]), rot)]
    r = rt.score(stamps, poses, saved, reloc)
    assert abs(r['error_first_m'] - 0.2) < 1e-6


def test_the_relocalised_poses_are_not_part_of_the_alignment():
    # A wildly wrong relocalisation must score wildly wrong, not drag the alignment
    # toward itself and come out looking moderate.
    stamps, poses = ground_truth()
    saved = saved_from(stamps, poses)
    p, rot = into_map(poses[80])
    reloc = [(stamps[80], p + np.array([5.0, 5.0, 5.0]), rot)]
    r = rt.score(stamps, poses, saved, reloc)
    assert r['saved_ate_m'] < 1e-9
    assert r['error_first_m'] > 15.0


def test_a_stamp_ground_truth_does_not_cover_is_unjudged_not_zero():
    stamps, poses = ground_truth()
    saved = saved_from(stamps, poses)
    p, rot = into_map(poses[10])
    r = rt.score(stamps, poses, saved, [(5000.0, p, rot)])
    assert r['unjudged'] == 1
    assert r['judged'] == 0
    assert r['error_max_m'] == -1.0


def test_too_few_saved_keyframes_judge_nothing():
    stamps, poses = ground_truth()
    saved = saved_from(stamps, poses)[:2]
    r = rt.score(stamps, poses, saved, [(stamps[5], *into_map(poses[5]))])
    assert r['scale'] == -1.0
    assert r['unjudged'] == 1


def test_the_log_line_is_parsed_as_the_node_writes_it():
    line = ('[odometry_node]: relocalised query_ns=1305031463000000000 candidate_ns=1 '
            'inliers=40 matches=60 searched=12 ms=31.0 lost_frames=4 '
            'pose=0.100000,-0.200000,1.500000,0.000000,0.000000,0.000000,1.000000\n')
    out = rt.parse_relocalisations(['noise\n', line])
    assert len(out) == 1
    stamp, p, rot = out[0]
    assert abs(stamp - 1305031463.0) < 1e-6
    assert np.allclose(p, [0.1, -0.2, 1.5])
    assert np.allclose(rot, np.eye(3))
