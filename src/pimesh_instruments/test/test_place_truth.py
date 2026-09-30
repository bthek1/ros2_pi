"""tools/eval/place_truth.py decides gates/place.sh, so it needs tests of its own.

Every way this instrument can be wrong makes the detector look *better*: an
unjudged closure counted as true, a relative pose compared in the wrong
direction (which a symmetric test scene would never notice), or a precision of
1.000 printed over nothing. Each is pinned here with synthetic ground truth.
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'eval'))
import place_truth as pt  # noqa: E402


def yaw(a):
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def gt_from(poses_by_time):
    stamps = sorted(poses_by_time)
    return stamps, [poses_by_time[s] for s in stamps]


def rvec(r):
    """Rotation matrix -> Rodrigues vector."""
    a = pt.angle_deg(r)
    if a < 1e-9:
        return [0.0, 0.0, 0.0]
    th = math.radians(a)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(th))
    return list(v * th)


def query(t_q, t_c, est, accepted=1):
    r = rvec(est[:3, :3])
    return {'query_ns': int(t_q * 1e9), 'match_ns': int(t_c * 1e9), 'accepted': accepted,
            'inliers': 50, 'rx': r[0], 'ry': r[1], 'rz': r[2],
            'tx': est[0, 3], 'ty': est[1, 3], 'tz': est[2, 3]}


def asymmetric_scene():
    # Candidate at the origin; query turned 25 degrees and moved. Asymmetric on
    # purpose: a comparison made in the wrong direction (candidate_from_query) would
    # agree with the truth in a scene where the two are each other's inverse.
    w_c = pt.pose(np.eye(3), [0.0, 0.0, 0.0])
    w_q = pt.pose(yaw(0.44), [0.25, 0.05, 0.10])
    return w_c, w_q


def test_a_correct_closure_is_true_and_a_revisit_is_recalled():
    w_c, w_q = asymmetric_scene()
    stamps, poses = gt_from({1.0: w_c, 10.0: w_q})
    truth = np.linalg.inv(w_q) @ w_c
    qs = [query(1.0, 0.0, np.eye(4), accepted=0), query(10.0, 1.0, truth)]
    r = pt.judge(qs, stamps, poses, gap_s=3.0)
    assert (r['true'], r['false'], r['unjudged']) == (1, 0, 0)
    assert r['rot_err'][0] < 1e-6


def test_the_inverse_pose_is_a_false_closure():
    # The direction pin. `query_from_candidate` inverted is still a rigid pose with
    # the same angle, so a check on rotation *magnitude* alone would pass it.
    w_c, w_q = asymmetric_scene()
    stamps, poses = gt_from({1.0: w_c, 10.0: w_q})
    truth = np.linalg.inv(w_q) @ w_c
    qs = [query(1.0, 0.0, np.eye(4), accepted=0), query(10.0, 1.0, np.linalg.inv(truth))]
    r = pt.judge(qs, stamps, poses, gap_s=3.0)
    assert (r['true'], r['false']) == (0, 1)
    assert len(r['false_list']) == 1


def test_a_closure_without_ground_truth_is_unjudged_not_true():
    w_c, w_q = asymmetric_scene()
    stamps, poses = gt_from({1.0: w_c, 10.0: w_q})
    truth = np.linalg.inv(w_q) @ w_c
    # The query stamp is 0.5 s from any ground truth.
    qs = [query(10.5, 1.0, truth)]
    r = pt.judge(qs, stamps, poses, gap_s=3.0)
    assert (r['true'], r['false'], r['unjudged']) == (0, 0, 1)


def test_precision_over_nothing_prints_minus_one(tmp_path, capsys):
    gt = tmp_path / 'gt.txt'
    gt.write_text('# ts tx ty tz qx qy qz qw\n1.0 0 0 0 0 0 0 1\n')
    log = tmp_path / 'launch.log'
    log.write_text('nothing about places here\n')
    assert pt.main([str(log), str(gt)]) == 0
    out = capsys.readouterr().out
    assert 'precision=-1' in out
    assert 'recall=-1' in out
    assert 'queries=0' in out


def test_availability_respects_the_gap():
    # The same place two seconds apart is not a revisit available to a 3 s gap —
    # counting it would make recall look worse for a rule the detector obeys. (A
    # keyframe looking away sits between, so only the gap decides.)
    w = pt.pose(np.eye(3), [0, 0, 0])
    away = pt.pose(yaw(math.pi / 2), [0, 0, 0])
    stamps, poses = gt_from({1.0: w, 2.0: away, 3.0: w, 10.0: w})
    qs = [query(1.0, 0, np.eye(4), 0), query(2.0, 0, np.eye(4), 0), query(3.0, 0, np.eye(4), 0)]
    assert pt.judge(qs, stamps, poses, gap_s=3.0)['revisit_queries'] == 0
    qs.append(query(10.0, 0, np.eye(4), 0))
    assert pt.judge(qs, stamps, poses, gap_s=3.0)['revisit_queries'] == 1


def test_it_reads_the_line_odometry_node_writes():
    # The printf format and this parser are a language-boundary pair; this is the
    # line as the node formats it (test_gate_contract checks the key set).
    line = ('[odometry_node]: place query_ns=1305031452791720000 accepted=1 searched=12 '
            'too_recent=5 verified=3 match_ns=1305031449000000000 matches=80 '
            'with_landmark=60 inliers=44 rx=0.01000 ry=-0.20000 rz=0.00300 '
            'tx=0.1200 ty=-0.0100 tz=0.0500')
    (q,) = pt.parse_queries([line])
    assert q['query_ns'] == 1305031452791720000
    assert q['match_ns'] == 1305031449000000000
    assert q['accepted'] == 1 and q['inliers'] == 44
    assert q['ry'] == pytest.approx(-0.2) and q['tx'] == pytest.approx(0.12)


def test_a_neighbour_is_not_an_available_revisit_until_the_camera_leaves():
    # Two keyframes of one view with nothing between them: the tracker never lost
    # it, so no revisit is available. Put a keyframe looking 90 degrees away between
    # them and the same pair becomes a loop.
    w = pt.pose(np.eye(3), [0, 0, 0])
    away = pt.pose(yaw(math.pi / 2), [0, 0, 0])
    stamps, poses = gt_from({1.0: w, 5.0: away, 10.0: w})
    adjacent = [query(1.0, 0, np.eye(4), 0), query(10.0, 1.0, np.eye(4), 1)]
    r = pt.judge(adjacent, stamps, poses, gap_s=3.0)
    assert r['revisit_queries'] == 0
    assert (r['true'], r['loops']) == (1, 0), 'a correct match onto a neighbour, not a loop'

    looped = [query(1.0, 0, np.eye(4), 0), query(5.0, 0, np.eye(4), 0),
              query(10.0, 1.0, np.eye(4), 1)]
    r = pt.judge(looped, stamps, poses, gap_s=3.0)
    assert r['revisit_queries'] == 1
    assert (r['true'], r['loops'], r['recalled']) == (1, 1, 1)


def with_odom(q, odom):
    r = rvec(odom[:3, :3])
    q.update({'orx': r[0], 'ory': r[1], 'orz': r[2],
              'otx': odom[0, 3], 'oty': odom[1, 3], 'otz': odom[2, 3]})
    return q


def test_a_closure_onto_a_different_place_is_wrong_place_not_just_imprecise():
    # Two cameras 3 m apart cannot share a view. Counted as wrong_place — the failure
    # that destroys a pose graph — and as false, and never confused with a right
    # place posed a few degrees off.
    w_c = pt.pose(np.eye(3), [0, 0, 0])
    w_q = pt.pose(np.eye(3), [3.0, 0, 0])
    stamps, poses = gt_from({1.0: w_c, 10.0: w_q})
    r = pt.judge([query(1.0, 0, np.eye(4), 0), query(10.0, 1.0, np.eye(4))], stamps, poses, 3.0)
    assert (r['false'], r['wrong_place']) == (1, 1)

    near = pt.pose(yaw(math.radians(7)), [0.1, 0, 0])   # right place, 7 degrees off
    stamps, poses = gt_from({1.0: w_c, 10.0: near})
    r = pt.judge([query(1.0, 0, np.eye(4), 0), query(10.0, 1.0, np.eye(4))], stamps, poses, 3.0)
    assert (r['false'], r['wrong_place']) == (1, 0)


def test_beats_odom_compares_both_against_the_truth():
    # The closure is 1 degree off the truth, odometry 20: the closure wins. Swap them
    # and it loses, and is listed. A comparison of the two against each other rather
    # than against the truth would call both of these the same.
    w_c, w_q = asymmetric_scene()
    stamps, poses = gt_from({1.0: w_c, 10.0: w_q})
    truth = np.linalg.inv(w_q) @ w_c
    off1 = truth @ pt.pose(yaw(math.radians(1)), [0, 0, 0])
    off20 = truth @ pt.pose(yaw(math.radians(20)), [0, 0, 0])
    good = [query(1.0, 0, np.eye(4), 0), with_odom(query(10.0, 1.0, off1), off20)]
    r = pt.judge(good, stamps, poses, 3.0)
    assert (r['compared'], r['beats_odom'], len(r['worse_than_odom'])) == (1, 1, 0)
    bad = [query(1.0, 0, np.eye(4), 0), with_odom(query(10.0, 1.0, off20), off1)]
    r = pt.judge(bad, stamps, poses, 3.0)
    assert (r['compared'], r['beats_odom'], len(r['worse_than_odom'])) == (1, 0, 1)
