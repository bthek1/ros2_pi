#!/usr/bin/env python3
"""Judge place recognition against motion capture (#12's P16).

odometry_node logs one `place query_ns=...` line per keyframe query. This reads
them beside a TUM `groundtruth.txt` and says, for every accepted closure, whether
the two keyframes really were the same place and whether the relative pose that
came with it is right; and, for every keyframe that *had* a revisit available,
whether it was found.

**Precision is the number the phase is about.** A false closure does not degrade
a pose graph, it destroys it, so this prints the false ones one by one.

Three outcomes for an accepted closure, and the third is not the first:

  true      ground truth at both stamps, relative rotation and translation agree
  false     ground truth at both stamps, and they do not
  unjudged  no ground truth within the tolerance of one of the stamps

An unjudged closure counted as a true one would be a precision of 1.000 over
closures nobody checked — an unmeasured value and a good one with one spelling.

TUM ground truth is the pose of the colour camera's optical centre in the
motion-capture world, which is the same frame `query_from_candidate` is between.

Usage: place_truth.py <launch.log> <groundtruth.txt> [--gap S]
Prints `key=value` lines; exits 0 unless it could not read its inputs.
"""

import argparse
import bisect
import math
import re
import sys

import numpy as np

LINE = re.compile(r'place query_ns=(-?\d+) (.*)$')
# How far a stamp may be from the nearest ground-truth sample. The capture runs
# at 100 Hz, so 20 ms is two samples.
GT_TOLERANCE_S = 0.02
# A closure is true when its relative pose agrees with the truth this well. The
# translation allows for the depth network's scale — a relative translation between
# two nearby keyframes is short, so a scale error of 20% is a few centimetres.
MAX_ROT_ERR_DEG = 5.0
MAX_TRANS_ERR_M = 0.20
# A closure onto a **different place**: by ground truth the two cameras were this
# far apart, or turned this far, so the views could not overlap. This is the failure
# that destroys a pose graph; a right place with an imprecise pose is a different,
# smaller failure, and the two are counted apart.
WRONG_PLACE_DIST_M = 1.0
WRONG_PLACE_ANGLE_DEG = 60.0
# A revisit is *available* to a query when some keyframe older than the gap was
# within this distance and viewing angle of it, by ground truth — **and the camera
# left in between**: some keyframe between the two was not near the query. Without
# that, the keyframe just before a query counts as a revisit it is available to,
# which is a view the tracker never lost and the detector rightly refuses.
REVISIT_DIST_M = 0.30
REVISIT_ANGLE_DEG = 20.0


def quat_to_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rodrigues(r):
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(r, dtype=float) / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * kx + (1 - math.cos(theta)) * kx @ kx


def angle_deg(r):
    c = (np.trace(r) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def pose(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def load_groundtruth(path):
    """Sorted stamps (s) and 4x4 world_from_camera poses."""
    stamps, poses = [], []
    with open(path) as fh:
        for line in fh:
            f = line.split()
            if not f or f[0].startswith('#') or len(f) != 8:
                continue
            v = [float(x) for x in f]
            stamps.append(v[0])
            poses.append(pose(quat_to_matrix(v[4], v[5], v[6], v[7]), v[1:4]))
    return stamps, poses


def lookup(stamps, poses, t):
    """The nearest ground-truth pose to t, or None if none is within tolerance."""
    i = bisect.bisect_left(stamps, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(stamps) and (best is None or abs(stamps[j] - t) < abs(stamps[best] - t)):
            best = j
    if best is None or abs(stamps[best] - t) > GT_TOLERANCE_S:
        return None
    return poses[best]


def parse_queries(lines):
    out = []
    for line in lines:
        m = LINE.search(line)
        if not m:
            continue
        q = {'query_ns': int(m.group(1))}
        for kv in m.group(2).split():
            if '=' in kv:
                k, v = kv.split('=', 1)
                q[k] = float(v) if ('.' in v or k.startswith(('r', 't'))) else int(v)
        out.append(q)
    return out


def near(w_a, w_b):
    d = np.linalg.norm(w_a[:3, 3] - w_b[:3, 3])
    return d < REVISIT_DIST_M and angle_deg(w_a[:3, :3].T @ w_b[:3, :3]) < REVISIT_ANGLE_DEG


def left_between(seen, i_c, w_q):
    """Did any keyframe after seen[i_c] (and before the query) look elsewhere?"""
    return any(w is not None and not near(w, w_q) for _, w in seen[i_c + 1:])


def judge(queries, stamps, poses, gap_s, epsilon_deg=0.0):
    """Everything the gate prints, as a dict."""
    res = {'queries': len(queries), 'accepted': 0, 'true': 0, 'false': 0, 'unjudged': 0,
           'revisit_queries': 0, 'recalled': 0, 'loops': 0,
           'wrong_place': 0, 'compared': 0, 'beats_odom': 0, 'odom_rot_err': [],
           'worse_than_odom': [], 'beyond_epsilon': 0, 'margins': [], 'rot_err': [], 'trans_err': [],
           'false_list': []}
    seen = []  # (stamp_s, world_from_camera or None) of every earlier keyframe
    for q in queries:
        t_q = q['query_ns'] * 1e-9
        w_q = lookup(stamps, poses, t_q)

        # Was a revisit available to this query, by ground truth?
        available = False
        if w_q is not None:
            for i_c, (t_c, w_c) in enumerate(seen):
                if t_c > t_q - gap_s or w_c is None:
                    continue
                if near(w_q, w_c) and left_between(seen, i_c, w_q):
                    available = True
                    break
        if available:
            res['revisit_queries'] += 1

        if q.get('accepted', 0) == 1:
            res['accepted'] += 1
            w_c = lookup(stamps, poses, q['match_ns'] * 1e-9)
            if w_q is None or w_c is None:
                res['unjudged'] += 1
            else:
                truth = np.linalg.inv(w_q) @ w_c  # query_from_candidate
                est = pose(rodrigues([q['rx'], q['ry'], q['rz']]), [q['tx'], q['ty'], q['tz']])
                rot_err = angle_deg(truth[:3, :3].T @ est[:3, :3])
                trans_err = float(np.linalg.norm(truth[:3, 3] - est[:3, 3]))
                res['rot_err'].append(rot_err)
                res['trans_err'].append(trans_err)
                true_dist = float(np.linalg.norm(w_q[:3, 3] - w_c[:3, 3]))
                if (true_dist > WRONG_PLACE_DIST_M or
                        angle_deg(w_q[:3, :3].T @ w_c[:3, :3]) > WRONG_PLACE_ANGLE_DEG):
                    res['wrong_place'] += 1
                # What P17 needs: the closure edge nearer the truth than the odometry
                # edge it would correct. Only when odometry's pose was logged.
                if 'orx' in q:
                    odom = pose(rodrigues([q['orx'], q['ory'], q['orz']]),
                                [q['otx'], q['oty'], q['otz']])
                    odom_rot_err = angle_deg(truth[:3, :3].T @ odom[:3, :3])
                    res['odom_rot_err'].append(odom_rot_err)
                    res['compared'] += 1
                    # The margin a closure loses by, negative when it wins. **Bounded,
                    # not forbidden** (#12's P16, decided 2026-10-02): a closure may lose
                    # to odometry by up to `epsilon_deg`, the spread of closure error
                    # itself, on the short pairs where odometry happens to be good.
                    res['margins'].append(rot_err - odom_rot_err)
                    if rot_err < odom_rot_err:
                        res['beats_odom'] += 1
                    else:
                        res['worse_than_odom'].append((q['query_ns'], rot_err, odom_rot_err))
                        if rot_err - odom_rot_err > epsilon_deg:
                            res['beyond_epsilon'] += 1
                if rot_err <= MAX_ROT_ERR_DEG and trans_err <= MAX_TRANS_ERR_M:
                    res['true'] += 1
                    # A loop, as opposed to a correct match onto a neighbour: the
                    # camera looked elsewhere between the two keyframes.
                    i_c = next((i for i, (t, _) in enumerate(seen)
                                if abs(t - q['match_ns'] * 1e-9) < 1e-6), None)
                    if i_c is not None and left_between(seen, i_c, w_q):
                        res['loops'] += 1
                    if available:
                        res['recalled'] += 1
                else:
                    res['false'] += 1
                    res['false_list'].append(
                        (q['query_ns'], q['match_ns'], q.get('inliers', -1), rot_err, trans_err,
                         float(np.linalg.norm(w_q[:3, 3] - w_c[:3, 3]))))
        seen.append((t_q, w_q))
    return res


def fmt(v):
    return '-1' if v is None else f'{v:.4f}'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('log')
    ap.add_argument('groundtruth')
    ap.add_argument('--epsilon-deg', type=float, default=0.0,
                    help='how far a closure may lose to odometry before it counts '
                         'against beyond_epsilon; 0 is the strict rule')
    ap.add_argument('--gap', type=float, default=3.0,
                    help="odometry_node's place_min_gap_s, so availability uses the same rule")
    args = ap.parse_args(argv)
    with open(args.log, errors='replace') as fh:
        queries = parse_queries(fh)
    stamps, poses = load_groundtruth(args.groundtruth)
    if not stamps:
        print('error=no ground truth read', file=sys.stderr)
        return 2
    r = judge(queries, stamps, poses, args.gap, args.epsilon_deg)
    judged = r['true'] + r['false']
    print(f"queries={r['queries']}")
    print(f"accepted={r['accepted']}")
    print(f"true={r['true']}")
    print(f"false={r['false']}")
    print(f"unjudged={r['unjudged']}")
    print(f"loops={r['loops']}")
    print(f"wrong_place={r['wrong_place']}")
    print(f"compared_with_odom={r['compared']}")
    print(f"beats_odom={r['beats_odom']}")
    print(f"beats_odom_fraction={fmt(r['beats_odom'] / r['compared'] if r['compared'] else None)}")
    print(f"epsilon_deg={args.epsilon_deg:.4f}")
    print(f"beyond_epsilon={r['beyond_epsilon']}")
    print(f"worst_margin_deg={fmt(max(r['margins']) if r['margins'] else None)}")
    print(f"odom_rot_err_median_deg={fmt(float(np.median(r['odom_rot_err'])) if r['odom_rot_err'] else None)}")
    # -1, never 1.000, when nothing was judged.
    print(f"precision={fmt(r['true'] / judged if judged else None)}")
    print(f"revisit_queries={r['revisit_queries']}")
    print(f"recalled={r['recalled']}")
    print(f"recall={fmt(r['recalled'] / r['revisit_queries'] if r['revisit_queries'] else None)}")
    print(f"rot_err_median_deg={fmt(float(np.median(r['rot_err'])) if r['rot_err'] else None)}")
    print(f"trans_err_median_m={fmt(float(np.median(r['trans_err'])) if r['trans_err'] else None)}")
    for qn, cn, inl, re_, te, d in r['false_list']:
        print(f'false_closure query_ns={qn} match_ns={cn} inliers={inl} '
              f'rot_err_deg={re_:.2f} trans_err_m={te:.3f} true_distance_m={d:.3f}')
    for qn, re_, oe in r['worse_than_odom']:
        print(f'worse_than_odom query_ns={qn} rot_err_deg={re_:.2f} odom_rot_err_deg={oe:.2f} '
              f'margin_deg={re_ - oe:.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
