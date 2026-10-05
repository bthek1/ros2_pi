#!/usr/bin/env python3
"""#13's P20 instrument: relocalised poses against motion capture.

    reloc_truth.py --groundtruth groundtruth.txt --saved saved.tum --log session.log

`--saved` is the saving session's keyframe trajectory (TUM, in its map frame) —
the same keyframes the map file holds. `--log` is the loading session's log; every
`relocalised ... pose=x,y,z,qx,qy,qz,qw` line is a pose that session claimed in the
*saved* map's frame.

**How a pose in somebody's map frame is scored.** The saved keyframes are aligned
onto ground truth by Sim(3) (Umeyama over positions) — that alignment *is* the saved
map's frame, expressed in motion capture's — and each relocalised position is
carried through the same alignment and compared with ground truth at its own stamp.
The scale matters: on fr1/desk this pipeline's metres were ~0.5 of TUM's when P11
measured at depth_scale 10.0, and are ~0.9 since P12 pinned it at 4.6 (fitted 1.13 on
2026-10-02) — an error read in the map's own units is off by whatever that is today.

**The relocalisation cannot be more accurate than the map it relocalised into**, so
the saved keyframes' own residual after alignment (`saved_ate_m`) is printed beside
it. A relocalisation error near that figure is the map's error, not the search's.

Prints `key=value` lines; -1 for anything that could not be measured.
"""

import argparse
import math
import re
import sys

import numpy as np

from place_truth import angle_deg, load_groundtruth, lookup, quat_to_matrix

RELOCALISED = re.compile(r'relocalised query_ns=(\d+) .*pose=([-\d.e,]+)')


def load_tum(path):
    """[(stamp_s, 3-vector, 3x3)] from a TUM file."""
    out = []
    with open(path) as fh:
        for line in fh:
            f = line.split()
            if not f or f[0].startswith('#') or len(f) != 8:
                continue
            v = [float(x) for x in f]
            out.append((v[0], np.array(v[1:4]), quat_to_matrix(*v[4:8])))
    return out


def parse_relocalisations(lines):
    """[(stamp_s, 3-vector, 3x3)] from `relocalised` log lines, in order.

    The stamp is integer nanoseconds in the log; it is divided once, here, and only
    for the nearest-neighbour lookup into ground truth at 100 Hz, where a double's
    ~200 ns of resolution at 1.3e9 s is five orders below the spacing.
    """
    out = []
    for line in lines:
        m = RELOCALISED.search(line)
        if not m:
            continue
        v = [float(x) for x in m.group(2).split(',')]
        if len(v) != 7:
            continue
        out.append((int(m.group(1)) / 1e9, np.array(v[:3]), quat_to_matrix(*v[3:7])))
    return out


def umeyama(source, target):
    """s, R, t minimising |target - (s R source + t)|^2, both 3xN."""
    mu_s = source.mean(axis=1, keepdims=True)
    mu_t = target.mean(axis=1, keepdims=True)
    xs, xt = source - mu_s, target - mu_t
    n = source.shape[1]
    cov = xt @ xs.T / n
    u, d, vt = np.linalg.svd(cov)
    sign = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sign[2, 2] = -1
    r = u @ sign @ vt
    var_s = (xs ** 2).sum() / n
    s = float(np.trace(np.diag(d) @ sign) / var_s)
    t = mu_t - s * r @ mu_s
    return s, r, t.ravel()


def score(gt_stamps, gt_poses, saved, relocalised):
    pairs = [(p, rot, lookup(gt_stamps, gt_poses, t)) for t, p, rot in saved]
    pairs = [x for x in pairs if x[2] is not None]
    result = {'saved_keyframes': len(saved), 'saved_associated': len(pairs),
              'relocalisations': len(relocalised), 'judged': 0, 'unjudged': 0,
              'scale': -1.0, 'saved_ate_m': -1.0, 'saved_rot_median_deg': -1.0,
              'error_first_m': -1.0, 'error_median_m': -1.0, 'error_max_m': -1.0,
              'rot_first_deg': -1.0, 'rot_median_deg': -1.0}
    if len(pairs) < 3:
        result['unjudged'] = len(relocalised)
        return result
    source = np.array([p for p, _, _ in pairs]).T
    target = np.array([g[:3, 3] for _, _, g in pairs]).T
    s, r, t = umeyama(source, target)
    aligned = s * r @ source + t[:, None]
    result['scale'] = s
    result['saved_ate_m'] = float(np.sqrt(((aligned - target) ** 2).sum(axis=0).mean()))
    # Rotation is reported, not trusted to be convention-free: the saved keyframes' own
    # residual is the floor any relocalised rotation can be read against.
    result['saved_rot_median_deg'] = float(np.median(
        [angle_deg(g[:3, :3].T @ (r @ rot)) for _, rot, g in pairs]))

    errors, rotations = [], []
    for stamp, p, rot in relocalised:
        g = lookup(gt_stamps, gt_poses, stamp)
        if g is None:
            result['unjudged'] += 1
            continue
        errors.append(float(np.linalg.norm(s * r @ p + t - g[:3, 3])))
        rotations.append(angle_deg(g[:3, :3].T @ (r @ rot)))
    result['judged'] = len(errors)
    if errors:
        result['error_first_m'] = errors[0]
        result['error_median_m'] = float(np.median(errors))
        result['error_max_m'] = max(errors)
        result['rot_first_deg'] = rotations[0]
        result['rot_median_deg'] = float(np.median(rotations))
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--groundtruth', required=True)
    ap.add_argument('--saved', required=True)
    ap.add_argument('--log', required=True)
    args = ap.parse_args(argv)
    gt_stamps, gt_poses = load_groundtruth(args.groundtruth)
    with open(args.log, errors='replace') as fh:
        relocalised = parse_relocalisations(fh)
    result = score(gt_stamps, gt_poses, load_tum(args.saved), relocalised)
    for key, value in result.items():
        if isinstance(value, float):
            print(f'{key}={value:.4f}' if math.isfinite(value) else f'{key}=-1')
        else:
            print(f'{key}={value}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
