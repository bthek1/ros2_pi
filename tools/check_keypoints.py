#!/usr/bin/env python3
"""Assertions for the P3 keypoint gate.

Reads the `keypoints` record out of a captured `/pipeline/stats` stream and
judges the three numbers the phase promised: the stage keeps up with the
camera, one frame costs what it was budgeted, and the matching finds about as
much as the predecessor's did on the same clip.

SELECT ON `stage`, ALWAYS
-------------------------
`/pipeline/stats` is ONE topic shared by every node, keyed by the `stage`
field. A reader that takes the first record gets whichever node published
first — normally the camera — and then asserts the camera's rate and latency as
this stage's. That is not hypothetical: it is exactly what P2's gate did on its
first run, and it passed while measuring the wrong node.

MEASURE COST ON THE NODE'S OWN CLOCK
------------------------------------
`latency_ms` here is a steady_clock interval measured inside the worker thread
around the work itself. It is NOT derived from `header.stamp`, and it must not
be: stamps in this pipeline come from the Pi, so any figure computed from them
carries the Wi-Fi link and the offset between two machines' clocks as well as
the thing being measured.

A REPLAYED RATE IS THE BAG'S RATE, NOT THE NODE'S
-------------------------------------------------
The phase asked for ">= 30 Hz sustained", and the first version of this tool
asserted that on the minimum window rate off the replay. It failed at 8 Hz —
and decode, in the very same windows, read 10 Hz. The stage was processing 8
of the 10 frames it was given. The bag was recorded over Wi-Fi while the camera
was carried around a room, so its instantaneous rate swings between 10 and
60 Hz, and any absolute rate measured off it is a measurement of the FIXTURE.

So the node's own claim is asserted as a RATIO: keypoints must keep up with
whatever decode delivers, in the same window, both measured inside one process
on one clock. That comparison is unambiguous in a way an absolute Hz figure off
a replay can never be. The absolute rate is asserted only as a floor on the
fixture being usable at all, and is labelled as such. The capability claim
lives in `latency_ms`: 8 ms/frame is a 125 Hz ceiling.

WHAT IS ASSERTED AND WHAT IS ONLY PRINTED
-----------------------------------------
The pose-gate reject rate is PRINTED, not asserted. Until the camera has a
calibration, K is all zeros and the rotation estimator is switched off by
design — every frame rejects with `no_intrinsics`, and asserting on a rate that
is 100% for an honest reason would only encode the current gap as a
requirement. The reject BREAKDOWN is printed so the reason is never a mystery.
"""

import argparse
import re
import sys

# A floor on the FIXTURE, not on the node: a bag whose median rate is below
# this was recorded badly (the first desk1 take came out at 13.7 Hz with the
# C922's exposure_dynamic_framerate left on) and is not worth measuring
# against. Median, not minimum, because a Wi-Fi dropout mid-sweep is a property
# of the recording session and says nothing about any node.
MIN_FIXTURE_MEDIAN_HZ = 30.0
# The node's actual claim: it keeps up with whatever decode hands it. Both
# numbers come off /pipeline/stats from the same process in the same window.
MIN_KEEPUP_RATIO = 0.95
# And it must not be silently dropping the rest. The mailbox is one deep by
# design and newest-wins, so SOME loss is the design working — a collapse is
# not.
MIN_PROCESSED_FRACTION = 0.90
# Mean per-frame cost, and the real capability claim: 8 ms/frame is a 125 Hz
# ceiling. ORB at 500 features over 4 pyramid levels measured 7.3-7.9 ms live
# on 2026-09-07 with the preview still on the tracking thread, 6.8 ms once it
# moved to its own.
MAX_MEAN_MS = 8.0
# The predecessor held ~90% matched at 500 features with a 10-frame window
# (piros2, measured 2026-08-04). "Within 5 points" is the phase's wording.
PREDECESSOR_MATCHED = 0.90
MATCHED_TOLERANCE = 0.05


def parse_stats(text, stage):
    """Every record for one stage, as a list of dicts.

    `ros2 topic echo` emits YAML documents separated by `---`. Parsed by hand
    rather than with a YAML library so this tool has no dependency beyond the
    standard library — it runs from a justfile recipe, not from a package.
    """
    records, current = [], {}
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "---":
            if current:
                records.append(current)
            current = {}
            continue
        match = re.match(r"^(\w+):\s*(.*)$", stripped)
        if match:
            current[match.group(1)] = match.group(2).strip().strip("'\"")
    # Anything still in `current` never saw its `---` terminator, so the echo
    # was killed part-way through writing it. Dropping it is not tidiness: a
    # half-written record still carries `stage` and a TRUNCATED `detail`, and
    # reading the reject breakdown off it printed "uncalibrated ?" for a run
    # whose numbers were all present.
    return [r for r in records if r.get("stage") == stage]


def parse_detail(detail):
    """`detail` is free-form by design, but free-form and unparseable differ.

    keypoint_node writes it as key=value pairs for exactly this reason.
    """
    out = {}
    for token in detail.split():
        if "=" in token:
            key, _, value = token.partition("=")
            out[key] = value
    return out


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stats", required=True,
                   help="file holding a captured `ros2 topic echo /pipeline/stats`")
    p.add_argument("--min-samples", type=int, default=3,
                   help="reject a run too short to have reached a steady state")
    a = p.parse_args()

    ok = True

    def check(good, message):
        nonlocal ok
        print(("  ok    " if good else "  FAIL  ") + message)
        ok = ok and good

    with open(a.stats, encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    records = parse_stats(text, "keypoints")
    decode = parse_stats(text, "decode")

    if len(records) < a.min_samples:
        check(False,
              f"only {len(records)} keypoints record(s) on /pipeline/stats, "
              f"need {a.min_samples} — the stage never ran, or never published")
        return 1

    # Drop the first record: it covers the window in which the node was still
    # loading, and its rate is measured over a partial second.
    steady = records[1:] if len(records) > a.min_samples else records

    rates = [float(r["rate_hz"]) for r in steady]
    means = [float(r["latency_ms"]) for r in steady]
    p95s = [float(r["latency_p95_ms"]) for r in steady]
    details = [parse_detail(r.get("detail", "")) for r in steady]
    matched = [float(d.get("matched", "nan")) for d in details]

    mean_ms = sum(means) / len(means)
    worst_p95 = max(p95s)
    mean_matched = sum(matched) / len(matched)
    median_rate = sorted(rates)[len(rates) // 2]

    check(median_rate >= MIN_FIXTURE_MEDIAN_HZ,
          f"the FIXTURE delivers {median_rate:.1f} Hz median over {len(steady)} "
          f"windows >= {MIN_FIXTURE_MEDIAN_HZ:.0f} Hz — this judges the bag, "
          f"not the node")

    # The node's own claim. Compared window for window against decode, which
    # is the thing feeding it, rather than against an absolute number the
    # replay cannot honour.
    decode_total = sum(float(r["rate_hz"]) for r in decode[1:]) if len(decode) > 1 else 0.0
    keypoint_total = sum(rates)
    if decode_total > 0:
        ratio = keypoint_total / decode_total
        check(ratio >= MIN_KEEPUP_RATIO,
              f"keeps up with decode: {ratio * 100:.1f}% of the frames decode "
              f"delivered ({keypoint_total / len(rates):.1f} Hz vs "
              f"{decode_total / max(1, len(decode) - 1):.1f} Hz) "
              f">= {MIN_KEEPUP_RATIO * 100:.0f}%")
    else:
        check(False, "no decode record to compare against — did the container run?")

    check(mean_ms <= MAX_MEAN_MS,
          f"mean per-frame cost {mean_ms:.2f} ms <= {MAX_MEAN_MS:.1f} ms budget "
          f"(node's own clock, never header.stamp)")

    delta = abs(mean_matched - PREDECESSOR_MATCHED)
    check(delta <= MATCHED_TOLERANCE,
          f"matched fraction {mean_matched:.3f} is {delta * 100:.1f} points from "
          f"the predecessor's {PREDECESSOR_MATCHED:.2f} "
          f"(limit {MATCHED_TOLERANCE * 100:.0f})")

    # The mailbox is one deep and newest-wins, so a few drops are the design
    # working, not a fault. What would be a fault is most of the stream going
    # missing.
    processed = int(steady[-1]["processed"]) - int(steady[0]["processed"])
    backlog = [int(r["dropped_mailbox"]) for r in steady]
    dropped = backlog[-1] - backlog[0]
    offered = processed + dropped
    fraction = processed / offered if offered else 0.0
    check(fraction >= MIN_PROCESSED_FRACTION,
          f"processed {processed} of {offered} frames offered "
          f"({fraction * 100:.1f}%) >= {MIN_PROCESSED_FRACTION * 100:.0f}% — "
          f"{dropped} overtaken in the one-deep mailbox, which is the design")

    last = details[-1] if details else {}

    # `ros2 topic echo` elides any string past --truncate-length (128 by
    # default) with a trailing "...", so a `detail` that ran long arrives here
    # missing its last keys — and a missing key is indistinguishable from a
    # value the node never wrote. The phase promised this gate would PRINT the
    # pose-gate reject rate; if it cannot, that is a failure of the gate, not a
    # cosmetic one. The fix is `ros2 topic echo --full-length`.
    required = ("regime", "matched", "pose_ok", "pose_rej",
                "rej_few", "rej_resid", "rej_nointr")
    missing = [key for key in required if key not in last]
    check(not missing,
          f"the stats `detail` field arrived complete"
          + (f" — MISSING {', '.join(missing)}, which means `ros2 topic echo` "
             f"truncated it. Capture it with --full-length." if missing else ""))

    pose_ok = int(last.get("pose_ok", 0))
    pose_rej = int(last.get("pose_rej", 0))
    attempts = pose_ok + pose_rej
    reject_rate = (pose_rej / attempts * 100.0) if attempts else float("nan")
    print(f"  info  regime {last.get('regime', '?')}; pose gate rejected "
          f"{pose_rej}/{attempts} ({reject_rate:.1f}%) — "
          f"too_few_pairs {last.get('rej_few', '?')}, "
          f"residual {last.get('rej_resid', '?')}, "
          f"uncalibrated {last.get('rej_nointr', '?')} (printed, not asserted)")
    if last.get("regime") == "detect_only":
        print("  info  the camera is UNCALIBRATED (K all zeros), so the rotation "
              "estimator is off by design and every frame rejects. The geometry "
              "itself is covered by test_rotation.")
    print(f"  info  p95 per-frame cost {worst_p95:.2f} ms (reported: a single "
          f"slow frame is not a budget breach, a slow mean is)")
    print(f"  info  ORB {last.get('features', '?')} features over "
          f"{last.get('levels', '?')} pyramid levels, window "
          f"{last.get('window', '?')}; {last.get('previews', '?')} previews "
          f"drawn on their own thread")
    print(f"  info  window rates {min(rates):.0f}-{max(rates):.0f} Hz — the "
          f"bag's own profile (recorded over Wi-Fi while walking), not the "
          f"node's; see this tool's docstring")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
