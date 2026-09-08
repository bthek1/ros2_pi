#!/usr/bin/env python3
"""Assertions for the P4 depth gate.

Reads the `depth` record out of a captured `/pipeline/stats` stream and judges
the two things the phase promised: the session got the GPU, and a frame costs
what it was budgeted.

A CPU FALLBACK IS A FAILED TEST HOWEVER GOOD THE DEPTH LOOKS
------------------------------------------------------------
This is the whole point of the phase. ONNX Runtime will happily build a session
with no CUDA provider, run the model on the CPU, and produce depth maps that are
*correct* — just five times too slow. Measured on this box: 52.5 ms on the GPU
against the predecessor's 280-305 ms on the CPU. Nothing about the output
distinguishes them, so the provider is asserted directly and the timing is
asserted alongside it, because a session can hold the CUDA provider and still
have fallen back per-node.

WHY THE RATE IS NOT ASSERTED
----------------------------
Depth runs at whatever the GPU sustains — ~19 Hz against a camera delivering
42-60 — so most frames are dropped, on purpose, by a one-deep mailbox. A high
`dropped_mailbox` here is the design working, not a fault, and it is the exact
opposite of what it means at the keypoint stage. There is also no backlog to
grow behind a mailbox that is one deep. So the number is printed, and the claim
that matters is per-frame cost.

THE SAME FIXTURE CAVEAT AS P3
-----------------------------
`bags/desk1` was recorded over Wi-Fi while the camera was carried around a room,
so its instantaneous rate swings between 7 and 60 Hz. Any absolute rate measured
off a replay is a measurement of the bag — see tools/check_keypoints.py.
"""

import argparse
import os
import sys

# The stats parser is shared with the P3 checker rather than copied: both read
# the same topic, and two hand-rolled YAML readers that drift apart is a bug
# waiting for whichever gate is run less often.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_keypoints import parse_detail, parse_stats   # noqa: E402

# The plan's budget. Measured 55.9-56.7 ms here on 2026-09-08 — the model alone
# is 52.5 ms and the rest is preprocessing, the resize back up, and one memcpy.
MAX_MEAN_MS = 80.0
# The predecessor measured 280-305 ms for this model on this CPU. Anything in
# that neighbourhood is a fallback wearing a green tick, whatever the provider
# field claims.
CPU_SHAPED_MS = 150.0
REQUIRED_PROVIDER = "CUDAExecutionProvider"


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stats", required=True,
                   help="file holding a captured `ros2 topic echo --full-length "
                        "/pipeline/stats`")
    p.add_argument("--log", required=True,
                   help="the launch log, for the node's own startup line")
    p.add_argument("--min-samples", type=int, default=3)
    a = p.parse_args()

    ok = True

    def check(good, message):
        nonlocal ok
        print(("  ok    " if good else "  FAIL  ") + message)
        ok = ok and good

    with open(a.stats, encoding="utf-8", errors="replace") as handle:
        records = parse_stats(handle.read(), "depth")
    with open(a.log, encoding="utf-8", errors="replace") as handle:
        log = handle.read()

    # The startup line, asserted separately from the stats field. They come
    # from different places — one is what the node logged when it created the
    # session, the other what it reports every second — and a phase whose
    # entire risk is a silent fallback should not depend on a single source
    # for the answer.
    check(REQUIRED_PROVIDER in log,
          f"the startup log names {REQUIRED_PROVIDER}"
          + ("" if REQUIRED_PROVIDER in log else
             " — a CPU fallback is a FAILED test however good the depth looks"))

    if len(records) < a.min_samples:
        check(False,
              f"only {len(records)} depth record(s) on /pipeline/stats, need "
              f"{a.min_samples} — the stage never ran, or never published")
        return 1

    steady = records[1:] if len(records) > a.min_samples else records
    details = [parse_detail(r.get("detail", "")) for r in steady]
    last = details[-1]

    required = ("provider", "state", "side", "depth_scale", "max_depth_m")
    missing = [k for k in required if k not in last]
    check(not missing,
          "the stats `detail` field arrived complete"
          + (f" — MISSING {', '.join(missing)}; capture the echo with "
             f"--full-length" if missing else ""))

    check(last.get("provider") == REQUIRED_PROVIDER,
          f"the running session reports provider={last.get('provider', '?')}")
    check(last.get("state") == "ready",
          f"the model loaded and warmed (state={last.get('state', '?')})")

    # Only windows that actually did work: a window in which the replay had
    # not started yet reports 0.00 ms, and averaging those in would flatter
    # the number.
    means = [float(r["latency_ms"]) for r in steady if float(r["latency_ms"]) > 0.0]
    p95s = [float(r["latency_p95_ms"]) for r in steady if float(r["latency_ms"]) > 0.0]
    if not means:
        check(False, "no window did any inference at all")
        return 1

    mean_ms = sum(means) / len(means)
    worst_p95 = max(p95s)
    check(mean_ms <= MAX_MEAN_MS,
          f"mean per-frame cost {mean_ms:.1f} ms <= {MAX_MEAN_MS:.0f} ms budget "
          f"(node's own clock, never header.stamp)")
    check(mean_ms < CPU_SHAPED_MS,
          f"{mean_ms:.1f} ms is GPU-shaped (< {CPU_SHAPED_MS:.0f} ms; the CPU "
          f"path measured 280-305 ms for this model)")

    # .get, not indexing: the last record in the file is routinely half-written,
    # because the gate stops the echo with a signal.
    def span(field):
        return int(steady[-1].get(field, 0)) - int(steady[0].get(field, 0))

    processed = span("processed")
    dropped = span("dropped_mailbox")
    offered = processed + dropped
    print(f"  info  processed {processed} of {offered} frames offered "
          f"({100.0 * processed / offered if offered else 0:.0f}%); the rest were "
          f"overtaken in the one-deep mailbox. THIS IS THE DESIGN — depth is the "
          f"pipeline's clock and the camera is three times faster than it")
    print(f"  info  p95 per-frame cost {worst_p95:.1f} ms")
    print(f"  info  {last.get('side', '?')}x{last.get('side', '?')} input, "
          f"depth_scale={last.get('depth_scale', '?')} "
          f"max_depth_m={last.get('max_depth_m', '?')}")
    print(f"  info  depth_scale is ARBITRARY until P5 pins it with a tape "
          f"measure — every metre figure downstream is provisional")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
