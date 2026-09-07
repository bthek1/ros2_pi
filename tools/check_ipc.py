#!/usr/bin/env python3
"""Assertions for the P2 intra-process gate.

The claim under test is the one the whole architecture is shaped around: inside
`pimesh_container`, a decoded frame is **passed**, not copied. `rclcpp` gives no
signal when that fails. `use_intra_process_comms=True` is a request, and it is
silently ignored when the publisher hands over anything but a `unique_ptr`, when
the QoS is `transient_local`, or when the two nodes turn out to be in different
processes. In all three cases the pipeline still works and the memory traffic
quietly triples.

So the evidence is the buffer address itself. `decode_node` logs the address it
published and `ipc_probe_node` logs the address it received; if they are equal,
no copy happened, because a copy would live somewhere else.

WHY THIS SCRIPT ALSO READS A LOG WITH INTRA-PROCESS TURNED OFF
--------------------------------------------------------------
An address comparison that has only ever been seen to pass is not evidence that
it can fail. Address reuse is real — the allocator hands the same block back
frame after frame, which is visible in any of these logs — so "the two addresses
matched" could in principle be a coincidence rather than a shared buffer.

The gate therefore runs the container a second time with `intra_process:=false`
and requires the addresses to DIFFER. That run is the control: it fails the way
a broken container would, which is what makes the passing run mean something.
"""

import argparse
import re
import sys

# One reader on the Wi-Fi link, and this is not a preference. Five RELIABLE
# subscribers each pull their own unicast copy from the Pi: the predecessor
# measured ~2 frames/s per reader against 14.7 Hz for a single one.
EXPECTED_SUBSCRIBERS = 1
# Enough matched frames that a single lucky address collision cannot carry the
# result. Ten is what the probe logs by default.
MIN_PAIRS = 5
# A floor on decode throughput, to catch a collapse rather than to police the
# camera. Measured 30.0 Hz at 1.9 ms mean on 2026-09-04, against a ~4 ms target.
MIN_DECODE_HZ = 20.0
MAX_DECODE_MEAN_MS = 8.0

PUBLISHED = re.compile(r"ipc published stamp=(\d+) buffer=(0x[0-9a-f]+)")
RECEIVED = re.compile(r"ipc received stamp=(\d+) buffer=(0x[0-9a-f]+)")


def stats_for(stage, echo_text):
    """Pull one stage's record out of `ros2 topic echo /pipeline/stats`.

    **/pipeline/stats is shared by every node in the pipeline**, keyed by the
    `stage` field, so `--once` on it returns whichever stage published first —
    which on 2026-09-04 was the camera, and this gate happily asserted the
    camera's rate and latency while calling them decode's. The numbers even
    looked plausible. Selecting by stage is the only way to read this topic.

    Returns the LAST matching record: the first window after startup is short
    and unrepresentative.
    """
    found = None
    for block in echo_text.split("---"):
        if re.search(rf"^stage:\s*{re.escape(stage)}\s*$", block, re.MULTILINE) is None:
            continue
        record = {}
        for field in ("rate_hz", "latency_ms", "latency_p95_ms",
                      "dropped_mailbox", "dropped_transport", "processed"):
            m = re.search(rf"^{field}:\s*([0-9.]+)\s*$", block, re.MULTILINE)
            if m:
                record[field] = float(m.group(1))
        if record:
            found = record
    return found


def pairs(log_text):
    """Match each published buffer with the received buffer for the same frame.

    Keyed on the capture stamp, not on order: the two log lines come from
    different threads and interleaving is not guaranteed. A stamp that appears
    on only one side is dropped rather than guessed at.
    """
    published = {m.group(1): m.group(2) for m in PUBLISHED.finditer(log_text)}
    received = {m.group(1): m.group(2) for m in RECEIVED.finditer(log_text)}
    return [
        (stamp, published[stamp], received[stamp])
        for stamp in sorted(published)
        if stamp in received
    ]


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log-on", required=True,
                   help="launch log from the run with intra_process:=true")
    p.add_argument("--log-off", required=True,
                   help="launch log from the CONTROL run with intra_process:=false")
    p.add_argument("--subscribers", type=int, required=True,
                   help="subscription count on /image_raw/compressed")
    p.add_argument("--stats", required=True,
                   help="captured `ros2 topic echo /pipeline/stats` output. The "
                        "topic carries every stage; this script selects decode's")
    a = p.parse_args()

    ok = True

    def check(good, message):
        nonlocal ok
        print(("  ok    " if good else "  FAIL  ") + message)
        ok = ok and good

    with open(a.log_on, encoding="utf-8", errors="replace") as f:
        on = pairs(f.read())
    with open(a.log_off, encoding="utf-8", errors="replace") as f:
        off = pairs(f.read())

    # --- the claim -----------------------------------------------------------
    check(
        len(on) >= MIN_PAIRS,
        f"{len(on)} frames logged on both sides (need {MIN_PAIRS}) — "
        f"fewer would mean the probe never received anything")

    shared = [s for s, pub, rec in on if pub == rec]
    if on:
        example = on[0]
        print(f"  info  first frame: published {example[1]}, received {example[2]}")
    check(
        len(on) > 0 and len(shared) == len(on),
        f"{len(shared)}/{len(on)} frames arrived at the SAME address — "
        f"a serialised path cannot do that")

    # --- the control ---------------------------------------------------------
    copied = [s for s, pub, rec in off if pub != rec]
    check(
        len(off) >= MIN_PAIRS,
        f"{len(off)} frames logged in the intra_process:=false control run")
    check(
        len(off) > 0 and len(copied) == len(off),
        f"{len(copied)}/{len(off)} control frames arrived at a DIFFERENT "
        f"address — the check is capable of failing")

    # --- the Wi-Fi constraint ------------------------------------------------
    check(
        a.subscribers == EXPECTED_SUBSCRIBERS,
        f"/image_raw/compressed has {a.subscribers} subscriber "
        f"(must be exactly {EXPECTED_SUBSCRIBERS}; each extra one pulls its own "
        f"unicast copy over the Pi's Wi-Fi)")

    # --- the stage keeps up --------------------------------------------------
    with open(a.stats, encoding="utf-8", errors="replace") as f:
        decode = stats_for("decode", f.read())

    if decode is None:
        check(False, "no decode record on /pipeline/stats — the stage never reported")
        return 1

    hz = decode.get("rate_hz", 0.0)
    ms = decode.get("latency_ms", 0.0)
    p95 = decode.get("latency_p95_ms", 0.0)
    failures = int(decode.get("dropped_transport", 0))

    check(
        hz >= MIN_DECODE_HZ,
        f"decode runs at {hz:.2f} Hz >= {MIN_DECODE_HZ:.0f} Hz floor")
    check(
        0.0 < ms <= MAX_DECODE_MEAN_MS,
        f"decode costs {ms:.2f} ms/frame mean, {p95:.2f} ms p95 "
        f"(limit {MAX_DECODE_MEAN_MS:.0f} ms)")
    # Undecodable frames are a Wi-Fi fact, not a bug — but a decode stage that
    # fails on everything would otherwise pass every check above it.
    check(failures == 0, f"{failures} undecodable frames")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
