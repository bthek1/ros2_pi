#!/usr/bin/env python3
"""Assertions for the P1 capture gate.

Kept out of the justfile because a recipe body ends at the first unindented
line, so a heredoc terminator at column 0 silently truncates the recipe — and
because assertions with thresholds in them deserve to be readable.

Every check prints the number it compared, passing or failing: a gate that only
prints "ok" tells you nothing when the hardware changes underneath it.

MEASURE EACH QUANTITY WHERE IT IS UNAMBIGUOUS
---------------------------------------------
Both of this gate's assertions were wrong on the first attempt in the same way:
they were measured on the dev box, where the number carries the Wi-Fi link and
the two machines' clocks as well as the thing being tested.

* **Rate.** `ros2 topic hz` on the dev box counts frames that *arrived*, so it
  charges the node for Wi-Fi loss. The node's own capture count is on
  `/pipeline/stats` from the Pi. Judged against raw v4l2 in the same run, that
  is the honest "does this node drop frames" test — it read 5.3% loss from the
  dev box and 2.4% from the Pi's own count for the same working node.
* **Stamp age.** `ros2 topic delay` reports `now() - header.stamp` on the
  subscriber's machine, so from the dev box it is transport + inter-machine
  clock offset + any stamp error. Only the last is this node's business, and
  the middle term moves on its own — 11 ms on one launch and 38 ms on the next
  while the Pi-side figure did not move at all.

So both assertions are made on the Pi, and the dev-box figures are reported
because they are what the rest of the pipeline actually lives with.
"""

import argparse
import sys

# The node's claim is that it loses nothing to its own code, so its capture rate
# is judged against raw v4l2 measured on the same camera in the same run — never
# against a number written down on another day. The C922's rate tracks the
# auto-exposure time: 29.7 fps and 58.8 fps were both measured on 2026-09-02.
MAX_LOSS_VS_HARDWARE_PCT = 5.0
# A floor on what reaches the dev box, to catch a collapse rather than a slow
# camera or a lossy minute of Wi-Fi.
MIN_DELIVERED_HZ = 25.0
# On the Pi, stamp-to-receipt is publish + local delivery and nothing else.
MAX_PI_DELAY_MS = 15.0
# And it must not MOVE between launches: a shift there means a per-process
# epoch, which is exactly the usb_cam 0.8.1 bug (measured 0.223 / 0.362 /
# 0.979 s on three launches — a different draw every time).
MAX_OFFSET_DRIFT_MS = 5.0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hw-fps", type=float, required=True,
                   help="raw v4l2 capture rate, measured this run with no ROS in the loop")
    p.add_argument("--node-rate", type=float, nargs=2, required=True, metavar=("RUN1", "RUN2"),
                   help="the node's own capture rate from /pipeline/stats, per launch")
    p.add_argument("--delivered-rate", type=float, nargs=2, required=True, metavar=("RUN1", "RUN2"),
                   help="rate received on the dev box, per launch")
    p.add_argument("--pi-delay", type=float, nargs=2, required=True, metavar=("RUN1", "RUN2"),
                   help="stamp-to-receipt measured ON THE PI, seconds, per launch")
    p.add_argument("--dev-delay", type=float, nargs=2, required=True, metavar=("RUN1", "RUN2"),
                   help="stamp-to-receipt measured on the dev box, seconds (reported only)")
    a = p.parse_args()

    ok = True

    def check(good, message):
        nonlocal ok
        print(("  ok    " if good else "  FAIL  ") + message)
        ok = ok and good

    node = max(a.node_rate)
    delivered = max(a.delivered_rate)
    pi1, pi2 = a.pi_delay
    dev1, dev2 = a.dev_delay

    check(
        node > 0 and delivered > 0 and pi1 >= 0 and pi2 >= 0,
        "both launches published and were measured on both machines")

    if a.hw_fps > 0:
        lost = (a.hw_fps - node) / a.hw_fps * 100.0
        check(
            lost <= MAX_LOSS_VS_HARDWARE_PCT,
            f"the node captures {node:.2f} Hz against a raw v4l2 ceiling of "
            f"{a.hw_fps:.2f} Hz — losing {lost:.2f}% (limit {MAX_LOSS_VS_HARDWARE_PCT:.0f}%)")
    else:
        check(False, "no raw v4l2 measurement to compare against")

    check(
        delivered >= MIN_DELIVERED_HZ,
        f"{delivered:.2f} Hz reaches the dev box >= {MIN_DELIVERED_HZ:.0f} Hz floor")

    check(
        pi1 * 1000.0 < MAX_PI_DELAY_MS and pi2 * 1000.0 < MAX_PI_DELAY_MS,
        f"on-Pi stamp-to-receipt {pi1 * 1000:.1f} / {pi2 * 1000:.1f} ms "
        f"< {MAX_PI_DELAY_MS:.0f} ms")

    drift_ms = abs(pi1 - pi2) * 1000.0
    check(
        drift_ms < MAX_OFFSET_DRIFT_MS,
        f"on-Pi offset stable across launches: {drift_ms:.2f} ms apart "
        f"(limit {MAX_OFFSET_DRIFT_MS:.0f} ms) — this is the usb_cam test")

    # Reported, deliberately not asserted: these carry the Wi-Fi link and the
    # offset between two machines' clocks as well as the frame.
    if node > 0:
        wifi_loss = (node - delivered) / node * 100.0
        print(f"  info  Wi-Fi delivers {delivered:.2f} of {node:.2f} Hz captured "
              f"({wifi_loss:.1f}% lost in transport; not asserted)")
    print(f"  info  dev-box stamp-to-receipt {dev1 * 1000:.1f} / {dev2 * 1000:.1f} ms "
          f"(transport + inter-machine clock offset; not asserted)")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
