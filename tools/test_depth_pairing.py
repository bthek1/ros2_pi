"""Tests for the P4 probe's pairing arithmetic.

This is the logic that was wrong twice, both times because the number it
produced described the probe rather than the pipeline. These cases are written
as the two failures that actually happened, so that neither can come back
quietly.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from depth_pairing import MIN_PAIRED, pairing   # noqa: E402


def stamps(*ns):
    """Stamps as the probe holds them: (sec, nanosec) tuples."""
    return {(1788829000, n) for n in ns}


def test_a_perfectly_paired_stream_is_100_percent():
    seen, paired, ratio = pairing(stamps(1, 2, 3), stamps(1, 2, 3))
    assert (seen, paired) == (3, 3)
    assert ratio == 1.0


def test_a_node_publishing_depth_with_no_twin_scores_zero():
    # The failure this check exists for: /depth arriving with no /depth/rgb at
    # all. Fusion would then have depth it cannot colour.
    seen, paired, ratio = pairing(stamps(1, 2, 3), stamps(7, 8, 9))
    # None of 1..3 is inside [7, 9], so nothing is judged rather than
    # everything being failed — but the interior is empty, which the caller
    # treats as "nothing was proved".
    assert seen == 0
    assert ratio == 0.0


def test_an_overlapping_but_unpaired_stream_scores_zero():
    seen, paired, ratio = pairing(stamps(2, 4, 6), stamps(1, 7))
    assert seen == 3
    assert paired == 0
    assert ratio == 0.0


def test_the_boundary_frame_is_not_counted_as_an_orphan():
    # The SECOND bug. The probe stopped collecting at its tenth byte
    # comparison, so the newest depth map's twin had not arrived yet and it
    # looked orphaned. Judging only the interior removes it from the sample
    # instead of blaming the node for where the loop exited.
    depth = stamps(1, 2, 3, 4)
    rgb = stamps(1, 2, 3)          # the 4th rgb never arrived before the stop
    seen, paired, ratio = pairing(depth, rgb)
    assert seen == 3, "the trailing depth map must fall outside the window"
    assert paired == 3
    assert ratio == 1.0


def test_a_frame_before_the_probe_was_listening_is_not_counted():
    # The same effect at the other end: depth seen before the first rgb.
    depth = stamps(1, 5, 6)
    rgb = stamps(5, 6)
    seen, paired, ratio = pairing(depth, rgb)
    assert seen == 2
    assert ratio == 1.0


def test_an_interior_drop_still_counts_against_the_ratio():
    # And this is why the window is not simply "the intersection": a stamp
    # genuinely inside the observed range, with no twin, must still be
    # counted. Otherwise the check could never fail at all.
    depth = stamps(1, 2, 3, 4, 5)
    rgb = stamps(1, 2, 4, 5)       # 3 is missing from the middle
    seen, paired, ratio = pairing(depth, rgb)
    assert seen == 5
    assert paired == 4
    assert ratio == 0.8
    assert ratio < MIN_PAIRED, "an interior gap this large must fail the gate"


def test_no_rgb_at_all_proves_nothing_rather_than_passing():
    seen, paired, ratio = pairing(stamps(1, 2, 3), set())
    assert (seen, paired, ratio) == (0, 0, 0.0)


def test_no_depth_at_all_proves_nothing():
    seen, paired, ratio = pairing(set(), stamps(1, 2, 3))
    assert seen == 0
    assert ratio == 0.0


def test_stamps_are_ordered_by_seconds_then_nanoseconds():
    # Tuples, not floats: a (sec, nanosec) pair compares correctly across a
    # second boundary, where naive nanosecond arithmetic on 32-bit fields
    # would not.
    depth = {(10, 999_999_999), (11, 0), (11, 1)}
    rgb = {(10, 999_999_999), (11, 0), (11, 1)}
    seen, paired, ratio = pairing(depth, rgb)
    assert (seen, paired, ratio) == (3, 3, 1.0)


def test_a_realistic_gate_run_passes():
    # The shape of an actual run: 187 depth, 203 rgb, a handful lost to the
    # probe's own bounded queue.
    depth = {(1788829000, i) for i in range(0, 187)}
    rgb = {(1788829000, i) for i in range(0, 187) if i % 25 != 7}
    seen, paired, ratio = pairing(depth, rgb)
    assert ratio >= MIN_PAIRED
    assert seen > 180
