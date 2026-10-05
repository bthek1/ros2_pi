"""tools/eval/lost_timeline.py decides gates/lost.sh, so it needs tests of its own.

Three of the ways it can be wrong make the tracker look better than it is: an
UNKNOWN state counted as OK, a LOST from *before* the blackout credited to it, and
stamps compared as doubles — which at ~1.8e18 ns cannot tell two frames 64 ns
apart, so a frame on the blackout's boundary lands on whichever side rounding
puts it.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'eval'))
import lost_timeline as lt  # noqa: E402

BASE = 1_780_000_000_000_000_000   # a 2026 stamp in ns, past 2^53
STEP = 57_000_000                  # one depth frame at ~17.5 Hz


def rows(states, posed=None):
    """[(stamp, state, posed)] at one depth frame per entry."""
    posed = posed or [s == lt.OK for s in states]
    return [(BASE + i * STEP, s, p) for i, (s, p) in enumerate(zip(states, posed))]


def window(first_index, last_index):
    return BASE + first_index * STEP, BASE + last_index * STEP


def test_a_clean_blackout():
    # 10 OK, blackout over frames 10..19 with LOST from its fifth, 2 more LOST, then OK.
    states = [lt.OK] * 10 + [lt.OK] * 4 + [lt.LOST] * 6 + [lt.LOST] * 2 + [lt.OK] * 10
    posed = [True] * 10 + [False] * 10 + [True] * 12
    first, last = window(10, 19)
    t = lt.timeline(rows(states, posed), first, last)
    assert t['blackout_frames'] == 10
    assert t['to_lost'] == 5
    assert t['to_ok'] == 3
    assert t['outside'] == 22
    assert t['lost_outside'] == 2
    assert abs(t['ok_fraction'] - 20 / 22) < 1e-12


def test_a_lost_from_before_the_blackout_is_not_credited_to_it():
    # LOST once early and recovered; through the blackout it never goes LOST. A
    # search that started at the beginning of the run would find the early one and
    # report the blackout caught in 0 or negative frames.
    states = [lt.OK, lt.LOST, lt.LOST, lt.OK] + [lt.OK] * 6 + [lt.OK] * 5
    first, last = window(5, 9)
    t = lt.timeline(rows(states), first, last)
    assert t['to_lost'] == -1


def test_the_state_the_blackout_found_is_reported():
    # Measured on the gate's first run (2026-10-02): bags/desk1 loses track on its
    # own 17.5 s in, so a blackout at 20 s found the tracker already LOST and
    # to_lost came out 1 — a pass over a measurement of nothing. The gate refuses
    # unless the state before was OK, so it has to be reported.
    already = [lt.OK] * 3 + [lt.LOST] * 2 + [lt.LOST] * 5
    first, last = window(5, 9)
    t = lt.timeline(rows(already), first, last)
    assert t['to_lost'] == 1
    assert t['state_before'] == lt.LOST
    fine = [lt.OK] * 5 + [lt.OK] * 4 + [lt.LOST]
    assert lt.timeline(rows(fine), first, last)['state_before'] == lt.OK
    assert lt.timeline(rows(fine), BASE - STEP, last)['state_before'] == -1


def test_never_recovering_is_minus_one_not_zero():
    states = [lt.OK] * 5 + [lt.LOST] * 20
    first, last = window(5, 9)
    t = lt.timeline(rows(states), first, last)
    assert t['to_ok'] == -1
    assert t['ok_fraction'] < 0.5


def test_unknown_is_not_ok():
    states = [lt.UNKNOWN] * 10
    first, last = window(100, 110)
    t = lt.timeline(rows(states), first, last)
    assert t['unknown'] == 10
    assert t['ok_fraction'] == 0.0


def test_stamps_one_nanosecond_apart_are_told_apart():
    # The reason this is not awk: as doubles these two stamps are equal.
    assert float(BASE) == float(BASE + 1)
    data = [(BASE, lt.OK, True), (BASE + 1, lt.LOST, False)]
    t = lt.timeline(data, BASE + 1, BASE + 1)
    assert t['blackout_frames'] == 1
    assert t['outside'] == 1
    assert t['ok_fraction'] == 1.0


def test_the_echo_csv_is_read_as_the_message_flattened():
    lines = [
        '1780000000,57000000,base_link,1,True,0\n',
        'a warning line, not a row\n',
        '\n',
        '1780000000,114000000,base_link,2,False,5\n',
    ]
    parsed = lt.parse(lines)
    assert parsed == [
        (1_780_000_000_057_000_000, lt.OK, True),
        (1_780_000_000_114_000_000, lt.LOST, False),
    ]


def test_an_empty_run_reports_nothing_rather_than_a_fraction():
    t = lt.timeline([], BASE, BASE + STEP)
    assert t['frames'] == 0
    assert t['ok_fraction'] == -1.0
    assert t['to_lost'] == -1
