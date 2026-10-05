#!/usr/bin/env python3
"""#13's P19 instrument: the tracking state against an injected blackout.

Reads `ros2 topic echo --csv /tracking/state` — one row per depth frame
odometry_node handled — and the first and last black stamps decode_node logged,
and prints `key=value` lines for tools/gates/lost.sh:

    frames             rows read
    blackout_frames    rows whose stamp lies inside the blackout
    state_before       the state of the last row before the blackout; -1 if none.
                       **to_lost means nothing unless this is OK** — a tracker
                       already LOST when the lens cap goes on reports LOST "in one
                       frame", which is how the first run of the gate passed.
    to_lost            depth frames from the blackout's first through the first LOST
                       at or after it, inclusive; -1 if LOST never came
    to_ok              depth frames after the blackout's last through the first OK
                       after it, inclusive; -1 if OK never came back
    outside            rows outside the blackout — "the un-blacked remainder"
    ok_fraction        OK rows / outside
    posed_fraction     rows whose pose came from a fit / outside
    lost_outside       LOST rows outside the blackout, recovery included
    unknown            rows whose state is neither OK nor LOST

**In Python and not awk, for one reason.** A ROS stamp in nanoseconds is ~1.8e18,
past the 2^53 where a double stops holding every integer, and awk's numbers are
doubles: two stamps 64 ns apart compare equal. The comparison this instrument
exists for is a stamp against a stamp, so it is done in integers.

**UNKNOWN counts as not OK.** A state nobody set and a state that is fine must not
have the same spelling, here any more than in fusion_node.
"""

import sys

UNKNOWN, OK, LOST = 0, 1, 2


def parse(lines):
    """[(stamp_ns, state, posed)] from `ros2 topic echo --csv` rows.

    The row is the message flattened: sec, nanosec, frame_id, state, posed, holds.
    Lines that are not rows — a warning echo printed, a blank — are skipped rather
    than guessed at; the gate puts a floor on `frames`.
    """
    rows = []
    for line in lines:
        fields = line.strip().split(',')
        if len(fields) < 6:
            continue
        try:
            stamp = int(fields[0]) * 1_000_000_000 + int(fields[1])
            state = int(fields[3])
        except ValueError:
            continue
        posed = fields[4].strip().lower() in ('true', '1')
        rows.append((stamp, state, posed))
    rows.sort(key=lambda r: r[0])
    return rows


def timeline(rows, first_black_ns, last_black_ns):
    """The numbers above, as a dict."""
    inside = [r for r in rows if first_black_ns <= r[0] <= last_black_ns]
    outside = [r for r in rows if not first_black_ns <= r[0] <= last_black_ns]

    to_lost = -1
    count = 0
    for stamp, state, _ in rows:
        if stamp < first_black_ns:
            continue
        count += 1
        if state == LOST:
            to_lost = count
            break

    to_ok = -1
    count = 0
    for stamp, state, _ in rows:
        if stamp <= last_black_ns:
            continue
        count += 1
        if state == OK:
            to_ok = count
            break

    before = [r for r in rows if r[0] < first_black_ns]
    n = len(outside)
    return {
        'frames': len(rows),
        'blackout_frames': len(inside),
        'state_before': before[-1][1] if before else -1,
        'to_lost': to_lost,
        'to_ok': to_ok,
        'outside': n,
        'ok_fraction': (sum(1 for r in outside if r[1] == OK) / n) if n else -1.0,
        'posed_fraction': (sum(1 for r in outside if r[2]) / n) if n else -1.0,
        'lost_outside': sum(1 for r in outside if r[1] == LOST),
        'unknown': sum(1 for r in rows if r[1] not in (OK, LOST)),
    }


def main(argv):
    if len(argv) != 4:
        print('usage: lost_timeline.py <echo.csv> <first_black_ns> <last_black_ns>',
              file=sys.stderr)
        return 2
    with open(argv[1]) as handle:
        rows = parse(handle)
    result = timeline(rows, int(argv[2]), int(argv[3]))
    for key, value in result.items():
        print(f'{key}={value:.4f}' if isinstance(value, float) else f'{key}={value}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
