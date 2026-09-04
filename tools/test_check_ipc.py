"""Tests for the P2 gate's assertions.

check_ipc.py is what decides whether the container is zero-copy, so it is
tested like anything else that can say "everything is fine". The cases that
matter are the ones where it must say NO: matching addresses in the control
run, a second subscriber on the Wi-Fi topic, and a probe that received nothing.
"""

import subprocess
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parent / "check_ipc.py"


def log(pairs, tag_published="published", tag_received="received"):
    """A launch log holding one published/received line per (stamp, pub, rec)."""
    lines = []
    for stamp, pub, rec in pairs:
        lines.append(f"[decode_node]: ipc {tag_published} stamp={stamp} buffer={pub}")
        lines.append(
            f"[ipc_probe_node]: ipc {tag_received} stamp={stamp} buffer={rec} 1280x720 bgr8")
    return "\n".join(lines) + "\n"


def shared(n, first=0):
    """n frames that arrived at the same address they were published at."""
    return [(str(1000 + i + first), f"0x7f00{i:04x}", f"0x7f00{i:04x}") for i in range(n)]


def copied(n):
    """n frames that were serialised: the addresses differ."""
    return [(str(2000 + i), f"0x7f00{i:04x}", f"0x7e00{i:04x}") for i in range(n)]


def run(tmp_path, on_pairs, off_pairs, subscribers=1, hz=30.0, ms=1.9, failures=0):
    on = tmp_path / "on.log"
    off = tmp_path / "off.log"
    on.write_text(log(on_pairs))
    off.write_text(log(off_pairs))
    return subprocess.run(
        [sys.executable, str(TOOL),
         "--log-on", str(on), "--log-off", str(off),
         "--subscribers", str(subscribers),
         "--decode-hz", str(hz), "--decode-ms", str(ms),
         "--decode-failures", str(failures)],
        capture_output=True, text=True)


def test_a_healthy_run_passes(tmp_path):
    r = run(tmp_path, shared(10), copied(10))
    assert r.returncode == 0, r.stdout
    assert "10/10 frames arrived at the SAME address" in r.stdout


def test_a_copied_frame_fails(tmp_path):
    # The failure this gate exists to catch: the container serialised.
    r = run(tmp_path, copied(10), copied(10))
    assert r.returncode != 0
    assert "0/10 frames arrived at the SAME address" in r.stdout


def test_one_copied_frame_among_nine_shared_still_fails(tmp_path):
    # No partial credit. A single serialised frame means the guarantee does not
    # hold, and averaging it away is how a regression gets shipped.
    r = run(tmp_path, shared(9) + [("9999", "0xaaaa", "0xbbbb")], copied(10))
    assert r.returncode != 0
    assert "9/10" in r.stdout


def test_a_control_run_that_also_shared_fails(tmp_path):
    # If the intra_process:=false run ALSO shows equal addresses, the check
    # cannot distinguish anything and the passing run means nothing.
    r = run(tmp_path, shared(10), shared(10))
    assert r.returncode != 0
    assert "capable of failing" in r.stdout


def test_a_probe_that_received_nothing_fails(tmp_path):
    # Zero pairs must not pass vacuously — "all zero frames were shared" is
    # true and worthless.
    r = run(tmp_path, [], copied(10))
    assert r.returncode != 0
    assert "0 frames logged on both sides" in r.stdout


def test_too_few_frames_fails(tmp_path):
    r = run(tmp_path, shared(3), copied(10))
    assert r.returncode != 0


def test_unmatched_stamps_are_not_paired(tmp_path):
    # A published stamp with no received line is dropped rather than guessed
    # at, so it cannot silently pair with the wrong frame.
    on = tmp_path / "on.log"
    off = tmp_path / "off.log"
    on.write_text(
        log(shared(6))
        + "[decode_node]: ipc published stamp=777 buffer=0xdeadbeef\n")
    off.write_text(log(copied(10)))
    r = subprocess.run(
        [sys.executable, str(TOOL), "--log-on", str(on), "--log-off", str(off),
         "--subscribers", "1", "--decode-hz", "30", "--decode-ms", "2",
         "--decode-failures", "0"],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stdout
    assert "6 frames logged on both sides" in r.stdout


def test_a_second_subscriber_on_the_wifi_topic_fails(tmp_path):
    # The constraint the whole architecture is shaped around. Two RELIABLE
    # readers roughly halve the Pi's delivered rate.
    r = run(tmp_path, shared(10), copied(10), subscribers=2)
    assert r.returncode != 0
    assert "has 2 subscriber" in r.stdout


def test_no_subscriber_at_all_fails(tmp_path):
    r = run(tmp_path, shared(10), copied(10), subscribers=0)
    assert r.returncode != 0


def test_a_collapsed_decode_rate_fails(tmp_path):
    r = run(tmp_path, shared(10), copied(10), hz=3.0)
    assert r.returncode != 0
    assert "3.00 Hz" in r.stdout


def test_a_slow_decode_fails(tmp_path):
    r = run(tmp_path, shared(10), copied(10), ms=25.0)
    assert r.returncode != 0


def test_a_missing_decode_measurement_fails_rather_than_passing_quietly(tmp_path):
    # 0.0 ms is what an absent measurement looks like, and it would otherwise
    # sail through a "<= 8 ms" comparison.
    r = run(tmp_path, shared(10), copied(10), ms=0.0)
    assert r.returncode != 0


def test_undecodable_frames_fail(tmp_path):
    r = run(tmp_path, shared(10), copied(10), failures=7)
    assert r.returncode != 0
    assert "7 undecodable frames" in r.stdout


def test_the_measured_2026_09_04_run_passes(tmp_path):
    # The real numbers from the first working run, so a future threshold change
    # has to be made against evidence rather than by nudging a constant.
    r = run(tmp_path, shared(10), copied(10), subscribers=1, hz=30.001, ms=1.868)
    assert r.returncode == 0, r.stdout


def test_it_reads_real_launch_log_lines(tmp_path):
    # The exact format the nodes emit, prefix and all — a log-parsing test that
    # only ever sees text the test itself invented proves nothing about the
    # regex it is meant to pin.
    real = (
        "[component_container_mt-4] [INFO] [1788516173.175429815] [decode_node]: "
        "ipc published stamp=1788516173164056088 buffer=0x7360d62b8010\n"
        "[component_container_mt-4] [INFO] [1788516173.175548687] [ipc_probe_node]: "
        "ipc received stamp=1788516173164056088 buffer=0x7360d62b8010 1280x720 bgr8\n"
    )
    sys.path.insert(0, str(TOOL.parent))
    import check_ipc

    matched = check_ipc.pairs(real)
    assert matched == [("1788516173164056088", "0x7360d62b8010", "0x7360d62b8010")]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
