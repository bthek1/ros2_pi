"""Tests for the P3 gate's assertions.

check_keypoints.py is what decides whether the keypoint stage kept its
promises, so the cases that matter are the ones where it must say NO — a stage
that fell behind the camera, one that blew its budget, one whose matching
collapsed, and one whose backlog is growing. A gate that has only ever been
seen to pass is not evidence.

The one case with real history behind it is `stage`: `/pipeline/stats` is
shared by every node, and P2's gate passed while measuring the camera instead
of the node it was supposed to be judging.
"""

import subprocess
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parent / "check_keypoints.py"


def record(stage, rate, mean_ms, p95_ms=12.0, dropped=5, detail=None, processed=100):
    """One /pipeline/stats record as `ros2 topic echo` prints it."""
    if detail is None:
        detail = ("regime=rotation_only matched=0.905 features=500 levels=4 "
                  "window=10 pose_ok=90 pose_rej=10 rej_few=7 rej_resid=3 "
                  "rej_nointr=0")
    return (
        "header:\n"
        "  stamp:\n"
        "    sec: 1788771000\n"
        "    nanosec: 0\n"
        "  frame_id: camera_optical_frame\n"
        f"stage: {stage}\n"
        f"rate_hz: {rate}\n"
        f"latency_ms: {mean_ms}\n"
        f"latency_p95_ms: {p95_ms}\n"
        f"processed: {processed}\n"
        f"dropped_mailbox: {dropped}\n"
        "dropped_transport: 0\n"
        "stale: false\n"
        f"detail: {detail}\n"
        "---"
    )


def stats_file(tmp_path, records):
    path = tmp_path / "stats.txt"
    path.write_text("\n".join(records) + "\n", encoding="utf-8")
    return path


def run(path, *args):
    return subprocess.run(
        [sys.executable, str(TOOL), "--stats", str(path), *args],
        capture_output=True, text=True)


def healthy(n=6, decode_rate=59.0, processed_step=59, **kwargs):
    """A run that met every promise, with the decode records it is judged against.

    `processed` climbs so the tool can work out what fraction of the stream
    the stage actually got through, and decode is present because the node's
    real claim is a RATIO against it, not an absolute rate off a replay.
    """
    fields = {"rate": 59.0, "mean_ms": 7.5, "dropped": 5}
    fields.update(kwargs)
    out = []
    for i in range(n):
        out.append(record("decode", rate=decode_rate, mean_ms=1.9, processed=100 + i * 59))
        out.append(record("keypoints", processed=100 + i * processed_step, **fields))
    return out


@pytest.mark.parametrize("extra", [[], ["--min-samples", "3"]])
def test_healthy_run_passes(tmp_path, extra):
    result = run(stats_file(tmp_path, healthy()), *extra)
    assert result.returncode == 0, result.stdout
    assert "FAIL" not in result.stdout


def test_selects_the_keypoints_stage_not_whoever_published_first(tmp_path):
    # The trap that already caught P2's gate once. The camera publishes first
    # and looks healthy on rate while being wildly outside this stage's budget,
    # so a tool that reads the wrong record passes for the wrong reason.
    camera = [record("capture", rate=59.0, mean_ms=0.26,
                     detail="device=/dev/video0") for _ in range(6)]
    keypoints = healthy(mean_ms=25.0)     # this stage actually blew its budget
    result = run(stats_file(tmp_path, camera + keypoints))
    assert result.returncode == 1
    assert "25.00 ms" in result.stdout


def test_a_stage_that_never_published_fails_loudly(tmp_path):
    only_decode = [record("decode", rate=59.0, mean_ms=1.9) for _ in range(6)]
    result = run(stats_file(tmp_path, only_decode))
    assert result.returncode == 1
    assert "never ran" in result.stdout


def test_falling_behind_decode_fails(tmp_path):
    # The node's actual claim: it keeps up with whatever decode hands it.
    # Decode delivers 59 Hz, the stage manages 12 — that is the stage's fault
    # and no amount of fixture-blaming excuses it.
    result = run(stats_file(tmp_path, healthy(rate=12.0, decode_rate=59.0)))
    assert result.returncode == 1
    assert "keeps up with decode" in result.stdout


def test_a_slow_fixture_is_blamed_on_the_fixture_not_the_node(tmp_path):
    # The failure that sent the first version of this tool back to the drawing
    # board: the bag was recorded over Wi-Fi while walking and dips to 10 Hz.
    # The stage processing 9.6 of those 10 frames is working perfectly, and a
    # tool that reads that as a node failure is measuring the wrong thing.
    records = healthy(rate=9.6, decode_rate=10.0)
    result = run(stats_file(tmp_path, records))
    assert "FIXTURE" in result.stdout
    # The fixture floor fires; the node's own keep-up ratio does not.
    assert "FAIL  the FIXTURE" in result.stdout
    assert "  ok    keeps up with decode" in result.stdout


def test_a_missing_decode_record_fails_rather_than_passing_blind(tmp_path):
    only_keypoints = [record("keypoints", rate=59.0, mean_ms=7.5, processed=100 + i * 59)
                      for i in range(6)]
    result = run(stats_file(tmp_path, only_keypoints))
    assert result.returncode == 1
    assert "no decode record" in result.stdout


def test_blowing_the_per_frame_budget_fails(tmp_path):
    result = run(stats_file(tmp_path, healthy(mean_ms=11.0)))
    assert result.returncode == 1
    assert "11.00 ms" in result.stdout


def test_a_high_p95_alone_does_not_fail(tmp_path):
    # Reported, not asserted. One slow frame in a hundred is a scheduler
    # hiccup; a slow mean is a stage that does not fit its budget.
    result = run(stats_file(tmp_path, healthy(mean_ms=7.5, p95_ms=40.0)))
    assert result.returncode == 0
    assert "p95 per-frame cost 40.00 ms" in result.stdout


def test_matching_collapse_fails(tmp_path):
    collapsed = ("regime=rotation_only matched=0.310 features=500 levels=4 "
                 "window=10 pose_ok=0 pose_rej=100 rej_few=100 rej_resid=0 "
                 "rej_nointr=0")
    result = run(stats_file(tmp_path, healthy(detail=collapsed)))
    assert result.returncode == 1
    assert "0.310" in result.stdout


def test_matching_far_above_the_predecessor_also_fails(tmp_path):
    # "Within 5 points" is two-sided on purpose. A matched fraction of 100%
    # does not mean the tracker got better; it means the threshold stopped
    # rejecting anything, which is a broken matcher wearing a good number.
    perfect = ("regime=rotation_only matched=1.000 features=500 levels=4 "
               "window=10 pose_ok=100 pose_rej=0 rej_few=0 rej_resid=0 "
               "rej_nointr=0")
    result = run(stats_file(tmp_path, healthy(detail=perfect)))
    assert result.returncode == 1


def test_dropping_most_of_the_stream_fails(tmp_path):
    # Some loss is the one-deep mailbox working as designed. Losing most of
    # the stream is not, and no per-frame cost number would show it.
    records = []
    for i in range(6):
        records.append(record("decode", rate=59.0, mean_ms=1.9))
        records.append(record("keypoints", rate=59.0, mean_ms=7.5,
                              processed=100 + i * 10, dropped=i * 100))
    result = run(stats_file(tmp_path, records))
    assert result.returncode == 1
    assert "%" in result.stdout


def test_a_handful_of_overtaken_frames_is_the_design_working(tmp_path):
    # The mailbox is one deep and newest-wins on purpose: a frame the camera
    # took 400 ms ago has no value once a newer one exists.
    result = run(stats_file(tmp_path, healthy()))
    assert result.returncode == 0
    assert "which is the design" in result.stdout


def test_uncalibrated_is_reported_not_failed(tmp_path):
    # K all zeros switches the rotation estimator off by design, so every
    # frame rejects. Asserting on that would encode today's gap as tomorrow's
    # requirement.
    uncal = ("regime=detect_only matched=0.905 features=500 levels=4 "
             "window=10 pose_ok=0 pose_rej=1559 rej_few=0 rej_resid=0 "
             "rej_nointr=1559")
    result = run(stats_file(tmp_path, healthy(detail=uncal)))
    assert result.returncode == 0
    assert "UNCALIBRATED" in result.stdout
    assert "100.0%" in result.stdout


def test_the_reject_breakdown_is_printed(tmp_path):
    result = run(stats_file(tmp_path, healthy()))
    assert "too_few_pairs 7" in result.stdout
    assert "residual 3" in result.stdout


def test_a_truncated_run_fails_rather_than_guessing(tmp_path):
    result = run(stats_file(tmp_path, healthy(2)))
    assert result.returncode == 1
    assert "need 3" in result.stdout


def test_every_number_is_printed_whether_it_passed_or_failed(tmp_path):
    # A gate run has to be evidence on its own, not a green tick.
    for records in (healthy(), healthy(mean_ms=30.0)):
        result = run(stats_file(tmp_path, records))
        assert "FIXTURE" in result.stdout
        assert "keeps up with decode" in result.stdout
        assert "mean per-frame cost" in result.stdout
        assert "matched fraction" in result.stdout


def test_a_record_cut_off_mid_write_is_ignored(tmp_path):
    # `ros2 topic echo` is killed by the gate's timeout, so the last record in
    # the file is routinely half-written. It still carries `stage` and a
    # TRUNCATED `detail`, and reading the reject breakdown off it printed
    # "uncalibrated ?" for a run whose numbers were all present.
    good = healthy()
    truncated = ("stage: keypoints\n"
                 "rate_hz: 59.0\n"
                 "latency_ms: 7.5\n"
                 "detail: regime=rotation_only matched=0.905 features=500 lev")
    path = tmp_path / "stats.txt"
    path.write_text("\n".join(good) + "\n" + truncated, encoding="utf-8")
    result = run(path)
    assert result.returncode == 0
    assert "?" not in result.stdout.split("reject")[0]
    assert "uncalibrated 0" in result.stdout


def test_a_truncated_detail_field_fails_rather_than_printing_a_question_mark(tmp_path):
    # `ros2 topic echo` elides strings past 128 characters with "...", and the
    # gate printed "uncalibrated ?" for a run in which the number was present
    # all along. A missing key and a truncated one look identical from here, so
    # the tool has to refuse rather than quietly report less than it promised.
    elided = ("regime=detect_only matched=0.941 features=500 levels=4 "
              "window=10 previews=175 pose_ok=0 pose_rej=1071 rej_few=0 "
              "rej_resid=0 rej_no...")
    result = run(stats_file(tmp_path, healthy(detail=elided)))
    assert result.returncode == 1
    assert "rej_nointr" in result.stdout
    assert "--full-length" in result.stdout
