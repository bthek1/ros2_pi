"""Tests for the P4 gate's assertions.

check_depth.py decides whether the depth stage got the GPU, and the failure it
exists to catch is a silent one: ONNX Runtime will build a CPU session, run the
model at ~290 ms instead of ~55, produce depth maps that are perfectly
*correct*, and report it as a warning nobody reads. Nothing about the output
distinguishes the two. So most of these cases are the ways the tool must say NO
— and one of them is the nastiest version, a session that reports the CUDA
provider and still ran on the CPU.
"""

import subprocess
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parent / "check_depth.py"

CUDA = "CUDAExecutionProvider"
CPU = "CPUExecutionProvider"


def detail(provider=CUDA, state="ready", side=518, scale=10.0, max_depth=6.0):
    return (f"provider={provider} state={state} side={side} "
            f"depth_scale={scale:.3f} max_depth_m={max_depth:.2f}")


def record(stage, rate=17.0, mean_ms=55.4, p95_ms=61.0, processed=100,
           dropped=200, detail_text=None):
    """One /pipeline/stats record as `ros2 topic echo` prints it."""
    if detail_text is None:
        detail_text = detail()
    return (
        "header:\n"
        "  stamp:\n"
        "    sec: 1788829000\n"
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
        f"detail: {detail_text}\n"
        "---"
    )


def healthy(n=6, **kwargs):
    """A run that met every promise: CUDA, 55 ms, loaded and warmed."""
    return [record("depth", processed=100 + i * 17, dropped=200 + i * 30, **kwargs)
            for i in range(n)]


def files(tmp_path, records, log=f"[depth_node]: {CUDA}, 518x518, loaded and warmed"):
    stats = tmp_path / "stats.txt"
    stats.write_text("\n".join(records) + "\n", encoding="utf-8")
    logfile = tmp_path / "run.log"
    logfile.write_text(log + "\n", encoding="utf-8")
    return stats, logfile


def run(stats, log, *args):
    return subprocess.run(
        [sys.executable, str(TOOL), "--stats", str(stats), "--log", str(log), *args],
        capture_output=True, text=True)


def test_a_healthy_run_passes(tmp_path):
    result = run(*files(tmp_path, healthy()))
    assert result.returncode == 0, result.stdout
    assert "FAIL" not in result.stdout


def test_a_cpu_startup_line_fails(tmp_path):
    # The headline assertion of the whole phase.
    result = run(*files(tmp_path, healthy(),
                        log=f"[depth_node]: {CPU} — the GPU was NOT used"))
    assert result.returncode == 1
    assert "FAILED test however good the depth looks" in result.stdout


def test_a_cpu_provider_in_the_running_session_fails(tmp_path):
    result = run(*files(tmp_path, healthy(detail_text=detail(provider=CPU))))
    assert result.returncode == 1
    assert CPU in result.stdout


def test_the_provider_is_checked_in_two_independent_places(tmp_path):
    # The log says CUDA and the live session says CPU. Either source alone
    # would pass this run; the phase's entire risk is a silent fallback, so it
    # must not rest on one of them.
    result = run(*files(tmp_path, healthy(detail_text=detail(provider=CPU)),
                        log=f"[depth_node]: {CUDA}, loaded and warmed"))
    assert result.returncode == 1


def test_a_session_that_reports_cuda_but_ran_on_the_cpu_fails(tmp_path):
    # The nastiest case. ONNX Runtime falls back per-node, so a session can
    # hold the CUDA provider and still have run the graph on the CPU. Only the
    # timing distinguishes them: 280-305 ms was measured for this model on
    # this CPU.
    result = run(*files(tmp_path, healthy(mean_ms=290.0)))
    assert result.returncode == 1
    assert "GPU-shaped" in result.stdout


def test_blowing_the_budget_fails(tmp_path):
    result = run(*files(tmp_path, healthy(mean_ms=95.0)))
    assert result.returncode == 1
    assert "95.0 ms" in result.stdout


def test_a_model_still_loading_fails(tmp_path):
    # Loading and warming a CUDA session takes ~0.7 s and happens on the worker
    # thread. A run that never got past it proved nothing.
    result = run(*files(tmp_path, healthy(detail_text=detail(state="loading"))))
    assert result.returncode == 1
    assert "state=loading" in result.stdout


def test_a_model_that_failed_to_load_fails(tmp_path):
    result = run(*files(tmp_path, healthy(
        detail_text=detail(provider="none", state="failed"))))
    assert result.returncode == 1


def test_the_stage_never_running_fails_loudly(tmp_path):
    others = [record("keypoints", mean_ms=7.0) for _ in range(6)]
    result = run(*files(tmp_path, others))
    assert result.returncode == 1
    assert "never ran" in result.stdout


def test_it_selects_the_depth_stage_not_whoever_published_first(tmp_path):
    # /pipeline/stats carries every stage, keyed by `stage`. Reading the first
    # record gets the camera, which looks wonderful and is not this node.
    camera = [record("capture", rate=59.0, mean_ms=0.26,
                     detail_text="device=/dev/video0") for _ in range(6)]
    result = run(*files(tmp_path, camera + healthy(mean_ms=290.0)))
    assert result.returncode == 1
    assert "290" in result.stdout


def test_a_truncated_detail_field_fails(tmp_path):
    # `ros2 topic echo` elides strings past 128 characters. A missing key and a
    # truncated one look identical from here.
    elided = "provider=CUDAExecutionProvider state=ready side=518 depth_sc"
    result = run(*files(tmp_path, healthy(detail_text=elided)))
    assert result.returncode == 1
    assert "--full-length" in result.stdout


def test_windows_that_did_no_work_do_not_flatter_the_mean(tmp_path):
    # The replay starts after the container does, so the first windows report
    # 0.00 ms. Averaging those in would halve the reported cost and let a
    # genuinely slow stage pass.
    idle = [record("depth", mean_ms=0.0, p95_ms=0.0) for _ in range(4)]
    busy = [record("depth", mean_ms=290.0, p95_ms=300.0) for _ in range(4)]
    result = run(*files(tmp_path, idle + busy))
    assert result.returncode == 1
    assert "290.0 ms" in result.stdout


def test_a_run_with_no_inference_at_all_fails(tmp_path):
    result = run(*files(tmp_path, healthy(mean_ms=0.0, p95_ms=0.0)))
    assert result.returncode == 1
    assert "no window did any inference" in result.stdout


def test_a_large_mailbox_drop_count_is_reported_not_failed(tmp_path):
    # The opposite of what it means at the keypoint stage. Depth runs at ~19 Hz
    # against a 42-60 Hz camera, so most frames MUST be dropped; the one-deep
    # mailbox doing that is the design, and there is no backlog to grow behind
    # it.
    records = [record("depth", processed=100 + i * 17, dropped=200 + i * 500)
               for i in range(6)]
    result = run(*files(tmp_path, records))
    assert result.returncode == 0
    assert "THIS IS THE DESIGN" in result.stdout


def test_a_half_written_final_record_does_not_crash_it(tmp_path):
    # The gate stops the echo with a signal, so the last record is routinely
    # cut off mid-field.
    stats = tmp_path / "stats.txt"
    stats.write_text("\n".join(healthy()) + "\nstage: depth\nrate_hz: 17.0\n",
                     encoding="utf-8")
    log = tmp_path / "run.log"
    log.write_text(f"[depth_node]: {CUDA}, loaded and warmed\n", encoding="utf-8")
    result = run(stats, log)
    assert result.returncode == 0, result.stdout


def test_it_says_the_scale_is_arbitrary(tmp_path):
    # Monocular depth is relative. Every metre figure downstream is provisional
    # until P5's tape measure, and a gate that printed metres without saying so
    # would be inviting the assumption.
    result = run(*files(tmp_path, healthy()))
    assert "ARBITRARY" in result.stdout


def test_every_number_is_printed_whether_it_passed_or_failed(tmp_path):
    for records in (healthy(), healthy(mean_ms=290.0)):
        result = run(*files(tmp_path, records))
        assert "mean per-frame cost" in result.stdout
        assert "p95 per-frame cost" in result.stdout
        assert "provider" in result.stdout
