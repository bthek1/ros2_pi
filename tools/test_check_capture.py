"""Tests for the P1 gate's assertions.

A gate is only worth having if it fails when it should, so these drive
`check_capture.py` through its CLI — the same contract the justfile uses — and
check both the exit code and what it printed.

Run with `just test`, or directly:  python3 -m pytest tools/ -v
"""

import subprocess
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).with_name("check_capture.py")

# A run where everything is healthy, taken from the real 2026-09-02 gate pass.
HEALTHY = {
    "hw": 59.09,
    "node": (59.52, 59.52),
    "delivered": (59.04, 53.57),
    "pi": (0.005, 0.005),
    "dev": (0.026, 0.017),
}


def run(**kw):
    args = {**HEALTHY, **kw}
    cmd = [
        sys.executable, str(TOOL),
        "--hw-fps", str(args["hw"]),
        "--node-rate", *(str(x) for x in args["node"]),
        "--delivered-rate", *(str(x) for x in args["delivered"]),
        "--pi-delay", *(str(x) for x in args["pi"]),
        "--dev-delay", *(str(x) for x in args["dev"]),
    ]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_a_healthy_run_passes():
    result = run()
    assert result.returncode == 0, result.stdout
    assert "FAIL" not in result.stdout


def test_dropping_frames_against_the_hardware_fails():
    # The node captures 40 Hz while raw v4l2 gets 59 — a third of the frames
    # lost inside our own code.
    result = run(node=(40.0, 40.0))
    assert result.returncode == 1
    assert "losing 32" in result.stdout


def test_a_small_shortfall_against_the_hardware_passes():
    # Rates are sampled over different windows, so they never match exactly.
    # 3% under must not fail, or the gate cries wolf on every run.
    result = run(node=(57.3, 57.3))
    assert result.returncode == 0, result.stdout


def test_capturing_faster_than_the_measured_ceiling_passes():
    # The hardware measurement and the node's run happen minutes apart, and the
    # C922's rate tracks its auto-exposure, so the node legitimately comes out
    # slightly ahead sometimes. Negative loss is not a failure.
    result = run(hw=59.09, node=(59.52, 59.52))
    assert result.returncode == 0, result.stdout


def test_a_collapsed_delivered_rate_fails():
    # The node captures fine but almost nothing crosses the Wi-Fi.
    result = run(delivered=(3.0, 3.0))
    assert result.returncode == 1
    assert "floor" in result.stdout


def test_a_slow_stamp_on_the_pi_fails():
    result = run(pi=(0.050, 0.050))
    assert result.returncode == 1
    assert "on-Pi stamp-to-receipt" in result.stdout


def test_the_usb_cam_bug_fails_the_gate():
    """The whole point of the phase, as a test.

    usb_cam 0.8.1 draws a new sub-second epoch error at every launch: measured
    0.223 s on one launch and 0.362 s on the next. Both are 'small' in
    isolation; it is the MOVEMENT between launches that gives it away.
    """
    result = run(pi=(0.223, 0.362))
    assert result.returncode == 1
    assert "usb_cam test" in result.stdout
    assert "139.00 ms apart" in result.stdout


def test_a_stable_offset_across_launches_passes():
    # Same offset both times, and small: nothing per-process is leaking in.
    result = run(pi=(0.006, 0.007))
    assert result.returncode == 0, result.stdout


def test_a_missing_hardware_measurement_fails_rather_than_skips():
    # If the raw v4l2 step did not produce a number, the gate must not quietly
    # pass on the strength of the checks it could still run.
    result = run(hw=0.0)
    assert result.returncode == 1
    assert "no raw v4l2 measurement" in result.stdout


def test_the_dev_box_figures_are_reported_but_not_asserted():
    # A wandering dev-box delay is the two machines' clocks, not the node.
    result = run(dev=(0.011, 0.038))
    assert result.returncode == 0, result.stdout
    assert "not asserted" in result.stdout
    assert "11.0 / 38.0 ms" in result.stdout


def test_transport_loss_is_reported_as_information():
    result = run(node=(59.5, 59.5), delivered=(50.0, 50.0))
    assert result.returncode == 0, result.stdout
    assert "lost in transport" in result.stdout


@pytest.mark.parametrize("missing", ["--hw-fps", "--pi-delay"])
def test_missing_arguments_are_an_error_not_a_pass(missing):
    cmd = [sys.executable, str(TOOL)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode != 0
