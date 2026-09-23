# Milestone F — deferred

Work that came out of [#10](https://github.com/bthek1/ros2_pi/issues/10) and is
**not executable yet**. Each entry names the trigger that would make it a phase.
When a trigger fires, the entry is deleted from here and appended to the issue
body as the next unused phase number, with a test that is a command. It never
sits in both.

See [../README.md](../README.md) for the rules.

---

## A second dataset family — EuRoC MH_01

**What.** Fetch and replay EuRoC MH_01 alongside TUM fr1/desk, so the ATE is not
a claim about one room in Freiburg.

**Why not now.** EuRoC is a different enough animal that adding it before
fr1/desk works would mean debugging two things: **grayscale** 752×480 global
shutter at 20 Hz, aggressive drone motion rather than a desk sweep, ground truth
from a Vicon rather than a Kinect, and a stereo pair of which we would use one
eye. Depth Anything V2 on grayscale is itself an unmeasured question — the model
was trained on RGB, and feeding it a replicated single channel is a change to
the input distribution that nobody here has measured.

**Trigger.** `bash tools/gates/trajectory.sh` green on fr1/desk. At that point
the harness exists and EuRoC is a second `fetch-dataset.sh` entry and a second
`camera_info`, which is an afternoon rather than a phase — unless the grayscale
question turns out to be real, in which case that is what the phase is about.

---

## Re-calibrate on a flat mount — *not a new entry, a pointer*

The entry lives in
[project_final_state.md](project_final_state.md#re-calibrate-on-a-flat-mount-if-the-scale-turns-out-to-matter)
and its trigger is **P5's tape measure**, which is **P12** in this milestone.
Written here only so that whoever does P12 knows the trigger they may be
pulling: if the measured distance disagrees with the tape by more than ~2%, the
fix is a flat mount and a re-run of `record | select | solve` before touching
anything in the pipeline. `fx` is currently pinned only to ±2.2%, and that is a
±2.2% slack in every distance this project reports.

It stays in that file rather than moving here, because an entry never sits in
two places.
