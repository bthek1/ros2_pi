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

> **This trigger fired on 2026-09-25.** The gate is green: Sim(3)-aligned ATE
> 0.27–0.36 m over six runs of fr1/desk, with `dataset_node`, `fetch-dataset.sh` and
> `tum_freiburg1.yaml` all in place — so the afternoon this entry describes is now
> exactly the afternoon it predicted. The rule in [../README.md](../README.md) says
> a fired trigger means deleting this entry and appending it to
> [#10](https://github.com/bthek1/ros2_pi/issues/10) as **P21**, the next unused
> number. **That has not been done**, because promoting it commits somebody to
> building it and the person whose milestone this is has not said so yet. It is
> recorded here rather than acted on, and this paragraph is what should be deleted
> along with the entry when it moves.

---

## Re-derive `max_speed_m_s` once `depth_scale` is pinned

**What.** `odometry_node`'s plausibility guard refuses a pose whose implied speed
exceeds `max_speed_m_s`, which is **2.0 in the map's arbitrary units**. On
`bags/desk1` it fires rarely and catches exactly what it is for — a 9.4 m step
between two depth frames at a confident 1.23 px. On TUM fr1/desk it fired **27–36
times per run, on 7–10% of depth frames**, with the fastest surviving motion
sitting at 1.95–1.99 against the 2.0 ceiling. Those refusals are real hand motion
in a faster clip, not bad fits: they are logged at 1.2–1.8 px over 12–211 inliers.

**Why it matters more than the count.** The ceiling is in metres per second of a
unit that is currently arbitrary, so it means a different thing on every scene,
and it is *also* a rate limit on the log — `RCLCPP_WARN` goes through rclcpp's
process-global mutex behind a synchronous terminal write, which this project has
already measured stalling an RViz render loop. Five refusals a second is the same
mechanism that made TF_OLD_DATA flood.

**Why not now.** The honest fix needs the unit. With `depth_scale` pinned, the
ceiling becomes a real speed — something like "faster than a person can move a
hand-held camera" — and can be stated as one. Guessing at it now would replace one
arbitrary number with another, and it would move the P11 figure for a reason that
is not the estimator.

**Trigger.** **P12.** The tape measure. At that point re-derive the ceiling as a
physical speed, re-run `bash tools/gates/trajectory.sh` and record whether the
refusal count and the ATE moved — and throttle the warning while you are there.

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
