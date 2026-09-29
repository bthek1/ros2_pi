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
> 0.27–0.36 m over seven runs of fr1/desk, with `dataset_node`, `fetch-dataset.sh` and
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

> **This trigger fired on 2026-09-28, and the answer was that the existing value
> is now about right.** `depth_scale` went from 10.0 to 4.6002, so the map's units
> are metres and `max_speed_m_s = 2.0` means a real 2.0 m/s where it used to mean
> **0.92 m/s** — ordinary walking pace, which is why it refused 27-36 poses a run
> on TUM fr1/desk. Measured on `bags/walk1` the day after: the fastest published
> motion is **1.94 m/s** against a gate ceiling of 2.5, and the node refused
> **8-36** poses depending on the clip rather than a fixed 7-10%. So the ceiling no
> longer needs re-deriving; what is left of this entry is **throttling the
> warning**, which is a log-rate fix and not a unit one, and the
> `RCLCPP_WARN`-under-a-mutex argument above still stands. Recorded rather than
> promoted, because a throttle is not worth a phase of its own — fold it into the
> next change that touches `odometry_node`.

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

---

## `gates/scale.sh` has no view of what is *in* the patch

**What.** Two blind spots in P12's gate, both found by running it five times at a
real wall on 2026-09-28, and both of which let a worse clip score better.

1. **An object in the patch passes.** The accepted clip's patch is **31%
   wall-timer**, which the network puts 0.268 m nearer than the wall behind it.
   The median is still a wall pixel, but at the wall's ~27th percentile rather
   than its 50th, so `depth_scale` = 4.6002 is about **2% high**; wall-only gives
   ≈4.50. Nothing in the gate looks at whether the patch is one surface — only at
   its spread, and a 31%/69% split with a 0.27 m step scored 5.17% against a 10%
   ceiling.
2. **Defocus improves the spread.** A blurred depth map is a smoother depth map.
   One clip scored the *best* spread of the five (4.42%) and gave the *outlying*
   scale (5.3502), because its patch was nearly blank. The gate has no view of
   focus at all, so the metric it asserts on moves the wrong way as the image
   gets worse.

**Why not now.** Both fixes want the same input the gate does not currently
read — the *colour* frame, not just `/depth`. Texture (Laplacian variance over
the patch) and focus are cheap to measure there, and a bimodal depth histogram
would catch the object. But `scale_probe` subscribes to `/depth` alone, and
adding a second subscription to a gate whose number is already recorded means
re-measuring `depth_scale` in the same change, which is two things at once.

**Trigger.** **A re-measurement of `depth_scale` on a properly textured flat
surface** — which is the thing that would retire the 2% bias anyway. Do both in
one visit: add the patch checks, re-record, and record whether the figure moves
out of the 4.50–4.70 band that five clips agreed on.

---

## `desk1`'s agreement dropped and nothing here explains it

**What.** `gates/odom.sh`'s `sixdof` median agreement on `bags/desk1` was
**0.2102** when P7 derived the floor from it on 2026-09-19. Measured 2026-09-29
over four runs it is **0.1646, 0.1653, 0.1741, 0.1752** — down about 20%, on a
clip that has not changed.

**What it is not.** The obvious candidate was P12: `voxel_size_m` is 0.015 in
*map* units, and pinning `depth_scale` from 10.0 to 4.6002 changed what a map
unit means, so the TSDF went from ~6.9 mm physical voxels to 15 mm — 2.17×
coarser. **Tested and wrong.** Doubling the voxel size to 0.030 moved the two
regimes in *opposite* directions (sixdof 0.1653 → 0.1386, rotation_only 0.1456 →
0.1642), where a resolution effect would move both the same way.

Between 2026-09-19 and now the tree also took #14's renames, P11's `odom_probe`
frame change and P12 — so the cause is not attributable from what is recorded,
and guessing at it is what this file exists to avoid.

**Why it matters less than it looks.** The re-derived `MIN_AGREE` of 0.12 was
measured against the pre-P7 control *as the code is today*, so the assertion is
sound whatever moved the baseline. This is an unexplained observation, not a
known defect.

**Trigger.** **P14 or P15** — milestone G is the next work that changes the
mapping back end, and `MapPoint`s plus local bundle adjustment will move this
number again. Re-measure agreement on `desk1` before and after that change: two
observations either side of a known edit are what would separate a real
regression from whatever happened in the last ten days.
