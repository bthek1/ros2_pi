# Roadmap

**The goal is monocular visual SLAM: estimate the pose of a single moving RGB
camera and build a 3D map from it.** M0–M9 build the two halves that a SLAM
system needs — a tracking front end that estimates a pose per frame, and a
mapping back end that fuses a surface out of it. What they do **not** build is
the part that closes the loop: nothing recognises a place it has seen before, so
drift is bounded per step and unbounded over a session. That is M10–M19.

Status as of **2026-10-02**: M0–M12 and **M15–M19 done** — milestones F, H and I
are closed, so every piece of a monocular SLAM system now exists and has a gate:
place recognition (M15), a pose graph that publishes a real `map -> odom` (M16), a
volume rebuilt at the corrected poses (M17), a `LOST` state that stops the TSDF
being written under a pose the tracker does not trust (M18), and relocalisation into
a map saved by an earlier session (M19). **M13 and M14 are built and not done**: the
local map tracks worse than P7 because the depth network's scale differs 15–21%
between keyframes, and BA's median beats both without separating from no-BA at
three runs. **Most of it is off by default** — `loop_closure`, `rebuild`,
`local_map`, `local_ba`, and a map is loaded only when asked — so the default
pipeline is still P7's odometry, now with P19's refusal in front of the TSDF. Turning
the rest on is a gate run on the live camera, not more code.

The build order and per-phase tests live in
[../plans/future/project_final_state.md](../plans/future/project_final_state.md);
this page is the one-line status view. Milestones map to its phases: M1 = P0,
M2 = P1, and so on through M9 = P8 — then M10 = P11 through M19 = P20, the two
gaps being P9 (calibration, shown here as M2.5) and P10 (the Pi's stats row,
folded into M9).

Those phases are **built** as milestone issues, each a contiguous slice:
[#4 A](https://github.com/bthek1/ros2_pi/issues/4) = M1–M2,
[#5 B](https://github.com/bthek1/ros2_pi/issues/5) = M3–M4,
[#6 C](https://github.com/bthek1/ros2_pi/issues/6) = M5,
[#7 D](https://github.com/bthek1/ros2_pi/issues/7) = M6–M7,
[#8 E](https://github.com/bthek1/ros2_pi/issues/8) = M8–M9 — **all five closed,
and that is the pipeline** — then, opened 2026-09-23,
[#10 F](https://github.com/bthek1/ros2_pi/issues/10) = M10–M12 (**all three done
— M10 2026-09-25, M11 2026-09-28, M12 2026-09-29**),
[#11 G](https://github.com/bthek1/ros2_pi/issues/11) = M13–M14 (closed 2026-09-30 unmet, carried to [#16](https://github.com/bthek1/ros2_pi/issues/16)),
[#12 H](https://github.com/bthek1/ros2_pi/issues/12) = M15–M17,
[#13 I](https://github.com/bthek1/ros2_pi/issues/13) = M18–M19, which turn it
into **monocular visual SLAM**. Each carries a `just view-*` RViz recipe for
watching that stage by eye — a viewer, never the evidence.

**The dividing line between the two halves is not a feature.** A–E estimate a
pose and fuse a surface; F–I add a map, a backend that revisits it, loop closure
and the ability to say "I do not know where I am" — and, first of all, an
outside opinion about whether any of it worked.

| # | Milestone | Status |
| --- | --- | --- |
| M0 | Docs and working agreement | **done 2026-09-01** — this tree |
| M0.5 | Scaffolding proven: `pimesh_hello`, composed container, both distros, clean teardown | **done 2026-09-08** — [issue #2](https://github.com/bthek1/ros2_pi/issues/2), five `tools/gates/hello-*.sh` scripts. Teardown corrected **2026-09-09**: it held for `hello-lan` but not `hello-compose`, and the gate had only ever signalled the former. **Retired 2026-09-23**: the package and four of its gates were deleted once `gates/build.sh` (package list and interfaces, both machines), `gates/ipc.sh` (zero-copy, with two consumers rather than one) and `gates/capture.sh` (cross-distro over the LAN, with real hardware) covered the same claims on the pipeline itself. The fifth was never about hello at all and survives as `tools/gates/teardown.sh` |
| M1 | Workspace skeleton: `pimesh_msgs`, `pimesh_bringup`, justfile, both machines build | **done 2026-09-09** — [issue #4](https://github.com/bthek1/ros2_pi/issues/4) P0, `bash tools/gates/build.sh`: clean builds 10.2 s here (Lyrical) / 32.0 s on the Pi (Jazzy), all 5 interfaces byte-identical across the two |
| M2 | `pimesh_camera` on the Pi — V4L2 MJPEG, honest capture stamps, fails loudly | **done 2026-09-09** — [issue #4](https://github.com/bthek1/ros2_pi/issues/4) P1, `bash tools/gates/capture.sh`: 44–59 Hz on the dev box, stamp offset 4.21 ms single-clock, two launches agreeing to 0.30–1.02 ms, busy device refused in 0.24 s |
| M2.5 | Camera calibration — real C922 intrinsics, served on `/camera_info` | **done 2026-09-12** — [issue #9](https://github.com/bthek1/ros2_pi/issues/9) P9, `bash tools/gates/calibration.sh`: fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px, median straightness 0.7796 px, coverage 0.896 over 4/4 quadrants, 24 marker-confirmed frames. Numbered `P9` and sequenced before P3 — the number is an identity, the order is a schedule |
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | **done 2026-09-12** — [issue #5](https://github.com/bthek1/ros2_pi/issues/5) P2, `bash tools/gates/ipc.sh`: 504/504 published buffer addresses reaching their consumers with intra-process comms on against 0/395 with it off, exactly 1 subscriber on the Wi-Fi topic, decode 1.90 ms/frame. The zero-copy claim had to be earned twice — it passed at 429/429 with one consumer and failed at 0/574 the moment a second one arrived, because rclcpp copies for every ownership-taking subscription but the last |
| M4 | `keypoint_node` — ORB, matching, annotated preview | **done 2026-09-13; split 2026-09-23** — [issue #5](https://github.com/bthek1/ros2_pi/issues/5) P3, `bash tools/gates/keypoints.sh` over all 3489 frames of `bags/desk1`: 57.9 Hz sustained, 5.99 ms/frame on the node's own clock against an 8 ms budget, matched fraction 0.9063 against the predecessor's algorithm at 0.9065, pose-gate reject rate 8.2% at a mean residual of 0.0017 rad. **The pose moved out of this node on 2026-09-23** into `odometry_node` — it had published two `/pipeline/stats` rows since P7 because it was two stages wearing one name, and the pipeline table had listed them as two since then. Re-measured after the split: 57.8 Hz at 6.82 ms/frame, matched fraction **0.9062** against the reference's 0.9065, which is the number that says the move changed nothing |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | **done 2026-09-15** — [issue #6](https://github.com/bthek1/ros2_pi/issues/6) P4. `bash tools/gates/depth.sh` replays `bags/desk1` through the real container: `CUDAExecutionProvider`, **55.10 ms mean per frame / 58.21 ms p95** against an 80 ms budget, 17.42 Hz sustained on `/depth`, **1048/1048** `/depth/rgb` frames byte-identical to the frame their depth was inferred on, and 0 non-finite or out-of-range values in 966,625 sampled distances. The control — same binary, `use_cuda:=false` — is `CPUExecutionProvider` at 287.92 ms, outside the same budget. Toolchain gated separately by `bash tools/gates/gpu-stack.sh`: 51.08 ms mean for inference alone, installed rootless and sha256-pinned by `bash tools/fetch-gpu-stack.sh` |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P5, `bash tools/gates/fusion.sh`: 15.3 ms per integration against a 20 ms budget, 17.1 Hz, 1030 of 1030 frames integrated, 0.19% displaced, 0 without a pose or a colour twin. **The scale aligner made no measurable difference on this clip** — rotation-only odometry was named as the larger error and P7 as the trigger to re-measure. **P7 fired and the comparison moved**: re-measured 2026-09-19 with the corrected pose, 0.4805 m aligned against 0.5330 m unaligned and agreement 0.2191 against 0.1606, the aligner ahead on both for the first time. Still printed rather than asserted — one run of a number that has already flipped once |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P6, `bash tools/gates/mesh.sh`: 790 668 triangles marched in 2.8 s and decimated to 120 000, boundary loops 5119 → 448 with the frontier still open, a 779 740-triangle PLY, and the worst integration gap 374.7 ms against a control's 401.3 ms — no dip |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | **done 2026-09-19** — [issue #8](https://github.com/bthek1/ros2_pi/issues/8) P7, `bash tools/gates/odom.sh` over the whole of `bags/desk1`: **1.421 px** mean inlier reprojection over **97 inliers**, **79.9%** of depth frames posed, 1043 poses at **17.47 Hz**, a 31.1 m path and 4.87 m of net displacement against the control's identical zero, fastest published motion 1.9986 m/s against a 2.0 ceiling the node enforces itself. **The larger result is a bug it found on the way**: the rotation had been composed *inverted* since P3 — pan right, the published frame turns left — and correcting it is worth **3× on the paired-surface gap**, 1.3440 m → 0.4456 m, the first of those reproducing what M6 recorded. **The 6-DoF translation itself makes no measurable difference on this clip**, 0.4456 m against the control's 0.4471 m, and the gate prints that rather than asserting it: `desk1` is a pan, and what is left after the pose is the depth network's own shape error |
| M9 | `dashboard_node` — the web view | **done 2026-09-19** — [issue #8](https://github.com/bthek1/ros2_pi/issues/8) P8 and P10, `bash tools/gates/dashboard.sh`. An HTTP and WebSocket server inside a ROS 2 node in its own process, no library and no three.js: five channels at 10.01 Hz stats / 9.16 Hz rgb / 4.47 Hz depth / 10.01 Hz pose / ~2.1 MB of mesh, the STALE flag 2.10 s after `/odom` stops against a 2.0 s threshold, and every stage of the pipeline — including `capture` from the Pi at 60.00 Hz — reporting itself on `/pipeline/stats`. Worst stage drift with a client attached: **0.77%**, against a **0.29%** floor measured between two runs with none — so P8's 2% bound is met. The gate states it as *floor plus slack* rather than a constant, because an earlier run of it saw 15% between two identical no-client runs and that was a `colcon build` sharing the box, not the pipeline |
| M10 | A trajectory measured against ground truth — ATE on TUM fr1/desk | **done 2026-09-25** — [#10](https://github.com/bthek1/ros2_pi/issues/10) P11, `bash tools/gates/trajectory.sh`. **Sim(3)-aligned ATE RMSE 0.27–0.36 m over seven runs** of ~350 poses of TUM fr1/desk, 100% associated against the motion-capture truth, RPE 0.14–0.15 m over a 1 s window, with the `rotation_only` control failing the same ceiling — it cannot be aligned at all. The fitted scale puts `depth_scale` at **4.6–5.2**, which is M11's answer arrived at without a tape measure. Everything M1–M9 asserts about the trajectory is internal, and P7 showed what that is worth: the rotation was composed inverted from P3 to P7 and every number describing it was correct |
| M11 | `depth_scale` pinned with a tape measure, `bags/scale1` recorded | **done 2026-09-28** — [#10](https://github.com/bthek1/ros2_pi/issues/10) P12, `bash tools/gates/scale.sh` PASS. **`depth_scale` = 4.6002** against a 1.730 m tape, within-frame spread 0.0527 of the median (ceiling 0.10), zero clipping, 346 of 351 frames measured; a replay implies 4.6125. **Every distance this pipeline reports is now in metres** — it had been an arbitrary constant since P4. M10's Sim(3) fit on TUM had independently said 4.6–5.2. Five clips were taken and the finding is worth more than the number: **a blank wall defeats the depth network** (23.5% within-patch spread on a surface confirmed perpendicular, twice), so the surface must carry texture. The accepted figure is ~2% high from a 31% wall-timer in the patch; wall-only gives ≈4.50. Checklist in [setup.md](setup.md#the-visit-to-the-room) |
| M12 | `bags/walk1` — a clip with real translation | **done 2026-09-29** — [#10](https://github.com/bthek1/ros2_pi/issues/10) P13, `bash tools/gates/odom.sh walk1`. **6-DoF beats rotation-only for the first time**: median paired-surface gap **0.4545 / 0.5150 / 0.5204 m** against rotation-only's **0.6688 / 0.7234 / 0.6704** over three runs, non-overlapping, margin 0.1500–0.2876 m over five, sign never flipping. On `bags/desk1` the two tie (0.3830 against 0.3847), which is why P7 could only print the comparison — a ~0.9 m arm arc is explained almost entirely by rotation. The gate now **asserts** it on a walking clip. Took three clips: the first two spent 26% and 49% of their frames on this room's blank walls, where the tenth-percentile frame carried 9 ORB features |
| M13 | Map points observed by many keyframes, tracking against the local map | **built 2026-09-30, claim not met** — [#16](https://github.com/bthek1/ros2_pi/issues/16) P14 (built under [#11](https://github.com/bthek1/ros2_pi/issues/11)), `bash tools/gates/map.sh` **FAIL on the ATE only**. `pimesh_backend` holds map points, covisibility, triangulation and culling; `odometry_node` tracks against the local map behind `local_map:=` (default **false**). Every structural claim passes — local map median 2 keyframes, culls judged and fired (9 keyframes culled on walk1), no duplicate observations — but tracking against the map measured **0.46–0.50 m** Sim(3) ATE against P7's **0.28–0.46 m**, and 7 variants over 24 more runs never beat P7. The mechanism is measured: Depth Anything's scale differs by **~15–21% between consecutive keyframes** (`align_dev`), P7 never mixes two depth maps, and a map mixes them in every PnP |
| M14 | Local bundle adjustment | **built 2026-09-30, claim not met at N=3** — [#16](https://github.com/bthek1/ros2_pi/issues/16) P15 (built under [#11](https://github.com/bthek1/ros2_pi/issues/11)), `bash tools/gates/ba.sh` **FAIL on non-overlap only**. g2o on the backend thread, niced, depth readings as a 15% prior; `local_ba:=` (default **false**). BA median **0.259 m** against no-BA 0.429 and P7 0.309, best run **0.151 m** — the best fr1/desk figure this project has — but its worst (0.418) overlaps no-BA's best (0.318). 5–6 ms per solve, p95 < 9 ms, 0 keyframes dropped, no upstream stage slower (depth +1.9% against a 13.7% bound) |
| M15 | Place recognition against the whole keyframe store | **done 2026-10-02** — [#12](https://github.com/bthek1/ros2_pi/issues/12) P16, `bash tools/gates/place.sh` **PASS** under the bounded rule: **0 wrong-place closures** by motion capture over three fr1/desk runs (34 closures; 71 over two gate runs), no closure losing to odometry by more than ε = 2.69° (the closures' own interquartile range), at least 80% beating it (0.82/0.91/1.00 — run 1 is close to the floor), `bags/walk1`'s return to its start found. The plan's desk1 control ("revisits nothing") was measured false and moved to fr1/desk |
| M16 | Pose graph — `map -> odom` stops being a static identity | **done 2026-09-30** — [#12](https://github.com/bthek1/ros2_pi/issues/12) P17, `bash tools/gates/loop.sh` **PASS**: keyframe ATE (Sim(3), TUM fr1/desk, motion capture) **0.118–0.153 m with loop closure against 0.361–0.533 m without**, three runs each, non-overlapping by 0.21 m; 9–11 closures and sub-millisecond solves a run; `map -> odom` a real, non-identity dynamic edge with `map -> base_link` and `tf_static` resolving through it and 0 TF warnings, on fr1/desk and through `bags/walk1`'s return. `loop_closure` defaults false until the online trajectory is scored too |
| M17 | Frame memory and a volume rebuilt at corrected poses | **done 2026-10-02** — [#12](https://github.com/bthek1/ros2_pi/issues/12) P18, `bash tools/gates/rebuild.sh` **PASS**: over the same remembered frames, the rebuild at the pose graph's corrections sits **0.513–0.525 m** from the same frames rebuilt at motion-capture poses against **0.658–0.898 m** for odometry alone, agreement 0.083–0.116 against 0.057–0.066, every run and non-overlapping; six rebuilds a run, each integrating its whole memory; no stage slower. The self-consistency gap #12 named could not see this and was replaced by an outside reference (`rebuild_eval`) |
| M18 | A `LOST` state that stops fusing | **done 2026-10-02** — [#13](https://github.com/bthek1/ros2_pi/issues/13) P19, `bash tools/gates/lost.sh` **PASS** twice: a 3 s lens cap on `bags/desk1` is LOST in **5 depth frames** and OK **3 frames** after it clears, **0 voxels** integrated while LOST beside 136–148 frames refused, OK over the rest of the clip **0.898–0.902** against P7's 0.799; the control (fuse while LOST, never recover) integrates 514–622 M voxel updates while LOST and scores 0.20–0.33. OK/LOST travels on `/tracking/state`, a value at every depth stamp, because a withheld transform is interpolated by tf2. **desk1 goes LOST on its own** for ~10% of its depth frames — fused at a stale pose until now; the gate's first run put the cap inside one of those stretches and passed over nothing |
| M19 | Relocalisation from a persisted map | **done 2026-10-02** — [#13](https://github.com/bthek1/ros2_pi/issues/13) P20, `bash tools/gates/relocalise.sh` **PASS**: a map of TUM fr1/desk's first 300 frames (14 keyframes, 42.9 kB each), loaded by a new container replaying from frame 330, relocalises after **2 LOST depth frames** to **0.18 m median / 0.25 m worst** of motion capture (the saved map's own error: 0.14 m); **819 queries from `bags/desk1` into that map, 0 accepted**. Moved from walk1 to fr1/desk for the ground truth, and the control to a different room, with reasons in the gate header |

## What "done" means here

A milestone is done when it **runs and a test command said so**. Not when the
code compiles, and not when it looked right in RViz once. Each phase in
`project_final_state.md` ends in a `tools/gates/*.sh` script that exits 0 or non-zero and
prints the number it asserted on; the phase is then annotated with the date and
what that recipe printed.

Unit tests are a different instrument and do not close a milestone on their own.
`bash tools/test.sh` runs **433 hermetic tests across twenty-eight suites** on
both machines — the stamp arithmetic, a matrix layout, the static-transform
quaternions, the arithmetic either side of the depth model, the bytes of a saved
PLY, the JSON the dashboard is built out of, and the percentile and hash every
probe reports its numbers through — and they catch the things that are wrong
*silently*. Four of those suites exist to check a **gate's own instrument**
rather than the pipeline: `test_straightness`, `test_mesh_render`,
`test_orb_reference` and — since 2026-09-21 — `test_dashboard_contract`, which
asserts that the fields `dashboard_probe` scrapes are fields something actually
sends. That last one matters for the reason the others do and one more besides:
the probe finds its numbers by string search, so a renamed field gives it a
**zero rather than an error**, and `gates/dashboard.sh` would report
`ws_dropped=0` — the pacing rule holding perfectly — while measuring nothing at
all. A number asserted on by a gate is worth what the thing computing it is
worth. A gate is what says the running system did the thing. Both are
required; neither substitutes for the other.

The division is about visibility rather than importance: if a mistake would
announce itself with a crash or a topic that stops, a gate is the cheaper place to
catch it; if it would produce a plausible number or a room-shaped picture of the
wrong thing, it belongs here. `bash tools/gates/test.sh` asserts the count, zero
skips, and that both machines run the *same* suites.

The exceptions are physical-world checks that no script can close: the tape
measure that pins `depth_scale`, exposure in a real room, and whether the mesh
looks like the room. Those say "needs a human" explicitly and name the one
recording that would turn them into a replayable gate.

## Deferred, not forgotten

Work that is real but not executable yet is **not** a phase. It sits in the
deferred half of
[../plans/future/project_final_state.md](../plans/future/project_final_state.md#part-2--deferred),
each entry with the trigger that would make it executable — CUDA TSDF kernels,
the TensorRT provider, a CUDA OpenCV build, and **re-calibrating on a flat
mount** if the scale turns out to matter — plus one companion future file per
milestone issue. **Two entries left that register on 2026-09-23** when their
triggers fired: loop closure became issue
[#12](https://github.com/bthek1/ros2_pi/issues/12) and relocalisation became
P20 in [#13](https://github.com/bthek1/ros2_pi/issues/13). That last one is the freshest and has the most concrete trigger: P9's `fx` is
pinned only to **±2.2%**, which is a ±2.2% slack in every distance this pipeline
reports, and **M6's tape measure is the first thing that can check a scale
independently of the calibration that produced it**. Disagreement beyond about 2% is
the trigger. When a trigger fires, the entry moves into the phase list as the next
phase number. Nothing waits in both places.

## Never in scope

These are non-goals, not deferrals — they do not belong in the future file
either:

- **Multi-camera or stereo.** One webcam is the constraint the project is built
  around; a second view would remove the interesting problem.
- **Autonomy of any kind** — navigation, planning, control. This is perception
  and reconstruction.
- **Running inference on the Pi.** The Pi is a sensor head. That is the design.
