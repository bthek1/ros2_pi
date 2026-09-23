# Roadmap

Status as of **2026-09-23**: M0–M9 done — that is the whole pipeline, one webcam
to a live mesh — and **M10–M19 filed that day as issues #10–#13, none started**,
which is the SLAM work. The last thing to close was M8 and M9 on 2026-09-19; M5
on 2026-09-15, M4 on 2026-09-13, M2.5 and M3 the day before.

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
and that is the pipeline** — then, opened 2026-09-23 and **not started**,
[#10 F](https://github.com/bthek1/ros2_pi/issues/10) = M10–M12,
[#11 G](https://github.com/bthek1/ros2_pi/issues/11) = M13–M14,
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
| M0.5 | Scaffolding proven: `pimesh_hello`, composed container, both distros, clean teardown | **done 2026-09-08** — [issue #2](https://github.com/bthek1/ros2_pi/issues/2), five `tools/gates/hello-*.sh` scripts. Teardown corrected **2026-09-09**: it held for `hello-lan` but not `hello-compose`, and the gate had only ever signalled the former |
| M1 | Workspace skeleton: `pimesh_msgs`, `pimesh_bringup`, justfile, both machines build | **done 2026-09-09** — [issue #4](https://github.com/bthek1/ros2_pi/issues/4) P0, `bash tools/gates/build.sh`: clean builds 10.2 s here (Lyrical) / 32.0 s on the Pi (Jazzy), all 5 interfaces byte-identical across the two |
| M2 | `pimesh_camera` on the Pi — V4L2 MJPEG, honest capture stamps, fails loudly | **done 2026-09-09** — [issue #4](https://github.com/bthek1/ros2_pi/issues/4) P1, `bash tools/gates/capture.sh`: 44–59 Hz on the dev box, stamp offset 4.21 ms single-clock, two launches agreeing to 0.30–1.02 ms, busy device refused in 0.24 s |
| M2.5 | Camera calibration — real C922 intrinsics, served on `/camera_info` | **done 2026-09-12** — [issue #9](https://github.com/bthek1/ros2_pi/issues/9) P9, `bash tools/gates/calibration.sh`: fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px, median straightness 0.7796 px, coverage 0.896 over 4/4 quadrants, 24 marker-confirmed frames. Numbered `P9` and sequenced before P3 — the number is an identity, the order is a schedule |
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | **done 2026-09-12** — [issue #5](https://github.com/bthek1/ros2_pi/issues/5) P2, `bash tools/gates/ipc.sh`: 504/504 published buffer addresses reaching their consumers with intra-process comms on against 0/395 with it off, exactly 1 subscriber on the Wi-Fi topic, decode 1.90 ms/frame. The zero-copy claim had to be earned twice — it passed at 429/429 with one consumer and failed at 0/574 the moment a second one arrived, because rclcpp copies for every ownership-taking subscription but the last |
| M4 | `keypoint_node` — ORB, matching, annotated preview | **done 2026-09-13** — [issue #5](https://github.com/bthek1/ros2_pi/issues/5) P3, `bash tools/gates/keypoints.sh` over all 3489 frames of `bags/desk1`: 57.9 Hz sustained, 5.99 ms/frame on the node's own clock against an 8 ms budget, matched fraction 0.9063 against the predecessor's algorithm at 0.9065, pose-gate reject rate 8.2% at a mean residual of 0.0017 rad |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | **done 2026-09-15** — [issue #6](https://github.com/bthek1/ros2_pi/issues/6) P4. `bash tools/gates/depth.sh` replays `bags/desk1` through the real container: `CUDAExecutionProvider`, **55.10 ms mean per frame / 58.21 ms p95** against an 80 ms budget, 17.42 Hz sustained on `/depth`, **1048/1048** `/depth/rgb` frames byte-identical to the frame their depth was inferred on, and 0 non-finite or out-of-range values in 966,625 sampled distances. The control — same binary, `use_cuda:=false` — is `CPUExecutionProvider` at 287.92 ms, outside the same budget. Toolchain gated separately by `bash tools/gates/gpu-stack.sh`: 51.08 ms mean for inference alone, installed rootless and sha256-pinned by `bash tools/fetch-gpu-stack.sh` |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P5, `bash tools/gates/fusion.sh`: 15.3 ms per integration against a 20 ms budget, 17.1 Hz, 1030 of 1030 frames integrated, 0.19% displaced, 0 without a pose or a colour twin. **The scale aligner made no measurable difference on this clip** — rotation-only odometry was named as the larger error and P7 as the trigger to re-measure. **P7 fired and the comparison moved**: re-measured 2026-09-19 with the corrected pose, 0.4805 m aligned against 0.5330 m unaligned and agreement 0.2191 against 0.1606, the aligner ahead on both for the first time. Still printed rather than asserted — one run of a number that has already flipped once |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P6, `bash tools/gates/mesh.sh`: 790 668 triangles marched in 2.8 s and decimated to 120 000, boundary loops 5119 → 448 with the frontier still open, a 779 740-triangle PLY, and the worst integration gap 374.7 ms against a control's 401.3 ms — no dip |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | **done 2026-09-19** — [issue #8](https://github.com/bthek1/ros2_pi/issues/8) P7, `bash tools/gates/odom.sh` over the whole of `bags/desk1`: **1.421 px** mean inlier reprojection over **97 inliers**, **79.9%** of depth frames posed, 1043 poses at **17.47 Hz**, a 31.1 m path and 4.87 m of net displacement against the control's identical zero, fastest published motion 1.9986 m/s against a 2.0 ceiling the node enforces itself. **The larger result is a bug it found on the way**: the rotation had been composed *inverted* since P3 — pan right, the published frame turns left — and correcting it is worth **3× on the paired-surface gap**, 1.3440 m → 0.4456 m, the first of those reproducing what M6 recorded. **The 6-DoF translation itself makes no measurable difference on this clip**, 0.4456 m against the control's 0.4471 m, and the gate prints that rather than asserting it: `desk1` is a pan, and what is left after the pose is the depth network's own shape error |
| M9 | `dashboard_node` — the web view | **done 2026-09-19** — [issue #8](https://github.com/bthek1/ros2_pi/issues/8) P8 and P10, `bash tools/gates/dashboard.sh`. An HTTP and WebSocket server inside a ROS 2 node in its own process, no library and no three.js: five channels at 10.01 Hz stats / 9.16 Hz rgb / 4.47 Hz depth / 10.01 Hz pose / ~2.1 MB of mesh, the STALE flag 2.10 s after `/odom` stops against a 2.0 s threshold, and every stage of the pipeline — including `capture` from the Pi at 60.00 Hz — reporting itself on `/pipeline/stats`. Worst stage drift with a client attached: **0.77%**, against a **0.29%** floor measured between two runs with none — so P8's 2% bound is met. The gate states it as *floor plus slack* rather than a constant, because an earlier run of it saw 15% between two identical no-client runs and that was a `colcon build` sharing the box, not the pipeline |
| M10 | A trajectory measured against ground truth — ATE on TUM fr1/desk | not started — [#10](https://github.com/bthek1/ros2_pi/issues/10) P11, `bash tools/gates/trajectory.sh`. **The first number this project will have about its pose that this project did not produce.** Everything M1–M9 asserts about the trajectory is internal, and P7 showed what that is worth: the rotation was composed inverted from P3 to P7 and every number describing it was correct |
| M11 | `depth_scale` pinned with a tape measure, `bags/scale1` recorded | not started — [#10](https://github.com/bthek1/ros2_pi/issues/10) P12, `bash tools/gates/scale.sh`. **Needs a human.** Until it happens every distance this pipeline reports is plausibly shaped and in an unknown unit |
| M12 | `bags/walk1` — a clip with real translation | not started — [#10](https://github.com/bthek1/ros2_pi/issues/10) P13, `bash tools/gates/odom.sh walk1`. **Needs a human**, and the same visit to the room as M11. Settles the comparison `gates/odom.sh` currently prints rather than asserts |
| M13 | Map points observed by many keyframes, tracking against the local map | not started — [#11](https://github.com/bthek1/ros2_pi/issues/11) P14, `bash tools/gates/map.sh`. A landmark stops dying with the keyframe that saw it, which is the noun SLAM has and this pipeline does not |
| M14 | Local bundle adjustment | not started — [#11](https://github.com/bthek1/ros2_pi/issues/11) P15, `bash tools/gates/ba.sh`. g2o is already installed on both machines from ROS itself, with identical target names — measured 2026-09-23 |
| M15 | Place recognition against the whole keyframe store | not started — [#12](https://github.com/bthek1/ros2_pi/issues/12) P16, `bash tools/gates/place.sh`. The store's second reader, which P7 built it for. **The control is the phase**: zero accepted closures on `bags/desk1`, which revisits nothing |
| M16 | Pose graph — `map -> odom` stops being a static identity | not started — [#12](https://github.com/bthek1/ros2_pi/issues/12) P17, `bash tools/gates/loop.sh` |
| M17 | Frame memory and a volume rebuilt at corrected poses | not started — [#12](https://github.com/bthek1/ros2_pi/issues/12) P18, `bash tools/gates/rebuild.sh`. Without it a closure corrects the trajectory and leaves the room where it was |
| M18 | A `LOST` state that stops fusing | not started — [#13](https://github.com/bthek1/ros2_pi/issues/13) P19, `bash tools/gates/lost.sh`. Today a failed solve holds the last pose — 20.1% of `desk1`'s depth frames — and a held pose under a moving camera is permanent damage to the TSDF |
| M19 | Relocalisation from a persisted map | not started — [#13](https://github.com/bthek1/ros2_pi/issues/13) P20, `bash tools/gates/relocalise.sh` |

## What "done" means here

A milestone is done when it **runs and a test command said so**. Not when the
code compiles, and not when it looked right in RViz once. Each phase in
`project_final_state.md` ends in a `tools/gates/*.sh` script that exits 0 or non-zero and
prints the number it asserted on; the phase is then annotated with the date and
what that recipe printed.

Unit tests are a different instrument and do not close a milestone on their own.
`bash tools/test.sh` runs **414 hermetic tests across twenty-seven suites** on
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
