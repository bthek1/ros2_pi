# Roadmap

Status as of **2026-09-15** (M5 closed that day: depth on the GPU, measured
through the real container rather than a standalone probe; M4 on 2026-09-13
against the reference clip, M2.5 and M3 the day before). The build order and per-phase tests live in
[../plans/future/project_final_state.md](../plans/future/project_final_state.md);
this page is the one-line status view. Milestones map to its phases: M1 = P0,
M2 = P1, and so on through M9 = P8.

Those phases are **built** as five milestone issues, each a contiguous slice:
[#4 A](https://github.com/bthek1/ros2_pi/issues/4) = M1–M2,
[#5 B](https://github.com/bthek1/ros2_pi/issues/5) = M3–M4,
[#6 C](https://github.com/bthek1/ros2_pi/issues/6) = M5,
[#7 D](https://github.com/bthek1/ros2_pi/issues/7) = M6–M7,
[#8 E](https://github.com/bthek1/ros2_pi/issues/8) = M8–M9. Each carries a
`just view-*` RViz recipe for watching that stage by eye — a viewer, never the
evidence.

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
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P5, `bash tools/gates/fusion.sh`: 15.3 ms per integration against a 20 ms budget, 17.1 Hz, 1030 of 1030 frames integrated, 0.19% displaced, 0 without a pose or a colour twin. **The scale aligner makes no measurable difference on this clip** — rotation-only odometry is the larger error, and P7 is the trigger to re-measure |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | **done 2026-09-16** — [issue #7](https://github.com/bthek1/ros2_pi/issues/7) P6, `bash tools/gates/mesh.sh`: 790 668 triangles marched in 2.8 s and decimated to 120 000, boundary loops 5119 → 448 with the frontier still open, a 779 740-triangle PLY, and the worst integration gap 374.7 ms against a control's 401.3 ms — no dip |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | not started |
| M9 | `dashboard_node` — the web view | not started |
| M10 | Loop closure + pose graph + volume rebuild | not started |

## What "done" means here

A milestone is done when it **runs and a test command said so**. Not when the
code compiles, and not when it looked right in RViz once. Each phase in
`project_final_state.md` ends in a `tools/gates/*.sh` script that exits 0 or non-zero and
prints the number it asserted on; the phase is then annotated with the date and
what that recipe printed.

Unit tests are a different instrument and do not close a milestone on their own.
`bash tools/test.sh` runs **300 hermetic tests across twenty suites** on both
machines — the stamp arithmetic, a matrix layout, the static-transform
quaternions, the arithmetic either side of the depth model, the bytes of a saved
PLY, and the percentile and hash every probe reports its numbers through — and
they catch the things that are wrong *silently*. Three of those suites exist to
check a **gate's own instrument** rather than the pipeline: `test_straightness`,
`test_mesh_render` and `test_orb_reference`, because a number asserted on by a
gate is worth what the thing computing it is worth. A gate is what says the running system did the thing. Both are
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
the TensorRT provider, relocalisation, a CUDA OpenCV build, the loop-closure work
that becomes M10, and **re-calibrating on a flat mount** if the scale turns out to
matter. That last one is the freshest and has the most concrete trigger: P9's `fx` is
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
