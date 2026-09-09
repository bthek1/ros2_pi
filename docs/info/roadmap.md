# Roadmap

Status as of **2026-09-09** (M1 and M2 done that day). The build order and per-phase tests live in
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
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | not started |
| M4 | `keypoint_node` — ORB, matching, annotated preview | not started |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | not started |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | not started |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | not started |
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
`bash tools/test.sh` runs 29 hermetic tests on both machines — the stamp
arithmetic, a matrix layout, the static-transform quaternions — and they catch
the things that are wrong *silently*. A gate is what says the running system did
the thing. Both are required; neither substitutes for the other.

The exceptions are physical-world checks that no script can close: the tape
measure that pins `depth_scale`, exposure in a real room, and whether the mesh
looks like the room. Those say "needs a human" explicitly and name the one
recording that would turn them into a replayable gate.

## Deferred, not forgotten

Work that is real but not executable yet is **not** a phase. It sits in the
deferred half of
[../plans/future/project_final_state.md](../plans/future/project_final_state.md#part-2--deferred),
each entry with the trigger that would make it executable — CUDA TSDF kernels,
the TensorRT provider, relocalisation, a CUDA OpenCV build, and the
loop-closure work that becomes M10. When a trigger fires, the entry moves into
the phase list as the next phase number. Nothing waits in both places.

## Never in scope

These are non-goals, not deferrals — they do not belong in the future file
either:

- **Multi-camera or stereo.** One webcam is the constraint the project is built
  around; a second view would remove the interesting problem.
- **Autonomy of any kind** — navigation, planning, control. This is perception
  and reconstruction.
- **Running inference on the Pi.** The Pi is a sensor head. That is the design.
