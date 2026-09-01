# Roadmap

Status as of **2026-09-01**. The build order and per-phase gates live in
[../plans/in-progress/bootstrap-plan.md](../plans/in-progress/bootstrap-plan.md);
this page is the one-line status view.

| # | Milestone | Status |
| --- | --- | --- |
| M0 | Docs and working agreement | **done 2026-09-01** — this tree |
| M1 | Workspace skeleton: `pimesh_msgs`, `pimesh_bringup`, justfile, both machines build | not started |
| M2 | `pimesh_camera` on the Pi — V4L2 MJPEG, honest capture stamps, fails loudly | not started |
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | not started |
| M4 | `keypoint_node` — ORB, matching, annotated preview | not started |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | not started |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | not started |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | not started |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | not started |
| M9 | `dashboard_node` — the web view | not started |
| M10 | Loop closure + pose graph + volume rebuild | not started |

## What "done" means here

A milestone is done when it **runs and something measured it**. Not when the
code compiles, and not when it looked right in RViz once. Each phase in the
bootstrap plan names its own evidence — a number on a topic, a log line with a
threshold, a rendered image file — and the phase is annotated with what actually
happened and on what date.

The exceptions are physical-world checks that no script can close: the tape
measure that pins `depth_scale`, exposure in a real room, and whether the mesh
looks like the room. Those say "needs a human" explicitly and name the one
recording that would turn them into a replayable gate.

## Deliberately not in scope yet

- **Multi-camera or stereo.** One webcam is the constraint the project is built
  around; a second view would remove the interesting problem.
- **CUDA TSDF kernels.** Only after the CPU integrator is correct and profiled,
  and only once a CUDA toolkit is actually installed.
- **Autonomy of any kind** — navigation, planning, control. This is perception
  and reconstruction.
- **Running inference on the Pi.** The Pi is a sensor head. That is the design.
