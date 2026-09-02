# Roadmap

Status as of **2026-09-02**. The build order and per-phase tests live in
[../plans/in-progress/bootstrap-plan.md](../plans/in-progress/bootstrap-plan.md);
this page is the one-line status view. Milestones map to plan phases: M1 = P0,
M2 = P1, and so on through M9 = P8, plus **M11 = P9**. M10 is not a phase — it
is the loop-closure work, still deferred.

| # | Milestone | Status |
| --- | --- | --- |
| M0 | Docs and working agreement | **done 2026-09-01** — this tree |
| M1 | Workspace skeleton: `pimesh_msgs`, `pimesh_bringup`, justfile, both machines build | **done 2026-09-01** — `just gate-build` PASS |
| M2 | `pimesh_camera` on the Pi — V4L2 MJPEG, honest capture stamps, fails loudly | not started |
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | not started |
| M4 | `keypoint_node` — ORB, matching, annotated preview | not started |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | not started |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | not started |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | not started |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | not started |
| M9 | `dashboard_node` — the web view | not started |
| M10 | Loop closure + pose graph + volume rebuild | not started — deferred, not a phase |
| M11 | The Pi's configuration as code: `ansible/`, applied and idempotent (**P9**) | **done 2026-09-02** — `just gate-provision` PASS, first apply 11 changes then 0 |

## What "done" means here

A milestone is done when it **runs and a test command said so**. Not when the
code compiles, and not when it looked right in RViz once. Each phase in the
bootstrap plan ends in a `just gate-*` recipe that exits 0 or non-zero and
prints the number it asserted on; the phase is then annotated with the date and
what that recipe printed.

The exceptions are physical-world checks that no script can close: the tape
measure that pins `depth_scale`, exposure in a real room, and whether the mesh
looks like the room. Those say "needs a human" explicitly and name the one
recording that would turn them into a replayable gate.

## Deferred, not forgotten

Work that is real but not executable yet is **not** in the plan. It sits in
[../plans/future/bootstrap-future.md](../plans/future/bootstrap-future.md), each
entry with the trigger that would make it executable — CUDA TSDF kernels, the
TensorRT provider, relocalisation, a CUDA OpenCV build, bringing the dev box
under this repo's playbook, and the loop-closure work that becomes M10. When a
trigger fires, the entry moves into the plan as the next phase number. Nothing
waits in both places.

## Never in scope

These are non-goals, not deferrals — they do not belong in the future file
either:

- **Multi-camera or stereo.** One webcam is the constraint the project is built
  around; a second view would remove the interesting problem.
- **Autonomy of any kind** — navigation, planning, control. This is perception
  and reconstruction.
- **Running inference on the Pi.** The Pi is a sensor head. That is the design.
- **Configuring the Pi by hand.** Machine state is a role in `ansible/` or it
  does not survive the next reflash. This is a rule, not a milestone.
