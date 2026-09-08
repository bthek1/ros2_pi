# Roadmap

Status as of **2026-09-08**. The build order and per-phase tests live in
[../plans/in-progress/bootstrap-plan.md](../plans/in-progress/bootstrap-plan.md);
this page is the one-line status view. Milestones map to plan phases: M1 = P0,
M2 = P1, and so on through M9 = P8, plus **M11 = P9** and **M12 = P11**. M10 is
not a phase — it is the loop-closure work, still deferred. **P10 has no
milestone**: it is the test layer, and the Tests section below is its status
line — a layer the other milestones are built with, not a step on the way to a
mesh.

| # | Milestone | Status |
| --- | --- | --- |
| M0 | Docs and working agreement | **done 2026-09-01** — this tree |
| M1 | Workspace skeleton: `pimesh_msgs`, `pimesh_bringup`, justfile, both machines build | **done 2026-09-01** — `just gate-build` PASS |
| M2 | `pimesh_camera` on the Pi — V4L2 MJPEG, honest capture stamps, fails loudly | **done 2026-09-02** — `just gate-capture` PASS, 5 ms stamps with 0.00 ms drift across launches |
| M3 | Dev-box container — `decode_node` with intra-process comms proven zero-copy | **done 2026-09-04** — `just gate-ipc` PASS ×2, 10/10 frames at the same address, 10/10 differing in the control run |
| M4 | `keypoint_node` — ORB, matching, annotated preview | **done 2026-09-07** — `just gate-keypoints` PASS ×2, 6.99 ms/frame, 94.1% matched, keeping up with 96.4% of what decode delivers |
| M5 | `depth_node` — ONNX Runtime C++ on the GPU, metric depth published | **done 2026-09-08** — `just gate-depth` PASS ×2, 55-61 ms/frame on CUDAExecutionProvider, 10/10 RGB twins byte-identical. Depth is *relative* until P5's tape measure |
| M6 | `fusion_node` — TSDF integration with per-frame scale alignment | not started |
| M7 | `mesh_node` — marching cubes, cleanup, Marker + PLY export | not started |
| M8 | 6-DoF odometry from RGB-D keypoints, so the surface stops smearing | not started |
| M9 | `dashboard_node` — the web view | not started |
| M10 | Loop closure + pose graph + volume rebuild | not started — deferred, not a phase |
| M11 | The Pi's configuration as code: `ansible/`, applied and idempotent (**P9**) | **done 2026-09-02** — `just gate-provision` PASS, first apply 11 changes then 0 |
| M12 | Camera calibration loaded and published, so the odometer runs (**P11**) | not started — promoted out of the future file on 2026-09-07 by P3 |

## Tests

Separate from the milestones, because they are a layer rather than a step. As
of 2026-09-08, **0 failures**: `just test` reports **71 gtest** and **50
pytest** on the dev box, `just test-pi` **11 gtest** under Jazzy on aarch64.
The gtest cases cover `pimesh_camera` (timestamp arithmetic, failure paths) and
`pimesh_perception` (the one-deep mailbox, JPEG decode, the rotation geometry,
and the ORB tracker's two matchings); the pytest cases cover the gate tools and
are dev-box only, since `tools/` never ships to the Pi. The Pi builds only
`pimesh_msgs` and `pimesh_camera`, so it runs the subset that belongs to it.
(`colcon test-result` counts each test binary alongside its cases, so its
totals run a little ahead of the gtest ones.) `just test`, `just test-pi`, and
[testing.md](testing.md) for what each covers.

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
under this repo's playbook, making 60 fps repeatable, the linters, and the
loop-closure work that becomes M10. When a trigger fires, the entry moves into
the plan as the next phase number. Nothing waits in both places.

**This has happened once.** Loading a camera calibration was deferred on the
grounds that a file-loading path with no file to load is a more elaborate way of
publishing zeros. P3 fired the trigger by shipping a rotation estimator that
K-all-zeros keeps switched off, and the entry became **P11 / M12** on
2026-09-07.

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
