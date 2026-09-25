# ros2_pi — monocular visual SLAM from one moving webcam

**The goal: estimate the camera's pose and build a 3D map of the room, from a
single moving RGB camera and nothing else.** A Raspberry Pi 5 with a USB webcam,
an Ubuntu dev box with a GPU, and ROS 2 in between. No depth sensor, no IMU, no
wheel odometry — everything the system knows about where it is and what the room
looks like is inferred from one moving view.

```
RGB frame → keypoints (ORB) → pose (PnP vs keyframe) → monocular depth (Depth Anything V2) → TSDF fusion → triangle mesh → dashboard
             └──────── tracking front end ────────┘    └──────────── mapping back end ────────────┘
```

**What is built is the front end and the map; what is missing is the loop.** The
tracking half estimates a 6-DoF pose per frame and the mapping half fuses depth
into a surface, which is *visual odometry plus dense mapping* — a SLAM system's
two halves without the part that makes the acronym honest. There is no loop
closure, no pose graph and no relocalisation, so drift is never corrected and a
room re-entered is a room seen for the first time. The keyframe store those need
is built and has one reader; the second reader — matching a frame against *every*
keyframe rather than the newest — is the next piece of work, and it is named as
such in [docs/info/roadmap.md](docs/info/roadmap.md). Calling this SLAM today
would be claiming the half that is not there.

**Written in C++.** The Pi is a sensor head — it captures, stamps and ships
JPEG, and nothing else. Every expensive stage runs on the dev box, on the GPU
where it pays.

## Status

**The whole pipeline runs, end to end.** As of **2026-09-23** the repository holds
nine packages, and a webcam on a Pi becomes a triangle surface on the dev box with
a browser tab watching it. Since **2026-09-25** it has also been measured against
a trajectory recorded by something that has never heard of this repository. The
surface does not look like a room yet, for a reason given below:

- **Capture** (`pimesh_camera`, on the Pi) — 1280×720 MJPEG stamped with the
  kernel's capture time, **44–59 Hz received on the dev box**, serving real
  intrinsics on `/camera_info` (fx=953.4, fy=957.6, held-out reprojection
  0.4955 px).
- **Decode** (`pimesh_frontend`, here) — the container's *one* subscriber on the
  only topic that crosses Wi-Fi, `cv::imdecode` at **1.90 ms/frame**, handing the
  2.7 MB frame to its consumers as a pointer: **504/504** buffer addresses matched
  with intra-process comms on against **0/395** with it off.
- **Keypoints** (`pimesh_frontend`, here) — ORB at 500 features with pooled
  matching over a 10-frame window, **57.8 Hz sustained at 6.82 ms/frame** with
  depth and odometry running beside it, publishing corners, track ids and each
  corner's position one frame earlier.
- **Pose** (`pimesh_frontend`, here) — **the tracking front end**, `odometry_node`:
  `cv::solvePnPRansac` against the newest keyframe's 3D landmarks, **1.37 px mean
  inlier reprojection over 94 inliers on 81.6% of depth frames**, 16.6 Hz. It
  holds its last pose rather than guessing when its gates fail, and refuses a fit
  whose *motion* is implausible even when the fit itself is confident — a
  reprojection error cannot tell you the 3D points were where the depth network
  claimed. `rotation_only` stays selectable as the control run.
- **Depth** (`pimesh_depth`, here, on the GPU) — Depth Anything V2 Small at
  518² through ONNX Runtime's CUDA execution provider, **55.1 ms/frame and
  17.4 Hz** against an 80 ms budget, publishing `/depth` in metres alongside
  `/depth/rgb` — the exact frame each map was inferred on, **1048/1048 measured
  byte-identical**. The metres are the right *shape* and an arbitrary *size*:
  monocular depth is scale-ambiguous until something measures a known distance.
- **Fusion** (`pimesh_mapping`, here) — a spatially hashed TSDF at 15 mm voxels,
  **15.3 ms per integration** against a 20 ms budget and **17.1 Hz** sustained,
  with every frame posed at its own capture stamp and paired with the exact colour
  frame its depth was inferred on: 1030 of 1030 frames offered actually
  integrated, 0.19% displaced, 0 without a pose, 0 without a colour twin.
- **Surface** (`pimesh_mapping`, here) — marching cubes over a chunked snapshot of
  the volume every ten seconds, on a niced thread: **790 668 triangles in 2.8 s**,
  debris pruned, interior holes fanned shut with **every component's frontier left
  open**, decimated to **120 000** for `/world/mesh` and written full-detail as a
  **779 740-triangle** PLY. The worst gap between two integrations was **374.7 ms
  with meshing running against 401.3 ms in a control with nothing meshing** — the
  extraction costs the integrator nothing measurable.

- **Truth** (`pimesh_dataset` + `evo`, offline) — TUM RGB-D fr1/desk, 613 frames
  with a 100 Hz motion-capture trajectory, replayed through the real container at
  its own stamps with its own intrinsics: **Sim(3)-aligned ATE RMSE 0.27–0.36 m**
  over seven runs of ~350 poses, **100%** of them associated against the truth, RPE **0.15 m** over a
  1 s window, with the `rotation_only` control unable to be aligned at all. It is
  the first figure in this project about the pose that this project did not
  produce, and it was worth having: everything above is internal, and P7 is what
  that is worth — the rotation was composed inverted for six days and every
  internal number describing it was right. `bash tools/gates/trajectory.sh`.

- **Unit** (`pimesh_depth`, here) — **written and waiting on a person.**
  `bash tools/gates/scale.sh` compares the median of `/depth` over a centred patch
  against a tape measure, so that `depth_scale` stops being 10.0 because somebody
  typed it. Every path through it is exercised except the one that needs a flat
  wall at a measured distance — including both refusal budgets, which
  `bags/desk1` fails at **0.30 of the patch clipped** while reporting a
  plausible 4.42 m. The checklist for the visit is in
  [docs/info/setup.md](docs/info/setup.md#the-visit-to-the-room).

`just build` then `just view-mesh` shows it running; `just --list` is the
whole of what you type on a normal day. The tests are the `tools/gates/*.sh`
scripts, run directly, and each milestone issue records what they printed —
[#4](https://github.com/bthek1/ros2_pi/issues/4) for capture,
[#5](https://github.com/bthek1/ros2_pi/issues/5) for decode and keypoints,
[#6](https://github.com/bthek1/ros2_pi/issues/6) for depth,
[#7](https://github.com/bthek1/ros2_pi/issues/7) for fusion and the mesh,
[#9](https://github.com/bthek1/ros2_pi/issues/9) for the calibration, and
[#10](https://github.com/bthek1/ros2_pi/issues/10) for the ATE.

**The mesh is still not a room, and the reason moved on 2026-09-19.** Two causes
were named for it — rotation-only odometry and an unpinned `depth_scale` — and P7
settled the first. It found that the rotation had been composed **inverted** since
P3: a fit answers where the *points* went, and the camera composes with the
inverse of that, so the published frame turned left when the camera panned right.
Nothing failed; a frame that moves when you pan looks correct in RViz. Correcting
it is worth **3× on the paired-surface gap** — 1.3440 m to 0.4456 m — and the
first of those numbers reproduces exactly what milestone D recorded.

6-DoF translation is real now too, but on the reference clip it makes no
measurable difference to the surface: `bags/desk1` is a *pan*, so rotation already
explains most of the frame motion, and what is left is the depth network's own
shape error rather than the pose's. What would settle it is a clip with deliberate
translation, which needs a person and the camera — that is **P13** of
[#10](https://github.com/bthek1/ros2_pi/issues/10), alongside **P12**, the tape
measure that pins `depth_scale`.

**P11 has since bounded that second number without a tape measure.** Fitting a
Sim(3) between our trajectory and TUM's metric ground truth gives a scale of
0.46–0.52 over seven runs, so that sequence says `depth_scale` should be **4.6–5.2** against the
10.0 in the YAML. It does not transfer — different camera, different scene, and
Depth Anything's scale is per-image — so the visit to the room still has to
happen. What changed is that it is now a *check* on a figure that exists rather
than the only source of it.

**Every number in `docs/` is now this project's own.** Nothing is quoted from the
Python predecessor at [`~/Documents/piros2`](../piros2) as a stand-in any more; it
remains the yardstick where a comparison is useful, not a source.

## Why a rewrite

The Python version works. It also runs each stage as its own process, so five
subscribers each pulled their own copy of the camera stream over the Pi's Wi-Fi
and collapsed the link (~2 frames/s per reader, against 14.7 Hz for one), and it
needed a relay node and eventually a second interpreter to work around a missing
wheel.

In C++ those problems dissolve: the dev-box stages compose into **one process
with intra-process communication**, so the stream is read once, decoded once, and
passed onward as a pointer. One network subscriber, no relay, no serialisation
between stages, and the meshing library lives in the same process as the node
that uses it.

## The machines

| | Dev box | Raspberry Pi |
| --- | --- | --- |
| OS | Ubuntu 26.04, x86_64 | Ubuntu 24.04, aarch64 (Pi 5, 8 GB) |
| ROS | Lyrical | Jazzy |
| GPU | GTX 1660 SUPER, 6 GB | — |
| Runs | decode, keypoints, odometry, depth, fusion, meshing, dashboard | capture only |

Two different ROS distros, deliberately — there is no ABI compatibility across
them, so every package builds from source on the machine that runs it.

## The budget

Measured on this hardware — the depth row here, the rest via the predecessor:

| Stage | Cost |
| --- | --- |
| Capture + ship (Pi) | ~16 ms/frame, up to 60 fps at 720p MJPEG |
| Decode + ORB (dev box CPU) | ~9 ms/frame |
| **Depth (dev box GPU)** | **55.1 ms/frame** in the node, 51.1 ms of it inference — 287.9 ms on CPU ([gate](docs/info/setup.md#gpu), 2026-09-15) |
| TSDF integrate | ~15 ms/frame (target) |
| Mesh extraction | 300–900 ms, every ~10 s, off the hot path |

**Depth is the pipeline's clock: 17.4 Hz measured.** Every stage drops rather than
queues.

## Documentation

| | |
| --- | --- |
| [CLAUDE.md](CLAUDE.md) | The working agreement — machines, conventions, and the constraints that have already cost real debugging time |
| [docs/info/architecture.md](docs/info/architecture.md) | Node graph, topics, QoS, TF tree, threading |
| [docs/info/pipeline.md](docs/info/pipeline.md) | Every stage: algorithms, libraries, message shapes, costs |
| [docs/info/dashboard.md](docs/info/dashboard.md) | The web dashboard |
| [docs/info/hardware.md](docs/info/hardware.md) | Measured specs of both machines and the camera |
| [docs/info/setup.md](docs/info/setup.md) | Getting both machines to build and run this |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones |
| [docs/plans/README.md](docs/plans/README.md) | How plans are written here: a GitHub issue of stable phases, a command for a test, executable phases only |
| [Issue #2 — hello-world plan](https://github.com/bthek1/ros2_pi/issues/2) | Closed 2026-09-08: the scaffolding, and what each of its gates measured. The `pimesh_hello` package and its five gates were deleted on 2026-09-23, once the real pipeline's gates covered the same claims |
| [Issue #3 — justfile plan](https://github.com/bthek1/ros2_pi/issues/3) | Closed 2026-09-09: grouped recipes, gate bodies in `tools/`, and the first shellcheck run over this repo's shell |
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | **Where this went.** The build order P0–P8, each ending in a `tools/gates/*.sh` test and each annotated with what that test printed, followed by the deferred register — each entry with the trigger that would make it a phase |

Plans live in the issue tracker, not in this tree: `gh issue list --label plan`.

## What one webcam can honestly do

**Monocular depth is relative, not metric.** One measured distance fixes the
scale, and until a tape measure supplies it every distance this reports is
plausibly shaped and the wrong size. The model's output also wobbles ~4% frame to
frame on a static scene; per-frame scale alignment against the volume already
built is what keeps that wobble from thickening every surface. Anything beyond
~6 m is a guess and is clipped.

**And a monocular system has no absolute reference for where it is.** The pose is
measured against keyframes, which bounds the per-step error but not the
accumulated one — nothing here ever recognises a place it has been, so the map
drifts without limit over a long enough session. That is the loop closure named
at the top of this file, and it is the difference between what is built and SLAM.

The mesh this produces is a good room; it is not a survey.
