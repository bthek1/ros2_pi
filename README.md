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

**Both halves run end to end, and every distance is in metres.** Ten packages
build from source under both distros; a webcam on a Pi becomes a live triangle
surface on the dev box with a browser tab watching it. As of **2026-09-28**
milestones M0–M12 are done — **milestone F is closed**
([roadmap](docs/info/roadmap.md)):

| Stage | Package | Where | Measured |
| --- | --- | --- | --- |
| Capture | `pimesh_camera` | Pi | 720p MJPEG stamped at `VIDIOC_DQBUF`, **44–59 Hz** on the dev box, calibrated intrinsics (held-out reprojection 0.4955 px) |
| Capture (eval) | `pimesh_dataset` | dev box | TUM RGB-D fr1/desk at its own stamps and intrinsics, 613 of 613 frames |
| Decode | `pimesh_frontend` | dev box | **1.90 ms/frame**, the container's *one* network subscriber; zero-copy **504/504** with intra-process on vs **0/395** off |
| Keypoints | `pimesh_frontend` | dev box | ORB ×500, **57.8 Hz at 6.82 ms/frame** against an 8 ms budget |
| Pose | `pimesh_frontend` | dev box | PnP vs newest keyframe, **1.37 px over 94 inliers, 81.6% posed**, 16.6 Hz |
| Depth | `pimesh_depth` | dev box, **GPU** | Depth Anything V2 Small via ONNX Runtime CUDA, **55.1 ms/frame, 17.4 Hz** (287.9 ms on the CPU control) |
| Fusion | `pimesh_mapping` | dev box | TSDF at 15 mm voxels, **15.3 ms/integration at 17.1 Hz** |
| Surface | `pimesh_mapping` | dev box | marching cubes every 10 s, **2.8 s per extraction** off the hot path, 120 k triangles on `/world/mesh` |
| View | `pimesh_dashboard` | dev box, own process | HTTP + WebSocket, **10.01 Hz stats**, costs the pipeline **0.77%** |

**Depth is the pipeline's clock:** 17.4 Hz against a 59 Hz input. Every stage
drops rather than queues.

**Measured against an outside truth — M10, 2026-09-25.** TUM fr1/desk replayed
through the real container and scored by `evo`: **Sim(3)-aligned ATE RMSE
0.27–0.36 m** over seven runs of ~350 poses, 100% associated against the
motion-capture trajectory, RPE **0.14–0.15 m** over 1 s, with the
`rotation_only` control unable to be aligned at all. It is the first number
about the pose that this project did not produce, and it matters: the rotation
was once composed *inverted* for six days while every internal number
describing it was right. `bash tools/gates/trajectory.sh`.

**Pinned to metres with a tape measure — M11, 2026-09-28.** `depth_scale` =
**4.6002** from a 1.730 m tape figure (`bash tools/gates/scale.sh`: spread 0.0527
against a 0.10 ceiling, zero clipping, 346 of 351 frames). M10's Sim(3) fit had
independently bounded it at 4.6–5.2. The lesson worth more than the number:
**a blank wall defeats monocular depth** — it came back as a confident 1.7 m →
6.0 m ramp on a wall that was provably square-on — so point the camera at
texture.

**6-DoF odometry earns its keep — M12, 2026-09-29.** The reference clip
`bags/desk1` is a *pan*, so rotation alone explains almost all of its frame
motion and the two regimes tie there (0.3830 m against 0.3847 m of
paired-surface gap). `bags/walk1` carries 18.2 m of path, and on it 6-DoF wins:
**0.4545 / 0.5150 / 0.5204 m against rotation-only's 0.6688–0.7234**, five runs,
non-overlapping, the sign never flipping. `bash tools/gates/odom.sh walk1` now
asserts it. That question had been open since P7 in September.

It took three clips to record, and the reason is the finding above: this room's
walls are blank, and the first two walks spent 26% and 49% of their frames
facing them, where the tenth-percentile frame carried **9 ORB features**.

**Next is the SLAM half:** map points shared across keyframes and local bundle
adjustment ([#11](https://github.com/bthek1/ros2_pi/issues/11)), place
recognition, a pose graph and a rebuilt TSDF
([#12](https://github.com/bthek1/ros2_pi/issues/12)), then a `LOST` state and
relocalisation ([#13](https://github.com/bthek1/ros2_pi/issues/13)).

### Running it

```bash
just build          # colcon, with the flags this box needs
just view-mesh      # the pipeline on the Pi's camera, surface in RViz
just dashboard      # the same, watched from a browser tab
just --list         # everything you type on a normal day
```

The tests are the `tools/gates/*.sh` scripts, run directly — each exits 0 or
non-zero and prints the number it asserted on — plus **559 unit tests across
thirty-seven suites**, identical on both machines (`bash tools/test.sh`, catalogue
in [docs/info/testing.md](docs/info/testing.md)). Each closed milestone issue
records what its gates printed: [#4](https://github.com/bthek1/ros2_pi/issues/4)
capture, [#5](https://github.com/bthek1/ros2_pi/issues/5) decode and keypoints,
[#6](https://github.com/bthek1/ros2_pi/issues/6) depth,
[#7](https://github.com/bthek1/ros2_pi/issues/7) fusion and mesh,
[#8](https://github.com/bthek1/ros2_pi/issues/8) odometry and dashboard,
[#9](https://github.com/bthek1/ros2_pi/issues/9) calibration.

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
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones M0–M19 and their status |
| [docs/info/build-log.md](docs/info/build-log.md) | **How this got built, and the dozen times a number was wrong before it was right.** Read it before adding a gate |
| [docs/info/testing.md](docs/info/testing.md) | The unit suites and what each exists to catch |
| [docs/plans/README.md](docs/plans/README.md) | How plans are written here: a GitHub issue of stable phases, a command for a test, executable phases only |
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | The one phase list, P0–P20, each ending in a `tools/gates/*.sh` test, and the deferred register |

Plans live in the issue tracker, not in this tree: `gh issue list --label plan`.

## What one webcam can honestly do

**Monocular depth is relative, not metric.** One measured distance fixes the
scale — here a tape measure, 1.730 m to a wall, which put `depth_scale` at
4.6002 — and that constant is only as good as the surface it was read off. The model's output also wobbles ~4% frame to
frame on a static scene; per-frame scale alignment against the volume already
built is what keeps that wobble from thickening every surface. Anything beyond
~6 m is a guess and is clipped, and a textureless surface is a confident guess
at any range.

**And a monocular system has no absolute reference for where it is.** The pose is
measured against keyframes, which bounds the per-step error but not the
accumulated one — nothing here ever recognises a place it has been, so the map
drifts without limit over a long enough session. That is the loop closure named
at the top of this file, and it is the difference between what is built and SLAM.

The mesh this produces is a good room; it is not a survey.
