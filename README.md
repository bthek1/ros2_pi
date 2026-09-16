# ros2_pi — a live 3D mesh of a room, from one webcam

A Raspberry Pi 5 with a single USB webcam, an Ubuntu dev box with a GPU, and
ROS 2 in between. The camera is the only sensor; everything the system knows
about the room's shape is inferred from one moving view.

```
RGB frame → keypoints (ORB) → monocular depth (Depth Anything V2) → TSDF fusion → triangle mesh → dashboard
```

**Written in C++.** The Pi is a sensor head — it captures, stamps and ships
JPEG, and nothing else. Every expensive stage runs on the dev box, on the GPU
where it pays.

## Status

**Six stages run: the camera reaches a triangle surface.** As of **2026-09-16**
the repository holds six packages, and a webcam on a Pi becomes a mesh on the dev
box. Only the dashboard is missing — and the surface does not look like a room
yet, for a reason given below:

- **Capture** (`pimesh_camera`, on the Pi) — 1280×720 MJPEG stamped with the
  kernel's capture time, **44–59 Hz received on the dev box**, serving real
  intrinsics on `/camera_info` (fx=953.4, fy=957.6, held-out reprojection
  0.4955 px).
- **Decode** (`pimesh_perception`, here) — the container's *one* subscriber on the
  only topic that crosses Wi-Fi, `cv::imdecode` at **1.90 ms/frame**, handing the
  2.7 MB frame to its consumers as a pointer: **504/504** buffer addresses matched
  with intra-process comms on against **0/395** with it off.
- **Keypoints** (`pimesh_perception`, here) — ORB at 500 features with pooled
  matching over a 10-frame window, **57.9 Hz sustained at 5.75 ms/frame** with
  depth running beside it, and a rotation-only `odom → base_link` that holds its
  last pose rather than guessing when its gates fail.
- **Depth** (`pimesh_perception`, here, on the GPU) — Depth Anything V2 Small at
  518² through ONNX Runtime's CUDA execution provider, **55.1 ms/frame and
  17.4 Hz** against an 80 ms budget, publishing `/depth` in metres alongside
  `/depth/rgb` — the exact frame each map was inferred on, **1048/1048 measured
  byte-identical**. The metres are the right *shape* and an arbitrary *size*:
  monocular depth is scale-ambiguous until something measures a known distance.
- **Fusion** (`pimesh_world`, here) — a spatially hashed TSDF at 15 mm voxels,
  **15.3 ms per integration** against a 20 ms budget and **17.1 Hz** sustained,
  with every frame posed at its own capture stamp and paired with the exact colour
  frame its depth was inferred on: 1030 of 1030 frames offered actually
  integrated, 0.19% displaced, 0 without a pose, 0 without a colour twin.
- **Surface** (`pimesh_world`, here) — marching cubes over a chunked snapshot of
  the volume every ten seconds, on a niced thread: **790 668 triangles in 2.8 s**,
  debris pruned, interior holes fanned shut with **every component's frontier left
  open**, decimated to **120 000** for `/world/mesh` and written full-detail as a
  **779 740-triangle** PLY. The worst gap between two integrations was **374.7 ms
  with meshing running against 401.3 ms in a control with nothing meshing** — the
  extraction costs the integrator nothing measurable.

`just build` then `bash tools/view-mesh.sh` shows it running; `just --list` is the
whole of what you type on a normal day. The tests are the `tools/gates/*.sh`
scripts, run directly, and each milestone issue records what they printed —
[#4](https://github.com/bthek1/ros2_pi/issues/4) for capture,
[#5](https://github.com/bthek1/ros2_pi/issues/5) for decode and keypoints,
[#6](https://github.com/bthek1/ros2_pi/issues/6) for depth,
[#7](https://github.com/bthek1/ros2_pi/issues/7) for fusion and the mesh,
[#9](https://github.com/bthek1/ros2_pi/issues/9) for the calibration.

**The mesh is not a room yet, and that is the honest state rather than a
disclaimer.** `keypoint_node` publishes **rotation only** — the translation is
identically zero — so a hand-held sweep's ~0.9 m of real arm arc is modelled as no
motion at all, and the same wall is integrated at a different distance every time
the camera turns. The volume fills with layers of it: 200 000 blocks is over
2000 m² of surface for a room with perhaps 60 m² in it. **P7 is what fixes that.**
And `depth_scale` is still arbitrary, so every distance the pipeline reports is
plausibly shaped and the wrong size — pinning it needs a person, a tape measure
and a recording of a surface at a known distance, which is the one thing in
milestone D a script could not close.

**No dashboard code exists yet** — that is
[docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md),
and [docs/info/roadmap.md](docs/info/roadmap.md) tracks it. Numbers quoted in
`docs/` for that stage are measured on the Python predecessor at
[`~/Documents/piros2`](../piros2), which implements the same pipeline on the same
hardware; numbers for the six stages above are this project's own.

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
| Runs | decode, keypoints, depth, fusion, meshing, dashboard | capture only |

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
| [Issue #2 — hello-world plan](https://github.com/bthek1/ros2_pi/issues/2) | Closed 2026-09-08: the scaffolding that exists, and what each gate measured |
| [Issue #3 — justfile plan](https://github.com/bthek1/ros2_pi/issues/3) | Closed 2026-09-09: grouped recipes, gate bodies in `tools/`, and the first shellcheck run over this repo's shell |
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | **Where this is going.** The build order P0–P8, each ending in a `tools/gates/*.sh` test — none started — followed by the deferred register, each entry with its trigger |

Plans live in the issue tracker, not in this tree: `gh issue list --label plan`.

## What one webcam can honestly do

Monocular depth is **relative**, not metric — one measured distance fixes the
scale, and the model's output still wobbles ~4% frame to frame on a static
scene. Per-frame scale alignment against the volume already built is what keeps
that wobble from thickening every surface. Anything beyond ~6 m is a guess and
is clipped. The mesh this produces is a good room; it is not a survey.
