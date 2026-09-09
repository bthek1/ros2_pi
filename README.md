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

**The scaffolding runs; the pipeline is not started.** As of 2026-09-08 the
repository holds one package, `src/pimesh_hello/`, which publishes `"hello
world"` and exists to prove the structure everything else will be built on —
components composed in one container with the message handed over as a pointer,
parameters from a keyed YAML, the same source built under both distros, and a
session that leaves nothing running on either machine. `just build && just
hello-compose` is the whole getting-started path — the justfile is only ever the
handful of commands you type on a normal day. The five `tools/gates/hello-*.sh`
scripts are the tests, run directly, and
[gh issue #2](https://github.com/bthek1/ros2_pi/issues/2) records what each one
printed.

No camera, depth, fusion, mesh or dashboard code exists yet — that is
[docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md),
and [docs/info/roadmap.md](docs/info/roadmap.md) tracks it. The numbers quoted
throughout `docs/` are measured, but on the Python predecessor at
[`~/Documents/piros2`](../piros2), which implements the same pipeline on the
same hardware.

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

Measured on this hardware, via the predecessor:

| Stage | Cost |
| --- | --- |
| Capture + ship (Pi) | ~16 ms/frame, up to 60 fps at 720p MJPEG |
| Decode + ORB (dev box CPU) | ~9 ms/frame |
| **Depth (dev box GPU)** | **72–79 ms/frame** — 280–305 ms on CPU |
| TSDF integrate | ~15 ms/frame (target) |
| Mesh extraction | 300–900 ms, every ~10 s, off the hot path |

**Depth is the pipeline's clock: ~13 Hz.** Every stage drops rather than queues.

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
