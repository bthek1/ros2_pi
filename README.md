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

**P0 done — the workspace builds on both machines.** `pimesh_msgs` and
`pimesh_bringup` exist and `just gate-build` passes: the same source builds
under Lyrical here and Jazzy on the Pi, the generated interfaces are
byte-identical across the two distros, and the static frame tree comes up. There
are no pipeline nodes yet; the container launches empty.

**P9 done — the Pi's configuration is a playbook in this repo.** `ansible/`
provisions it and `just gate-provision` passes: idempotent (first apply changed
11 tasks, every apply since changes nothing), the Pi's ROS environment equal to
this machine's, and the build dependencies asserted rather than assumed. Nothing
on the Pi is installed by hand — see
[docs/info/ansible.md](docs/info/ansible.md).

**P1 done — the Pi captures.** `camera_node` publishes
`/image_raw/compressed` at up to 59 Hz, within a percent of what raw `v4l2-ctl`
gets from the same camera, stamped from the buffer's own `CLOCK_MONOTONIC`
capture time: **5 ms** from stamp to receipt on the Pi, and **0.00 ms** of
movement between separate launches. The driver it replaces misses that by
0.2–1.0 s, redrawn every launch.

**P2 is next**: the dev-box container, and proving intra-process comms.

Everything else in `docs/` is design intent.
[docs/info/roadmap.md](docs/info/roadmap.md) tracks what has actually been
built. The pipeline numbers quoted below are measured, but they were measured on
the Python predecessor at [`~/Documents/piros2`](../piros2), which implements the
same pipeline on the same hardware.

```bash
just build            # dev box
just build-pi         # sync + build on the Pi
just provision        # apply the playbook to the Pi
just gate-build       # the P0 gate
just gate-provision   # the P9 gate
just stragglers       # sweep both machines
```

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
| Configured | by hand (control node) | by **Ansible**, from `ansible/` in this repo |

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
| [docs/info/ansible.md](docs/info/ansible.md) | Provisioning the Pi — the playbook owns the machine, the justfile owns the code |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones |
| [docs/plans/README.md](docs/plans/README.md) | How plans are written here: stable phases, a command for a test, executable phases only |
| [docs/plans/in-progress/bootstrap-plan.md](docs/plans/in-progress/bootstrap-plan.md) | The build order, P0–P9, each ending in a `just gate-*` test |
| [docs/plans/future/bootstrap-future.md](docs/plans/future/bootstrap-future.md) | Deferred work, each entry with the trigger that promotes it into the plan |

## What one webcam can honestly do

Monocular depth is **relative**, not metric — one measured distance fixes the
scale, and the model's output still wobbles ~4% frame to frame on a static
scene. Per-frame scale alignment against the volume already built is what keeps
that wobble from thickening every surface. Anything beyond ~6 m is a guess and
is clipped. The mesh this produces is a good room; it is not a survey.
