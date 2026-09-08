# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

**One webcam on a Raspberry Pi, a live 3D mesh of the room on the dev box — in C++.**

The camera is the only sensor. Everything downstream is inference and geometry:

```
RGB frame → keypoints (ORB) → monocular depth (Depth Anything V2) → TSDF fusion → triangle mesh → dashboard
```

The Pi is a **sensor head only**: capture, stamp, ship JPEG. Every expensive
stage runs on the dev box, on the GPU where it pays.

**This repo is a C++ rewrite.** The predecessor
[`~/Documents/piros2`](../piros2) is a working Python implementation of the same
pipeline (`piros2_world_mesh`, `piros2_perception`). It is **reference, not a
dependency** — read it for measured numbers, traps and topic shapes, but do not
copy its structure wholesale: the point of the rewrite is to do in one process
with `rclcpp` components what Python needed three processes and two interpreters
to do.

### Status: the scaffolding runs, the pipeline does not

As of **2026-09-08** there is exactly one package, `src/pimesh_hello/`, and it
exists to prove the structure rather than to do anything: a C++ `ament_cmake`
package, two `rclcpp_components` components composed into one container with
intra-process comms measured handing over the pointer, parameters from a keyed
YAML, the same source built from scratch under **both** distros, and a session
that tears itself down on either machine. Five `just gate-hello-*` recipes
assert all of it — [gh issue #2](https://github.com/bthek1/ros2_pi/issues/2)
carries the numbers each one printed.

**Nothing of the actual pipeline exists.** No camera, no depth, no fusion, no
mesh, no dashboard — that is
[gh issue #1](https://github.com/bthek1/ros2_pi/issues/1), and everything the
rest of `docs/` says about those stages is **design intent**, not a description
of running code. When you build something, change the doc that describes it from
future tense to a measured statement, and say what you measured it with.

Do not write "the node publishes X at Y Hz" until a node has published X and you
have watched it do Y.

## The two machines

Measured 2026-09-01 unless noted.

| | Dev box (here) | Raspberry Pi |
| --- | --- | --- |
| Reach it | local | `ssh pi` (key auth, works with `BatchMode=yes`) |
| OS | Ubuntu 26.04.1 LTS "resolute", x86_64, kernel `7.0.0-30-generic` | Ubuntu 24.04.4 LTS "noble", aarch64, kernel `6.8.0-1060-raspi` |
| ROS | **Lyrical** (`/opt/ros/lyrical`) | **Jazzy** (`/opt/ros/jazzy`) |
| CPU / RAM | 16 threads / 18 GB | 4 cores / 8 GB |
| GPU | **GTX 1660 SUPER, 6 GB, driver 595.84** | none |
| Runs | everything except capture | `pimesh_camera` and nothing else |
| Network | LAN on `ens18` | LAN over **`wlan0`** — Wi-Fi, no cable |

**The two machines are on different ROS distros.** That is inherited from
`piros2` and it is deliberate, not drift: `packages.ros.org` is pinned by Ubuntu
suite, so the dev box's 26.04 upgrade replaced every `ros-jazzy-*` package with
`ros-lyrical-*`. Jazzy ↔ Lyrical interop was measured working over the LAN in
`piros2` on 2026-08-31 (topics, `camera_info`, `tf_static` all crossed).

**Consequence for C++, and it is the sharpest one in the project:** ROS 2 has no
ABI compatibility guarantee across distros. A `.so` built here does not run
there. **Every package must build from source on both machines** — no
cross-compiled binaries, no shipped `install/` tree, and nothing in
`pimesh_camera` may depend on a Lyrical-only API. C++17 (Jazzy's baseline), not
C++20, in anything the Pi builds.

The Pi is reachable non-interactively, so **verify hardware claims by running
commands over SSH** rather than assuming:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 pi "bash -lc 'v4l2-ctl --list-devices'"
```

A non-interactive `ssh pi '...'` does **not** get the ROS environment — the
exports live in `~/.profile`. Always use a login shell (`bash -lc`), or you are
silently on domain 0 with the wrong RMW and the result means nothing.

Full specs: [docs/info/hardware.md](docs/info/hardware.md).

## The pipeline

Five stages, one node each, all but the first on the dev box. Full design with
message types and rates: [docs/info/pipeline.md](docs/info/pipeline.md).

| Stage | Node | Where | Budget |
| --- | --- | --- | --- |
| Capture | `camera_node` | Pi | 1280×720 MJPEG, up to 60 fps, stamped at `VIDIOC_DQBUF` |
| Keypoints | `keypoint_node` | dev box | ORB, 500 features, ~5 ms/frame target |
| Depth | `depth_node` | dev box, **GPU** | Depth Anything V2 Small, 518², **72–79 ms/frame measured on this GPU** |
| Fusion | `fusion_node` | dev box | TSDF, 1.5 cm voxels, integrate at depth rate |
| Surface | `mesh_node` | dev box | marching cubes, re-mesh every ~10 s |
| View | `dashboard_node` | dev box | web UI, 10 Hz stats, ~10 fps preview |

The 72–79 ms figure is real: it is what `piros2` measured for the same ONNX model
on this exact GPU through the CUDA execution provider (CPU fallback was
280–305 ms). **Depth is the pipeline's clock.** Nothing downstream of it can run
faster than ~13 Hz, and design accordingly — do not build a fusion stage that
assumes 30 Hz input.

## Constraints that are easy to get wrong

Items marked *(inherited)* were measured in `piros2`, not here. Trust them as
strong priors, re-verify before quoting a number as this project's own.

- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is
  ~83 MB/s over the Pi's Wi-Fi. Compressed transport only, and exactly **one
  subscriber on the dev box** — see the next item.
- **One Wi-Fi reader, not five** *(inherited, measured 2026-08-16)*. Five RELIABLE
  subscribers each pull their own unicast copy and collapse the link into a
  retransmit storm: ~2 frames/s per reader against 14.7 Hz for a single reader.
  In C++ this is solved properly: **the dev box runs one component container
  with intra-process communication on**, so the decode happens once and every
  downstream component gets a `shared_ptr` to the same buffer. That is the
  main structural reason this rewrite exists — do not break it by launching
  components as separate processes "for debugging".
- **BEST_EFFORT delivers zero large frames** *(inherited)*. Megabyte-class
  messages fragment past the socket buffer and never reassemble. Every image and
  depth topic here is `RELIABLE` + `KEEP_LAST(1)` — freshest frame, no backlog.
- **Never gate on `header.stamp` age** *(inherited)*. `usb_cam` 0.8.1 has a
  once-per-process epoch bug that puts stamps a random sub-second amount in the
  past, redrawn at every launch. A stamp-age freshness gate silently dropped
  100% of frames. Stamp *deltas* are trustworthy (they are kernel capture
  intervals) — absolute ages are not. **`pimesh_camera` exists partly to fix
  this**: stamp from `CLOCK_MONOTONIC`-derived `v4l2_buffer.timestamp` at
  dequeue, and once it is verified, this constraint becomes a non-issue for our
  own capture path only.
- **V4L2 controls persist inside the camera** *(inherited)* across processes and
  reboots. A manual exposure left by a benchmark makes every later session
  black; the C922 powers on with `exposure_dynamic_framerate=1`, which costs
  ~10 fps in indoor light. Treat camera state as inspectable machine state and
  reset it before diagnosing black frames or low fps as a software bug.
- **`/dev/video1` is not a capture device** — it is the C922's UVC metadata node.
  Capture is `/dev/video0`.
- **The GPU has no CUDA toolkit installed** (measured: `nvcc` absent, no
  `libcudart` in `/usr/lib`). The driver is there (595.84) and Python's
  `onnxruntime-gpu` works because pip wheels vendor the CUDA runtime. **A C++
  build gets none of that** — `depth_node` needs the ONNX Runtime GPU release
  tarball, and any hand-written CUDA kernel needs a real toolkit install first.
  [docs/info/setup.md](docs/info/setup.md#gpu) has the plan; do not assume `nvcc`.
- **The GPU is Turing TU116: compute capability 7.5, 6 GB, no tensor cores.**
  fp16 buys bandwidth, not math throughput. Budget fp32 and do not plan around
  TensorRT fp16 speedups you have not measured.
- **The apt OpenCV (4.10.0) has no CUDA module.** `cv::cuda::` will not link.
  ORB runs on the CPU; that is fine at 500 features, but do not write code that
  reaches for `cv::cuda` and then "fix" it by building OpenCV from source
  without saying so.
- **PCL 1.15 and `pcl_ros` are installed; Open3D is not.** The Python side
  needed Open3D for TSDF and meshing and paid for it with a process boundary
  (no Python 3.14 wheel). In C++, write the TSDF and marching cubes ourselves or
  use PCL — that boundary should not come back.
- **`ROS_DOMAIN_ID=42`** on both machines, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  and CycloneDDS pinned to `ens18` here / `wlan0` there. The dev box has Docker
  bridges and a Tailscale interface DDS will happily bind to instead, advertising
  an address the Pi cannot route to. After changing any `ROS_*` or DDS variable,
  `ros2 daemon stop && ros2 daemon start` — it caches discovery state and will
  otherwise show you a stale graph and mask a fix that worked.
- **Two Pythons shadow the system one on this box**, and they break different
  things. PlatformIO's venv wins `#!/usr/bin/env python3`, so rqt and other GUI
  tools crash with `No module named 'yaml'` — prefix `PATH=/usr/bin:$PATH`. A
  uv-managed `~/.local/bin/python3.14` wins CMake's `FindPython3`, so **every
  `ament_cmake` package fails at `ament_package()`** with
  `No module named 'catkin_pkg'` (measured 2026-09-08): ament shells out to
  Python at *configure* time, so a C++-only package is not immune. Build with
  `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`, which `just build`
  already passes. Running C++ nodes are immune; building them is not.
- **The session is Wayland.** `rviz2` renders through GLX and needs
  `QT_QPA_PLATFORM=xcb`; it was measured working with hardware GL (4.6) on
  driver 595.84 as of 2026-08-31, so the old software-GL workaround is obsolete.
- **The Pi's Wi-Fi link dies while the Pi keeps running** *(inherited)*. Never
  diagnose an unreachable Pi as "crashed" without evidence — `ping` first, then
  read `journalctl -b -1` after recovery. Every scripted `ssh pi` must carry
  `-o BatchMode=yes -o ConnectTimeout=5`; a bare ssh hangs ~2 minutes against a
  dead link and wedges whatever trap it sits in.

## Conventions

- **This repo is the colcon workspace.** Packages go in `src/`, named
  `pimesh_<thing>` — `pimesh_camera`, `pimesh_perception`, `pimesh_world`,
  `pimesh_dashboard`, `pimesh_msgs`, `pimesh_bringup`. (Not `ros2_pi_*`: a
  `ros2_` prefix reads as core tooling.)
- **C++ only for nodes.** `ament_cmake`, C++17, no Python nodes. Launch files
  and one-off tools may be Python — that is not a licence to move logic there.
- **Nodes are `rclcpp_components` components**, registered with
  `RCLCPP_COMPONENTS_REGISTER_NODE`, each with a thin `*_main.cpp` so it can also
  run standalone. The bringup launch composes the dev-box ones into a single
  container with `use_intra_process_comms=True`. A node that only works
  standalone is a bug.
- **No work in a subscription callback beyond a bounded copy.** Anything that
  costs milliseconds (inference, fusion, meshing) runs on its own thread with a
  single-slot mailbox: newest frame wins, older one dropped. Queues that grow
  are how this pipeline dies.
- **Parameters live in `config/*.yaml`, keyed by node name**, and launch files in
  `launch/*.launch.py`. A key that does not match the node name silently applies
  nothing — a trap that has cost this project's predecessor real time. Declare
  every parameter with a description and validate ranges at declaration.
- Build with `colcon build --symlink-install`. Day-to-day commands are `just`
  recipes; add a recipe rather than documenting a long one-off command, and keep
  recipes and docs in agreement.
- **Sessions tear themselves down — no stragglers.** Ctrl-C and closing the
  window must both end everything the recipe started, **on both machines**. The
  mechanism: viewer in the foreground, `trap … EXIT` that `pkill -f`s every node
  pattern the recipe started (bash fires EXIT on Ctrl-C too). Killing a
  background `bash -lc` wrapper orphans its grandchildren — always pattern-match
  the node, never `kill %N`.
- **This applies to ad-hoc runs too — that means you, Claude.** Anything you
  start by hand while verifying has no EXIT trap. Bound it up front
  (`timeout -s INT 30 …`, and on the Pi
  `ssh pi 'timeout -s INT 30 bash -lc "…"'`) or `pkill -f` it when done, and
  check both hosts are clean before reporting. **A leaked camera process holds
  `/dev/video0` exclusively** and every later session dies with
  `Device or resource busy`.
- **Claims are closed by scripts, not by eyes.** A gate names its evidence: a
  number on a topic, a log line with a threshold, a rendered image file. Reserve
  "needs a human" for the physical world — a tape-measure scale check, exposure
  in a real room, whether the mesh looks like the room. The RViz window is a
  viewer, not the evidence.
- **Docs split by kind**: reference lives in `docs/info/`; `docs/plans/` holds
  only the rules and the future files.
- **A plan is a GitHub issue, never a markdown file in this repo.** When asked
  to write a plan, open one with
  `gh issue create --label plan --title "<name> plan — <one line>" --body-file <file>`
  — the issue body *is* the plan. Do not create `docs/plans/*.md`, and do not
  paste a plan into chat instead of filing it. Add the `deferred` label if it is
  written down but not being started now.
- **Completion is closing the issue** —
  `gh issue close <n> --reason completed` — and that is the only status change
  there is. Close it only when **every phase is annotated done in the body and
  its gate has been run**, not when the code exists; abandoned work is closed
  with `--reason "not planned"` and a comment saying what replaced it. A closed
  plan issue is the **build log**: annotate phases as you go with the date and
  what the test printed, and never edit that history out. Fix inbound links in
  `docs/plans/README.md` and here when a plan's status changes.
- **Every plan is a list of executable phases, and nothing else.** The
  three rules, in full in [docs/plans/README.md](docs/plans/README.md):
  1. **Stable phases.** `## P0`, `## P1`, … Once written, a phase's number and
     scope never change, so "P3" means the same thing in every doc, commit
     message and conversation. Record progress by annotating the phase with
     dates and what actually happened — never by renumbering or reshuffling.
  2. **Every phase ends in a test that is a command.** Not "verify it looks
     right", not "check the mesh" — a recipe someone can run that exits 0 or
     non-zero and prints the number it asserted on. The test recipe is written
     in the same change as the phase's code.
  3. **Executable phases only.** A phase must be startable now, by the person
     reading it, with the hardware and code that exist. Anything that is
     waiting on something — a later phase, a purchase, an upstream release, or
     the passage of time — **is not a phase**. Never write a phase like "check
     back in 48 hours", "monitor for a week", "revisit once we have more data",
     or "decide later whether to keep it".
- **Deferred work lives in `docs/plans/future/`, never in a plan.** Each plan
  issue may have one companion `docs/plans/future/<name>-future.md` holding the items
  that are not executable yet. Every entry there names **the trigger that would
  make it executable** — the measurement, the phase, or the hardware it is
  waiting on. When the trigger fires, the entry is deleted from the future file
  and appended to the plan **issue's body** as the **next unused phase number**,
  with a test. Moving work into a plan is the only way it gets built; moving it
  into the future file is the only way it gets deferred. It never sits in both.
- `build/`, `install/`, `log/`, model weights and bag files are git-ignored.

## Working here

This is a **learning project**: the value is in understanding ROS 2, real-time
C++ and 3D reconstruction, not in shipping a product. So when implementing
something, explain the concept it exercises — why this QoS, what the executor is
actually doing, why the TF tree is shaped this way. Prefer the idiomatic ROS 2
way over a shortcut that happens to work. Build in steps that each run rather
than generating a finished subsystem the user cannot reason about.

Before writing code that touches an area, read its doc in `docs/info/` and the
corresponding `piros2` implementation. Most sharp edges here have already cut
somebody once.

## Documentation map

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/info/architecture.md](docs/info/architecture.md) | The node graph, topics, TF tree, and where each stage runs |
| [docs/info/pipeline.md](docs/info/pipeline.md) | Stage by stage: algorithms, libraries, message shapes, cost budgets |
| [docs/info/dashboard.md](docs/info/dashboard.md) | The web dashboard: transport, payloads, mesh streaming, layout |
| [docs/info/hardware.md](docs/info/hardware.md) | Measured specs of both machines, the camera, and the GPU |
| [docs/info/setup.md](docs/info/setup.md) | Getting both machines to build and run this, including the GPU stack |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause, mostly inherited and worth reading before debugging |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones and their status |
| [docs/plans/README.md](docs/plans/README.md) | How a plan is written here: a GitHub issue of stable phases, a command for a test, executable-only, and the future file |
| `gh issue list --label plan --state all` | **The plans themselves.** [#2 hello-world](https://github.com/bthek1/ros2_pi/issues/2) — closed 2026-09-08, the build log for the scaffolding that exists; [#1 bootstrap](https://github.com/bthek1/ros2_pi/issues/1) — the whole pipeline, P0–P8, not started; [#3 justfile](https://github.com/bthek1/ros2_pi/issues/3) — group the recipes, move gate bodies to `tools/`, not started |
| [docs/plans/future/hello-world-future.md](docs/plans/future/hello-world-future.md) | Work deferred out of the hello-world plan, each entry with its trigger |
| [docs/plans/future/bootstrap-future.md](docs/plans/future/bootstrap-future.md) | Work deferred out of the bootstrap plan, each entry with the trigger that would make it executable |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note
the date.
