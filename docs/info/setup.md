# Setup

Getting both machines to build and run this. Nothing here is installed by the
repo — read [hardware.md](hardware.md) for what is already present.

## Dev box

Most of it is already in place from the predecessor project: ROS Lyrical,
CycloneDDS pinned to `ens18`, `ROS_DOMAIN_ID=42`, ROS sourced for every shell.
What this project adds:

```bash
sudo apt install -y \
  ros-lyrical-rclcpp-components ros-lyrical-cv-bridge \
  ros-lyrical-image-transport ros-lyrical-image-transport-plugins \
  ros-lyrical-tf2-ros ros-lyrical-tf2-eigen ros-lyrical-vision-opencv \
  libopencv-dev libpcl-dev libeigen3-dev nlohmann-json3-dev
```

`libopencv-dev` is 4.10.0 with **no CUDA module**; PCL is 1.15.

### GPU

This is the part that needs doing and is not trivial. The driver (595.84) is
installed. Nothing else is.

**`nvcc` is absent and there is no CUDA runtime in `/usr/lib`.** Python's
`onnxruntime-gpu` works anyway because its wheels vendor the CUDA libraries — a
C++ build gets none of that.

Two things need the toolkit, and they are separable:

1. **ONNX Runtime with the CUDA execution provider** — needed by `depth_node`.
   The lowest-friction route is the **prebuilt GPU release tarball** from ONNX
   Runtime's GitHub releases: headers plus `libonnxruntime.so` and the CUDA
   provider shared library, unpacked into `/opt/onnxruntime` and found by CMake
   via an explicit `ONNXRUNTIME_ROOT`. It still needs the matching CUDA runtime
   and cuDNN present at load time — check the release's stated versions against
   what driver 595.84 supports before downloading, and **log which provider the
   session actually got at startup** so a silent CPU fallback is visible in one
   line rather than as "the mesh got slow".
2. **Hand-written CUDA kernels** (a future TSDF integrator) — needs a real
   `cuda-toolkit` install and a `find_package(CUDAToolkit)` in CMake. **Do not
   plan on this until the CPU integrator works and has been profiled.**

Sanity checks, in order:

```bash
nvidia-smi                       # driver alive, GPU visible
nvcc --version                   # currently absent — expected
ls /opt/onnxruntime/lib          # after installing the tarball
```

### Model weights

Depth Anything V2 Small, ONNX, ~99 MB. **Fetched and checksummed, never
committed** — `models/` and `*.onnx` are git-ignored.

```
https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx
sha256  afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c
```

The fetch belongs in a `just fetch-model` recipe that verifies the checksum and
exits early if the file is already good. **The model does not go on the Pi** —
inference is dev-box-only, so any sync to the Pi excludes it.

## Raspberry Pi

The Pi already runs `ros-jazzy-ros-base` with the environment provisioned by the
predecessor's Ansible tree. For this project it additionally needs a C++
toolchain and V4L2 headers:

```bash
ssh pi "bash -lc 'sudo apt install -y build-essential cmake \
  ros-jazzy-rclcpp-components ros-jazzy-image-transport v4l-utils libv4l-dev'"
```

`v4l-utils` comes from provisioning, not from the OS image — Ubuntu Server does
not ship it, and a fresh reflash loses it. Check before reporting a camera
command as broken.

## Building

The repo is the colcon workspace. Both machines build **from source** — there is
no ABI compatibility between Lyrical and Jazzy, so nothing is copied between
them but source.

```bash
just build                   # dev box
bash tools/build-pi.sh       # rsync source to the Pi, then build there under Jazzy
bash tools/test.sh           # build, then colcon test, then colcon test-result
bash tools/test-pi.sh        # the same, on the Pi, under Jazzy
bash tools/clean.sh          # drop build/ install/ log/ here
bash tools/clean-pi.sh       # and there
```

`just build` is the only one of these in the justfile, because it is the only
one typed every day; the rest are scripts, like everything else that is not a
day-to-day action.

**Use these rather than bare `colcon`.** `just build` expands to

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

and that argument is not optional here — without it every `ament_cmake` package
fails at configure time with `No module named 'catkin_pkg'`. See
[troubleshooting.md](troubleshooting.md).

`tools/sync-pi.sh` ships `src/`, `tools/` and the `justfile` with `rsync --delete`,
and **never** `build/`, `install/` or `log/`. It then runs
`tools/check-stale.sh` on the Pi and clears the Pi's build tree if the sync left
artefacts whose sources are gone — colcon never forgets a package on its own, so
a stale overlay will otherwise keep answering `ros2 pkg list` with packages that
no longer exist.

Keep everything the Pi builds at **C++17** and to APIs that exist in both
distros. A build that only succeeds here is half a build — and the failure is
not always in the direction you expect: `ament_target_dependencies()` is gone in
Lyrical and still present in Jazzy, so that one breaks *here* on CMake the Pi
would accept.

First-build times, measured 2026-09-08 on `pimesh_hello` (one small package, two
components): **~9 s** on the dev box, **20.5 s** on the Pi.

## Running

`just` with no arguments lists what exists, by group. Today that is the
scaffolding only:

<!-- gate-justfile keeps the block below equal to `just --list`. Edit the
     justfile, then re-run `bash tools/gates/justfile.sh`; do not edit by hand. -->

```text
Available recipes:
    default                    # List the recipes

    [build]
    build *args                # Build the workspace

    [run]
    hello-compose seconds="30" # Hello world, here: both components in one container. seconds = how long to run
    hello-lan seconds="20"     # Hello world, across the LAN: talker on the Pi, listener here
    view-camera seconds="600"  # The Pi's camera and the frame tree, in RViz. A viewer, not evidence
```

That is the whole list, and the shortness is the point: `build` is how you
build, `run` is what you start in order to *watch* something. If you have just
cloned this, `just build && just hello-compose` is the entire getting-started
path.

**Everything else is a script in `tools/`, run directly.** The gates especially
— there are ten, they are run constantly, and as recipes they had buried
`hello-compose` under an alphabetised wall of `gate-*`:

```bash
# The pipeline, phase by phase
bash tools/gates/build.sh         # P0: one source tree, two distros, same messages
bash tools/gates/capture.sh       # P1: 720p MJPEG on the LAN, stamped honestly

# Across all phases
bash tools/gates/test.sh          # the unit tests pass on both machines, and there are some
bash tools/gates/view-configs.sh  # every .rviz topic is one src/ publishes
bash tools/gates/justfile.sh      # the justfile's own shape, plus shellcheck over tools/

# The scaffolding (gh issue #2), still asserted
bash tools/gates/hello-build.sh   # one real package builds
bash tools/gates/hello-talk.sh    # the talker honours its rate parameter
bash tools/gates/hello-ipc.sh     # one process, message handed over as a pointer
bash tools/gates/hello-lan.sh     # one source tree, two distros, over the LAN
bash tools/gates/hello-clean.sh   # Ctrl-C leaves nothing running, either machine,
                                  #   for every recipe in the justfile's `run` group

bash tools/camera-reset.sh        # clear the camera's persistent V4L2 controls
bash tools/stragglers.sh          # assert nothing outlived its session
bash tools/sync-pi.sh             # ship source to the Pi — source only
bash tools/build-pi.sh            # ...and build it there, under Jazzy
bash tools/clean.sh               # delete the colcon trees (clean-pi.sh for the Pi's)
```

**A gate and a unit test are different things.** The gates above are the phase
tests — slow, mostly needing the Pi and the camera, and what actually closes a
claim. `bash tools/test.sh` runs `colcon test`: 29 hermetic tests over four
suites that need no hardware and run identically under both distros. Note that
`colcon test` exits 0 when a test *fails*, and again when a package has no tests
at all, so `colcon test-result --all` is what decides — which is why
`tools/gates/test.sh` asserts on the counts rather than on the exit status.

Each gate exits non-zero and prints the number it asserted on.
[gh issue #2](https://github.com/bthek1/ros2_pi/issues/2) records what each
`hello-*` gate measured, and `tools/gates/justfile.sh` is
[#3](https://github.com/bthek1/ros2_pi/issues/3)'s.

**The justfile is the user-facing surface; the shell is in `tools/`.** Every
recipe is one line that runs a script — `tools/build.sh` — because `just` gives a recipe body no way to share code with another recipe, so
inlined bash gets copy-pasted and drifts. `tools/just-lib.sh` is what they all
source: the prelude, the single spelling of the Pi's `ssh` invocation, the
bracketed `pkill` patterns and `in_range`. Two consequences worth knowing:
the scripts run without `just` (which is why `gate-hello-clean` can signal one
inside a bare `setsid` session), and they can be linted — `shellcheck` cannot
parse `{{ }}`, so none of this bash was checked by anything until it moved.
Install it with `uv tool install shellcheck-py` (no sudo needed) or
`sudo apt install shellcheck`; `bash tools/gates/justfile.sh` runs it over all
of `tools/` and asserts zero findings.

**Planned, not yet written** — `just cam` (Pi-side camera), `just pipeline` (the
dev-box container), `just dev` (both plus the dashboard), `just dash`. They
arrive with the phases of
[../plans/future/project_final_state.md](../plans/future/project_final_state.md)
that build the nodes they run.

Every session recipe **tears itself down on both machines**: the viewer runs in
the foreground and `arm_cleanup` (in `tools/just-lib.sh`) installs a handler on
EXIT and on INT/TERM/HUP that `pkill -f`s each node pattern, locally and over
SSH, so Ctrl-C and closing the window both work. The handler cleans up once and
re-raises, so an interrupted run exits 130 rather than falling through to the
next line. When a session gains a node, its pattern goes into `PIMESH_PATTERNS`
in the same change — and `bash tools/stragglers.sh` is how you find out that it
did not.

**Bound a foreground command with `run_for`, never a bare `timeout`.** GNU
`timeout` moves its child into its own process group, where a terminal's Ctrl-C
cannot reach it, and bash will not run the trap until that foreground child
returns — so the session ignores Ctrl-C entirely and ends when the timer
expires. `run_for` (`timeout --foreground -s INT`) keeps the child in the
caller's group, which is why `just hello-compose` now dies 0.30 s after the
keypress with `ros2 launch` shutting down gracefully. Full account in
[troubleshooting.md](troubleshooting.md).

**A leaked camera process holds `/dev/video0` exclusively**, and every later
session then dies with `Device or resource busy`. Check both machines are clean
before walking away.

## Environment traps

- **`ssh pi '...'` does not read the ROS environment.** Use `ssh pi "bash -lc
  '...'"` — otherwise you are silently on domain 0 with the default RMW and the
  output means nothing.
- **Every scripted `ssh pi` needs `-o BatchMode=yes -o ConnectTimeout=5`.** A
  bare ssh hangs ~2 minutes against a dead Wi-Fi link and wedges the trap it
  sits in.
- **Restart the ROS daemon after changing any `ROS_*` or DDS variable**
  (`ros2 daemon stop && ros2 daemon start`). It caches discovery state and will
  otherwise show a stale graph — which masks fixes that actually worked.
- **`python3` here is PlatformIO's venv**, which shadows the system Python for
  `#!/usr/bin/env python3` shebangs. rqt tools crash with
  `No module named 'yaml'`; prefix `PATH=/usr/bin:$PATH`.
- **A second Python breaks the *build*.** CMake's `FindPython3` picks
  `~/.local/bin/python3.14` (uv-managed, no `catkin_pkg`) over `/usr/bin/python3`
  (apt's, which owns ROS's `dist-packages`), and `ament_cmake` shells out to
  Python at configure time — so a C++-only package fails to build. `just build`
  passes `-DPython3_EXECUTABLE=/usr/bin/python3`; a hand-run `colcon build` does
  not.
- **`pkill -f <plain word>` kills the shell that runs it**, because `-f` matches
  full command lines including its own. Bracket the first character and anchor
  on the installed path: `pkill -f '/lib/[p]imesh_hello/'`.
- **`rviz2` needs `QT_QPA_PLATFORM=xcb`** on this Wayland session. It was
  measured running on hardware GL (4.6) with driver 595.84, so the old
  software-GL workaround is obsolete.
