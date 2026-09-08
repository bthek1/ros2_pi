# Setup

Getting both machines to build and run this. Read [hardware.md](hardware.md) for
what is already present. **The Pi's half of this is automated** — it is an
Ansible playbook in this repo ([ansible.md](ansible.md)), and the commands below
are shown so the roles are readable, not so you run them by hand. The dev box's
half is manual, deliberately: it is the control node and its GPU stack is a
one-off.

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

**Done at P4, 2026-09-08.** `just install-onnxruntime` installs both halves and
proves the CUDA provider resolves; `just gpu-probe` then measures the model on
the GPU with no ROS in the picture. Measured: **52.5 ms/frame** for Depth
Anything V2 Small at 518², against the predecessor's 72-79 ms for the same ONNX
file on the same GPU.

```bash
just install-onnxruntime   # ONNX Runtime 1.29.0 (cuda13) + the CUDA runtime
just fetch-model           # the model, checksummed
just gpu-probe             # prints the provider and the per-frame cost
```

What follows is what that automates, and the traps it took to get there.

**`nvcc` is absent and there is no CUDA runtime in `/usr/lib`.** Python's
`onnxruntime-gpu` works anyway because its wheels vendor the CUDA libraries — a
C++ build gets none of that.

Two things need the toolkit, and they are separable:

1. **ONNX Runtime with the CUDA execution provider** — needed by `depth_node`.
   The **prebuilt GPU release tarball**, `onnxruntime-linux-x64-gpu_cuda13`:
   headers, `libonnxruntime.so`, the CUDA provider, and a CMake config. Found
   by CMake through `ONNXRUNTIME_ROOT`, which `just build` exports.
   **cuda13, not cuda12** — the runtime that works on this box with driver
   595.84 is 13.3, and picking the wrong one loads, finds nothing, and falls
   back to CPU.

   **The tarball ships no CUDA runtime.** `objdump -p` on the CUDA provider
   names four sonames it needs and does not carry: `libcudart.so.13`,
   `libcublas.so.13`, `libcublasLt.so.13`, `libcurand.so.10`. Those come from
   NVIDIA's **pip wheels**, pinned — the runtime alone is ~2 GB against the apt
   toolkit's ~5, needs no root and no apt repo, and these versions are the ones
   already measured working here. `nvcc` stays absent, which is correct:
   nothing in P4 compiles CUDA.

   **It installs to `~/.local/opt`, not `/opt`.** `sudo` on this box needs a
   password, and a recipe that stops to prompt for one cannot be run by a gate.
   Set `PIMESH_OPT=/opt` if you would rather install it as root.

   **The CUDA libraries are dlopened by ONNX Runtime's provider, not linked by
   our binary**, so no rpath of ours reaches them. Every recipe that starts the
   container exports `LD_LIBRARY_PATH`. Forget it and the session silently
   falls back to the CPU: ~290 ms a frame instead of ~55, correct-looking depth,
   and a warning nobody reads. `depth_node` logs the provider it got at
   **ERROR** level when it is not CUDA, and `just gate-depth` asserts it from
   two independent places — the startup log and the per-second stats field.
2. **Hand-written CUDA kernels** (a future TSDF integrator) — needs a real
   `cuda-toolkit` install and a `find_package(CUDAToolkit)` in CMake. **Do not
   plan on this until the CPU integrator works and has been profiled.**

Sanity checks, in order:

```bash
nvidia-smi                            # driver alive, GPU visible
nvcc --version                        # absent, and expected to be
ls ~/.local/opt/onnxruntime/lib       # after just install-onnxruntime
just gpu-probe                        # the answer that actually matters
```

### Model weights

Depth Anything V2 Small, ONNX, ~99 MB. **Fetched and checksummed, never
committed** — `models/` and `*.onnx` are git-ignored.

```
https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx
sha256  afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c
```

`just fetch-model` verifies the checksum and exits early if the file is already
good. It checks **before** the file lands at its final path, so a truncated
download can never become the file `depth_node` loads. **The model does not go
on the Pi** — inference is dev-box-only, and `sync-pi` excludes `models/`.

`models/` is in the source workspace, not the installed `share/` tree, so the
launch cannot compute the path: the `just` recipes export `PIMESH_MODEL` and
the launch reads it, overridable with `model:=`.

## Raspberry Pi

**The Pi is provisioned by Ansible, not by hand** — design, roles and traps in
[ansible.md](ansible.md). The one command is:

```bash
just provision          # ansible-playbook site.yml  (the Pi is the only host)
just gate-provision     # applies it twice; the second run must report changed=0
```

Both recipes are built by **P9**; until then the Pi is still provisioned by the
predecessor's tree at `~/Documents/piros2/ansible`.

The plain-`apt` equivalent, so the roles are not a black box — this is what the
`toolchain` and `camera` roles assert:

```bash
ssh pi "bash -lc 'sudo apt install -y build-essential cmake \
  ros-jazzy-rclcpp-components ros-jazzy-image-transport \
  ros-jazzy-camera-info-manager v4l-utils'"
```

**Run it through the playbook, not this line.** A package installed by hand
survives until the next reflash and then vanishes, and nothing records that the
project depended on it. `v4l-utils` is exactly that case: it is on the Pi today
because the predecessor's playbook put it there, and Ubuntu Server does not ship
it.

Measured **2026-09-02**: everything in that list is already installed, and
`linux/videodev2.h` is present from `linux-libc-dev` — which is all a raw-ioctl
V4L2 node compiles against, so **`libv4l-dev` is not needed**. (It provides the
`libv4l2` format-conversion wrapper; `camera_node` does MJPEG passthrough and
never converts.) The toolchain is g++ 13.3.0 and cmake 3.28.3 — g++ 13 is also
why the Pi's packages are held at **C++17**.

## Building

The repo is the colcon workspace. Both machines build **from source** — there is
no ABI compatibility between Lyrical and Jazzy, so nothing is copied between
them but source.

**Use `just build`, not a bare `colcon build`.** The recipe carries three
things a hand-typed command does not: `/usr/bin` first on `PATH` (rosidl
generates message code in Python), `ONNXRUNTIME_ROOT` (without it the package
still builds, minus `depth_node`), and **`-DCMAKE_BUILD_TYPE=RelWithDebInfo`**.
That last one is not a preference: colcon's default build type is the empty
string, which passes no `-O` flag at all, and the rotation estimator measured
1.19 ms/frame unoptimised against 0.03 ms optimised.

```bash
# dev box: everything
just build
source install/setup.bash

# Pi: the two packages it runs
rsync -av --delete --exclude build --exclude install --exclude log \
      --exclude models ~/Documents/ros2_pi/ pi:~/ros2_pi/
ssh pi "bash -lc 'cd ~/ros2_pi && colcon build --symlink-install \
        --packages-select pimesh_msgs pimesh_camera'"
```

Keep the two `pimesh_camera`-side packages at **C++17** and to APIs that exist in
both distros. A build that only succeeds here is half a build.

## Running

Recipes that exist today (P0-P4, P9, P10):

```bash
just build              # colcon build here, optimised, with ONNXRUNTIME_ROOT set
just sync-pi            # source only — no build products cross the distro boundary
just build-pi           # sync, then build on the Pi
just fetch-model        # the depth model, checksummed (P4)
just install-onnxruntime  # ONNX Runtime GPU + the CUDA runtime it dlopens (P4)
just gpu-probe          # prove the GPU runs the model, with no ROS involved (P4)
just provision-check    # dry-run the Pi's playbook, with diffs
just provision          # apply it
just cam                # run the Pi's camera (just cam 30 bounds it at 30 s)
just camera             # every V4L2 control, current vs default
just camera-reset       # restore the C922's known-good baseline — DO THIS BEFORE RECORDING
just record NAME SECS   # record the camera stream to bags/NAME (P3)
just pipeline           # the dev-box container: decode + keypoints + depth
just test               # gtest + pytest on the dev box
just test-pi            # the Pi's gtest cases under Jazzy
just gate-build         # P0
just gate-capture       # P1
just gate-ipc           # P2
just gate-keypoints     # P3
just gate-depth         # P4
just gate-provision     # P9
just stragglers         # sweep both machines for leftovers
```

Later phases add `just mesh-views` (P6), `just dash` (P8), and one `just gate-*`
per phase. **Keep this list and the justfile in agreement** — a recipe
documented but absent is worse than one that was never mentioned.

Every session recipe **tears itself down on both machines**: the viewer runs in
the foreground and a `trap … EXIT` `pkill -f`s each node pattern the recipe
started, so Ctrl-C and closing the window both work. When a session gains a
node, its pkill pattern goes into the trap in the same change.

**A leaked camera process holds `/dev/video0` exclusively**, and every later
session then dies with `Device or resource busy`. Check both machines are clean
before walking away.

## Working in VS Code

The workspace configuration is **checked in** (`.vscode/`), because it encodes
project facts rather than personal taste: which Python interpreter is safe here,
where the compile database lives, and what the tasks are. Only per-user state is
git-ignored.

**Run `just venv` once**, then **install the recommended extensions when
prompted.** The one that matters is
`ms-vscode.cpptools` — without it there is no C++ IntelliSense at all, and this
is a C++ project. `clangd` is listed as *unwanted*: it and cpptools fight over
the same files, and the workspace is set up for cpptools.

### What is configured, and why

- **The Python interpreter is pinned through a `.venv`, and this took two
  attempts.** This box has *three* python3.14 interpreters and only one is
  usable: `/usr/bin/python3` has pytest, PlatformIO's venv cannot import `em`
  or `yaml`, and uv's `~/.local/bin/python3.14` has no pytest. Naming the right
  one in `python.defaultInterpreterPath` was **not enough** — that setting is
  only a *default*, and the Python Environments extension
  (`ms-python.vscode-python-envs`) overrides it. It picked uv's interpreter and
  the Testing sidebar failed with `No module named pytest`.

  The fix is `just venv`, which builds `.venv` from `/usr/bin/python3` with
  `--system-site-packages`. **It is not a dependency sandbox** — it installs
  nothing and sees the system pytest; it is a deterministic pointer at the right
  interpreter, and `.venv` is the first place that extension looks
  (`python-envs.workspaceSearchPaths` defaults to `['.venv', '*/.venv']`).
  `just test` uses the same interpreter, so a green sidebar and a green
  `just test` cannot disagree. On a fresh clone it falls back to
  `/usr/bin/python3` — the same interpreter, so nothing changes but the path in
  the log.

  Note `python-envs.alwaysUseUv` defaults to **true** and is *machine*-scoped,
  so a workspace setting cannot turn it off. That is why the fix works *with*
  the extension's discovery rather than against it.
- **C++ IntelliSense is driven entirely by `build/compile_commands.json`**, so
  it cannot drift from the build: every include path and define comes from how
  the file was actually compiled. `just build` regenerates it — the compiled
  packages set `CMAKE_EXPORT_COMPILE_COMMANDS`, and
  `tools/merge_compile_commands.py` merges colcon's per-package databases into
  the one file the editor reads. **If IntelliSense is confused, build first.**
- **`cppStandard` is `c++17`, not 20.** The Pi builds this same code under
  Jazzy; an editor set to 20 would accept code the Pi then refuses.
- **Tasks shell out to `just`** rather than reimplementing anything. The recipes
  already know how to source ROS, fix `PATH`, reach the Pi with the right ssh
  options, and tear themselves down. `Ctrl+Shift+B` builds; the test task runs
  both suites; the gates and the camera recipes are all there.
- **No format-on-save.** The ament linters are deliberately not enabled yet
  ([testing.md](testing.md)), so a formatter would churn style the project has
  not decided on.

### Debugging

`camera_node` runs **on the Pi** and cannot be launched from here — there is no
camera. To debug it in place, connect with Remote-SSH to `pi`, open
`~/ros2_pi`, and use the same configurations there.

What *is* usefully debuggable locally is the gtest binary, which is where the
timestamp arithmetic lives: `launch.json` has a configuration for the whole
suite and one that prompts for a `--gtest_filter`. The pytest suite for the gate
tools debugs through `debugpy`, pinned to the system interpreter.

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
  `No module named 'yaml'`; prefix `PATH=/usr/bin:$PATH`. The same shadowing
  breaks Ansible from the other end — interpreter auto-discovery finds a Python
  that cannot `import apt` — which is why `ansible.cfg` pins
  `interpreter_python = /usr/bin/python3`.
- **`rviz2` needs `QT_QPA_PLATFORM=xcb`** on this Wayland session. It was
  measured running on hardware GL (4.6) with driver 595.84, so the old
  software-GL workaround is obsolete.
