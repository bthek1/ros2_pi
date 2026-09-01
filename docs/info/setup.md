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
# dev box: everything
colcon build --symlink-install
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

```bash
just cam        # Pi-side camera only, with teardown
just pipeline   # the dev-box container against a running camera
just dev        # both, plus the dashboard — the day-to-day session
just dash       # dashboard alone, against a running pipeline
```

Every session recipe **tears itself down on both machines**: the viewer runs in
the foreground and a `trap … EXIT` `pkill -f`s each node pattern the recipe
started, so Ctrl-C and closing the window both work. When a session gains a
node, its pkill pattern goes into the trap in the same change.

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
- **`rviz2` needs `QT_QPA_PLATFORM=xcb`** on this Wayland session. It was
  measured running on hardware GL (4.6) with driver 595.84, so the old
  software-GL workaround is obsolete.
