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

**Done and gated as of 2026-09-15** — `bash tools/gates/gpu-stack.sh`. One
command installs it:

```bash
bash tools/fetch-gpu-stack.sh     # ~2.2 GB down, 2.0 GB installed, no sudo
```

It lands in `~/.local/opt/pimesh-gpu` (override with `PIMESH_GPU_PREFIX`), every
component pinned by version and verified by sha256. Removing it is `rm -rf` of
that one directory.

| | Version | Why this one |
| --- | --- | --- |
| ONNX Runtime | 1.30.0 `gpu_cuda13` | The first release with a CUDA 13 tarball |
| CUDA runtime | 13.1 redistributables | cudart, cuBLAS 13.2, cuRAND, nvrtc, nvJitLink |
| cuDNN | 9.26.0.51 `_cuda13` | Without it the CUDA provider does not load at all |

The driver is the one piece this does not install — 595.91.07, already present,
advertising CUDA 13.2, which is the ceiling the versions above were chosen
against.

**Why not `apt`.** Ubuntu 26.04's multiverse does carry `cuda-cudart-13-1` and
friends, and `sudo` on this box prompts for a password that no script here has.
NVIDIA publish the same libraries as public redistributable tarballs with a
sha256 manifest, so the whole stack installs rootless — and being pinned and
checksummed, it is reproducible in a way an `apt install` of a moving target is
not.

**The ONNX Runtime tarball vendors no CUDA whatsoever.** `objdump -p` on
`libonnxruntime_providers_cuda.so` lists `libcudart.so.13`, `libcublas.so.13`,
`libcublasLt.so.13` and `libcurand.so.10` as NEEDED, and before 2026-09-15 none
of those existed on this machine. Python's `onnxruntime-gpu` works only because
pip wheels vendor them; a C++ build gets none of that.

#### Four things that bite, all measured on 2026-09-15

**1. Link with `-Wl,--disable-new-dtags` or you silently get the CPU.**
`libonnxruntime_providers_cuda.so` is *dlopened* by `libonnxruntime.so` and
carries no `RPATH` or `RUNPATH` of its own. `DT_RUNPATH` — what CMake and every
modern linker emit by default — is **not inherited down a dlopen chain**, while
the older `DT_RPATH` is. So with default flags the provider cannot find
`libcublas.so.13` sitting in the same directory, and ONNX Runtime falls back to
the CPU. Same source, same libraries, one linker flag apart:

| | Provider | Mean |
| --- | --- | --- |
| `--enable-new-dtags` (CMake default) | `CPUExecutionProvider` | 236.62 ms |
| `--disable-new-dtags` | `CUDAExecutionProvider` | 51.20 ms |

`gates/gpu-stack.sh` runs both and asserts the default-flags build does *not*
reach CUDA, so the flag can never quietly stop being load-bearing.

**2. That flag rescues an executable and not a component, and the two look
identical from the outside.** `tools/gpu_probe` is a program this workspace links,
so it carries the `RPATH` above. A `rclcpp_components` component is loaded into
`component_container_isolated`, which was built by somebody else and carries none
— and **the main executable's `DT_RPATH` is the only one glibc has left to
consult** for a dlopened object's dependencies, because a dlopened object has no
loader chain of its own. Same libraries, same flags, one container apart:

| | Provider | Mean |
| --- | --- | --- |
| `tools/gpu_probe`, an executable | `CUDAExecutionProvider` | 51.08 ms |
| `depth_node`, a component | `CPUExecutionProvider` | 517 ms |

So `depth_node` loads the CUDA libraries itself, by absolute path, before ONNX
Runtime asks for them — `preload_cuda_provider()` in
`src/pimesh_depth/src/depth_engine_ort.cpp`. Their `DT_NEEDED` entries are
then satisfied from what is already in the process and no search happens.
`bash tools/gates/depth.sh` is what covers this, because `gates/gpu-stack.sh`
structurally cannot: its instrument is an executable.

**3. Everything goes in one lib directory, and that is structural.** Since the
provider has no search path of its own, the only two ways it can find cuBLAS are
`LD_LIBRARY_PATH` and sitting in the same directory as the thing that loaded it.
`LD_LIBRARY_PATH` is read by the loader at *process start* and cannot be fixed up
later with `setenv`, which rules it out for a component loaded into a container
somebody else launched. So the CUDA and cuDNN shared objects are installed
*beside* the ONNX Runtime ones and resolve through `libonnxruntime.so`'s
`RUNPATH $ORIGIN`.

**4. Never flatten `lib/` and `lib/stubs/` together.** Every CUDA redistributable
ships link-time stub libraries — `libcublas.so`, `libcublasLt.so`, and in cudart
a `libcuda.so` standing in for the driver. Copying both into one prefix let the
22 kB stub overwrite the 54 MB real cuBLAS. The result loaded, resolved every
symbol, printed `You are running using the stub version of cublas` on *stdout*
where nobody looks, and segfaulted on the first inference. Note what that means
for verification: `ldd` reporting no unresolved dependencies was true of the
broken install. **A library that resolves is not a library that works** — which
is why the fetch script also asserts a size floor on `libcublas.so.13`.

#### Checks, in order

```bash
nvidia-smi                                  # driver alive, GPU visible
nvcc --version                              # still absent, still expected
ls ~/.local/opt/pimesh-gpu/lib              # after the fetch
bash tools/gates/gpu-stack.sh               # the one that actually decides
```

#### Still not installed: the toolkit

Hand-written CUDA kernels — a future TSDF integrator — need a real
`cuda-toolkit` install and `find_package(CUDAToolkit)`. Runtime libraries are not
a compiler. **Do not plan on this until the CPU integrator works and has been
profiled.**

### Model weights

Depth Anything V2 Small, ONNX, ~99 MB. **Fetched and checksummed, never
committed** — `models/` and `*.onnx` are git-ignored.

```
https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx
sha256  afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c
```

```bash
bash tools/fetch-model.sh          # fetch if missing, verify either way
```

It is a script and not a `just` recipe on purpose — the justfile's bar is "would
a newcomer's first `just` need to see this?", and fetching weights is setup, not
a normal day. It **verifies on every run**, not only after a download: a
truncated file has the right name and a plausible size, and ONNX Runtime opens it
and fails with a protobuf parse error that names no cause.

**The model does not go on the Pi** — inference is dev-box-only, and
`tools/pi/sync-pi.sh` ships `src`, `tools` and the justfile and nothing else, so
`models/` is excluded by construction rather than by a rule somebody has to
remember.

### A dataset with ground truth

**TUM RGB-D fr1/desk**, 344 MB compressed. Fetched, checksummed and unpacked into
`~/.local/share/pimesh-datasets` — outside the workspace, because it is not this
project's data and a second checkout should find it already there.

```bash
bash tools/fetch-dataset.sh                  # fetch if missing, verify either way
bash tools/fetch-dataset.sh --print-path     # where the sequence is
```

Three hashes are pinned rather than one: the tarball's says the download is what
TUM published, `rgb.txt`'s says the frame list has not moved, and
**`groundtruth.txt`'s says the truth has not** — which is the one that would
otherwise be unfalsifiable, since a corrupted truth file does not produce an
error, it produces an ATE. A tampered tree is detected, named, and re-extracted
from the tarball.

`tools/gates/trajectory.sh` also needs **`evo`**, which computes the ATE. It is
somebody else's tool on purpose: writing our own would be writing the instrument
and the thing it measures in the same afternoon.

```bash
uv tool install evo          # the way shellcheck-py is installed — no sudo
```

Neither goes on the Pi.

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
bash tools/pi/build-pi.sh       # rsync source to the Pi, then build there under Jazzy
bash tools/test.sh           # build, then colcon test, then colcon test-result
bash tools/pi/test-pi.sh        # the same, on the Pi, under Jazzy
bash tools/clean.sh          # drop build/ install/ log/ here
bash tools/pi/clean-pi.sh       # and there
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

`tools/pi/sync-pi.sh` ships `src/`, `tools/` and the `justfile` with `rsync --delete`,
and **never** `build/`, `install/` or `log/`. It then runs
`tools/pi/check-stale.sh` on the Pi and clears the Pi's build tree if the sync left
artefacts whose sources are gone — colcon never forgets a package on its own, so
a stale overlay will otherwise keep answering `ros2 pkg list` with packages that
no longer exist.

Keep everything the Pi builds at **C++17** and to APIs that exist in both
distros. A build that only succeeds here is half a build — and the failure is
not always in the direction you expect: `ament_target_dependencies()` is gone in
Lyrical and still present in Jazzy, so that one breaks *here* on CMake the Pi
would accept.

First-build times, measured 2026-09-08 on a single small two-component package:
**~9 s** on the dev box, **20.5 s** on the Pi. The whole six-package workspace is
several minutes at either end — `bash tools/gates/build.sh` prints both.

## Running

`just` with no arguments lists what exists, by group:

<!-- gate-justfile keeps the block below equal to `just --list`. Edit the
     justfile, then re-run `bash tools/gates/justfile.sh`; do not edit by hand. -->

```text
Available recipes:
    default                                        # List the recipes

    [build]
    build *args                                    # Build the workspace

    [run]
    dashboard seconds="600" bag="" port="8080"     # The whole pipeline in a browser tab: http://localhost:8080. Not evidence
    replay bag seconds="600"                       # A recorded bag in RViz, looping. bag = a name under bags/, or a path to one
    view-camera seconds="600"                      # The Pi's camera and the frame tree, in RViz. A viewer, not evidence
    view-depth seconds="600" bag=""                # The room as a depth cloud, in RViz. bag = optional, else the camera
    view-keypoints seconds="600" bag=""            # ORB corners and the pose, in RViz. bag = optional, else the camera
    view-mesh seconds="600" bag=""                 # The room as a triangle surface, in RViz. bag = optional, else the camera
    view-odom seconds="600" bag="" regime="sixdof" # The camera's trajectory, in RViz. regime = sixdof (default) or rotation_only
```

That is the whole list, and the shortness is the point: `build` is how you
build, `run` is what you start in order to *watch* something. If you have just
cloned this, `just build && just view-camera` is the entire getting-started
path — or `just dashboard` once the GPU stack is installed.

**Everything else is a script in `tools/`, run directly.** The gates especially
— there are eighteen, they are run constantly, and as recipes they buried the
handful of commands a person actually types under an alphabetised wall of
`gate-*`:

```bash
# The pipeline, phase by phase
bash tools/gates/build.sh         # P0: one source tree, two distros, same messages
bash tools/gates/capture.sh       # P1: 720p MJPEG on the LAN, stamped honestly
bash tools/gates/ipc.sh           # P2: one reader, one decode, handed on as a pointer
bash tools/gates/keypoints.sh     # P3: ORB against the reference implementation
bash tools/gates/depth.sh         # P4: the depth network, on the GPU, in the container
bash tools/gates/fusion.sh        # P5: the TSDF integrates every frame it is offered
bash tools/gates/mesh.sh          # P6: marching cubes, off the integration path
bash tools/gates/odom.sh          # P7: 6-DoF pose against a rotation-only control
bash tools/gates/dashboard.sh     # P8: five channels, and 0 cost to the pipeline
bash tools/gates/trajectory.sh    # P11: the ATE against TUM fr1/desk's motion capture
bash tools/gates/scale.sh         # P12: /depth against a tape measure (needs bags/scale1)
bash tools/gates/calibration.sh   # P9: the C922's real intrinsics, and that they straighten it
bash tools/gates/gpu-stack.sh     # the GPU toolchain, before any node uses it

# Across all phases
bash tools/gates/test.sh          # the unit tests pass on both machines, and there are some
bash tools/gates/view-configs.sh  # every .rviz topic is one src/ publishes
bash tools/gates/justfile.sh      # the justfile's own shape, plus shellcheck over tools/
bash tools/gates/teardown.sh      # Ctrl-C, a closed terminal and a closed window
                                  #   each leave nothing running, either machine,
                                  #   for every recipe in the justfile's `run` group

bash tools/calib/calibrate.sh record    # a bag of the board from many angles
bash tools/calib/calibrate.sh select    #   ...best frames out of it, marker-confirmed
bash tools/calib/calibrate.sh solve     #   ...fit, and write the YAML + report.txt
bash tools/calib/camera-reset.sh        # clear the camera's persistent V4L2 controls
bash tools/stragglers.sh          # assert nothing outlived its session
bash tools/fetch-dataset.sh       # TUM fr1/desk, for gates/trajectory.sh
bash tools/pi/sync-pi.sh             # ship source to the Pi — source only
bash tools/pi/build-pi.sh            # ...and build it there, under Jazzy
bash tools/clean.sh               # delete the colcon trees (clean-pi.sh for the Pi's)
```

## In the editor

`.vscode/` carries three committed files — `tasks.json`, `settings.json`,
`extensions.json` — and nothing in them reimplements a command. Every task is
one line running the same `tools/` script a person runs in a terminal, for the
reason the justfile gives for its own recipes: a second spelling of a command is
a second thing to keep true.

- **Ctrl+Shift+B** builds (`tools/build.sh`), **Run Test Task** runs the unit
  suites (`tools/test.sh`), and a `gate: pick one` task runs any of the
  fifteen gates. Every task runs under `bash -lc`, because a task shell that
  read no profile is on domain 0 with the wrong RMW.
- **The Testing panel** shows all twenty-eight suites once the two recommended
  extensions are installed — VSCode offers them on first open. The twenty-three
  gtest ones come from `build/*/test_<name>`, and a run **builds first**: a stale
  binary against edited source is a green test for code that does not exist.

**Both halves need `.vscode/ros.env`, and it is generated.** The Python
extension does not run tests through a shell — it spawns the interpreter
directly, so `source install/setup.bash` never happens, and
`test_transforms.py`'s ament-index lookups would resolve against whatever the
desktop session had on `AMENT_PREFIX_PATH`, which on this box is *another
workspace*. `python.envFile` and TestMate's `envFile` are the one hook either
extension offers, and `bash tools/lib/vscode-env.sh` writes it: the essential ROS
variables unconditionally, plus whatever else the sourcing changed, so a future
setup script learning a new variable does not need this script edited.

It is a generated file — git-ignored, absolute paths, this machine's distro — so
a task regenerates it on folder open (VSCode asks once whether to allow
automatic tasks; say yes). **Run it by hand after adding a package**, or after a
fresh clone. The failure is loud rather than quiet: an `AMENT_PREFIX_PATH`
missing a package fails the launch-file tests with a `PackageNotFoundError`
naming every path it searched.

Measured 2026-09-19, with only that file's variables in an otherwise empty
environment: **83 Python tests passed and all 16 gtest suites passed**. The
editor is a convenience, though, not evidence — `bash tools/test.sh` and
`bash tools/gates/test.sh` are what decide, and they are what CI would run.

## Calibrating the camera

Done once, on 2026-09-12, and the result is committed — `camera_node` loads
`pimesh_bringup/config/camera_info/c922_720p.yaml` by default and there is nothing to
run on a fresh checkout. Redo it only if the camera or the printed board changes:

```bash
bash tools/calib/calibrate.sh record --square 0.02475 --squares 7x9 --marker 0.01782 --seconds 120
bash tools/calib/calibrate.sh select --bag bags/calib_<timestamp> --square 0.02475 --squares 7x9 --marker 0.01782
bash tools/calib/calibrate.sh solve  --square 0.02475 --squares 7x9 --marker 0.01782
bash tools/build.sh --packages-select pimesh_bringup && bash tools/pi/sync-pi.sh && bash tools/pi/build-pi.sh
bash tools/gates/calibration.sh
```

The numbers are the **measured** size of `docs/charuco_a4_7x9_25mm.pdf` as printed —
its 100 mm bar measures 99 mm, so the nominal 25/18 mm are really 24.75/17.82. Check
that bar with a ruler if the sheet is ever reprinted: nothing in software can detect a
scale error, and it goes straight into every distance the pipeline reports.

`--squares` is the count of **squares**, not interior corners. The command printed
along the bottom of the sheet itself passes the corner count and interpolates nothing;
see [troubleshooting.md](troubleshooting.md). And `session`/`install` exist but are
superseded — `cameracalibrator` does not run on Lyrical at all.

Two results from that phase that a newcomer should not have to rediscover: this camera
has **essentially no lens distortion at 720p**, and its `fx` is pinned only to
**±2.2%**. Both are in [hardware.md](hardware.md#calibration-target).

**A gate and a unit test are different things.** The gates above are the phase
tests — slow, mostly needing the Pi and the camera, and what actually closes a
claim. `bash tools/test.sh` runs `colcon test`: 82 hermetic tests over six
suites that need no hardware and run identically under both distros. Note that
`colcon test` exits 0 when a test *fails*, and again when a package has no tests
at all, so `colcon test-result --all` is what decides — which is why
`tools/gates/test.sh` asserts on the counts rather than on the exit status.

Each gate exits non-zero and prints the number it asserted on.
[gh issue #2](https://github.com/bthek1/ros2_pi/issues/2) records what the
scaffolding's own `hello-*` gates measured — they and the `pimesh_hello` package
were deleted on 2026-09-23, once `gates/build.sh`, `gates/ipc.sh` and
`gates/capture.sh` covered the same claims on the real pipeline — and
`tools/gates/justfile.sh` is [#3](https://github.com/bthek1/ros2_pi/issues/3)'s.

**The justfile is the user-facing surface; the shell is in `tools/`.** Every
recipe is one line that runs a script — `tools/build.sh` — because `just` gives a recipe body no way to share code with another recipe, so
inlined bash gets copy-pasted and drifts. `tools/lib/just-lib.sh` is what they all
source: the prelude, the single spelling of the Pi's `ssh` invocation, the
bracketed `pkill` patterns and `in_range`. Two consequences worth knowing:
the scripts run without `just` (which is why `gate-teardown` can signal one
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
the foreground and `arm_cleanup` (in `tools/lib/just-lib.sh`) installs a handler on
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

## The visit to the room

**Two things in this project cannot be closed by a script**, and they are one
trip: `depth_scale` has been 10.0 since P4 because somebody typed it, and
`bags/desk1` is a pan from one spot so 6-DoF translation has never been shown to
beat rotation-only. Both are milestone F —
[#10](https://github.com/bthek1/ros2_pi/issues/10) P12 and P13 — and everything
either of them needs from software is already written. What is left is a tape
measure, a wall, and a walk.

Do them in one visit. Each clip turns its phase into a replayable test forever;
a second trip does not.

### Before you start

```bash
bash tools/stragglers.sh            # nothing of this project's is already running
bash tools/calib/camera-reset.sh    # V4L2 controls persist inside the camera,
                                    #   across processes and reboots
```

A manual exposure left by an earlier benchmark makes every later session black,
and a clip recorded at 20 fps under a stale one cannot be un-recorded.
`record-clip.sh` resets the controls itself, so this is belt and braces — but the
reset also prints the whole control table, which is worth a look before you spend
half an hour recording.

### P12 — the tape measure, and `bags/scale1`

1. Find a **flat surface with nothing in front of it** — a wall, a door, a closed
   cupboard — and measure from the camera's front element to it. **1.5–2.5 m.**
   Not closer: `min_range_m` is 0.15 m and the near field is where a monocular
   network is least like a metric sensor. Not further: at the *unpinned* scale of
   10.0 the map reads roughly twice what it should against a 6 m clip, so a
   surface at 3.5 m true reads past it — and a clipped pixel is not a distance,
   it is the absence of one written as 6 m.
2. Point the camera **square-on**, filling the middle of the frame. A depth map
   carries *z*, not distance along the ray, so an oblique patch reads a range of
   distances rather than one.
3. **Look before you record**: `just view-depth`. Near is bright and the clip is
   black in that preview, so a colour is a distance — **the middle of the frame
   must be bright.** Black means the gate will refuse the clip, and you will find
   that out after walking back from the wall.
4. `bash tools/record-clip.sh scale1 20`.
5. `bash tools/gates/scale.sh <the metres you measured>`. It **refuses** — nothing
   is recorded in git yet — and prints the two lines to paste into
   `src/pimesh_bringup/config/pimesh.yaml`. If the clip failed its checks it
   prints no number at all and says why, which is deliberate: an implied scale
   computed over a patch that is mostly clip is wrong in the direction that looks
   plausible.
6. Paste, `bash tools/build.sh`, then `bash tools/gates/scale.sh` with no
   argument. That is P12's test.

**What to expect.** The predecessor's room came out at **2.69**. P11's Sim(3) fit
against TUM fr1/desk says **4.6–5.2** for that sequence, which bounds the number
without transferring to this camera in this room — so a result in that region is
unsurprising and a result at 9, or at 2, is worth stopping over and writing into
the issue. If the tape and the calibration disagree by more than ~2%, the trigger
in [milestone-f-future.md](../plans/future/milestone-f-future.md) has fired: `fx`
is pinned only to ±2.2% and the fix is a flat mount and a re-run of
`record | select | solve`.

### P13 — `bags/walk1`

`bash tools/record-clip.sh walk1 60` — a **slow walk**, camera held level, moving
**metres** rather than centimetres, **returning to where you started**.

The return is not decoration. Milestone H cannot detect a loop on a clip that
never revisits anywhere, and `bags/desk1` is a ~0.9 m arm arc against 2–3 m of
scene, which is why rotation already explains most of its frame motion.

Then `bash tools/gates/odom.sh walk1`. It prints both regimes' median
paired-surface gap, agreement, path length and net displacement. On `desk1` the
two are a dead heat — 0.4456 m against 0.4471 m — which is why that comparison is
printed rather than asserted. **If 6-DoF wins on a clip that carries real
translation, the comparison gets promoted to an assertion keyed to the clip, with
the margin taken from three runs, not one.** If it still does not win, that is the
more interesting result and it points at the depth network rather than at the
pose — annotate the phase with both numbers and open a deferred entry naming the
network. It does not become an assertion on a number that did not move.

### Afterwards

```bash
bash tools/stragglers.sh
```

`bags/` is git-ignored, so the two clips exist only on the machine that recorded
them. Their identity is the sha256 the way `bags/desk1`'s is — record it in the
issue alongside the numbers.

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
