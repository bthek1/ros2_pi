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

As of **2026-09-09** there is exactly one package, `src/pimesh_hello/`, and it
exists to prove the structure rather than to do anything: a C++ `ament_cmake`
package, two `rclcpp_components` components composed into one container with
intra-process comms measured handing over the pointer, parameters from a keyed
YAML, the same source built from scratch under **both** distros, and a session
that tears itself down on either machine. Five scripts in `tools/gates/`
assert all of it — [gh issue #2](https://github.com/bthek1/ros2_pi/issues/2)
carries the numbers each one printed.

**The teardown claim had to be earned twice, and the way it failed is worth more
than the fix.** It was true for `hello-lan` and false for `hello-compose` until
2026-09-09: a foreground `timeout` had put `ros2 launch` in a process group the
terminal's Ctrl-C never reached, so the recipe swallowed six of them and ended
on its own when the timer expired. The gate said PASS throughout, because it
only ever signalled the *other* recipe. That is a green gate over broken
behaviour — worse than no gate, because it is a false claim with a script's
authority behind it. Both halves are fixed (`run_for`, and a gate that signals
both recipes), and the lesson is the one to carry into every later phase: ask
what the gate does **not** touch.

### The pipeline has started: capture is real

**As of 2026-09-09 the first two phases are built and measured** — milestone A,
[gh issue #4](https://github.com/bthek1/ros2_pi/issues/4). `pimesh_msgs`,
`pimesh_bringup` and `pimesh_camera` join `pimesh_hello`, all four building from
source under both distros, and the Pi puts stamped 720p MJPEG on the LAN:
**44–59 Hz received on the dev box** (`bash tools/gates/capture.sh`), 0 duplicate
payloads, **4.21 ms** median dequeue-to-subscriber measured on the Pi's own
clock, and two launches agreeing on their stamp offset to **0.30–1.02 ms** —
which is the assertion that `usb_cam` 0.8.1 fails by hundreds of milliseconds.

**Everything downstream of capture still does not exist.** No decode, no
keypoints, no depth, no fusion, no mesh, no dashboard — that is
[docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md)
and milestone issues [#5](https://github.com/bthek1/ros2_pi/issues/5)–[#8](https://github.com/bthek1/ros2_pi/issues/8),
and everything the rest of `docs/` says about those stages is **design intent**,
not a description of running code. When you build something, change the doc that
describes it from future tense to a measured statement, and say what you
measured it with.

**Two things P0–P1 cost, and both are the same lesson as the teardown one
above.** A `static_transform_publisher` given `parameters=[...]` dies before it
reads them — it parses `argv` first — so the launch came up with no TF tree and
nothing failing. And extending `gates/hello-clean.sh` to signal `view-camera`
immediately found that recipe leaking RViz *and* the Pi's camera, because bash
will not run a trap while a foreground child is running and an rviz2 signalled
during its own startup never exits. The same gate also turned out to be deducing
the process group from `$!`, which is empty whenever `setsid` forks — a kill
that had been silently doing nothing in some contexts. Ask what the gate does
**not** touch.

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
`piros2` on 2026-08-31 (topics, `camera_info`, `tf_static` all crossed), and
re-measured as **this** project's own on 2026-09-08: a Jazzy publisher on the Pi
delivered 39 of 40 messages in 20 s at 2 Hz to a Lyrical subscriber here
(`bash tools/gates/hello-lan.sh`). DDS is wire-compatible across distros; the C++ ABI is
not, and that one sentence is the whole reason for the build-from-source rule.

**Consequence for C++, and it is the sharpest one in the project:** ROS 2 has no
ABI compatibility guarantee across distros. A `.so` built here does not run
there. **Every package must build from source on both machines** — no
cross-compiled binaries, no shipped `install/` tree, and nothing in
`pimesh_camera` may depend on a Lyrical-only API. C++17 (Jazzy's baseline), not
C++20, in anything the Pi builds.

The drift runs in **both** directions, and the dangerous one is the direction
that fails *here*: `ament_target_dependencies()` was deprecated in Jazzy and is
**removed in Lyrical**, so the dev box stops with `Unknown CMake command` on
CMake that the Pi would have built without complaint (measured 2026-09-08).
Where a build-system API differs, prefer the spelling that exists on **both** —
plain `target_link_libraries()` against the exported targets — and confirm it on
both before relying on it, with something like

```bash
grep -rh "add_library(rclcpp::" /opt/ros/lyrical/share/rclcpp/cmake/*.cmake
ssh pi 'bash -lc "grep -rh \"add_library(rclcpp::\" /opt/ros/jazzy/share/rclcpp/cmake/*.cmake"'
```

The reverse case is worse because it is silent: a Lyrical-only API compiles here
and is only discovered at the far end of an `rsync`. `bash tools/build-pi.sh` is cheap —
run it before believing a CMake change.

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
  downstream component gets a pointer to the same buffer. That is the
  main structural reason this rewrite exists — do not break it by launching
  components as separate processes "for debugging".

  **This one is no longer inherited: it is measured here.** `bash tools/gates/hello-ipc.sh`
  runs the same container twice, with intra-process on and off, and compares the
  payload address the publisher logged against the one the subscriber received —
  19/19 equal with it on, 0/16 with it off (2026-09-08). It takes both halves to
  qualify: the publisher must move a `unique_ptr` into `publish()`, and the
  subscription callback must take a `unique_ptr`. A `const &` callback works
  perfectly and quietly copies. **Address equality alone is not evidence** —
  two allocations in one process can coincide, and one did, at 1/22 — so any
  future zero-copy claim needs the with/without control, not a single run.
- **BEST_EFFORT delivers zero large frames** *(inherited, and it is about
  size)*. Megabyte-class messages fragment past the socket buffer and never
  reassemble. Every image and depth **publisher** here is `RELIABLE` +
  `KEEP_LAST(1)` — freshest frame, no backlog.

  **A viewer subscribing to the ~80 kB compressed stream is the exception, and
  it is measured.** A RELIABLE reader delivers in sequence, so one lost fragment
  head-of-line blocks every frame behind it for a heartbeat round trip; over the
  Pi's Wi-Fi that is a visible freeze several times a minute. Changing only the
  reader's QoS, 20 s windows of ~1100 frames (2026-09-09): RELIABLE gave 10 gaps
  over 50 ms with a worst of 490 ms; BEST_EFFORT gave 3, worst 181 ms, with 0
  undecodable frames. `rviz/camera.rviz` therefore asks for BEST_EFFORT, which a
  RELIABLE writer satisfies (only the reverse is incompatible). At 80 kB a frame
  is ~56 fragments; at 2.7 MB raw it is ~1900, which is why this scopes the rule
  rather than contradicting it. **Do not carry it to a raw image topic**, and
  treat the same head-of-line question as open for `decode_node` in P2.
- **Never gate on `header.stamp` age** *(inherited, and now only half true)*.
  `usb_cam` 0.8.1 has a once-per-process epoch bug that puts stamps a random
  sub-second amount in the past, redrawn at every launch. A stamp-age freshness
  gate silently dropped 100% of frames.

  **`pimesh_camera` fixes this for our own capture path, measured 2026-09-09.**
  It stamps `ros_now - (monotonic_now - v4l2_buffer.timestamp)` — an *interval*,
  not an epoch — so there is no per-process constant to be wrong, and two
  launches agree to within 1 ms. Frames on `/image_raw/compressed` therefore
  carry honest capture times and may be reasoned about.

  **But not across the two machines.** The stamp is set on the Pi's system clock
  and read on the dev box's, and the gap between them is NTP's business: it
  measured +8 ms and −19 ms an hour apart on 2026-09-09 with nothing changed.
  So a stamp-age gate on the dev box is *still* forbidden — it would be
  measuring NTP. Compare stamps to stamps (deltas are kernel capture intervals
  and are trustworthy), and measure latency where one clock covers both ends.
- **V4L2 controls persist inside the camera** *(inherited)* across processes and
  reboots. A manual exposure left by a benchmark makes every later session
  black; the C922 powers on with `exposure_dynamic_framerate=1`, which costs
  ~10 fps in indoor light. Treat camera state as inspectable machine state and
  reset it before diagnosing black frames or low fps as a software bug.
- **`/dev/video1` is not a capture device** — it is the C922's UVC metadata node.
  Capture is `/dev/video0`. `V4l2Capture` checks `device_caps` rather than
  `capabilities` for exactly this: the latter is the union over every node the
  driver owns, so the metadata node reports its sibling's capture bit and passes
  a naive check, failing later and worse.
- **Reset the camera before measuring anything about it.**
  `bash tools/camera-reset.sh` puts every control back to its default, forces
  `exposure_dynamic_framerate=0` (whose reported default of 0 is a lie about
  what the camera powers on with), prints the whole control table
  current-vs-default, and exits non-zero if the one control that matters did not
  stick. `gates/capture.sh` runs it first; a rate measured without it is a
  measurement of whatever the last person left behind.
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
  `pimesh_<thing>` — `pimesh_hello`, `pimesh_msgs`, `pimesh_bringup` and
  `pimesh_camera` exist; planned: `pimesh_perception`, `pimesh_world`,
  `pimesh_dashboard`. (Not `ros2_pi_*`: a `ros2_` prefix reads as core tooling.)
  Shared shell helpers live in `tools/` and are rsynced to the Pi, so they must
  work on both distros — `tools/ros-env.sh` discovers the distro rather than
  naming it, and `tools/check-stale.sh` is run by both `gate-hello-build` here
  and `sync-pi` there.
- **C++ only for nodes.** `ament_cmake`, C++17, no Python nodes. Launch files
  and one-off tools may be Python — that is not a licence to move logic there.
- **Nodes are `rclcpp_components` components**, registered with
  `RCLCPP_COMPONENTS_REGISTER_NODE`, each with a thin `*_main.cpp` so it can also
  run standalone. The bringup launch composes the dev-box ones into a single
  container with `use_intra_process_comms=True`. A node that only works
  standalone is a bug. **`src/pimesh_hello/` is the worked example** and
  `src/pimesh_camera/` is the same shape doing real work — copy either rather
  than rediscovering it: the class declared in
  `include/pimesh_hello/`, defined in `src/`, the register macro at the foot of
  the .cpp, `rclcpp_components_register_nodes` (plural — the singular form
  generates its own `main` and makes the thin one dead code), and the library
  installed to `lib/` while the executable goes to `lib/${PROJECT_NAME}/`.
- **No work in a subscription callback beyond a bounded copy.** Anything that
  costs milliseconds (inference, fusion, meshing) runs on its own thread with a
  single-slot mailbox: newest frame wins, older one dropped. Queues that grow
  are how this pipeline dies.
- **Parameters live in `config/*.yaml`, keyed by node name**, and launch files in
  `launch/*.launch.py`. A key that does not match the node name silently applies
  nothing — a trap that has cost this project's predecessor real time. Declare
  every parameter with a description and validate ranges at declaration.
- **Two kinds of test, and conflating them is a mistake.** The
  `tools/gates/*.sh` scripts are the **phase tests**: slow, often needing the Pi
  and the camera, and they are what closes a claim. `bash tools/test.sh` runs
  the **unit tests** (`colcon test`) — fast, hermetic, no hardware, and they run
  on both machines. Write a unit test for logic that can be got wrong silently
  (the stamp arithmetic, a matrix layout, a quaternion); write a gate for
  anything that is a number about a running system.

  **`colcon test` exits 0 when a test fails**, because it is reporting that the
  run completed — and it exits 0 again when a package has no tests at all, which
  is what an unbuilt tree looks like. `colcon test-result --all` is the thing
  that decides, and `bash tools/gates/test.sh` asserts on the counts: zero
  failures, zero skips, a floor on how many tests ran, and the same suites at
  both ends. Raise the floor when you add tests; never lower it to make a run
  pass.

  Tests that need a camera do not belong in `colcon test` — the dev box has no
  capture device, and a suite that only runs on the Pi is one that stops being
  run. `src/pimesh_camera/test/` covers the refusal paths with `/dev/null` and a
  temp file; the busy-device case is `tools/gates/capture.sh`'s job.
- **Build with `just build`**, not bare `colcon`. The recipe is
  `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`,
  and without that argument every `ament_cmake` package fails at configure time
  on this box (see the Python bullet above). `bash tools/build-pi.sh` does the same over
  SSH after `bash tools/sync-pi.sh` ships source — source only, never a built tree.
  Keep the commands and the docs in agreement — `docs/info/setup.md` quotes
  `just --list` verbatim and a script asserts it has not drifted.
- **The justfile is the user-facing surface — everything else is a script.**
  It holds `build`, `hello-compose` and `hello-lan`, and that is the whole of
  it: what someone types on a normal day. Gates, the Pi plumbing, the straggler
  sweep and the tree deletions are `bash tools/<name>.sh` and
  `bash tools/gates/<name>.sh`, run directly. **Resist adding a recipe.** The
  bar is not "is this useful" — every one of those scripts is useful — it is
  "would a newcomer's first `just` need to see this?". Seven gate recipes had
  buried `hello-compose`, which is the one command that shows the workspace
  doing something, and the file is trimmed to `build` and `run` precisely so
  that cannot recur. Adding a group is the thing to argue about, not adding a
  line.
- **The shell lives in `tools/`.** Every recipe is one line that runs a script,
  and every recipe carries a `[group('build'|'run')]`. The shared prelude, the
  one spelling of the Pi's
  `ssh` invocation, the bracketed kill patterns and `in_range` are in
  `tools/just-lib.sh`, which every script sources; `tools/` is rsynced, so the
  same functions work at both ends. The reason is not tidiness: `just` gives a
  recipe body no way to share code with another, so inlined bash is copy-pasted
  and drifts, and `shellcheck` cannot parse `{{ }}`, so inlined bash is never
  linted. **`bash tools/gates/justfile.sh` is the check** — groups exactly
  `build run`, 0 ungrouped recipes, justfile under 80 lines, no recipe body over
  10 lines, no `ssh`/`pkill`/prelude inlined in a body, `setup.md`'s quoted
  recipe list equal to `just --list`, and 0 shellcheck findings over `tools/`
  (`uv tool install shellcheck-py`).
- **Sessions tear themselves down — no stragglers.** Ctrl-C and closing the
  window must both end everything the recipe started, **on both machines**. The
  mechanism: viewer in the foreground, `arm_cleanup` in `tools/just-lib.sh`
  installing a handler on EXIT and on INT/TERM/HUP that `pkill -f`s every node
  pattern, locally and over SSH. (EXIT alone does fire on Ctrl-C; naming the
  signals is what makes the closed-window case deliberate rather than lucky.)
  The signal handler cleans up **once** and then re-raises after `trap -`,
  because a bare `trap handler INT` does not end a script — bash runs the
  handler and resumes at the next line, so an interrupted gate carries on
  measuring what it just killed, and the caller sees exit 0 where it should see
  130.
  **A trap is not always installed just because you wrote one**: a command
  started in the background by a non-interactive shell inherits SIGINT as
  SIG_IGN, and bash refuses to trap a signal that was ignored on entry — so any
  test that sends a fake Ctrl-C must reset the disposition first
  (`setsid env --default-signal=INT,TERM,HUP …`, measured 2026-09-09), or it is
  testing a session that cannot receive the signal.
  **And a trap that *is* installed still will not run while a foreground child
  is.** GNU `timeout` puts its child in a new process group so it can kill the
  tree on expiry; a terminal signals only the *foreground* group, so the command
  under `timeout` never sees Ctrl-C — and bash will not run the trap until that
  foreground child returns, which is exactly what it is refusing to do. Use
  **`run_for`** (`timeout --foreground -s INT`, in `tools/just-lib.sh`) for
  anything run in the foreground; a backgrounded `timeout … &` plus `wait` is
  equally sound, because then the trap fires on arrival. Measured 2026-09-09:
  0.30 s from a real Ctrl-C to a container logging *finished cleanly*, against a
  30 s timer that used to have to expire first.
  Killing a background `bash -lc` wrapper
  orphans its grandchildren — always pattern-match the node, never `kill %N`.
  **`bash tools/stragglers.sh` is the check**: it greps both machines and exits non-zero
  with the pid and full path of anything that survived. Every `pkill -f` and
  `pgrep -f` pattern is bracketed and path-anchored (`/lib/[p]imesh_hello/`) —
  see the troubleshooting entry on why the plain spelling kills the shell that
  runs it.
- **This applies to ad-hoc runs too — that means you, Claude.** Anything you
  start by hand while verifying has no EXIT trap. Bound it up front —
  **`timeout --foreground -s INT 30 …`**, and on the Pi
  `ssh pi 'timeout --foreground -s INT 30 bash -lc "…"'` — or `pkill -f` it when
  done, and check both hosts are clean before reporting. The `--foreground` is
  not decoration: without it the command is in a process group your own Ctrl-C
  cannot reach, so a run you meant to bound becomes one you cannot interrupt.
  **A leaked camera process holds `/dev/video0` exclusively** and every later
  session dies with `Device or resource busy`.
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
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | **Where this is going.** The whole pipeline as phases P0–P8, none started, each ending in a `tools/gates/*.sh` test, followed by the deferred register |
| [#4](https://github.com/bthek1/ros2_pi/issues/4) **(closed 2026-09-09)** [#5](https://github.com/bthek1/ros2_pi/issues/5) [#6](https://github.com/bthek1/ros2_pi/issues/6) [#7](https://github.com/bthek1/ros2_pi/issues/7) [#8](https://github.com/bthek1/ros2_pi/issues/8) — milestones A–E | **The pipeline, being built.** A is done — P0 and P1, the cross-distro workspace and capture. Five issues over the *one* phase list in `project_final_state.md`, a contiguous slice each: A = P0–P1, B = P2–P3, C = P4, D = P5–P6, E = P7–P8. No issue renumbers from zero. Each also has a `just view-*` RViz recipe — a viewer for a person, never a gate |
| `bash tools/test.sh` / `bash tools/gates/test.sh` | **The unit tests.** 29 of them across four suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, and the static transforms and launch conversion in `test_transforms` |
| `gh issue list --label plan --state all` | **The plans themselves.** [#2 hello-world](https://github.com/bthek1/ros2_pi/issues/2) — closed 2026-09-08, the build log for the scaffolding that exists; [#3 justfile](https://github.com/bthek1/ros2_pi/issues/3) — closed 2026-09-09, why the shell lives in `tools/`; the justfile was trimmed further the same day to `build` + `run` only, so that issue's `just gate-*` spelling is history, not instruction |
| [docs/plans/future/milestone-a-future.md](docs/plans/future/milestone-a-future.md) | Work deferred out of milestone A, each entry with its trigger: the checkerboard calibration (waiting on P5's tape-measure visit), `PipelineStats` from `camera_node` (waiting on the dashboard), the dev-box rate margin, and device reconnection |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note
the date.
