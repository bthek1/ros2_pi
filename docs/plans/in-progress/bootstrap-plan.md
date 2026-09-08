# Bootstrap plan — one webcam to a live mesh, in C++

**Started 2026-09-01. Last updated 2026-09-08.** The build order for the whole
pipeline.

Written to the rules in [../README.md](../README.md): **stable phase numbers**,
**every phase ends in a test that is a command**, and **every phase is
executable** — startable the moment the plan reaches it, with nothing to wait
for. Work that is not executable yet lives in
[../future/bootstrap-future.md](../future/bootstrap-future.md), never here.

Each phase's test recipe is written **in the same change as its code**.
`just gate-build`, `just gate-capture` and `just gate-provision` exist; every
other recipe named below is written by the phase that needs it. Alongside them,
`just test` and `just test-pi` run the unit tests — a separate layer, built at
P10 and described in [../../info/testing.md](../../info/testing.md).

The predecessor [`~/Documents/piros2`](../../../../piros2) already does all of
this in Python. **It is the reference and the yardstick** — where a number exists
over there, the C++ version is measured against it, so "the rewrite is faster"
stays a measurement rather than an article of faith.

**Status legend:** ☐ not started · ▶ in progress · ✓ done (annotated with the
date and what the test printed).

## Progress

| Phase | Delivers | Test | Status |
| --- | --- | --- | --- |
| **P0** | `pimesh_msgs`, `pimesh_bringup`, the justfile | `just gate-build` | **✓ 2026-09-01** — PASS ×3, both machines clean (`b96f63c`) |
| **P1** | `camera_node` on the Pi | `just gate-capture` | **✓ 2026-09-02** — PASS ×2, 0.00 ms stamp drift |
| **P2** | `decode_node` + the container | `just gate-ipc` | **✓ 2026-09-04** — PASS ×2, 10/10 frames at the same address |
| **P3** | `keypoint_node`, `bags/desk1` | `just gate-keypoints` | **✓ 2026-09-07** — PASS ×2, 6.99 ms/frame, 94.1% matched |
| **P4** | `depth_node` on the GPU | `just gate-depth` | **✓ 2026-09-08** — PASS ×2, 55-61 ms/frame on CUDA, 10/10 RGB twins byte-identical |
| **P5** | `fusion_node` (TSDF) | `just gate-fusion` | ☐ **next** |
| **P6** | `mesh_node` (marching cubes) | `just gate-mesh` | ☐ |
| **P7** | 6-DoF odometry | `just gate-odom` | ☐ |
| **P8** | `dashboard_node` | `just gate-dashboard` | ☐ |
| **P9** | `ansible/` — the Pi's configuration as code | `just gate-provision` | **✓ 2026-09-02** — PASS ×2, idempotent (11 → 0) |
| **P10** | Unit tests and the recipes that run them | `just test`, `just test-pi` | **✓ 2026-09-04** — 0 failures; now 71 gtest here, 11 on the Pi, 50 pytest |
| **P11** | Camera calibration, loaded and published | `just gate-calibration` | ☐ — promoted out of the future file by P3 |

**7 of 12 phases done.** No phase has been abandoned or rescoped. **One entry
has been promoted** out of
[../future/bootstrap-future.md](../future/bootstrap-future.md): loading a camera
calibration, which became P11 when P3 shipped a rotation estimator that K-all-
zeros keeps switched off.

**Phase numbers are identities, not an execution order.** Twice now a phase has
been written after the ones that follow it and executed before them: P9 on
2026-09-02, ahead of P1, putting the Pi's toolchain under version control before
P1 built against it; and P10 on 2026-09-04, ahead of P2, so that every phase
from P2 on inherits a place to put a unit test instead of inventing one.
**Next is P5.** Rule 1 says a plan never grows a phase in the middle, so all of
these were appended at the next unused number and the table's Status column
carries the ordering. That is the rule working, not a wart: "P1" still means the
same thing it meant yesterday. P11 arrived the same way — a promotion out of the
future file is an *append*, never an insertion next to the phase that wanted it.

## House rules for gate recipes

Learned at P0, and they apply to every `gate-*` recipe from here on — writing
one without them repeats a debugging session that has already happened.

- **Put `/usr/bin` first on `PATH`** in any recipe that builds. This box's
  `python3` is PlatformIO's venv, and `rosidl` generates message code in
  Python. If a build still fails after that, CMake cached the wrong interpreter
  — `rm -rf build install`, not another `colcon build`.
- **Run the launch in the foreground under `timeout -s INT`, background the
  probe.** A shell without job control sets SIGINT to `SIG_IGN` for background
  children, which leaves the launch un-interruptible and its children orphaned.
- **Detect leaks, do not `pkill` them.** A `pkill -f` pattern broad enough to
  catch a launch also matches any shell whose command line contains that
  pattern — including the one running the gate. Every gate ends by asserting
  the session tore itself down; `just stragglers` is the sweep.
- **Every gate prints its numbers**, passing or failing, so a run is evidence
  on its own and not just a green tick.
- **Measure each quantity on the machine where it is unambiguous.** P1's gate
  got this wrong twice in a row: rate measured on the dev box charges the node
  for Wi-Fi loss, and stamp age measured across two hosts carries their clock
  offset. Cross-machine numbers get printed, not asserted.
- **A phase also brings unit tests** for whatever logic it adds that can be
  tested without hardware, run by `just test` and `just test-pi` —
  [../../info/testing.md](../../info/testing.md). The gate is not a substitute
  for them, nor they for it.

---

## ✓ P0 — Workspace skeleton

**Done 2026-09-01.** `just gate-build` PASS: both distros build, all five
interface definitions byte-identical across them, `camera_link →
camera_optical_frame` resolving to the expected `[-0.5, 0.5, -0.5, 0.5]`, the
container up, and no stragglers on either machine afterwards. Run three times
for determinism, including once after a `rm -rf build install` clean rebuild.
What it cost, and what it taught, is annotated at the end of the phase.

**Goal:** the same source builds under two different ROS distros.

**Work**

- `pimesh_msgs`: `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`,
  `SaveMesh.srv`, `ResetMap.srv`.
- `pimesh_bringup`: the static `base_link → camera_link → camera_optical_frame`
  transforms, `config/pimesh.yaml`, an empty component-container launch.
- Justfile: `build`, `sync-pi`, `build-pi`, `test`, `stragglers`.

**Test:** `just gate-build` — builds here and, over SSH, on the Pi; diffs
`ros2 interface show` output for **all five** interfaces between the two
machines; then launches the frame tree and asserts `camera_link →
camera_optical_frame` resolves, the container comes up, and the session leaves
nothing running. Exits non-zero on any of those. Prints both build times and the
measured rotation.

The cross-distro build **is** this phase; a build that only succeeds here is
half a build.

### What happened

- **Built:** `pimesh_msgs` (3 messages, 2 services) and `pimesh_bringup`
  (`pimesh.launch.py`, `frames.launch.py`, `config/pimesh.yaml`), plus the
  justfile with `build`, `sync-pi`, `build-pi`, `test`, `pipeline`,
  `stragglers`, `gate-build`.
- **Build cost:** 8.2 s clean on the dev box, 23.1 s on the Pi (`pimesh_msgs`
  alone — rosidl generation on four cores). The gate's own figures are
  incremental rebuilds and read as 1 s / 3 s.
- **`camera_link → camera_optical_frame` = `[-0.5, 0.5, -0.5, 0.5]`**, i.e.
  roll −90°, yaw −90°. The standard body-to-optical rotation, confirmed by
  `tf2_echo` rather than by reading the launch file.
- **Trap found, in our own docs:** the dev box's `python3` is PlatformIO's venv,
  and **rosidl generates message code with Python** — so an interface package is
  *not* immune to it. `pimesh_msgs` failed with `No module named 'em'` until the
  build recipe put `/usr/bin` first on `PATH`, and it needed a **clean rebuild**
  because CMake had already cached the wrong interpreter. The docs said C++ was
  immune; they were wrong, and are now fixed.
- **Trap found, new:** a `pkill -f` teardown pattern also matches *any shell
  whose command line contains that string* — including the one running the
  gate, which killed itself twice this way. The gate now bounds the launch with
  `timeout -s INT` in the **foreground** (a shell without job control sets
  SIGINT to SIG_IGN for background children, which left the launch
  un-interruptible and its `static_transform_publisher`s orphaned) and *detects*
  leaks rather than pkilling them.

---

## ✓ P1 — Capture on the Pi

**Done 2026-09-02.** `just gate-capture` PASS twice: the node captures
**59.52 Hz against a 59.09 Hz raw-`v4l2-ctl` ceiling** measured in the same run
(and 2.4% under it in a darker run), stamps from `CLOCK_MONOTONIC` capture
times, holds a **5 ms** on-Pi stamp-to-receipt offset that moves **0.00 ms
between two separate launches** — the test `usb_cam` 0.8.1 fails — and exits
non-zero within 1-2 s on a missing or busy device. What it
cost, and what it taught, is annotated at the end of the phase.

**Goal:** frames off the sensor with honest timestamps, and nothing else.

**Work**

- `pimesh_camera/camera_node`: V4L2, `V4L2_PIX_FMT_MJPEG`, 1280×720, `mmap`
  buffer pool, publish `CompressedImage` verbatim plus transient-local
  `CameraInfo`.
- Stamp from the dequeued buffer's own timestamp, not from `now()` at publish.
- Exit non-zero with a clear message on a missing or busy device — never idle.
- A `just camera-reset` recipe that clears the C922's persistent V4L2 controls
  to a known baseline and prints every control current-vs-default.

**Test:** `just gate-capture` — with the camera reset, measures
`/image_raw/compressed` on the **dev box** over 30 s and asserts **≥ 40 Hz**
(the predecessor measured 42–60 fps at true 720p MJPG); launches the node twice
and asserts the stamp-vs-receipt offset is **within one frame interval and
differs between the two launches by < 5 ms** — which is exactly what `usb_cam`
0.8.1 fails, and the reason this node exists; then unplugs-by-proxy (opens
`/dev/video0` exclusively from a helper) and asserts the node exits non-zero
within 2 s. Prints the measured rate, both offsets, and the exposure mode they
were measured under.

**As built, the test differs from that sketch in two ways, both because the
sketch measured the wrong thing** (see *What happened*): the rate is asserted as
a **ratio against raw `v4l2-ctl` measured in the same run** rather than against
a fixed 40 Hz, and both the rate and the offset are measured **on the Pi**
rather than on the dev box. The dev-box figures are printed, not asserted.

### What happened

- **Built:** `pimesh_camera` — `V4l2Capture` (a ROS-free RAII wrapper over the
  device) and `CameraNode` (an `rclcpp` component with a thin `main`), plus
  `camera.launch.py`, `config/camera.yaml`, `tools/check_capture.py`, and the
  recipes `just cam`, `just camera`, `just camera-reset`, `just gate-capture`.
- **Measured:** 37 µs per frame from dequeue to publish. Capture between
  **-0.7% and 2.4%** of the raw v4l2 ceiling — i.e. indistinguishable from the
  hardware, and identical to four decimals across two launches
  (59.5204 / 59.5209 Hz). Wi-Fi then delivered **59.04 of 59.52 Hz** on one run
  and 53.6 on another. On-Pi stamp-to-receipt **5 ms**, drifting **0.00 ms**
  across launches.
- **The gate's first two designs were both wrong, in the same way: they measured
  on the dev box.** `ros2 topic hz` there counts frames that *arrived*, so it
  charged the node for Wi-Fi loss (5.3% "loss" for a node actually losing 2.4%);
  `ros2 topic delay` there is `now() - stamp` across two machines, so it carries
  their clock offset, which moved 11 → 38 ms between launches while the Pi-side
  figure did not move at all. **Measure each quantity on the machine where it is
  unambiguous**, and report the cross-machine number without asserting on it.
- **The ≥40 Hz threshold was not measurable as written.** The C922's rate tracks
  its auto-exposure time: 29.7 fps and 58.8 fps were both measured on this same
  day, same link, same control baseline, differing only in the light. Raw
  `v4l2-ctl` showed the same spread, so it is the camera, not the code. A fixed
  absolute threshold would have been a lighting test — hence the ratio.
- **Two parsing traps, both self-inflicted, both caught by the gate failing
  honestly:** `v4l2-ctl`'s closing line `Frame rate set to 60.000 fps` is the
  *request* echoed back, not a measurement (taking the last match compared the
  node against 60 and called a working node a 50% loss); and `pgrep -f
  camera_node` **matches the shell running it**, so the teardown check reported
  a straggler that was its own query. The project's own docs warn about the
  second one; it still landed.
- **A heredoc terminator at column 0 truncates a justfile recipe.** The
  assertions moved into `tools/check_capture.py`, which is better anyway — the
  thresholds are now readable and testable on their own.
- **`ament_target_dependencies` no longer exists in Lyrical** but is still
  present in Jazzy. Namespaced targets (`rclcpp::rclcpp`, `${sensor_msgs_TARGETS}`)
  exist in both, so that is what a package building under two distros must use.
- **Tests, added 2026-09-04:** 10 gtest cases in `pimesh_camera` and 13 pytest
  cases for `tools/check_capture.py`, passing on both machines (`just test`,
  `just test-pi`). Getting them written moved the timestamp conversion out of
  `wait_frame` — which needs a camera — into the free function
  `to_system_clock_ns`, which does not; **that refactor was worth more than the
  tests**. One case reproduces the usb_cam epoch bug in arithmetic and shows it
  landing 0.72 s late; another feeds its measured 0.223/0.362 s offsets to the
  gate's own assertions and checks the gate fails. The suite was mutation-checked
  (a 1 ms error turns 4 of 10 cases red).
- **`camera_info_manager` was dropped**, not added: it is not installed on the
  dev box, and at P1 it would only have served zeros. `CameraInfo` is published
  with **K all zeros** and a startup warning — deferred with a trigger in
  [../future/bootstrap-future.md](../future/bootstrap-future.md).

---

## ✓ P2 — The container, and proving intra-process

**Done 2026-09-04.** `just gate-ipc` PASS, twice: **10 of 10 frames arrived at
the address they were published at**, the same runs with `intra_process:=false`
produced **10 of 10 differing** addresses, `/image_raw/compressed` had exactly
**one** subscriber, and decode ran at **30.00 Hz for 1.88 ms mean / 2.41 ms
p95** against a ~4 ms target. What it changed, and the two bugs it found, is
annotated at the end.

**Goal:** one network subscriber, one decode, zero copies downstream.

**Work**

- `pimesh_perception/decode_node`: subscribe `/image_raw/compressed`,
  `cv::imdecode`, publish `bgr8` intra-process.
- The bringup container with `use_intra_process_comms=True`, and a probe
  component that logs the address of the buffer it received.

**Test:** `just gate-ipc` — asserts the probe's received-buffer address **equals**
the publisher's (a serialised path cannot produce that), and that
`ros2 topic info -v /image_raw/compressed` reports **exactly one** subscriber.
Prints both addresses and the subscriber count.

The single-subscriber assertion is the Wi-Fi constraint the whole architecture is
shaped around — see [../../info/architecture.md](../../info/architecture.md#why-one-container).

### What happened

- **Built:** `pimesh_perception` — `decode_node` (component + standalone
  `decode_node` executable), `ipc_probe_node`, and the ROS-free half the tests
  reach: `Mailbox<T>` and `decode_bgr8`. The container gained `decode_node` and
  three launch arguments: `probe`, and `intra_process` for the control run.
  `tools/check_ipc.py` holds the assertions, with 18 pytest cases of its own.

- **The gate proves the negative as well as the positive.** An address
  comparison that has only ever been seen to pass is not evidence that it can
  fail — and address *reuse* is visible in every one of these logs, the
  allocator handing the same block back frame after frame, so matching
  addresses could in principle be luck. So the gate runs the container a second
  time with `intra_process:=false` and requires the addresses to **differ**.
  They do, and the serialised path is ~1.4 ms slower per frame in the log
  timestamps against ~50 µs for the shared one.

- **Bug found by a unit test, before the node existed.** `cv::imdecode`'s
  three-argument form leaves its destination **untouched** when a decode fails
  — still holding the previous frame — so the obvious `!bgr.empty()` check
  reports success on a corrupt buffer. A node trusting it would have
  republished the last good image with a fresh timestamp: a frozen picture that
  every downstream stage and every rate check reads as live. Over Wi-Fi, where
  truncated frames are routine, that would have been a permanent low-grade
  fault nobody could see. Read the **return value**, not the destination.

- **Bug found by the gate's first run, in the gate itself.**
  **`/pipeline/stats` is shared by every node**, keyed by the `stage` field, so
  `ros2 topic echo --once` returns whichever stage published first — the
  camera. The gate asserted the *camera's* 59 Hz and 0.26 ms as if they were
  decode's, and they passed every threshold. Selecting by stage is the only way
  to read that topic; the selection now lives in `check_ipc.py` with tests, not
  in a `grep | tail -1`.

- **`ros2 launch` can hang forever on SIGINT, and it is not our code.** Roughly
  one run in three (measured: 1 of 2 consecutive runs, same command) its signal
  handler prints `This event loop is already running`, never signals its
  children, and never exits — and `timeout -s INT` without `-k` then waits
  forever with it, so the *gate* hangs instead of failing. Every `ros2 launch`
  in the justfile now carries `timeout -s INT -k <grace>`: SIGINT for an
  orderly shutdown, SIGKILL as a backstop. The gate reports which one was
  needed and still hard-asserts that nothing survived on either machine. This
  was a latent hang in `gate-build`, `gate-capture` and `just cam` too, and
  they were fixed in the same change.

- **Measured, and better than budgeted:** decode is **1.88–2.07 ms mean and
  2.41–2.56 ms p95** at 1280×720, against the ~4 ms target inherited from the
  predecessor. Zero mailbox drops and zero undecodable frames over the gate's
  runs — decode keeps up with the camera comfortably, which is what makes it
  safe to hang the rest of the pipeline off it.

- **The one copy that remains is at the message boundary**, not between stages:
  `cv::Mat` owns its pixels and the message must own its bytes, so the worker
  memcpys ~2.7 MB into the outgoing `Image`. Decoding straight into the
  message's `data` buffer is possible and was deliberately not done — it needs
  the frame size known in advance, and OpenCV silently reallocates elsewhere if
  it guesses wrong, which would publish a buffer nobody wrote to.

---

## ✓ P3 — Keypoints and a recorded clip

**Goal:** repeatable corners, matched across frames, cheap.

**Work**

- `keypoint_node`: `cv::ORB` at 500 features, pooled matching over a 10-frame
  window, Hamming distance threshold 64, annotated preview on
  `/keypoints/image/compressed`.
- Rotation-only odometry with its gates: ≥ 8 matched pairs, mean ray residual
  < 0.03 rad, and **hold the last pose** rather than publish a guess when the
  gate fails. The node logs which regime it is in.
- **Record `bags/desk1`** — a 60 s hand-held sweep of the room, `just record`.
  Every later phase replays it, so the numbers compare like for like.

**Test:** `just gate-keypoints` — replays `bags/desk1` and asserts ≥ 30 Hz
sustained, mean per-frame cost ≤ 8 ms measured against the node's own clock
(never against `header.stamp`), and matched-keypoint fraction within 5 points of
the predecessor's on the same clip. Prints all three, plus the pose-gate reject
rate.

### What happened

Landed 2026-09-07. `just gate-keypoints` PASS ×2, six assertions:

```
ok  the FIXTURE delivers 46.0 Hz median over 24 windows >= 30 Hz
ok  keeps up with decode: 96.4% of the frames decode delivered (42.4 vs 44.0 Hz)
ok  mean per-frame cost 6.99 ms <= 8.0 ms budget
ok  matched fraction 0.941 is 4.1 points from the predecessor's 0.90
ok  processed 958 of 993 frames offered (96.5%)
ok  the stats `detail` field arrived complete
```

`bags/desk1` is 61.6 s, 2608 frames at 42.4 Hz, 185 MiB of MJPEG.

**The rate assertion had to change, and the reason is the point.** The phase
asked for "≥ 30 Hz sustained", and the first version of the gate asserted that
on the minimum window rate off the replay. It failed at **8 Hz** — and decode,
in the very same window, read **10 Hz**. The stage was processing 8 of the 10
frames it was given, which is not a failure of anything. The bag was recorded
over Wi-Fi while the camera was carried around a room, so its instantaneous
rate swings between 7 and 60 Hz; **any absolute rate measured off a replay is a
measurement of the fixture.** So the node's own claim is asserted as a ratio
against decode — both numbers measured inside one process on one clock, which
is unambiguous in a way an absolute figure off a bag can never be — and the
absolute rate survives only as a floor on the fixture being usable, labelled as
such. The capability claim now lives where it belongs, in `latency_ms`: 7 ms a
frame is a 140 Hz ceiling. Live against the camera the stage measured
**59-60 Hz**, which is the number the phase was really asking about.

**The whole workspace was compiling at `-O0`.** colcon's default
`CMAKE_BUILD_TYPE` is the empty string, which passes no `-O` flag at all. The
rotation estimator ran **1.19 ms/frame unoptimised and 0.03 ms optimised — 40×**
— while OpenCV's own cost did not move a millisecond, because that code is
already optimised inside `libopencv`. So the effect is invisible for as long as
every expensive thing you call belongs to somebody else, and P5's TSDF and P6's
marching cubes are exactly where it stops being invisible. `just build` and
`just build-pi` now pass `-DCMAKE_BUILD_TYPE=RelWithDebInfo`.

**The preview moved to its own thread.** Drawing 500 rich keypoints and
JPEG-encoding a 1280×720 frame costs ~9 ms, and on the tracking thread that
showed as a p95 of **20.6 ms against a 10.9 ms mean** plus a steady trickle of
dropped frames. It is the house rule applied to our own code: a picture for
humans is work that costs milliseconds, so it gets a thread and a one-deep
mailbox like every other expensive stage. p95 fell to **9.6 ms**.

**The first `desk1` take was thrown away at 13.7 Hz.** `/camera_info` (500
bytes) and `/image_raw/compressed` (90 kB) arrived at *the same* rate, which
rules out Wi-Fi loss — a link dropping megabyte frames does not drop tiny ones
equally. The cause was `exposure_dynamic_framerate=1`, the trap CLAUDE.md
already documents: the C922 trades frame rate for exposure time in dim light,
and a room sweep points at dim things. At 13.7 Hz the exposure is ~73 ms, so
that clip was also heavily motion-blurred — a bad fixture twice over.
`just camera-reset` first, then re-record: 42.4 Hz.

**`ros2 topic echo` silently truncates strings past 128 characters** with a
trailing `...`, and `detail` carries the reject breakdown past that mark. The
gate printed `uncalibrated ?` for a run in which the number was present all
along. `--full-length` fixes it, and the gate now *fails* on an incomplete
`detail` rather than reporting less than it promised — a truncated field and a
missing one look identical from the far end.

**The odometer has never run on a real frame.** K is all zeros, so every one of
the 1071 frames rejected with `no_intrinsics` and the node reported
`regime=detect_only`. That is the honest behaviour — a fabricated focal length
would turn "no pose" into "a confident wrong pose" — but it means the rotation
gates are covered by `test_rotation` and by nothing else. **P11 closes this**,
and it was promoted out of the future file for exactly this reason.

**Tests: 26 new gtest cases and 19 pytest.** `test_rotation` (16) drives the
geometry with synthetic ray bundles and known rotations; `test_orb_tracker`
(10) drives detection and both matchings with synthetic scenes. Two were
mutation-checked: disabling the determinant guard turns
`Kabsch.NeverReturnsAReflection` red, and unbounding the pooled window turns
`TheWindowForgivesAFrameOfChurn...` red. The reflection test **failed to fail**
on its first writing — it was built from a cleanly rotated bundle, and
`det(P·(R·P)ᵀ)` is always positive, so the SVD could not have reflected no
matter what the code did. It was rebuilt around a bundle whose best-fit
orthogonal transform genuinely is a reflection.

---

## ☐ P4 — Depth on the GPU

**Goal:** metric depth at the rate the GPU can sustain.

This phase carries the project's real setup risk — see
[../../info/setup.md](../../info/setup.md#gpu). Do the toolchain work **first and
standalone**: a 20-line C++ program that loads the model, runs one frame, and
prints the provider and the time. No ROS code until that prints
`CUDAExecutionProvider`.

**Work**

- Install the ONNX Runtime GPU release tarball and a `just fetch-model` recipe
  that downloads Depth Anything V2 Small and verifies its sha256.
- `depth_node`: ONNX Runtime C++, CUDA execution provider with an explicit CPU
  fallback that **logs which provider it got**, 518² input, warm the session at
  startup.
- Publish `/depth` (32FC1, metres, clipped at 6 m before the reciprocal) and
  `/depth/rgb`, both carrying the **input frame's** stamp and
  `camera_optical_frame`.

**Test:** `just gate-depth` — replays `bags/desk1`, asserts the startup log names
`CUDAExecutionProvider`, mean per-frame cost **≤ 80 ms** (the predecessor
measured 72–79 ms on this GPU), and that `/depth/rgb` is byte-identical to the
frame each depth map was inferred on. Prints mean, p95, and the provider.

A CPU fallback is a **failed** test however good the mesh looks.

### What happened

Landed 2026-09-08. `just gate-depth` PASS ×2:

```
ok  the startup log names CUDAExecutionProvider
ok  the running session reports provider=CUDAExecutionProvider
ok  the model loaded and warmed (state=ready)
ok  mean per-frame cost 55.4 ms <= 80 ms budget
ok  55.4 ms is GPU-shaped (< 150 ms; the CPU path measured 280-305 ms)
ok  179/187 depth messages (96%) have a /depth/rgb at the same stamp
ok  10/10 /depth/rgb frames are BYTE-IDENTICAL to the camera frame
```

**The setup risk did not materialise, and the standalone-first order is why.**
`just gpu-probe` — 130 lines of C++ with no ROS, no colcon and no CMake in it —
printed `CUDAExecutionProvider, 52.5 ms/frame` before a single line of
`depth_node` existed. Every later failure could therefore be attributed to our
code rather than to the toolchain, which is the whole value of doing it in that
order.

**52.5 ms, against the predecessor's 72-79 ms** for the same ONNX file on the
same GPU. The rewrite is 1.4× faster here and the budget has real headroom. In
the node the figure is **55-61 ms**, the difference being preprocessing, the
resize back up to 1280×720, and one memcpy into the message.

**ONNX Runtime's GPU tarball ships no CUDA runtime.** It carries
`libonnxruntime.so` and the CUDA provider and expects `libcudart.so.13`,
`libcublas`, `libcublasLt` and `libcurand` to already be on the loader path —
`objdump -p` on the provider names all four. They come from NVIDIA's pip wheels
rather than the apt toolkit: the runtime alone is ~2 GB against the toolkit's
~5, it needs no root and no apt repo, and the pinned versions are the ones the
predecessor already had working on driver 595.84. `nvcc` is still absent
afterwards, which is correct — nothing in this phase compiles CUDA.

**It installs to a user-owned prefix, not `/opt`.** `sudo` on this box needs a
password, and a recipe that stops to prompt for one cannot be run by a gate.
`docs/info/setup.md` said `/opt`; it now says why it does not. `PIMESH_OPT`
overrides it for anyone who does install as root.

**The CUDA libraries are dlopened by ONNX Runtime's provider, not linked by
us**, so no rpath of ours reaches them and `LD_LIBRARY_PATH` has to be exported
by every recipe that starts the container. Get that wrong and the session falls
back to the CPU — five times slower, still producing correct-looking depth, and
reported as a warning nobody reads. That is why the gate asserts the provider
from **two independent places**: the startup log line and the per-second stats
field.

**The pairing probe was wrong twice, in the same way P3's rate assertion was.**
It first reported "1 depth message has no `/depth/rgb`" — but it subscribes from
outside the container with `KEEP_LAST(1)` and services one callback per
`spin_once`, so it dropped independently on each topic and compared different
subsets: 9 depth against 11 rgb. Deepening the queue was not enough, because it
also *stopped* at its tenth byte comparison and truncated both stamp sets
mid-stream, leaving the boundary frame looking orphaned. Running the full window
and judging only the interior of it took the sample from 8 depth messages to
187. **Both fixes removed an artefact rather than loosening a threshold**, which
is the distinction that matters: the pairing figure is asserted as a ratio and
labelled as bounded by the probe's own reception, while the exactness claim
rides on the byte comparison, which no amount of dropping can fake.

**Tests: 12 new gtest cases**, and the refactor that made them possible.
`preprocess_frame` and `relative_to_metres` were pulled out of `DepthModel`
into `perception_core`, where nothing includes an ONNX Runtime header — so they
run on a machine that has never installed it, and on the Pi's toolchain if it
ever needs to. They cover the class of bug this stage is most exposed to: the
kind that yields a plausible depth map that is quietly wrong and throws
nothing. Both were mutation-checked — skipping the BGR→RGB conversion turns
`ConvertsBgrToRgb` red, and dividing before bounding instead of after turns
`NeverProducesInfinityOrNaN` red.

**`depth_scale` is still 10.0 and still arbitrary.** Monocular depth is
relative; every metre figure this stage publishes is provisional until P5's tape
measure pins it. The gate says so on every run rather than letting a reader
assume otherwise.

---

## ☐ P5 — Fusion

**Goal:** a stream of posed depth maps becomes one consistent volume.

**Work**

- `fusion_node`: spatially hashed TSDF, 1.5 cm voxels, truncation ~4 voxels,
  weighted colour, weight threshold 3.
- Per-frame scale alignment against a ray-cast of the existing volume — skip
  below 20% valid overlap, refuse corrections beyond 15%, first frame defines
  the map's scale.
- Single-slot mailbox with a drop counter published on `/pipeline/stats`.
- **Pin `depth_scale` by tape measure.** *Needs a person:* measure one flat
  surface at a known distance, read the depth at its centre, and set the scale so
  they agree — the predecessor's room came out at 2.69. One recording of that
  surface at a measured distance turns this into a replayable test later; make
  the recording while you are there (`bags/scale1`).

**Test:** `just gate-fusion` — replays `bags/desk1` and asserts integrate cost
≤ 20 ms at 13 Hz with **zero growth** in the mailbox backlog over the clip, then
runs the paired-surface check: two views of the same wall, integrated, reporting
the gap between the two surfaces with alignment **on and off**. Asserts the
aligned gap is smaller. Prints both gaps (the predecessor's went 7.8 → 5.7 cm)
and the integrate cost.

---

## ☐ P6 — Surface

**Goal:** a mesh out of the volume, without stalling the integrator.

**Work**

- `mesh_node`: marching cubes over allocated blocks on a **snapshot copy**, taken
  under a short lock and meshed without holding it.
- Cleanup: drop components under 30 triangles, fan-fill interior boundary loops
  under 0.25 m, **leave each component's largest loop open** — unseen space is
  never invented.
- `/world/mesh` as a `Marker` capped at 120 k triangles by **quadric
  decimation, never subsampling**; `/world/save_mesh` writes the full-detail PLY.
- `just mesh-views` — offscreen renders of a saved PLY from three fixed angles.

**Test:** `just gate-mesh` — replays `bags/desk1` and asserts the integrate rate
shows **no dip** at mesh time (max inter-integration gap ≤ 2× the median), the
published triangle count is under the cap, and the mesh has **no pinholes**
(boundary-loop count below the pre-decimation count). Then runs `just mesh-views`
and writes three PNGs. Prints the counts and the paths of the renders — those
images are the evidence, not the RViz window.

---

## ☐ P7 — 6-DoF odometry

**Goal:** translation stops being invisible, so the surface stops smearing.

**Work**

- Back-fill `keypoint_node` with depth-backed 3D–3D pose estimation on exact
  RGB-D triples (a frame's ORB output, its own depth, and the TF at its stamp).
- Rotation-only stays as a selectable fallback; the node logs its regime.
- Keyframe store: descriptors, bearing rays and 3D landmarks, a new keyframe at
  ~18° of view change or 0.3 m of motion, ~16 kB each.

**Test:** `just gate-odom` — replays `bags/desk1` through both regimes and
asserts the 6-DoF run's paired-surface gap is smaller than the rotation-only
run's on the same clip. Prints both gaps and the trajectory length each regime
reported (a hand-held pan carries ~0.9 m of real arm arc, which rotation-only
reports as zero).

---

## ☐ P8 — Dashboard

**Goal:** one browser tab that shows the pipeline, and cannot slow it down.

**Work**

- `dashboard_node`: HTTP + WebSocket in one C++ node, vendored assets, the
  channels and layout in [../../info/dashboard.md](../../info/dashboard.md).
- Server-side pacing with a send-buffer threshold and a drop counter; the panel
  separates *dropped by design* from *dropped in transport*.
- Staleness measured on **receipt time**, never `header.stamp`.

**Test:** `just gate-dashboard` — replays `bags/desk1` twice, once with a
headless browser client attached, and asserts every pipeline rate is within 2% of
the no-client run; kills the client mid-clip and asserts no rate change; stops a
publisher and asserts the STALE flag appears within 2 s. Prints the two rate
tables side by side.

---

## ✓ P9 — Provision the Pi with Ansible

**Done 2026-09-02.** `just gate-provision` PASS, twice: the playbook is
idempotent, the Pi's three ROS variables equal the dev box's, `cyclonedds.xml`
pins `wlan0`, the six build dependencies are installed, `linux/videodev2.h` is
present, the C922 by-id symlink resolves to `/dev/video0`, `~/.profile` carries
**exactly one** managed block, and `just gate-build` still passes. g++ 13.3.0.
What it changed, and what it taught, is annotated at the end of the phase.

**Added 2026-09-02, and it was the next phase to execute — before P1.** It is
numbered 9 because rule 1 forbids growing a plan in the middle, not because it
comes last. Reference: [../../info/ansible.md](../../info/ansible.md).

**Goal:** the Pi's configuration is a file in this repo, and re-asserting it is
one command that reports how much it had to change.

Today it is not. Measured 2026-09-02, the Pi is already in the state P1 needs —
`ROS_DOMAIN_ID=42`, `rmw_cyclonedds_cpp`, `CYCLONEDDS_URI` pinned, `v4l-utils`,
`build-essential`, `ros-jazzy-rclcpp-components`, `-image-transport` and
`-camera-info-manager` all installed, and `linux/videodev2.h` present from
`linux-libc-dev`, which is all a raw-ioctl V4L2 node compiles against.
(`libv4l-dev` is *not* installed and is **not** needed: it provides the
`libv4l2` conversion wrapper, and `camera_node` does MJPEG passthrough with
plain ioctls. If P1 ever reaches for `libv4l2.h`, that is the moment it becomes
a role task — not before.)

So this phase changes almost nothing on the machine, and that is the point. The
Pi is correct **because the predecessor's playbook put it that way**, and
nothing in this repo records which of those facts this project depends on. P9's
deliverable is that the list becomes re-assertable and survives a reflash, not
that it is long. Expect the first apply to report a small `changed` count and
the second to report zero — and if the first is large, the roles have drifted
from what the machine actually needs, which is itself the finding.

**Work**

- `ansible/` at the repo root: `ansible.cfg` (with
  `interpreter_python = /usr/bin/python3` — same shadowed-Python trap as the
  build), `inventory.yml` with the single managed host `pi`,
  `requirements.yml` pinning `ansible.posix >= 2.0`, `site.yml`, and
  `group_vars/robot.yml`.
- Roles **forked from `~/Documents/piros2/ansible` and trimmed**, not written
  fresh — they have run against this Pi for weeks: `ros2_apt`, `ros2_install`,
  `ros2_env`, `toolchain` (new: `build-essential`, `cmake`, and the
  `ros-jazzy-*` build dependencies P1 links against), `camera`, `wifi`.
- **No `workspace` role.** Ansible owns machine state; `just sync-pi` and
  `just build-pi` own the code. Two mechanisms for one job is how they end up
  disagreeing about which ran last.
- `ros2_env` reuses the **predecessor's `blockinfile` markers** so it replaces
  that block rather than stacking a second copy of the same three exports.
- Recipes: `just provision`, `just provision-check` (`--check --diff`),
  `just gate-provision`.

**Test:** `just gate-provision` — runs `ansible robot -m ping`, then applies
`site.yml` **twice** and asserts the second run reports **`changed=0` and
`failed=0`** (idempotence is the whole claim); then asserts over
`ssh pi "bash -lc '…'"` that `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and
`RMW_IMPLEMENTATION` are **equal to the dev box's own values** — not merely
non-empty, since drift is what the playbook exists to prevent — that
`cyclonedds.xml` pins `wlan0`, that `v4l-utils` and the `ros-jazzy-*` build
dependencies are installed and `linux/videodev2.h` exists, that the C922's
`by-id` capture symlink resolves, and that `~/.profile` contains
**exactly one** managed ROS block (two means the predecessor's tree also still
owns this host). Finishes by running `just gate-build`, so a provisioning change
that breaks the cross-distro build fails here rather than at P1. Prints both
`changed=` counts, the three variables side by side for both machines, and
`g++ --version`.

Asserting *equality across the two machines* is the point. A gate that only
checked the Pi would pass while the dev box drifted, which is the exact failure
this project is most exposed to and the one that produces silence instead of an
error.

**Not in scope:** the dev box. It is the control node, it is still provisioned
by the predecessor's tree, and bringing it under this playbook is a deferred
entry with a trigger in
[../future/bootstrap-future.md](../future/bootstrap-future.md).

### What happened

- **Built:** `ansible/` — `ansible.cfg`, `inventory.yml` (one host),
  `requirements.yml`, `site.yml`, `group_vars/robot.yml`, and six roles forked
  from the predecessor and trimmed: `ros2_apt`, `ros2_install`, `ros2_env`,
  `toolchain` (new), `camera`, `wifi`. Recipes: `just provision`,
  `just provision-check`, `just gate-provision`.
- **First apply changed 11 tasks; the second changed 0**, which is the
  idempotence claim. Those 11 were the ownership hand-off, *not* drift in what
  the machine had installed — every apt package the roles name was already
  present, exactly as the phase predicted.
- **Trimmed, deliberately:** no `usb_cam` (this project replaces it, and a
  second thing able to open the exclusive `/dev/video0` is a liability), no
  `workspace` role (`just sync-pi` owns the code), no fish (the Pi has bash),
  no `libv4l-dev` (MJPEG passthrough uses plain ioctls, and
  `linux/videodev2.h` from `linux-libc-dev` is all it compiles against).
- **The hand-off is real and worth knowing about.** The Pi's login shells now
  source **this repo's** overlay (`~/ros2_pi/install`) instead of the
  predecessor's (`~/piros2/install`), so `ssh pi "bash -lc 'ros2 …'"` sees
  `pimesh_*` and no longer sees `piros2_*`. The predecessor's playbook must not
  be run against the Pi again — its `workspace` role would point it back.
- **Bug found in the fork, before it bit.** The shell snippet reads a *cached*
  copy of the ROS environment, invalidated by the mtime of the underlay and of
  the workspace's `local_setup.bash` — **neither of which changes when the
  snippet itself does**. Moving the overlay path would therefore have left every
  login shell sourcing a cache built against the old workspace, with nothing
  visibly wrong anywhere. The role now drops the cache whenever the snippet
  changes. The predecessor has the same latent gap; it never surfaced there
  because its overlay path never moved.
- **A `--check` diff is not the apply.** The dry run showed a *second*
  `.profile` block being added, because in check mode the "remove legacy
  distro-named blocks" task removes nothing, so the task after it still sees the
  old block. The real apply removed, then added, leaving one. Read a check diff
  as "what each task would do given the state it sees", never as "what the file
  will look like".
- **The Pi was still on the older distro-named marker scheme**
  (`— ROS 2 jazzy environment`) while the predecessor's tree had moved to
  distro-free markers. The legacy-sweep loop this role inherited is what made
  the hand-off a replacement rather than a second block.

---

## ✓ P10 — Tests, and one command that runs them

**Done 2026-09-04.** `just test` — 10 gtest cases in `pimesh_camera` and 13
pytest cases for the gate tools, 0 failures — and `just test-pi`, the same gtest
cases under Jazzy on aarch64, 0 failures. Reference:
[../../info/testing.md](../../info/testing.md).

**Added 2026-09-04, and executed ahead of P2**, for the same reason P9 was
executed ahead of P1: it is machinery the phases after it depend on, and the
cost of adding it later is that the phases in between quietly do without.
Numbered 10 because rule 1 forbids growing a plan in the middle.

**Goal:** a phase that adds testable logic has somewhere to put a test and one
command that runs it, on **both** machines — and that command exits non-zero
when something is wrong.

Before this phase, it did not. All three `package.xml` files declared
`ament_lint_auto` and `ament_lint_common` as `test_depend`, no `CMakeLists.txt`
had a `BUILD_TESTING` block, and `colcon test` reported **`3 packages finished`,
`0 tests`** — a green result that asserted nothing. A `test_depend` that names
nothing which runs is decoration, and a test command that passes on an empty
suite is worse than no test command: it answers the question "is this covered?"
with a tick.

The **gate recipes are not this layer and do not replace it**. A gate exercises
the real system across two machines and a radio link and takes minutes; when one
fails it tells you *something* is wrong in a system with a dozen candidates.
Unit tests are what make that bisectable. The distinction is now written into
rule 2 of [../README.md](../README.md), so every later phase inherits it.

**Work**

- **`if(BUILD_TESTING)` in `pimesh_camera`**, with `ament_cmake_gtest` and
  `ament_add_gtest(test_v4l2_capture test/test_v4l2_capture.cpp)` linking
  `v4l2_capture` — the ROS-free half of the package, which is a separate CMake
  target precisely so a test can link it without spinning a node.
  `<test_depend>ament_cmake_gtest</test_depend>` replaces the two lint
  declarations; the same unused declarations come out of `pimesh_msgs` and
  `pimesh_bringup`, which have no compiled logic to test.
- **A refactor, which was the real deliverable.** The stamp conversion was three
  lines inside `wait_frame`, a 90-line method that needs a camera, so it could
  not be tested at all. It is now the free function `to_system_clock_ns`, with
  `timestamp_source_from_flags` beside it, and the reasoning about usb_cam's
  epoch bug written above it. **If logic is hard to test, that is a fact about
  the code** — the extraction was worth more than the cases it enabled.
- **`src/pimesh_camera/test/test_v4l2_capture.cpp`** — 10 cases in 3 suites:
  the offset arithmetic and that it is independent of when the clock pair was
  sampled; the `V4L2_BUF_FLAG_TIMESTAMP_*` values restated as literals, so a
  change in `<videodev2.h>` fails a test rather than silently changing
  provenance; and the three failure paths (missing device, regular file,
  non-V4L2 character device) each throwing with the path and errno.
  One case, `DoesNotReproduceTheUsbCamEpochBug`, reproduces the inherited bug
  in arithmetic and shows it landing ~0.72 s late where ours is exact.
- **`tools/test_check_capture.py`** — 13 cases. `check_capture.py` is what
  decides whether P1 passes, so it is tested like anything else that can say
  "everything is fine": a healthy run passes, a third of the frames dropped
  fails, a 3% sampling difference does *not* fail (or the gate cries wolf every
  run), and feeding it usb_cam's measured 0.223 / 0.362 s offsets makes it fail.
  One case asserts a **missing** hardware measurement fails rather than quietly
  passing on the strength of the checks that could still run.
- **Recipes:** `just test` runs colcon and pytest, reports **both** rather than
  stopping at the first, and exits non-zero if either fails — `tools/` is not a
  ROS package, so colcon cannot see it, which is why there are two passes.
  `just test-pi` syncs, builds and runs the gtest cases on the Pi.
- **`docs/info/testing.md`** — what each layer is for, what is covered today,
  how to add a case, and why the linters are off.

**Every test is code the Pi builds.** `just test-pi` compiles these cases under
Jazzy with g++ 13.3.0, so the same cross-distro rules apply to a test file as to
a node: C++17, namespaced targets, no `ament_target_dependencies`. A test that
has only ever run on Lyrical says nothing about the machine that runs the camera.

**Test:** `just test` — asserts the workspace's gtest cases and the `tools/`
pytest cases all pass on the dev box; prints `colcon test-result`'s count and
pytest's, and exits non-zero if either suite fails. Then `just test-pi` — the
same gtest cases under the other distro and compiler, printing its own count.
Neither may require hardware: **the moment a test needs a device it is a gate**,
and belongs in a `gate-*` recipe instead.

**A suite that has never failed is not evidence.** The claim that these tests
would catch a regression is closed by breaking the thing they cover and watching
them go red — done here, and recorded below. Every later phase's tests carry the
same obligation.

**Not in scope:** the `ament_lint_auto` linters (`copyright`, `cpplint`,
`uncrustify`). `ament_copyright` wants a header on every file and a `LICENSE` in
every package, and `uncrustify` would reformat code that is currently readable;
that is a deliberate change in its own commit, not a side effect of adding the
first real tests. Deferred with a trigger in
[../future/bootstrap-future.md](../future/bootstrap-future.md). Also not in
scope: node-level `launch_testing` tests — there is one node and it needs a
camera, so there is nothing yet for a launch test to assert that the P1 gate
does not. P2 brings the first node that can be tested without hardware.

### What happened

- **Built:** the `BUILD_TESTING` block and `test/test_v4l2_capture.cpp` in
  `pimesh_camera`, `tools/test_check_capture.py`, the `just test` and
  `just test-pi` recipes, and `docs/info/testing.md`. Rule 2 in
  [../README.md](../README.md) gained the gate-vs-test distinction, and the
  house rules above gained the bullet that every phase brings unit tests.
- **Measured:** `just test` → `Summary: 11 tests, 0 errors, 0 failures,
  0 skipped` from colcon and `13 passed in 0.40s` from pytest, in about a
  second. `just test-pi` → `Summary: 11 tests, 0 errors, 0 failures, 0 skipped`
  under Jazzy on aarch64.
- **`colcon test-result` says 11 where gtest says 10**, and the difference is
  not a missing case: it aggregates two XML files, the gtest report with its 10
  cases and CTest's own record of having run the binary. `colcon test-result
  --all` prints them separately and is worth remembering before hunting for an
  eleventh test that does not exist.
- **Mutation-checked.** Adding 1 ms to `to_system_clock_ns` turns 4 of the 10
  cases red; removing it turns them green again. That is the only reason to
  believe the suite covers what it claims to.
- **The lint declarations were removed rather than left.** They had been in
  every `package.xml` since P0, generated by `ros2 pkg create` and never
  invoked. Deleting a `test_depend` that names nothing which runs is not a loss
  of coverage — there was none — it is the file stopping making a claim it did
  not keep.
- **The refactor came out of trying to write the test, not the other way
  round.** Nothing about `wait_frame` looked wrong until something had to call
  its arithmetic without a camera attached, at which point the three lines that
  matter were visibly buried in ninety that do not.

---

## ☐ P11 — Camera calibration, loaded and published

**Promoted out of [../future/bootstrap-future.md](../future/bootstrap-future.md)
on 2026-09-07**, by P3. Its trigger there was "a calibration YAML exists", and
the reasoning was that a file-loading path with no file to load is a more
elaborate way of publishing zeros. P3 changed the calculation: it shipped a
rotation estimator with gates, thresholds and 16 unit tests that **has never run
on a real frame** — every one of the 1071 frames in the gate run rejected with
`no_intrinsics`. Untested-on-real-data geometry does not get more trustworthy by
waiting, and P4's depth unprojection and P5's TSDF both need real intrinsics
anyway.

**Goal:** `/camera_info` carries a measured K, and the odometer runs.

**Work**

- Run `camera_calibration` against the C922 at 1280×720 and write the YAML.
  *Needs a person:* holding a checkerboard in front of a camera is a physical
  act. Store it in `pimesh_camera/config/` — it describes this camera, and it
  belongs beside the node that publishes it.
- Load it in `camera_node` through `camera_info_manager`, driven by the
  `camera_info_url` parameter the node **already declares and already warns
  about**. The package is on the Pi (the `toolchain` role installs it, and P9's
  gate names it for this reason).
- Keep the uncalibrated path exactly as it is. A missing or unreadable file must
  still publish zeros and still warn — the honest signal is what makes
  `is_calibrated()` downstream mean anything, and a calibration that silently
  falls back to a plausible-looking guess is worse than none.
- Nothing in `keypoint_node` changes. It already subscribes `/camera_info`,
  already tests K for zeros, and already reports `regime=` on every stats
  message; a real K simply switches it from `detect_only` to `rotation_only`.

**Test:** `just gate-calibration` — asserts `/camera_info` carries `k[0] > 0`
with a plausible focal length (the C922's spec FOV puts fx near 900 px at 720p,
so 700–1100 is the sanity window, and the point of the bound is to catch a
calibration that converged on nonsense rather than to grade it); replays
`bags/desk1` and asserts `keypoint_node` reports `regime=rotation_only` with a
pose-gate reject rate **below 40%** and `rej_nointr=0`; and asserts the
uncalibrated path still works by launching with `camera_info_url` pointing at a
file that does not exist and requiring K all zeros plus the warning. Prints the
recovered fx, fy, cx, cy, the reprojection error the calibration reported, and
the reject-rate breakdown.

The reject rate is the assertion that matters: it is the first evidence that the
rotation gates behave on real data rather than on synthetic ray bundles.
