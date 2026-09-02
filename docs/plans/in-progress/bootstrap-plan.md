# Bootstrap plan — one webcam to a live mesh, in C++

**Started 2026-09-01.** The build order for the whole pipeline.

Written to the rules in [../README.md](../README.md): **stable phase numbers**,
**every phase ends in a test that is a command**, and **every phase is
executable** — startable the moment the plan reaches it, with nothing to wait
for. Work that is not executable yet lives in
[../future/bootstrap-future.md](../future/bootstrap-future.md), never here.

Each phase's test recipe is written **in the same change as its code**. The
recipes named below do not exist yet; creating them is part of the phase.

The predecessor [`~/Documents/piros2`](../../../../piros2) already does all of
this in Python. **It is the reference and the yardstick** — where a number exists
over there, the C++ version is measured against it, so "the rewrite is faster"
stays a measurement rather than an article of faith.

**Status legend:** ☐ not started · ▶ in progress · ✓ done (annotated with the
date and what the test printed).

---

## ✓ P0 — Workspace skeleton

**Done 2026-09-01.** `just gate-build` PASS: both distros build, all five
interface definitions byte-identical across them, `camera_link →
camera_optical_frame` resolving to the expected `[-0.5, 0.5, -0.5, 0.5]`, the
container up, and no stragglers on either machine afterwards. Run twice for
determinism. What it cost, and what it taught, is annotated at the end of the
phase.

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

## ☐ P1 — Capture on the Pi

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

---

## ☐ P2 — The container, and proving intra-process

**Goal:** one network subscriber, one decode, zero copies downstream.

**Work**

- `pimesh_perception/decode_node`: subscribe `/image_raw/compressed`,
  `cv::imdecode`, publish `bgr8` intra-process.
- The bringup container with `use_intra_process_comms=True`, and a temporary
  probe component that logs the address of the buffer it received.

**Test:** `just gate-ipc` — asserts the probe's received-buffer address **equals**
the publisher's (a serialised path cannot produce that), and that
`ros2 topic info -v /image_raw/compressed` reports **exactly one** subscriber.
Prints both addresses and the subscriber count.

The single-subscriber assertion is the Wi-Fi constraint the whole architecture is
shaped around — see [../../info/architecture.md](../../info/architecture.md#why-one-container).

---

## ☐ P3 — Keypoints and a recorded clip

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
