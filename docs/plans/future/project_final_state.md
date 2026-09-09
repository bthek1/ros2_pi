# Project final state — one webcam to a live mesh, in C++

**Everything this project is meant to become, in one file.** Two halves: the
**build order** (P0–P8, the phases that get there) and the **deferred register**
(work that is not executable yet, each entry with its trigger).

This replaces the bootstrap plan issue and its companion future file, which were
combined here on 2026-09-09. Nothing was dropped in the merge — the phases below
are verbatim from the issue body, the deferred entries verbatim from
`bootstrap-future.md`.

Written to the rules in [../README.md](../README.md): **stable phase numbers**,
**every phase ends in a test that is a command**, and **every phase is
executable** — startable the moment the work reaches it, with nothing to wait
for.

Each phase's test recipe is written **in the same change as its code**. The
recipes named below do not exist yet; creating them is part of the phase.

The predecessor `~/Documents/piros2` already does all of this in Python. **It is
the reference and the yardstick** — where a number exists over there, the C++
version is measured against it, so "the rewrite is faster" stays a measurement
rather than an article of faith.

**Status legend:** ☐ not started · ▶ in progress · ✓ done (annotated with the
date and what the test printed).

---

## The five milestones

The nine phases are built as five milestone issues, each a **contiguous slice of
the phase list below**. No issue renumbers from zero — `P4` means depth in every
doc, commit and conversation.

| | Issue | Phases | True when it closes |
| --- | --- | --- | --- |
| A | [#4](https://github.com/bthek1/ros2_pi/issues/4) | P0–P1 | One source tree builds under both distros; the Pi ships stamped MJPEG |
| B | [#5](https://github.com/bthek1/ros2_pi/issues/5) | P2–P3 | One reader, one decode, corners on it, and `bags/desk1` exists |
| C | [#6](https://github.com/bthek1/ros2_pi/issues/6) | P4 | Depth on the GPU at ≤ 80 ms, CUDA provider named in the log |
| D | [#7](https://github.com/bthek1/ros2_pi/issues/7) | P5–P6 | A triangle mesh you can recognise your room in |
| E | [#8](https://github.com/bthek1/ros2_pi/issues/8) | P7–P8 | Translation is visible, and one tab shows the pipeline |

Each issue also carries a **`just view-*` RViz recipe** — a viewer for a person,
never the evidence. The gates below are what pass or fail a phase. The view
layer adds no new topics, so it adds no scope to any phase.

---

# Part 1 — The build order

## ☐ P0 — Workspace skeleton

**Goal:** the same source builds under two different ROS distros.

**Work**

- `pimesh_msgs`: `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`,
  `SaveMesh.srv`, `ResetMap.srv`.
- `pimesh_bringup`: the static `base_link → camera_link → camera_optical_frame`
  transforms, `config/pimesh.yaml`, an empty component-container launch.
- Justfile: `build`, `sync-pi`, `build-pi`, `test`, `stragglers`.

**Test:** `bash tools/gates/build.sh` — runs `colcon build` here and, over SSH, on the Pi,
then `ros2 interface show pimesh_msgs/msg/Keypoints` on both machines and
diffs the two outputs. Exits non-zero on any build failure or if the interface
definitions differ. Prints both distro names and both build times.

The cross-distro build **is** this phase; a build that only succeeds here is
half a build.

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

**Test:** `bash tools/gates/capture.sh` — with the camera reset, measures
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

**Test:** `bash tools/gates/ipc.sh` — asserts the probe's received-buffer address **equals**
the publisher's (a serialised path cannot produce that), and that
`ros2 topic info -v /image_raw/compressed` reports **exactly one** subscriber.
Prints both addresses and the subscriber count.

The single-subscriber assertion is the Wi-Fi constraint the whole architecture is
shaped around — see
[../../info/architecture.md](../../info/architecture.md#why-one-container).

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

**Test:** `bash tools/gates/keypoints.sh` — replays `bags/desk1` and asserts ≥ 30 Hz
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

**Test:** `bash tools/gates/depth.sh` — replays `bags/desk1`, asserts the startup log names
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

**Test:** `bash tools/gates/fusion.sh` — replays `bags/desk1` and asserts integrate cost
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
- `tools/mesh-views.sh` — offscreen renders of a saved PLY from three fixed angles.

**Test:** `bash tools/gates/mesh.sh` — replays `bags/desk1` and asserts the integrate rate
shows **no dip** at mesh time (max inter-integration gap ≤ 2× the median), the
published triangle count is under the cap, and the mesh has **no pinholes**
(boundary-loop count below the pre-decimation count). Then runs `tools/mesh-views.sh`
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

**Test:** `bash tools/gates/odom.sh` — replays `bags/desk1` through both regimes and
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

**Test:** `bash tools/gates/dashboard.sh` — replays `bags/desk1` twice, once with a
headless browser client attached, and asserts every pipeline rate is within 2% of
the no-client run; kills the client mid-clip and asserts no rate change; stops a
publisher and asserts the STALE flag appears within 2 s. Prints the two rate
tables side by side.

---

# Part 2 — Deferred

Everything here is **not executable yet**, which is why it is not a phase. Each
entry names the **trigger** that would make it executable. When a trigger fires,
the entry is **deleted from this section** and appended to Part 1 as the next
unused phase number, with a test — see [../README.md](../README.md).

"Later" is not a trigger. If an entry's trigger is not something that can be
observed happening, it is not written down properly yet.

---

## Loop closure, pose graph, and volume rebuild

**What.** Always-on loop detection against the keyframe store, a pose-graph
backend owning `map → odom`, and a TSDF rebuild from frame memory when the
optimised trajectory moves the frames. The predecessor has all three, with
numbers to compare against: 2.3 cm / 0.85° on its loop bag, fr1/desk ATE
0.163 → 0.089 m, paired-surface gap 7.8 → 5.7 cm.

**Why not now.** It needs a keyframe store that exists (P7) and a surface worth
correcting (P6), and it is a plan of its own — four phases at least, with its own
gates and its own reference bag.

**Trigger.** P7 done **and** `bash tools/gates/odom.sh` showing drift over the 60 s clip
that a closure could remove. At that point it becomes its own plan issue, not a
phase appended here.

---

## CUDA kernels for TSDF integration

**What.** Move the frustum integration loop onto the GPU. It is embarrassingly
parallel and is the stage most likely to become the bottleneck once fusion runs
at full rate.

**Why not now.** There is **no CUDA toolkit installed** — `nvcc` is absent and
there is no `libcudart` in `/usr/lib`. And optimising a CPU integrator that has
never been profiled is guessing.

**Trigger.** `bash tools/gates/fusion.sh` reporting integrate cost **above 20 ms** at
13 Hz, or the profile showing integration above 30% of the frame budget. Either
one makes it real work; until then the CPU version is fast enough by
measurement, not by hope.

---

## TensorRT execution provider, and smaller depth input

**What.** Two independent levers on depth cost: a cached TensorRT engine instead
of the CUDA provider, and 392² input instead of 518² (roughly halves the cost,
coarsens thin structure).

**Why not now.** Both are optimisations of a stage that does not run yet, and
the GPU is Turing without tensor cores, so the usual fp16 argument does not
apply here — the win has to be measured, not assumed. A TensorRT engine is also
a long build and a per-machine cache, which is real operational cost.

**Trigger.** `bash tools/gates/depth.sh` passing (so there is a baseline) **and** the
pipeline needing more than ~13 Hz for a reason that has been written down. Take
the input-size lever first: it is a parameter change, and it can be measured in
an afternoon.

---

## Relocalisation from a saved room map

**What.** Persist the keyframe store to disk, load it at startup, and recover an
absolute pose against a room seen in an earlier session.

**Why not now.** The store lands in P7 because it is cheap to build alongside the
6-DoF work, but nothing consumes it until there is a backend that can act on a
recovered pose.

**Trigger.** The loop-closure plan above existing as a plan. Relocalisation is a
phase in that plan, not a phase in this one.

---

## Building OpenCV with CUDA

**What.** A source build of OpenCV with the CUDA module, so `cv::cuda::` links
and ORB could run on the GPU.

**Why not now.** The apt OpenCV (4.10.0) has no CUDA module, and ORB at 500
features is budgeted at ~5 ms on the CPU — well under the depth stage's 76 ms. A
source build of OpenCV is a large, self-inflicted maintenance burden for a stage
that is not the bottleneck.

**Trigger.** `bash tools/gates/keypoints.sh` reporting ORB cost **above 15 ms/frame**, or a
feature count above ~2000 becoming necessary for matching quality. Anything less
is not worth the build.

---

## Retired

Nothing yet. When an entry's trigger can no longer fire — the hardware went
away, the approach was superseded — delete it from above and record one line
here saying why, so the reasoning survives.
