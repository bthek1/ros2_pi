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
| A | [#4](https://github.com/bthek1/ros2_pi/issues/4) | P0–P1 | ✓ **closed 2026-09-09** — one source tree builds under both distros; the Pi ships stamped MJPEG |
| B | [#5](https://github.com/bthek1/ros2_pi/issues/5) | P2–P3 | One reader, one decode, corners on it, and `bags/desk1` exists — ✓ **done 2026-09-13** |
| C | [#6](https://github.com/bthek1/ros2_pi/issues/6) | P4 | Depth on the GPU at ≤ 80 ms, CUDA provider named in the log — ✓ **done 2026-09-15**, 55.10 ms mean |
| D | [#7](https://github.com/bthek1/ros2_pi/issues/7) | P5–P6 | A triangle mesh you can recognise your room in |
| E | [#8](https://github.com/bthek1/ros2_pi/issues/8) | P7–P8 | Translation is visible, and one tab shows the pipeline |

Each issue also carries a **`just view-*` RViz recipe** — a viewer for a person,
never the evidence. The gates below are what pass or fail a phase. The view
layer adds no new topics, so it adds no scope to any phase.

**Machinery every phase from P2 on extends rather than reinvents**, all of it
built in A and all of it failing a script when skipped:

- the new `.rviz` goes into `tools/gates/view-configs.sh`'s reach (it globs
  `src/**/*.rviz`, so this is automatic — but its topics must be ones `src/`
  publishes);
- the new `view-*` recipe goes into `RECIPES` in `tools/gates/hello-clean.sh`,
  or its teardown is untested — that gate found `view-camera` leaking RViz and
  the Pi's camera the day it was added to the list;
- `MIN_TESTS` in `tools/gates/test.sh` goes up when unit tests are added;
- the gate's measuring instrument is written in **C++**, the way
  `pimesh_camera/capture_probe` is — `ros2 topic hz` is Python over
  megabyte-class messages and its own scheduling lands in the number;
- anything on the Pi goes through `pi_run_for`, which puts `timeout` inside the
  login shell rather than orphaning the node;
- **no tight bound on a number measured across the two machines** — P1's stamp
  assertion had to move to the Pi because a cross-host figure carries the NTP
  relationship, which moved +8 ms to −19 ms in an afternoon.

---

# Part 1 — The build order

## ✓ P0 — Workspace skeleton

**Done 2026-09-09.** `bash tools/gates/build.sh` printed: `mode: scratch`,
`distros: dev=lyrical pi=jazzy`, `build times: dev=10.2s pi=32.0s`,
`packages: pimesh_bringup pimesh_camera pimesh_hello pimesh_msgs` identical on
both, `interfaces equal: 5 of 5`. The gate cleans both colcon trees and rebuilds
from scratch by default; `--incremental` skips that.

Two things came out of it worth carrying forward. `tools/gates/hello-build.sh`
asserted the package list was *exactly* `[pimesh_hello]`, so the first new
package failed it — it now reads the expected set from `src/` instead. And
`tf2_ros/static_transform_publisher` cannot be configured with
`parameters=[...]`: it declares `frame_id` and `translation.x` as parameters but
its `main()` parses `argv` first and exits non-zero with "Frame id must not be
empty" before reading any of them. The launch file therefore loads
`config/pimesh.yaml` itself and converts the entries to flags, keeping one
source of truth. Getting this wrong produces three dead processes and a launch
that carries on with no TF tree.

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

## ✓ P1 — Capture on the Pi

**Done 2026-09-09.** `bash tools/gates/capture.sh` printed, on its final run:

```
exposure mode    : Aperture Priority Mode  (exposure_dynamic_framerate=0)
rate             : 44.33 Hz on the dev box over 30s  (assert >= 40)
distinct frames  : 1331 of 1331  (assert equal)
mean JPEG        : 76152 bytes  (3.2 MB/s over wlan0)
subscribers      : 1  (assert 1; RViz closed)
stamp, one clock : 4.207 ms measured on the Pi at 59.33 Hz
                   assert |offset| <= one frame interval = 22.56 ms
stamp, cross-host : launch1=-13.510 ms  launch2=-12.518 ms
launch delta     : 0.992 ms  (assert < 5.0 — this is the usb_cam bug)
busy device      : exit 1, node refused in 0.24s  (assert non-zero, < 2.0s)
```

Across five runs: dev-box rate 44.3–58.6 Hz (Pi-side a steady 59.3 Hz),
0 duplicate payloads every time, launch delta 0.30–1.02 ms.

**The test as specified had to be corrected, and the correction is the
interesting part.** It asked for the stamp-vs-receipt offset to sit "within one
frame interval", measured on the dev box. That number cannot carry the claim: it
contains the two machines' NTP relationship, which measured +8 ms and −19 ms an
hour apart with the node unchanged, and one frame interval is ~18 ms. So the
tight assertion is now made **on the Pi**, where the stamp and the receipt come
from one clock and the residue is the real thing P1 claims — 4.21 ms, repeatable
to 0.02 ms across runs. The dev-box offsets keep a loose ±100 ms bound, which is
still far below every usb_cam sample (223/362/979 ms) and far above clock skew.
The two-launch delta, which is the actual usb_cam detector, was always sound and
is unchanged.

A first attempt to print the clock skew alongside was removed rather than fixed:
bracketing `ssh pi date` between two local reads returns roughly +RTT/2 whatever
the truth, and reported 195–430 ms of skew while explaining offsets of −15 ms.
A confident, contradictory number in a gate's output is worse than no number.

**The view layer landed with it** — `just view-camera`,
`src/pimesh_bringup/rviz/camera.rviz`, and `bash tools/gates/view-configs.sh`,
which parses every committed `.rviz` and asserts each display's topic is one
`src/` actually publishes (verified by breaking it: a renamed topic exits 1).
Extending `tools/gates/hello-clean.sh` to signal `view-camera` immediately
caught that recipe leaking both RViz and the Pi's camera_node: bash will not run
a trap while a foreground child is running, and an rviz2 signalled during its
own startup never exits, so the trap that cleans up the Pi never fired. The
viewer is now backgrounded and waited on. The same change found the gate
deducing its process group from `$!`, which is empty whenever `setsid` forks —
a group kill that had been silently doing nothing.

**Goal:** frames off the sensor with honest timestamps, and nothing else.

**Work**

- `pimesh_camera/camera_node`: V4L2, `V4L2_PIX_FMT_MJPEG`, 1280×720, `mmap`
  buffer pool, publish `CompressedImage` verbatim plus transient-local
  `CameraInfo`.
- Stamp from the dequeued buffer's own timestamp, not from `now()` at publish.
- Exit non-zero with a clear message on a missing or busy device — never idle.
- A `camera-reset` script that clears the C922's persistent V4L2 controls
  to a known baseline and prints every control current-vs-default. Built as
  `bash tools/camera-reset.sh`, not a `just` recipe — the justfile is the
  user-facing surface and this is a tool.

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

## ✓ P2 — The container, and proving intra-process

**Done 2026-09-12.** `bash tools/gates/ipc.sh` printed, on its final run:

```
decode_node.input_topic   : /image_raw/compressed  (assert the YAML key applied)
container processes       : 1  (assert exactly 1)
/image_raw/compressed subs: 1  (assert exactly 1 — the Wi-Fi constraint)
/image_raw subs           : 2  (assert >= 2 — the fan-out is the case that can fail)
first published buffer    : 0x7025f409d4f0
first buffer probed       : 0x7025f409d4f0
intra-process ON          : 325/325 addresses matched (assert all)
intra-process OFF         : 0/373 = 0% (assert <= 25%)
decode throughput         : stats in=47.0Hz out=46.2Hz dropped=4 failed=0 gaps=21
                            cost_mean=1.93ms cost_max=5.72ms
```

Earlier runs the same day: 504/504 against 0/395, and 429/429 against 0/389.
Decode costs **1.90–1.93 ms/frame** against a 4 ms budget.

**The zero-copy claim had to be earned twice, and the way it failed is the thing
to carry forward.** The gate passed at **429/429 with one consumer** — the probe
alone — and reported **0/574** on the very next run, with `keypoint_node` loaded
beside it. Same code, same flags, same container. rclcpp's intra-process manager
serves *ownership-taking* subscriptions by moving the buffer into the **last** one
and copying it for every other (`add_owned_msg_to_buffers`: "Copy the message
since we have additional subscriptions to serve"), so two `unique_ptr` consumers
means one of them gets a 2.7 MB copy, at 59 Hz, with nothing anywhere saying so.
Subscriptions taking a shared const pointer are served by
`add_shared_msg_to_buffers`, which hands one buffer to all of them however many
there are — so the consumers take `ConstSharedPtr`, and this project's standing
note that a `const &` callback "quietly copies" was the wrong way round.

**One consumer is the configuration that cannot fail**, which is why the gate now
asserts the decoded topic has **at least two** subscribers while it measures. A
gate that measures the easy case is a gate whose green means nothing the day a
stage is added — the same lesson as P0's teardown gate signalling only one of two
recipes.

Two other things changed under this phase, both found by it:

- **The container is `component_container_isolated`**, not `component_container_mt`,
  which is deprecated on Lyrical ("will be removed in M-turtle") and gives one
  shared thread pool where the isolated one gives each component its own executor.
  Both IPC gates were re-run to confirm the pointer handover is unaffected. The
  executor *behaviour* is unmeasured until there is a 76 ms callback to measure it
  with — see [milestone-b-future.md](milestone-b-future.md).
- **The workspace had been compiling with no optimisation flags at all.** colcon
  sets no `CMAKE_BUILD_TYPE` and nor did any of these packages, so every C++ cost
  this project had measured was a `-O0` number. `tools/build.sh` now passes
  `-DCMAKE_BUILD_TYPE=RelWithDebInfo`; P3's budget is what found it.

---

### P2 as specified, for the record

**Goal:** one network subscriber, one decode, zero copies downstream.

**Work**

- `pimesh_perception/decode_node`: subscribe `/image_raw/compressed`,
  `cv::imdecode`, publish `bgr8` intra-process. **Budget it for up to 59 Hz** —
  P1 measured 44.3–58.6 Hz arriving on the dev box at ~80 kB/frame, not the
  30 fps the older parts of these docs assume.
- **Add `use_intra_process_comms=True` to the bringup container.** The container
  itself exists and runs from P0, with an empty `composable_node_descriptions`;
  what P2 adds is the first `ComposableNode` in that list and the
  `extra_arguments` carrying the option. There was deliberately no
  `intra_process` launch argument before this phase, because
  `use_intra_process_comms` is a per-*component* option and a container with no
  components has nowhere to put it. It is an argument now, along with
  `log_payloads` and `probe`, because the gate needs all three — `ros2 launch` has
  no way to override one node's parameter from the command line.
- A temporary probe component that logs the address of the buffer it received.

**Test:** `bash tools/gates/ipc.sh` — asserts the probe's received-buffer address **equals**
the publisher's (a serialised path cannot produce that), and that
`ros2 topic info -v /image_raw/compressed` reports **exactly one** subscriber.
Prints both addresses and the subscriber count.

The single-subscriber assertion is the Wi-Fi constraint the whole architecture is
shaped around — see
[../../info/architecture.md](../../info/architecture.md#why-one-container).

---

## ✓ P3 — Keypoints and a recorded clip

**Done 2026-09-13.** `bags/desk1` is a 59.7 s hand-held sweep, 3489 frames at
58.5 Hz, 220 MB, `sha256 1333c5bd29a256157e590ef1e10c1753b7e170dab66051783683d77e4fccb3d0`
(`bags/` is git-ignored, so that hash is the clip's only identity). Every phase from
here replays it. `bash tools/gates/keypoints.sh` printed:

```
clip                : desk1  (220M, 59.7s, 3489 frames)
messages measured   : 3442 of 3489 = 98.7%  (assert >= 85%; 15 warm-up skipped)
frames with corners : 3265  (177 had none — the clip's texture, not the tracker)
sustained rate      : 57.93 Hz  (assert >= 30)
worst interval      : 24.86 ms at p95  (printed: a mean hides a stall)
keypoints per frame : 408.0  (cap is 500)
per-frame cost      : 5.99 ms  (assert <= 8.0, node's own clock)
  ... detection     : 4.04 ms
  ... matching      : 1.77 ms  (500 features against a 10-frame window)
  ... preview       : 2.19 ms  (not in the cost above: ~10 Hz, for a person)
frames dropped      : 0 in the last stats window  (by design: newest wins)
matched fraction    : 0.9063  (p05 0.8180; over frames that had corners)
  ... reference     : 0.9065 over 3297 frames, 408.4 kp, 25s to run
  ... difference    : 0.02 points  (assert <= 5.0)
pose gate           : reject rate 0.082, mean residual 0.0017 rad
descriptor bytes    : 32  (assert 32)
```

**Two independent implementations agreeing to 0.02 points** is the number worth
keeping: `tools/orb_reference.py` is the predecessor's pooled-window matching written
in Python from its description, sharing nothing with the C++ but OpenCV's ORB itself.

**The gate failed twice on a correct tracker first, and both failures were the
gate's.** It is the same lesson as P0's teardown gate and P2's single consumer —
what a measurement *does not* cover decides what its number means.

1. **It measured a 20 s window of a looping 60 s clip and compared it against the
   reference's whole-clip average**, and reported the two as **11.26 points** apart.
   A hand-held sweep is not uniform: the node matched 87% of its corners over the
   early part of `desk1` and far fewer in a stretch near the end, so *which seconds*
   each side averaged moved the answer further than a real regression would. Three
   windows that each covered something slightly different was not a tolerance
   problem, it was three different measurements being compared. The clip now plays
   **once, start to finish**, the probe's window is the clip's own duration from its
   metadata, and the gate asserts that **≥85% of the clip's frames reached the
   probe** — because a comparison only means something if both sides saw the same
   material.
2. **A frame with no features at all was averaged in as a zero by one side and
   skipped by the other.** Its matched fraction is 0/0 — undefined, not zero — and
   treating it as zero also conflates "no corners in this part of the room" with
   "corners found and none recognised", which are different failures and only the
   second is the tracker's. 177 of 3489 frames on this clip are in that category;
   the convention was worth ~5 points on its own. The node, the probe and the
   reference now all count the same way, and the count is printed rather than hidden.

A third, smaller one: the gate's first run failed with "/keypoint_node never
appeared" while the log beside it said `Loaded node '/keypoint_node'` — the `ros2`
daemon was serving a stale graph. It now waits on the container's own load
confirmation in the launch log, which is first-hand and needs no helper process.

---

### P3 as specified, for the record

**Goal:** repeatable corners, matched across frames, cheap.

**Work**

- `keypoint_node`: `cv::ORB` at 500 features, pooled matching over a 10-frame
  window, Hamming distance threshold 64, annotated preview on
  `/keypoints/image/compressed`.
- Rotation-only odometry with its gates: ≥ 8 matched pairs, mean ray residual
  < 0.03 rad, and **hold the last pose** rather than publish a guess when the
  gate fails. The node logs which regime it is in.
- **Record `bags/desk1`** — a 60 s hand-held sweep of the room,
  `bash tools/record-clip.sh desk1 60`. Every later phase replays it, so the numbers
  compare like for like. (Written as `just record` when this phase was drafted; the
  justfile is `build` + `run` only, so it is a script.)

**Test:** `bash tools/gates/keypoints.sh` — replays `bags/desk1` and asserts ≥ 30 Hz
sustained, mean per-frame cost ≤ 8 ms measured against the node's own clock
(never against `header.stamp`), and matched-keypoint fraction within 5 points of
the predecessor's on the same clip. Prints all three, plus the pose-gate reject
rate.

---

## ✓ P4 — Depth on the GPU *(done 2026-09-15, [#6](https://github.com/bthek1/ros2_pi/issues/6))*

**Goal:** metric depth at the rate the GPU can sustain.

> **Done 2026-09-15.** `bash tools/gates/depth.sh` replays `bags/desk1` through
> the real container and reports **`CUDAExecutionProvider`, 55.10 ms mean per
> frame / 58.21 ms p95 against an 80 ms budget**, 17.42 Hz sustained on `/depth`,
> **1048 of 1048** `/depth/rgb` frames byte-identical to the `/image_raw` frame
> with the same stamp (0 different, 0 unmatched), 1045 depth stamps matched to an
> input frame and 0 not, and 0 non-finite or out-of-range values in 966,625
> sampled distances. The control run — the same binary one parameter apart,
> `use_cuda:=false` — reports `CPUExecutionProvider` at **287.92 ms**, outside the
> same budget, which is what makes 80 ms an assertion rather than a number.
>
> The toolchain half was gated first and separately, as the phase demands:
> `bash tools/fetch-gpu-stack.sh` installs the stack rootless and sha256-pinned,
> and `bash tools/gates/gpu-stack.sh` reports `CUDAExecutionProvider` at
> **51.08 ms mean / 51.36 ms p95** for inference alone, with a CPU control at
> 181.95 ms, a default-linker-flags control that reaches only the CPU, and
> `nvidia-smi` witnessing the process holding a compute context.
>
> **And that gate could not see the bug this phase actually had.** It proves the
> GPU stack works for a program *we* link; a component loaded into somebody else's
> container is a different case, and it failed — `depth_node` ran at 517 ms/frame
> on the CPU inside `component_container_isolated` while `gpu_probe` ran at 51 ms
> on CUDA, same libraries, same flags, no error anywhere. See the constraint in
> [CLAUDE.md](../../../CLAUDE.md) and `preload_cuda_provider()` in
> `src/pimesh_perception/src/depth_engine_ort.cpp`. Ask what the gate does **not**
> touch.

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
  `camera_optical_frame`. Both are real as of P1: the input frame's stamp is the
  Pi kernel's capture time, and `camera_optical_frame` is a static edge
  published by `pimesh_bringup` and unit-tested to be the optical convention —
  do not re-derive that rotation inside a node.

**Test:** `bash tools/gates/depth.sh` — replays `bags/desk1`, asserts the startup log names
`CUDAExecutionProvider`, mean per-frame cost **≤ 80 ms** (the predecessor
measured 72–79 ms on this GPU), and that `/depth/rgb` is byte-identical to the
frame each depth map was inferred on. Prints mean, p95, and the provider.

A CPU fallback is a **failed** test however good the mesh looks.

**As built it asserts four more things, and three of them are about not fooling
itself.** The CPU control has to *fail* the same budget, or 80 ms is a threshold
nobody has watched exclude anything. `/depth/rgb` frames that could not be checked
are counted separately from frames that differed, because a run that checked
nothing would otherwise report zero mismatches and look perfect. Near and far have
to differ by at least 0.5 m, since a depth map at one distance everywhere is
exactly what a reciprocal taken on the wrong side of the clamp produces and it
renders as a convincing flat wall. And the scale is deliberately **not** asserted
on: monocular depth is scale-ambiguous and P5 is what pins it.

---

## ✓ P5 — Fusion *(done 2026-09-16, [#7](https://github.com/bthek1/ros2_pi/issues/7))*

**Goal:** a stream of posed depth maps becomes one consistent volume.

**Done.** `pimesh_world` joins the workspace with a spatially hashed TSDF at
15 mm voxels, 4 voxels of truncation and a weight threshold of 3, the
predecessor's high-pass scale aligner ported to C++, and `fusion_node` composed
into the same container as decode, keypoints and depth.

`bash tools/gates/fusion.sh` prints: **15.3 ms per integration** against a 20 ms
budget, **17.1 Hz** sustained, 1030 of 1030 frames offered actually integrated,
**0.19%** displaced in the mailbox, **0** without a pose at their own stamp, **0**
without their colour twin, worst arrival-to-integration p95 **13.1 ms** against a
57 ms frame interval, and `/pipeline/stats` carrying stage `fusion`.

**Four things were wrong before they were right and every one was invisible in
the output**, which is the part worth carrying forward:

1. **The ray-cast skipped the front of the truncation band.** A fixed coarse
   stride of three quarters of a block over unallocated space lands past a
   positive side only one truncation thick, so the first sample inside the band is
   already *behind* the surface and there is no sign change left to find. A panned
   camera reported no surface where a camera at the same place looking straight
   ahead found the wall. A slab test to the block's exit cannot skip a band.
2. **Two hash lookups per march step**, into a map big enough to miss cache on
   every one: the alignment ray-cast cost 23.9 ms at 35 000 blocks and 36.0 ms at
   145 000 — growing with the map rather than with the picture.
3. **12% of frames integrated colourless with nothing upstream wrong.** A
   single-threaded executor runs callbacks in subscription-*registration* order,
   not publication order, so the depth callback ran before the colour twin
   published immediately before it.
4. **A surface that moves further than a truncation leaves a ghost**, because only
   blocks this frame's band names are updated. Pinned as a test rather than left
   to be discovered.

**The paired-surface assertion does not hold as written, and the gate says so
rather than asserting it.** P5 asks it to show that alignment makes two views of
the same wall agree better. Measured: median surface gap 1.32 m aligned against
1.30 m unaligned, agreement fraction 0.114 against 0.131 — a coin flip. The
aligner is not broken (`test_scale_aligner` pins its properties, including that a
constant bias produces corrections whose product is exactly 1); it is correcting
the smaller error, because rotation-only odometry throws away ~0.9 m of real arm
arc. **P7 is the trigger** to re-measure — see
[milestone-d-future.md](milestone-d-future.md). What the control *does* assert is
that alignment reaches the node and does not inflate the map: 203 300 blocks with
it against 221 918 without.

**Still outstanding and it needs a person:** `depth_scale` is arbitrary at 10.0
until a tape measure pins it, and `bags/scale1` has not been recorded. Every
distance the pipeline reports is plausibly shaped and the wrong size. In
[milestone-d-future.md](milestone-d-future.md) with its trigger.

**Test:** `bash tools/gates/fusion.sh` — replays `bags/desk1` twice, once with
`align:=false` as the control.

---

## ✓ P6 — Surface *(done 2026-09-16, [#7](https://github.com/bthek1/ros2_pi/issues/7))*

**Goal:** a mesh out of the volume, without stalling the integrator.

**Done.** `mesh_node` marches cubes over a snapshot of the volume on its own
niced thread, prunes debris, fans interior holes shut while leaving every
component's frontier open, decimates to the Marker's cap by quadric error, and
writes the full-detail surface as a binary PLY.

`bash tools/gates/mesh.sh` prints: **790 668 triangles marched in 2.8 s**,
7 788 debris components dropped, **4 671 of 5 119 boundary loops filled** leaving
**448 open**, 340 595 edge collapses down to exactly **120 000** on `/world/mesh`,
a **779 740-triangle** PLY from `/world/save_mesh`, and three offscreen renders
whose emptiest is 9.4% surface.

**The no-dip claim is measured against a control, because P6's own wording is not
achievable.** It asks for `max inter-integration gap <= 2 x median`. The *input*
does not meet that: `bags/desk1` stalls ~400 ms about 35 s in — the seventh
five-second window of every run, at 13.6, 14.4 and 14.6 Hz — and it is there in
runs recorded *before `mesh_node` existed at all*, with `depth_node`'s own
per-frame cost unchanged at 55.9 ms throughout. A gate asserting the ratio would
have failed on it and pointed at the wrong node. So the second run sets
`remesh_period_s` past the clip length and nothing meshes: **374.7 ms worst gap
with meshing against 401.3 ms without it.**

**Three things this phase learned the hard way:**

1. **A use-after-move that segfaulted the container**, immediately after a
   re-mesh that had done every hard thing correctly — and the whole cleanup
   replayed offline on the very same mesh without a murmur, because the offline
   harness never published anything. `publish` moves from the `unique_ptr`; a
   later `stats->triangles` is a null dereference the compiler is happy with.
2. **A niced extraction thread is not optional.** At equal priority the mesher
   starves the pipeline: `depth_node`'s rate fell from 17.8 to 14.6 Hz with its
   per-frame cost unchanged — not more work, just not being scheduled.
3. **A latched topic needs a latched reader.** A volatile subscriber against a
   transient-local writer is *compatible*, gets nothing stored, and waits. The
   gate reported "nothing was published on /world/mesh" over a run that had
   published eight surfaces, and `rviz/mesh.rviz` needed the same fix.

**Test:** `bash tools/gates/mesh.sh` — replays `bags/desk1` twice, the second with
`remesh_period_s:=600.0` as the control, then runs `tools/mesh-views.sh` and
asserts the renders have a surface in them.

**What the renders do not show is a room.** Rotation-only odometry models a
hand-held sweep's arm arc as zero, so the same wall lands at a different distance
every time the camera moves and the surface comes out as a shell at roughly
constant radius. That is P7's to fix, and the gate says so in its own output.

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

## ☑ P9 — Calibrate the C922 *(promoted 2026-09-10, **done 2026-09-12**, [#9](https://github.com/bthek1/ros2_pi/issues/9))*

**Goal:** real intrinsics for this camera at 1280×720, so every unprojection
downstream is honest.

**Numbered P9, sequenced before P3.** Those are not in conflict — the number is
an identity that never moves, the order is a schedule. It arrived after P8
existed, so it takes the next unused number; it runs early because P3 is the
first phase to unproject pixels (bearing-ray odometry, gated on mean ray
residual) *and* the phase that records `bags/desk1`, which carries
`/camera_info` and is replayed by everything after it.

Promoted out of `milestone-a-future.md`, where its trigger was recorded as the
P5 tape-measure session. That trigger was wrong: P5's measurement needs a fusion
volume and genuinely cannot happen earlier, while calibration needs only a
checkerboard. Pairing them to save one trip would have cost three phases and
baked nominal intrinsics into the reference clip.

**Test:** `bash tools/gates/calibration.sh` — asserts `/camera_info` serves
non-zero distortion and a non-placeholder `K` with no `NOMINAL intrinsics`
warning; undistorts the held-out board frames in `calib/c922_720p/frames/` and
asserts board rows are straighter with the calibrated `K`/`D` than with the
nominal placeholder **and** under 1.0 px; asserts the recorded reprojection error
in `calib/c922_720p/report.txt` is ≤ 0.5 px and recomputes it live beside the
stored figure; and asserts the frames actually reached the frame corners. The
with/without control is required for the same reason `hello-ipc` needs one — a
board near the optical axis is nearly straight before any correction. The 1.0 px
figure is a budget, not something measured; record what the first run prints.

**Done 2026-09-12.** `bash tools/gates/calibration.sh` **PASSES**: fx=953.391,
fy=957.589, cx=627.678, cy=334.620, held-out reprojection **0.4955 px**, median
straightness **0.7796 px**, coverage 0.896 over 4/4 quadrants, 24 marker-confirmed
frames. `camera_node` serves it and the `NOMINAL intrinsics` warning is gone.

**The phase's own test was re-scoped to get there, and the reason is in
[#9](https://github.com/bthek1/ros2_pi/issues/9):** this camera has essentially no
lens distortion at 720p, so "straighter than the nominal placeholder" is not a claim
that can be made, and the worst-line-of-worst-frame statistic tightens as frames are
added. The budget now applies to the median frame with the worst held at a 2.0 px
backstop. `cameracalibrator` also does not run on Lyrical at all, so acquisition is
`tools/calibrate.sh record | select | solve` rather than the session in the body.

*(The paragraph below was written while it was still in progress and is kept as the
build log.)* The
loading path, the gate, the measurement instrument and its tests exist and are
measured; what is left is moving the camera in front of the board. Two things
came out of building it that changed the phase rather than just implementing it,
and both are recorded in [#9](https://github.com/bthek1/ros2_pi/issues/9):

- **The coverage floor is a fourth assertion.** Sweeping board pose against the
  uncalibrated deviation on a synthetic board: frames reaching ~49% of the way to
  the frame corner put the *uncalibrated* control at 0.52 px — inside the 1.0 px
  budget — so the gate as originally specified would have passed on nominal
  intrinsics while printing a flattering ratio. At ~98% the control is 1.4–2.1 px.
  "Cover the corners" is a precondition of the measurement, not advice, so the
  gate asserts it and `tools/calib_grab.py` refuses to stop until it is met.
- **Square-on views calibrate to nonsense, and straightness does not notice.** A
  board on a wall is rigid, which is the precaution the phase body asks for, and it
  still fails: sliding the camera parallel to the wall leaves every view square-on,
  focal length and distortion trade off, and the solve returns `fx=4840.8` against
  a true 905 with `k1=+1.97` against a true `+0.085`. Its **in-sample** reprojection
  error is 0.084 px — inside the 0.5 px budget — and it passes the straightness
  assertion at 0.333 px against a 1.116 px control. What catches it is the
  **held-out** reprojection error, 3.83 px against 0.068 px, plus a plausibility
  bound on `fx`. Both are now asserted, and this is the measured justification for
  grabbing the gate's frames before the session rather than reusing the
  calibrator's.
- **The two controls were one measurement.** `undistortPoints` with `P=K` and
  `D=0` cancels the two `K`s exactly and is the identity, so the nominal
  placeholder does not correct badly — it corrects nothing, whatever its `fx`
  says. The gate asserts that equality instead of printing one piece of evidence
  as two.

**The board is known and measured, 2026-09-12.**
`docs/charuco_a4_7x9_25mm.pdf` on a wall: 7×9 squares, 6×8 interior corners,
`DICT_4X4_250`, 18 mm markers. Its 100 mm scale bar **measures 99 mm**, so the print
is scaled 0.9900 and the real sizes are `--square 0.02475` and `--marker 0.01782` —
a 1% error that no software check can see, going straight into every distance the
pipeline reports. Two traps came with it, both in
[hardware.md](../../info/hardware.md#calibration-target): the command printed on the
sheet passes interior corners to `--size` where `-p charuco` wants squares (0 corners
interpolated against 42), and a marker-to-square ratio measured off camera frames
reads 7% low because a 32 px marker loses a pixel per side to blur.

**`bags/cam_2026_09_12` cannot calibrate anything, and says why.** 492 frames, board
found in 41 of 41 sampled — and the camera never moved: 0.5 mm of position spread,
0.24° of tilt spread, coverage 0.446 against the gate's 0.85 floor. Calibrating on it
alone gives `fx=817.7` and `d=[0.65, -3.68, 0.04, -0.005, 7.76]` at an in-sample
reprojection error of 0.66 px. The gate rejects it on coverage and reprojection —
but **not** on the `fx` bound, which 817.7 passes, nor on the fx/fy ratio at 1.040.
The coverage floor is what catches this one, which is the argument for having added it.

It also does **not** use `camera_info_manager`, which is what a ROS driver
normally loads a calibration with. That package is absent on the dev box under
Lyrical, so it would have made the Pi's package depend on an apt install on the
machine that never runs the camera; and `CameraInfoManager` advertises a
`set_camera_info` service from its constructor, which this node deliberately does
not offer. Reading the same standard YAML with `yaml-cpp` — present on both
machines already — costs ~150 lines and is unit-testable without a camera.

Full body, including the precondition that could make it non-executable (a
*rigid* board — a flexing printout converges happily and is wrong), in
[#9](https://github.com/bthek1/ros2_pi/issues/9).

---

# Part 2 — Deferred

Everything here is **not executable yet**, which is why it is not a phase. Each
entry names the **trigger** that would make it executable. When a trigger fires,
the entry is **deleted from this section** and appended to Part 1 as the next
unused phase number, with a test — see [../README.md](../README.md).

"Later" is not a trigger. If an entry's trigger is not something that can be
observed happening, it is not written down properly yet.

---

## Re-calibrate on a flat mount, if the scale turns out to matter

**What.** Redo P9 with the board mounted on something rigid *and flat* — foam board,
MDF, a clipboard — rather than taped to a wall, and re-fit.

**Why it is not a phase now.** P9's gate passes, and the thing wrong with the current
calibration is not visible to any assertion this project has. Two measurements say it
is imperfect: the printed target still has **~1.6 mm of bow**, and **`fx` is pinned
only to ±2.2%** — 906 to 973 across twelve random half-subsets of the same 24 frames
(2026-09-12). `fx` scales every distance the pipeline reports, so a 2.2% slack in it is
a 2.2% slack in every measurement downstream.

Doing it now would be guessing at whether that matters. Nothing built so far consumes
a metric scale: capture does not, and P2–P4 work in pixels, rays and relative depth.

**Trigger — P5's tape measure.** P5 reads fused depth at a surface a known distance
away, which is the first measurement in this project that can check a scale
independently of the calibration that produced it. **If P5's measured distance
disagrees with the tape by more than about 2%, that is this entry firing**, and the
fix is a flat mount and a re-run of `record | select | solve` before touching the
fusion code. If it agrees, the current calibration was good enough and this entry is
deleted.

**Also worth knowing when it fires.** `fx`'s spread across subsets is a cheap,
hardware-free check that the re-run improved things — it needs no tape measure, only
the frames — and it is not asserted by `gates/calibration.sh` today. Adding it would
be part of the same change.

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
