# The pipeline, stage by stage

*Design intent, 2026-09-01. Numbers marked **(measured)** come from the Python
predecessor `piros2` on this exact hardware and are strong priors for the C++
port; numbers marked **(target)** are budgets to hit, not results.*

RGB → keypoints → depth → fusion → mesh. One camera, no other sensor. Everything
this project knows about the room's shape is inferred from a single moving view.

## What one webcam can honestly do

Be clear about this before writing any code, because it sets what "working"
means:

- **Monocular depth is relative, not metric.** The network outputs an
  inverse-depth map with an unknown scale. One number turns it into metres
  (`z = depth_scale / output`), fixed by measuring one known distance. In the
  predecessor's room that constant was **2.69**, pinned by a tape measure
  against a door face at 2.50 m **(measured 2026-08-10)**. It is valid for that
  room's scale of scene, and re-deriving it here is a physical-world check a
  script cannot close.
- **The model's output wobbles ~4% frame to frame on a static scene
  (measured).** Naive integration of that wobble thickens every surface. The fix
  is per-frame scale alignment against the volume the camera has already built
  (below).
- **A single camera cannot recover scale from motion either.** Monocular
  odometry is scale-ambiguous by construction; the depth network's scale is what
  makes translation metric here, which is the reverse of the usual RGB-D setup.
- Far depth is a guess. Anything beyond ~6 m is clipped before it reaches the
  volume, and the 1/x inversion makes background values explode, so the clip is
  applied *before* the reciprocal.

## Stage 1 — Capture (`camera_node`, on the Pi) — **built 2026-09-09**

**Job:** get frames off the sensor, stamp them honestly, put them on the wire.
Nothing else. No decode, no re-encode, no processing.

This stage is built and measured. `bash tools/gates/capture.sh` is what closes
it; the numbers below marked **(measured here)** are that gate's output.

- Opens `/dev/video0` (the C922's only capture node — `/dev/video1` is its UVC
  metadata node), `V4L2_PIX_FMT_MJPEG`, 1280×720, requests 60 fps.
- `mmap` buffer pool, 4 buffers, `VIDIOC_DQBUF` → publishes the JPEG bytes
  verbatim as `CompressedImage` with `format: "jpeg"`. One copy in the whole
  path, kernel buffer to message.
- **Stamps at dequeue from the buffer's own timestamp.** `usb_cam` 0.8.1 has a
  once-per-process epoch bug that offsets every stamp by a random sub-second
  amount **(measured: 0.223 / 0.362 / 0.979 s on three launches)**. Fixing that
  is a large part of why this node exists rather than reusing `usb_cam`.

  The fix is to convert an *interval* rather than an epoch:
  `stamp = ros_now - (monotonic_now - buffer_monotonic)`. Both clock readings
  are taken microseconds apart on one machine, so their relative drift is
  irrelevant, and there is no per-process constant left to be wrong.
  **(measured here)** two launches of the node agreed on their stamp-to-receipt
  offset to within **0.30–1.02 ms** across five runs — against usb_cam, which
  redraws hundreds of milliseconds of it every launch. The node also checks the
  buffer's `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` flag rather than trusting the
  field, and falls back to stamping at dequeue with a warning if the driver is
  reporting some other clock.
- **Fails loudly.** A missing or busy device exits non-zero with a clear message
  naming the cause. **(measured here)** against a device held by another
  streaming process, the node refused in **0.24 s** with "another process is
  streaming /dev/video0. Find it with: fuser -v /dev/video0". `usb_cam` logs one
  ERROR and then idles forever, which looks like a working node publishing
  nothing.
- Publishes `/camera_info` transient-local, from
  `package://pimesh_bringup/config/camera_info/c922_720p.yaml` when that file
  exists and from **nominal** parameters with a startup WARNING when it does not.
  The file is the standard `camera_info` YAML, which is exactly what
  `cameracalibrator` writes, so it is copied in verbatim rather than transcribed
  into parameters. **Calibrated 2026-09-12** ([P9](https://github.com/bthek1/ros2_pi/issues/9),
  closed): fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection **0.4955 px**
  over 24 marker-confirmed frames, and the startup WARNING is gone. Produced with
  `bash tools/calibrate.sh record | select | solve` and checked by
  `bash tools/gates/calibration.sh`.

  Two results from that phase are worth carrying: **this camera has essentially no
  lens distortion at 720p** (so `D` is near zero and correctly so — see
  [hardware.md](hardware.md#this-camera-has-essentially-no-distortion-at-720p)), and
  **`fx` is pinned only to ±2.2%**, which is a ±2.2% slack in every distance this
  pipeline reports. P5's tape measure is the first thing that can check that
  independently.

  Three refusals are measured on the Pi (2026-09-12), and the asymmetry between
  them is deliberate. A file that is **absent** warns and carries on, because that
  is the normal state before anybody has run the board. A file that is **present
  and wrong** refuses to start, with no fallback to the nominal numbers: a
  1280×720 calibration against a 1080p stream (`refusing to start: calibrated at
  1920x1080 but capturing at 1280x720`), a non-`plumb_bob` model, and unparseable
  YAML all exit 1 by the same route a busy device does. Quietly substituting the
  placeholder for a broken calibration would be a green light over a wrong one.

  There is no `calibrated` parameter. There was until P9, and it was the wrong
  shape: it let a human assert the very claim the startup warning exists to
  police. The flag is now derived from whether a file loaded *and* carries
  non-zero distortion coefficients, so a placeholder full of zeros cannot switch
  the warning off.

**Latency (measured here):** **4.21 ms** median from the kernel dequeuing a
buffer to a subscriber holding the message built from it, measured *on the Pi*
so that the stamp and the receipt come from one clock. The equivalent figure
taken on the dev box is not a latency at all — it carries the two machines' NTP
relationship, which moved between +8 ms and −19 ms over one afternoon with the
node unchanged. Quote the single-clock number; the cross-host one only ever had
the job of catching an epoch error.

**Frame rate reality:** the C922 delivers 18–21 fps on stock settings because
`exposure_dynamic_framerate=1` trades rate for exposure indoors; with the
control cleared it delivers **42–60 distinct frames/s at true 720p MJPEG
(inherited, measured 2026-08-04)**. **(measured here, 2026-09-09)** with
`bash tools/camera-reset.sh` run first and the camera in Aperture Priority Mode:
**59.3 Hz at the Pi**, and **44.3–58.6 Hz as received on the dev box** across
five runs — 0 duplicate payloads in every run, so those are distinct frames.
The gap between the two is the Wi-Fi hop, and it is the thing to watch: never
quote a frame rate without saying which machine measured it and which exposure
mode it was under.

## Stage 2 — Decode (`decode_node`, dev box)

**Built 2026-09-12** — P2, [#5](https://github.com/bthek1/ros2_pi/issues/5).

The container's only network subscriber. `cv::imdecode` on the JPEG, publish
`bgr8` on `/image_raw` intra-process. Exists as its own component so that the
decode happens exactly once no matter how many consumers appear.

**Measured:** **1.90 ms/frame** mean against a 4 ms target
(`bash tools/gates/ipc.sh`), 46–53 Hz in and out with 0–4 frames dropped per 5 s
window, exactly **one** subscriber on `/image_raw/compressed`, and every decoded
buffer reaching its consumers at the address it was published from — **504/504**
with intra-process comms on, **0/395** with it off.

The decode runs on its own thread behind a one-slot mailbox, which is the shape
every expensive stage here uses: the subscription callback does nothing but move
the pointer into the slot, and the slot keeps the newest frame and counts what it
displaced. Dropping is the design — at 59 Hz in and 13 Hz out (which is what P4
will be), a queue grows by 46 frames a second until the process dies.

**The cost that is not in that 1.90 ms** is one pass over 2.7 MB: `imdecode`
writes into a reused buffer and the pixels are then copied into the message,
because publishing a `unique_ptr` means a fresh allocation per frame. It is the
copy that makes every *downstream* copy unnecessary.

**The reader's QoS is an open question, and the parameter exists so it can be
measured rather than argued about.** `input_reliability` is `reliable` by default.
The BEST_EFFORT-reader measurement that `rviz/camera.rviz` relies on was taken on
a *viewer*, where a dropped frame costs nothing; here a dropped frame is a frame
the mesh never sees. `decode_node`'s stats line counts inter-arrival gaps over
50 ms, which is the instrument for settling it.

## Stage 3 — Keypoints (`keypoint_node`, dev box)

**Built 2026-09-12** — P3, [#5](https://github.com/bthek1/ros2_pi/issues/5). The
rotation-only regime is built; the RGB-D regime below is P7.

**Job:** find repeatable corners, match them across frames, and turn the matches
into camera motion.

**Measured** (`bash tools/gates/keypoints.sh`, over all 3489 frames of
`bags/desk1`): **5.99 ms/frame** mean against an 8 ms budget, on the node's own
clock — **4.04 ms** of detection and **1.77 ms** of matching — sustaining
**57.9 Hz**, with a matched-keypoint fraction of **0.9063** against the
predecessor's algorithm at **0.9065** on the same frames. The annotated preview
costs a further **2.2 ms** and is capped at ~10 Hz, which is why it is accounted
separately: it is an output for a person, and folding it into the pipeline's cost
would make that number depend on whether anybody was watching.

**Two conventions make those two numbers comparable, and each was worth ~5
points.** A frame in which ORB finds *no* features has no matched fraction — 0/0 is
undefined, not zero — so such frames are counted (`empty=` in the node's stats line;
177 of 3489 on this clip) and excluded from the average, on both sides. And the
measurement covers the **whole clip**: a hand-held sweep is not uniform, so a 20 s
window of it answers a different question from the clip's average. The first version
of this gate compared a 20 s window against a whole-clip reference and reported an
11-point gap on a tracker that was working correctly.

**Those are optimised numbers**, and before 2026-09-12 they could not have been:
`tools/build.sh` set no `CMAKE_BUILD_TYPE`, so the workspace compiled with no
optimisation flags at all and the same path cost **7.90 ms** — a 1% margin against
the budget that was really 13%. Every C++ cost in this document now assumes
`-O2 -g`.

- `cv::ORB`, **500 features** — enough that ranking churn at the cap does not
  dominate, cheap enough at 30+ fps. 4.04 ms/frame **(measured)**, finding **408 on
  average** in this room rather than the full 500: the cap is a ceiling the scene does
  not always reach. It is not an exact ceiling either, since ORB distributes its quota
  per pyramid level and rounds up — asking for 120 returned 121.
- Match against a **pooled window of the last 10 frames**, not just the previous
  one: strict frame-to-frame matching loses ~25% of keypoints to detection
  flicker at the feature cap **(inherited)**. Reject matches whose Hamming
  distance exceeds 64 of 256 bits — a lookalike corner is worse than no corner.
- **Two odometry regimes, and be honest about which is running:**
  - *Rotation only* (bearing rays, no depth): robust, cheap, and **wrong the
    moment the camera translates**. A hand pan carries ~0.9 m of arm arc, which
    smears the mesh. Fine for a compass, not for a surface.
  - *RGB-D* (keypoints backed by `/depth`, 3D–3D fit): 6-DoF and what the mesh
    actually needs. This is the default for meshing.
- Gates before a pose is trusted: at least 8 matched pairs, and a mean ray
  residual under 0.03 rad (~1.7°, ~28 px at fx=953) after reject-worst refits.
  Failing the gate means *hold the last pose*, not publish a guess — and the node
  logs the regime change when it starts and stops holding. **Measured** on
  `bags/desk1`, a real hand-held sweep: an **8.2% reject rate** and a mean residual
  of **0.0017 rad** over the frames it accepted. (On a clip of a stationary camera it
  is 0.1% and 0.0000 rad, which says more about the clip than about the gate.)

  **The residual ceiling is not equally sharp in all three axes**, which is worth
  knowing before trusting it. A ray on the axis of rotation does not move at all,
  and ORB's corners sit within ~35° of the optical axis, so the same 0.05 rad of
  error shows up as 0.05 rad of residual in pan or tilt and about **0.019 rad** in
  roll (measured in `test_rotation_fit`). A rolled hand-held sweep is exactly where
  the gate is least sharp.
- Keep a **keyframe store** — descriptors, bearing rays and 3D landmarks, a new
  keyframe whenever the view direction is ~18° from every stored one or the
  camera has moved 0.3 m. It is what makes relocalisation and loop closure
  possible later, and it costs ~16 kB per keyframe.

`/keypoints` carries positions, descriptors and per-feature match ids so the
dashboard can draw tracks without recomputing anything.

**Matching is one-to-one, and it is not by default.** A nearest-neighbour search
is many-to-one: two corners in a frame can both name the same older feature as
their best match, and both would then inherit its track id — one track in two
places at once, which no consumer can detect and the geometry cannot express.
Candidates are taken in order of Hamming distance and each older track is claimed
once.

**Keep away from `cv::aruco` in this stage, and from any OpenCV API that differs
between 4.6 and 4.10.** `ORB::create`, `BFMatcher` and `knnMatch` are identical on
both; the calibration work of P9 has the long version of why that matters.

## Stage 4 — Depth (`depth_node`, dev box, GPU)

**Job:** one RGB frame in, one metric depth map out.

**Status, 2026-09-15: built and measured.** `depth_node` is a component in the
bringup container, publishing `/depth` and `/depth/rgb`, and
`bash tools/gates/depth.sh` is what closes the claim: replaying `bags/desk1`
through the real container it reports `CUDAExecutionProvider` at **55.10 ms mean
per frame, 58.21 ms p95** against an 80 ms budget and **17.42 Hz** sustained on
`/depth`, with a CPU control at 287.92 ms that fails the same budget.

- **Depth Anything V2 Small (ViT-S/14), ONNX.** RGB in, ImageNet-normalised,
  spatial dims a multiple of 14 — **518 = 37×14** is the trained size. ~99 MB of
  weights, fetched and checksummed, never committed.
- **ONNX Runtime C++ with the CUDA execution provider**, CPU as an explicit
  fallback that logs which provider it got. Do not let it silently land on CPU
  and then wonder why the mesh stopped updating.
- **Measured in C++ on this GPU, 2026-09-15**, at two levels, and the gap between
  them is the interesting part. Inference alone
  (`bash tools/gates/gpu-stack.sh`, ONNX Runtime 1.30 + CUDA 13.1 + cuDNN 9.26):
  **51.08 ms mean, 51.36 ms p95** on CUDA against **181.95 ms** on the CPU
  provider, with a 217 ms first inference. The whole per-frame cost inside
  `depth_node` (`bash tools/gates/depth.sh`, the node's own clock): **55.10 ms
  mean, 58.21 ms p95**, against **287.92 ms** for the CPU control. So preprocessing,
  the reciprocal, the resize back to 1280×720 and two publishes together cost
  **~4 ms**, and the node sustains **17.42 Hz** on a 59 Hz input — it sees roughly
  one frame in three and drops the rest. The predecessor's Python path measured
  72–79 ms / 280–305 ms for the same model on the same card; different runtime,
  different CUDA, same ratio. **Warm the session at startup** so the first real
  frame is not the cold one — `depth_node` does, in its constructor.
- **Link it with `-Wl,--disable-new-dtags`, and that is only half of it.** The
  CUDA provider is dlopened and has no search path of its own, and `DT_RUNPATH` is
  not inherited down a dlopen chain — so with CMake's default flags ONNX Runtime
  silently falls back to the CPU and the only symptom is a pipeline four times
  slower than it should be. **The flag rescues an executable and not a component**:
  glibc consults the *main executable's* `DT_RPATH` for a dlopened object's
  dependencies, and `component_container_isolated` is not ours. `depth_node`
  therefore loads the CUDA libraries itself, by absolute path, before ONNX Runtime
  asks for them — `preload_cuda_provider()` in `depth_engine_ort.cpp`. See
  [setup.md](setup.md#gpu) and the constraint in `CLAUDE.md`.
- Convert to metres with `depth_scale`, clip beyond 6 m *before* the reciprocal,
  and publish `32FC1` with the **input frame's** stamp and `camera_optical_frame`
  — derived data keeps the header of what it describes, not the moment inference
  finished. The clip is in *inverse* space on purpose: the model's output tends to
  zero on anything it reads as "no idea", and `1/0` is not a large number, it is
  `inf`. Clamping the result afterwards is one branch too late, because anything
  that touched the infinity first is already NaN. Asserted by the gate: 0
  non-finite and 0 out-of-range values in 966,625 sampled distances.
- Publish `/depth/rgb`, the exact frame inferred on, so fusion gets a true
  RGB-D pair — **1048 of 1048 measured byte-identical** to the `/image_raw` frame
  with the same stamp. A separate republisher cannot serve this purpose: it would
  drop *different* frames from `depth_node`, so the two stamp sets would rarely
  intersect and an exact-sync consumer would limp at a fraction of either rate.
- **Publish `/depth/image/compressed`, an inferno preview at ~10 Hz**, on a
  **fixed** `[0, max_range]` scale rather than a per-frame one, inverted so near is
  bright and the clip is black. 32FC1 metres render as near-black in any viewer
  that does not know what they are, and RViz's Image display has no colour map —
  only `Normalize Range`, which rescales each frame to its own extremes and makes
  the same distance a different shade from frame to frame. Its cost sits *outside*
  the per-frame budget, as `keypoint_node`'s preview does: folding a JPEG encode
  drawn for a person into the number that decides whether this stage keeps up would
  make the 80 ms assertion partly a claim about a viewer. Measured: counting it
  moved `cost_mean` from 55 ms to 59 ms and a window's p95 over the ceiling, with
  nothing about the pipeline changed.
- **`depth_scale` is arbitrary until P5.** Monocular depth is scale-ambiguous —
  the model says "twice as far", never "three metres" — so the room comes out
  plausibly shaped and the wrong size. The predecessor's was 2.69× out.
  `gates/depth.sh` deliberately asserts nothing about the absolute values, only
  that near and far differ.

**Optimisations, in the order worth trying:** none of them yet — at 55 ms against
an 80 ms budget there is nothing to buy. When there is: input at 392² instead of 518² (roughly halves the
cost, coarsens thin structure), then TensorRT EP with a cached engine (a real
speedup but a long build and a per-machine cache), then fp16 — which on Turing
without tensor cores buys bandwidth only, so measure before believing.

## Stage 5 — Fusion (`fusion_node`, dev box)

**Job:** turn a stream of posed depth maps into one consistent volume.

- **A truncated signed distance field** in a spatially hashed voxel grid —
  1.5 cm voxels, truncation ~4 voxels, per-voxel weight and colour. Hashing, not
  a dense grid: a dense 6 m cube at 1.5 cm is 64 M voxels, and a room is mostly
  empty.
- Integrate every posed frame: for each voxel in the camera frustum, project,
  read depth, update `(d, w, rgb)` with a weighted average and a weight cap so
  late observations still move the surface.
- **Per-frame scale alignment, and this is the load-bearing trick.** The depth
  model wobbles ±4%, so before integrating, ray-cast the existing volume from the
  frame's own pose and scale the frame by the median ratio against it. The first
  frame defines the map's scale. Guards: skip alignment when valid overlap is
  under 20% (a mostly-new view has nothing to conform to), and refuse a
  correction beyond 15% (that is a bad pose or a depth failure, not a wobble —
  do not fold the map around it) **(inherited, all three thresholds)**.
- Voxels observed fewer than 3 times do not mesh — that is the noise floor.
- Keep the last ~500 integrated frames at half depth resolution (~250 MB) so the
  volume can be **rebuilt** when a pose-graph correction moves the trajectory.
  Without that, a loop closure corrects the poses and leaves the surface wrong.

**Cost target:** ~15 ms per integration at 13 Hz, single-threaded CPU with the
frustum loop tiled over voxel blocks. This is the stage most worth moving to
CUDA later — it is embarrassingly parallel — but only after the CPU version is
correct and measured, and only once a CUDA toolkit is actually installed.

## Stage 6 — Surface (`mesh_node`, dev box)

**Job:** extract a triangle mesh from the volume and hand it to the viewers.

- **Marching cubes** over the allocated voxel blocks, vertex colours interpolated
  from the volume, every ~10 s on a snapshot copy — never holding the
  integrator's lock.
- **Clean up before publishing:** drop connected components under ~30 triangles
  (the predecessor's census found ~370 noise flakes on one static scan,
  masquerading as holes), then close interior boundary loops smaller than 0.25 m
  by fan-filling from the ring. **Each component's largest loop is its frontier
  and stays open — unseen space is never invented.**
- Two outputs with different rules:
  - `/world/mesh` as a `Marker` **capped at ~120 k triangles by quadric
    decimation** — never by dropping triangles, which peppers the surface with
    pinholes.
  - `/world/save_mesh` writes the **full-detail PLY**, plus optionally a
    Poisson-closed watertight companion for downstream tools. The honest,
    hole-bearing mesh is always written; the closed one is clearly labelled as
    containing assumed geometry.

**Cost:** 300–900 ms per extraction depending on volume size. Off the hot path
by construction.

## Stage 7 — Dashboard (`dashboard_node`, dev box)

See [dashboard.md](dashboard.md).

## Data flow summary

| From | To | Payload | Rate |
| --- | --- | --- | --- |
| Pi | dev box | JPEG, ~100–200 kB | up to 60 Hz, ~2–12 MB/s |
| `decode` | `keypoint`, `depth` | `cv::Mat` pointer | intra-process, zero copy |
| `depth` | `fusion` | 3.7 MB float depth + its RGB frame | **17.4 Hz measured**, intra-process |
| `fusion` | `mesh` | TSDF snapshot | every ~10 s |
| `mesh` | RViz, dashboard | ~120 k triangles | every ~10 s |
| all | dashboard | stats | 10 Hz |
