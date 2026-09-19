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

**Built 2026-09-12** — P3, [#5](https://github.com/bthek1/ros2_pi/issues/5).
**6-DoF added 2026-09-19** — P7, [#8](https://github.com/bthek1/ros2_pi/issues/8).
Both regimes are built and `odometry: sixdof` is the default; `rotation_only` is
P3's estimator, kept as the control `tools/gates/odom.sh` measures against.

**Job:** find repeatable corners, match them across frames, and turn the matches
into camera motion.

**Measured** (`bash tools/gates/keypoints.sh`, over all 3489 frames of
`bags/desk1`): **6.54 ms/frame** mean against an 8 ms budget, on the node's own
clock — **4.02 ms** of detection and **2.39 ms** of matching — sustaining
**57.8 Hz**, re-measured 2026-09-19 with P7's pose worker in the same process
(P3 measured 5.99 ms with nothing but decode beside it), with a matched-keypoint fraction of **0.9063** against the
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
- **Two odometry regimes, and the node states which is running at startup.**
  - *Rotation only* (bearing rays, no depth): robust, cheap, and **silent about
    translation** — it publishes zero, which is the honest scope of what rays can
    support. Published at the camera's rate, ~58 Hz, at each frame's own stamp.
  - *`sixdof`* (the default): each depth frame's corners are paired with the
    **newest keyframe's** 3D landmarks by track id, and the pose is solved with
    `cv::solvePnPRansac` — the keyframe's landmarks against *this* frame's pixels.
    Published at the depth rate, ~17.5 Hz, at each depth frame's own stamp, which
    is exactly the set of stamps `fusion_node` looks up.

    **Measured** (`bash tools/gates/odom.sh`, whole clip, 2026-09-19): **1.421 px**
    mean inlier reprojection over **97 inliers**, **79.9%** of depth frames posed
    rather than holding, 1043 poses at **17.47 Hz**, a **31.1 m** path with
    **4.87 m** of net displacement in the map's arbitrary units, and 22 keyframes
    at 837 kB.

  **Three things about that design were arrived at by measurement and each is a
  correction of the obvious answer**, which is why they are written down rather
  than left in the code:

  1. **Not a rigid fit between two unprojected clouds**, which is what P7's plan
     described. Both clouds carry the depth network's error, and it is
     *structured* — a smooth warp, not per-pixel noise — so it does not average
     down over three hundred landmarks; and the network's overall scale breathes a
     few percent a frame, which a rigid fit can only absorb as translation along
     the view axis. Measured in order on `bags/desk1`: rigid frame-to-frame
     reported an **89.5 m** path over a 45 s desk sweep; dividing the scale out
     left it at 117 m; measuring against a keyframe brought it to 44 m; a low-pass
     brought it to 8.4 m. PnP changes the *measurement* rather than the filtering
     — the current frame contributes only pixels, so one depth map is involved
     instead of two and there is no scale ratio between them.
  2. **Not against the previous frame.** The pose is set **absolutely** from the
     keyframe's rather than accumulated onto the last one, so nothing chains and
     per-step error does not integrate. Simulated over a 35 s sweep: final
     position error **0.106 m** against **2.70 m** frame-to-frame, same noise and
     same estimator.
  3. **A plausibility gate on the motion, which a residual cannot supply.** PnP
     reports how well its pose explains the pixels it was given and has no opinion
     about whether the 3D points behind them are where the depth network said.
     Measured: a single step of **9.4 m** between two depth frames at a mean inlier
     reprojection of **1.23 px** — confident and wrong — which then became the
     reference every later frame was measured from. `max_speed_m_s` refuses it; on
     the reference clip it fires on 96 of 1043 poses and the fastest published
     motion comes out at 1.9986 m/s against its 2.0 ceiling.

  **`camera_step()` is the other half of P7 and is worth more than the 6-DoF work
  on this clip.** A fit answers `P_cur = M * P_prev` for a point seen twice; the
  camera composes with `M⁻¹`, because the point did not move. From P3 until
  2026-09-19 `update_pose()` composed `M` itself, so the published `odom → base_link`
  turned **left** when the camera panned right. Nothing failed: a frame that moves
  when you pan looks correct in RViz, the residual gate is indifferent to the sign,
  and a TSDF built from consistently mirrored poses still produces a surface.
  Measured on `bags/desk1` with the old composition restored, same binary
  otherwise — median paired-surface gap **1.3440 m** against **0.4456 m**, agreement
  **0.1057** against **0.2087** — and the first of those reproduces what milestone D
  recorded, 1.32–1.37 m at 0.115, which is what ties the number to the bug rather
  than to the afternoon. **3× on the surface, for one transpose.**
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
  keyframe at ~18° of view change or 0.3 m of motion. P7's plan said nothing would
  read it yet; the odometry above reads the newest entry as the view each frame is
  posed against, because frame-to-frame chaining measured as a random walk. What
  is still unbuilt is the *other* reader — matching against **every** keyframe to
  recognise a place seen minutes ago — which is the deferred loop-closure work.

  **It costs ~37 kB each, not the ~16 kB P7 budgeted**, and the difference is
  recorded rather than designed away: 500 32-byte descriptors *are* 16 kB, and the
  three geometric arrays beside them — a bearing ray per feature, a landmark for
  the half with a usable depth reading, and the row index tying them together —
  are the rest. At the 500-keyframe ceiling that is ~18 MB. Measured on the
  reference clip: 22 keyframes, 837 kB.

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

## Stage 5 — Fusion (`fusion_node`, dev box) — **built 2026-09-16**

**Job:** turn a stream of posed depth maps into one consistent volume. This is
where the pipeline stops being a stream and starts remembering.

**Measured** with `bash tools/gates/fusion.sh` on `bags/desk1`: **15.3 ms per
integration** against a 20 ms budget, **17.1 Hz** sustained, 1030 of 1030 frames
offered actually integrated, **0.19%** displaced in the mailbox, **0** frames
without a pose at their own stamp and **0** without their colour twin.

- **A truncated signed distance field** in a spatially hashed voxel grid —
  1.5 cm voxels, truncation 4 voxels, per-voxel weight and colour, 8³ blocks of
  6 kB each. Hashing, not a dense grid: a dense 6 m cube at 1.5 cm is 64 M voxels
  and 768 MB before a frame arrives, for a room that is almost entirely air.
- Integrate every posed frame: allocate the blocks a band around each ray
  touches (every 8th pixel, which at fx=953 and 6 m is 5 cm between samples and
  so misses nothing inside a 12 cm block), then update every voxel of those
  blocks against the full-resolution depth map. **The signed distance is
  projective and uses z, not ray length** — the two agree at the principal point
  and differ by 30% in the frame corners, which bends every wall into a bowl that
  looks like lens distortion.
- **Per-frame scale alignment** — ray-cast the volume from the frame's own pose,
  take the median ratio against the incoming depth, and correct only the
  *deviation* from a rolling median of recent ratios. The first frame defines the
  map's scale. Guards: no alignment under 20% valid overlap, no correction beyond
  15%. See `scale_aligner.hpp` for why it is a high-pass and what happened to the
  predecessor when it was not.

  **It makes no measurable difference on `bags/desk1`, and that is the phase's
  most useful finding.** Aligned against unaligned, the median surface gap came
  out 1.32 m against 1.30 m and the fraction of each frame the map already agreed
  with to 5% came out 0.114 against 0.131 — a coin flip, window by window. The
  aligner is not broken; `test_scale_aligner` pins its properties, including the
  one that matters most (a constant bias produces corrections whose product is
  exactly 1, so it never pushes the map). It is correcting the smaller error:
  `keypoint_node` published **rotation only** until P7, and a hand-held sweep's ~0.9 m of
  unmodelled arm arc is a 30-45% geometric error at 2-3 m against a scale wobble
  clamped at 15%. What is measured is that it does not make things worse:
  203 300 blocks with it against 221 918 without.

  **P7 was named as the trigger, it fired, and the comparison moved.** Re-measured
  2026-09-19 with P7's corrected rotation and 6-DoF translation in place, the same
  gate reports **0.4805 m aligned against 0.5330 m unaligned** and agreement
  **0.2191 against 0.1606** — the aligner ahead on both for the first time, where
  before it was a coin flip with the winner alternating window by window. That is
  consistent with the diagnosis above: the aligner was correcting the smaller error
  while a mis-composed pose supplied the larger one, and with the pose fixed its
  correction is visible.

  **It is still printed rather than asserted**, because it is one run of a
  measurement that has already flipped once, and a gate that asserts on a number
  which alternates is a flaky gate. Promoting it needs the result to repeat across
  runs; that is the entry in
  [milestone-d-future.md](../plans/future/milestone-d-future.md).
- Voxels observed fewer than 3 times do not ray-cast — that is the noise floor.
- **The map is bounded at 300 000 blocks (~1.9 GB) and `bags/desk1` reaches
  200 000 of them.** That is a symptom, not a resolution set too fine: 200 000
  blocks is over 2000 m² of surface for a room with perhaps 60 m² in it, which is
  thirty layers of the same wall. Two causes were named when that was written —
  rotation-only odometry and an unpinned `depth_scale` — and P7 settled the first:
  with 6-DoF poses and the composition corrected the clip still fills 200 000-plus
  blocks, so the layering is not the pose. The ceiling is what stops a session
  dying of memory while the rest is outstanding; it fixes neither.
- **A surface that moves further than one truncation leaves a ghost**, because
  only the blocks this frame's band names are updated. That is how every
  voxel-hashing integrator behaves and it is the price of not walking the whole
  frustum every frame — `test_tsdf_volume` pins it as a test rather than leaving
  it to be discovered.

**Not built:** the frame memory for volume rebuild after a loop closure. Without
it a pose-graph correction moves the poses and leaves the surface where it was.
See [the milestone D future file](../plans/future/milestone-d-future.md).

## Stage 6 — Surface (`mesh_node`, dev box) — **built 2026-09-16**

**Job:** extract a triangle mesh from the volume and hand it to the viewers,
without stalling the integrator.

**Measured** with `bash tools/gates/mesh.sh` on `bags/desk1`: **790 668 triangles
marched in 2.8 s**, decimated to exactly **120 000** for the Marker, boundary
loops **5119 → 448**, and the worst gap between two integrations **374.7 ms with
meshing against 401.3 ms in a control run with nothing meshing** — no dip.

- **Marching cubes** over the allocated blocks, vertex colours interpolated from
  the volume, every 10 s. A cell whose eight corners are not *all* above the
  meshing weight is skipped entirely rather than meshed with the missing corners
  read as zero — zero is the iso-value, so believing it seals the room inside a
  shell of invented walls.
- **On a snapshot copy, taken in chunks.** One lock over the whole map is not a
  short lock at 1.25 GB; chunked at 2048 blocks it is a few milliseconds at a
  time and the integrator interleaves with it. The copy is also **filtered by the
  meshing weight**, which changes nothing about the surface (a block nothing has
  reached that weight in has no corner the mesher would believe) and cuts it from
  200 000 blocks to 37 000.
- **The extraction thread is niced.** At equal priority it starves the pipeline:
  measured, while an extraction ran, `depth_node`'s rate fell from 17.8 to
  14.6 Hz with its per-frame cost unchanged at 55.9 ms — not more work, just not
  being scheduled.
- **Clean up before publishing:** drop connected components under 30 triangles,
  then close interior boundary loops smaller than 0.25 m by fan-filling from the
  ring. **Each component's largest loop is its frontier and stays open — unseen
  space is never invented**, and a sealed box looks *more* finished than a
  correct scan, which is why it is a unit test rather than something anyone would
  notice.
- Two outputs with different rules:
  - `/world/mesh` as a latched `Marker` **capped at 120 k triangles by quadric
    decimation** — never by dropping triangles, which peppers the surface with
    pinholes. Latched because the surface changes every ten seconds and a viewer
    started between two extractions would otherwise show an empty 3D view.
  - `/world/save_mesh` writes the **full-detail PLY** — 779 740 triangles against
    the Marker's 120 000 on the same extraction. A Marker is rebuilt and
    re-serialised on every publish; a file is written once.
- `bash tools/mesh-views.sh <mesh.ply>` renders three fixed angles offscreen, in
  numpy, with no GL and no Open3D. **Those PNGs are the evidence; the RViz window
  is not.**

**Cost:** 0.7 s at 6000 blocks to 4.2 s at 200 000, off the integration path by
construction. **`mesh_min_weight` is the honest lever** on a mesh that is mostly
layers of the same wall — it is 24 rather than the volume's 3, chosen from the
weight histogram `mesh_node` logs, and it is the difference between 15.7 M
triangles and 800 k.

**What the mesh does not look like yet:** a room. P7 corrected the pose — a
rotation that had been composed inverted since P3, worth 3× on the paired-surface
gap, and a real translation on top of it — and the surface is better for it
without being a room. What is left is the depth network rather than the geometry:
Depth Anything V2 estimates *relative* depth, its scale breathes a few percent a
frame, and its shape changes with viewpoint, so the same wall comes back at a
different distance however well the camera is posed. And the scale is still
arbitrary until a tape measure pins `depth_scale`.

## Stage 7 — Dashboard (`dashboard_node`, dev box)

**Built 2026-09-19** — P8, [#8](https://github.com/bthek1/ros2_pi/issues/8). Full
design, channels and layout in [dashboard.md](dashboard.md); what matters to this
document is the shape.

**It is the only dev-box node outside the container**, and that is a requirement
rather than an oversight. Everything else shares a process so a 2.7 MB frame is
handed on as a pointer; this one subscribes to five small topics and holds a
socket open to something outside the machine's control, and the promise it makes
is *it must be able to die*. A component in the container could not make that
promise — a crash there would take the TSDF with it.

**It computes nothing.** Every number on the page is a number some node measured
about itself and published on `/pipeline/stats`, so the page and `ros2 topic
echo` cannot disagree. That is also what makes the gate's instrument honest: a
transport carrying `rate_hz` values another node already computed cannot
influence them, which is why `ros2 topic echo` is an acceptable reader here where
`ros2 topic hz` would not be.

**Nothing it does may slow the pipeline down**, and that is the whole of what P8
asserts. A client's queued bytes past 8 MB are dropped and counted rather than
queued, so a backgrounded browser cannot apply backpressure; the count is on the
page. `bash tools/gates/dashboard.sh` replays the clip with a client attached,
without one, and without one *again* — the third run measuring the noise floor
the first two are compared against. On a quiet box that floor is **0.10%** and
the client costs **0.77%**, which meets P8's 2%; the gate states the bound as
floor-plus-slack anyway, because an earlier run of it measured 15% between two
identical no-client runs and that was a `colcon build` sharing the machine.

**Measured** (`bash tools/gates/dashboard.sh`): the five channels at 10.01 Hz
stats, 9.16 Hz rgb, 4.47 Hz depth, 10.01 Hz pose and ~2.1 MB of mesh per
extraction; the pose STALE flag 2.10 s after `/odom` stops, against a 2.0 s
threshold.

## Data flow summary

| From | To | Payload | Rate |
| --- | --- | --- | --- |
| Pi | dev box | JPEG, ~100–200 kB | up to 60 Hz, ~2–12 MB/s |
| `decode` | `keypoint`, `depth` | `cv::Mat` pointer | intra-process, zero copy |
| `depth` | `fusion` | 3.7 MB float depth + its RGB frame | **17.4 Hz measured**, intra-process |
| `fusion` | `mesh` | TSDF snapshot | every ~10 s |
| `mesh` | RViz, dashboard | ~120 k triangles | every ~10 s |
| all | dashboard | stats | 10 Hz |
