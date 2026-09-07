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

## Stage 1 — Capture (`camera_node`, on the Pi)

**Built and gated 2026-09-02** (`just gate-capture`). Everything in this section
is measured unless it says otherwise.

**Job:** get frames off the sensor, stamp them honestly, put them on the wire.
Nothing else. No decode, no re-encode, no processing.

- Opens the C922 through its serial-keyed `by-id` symlink (`/dev/video0`;
  `index1` is the UVC metadata node), `V4L2_PIX_FMT_MJPEG`, 1280×720, an `mmap`
  pool of 4 buffers, `poll()` → `VIDIOC_DQBUF` → publish the JPEG bytes verbatim
  as `CompressedImage` with `format: "jpeg"`.
- **A dedicated capture thread blocking in `poll()`, not a ROS timer.** A timer
  asks the camera for a frame at a rate the node picked, and beats against the
  sensor's own cadence — the predecessor's timer-driven driver delivered 24 fps
  while raw V4L2 on the same camera delivered 30.
- **Measured cost: 37 µs per frame** from dequeue to publish (the one
  unavoidable memcpy of ~150 kB out of the mmap'd buffer, so the buffer can go
  straight back to the driver). Against a 17–34 ms frame interval that is noise.
- **Measured throughput: within 1–2.5% of raw `v4l2-ctl` on the same camera in
  the same run.** That ratio is the claim the gate asserts; the absolute number
  is not a constant, because the C922's rate tracks its auto-exposure time
  (29.7 fps and 58.8 fps both measured on 2026-09-02 —
  [hardware.md](hardware.md#capture-behaviour)).

### The timestamps, which are why this node exists

`usb_cam` 0.8.1 computes its monotonic-to-wall offset **once per process** and
gets it wrong by up to a second — a `tv_sec * 1000000 + tv_usec / 1000.0`
mix-up — so every stamp it emits sits a random sub-second amount in the past,
redrawn at each launch (measured 0.223 / 0.362 / 0.979 s on three launches).

`camera_node` samples `CLOCK_MONOTONIC` and `CLOCK_REALTIME` **per frame**,
microseconds after `VIDIOC_DQBUF`, and adds the difference to the buffer's own
capture timestamp. There is no epoch to get wrong, so there is nothing to drift.
The node also checks `V4L2_BUF_FLAG_TIMESTAMP_MASK` and says so loudly if the
driver is not giving `CLOCK_MONOTONIC` capture times, because then every
downstream latency number would be fiction.

**Measured, on the Pi where publisher and subscriber share one clock: 5 ms
stamp-to-receipt, and 0.00 ms of movement between two separate launches.** That
last figure is the whole test — it is exactly what `usb_cam` fails.

Measured from the dev box the same quantity reads 29–38 ms and wanders, because
it also carries the Wi-Fi transfer and the offset between two machines' clocks.
That is the number the rest of the pipeline lives with, and it is reported, but
it is not what the node is judged on.

### Failing loudly

A missing device and a busy device both make the process **exit non-zero within
1–2 s**, with the failing ioctl and errno in the message
(`cannot start: VIDIOC_S_FMT failed: Device or resource busy (errno 16)`), and
`on_exit=Shutdown()` takes the launch down with it. `usb_cam` logs one ERROR and
then idles forever, which from the outside is indistinguishable from a healthy
node publishing into a topic nobody reads.

### Not yet true

`/camera_info` is published transient-local at the right rate and with the right
stamp, but **K is all zeros** — there is no calibration, and the node warns at
startup. Zeros are deliberate: a consumer can detect an uncalibrated camera with
`info.k[0] == 0.0`, where a fabricated focal length would let every downstream
stage compute confident nonsense. Loading a real calibration is deferred with a
trigger in
[../plans/future/bootstrap-future.md](../plans/future/bootstrap-future.md).

## Stage 2 — Decode (`decode_node`, dev box) — **built**

The container's only network subscriber. `cv::imdecode` on the JPEG, publish
`bgr8` intra-process. Exists as its own component so that the decode happens
exactly once no matter how many consumers appear.

**Measured 2026-09-04 (`just gate-ipc`, ×2):** 30.00 Hz at **1.88–2.07 ms mean,
2.41–2.56 ms p95** for 1280×720 — comfortably inside the ~4 ms target. Zero
mailbox drops (decode keeps up with the camera) and zero undecodable frames.

- The subscription callback moves a `shared_ptr` into a **one-deep mailbox** and
  returns; a worker thread does the decode. Newest frame wins, and the mailbox
  counts what it displaced so `dropped_mailbox` is measured rather than
  estimated.
- The output is published as a **`unique_ptr`**, which is what lets rclcpp hand
  the buffer downstream instead of serialising it. `cv_bridge`'s `toImageMsg()`
  returns a `shared_ptr` and would cost an extra copy per frame, so the message
  is filled by hand.
- **A failed decode is counted, never fatal.** A frame that lost a Wi-Fi
  fragment cannot be reassembled; that is a routine event, and
  `dropped_transport` is where it goes. Note that `cv::imdecode`'s
  three-argument form leaves its destination holding the *previous* frame on
  failure — read its return value, or the node republishes a stale image with a
  fresh stamp.

## Stage 3 — Keypoints (`keypoint_node`, dev box)

**Job:** find repeatable corners, match them across frames, and turn the matches
into camera motion.

- `cv::ORB`, **500 features** — enough that ranking churn at the cap does not
  dominate, cheap enough at 30+ fps. ~5 ms/frame **(target)**.
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
  residual under 0.03 rad (~1.7°, ~27 px at fx≈907) after reject-worst refits
  **(inherited)**. Failing the gate means *hold the last pose*, not publish a
  guess.
- Keep a **keyframe store** — descriptors, bearing rays and 3D landmarks, a new
  keyframe whenever the view direction is ~18° from every stored one or the
  camera has moved 0.3 m. It is what makes relocalisation and loop closure
  possible later, and it costs ~16 kB per keyframe.

`/keypoints` carries positions, descriptors and per-feature match ids so the
dashboard can draw tracks without recomputing anything.

## Stage 4 — Depth (`depth_node`, dev box, GPU)

**Job:** one RGB frame in, one metric depth map out.

- **Depth Anything V2 Small (ViT-S/14), ONNX.** RGB in, ImageNet-normalised,
  spatial dims a multiple of 14 — **518 = 37×14** is the trained size. ~99 MB of
  weights, fetched and checksummed, never committed.
- **ONNX Runtime C++ with the CUDA execution provider**, CPU as an explicit
  fallback that logs which provider it got. Do not let it silently land on CPU
  and then wonder why the mesh stopped updating.
- **Measured on this GPU (GTX 1660 SUPER, via the predecessor):**
  **72–79 ms/frame** on CUDA, **280–305 ms/frame** on CPU, ~1.3 s first-inference
  warm-up. So: **~13 Hz with the GPU, ~3 Hz without.** Warm the session at
  startup so the first real frame is not the slow one.
- Convert to metres with `depth_scale`, clip beyond 6 m *before* the reciprocal,
  and publish `32FC1` with the **input frame's** stamp and `camera_optical_frame`
  — derived data keeps the header of what it describes, not the moment inference
  finished.
- Publish `/depth/rgb`, the exact frame inferred on, so fusion gets a true
  RGB-D pair.

**Optimisations, in the order worth trying:** none of them until the CUDA EP is
running and measured. Then: input at 392² instead of 518² (roughly halves the
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
| `depth` | `fusion` | 3.7 MB float depth + its RGB frame | ~13 Hz, intra-process |
| `fusion` | `mesh` | TSDF snapshot | every ~10 s |
| `mesh` | RViz, dashboard | ~120 k triangles | every ~10 s |
| all | dashboard | stats | 10 Hz |
