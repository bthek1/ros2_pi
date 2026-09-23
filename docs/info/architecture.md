# Architecture

*Design intent, 2026-09-01, except where a row says otherwise. **Capture and the
frame tree are built and measured** as of 2026-09-09 (P0–P1,
[issue #4](https://github.com/bthek1/ros2_pi/issues/4)); everything from
`decode_node` rightwards is still a plan. See [roadmap.md](roadmap.md) for the
one-line status view.*

## The shape of it

One webcam. Two machines. Six packages. The Pi does nothing but capture and
ship; the dev box turns that stream into a surface and shows it.

```
Raspberry Pi (Jazzy, aarch64)          │  Dev box (Lyrical, x86_64 + GTX 1660S)
                                       │
  /dev/video0                          │   ┌──────── pimesh_container ────────┐
      │ V4L2 MJPEG, no decode          │   │  (one process, intra-process on) │
      ▼                                │   │                                  │
  camera_node ──/image_raw/compressed──┼──▶│  decode_node ──▶ /rgb/image      │
      └────────── /camera_info ────────┼──▶│      │                           │
                                       │   │      ├──▶ keypoint_node          │
                                       │   │      │      ├─▶ /keypoints ──┐   │
                                       │   │      │      └─▶ /keypoints/image │
                                       │   │      │                        │  │
                                       │   │      │      odometry_node ◀───┘  │
                                       │   │      │         ▲  └─▶ /odom + tf │
                                       │   │      │         │  (the SLAM front
                                       │   │      │         │   end: pose only)
                                       │   │      └──▶ depth_node (GPU)    │  │
                                       │   │             └─▶ /depth ───────┘  │
                                       │   │                 (+ /depth/rgb)   │
                                       │   │                    │             │
                                       │   │  fusion_node ◀─────┘             │
                                       │   │      └─▶ TSDF volume             │
                                       │   │             │                    │
                                       │   │  mesh_node ─┘                    │
                                       │   │      ├─▶ /world/mesh (Marker)    │
                                       │   │      └─▶ /world/mesh_stats       │
                                       │   └──────────────┬───────────────────┘
                                       │                  │
                                       │        dashboard_node ──▶ :8080 web UI
                                       │        rviz2 (optional second view)
```

Only one arrow crosses the network, and it carries ~100–200 kB JPEGs. That is
the whole network design.

## Why one container

The Python predecessor ran each stage as its own process, and five of them
subscribed the Pi's stream directly. Each RELIABLE subscriber pulls its own
unicast copy over Wi-Fi: measured ~2 frames/s per reader against 14.7 Hz for a
single reader. The fix there was a relay node — one reader, republished on
loopback.

C++ makes the relay unnecessary. `rclcpp_components` composed into one container
with `use_intra_process_comms=True` hands the message pointer from publisher to
subscriber with no serialisation and no copy. So:

- **One network subscriber** (`decode_node`), one JPEG decode, one `cv::Mat`.
- Downstream components share that buffer. A 1280×720 BGR8 frame is 2.7 MB; at
  30 fps, avoiding four extra copies saves ~330 MB/s of memory traffic and the
  serialisation on top of it.
- The dashboard and RViz stay *outside* the container — they are viewers, they
  can pay for serialisation, and they must be able to die without taking the
  pipeline with them.

**Rule: a component that only works standalone is a bug.** Publish and subscribe
with pointer moves, never with stack copies, or intra-process quietly falls back
to serialising. Both halves are required: the publisher moves a `unique_ptr` into
`publish()`, **and** the subscription callback takes a pointer rather than a
value.

**Which pointer depends on how many consumers the topic has, and getting this
wrong costs a full copy per frame in silence.** Measured 2026-09-12, with
`decode_node` publishing 2.7 MB frames to two consumers in one container:

| Consumers' callbacks | Frames arriving at the published address |
| --- | --- |
| two × `std::unique_ptr` | **0 of 574** |
| two × `ConstSharedPtr` | **504 of 504** |

rclcpp's intra-process manager serves *ownership-taking* subscriptions by moving
the buffer into the last one and **copying it for every other** — the comment in
`rclcpp/experimental/intra_process_manager.hpp` reads "Copy the message since we
have additional subscriptions to serve". Subscriptions that take a shared const
pointer go through `add_shared_msg_to_buffers` instead, which hands *one* buffer
to all of them however many there are.

So: **`unique_ptr` for a topic with exactly one consumer** (which is what
`decode_node` uses on its inter-process
stream from the Pi, where the middleware allocates a fresh message anyway);
**`ConstSharedPtr` for a fan-out**, which is every derived topic in this pipeline.
A `const &` callback is also a shared subscription and does not copy — the
long-standing note in this project that it "works perfectly and copies in
silence" was the wrong way round, and only the fan-out measurement could show
that.

The failure mode deserves restating, because it is the reason this is written
down rather than inferred: **one consumer is the case that cannot fail.** This
project's gate passed at 429/429 with a single probe subscribed, and reported
0/574 on the next run with a second consumer beside it — same code, same flags.
`tools/gates/ipc.sh` therefore asserts that the decoded topic has **at least two
subscribers** while it measures, because a gate that measures the easy
configuration is a gate whose green means nothing the day a stage is added.

### It is measured, and the measurement needs a control

`bash tools/gates/hello-ipc.sh` runs one container twice — `intra_process:=true` and
`intra_process:=false`, same binary, same launch file — and compares the payload
address the publisher logged against the address the subscriber was handed.
Measured 2026-09-08: **19/19 equal with it on, 0/16 with it off**.

`bash tools/gates/ipc.sh` is the same experiment on the real pipeline, with the
Pi's camera feeding it: the probe is loaded into the live container beside
`keypoint_node` and `decode_node`. Measured 2026-09-12: **504/504 equal with it
on, 0/395 with it off**, exactly one subscriber on `/image_raw/compressed`, and
decode at **1.90 ms/frame** against a 4 ms budget.

**The container is `component_container_isolated`, not `component_container_mt`**
(changed 2026-09-12). `_mt` is deprecated on Lyrical — "will be removed in
M-turtle" — and the isolated one is the better fit anyway: it gives each
component its own executor, so P4's 76 ms depth callback cannot delay another
node's callbacks at all, where one shared thread pool makes that a question of
how many threads happen to be free. Intra-process comms is unaffected, which was
checked rather than assumed by re-running both gates above.

The control run is not ceremony. Two allocations in one process can land on the
same address by coincidence — the publisher frees, the subscriber allocates the
same size, malloc obliges — and in an early run that happened at 1/22. So
address equality on its own proves nothing; the claim is that the addresses
match *and stop matching the moment the mechanism is switched off*. Any later
zero-copy claim in this project needs the same shape of evidence.

## Packages

Four of the seven exist; the rest is the plan.

| Package | Build | Runs on | Contents |
| --- | --- | --- | --- |
| `pimesh_msgs` | `ament_cmake` (rosidl) | both | **Built 2026-09-09.** `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`, `SaveMesh.srv`, `ResetMap.srv`. All five generate identically under Lyrical and Jazzy — `bash tools/gates/build.sh` diffs `ros2 interface show` across the two machines |
| `pimesh_camera` | `ament_cmake` | **Pi** | **Built 2026-09-09.** `camera_node` — V4L2 capture, MJPEG passthrough, capture-time stamps; plus `capture_probe`, the subscriber `gates/capture.sh` measures the stream with |
| `pimesh_dashboard` | `ament_cmake` | dev box (**builds on both**) | **Built 2026-09-19** — P8. `dashboard_node` — an HTTP and WebSocket server inside a ROS 2 node, **in its own process**, serving vendored assets from `share/web` and forwarding what every stage published about itself; `dashboard_probe`, the WebSocket client `gates/dashboard.sh` measures with. **No WebSocket library and no three.js**: the server half of RFC 6455 is a SHA-1, a base64 and a frame header, all pinned against published vectors in `test_websocket`, and the 3D view is ~200 lines of raw WebGL. A vendored library would be a third thing that has to behave identically on Jazzy and Lyrical, which is the failure mode this workspace keeps paying for |
| `pimesh_perception` | `ament_cmake` | dev box (**builds on both**) | **Built 2026-09-12, depth added 2026-09-15.** `decode_node` — the container's one network subscriber, JPEG → `bgr8`, **1.90 ms/frame**; `keypoint_node` — ORB 500 features, pooled matching, and each corner's position one frame earlier, **6.82 ms/frame** with depth and odometry beside it against an 8 ms budget; `odometry_node` — **the SLAM front end**, split out of `keypoint_node` on 2026-09-23 because that node had been publishing two `/pipeline/stats` rows since P7, being two stages wearing one name: a 6-DoF pose from `cv::solvePnPRansac` against the newest keyframe's landmarks at **1.37 px over 94 inliers, 81.6% of depth frames posed**, with `rotation_only` kept selectable as `gates/odom.sh`'s control; `depth_node` — Depth Anything V2 Small on the CUDA execution provider, **55.10 ms/frame, 17.4 Hz**; `ipc_probe`, `depth_probe` and `odom_probe`, the instruments `gates/ipc.sh`, `gates/depth.sh` and `gates/odom.sh` measure with — the last of those reads the trajectory off `/odom` rather than off the node's own counters, because the failure P7 fixed was a pose every internal number described correctly and that left the node inverted; and two headers that exist so a test can reach what used to sit inside a node: `stats.hpp`, the percentile and FNV-1a hash every probe reports its numbers through (four anonymous-namespace copies before it), and `keypoints_view.hpp`, the conversion from a `Keypoints` message back into corners, ids, descriptors and one-frame-apart pairs — three loops inside `odometry_node`'s callback until 2026-09-23, two of which read past the end of a vector on a message whose parallel arrays disagreed. **The whole package builds on the Pi too**, which is why the inference engine is behind an interface: `depth_engine_ort.cpp` is compiled only where CMake finds ONNX Runtime, `depth_engine_null.cpp` everywhere else, and the node, its registration and its parameters are identical on both machines |
| `pimesh_world` | `ament_cmake` | dev box | `fusion_node` (TSDF), `mesh_node` (marching cubes, PLY export) |
| `pimesh_dashboard` | `ament_cmake` | dev box | `dashboard_node` — HTTP + WebSocket server, vendored web UI |
| `pimesh_bringup` | `ament_cmake` | both | **Built 2026-09-09.** launch files, `config/pimesh.yaml`, `rviz/camera.rviz`. Compiles nothing — every dependency is an `exec_depend` |

`pimesh_camera` and `pimesh_msgs` build on the Pi under
**Jazzy**; everything else is dev-box-only under **Lyrical**. Keep the Pi-side pair to C++17 and to
APIs that exist in both distros — see [CLAUDE.md](../../CLAUDE.md) on the ABI
split.

## Topics

| Topic | Type | Publisher | QoS | Notes |
| --- | --- | --- | --- | --- |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-09.** The only topic on the LAN. MJPEG straight from V4L2, never re-encoded. ~80 kB/frame, measured 44–59 Hz on the dev box against 59 Hz at the Pi |
| `/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | RELIABLE, KEEP_LAST(1), transient local | **Live 2026-09-09, and carrying a real calibration since 2026-09-12** ([P9](https://github.com/bthek1/ros2_pi/issues/9), closed): fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px, loaded from `package://pimesh_bringup/config/camera_info/c922_720p.yaml`. Falls back to *nominal* intrinsics with a startup WARNING if that file is absent |
| `/image_raw` | `sensor_msgs/Image` (bgr8) | `decode_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-12.** Intra-process only, and meant to stay that way: an out-of-process subscriber forces a 2.7 MB serialisation per frame, which is why `rviz/keypoints.rviz` watches the compressed preview instead. Consumers take `ConstSharedPtr`, for the reason in [Why one container](#why-one-container) |
| `/pipeline/stats` | `pimesh_msgs/PipelineStats` | every stage | RELIABLE, KEEP_LAST(10) | **All seven rows live 2026-09-19** — P8 and P10. `capture` from the Pi (60.00 Hz, and the only stage that can report the frames the **kernel** dropped before dequeue), `decode`, `keypoints`, `odometry`, `depth`, `fusion`, `mesh`. `keypoints` and `odometry` are **two nodes** as of 2026-09-23 and one row each. They were one node publishing two rows, because ORB runs at the camera's rate and the pose solve at the depth rate, and a single row would have had to pick a rate while the numbers beside it belonged to the other — which is the observation the split came out of. **`dropped_by_design` and `dropped_in_transport` are never summed**: `depth` dropping ~72% of what it is offered is its single-slot mailbox working exactly as intended, and one column for both would make the healthy pipeline and the broken one look identical |
| `/keypoints` | `pimesh_msgs/Keypoints` | `keypoint_node` | RELIABLE, **KEEP_LAST(120)** | **Live 2026-09-12; carries the geometry's pairing since 2026-09-23.** 500 features per frame, struct-of-arrays, 32-byte descriptors, `track_id` (always ≥ 0) with `is_new` beside it, and `prev_x`/`prev_y` — where each corner sat one frame earlier, NaN where it was not mutually matched. **Two matchings go out here and they are not interchangeable**: track ids come from a pooled ten-frame window, the `prev_*` pair from a stricter mutual-best pass against the previous frame alone, which is what a rotation fit needs. **The depth is 120, not 1, and that is a correctness requirement**: `odometry_node` looks each message up by exact stamp and, in `rotation_only`, composes an increment from every one, so a message the middleware dropped to keep the queue at one is a rotation that silently never happened |
| `/keypoints/image/compressed` | `sensor_msgs/CompressedImage` | `keypoint_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-12.** Annotated preview, green for a followed corner and yellow for a new one, capped at ~10 Hz because encoding it (2.2 ms) costs more than detecting the features. RELIABLE *writer*; the viewer asks BEST_EFFORT, which is the compatible direction |
| `/depth` | `sensor_msgs/Image` (32FC1) | `depth_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-15.** Metres, clipped at 6 m *in inverse space, before the reciprocal* — `1/0` is `inf`, not a big number. Carries the *input frame's* stamp and `camera_optical_frame`. **17.4 Hz measured**, which is the rate everything downstream inherits. The absolute scale is arbitrary until P5 |
| `/depth/rgb` | `sensor_msgs/Image` (bgr8) | `depth_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-15.** The exact frame inferred on, so fusion gets a true RGB-D pair with no sync guessing — **1048 of 1048 measured byte-identical** to the `/image_raw` frame with the same stamp (`gates/depth.sh`). A separate republisher cannot do this: it would drop different frames, so the two stamp sets would rarely intersect |
| `/depth/image/compressed` | `sensor_msgs/CompressedImage` | `depth_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-15.** Inferno colour map of `/depth` as JPEG, ~10 Hz, for viewers and the dashboard. **A fixed `[0, max_range_m]` scale, inverted** — near bright, the clip black — so a colour *is* a distance: RViz's Image display has no colour map at all, only a per-frame `Normalize Range` that makes the same distance a different shade each frame. Its cost is counted **outside** the per-frame budget, like `keypoint_node`'s preview: it is a picture for a person and nothing downstream reads it |
| `/odom` | `nav_msgs/Odometry` | `odometry_node` | RELIABLE, KEEP_LAST(50) | **Live 2026-09-19** — P7. The pose as a *sequence* where TF is a tree queried by time, which is what an RViz Odometry display draws a trail from: `rviz/odom.rviz` keeps 500 of them, so P7's whole result is visible without a `nav_msgs/Path` publisher existing anywhere. In `sixdof` it is published at the depth rate, ~17.5 Hz, at each depth frame's own stamp; in `rotation_only` at the camera's rate with translation identically zero. `pose.covariance` is a **large positive diagonal** — the conventional spelling of *unconstrained*. It published `-1` in element 0 from P7 until 2026-09-23 on the belief that nav_msgs documents that sentinel; it does not (`sensor_msgs/Imu` does), and the result was a matrix that is not positive semidefinite, which RViz's Odometry display reported at the pose rate for the length of every session. Not zeros either: zeros are legal and read to a fusion filter as a *perfectly certain* pose. `twist` is zero-valued with the same covariance, which is what says nothing here estimates a velocity |
| `/world/mesh` | `visualization_msgs/Marker` | `mesh_node` | RELIABLE, transient local | `TRIANGLE_LIST`, vertex-coloured, capped for RViz |
| `/world/mesh_stats` | `pimesh_msgs/MeshStats` | `mesh_node` | RELIABLE, KEEP_LAST(1) | Triangles, vertices, volume extent, last mesh duration |
| `/pipeline/stats` | `pimesh_msgs/PipelineStats` | every node | RELIABLE, KEEP_LAST(1) | Per-stage rate, latency, drop count. The dashboard's data source |

Services: `/world/save_mesh` (writes a timestamped PLY), `/world/reset` (clears
the TSDF volume and the map frame).

**Why `/depth` and `/depth/rgb` are a pair.** Depth is derived from a specific
RGB frame. Republishing that frame alongside its depth turns a synchronisation
problem into a non-problem: fusion never has to guess which colour frame belongs
to which depth map, and colour never smears across the surface. The predecessor
learned this the expensive way.

**Why `/depth` is `32FC1` metres and not `16UC1` millimetres.** The monocular
model outputs relative inverse depth; converting it to metres involves a scale
factor and a reciprocal, both of which lose precision badly in integer
millimetres near the far clip. Float costs 3.7 MB/frame, which is free
intra-process and never leaves the machine.

## TF tree

REP-105 and REP-103, no shortcuts:

```
map ──(pose graph correction, discrete jumps)──▶ odom
odom ──(visual odometry, continuous, drifts)──▶ base_link
base_link ──(static)──▶ camera_link ──(static)──▶ camera_optical_frame
```

- `camera_optical_frame` is the optical convention (z forward, x right, y down).
  All geometry — keypoint rays, depth projection, TSDF integration — is done in
  it. `camera_link` is the ROS body convention (x forward, z up); the static
  transform between them is the standard −90°/+90° pair and is published by
  `pimesh_bringup`, not hand-rolled per node.
- **Exactly one publisher per edge.** Two nodes publishing `odom → base_link`
  is the failure that makes a mesh smear and is not obvious from any single log.
- `map → odom` is only published once there is a backend to publish it. Until
  then the launch publishes a static identity so the frame exists and RViz has a
  fixed frame, and that fact is stated in the launch file, not hidden.
- **A dynamic edge and a looping bag are mutually exclusive**, measured
  2026-09-13. `odometry_node` stamps `odom → base_link` with the frame's own
  stamp, and `ros2 bag play --loop` sends those stamps back by the bag's length
  at every wrap, which `tf2::BufferCore` rejects — so the edge freezes at the
  bag's last stamp and every listener logs `TF_OLD_DATA` at the frame rate, from
  inside the buffer's own mutex, which stalls RViz's render loop. The launch
  therefore takes `pipeline:=false`, bringing up the three static edges and no
  components at all; that is what `tools/replay.sh` uses to loop a clip safely,
  and `tools/view-keypoints.sh`, which needs the pose, plays a bag once instead.
  `--clock` with `use_sim_time` does not rescue it: the backwards jump makes
  `tf2_ros::Buffer` clear the whole buffer, `tf_static` included, and nothing
  republishes a latched topic. Full account in
  [troubleshooting.md](troubleshooting.md).
- **All three static edges are published as of 2026-09-09**, by
  `tf2_ros/static_transform_publisher` instances that `pimesh_bringup`'s launch
  starts, with their numbers read from `config/pimesh.yaml`. They arrive as
  command-line flags rather than as a parameter file because that executable
  parses `argv` and exits non-zero before it ever reads parameters — it declares
  `frame_id` and `translation.x` and never gets to them. Passing
  `parameters=[...]` produces three dead processes and a launch that carries on
  without a TF tree, which is worse than a hard failure because RViz then shows
  an empty Fixed Frame and looks like its own bug.

## Where the time goes

The measured per-frame budget on this hardware, from the predecessor's numbers
(see [pipeline.md](pipeline.md) for provenance):

```
capture + JPEG ship   ~16 ms   Pi, overlapped with everything else
decode                 ~4 ms   dev box CPU
ORB, 500 features      ~5 ms   dev box CPU  ─┐ these two run in parallel
depth inference    72–79 ms    dev box GPU  ─┘ on separate threads
TSDF integrate        ~15 ms   dev box CPU (target)
mesh extraction    300–900 ms  dev box CPU, every ~10 s, off the hot path
```

So the pipeline's steady rate is **~13 Hz, set by depth**, and the mesh refresh
is a background job that must never block integration. Keypoints run at full
camera rate because they are cheap and odometry wants them; depth and fusion run
at whatever the GPU sustains. **Every stage drops rather than queues**: a
single-slot mailbox where the newest frame overwrites the unprocessed one.

## Threading model

- One `rclcpp::executors::MultiThreadedExecutor` for the container.
- Each expensive node owns **one worker thread and a one-deep mailbox**. The
  subscription callback does a pointer swap and returns; the worker does the
  milliseconds.
- Callback groups: each node's subscription and its timer go in a
  `MutuallyExclusive` group, so a node is never re-entered while its own
  publisher is running.
- The TSDF volume is owned by `fusion_node` and never handed out as a message.
  `mesh_node` gets a snapshot copy of the truncated distance field and meshes
  without holding the lock — a 3 s extraction must not stall a 17 Hz integrator.
  **The two rendezvous through a process-local registry rather than a topic**
  (`pimesh_world/shared_volume.hpp`): the volume is over a gigabyte and its only
  consumer is in the same process, so publishing it would be a serialisation of
  the whole map ten times a minute between two components that share an address
  space. The consequence is stated rather than hidden — a `mesh_node` started
  alone, or in another container, finds no volume and says so.
- **The snapshot is taken in chunks**, releasing the lock between them, because
  one lock over a 1.25 GB copy is not a short lock: it holds the integrator out
  for over a hundred milliseconds. The copy therefore spans two instants, which
  for a weighted average already in the tens is invisible and is the right trade
  for not stalling the stage the whole pipeline feeds. **And the extraction
  thread is niced**, because at equal priority it starves the pipeline of CPU
  rather than of the lock — measured as `depth_node` dropping from 17.8 to
  14.6 Hz with its per-frame cost unchanged.
