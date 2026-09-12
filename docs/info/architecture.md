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
                                       │   │      │      ├─▶ /keypoints       │
                                       │   │      │      ├─▶ /odom + tf       │
                                       │   │      │      └─▶ /keypoints/image │
                                       │   │      │                           │
                                       │   │      └──▶ depth_node (GPU)       │
                                       │   │             └─▶ /depth (+ /depth/rgb)
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
`pimesh_hello` demonstrates, and what `decode_node` uses on the inter-process
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
| `pimesh_hello` | `ament_cmake` | both | **Built 2026-09-08.** `hello_node`, `echo_node` — the scaffolding reference: components, thin mains, keyed YAML, composed launch. Carries no pipeline logic and is not in its path |
| `pimesh_msgs` | `ament_cmake` (rosidl) | both | **Built 2026-09-09.** `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`, `SaveMesh.srv`, `ResetMap.srv`. All five generate identically under Lyrical and Jazzy — `bash tools/gates/build.sh` diffs `ros2 interface show` across the two machines |
| `pimesh_camera` | `ament_cmake` | **Pi** | **Built 2026-09-09.** `camera_node` — V4L2 capture, MJPEG passthrough, capture-time stamps; plus `capture_probe`, the subscriber `gates/capture.sh` measures the stream with |
| `pimesh_perception` | `ament_cmake` | dev box (**builds on both**) | **Built 2026-09-12.** `decode_node` — the container's one network subscriber, JPEG → `bgr8`, **1.90 ms/frame**; `keypoint_node` — ORB 500 features, pooled matching, rotation-only pose, **6.7 ms/frame**; `ipc_probe`, the instrument `gates/ipc.sh` measures the pointer handover with. `depth_node` is P4 and does not exist |
| `pimesh_world` | `ament_cmake` | dev box | `fusion_node` (TSDF), `mesh_node` (marching cubes, PLY export) |
| `pimesh_dashboard` | `ament_cmake` | dev box | `dashboard_node` — HTTP + WebSocket server, vendored web UI |
| `pimesh_bringup` | `ament_cmake` | both | **Built 2026-09-09.** launch files, `config/pimesh.yaml`, `rviz/camera.rviz`. Compiles nothing — every dependency is an `exec_depend` |

`pimesh_hello`, `pimesh_camera` and `pimesh_msgs` build on the Pi under
**Jazzy**; everything else is dev-box-only under **Lyrical**. Keep the Pi-side pair to C++17 and to
APIs that exist in both distros — see [CLAUDE.md](../../CLAUDE.md) on the ABI
split.

## Topics

| Topic | Type | Publisher | QoS | Notes |
| --- | --- | --- | --- | --- |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-09.** The only topic on the LAN. MJPEG straight from V4L2, never re-encoded. ~80 kB/frame, measured 44–59 Hz on the dev box against 59 Hz at the Pi |
| `/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | RELIABLE, KEEP_LAST(1), transient local | **Live 2026-09-09, and carrying a real calibration since 2026-09-12** ([P9](https://github.com/bthek1/ros2_pi/issues/9), closed): fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px, loaded from `package://pimesh_bringup/config/camera_info/c922_720p.yaml`. Falls back to *nominal* intrinsics with a startup WARNING if that file is absent |
| `/image_raw` | `sensor_msgs/Image` (bgr8) | `decode_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-12.** Intra-process only, and meant to stay that way: an out-of-process subscriber forces a 2.7 MB serialisation per frame, which is why `rviz/keypoints.rviz` watches the compressed preview instead. Consumers take `ConstSharedPtr`, for the reason in [Why one container](#why-one-container) |
| `/keypoints` | `pimesh_msgs/Keypoints` | `keypoint_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-12.** 500 features per frame, struct-of-arrays, 32-byte descriptors, `track_id` = −1 for a first sighting |
| `/keypoints/image/compressed` | `sensor_msgs/CompressedImage` | `keypoint_node` | RELIABLE, KEEP_LAST(1) | **Live 2026-09-12.** Annotated preview, green for a followed corner and yellow for a new one, capped at ~10 Hz because encoding it (2.2 ms) costs more than detecting the features. RELIABLE *writer*; the viewer asks BEST_EFFORT, which is the compatible direction |
| `/depth` | `sensor_msgs/Image` (32FC1) | `depth_node` | RELIABLE, KEEP_LAST(1) | Metres. Carries the *input frame's* stamp and optical frame |
| `/depth/rgb` | `sensor_msgs/Image` (bgr8) | `depth_node` | RELIABLE, KEEP_LAST(1) | The exact frame inferred on, so fusion gets a true RGB-D pair with no sync guessing |
| `/depth/image/compressed` | `sensor_msgs/CompressedImage` | `depth_node` | BEST_EFFORT, KEEP_LAST(1) | Colourised preview for the dashboard |
| `/odom` | `nav_msgs/Odometry` | `keypoint_node` | RELIABLE, KEEP_LAST(10) | **Not published yet.** As of P3 `keypoint_node` publishes only the `odom → base_link` TF, rotation-only with translation identically zero; there is no Odometry message until there is a translation and a covariance worth putting in one (P7) |
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
- The TSDF volume is owned by `fusion_node` and never handed out. `mesh_node`
  gets a snapshot copy of the truncated distance field under a short lock, then
  meshes without holding it — a 900 ms mesh must not stall a 13 Hz integrator.
