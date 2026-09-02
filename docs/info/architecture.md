# Architecture

*Design intent, 2026-09-01, except where marked **built**. `pimesh_msgs` and
`pimesh_bringup` exist and their gate passes (P0); no pipeline nodes do yet —
see [roadmap.md](roadmap.md).*

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
with `use_intra_process_comms=True` passes a `std::shared_ptr` between
publisher and subscriber with no serialisation and no copy. So:

- **One network subscriber** (`decode_node`), one JPEG decode, one `cv::Mat`.
- Downstream components share that buffer. A 1280×720 BGR8 frame is 2.7 MB; at
  30 fps, avoiding four extra copies saves ~330 MB/s of memory traffic and the
  serialisation on top of it.
- The dashboard and RViz stay *outside* the container — they are viewers, they
  can pay for serialisation, and they must be able to die without taking the
  pipeline with them.

**Rule: a component that only works standalone is a bug.** Publish and subscribe
with `unique_ptr`/`shared_ptr` message moves, never with stack copies, or intra
process quietly falls back to serialising.

## Packages

| Package | Build | Runs on | Contents |
| --- | --- | --- | --- |
| `pimesh_msgs` | `ament_cmake` (rosidl) | both | **built** — `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`, `SaveMesh.srv`, `ResetMap.srv` |
| `pimesh_camera` | `ament_cmake` | **Pi** | `camera_node` — V4L2 capture, MJPEG passthrough, capture-time stamps |
| `pimesh_perception` | `ament_cmake` | dev box | `decode_node`, `keypoint_node`, `depth_node` |
| `pimesh_world` | `ament_cmake` | dev box | `fusion_node` (TSDF), `mesh_node` (marching cubes, PLY export) |
| `pimesh_dashboard` | `ament_cmake` | dev box | `dashboard_node` — HTTP + WebSocket server, vendored web UI |
| `pimesh_bringup` | `ament_cmake` | both | **built** — `pimesh.launch.py` (the container), `frames.launch.py` (static TF), `config/pimesh.yaml`; RViz config still to come |

`pimesh_camera` and `pimesh_msgs` build on the Pi under **Jazzy**; everything
else is dev-box-only under **Lyrical**. Keep the Pi-side pair to C++17 and to
APIs that exist in both distros — see [CLAUDE.md](../../CLAUDE.md) on the ABI
split.

## Topics

| Topic | Type | Publisher | QoS | Notes |
| --- | --- | --- | --- | --- |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_node` | RELIABLE, KEEP_LAST(1) | The only topic on the LAN. MJPEG straight from V4L2, never re-encoded |
| `/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | RELIABLE, KEEP_LAST(1), transient local | Real intrinsics from calibration, not defaults |
| `/rgb/image` | `sensor_msgs/Image` (bgr8) | `decode_node` | RELIABLE, KEEP_LAST(1) | Intra-process only. Never crosses the network |
| `/keypoints` | `pimesh_msgs/Keypoints` | `keypoint_node` | RELIABLE, KEEP_LAST(1) | Positions, descriptors, match ids for the frame |
| `/keypoints/image/compressed` | `sensor_msgs/CompressedImage` | `keypoint_node` | BEST_EFFORT, KEEP_LAST(1) | Annotated preview for the dashboard. Small, droppable |
| `/depth` | `sensor_msgs/Image` (32FC1) | `depth_node` | RELIABLE, KEEP_LAST(1) | Metres. Carries the *input frame's* stamp and optical frame |
| `/depth/rgb` | `sensor_msgs/Image` (bgr8) | `depth_node` | RELIABLE, KEEP_LAST(1) | The exact frame inferred on, so fusion gets a true RGB-D pair with no sync guessing |
| `/depth/image/compressed` | `sensor_msgs/CompressedImage` | `depth_node` | BEST_EFFORT, KEEP_LAST(1) | Colourised preview for the dashboard |
| `/odom` | `nav_msgs/Odometry` | `keypoint_node` | RELIABLE, KEEP_LAST(10) | Plus the `odom → base_link` TF |
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
