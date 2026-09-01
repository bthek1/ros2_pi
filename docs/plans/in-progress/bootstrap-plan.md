# Bootstrap plan — one webcam to a live mesh, in C++

**Started 2026-09-01.** Build order for the whole pipeline. Nine phases, each
ending in something that runs and a gate that names its own evidence.

The predecessor [`~/Documents/piros2`](../../../../piros2) already does all of
this in Python. **That is the reference and the yardstick**: at every phase where
a number exists over there, the C++ version is measured against it, so "the
rewrite is faster" is a measurement rather than an article of faith.

## P0 — Workspace skeleton

`pimesh_msgs` and `pimesh_bringup`, a justfile, and a build that succeeds on
**both machines**.

- `pimesh_msgs`: `Keypoints.msg`, `PipelineStats.msg`, `MeshStats.msg`,
  `SaveMesh.srv`, `ResetMap.srv`.
- `pimesh_bringup`: the static `base_link → camera_link →
  camera_optical_frame` transforms, `config/pimesh.yaml`, an empty container
  launch.
- Justfile recipes: `build`, `sync-pi`, `build-pi`, `test`, `stragglers`.

**Gate:** `colcon build` clean here **and** on the Pi over SSH, and
`ros2 interface show pimesh_msgs/msg/Keypoints` prints on both machines. The
cross-distro build is the point of this phase — Lyrical and Jazzy must both be
satisfied by the same source.

## P1 — Capture on the Pi

`pimesh_camera/camera_node`: V4L2, MJPEG, `mmap` buffers, publish
`CompressedImage` + transient-local `CameraInfo`.

- Stamp from the dequeued buffer's own timestamp, not from `now()` at publish.
- Exit non-zero with a clear message on a missing or busy device — never idle.
- No decode, no re-encode, no processing. The bytes go out as they came in.

**Gates:**
1. `ros2 topic hz /image_raw/compressed` on the **dev box** ≥ 40 Hz with the
   camera's exposure control cleared — matching the 42–60 fps the predecessor
   measured, and stating the exposure mode.
2. **Stamps are honest**: a script comparing `header.stamp` against dev-box
   receipt time shows an offset within one frame interval and *stable across
   two separate launches* — which is exactly what `usb_cam` 0.8.1 fails. This
   gate is the reason the node exists.
3. Unplug the camera → the node exits non-zero within 2 s.

## P2 — The container, and proving intra-process

`pimesh_perception/decode_node` plus the bringup container with
`use_intra_process_comms=True`.

**Gate:** log the address of the received message buffer in a second component
and show it **equals** the publisher's — a serialised path cannot produce that.
Plus: exactly one subscriber on `/image_raw/compressed` (`ros2 topic info -v`),
because that is the Wi-Fi constraint the whole architecture is shaped around.

## P3 — Keypoints

`keypoint_node`: `cv::ORB` at 500 features, pooled matching over a 10-frame
window, Hamming threshold 64, annotated preview on
`/keypoints/image/compressed`.

**Gates:**
1. ≥ 30 Hz sustained with ≤ 8 ms mean per-frame cost measured against the node's
   own clock (never against `header.stamp`).
2. Matched-keypoint fraction on a static scene within a few points of the
   predecessor's, on the same recorded input.

Rotation-only odometry lands here too, with its gates: ≥ 8 matched pairs and
mean ray residual < 0.03 rad, and **hold the last pose** rather than publishing
a guess when the gate fails.

## P4 — Depth on the GPU

`depth_node`: ONNX Runtime C++, CUDA execution provider, Depth Anything V2 Small
at 518². Publish `/depth` (32FC1, metres) and `/depth/rgb`.

This is the phase with real setup risk — see [../../info/setup.md](../../info/setup.md#gpu).
Do the toolchain work first, standalone, before writing any ROS code: a
20-line C++ program that loads the model, runs one frame and prints the provider
and the time.

**Gates:**
1. Startup log names `CUDAExecutionProvider`. If it says CPU, the phase is not
   done, however well the mesh looks.
2. **≤ 80 ms/frame steady**, matching the predecessor's 72–79 ms. A number
   materially worse than Python's means something is wrong with the C++ setup,
   not with C++.
3. `/depth` carries its **input frame's** stamp and `camera_optical_frame`, and
   `/depth/rgb` is byte-identical to the frame that was inferred on.

## P5 — Fusion

`fusion_node`: spatially hashed TSDF, 1.5 cm voxels, weighted colour, per-frame
scale alignment against a ray-cast of the existing volume (skip below 20%
overlap, refuse corrections beyond 15%).

**Gates:**
1. ≤ 20 ms per integration at 13 Hz, single-threaded, and no growth in the
   mailbox drop count over a 5-minute run — the queue must not grow.
2. **A paired-surface check**: two views of the same wall, integrated, and the
   measured gap between the two surfaces reported. The predecessor got that gap
   from 7.8 cm to 5.7 cm with alignment on. Alignment off must be measurably
   worse here too, or the alignment is not doing what we think.

## P6 — Surface

`mesh_node`: marching cubes on a snapshot, component pruning under 30 triangles,
interior loops under 0.25 m fan-filled (frontier loops left open), quadric
decimation to 120 k triangles for the Marker, full-detail PLY on
`/world/save_mesh`.

**Gates:**
1. Meshing **never blocks integration**: the integrate rate over a 5-minute run
   shows no dip at mesh time.
2. `just mesh-views` renders the saved PLY from three fixed angles to files —
   that image is the evidence, not the RViz window.
3. Triangle count under the cap with **no pinholes** — decimation, never
   subsampling.

## P7 — 6-DoF odometry

Back-fill the keypoint node with depth-backed 3D–3D pose estimation, so
translation stops being invisible. Rotation-only stays as a fallback mode and
the node **logs which regime it is in**.

**Gate:** on a recorded sweep, mesh smear measurably drops versus P3's
rotation-only poses, on the same bag. A hand-held pan carries ~0.9 m of real
arm arc — the predecessor measured that — so this is the phase where the surface
starts being a surface.

## P8 — Dashboard

`dashboard_node`: HTTP + WebSocket, vendored assets, the layout in
[../../info/dashboard.md](../../info/dashboard.md).

**Gates:**
1. With the browser open, **every pipeline rate is unchanged** — the dashboard
   applying backpressure to the pipeline is the failure mode to prove absent.
2. Kill the browser mid-session, then the node: the pipeline does not notice.
3. Stale feeds show STALE within 2 s of the publisher stopping, measured on
   receipt time.

## Deferred, deliberately

- **Loop closure, pose graph, volume rebuild** (M10). The predecessor has all
  three and the numbers to compare against; it is a plan of its own once P0–P8
  are real.
- **CUDA TSDF kernels.** After P5 is correct and profiled, not before, and not
  until a CUDA toolkit is installed.
- **Relocalisation from a saved room map.** The keyframe store goes in at P3
  because it is cheap; using it is later work.
