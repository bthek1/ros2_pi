# The dashboard

*Design intent, 2026-09-01. Not built yet.*

## What it is for

One browser tab that answers three questions without a ROS installation:

1. **Is the pipeline alive?** Per-stage rate, latency and drop count, with a
   stage going amber the moment it stalls.
2. **What is the camera seeing?** The RGB frame, the ORB keypoints drawn on it,
   and the colourised depth map, side by side.
3. **What have we built?** The live mesh, orbitable, with the camera's current
   pose and its trajectory drawn in the same scene.

RViz already shows the mesh, and better. The dashboard exists because RViz needs
a ROS install, an X session and a config file, and because the two-dimensional
half of this pipeline — the stage rates, the frame, the keypoints, the depth
map — is exactly what RViz is worst at showing together.

## Shape

`dashboard_node` is a C++ ROS 2 node that is also an HTTP and WebSocket server.
No Python, no separate web backend, no `rosbridge` — one node, one port, one
process to start and stop.

```
   ROS topics                dashboard_node                     browser
────────────────         ─────────────────────            ─────────────────
/pipeline/stats  ──┐
/keypoints/image ──┤     subscribe → downsample →   WS    ┌─ stats panel
/depth/image     ──┼──▶  pace → frame → send      ═════▶  ├─ image strip
/world/mesh      ──┤                                      ├─ three.js scene
/odom, /tf       ──┘     serve static assets      HTTP    └─ (vendored JS)
                                                  ─────▶
```

**Library:** a single-header C++ WebSocket/HTTP server, vendored into the
package (no CDN, no npm at runtime). The web assets — `index.html`, one CSS
file, one JS bundle, three.js — are installed into the package's `share/` and
served from there, so `ros2 launch` is the only thing anyone has to run.

**Port** 8080 by default, bound to the LAN so a phone on the same network can
watch. It is a read-only view; the only writes are two buttons (below), and they
call the same services RViz would.

## What goes over the WebSocket

One connection, binary frames, each with a one-byte channel tag. Text JSON for
anything small, binary for anything large — never base64, which costs 33% for
nothing.

| Channel | Payload | Rate | Notes |
| --- | --- | --- | --- |
| `stats` | JSON | 10 Hz | Per-stage rate/latency/drops, GPU provider in use, TSDF voxel count, mesh triangle count, uptime |
| `rgb` | JPEG bytes | ~10 fps | The annotated keypoint frame — one image, not two, since the keypoints are drawn on the RGB |
| `depth` | JPEG bytes | ~5 fps | Colourised (turbo), with the metric range in the `stats` channel so the legend is honest |
| `mesh` | binary: vertices `f32×3`, colours `u8×3`, indices `u32×3` | on change, ~every 10 s | Sent as a whole replacement, not a delta |
| `pose` | JSON | 10 Hz | Current camera pose plus the trajectory tail |

**Pacing is the server's job, not the browser's.** The node subscribes at
whatever rate the pipeline runs and sends at the dashboard's rate, dropping in
between. A slow or backgrounded browser must never apply backpressure to the
pipeline: if the socket's send buffer is above a threshold, the frame is dropped
and counted, and the drop count is visible in the stats panel. **Nothing the
dashboard does may slow the mesh down.**

**Mesh transfer** is the one payload big enough to think about: 120 k triangles
is ~4.3 MB raw (vertices + colours + indices). At every-10-s refresh that is
~0.4 MB/s on a LAN, which is fine, and the browser rebuilds the
`BufferGeometry` in one go. If it ever hurts: send only changed voxel blocks, or
decimate harder for the web view than for RViz. Do not reach for Draco before
measuring — it adds a decoder and a build step to save bandwidth that is not
scarce here.

## Layout

```
┌──────────────────────────────────────────────────────────────────┐
│  pimesh          ● live   13.1 Hz depth   camera 47 fps   4m12s  │
├───────────────────────────────┬──────────────────────────────────┤
│                               │  camera + keypoints              │
│                               │  ┌────────────────────────────┐  │
│      3D mesh (three.js)       │  │                            │  │
│      orbit / pan / zoom       │  └────────────────────────────┘  │
│                               │  depth                           │
│      camera frustum           │  ┌────────────────────────────┐  │
│      trajectory               │  │                            │  │
│                               │  └────────────────────────────┘  │
│                               ├──────────────────────────────────┤
│                               │  stage      rate    ms    drop   │
│                               │  capture   47.2 Hz   16     0    │
│                               │  decode    47.0 Hz    4     0    │
│                               │  keypoints 46.8 Hz    5   0.4%   │
│                               │  depth     13.1 Hz   76    72%   │
│                               │  fusion    13.0 Hz   15     0    │
│                               │  mesh       0.1 Hz  480     —    │
│                               ├──────────────────────────────────┤
│                               │  [ save mesh ]  [ reset volume ] │
└───────────────────────────────┴──────────────────────────────────┘
```

- **A high drop percentage is not a fault** — `depth` dropping 72% of frames is
  the design working, because keypoints run at camera rate and depth cannot. The
  panel must distinguish *dropped by design* (mailbox overwrite) from *dropped
  in transport* (QoS loss), because only the second one is a problem. Two
  columns, not one.
- **Stale beats wrong.** A feed with no arrival for 2 s shows STALE rather than
  freezing on its last value looking healthy. Staleness is measured on **receipt
  time, never on `header.stamp`** — see the stamp-lag constraint in
  [CLAUDE.md](../../CLAUDE.md).
- Dark by default. This gets watched next to a terminal, and the mesh reads
  better against a dark ground.
- The two buttons call `/world/save_mesh` and `/world/reset`. Reset asks for
  confirmation; it throws away a session's work.

## Rules

- **The dashboard never computes.** It draws what the pipeline published. Any
  number it shows is a number some node measured about itself, so the dashboard
  and a `ros2 topic hz` can never disagree.
- **The dashboard is not evidence.** A gate is closed by a script that names its
  numbers, not by a screenshot of a panel — same rule as the RViz window.
- **It must be able to die.** Kill the browser, kill the node: the pipeline does
  not notice. It runs outside the component container for exactly this reason.
- Assets are vendored and versioned in-repo. No CDN — the LAN may not have
  internet, and a dashboard that breaks when DNS does is not a monitoring tool.
