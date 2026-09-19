# The dashboard

**Built 2026-09-19** — P8, [#8](https://github.com/bthek1/ros2_pi/issues/8).
`bash tools/dashboard.sh` starts it; `http://localhost:8080` is the page;
`bash tools/gates/dashboard.sh` is what passes or fails it.

Everything below described intent until then. Where a number appears now it was
measured, and where the design was wrong the correction is in place of it rather
than beside it.

## What it is for

One browser tab that answers three questions without a ROS installation:

1. **Is the pipeline alive?** Per-stage rate, latency and drop count, with a
   stage going amber the moment it stalls.
2. **What is the camera seeing?** The RGB frame, the ORB keypoints drawn on it,
   and the colourised depth map, side by side. **Two of those three already
   exist**: `keypoint_node` has published `/keypoints/image/compressed` since P3
   and `depth_node` has published `/depth/image/compressed` since P4, both as
   ~80 kB JPEG at ~10 Hz and both colour-mapped in the node rather than in the
   viewer. That is deliberate and this page depends on it — a browser cannot
   colour-map a 3.7 MB float depth map, and a fixed `[0, max_range]` scale is what
   makes a colour mean a distance across frames instead of per frame.
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
/depth/image/... ──┼──▶  pace → frame → send      ═════▶  ├─ image strip
/world/mesh      ──┤                                      ├─ three.js scene
/odom, /tf       ──┘     serve static assets      HTTP    └─ (vendored JS)
                                                  ─────▶
```

**Library: none, and that is a change from what this section planned.** It said
a single-header C++ WebSocket/HTTP server, vendored. What is there instead is
`websocket.cpp` and `web_server.cpp` in `pimesh_dashboard` — about 500 lines
between them — because the server half of RFC 6455 is a SHA-1, a base64 and a
frame header, and a vendored library would be a third thing that has to exist,
behave identically and be packaged on **both** Ubuntu 24.04/Jazzy and
26.04/Lyrical. This workspace has already lost an afternoon to an API that
existed at both ends and meant different things.

All three pieces are pinned against published vectors in `test_websocket`: FIPS
180-4's SHA-1 vectors, RFC 4648's base64 cases, and RFC 6455's own worked
handshake example. **That is not ceremony.** The FNV-1a offset basis in this
project sat wrong for a whole milestone because a hash with a wrong constant
avalanches just as well and answers every question asked of it correctly; a hash
cannot be told it is the wrong hash by testing its behaviour. The 56-byte SHA-1
expectation in that suite was itself written from memory first, and was wrong —
the suite failed while the code was right, which is the habit working from the
other direction.

**And no three.js**, which this section also named. The 3D view is ~200 lines of
raw WebGL in `app.js`. Vendoring three.js means ~600 kB of third-party minified
JavaScript committed here, which nobody in this repo can read, review or fix, to
draw one non-indexed triangle soup and a line — the same trade this project
already made twice in writing the TSDF and marching cubes rather than taking
Open3D. If the view ever needs materials, lighting or loaders, the trade flips.

The assets — `index.html`, `style.css`, `app.js` — are installed into the
package's `share/web` and served from there, so `ros2 launch` is the only thing
anyone has to run.

**Port** 8080 by default, bound to the LAN so a phone on the same network can
watch. It is a read-only view; the only writes are two buttons (below), and they
call the same services RViz would.

## What goes over the WebSocket

One connection, binary frames, each with a one-byte channel tag. Text JSON for
anything small, binary for anything large — never base64, which costs 33% for
nothing.

| Channel | Payload | Rate | Notes |
| --- | --- | --- | --- |
| `stats` | JSON | **10.01 Hz measured** | Per-stage rate/latency/**two** drop counters/detail, plus uptime, client count and the server's own dropped-frame count |
| `rgb` | JPEG bytes | **9.16 Hz measured**, capped at 10 | The annotated keypoint frame — one image, not two, since the keypoints are drawn on the RGB |
| `depth` | JPEG bytes | **4.47 Hz measured**, capped at 5 | Colour-mapped in `depth_node` against a fixed `[0, max_range_m]`, so a colour is a distance across frames rather than within one |
| `mesh` | binary: `u32` vertex count, then vertices `f32×3` and colours `u8×3` | on change, ~every 10 s | **No index array.** A `TRIANGLE_LIST` Marker is already three vertices per triangle with no reuse, so the indices would be 0,1,2,… — a third of the payload saying nothing. ~2.1 MB measured for a 120 k-triangle surface |
| `pose` | JSON | **10.01 Hz measured** | Current camera pose, its staleness, and the trajectory tail |

**Both image rates had to be fixed after they were first measured, and the bug is
worth knowing.** `keypoint_node` already caps its preview at 10 Hz; a second 10 Hz
gate in the dashboard rejects any frame arriving a hair early, which with two
independent clocks is about half of them. Measured before the fix: **5.48 Hz** of
a 10 Hz stream and **2.80 Hz** of a 5 Hz cap on a 10 Hz stream — with every number
involved correct on its own. A cap is a ceiling on a faster source, so it has to
admit a source running at exactly its own rate; the caps now compare against 90%
of the period.

**Pacing is the server's job, not the browser's.** The node subscribes at
whatever rate the pipeline runs and sends at the dashboard's rate, dropping in
between. A slow or backgrounded browser must never apply backpressure to the
pipeline: if a client's queued bytes are over `send_limit_bytes` (8 MB), the
frame is dropped and counted, and the count is on the page. **Nothing the
dashboard does may slow the mesh down.**

**Measured** (`bash tools/gates/dashboard.sh`, quiet machine): the worst stage
moved **0.77%** with a client attached, against a **0.29%** floor between two runs
with none. The gate states the bound as that floor plus slack rather than as
P8's flat 2%, because the floor belongs to the machine and not to the pipeline —
an earlier run of that gate saw 15% between two identical no-client runs, and
that was a `colcon build` sharing the box.

**And the rule was broken once during the build, by this file's own code, in the
quietest possible way.** The first version ran the two buttons' ROS service calls
inline in the HTTP handler — which runs with the broadcast mutex held. It
deadlocked immediately, because the handler took the same non-recursive mutex to
read itself, and every later request timed out with nothing in any log. Removing
that second lock would have fixed the deadlock and left something worse: a
service call waits up to ten seconds, and holding the mutex across it blocks
`broadcast()`, which is called from the ROS callbacks. **A button press would have
applied backpressure to the pipeline.** A deadlock is a loud bug; that would have
been a silent one. Actions are now parked by the handler and run outside the lock
entirely.

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
  [CLAUDE.md](../../CLAUDE.md). Measured: the pose flag appears **2.10 s** after
  `/odom` stops, against a 2.0 s threshold.

  **A stage *row* cannot make that promise and the gate says so rather than
  pretending.** Every dev-box node publishes `/pipeline/stats` once per
  `stats_period_s`, which is 5 s, so a stage that stopped is undetectable for up
  to 5 s before the 2 s window even starts. The tight bound belongs to the pose
  channel, where `/odom` runs at 17 Hz. Lowering every node's `stats_period_s` to
  2 s would make the row bound achievable and put five more log lines a second
  into every measurement this workspace takes.
- Dark by default. This gets watched next to a terminal, and the mesh reads
  better against a dark ground.
- The two buttons call `/world/save_mesh` and `/world/reset_map` — the same
  services RViz would, with no privileged path into the map: if the surface can
  be saved from a terminal it can be saved from here, and not otherwise. Reset
  asks for confirmation; it throws away a session's work and there is no undo,
  because the volume is process memory and nothing else holds a copy.

  They are a `POST /action/<name>`, not a WebSocket message. A command channel
  multiplexed into the same socket as five telemetry streams is a place for a
  stray frame to become an action; an HTTP POST is one request, one answer, and
  it appears in any log between here and the browser as what it is. The browser
  never chooses the save path — it names the action and the node supplies the
  directory.

## Rules

- **The dashboard never computes.** It draws what the pipeline published. Any
  number it shows is a number some node measured about itself, so the dashboard
  and a `ros2 topic hz` can never disagree.

  That rule pays for itself in an unexpected place: it is what lets
  `tools/gates/dashboard.sh` read the pipeline's rates with `ros2 topic echo`
  without the usual objection. The rule against `ros2 topic hz` is about an
  instrument whose own scheduling lands in the number it reports — and a
  transport carrying `rate_hz` values some other node already computed cannot
  influence them.
- **The dashboard is not evidence.** A gate is closed by a script that names its
  numbers, not by a screenshot of a panel — same rule as the RViz window.
- **It must be able to die.** Kill the browser, kill the node: the pipeline does
  not notice. It runs outside the component container for exactly this reason,
  and `test_the_dashboard_is_opt_in_and_runs_outside_the_container` is what stops
  the one exception quietly becoming the rule. `gates/dashboard.sh` kills the
  client at the halfway mark of the clip and compares the seconds after against a
  control run's same seconds.
- Assets are vendored and versioned in-repo. No CDN — the LAN may not have
  internet, and a dashboard that breaks when DNS does is not a monitoring tool.
