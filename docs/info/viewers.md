# Viewers

What to look at in each `just view-*` session, and what each part of the picture
tells you. **A viewer is not evidence**: every claim a view makes visible is
closed by a gate in `tools/gates/`, named under each heading. These pages are here
so a person can see the thing exists, and recognise the failure shapes on sight.

Every viewer is one launch file, `pimesh_bringup/launch/view.launch.py`, with a
`view:=` argument choosing an entry of `pimesh_bringup/config/views.yaml`, run
through `tools/session.sh` so that the Pi is torn down and checked when the
session ends. A session ends on Ctrl-C, on a closed terminal, on a closed RViz
window, or after its `seconds`. `launch` prints one line naming the source and
pointing here.

Arguments `pimesh.launch.py` declares pass straight through to the pipeline —
`ros2 launch pimesh_bringup view.launch.py view:=mesh loop_closure:=true` is a
mesh view with loop closure on. `ros2 launch pimesh_bringup view.launch.py -s`
lists them all.

## view-mesh

`just view-mesh [seconds] [bag]` — the room becoming a triangle surface. With no
bag, the Pi's live camera; with one, the bag **once**, not looped. Closed by
`bash tools/gates/mesh.sh` (P6), whose evidence is the three PNGs
`tools/eval/mesh-views.sh` renders.

**This is the most seductive false positive in the project.** A sealed box with no
openings looks *more* finished than a correct scan, and a room can be the right
shape and the wrong size with nothing on screen saying so.

| Display | What you should see |
| --- | --- |
| Mesh | A triangle surface on `/world/mesh`, growing every ten seconds. Capped at 120 000 triangles by quadric decimation, so it looks coarser than the saved PLY — that is the Marker's budget, not the map's resolution. |
| DepthCloud | The live frame, overlaid on the surface it is being fused into. The cloud sitting **on** the mesh rather than in front of or behind it is the thing to look at: an offset is drift. |
| TF | `map -> odom -> base_link -> camera_*`. The camera moves through the volume (6-DoF odometry since P7). `map -> odom` is identity unless `loop_closure:=true`. |

Fixed Frame is **map**, not `base_link`: the mesh lives in the map frame, and
viewing it from a camera-attached frame makes a static room appear to swim.

**Give it half a minute.** `depth_node` loads a 99 MB model and warms a CUDA
session, fusion needs frames before there is a volume, and `mesh_node` extracts on
a 10 s timer — RViz is started eight seconds in for that reason.

**What the failure shapes look like.** Doubled walls are drift, or a scale that
breathed between keyframes; a sealed box is a mesher closing the frontier; a
visible hitch every ten seconds is extraction starving the pipeline. On
`bags/desk1`, nothing is fused for a few seconds around 17–21 s and 24–25 s: the
tracker is LOST there on every run, and `fusion_node` refuses LOST frames (P19).
That is the refusal working, not a stall.

**The scale is metres** since P12 (`depth_scale` 4.6002 off a tape measure). Do
not adjust it by eye against a mesh that looks about right — a blank wall in view
defeats the depth network and will make it look wrong when it is not.

Save the surface at full detail, from another terminal:

```bash
ros2 service call /world/save_mesh pimesh_msgs/srv/SaveMesh "{path: '/tmp/room.ply'}"
bash tools/eval/mesh-views.sh /tmp/room.ply
```

Throw the map away and start again:

```bash
ros2 service call /world/reset_map pimesh_msgs/srv/ResetMap "{}"
```

A bag plays once and then the surface stops growing. That is the end of the clip,
not a crash; the window stays until it is closed, Ctrl-C, or `seconds`.

## view-camera

`just view-camera [seconds]` — the Pi's camera, live, with the frame tree beside
it. Closed by `bash tools/gates/capture.sh` (P1). The pipeline runs too, as it
always has for this recipe, so `odom -> base_link` moves.

| Display | What you should see |
| --- | --- |
| Image | Live video from the C922, at roughly the rate `gates/capture.sh` printed. Grey means the LAN hop or the QoS match is broken, not the camera — check `ros2 topic hz /image_raw/compressed` first. |
| TF | `base_link -> camera_link -> camera_optical_frame`, the optical frame rotated into z-forward: its blue axis points the way the camera looks, its red axis to the image's right. |

Fixed Frame is `base_link`.

This window is a *second* subscriber on the only topic that crosses Wi-Fi. That
is what the architecture forbids at scale, tolerable here only because a
compressed frame is ~86 kB and RViz asks for BEST_EFFORT. Close it before
measuring anything.

## view-replay

`just replay <bag> [seconds]` — a recorded bag, **looping**, with the static frame
tree and no pipeline (`pipeline:=false`). The one view that loops, and it can
because it publishes no pose: a looping bag sends every stamp back ~60 s at each
wrap, and a pose published from those stamps freezes and floods every TF listener.

| Display | What you should see |
| --- | --- |
| Image | The recorded video, at the rate it was recorded. Grey means the replay is not publishing — check `ros2 topic hz /image_raw/compressed` before suspecting RViz. |
| TF | `base_link -> camera_link -> camera_optical_frame`, from `pimesh.launch.py` rather than from the bag: these recordings carry no `tf_static`. |

Fixed Frame is `base_link`. `map -> odom` is the static identity here — the one
case it still is — and `odom` has no edge to `base_link`, which RViz reports. That
is correct: the edge is `odometry_node`'s, and nothing downstream of capture runs.
The pipeline on the same clip is `just view-keypoints 600 <bag>`.

The intrinsics are whatever was true when the bag was recorded; a bag recorded
before the calibration carries the nominal placeholder. `ros2 topic echo --once
/camera_info` during playback is the only honest way to know which.

## view-keypoints

`just view-keypoints [seconds] [bag]` — ORB corners on the preview and the pose
they produce. Closed by `bash tools/gates/keypoints.sh` (P3); the pose by
`bash tools/gates/odom.sh` (P7).

| Display | What you should see |
| --- | --- |
| Keypoints | A few hundred circles. **Green was already being followed, yellow is new this frame.** Mostly green with a scatter of yellow is the tracker working; mostly yellow is it detecting corners and recognising none. They cluster on texture and are absent on blank wall. |
| TF | `camera_optical_frame` turning *and* moving as the camera does — 6-DoF since P7. |

Fixed Frame is `odom`, because `base_link` is the frame that moves.

When the solve fails the node **holds** the last pose, and after five held depth
frames it reports LOST on `/tracking/state` (P19). During a fast flick the axes
stop rather than jump.

## view-depth

`just view-depth [seconds] [bag]` — monocular depth from the GPU, as a coloured
cloud of the room. Closed by `bash tools/gates/depth.sh` (P4).

| Display | What you should see |
| --- | --- |
| DepthCloud | A recognisable shell of the room, coloured by the camera image, **in front of** `camera_optical_frame`. A desk edge nearer than the wall behind it. A flat plane at one distance means the reciprocal or the clip is wrong. |
| Depth | The same map with **inferno over a fixed 0–6 m** — near yellow, mid orange and red, far black. The scale does not move between frames, so a colour is a distance; large black regions are the 6 m clip, this pipeline's "far away or don't know". |
| TF | `camera_optical_frame` with z into the scene. A cloud rotated 90° from those axes means something re-derived the optical convention instead of naming the static edge. |

Fixed Frame is `odom`. The scale is metres since P12. A blank wall in view is the
worst case: the network returns something smooth, confident and wrong. The cloud
appears a few seconds after the preview, while `depth_node` loads its model.

## view-odom

`just view-odom [seconds] [bag] [odom_regime]` — the camera's trajectory, and the
recipe meant to be run twice: `odom_regime` is `sixdof` (the default) or
`rotation_only`, the control. Closed by `bash tools/gates/odom.sh` (P7); measured
against motion capture by `bash tools/gates/trajectory.sh` (P11).

| Display | What you should see |
| --- | --- |
| Odometry | Arrows on `/odom`, the last 500 kept, so the trail *is* the trajectory. In `sixdof` an arc a hand could have made; in `rotation_only` every arrow at the origin, only turning — bearing rays cannot see translation, and the pose says so rather than guessing. |
| Marker | The surface on `/world/mesh`, which the trajectory is judged by. |
| DepthCloud | The live frame over the surface. Sitting **on** the mesh, not in front of or behind it; an offset is drift. |
| TF | `map -> odom -> base_link -> camera_*`, `base_link` moving. |

Fixed Frame is `map`. Run the same clip both ways side by side. On `bags/desk1`,
a pan, the two are a dead heat on the surface gap; `bags/walk1` is the clip that
separates them (P13).

## view-map

`just view-map [seconds] [bag] [local_ba]` — map points and the trail, with
tracking against the local map (`local_map:=true`) and bundle adjustment on unless
`local_ba` is `false`, the control. Gated by `bash tools/gates/map.sh` and
`bash tools/gates/ba.sh` (P14, P15), both of which still fail on their ATE.

| Display | What you should see |
| --- | --- |
| MapPoints | Orange, on `/map/points`, republished at most once a second when a keyframe changes the map. On the room's surfaces; a haze in depth around each is the depth network's ~15% keyframe-to-keyframe scale disagreement. |
| Odometry | The last 500 poses on `/odom` — the trail through the points. |
| Marker | The TSDF surface. Points *on* the mesh are a map that agrees with the fusion. |
| TF | `map -> odom -> base_link -> camera_*`. `map -> odom` is published by `odometry_node`: identity, unless the session adds `loop_closure:=true`. |

Fixed Frame is `map`. **On TUM fr1/desk the local map tracks worse than P7's
newest-keyframe tracker** (`gates/map.sh`); this window shows the map, not which
tracker is better.

## view-dashboard

`just dashboard [seconds] [bag] [dashboard_port]` — the whole pipeline in a
browser tab at `http://localhost:8080`, with no RViz: it ends on Ctrl-C, a closed
terminal or `seconds`, never on a closed tab. `dashboard_node` runs in its own
process, outside the container, because it must be able to die. Closed by
`bash tools/gates/dashboard.sh` (P8); the layout is
[dashboard.md](dashboard.md).

| Part | What it tells you |
| --- | --- |
| the table | One row per stage, off `/pipeline/stats`. **Two drop columns, never one**: "by design" is a mailbox overwrite — depth dropping two frames in three is its single-slot mailbox working — and "lost" is a fault. |
| STALE | A row with nothing for two seconds says so rather than freezing on its last value. Measured on receipt, never on a stamp. |
| the strips | The annotated camera frame and the depth map against a fixed scale, so a colour is a distance across frames. |
| the scene | The surface with the trajectory through it, oldest dim, newest bright, and LOST flagged when tracking is lost. Drag to orbit, wheel to zoom, shift-drag to pan. |
| footer | Frames dropped to slow clients — 0 with one browser on a LAN. It climbs if the tab is backgrounded, and the pipeline's rates must not move when it does. |

On a bag the capture row does not appear: `camera_node` on the Pi publishes it.

## record

`just record <name> [seconds]` — a clip from the Pi's camera into `bags/<name>`,
recording `/image_raw/compressed` and `/camera_info`. Not a viewer, but the same
shape: `record.launch.py` under `tools/session.sh`. It resets the camera's V4L2
controls first (`tools/calib/camera-reset.sh`, because a clip recorded under a
stale manual exposure cannot be un-recorded), starts the camera, and starts the
recorder **on the first frame**, so `seconds` is seconds of room. It refuses a
name that already exists — a reference clip is recorded once; delete it
deliberately — and fails if the recording has no `metadata.yaml` at the end.

What makes a clip useful to the phases that replay it:

- **Sweep slowly.** ORB matches corners between consecutive frames; a fast flick
  blurs them and the tracker holds through the whole turn. A sweep that takes the
  full minute to cross the room is not too slow.
- **Texture, not blank wall.** Corners land on edges, print, clutter and
  furniture, and a blank wall defeats the depth network as well.
- **Come back to where you started.** Place recognition and loop closure need a
  revisit to find.
- **Move, as well as turn**, if the clip is for odometry: a pan is explained by
  rotation alone (`bags/desk1`'s lesson; `bags/walk1` is the clip that is not).
- **Keep the room still.** A person walking through the clip is a moving object
  in a map that assumes the world is static.

It prints the topics, message counts, duration, image rate and each file's
sha256 at the end — `bags/` is git-ignored, so that hash is the clip's identity.
Look at it before trusting it: `just replay <name>`.
