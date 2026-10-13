# Viewers

What to look at in `just run`, and what each part of the picture tells you. **A
viewer is not evidence**: every claim it makes visible is closed by a gate in
`tools/gates/`, named below. This page is here so a person can see the thing
exists, and recognise the failure shapes on sight.

`just run` is one launch file, `pimesh_bringup/launch/view.launch.py view:=run`
(its entry in `pimesh_bringup/config/views.yaml`), run through `tools/session.sh`
so that the Pi is torn down and checked when the session ends. A session ends on
Ctrl-C, on a closed terminal, on a closed RViz window, or after its `seconds`.
`launch` prints one line naming the source and pointing here.

Until 2026-10-13 there were eight viewers, one per stage (`view-camera`,
`view-keypoints`, `view-depth`, `view-odom`, `view-map`, `view-mesh`, `replay`,
`dashboard`). They were removed when the justfile was cut to `just run`; the
build log and the closed issues still name them.

## run

`just run [seconds] [bag] [name:=value ...]` — monocular visual SLAM, camera to
surface. With no bag, the Pi's live camera; with one, the bag **once**, not
looped. Everything after the bag is a `pimesh.launch.py` argument
(`ros2 launch pimesh_bringup view.launch.py -s` lists them):

| Command | What runs |
| --- | --- |
| `just run` | The SLAM default: tracking, place recognition, **loop closure** (`map -> odom` is the pose graph's correction), the LOST refusal, and the TSDF **rebuilt** at corrected poses. |
| `just run 600 desk1` | The same on the reference clip. |
| `just run 600 desk1 loop_closure:=false` | Visual odometry alone, `map -> odom` identity: the control `gates/loop.sh` compares against. |
| `just run 600 "" dashboard:=true` | Plus the browser page on `http://localhost:8080` (below). |

Closed by `bash tools/gates/mesh.sh` (P6, the surface), `odom.sh` (P7),
`trajectory.sh` (P11, ATE against motion capture), `loop.sh` (P17),
`rebuild.sh` (P18), `lost.sh` (P19) and `relocalise.sh` (P20).

**This is the most seductive false positive in the project.** A sealed box with no
openings looks *more* finished than a correct scan, and a room can be the right
shape and the wrong size with nothing on screen saying so.

| Display | What you should see |
| --- | --- |
| Mesh | A triangle surface on `/world/mesh`, growing every ten seconds. Capped at 120 000 triangles by quadric decimation, so it looks coarser than the saved PLY — that is the Marker's budget, not the map's resolution. |
| Odometry | Arrows on `/odom`, the last 500 kept, so the trail *is* the trajectory: an arc a hand could have made, through the surface. |
| Depth | The depth map against a fixed [0, max_range_m] inferno scale — near bright, the 6 m clip black — so a colour is a distance. Black in the middle of the frame is the absence of a reading. |
| DepthCloud | Off by default. Enabled, the live frame sits **on** the mesh rather than in front of or behind it; an offset is drift. |
| TF | `map -> odom -> base_link -> camera_*`. `map -> odom` is published by `odometry_node`: identity until a loop closes, the pose graph's correction after. |

Fixed Frame is **map**, not `base_link`: the mesh lives in the map frame, and
viewing it from a camera-attached frame makes a static room appear to swim.

**Give it half a minute.** `depth_node` loads a 99 MB model and warms a CUDA
session, fusion needs frames before there is a volume, and `mesh_node` extracts on
a 10 s timer — RViz is started eight seconds in for that reason.

**What the failure shapes look like.** Doubled walls are drift, or a scale that
breathed between keyframes — and with loop closure on, they should *close up* when
a revisit is found and the volume is rebuilt (a visible jump in the surface, not a
fault); a sealed box is a mesher closing the frontier; a
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

### With `dashboard:=true`

The whole pipeline in a browser tab at `http://localhost:8080` as well. Closing the
tab ends nothing. `dashboard_node` runs in its own process, outside the
container, because it must be able to die. Closed by
`bash tools/gates/dashboard.sh` (P8); the layout is [dashboard.md](dashboard.md).

| Part | What it tells you |
| --- | --- |
| the table | One row per stage, off `/pipeline/stats`. **Two drop columns, never one**: "by design" is a mailbox overwrite — depth dropping two frames in three is its single-slot mailbox working — and "lost" is a fault. |
| STALE | A row with nothing for two seconds says so rather than freezing on its last value. Measured on receipt, never on a stamp. |
| the strips | The annotated camera frame and the depth map against a fixed scale, so a colour is a distance across frames. |
| the scene | The surface with the trajectory through it, oldest dim, newest bright, and LOST flagged when tracking is lost. Drag to orbit, wheel to zoom, shift-drag to pan. |
| footer | Frames dropped to slow clients — 0 with one browser on a LAN. It climbs if the tab is backgrounded, and the pipeline's rates must not move when it does. |

On a bag the capture row does not appear: `camera_node` on the Pi publishes it.

## record

`bash tools/record.sh <name> [seconds]` — a clip from the Pi's camera into `bags/<name>`,
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
Look at it before trusting it: `just run 600 <name>`.
