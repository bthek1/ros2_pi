# Milestone I — deferred

Work that came out of [#13](https://github.com/bthek1/ros2_pi/issues/13) and is
**not executable yet**. Each entry names the trigger. See
[../README.md](../README.md) for the rules.

---

## A versioned on-disk map format

**What.** A map file with a version header, a refusal on mismatch, and a
byte-level round-trip test — the shape `test_mesh_io` already has, where the PLY
is read back **both** through the reader and byte by byte against the layout the
header promises, because writer and reader being wrong *together* round-trips
perfectly.

**Why not now.** P20 needs a map on disk and will write one, but there is
exactly one reader and one writer and they ship together. A version field whose
mismatch branch has never been taken is a branch nobody has tested — this
project's recurring finding, most recently the guard that was armed only when an
unrelated parameter was non-zero.

**Trigger.** The first time a map written by an older build has to be read by a
newer one — which is the first time the format actually changes. At that point
the refusal is written *with* the change, and the old file is kept as the test
fixture.

---

## Publish the tracking state as a diagnostic, not only a stat

**What.** `diagnostic_msgs/DiagnosticArray` on `/diagnostics` alongside the
`/pipeline/stats` row P19 adds, so standard ROS tooling can see `LOST`.

**Why not now.** Nothing in this project reads `/diagnostics` and nothing here
runs `rqt_robot_monitor` — and a topic nobody reads is where a wrong number
lives undisturbed, which is the argument this project already made about a
measured covariance on `/odom`. The dashboard is the consumer that exists.

**Trigger.** A consumer that reads `/diagnostics` rather than drawing it.

---

## Save the map points with the keyframes

**What.** A second section in the map file holding `pimesh_backend::Map`'s points
and their observations, loaded back into the backend at startup.

**Why not now.** P20 relocalises against each keyframe's *own* depth landmarks,
which travel inside the keyframe; the one reader map points have is local-map
tracking (#11's P14), which defaults off because it measured worse than P7. A file
section nothing reads is where a wrong number lives undisturbed.

**Trigger.** `local_map` defaulting to true — the day a loaded session would track
against the saved points rather than only relocalise against the keyframes.

---

## Extend a saved map in a later session

**What.** `map_load_path` and `map_save_path` together: load a room, relocalise,
and save the union — the loaded keyframes plus this session's, all in the saved
map's frame.

**Why not now.** The node refuses the combination. Its meaning is undefined until
two things are: what a new keyframe's map pose is before the first relocalisation
(there is none), and what happens to the saved keyframes' poses when this session's
pose graph would like to move them.

**Trigger.** A second recording session of the same room that needs both — i.e. a
map somebody wants to grow rather than reuse.

---

## Relocalise against this session's own keyframes, not only a loaded map

**What.** On LOST without a loaded map, search this session's keyframe store for
where the camera now is and set `map -> odom` from the answer — ORB-SLAM's
relocalisation — instead of P19's odometric recovery, which resumes from the held
pose with a fresh reference.

**Why not now.** P19's recovery is measured fast (3 depth frames on desk1) and its
cost — the map is offset by whatever motion the blackout hid — has not been
measured at all. Replacing a measured mechanism with an unmeasured one on the
strength of a textbook is the order this project does not work in.

**Trigger.** That offset measured: a blackout injected into **`dataset_node`** (P19's
lens cap lives in `decode_node`, which a TUM replay does not pass through) so a
before/after pose can be scored against motion capture on fr1/desk.

---

## Load a map beside loop closure

**What.** A loaded map and `loop_closure:=true` in one session, the pose graph
anchored on the relocalisation.

**Why not now.** Two authorities for `map -> odom`. The node refuses it.

**Trigger.** `loop_closure` defaulting to true (#12's P17 note: when a gate run on
the live camera says so).

---

## Persist the surface too

**What.** Save the TSDF — or P18's frame memory — beside the keyframes, so a
relocalised session starts with the room's surface rather than rebuilding it.

**Why not now.** P20's claim is that a room is *recognised*; nothing yet needs the
old surface back, and a 128 k-block volume is ~0.5 GB of voxels to write.

**Trigger.** A session that has to show the room before the camera has re-swept it.
