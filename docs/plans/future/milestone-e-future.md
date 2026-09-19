# Milestone E — deferred

Work that came out of [#8](https://github.com/bthek1/ros2_pi/issues/8) and is
**not executable yet**. Each entry names the trigger that would make it a phase.
When a trigger fires, the entry is deleted from here and appended to the issue
body as the next unused phase number, with a test that is a command. It never
sits in both.

---

## A clip with deliberate translation — `bags/walk1`

**The trigger is a person and the camera.** Everything else here is software.

P7 asks its gate to assert that 6-DoF odometry makes the paired-surface gap
smaller than rotation-only odometry's. On `bags/desk1` it cannot be asserted,
and `tools/gates/odom.sh` prints the comparison instead and says so in its own
output. The reason is the clip rather than the estimator:

- `desk1` is a **pan**. A hand sweep about the wrist carries ~0.9 m of arm arc
  against 2-3 m of scene, so rotation already explains most of the frame motion
  and the residue is small.
- That residue is dominated by the depth network, not by the pose. Depth
  Anything V2 estimates *relative* depth: its scale breathes a few percent a
  frame — `fusion_node`'s aligner hits its own 15% clamp on one frame in seven of
  this clip — and its *shape* changes with viewpoint, so the same wall comes back
  at a different distance however well the camera is posed.

A slow walk around the room, camera held level and moving metres rather than
centimetres, is a clip rotation-only odometry has no way to explain. Record it
the way every other clip here is recorded:

```bash
bash tools/record-clip.sh walk1 60
```

**When it exists**, the phase is: run `bash tools/gates/odom.sh walk1`, and if
the 6-DoF run's median paired-surface gap is smaller than the control's, promote
that from a printed comparison to an assertion — in `odom.sh`, keyed to the clip,
with both numbers in the issue. If it is *not* smaller on a clip that carries
real translation, that is a much more interesting result than the one `desk1`
gives and it points at the depth network rather than at the pose.

**Also record what the room really measures while you are there**, because the
other deferred item that needs a person is in
[milestone-d-future.md](milestone-d-future.md) and wants the same visit: a tape
measure and a surface at a known distance, to pin `depth_scale`. Until that
number exists every distance in this project — including the trajectory lengths
P7 prints — is plausibly shaped and in an unknown unit.

---

## Consume the keyframe store for loop closure

**Trigger: `bags/walk1` above, or any clip where the camera returns to a place it
has already been.** A pan from one spot never revisits anything, so there is no
loop to close on `desk1` and a relocaliser could not be shown to work on it.

P7 built the store and — contrary to the phase's own text, which said nothing
would read it — the odometry now reads the newest keyframe as the view each frame
is posed against. What is still missing is the *other* reader: matching a frame's
descriptors against **every** keyframe rather than the newest, recognising a place
seen minutes ago, and correcting `map -> odom` by the discrepancy.

The store already holds what that needs: descriptors, bearing rays, 3D landmarks
and the pose each was taken at, ~40 kB per keyframe. The pieces not yet written
are a descriptor index (brute force over a few hundred keyframes is likely enough
to start), a geometric check on the candidate match, and a pose-graph optimiser.
`map -> odom` is published as static identity today precisely so that this can
become real without moving the surface underneath the frames —
`config/pimesh.yaml` says so where the edge is declared.

Deferred rather than done because a loop closure that has never closed a loop is
untestable, and this project's rule is that a phase ends in a command that exits
non-zero.

---

## Bound the drift against something outside the pipeline

**Trigger: a second source of pose.** Everything P7 asserts about the trajectory
is either internal consistency (the fit's reprojection error) or plausibility (a
camera cannot cross what it can see in a tenth of a second). Neither is an
accuracy claim, and the gate does not make one.

What would make it one, cheapest first:

- **A closed loop by hand.** Start and end the clip at the same marked spot,
  measured, and assert the trajectory returns to within some fraction of its own
  path length. That needs only a tape measure and a floor mark — but it needs
  `bags/walk1` first, because a pan that never leaves one spot returns to it
  trivially.
- **A ChArUco board in the scene.** The board this project already owns, left in
  view, gives an independent pose per frame through `cv::solvePnP` at a *known*
  scale — which would pin `depth_scale` and bound the odometry in one measurement.
  The board is on a wall with ~1.6 mm of bow in it and would want mounting on
  something flat first; see the calibration entries in
  [#9](https://github.com/bthek1/ros2_pi/issues/9).

---

## Recover the rotation rate that `sixdof` gives up

**Trigger: a measurement showing it costs something.** In `rotation_only` the
pose is published at the camera's rate, ~58 Hz, because bearing rays are fitted
on every frame. In `sixdof` it is published at the depth rate, ~17 Hz, because
that is when a pose is estimated — and it is stamped at the depth frame's own
stamp, which is exactly the set of stamps `fusion_node` looks up.

So nothing in the pipeline is currently short of a pose. What is lost is
*between* those stamps: any future consumer wanting a pose at an arbitrary
instant gets tf2's interpolation across a 57 ms gap instead of a measurement.

The obvious fix is to keep the ray-based rotation chain running at the camera's
rate and publish it between depth frames, with the translation held. It is
deliberately **not** done, because it means two things writing one pose and the
first version of it measured worse: publishing at the image rate with whatever
the pose worker last wrote is smooth and wrong by one depth interval, which at a
20 deg/s pan is 1.1 degrees of stale orientation — about 6 cm of surface error at
3 m, larger than the translation P7 exists to add.
