# Milestone E — deferred

Work that came out of [#8](https://github.com/bthek1/ros2_pi/issues/8) and is
**not executable yet**. Each entry names the trigger that would make it a phase.
When a trigger fires, the entry is deleted from here and appended to the issue
body as the next unused phase number, with a test that is a command. It never
sits in both.

---

*Promoted 2026-09-23 to **P13** in [#10](https://github.com/bthek1/ros2_pi/issues/10) — the trigger was always
a person and the camera. Deleted from here. **P12 wants the same visit**:
the tape measure that pins `depth_scale`.*

---

*Promoted 2026-09-23 to **P16** in [#12](https://github.com/bthek1/ros2_pi/issues/12), which is the whole
loop-closure plan — place recognition, a pose graph that owns `map -> odom`,
and a volume rebuilt at the corrected poses. Its trigger was a clip that
revisits somewhere, which P13 records. Deleted from here.*

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

---

## A measured covariance on `/odom`

`keypoint_node` publishes `unconstrained_covariance()` — a large diagonal that
says "this dimension is unconstrained" and is deliberately an overstatement of
ignorance. It is **not a measurement**, and the header says so. It replaced a
`-1` sentinel borrowed from `sensor_msgs/Imu` that made the matrix invalid
(2026-09-23); the large diagonal is the least-wrong of the three legal answers,
not a right one.

A real one is computable: PnP's covariance comes from the Jacobian of the
reprojection residual at the solution, scaled by the inlier residual variance —
`cv::solvePnPRansac` does not report it, but `cv::projectPoints` returns the
Jacobian and the normal-equation inverse is a 6x6 solve over the inlier set.
That would make the number mean something, and would let the two regimes be
compared on their *confidence* as well as their trajectories.

**Trigger: the first consumer that fuses `/odom` rather than drawing it.**
Today RViz draws it, `odom_probe` measures the trajectory and the dashboard
shows the pose — none of them reads the covariance, so a real one would be
arithmetic nobody checks, which is how a wrong number survives. A fusion filter
downstream is what makes it load-bearing and therefore testable.
