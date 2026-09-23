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
