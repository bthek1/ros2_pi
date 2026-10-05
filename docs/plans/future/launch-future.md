# Launch plan — deferred

Companion to [#15](https://github.com/bthek1/ros2_pi/issues/15). Everything here
is waiting on something; each entry names the trigger that would make it a phase.
When a trigger fires, delete the entry here and append it to #15's body as the
next unused phase number, with a test.

## Port the remaining gates to `launch_testing`

**What.** Move `gates/{keypoints,depth,fusion,mesh,odom,dashboard,map,ba,scale}.sh`
into `launch_testing` tests, the way P3 moved `trajectory.sh`. Each keeps its
controls and refusals.

Milestones H and I (closed 2026-10-02) added
`gates/{place,loop,rebuild,lost,relocalise}.sh` to the list. `relocalise.sh` runs
two containers in sequence — save a map, load it in a fresh one — so port it
after a single-container gate has shown the method works.

**Why not now.** Nobody knows yet whether a port gives the same numbers. The
gates' value is their controls, and a pytest can be green over broken behaviour
exactly as a bash script can.

**Trigger.** #15 P3 annotated done with parity shown: the pilot's ATE and scale
inside the bash gate's three-run spread, and every control refusing in both.

**Status, 2026-10-05: not fired.** P3 ran and every control refused in both, but
the pilot's Sim(3) ATE spread 0.169–0.513 m against the bash gate's 0.386–0.415 m
over the same three alternated hours, so the ATE was not inside the spread. The
next step is to find out *why* the pilot spreads wider when both run the same
launch — until that is known, porting more gates would carry the difference along
with them.

## Publish the `stats` lines as typed messages

**What.** `odometry_node`'s `stats regime=`, `stats map` and `stats backend`
lines become fields on a `.msg`. Gates subscribe to them instead of
`grep | awk`-ing the log, and `test_gate_contract` is deleted because a renamed
field then fails to build.

**Why not now.** It changes a node's interface in the middle of milestone G,
and gates that still parse the log would need both paths at once.

Since 2026-10-05 `lost.sh` and `relocalise.sh` also `grep -o` facts out of
*free-text* log lines (`map loaded: keyframes=`, `last_black_ns=`), which
`test_gate_contract` covers by rendering the format strings. Those are part of
the same move.

**Trigger.** The P3 pilot's diff shows log parsing as a separable part of the
port, and milestone G's gates (`map.sh`, `ba.sh`) are either closed or ported.

## `launch_pytest` instead of `launch_testing`

**What.** Rewrite the system tests as `launch_pytest` fixtures, which are closer
to ordinary pytest than `launch_testing`'s unittest style.

**Why not now.** It is installed on neither machine (checked 2026-09-30). Apt
has it at both ends (`ros-lyrical-launch-pytest` 3.9.8,
`ros-jazzy-launch-pytest` 3.4.11), but installing needs `sudo`, which only a
person has.

**Trigger.** `ros2 pkg prefix launch_pytest` succeeds on both machines.

## `camera_node` as a systemd unit on the Pi

**What.** The dev box starts and stops the Pi's camera with
`ssh pi systemctl --user {start,stop} pimesh-camera`. systemd then owns the
process tree, and the login-shell/`timeout`/`ros2 run` wrapper chain that
`PIMESH_PI_WRAP_PAT` exists to catch goes away.

**Why not now.** P0's `session.sh` keeps today's `kill_pi`, which is measured.
Nothing yet shows it failing under launch.

**Trigger.** `gates/teardown.sh` fails a Pi-side case against a launch-based
recipe in #15 P0 or P1 in a way `kill_pi` cannot fix.
