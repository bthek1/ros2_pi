# Milestone G — deferred

Work that came out of [#11](https://github.com/bthek1/ros2_pi/issues/11) and is
**not executable yet**. Each entry names the trigger. See
[../README.md](../README.md) for the rules.

---

## Global bundle adjustment over the whole map

**What.** A full BA over every keyframe and map point, run rarely, rather than
the covisible window P15 optimises every keyframe.

**Why not now.** It is an optimisation of a stage that does not exist, and the
error it removes may not be the error that dominates. Local BA removes *local*
inconsistency; accumulated drift around a loop is removed by a **pose graph**,
which is P17 and is far cheaper. Doing global BA before P17 would be paying the
expensive fix for the problem the cheap one solves.

**Trigger.** P17 green **and** `gates/trajectory.sh` still showing ATE dominated
by accumulated drift rather than by local error — which is exactly the
distinction RPE-over-1 s versus ATE separates, and P11 prints both.

---

## GTSAM / iSAM2 instead of g2o batch solves

**What.** Incremental smoothing rather than re-solving the window each time.

**Why not now.** g2o is installed at both ends from ROS itself (2026-09-23) and
GTSAM is not in either apt, so this trades a zero-cost dependency for a
submodule or a source build on **two** distros — the cost this workspace keeps
paying and the reason `cv2.aruco` version-branching exists. And re-solving a
window of ten keyframes is not obviously slow.

**Trigger.** `gates/ba.sh` reporting BA cost per keyframe above the budget it
sets, with the window at its intended size. Measure first.

---

## Learned features instead of ORB

**What.** SuperPoint or a similar detector-descriptor through ONNX Runtime,
replacing ORB in the tracker.

**Why not now.** ORB is measured at **5.75 ms/frame** with depth beside it,
against an 8 ms budget, and it is not the bottleneck — depth at 55.10 ms is the
pipeline's clock. Swapping the feature front end before the map and the backend
exist would change the thing every later measurement is compared against, and
P14's whole value is a before-and-after on the same features.

**Trigger.** `gates/map.sh` showing tracking failing for want of *matches*
rather than for want of geometry — a local-map size that is healthy with an
inlier count that is not. The GPU stack for it already exists (P4), which is
what makes this cheap when the trigger fires.
