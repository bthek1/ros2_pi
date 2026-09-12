# Milestone B — deferred

Companion to [issue #5](https://github.com/bthek1/ros2_pi/issues/5) (phases
P2–P3). Work that came up while building the container, the decode stage and the
ORB tracker, and that is **not executable yet**.

Every entry names **the trigger that would make it executable**. When a trigger
fires, the entry is deleted from this file and appended to issue #5's body as the
next unused phase number, with a test. Moving work into the plan is the only way
it gets built; moving it here is the only way it gets deferred. It never sits in
both.

---

## Settle `decode_node`'s reader QoS: RELIABLE or BEST_EFFORT

`input_reliability` is a parameter with a conservative default (`reliable`) and no
measurement behind it. The question is the one `rviz/camera.rviz` already answered
for a *viewer* and which does not transfer: a RELIABLE reader delivers in
sequence, so one lost fragment head-of-line blocks every frame behind it for a
heartbeat round trip — measured at 10 gaps over 50 ms in 20 s against
BEST_EFFORT's 3 — while a BEST_EFFORT reader simply loses the frame.

The instrument exists: `decode_node`'s stats line counts inter-arrival gaps over
`gap_threshold_ms`, and the parameter makes the experiment a launch argument
rather than a rebuild. What is missing is the other half of the trade-off.

**Trigger: P4, depth.** A dropped frame has no cost today — at 46 Hz in and 46 Hz
out, `decode_node` has nothing to fall behind. Once depth runs at ~13 Hz and the
mesh is built from what reaches it, "lost three frames" and "stalled for 490 ms"
have different consequences and the comparison has a unit. Measuring it now would
produce two gap counts and no way to prefer either.

---

## Stop paying the decode copy

`decode_node` decodes into a reused `cv::Mat` and then copies 2.7 MB into the
message, because publishing a `unique_ptr` means a fresh allocation per frame and
`cv::imdecode` has nowhere to write except its own buffer. One pass over 2.7 MB,
inside a measured 1.90 ms total.

Two ways out, both real work: decode straight into the message's `data` vector
once the frame size is known (`cv::imdecode`'s `dst` overload reuses a correctly
sized buffer, so a `cv::Mat` header over `msg->data` would do it, at the cost of
having to get the size right *before* decoding), or a `rclcpp::TypeAdapter` so the
published type is a `cv::Mat` and the conversion happens only for out-of-process
subscribers.

**Trigger: decode becoming a cost worth attacking.** It is 1.90 ms of a 17 ms
frame interval and ~1/4 of the stage's own time; depth will cost 40 times as much.
Optimising the cheapest stage in the pipeline first is how a budget gets spent on
the wrong thing. Revisit if `gates/ipc.sh` ever reports decode over ~4 ms, or if
the dev box stops keeping up with 59 Hz.

---

## Make the dev box enforce C++17, rather than relying on the Pi's build to fail

Every `CMakeLists.txt` here sets `CMAKE_CXX_STANDARD 17` and says why: the Pi
builds the same sources under Jazzy, whose baseline is 17. Measured 2026-09-12,
**that setting does nothing on the dev box**. Lyrical's `ament_cmake_ros_core`
exports an INTERFACE target requiring `cxx_std_20`
(`ament_ros_defaults.cmake`: `target_compile_features(ament_ros_cxx_standard
INTERFACE cxx_std_20)`), and a compile *feature* requirement raises the standard
above what the project asked for. So the dev box compiles everything as
`-std=c++20` and the Pi compiles it as `-std=c++17`.

The consequence is the dangerous direction of the cross-distro split: a C++20
feature compiles here and is only discovered at the far end of an `rsync`. Nothing
is broken today — `bash tools/build-pi.sh` builds all five packages — but the
enforcement is a five-minute remote compile rather than a local compiler error.

**Trigger: the first time the Pi's build fails on a standard-level error**, or
sooner if a cheap local guard turns up. It is not obviously cheap: the requirement
arrives through rclcpp's interface, so suppressing it means either filtering the
INTERFACE property off an imported target or compiling a canary translation unit
with an explicit `-std=c++17` and no ROS includes — and a guard that is wrong
about which APIs matter would be worse than the honest remote build.

---

## `component_container_isolated`'s executor model is chosen, not measured

The container was switched from `component_container_mt` to
`component_container_isolated` on 2026-09-12 — `_mt` is deprecated on Lyrical, and
the isolated one gives each component its own executor instead of one shared
multi-threaded pool. The argument for it is that P4's 76 ms depth callback cannot
then delay another node's callbacks. That argument is sound and **unmeasured**:
both gates were re-run to confirm intra-process comms still hands the pointer
over, and nothing compared the two containers' scheduling behaviour under a
callback that costs 76 ms, because there is no such callback yet.

**Trigger: P4.** With depth in the container, the comparison is one launch
argument and a latency histogram per stage: whether a 76 ms inference delays
`keypoint_node`'s 6.7 ms frames, and by how much, under each container. Until then
there is nothing slow enough in the process for the difference to show up in.

---

## `decode_node` and `keypoint_node` publish `PipelineStats`

Both log a stats line that nothing can read programmatically; `gates/keypoints.sh`
parses a log message to get the per-frame cost, which works and is not a
structured interface. `pimesh_msgs/PipelineStats` exists for this.

**Trigger: P7, the dashboard** — the same trigger as `camera_node`'s entry in
[milestone-a-future.md](milestone-a-future.md), and for the same reason: the
message's only consumer is the dashboard, and publishing it earlier means a topic
with no subscriber and a rate chosen by guesswork.
