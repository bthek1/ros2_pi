# Milestone A — deferred

Companion to [issue #4](https://github.com/bthek1/ros2_pi/issues/4) (phases
P0–P1). Work that came up while building the workspace skeleton and the capture
node, and that is **not executable yet**.

Every entry names **the trigger that would make it executable**. When a trigger
fires, the entry is deleted from this file and appended to issue #4's body as
the next unused phase number, with a test. Moving work into the plan is the only
way it gets built; moving it here is the only way it gets deferred. It never
sits in both.

---

## Calibrate the C922, and publish `camera_info` from the result

`camera_node` publishes `camera_info` with **nominal** intrinsics — `fx = fy =
907`, principal point at the image centre, zero distortion — carried in
`camera_matrix` and `distortion_coefficients` parameters, with a `calibrated`
flag that is false and a WARNING on every startup saying so. That is honest and
it is not good enough for a pipeline that unprojects every pixel: a plumb_bob
model with all-zero coefficients is a claim that a consumer webcam has no
distortion, and the error it hides shows up as a mesh that is subtly the wrong
shape rather than as anything failing.

The work is a checkerboard run, the resulting YAML installed by
`pimesh_bringup`, and `camera_node` loading it (probably via
`camera_info_manager`, which would be its first dependency beyond the three it
has).

**Trigger: the tape-measure session in P5.** P5 already needs a person in the
room with the camera to pin `depth_scale` against a surface at a known distance.
Both are physical measurements of the same camera in the same room and neither
can be closed by a script, so they are one visit — and doing the calibration
first makes P5's number better. Splitting them means going twice.

---

## `camera_node` publishes `PipelineStats`

`pimesh_msgs/PipelineStats` exists and is specified as "published by every
node"; `camera_node` publishes none. It has the numbers already — it counts
frames, and it counts the frames the kernel dropped before dequeue, which is a
figure no other stage can see and which nothing currently reads.

**Trigger: P7, the dashboard.** The message is the dashboard's data source and
nothing else consumes it. Publishing it earlier would mean a topic with no
subscriber and a rate chosen by guesswork; the dashboard's 10 Hz stats panel is
what fixes the window and the cadence. P7 is also where "a high drop percentage
is not a fault" has to be rendered as two columns rather than one, and
`dropped_by_design` versus `dropped_in_transport` needs a consumer to be
designed against.

---

## Raise the dev-box frame-rate floor, or explain the variance

`tools/gates/capture.sh` asserts ≥ 40 Hz measured on the dev box. Across five
runs on 2026-09-09 that number came out 44.3, 51.3, 51.5, 55.9 and 58.6 Hz,
while the Pi-side rate measured in the same runs sat at a steady 59.3 Hz. So
between a quarter and none of the camera's frames are being lost on the Wi-Fi
hop depending on conditions, and the gate's floor has as little as 4 Hz of
margin on a bad moment.

That is a real property of the link and possibly of the QoS — RELIABLE with
KEEP_LAST(1) means a writer that cannot keep up drops the frame it is holding,
which is the intended behaviour and also indistinguishable, from the dev box,
from a camera that produced fewer frames. Distinguishing them needs the
publisher's own count compared against the subscriber's over the same window.

**Trigger: the first `gates/capture.sh` run that fails on rate alone** — that
is, dev-box rate below 40 Hz while the Pi-side probe in the same run reports
≥ 55 Hz. Until that happens there is nothing to fix and the floor is doing its
job; when it happens, the fix is to have `camera_node` publish its own frame
count so the gate can attribute the gap to the link rather than infer it.

---

## Recover from a device that comes back

`camera_node` exits non-zero when `/dev/video0` disappears mid-stream, which is
the right behaviour for this milestone: a node that cannot capture must not sit
there being a node. It means a briefly reset USB link ends the session, and a
person has to restart it.

**Trigger: the first time this happens for a reason other than a leaked
process.** Every device-loss event so far has been another process holding the
camera, which reconnection would not fix and would in fact paper over. Retrying
around a real fault is only worth building once there is a real fault to retry
around — and if it is built before then, the first thing it will do is hide the
leaked-process case behind a retry loop.
