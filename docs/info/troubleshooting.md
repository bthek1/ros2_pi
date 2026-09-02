# Troubleshooting

Symptom → cause. Most of this is **inherited from the predecessor project**
`piros2`, where it was diagnosed the expensive way. Read it before debugging;
the odds are good that whatever just happened has happened before.

## Nothing arrives on a topic across the LAN

- **The daemon is showing you a stale graph.** `ros2 daemon stop && ros2 daemon
  start` after any `ROS_*` or DDS change. This one masks real fixes.
- **Mismatched `ROS_DOMAIN_ID`.** It is 42 on both machines. A non-login
  `ssh pi 'ros2 topic list'` runs on domain 0 and shows you nothing — use
  `ssh pi "bash -lc '...'"`.
- **DDS bound to the wrong interface.** The dev box has Docker bridges and
  Tailscale; unpinned, DDS advertises an address the Pi cannot route to. Pin via
  `CYCLONEDDS_URI`.

## Large messages never arrive, small ones do

**BEST_EFFORT QoS on megabyte-class messages delivers zero frames.** They
fragment past the socket buffer and never reassemble. Every image and depth
topic in this project is RELIABLE + KEEP_LAST(1). If you added a topic and it is
silent while `ros2 topic list` shows it, check its reliability first.

## Frame rate collapses when a second consumer starts

**Every RELIABLE subscriber pulls its own unicast copy over the Pi's Wi-Fi.**
Measured: ~2 frames/s per reader with five readers, against 14.7 Hz with one.
The dev box must have **exactly one** subscriber to `/image_raw/compressed` —
`decode_node` inside the component container. If you started a second one
"just to look", that is the cause.

## Every frame is rejected as stale

**Never gate on `header.stamp` age.** `usb_cam` 0.8.1 offsets every stamp by a
random sub-second amount, redrawn at each launch (measured 0.223 / 0.362 /
0.979 s on three launches, each steady to ±4 ms within its process). A
stamp-age freshness gate silently dropped 100% of frames. Measure freshness on
**receipt time**; stamp *deltas* are still trustworthy.

## The image is black, or the frame rate is far below spec

**V4L2 controls persist inside the camera** across processes and reboots. A
manual exposure left by an earlier run makes every later session black. And the
C922 powers on with `exposure_dynamic_framerate=1`, which costs ~10 fps indoors
despite the driver reporting the default as 0. Print every control
current-vs-default and reset to a known baseline **before** looking for a
software bug. A dim room also needs gain raised by hand — it is never
auto-adjusted on Linux.

## `Device or resource busy` on `/dev/video0`

A leaked camera process from an earlier session still holds exclusive capture.
Sweep both machines for stragglers. This is why every recipe has an EXIT trap
and why ad-hoc runs get `timeout -s INT` or an explicit `pkill -f`.

## `/dev/video1` gives no frames

It is not a capture device — it is the C922's UVC metadata node. Capture is
`/dev/video0` only. The Pi's `/dev/video2x` nodes belong to the Pi 5's `pispbe`
ISP and are unrelated to this camera.

## The Pi is unreachable

**The Wi-Fi link dies; the Pi does not.** It has happened twice — once with the
AP rejecting re-association, once silently for 15 hours — while the OS ran on
undisturbed. Never diagnose it as "crashed" without evidence: `ping` first, and
after recovery read `journalctl -b -1`, because the truth is in the previous
boot.

## Depth inference is suddenly 4× slower

**ONNX Runtime fell back to the CPU execution provider.** 72–79 ms/frame on
CUDA, 280–305 ms/frame on CPU. It falls back silently when the CUDA provider
library, the CUDA runtime or cuDNN cannot be loaded. `depth_node` must log the
provider it actually got at startup — if that log line is missing, add it before
debugging anything else.

## `cv::cuda::` will not link

The apt OpenCV (4.10.0) is built without the CUDA module. ORB runs on the CPU
here, which is fine at 500 features. Building OpenCV from source to change that
is a decision to state out loud, not a fix to slip in.

## A build fails with `No module named 'em'` (or `yaml`)

**Measured here, 2026-09-01.** The dev box's `python3` is PlatformIO's venv,
which shadows the system Python for `#!/usr/bin/env python3` shebangs. rqt and
other GUI tools crash with `No module named 'yaml'` — and **C++ interface
packages are not immune**, because `rosidl` generates message code in Python:
`pimesh_msgs` failed with `No module named 'em'` (empy lives in
`/usr/lib/python3/dist-packages`, which the venv does not see).

`just build` puts `/usr/bin` first on `PATH`, which fixes it. If it still fails
after that, **CMake has cached the wrong interpreter** — `rm -rf build install`
and build again; another `colcon build` will not clear it.

## `rviz2` will not start

It renders through GLX and needs `QT_QPA_PLATFORM=xcb` on this Wayland session.
With driver 595.84 it reports OpenGL 4.6 in hardware — the old software-GL
workaround (`LIBGL_ALWAYS_SOFTWARE=1`) is obsolete and should not be
reintroduced.

## Nodes keep logging after a recipe ends

Killing a background `bash -lc` wrapper orphans its ros2 grandchildren, which
sit silent until something feeds their subscriptions again. Cleanup traps must
`pkill -f` the node patterns, never `kill %N`.

Two more, both measured here on 2026-09-01 while building the P0 gate:

- **A backgrounded `ros2 launch` can be un-interruptible.** A shell without job
  control sets SIGINT to `SIG_IGN` for background children, so
  `timeout -s INT 20 ros2 launch … &` ran to completion and left its three
  `static_transform_publisher`s orphaned, while the identical command in the
  foreground shut down cleanly. Run the launch in the foreground under
  `timeout -s INT` and background the probe instead.
- **`pkill -f <pattern>` matches the shell that is running it** if that
  pattern appears anywhere in its command line. `pkill -f "ros2 launch
  pimesh_bringup"` typed in a command that also *mentions* that string kills the
  command itself, mid-script, with an exit code that looks like something else
  entirely (144). Inside a justfile recipe the body is a temp file so the
  pattern is not on any command line; typed at a prompt it is.

## Parameters in the YAML seem to do nothing

The file is keyed by **node name**, and a key that does not match applies
nothing — silently, with no warning. Check the node's actual name (`ros2 node
list`) against the top-level key. This trap has cost real time in the
predecessor project more than once.
