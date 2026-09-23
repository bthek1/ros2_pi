# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

**Monocular visual SLAM in C++: one webcam on a Raspberry Pi, the camera's pose
and a live 3D mesh of the room on the dev box.**

The camera is the only sensor — no depth sensor, no IMU, no wheel odometry.
Everything downstream is inference and geometry:

```
RGB frame → keypoints (ORB) → pose (PnP vs keyframe) → monocular depth → TSDF fusion → triangle mesh → dashboard
             └──────── tracking front end ────────┘    └────────── mapping back end ──────────┘
```

The Pi is a **sensor head only**: capture, stamp, ship JPEG. Every expensive
stage runs on the dev box, on the GPU where it pays.

**This repo is a C++ rewrite.** The predecessor
[`~/Documents/piros2`](../piros2) is a working Python implementation of the same
pipeline (`piros2_world_mesh`, `piros2_perception`). It is **reference, not a
dependency** — read it for measured numbers, traps and topic shapes, but do not
copy its structure wholesale: the point of the rewrite is to do in one process
with `rclcpp` components what Python needed three processes and two interpreters
to do.

### The goal, and what is actually built

**Monocular visual SLAM: estimate the pose of a single moving RGB camera and
build a 3D map from it.** Two halves — a *tracking front end* that answers where
the camera is, and a *mapping back end* that turns those poses and a depth
estimate into a surface.

**Both halves run, end to end, and the loop does not close.** As of
**2026-09-23** six packages build from source under both distros and a webcam on
a Pi becomes a live triangle mesh on the dev box with a browser tab watching it.
What is built is therefore **visual odometry plus dense mapping**: there is no
loop closure, no pose graph and no relocalisation, so nothing ever recognises a
place it has been and drift is bounded per step but unbounded over a session. The
keyframe store those need exists and has one reader — the newest keyframe, which
each frame is posed against; the second reader, matching against *every* keyframe,
is the next body of work. **Do not call this SLAM in the docs without saying which
half is missing.**

| Stage | Node | Where | Measured |
| --- | --- | --- | --- |
| Capture | `camera_node` | Pi | 720p MJPEG, 44–59 Hz on the LAN, stamped at `VIDIOC_DQBUF`, calibrated intrinsics |
| Decode | `decode_node` | dev box | 1.90 ms/frame, the container's **one** network subscriber |
| Keypoints | `keypoint_node` | dev box | ORB ×500, **57.8 Hz at 6.82 ms/frame** against an 8 ms budget |
| Pose | `odometry_node` | dev box | PnP vs keyframe, **1.37 px over 94 inliers, 81.6% posed, 16.6 Hz** |
| Depth | `depth_node` | dev box, **GPU** | Depth Anything V2 Small, **55.1 ms/frame, 17.4 Hz** |
| Fusion | `fusion_node` | dev box | TSDF at 15 mm voxels, **15.3 ms/integration at 17.1 Hz** |
| Surface | `mesh_node` | dev box | marching cubes every 10 s, **2.8 s per extraction**, off the hot path |
| View | `dashboard_node` | dev box, **own process** | **10.01 Hz stats**, costs the pipeline **0.77%** |

**Depth is the pipeline's clock.** 17.4 Hz against a 59 Hz input — roughly one
frame in three, the rest dropped through a one-slot mailbox. Nothing downstream
of it can run faster.

**Two things are unpinned and both need a person**, not a script: `depth_scale` is
arbitrary at 10.0 until a tape measure fixes it, so every distance here is
plausibly shaped and the wrong size; and `bags/desk1` is a *pan*, so 6-DoF
translation makes no measurable difference on it and a clip with deliberate
translation is what would settle that. Both are in
[docs/plans/future/milestone-e-future.md](docs/plans/future/milestone-e-future.md).

**`bags/desk1` is the reference clip** — 59.7 s, 3489 frames at 58.5 Hz, 220 MB,
sha256 `1333c5bd…`. `bags/` is git-ignored, so that hash is its only identity, and
every phase from P3 on measures against the same seconds of room. Record one with
`bash tools/record-clip.sh <name> <seconds>`; it resets the camera's V4L2 controls
first, because a clip recorded at 20 fps under a stale manual exposure cannot be
un-recorded.

### How this project got here, and the one lesson to carry

The milestone-by-milestone narrative — every phase, what it measured, and the
dozen or so times a number was wrong before it was right — is
**[docs/info/build-log.md](docs/info/build-log.md)**. Status per milestone is
[docs/info/roadmap.md](docs/info/roadmap.md); the closed plan issues are the
per-phase record (`gh issue list --label plan --state all`).

**Read the build log before adding a gate.** Its single recurring finding, and the
most valuable thing this project has produced, is that **a gate is green over
broken behaviour far more often than the code is broken**. A teardown test
signalled one of two recipes and passed over six leaked processes. A zero-copy
test measured the one configuration that cannot fail. A cost budget asserted
`0.00 ms` against 8 ms and printed PASS. A calibration instrument reported
`topics referenced: 0` and nobody read the zero. An `implausible=0` came from a
counter inside an `if` that never ran.

In almost every case the *instrument* was wrong and the code was fine, and in
every case the wrong output looked plausible — a zero in a cost field reads as
fast, a matched fraction of 1.000 reads as perfect. So:

- Ask of every gate **what it does not touch**, never whether it passed.
- Give a threshold a **control** that has been watched to fail it. A budget nobody
  has seen excluded anything is not an assertion.
- Run a new check **against the failure it was written for** before keeping it.
- An unmeasured value and a good one **must not have the same spelling**.

Do not write "the node publishes X at Y Hz" until a node has published X and you
have watched it do Y. Everything the rest of `docs/` says about work not yet built
is **design intent**; when you build it, change the tense and say what you
measured it with.

## The two machines

Measured 2026-09-01 unless noted.

| | Dev box (here) | Raspberry Pi |
| --- | --- | --- |
| Reach it | local | `ssh pi` (key auth, works with `BatchMode=yes`) |
| OS | Ubuntu 26.04.1 LTS "resolute", x86_64, kernel `7.0.0-30-generic` | Ubuntu 24.04.4 LTS "noble", aarch64, kernel `6.8.0-1060-raspi` |
| ROS | **Lyrical** (`/opt/ros/lyrical`) | **Jazzy** (`/opt/ros/jazzy`) |
| CPU / RAM | 16 threads / 18 GB | 4 cores / 8 GB |
| GPU | **GTX 1660 SUPER, 6 GB, driver 595.91.07** | none |
| Runs | everything except capture | `pimesh_camera` and nothing else |
| Network | LAN on `ens18` | LAN over **`wlan0`** — Wi-Fi, no cable |

**The two machines are on different ROS distros.** That is inherited from
`piros2` and it is deliberate, not drift: `packages.ros.org` is pinned by Ubuntu
suite, so the dev box's 26.04 upgrade replaced every `ros-jazzy-*` package with
`ros-lyrical-*`. Jazzy ↔ Lyrical interop was measured working over the LAN in
`piros2` on 2026-08-31 (topics, `camera_info`, `tf_static` all crossed), and
re-measured as **this** project's own on 2026-09-08: a Jazzy publisher on the Pi
delivered 39 of 40 messages in 20 s at 2 Hz to a Lyrical subscriber here
(the retired `gates/hello-lan.sh`; `bash tools/gates/capture.sh` covers the claim
now). DDS is wire-compatible across distros; the C++ ABI is
not, and that one sentence is the whole reason for the build-from-source rule.

**Consequence for C++, and it is the sharpest one in the project:** ROS 2 has no
ABI compatibility guarantee across distros. A `.so` built here does not run
there. **Every package must build from source on both machines** — no
cross-compiled binaries, no shipped `install/` tree, and nothing in
`pimesh_camera` may depend on a Lyrical-only API. C++17 (Jazzy's baseline), not
C++20, in anything the Pi builds.

The drift runs in **both** directions, and the dangerous one is the direction
that fails *here*: `ament_target_dependencies()` was deprecated in Jazzy and is
**removed in Lyrical**, so the dev box stops with `Unknown CMake command` on
CMake that the Pi would have built without complaint (measured 2026-09-08).
Where a build-system API differs, prefer the spelling that exists on **both** —
plain `target_link_libraries()` against the exported targets — and confirm it on
both before relying on it, with something like

```bash
grep -rh "add_library(rclcpp::" /opt/ros/lyrical/share/rclcpp/cmake/*.cmake
ssh pi 'bash -lc "grep -rh \"add_library(rclcpp::\" /opt/ros/jazzy/share/rclcpp/cmake/*.cmake"'
```

The reverse case is worse because it is silent: a Lyrical-only API compiles here
and is only discovered at the far end of an `rsync`. `bash tools/build-pi.sh` is cheap —
run it before believing a CMake change.

**And the C++17 rule is not enforced on this box at all, measured 2026-09-12.**
Every `CMakeLists.txt` here sets `CMAKE_CXX_STANDARD 17`, and Lyrical's
`ament_cmake_ros_core` exports an INTERFACE target requiring `cxx_std_20`
(`ament_ros_defaults.cmake`), which *raises* it: the dev box compiles everything
as `-std=c++20` while the Pi compiles the same sources as `-std=c++17`. So a C++20
feature is caught by the Pi's build and by nothing else. A worked example of the
same class, from the same afternoon: `tf2/LinearMath/Matrix3x3.h` exists on Jazzy
and has been **deleted** on Lyrical in favour of `.hpp`, so that one fails here
and builds there. Where two spellings exist, take the one that exists at both ends.

The header spellings that differ are worth knowing before reaching for them: all
of `tf2/LinearMath/*` and `tf2_ros/*` are `.hpp` on both distros, with the `.h`
forms deprecated on Jazzy and partly gone on Lyrical.

The Pi is reachable non-interactively, so **verify hardware claims by running
commands over SSH** rather than assuming:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 pi "bash -lc 'v4l2-ctl --list-devices'"
```

A non-interactive `ssh pi '...'` does **not** get the ROS environment — the
exports live in `~/.profile`. Always use a login shell (`bash -lc`), or you are
silently on domain 0 with the wrong RMW and the result means nothing.

Full specs: [docs/info/hardware.md](docs/info/hardware.md).

## The pipeline

Eight stages, one node each, all but the first on the dev box. The measured table
is under *The goal* above; the full design — message types, QoS, cost budgets — is
[docs/info/pipeline.md](docs/info/pipeline.md).

**Where the depth budget goes, measured 2026-09-15.** Inference alone is
**51.08 ms** (`bash tools/gates/gpu-stack.sh`); the whole per-frame cost inside
`depth_node`, on the node's own clock, is **55.10 ms**
(`bash tools/gates/depth.sh`) — so preprocessing, the reciprocal, the resize back
to 1280×720 and two publishes cost about 4 ms together. The predecessor measured
72–79 ms for the same model on the same card through Python and an older ONNX
Runtime, with a 280–305 ms CPU fallback; ours measures 182 ms on the CPU for
inference and 288 ms per frame. The two sets are not directly comparable and both
say the same thing about the ratio.

**`keypoint_node` and `odometry_node` are two nodes because they are two stages.**
They were one node until 2026-09-23, publishing *two* `/pipeline/stats` rows —
ORB at the camera's rate on one thread, the pose solve at the depth rate on
another — which is the observation the split came out of. The detector publishes
`/keypoints` and the estimator subscribes to it in-process, so the boundary costs
a pointer.

## Constraints that are easy to get wrong

Items marked *(inherited)* were measured in `piros2`, not here. Trust them as
strong priors, re-verify before quoting a number as this project's own.

- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is
  ~83 MB/s over the Pi's Wi-Fi. Compressed transport only, and exactly **one
  subscriber on the dev box** — see the next item.
- **One Wi-Fi reader, not five** *(inherited, measured 2026-08-16)*. Five RELIABLE
  subscribers each pull their own unicast copy and collapse the link into a
  retransmit storm: ~2 frames/s per reader against 14.7 Hz for a single reader.
  In C++ this is solved properly: **the dev box runs one component container
  with intra-process communication on**, so the decode happens once and every
  downstream component gets a pointer to the same buffer. That is the
  main structural reason this rewrite exists — do not break it by launching
  components as separate processes "for debugging".

  **This one is no longer inherited: it is measured here.** The retired
  `gates/hello-ipc.sh` ran the same container twice, with intra-process on and
  off, and compared the
  payload address the publisher logged against the one the subscriber received —
  19/19 equal with it on, 0/16 with it off (2026-09-08). `bash tools/gates/ipc.sh`
  is the same experiment on the real pipeline with the Pi's camera feeding it:
  504/504 against 0/395 (2026-09-12). **Address equality alone is not evidence** —
  two allocations in one process can coincide, and one did, at 1/22 — so any
  future zero-copy claim needs the with/without control, not a single run.

  **Which pointer the subscriber takes depends on how many consumers the topic
  has, and this is the trap, measured here on 2026-09-12.** rclcpp serves
  *ownership-taking* subscriptions by moving the buffer into the **last** one and
  copying it for every other (`add_owned_msg_to_buffers`: "Copy the message since
  we have additional subscriptions to serve"); subscriptions taking a shared const
  pointer go through `add_shared_msg_to_buffers`, which hands **one** buffer to
  all of them, however many. So with `decode_node` publishing to two consumers
  that both took `std::unique_ptr`, **0 of 574** frames arrived at the published
  address, and with both taking `ConstSharedPtr` it was **504/504**. Use
  `unique_ptr` where a topic has exactly one consumer — `decode_node`'s own
  inter-process subscription, where the middleware allocates a fresh message
  anyway — and `ConstSharedPtr` for every fan-out. **A topic gains consumers as
  the pipeline grows**, which is how `/keypoints` went from one reader to two on
  2026-09-23 and `keypoint_probe` had to stop taking ownership. A `const &`
  callback is a *shared* subscription and does not copy; the note this file used
  to carry, that it "works perfectly and quietly copies", was the wrong way round.

  **And one consumer is the case that cannot fail**, which is why `gates/ipc.sh`
  asserts the decoded topic has at least two subscribers while it measures. The
  gate passed at 429/429 with one probe attached and failed at 0/574 on the next
  run with `keypoint_node` beside it. Ask what the gate does **not** touch.
- **A queue depth can be a correctness requirement, not a performance knob, and
  `KEEP_LAST(1)` is the default that hides it.** Every image topic here keeps 1
  on purpose — the freshest frame is the only one anybody wants. `/keypoints` is
  the exception and keeps **120**, because `odometry_node` looks each message up
  by *exact stamp* to pair it with a depth map, and in `rotation_only` composes a
  rotation increment out of **every** one of them. A RELIABLE writer at
  `KEEP_LAST(1)` holds only the newest sample for retransmission, so a reader that
  fell one frame behind loses that frame permanently: an increment that silently
  never happened, and a pose that under-rotates with nothing in any log to say so.
  The same reasoning is why that topic is not behind a `Mailbox`, which is
  newest-wins by construction. Ask of any stream whether a dropped message is *one
  missed update* or *a hole in a chain*; depth is the first, keypoints the second.
- **One field saying two things is a landmark that can never be matched.**
  `pimesh_msgs/Keypoints` published `track_id = -1` for a feature's first
  sighting from P3 until 2026-09-23 — the id and "is this new" in one column.
  That is fine while the detector is also the consumer, and fatal once it is not:
  `odometry_node` intersects a keyframe's landmarks with the current frame's **by
  id**, so a feature whose id is hidden on the frame a keyframe was taken from can
  never be matched against itself again — silently, on the ~10% of features that
  are new in any frame. It is now `track_id` (always ≥ 0) and `is_new` beside it.
  **The migration has its own trap**: `keypoint_probe` counted matched features as
  `track_id >= 0`, which after the change is *every* feature — it would have
  reported a matched fraction of exactly **1.000** over any clip, and 1.000 reads
  as a tracker working perfectly. This is the `-1` covariance lesson and the
  `cost_mean=0.00` lesson arriving together: **an unmeasured value and a good one
  must not have the same spelling.**
- **Two matchings leave the detector and they are not interchangeable.** Track ids
  come from a *pooled* pass over a ten-frame window, which forgives detection
  churn; `prev_x`/`prev_y` come from a *mutual-best* pass against the previous
  frame alone. The geometry needs the second — a match five frames back spans five
  times the motion and would be weighted as though it spanned one — so both travel
  on `/keypoints`. A consumer that reconstructed the pairs by intersecting two
  frames' track ids would get the looser pairing, and **a rotation fit on looser
  pairs does not fail: it returns a wrong answer with a plausible residual.**
- **BEST_EFFORT delivers zero large frames** *(inherited, and it is about
  size)*. Megabyte-class messages fragment past the socket buffer and never
  reassemble. Every image and depth **publisher** here is `RELIABLE` +
  `KEEP_LAST(1)` — freshest frame, no backlog.

  **A viewer subscribing to the ~80 kB compressed stream is the exception, and
  it is measured.** A RELIABLE reader delivers in sequence, so one lost fragment
  head-of-line blocks every frame behind it for a heartbeat round trip; over the
  Pi's Wi-Fi that is a visible freeze several times a minute. Changing only the
  reader's QoS, 20 s windows of ~1100 frames (2026-09-09): RELIABLE gave 10 gaps
  over 50 ms with a worst of 490 ms; BEST_EFFORT gave 3, worst 181 ms, with 0
  undecodable frames. `rviz/camera.rviz` therefore asks for BEST_EFFORT, which a
  RELIABLE writer satisfies (only the reverse is incompatible). At 80 kB a frame
  is ~56 fragments; at 2.7 MB raw it is ~1900, which is why this scopes the rule
  rather than contradicting it. **Do not carry it to a raw image topic**, and
  treat the same head-of-line question as open for `decode_node` in P2.
- **Never gate on `header.stamp` age** *(inherited, and now only half true)*.
  `usb_cam` 0.8.1 has a once-per-process epoch bug that puts stamps a random
  sub-second amount in the past, redrawn at every launch. A stamp-age freshness
  gate silently dropped 100% of frames.

  **`pimesh_camera` fixes this for our own capture path, measured 2026-09-09.**
  It stamps `ros_now - (monotonic_now - v4l2_buffer.timestamp)` — an *interval*,
  not an epoch — so there is no per-process constant to be wrong, and two
  launches agree to within 1 ms. Frames on `/image_raw/compressed` therefore
  carry honest capture times and may be reasoned about.

  **But not across the two machines.** The stamp is set on the Pi's system clock
  and read on the dev box's, and the gap between them is NTP's business: it
  measured +8 ms and −19 ms an hour apart on 2026-09-09 with nothing changed.
  So a stamp-age gate on the dev box is *still* forbidden — it would be
  measuring NTP. Compare stamps to stamps (deltas are kernel capture intervals
  and are trustworthy), and measure latency where one clock covers both ends.
- **A looping bag and a node publishing TF from it cannot both be right**,
  measured here 2026-09-13. `ros2 bag play --loop` restarts the clip, so every
  `header.stamp` jumps back by the bag's length; `odometry_node` stamps
  `odom -> base_link` with the frame's own stamp, as it must; and
  `tf2::BufferCore` refuses any transform older than the newest it already
  holds. So after the first wrap the edge **froze at the bag's final stamp for
  the rest of the run** — it can never catch up, because the newest stamp the
  loop will ever produce is the one already in the buffer — and every rejection
  was logged, at the frame rate, by every listener in the domain.

  **That flood is why RViz flickered, and the mechanism is worth knowing.**
  `RCUTILS_LOG_WARN("TF_OLD_DATA ...")` is emitted *inside* `setTransform`'s
  `std::unique_lock<std::mutex> lock(frame_mutex_)` (geometry2,
  `buffer_core.cpp`), and rclcpp serialises log output on a process-global mutex
  behind a synchronous write to the terminal. So ~59 times a second the TF buffer
  was held shut across a terminal write while RViz's render loop waited on the
  same mutex for `lookupTransform`. Both panels are drawn by one Qt loop, so the
  **empty** 3D view stuttered too — which is the tell that a stall like this is
  not a graphics problem. **A log line under a lock is a rate limit on everything
  that lock protects.**

  **`--clock` with `use_sim_time` is the obvious fix and is measured wrong.** The
  backwards jump fires `tf2_ros::Buffer::onTimeJump`, which calls `clear()` on
  the **whole** buffer — `tf_static` included — and nothing republishes a latched
  topic afterwards. Over four wraps the tree went away at the first "Detected
  jump back in time. Clearing TF buffer." and never came back. RViz does the same
  one layer up ("Detected jump back in time. Resetting RViz."). A bag that
  carried `/tf_static` would re-seed it on each loop; ours do not.
  So: `tools/replay.sh` loops and starts `pimesh.launch.py pipeline:=false` —
  the static frame tree, no components — and `tools/view-keypoints.sh` needs the
  pose, so it plays a bag **once**. `gates/keypoints.sh` had already reached the
  same conclusion independently.
- **One session at a time, and this is the same rule as "one Wi-Fi reader" seen
  from the publisher's side.** Two sessions on one ROS domain are not two
  independent sessions. Measured 2026-09-13: `just view-camera` and
  `just replay desk1` up together put the Pi's live camera *and* a three-minute-old
  bag on `/image_raw/compressed`, so one `keypoint_node` decoded the interleaved
  mixture and published a pose whose stamps jumped minutes back and forth —
  flooding both RViz windows through the mutex above. **Neither session had
  anything wrong with it**, and each was silent run alone (0 warnings over 195 s
  of replay, 75 s of view-camera). The tell is in the warning text: the same
  timestamp printed two or three times, and timestamps going *backwards* within
  one second of log.

  `assert_no_session` in `tools/just-lib.sh` is the guard, and **every script
  that starts a session calls it, gates included** — for a gate it is the sharper
  case, because a measurement taken beside another session is a measurement of a
  mixture with nothing in the output saying so. It is called **before
  `arm_cleanup`**, always: the cleanup handler kills this workspace's processes,
  so a refusal after the trap is armed would tear down the session it is refusing
  to disturb.
- **V4L2 controls persist inside the camera** *(inherited)* across processes and
  reboots. A manual exposure left by a benchmark makes every later session
  black; the C922 powers on with `exposure_dynamic_framerate=1`, which costs
  ~10 fps in indoor light. Treat camera state as inspectable machine state and
  reset it before diagnosing black frames or low fps as a software bug.
- **`/dev/video1` is not a capture device** — it is the C922's UVC metadata node.
  Capture is `/dev/video0`. `V4l2Capture` checks `device_caps` rather than
  `capabilities` for exactly this: the latter is the union over every node the
  driver owns, so the metadata node reports its sibling's capture bit and passes
  a naive check, failing later and worse.
- **Reset the camera before measuring anything about it.**
  `bash tools/camera-reset.sh` puts every control back to its default, forces
  `exposure_dynamic_framerate=0` (whose reported default of 0 is a lie about
  what the camera powers on with), prints the whole control table
  current-vs-default, and exits non-zero if the one control that matters did not
  stick. `gates/capture.sh` runs it first; a rate measured without it is a
  measurement of whatever the last person left behind.
- **`/camera_info` comes from a file, and there is no `calibrated` flag.**
  `camera_node` loads
  `package://pimesh_bringup/config/camera_info/c922_720p.yaml` — the standard
  `camera_info` YAML, byte-for-byte what `cameracalibrator` writes — and falls
  back to nominal intrinsics with a startup WARNING when it is absent. **The
  absence of that file is non-fatal and a broken one is fatal**, measured on the
  Pi 2026-09-12: a wrong resolution, a non-`plumb_bob` model or unparseable YAML
  each exit 1 by the same route a busy device does, because substituting the
  placeholder for a calibration somebody put there on purpose would be a green
  light over a wrong one. The `calibrated` parameter is gone; the flag is derived
  from whether a file loaded *and* carries non-zero distortion, so a file full of
  zeros cannot switch the warning off. Produce the file with
  `bash tools/calibrate.sh`, check it with `bash tools/gates/calibration.sh`.

  **Not `camera_info_manager`**, which is the obvious choice and was rejected
  twice over: it is absent on the dev box under Lyrical, so the Pi's package
  would depend on an apt install on the machine that never runs the camera, and
  `CameraInfoManager` advertises a `set_camera_info` service from its
  constructor — letting anything on the domain rewrite a running camera's
  intrinsics on disk. `yaml-cpp` and `ament_index_cpp` are on both machines
  already.
- **A calibration checked on a centred board is not checked.** Distortion is
  radial, so near the optical axis there is nothing to correct: frames whose
  corners reach only ~half way to the frame corner put the *uncalibrated*
  straightness at 0.52 px, inside the 1.0 px budget, while frames reaching ~98%
  put it at 1.4–2.1 px (synthetic sweep, 2026-09-12,
  `src/pimesh_instruments/test/test_straightness.py`). So "cover the frame corners"
  is a precondition of the measurement rather than advice about technique, and
  `gates/calibration.sh` asserts a floor on coverage. A precondition that is not
  asserted is a comment.
- **The calibration board is a ChArUco sheet on the wall, and its print came out
  1% small.** `docs/charuco_a4_7x9_25mm.pdf`: 7×9 squares, 6×8 interior corners,
  `DICT_4X4_250`, 18 mm markers. Its 100 mm scale bar **measures 99 mm**
  (2026-09-12), so the real numbers are `--square 0.02475` and `--marker 0.01782`,
  not the 0.025/0.018 in the filename. A 1% scale error is a 1% error in every
  distance the pipeline ever reports and **nothing in software can detect it** —
  this is the one number that must come off a ruler. Full table in
  [docs/info/hardware.md](docs/info/hardware.md#calibration-target).
- **Calibration frames come from a bag, chosen offline.**
  `bash tools/calibrate.sh record` then `select` — the live `grab` is the quick-look
  path. Recording separates moving the camera from judging the frames, so selection
  sees every candidate at once rather than deciding greedily as they arrive, and the
  bag can be re-selected without another session at the wall. `select` marker-confirms
  every frame, drops the blurred half-median, rejects past 45° oblique, then picks for
  coverage first and pose diversity second. **It cannot rescue a bowed board** — see
  the next bullet; frame choice does not undo a bulge in the paper.
- **Rigid is not flat, and a print taped to a wall is not a calibration target.**
  A wall satisfies "rigid" completely and still leaves the paper bowed, because
  paper taped at its edges bulges between them. Measured 2026-09-12 over 75 real
  frames by solving a per-corner out-of-plane offset with the intrinsics: **3.02 mm
  peak-to-peak of bow**, and forcing the flat model onto it drove reprojection from
  0.77 px to **1.04 px** while flipping **`k1` from +0.0098 to −0.0569** — inventing
  pincushion on a camera that has barrel, and shifting `cx` by 32 px to
  accommodate. **The sign of `k1` is the cheapest tell that a board is not flat.**
  The surface is real, not a fit artefact: solved from disjoint halves of the frame
  set it correlates at +0.994, differing by 0.08 mm rms. Mount the sheet on foam
  board, MDF or a clipboard and put *that* on the wall.
- **The ArUco markers corrupt `cornerSubPix` at its default window.** On our sheet
  the marker border sits 3.375 mm from each chessboard corner — ~6.7 px at a 49 px
  square pitch — inside the default 11×11 refinement window. Measured: 11×11 gives
  1.040 px, **7×7 gives 0.871 px**, 3×3 gives 1.171 px. Worth ~17%, secondary to the
  bow, and it does not change `k1`'s sign.
- **Two board conventions, one apart, and our own printed sheet gets it wrong.**
  `cameracalibrator -p charuco --size N` wants **squares** (it goes into
  `cv2.aruco.CharucoBoard`); `cv2.findChessboardCorners` wants **interior
  corners**. The line printed along the bottom of our A4 sheet says `--size 6x8`,
  the corner count, and measured against a real frame that interpolates **0**
  corners where `--size 7x9` interpolates **42**. The PDF is on the wall and cannot
  be edited, so do not copy the command off it — `bash tools/calibrate.sh` takes
  `--squares 7x9` and derives the corner count itself for exactly this reason.
- **`findChessboardCorners` can lock onto a lattice one square out, and only the
  markers can tell.** Measured 2026-09-12: 1 frame in 35 of a real grab set produced
  a genuine, internally consistent 6×8 corner grid that disagreed with the ChArUco
  ids by **45.8 px** against a 49 px square pitch, while looking sharp and nearly
  square-on. One such frame was enough to make the calibration straighten *nothing*
  (4.16 px against a 4.17 px control) at a reprojection error of 0.768 px. Rejecting
  it and the frames the markers could not confirm gave **0.4548 px** and held-out
  0.46/0.47. So every frame is confirmed against the markers before use
  (`confirm_grid` in `tools/calib_straightness.py`), in the grabber *and* in the gate.
  A chessboard corner carries only its position in whatever lattice was found; a
  ChArUco corner carries the id saying which corner it is. That identity is the whole
  value.
  One check rejects three failure modes: misregistration, motion blur, and obliquity
  past ~50° where markers stop resolving — and those oblique frames were independently
  the least accurate (mean reprojection 0.933 px beyond 50° against **0.429 px in the
  20–35° band**, which is the band to actually shoot in).
- **`cv2.aruco` constructors that exist on both machines and mean different things —
  branch on the version, never try/except.** On the Pi's OpenCV 4.6,
  `cv2.aruco.CharucoBoard((7, 9), sq, mk, d)` (the 4.8+ spelling) **does not raise**:
  it constructs a default, uninitialised board that **segfaults the interpreter** the
  first time anything draws with it. `DetectorParameters()` is worse — it also
  constructs, with zeroed thresholding fields, so marker detection finds nothing and
  the code reports "no markers decoded", a legitimate-looking result. Both are
  version-branched in `tools/calib_straightness.py` (`>= 4.8` and `>= 4.7`
  respectively). **An API that exists at both ends and means different things is worse
  than one that is missing at one end, because the missing one fails loudly** — the
  same lesson as `ament_target_dependencies` in reverse.
- **Read that board with `findChessboardCorners`, not `cv2.aruco`.** `cv2.aruco`'s
  API differs across the two machines — the Pi's OpenCV 4.6 has no
  `CharucoDetector`, this box's 4.10 does — and an instrument that differs per
  machine is the ABI split in a new costume. `interpolateCornersCharuco` is the one
  entry point on both, if partial views ever become necessary. That a chessboard
  detector works on a ChArUco board is verified, not assumed: all 42 aruco-
  interpolated corners agree with the 6×8 grid at the same (row, col) index, median
  0.73 px (2026-09-12).
- **Never measure the board off a picture of the board.** The marker-to-square
  ratio is 0.7199 in the PDF and 0.654–0.674 in camera frames — 7% low, unchanged
  by sub-pixel refinement, because a 32 px marker loses a pixel per side of its
  black border to blur and JPEG. Measure the sheet, or render the PDF.
- **The C922 at 720p has essentially no lens distortion, which invalidates a premise
  of P9.** The phase was written on "a plumb_bob model with all-zero coefficients
  asserts that a consumer webcam has no barrel distortion; it has". Measured
  2026-09-12, at 1280×720 it very nearly does not — most likely corrected in firmware
  for this mode. Three independent lines:
  (1) over 243 marker-confirmed frames, correlation between how far the board reached
  from the image centre and how bent its rows were was **−0.160**, where a radial
  distortion would make it strongly positive;
  (2) real straight edges 430–473 px long at 0.62–0.73 reach depart from straight by
  only **0.93–1.42 px**, where `k1 = +0.08` would bow them several times more;
  (3) every fit off a real set lands **|k1| < 0.02** with the sign flipping as frames
  are added — a parameter with nothing to estimate.
  **Consequences:** never assert `k1 > 0` (it fails a *correct* calibration here), and
  the straightness test cannot be "strictly better than the placeholder" — the
  placeholder is already almost right. `gates/calibration.sh` asserts "not worse" plus
  the absolute budget, and prints the ratio. **Re-measure before assuming this holds at
  another resolution**; a cropped or uncorrected mode may behave completely differently.
- **A board photographed only square-on calibrates to nonsense, and every check
  that should catch it except one says it is fine.** Focal length and radial
  distortion trade off when the board is never tilted, so the solve is poorly
  conditioned and returns a self-consistent wrong answer. Synthetic, 2026-09-12,
  14 views, true `fx=905`: square-on gave **`fx=4840.8`, `k1=+1.97`** (true
  `+0.085`) with an **in-sample reprojection error of 0.084 px** — better than the
  0.5 px budget and about as good as the correct fit's 0.068 px. It also *passes*
  the straightness assertion, at 0.333 px against a 1.116 px control. Two things
  catch it: the **held-out** reprojection error (3.83 px against 0.068 px) and a
  plausibility bound on `fx`, and `gates/calibration.sh` now does both. **This is
  why `tools/calibrate.sh` grabs its frames before the calibration session** — an
  in-sample number cannot see it. Being rigid does not help: a board taped flat to
  a wall is rigid and walks straight into this, because sliding the camera parallel
  to the wall leaves every view square-on. Tilt 20–40°, both axes.
- **Undistorting with zero coefficients is the identity, whatever `K` says.**
  `cv2.undistortPoints(pts, K, 0, P=K)` cancels the two `K`s exactly. So the
  nominal placeholder does not correct the lens badly — it does not correct it at
  all, and "straighter than the placeholder" and "straighter than the raw
  corners" are one claim, not two. Worth knowing before designing a control
  around swapping `K`.
- **The C++ GPU stack exists as of 2026-09-15, in a user prefix, and `nvcc` is
  still absent.** `bash tools/fetch-gpu-stack.sh` installs ONNX Runtime 1.30
  (`gpu_cuda13`), CUDA 13.1's redistributable runtime libraries and cuDNN 9.26
  into `~/.local/opt/pimesh-gpu` — version-pinned, sha256-verified, no sudo
  (this box's `sudo` prompts for a password and no script here has one). The ONNX
  Runtime tarball **vendors no CUDA at all**; Python's `onnxruntime-gpu` only
  works because pip wheels do. `bash tools/gates/gpu-stack.sh` is the check:
  **CUDAExecutionProvider at 51.20 ms mean, p95 51.50 ms** on Depth Anything V2
  Small at 518², against **213.18 ms** for the same binary on the CPU provider.
  Runtime libraries are **not a compiler** — a hand-written CUDA kernel still
  needs a real toolkit install. Driver is **595.91.07** (the 595.84 in older docs
  has drifted).
- **A linker flag that works in an executable does nothing for a component, and
  this is the sharpest finding of P4.** Resolving a **dlopened** object's
  `DT_NEEDED` entries, glibc searches the object's own `DT_RPATH`, its *loader
  chain's*, and the **main executable's** — and a dlopened object has no loader
  chain, so for `libonnxruntime_providers_cuda.so` the executable's is the only one
  left. `tools/gpu_probe` is an executable this workspace links, so it has ours;
  `component_container_isolated` is somebody else's and has none. Measured
  2026-09-15, same libraries and same flags, one container apart: `gpu_probe` at
  **51 ms on CUDA**, `depth_node` at **517 ms on the CPU**, no error anywhere but a
  single line naming `libcublasLt.so.13`. The fix is `preload_cuda_provider()` in
  `depth_engine_ort.cpp`: `dlopen` the CUDA libraries **by absolute path** before
  ONNX Runtime asks for them, so its `DT_NEEDED` entries are satisfied from what is
  already loaded and no search happens. Use `RTLD_LAZY` on the provider itself —
  `RTLD_NOW` reports `undefined symbol: Provider_GetHost` on a provider that is
  completely fine, because ONNX Runtime supplies those symbols *after* loading it.
  `LD_LIBRARY_PATH` is not available: it is read once at process start and cannot
  be set for a component `ros2 component load` put in somebody's container.
- **Link anything that uses ONNX Runtime with `-Wl,--disable-new-dtags`, or it
  silently runs on the CPU.** `libonnxruntime_providers_cuda.so` is *dlopened* by
  `libonnxruntime.so` and carries no `RPATH` or `RUNPATH` of its own, and
  `DT_RUNPATH` — CMake's default, and every modern linker's — is **not inherited
  down a dlopen chain**, while the older `DT_RPATH` is. Measured 2026-09-15, same
  source and same libraries, one flag apart: `RUNPATH` gave
  `CPUExecutionProvider` at 236.62 ms, `RPATH` gave `CUDAExecutionProvider` at
  51.20 ms. There is no error message — only a pipeline four times slower than it
  should be. `gates/gpu-stack.sh` runs the default-flags build as a control and
  asserts it does *not* reach CUDA, so the flag cannot quietly stop mattering.
  The same fact is why the CUDA and cuDNN libraries are installed **beside** the
  ONNX Runtime ones in one directory: `$ORIGIN` is the only search path that
  works, since `LD_LIBRARY_PATH` is read at process start and cannot be set for a
  component somebody else's container launched. **And `gates/gpu-stack.sh` cannot
  see the component case at all** — its instrument is an executable, which is
  exactly the configuration that cannot fail. `gates/depth.sh` is what covers it;
  see the bullet above.
- **A library that resolves is not a library that works.** Every CUDA
  redistributable ships link-time stubs in `lib/stubs/` — a `libcublas.so`, and
  in cudart a `libcuda.so` standing in for the driver. Flattening `lib/` and
  `lib/stubs/` into one prefix let the 22 kB stub overwrite the 54 MB real
  cuBLAS; it loaded, resolved every symbol, printed `You are running using the
  stub version of cublas` on *stdout*, and segfaulted on the first inference. The
  `ldd`-says-no-unresolved-dependencies check passed over it throughout, which is
  why `fetch-gpu-stack.sh` also asserts a size floor on `libcublas.so.13`.
- **A mutex held across a client loop is a backpressure path, and it is the one
  thing a monitoring tool must not have.** `dashboard_node`'s web server holds one
  mutex while it walks its client list, and the first version of the two page
  buttons called a ROS service from inside that walk. It **deadlocked instantly**
  — the handler took the same non-recursive mutex to read itself — and every later
  HTTP request timed out with nothing in any log. Removing the second lock would
  have fixed the deadlock and left something far worse: a service call waits up to
  ten seconds, and holding the mutex across it blocks `broadcast()`, which is
  called from the ROS callbacks. **A button press would have slowed the
  pipeline**, which is precisely what P8 exists to make impossible. A deadlock is
  a loud bug; that would have been a silent one. Actions are parked by the handler
  and run outside the lock.
- **Two rate caps in series beat against each other, and every number involved
  looks right.** `keypoint_node` caps its preview at 10 Hz; the dashboard capped
  the same stream at 10 Hz on the way out, and rejected every frame that arrived a
  hair early — which, with two independent clocks, is about half of them. Measured
  2026-09-19: the page received **5.48 Hz** of a 10 Hz stream and **2.80 Hz** of a
  5 Hz cap on a 10 Hz stream. Nothing was wrong at either end. A cap is a ceiling
  on a *faster* source, so it has to admit one running at exactly its own rate:
  compare against 90% of the period, not 100%.
- **A run's second half is slower than its first, and a gate that compares them
  is measuring the clip.** `mesh_node`'s extraction grows from 0.7 s to 3.2 s as
  the volume fills and takes CPU from everything above it, so every stage is
  genuinely slower late in a replay. `gates/dashboard.sh` compared a run's halves
  to judge what killing a client cost and reported every stage **8-18% slower
  after** — precisely backwards, and entirely that. Compare the same seconds of
  clip across runs, never two parts of one.
- **A gate measures the machine as well as the pipeline, so do not build on the
  box while one is running — and that means you, Claude.** `gates/dashboard.sh`
  measured **15% on `fusion` and 9% on `keypoints`** between two runs with
  *nothing* attached, which reads as a pipeline far too noisy to hold to P8's 2%
  bound. It was a `colcon build` and a `colcon test` on the same box, started by
  the person writing the gate. Re-measured quiet: **0.10%**. This file already
  says to check the machine is yours before measuring; a build of your own counts.
  It is also why that gate's bound is the *measured floor plus slack* rather than
  a constant — on a quiet box it collapses to 2%, and on a loaded one it widens
  with the load instead of failing and blaming the thing under test.

  **That widening only works when the load lands on the control runs, and on
  2026-09-21 it did not.** `gates/dashboard.sh` plays the clip four times and
  derives its bound from the two runs with nothing attached; interference that
  happens to fall on the *client* run inflates the number being judged while
  leaving the bound narrow. Measured that day on a desktop in ordinary use —
  Firefox and VS Code both holding GPU contexts on the card `depth_node` runs
  on — three runs of the same gate, one of them against the **pre-change**
  binary as a control: the noise floor came out **1.00%, 6.56% and 1.34%** on
  consecutive runs, and the verdict tracked the floor rather than the code. The
  pre-change control *passed*, and passed only because its floor happened to
  come out at 6.56%; it did not demonstrate anything about the dashboard. Every
  stage ran ~11% below its recorded quiet-box rate in all three runs
  (`depth` 15.5–16.1 Hz against **17.47 Hz** recorded), on code unchanged in
  that path.

  **So a pass from this gate on a busy box is not evidence, and neither is a
  fail.** What stayed stable across all three runs is the part that does not
  depend on timing: the handshake, six stage rows, `mesh=3` deliveries,
  **0** dropped to clients, and STALE at 2.01–2.02 s. Read those; treat the rate
  table as unmeasured until the box is idle. The desktop counts as load — this
  bullet already says a build of your own does, and a compositor sharing the GPU
  with CUDA inference is the same fact wearing different clothes.
- **A fit that explains its measurements is not a fit that is right, and no
  residual can tell you the difference.** `cv::solvePnPRansac` answers how well a
  pose explains the pixels it was handed; it has no opinion whatever about whether
  the 3D points those pixels were matched against are where they were claimed to
  be. Measured 2026-09-19 on `bags/desk1`: a single step of **9.4 m between two
  depth frames** at a mean inlier reprojection of **1.23 px** over ~90 inliers —
  as confident as any accepted frame in the run — which then became the reference
  every later frame was posed against. The mean step over the run stayed at 2.7 cm
  and every summary number looked healthy. The plausibility of the *motion* is a
  different question from the quality of the *fit*, and it needs its own refusal:
  `max_speed_m_s` in `odometry_node`, and `gates/odom.sh` asserts on the fastest
  published motion rather than on the residual.
- **A guard that is only armed when an unrelated feature is enabled is worse than
  no guard, because its counter reads as evidence.** That plausibility bound's
  bookkeeping — the stamp of the last accepted pose — started life inside the
  `if (translation_tau_s_ > 0.0)` branch that also does the low-pass. With the
  filter switched off the guard never ran, and the node reported **`implausible=0`
  over a trajectory containing a 6.95 m step**. A zero from a check that did not
  execute and a zero from a check that found nothing have the same spelling. It is
  the `cost_mean=0.00` lesson from `gates/keypoints.sh` arriving through a third
  door.
- **Do not difference two monocular depth maps to get a camera motion.** The
  obvious RGB-D odometry is to unproject both frames and fit a rigid transform
  between the clouds. Measured 2026-09-19 it does not work here, and not because
  of the estimator: Depth Anything V2 estimates *relative* depth, so (a) both
  clouds carry an error that is a smooth **warp** rather than per-pixel noise and
  therefore does not average down over three hundred landmarks, and (b) the
  overall scale breathes a few percent a frame — `fusion_node`'s aligner hits its
  own 15% clamp on one frame in seven of `bags/desk1` — which a rigid fit can only
  absorb as translation along the view axis. In order: rigid frame-to-frame
  reported an **89.5 m** path over a 45 s desk sweep; dividing the scale out left
  it at 117 m; measuring against a keyframe brought it to 44 m; a low-pass brought
  it to 8.4 m. All four were worse than publishing no translation at all. **PnP**
  works because it changes what is measured rather than how it is filtered: the
  keyframe's 3D landmarks against *this* frame's **pixels**, so one depth map is
  involved instead of two, there is no scale ratio between them, and the residual
  comes out in pixels — a unit this project has a calibration for, unlike the
  metres everything else is in.
- **A constant that is wrong but works is invisible, and only an outside
  reference finds it.** The FNV-1a offset basis in this project was
  `1469598103934665603` — the real one, `14695981039346656037`, with its last
  digit dropped in a paste — in both copies, since milestone A, and it was found on
  2026-09-15 by the first test that compared it against the published reference
  vectors. **Nothing had been wrong.** A hash with a different basis avalanches
  just as well and answers "are these two byte arrays equal" correctly every time,
  so every number taken with it (`duplicate payloads`, `/depth/rgb identical`)
  was and remains right; it simply was not FNV-1a, while two comments said it was.
  Write named constants in the form they are published in — hex for a hash basis —
  and pin them against a reference vector, because no amount of testing the
  *behaviour* of a hash will ever tell you it is the wrong hash.
- **A helper with no home has no tests.** `percentile` existed four times over and
  FNV-1a twice, each in an anonymous namespace inside a file with a ROS node in it
  — unreachable by any test and free to drift apart. Every gate in this project
  prints a number that comes out of one of them. They now live in
  `pimesh_core/stats.hpp` with `test_stats` behind them; `pimesh_camera`
  keeps its own copy on purpose, because a dependency edge from the Pi's package to
  a dev-box one would point the wrong way down the pipeline for the sake of twelve
  lines, and that cost is written down in the header rather than discovered later.

  **It was true again on 2026-09-19, and the copy was a `cv::Mat` over a message
  rather than a number.** `depth_mat_over` — the 32FC1 twin of `image_buffer.hpp`'s
  `mat_over`, and the thing every distance the TSDF integrates passes through —
  sat in an anonymous namespace inside `fusion_node.cpp` with no test able to
  reach it. It is in `image_buffer.hpp` now, beside the function it is a copy of,
  and `test_image_buffer` covers both. Moving it also made visible something the
  two files had kept apart: they **disagree about `step == 0`**, which `mat_over`
  reads as "not set, derive it" and this one refuses. That is defensible — bgr8
  arrives from publishers this workspace does not own and 32FC1 does not — so it
  is pinned by a test that asserts the *pair*, because a test saying only "32FC1
  refuses a zero step" is satisfied by making them consistent, which is exactly
  the change the asymmetry is there to make somebody think about.

  **And a third time on 2026-09-21, in the package written most recently.**
  `quote` and `number` — JSON string escaping and number formatting — sat in an
  anonymous namespace inside `dashboard_node.cpp`, and `stats_json()` and
  `pose_json()` are those two functions and nothing else, so *the entire contents
  of the page* went through code no test could reach. The mesh payload packing was
  the same story one step along: thirty lines of stride arithmetic inside a member
  function that took a `Marker` and ended in a `broadcast`, so reaching it meant
  standing up a node and a socket. Both are headers now (`json.hpp`,
  `mesh_payload.hpp`) with `test_json` and `test_mesh_payload` behind them, and
  moving the second one found a figure two files had been repeating: 120 k
  triangles is **5.4 MB** on the wire, not the 4.3 MB both claimed, which is the
  position array with the colour bytes left out.

  **And a fourth time on 2026-09-23, in the code the split had just created.**
  Reading a `Keypoints` message back into corners, ids, descriptors and pairs was
  three loops inside `odometry_node`'s subscription callback — the conversion every
  landmark the pose is fitted to passes through, and no test could call any of it.
  It is `keypoints_view.hpp` now with `test_keypoints_view` behind it, and unlike
  the first three **something was wrong**: see the next bullet.

  **The question to ask of a helper is not whether it is interesting but whether
  a test could call it if it wanted to.** Four times now, the answer was no; three
  times nothing was wrong yet, and the fourth had two bugs in it.
- **A loop bounded by one array's length while indexing a different one, in a
  message whose parallel arrays are a contract nothing enforces.** Measured
  2026-09-23 in `odometry_node`: the pixel loop ran to `x.size()` and indexed
  `y[i]`; the pair loop ran to `prev_x.size()` and indexed `prev_y[i]`. Both read
  past the end of a `std::vector` on any message whose arrays disagree — undefined
  behaviour, not a wrong number, and reachable only from a publisher nobody had
  written yet, which is why reading the code did not find it and a test did.

  **A struct-of-arrays message is a ragged-array hazard by construction.** The
  `.msg` says every per-feature array is the same length; nothing checks it at
  runtime, and the single publisher in this workspace happens to be correct. The
  fix is two lines of defence and they do different jobs: `keypoints_well_formed()`
  is the predicate the node checks so it can **refuse and count** a bad message,
  and the accessors clamp to the shortest array they touch so a caller who forgot
  the predicate still cannot read off the end. Clamping alone would have been
  worse than the bug — silently converting half a frame pairs corners with the
  wrong track ids, which is a plausible wrong answer where the crash was at least
  a crash.
- **Two things that have to hold the same value across a *language* boundary are
  the `volume_key` trap with nothing left to catch it.** Within C++ a shared
  constant is at least possible; between `dashboard_node.cpp` and `web/app.js`
  there is no mechanism at all. Four pairs are typed twice, and every one fails
  quietly: the five **channel numbers** (`Channel` against `CH` — swap two and
  JPEG bytes go through `JSON.parse`, killing two panels while the socket stays
  up); the **JSON field names** (a renamed field reads as `undefined`, which
  `fmt` renders as an em dash — *identical* to how the page draws a stage that
  has not reported yet); the **stage strings** the page keys its headline figures
  on, `by.depth` and `by.capture`, where a rename gives a permanent dash; and the
  **binary mesh offsets**, where a stride read one byte out still yields floats,
  so the page draws believable geometry in the wrong places — a mesh that looks
  like a poor reconstruction, which is what this project has spent two milestones
  expecting to see. `test_dashboard_contract` reads both files as text and checks
  all four. Reading source as text to test it is inelegant; the reader is in a
  browser and cannot be linked against, so it is that or nothing.
- **A gate that cannot read its inputs reports a number, not an error.**
  `gates/view-configs.sh` shells out to `python3` twice: one call was pinned to
  `/usr/bin/python3` and the other was not, so the second resolved to PlatformIO's
  venv, `import yaml` raised, and the gate printed **`topics referenced: 0`** and
  failed. Zero referenced topics is not an error message — it is a plausible
  reading of an empty config directory, and the assertion it feeds ("every
  referenced topic is published") is *vacuously satisfiable* by it. Found
  2026-09-23; it reports 17 now. Two lessons, and the second is the general one:
  this file already says to pin the interpreter, and one of the two call sites had
  been fixed while the other was not — **fix a spelling everywhere or nowhere**,
  because a half-applied fix is indistinguishable from an applied one until the
  unfixed half runs.
- **A gate's instrument needs its own tests, and the reference implementation is
  an instrument.** `gates/keypoints.sh` closes P3 by asserting `keypoint_node`'s
  matched-keypoint fraction is within 5 points of `tools/orb_reference.py` over
  the same clip, which makes that script the thing deciding the gate — and it had
  no tests until 2026-09-19, the same gap `test_straightness` and
  `test_mesh_render` exist to close for P9's and P6's instruments. **Two of the
  ways it can be wrong move the number *up*:** drop the one-to-one claim and
  several features inherit one track, so the reference agrees with the node more
  readily; count a featureless frame as a matched fraction of zero and it drifts
  about five points, which is the whole tolerance. Neither raises anything.
  `test_orb_reference` pins them with synthetic descriptors and no bag, and pins
  deliberately the *same* properties `test_orb_tracker` pins on the C++ side:
  two implementations that do not answer the same question are not comparable
  however close their numbers land. Six mutations of the script were checked
  against the suite and each was caught by the test that claims to catch it.
- **A budget nobody has seen fail is not an assertion.** `gates/gpu-stack.sh`
  asserts the CPU path is *slower* than the 80 ms budget as well as the GPU path
  being faster, because a threshold only means something once the thing it is
  meant to exclude has been shown to fail it. `gates/depth.sh` does the same one
  level up, on the node's whole per-frame cost: `use_cuda:=false` is a launch
  argument so the control is the same binary one parameter apart, and it measures
  287.92 ms against the same 80 ms.
- **A number parsed out of a shared log is only unambiguous until a second node
  logs the same prefix.** `gates/keypoints.sh` read ORB's per-frame cost with
  `grep 'stats rate=' | tail -1`. P4 put `depth_node` in the same container, it
  logs a `stats rate=… cost_mean=…` line too, and `tail -1` began returning its
  final window — the idle seconds after the clip ended, `cost_mean=0.00`. The gate
  asserted **0.00 ms against an 8 ms budget and printed PASS** (2026-09-15).
  Nothing looked wrong except a zero, and a zero in a cost field reads as *fast*.
  Grep by node name, take the last window that had frames in it, and assert the
  number is **greater than zero** — an unmeasured value and a good one must not
  have the same spelling.
- **A callback order that depends on subscription-registration order, not on
  publication order.** A single-threaded executor collects everything that became
  ready in one wait cycle and runs the callbacks in the order the *subscriptions
  were created*. `depth_node` publishes `/depth/rgb` and then `/depth`, and
  `fusion_node` pairs them by exact stamp — and with the depth subscription
  registered first, the depth callback ran before the colour it was looking for
  had arrived. Measured 2026-09-16: **67 of ~750 frames**, about 12%, integrated
  colourless on `bags/desk1` with nothing whatever wrong upstream. Registering
  colour first took it to 0, and the worker re-checks as well, because an
  ordering that holds for this executor is not a thing to depend on twice.
- **`publish` moves from the pointer you handed it, and reading the message
  afterwards is a null dereference the compiler is happy with.** Measured
  2026-09-16: `mesh_node` filled a `MeshStats`, published it, and then read
  `stats->triangles` to build a log string. The container segfaulted immediately
  after a re-mesh that had done marching cubes, component pruning, hole filling
  and 400 000 edge collapses correctly — and the entire cleanup, replayed offline
  on the very same mesh under AddressSanitizer, ran without a murmur, because the
  offline harness never published anything. **The stage that crashes is not always
  the stage that is wrong**, and a use-after-move is invisible to every test that
  does not run the real transport. Read what you need out of a message *before*
  you publish it.
- **A latched publisher needs a latched reader, and the mismatch is legal.** A
  VOLATILE subscriber against a TRANSIENT_LOCAL writer is QoS-*compatible*: it
  simply does not receive the stored message and waits for the next one. Measured
  2026-09-16: `ros2 topic echo --once /world/mesh` returned nothing over a run
  that had published eight surfaces, and `gates/mesh.sh` reported "nothing was
  published". `rviz/mesh.rviz` needs `Durability Policy: Transient Local` for the
  same reason. **A mismatch that is legal is worse than one that is not**, because
  nothing anywhere says the two disagree.
- **A CPU-heavy thread in the container starves the pipeline rather than blocking
  it.** `mesh_node`'s extraction is 3-4 s of marching cubes and quadric
  decimation with no deadline; every stage above it has one. Measured while an
  extraction ran, before the thread was niced: `depth_node`'s rate fell from 17.8
  to **14.6 Hz with its per-frame cost unchanged at 55.9 ms** — not more work,
  just not being scheduled — and `fusion_node` inherited a 404 ms gap between
  integrations. Linux `nice` is **per thread**, so `setpriority` from inside the
  worker pushes only the extraction down and leaves the node's executor, timer and
  services alone.
- **The last stats window of a run is the idle tail, and `rate > 0` keeps it.**
  Every gate that averages a node's own windows has to filter on a *rate*, not on
  "more than no frames". Measured 2026-09-16: `fusion_node`'s final window covered
  the seconds after the clip ended — one straggling frame, `mesh_node` still
  grinding — and reported a lag of 51 ms mean and 121 ms p95 against 0.02-1.96 ms
  in every window where the pipeline was running. The gate failed a run in which
  nothing was wrong. This is `gates/keypoints.sh`'s `cost_mean=0.00` lesson
  arriving by a different door.
- **`bags/desk1` has a ~400 ms stall about 35 s in, and it is not the pipeline's.**
  It lands in the seventh five-second window of every run — 13.6, 14.4 and 14.6 Hz
  — and it is present in runs recorded before `mesh_node` existed, with
  `depth_node`'s own per-frame cost unchanged at 55.9 ms throughout. Any gate
  asserting a ratio of worst-to-typical interval on this clip will fail on it and
  point at the wrong node. `gates/mesh.sh` uses a control run instead.
- **The GPU is Turing TU116: compute capability 7.5, 6 GB, no tensor cores.**
  fp16 buys bandwidth, not math throughput. Budget fp32 and do not plan around
  TensorRT fp16 speedups you have not measured.
- **The apt OpenCV (4.10.0) has no CUDA module.** `cv::cuda::` will not link.
  ORB runs on the CPU; that is fine at 500 features, but do not write code that
  reaches for `cv::cuda` and then "fix" it by building OpenCV from source
  without saying so.
- **PCL 1.15 and `pcl_ros` are installed; Open3D is not.** The Python side
  needed Open3D for TSDF and meshing and paid for it with a process boundary
  (no Python 3.14 wheel). In C++, write the TSDF and marching cubes ourselves or
  use PCL — that boundary should not come back.
- **`ROS_DOMAIN_ID=42`** on both machines, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  and CycloneDDS pinned to `ens18` here / `wlan0` there. The dev box has Docker
  bridges and a Tailscale interface DDS will happily bind to instead, advertising
  an address the Pi cannot route to. After changing any `ROS_*` or DDS variable,
  `ros2 daemon stop && ros2 daemon start` — it caches discovery state and will
  otherwise show you a stale graph and mask a fix that worked.
- **Two Pythons shadow the system one on this box**, and they break different
  things. PlatformIO's venv wins `#!/usr/bin/env python3`, so rqt and other GUI
  tools crash with `No module named 'yaml'` — prefix `PATH=/usr/bin:$PATH`. A
  uv-managed `~/.local/bin/python3.14` wins CMake's `FindPython3`, so **every
  `ament_cmake` package fails at `ament_package()`** with
  `No module named 'catkin_pkg'` (measured 2026-09-08): ament shells out to
  Python at *configure* time, so a C++-only package is not immune. Build with
  `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`, which `just build`
  already passes. Running C++ nodes are immune; building them is not.
- **The session is Wayland.** `rviz2` renders through GLX and needs
  `QT_QPA_PLATFORM=xcb`; it was measured working with hardware GL (4.6) on
  driver 595.84 as of 2026-08-31, so the old software-GL workaround is obsolete.
- **The Pi's Wi-Fi link dies while the Pi keeps running** *(inherited)*. Never
  diagnose an unreachable Pi as "crashed" without evidence — `ping` first, then
  read `journalctl -b -1` after recovery. Every scripted `ssh pi` must carry
  `-o BatchMode=yes -o ConnectTimeout=5`; a bare ssh hangs ~2 minutes against a
  dead link and wedges whatever trap it sits in.

## Conventions

- **This repo is the colcon workspace.** Packages go in `src/`, named
  `pimesh_<thing>` — eight exist: `pimesh_msgs`, `pimesh_bringup`,
  `pimesh_camera`, `pimesh_dashboard`, and — since #14's P2 on 2026-09-23 —
  `pimesh_core`, `pimesh_frontend`, `pimesh_depth` and `pimesh_mapping`, which
  are `pimesh_perception` split in two and `pimesh_world` renamed. **The prefix
  stays and the suffix has to say what is inside**: `pimesh_perception` was four
  stages in one package and `pimesh_world` said nothing at all, while
  `pimesh_camera` and `pimesh_dashboard` already named their contents and were
  left alone. `pimesh_core` is the three helpers more than one stage needs —
  `Mailbox`, the `cv::Mat` view over a message, and the percentile — and it
  exists because the include graph said so rather than because symmetry did:
  `pimesh_world` was already reaching into `pimesh_perception` for all three, a
  mapping package depending on a perception one for a percentile.
  A seventh, `pimesh_hello`, was the scaffolding reference and was **deleted
  2026-09-23** along with four of its five gates, once `gates/build.sh`,
  `gates/ipc.sh` and `gates/capture.sh` covered the same claims on the real
  pipeline — with better instruments, since `ipc.sh` asserts two consumers where
  `hello-ipc.sh` measured the one configuration that cannot fail. Its fifth gate
  was never about hello and lives on as `tools/gates/teardown.sh`. **`pimesh_dashboard` builds on the Pi
  and is the one package nothing runs there** — it is built at both ends only
  because `gates/build.sh` builds the whole workspace at both ends and
  `gates/test.sh` asserts the *same suites* on both, which is what would catch a
  C++20 spelling or a Lyrical-only header slipping into it. Its node is also the
  one dev-box node outside the container, because the promise it makes is *it
  must be able to die* and a component there would take the TSDF with it.
  **`pimesh_depth` builds on the Pi too, and the GPU code
  inside it is why that took arranging**: `depth_node`, its registration and its
  parameters are identical on both machines, and only the inference engine behind
  a `DepthEngine` interface is conditional — `depth_engine_ort.cpp` where CMake
  found ONNX Runtime, `depth_engine_null.cpp` where it did not. The Pi therefore
  builds a `depth_node` that refuses to start, with a message saying why, which is
  correct for a node that machine must never run. Building it only where the GPU
  stack exists would break `test_transforms`, which asserts every plugin string in
  the launch file resolves in the ament index — on the machine where that check
  matters least, by weakening it everywhere. (Not `ros2_pi_*`: a `ros2_` prefix reads as core tooling.)
  Shared shell helpers live in `tools/` and are rsynced to the Pi, so they must
  work on both distros — `tools/ros-env.sh` discovers the distro rather than
  naming it, and `tools/check-stale.sh` is run by both `gates/build.sh` here and
  `sync-pi` there.
- **C++ only for nodes.** `ament_cmake`, C++17, no Python nodes. Launch files
  and one-off tools may be Python — that is not a licence to move logic there.
- **Nodes are `rclcpp_components` components**, registered with
  `RCLCPP_COMPONENTS_REGISTER_NODE`, each with a thin `*_main.cpp` so it can also
  run standalone. The bringup launch composes the dev-box ones into a single
  container with `use_intra_process_comms=True`. A node that only works
  standalone is a bug. **`pipeline:=false` leaves the container out entirely**,
  bringing up the three static transforms alone — it is the shape the launch had
  before P2, and it exists again because `tools/replay.sh` must not publish a
  pose over a looping bag (see the constraint above). Everything or nothing, not
  a smaller pipeline: the components share one process precisely so a frame is
  handed on as a pointer. **`src/pimesh_camera/` is the worked example** — it was
  `src/pimesh_hello/` until that package was deleted, and the camera is the same
  shape doing real work, so copy it rather than rediscovering the shape: the class
  declared in `include/pimesh_camera/`, defined in `src/`, the register macro at
  the foot of the .cpp, `rclcpp_components_register_nodes` (plural — the singular form
  generates its own `main` and makes the thin one dead code), and the library
  installed to `lib/` while the executable goes to `lib/${PROJECT_NAME}/`.
- **No work in a subscription callback beyond a bounded copy.** Anything that
  costs milliseconds (inference, fusion, meshing) runs on its own thread with a
  single-slot mailbox: newest frame wins, older one dropped. Queues that grow
  are how this pipeline dies.
- **Parameters live in `config/*.yaml`, keyed by node name**, and launch files in
  `launch/*.launch.py`. A key that does not match the node name silently applies
  nothing — a trap that has cost this project's predecessor real time. Declare
  every parameter with a description and validate ranges at declaration.

  **The same trap exists one level down, on the parameter *names*, and it caught
  this project on 2026-09-15**: three keys for `depth_node`'s colour preview sat in
  the YAML before the node declared any of them, which is an entirely ordinary
  order to write things in and leaves no trace once it is wrong — the file loads,
  the node runs on its code defaults, and it looks configured.
  `test_no_parameter_in_the_yaml_is_read_by_nobody` in `test_transforms.py` is the
  guard: it reads every `declare_parameter` name out of the package source and
  asserts the YAML sets nothing else, so the config and the code have to arrive
  together.

  **And the same trap once more between two nodes, where nothing relates their
  keys at all.** A ROS parameter file is not validated against anything, and two
  nodes that have to agree about a value agree only because somebody typed it
  twice. `fusion_node` and `mesh_node` find the shared TSDF by a `volume_key`
  string, and a mismatch is not a partial failure — it is two separate volumes,
  one filled and never meshed, one meshed and never filled, and `/world/mesh`
  empty for the life of the session with no error anywhere. The same shape covers
  `fusion_node`'s `depth_topic` against `depth_node`'s, the two nodes' shared
  `max_range_m`, the frame the map lives in, and `mesh_min_weight` sitting between
  the volume's floor and its ceiling. All six are asserted in `test_transforms.py`
  against the YAML, hermetically — **write a test for every pair of keys that has
  to hold the same value**, because the failure is always a pipeline that runs.

  **And a launch argument threaded into a node's parameters needs an explicit
  `value_type`.** A `LaunchConfiguration` is a string, so the raw substitution sets
  a *string* parameter of that name, which a node expecting a bool or a double
  ignores while saying nothing — the same failure `use_intra_process_comms` once
  had, where every frame was serialised and no log line mentioned it.
  `test_every_launch_argument_override_declares_a_value_type` asserts it over the
  whole override dict, so a new one is covered without anyone remembering to.
- **Two kinds of test, and conflating them is a mistake.** The
  `tools/gates/*.sh` scripts are the **phase tests**: slow, often needing the Pi
  and the camera, and they are what closes a claim. `bash tools/test.sh` runs the
  **unit tests** (`colcon test`) — fast, hermetic, no hardware, and they run on
  both machines. Write a unit test for logic that can be got wrong silently (the
  stamp arithmetic, a matrix layout, a quaternion); write a gate for anything that
  is a number about a running system.

  **What belongs there is decided by whether a mistake is *visible*, not by
  whether the code is interesting.** Every suite in this workspace exists because
  some wrong version of that code produces a plausible result. If a bug in it
  would announce itself — a crash, an exception, a topic that stops — a gate is
  the cheaper place to catch it.

  **`colcon test` exits 0 when a test fails**, because it is reporting that the
  run completed — and it exits 0 again when a package has no tests at all, which
  is what an unbuilt tree looks like. `colcon test-result --all` decides, and
  `bash tools/gates/test.sh` asserts on the counts: zero failures, zero skips, a
  floor on how many tests ran, and the same suites at both ends. Raise the floor
  when you add tests; never lower it to make a run pass.

  Tests that need a camera do not belong in `colcon test` — the dev box has no
  capture device, and a suite that only runs on the Pi is one that stops being
  run.

  **The per-suite catalogue is [docs/info/testing.md](docs/info/testing.md)**, and
  it is worth reading before adding one: it records the three recurring reasons a
  suite exists here, of which the sharpest is that **a helper with no home has no
  tests** — the question is not whether a helper is interesting but whether a test
  could call it if it wanted to.
- **Build with `just build`**, not bare `colcon`. The recipe is
  `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  -DCMAKE_BUILD_TYPE=RelWithDebInfo`, and without the first argument every
  `ament_cmake` package fails at configure time on this box (see the Python bullet
  above). **The second was missing until 2026-09-12, and every C++ number this
  project had measured was therefore unoptimised** — colcon sets no build type and
  an `ament_cmake` package that does not set one compiles with no optimisation
  flags at all. P3's 8 ms per-frame budget is what found it: 7.90 ms without,
  6.71 ms with. `RelWithDebInfo` rather than `Release` because `-O2 -g` measured
  within 2% of `-O3` and leaves a node you can put a debugger on. `bash tools/build-pi.sh` does the same over
  SSH after `bash tools/sync-pi.sh` ships source — source only, never a built tree.
  Keep the commands and the docs in agreement — `docs/info/setup.md` quotes
  `just --list` verbatim and a script asserts it has not drifted.
- **The justfile is the user-facing surface — everything else is a script.**
  It holds `build`, `hello-compose` and `hello-lan`, and that is the whole of
  it: what someone types on a normal day. Gates, the Pi plumbing, the straggler
  sweep and the tree deletions are `bash tools/<name>.sh` and
  `bash tools/gates/<name>.sh`, run directly. **Resist adding a recipe.** The
  bar is not "is this useful" — every one of those scripts is useful — it is
  "would a newcomer's first `just` need to see this?". Seven gate recipes had
  buried `hello-compose`, which is the one command that shows the workspace
  doing something, and the file is trimmed to `build` and `run` precisely so
  that cannot recur. Adding a group is the thing to argue about, not adding a
  line.
- **A sentinel documented in one message type means nothing in another, and the
  wrong one produces a value that is not merely unused but *invalid*.**
  `odometry_node` (then `keypoint_node`) published `pose.covariance[0] = -1.0`
  on `/odom` from P7 until
  2026-09-23, with a comment calling it "nav_msgs' documented way of saying no
  covariance here". It is **`sensor_msgs/Imu`'s** convention, documented in that
  message and nowhere else; `geometry_msgs/PoseWithCovariance` says only
  "row-major representation of the 6x6 covariance matrix". So what went on the
  wire was a matrix that is **not positive semidefinite**, and RViz's Odometry
  display — which eigen-decomposes the position block on receipt — said so at
  the pose rate, ~17 Hz, for the length of every session.

  **Two things about how that hid.** The field is one nothing in this workspace
  reads: RViz draws `/odom`, `odom_probe` measures it and the dashboard shows
  it, and not one of them touches the covariance, so every gate was green over
  it. And the warning was dismissed *in this file* as cosmetic before the
  message definition was read — a code comment was taken as the citation. Read
  the `.msg`; it is one `cat` away on both machines.

  It is also not cosmetic: `RCUTILS_LOG_WARN` goes through rclcpp's
  process-global log mutex behind a synchronous terminal write, which is the
  same mechanism that made the TF_OLD_DATA flood stutter RViz's render loop.
  **A log line under a lock is a rate limit on everything that lock protects**,
  and this one fired five times a second. `unconstrained_covariance()` in
  `rgbd_odometry.hpp` is the fix — a large diagonal, positive definite, the
  conventional spelling of "unconstrained". Not zeros, which is legal and reads
  to a fusion filter as a *perfectly certain* pose; not a measured covariance,
  which `solvePnPRansac` cannot give and which inventing would be exactly what
  `camera_node` refuses to do for `latency_ms`. Overstate ignorance, never
  confidence. `test_rgbd_odometry`'s **OdomCovariance** suite pins the property
  RViz actually checks, and three of its four cases fail against the `-1`.
- **A `{{ }}` substitution in a recipe body is text, not an argument, and an
  empty default therefore *vanishes*.** just pastes the value into the line and
  hands it to `sh`, so an unquoted parameter defaulting to `""` disappears from
  the word list and **every argument after it shifts one place left**. Measured
  2026-09-23, reported by a user running the plain command: `just view-odom` ran
  `view-odom.sh 600 sixdof`, the script read `sixdof` as the bag name, and the
  session died with `no bag at 'sixdof'`. `dashboard` had it too — its port
  would have been read as the bag. Quote every substitution (`"{{ bag }}"`),
  except a variadic `*args`, which has to stay unquoted to stay several
  arguments.

  **What let it ship is the more useful half.** `gates/hello-clean.sh` signals
  both recipes, five endings each — and it invokes `tools/view-odom.sh`
  *directly*, with a bag it names itself, because it needs control of the
  process group. So the one gate that starts these recipes has never once gone
  through the justfile, and **the justfile is the user-facing surface**. Three
  more recipes had the same latent defect with the empty parameter last, where
  it is harmless right up until somebody adds a parameter after it — which is
  precisely how `view-odom` was written, by copying `view-mesh` and adding
  `regime`. `gates/justfile.sh` now dry-runs every recipe and asserts the
  argument count matches the parameter count. Ask what the gate does **not**
  touch.

  **And the first version of that check passed over the bug**, which is the
  third time this file has had to record one. `just -n` echoes the command to
  **stderr**; the check captured stdout with `2>/dev/null`, so `$line` was empty,
  the loop `continue`d past every recipe, and it printed `0 recipe(s) lose an
  argument` over five that did. It was caught only because the broken justfile
  was kept and the check run against it before being kept. **Run a new check
  against the failure it was written for, every time** — a check nobody has seen
  fail is not an assertion.
- **The shell lives in `tools/`.** Every recipe is one line that runs a script,
  and every recipe carries a `[group('build'|'run')]`. The shared prelude, the
  one spelling of the Pi's
  `ssh` invocation, the bracketed kill patterns and `in_range` are in
  `tools/just-lib.sh`, which every script sources; `tools/` is rsynced, so the
  same functions work at both ends. The reason is not tidiness: `just` gives a
  recipe body no way to share code with another, so inlined bash is copy-pasted
  and drifts, and `shellcheck` cannot parse `{{ }}`, so inlined bash is never
  linted. **`bash tools/gates/justfile.sh` is the check** — groups exactly
  `build run`, 0 ungrouped recipes, justfile under 80 lines, no recipe body over
  10 lines, no `ssh`/`pkill`/prelude inlined in a body, `setup.md`'s quoted
  recipe list equal to `just --list`, **every recipe passing as many arguments
  as it has parameters** (`just -n`, since 2026-09-23 — see the bullet above),
  and 0 shellcheck findings over `tools/` (`uv tool install shellcheck-py`).
- **Sessions tear themselves down — no stragglers.** Ctrl-C, a closed terminal
  and a **closed viewer window** must all end everything the recipe started, **on
  both machines**. The mechanism: viewer backgrounded and waited on,
  `arm_cleanup` in `tools/just-lib.sh` installing a handler on EXIT and on
  INT/TERM/HUP that kills every pattern, locally and over SSH, and then *checks*.
  (EXIT alone does fire on Ctrl-C; naming the signals is what makes the
  closed-window case deliberate rather than lucky.)

  **The three endings are three different code paths, and only two of them were
  ever tested.** A signal runs the handler; a closed window sends no signal at
  all — `wait` returns because its child exited and the EXIT trap runs from an
  ordinary end of script. `gates/hello-clean.sh` had signalled every recipe ten
  times over and had never once closed a window, and that is the path that leaked
  on 2026-09-14. It now covers `INT`, `INT-TWICE`, `HUP`, `CLOSE` and
  `CLOSE-EARLY` — 21 cases over seven recipes — and waits for the recipe to
  *return* rather than sleeping three seconds and sweeping, because the contract
  is that nothing is running by the time it has returned.

  **A second Ctrl-C used to kill the teardown, and that is what `INT-TWICE` is
  for.** `_pimesh_on_signal` restored the default disposition *before* cleaning
  up, so from that instant the script was killable — and a verified teardown takes
  7-9 s while printing nothing, which is long enough to look hung. Measured
  2026-09-18 against the pre-fix handler, second SIGINT at 0.2, 1, 2, 4 and 6 s:
  **3 processes left on the Pi at 0.2, 1 and 2 s, none at 4 and 6 s**, because
  `kill_pi` has finished by then. The real one was `just view-mesh` ended with
  `^C^C`, which left a `camera_node` holding `/dev/video0` with a clean dev box
  beside it and no teardown message anywhere — and the split is always the tell,
  because `kill_local` goes first. The fix is two parts: `_pimesh_cleanup_once`
  **ignores** INT/TERM/HUP for the duration rather than defaulting them (which
  covers the EXIT path too, where a Ctrl-C during a closed-window teardown could
  do the same), and teardown now *says* it is working, because nine silent seconds
  is what makes a second Ctrl-C tempting. Ignoring is only safe because each half
  is bounded by `PIMESH_TEARDOWN_SECONDS` and reports when it gives up.

  **And the gate's own method could not see that leak, which is the finding worth
  carrying.** Every other ending is asserted by waiting for the session's process
  *group* to empty and then sweeping — but the local `ssh` client sits in the group
  until its remote command returns, so a script killed mid-teardown leaves a group
  that does not empty until the **recipe's own** `timeout` expires on the Pi, which
  is the same event that kills the leaked `camera_node`. By the time the group was
  empty the evidence had reaped itself and the sweep reported 0/0 truthfully:
  measured 2026-09-18, the first version of this case **passed against the bug**.
  So it waits for the *script* — the pgid leader, whose exit is the contract's "the
  recipe has returned" — and looks at the Pi at once. A leak whose lifetime your
  own session timer bounds is invisible to a sweep taken after the timer.

  **A test whose trigger ships with the fix tests the fix's presence, not the
  behaviour.** The first version of `INT-TWICE` aimed its second signal by waiting
  for the teardown's new "further Ctrl-C ignored" line, so against the pre-fix code
  it reported "never announced a teardown" and proved nothing. It now aims off the
  clock — cleanup starts synchronously at handler entry, so a session still alive
  0.5 s later is inside its teardown — and asserts the announcement separately,
  after the fact, where it cannot become the thing the case is aimed with.
  The signal handler cleans up **once** and then re-raises after `trap -`,
  because a bare `trap handler INT` does not end a script — bash runs the
  handler and resumes at the next line, so an interrupted gate carries on
  measuring what it just killed, and the caller sees exit 0 where it should see
  130.
  **A trap is not always installed just because you wrote one**: a command
  started in the background by a non-interactive shell inherits SIGINT as
  SIG_IGN, and bash refuses to trap a signal that was ignored on entry — so any
  test that sends a fake Ctrl-C must reset the disposition first
  (`setsid env --default-signal=INT,TERM,HUP …`, measured 2026-09-09), or it is
  testing a session that cannot receive the signal.
  **And a trap that *is* installed still will not run while a foreground child
  is.** GNU `timeout` puts its child in a new process group so it can kill the
  tree on expiry; a terminal signals only the *foreground* group, so the command
  under `timeout` never sees Ctrl-C — and bash will not run the trap until that
  foreground child returns, which is exactly what it is refusing to do. Use
  **`run_for`** (`timeout --foreground -s INT`, in `tools/just-lib.sh`) for
  anything run in the foreground; a backgrounded `timeout … &` plus `wait` is
  equally sound, because then the trap fires on arrival. Measured 2026-09-09:
  0.30 s from a real Ctrl-C to a container logging *finished cleanly*, against a
  30 s timer that used to have to expire first.
  Killing a background `bash -lc` wrapper
  orphans its grandchildren — always pattern-match the node, never `kill %N`.
  **`bash tools/stragglers.sh` is the check**: it greps both machines and exits non-zero
  with the pid and full path of anything that survived. **Its pattern list is only
  as good as the last thing somebody remembered to add** — on 2026-09-12 three
  `ros2 run image_transport republish` processes were found three and a quarter
  hours old, subscribing to the one topic that crosses Wi-Fi, with the sweep
  reporting 0 on both machines throughout. `republish`'s subscription is *lazy*, so
  `ros2 topic info -v` showed 0 subscribers while all three were alive. Patterns
  are path-anchored for a reason that bit again here: the first spelling of that
  pattern matched prose, so the sweep reported the shell running it. Every `pkill -f` and
  `pgrep -f` pattern is bracketed and path-anchored (`/lib/[p]imesh_hello/`) —
  see the troubleshooting entry on why the plain spelling kills the shell that
  runs it.

  **It happened again on 2026-09-14, and that time the missing patterns were the
  *wrappers* rather than another process.** Every pattern matched the leaf of the
  Pi's four-deep chain — the installed binary — and none matched the login shell,
  the `timeout` or the `ros2 run` above it. One second after a remote start the
  sweep printed `stragglers on pi: 1` where `pgrep` at the far end listed three
  of ours, and during the startup window it printed 0 over a chain that was about
  to open `/dev/video0`. `PIMESH_RUN_PAT` and `PIMESH_PI_WRAP_PAT` close it. The
  question to ask of a new pattern is not "is this a process we start" but "what
  starts it, and would the sweep see *that* in the second before the node
  exists".

  **`assert_no_session` is the same question asked before the fact**, out of the
  same pattern list and the same `pimesh_local_processes`, so "what of ours is
  running" has exactly one spelling. The sweep says what outlived a session; the
  guard refuses to start beside one. Every script that starts a session calls it,
  gates included, before `arm_cleanup`.
- **Teardown verifies; it does not fire and return. Measured 2026-09-14, and
  this is the rule the rest of the bullet above was missing.** `kill_pi` used to
  be one line — one `pkill -f` of the node pattern, over one ssh whose failure
  was discarded twice — and it returned 0 whether the far end died, was never
  matched, or was never reached. A `just view-keypoints` closed at the window
  therefore exited believing itself clean while a `camera_node` went on holding
  `/dev/video0` on the Pi, and the next session refused to start. The sweep
  reported a clean dev box throughout.

  Two mechanisms, and both generalise past this one function:

  1. **A pattern kill can only end what already exists.** `pi_run_for` puts a
     login shell, a `timeout` and a `ros2 run` between ssh and the node, so a
     kill aimed at the leaf can land *before the leaf exists*, match nothing,
     report success, and leave the wrapper to exec it a moment later. Measured:
     `kill_pi` half a second after a remote start returned 0, and twelve seconds
     later the Pi had the whole chain running. So teardown kills every link of
     the chain — `PIMESH_RUN_PAT` and `PIMESH_PI_WRAP_PAT` are there for this —
     and then **keeps asking until two consecutive sweeps come back empty**,
     which is a terminal answer rather than a snapshot, because with the
     wrappers dead nothing can create a node.
  2. **A cleanup that cannot fail is a cleanup nobody checks.** `kill_local` and
     `kill_pi` now return non-zero when they could not get their machine clean,
     print the pid and full path of what survived on stderr, and
     `_pimesh_on_exit` turns that into the script's exit status. A viewer may
     exit 0 for having shown somebody a picture; it may not exit 0 having left a
     `camera_node` on the Pi. An unreachable Pi lands in the same branch on
     purpose — the sweep cannot tell "asked and found nothing" from "could not
     ask", and reporting the second as a failure to clean is the reading that
     sends somebody to look.

  **Killing the local `ssh` does not reach the far end** (measured the same day:
  the remote `timeout`/`ros2 run`/`camera_node` chain carried on after its client
  was killed), so the client is housekeeping and `kill_pi` is the teardown. That
  is also why `kill_local` runs first: it ends this session's client before
  `kill_pi` opens a connection of its own.

  **Verifying costs about five seconds and it is worth them.** Closing
  `view-camera`'s window to the recipe returning with both machines swept clean:
  **6.94, 7.42 and 8.99 s** over three runs (2026-09-14), against roughly 2.5 s
  for the fire-and-forget version that could not tell you whether it had worked.
  `PIMESH_TEARDOWN_SECONDS` (default 20) is the ceiling on how long each half
  keeps insisting before it gives up and says so; it is only ever paid when
  something is genuinely refusing to die.
- **This applies to ad-hoc runs too — that means you, Claude.** Anything you
  start by hand while verifying has no EXIT trap. Bound it up front —
  **`timeout --foreground -s INT 30 …`**, and on the Pi
  `ssh pi 'timeout --foreground -s INT 30 bash -lc "…"'` — or `pkill -f` it when
  done, and check both hosts are clean before reporting. The `--foreground` is
  not decoration: without it the command is in a process group your own Ctrl-C
  cannot reach, so a run you meant to bound becomes one you cannot interrupt.
  **A leaked camera process holds `/dev/video0` exclusively** and every later
  session dies with `Device or resource busy`.

  **And check the machine is yours before you measure on it, not only after.**
  A number taken while somebody else's viewer is up is a number about a mixture
  of two sources — that is how an afternoon went on 2026-09-13, with a flood
  reproduced under `just replay` that `replay.sh` could not possibly have caused.
  `bash tools/stragglers.sh` before the first measurement, and read the user's
  terminal for a session they have open; the scripts now refuse on their own, but
  an ad-hoc `ros2 launch` you type yourself does not.
- **Claims are closed by scripts, not by eyes.** A gate names its evidence: a
  number on a topic, a log line with a threshold, a rendered image file. Reserve
  "needs a human" for the physical world — a tape-measure scale check, exposure
  in a real room, whether the mesh looks like the room. The RViz window is a
  viewer, not the evidence.
- **Docs split by kind**: reference lives in `docs/info/`; `docs/plans/` holds
  only the rules and the future files.
- **A plan is a GitHub issue, never a markdown file in this repo.** When asked
  to write a plan, open one with
  `gh issue create --label plan --title "<name> plan — <one line>" --body-file <file>`
  — the issue body *is* the plan. Do not create `docs/plans/*.md`, and do not
  paste a plan into chat instead of filing it. Add the `deferred` label if it is
  written down but not being started now.
- **Completion is closing the issue** —
  `gh issue close <n> --reason completed` — and that is the only status change
  there is. Close it only when **every phase is annotated done in the body and
  its gate has been run**, not when the code exists; abandoned work is closed
  with `--reason "not planned"` and a comment saying what replaced it. A closed
  plan issue is the **build log**: annotate phases as you go with the date and
  what the test printed, and never edit that history out. Fix inbound links in
  `docs/plans/README.md` and here when a plan's status changes.
- **Every plan is a list of executable phases, and nothing else.** The
  three rules, in full in [docs/plans/README.md](docs/plans/README.md):
  1. **Stable phases.** `## P0`, `## P1`, … Once written, a phase's number and
     scope never change, so "P3" means the same thing in every doc, commit
     message and conversation. Record progress by annotating the phase with
     dates and what actually happened — never by renumbering or reshuffling.
  2. **Every phase ends in a test that is a command.** Not "verify it looks
     right", not "check the mesh" — a recipe someone can run that exits 0 or
     non-zero and prints the number it asserted on. The test recipe is written
     in the same change as the phase's code.
  3. **Executable phases only.** A phase must be startable now, by the person
     reading it, with the hardware and code that exist. Anything that is
     waiting on something — a later phase, a purchase, an upstream release, or
     the passage of time — **is not a phase**. Never write a phase like "check
     back in 48 hours", "monitor for a week", "revisit once we have more data",
     or "decide later whether to keep it".
- **Deferred work lives in `docs/plans/future/`, never in a plan.** Each plan
  issue may have one companion `docs/plans/future/<name>-future.md` holding the items
  that are not executable yet. Every entry there names **the trigger that would
  make it executable** — the measurement, the phase, or the hardware it is
  waiting on. When the trigger fires, the entry is deleted from the future file
  and appended to the plan **issue's body** as the **next unused phase number**,
  with a test. Moving work into a plan is the only way it gets built; moving it
  into the future file is the only way it gets deferred. It never sits in both.
- `build/`, `install/`, `log/`, model weights and bag files are git-ignored.

## Working here

This is a **learning project**: the value is in understanding ROS 2, real-time
C++ and 3D reconstruction, not in shipping a product. So when implementing
something, explain the concept it exercises — why this QoS, what the executor is
actually doing, why the TF tree is shaped this way. Prefer the idiomatic ROS 2
way over a shortcut that happens to work. Build in steps that each run rather
than generating a finished subsystem the user cannot reason about.

Before writing code that touches an area, read its doc in `docs/info/` and the
corresponding `piros2` implementation. Most sharp edges here have already cut
somebody once.

## Documentation map

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/info/architecture.md](docs/info/architecture.md) | The node graph, topics, TF tree, and where each stage runs |
| [docs/info/pipeline.md](docs/info/pipeline.md) | Stage by stage: algorithms, libraries, message shapes, cost budgets |
| [docs/info/dashboard.md](docs/info/dashboard.md) | The web dashboard: transport, payloads, mesh streaming, layout |
| [docs/info/hardware.md](docs/info/hardware.md) | Measured specs of both machines, the camera, and the GPU |
| [docs/info/setup.md](docs/info/setup.md) | Getting both machines to build and run this, including the GPU stack |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause, mostly inherited and worth reading before debugging |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones and their status — M0–M9 are the two halves that run, M10–M19 are the SLAM work that closes the loop |
| [docs/info/build-log.md](docs/info/build-log.md) | **How this got built, and the dozen times a number was wrong before it was right.** Moved out of this file 2026-09-23. Read it before adding a gate |
| [docs/info/testing.md](docs/info/testing.md) | The 433 unit tests: what each suite exists to catch, and why a wrong version of that code would otherwise look fine |
| [docs/plans/README.md](docs/plans/README.md) | How a plan is written here: a GitHub issue of stable phases, a command for a test, executable-only, and the future file |
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | **Where this is going.** The whole pipeline as phases P0–P8, none started, each ending in a `tools/gates/*.sh` test, followed by the deferred register |
| [#9](https://github.com/bthek1/ros2_pi/issues/9) **(closed 2026-09-12)** — camera calibration | **P9, done.** The C922's real intrinsics at 720p: fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px. `camera_node` loads them from `pimesh_bringup/config/camera_info/c922_720p.yaml` and the NOMINAL warning is gone. Read the closed issue before touching calibration — three of its assumptions turned out to be false, including that this camera has barrel distortion |
| [#4](https://github.com/bthek1/ros2_pi/issues/4) [#5](https://github.com/bthek1/ros2_pi/issues/5) [#6](https://github.com/bthek1/ros2_pi/issues/6) [#7](https://github.com/bthek1/ros2_pi/issues/7) **(all closed)** [#8](https://github.com/bthek1/ros2_pi/issues/8) — milestones A–E | **The pipeline, built.** All five closed: the cross-distro workspace and capture (P0, P1), one reader and one decode with keypoints (P2, P3), depth on the GPU at 55.10 ms/frame in the container (P4), a TSDF at 15.3 ms per integration and a triangle surface out of it every ten seconds (P5, P6), and **E as of 2026-09-19 — 6-DoF odometry, the rotation that had been composed inverted since P3, and a browser tab that costs the pipeline 0.77% (P7, P8, P10).** Five issues over the *one* phase list in `project_final_state.md`, a contiguous slice each: A = P0–P1, B = P2–P3, C = P4, D = P5–P6, E = P7–P8 plus P10, promoted out of milestone A's future file when the dashboard gave it a consumer. No issue renumbers from zero. Each also has a `just view-*` recipe — a viewer for a person, never a gate. **What comes next is milestones F–I, filed 2026-09-23 as #10–#13** — see the row below |
| [#10](https://github.com/bthek1/ros2_pi/issues/10) [#11](https://github.com/bthek1/ros2_pi/issues/11) [#12](https://github.com/bthek1/ros2_pi/issues/12) [#13](https://github.com/bthek1/ros2_pi/issues/13) **(all open, none started)** — milestones F–I | **Turning the pipeline into monocular visual SLAM.** Phases P11–P20 on the same one list, four contiguous slices. **F (P11–P13) is the only one not labelled `deferred`, and it is first for a reason that is this file's whole argument**: nothing in P0–P10 ever measured the pose against a truth, so P11 is a dataset source, a TUM-format trajectory and an ATE through `evo`, and P12–P13 are the visit to the room that everything else has been waiting on — a tape measure for `depth_scale`, and `bags/walk1`. Then **G (P14–P15)**: `MapPoint`s observed by many keyframes and local bundle adjustment, which is the noun this pipeline does not have — a landmark currently dies with the keyframe that saw it, and `rgbd_odometry` poses each frame against the **newest keyframe only**. Then **H (P16–P18)**: place recognition against the whole store, a pose graph that finally publishes `map -> odom` for real, and a TSDF **rebuilt** at the corrected poses — without that last one a closure fixes the trajectory and leaves the room where it was. Then **I (P19–P20)**: a `LOST` state that stops fusing, and relocalisation. **One dependency was checked before any of it was filed**: g2o ships with ROS at both ends — `ros-lyrical-libg2o` and `ros-jazzy-libg2o`, both 2020.5.29, both exporting `g2o::core`, `g2o::types_sba`, `g2o::types_sim3` and `g2o::solver_eigen` under identical names — so the cross-distro tax here is **zero** and no `depth_engine_null.cpp`-style conditional engine is needed. **DBoW2 is in neither apt**, so P16 starts brute force with the trigger written down |
| `bash tools/gates/ipc.sh` / `bash tools/gates/keypoints.sh` | **P2 and P3's gates.** `ipc.sh` runs the real container twice against the Pi's live camera and compares published buffer addresses with intra-process comms on and off — and asserts the decoded topic has at least two subscribers, because one consumer is the configuration that cannot fail. `keypoints.sh` replays `bags/desk1` and measures three things three ways: the rate from a C++ subscriber's steady clock, the per-frame cost from the node's own log line, and the matched-keypoint fraction against `tools/orb_reference.py` — the predecessor's algorithm reimplemented in Python over the same clip, which is the only part of the gate with an outside opinion about whether the corners mean anything |
| `bash tools/gates/depth.sh` | **P4's gate.** Replays `bags/desk1` through the real container — `decode_node`, `keypoint_node` and `depth_node` in one process — and measures four things. The provider, off `depth_node`'s own startup line. The per-frame cost on the node's own clock, against 80 ms. Whether `/depth/rgb` is **byte-identical** to the `/image_raw` frame with the same stamp, by hashing every source frame as it goes past and comparing — with "could not check" counted separately from "checked and differed", because a run that checked nothing would otherwise report zero mismatches and look perfect. And a **control**: the same binary with `use_cuda:=false`, which has to *fail* the same budget. `depth_probe` is its instrument, loaded into the container with `probe:=depth_probe` — out of process it would be subscribing to ~255 MB/s of images and would be the dominant load on the thing it is measuring |
| `bash tools/gates/dashboard.sh` / `bash tools/dashboard.sh` | **P8's gate and the page itself.** The gate replays `bags/desk1` **three** times — with a client, without one, and without one again — because the third run is what makes the first assertion possible: two identical runs differ by up to 15% on `fusion`, so the bound is the measured noise floor plus slack rather than P8's flat 2%, which would be a gate that fails on the weather. It also kills the client at the halfway mark and compares the seconds after against a *control's* same seconds, since a run's second half is systematically slower than its first; verifies the WebSocket handshake against the probe's own key; asserts both image strips arrive near their caps (two caps in series halve a stream while every number looks right); and asserts the pose says STALE 2.10 s after `/odom` stops. `dashboard_probe` is its instrument — a C++ WebSocket client and **not a headless browser**, which would add a 200 MB process and a GPU context to the thing being measured. **Its rate table needs an idle box and says nothing on a busy one**: the bound comes from the two runs with nothing attached, so interference landing on the *client* run inflates the number being judged while leaving the bound narrow — measured 2026-09-21, three runs on a desktop in ordinary use gave noise floors of 1.00%, 6.56% and 1.34% and a verdict that tracked the floor rather than the code. The timing-independent half (handshake, six stage rows, `mesh=3`, **0** dropped to clients, STALE at ~2.0 s) is stable and is what to read when the machine is not yours. See the constraint bullet above |
| `bash tools/gates/odom.sh` | **P7's gate, and it runs the clip twice — `odom_regime:=sixdof` and the `rotation_only` control — with `odom_probe` loaded into the container.** It asserts the control publishes translation *identically zero* (anything else means translation is leaking into the run everything is measured against); that the 6-DoF run reports a trajectory whose **fastest published motion** is bounded — a speed and not a displacement, because a step taken after a run of holds spans several depth intervals and bounding the step alone compares it against the wrong clock; that the solve reaches ≤ 2 px over ≥ 55% of depth frames; and that **both** regimes' median paired-surface gap is under a ceiling the *pre-P7 composition fails*, which is what makes it an assertion rather than a number nobody has seen excluded. It **prints rather than asserts** the comparison P7 asked for — the two regimes are a dead heat on this clip, 0.4456 m against 0.4471 m — and names the trigger in its own output. `odom_probe` measures off `/odom` rather than off `keypoint_node`'s own counters, because the failure P7 fixed was a pose that every internal number described correctly |
| `bash tools/gates/fusion.sh` / `bash tools/gates/mesh.sh` | **P5 and P6's gates, and both of them run a control.** `fusion.sh` replays `bags/desk1` twice, once with `align:=false`, and measures the integration cost on the node's own clock, the rate, the mailbox drop fraction, the arrival-to-integration lag against one depth frame interval, and whether every frame found a pose at its own stamp and its exact colour twin. It **prints rather than asserts** the paired-surface comparison P5 asks for, because measurement said the two runs were a coin flip — and says so in its own output. **P7 was named there as the trigger, it fired, and the comparison moved**: re-measured 2026-09-19 with the corrected pose, 0.4805 m aligned against 0.5330 m unaligned and agreement 0.2191 against 0.1606, the aligner ahead on both for the first time. It stays printed rather than asserted because that is one run of a number which has already flipped once. `mesh.sh` replays it twice again, the second with `remesh_period_s` past the clip so nothing meshes, because P6's `max ≤ 2× median` is not achievable against a clip that stalls 400 ms on its own. It asserts the worst integration gap is no worse with meshing than without, the published Marker is under the cap **read off the topic**, the boundary-loop count *falls* across the fill and stays **above zero** (pinholes closed, frontier open — a sealed box is the most seductive false positive in this project), the saved PLY has *more* triangles than the Marker, and three offscreen renders have a surface in them |
| `just view-odom` | **P7's viewer, and the only one here meant to be run twice.** `bash tools/view-odom.sh 600 desk1` and `bash tools/view-odom.sh 600 desk1 rotation_only` put the two regimes side by side: rotation-only piles every arrow at the origin and spins in place, 6-DoF draws an arc through space. An RViz **Odometry** display with `Keep: 500` draws the trail straight off `/odom`, so no `nav_msgs/Path` publisher exists anywhere in this project. A viewer, not evidence — `gates/odom.sh` is what passes or fails P7 |
| `bash tools/mesh-views.sh <mesh.ply>` / `just view-mesh` | **The evidence and the viewer, and they are not the same thing.** `mesh-views.sh` renders three fixed angles offscreen through a numpy software rasteriser — no GL, no display, no Open3D — and reports what fraction of each frame is surface, which is what tells a real mesh from the black rectangle that an empty volume, a camera inside the geometry and a sign error in the projection all produce. Those PNGs are what closes P6. `view-mesh.sh` is for a person, and its checklist says plainly that the surface will not look like a room until P7 gives odometry a translation |
| `bash tools/fetch-gpu-stack.sh` / `bash tools/fetch-model.sh` / `bash tools/gates/gpu-stack.sh` | **P4's toolchain, which is as far as milestone C has got.** The first installs ONNX Runtime 1.30 + CUDA 13.1 runtime + cuDNN 9.26 into `~/.local/opt/pimesh-gpu` with no sudo, every component version-pinned and sha256-verified; the second does the weights. The gate is four runs and three of them are controls — CUDA at 51.20 ms, the CPU provider at 213.18 ms (so the 80 ms budget is shown to *discriminate* rather than merely be met), a build with CMake's default linker flags that reaches only the CPU (so `-Wl,--disable-new-dtags` cannot quietly stop being load-bearing), and `nvidia-smi` sampled while the first runs, which is the only witness here that does not go through ONNX Runtime. `tools/gpu_probe.cpp` is its instrument: no ROS, no colcon, compiled by the gate with `g++` so that what is being tested is the toolchain and not four things at once |
| `bash tools/record-clip.sh desk1 60` | **The reference clip.** A 60 s hand-held sweep, recorded once, that every phase from P3 on replays so the numbers compare like for like. `bags/` is git-ignored, so a fresh clone has none and `gates/keypoints.sh` says so rather than pretending. The script resets the camera's V4L2 controls first and records `/camera_info` alongside the frames, because a clip recorded at 20 fps under a stale manual exposure cannot be un-recorded |
| `bash tools/replay.sh` / `view-camera.sh` / `view-keypoints.sh` / `view-depth.sh` | **The four viewers, and none of them is evidence.** `replay` loops a bag with `pipeline:=false` — the static frame tree and no components, because a pose published over a looping bag freezes and floods every TF listener; `view-camera` is the Pi's live camera; `view-keypoints` is the pipeline, on the camera or on a bag **played once** for the same reason; `view-depth` is the same again with the room as a colour-mapped depth cloud, and it waits longer before starting RViz because `depth_node` loads a 99 MB model and warms a CUDA session first. All four call `assert_no_session` before `arm_cleanup`, as does every gate: two sessions on one domain put two publishers on `/image_raw/compressed` and make both of them look broken |
| `bash tools/test.sh` / `bash tools/gates/test.sh` | **The unit tests** — **433 across twenty-eight suites, identical on both distros.** What each suite exists to catch, and the three recurring reasons any of them exist, is [docs/info/testing.md](docs/info/testing.md). The short version: a suite is here when a wrong version of the code produces a *plausible* result rather than a crash |
evaluates the launch file's `pipeline` condition both ways, so `pipeline:=false`
cannot quietly stop removing the container —  the calibration loader's refusals in `test_calibration`, and the calibration gate's own instrument in `test_straightness` — which measures a chessboard projected through a *known* K and D and is what makes `gates/calibration.sh`'s pixel figure worth asserting on — plus milestone B's four: `test_mailbox` (newest-wins and its drop accounting), `test_image_buffer` (the bgr8 layout arithmetic, that a `cv::Mat` over a message shares its memory, and since 2026-09-19 the same over a 32FC1 depth map — `depth_mat_over` had lived in an anonymous namespace inside `fusion_node.cpp` where no test could call it, and every distance the TSDF integrates comes through it), `test_rotation_fit` (Kabsch against known rotations, the reflection guard, the reject-worst refits, and the optical-to-body change of basis) and `test_orb_tracker` (synthetic frames with a known displacement: that the window forgives detection churn, that unrelated scenes do not match, and that a track id is never claimed twice in one frame); milestone D's five — `test_tsdf_volume` (a plane comes back out at the distance it went in; a corner ray reports **z and not ray length**, which agrees perfectly at the principal point and is 30% wrong in the corners; the signed distance is positive in *front* of the surface, because inverting it finds the identical zero crossing and winds every triangle the other way; nothing is written more than a truncation behind a wall; and a surface that jumps further than the band **leaves a ghost**, pinned as behaviour rather than left to be discovered), `test_scale_aligner` (that a constant bias produces corrections whose product is exactly 1 — the property that separates a high-pass from a feedback loop that walks the map away at 1% a frame, and the one thing no running system can show you), `test_marching_cubes` (a sphere comes out **closed**, which is how 256 entries of copied lookup table get checked at all: one wrong case leaves one edge used once) and `test_mesh_cleanup` (a hole is filled and the *rim is not*), `test_mesh_io` (the saved PLY, read back **both** ways: through `read_ply`, and byte by byte against the layout the header promises — because writer and reader being wrong *together* round-trips perfectly, and P6's evidence is a triangle count taken off that file through `read_ply`. Measured 2026-09-19: with colour written before position in both halves, the round-trip test passes and only the byte-level one fails. It also covers the refusals a gate instrument has to make rather than guess at — an ASCII PLY, a truncated file, a quad, and a failed read leaving the caller's mesh empty instead of the previous surface), `test_shared_volume` (the rendezvous the two world nodes find each other through, and the chunked copy between them: the same key is the same object, a chunked snapshot equals an unchunked one whatever the chunk size, the snapshot is a *copy* and does not move when the volume does, and the weight filter changes the memory and **not the mesh** — all of which fail by producing a surface) and `test_mesh_render` (the instrument P6's evidence comes out of: that the three fixed views are **three different pictures**, which a renderer ignoring its azimuth would fail while satisfying every check the gate makes, and that colour survives the trip to cv2's channel order) — and P7's two — `test_rgbd_odometry`, whose **CameraStep** suite is the one that closes the loop `test_rotation_fit` left open (simulate a camera with a known trajectory, show it what it would have seen, run the whole estimator, and assert the pose that comes out is the trajectory that went in — the check whose absence let the rotation be composed inverted for six days while every other property of that pose was right), plus — since 2026-09-23 — the **OdomCovariance** suite, which pins the one property RViz checks on every `/odom` message and which nothing in this workspace reads: that the 6x6 block is symmetric with a non-negative diagonal. It published `-1` in element 0 from P7, sensor_msgs/Imu's sentinel attributed in a comment to nav_msgs, and so shipped a matrix that is not positive semidefinite — invisible to every gate and audible at 17 Hz in the one viewer that decomposes it; plus the depth-reading refusals that keep a corner on an occlusion edge out of the fit, the reflection guard, and the scale story: that dividing the network's per-frame scale out removes 4% of breathing without eating a real translation, that a flat wall is **not** the degeneracy it looks like (a camera moving forward leaves a landmark's x and y untouched where a scaling shrinks them, so the two separate on any patch that subtends an angle), and that the one geometry where they *do* come apart — a 5° cone at a single distance — is milder with the robust refits than an unrobust fit suggests, 0.961 rather than 0.75; and `test_keyframe_store`, which exists precisely because nothing downstream would notice it being wrong — a threshold that never fires and one that fires every frame both produce a pipeline that runs, one keyframe a session against eight thousand, with only a memory figure to tell them apart; and milestone C's one, `test_depth_model` — the arithmetic either side of the network, which is the whole of P4 that can be got wrong in silence: a channel order swapped, planes interleaved instead of planar, or a reciprocal taken before the clamp each produce a depth map that renders as a plausible room and is numerically nonsense. It needs no ONNX Runtime, no GPU and no camera, which is *why* `depth_model.hpp` is a header separate from the engine behind it — so this suite runs identically on the Pi. It also covers the colour preview's mapping, where a single sign decides whether near is bright or the 6 m clip is: flip it and the picture is still a perfectly plausible depth image of exactly the wrong thing. `test_orb_reference` — P3's outside opinion, which decides `gates/keypoints.sh` and had no tests of its own until 2026-09-19: the one-to-one track claim, the Hamming threshold in bits, the pooled window forgiving churn and forgetting past its length, and that a featureless frame is *excluded* rather than scored zero. Two of those fail **upwards** — a reference that matches many-to-one agrees with the node more readily — so the failure is a gate that passes for the wrong reason. It runs on synthetic descriptors with no bag and no camera. And `test_stats` — the percentile and the FNV-1a hash every probe reports its numbers through, which had no tests because they had no *home*: four copies of one and two of the other, each in an anonymous namespace inside a translation unit with a node in it. The first run of that suite found the FNV-1a offset basis had been wrong since milestone A. And P8's three, added 2026-09-21, all of which fail by **blanking a panel rather than erroring**: `test_json` — `quote` and `number`, the two functions every byte of the page is built out of, which had lived in an anonymous namespace inside `dashboard_node.cpp` where nothing could call them. A stray quote in a `detail` string another node wrote, or a rate that came out NaN, is a *parse error* in the browser and not a bad value — `JSON.parse` throws in `onmessage`, `onStats` never runs, and the stage table holds its last value with the socket up and the pipeline fine, which is exactly what a **stalled pipeline** looks like. It pins the escapes against literal expected bytes rather than round-tripping through a parser written beside them, for `test_mesh_io`'s reason, and asserts over all 256 byte values that whatever a node writes comes back out as a string literal a parser reaches the end of; `test_mesh_payload` — the 4/12/3 wire layout of the one payload here whose reader is hand-written JavaScript, pinned byte for byte: positions planar and not interleaved with colour, the count little-endian, the clamp **before** the cast (an out-of-range float wraps rather than saturating, so an over-bright vertex renders *dark*), a partial colour array ignored rather than paired up by index, and an empty refusal that is not a zero-vertex payload — because a four-byte 'zero vertices' would *clear* a surface that was fine. Writing it corrected the payload size this project had recorded: 120 k triangles is **5.4 MB**, not the 4.3 MB two files claimed, which was the position array with the colour bytes forgotten; and `test_dashboard_contract` — the pairs that cross a language boundary and so agree only because somebody typed them twice: the five channel numbers in `web_server.hpp` against the `CH` table in `app.js`, every JSON field `app.js` and `dashboard_probe.cpp` read against what `stats_json` and `pose_json` send, the stage *strings* the page keys its two headline figures on (`by.depth`, `by.capture`) against what any node actually publishes, and the mesh offsets at the far end where no C++ test can reach. It reads both files as text, which is inelegant and is the only thing that can check it at all. **All three were checked by mutation before being kept** — 21 deliberate breakages, each caught by the test that claims to catch it |
| `gh issue list --label plan --state all` | **The plans themselves.** [#2 hello-world](https://github.com/bthek1/ros2_pi/issues/2) — closed 2026-09-08, the build log for the scaffolding that exists; [#3 justfile](https://github.com/bthek1/ros2_pi/issues/3) — closed 2026-09-09, why the shell lives in `tools/`; the justfile was trimmed further the same day to `build` + `run` only, so that issue's `just gate-*` spelling is history, not instruction |
| [docs/plans/future/milestone-d-future.md](docs/plans/future/milestone-d-future.md) | Work deferred out of milestone D, each entry with its trigger. **The one that needs a person is first**: pinning `depth_scale` with a tape measure and recording `bags/scale1` while you are at the wall, without which every distance this pipeline reports is plausibly shaped and the wrong size. Then re-measuring whether scale alignment helps once P7 gives odometry a translation; keeping the integrated frames so a loop closure can rebuild the volume; free-space carving; a smaller voxel record; and a watertight companion mesh |
| [docs/plans/future/milestone-a-future.md](docs/plans/future/milestone-a-future.md) | Work deferred out of milestone A, each entry with its trigger: the checkerboard calibration (waiting on P5's tape-measure visit), `PipelineStats` from `camera_node` (waiting on the dashboard), the dev-box rate margin, and device reconnection |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note
the date.
