# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

**One webcam on a Raspberry Pi, a live 3D mesh of the room on the dev box — in C++.**

The camera is the only sensor. Everything downstream is inference and geometry:

```
RGB frame → keypoints (ORB) → monocular depth (Depth Anything V2) → TSDF fusion → triangle mesh → dashboard
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

### Status: the scaffolding runs, the pipeline does not

As of **2026-09-09** there is exactly one package, `src/pimesh_hello/`, and it
exists to prove the structure rather than to do anything: a C++ `ament_cmake`
package, two `rclcpp_components` components composed into one container with
intra-process comms measured handing over the pointer, parameters from a keyed
YAML, the same source built from scratch under **both** distros, and a session
that tears itself down on either machine. Five scripts in `tools/gates/`
assert all of it — [gh issue #2](https://github.com/bthek1/ros2_pi/issues/2)
carries the numbers each one printed.

**The teardown claim had to be earned twice, and the way it failed is worth more
than the fix.** It was true for `hello-lan` and false for `hello-compose` until
2026-09-09: a foreground `timeout` had put `ros2 launch` in a process group the
terminal's Ctrl-C never reached, so the recipe swallowed six of them and ended
on its own when the timer expired. The gate said PASS throughout, because it
only ever signalled the *other* recipe. That is a green gate over broken
behaviour — worse than no gate, because it is a false claim with a script's
authority behind it. Both halves are fixed (`run_for`, and a gate that signals
both recipes), and the lesson is the one to carry into every later phase: ask
what the gate does **not** touch.

### The pipeline has started: capture is real

**As of 2026-09-09 the first two phases are built and measured** — milestone A,
[gh issue #4](https://github.com/bthek1/ros2_pi/issues/4). `pimesh_msgs`,
`pimesh_bringup` and `pimesh_camera` join `pimesh_hello`, all four building from
source under both distros, and the Pi puts stamped 720p MJPEG on the LAN:
**44–59 Hz received on the dev box** (`bash tools/gates/capture.sh`), 0 duplicate
payloads, **4.21 ms** median dequeue-to-subscriber measured on the Pi's own
clock, and two launches agreeing on their stamp offset to **0.30–1.02 ms** —
which is the assertion that `usb_cam` 0.8.1 fails by hundreds of milliseconds.

**Decode and keypoints are built, and milestone B is closed as of 2026-09-13** —
P2 and P3, [gh issue #5](https://github.com/bthek1/ros2_pi/issues/5).
`pimesh_perception` joins the four packages: one container, one network subscriber,
`cv::imdecode` at **1.87–1.93 ms/frame** keeping up with the Pi's full 59.4 Hz, and
the decoded 2.7 MB buffer reaching its consumers at the address it was published
from — **529/529** with intra-process comms on against **0/387** with it off
(`bash tools/gates/ipc.sh`). ORB then runs at **57.9 Hz sustained, 5.99 ms/frame**
on the node's own clock against an 8 ms budget, with a matched-keypoint fraction of
**0.9063** against the predecessor's algorithm at **0.9065** over the same 3489
frames (`bash tools/gates/keypoints.sh`), and publishes a rotation-only
`odom -> base_link` that holds its last pose rather than guessing when its gates
fail (8.2% of frames on the reference clip).

**`bags/desk1` is the reference clip** — 59.7 s, 3489 frames at 58.5 Hz, 220 MB,
sha256 `1333c5bd…`. `bags/` is git-ignored, so that hash is its only identity, and
every phase from here measures against the same seconds of room. Record one with
`bash tools/record-clip.sh <name> <seconds>`; it resets the camera's V4L2 controls
first, because a clip recorded at 20 fps under a stale manual exposure cannot be
un-recorded.

**Three of that milestone's measurements were wrong before they were right, and
every one of them was the gate rather than the code.** They are the most useful
thing it produced, because each was a *false green or a false red that no number
looked wrong in*:

1. **The zero-copy claim passed while measuring the case that cannot fail.**
   429/429 with one consumer, then **0/574** on the very next run with a second
   consumer beside it — same code, same flags. rclcpp moves the buffer into the
   *last* ownership-taking subscription and **copies it for every other**; a fan-out
   wants `ConstSharedPtr`, which rclcpp hands to all of them at once.
   `gates/ipc.sh` now asserts at least two subscribers.
2. **A 20 s window of a 60 s clip was compared against that clip's average**, and
   reported an 11-point regression in a tracker that was working. A hand-held sweep
   is not uniform, so *which seconds you measure* moved the answer further than a
   real regression would. The gate now plays the clip once, start to finish, and
   asserts that ≥85% of its frames reached the probe.
3. **A frame with no features was a matched fraction of zero to one side and
   skipped by the other** — 0/0 is undefined, not zero, and counting it as zero
   also conflates "no corners in this part of the room" with "corners found and
   none recognised". Worth ~5 points, which was the whole tolerance.

And the workspace had been compiling with **no optimisation flags at all**, so
every C++ cost this project had ever measured was a `-O0` number. P3's budget was
what found it, at 7.90 ms against an 8 ms ceiling where `-O2` gives 5.99 ms.

**Milestone C is closed as of 2026-09-15 — the pipeline publishes distances.**
P4, [gh issue #6](https://github.com/bthek1/ros2_pi/issues/6). The toolchain was
done first and standalone, as the phase demands: `bash tools/fetch-gpu-stack.sh`
installs ONNX Runtime 1.30, CUDA 13.1's runtime libraries and cuDNN 9.26 into
`~/.local/opt/pimesh-gpu`, pinned and sha256-verified, with no sudo;
`bash tools/fetch-model.sh` does the same for the weights; and
`bash tools/gates/gpu-stack.sh` closes *that* claim with **51.08 ms mean, p95
51.36 ms** for inference alone, three controls beside it (a CPU run at 181.95 ms,
a default-linker-flags run that reaches only the CPU, and `nvidia-smi`
independently witnessing the process holding a compute context).

Then `depth_node`: a component in the same container as `decode_node` and
`keypoint_node`, publishing `/depth` (32FC1 metres) and `/depth/rgb`.
`bash tools/gates/depth.sh` replays `bags/desk1` through the real container and
reports **`CUDAExecutionProvider`, 55.10 ms mean per frame and 58.21 ms p95
against an 80 ms budget**, **17.42 Hz** sustained on `/depth`, **1048 of 1048**
`/depth/rgb` frames byte-identical to the `/image_raw` frame with the same stamp,
1045 depth stamps matched to an input frame and **0 not**, and **0** non-finite or
out-of-range values in 966,625 sampled distances — with a control run, the same
binary one parameter apart, at `CPUExecutionProvider` and **287.92 ms**, outside
the same budget.

**The most valuable thing that phase produced is a bug the gate before it could
not have seen, and it is the `--disable-new-dtags` lesson one level deeper.**
`gates/gpu-stack.sh` proves the GPU stack works for a program *this workspace
links*. A `rclcpp_components` component is loaded into
`component_container_isolated`, which is somebody else's executable — and for a
**dlopened** object's dependencies, glibc consults the object's own `DT_RPATH`, its
*loader chain's*, and the **main executable's**, of which a dlopened object has no
loader chain at all. So the only `RPATH` that could apply was the executable's, and
ours was not it. Measured, same libraries and same flags, one container apart:
`gpu_probe` at 51 ms on CUDA, `depth_node` at **517 ms on the CPU**, with the whole
pipeline working perfectly around it and no error in any log but one line naming
`libcublasLt.so.13`. The flag had not stopped mattering; it had stopped *reaching*.
`preload_cuda_provider()` in `depth_engine_ort.cpp` is the fix — load the CUDA
libraries by absolute path before ONNX Runtime asks for them — and
`gates/depth.sh` exists in the shape it does because `gates/gpu-stack.sh`
structurally cannot cover this: its instrument is an executable.

**And adding one node to the container turned a passing gate into a false green.**
`gates/keypoints.sh` read the per-frame cost with `grep 'stats rate=' | tail -1`,
which was unambiguous for exactly as long as one node logged a line beginning that
way. `depth_node` logs one too, so `tail -1` started returning *its* last window —
the seconds after the clip ended, `cost_mean=0.00` — and the gate asserted 0.00 ms
against an 8 ms budget and printed **PASS**. Nothing in the output looked wrong
except a zero, and a zero in a cost field reads as "fast". It now selects by node
name, takes the last window with frames in it, and **asserts the cost is greater
than zero**, because a per-frame cost of exactly zero is not a measurement. With
that fixed, ORB measures **5.75 ms** with depth beside it.

**Three of that afternoon's findings are in the constraints list below and all are
the same shape: a wrong thing that resolved, loaded and ran.** A stub cuBLAS that
`ldd` was perfectly happy with and that segfaulted on first use, a linker flag
whose absence costs the GPU with no error message anywhere, and that same flag
being correct and irrelevant once the code moved into a container.

**Everything downstream of depth still does not exist.** No fusion,
no mesh, no dashboard — that is
[docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md)
and milestone issues [#6](https://github.com/bthek1/ros2_pi/issues/6)–[#8](https://github.com/bthek1/ros2_pi/issues/8),
and everything the rest of `docs/` says about those stages is **design intent**,
not a description of running code. When you build something, change the doc that
describes it from future tense to a measured statement, and say what you
measured it with.

**The camera is calibrated as of 2026-09-12** — P9,
[gh issue #9](https://github.com/bthek1/ros2_pi/issues/9), closed. `camera_node`
serves real intrinsics on `/camera_info` (fx=953.4, fy=957.6, cx=627.7, cy=334.6,
held-out reprojection 0.4955 px over 24 marker-confirmed frames) loaded from
`pimesh_bringup/config/camera_info/c922_720p.yaml`, and the `NOMINAL intrinsics`
warning is gone. `bash tools/gates/calibration.sh` is the check.

**That phase is the sharpest example so far of the rule above about design intent.**
Three things it asserted turned out to be false when measured, and the closed issue is
worth reading before touching anything to do with calibration: this camera has
essentially **no lens distortion** at 720p (so the phase's "straight edges come out
straighter" test could not pass as written and was re-scoped), `cameracalibrator`
**does not run on Lyrical at all**, and the command printed on our own board sheet is
wrong. Two limitations survive it, both deferred with triggers rather than forgotten:
the printed target still has ~1.6 mm of bow, and **`fx` is pinned only to ±2.2%**,
which is a ±2.2% slack in every distance this pipeline will report.

**Two things P0–P1 cost, and both are the same lesson as the teardown one
above.** A `static_transform_publisher` given `parameters=[...]` dies before it
reads them — it parses `argv` first — so the launch came up with no TF tree and
nothing failing. And extending `gates/hello-clean.sh` to signal `view-camera`
immediately found that recipe leaking RViz *and* the Pi's camera, because bash
will not run a trap while a foreground child is running and an rviz2 signalled
during its own startup never exits. The same gate also turned out to be deducing
the process group from `$!`, which is empty whenever `setsid` forks — a kill
that had been silently doing nothing in some contexts. Ask what the gate does
**not** touch.

Do not write "the node publishes X at Y Hz" until a node has published X and you
have watched it do Y.

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
(`bash tools/gates/hello-lan.sh`). DDS is wire-compatible across distros; the C++ ABI is
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

Five stages, one node each, all but the first on the dev box. Full design with
message types and rates: [docs/info/pipeline.md](docs/info/pipeline.md).

| Stage | Node | Where | Budget |
| --- | --- | --- | --- |
| Capture | `camera_node` | Pi | 1280×720 MJPEG, up to 60 fps, stamped at `VIDIOC_DQBUF`, calibrated intrinsics on `/camera_info` |
| Keypoints | `keypoint_node` | dev box | ORB, 500 features, ~5 ms/frame target |
| Depth | `depth_node` | dev box, **GPU** | Depth Anything V2 Small, 518², **55.1 ms/frame, 17.4 Hz measured in the container** |
| Fusion | `fusion_node` | dev box | TSDF, 1.5 cm voxels, integrate at depth rate |
| Surface | `mesh_node` | dev box | marching cubes, re-mesh every ~10 s |
| View | `dashboard_node` | dev box | web UI, 10 Hz stats, ~10 fps preview |

Both halves of that are measured, 2026-09-15. Inference alone is **51.08 ms**
(`bash tools/gates/gpu-stack.sh`); the whole per-frame cost inside `depth_node`, on
the node's own clock, is **55.10 ms** (`bash tools/gates/depth.sh`) — so
preprocessing, the reciprocal, the resize back to 1280×720 and two publishes cost
about 4 ms together. The predecessor measured 72–79 ms for the same model on the
same card through Python and an older ONNX Runtime, with a 280–305 ms CPU
fallback; ours measures 182 ms on the CPU for inference and 288 ms per frame. The
two sets of numbers are not directly comparable and both say the same thing about
the ratio.

**Depth is the pipeline's clock.** Measured at **17.4 Hz** against a 59 Hz input —
it sees roughly one frame in three and drops the rest through a one-slot mailbox.
Nothing downstream of it can run faster, so design accordingly: do not build a
fusion stage that assumes 30 Hz input.

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

  **This one is no longer inherited: it is measured here.** `bash tools/gates/hello-ipc.sh`
  runs the same container twice, with intra-process on and off, and compares the
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
  `unique_ptr` where a topic has exactly one consumer — `pimesh_hello`, and
  `decode_node`'s own inter-process subscription where the middleware allocates a
  fresh message anyway — and `ConstSharedPtr` for every fan-out. A `const &`
  callback is a *shared* subscription and does not copy; the note this file used
  to carry, that it "works perfectly and quietly copies", was the wrong way round.

  **And one consumer is the case that cannot fail**, which is why `gates/ipc.sh`
  asserts the decoded topic has at least two subscribers while it measures. The
  gate passed at 429/429 with one probe attached and failed at 0/574 on the next
  run with `keypoint_node` beside it. Ask what the gate does **not** touch.
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
  `header.stamp` jumps back by the bag's length; `keypoint_node` stamps
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
  `src/pimesh_bringup/test/test_straightness.py`). So "cover the frame corners"
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
  `pimesh_perception/stats.hpp` with `test_stats` behind them; `pimesh_camera`
  keeps its own copy on purpose, because a dependency edge from the Pi's package to
  a dev-box one would point the wrong way down the pipeline for the sake of twelve
  lines, and that cost is written down in the header rather than discovered later.
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
  `pimesh_<thing>` — `pimesh_hello`, `pimesh_msgs`, `pimesh_bringup`,
  `pimesh_camera` and `pimesh_perception` exist; planned: `pimesh_world`,
  `pimesh_dashboard`. **`pimesh_perception` builds on the Pi too, and the GPU code
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
  naming it, and `tools/check-stale.sh` is run by both `gate-hello-build` here
  and `sync-pi` there.
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
  handed on as a pointer. **`src/pimesh_hello/` is the worked example** and
  `src/pimesh_camera/` is the same shape doing real work — copy either rather
  than rediscovering it: the class declared in
  `include/pimesh_hello/`, defined in `src/`, the register macro at the foot of
  the .cpp, `rclcpp_components_register_nodes` (plural — the singular form
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

  **And a launch argument threaded into a node's parameters needs an explicit
  `value_type`.** A `LaunchConfiguration` is a string, so the raw substitution sets
  a *string* parameter of that name, which a node expecting a bool or a double
  ignores while saying nothing — the same failure `use_intra_process_comms` once
  had, where every frame was serialised and no log line mentioned it.
  `test_every_launch_argument_override_declares_a_value_type` asserts it over the
  whole override dict, so a new one is covered without anyone remembering to.
- **Two kinds of test, and conflating them is a mistake.** The
  `tools/gates/*.sh` scripts are the **phase tests**: slow, often needing the Pi
  and the camera, and they are what closes a claim. `bash tools/test.sh` runs
  the **unit tests** (`colcon test`) — fast, hermetic, no hardware, and they run
  on both machines. Write a unit test for logic that can be got wrong silently
  (the stamp arithmetic, a matrix layout, a quaternion); write a gate for
  anything that is a number about a running system.

  **What belongs here is decided by whether a mistake is *visible*, not by whether
  the code is interesting.** Every suite in this workspace exists because some
  wrong version of that code produces a plausible result: a depth map that renders
  as a room, a preview whose colours mean the opposite of what they say, a
  percentile that is quietly the maximum, a quaternion that still publishes three
  frames. If a bug in it would announce itself — a crash, an exception, a topic
  that stops — a gate is the cheaper place to catch it.

  **`colcon test` exits 0 when a test fails**, because it is reporting that the
  run completed — and it exits 0 again when a package has no tests at all, which
  is what an unbuilt tree looks like. `colcon test-result --all` is the thing
  that decides, and `bash tools/gates/test.sh` asserts on the counts: zero
  failures, zero skips, a floor on how many tests ran, and the same suites at
  both ends. Raise the floor when you add tests; never lower it to make a run
  pass.

  Tests that need a camera do not belong in `colcon test` — the dev box has no
  capture device, and a suite that only runs on the Pi is one that stops being
  run. `src/pimesh_camera/test/` covers the refusal paths with `/dev/null` and a
  temp file; the busy-device case is `tools/gates/capture.sh`'s job.
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
  recipe list equal to `just --list`, and 0 shellcheck findings over `tools/`
  (`uv tool install shellcheck-py`).
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
  on 2026-09-14. It now covers `INT`, `HUP`, `CLOSE` and `CLOSE-EARLY` — 14 cases
  over five recipes — and waits for the recipe to *return* rather than sleeping
  three seconds and sweeping, because the contract is that nothing is running by
  the time it has returned.
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
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones and their status |
| [docs/plans/README.md](docs/plans/README.md) | How a plan is written here: a GitHub issue of stable phases, a command for a test, executable-only, and the future file |
| [docs/plans/future/project_final_state.md](docs/plans/future/project_final_state.md) | **Where this is going.** The whole pipeline as phases P0–P8, none started, each ending in a `tools/gates/*.sh` test, followed by the deferred register |
| [#9](https://github.com/bthek1/ros2_pi/issues/9) **(closed 2026-09-12)** — camera calibration | **P9, done.** The C922's real intrinsics at 720p: fx=953.4, fy=957.6, cx=627.7, cy=334.6, held-out reprojection 0.4955 px. `camera_node` loads them from `pimesh_bringup/config/camera_info/c922_720p.yaml` and the NOMINAL warning is gone. Read the closed issue before touching calibration — three of its assumptions turned out to be false, including that this camera has barrel distortion |
| [#4](https://github.com/bthek1/ros2_pi/issues/4) **(closed 2026-09-09)** [#5](https://github.com/bthek1/ros2_pi/issues/5) [#6](https://github.com/bthek1/ros2_pi/issues/6) [#7](https://github.com/bthek1/ros2_pi/issues/7) [#8](https://github.com/bthek1/ros2_pi/issues/8) — milestones A–E | **The pipeline, being built.** A is done — P0 and P1, the cross-distro workspace and capture — B's two phases are built and measured (P2, P3), and **C is closed (P4): depth on the GPU, 55.10 ms/frame in the container.** Five issues over the *one* phase list in `project_final_state.md`, a contiguous slice each: A = P0–P1, B = P2–P3, C = P4, D = P5–P6, E = P7–P8. No issue renumbers from zero. Each also has a `just view-*` RViz recipe — a viewer for a person, never a gate |
| `bash tools/gates/ipc.sh` / `bash tools/gates/keypoints.sh` | **P2 and P3's gates.** `ipc.sh` runs the real container twice against the Pi's live camera and compares published buffer addresses with intra-process comms on and off — and asserts the decoded topic has at least two subscribers, because one consumer is the configuration that cannot fail. `keypoints.sh` replays `bags/desk1` and measures three things three ways: the rate from a C++ subscriber's steady clock, the per-frame cost from the node's own log line, and the matched-keypoint fraction against `tools/orb_reference.py` — the predecessor's algorithm reimplemented in Python over the same clip, which is the only part of the gate with an outside opinion about whether the corners mean anything |
| `bash tools/gates/depth.sh` | **P4's gate.** Replays `bags/desk1` through the real container — `decode_node`, `keypoint_node` and `depth_node` in one process — and measures four things. The provider, off `depth_node`'s own startup line. The per-frame cost on the node's own clock, against 80 ms. Whether `/depth/rgb` is **byte-identical** to the `/image_raw` frame with the same stamp, by hashing every source frame as it goes past and comparing — with "could not check" counted separately from "checked and differed", because a run that checked nothing would otherwise report zero mismatches and look perfect. And a **control**: the same binary with `use_cuda:=false`, which has to *fail* the same budget. `depth_probe` is its instrument, loaded into the container with `probe:=depth_probe` — out of process it would be subscribing to ~255 MB/s of images and would be the dominant load on the thing it is measuring |
| `bash tools/fetch-gpu-stack.sh` / `bash tools/fetch-model.sh` / `bash tools/gates/gpu-stack.sh` | **P4's toolchain, which is as far as milestone C has got.** The first installs ONNX Runtime 1.30 + CUDA 13.1 runtime + cuDNN 9.26 into `~/.local/opt/pimesh-gpu` with no sudo, every component version-pinned and sha256-verified; the second does the weights. The gate is four runs and three of them are controls — CUDA at 51.20 ms, the CPU provider at 213.18 ms (so the 80 ms budget is shown to *discriminate* rather than merely be met), a build with CMake's default linker flags that reaches only the CPU (so `-Wl,--disable-new-dtags` cannot quietly stop being load-bearing), and `nvidia-smi` sampled while the first runs, which is the only witness here that does not go through ONNX Runtime. `tools/gpu_probe.cpp` is its instrument: no ROS, no colcon, compiled by the gate with `g++` so that what is being tested is the toolchain and not four things at once |
| `bash tools/record-clip.sh desk1 60` | **The reference clip.** A 60 s hand-held sweep, recorded once, that every phase from P3 on replays so the numbers compare like for like. `bags/` is git-ignored, so a fresh clone has none and `gates/keypoints.sh` says so rather than pretending. The script resets the camera's V4L2 controls first and records `/camera_info` alongside the frames, because a clip recorded at 20 fps under a stale manual exposure cannot be un-recorded |
| `bash tools/replay.sh` / `view-camera.sh` / `view-keypoints.sh` / `view-depth.sh` | **The four viewers, and none of them is evidence.** `replay` loops a bag with `pipeline:=false` — the static frame tree and no components, because a pose published over a looping bag freezes and floods every TF listener; `view-camera` is the Pi's live camera; `view-keypoints` is the pipeline, on the camera or on a bag **played once** for the same reason; `view-depth` is the same again with the room as a colour-mapped depth cloud, and it waits longer before starting RViz because `depth_node` loads a 99 MB model and warms a CUDA session first. All four call `assert_no_session` before `arm_cleanup`, as does every gate: two sessions on one domain put two publishers on `/image_raw/compressed` and make both of them look broken |
| `bash tools/test.sh` / `bash tools/gates/test.sh` | **The unit tests.** 172 of them across twelve suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also
evaluates the launch file's `pipeline` condition both ways, so `pipeline:=false`
cannot quietly stop removing the container —  the calibration loader's refusals in `test_calibration`, and the calibration gate's own instrument in `test_straightness` — which measures a chessboard projected through a *known* K and D and is what makes `gates/calibration.sh`'s pixel figure worth asserting on — plus milestone B's four: `test_mailbox` (newest-wins and its drop accounting), `test_image_buffer` (the bgr8 layout arithmetic, and that a `cv::Mat` over a message shares its memory), `test_rotation_fit` (Kabsch against known rotations, the reflection guard, the reject-worst refits, and the optical-to-body change of basis) and `test_orb_tracker` (synthetic frames with a known displacement: that the window forgives detection churn, that unrelated scenes do not match, and that a track id is never claimed twice in one frame); and milestone C's one, `test_depth_model` — the arithmetic either side of the network, which is the whole of P4 that can be got wrong in silence: a channel order swapped, planes interleaved instead of planar, or a reciprocal taken before the clamp each produce a depth map that renders as a plausible room and is numerically nonsense. It needs no ONNX Runtime, no GPU and no camera, which is *why* `depth_model.hpp` is a header separate from the engine behind it — so this suite runs identically on the Pi. It also covers the colour preview's mapping, where a single sign decides whether near is bright or the 6 m clip is: flip it and the picture is still a perfectly plausible depth image of exactly the wrong thing. And `test_stats` — the percentile and the FNV-1a hash every probe reports its numbers through, which had no tests because they had no *home*: four copies of one and two of the other, each in an anonymous namespace inside a translation unit with a node in it. The first run of that suite found the FNV-1a offset basis had been wrong since milestone A |
| `gh issue list --label plan --state all` | **The plans themselves.** [#2 hello-world](https://github.com/bthek1/ros2_pi/issues/2) — closed 2026-09-08, the build log for the scaffolding that exists; [#3 justfile](https://github.com/bthek1/ros2_pi/issues/3) — closed 2026-09-09, why the shell lives in `tools/`; the justfile was trimmed further the same day to `build` + `run` only, so that issue's `just gate-*` spelling is history, not instruction |
| [docs/plans/future/milestone-a-future.md](docs/plans/future/milestone-a-future.md) | Work deferred out of milestone A, each entry with its trigger: the checkerboard calibration (waiting on P5's tape-measure visit), `PipelineStats` from `camera_node` (waiting on the dashboard), the dev-box rate margin, and device reconnection |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note
the date.
