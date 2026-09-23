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
and why ad-hoc runs get `run_for` / `timeout --foreground -s INT` or an
explicit `pkill -f`.

## A recipe refuses to start, naming a `camera_node` on the Pi that nothing killed

```
just view-keypoints refuses to start: a session of this workspace is already running.
  on pi:
    308947 /home/bthek1/ros2_pi/install/pimesh_camera/lib/pimesh_camera/camera_node
```

…after the window was closed, with `stragglers on dev: 0`. **Fixed
2026-09-14; the shape of the failure is the part worth keeping.**

`kill_pi` was one `pkill -f` of the node pattern, over one ssh whose failure was
discarded, and it returned 0 in every case — including the ones where it killed
nothing. Two things made that a permanent leak rather than a missed beat:

- **The kill can land before the thing it kills exists.** `pi_run_for` starts a
  login `bash -lc`, which starts `timeout`, which starts `/usr/bin/python3
  …/bin/ros2 run`, which starts the node. A kill aimed at the leaf half a second
  after the session began matched nothing, reported success, and the wrapper
  exec'd the node a moment later. Measured: `kill_pi` at t=0.5 s returned 0, and
  the Pi had the full chain running at t=12 s.
- **Nothing on either machine would ever end it.** The dev-box script had exited
  believing itself clean; the far end runs to its own `timeout`, which for a
  viewer is ten minutes. Killing the leftover local `ssh` client does **not**
  reach it — measured, the remote chain carried on afterwards.

And the sweep could not see two thirds of it: every pattern matched the leaf,
none matched `timeout` or `ros2 run`, so one second after a remote start
`tools/stragglers.sh` printed `stragglers on pi: 1` where `pgrep` at the far end
listed three.

**If you see this now**, it is a genuinely unreachable Pi rather than the old
bug — teardown says so on stderr when it gives up. Sweep with `bash
tools/stragglers.sh`, which reports the whole chain, and clear it with:

```bash
ssh pi 'bash -lc "pkill -f \"[t]imeout -s INT [0-9]* ros2 run pimesh_\"; pkill -f \"/[r]os2 run pimesh_\"; pkill -f \"/lib/[p]imesh_[a-z]*/\""'
```

## A session exits non-zero after a perfectly normal run

Teardown could not confirm both machines were clean, and it prints what survived
on stderr before the script exits. That is deliberate: a viewer may exit 0 for
having shown somebody a picture, and may not exit 0 having left a `camera_node`
holding `/dev/video0`. An unreachable Pi lands in the same branch, because the
sweep cannot tell "asked and found nothing" from "could not ask" — and of the two
readings, the one that sends somebody to look is the right default.

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

**ONNX Runtime fell back to the CPU execution provider.** Measured here in C++,
2026-09-15: **51.2 ms/frame** on CUDA against **213.2 ms** on CPU. It falls back
**silently** whenever the CUDA provider library, the CUDA runtime or cuDNN cannot
be loaded — there is no error, only the slowdown. Anything using ONNX Runtime
must log the provider it actually got at startup; if that line is missing, add it
before debugging anything else.

Four causes, in the order they have actually happened:

1. **The binary was linked without `-Wl,--disable-new-dtags`.** This is the one
   that bites on a clean, correct install. `libonnxruntime_providers_cuda.so` is
   *dlopened* and carries no `RPATH`/`RUNPATH`, and `DT_RUNPATH` — CMake's default
   — is not inherited down a dlopen chain, so the provider cannot find
   `libcublas.so.13` in its own directory. Check with
   `objdump -p <binary> | grep -E 'RPATH|RUNPATH'`: it must say **RPATH**.
2. **It is a component, and the flag above does not reach it.** Measured
   2026-09-15, and it is the more confusing half of the same fact: the same
   libraries and the same flags gave `tools/gpu_probe` `CUDAExecutionProvider` at
   51 ms and `depth_node` inside `component_container_isolated`
   `CPUExecutionProvider` at 517 ms. Resolving a dlopened object's `DT_NEEDED`
   entries, glibc searches the object's own `DT_RPATH`, its **loader chain's**, and
   the **main executable's** — and a dlopened object has no loader chain, so the
   only one that could apply is the executable's. `gpu_probe` is an executable we
   link; `component_container_isolated` was built by somebody else and has no
   `RPATH` at all. The tell is the error text, which does name the missing library:

   ```
   Failed to load library …/libonnxruntime_providers_cuda.so with error:
   libcublasLt.so.13: cannot open shared object file
   ```

   The fix is in `preload_cuda_provider()` (`src/pimesh_depth/src/depth_engine_ort.cpp`):
   load the CUDA libraries **by absolute path** before ONNX Runtime asks for them,
   so its `DT_NEEDED` entries are satisfied from what is already in the process and
   no search happens at all. `LD_LIBRARY_PATH` is not an option — it is read once
   at process start, and this node must work in a container launched by
   `ros2 component load`.
3. **The stack is incomplete or the wrong shape.** Run
   `bash tools/fetch-gpu-stack.sh` — it verifies rather than assuming, and will
   name what is missing. `ldd` on the provider library is *not* sufficient
   evidence: a link-time stub resolves every symbol and then segfaults.
4. **Something else is using the card.** 6 GB is not much; `nvidia-smi` names the
   processes.

`bash tools/gates/gpu-stack.sh` distinguishes 1, 3 and 4 and prints which — and
**cannot see 2 at all**, because its instrument is an executable. `bash
tools/gates/depth.sh` is the one that covers the component, and it exists in that
shape because of this. Ask what the gate does not touch.

## `cv::cuda::` will not link

The apt OpenCV (4.10.0) is built without the CUDA module. ORB runs on the CPU
here, which is fine at 500 features. Building OpenCV from source to change that
is a decision to state out loud, not a fix to slip in.

## rqt or another Python tool crashes with `No module named 'yaml'`

The dev box's `python3` is PlatformIO's venv, which shadows the system Python
for `#!/usr/bin/env python3` shebangs. Prefix `PATH=/usr/bin:$PATH`. Running C++
nodes are immune — *building* them is not, see the next entry.

## A C++ package fails to build with `No module named 'catkin_pkg'`

Measured on the dev box 2026-09-08, on the very first `colcon build` this
workspace ever ran:

```
CMake Error at .../ament_package_xml.cmake:95 (message):
  execute_process(/home/proxmox-ml5/.local/bin/python3.14 .../package_xml_2_cmake.py ...)
  returned error code 1
```

`ament_cmake` is not a pure-CMake buildtool: it shells out to Python at
*configure* time to turn `package.xml` into CMake variables. CMake's
`FindPython3` picks the highest version it can see, and this box has two 3.14s
on `PATH` — apt's in `/usr/bin`, which owns ROS's `dist-packages`, and a
uv-managed one in `~/.local/bin`, which has never heard of `catkin_pkg`.

Name the interpreter rather than reordering `PATH`, which only moves the
coin-flip: `colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.
`just build` passes this already; a hand-run `colcon build` does not, which is
the main reason to use the recipe.

## The camera feed stutters — freezes, then catches up in a burst

**A RELIABLE subscriber on a lossy link delivers in sequence, so one lost
fragment blocks every frame behind it** until the loss is resolved by a
heartbeat round trip. At ~56 fps that reads as flicker.

Measured 2026-09-09, same publisher and same session, changing only the
*reader's* QoS, 20 s windows of ~1100 frames:

| reader | gaps > 50 ms | worst gap | p99 gap |
| --- | --- | --- | --- |
| RELIABLE | 10 of 1116 | 490 ms | 36.2 ms |
| BEST_EFFORT | 3 of 1097 | 181 ms | 27.4 ms |

So the viewer's `.rviz` asks for **Best Effort** against the RELIABLE publisher.
That pairing is legal — reliability is request-offered, and a RELIABLE writer
satisfies a BEST_EFFORT reader; only the reverse is incompatible. A dropped
frame in a live view costs nothing and a half-second stall looks broken.

**It reduces the stutter, it does not remove it.** The residue is Wi-Fi loss
bursts, and no QoS setting fixes those. If it matters more later, the levers are
fewer packets on the wire — a lower capture rate, or smaller frames — not
reliability.

**Rule out the other cause first**, because it looks identical in the panel and
has the opposite fix: if the *image content* is pulsing rather than the stream
stalling, that is lighting or exposure, not transport — a `power_line_frequency`
that does not match the local mains, or auto-exposure hunting. Decide it by
measuring, not by looking. On 2026-09-09 the per-frame mean luma over 1113
frames swung 0.73 of 255 levels (0.3% of full scale, std 0.11), which ruled
lighting out and left transport as the only candidate. In the same run 0 of 1113
frames failed to decode, which ruled out corrupt JPEGs too.

**This does not overturn the BEST_EFFORT rule** in
[CLAUDE.md](../../CLAUDE.md). That rule is about megabyte-class *raw* images:
2.7 MB is ~1900 UDP fragments, losing one per frame is near-certain, and the
result is zero frames delivered. These are ~80 kB compressed, about 56
fragments. Size is the whole difference — do not carry this to a raw image topic.

## The RViz Image panel is grey while the camera is streaming

**The display's topic must be the full name, transport suffix included** —
`/image_raw/compressed`, not `/image_raw`.

RViz 2's Image display has no transport property. It infers the transport *from
the topic name* (`rviz_default_plugins/displays/image/get_transport_from_topic.cpp`),
so the suffix is the only thing that selects the compressed subscriber. The ROS 1
habit of naming the base topic and setting a transport hint beside it does not
apply, and there is no such property to set: the only transport hints in the
plugin library belong to the DepthCloud display (`Color Transport Hint`,
`Depth Map Transport Hint`).

The failure is completely silent. RViz ignores an unknown key in a `.rviz`
without comment, infers `raw` from the shorter name, and subscribes to a topic
nothing publishes. The panel is grey, the display's status is not an error, and
the log says nothing — it looks exactly like a camera that is not running.
Measured 2026-09-09: `ros2 topic info -v /image_raw/compressed` reported
**Subscription count: 0** with the window open and the Pi capturing at 59 fps
(`camera_node` logged 3040 frames that session).

**How to tell in one command**, with the viewer open:

```bash
ros2 topic info -v /image_raw/compressed | grep -E 'count|Node name'
```

Two endpoints — `camera_node` publishing, `rviz` subscribing — means the config
is right and the problem is elsewhere. A subscription count of 0 means RViz is
listening to a different name than the one you think.
`ros2 node info /rviz` lists what it actually subscribed to, and needs no
publisher to answer.

`bash tools/gates/view-configs.sh` asserts every topic in a committed `.rviz` is
one `src/` actually publishes, exactly so this cannot ship again.

## `rviz2` will not start

It renders through GLX and needs `QT_QPA_PLATFORM=xcb` on this Wayland session.
With driver 595.84 it reports OpenGL 4.6 in hardware — the old software-GL
workaround (`LIBGL_ALWAYS_SOFTWARE=1`) is obsolete and should not be
reintroduced.

## `rviz2` segfaults part-way through `gates/view-configs.sh`

Symptom, measured 2026-09-21: stage 1 passes completely, then stage 2 gets
through one or two configs and the run ends with
`Segmentation fault (core dumped) run_for 45 rviz2 -d "$config"` and a gate that
exits 1 with no verdict. On other runs the same stage prints *nothing at all*
before the gate exits.

**It is the desktop sharing the GPU, not the configs.** The same `rviz2` started
by hand on the same config runs its full 40 s and reports `OpenGl version: 4.6`,
and the `.rviz` files have not changed since 2026-09-19 when this gate was last
green. The dev box's GTX 1660 SUPER is also carrying gnome-shell's compositor,
Firefox and VS Code's GPU process — the same contention that puts
`gates/dashboard.sh` an order of magnitude outside its recorded noise floor on
the same afternoon.

So: **close the browser and the editor before running any gate that starts a
viewer**, the same rule CLAUDE.md states for builds. If it still crashes with
the desktop quiet, that is a real regression and worth a core dump; until then
the useful half of the gate is stage 1, which is hermetic, reads only source and
`.rviz` YAML, and passes every time.

## RViz warns `Negative eigenvalue found for position` under `just view-odom`

Fixed 2026-09-23, and **it was not cosmetic** — this entry said it was for about
an hour, on the strength of a code comment that turned out to be wrong.

`keypoint_node` published `pose.covariance[0] = -1.0`. That sentinel is
**`sensor_msgs/Imu`'s**, documented in that message and nowhere else:

```
$ cat /opt/ros/lyrical/share/sensor_msgs/msg/Imu.msg
# ... please set element 0 of the associated covariance matrix to -1
$ cat /opt/ros/lyrical/share/geometry_msgs/msg/PoseWithCovariance.msg
# Row-major representation of the 6x6 covariance matrix     ← and nothing else
```

`nav_msgs/Odometry` documents no such convention, so what went on the wire was
simply a matrix that is **not positive semidefinite**. RViz's Odometry display
eigen-decomposes the position block on every message and was correctly reporting
that — at the pose rate, ~17 Hz, for the length of the session, roughly five
warnings a second.

**Why that matters here and not only in the scrollback.** `RCUTILS_LOG_WARN`
goes through rclcpp's process-global log mutex behind a synchronous terminal
write. That is the same mechanism documented above for the TF_OLD_DATA flood,
where a log line at frame rate stuttered RViz's render loop — a log line under a
lock is a rate limit on everything that lock protects. Setting every
`Covariance -> Value: false` in `rviz/odom.rviz` does **not** help; the
decomposition happens on receipt, not on draw.

The fix is `unconstrained_covariance()` in `pimesh_frontend/rgbd_odometry.hpp`:
a large diagonal, which is positive definite and is the conventional spelling of
"this dimension is unconstrained". Not zeros — that is legal but reads to a
fusion filter as a *perfectly certain* pose, which is the stronger false claim.
Not a measured covariance either: `cv::solvePnPRansac` reports no Jacobian, and
inventing the number would be what `camera_node` refuses to do for `latency_ms`.
It overstates ignorance rather than confidence, which is the direction to err in,
and `test_rgbd_odometry`'s `OdomCovariance` suite pins it — three of its four
cases fail against the `-1` that shipped.

Measured after the fix: **0 warnings over a 45 s run.**

## `just view-odom` says `no bag at 'sixdof'`, and `just dashboard` says `'8080'`

Fixed 2026-09-23; here because the message points at the wrong thing entirely.
Neither string is a bag name anybody typed — they are the recipe's *next*
argument, arriving where the bag was meant to be.

A `{{ }}` in a recipe body is **text substitution, not an argument**. just pastes
the value into the line and hands it to `sh`, so a parameter defaulting to `""`
leaves nothing between two spaces, the shell collapses that to no word at all,
and every argument after it shifts one place left:

```
view-odom seconds="600" bag="" regime="sixdof":
    @bash "{{ ws }}/tools/view-odom.sh" {{ seconds }} {{ bag }} {{ regime }}
                                        ↓        ↓         ↓
                                       600                sixdof      ← $2
```

The fix is to quote every substitution — `"{{ bag }}"` — so an empty default
stays an empty *argument*. A variadic `*args` must stay unquoted, or several
arguments become one string.

Three other recipes had the same defect harmlessly, because their empty
parameter was last. That is a latent bug, not a safe pattern: `view-odom` was
written by copying `view-mesh` and adding one parameter after `bag`.
`bash tools/gates/justfile.sh` now dry-runs every recipe and asserts it passes
as many arguments as it has parameters.

## `pkill -f` killed the shell that ran it

`pkill -f` matches full command lines, and the command line running the `pkill`
is one of them. Typing `pkill -f component_container_mt` at a terminal kills that
terminal's shell. Measured three times in one afternoon, 2026-09-08.

Two defences, both in `tools/just-lib.sh` and used by every script:

- **Bracket the first character** — `[c]omponent_container_mt` matches the
  process and not the pattern's own text.
- **Anchor on the installed path** — `/lib/[p]imesh_hello/` rather than a bare
  word. The bracket only protects the pattern's own characters; a command that
  mentions the plain word anywhere else, in a comment included, still matches.

`tools/stragglers.sh` goes further and drops every process in the caller's own
process group, which is the complete fix: a genuine straggler has outlived its
session and is therefore in a different one.

## A `trap ... INT` never runs, and the script shrugs off Ctrl-C

Bash will not install a handler for a signal that was **ignored when the shell
started**, and it says nothing when it declines: `trap cleanup INT` is silently
a no-op. A command started in the background by a *non-interactive* shell —
`setsid bash session.sh &` inside a script — inherits SIGINT and SIGQUIT as
`SIG_IGN` under POSIX job control, so exactly the scripted-test case is the one
where the trap you are testing does not exist.

Read the disposition rather than guessing; the mask is a hex bitmask with bit
*n-1* for signal *n*, so SIGINT (2) is `0x2`:

```bash
grep -E '^Sig(Ign|Cgt):' /proc/<pid>/status
# SigIgn: ...0006  = SIGINT|SIGQUIT ignored — your INT trap was never installed
# SigIgn: ...0004  = only SIGQUIT ignored — the trap is real
```

The fix is to reset the disposition before exec'ing the thing under test:

```bash
setsid env --default-signal=INT,TERM,HUP bash tools/view-camera.sh 45 &
```

That is also the *faithful* spelling, not a workaround: a terminal's Ctrl-C
reaches a foreground job whose SIGINT is at its default. Measured 2026-09-09,
when `gate-teardown` (then named `gate-hello-clean`) started launching the
session directly instead of
through `just` — `just` had been resetting the disposition for its child as a
side effect, so the gate had been passing for a reason it never stated.

## Ctrl-C does nothing at all, and the recipe ends on its own later

The other way a session goes deaf, and it needs no ignored signal — the trap is
installed and correct, and still nothing happens. Two things have to line up:

**GNU `timeout` moves its child into a new process group** so that it can
signal the whole tree when the timer fires. A terminal delivers Ctrl-C to the
**foreground** group only, so the command under `timeout` is not in the blast
radius:

```
bash hello-compose.sh   pgid 2565095   <- Ctrl-C lands here
  timeout 5 sleep 5     pgid 2565096   <- and never here
    sleep 5             pgid 2565096
```

**Bash will not run a trap while a foreground child is running**, so the trap
cannot compensate: the interrupt is recorded and the handler waits for
`timeout` to return, which is precisely what it is refusing to do. Measured
2026-09-09, sending SIGINT to the process group at t=1 s against
`timeout -s INT 8 sleep 8`:

| form | result |
| --- | --- |
| plain `timeout`, foreground | trap never fired, script alive at t=3 |
| `timeout --foreground` | trap fired immediately, group gone |
| `timeout ... &` then `wait` | trap fired immediately, group gone |

Both fixes work and they are not interchangeable. **`run_for` in
`tools/just-lib.sh`** (`timeout --foreground -s INT`) is the one for anything a
person watches: the child stays in the caller's group, so Ctrl-C reaches
`ros2 launch` directly and ROS shuts down the way it does under a bare launch —
measured at 0.30 s from keypress to a container that logged *process has
finished cleanly*. Backgrounding and `wait`ing is equally sound but can only
ever kill by pattern, so it skips the graceful path.

Giving up `timeout`'s group-kill costs nothing here, because everything run
this way is a launcher that already shuts its own children down.

Symptomatically this is unmistakable once you know it: the `^C`s echo, the logs
keep scrolling at their usual rate, and the session exits by itself exactly
when its timer was due. `just hello-compose` swallowed six of them before
ending on schedule 30 s in.

## Nodes keep logging after a recipe ends

Killing a background `bash -lc` wrapper orphans its ros2 grandchildren, which
sit silent until something feeds their subscriptions again. Cleanup traps must
`pkill -f` the node patterns, never `kill %N`.

## Parameters in the YAML seem to do nothing

The file is keyed by **node name**, and a key that does not match applies
nothing — silently, with no warning. Check the node's actual name (`ros2 node
list`) against the top-level key. This trap has cost real time in the
predecessor project more than once.

## The calibration sheet's own printed command line is wrong

**Symptom.** `cameracalibrator -p charuco` starts, sees the board, and never
registers a sample — the X/Y/Size/Skew bars stay empty.

**Cause.** `docs/charuco_a4_7x9_25mm.pdf` prints
`--pattern charuco --size 6x8 --square 0.025 --charuco_marker_size 0.018 --aruco_dict 4x4_250`
along the bottom. That `--size` is the **interior-corner** count. For `-p charuco`
the value goes into `cv2.aruco.CharucoBoard`, whose first argument is the number of
**squares**, so a 7x9-square board must be given `--size 7x9`. Measured against a
real frame from `bags/cam_2026_09_12`, 2026-09-12:

| `--size` | chessboard corners interpolated |
| --- | --- |
| `7x9` (squares) | **42** |
| `6x8` (corners) | **0** |

Inherited from `piros2/tools/calib/make_calib_target.py`, which composes that line
when it draws the sheet. The PDF is a printed artefact, so the wrong line is on the
wall and cannot be edited there.

**Fix.** Use `bash tools/calibrate.sh`, which takes `--squares 7x9` and derives the
interior-corner count itself, precisely so the two conventions cannot be typed
inconsistently. If running `cameracalibrator` by hand, pass squares to `--size`.

Note the same number means the *other* thing for `findChessboardCorners`, which is
what `tools/calib_straightness.py` and the gate use: there it is 6x8. One input,
two conventions, and they are one apart — which is why neither is typed twice.

## A replayed bag shows a grey Image panel and publishes nothing

**Symptom.** `ros2 bag play <bag> &` runs without an error, `ros2 topic list`
shows `/image_raw/compressed`, and `ros2 topic info` reports a publisher — but
RViz's Image panel stays grey, `ros2 topic hz` never prints a rate, and the
player logs nothing at all. It looks exactly like a QoS mismatch and is not one.

**Cause.** Playback enables keyboard controls by default — space to pause,
cursor keys to step — so the player reads the controlling terminal. A process in
the **background** that reads its controlling TTY is sent SIGTTIN by the kernel
and *stopped*. `ps` shows state `T`; it holds its DDS publisher open, which is
why the topic and the publisher count both look healthy, and it will never send
a message. Being stopped is not a failure it gets to report, so there is no log
line to find. Measured 2026-09-12: two players stopped this way sat for four
minutes while `ros2 topic info` reported `Publisher count: 1`.

**Fix.** `bash tools/replay.sh` (`just replay <bag>`) passes both
`--disable-keyboard-controls` and `</dev/null`. By hand, either of those works;
in the foreground, neither is needed. Confirm with `ps -o stat= -p <pid>` — a
`T` is this, not a hang.

**Related.** A bag of `/image_raw/compressed` and `/camera_info` carries no
`tf_static`, and `rviz/camera.rviz` has `base_link` as its Fixed Frame, so a bag
replayed without `pimesh.launch.py` produces the same grey panel for an entirely
different reason. `just replay` starts the launch for exactly this.

## A looping replay flickers, and floods the terminal with `TF_OLD_DATA`

**Symptom.** `bash tools/replay.sh desk1` comes up fine, and after about a minute
the RViz Image panel starts stuttering, the 3D view stutters **while displaying
nothing at all**, and the terminal fills with

```
[WARN] []: TF_OLD_DATA ignoring data from the past for frame base_link at time ...
```

at roughly the frame rate. Nothing has been touched; the first pass looked
perfect.

**Cause.** About a minute is the length of the clip, and the wrap is the event.
`ros2 bag play --loop` restarts at the beginning, so every `header.stamp` jumps
~60 s into the past; `keypoint_node` stamps `odom -> base_link` with the frame's
own stamp, as it must; and `tf2::BufferCore` refuses any transform older than
the newest it already holds. Measured 2026-09-13 with `tf2_echo odom base_link`
beside a looping player: the edge advanced for the first pass and then **froze at
the bag's final stamp for the whole rest of the run**. It cannot recover, because
the newest stamp the loop will ever produce is the one already in the buffer.

The flicker is the *logging*, not the rendering. `RCUTILS_LOG_WARN("TF_OLD_DATA
...")` is emitted inside `setTransform`'s `std::unique_lock<std::mutex>
lock(frame_mutex_)` (geometry2, `buffer_core.cpp`), and rclcpp serialises log
output on a process-global mutex behind a synchronous write to the terminal. So
~59 times a second RViz's TF buffer is held shut across a terminal write while
its render loop waits on the same mutex for `lookupTransform`. Both panels are
drawn by that one Qt loop, which is why an **empty** 3D view stutters too — that
is the tell that the problem is not in the graphics.

**Fix.** Do not publish a dynamic transform over a looping bag.
`bash tools/replay.sh` starts `pimesh.launch.py pipeline:=false`, which brings up
the static frame tree and no components at all; `bash tools/view-keypoints.sh
<seconds> <bag>` needs the pose, so it plays the clip **once** and ends when the
clip does. With a purely static chain `_getLatestCommonTime` returns
`TimePointZero`, which `FrameInfo::setLastUpdate` special-cases into a refresh on
every tick, so the frames stay lit rather than ageing out under camera.rviz's
15 s Frame Timeout.

**The obvious fix is measured wrong.** `ros2 bag play --clock` with
`use_sim_time:=true` makes the wrap a clean backwards time jump, which is what
`tf2_ros::Buffer::onTimeJump` exists for — and it handles it by calling
`clear()` on the **whole** buffer, `tf_static` included. Nothing republishes a
latched topic afterwards. Measured the same day over four wraps: the tree went
away at the first `Detected jump back in time. Clearing TF buffer.` and never
came back. RViz does the same thing to its own displays one layer up
(`Detected jump back in time. Resetting RViz.`).

**Related, and check this first if the stamps look odd.** Out-of-order *and*
duplicated stamps in that flood — the same time printed two or three times — are
a different fault with the same symptom. See the next entry.

## Two viewers at once, and both of them flicker

**Symptom.** `just view-camera` and `just replay <bag>` are both up, in two
terminals, and **both** windows stutter with the `TF_OLD_DATA` flood above. Each
one looks broken. Run on its own, neither is: measured 2026-09-13, 0 warnings
over 195 s of `just replay desk1` and 75 s of `just view-camera`.

**Cause.** One ROS domain, one set of topics. The Pi's live camera and the bag
player both publish `/image_raw/compressed`, so the single `keypoint_node` in the
container decodes an interleaved mixture of frames whose stamps are minutes
apart, and publishes `odom -> base_link` with stamps that jump back and forth.
tf2 rejects whichever arrives out of order, at the frame rate, from inside its own
buffer mutex — so both RViz windows stall on `lookupTransform`. The tell is in
the warning text: the same timestamp printed two or three times, and timestamps
that go *backwards* within a second of log.

This is the rule about one reader on the Wi-Fi topic, seen from the publisher's
side, and it is not limited to two viewers — a gate run beside a viewer measures
the same mixture and says nothing about it.

**Fix.** `assert_no_session` in `tools/just-lib.sh` refuses to start while
anything of this workspace's is already running, and prints the pids it found.
**Every script that starts a session calls it — the three viewers, the recording
and calibration tools, and the gates.** For a gate it is the sharper case: a
measurement taken beside another session is a measurement of a mixture, with
nothing in the output saying so.

It is called before `arm_cleanup`, deliberately: the cleanup handler kills this
workspace's processes, so refusing *after* the trap is armed would tear down the
session it was refusing to disturb. To find what is running at any time,
`bash tools/stragglers.sh` — the same pattern list and the same
`pimesh_local_processes`, asked after the fact instead of before it.

**A gate that suddenly refuses is usually telling the truth.** If several in a
row refuse, something is up in another terminal; check before assuming the guard
is wrong.

## `cameracalibrator` sees nothing, and `republish` is why

**Symptom.** `bash tools/calibrate.sh session` comes up, the calibrator window opens,
and it never registers a single sample — as if the board were not there at all.

**Cause.** `image_transport republish` takes its transports as **parameters** on
Lyrical, not as positional arguments. The form P9's body gives,

```bash
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=...
```

starts a node that logs `The 'in_transport' parameter is set to: raw`, subscribes to a
*raw* topic nothing publishes, and emits nothing. There is no error: the positional
words are simply ignored. Measured 2026-09-12.

**Fix**, and what `tools/calibrate.sh` now uses:

```bash
ros2 run image_transport republish \
    --ros-args -p in_transport:=compressed -p out_transport:=raw \
    -r in/compressed:=/image_raw/compressed -r out:=/image_raw_uncompressed
```

`session` now asserts one message arrives on the raw topic before starting the
calibrator, so this fails in three seconds with a message instead of looking like a
board-detection problem.

## `/world/mesh` is empty forever, and `mesh_node` is running

**Symptom.** The container is up, `fusion_node` logs blocks climbing into the tens
of thousands, and `mesh_node` logs nothing at all — or logs a warning about
waiting for a volume. RViz's Marker panel stays blank.

**Cause.** The two nodes share the TSDF through a **process-local registry**, not
through a topic, and they find each other by the `volume_key` parameter and
nothing else. Three ways for that to fail, and all three look identical from
outside:

- the keys disagree (`fusion_node` fills `world`, `mesh_node` meshes `worlds`) —
  two separate volumes, one filled and never meshed;
- `mesh_node` is in a **different process** — started with `ros2 run`, or loaded
  into a second container. The registry is process-local by construction; see
  `pimesh_mapping/shared_volume.hpp` for why the volume is shared by pointer rather
  than published, and why this limitation is deliberate rather than an oversight;
- `mesh_min_weight` is above `max_weight`, so no voxel can ever reach it.

**Fix.** `mesh_node` names the key it is waiting for and lists every key the
process has, so read its warning first:

```
no configured volume under key 'world' (registered: worlds (configured)) — waiting.
```

`test_transforms.py` asserts the two YAML keys match and that the meshing weight
sits between the volume's floor and its ceiling, so a mismatch committed to
`config/pimesh.yaml` fails `bash tools/test.sh` rather than a session.

## A latched topic reads as empty, and the QoS is "compatible"

**Symptom.** `ros2 topic echo --once /world/mesh` prints nothing and times out,
over a run that has demonstrably published several surfaces. An RViz panel on the
same topic is blank for up to ten seconds after startup and then fills in.

**Cause.** `mesh_node` publishes `/world/mesh` **transient local** on purpose: the
surface changes every ten seconds, and a viewer started between two extractions
would otherwise show an empty 3D view that looks exactly like a dead topic. A
**volatile** subscriber against a transient-local publisher is QoS-*compatible* —
it connects, it is listed in `ros2 topic info -v`, and it simply does not receive
the stored message. It then waits for the next one.

**A mismatch that is legal is worse than one that is not**, because nothing
anywhere reports it. Measured 2026-09-16: `tools/gates/mesh.sh` reported "nothing
was published on /world/mesh" over a run that had published eight surfaces.

**Fix.** Ask for the durability you need:

```bash
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /world/mesh
```

In an `.rviz` config it is `Durability Policy: Transient Local` on the display's
`Topic` block — `rviz/mesh.rviz` carries it, and `tools/gates/view-configs.sh`
watches rviz2 actually subscribe.

## The map stops growing, and the room is half-built

**Symptom.** `fusion_node`'s stats line shows `blocks=300000` unchanged window
after window and `refused=` climbing into the hundreds of thousands. New parts of
the room never appear in the mesh; the parts already mapped keep refining.

**Cause.** `max_blocks` is a hard ceiling — 300 000 blocks of 6 kB is about
1.9 GB, and P6's mesher takes a copy. Past it, existing blocks keep updating and
nothing new is taken on. The node warns, throttled.

**This is a symptom and not a resolution set too fine.** `bags/desk1` reaches
200 000 blocks, which is over 2000 m² of surface for a room with perhaps 60 m² in
it — about thirty layers of the same wall, laid down by rotation-only odometry
(P7) and an unpinned `depth_scale`. Raising the ceiling buys a longer session and
fixes neither.

**Fix.** Raise `max_blocks` if the box has the memory, or shorten the clip. The
real fix is upstream, and `mesh_min_weight` is the lever that keeps the *mesh*
sane in the meantime — the histogram `mesh_node` logs each extraction
(`weights blocks=… >=1:… >=4:… >=8:… >=16:… >=32:…`) is the evidence to choose it
from, rather than eyeing a picture.

## A node segfaults immediately after doing something difficult correctly

**Symptom.** `process has died … exit code -11`, right after a log line reporting
a long, complicated piece of work that clearly succeeded. Replaying the same work
offline — same input, same functions, under AddressSanitizer — shows nothing at
all.

**Cause, at least once here:** a **use-after-move** across a `publish`. `publish`
takes the message `unique_ptr` by value and moves from it, leaving the caller's
pointer null; reading a field off it afterwards to build a log string or a second
message is a null dereference the compiler is perfectly happy with. Measured
2026-09-16 in `mesh_node`, after an extraction that had done marching cubes,
component pruning, hole filling and 400 000 edge collapses correctly.

**The stage that crashes is not always the stage that is wrong**, and an offline
harness that never publishes anything cannot see this class at all.

**Fix.** Read what you need out of a message *before* you publish it. When
bisecting a crash like this, log between stages rather than at the end — the
missing log line is the bisection.
