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
setsid env --default-signal=INT,TERM,HUP bash tools/hello-lan.sh 45 &
```

That is also the *faithful* spelling, not a workaround: a terminal's Ctrl-C
reaches a foreground job whose SIGINT is at its default. Measured 2026-09-09,
when `gate-hello-clean` started launching the session directly instead of
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
