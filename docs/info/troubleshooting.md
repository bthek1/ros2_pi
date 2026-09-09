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
