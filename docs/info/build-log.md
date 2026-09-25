# Build log — what each milestone cost, and what it taught

This is the narrative that used to sit at the top of
[CLAUDE.md](../../CLAUDE.md), moved here on 2026-09-23 so that file could be the
constraint catalogue it actually is. **Nothing here is instruction.** It is the
record of how the pipeline got built and — far more usefully — of the times a
number was wrong before it was right.

The authoritative status is [roadmap.md](roadmap.md) and the closed plan issues
(`gh issue list --label plan --state all`); a closed issue *is* the build log for
its milestone, annotated phase by phase with what each gate printed. What this
page adds is the connective prose between them.

**The rules those findings turned into live in CLAUDE.md**, under *Constraints
that are easy to get wrong* and *Conventions*. If you are looking for what to do,
read those. If you are looking for why, read this.

**The single lesson worth carrying out of all of it:** more than a dozen times in
this project a gate has been **green over broken behaviour**, or red over nothing
at all, and in almost every case the code was fine and the *instrument* was
wrong. A zero in a cost field reads as "fast". A matched fraction of 1.000 reads
as "perfect". An `implausible=0` from a check that never executed reads exactly
like one from a check that found nothing. So the question to ask of any gate in
this repo is never "did it pass" but **"what does it not touch?"**

---

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
`pimesh_frontend` joins the four packages: one container, one network subscriber,
`cv::imdecode` at **1.87–1.93 ms/frame** keeping up with the Pi's full 59.4 Hz, and
the decoded 2.7 MB buffer reaching its consumers at the address it was published
from — **529/529** with intra-process comms on against **0/387** with it off
(`bash tools/gates/ipc.sh`). ORB then runs at **57.9 Hz sustained, 5.99 ms/frame**
on the node's own clock against an 8 ms budget, with a matched-keypoint fraction of
**0.9063** against the predecessor's algorithm at **0.9065** over the same 3489
frames (`bash tools/gates/keypoints.sh`), and publishes an `odom -> base_link`
that holds its last pose rather than guessing when its gates fail (8.2% of frames
on the reference clip). That pose was rotation-only until P7 and turned the
**wrong way** throughout — see the P7 section below, which is where the three-fold
improvement in the surface came from.

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

**Milestone D is closed as of 2026-09-16 — the room is a triangle surface.**
P5 and P6, [gh issue #7](https://github.com/bthek1/ros2_pi/issues/7).
`pimesh_mapping` joins the workspace: a spatially hashed TSDF at 15 mm voxels with
4 voxels of truncation and a weight threshold of 3, the predecessor's high-pass
scale aligner ported to C++, and marching cubes over a snapshot of the volume.
`bash tools/gates/fusion.sh` reports **15.3 ms per integration** against a 20 ms
budget, **17.1 Hz** sustained, 1030 of 1030 frames offered actually integrated,
**0.19%** displaced in the mailbox, and **0** frames without a pose at their own
stamp or without their colour twin. `bash tools/gates/mesh.sh` reports **790 668
triangles marched in 2.8 s**, decimated to exactly **120 000** for the Marker,
boundary loops **5119 → 448** with the frontier still open, a **779 740-triangle**
full-detail PLY, and three offscreen renders whose emptiest is 9.4% surface.

**Both of that milestone's tests had to be re-scoped by measurement, and that is
the most useful thing it produced.** P5 asks its gate to assert that per-frame
scale alignment makes two views of the same wall agree better; on `bags/desk1` it
does not, and the two runs are indistinguishable — median surface gap 1.32 m
aligned against 1.30 m unaligned, agreement 0.114 against 0.131, the winner
alternating window by window. The aligner is not broken (`test_scale_aligner`
pins its properties, including that a constant bias produces corrections whose
product is exactly 1, so it never pushes the map); it is correcting the *smaller*
error, because `keypoint_node` publishes rotation only and a hand-held sweep's
~0.9 m of real arm arc is a 30-45% geometric error at 2-3 m against a wobble
clamped at 15%. P6 asks for `max inter-integration gap ≤ 2× median`, and the
**input** does not meet that: `bags/desk1` stalls ~400 ms about 35 s in — the
seventh five-second window of every run — in runs recorded *before `mesh_node`
existed*. Both gates grew a control run instead, and each says in its own output
what it is not asserting and what would let it.

**Four findings from those phases are worth carrying, and every one was invisible
in the output:**

1. **A ray-cast that skipped the front of the truncation band.** A fixed coarse
   stride over unallocated space lands past a positive side one truncation thick,
   so the first sample inside the band is already behind the surface and there is
   no sign change to find. A panned camera reported *no surface* where a camera at
   the same place looking straight ahead found the wall.
2. **12% of frames integrated colourless with nothing upstream wrong.** A
   single-threaded executor runs callbacks in subscription-*registration* order,
   not publication order, so `fusion_node`'s depth callback ran before the colour
   twin published immediately before it. Registering colour first took it to 0.
3. **A use-after-move that segfaulted the container**, immediately after a re-mesh
   that had done every hard thing correctly — and the entire cleanup replayed
   offline on the very same mesh without a murmur, because the offline harness
   never published anything. `publish` moves from the `unique_ptr`; reading
   `stats->triangles` afterwards is a null dereference the compiler is happy with.
   **The stage that crashes is not always the stage that is wrong.**
4. **A niced extraction thread is not optional.** At equal priority the mesher
   starves the pipeline rather than blocking on its lock: `depth_node`'s rate fell
   from 17.8 to 14.6 Hz with its per-frame cost unchanged at 55.9 ms.

**The mesh did not look like a room, and two causes were named: rotation-only
odometry and an unpinned `depth_scale`. P7 settled the first and it was not the
one anybody expected.**

**The pose is 6-DoF as of 2026-09-19** — P7,
[gh issue #8](https://github.com/bthek1/ros2_pi/issues/8). `keypoint_node` pairs
each depth frame's corners with the **newest keyframe's** 3D landmarks by track id
and solves with `cv::solvePnPRansac`; `odometry: rotation_only` is P3's estimator,
kept as the control. `bash tools/gates/odom.sh` replays `bags/desk1` through both
and reports **1.421 px** mean inlier reprojection over **97 inliers**, **79.9%** of
depth frames posed rather than held, **1043 poses at 17.47 Hz**, a **31.1 m** path
and **4.87 m** of net displacement against the control's identical zero, and a
fastest published motion of **1.9986 m/s** against the 2.0 ceiling the node
enforces on itself.

**The valuable finding is a bug that had been running since P3 and that nothing
could see.** A fit answers `P_cur = M · P_prev` for a point seen twice; the camera
composes with `M⁻¹`, because the point did not move. `update_pose()` composed `M`
itself, so the published `odom -> base_link` turned **left** when the camera panned
right. *Nothing failed.* A frame that moves when you pan looks correct in RViz, the
residual gate is indifferent to the sign, `test_rotation_fit` asserted the fit was a
rotation of the right size about the right axis and that a change of basis moved the
axis correctly — every property except the direction of the one inverse between them
— and a TSDF built from consistently mirrored poses still produces a surface.
Measured with the old composition restored, same binary otherwise: median
paired-surface gap **1.3440 m** against **0.4456 m**, agreement **0.1057** against
**0.2087**. The first of those reproduces what milestone D recorded (1.32–1.37 m at
0.115), which is what ties the number to the bug rather than to the afternoon.
**3× on the surface, for one transpose.** `camera_step()` is that inverse, named,
and `test_rgbd_odometry` closes the loop `test_rotation_fit` left open: simulate a
camera with a known trajectory, show it what it would have seen, run the whole
estimator, and assert the pose that comes out is the trajectory that went in.

**Three more findings from that phase, and every one is a measurement overruling
the obvious design:**

1. **Do not difference two monocular depth maps.** The phase asked for a 3D–3D fit
   between two unprojected clouds. Both carry the depth network's error, and it is
   *structured* — a smooth warp, not per-pixel noise — so it does not average down
   over three hundred landmarks; and the network's scale breathes a few percent a
   frame, which a rigid fit can only absorb as translation along the view axis.
   Measured in order: rigid frame-to-frame reported an **89.5 m** path over a 45 s
   desk sweep, dividing the scale out left it at 117 m, measuring against a keyframe
   brought it to 44 m, a low-pass brought it to 8.4 m — and every one of those was
   still worse than publishing no translation at all. **PnP** works because it
   changes the *measurement*: the keyframe's landmarks against this frame's
   **pixels**, so one depth map instead of two, no scale ratio between them, and a
   residual in pixels rather than in the arbitrary unit everything else here is in.
2. **A confident fit is not a correct one, and a residual cannot tell you.** PnP
   reports how well its pose explains the pixels it was given and has no opinion
   about whether the 3D points behind them are where the depth network said.
   Measured: a single step of **9.4 m** between two depth frames at **1.23 px** mean
   inlier reprojection, which then became the reference every later frame was posed
   against, while the mean step stayed at 2.7 cm and looked healthy. A plausibility
   bound on the *motion* is a separate question and gets a separate refusal.
3. **A guard armed only when an unrelated feature is on is worse than no guard**,
   because its counter reads as evidence. That plausibility bound's bookkeeping
   started life inside the `translation_tau_s > 0` branch, so with the low-pass off
   it reported `implausible=0` over a trajectory containing that 9.4 m step.

**The keyframe store is built as P7 specifies and is consumed, which the phase said
it would not be.** Frame-to-frame chaining is a random walk; measuring each frame
against a keyframe and **setting** the pose from it rather than accumulating gives a
simulated final position error of **0.106 m** against **2.70 m**. What is still
unbuilt is the other reader — matching against *every* keyframe to recognise a place
seen minutes ago — which is the deferred loop-closure work. It costs **~37 kB** per
keyframe, not the ~16 kB the phase budgeted: the descriptors alone are 16 kB.

**And the 6-DoF translation itself makes no measurable difference on `bags/desk1`
— 0.4456 m against the control's 0.4471 m — so `gates/odom.sh` prints that
comparison rather than asserting it**, the same call `gates/fusion.sh` made about
the scale aligner. `desk1` is a *pan*: ~0.9 m of arm arc against 2–3 m of scene, so
rotation already explains most of the frame motion, and what is left is the depth
network's own shape error rather than the pose's. The trigger is a clip with
deliberate translation — a slow walk around the room — which needs a person and the
camera and is in
[milestone-e-future.md](../plans/future/milestone-e-future.md).
`depth_scale` is still arbitrary at 10.0 and still needs a tape measure, which is
the same visit.

**The dashboard exists as of 2026-09-19** — P8 and P10,
[gh issue #8](https://github.com/bthek1/ros2_pi/issues/8). `bash tools/view/dashboard.sh`
starts it, `http://localhost:8080` is the page, and
`bash tools/gates/dashboard.sh` closes the claim. `dashboard_node` is an HTTP and
WebSocket server inside a ROS 2 node, **in its own process** — the one dev-box
node outside the container, because the promise it makes is *it must be able to
die* and a component there would take the TSDF with it. Measured: five channels
at **10.01 Hz** stats, **9.16 Hz** rgb, **4.47 Hz** depth, **10.01 Hz** pose and
~2.1 MB of mesh per extraction; the STALE flag **2.10 s** after `/odom` stops
against a 2.0 s threshold; and every stage reporting itself on `/pipeline/stats`,
including `capture` from the Pi at **60.00 Hz** with the frames the kernel dropped
before dequeue — a figure no other stage can see, which is P10.

**No WebSocket library and no three.js, and both are deviations from what
`docs/info/dashboard.md` specified.** The server half of RFC 6455 is a SHA-1, a
base64 and a frame header, all three pinned against published vectors; a vendored
single-header library would be a third thing that has to exist and behave
identically on Jazzy and Lyrical, which is the failure mode this workspace keeps
paying for. The 3D view is ~200 lines of raw WebGL rather than 600 kB of
third-party minified JavaScript nobody here can read, to draw one triangle soup
and a line.

**P8's gate needed two corrections and the second one is about this project's own
rule.** It asks that every stage's rate be within 2% of a no-client run.

The first version compared a run's second half against its own **first** half to
judge what killing the client cost, and reported every stage **8-18% slower**
afterwards. That is backwards and it was entirely the clip: `mesh_node`'s
extraction grows from 0.7 s to 3.2 s as the volume fills and takes CPU from
everything above it, so the late seconds of any run are slower than the early
ones. It now compares the same seconds of clip across runs.

The second: that gate once measured **15% on `fusion` and 9% on `keypoints`
between two runs with nothing attached**, which looked like a pipeline far too
noisy for a 2% bound. It was not the pipeline — it was a `colcon build` and a
`colcon test` running on the same box, started by the person writing the gate.
**Do not build, test or run anything else on this machine while a gate is
measuring on it**; that rule is already in this file for sessions and it applies
to your own terminal just as hard. Re-measured on a quiet box: the worst stage
moved **0.77%** with a client attached, against a **0.29%** floor between two
control runs.

The gate keeps the floor-plus-slack form rather than a flat 2% anyway, because
the floor is a property of the *machine*: on a quiet box the bound collapses to
P8's 2%, and on a loaded one it widens with the load instead of failing and
blaming the dashboard.

Everything the rest of `docs/` says about work not yet built is **design intent**,
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

**The pipeline has an outside opinion as of 2026-09-25** — P11,
[gh issue #10](https://github.com/bthek1/ros2_pi/issues/10). TUM RGB-D fr1/desk,
613 frames with a 100 Hz motion-capture trajectory, replayed through the real
container by `dataset_node` at the dataset's own stamps and intrinsics, with
`odom_probe` writing a TUM-format trajectory that `evo` scores: **Sim(3)-aligned
ATE RMSE 0.27–0.36 m** over seven runs of ~350 poses, 100% associated, RPE 0.14–0.15 m
over a 1 s window.
`bash tools/gates/trajectory.sh`.

**The finding is not the ATE, it is the frame the trajectory was written in.**
`/odom` carries `odom -> base_link`, the REP-103 body convention; every public
benchmark's ground truth is the colour camera's *optical* frame. They differ by a
constant rotation and no translation, so an ATE over the translation part is
**identical** either way — and a relative-pose error is not, because the error
transform composes the rotations. Measured by rotating TUM's own ground truth by
that constant and scoring it against itself: **0.654 m RPE over a 1 s window, for
a trajectory that is exactly right.** The first run of the probe reported 0.768 m
in the body frame and 0.150 m in the optical one, so essentially the whole of that
number was the convention. The ATE moved by less than the run-to-run spread and
would never have shown it. `odom_probe` now looks the rotation up in the TF tree
rather than carrying a quaternion of its own, and **refuses to write the file** if
the lookup fails — a trajectory in the wrong frame produces a number, not an
error.

**Two smaller ones from the same afternoon.** The `rotation_only` control does
not merely fail the ATE ceiling, it cannot be scored at all: translation
identically zero is a rank-deficient covariance and Umeyama has nothing to fit,
so `evo` refuses. That is a stronger outcome than a large number and a weaker
*assertion*, because an `evo` failure for any other reason would look the same —
so the gate also asserts the ceiling is below the 0.8559 m an estimate that never
moves would score, **derived from `groundtruth.txt`** rather than written down.
And the fitted Sim(3) scale, 0.455–0.517 over seven runs, is `depth_scale`'s answer
without a tape measure: 4.6–5.2 against the 10.0 that has been in the YAML since
P4 because somebody typed it. It does not transfer to the C922 in this room, but
P12 is now a check on a number rather than the only source of one.

**P12's gate exists and its budgets have been watched to fail, 2026-09-25** —
without the clip it is for. `bash tools/gates/scale.sh` replays a flat surface at
a tape-measured distance and compares the median of `/depth` over a centred patch
against the tape. What is missing is a person, a wall and a tape; the checklist is
[setup.md](setup.md#the-visit-to-the-room).

**Two things came out of preparing it.** Running it against `bags/desk1` standing
in for the clip refuses at **0.30 of the patch clipped typically, 0.9998 at
worst**, with a plausible-looking median of 4.42 m beside it — so the clipping
budget has been watched to exclude something on real data, which is the only kind
of threshold this project keeps. The reason it matters: `depth_to_metres` writes
exactly `max_range_m` wherever the model's inverse depth falls below its floor, so
a clipped pixel is the *absence* of a distance dressed as 6 m, and depth is linear
in `depth_scale` **only below the clip** — an implied scale over a clipped patch
comes out smaller than it should be and looks entirely ordinary.

And the run found a bug in the gate itself: the first version printed the implied
scale, and the lines to paste into `config/pimesh.yaml`, *after* declaring the
measurement invalid. **Handing somebody a number you have just called wrong is
worse than printing nothing**, because the number is plausible and the warning is
three lines further up. It prints no scale at all now when a check has failed.

Do not write "the node publishes X at Y Hz" until a node has published X and you
have watched it do Y.
