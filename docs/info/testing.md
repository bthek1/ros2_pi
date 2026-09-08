# Testing

*Current as of 2026-09-08: 65 gtest cases (10 in `pimesh_camera`, 55 in
`pimesh_perception`) and 77 pytest cases for the gate tools, plus six gates.
All passing. The Pi builds only `pimesh_msgs` and `pimesh_camera`, so
`just test-pi` runs the 10 that belong to it; `just test` runs everything here
and reports 71 through `colcon test-result`, which counts each test binary
alongside its cases.*

## Two layers, and they answer different questions

| | **Tests** | **Gates** |
| --- | --- | --- |
| Ask | is this logic right? | does the real system do what we claim? |
| Need hardware | **never** | yes — camera, Wi-Fi, two machines |
| Run with | `just test`, `just test-pi` | `just gate-build`, `just gate-capture`, `just gate-ipc`, `just gate-keypoints`, `just gate-depth`, `just gate-provision` |
| Take | under a second | 1–4 minutes |
| Live in | `src/*/test/`, `tools/test_*.py` | the justfile, one per plan phase |

Both are required and neither substitutes for the other. The unit tests would
happily pass on a machine with no camera attached; the gates are the only thing
that can tell you the camera is delivering frames at the rate the hardware
allows. Equally, a gate that fails tells you *something* is wrong across two
machines and a radio link — the tests are what make that bisectable.

**Every phase of [the bootstrap plan](../plans/in-progress/bootstrap-plan.md)
ends in a gate.** Tests are added alongside whatever logic the phase introduces
that can be tested without hardware.

## What is tested today

### `pimesh_camera` — 10 cases, `src/pimesh_camera/test/test_v4l2_capture.cpp`

Run on **both** machines, so nothing in them may require a device — the dev box
has no camera.

- **The timestamp conversion (5 cases).** This is the most consequential logic
  in the package and the reason the node exists at all, so it was pulled out of
  `wait_frame` into a free function, `to_system_clock_ns`, purely so it could be
  tested. The cases assert the offset arithmetic, that a frame's *age* is
  preserved, that the answer does not depend on when the clock pair was
  sampled — and one case **reproduces the usb_cam 0.8.1 epoch bug** in
  arithmetic and shows it landing ~0.72 s late where ours is exact.
- **Timestamp provenance (2 cases).** The `V4L2_BUF_FLAG_TIMESTAMP_*` values
  are restated as literals, so a change in `<videodev2.h>` becomes a test
  failure rather than silently wrong provenance.
- **Failure paths (3 cases).** A missing device, a regular file, and a
  character device that is not V4L2 (`/dev/null`) must each throw with the path
  and the errno in the message. These matter most and would otherwise only be
  exercised by accident.

### `pimesh_perception` — 55 cases, `src/pimesh_perception/test/`

- **The mailbox (9 cases).** The pipeline's back-pressure policy: newest frame
  wins, the unread one is dropped and counted. The cases pin the property that
  matters — an old frame must never be delivered in preference to a new one —
  plus the shutdown path (`close()` must wake a parked taker, or the node's
  destructor never joins its worker), that it *moves* rather than copies (a
  `unique_ptr` payload would not compile otherwise), and a 20 000-frame
  producer/consumer race asserting that every frame is either delivered or
  counted as dropped.
- **JPEG decode (8 cases).** Exercised against JPEGs made on the spot with
  `cv::imencode`, so nothing needs a camera. Channel order, forced 3-channel
  output, allocation reuse — and the failure modes, which are the point: empty
  buffers, garbage, and a truncated frame, which is what a lost Wi-Fi fragment
  actually looks like. **One of these found a real bug before `decode_node`
  existed:** `cv::imdecode`'s three-argument form leaves its destination
  holding the *previous* frame on failure, so `!bgr.empty()` reports success on
  a corrupt buffer and the node would republish a stale image with a fresh
  timestamp.

### `test_rotation.cpp` — 16 cases, the P3 geometry

The file that justified pulling the rotation estimator out of the node. Inside a
subscription callback it could only be tested by pointing a camera at a room and
squinting at RViz; as free functions over ray bundles it can be handed a **known
rotation and asked to find it back**.

- **Unprojection (4 cases).** The pixel at `(cx, cy)` must unproject to exactly
  `+Z` — if that fails, `cx`/`cy` have been read out of the wrong slots of K,
  which puts a small constant bias into every pose. Rays are unit length and
  point forward, because a negative Z means the unprojection flipped and no
  amount of downstream fitting recovers from it. And `is_calibrated()` must
  recognise K-all-zeros, which is what keeps the odometer honest rather than
  confidently wrong.
- **Kabsch (3 cases).** A known rotation is recovered to 1e-9, and the
  determinant guard is exercised by a bundle whose best-fit orthogonal transform
  genuinely *is* a reflection. **The reflection case failed to fail on its first
  writing:** it was built from a cleanly rotated bundle, and `det(P·(R·P)ᵀ)` is
  always positive, so the SVD could not have reflected no matter what the code
  did. It passed with the guard commented out — which is the whole reason
  mutation-checking a new test is not optional.
- **The pose gate (6 cases).** Too few pairs is refused rather than fitted; 10%
  false matches are dropped by the refit rounds and the truth is still found to
  0.005 rad; a bundle that agrees on no rotation at all is refused with the
  *residual* reason rather than the *pairs* reason, because those two call for
  different fixes.
- **Composition helpers (3 cases).** `orthonormalize` must repair drift without
  moving a clean rotation — composing thousands of per-frame rotations
  accumulates error until the product is a slightly-scaling transform that
  quietly grows the map.

### `test_orb_tracker.cpp` — 10 cases, the P3 bookkeeping

Synthetic scenes of drawn blobs on noise, translated by a known amount. What is
tested is not OpenCV's ORB — that is upstream's job — but the bookkeeping around
it.

- **The two matchings stay separate.** The trap the file exists to prevent is
  using the *pooled* result for odometry: a "match" six frames back carries six
  frames of motion. After an interrupting frame the pooled window recovers
  (>40% matched) while the strict pair set stays thin, and one case asserts
  exactly that gap. Another shows the window earning its keep — pooled matching
  recovers from a frame of churn that consecutive-only matching does not, by
  more than 20 points.
- **Track ids follow the point, not the index.** A matched feature inherits the
  previous frame's id; inheriting by index would hand the id to whichever corner
  happened to sort into that slot. Ids are never reused, even across a `reset()`,
  so a consumer holding an old id can never find it pointing at something else.
- **Parallel arrays stay in step.** The `Keypoints` message is flat parallel
  arrays that a consumer indexes with one loop counter, so a length mismatch is
  a wrong-answer bug rather than a crash.
- **Matched pairs move the way the scene moved.** The property the rotation
  estimator actually depends on, checked in the one case where the answer is
  known: a pure 5 px translation must show a 5 px median displacement.
- **A featureless frame yields nothing rather than garbage**, and the tracker
  survives it — a hand over the lens mid-session must not poison the state.

### `test_depth_convert.cpp` — 12 cases, the P4 arithmetic

The two pure steps either side of the depth model, pulled out of `DepthModel`
so they could be tested at all: inside that class they sit either side of an
`Ort::Session`, which needs a 99 MB model file and a GPU to exist. They live in
`perception_core` and **not** `depth_core`, so nothing near them includes an
ONNX Runtime header and they run on a machine that has never installed it.

What they cover is the class of bug this stage is most exposed to — the kind
that produces a plausible depth map that is quietly wrong and throws nothing.

- **BGR to RGB (1 case).** OpenCV hands us BGR; the model was trained on RGB.
  A pure-blue BGR image must land in the *last* plane. Swapping them throws
  nothing and yields depth that looks roughly right.
- **CHW, not HWC (1 case).** A left-black / right-white frame must show that
  split independently in every plane; interleaved layout would alternate every
  three elements instead.
- **ImageNet normalisation (1 case).** Checked against the arithmetic —
  `(v/255 − mean) / std` per channel — rather than against itself.
- **The patch constraint (2 cases).** A side that is not a multiple of 14 fails
  here, loudly, rather than inside ONNX Runtime.
- **Inverse depth (3 cases).** The model emits relative *inverse* depth: bigger
  means nearer. A stage that forgot the inversion would build a room turned
  inside out, so one case asserts the ordering directly. Another asserts that
  doubling `depth_scale` doubles every distance and changes nothing else,
  because that is the one knob P5's tape measure will move.
- **The bound, and why it comes first (2 cases).** Zero and negative values are
  what the model emits for "background, no idea". Flooring the *denominator*
  bounds the output by construction; dividing first and clipping after leaves
  infinities and NaNs, and one NaN propagates through a TSDF integration
  silently.

Both load-bearing cases were mutation-checked: skipping the BGR→RGB conversion
turns `ConvertsBgrToRgb` red, and dividing before bounding turns
`NeverProducesInfinityOrNaN` red.

### The gate tools — 77 cases, `tools/test_*.py`

`check_ipc.py` (18 cases) decides whether the container is zero-copy. Its cases
are mostly the ways it must say **no**: one serialised frame among nine shared
still fails, a control run that *also* showed matching addresses fails (the
check would then be incapable of failing), a probe that received nothing fails
rather than passing vacuously on "all zero frames matched", and a second
subscriber on the Wi-Fi topic fails. One case feeds it a real launch-log line,
prefix and all — a log parser tested only against text the test invented proves
nothing about the regex it is meant to pin. Another pins the bug the gate
shipped with for exactly one run: `/pipeline/stats` carries every stage, so the
tool must select decode's record and not the camera's.

`check_capture.py` (13 cases) — the P1 gate's decision script.

`check_capture.py` decides whether P1 passes, so it gets tested like anything
else that can say "everything is fine". The cases drive its CLI and check both
the exit code and the output: a healthy run passes, dropping a third of the
frames fails, a 3% sampling difference does **not** fail (or the gate cries wolf
every run), a collapsed delivered rate fails, and — the one that matters —
**feeding it usb_cam's measured 0.223 s / 0.362 s offsets makes it fail**.

There is also a case asserting that a *missing* hardware measurement fails
rather than quietly passing on the strength of the checks that could still run.

`check_depth.py` (17 cases) — the P4 gate's decision script. Nearly all of its
cases are ways it must say **no**, because the failure it exists to catch is
silent: a CPU session produces depth maps that are perfectly *correct*, just
five times too slow. Two are worth naming. One feeds it a log saying CUDA and a
live session saying CPU, because the phase's whole risk is a fallback and the
answer must not rest on a single source. The other is the nastiest version — a
session that **reports the CUDA provider and still ran on the CPU**, which
happens because ONNX Runtime falls back per-node, and which only the timing
distinguishes. Deleting either half of the provider check turns four cases red.

`depth_pairing.py` (10 cases) — the arithmetic behind the P4 gate's "every
depth map has its RGB twin" line, in a module with no ROS import so it can be
driven without a graph. **This is the logic that was wrong twice**, both times
because the number it produced described the probe rather than the pipeline, so
the cases are written as the two failures that actually happened: a boundary
frame counted as an orphan because the probe stopped collecting, and a frame
seen before the probe was listening. Reinstating the original whole-set
comparison turns three of them red. One case guards the other direction — an
interior gap must still count against the ratio, or the check could never fail
at all.

`check_keypoints.py` (19 cases) — the P3 gate's decision script, and the one
whose cases are mostly about **not blaming the node for the fixture**. The bag
was recorded over Wi-Fi while the camera was carried around a room, so its
instantaneous rate swings between 7 and 60 Hz. One case feeds it a window where
decode delivered 10 Hz and the stage processed 9.6 of them: the fixture floor
fires, the node's keep-up ratio does not. Another feeds it a stage genuinely
falling behind a healthy decode, which must fail. Two more pin traps the gate
shipped with: `/pipeline/stats` carries every stage, so reading the camera's
record instead of the keypoint node's passes for the wrong reason; and
**`ros2 topic echo` silently elides strings past 128 characters**, so a `detail`
field that arrives truncated must fail rather than print `uncalibrated ?` for a
number that was there all along.

## Rules

- **A test suite that has never failed is not evidence.** Break the thing it
  covers, watch the suite go red, put it back. Done for the timestamp
  conversion on 2026-09-04: adding 1 ms to `to_system_clock_ns` turns 4 of the
  10 cases red, and removing it turns them green again. Done for the mailbox
  the same day: making `put()` keep the *old* value instead of the new one
  turns `NewestValueWinsAndTheOldOneIsGone` red and nothing else, which is the
  case named for exactly that claim. The decode and stats-parsing suites needed
  no synthetic mutation — each caught a real bug on its first run, recorded in
  P2's phase notes.
- **If logic is hard to test, that is a fact about the code.** The stamp
  conversion was three lines inside a 90-line method that needs a camera; it is
  now a free function with the reasoning in a comment above it. The refactor
  was worth more than the tests.
- **Tests run on both distros.** `just test-pi` builds and runs the same cases
  under Jazzy on aarch64 with g++ 13.3.0. A test that has only ever run on
  Lyrical says nothing about the machine that actually runs the camera.
- **No test may need hardware.** The moment one does, it is a gate.
- **Name the case after the claim**, not the function:
  `DoesNotReproduceTheUsbCamEpochBug`, not `TestConvert2`. A failing test name
  should tell you what broke without opening the file.

## What is deliberately not run

**The `ament_lint_auto` linters** (copyright, cpplint, uncrustify). The
generated `package.xml` files declared them and nothing used them, so the
declarations were removed on 2026-09-04 rather than left as decoration — a
`test_depend` should name something that runs.

They are worth turning on, and the reason they are not on yet is honest rather
than principled: `ament_copyright` wants a header on every file and a `LICENSE`
in every package, and `uncrustify` would reformat code that is currently
readable. That is a change to make deliberately, in its own commit, not as a
side effect of adding the first real tests. If it happens, it belongs in the
future file with a trigger.

## Adding a test

C++ goes beside the code it covers:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_<thing> test/test_<thing>.cpp)
  target_link_libraries(test_<thing> <the library under test>)
endif()
```

Python for the repo's own tools goes in `tools/test_*.py` and is picked up by
`just test`'s pytest pass — `tools/` is not a ROS package, so colcon cannot see
it, which is why the recipe runs both.

## In the editor

The pytest suite appears in VS Code's Testing sidebar once `just venv` has been
run — the workspace points the Python extension at `.venv`, a
`--system-site-packages` pointer at `/usr/bin/python3`, because naming the
interpreter in settings alone was silently overridden and discovery failed with
`No module named pytest`
([setup.md](setup.md#working-in-vs-code)). The C++ cases run from the
`test` task or the debugger — see
[setup.md](setup.md#working-in-vs-code).

## Running

```bash
just venv        # once: the interpreter the editor and pytest both use
just test        # colcon (gtest) + pytest, dev box, ~1 s
just test-pi     # the same gtest cases on the Pi, under Jazzy
just gate-build      # P0: builds on both distros, interfaces identical
just gate-capture    # P1: the camera, against real hardware
just gate-ipc        # P2: the container is really zero-copy, both machines
just gate-keypoints  # P3: ORB keeps up and matches, replayed off bags/desk1
just gate-depth      # P4: the GPU runs the model, and /depth/rgb is the right frame
just gate-provision  # P9: the playbook is idempotent and the Pi matches
```

`just test` exits non-zero if either suite fails, and reports both rather than
stopping at the first.
