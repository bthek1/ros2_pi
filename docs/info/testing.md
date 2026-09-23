# The tests, and why each suite exists

**Two kinds of test, and conflating them is a mistake.** `tools/gates/*.sh` are
the **phase tests**: slow, often needing the Pi and the camera, and they are what
closes a claim. `bash tools/test.sh` runs the **unit tests** (`colcon test`) —
fast, hermetic, no hardware, and they run on both machines.

Write a unit test for logic that can be got wrong *silently*; write a gate for
anything that is a number about a running system. **What belongs here is decided
by whether a mistake is visible, not by whether the code is interesting.** Every
suite below exists because some wrong version of that code produces a *plausible*
result: a depth map that renders as a room, a preview whose colours mean the
opposite of what they say, a percentile that is quietly the maximum, a quaternion
that still publishes three frames. If a bug would announce itself — a crash, an
exception, a topic that stops — a gate is the cheaper place to catch it.

**`colcon test` exits 0 when a test fails**, because it is reporting that the run
completed — and it exits 0 again when a package has no tests at all, which is what
an unbuilt tree looks like. `colcon test-result --all` is what decides, and
`bash tools/gates/test.sh` asserts on the counts: zero failures, zero skips, a
floor on how many tests ran, and the same suites at both ends. **Raise the floor
when you add tests; never lower it to make a run pass.**

Tests that need a camera do not belong in `colcon test` — the dev box has no
capture device, and a suite that only runs on the Pi is one that stops being run.
`src/pimesh_camera/test/` covers the refusal paths with `/dev/null` and a temp
file; the busy-device case is `tools/gates/capture.sh`'s job.

**Status: 433 tests across twenty-eight suites, identical on both distros**
(`bash tools/gates/test.sh`, 2026-09-23).

## The suites

433 across twenty-eight suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also

**`test_keypoints_view`** (added 2026-09-23) is the newest and the only one in
this workspace that asserts about *undefined behaviour* rather than about a
plausible wrong answer. Reading a `Keypoints` message back into corners, track
ids, descriptors and one-frame-apart pairs was three loops inside
`odometry_node`'s subscription callback; two of them were bounded by one array's
length while indexing another, so a message whose parallel arrays disagreed read
past the end of a `std::vector`. Nothing in this workspace publishes such a
message, which is exactly why reading the code did not find it. The suite covers
the refusal predicate array by array (a predicate that checks five of eight passes
over the other three and its name does not say so), that descriptors are copied
into **their own storage** rather than aliasing a message the callback is about to
release, that NaN holes are skipped rather than handed to a fit as `(NaN, NaN)`,
and that a half-hole — NaN in one component — is still a hole.

## Three recurring reasons a suite exists here

1. **A helper with no home has no tests.** `percentile` existed four times over
   and FNV-1a twice, each in an anonymous namespace inside a file with a ROS node
   in it — unreachable by any test and free to drift apart. The same was true of
   `depth_mat_over` (every distance the TSDF integrates passes through it), of
   `quote`/`number` in the dashboard (*the entire contents of the page*), and of
   the `Keypoints` conversion in `odometry_node` (*every landmark the pose is
   fitted to*). The question to ask of a helper is not whether it is interesting
   but **whether a test could call it if it wanted to.** Four times the answer was
   no; three times nothing was wrong yet, and the fourth had two out-of-bounds
   reads in it.
2. **A constant that is wrong but works is invisible.** The FNV-1a offset basis
   was `1469598103934665603` — the real one with its last digit dropped in a
   paste — in both copies, since milestone A, found by the first test that
   compared it against the published reference vectors. Nothing had been wrong: a
   hash with a different basis avalanches just as well. Write named constants in
   the form they are published in, and pin them against a reference vector.
3. **Two things that must hold the same value need a test for the pair**,
   especially across a language boundary where no mechanism relates them at all —
   `dashboard_node.cpp` against `web/app.js`, or two nodes' shared `volume_key`.
   The failure is always a pipeline that runs.

## Mutation-check them

A check nobody has seen fail is not an assertion. Every suite added since
2026-09-19 was run against the failure it claims to catch before being kept —
21 deliberate breakages for the dashboard's three, six for `test_orb_reference`,
two for `test_orb_tracker`'s parallel-array pair, three for `test_keypoints_view`
(restore the unclamped loops, alias the descriptors instead of copying them, stop
skipping the NaN holes) and one for the `/keypoints` queue depth. Twice this has caught a *check*
that could not fail: `gates/justfile.sh`'s argument check captured stdout when
`just -n` writes to stderr, and reported "0 recipes lose an argument" over five
that did.
