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
floor on how many tests ran, the same suites at both ends, and — since #14's P5
— **the same number of tests at both ends**. **Raise the floor when you add
tests; never lower it to make a run pass.**

**Two things make the count lie, and both were found on 2026-09-23 by moving
three suites between packages.**

`colcon test-result --all` reads every XML under `build/` whether the run that
just finished wrote it or not. So **a suite that stops being run keeps reporting
its last result — passing — for as long as the file survives**: a deleted suite
is invisible, and a moved one is counted twice. The three suites moved and the
total went 433 → 490, which is 433 plus the same three counted again out of the
old package's leftovers. Nothing in the output said so; the totals simply got
better. `tools/test.sh` deletes the result directories before running, which is
why it must be the thing that runs them.

Then the gate printed `dev=433 pi=490` and **PASS**, because it asserted a floor
on each machine and identical *suite names* — and stale files carry the same
names as the suites that wrote them, so nothing compared the two numbers. Hence
the equality assertion. Same sources, same suites, two machines: a difference
means different tests ran.

**The reason the Pi kept its stale files is the one to carry.** `tools/pi/test-pi.sh`
inlined its own `colcon build && colcon test && colcon test-result` — a second
spelling of `tools/test.sh` that agreed until `tools/test.sh` grew a step. It
runs `bash tools/test.sh` on the Pi now, and `tools/` is rsynced, so both ends
run the same script. A gate proves the thing a person runs only if it *runs* the
thing a person runs.

Tests that need a camera do not belong in `colcon test` — the dev box has no
capture device, and a suite that only runs on the Pi is one that stops being run.
`src/pimesh_camera/test/` covers the refusal paths with `/dev/null` and a temp
file; the busy-device case is `tools/gates/capture.sh`'s job.

**Status: 459 tests across thirty suites, identical on both distros**
(`bash tools/gates/test.sh`, 2026-09-25).

## The suites

459 across thirty suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also

**`test_tum_trajectory` and `test_dataset_reader`** (added 2026-09-25, #10's P11)
are the newest, and both are there because the code under them is read by
something outside this project.

`test_tum_trajectory` covers the file `odom_probe` hands to `evo` — a gate's
instrument, and the argument `test_orb_reference` makes applies with more force
here, because the *reader* is somebody else's parser. `evo` accepts nearly
anything that writer could emit, so every way of getting the format wrong ends in
a number rather than an error: `%g` on the timestamp collapses a 20 s clip onto
one instant and reports a small, confident ATE over nothing; a `w x y z`
quaternion is invisible in exactly the figure P11 is about, since an ATE over the
translation part never looks at the rotation; and a one-pose file aligns exactly
onto its reference for an ATE of 0.000000, which is `cost_mean=0.00` in a new
format. The expectations are literal bytes, not a round trip through a parser
written beside the writer — `test_mesh_io`'s lesson.

`test_dataset_reader` covers the index parser, and it exists for one line of it:
`std::stod(text) * 1e9` is the obvious way to turn `1305031452.791720` into
nanoseconds, it is wrong by a few hundred of them, and nothing downstream would
ever report it — `evo` associates within 10 ms and the pipeline is internally
consistent because it is wrong the same way everywhere. The suite asserts the
exact integer *and* asserts that the naive spelling disagrees with it, so a
compiler with wider intermediates cannot quietly turn the test into a tautology.
The rest is refusals: an index whose stamps repeat (two frames at one stamp is a
second answer to a lookup `odometry_node` does by exact stamp, not a duplicate),
one that goes backwards (the `--loop` failure arriving through a file), a listed
image that is not on disk, and a three-field line, which is what a path with a
space in it looks like.

**`test_keypoints_view`** (added 2026-09-23) is the only one in
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
