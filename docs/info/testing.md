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

**Status: 478 tests across thirty-one suites, identical on both distros**
(`bash tools/gates/test.sh`, 2026-09-25).

## The suites

478 across thirty-one suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also

**`test_depth_patch`** (added 2026-09-25, #10's P12) covers the statistic this
project's *unit* comes out of. `tools/gates/scale.sh` divides a tape measure by
the number `centred_patch_stats` returns and calls the result `depth_scale`,
which sits under every distance the pipeline reports — and there is no second
opinion on it anywhere. Every way of getting it wrong returns a plausible
distance rather than an error: a patch that is not centred reads a different part
of a wall, a NaN in the sort gives an implementation-defined median, and an empty
patch reported as a median of zero is `cost_mean=0.00` again, because 0 m reads
as "very close". The sharpest case is the far clip: `depth_to_metres` writes
exactly `max_range_m` wherever the model's inverse depth falls below its floor, so
a clipped pixel is the *absence* of a distance dressed as 6 m — and depth is
linear in `depth_scale` only below the clip, so counting them makes an implied
scale come out **too small** with nothing saying so. The suite pins them out and
counted separately, and pins the boundary as `>=` rather than `>`, because
`max_range` is the common value and an exclusive comparison would let every one
through.

It also pinned the centring convention on an odd leftover, which exists so that
"fixing" it the other way is a visible change rather than a silent shift of where
the unit is read from.

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
skipping the NaN holes) and one for the `/keypoints` queue depth.

**It keeps catching *checks* that could not fail**, which is the return that
justifies the practice. `gates/justfile.sh`'s argument check captured stdout when
`just -n` writes to stderr, and printed "0 recipes lose an argument" over five
that did. The 2026-09-25 sweep below found the first one that was a **unit test**
rather than a gate's instrument.

**Milestone F's four suites were swept on 2026-09-25: 55 deliberate breakages,
all caught** — 36 mutations of the code, 17 of the YAML (10 malformed markers and
7 mis-keyed pairs), and 2 divergences between the two copies of the marker's
grammar. The breakdown is 8 for
`test_tum_trajectory`, 10 for `test_depth_patch`, 12 for `test_dataset_reader` and
6 for the `quartiles` additions to `test_stats`. Every one of the obvious wrong
versions is in there — the quaternion written `w x y z`, `%g` on the timestamp,
`std::stod(text) * 1e9`, far-clip values counted as distances, the patch anchored
at the origin, the clip boundary made exclusive, equal timestamps allowed through.

**And it found four things, which is the return on doing it.**

**A third check that could not fail, and this time a unit test.**
`Percentile.DoesNotDisturbTheCallersVector` has been in this suite since it was
written, with a comment claiming to be "the test that keeps it that way" about the
by-value signature. It is not: changing the parameter to `std::vector<double> &`
does not fail that test, it fails to *compile*, at five other cases in the same
file that pass a temporary. A reference overload is ambiguous at the same places.
So the property was enforced — but by an accident of which cases happen to exist,
which a future refactor could remove without noticing. The contract is now a
`static_assert` on each signature, carrying its own message, and the two runtime
tests say what they actually cover: a body that reaches around the parameter.
**The comment was the problem as much as the gap** — this project has been here
before, with `sensor_msgs/Imu`'s covariance sentinel attributed to `nav_msgs` in a
code comment that was taken as a citation.

**A test whose subject is a file, asserting a file cannot exist.**
`test_tum_trajectory` checks that a *refused* trajectory leaves no file behind,
using a temp path built from an unseeded `::rand()` — so the same name every run.
A mutation deliberately left a file there, and the next clean run reported two
failures over correct code. `temp_path()` now removes the path before returning
it, which makes the precondition part of the fixture instead of part of the
weather.

**Two copies of one grammar, with nothing relating them.** The
`depth_scale_reference` comment is parsed by `tools/gates/scale.sh` and validated
by `test_transforms`, in two regexes that agreed only because one person wrote
both. A divergence is quiet in the worse direction: a test looser than the gate
accepts a marker the gate then cannot find, and the gate refuses with "no measured
distance recorded" about a line sitting right there. `test_transforms` now reads
the gate's regex out of the shell script and asserts it is the same string —
`test_dashboard_contract`'s method, applied to a second pair.

**The seven YAML-pair cases in `test_transforms` were swept the same way** — each
key mutated to a plausible wrong value — and all seven were caught by the test
that claims them: a probe reading a topic nobody publishes, a probe told a
different far clip from the map it is reading, a dataset publishing where nothing
decodes, one stamping the body frame instead of the optical one, one serving the
C922's calibration, and a trajectory written in a frame the tree does not publish.
None of those is a loud failure at runtime; each is a pipeline that runs and a
number that means something else.

**And one thing that is correct and reads as a gap.**
`Quartiles.AgreeWithPercentileAtEverySampleCount` cannot catch a wrong
`percentile_index`, because both sides call it and a wrong rank moves them
together. That is what sharing the formula is *for*; the `Percentile.*` cases
cover it, and clamping the index one rank low fails three of them while the
agreement test passes. It is written into the test, because "the two agree" reads
like it covers more than it does.
