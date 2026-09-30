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

**Status: 587 tests across thirty-nine suites, identical on both distros**
(`bash tools/gates/test.sh`, 2026-09-30, after #12's P16).

## The suites

587 across thirty-nine suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also

**`test_place_recognition` and `test_place_truth`** (added 2026-09-30, #12's P16)
cover place recognition and the instrument that judges it, and both exist because
the failure is a *confident* one. `test_place_recognition`'s sharpest case is a
candidate carrying the query's own descriptors over **shuffled** landmarks: it wins
the descriptor vote outright and only the PnP can refuse it — accept on match count
and that test fails, which was checked by doing it. `AViewTheTrackerStillFollowsIsNotARevisit`
pins the rule bags/desk1 forced (7 of 16 queries "closed" onto the keyframe just
before them), and `TheGuidedSearchFindsWhatTheBlindOneCouldNot` asserts the RANSAC
seed *below* the floor and the final count above it, so it cannot pass by the blind
search succeeding; disabling the guided search fails it. The first version of the
suite also found a bug in the code: a stamp of 0 was the "no candidate" sentinel, and
the first test's genuine candidate was stamped 0 — refused at 205 inliers.
`test_place_truth` covers `tools/eval/place_truth.py`, which decides
`gates/place.sh`, and every way it can be wrong **flatters** the detector: an unjudged
closure counted true, a relative pose compared inverted (an asymmetric scene is what
catches it — both mutations checked), a precision of 1.000 printed over nothing, a
neighbour counted as a revisit the detector should have found, and "beats odometry"
decided by comparing the two poses with each other instead of each with the truth.
`test_gate_contract` gained the three new reader/writer pairs the gate adds.

Four more cases were added the same evening, after an audit of what the suite still
could not see: that one candidate corner cannot collect matches from the whole query
(one-to-one), that a query never finds itself even with the gap at 0 (the thread
searches *before* inserting), that the thread stops with a backlog queued, and that
**ragged keyframes are not read past the end** — `Keyframe` is struct-of-arrays, the
hazard `keypoints_view` had. **That last one passed over the bug it was written
for**: with the `landmark_row` bound check deleted, a row 50 past the end wrote into
mapped heap and the test stayed green. It now points 2^28 rows past the end, and the
same mutation segfaults inside that test. Mutations for the other three are each
caught by the case that claims them.

**`test_map`, `test_triangulation`, `test_local_map_match`, `test_local_ba` and
`test_local_mapper`** (added 2026-09-29/30, #11's P14 and P15) cover milestone G,
and they share one property: **everything wrong in a map makes its numbers look
better.** A map that never culls has more points and more keyframes. A point that
accepts two observations from one keyframe reaches "three observations" sooner. A
covisibility query that returns only the reference keyframe tracks exactly as well
as P7, because it *is* P7. So each suite asserts what a count would never show.

`test_map` pins the plan's three false greens directly — a cull that ran (and,
since a cull that found nothing and one that never looked both report zero, that
it *judged* something), one observation per keyframe however many features claim a
point, a track id re-admitted onto a feature the geometry disagrees with being
refused, and a local map larger than its reference. It also pins two decisions that
came out of measurement rather than design: a point sits at its **freshest** depth
reading, not its first (tracking against first readings measured 0.47-0.54 m of ATE
against 0.25-0.35 m for P7), and each new keyframe's depth map is moved only
**part** of the way onto the map's scale, because full correction is inheritance and
inheritance of a measured quantity is a random walk — one fr1/desk run finished at a
fitted Sim(3) scale of 0.699 that way. `test_triangulation` pins the refusals —
parallax, cheirality, and the worst view rather than the mean — and pins *why* the
parallax floor exists as a measurement: at one degree, half a pixel of noise puts a
point several percent out in depth while reprojecting better than the budget.

`test_local_ba` is the one suite in the workspace whose failure mode is a **process
abort** rather than a wrong answer. This g2o is built with asserts on, and a window
with no free pose trips `BlockSolver::resize`'s `_sizePoses > 0` — measured on both
machines while probing the library. The refusals are tested by handing the solver
exactly those windows; if one is ever removed, the test binary dies, which is the
louder of the two. Its sharpest case is `TheDepthPriorIsWhatAnchorsScale`: a window
30% too large reprojects exactly as well as the right one, so monocular BA leaves it
30% too large, and only the depth readings pull it back. **`LocalBaScale`**
(2026-09-30) pins the shared depth-scale model: a keyframe whose *whole* depth map
reads 20% far comes back as a scale of 1/1.2 with its pose within a centimetre,
where the per-reading model takes the same error as more than 2 cm of pose — both
halves asserted, so the pair cannot pass by the scale model getting worse. A keyframe
with no scale solved reports **0.0, not 1.0**, because 1.0 is what a perfect depth
map solves to and the backend averages `|s - 1|`. The first version of the suite
expected 1.2 and failed at 0.835: the solver had the convention right (the point is
at `s * d`, so `s` corrects the reading) and the test did not. A residual that ignores
the scale vertex fails two of the four cases. `test_local_mapper` pins
that the backend's queue *refuses* when full rather than dropping — the one queue in
this project that is not newest-wins, because a dropped keyframe is a hole in the
map — and that with bundle adjustment on, every keyframe after the first is solved
with iterations above zero.

**`test_gate_contract`** (added 2026-09-30, #11) is `test_dashboard_contract`'s
method applied to the two new gates: `tools/gates/map.sh` and `ba.sh` pull ~35 keys
off `odometry_node`'s `stats map` and `stats backend` lines in awk, and the node
writes them as C printf format strings. A renamed key parses as an empty string,
which those gates read as *the local map was the reference alone* or *zero culls*
— a failure of the map, reported against a map that was fine. The suite extracts
both sides as text and asserts subset, and asserts a **floor** on each extraction
first, because an extractor that finds nothing makes a subset check vacuously true.

**The second mutation sweep over milestone G's suites found one test that tested
nothing.** `LocalMapper.AnIdleThreadStopsToo` destroyed the mapper straight after
`start()`, before the worker had reached its condition variable, so the worker saw
the stop flag on its way in and a destructor that never notified passed it. It now
parks the worker first — one keyframe, a flush, a moment — and the same mutation
hangs it until `timeout` kills it. Six mutations, six caught, after that change.

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
