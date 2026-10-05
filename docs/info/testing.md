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

**Status: 693 tests across forty-six suites, identical on both distros**
(`bash tools/gates/test.sh`, 2026-10-05, after milestone I's follow-up).

## The suites

693 across forty-six suites, identical on both distros: the stamp arithmetic (`test_stamp` encodes the usb_cam bug as a failing assertion), the `CameraInfo` matrix layout, `V4l2Capture`'s refusal paths, the static transforms and launch conversion in `test_transforms` — which also

**After milestone I, 2026-10-05: three helpers given a home, and four contracts
pinned.** The audit question was this file's own — *could a test call it if it wanted
to?* — and three pieces of P20 logic said no, each sitting inline in a node:
`dataset_node`'s frame slice, the relocaliser's `map ← odom` product, and `save_map`'s
choice of each keyframe's map pose. They are `slice_frames` (`dataset_reader.hpp`),
`map_from_odom` (`relocaliser.hpp`) and `keyframes_in_map` (`map_io.hpp`) now, and each
has the case that its plausible wrong version fails: **max counted from the skip**
instead of from the sequence (which would silently close `relocalise.sh`'s 30-frame
gap between the saving and loading sessions), **the inverted product** checked on a
frame a metre and a turn *after* the query (the query alone passes either way whenever
odom is near identity — the first frames of a session), and **the correction composed
on the right**, or the graph's corrected pose ignored (a map saved at drifted poses,
relocalised into with confidence). Every mutation is caught by its case. Nothing was
wrong this time; the point is that next time a test can see it.

`test_gate_contract` now covers `relocalise.sh` and `lost.sh` **generically**: every
`stats` key either reads through `stat_of` must be in that node's printf format, and
every literal pattern either `grep`s must match some line the nodes can actually print —
each format string rendered with sample values, the grep translated to Python. Plus
`reloc_truth.py`'s regex parsing a `relocalised` line rendered from the node's own
format, all seven pose numbers in TUM order. Five mutations — a renamed key on each
side, `map loaded:` reworded, `last_black_ns` renamed, the pose separator changed —
each fail. `test_dashboard_contract` gained the tracking strings: the page decides LOST
as `pose.tracking !== 'OK'`, so the node spelling OK any other way would flag LOST for
ever over a tracker that was fine; it pins that `'OK'` is sent only for
`TrackingState::OK` and that the fallback — what a state nobody set gets — is
`UNKNOWN`. Its first run failed on the test's own regex, not the code. And the
`UNKNOWN == 0` contract both nodes lean on is a **`static_assert` carrying its message**
in `fusion_node.cpp` and `dashboard_node.cpp`, the place `Percentile`'s signature lesson
says a contract belongs, rather than a comment in the `.msg`.

**Milestone I's suites (added 2026-10-02, #13's P19 and P20).** Every one of them
covers code whose wrong version still runs.

**`test_tracking_state`** is the OK / LOST monitor, and both directions of getting it
wrong produce a pipeline: a monitor that goes LOST eagerly refuses a fifth of
`bags/desk1`, one that never does fuses through every blackout and has a perfect
"nothing fused while LOST" record. It pins LOST on exactly the `lost_after_holds`-th
consecutive hold and never on scattered ones (desk1's ~20% held frames were singles
when P7 measured them), recovery on *consecutive* fits, and the trap the header
documents: the monitor counts its own run, because `odometry_node`'s hold counter is
reset by the stall rule every 5 frames, and a threshold above 5 driven from it would
be a LOST that can never be entered. P20's half: with a loaded map it starts LOST,
fits alone never recover it, `relocalised()` does exactly once, and a late answer
while OK changes nothing.

**`test_map_io`** is the saved map. Read back both ways — through the loader and byte
by byte against the layout `map_io.hpp` documents, `test_mesh_io`'s lesson — and
**cut at every byte**, each cut asserted to refuse and to leave the caller's map
alone, because a map read leniently from a short file is a smaller map and a smaller
map is a plausible one. Trailing bytes, a wrong magic and an implausible count are
refusals too; the last is a refusal and not an allocation.

**`test_place_recognition` gained the relocaliser.** `relocalised_pose` is one line
with its own case, for P7's reason: the composition is checked on an *asymmetric*
pair, and the case asserts the hurried composition lands half a metre away — two
coincident views would pass either way. `across_sessions` is pinned on a saved
keyframe that shares every track id with the query and is stamped *later*: within a
session both exclude it, across sessions neither means anything. The thread recovers
a query's map pose to 2 cm while its odom pose is somewhere unrelated, refuses
another room, keeps only the newest of three waiting queries, and stops with one
queued.

**`test_lost_timeline`** and **`test_reloc_truth`** are the two gates' instruments.
`lost_timeline.py` counts depth frames to LOST and back against an injected blackout,
and three of its failure modes flatter the tracker: UNKNOWN read as OK, a LOST from
*before* the blackout credited to it, and stamps compared as doubles — at 1.8e18 ns
two frames 64 ns apart are equal, which is why it is Python and not awk. The gate's
first run found a fourth: desk1 is LOST on its own from 17.5 s, so a blackout at 20 s
found it already LOST and "LOST in 1 frame" measured nothing; `state_before` is
reported and asserted since. `reloc_truth.py` scores relocalised poses against motion
capture through the Sim(3) of the *saved* keyframes; its cases pin that the error is
in TUM's metres (the pipeline's are ~0.5 of them on fr1/desk, so an error left in map
units reads at half size) and that a relocalised pose never joins its own alignment.
The first version of that suite failed — its fixture had the scale inverted, and the
instrument was right. Then it failed **on the Pi only**: a rotation bound of 1e-6° on
an angle off `acos`, which resolves ~8e-7° near zero — `test_ground_truth`'s lesson
from after milestone H, repeated a week later and caught the same way, by running the
suite at both ends. Mutations of both instruments are each caught.

**`test_transforms` gained the seventh and eighth pairs** — `fusion_node` and
`dashboard_node` reading the tracking state `odometry_node` writes — and a check
nothing had: **that no launch override's default replaces the YAML's value**. The
overrides are applied *after* the YAML on every launch, so a drifted default is not a
default at all. Its first run found one: `scale_probe`'s `duration_s: 30.0` had never
taken effect, because `probe_duration_s` defaults to 60.0 — harmless, since
`gates/scale.sh` always passes its own window, and exactly the kind of number that is
wrong in the file somebody reads. **`test_gate_contract`** gained the keys `lost.sh`
reads off `stats lost`, `stats tracking` and the timeline, each checked by renaming
one.

**Two gaps closed after milestone H, 2026-10-02.** `SharedVolume::replace` — the
swap that makes a rebuild visible to `mesh_node` — had no test: `test_shared_volume`
now asserts the next snapshot is the rebuilt volume, the old one comes back whole
rather than freed under the lock, the frame count becomes the rebuild's, and both
nodes still meet at one object after the swap (swapping the shared object instead of
its contents would be the `volume_key` failure by another door). And
`PlaceRecognizer::corrections()`, what `/pose_graph/corrections` carries, is pinned
at the thread: identity before a closure, and each correction applied to its
keyframe's odometry pose reproducing the corrected trajectory. Mutations — a
`replace` that does not swap, one that keeps the old count, and corrections never
refreshed — are each caught by the case that claims them.

**`test_ground_truth`** (added 2026-10-02, #12's P18) is the instrument that closed
P18, written because the first one could not see the claim. Its central case walks a
camera twice past a wall, the second pass's odometry drifted, with ground truth in
**another frame at twice the scale** so the alignment has real work to do, and
asserts over the *same twelve frames* that the drifted arm scores further from the
ground-truth surface **and** the true arm scores close to it — so it cannot pass by
both being bad or by the score ignoring poses. Umeyama is pinned against a known
similarity, the reflection guard and the degenerate refusals each by a case, and the
memory dump round-trips at full precision with a missing image refused. Four
mutations — no reflection guard, `apply` ignoring scale, the reference rebuilt at the
arm's own poses (the very mistake this instrument replaces), and the alignment fitted
backwards — are each caught. **And one case was too tight for a machine**: the first
rotation check used an angle off `acos`, whose resolution near 1 is ~8e-7°, against
a 1e-6° bound; the Pi's aarch64 rounding landed at 1.2e-6 and failed a correct fit.
It compares the matrices now. `test_place_truth` gained the bounded rule (a closure
losing by less than ε is tolerated and still listed; by more, it counts), and
`test_gate_contract` the keys `rebuild.sh` reads off `rebuild_eval` and
`stats rebuild`, and those `place.sh` reads off the judge — each checked by renaming
one.

**`test_rebuild`** (added 2026-09-30, #12's P18) covers the frame memory, the
correction a frame takes, and the rebuild, and every wrong version of them produces a
surface. Its central case builds a wall seen twice, the second pass's odometry
drifted 0.3 m toward it, and asserts **both** halves: rebuilt at the corrections, the
first surface is at 2.0 m; rebuilt without them — the control, the same function one
input apart — the ghost at 1.7 m is still there. The memory must *thin* the whole
session when full rather than forget its start (a ring buffer fails that case), a
frame takes its reference keyframe's correction and not an interpolation (an
interpolation fails that case), and the rebuild integrates every remembered frame.
**One of its cases passed over the bug it was written for**: the allocation-stride
test put its pole on a column the stride-8 sampler hits anyway, so deleting the
stride division left it green; moved between samples, and aimed at by a ray that
actually crosses it, the same mutation fails it. `test_pose_graph` gained the
per-keyframe corrections: exact identity until a solve (the rebuild's control has to
be the uncorrected poses to the bit), consistent with the corrected trajectory after.

**`test_pose_graph`** (added 2026-09-30, #12's P17) exists because every way a pose
graph is wrong *converges*. It pins g2o's edge convention against answers worked by
hand — a closure 2 m ahead of a 1 m odometry edge must land the pose past 1 m, and a
measurement taken inverted pulls it back past the origin; a closure trusted in
rotation and ignored in translation must leave translation alone, and the
information matrix's halves swapped does the opposite — and both mutations were run
and caught. **Its first run found a real flaw**: the correction was recomputed per
keyframe as `corrected · odom⁻¹`, identity only to 1e-14 and compounding, on the
control that must be odometry *exactly*; it is stored now. **And its second found a
property worth more than a fix**: under the odometry noise model measured on
fr1/desk, a 5 m false closure on a 16 m loop is absorbed with zero inconsistent
loops. `UnderTheMeasuredOdometryModelAFalseClosureIsAbsorbedSilently` pins that, so
nobody mistakes `inconsistent_loops == 0` for a safety check — the defence is P16's
precision. `test_place_recognition` gained the thread's half: a closure moves
`map_from_odom` only with `close_loops` on, and every keyframe of a backlog reaches
the graph (a skipped *query* is fine; a missing odometry edge is not) — both mutations
caught. `test_transforms` pins that `map → odom` has exactly one publisher with
`pipeline` either way, and `test_gate_contract` the keys `loop.sh` reads.

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
