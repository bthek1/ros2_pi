# Deferred out of milestone D

Work that came out of P5 and P6 and is **not executable now**. Each entry names
the trigger that would make it executable — the measurement, the phase or the
hardware it is waiting on. When a trigger fires, the entry is deleted from here
and appended to [issue #7](https://github.com/bthek1/ros2_pi/issues/7) as the
next unused phase number, with a test. It never sits in both.

See [docs/plans/README.md](../README.md) for the rules.

---

*Promoted 2026-09-23 to **P12** in [#10](https://github.com/bthek1/ros2_pi/issues/10) — its trigger was a
person, a tape measure and the camera in a room, and milestone F is the
visit. Deleted from here; the phase is the plan now.*

---

## Make the scale-alignment comparison an assertion, once it repeats

**Trigger: the result below holding across runs.** P7 was the old trigger and it
has fired; what is left is repeatability, which is a measurement anyone can take
today — three runs of `bash tools/gates/fusion.sh` and a look at whether the sign
is stable.

**P7 fired, and the comparison moved.** Re-measured 2026-09-19 with the corrected
pose and 6-DoF translation in place, the same gate reports **0.4805 m aligned
against 0.5330 m unaligned** and agreement **0.2191 against 0.1606** — the aligner
ahead on both for the first time, where before the winner alternated window by
window. That is consistent with the diagnosis below: it was correcting the smaller
error while a mis-composed pose supplied the larger one.

It is still **printed rather than asserted** because that is one run of a number
which has already flipped once, and a gate that asserts on a number which
alternates is a flaky gate — the worst kind, because it teaches people to re-run
until it passes. When three runs agree on the sign, the comparison goes back into
`gates/fusion.sh` as an assertion with a margin taken from the spread of those
runs, and the long note in its header is replaced by the measurement.

The original entry follows, because it is what the number above is evidence
about.

---

P5 asks `tools/gates/fusion.sh` to assert that per-frame scale alignment makes
two views of the same wall agree better. Measured 2026-09-16 on `bags/desk1` it
does not, and the two runs are indistinguishable: median surface gap 1.32 m
aligned against 1.30 m unaligned, agreement fraction 0.114 against 0.131, with
the winner alternating window by window.

The aligner is not broken — `test_scale_aligner` pins its properties, including
that a constant bias produces corrections whose product is exactly 1 — it is
correcting the smaller error. `keypoint_node` publishes rotation only, so a
hand-held sweep's ~0.9 m of real arm arc is modelled as zero; at 2-3 m that is a
30-45% geometric error, against a scale wobble the aligner clamps at 15% and
which hits that clamp on a fifth of frames.

When P7 lands, re-run `bash tools/gates/fusion.sh` and see whether the two runs
separate. If they do, the comparison goes back into the gate as an assertion and
the long note in its header is replaced by the measurement. If they still do not,
that is worth knowing too — and would say the aligner is not earning its 20 ms
of ray-casting per frame.

*(They separated. See the top of this entry.)*

---

*Promoted 2026-09-23 to **P18** in [#12](https://github.com/bthek1/ros2_pi/issues/12) — its trigger was P7's
keyframe store existing **and** a pose-graph backend that moves past poses;
P7 landed 2026-09-19 and P17 supplies the second. Deleted from here.*

---

## Free-space carving, so a moved surface does not leave a ghost

**Trigger: a measurement showing ghosts are costing something after P7 lands.**

Only the blocks this frame's truncation band names are updated, so a surface that
moves further than one truncation is not corrected — it is *joined*, and the old
one stands. `test_tsdf_volume` pins this as behaviour rather than leaving it to be
discovered.

It matters today because the depth model's frame-to-frame wobble is ±4%, which at
2.5 m is ±10 cm against a 6 cm truncation: unaligned frames genuinely land outside
each other's bands and stack into shingles. Carving the free space between the
camera and the surface would erode the stale layers.

Deferred because it is not obviously the right fix for the right problem. The
shingling on `bags/desk1` is dominated by the missing translation, and carving
would be a second expensive pass per frame added to remove a symptom of something
P7 removes at the source. Re-measure the block count per square metre of real
room after P7; if it is still an order of magnitude too high, this becomes a
phase with that ratio as its test.

---

## A smaller voxel record, if memory becomes the binding constraint

**Trigger: the map exceeding what the dev box can hold with P7's odometry in
place.**

A voxel is 12 bytes: a float distance, a float weight, three colour bytes and a
pad. Storing the distance as an int16 normalised over the truncation and the
weight as a uint16 would make it 8 bytes — a third off the map and off the
snapshot copy with it.

Not done because it trades real precision for memory that is not currently
scarce, and because the reason the map is large today is shingling rather than
resolution: 200 000 blocks is 2000 m² of surface for a 60 m² room. Fix the cause
first. If after P7 a real room still needs more than the box can hold, this is the
cheapest remaining lever and the test is a block count and a reprojection
comparison against the 12-byte record on the same clip.

---

## Poisson-closed companion mesh for downstream tools

**Trigger: something downstream actually needing a watertight surface.**

`docs/info/pipeline.md` used to promise `/world/save_mesh` would optionally write
a Poisson-closed watertight companion beside the honest hole-bearing mesh. P6
writes only the honest one.

Deferred rather than dropped because the requirement was never real: nothing in
this project reads a mesh yet. It becomes a phase when something does — and the
rule it would have to keep is the one `fill_interior_holes` already keeps, that
assumed geometry is clearly labelled as assumed rather than blended into observed
surface.
