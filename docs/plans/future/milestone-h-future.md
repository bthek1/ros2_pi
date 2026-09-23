# Milestone H — deferred

Work that came out of [#12](https://github.com/bthek1/ros2_pi/issues/12) and is
**not executable yet**. Each entry names the trigger. See
[../README.md](../README.md) for the rules.

---

## A DBoW2 vocabulary instead of brute-force descriptor search

**What.** A bag-of-words index over the keyframe store, so place recognition is
a vocabulary lookup rather than a Hamming scan of every keyframe.

**Why not now.** Two reasons, and the second is the real one.

The store is capped at **500 keyframes at ~40 kB each**, and a Hamming scan over
a few hundred descriptor sets is likely fast enough to start — P16 measures it
rather than assuming either way.

And **DBoW2 is in neither distro's apt** (checked 2026-09-23 on both machines;
`ros-lyrical-rtabmap` and its Jazzy twin vendor one internally). So it is a git
submodule that must build and behave **identically** on Jazzy and Lyrical —
which is the tax this workspace keeps paying, and the reason the dashboard has a
hand-written RFC 6455 rather than a vendored header. A vocabulary file is also a
large binary artefact with a fetch script and a sha256 behind it, like the depth
weights.

**Trigger.** `bash tools/gates/place.sh` reporting query cost per keyframe above
the budget it sets, **or** the store's ceiling needing to rise past ~500 for a
real room. Either makes it real work; until then brute force is fast enough by
measurement.

---

## CUDA kernels for the volume rebuild

**What.** Move P18's rebuild — re-integrating a few hundred remembered frames at
corrected poses — onto the GPU. The predecessor measured ~10 ms a frame there.

**Why not now.** Same blocker as the existing "CUDA kernels for TSDF
integration" entry in
[project_final_state.md](project_final_state.md#cuda-kernels-for-tsdf-integration):
there is **no CUDA toolkit installed**. `bash tools/fetch-gpu-stack.sh` installs
the *runtime* libraries, and **runtime libraries are not a compiler** — `nvcc`
is absent and a hand-written kernel needs a real toolkit install.

**Trigger.** `gates/rebuild.sh` reporting a rebuild wall time that makes a
closure visibly stall the session, **and** a CUDA toolkit installed. Two
conditions, because the second is not something this plan controls.

---

## Closures between sessions, not only within one

**What.** Detect that this session's room is a room a *previous* session mapped,
and merge the two maps rather than starting a new one.

**Why not now.** It needs a persisted map, which is **P20** in
[#13](https://github.com/bthek1/ros2_pi/issues/13), and merging two maps is a
strictly harder problem than recovering a pose in one — two pose graphs with no
shared odometry edge, and a scale ratio between them that is not 1 because
monocular scale is per-session.

**Trigger.** P20 green. It becomes a phase in a later plan, not one here.
