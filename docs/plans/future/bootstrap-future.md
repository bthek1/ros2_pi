# Future — deferred out of the bootstrap plan

Companion to [../in-progress/bootstrap-plan.md](../in-progress/bootstrap-plan.md).

Everything here is **not executable yet**, which is why it is not a phase. Each
entry names the **trigger** that would make it executable. When a trigger fires,
the entry is **deleted from this file** and appended to the plan as the next
unused phase number, with a test — see [../README.md](../README.md).

"Later" is not a trigger. If an entry's trigger is not something that can be
observed happening, it is not written down properly yet.

---

## Loop closure, pose graph, and volume rebuild

**What.** Always-on loop detection against the keyframe store, a pose-graph
backend owning `map → odom`, and a TSDF rebuild from frame memory when the
optimised trajectory moves the frames. The predecessor has all three, with
numbers to compare against: 2.3 cm / 0.85° on its loop bag, fr1/desk ATE
0.163 → 0.089 m, paired-surface gap 7.8 → 5.7 cm.

**Why not now.** It needs a keyframe store that exists (P7) and a surface worth
correcting (P6), and it is a plan of its own — four phases at least, with its own
gates and its own reference bag.

**Trigger.** P7 done **and** `just gate-odom` showing drift over the 60 s clip
that a closure could remove. At that point it becomes its own plan
(`slam-plan.md`), not a phase appended here.

---

## CUDA kernels for TSDF integration

**What.** Move the frustum integration loop onto the GPU. It is embarrassingly
parallel and is the stage most likely to become the bottleneck once fusion runs
at full rate.

**Why not now.** There is **no CUDA toolkit installed** — `nvcc` is absent and
there is no `libcudart` in `/usr/lib`. And optimising a CPU integrator that has
never been profiled is guessing.

**Trigger.** `just gate-fusion` reporting integrate cost **above 20 ms** at
13 Hz, or the profile showing integration above 30% of the frame budget. Either
one makes it real work; until then the CPU version is fast enough by
measurement, not by hope.

---

## TensorRT execution provider, and smaller depth input

**What.** Two independent levers on depth cost: a cached TensorRT engine instead
of the CUDA provider, and 392² input instead of 518² (roughly halves the cost,
coarsens thin structure).

**Why not now.** Both are optimisations of a stage that does not run yet, and
the GPU is Turing without tensor cores, so the usual fp16 argument does not
apply here — the win has to be measured, not assumed. A TensorRT engine is also
a long build and a per-machine cache, which is real operational cost.

**Trigger.** `just gate-depth` passing (so there is a baseline) **and** the
pipeline needing more than ~13 Hz for a reason that has been written down. Take
the input-size lever first: it is a parameter change, and it can be measured in
an afternoon.

---

## Relocalisation from a saved room map

**What.** Persist the keyframe store to disk, load it at startup, and recover an
absolute pose against a room seen in an earlier session.

**Why not now.** The store lands in P7 because it is cheap to build alongside the
6-DoF work, but nothing consumes it until there is a backend that can act on a
recovered pose.

**Trigger.** The loop-closure plan above existing as a plan. Relocalisation is a
phase in that plan, not a phase in this one.

---

## Building OpenCV with CUDA

**What.** A source build of OpenCV with the CUDA module, so `cv::cuda::` links
and ORB could run on the GPU.

**Why not now.** The apt OpenCV (4.10.0) has no CUDA module, and ORB at 500
features is budgeted at ~5 ms on the CPU — well under the depth stage's 76 ms. A
source build of OpenCV is a large, self-inflicted maintenance burden for a stage
that is not the bottleneck.

**Trigger.** `just gate-keypoints` reporting ORB cost **above 15 ms/frame**, or a
feature count above ~2000 becoming necessary for matching quality. Anything less
is not worth the build.

---

## Retired

Nothing yet. When an entry's trigger can no longer fire — the hardware went
away, the approach was superseded — delete it from above and record one line
here saying why, so the reasoning survives.
