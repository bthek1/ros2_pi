# Plans

**A plan is a GitHub issue, not a file in this repo.** The issue body is the
build order for a piece of work, as numbered phases, each ending in a test that
is a command.

Open one with `gh issue create --label plan`. Browse them with
`gh issue list --label plan`. What lives in this directory is only the rules
below and the [future files](#the-future-file).

## The three rules

### 1. Phases are stable

`## P0`, `## P1`, … Once a phase is written, **its number and its scope never
change**, so that "P3" means the same thing in every doc, commit message and
conversation that mentions it. Record progress by **editing the issue body** to
annotate the phase — a date, a ✓, what actually happened, what the test printed.
Never renumber, never reorder, never fold two phases together. A phase that
turns out to be wrong is annotated as abandoned and keeps its number; the
replacement is a new phase at the end.

The body is the plan. Comments are commentary — a phase is not done because a
comment says so, it is done because the body says so.

### 2. Every phase ends in a test that is a command

Not "verify it looks right". Not "check the mesh in RViz". A phase's test is a
recipe someone can run that **exits 0 or non-zero and prints the number it
asserted on**:

```
**Test:** `bash tools/gates/depth.sh` — runs 200 frames through depth_node, asserts the
startup log names CUDAExecutionProvider and that mean per-frame cost is ≤ 80 ms;
prints the measured mean and p95.
```

- The test recipe is written **in the same change as the phase's code**, not
  afterwards. A phase without a runnable test is not finished.
- The test names its own evidence: a number on a topic, a log line with a
  threshold, or a rendered image file it wrote. A screenshot is not evidence, and
  neither is a viewer window.
- Where a check genuinely needs a person — a tape measure, exposure in a real
  room, whether the mesh looks like the room — say so **inside the phase**, say
  what the person must do in one sentence, and name the one recording that would
  turn it into a replayable test later.

### 3. Executable phases only

**Every phase must be startable the moment the plan reaches it**, with the
hardware and the code that exist by then. Depending on an earlier phase in the
same plan is fine — that is what the ordering is for. What disqualifies a phase
is depending on something the plan does not control: a purchase, an upstream
release, an undecided outcome, or the passage of time.

The test: could someone open the issue at this phase and start work today, with
nothing to wait for? If not, it is not a phase.

Things that are never phases:

- "Check status in 48 hours" — or in a week, or next month.
- "Monitor for stability over a few sessions."
- "Revisit once we have more data."
- "Decide later whether to keep it."
- "If it turns out to be slow, optimise it."

Each of those is a deferred item. It goes in the future file.

## The future file

Deferred work stays in the repo as markdown: `future/<plan-name>-future.md` —
one companion file per plan issue, and **never inside the issue body**.

Every entry names three things:

| | |
| --- | --- |
| **What** | The work, in a sentence or two |
| **Why not now** | What makes it non-executable today |
| **Trigger** | The specific thing that would make it executable — a measurement crossing a threshold, a named phase completing, a package becoming installable |

The trigger is the load-bearing part. "Later" is not a trigger; "once P5's
integrate cost exceeds 20 ms at 13 Hz" is.

**Promotion.** When a trigger fires, the entry is **deleted** from the future
file and appended to the issue body as the **next unused phase number**, with a
test. That is the only route from deferred to built. An item never sits in both
places, and a plan never grows a phase in the middle.

**Retirement.** An entry whose trigger can no longer fire — the hardware went
away, the approach was superseded — is deleted with a one-line note in the issue
saying why. The future file is a live register, not a graveyard.

## Lifecycle

```
gh issue create --label plan        ──▶   gh issue close --reason completed
        │                                   (closing the issue IS
        │  defer                             the status change)
        ▼
docs/plans/future/<name>-future.md
        │  trigger fires
        └──▶ back into the issue body as the next phase number
```

- **Open, unlabelled beyond `plan`** — being worked on.
- **`plan` + `deferred`** — written down but not started; the body says what
  would start it.
- **Closed as completed** — done. A closed plan issue is the **build log**: the
  record of why things are the way they are, including the levers that were
  measured and rejected, with every phase annotated by what its test printed. Do
  not delete or edit history out of it. Its future file stays in this directory
  if entries remain.

Close a plan only when **every phase is annotated done and its gate has been
run** — not when the code exists. If work is abandoned, close it with
`--reason "not planned"` and a comment saying what replaced it.

```bash
gh issue close 2 --reason completed --comment "All gates green; see body for measured numbers."
```

## Index

| Plan | Status | Future file |
| --- | --- | --- |
| [#2 Hello-world plan](https://github.com/bthek1/ros2_pi/issues/2) | **Closed 2026-09-08** — P0–P4 done, all five gates green; kept as the build log | — (merged into [future/project_final_state.md](future/project_final_state.md) on 2026-09-09) |
| [#3 Justfile plan](https://github.com/bthek1/ros2_pi/issues/3) | **Closed 2026-09-09** — P0–P3 done, `tools/gates/justfile.sh` green; 627-line justfile → 102, then → 60 when the gates left it | — |
| [#4 Milestone A](https://github.com/bthek1/ros2_pi/issues/4) | **Closed 2026-09-09** — `P0–P1` done, `tools/gates/build.sh`, `capture.sh` and `view-configs.sh` green; kept as the build log | [future/milestone-a-future.md](future/milestone-a-future.md) |
| [#9 Camera calibration](https://github.com/bthek1/ros2_pi/issues/9) | **Closed 2026-09-12** — `P9` done, `tools/gates/calibration.sh` green (held-out reprojection 0.4955 px, median straightness 0.7796 px, coverage 0.896); kept as the build log. Its test was re-scoped mid-phase because the camera turned out to have no lens distortion to remove | [future/project_final_state.md](future/project_final_state.md) — "Re-calibrate on a flat mount", triggered by P5's tape measure |
| [#5 Milestone B](https://github.com/bthek1/ros2_pi/issues/5) | **Closed 2026-09-13** — `P2–P3` done, `tools/gates/ipc.sh` and `keypoints.sh` green (504/504 buffer addresses against 0/395 with intra-process off; 57.9 Hz at 5.99 ms/frame, matched fraction 0.9063 against the reference's 0.9065); kept as the build log. Both of its gates reported a false result first — one measuring a single consumer, one measuring a 20 s window of a non-uniform clip | [future/milestone-b-future.md](future/milestone-b-future.md) |
| [#6 Milestone C](https://github.com/bthek1/ros2_pi/issues/6) | **Closed 2026-09-15** — `P4` done, `tools/gates/gpu-stack.sh` and `depth.sh` green (`CUDAExecutionProvider`, 55.10 ms mean per frame and 58.21 ms p95 against an 80 ms budget, 17.42 Hz, 1048/1048 `/depth/rgb` frames byte-identical, CPU control at 287.92 ms); kept as the build log. Its toolchain gate passed throughout a run in which the node was on the CPU at 517 ms — the instrument was an executable and the node is a component, which is a case that gate structurally cannot reach | [future/project_final_state.md](future/project_final_state.md) |
| [#7 Milestone D](https://github.com/bthek1/ros2_pi/issues/7) | **Closed 2026-09-16** — `P5–P6` done, `tools/gates/fusion.sh` and `mesh.sh` green (15.3 ms per integration at 17.1 Hz with 0.19% displaced; 790 668 triangles marched in 2.8 s and decimated to 120 000, boundary loops 5119 → 448, worst integration gap 374.7 ms against a control's 401.3 ms); kept as the build log. **Both phases had their test re-scoped by measurement** — the scale aligner makes no measurable difference under rotation-only odometry, and the clip's own 400 ms stall makes P6's ratio unachievable, so each gate grew a control run instead | [future/milestone-d-future.md](future/milestone-d-future.md) |
| [#8 Milestone E](https://github.com/bthek1/ros2_pi/issues/8) | Phases `P7–P8` of the pipeline, plus `P10` promoted out of milestone A's future file when its trigger fired — all three done 2026-09-19 | [future/project_final_state.md](future/project_final_state.md), [future/milestone-e-future.md](future/milestone-e-future.md) |
| [#10 Milestone F](https://github.com/bthek1/ros2_pi/issues/10) | **Open 2026-09-23, P11 done 2026-09-25** — phases `P11–P13`: an ATE against TUM fr1/desk (**0.27–0.36 m Sim(3)-aligned over seven runs**, `bash tools/gates/trajectory.sh`), `depth_scale` pinned with a tape measure, and `bags/walk1` recorded. The first milestone whose phases are judged by a number this project did not produce — and P11's fitted scale has since bounded P12's answer at 4.6–5.2 without one. **P12 and P13's software is written and exercised as of 2026-09-25; what is left is one visit to a room**, and the checklist is [docs/info/setup.md](../info/setup.md#the-visit-to-the-room) | [future/milestone-f-future.md](future/milestone-f-future.md) |
| [#11 Milestone G](https://github.com/bthek1/ros2_pi/issues/11) | **Open 2026-09-23, `deferred`** — phases `P14–P15`: map points observed by many keyframes, and local bundle adjustment. Trigger: `tools/gates/trajectory.sh` green | [future/milestone-g-future.md](future/milestone-g-future.md) |
| [#12 Milestone H](https://github.com/bthek1/ros2_pi/issues/12) | **Open 2026-09-23, `deferred`** — phases `P16–P18`: place recognition, a pose graph that owns `map -> odom`, and a volume rebuilt at the corrected poses. Absorbs the "Loop closure" entry from `project_final_state.md`, whose trigger fired on 2026-09-19. Trigger: `tools/gates/ba.sh` green and `bags/walk1` existing | [future/milestone-h-future.md](future/milestone-h-future.md) |
| [#14 Rename plan](https://github.com/bthek1/ros2_pi/issues/14) | **Open 2026-09-23** — phases `P0`–`P8`, its own numbering. `P0` and `P2`–`P7` done 2026-09-24: `tools/gates/naming.sh` written first as the guard, `pimesh_perception` split into `pimesh_frontend` + `pimesh_depth`, `pimesh_world` → `pimesh_mapping`, the shared helpers into `pimesh_core`, component headers into `nodes/`, `apps/` and `probes/`, `pimesh_instruments` for the gates' Python instruments, and `tools/` grouped. **`P1` (the `pimesh_` prefix) and `P8` (the repository name) are abandoned deliberately** and keep their numbers | — |
| [#13 Milestone I](https://github.com/bthek1/ros2_pi/issues/13) | **Open 2026-09-23, `deferred`** — phases `P19–P20`: a `LOST` state that stops fusing, and relocalisation from a persisted map. Trigger: `tools/gates/loop.sh` green | [future/milestone-i-future.md](future/milestone-i-future.md) |

This table is a convenience, not the source of truth —
`gh issue list --label plan --state all` is.

**The pipeline is split across nine milestone issues, over one shared phase
list.** [future/project_final_state.md](future/project_final_state.md) holds the
whole thing — phases P0–P20 followed by the deferred register — and issues
#4–#8 (the pipeline, all closed) and #10–#13 (the SLAM half, opened 2026-09-23)
each take a **contiguous slice** of it. This is the one place where a plan's
phase list lives in the tree rather than in an issue body, and rule 1 is the
reason for the arrangement rather than a casualty of it: one shared numbering
means `P4` is depth in all nine issues, where nine separate plans would each
have started at P0 and collided.

**P0–P10 are written out in that file; P11–P20 are one line each with a link to
the issue.** The file holds the numbering, not the plans — a plan is an issue.
The older phases stay written out because they are now the **build log**, with
what each test printed annotated into them.

Promotion still works as described above — a fired trigger moves an entry out
of the deferred half of that file and into the phase list as the next unused
number, then into whichever milestone issue is open.

Each milestone issue also carries a **`just view-*` RViz recipe**. That is a
viewer for a person, not a gate: it does not close a phase, and the rule at the
top of this page still holds — the evidence is the number a script printed.
