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
**Test:** `just gate-depth` — runs 200 frames through depth_node, asserts the
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
| [#2 Hello-world plan](https://github.com/bthek1/ros2_pi/issues/2) | **Closed 2026-09-08** — P0–P4 done, all five gates green; kept as the build log | [future/hello-world-future.md](future/hello-world-future.md) |
| [#1 Bootstrap plan](https://github.com/bthek1/ros2_pi/issues/1) | Open, `deferred` — P0–P8, the whole pipeline | [future/bootstrap-future.md](future/bootstrap-future.md) |
| [#3 Justfile plan](https://github.com/bthek1/ros2_pi/issues/3) | Open — P0–P3, group the recipes and move the gate bodies to `tools/` | — |

This table is a convenience, not the source of truth —
`gh issue list --label plan --state all` is.
