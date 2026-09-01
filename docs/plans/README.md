# Plans

A plan is the build order for one piece of work: numbered phases, each ending in
something that runs and something that checks it.

- New plans start in `in-progress/` and are **moved** to `completed/` when the
  work is done. Moving the file *is* the status change — fix inbound links when
  it moves.
- **Phase numbers are stable.** Once written, a phase's number and scope never
  change, so that "P3" means the same thing in every doc, commit message and
  conversation that mentions it. Record progress by annotating the phase with
  dates, ✓ marks and what actually happened — never by renumbering or
  reshuffling.
- A completed plan is kept as the **build log**. It is the record of why things
  are the way they are, including the levers that were measured and rejected.
- Every phase ends with a gate that names its evidence. "Looks right" is not a
  gate.

| Plan | Status |
| --- | --- |
| [in-progress/bootstrap-plan.md](in-progress/bootstrap-plan.md) | In progress — the whole pipeline, P0–P8 |
