# AGENTS.md

The working agreement for this repository lives in [CLAUDE.md](CLAUDE.md). Read
it first — it carries the machine layout, the pipeline budgets, and a list of
constraints that have already cost this project's predecessor real debugging
time.

Short version:

- **C++ only for nodes** (`ament_cmake`, C++17, `rclcpp_components`). Python is
  for launch files and offline tools.
- **The Pi captures and ships JPEG; the dev box does everything else.** Never
  put raw images on the LAN.
- **Two different ROS distros** (Lyrical here, Jazzy on the Pi) — everything
  builds from source on both machines, nothing is shipped as a binary.
- **The scaffolding runs; the pipeline does not.** One package exists,
  `src/pimesh_hello/`, and five `tools/gates/hello-*.sh` scripts assert it. Camera,
  depth, fusion, mesh and dashboard are unwritten, and everything `docs/` says
  about them is intent. Do not write anything up as if it runs until you have
  watched it run.
- **Build with `just build`, not bare `colcon`** — the recipe passes
  `-DPython3_EXECUTABLE=/usr/bin/python3`, without which every `ament_cmake`
  package fails at configure time on this box.
- **The justfile is three commands** — `build`, `hello-compose`, `hello-lan` —
  and that is deliberate. Gates, Pi plumbing and the straggler sweep are
  `bash tools/gates/<name>.sh` and `bash tools/<name>.sh`, run directly. Resist
  adding a recipe.
- **Clean up after yourself**: `bash tools/stragglers.sh` checks both machines and exits
  non-zero if anything survived. A leaked camera process locks `/dev/video0` for
  everyone. Bound anything you start by hand with
  **`timeout --foreground -s INT <secs>`** — a *bare* `timeout` puts the command
  in a process group your Ctrl-C never reaches, so it cannot be interrupted at
  all.
- **A plan is a GitHub issue** (`gh issue create --label plan`), never a markdown
  file in this tree, and **completion is closing it**
  (`gh issue close <n> --reason completed`). Executable phases only — stable
  numbers, and every phase ending in a test that is a command. Anything that is
  waiting on something goes in `docs/plans/future/`, with the trigger that would
  make it executable. Never write a phase like "check back in 48 hours". Full
  rules: [docs/plans/README.md](docs/plans/README.md).
