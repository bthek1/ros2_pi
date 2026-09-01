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
- **Nothing is built yet.** `docs/` describes intent. Do not write it up as if
  it runs until you have watched it run.
- **Clean up after yourself**: no orphaned nodes on either machine, and a leaked
  camera process locks `/dev/video0` for everyone.
