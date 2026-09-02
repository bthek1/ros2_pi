# Provisioning the Pi with Ansible

> **Status: built and gated, 2026-09-02.** `ansible/` exists and
> `just gate-provision` passes — the playbook is idempotent (first apply
> `changed=11`, every apply since `changed=0`), and this repo is now the sole
> owner of the Pi's configuration. Built by **P9** of
> [the bootstrap plan](../plans/in-progress/bootstrap-plan.md#-p9--provision-the-pi-with-ansible),
> whose annotation carries what the first apply actually changed. Measured facts
> about the Pi below were taken **2026-09-02**.

**The Pi's configuration is a file in this repo, not a shell history.** Anything
the Pi needs in order to build and run `pimesh_camera` — apt packages, the ROS
environment, the Cyclone DDS interface pin, the `video` group, the Wi-Fi
keep-alive — is a task in a role here. Nothing is installed on it by hand.

## Why bother, on a one-managed-host project

Three reasons, in order of how much they hurt when ignored.

**1. Drift across two ROS distros.** `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and
`RMW_IMPLEMENTATION` must be byte-identical on both machines, and this project
makes that *harder* than the predecessor did: the dev box runs Lyrical and the
Pi runs Jazzy, so the two environments cannot simply be the same file. Half of
[troubleshooting.md](troubleshooting.md) is what happens when they quietly
disagree — and the failure mode is **silence**, not an error. As `group_vars`
those values have one definition and re-running the playbook is what makes
drift impossible rather than merely unlikely.

**2. The reflash.** The Pi is a sensor head on Wi-Fi; a bad network change
leaves it needing a keyboard and a monitor, and the recovery is a reflash. A
playbook makes that one command instead of an afternoon of remembering. Concrete
example, measured today: `v4l-utils` is on the Pi because the *predecessor's*
playbook put it there. Ubuntu Server does not ship it, and nothing in this repo
records that this project depends on it except this sentence — which is exactly
the gap P9 closes.

**3. It is checkable.** "The Pi has a C++ toolchain" is an assertion. A playbook
that reports `changed=0` on its second run is a measurement. And the first run's
`changed` count is a measurement too — of how far the machine had drifted from
what this repo believes it needs.

## What Ansible owns, and what it does not

The boundary matters, because the predecessor blurred it and ended up with two
ways to copy the repo to the Pi.

| | Owner | How |
| --- | --- | --- |
| apt packages, ROS install, shell env, DDS config, groups, systemd units | **Ansible** | `ansible-playbook site.yml` |
| Copying this workspace to the Pi, and building it | **the justfile** | `just sync-pi`, `just build-pi` |

Ansible configures the **machine**. The justfile moves and builds the **code**.
The predecessor had a `workspace` role that did an `rsync --delete` and a
`colcon build`; this repo deliberately does not, because `just sync-pi` and
`just build-pi` already exist, are used by `just gate-build`, and are what you
reach for twenty times a day. Two mechanisms for the same job is how they end up
disagreeing about which one ran last.

## Taking over from the predecessor's tree

**The Pi is currently provisioned by `~/Documents/piros2/ansible`**, and the
result is measurably fine (see below). P9 forks that tree into this repo and
this repo's playbook becomes the **sole owner** of the Pi.

That hand-off has one sharp edge. The predecessor's `ros2_env` role writes
managed blocks into `~/.profile` and `~/.bashrc` with `blockinfile` markers. If
this repo's role uses *different* markers, the Pi ends up with **two** blocks
exporting the same three variables — harmless until they disagree, at which
point the last one sourced wins and neither file is obviously wrong. So:

- This repo's `ros2_env` role uses the **same marker strings** as the
  predecessor's, so it *replaces* the existing block rather than stacking a
  second one.
- **Do not run the predecessor's playbook against the Pi again** — not even
  `--limit robot`. Its `workspace` role would re-point the Pi at `~/piros2`.
- `just gate-provision` asserts there is exactly **one** managed ROS block in
  `~/.profile`, which is what catches this if someone does.

The dev box stays outside this playbook for now — it is the control node and is
still provisioned by the predecessor's tree. Bringing it in is a deferred entry
with a trigger in
[../plans/future/bootstrap-future.md](../plans/future/bootstrap-future.md).

## Where the Pi actually stands today

Measured **2026-09-02** over `ssh pi "bash -lc '…'"`, i.e. the state P9 inherits:

| Check | Result |
| --- | --- |
| `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | `42` / `rmw_cyclonedds_cpp` — correct |
| `CYCLONEDDS_URI` | `file:///home/bthek1/.config/cyclonedds/cyclonedds.xml` — set |
| `ANSIBLE MANAGED` blocks in `~/.profile`, `~/.bashrc` | one each |
| `build-essential`, `cmake` | present — g++ 13.3.0, cmake 3.28.3 |
| `ros-jazzy-rclcpp-components`, `-image-transport`, `-camera-info-manager` | present |
| `v4l-utils` | present |
| `linux/videodev2.h` | present, from `linux-libc-dev` — all a raw-ioctl V4L2 node needs |
| `libv4l-dev` | absent, and **not required** — it is the `libv4l2` conversion wrapper, which MJPEG passthrough does not use |
| `/dev/v4l/by-id/usb-046d_C922_…-video-index0` | present, → `/dev/video0` |

So the playbook's job on day one is close to nothing, and that is the point: the
value is that the list is written down and re-assertable, not that it is long. A
large `changed` count on the first apply would mean the roles disagree with the
machine — a finding, not a success. **g++ 13.3.0 is also why the Pi's packages
are C++17**: Jazzy's baseline, and the compiler that has to accept the code.

## Layout

What P9 built (2026-09-02):

```
ansible/
├── ansible.cfg
├── inventory.yml            # one managed host: pi
├── requirements.yml         # ansible.posix >= 2.0, community.general
├── site.yml
├── group_vars/
│   └── robot.yml            # jazzy, wlan0, the C922 by-id path
└── roles/
    ├── ros2_apt/            # ros2-apt-source .deb + keyring
    ├── ros2_install/        # ros-jazzy-ros-base, colcon, rosdep
    ├── ros2_env/            # .profile/.bashrc blocks, cyclonedds.xml template
    ├── toolchain/           # build-essential, cmake, the ros-jazzy-* build deps
    ├── camera/              # v4l-utils, the video group, by-id symlink check
    └── wifi/                # power-save off, link watchdog, sshd ClientAlive
```

**Not forked:** the predecessor's `usb_cam` install (this project replaces that
driver, and a second thing able to open the exclusive `/dev/video0` is a
liability), its `workspace` role (see the ownership table above), and its fish
shell integration (the Pi has bash).

Roles are copied from the predecessor's tree and trimmed, not written from
scratch — they have been run against this Pi for weeks. The `wifi` role exists
because **the Pi's link has died twice while the OS kept running**; its ladder
is reassociate → driver reload → reboot, one rung per three consecutive
minute-spaced gateway-ping failures, never rebooting within 10 minutes of boot.

### Variables

```yaml
# ansible/group_vars/robot.yml
ros_distro: jazzy                            # per-host — see below
ros_metapackage: "ros-{{ ros_distro }}-ros-base"
ros_domain_id: 42
ros_localhost_only: 0
rmw_implementation: rmw_cyclonedds_cpp
dds_interface: wlan0                         # eth0 is NO-CARRIER; there is no cable
camera_device: /dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0
```

**`ros_distro` is per-host, and never spelled into a role.** The dev box is on
Lyrical and the Pi on Jazzy; a role that hardcodes `jazzy` would silently
install the wrong ROS the moment the dev box joins the playbook. Package names
derive from the variable.

**`camera_device` is the serial-keyed `by-id` path, not `/dev/video0`.** It is
stable across replugs and reboots; `index1` is the C922's UVC metadata node and
is not a capture device.

Templating `cyclonedds.xml` from `dds_interface` is where Ansible earns its
keep. It renders to `~/.config/cyclonedds/cyclonedds.xml` — **outside the
workspace**, because `just sync-pi` runs `rsync --delete` and would remove a
per-host file living under `config/`, and a missing `CYCLONEDDS_URI` target
fails silently.

## Gotchas

Inherited from the predecessor, where each was diagnosed the expensive way.

- **Pin `interpreter_python = /usr/bin/python3` in `ansible.cfg`.** This is the
  same trap as the build: auto-discovery walks `PATH` and on this box finds
  uv's or PlatformIO's Python, neither of which can `import apt`. Every apt task
  then fails with a module error that looks nothing like its cause.
- **`rosdep init` is not idempotent** — it exits non-zero the second time. Guard
  it with `args: creates: /etc/ros/rosdep/sources.list.d/20-default.list`.
  `rosdep update` is the opposite: it runs as the login user, and under
  `become: true` it writes the cache into root's home.
- **Split the shell env across `.profile` and `.bashrc`.** Ubuntu's `.bashrc`
  opens with an interactivity guard that returns early, so anything appended to
  it is invisible to every non-interactive shell — including the `bash -lc`
  login shells this project verifies over SSH. `.profile` is read by all login
  shells; put the exports there.
- **`apt` needs `become: true`; `colcon build` must not have it.** Building as
  root leaves a `build/` tree the login user cannot overwrite, and it surfaces
  much later as a confusing permission error.
- **Adding a user to `video` does not affect the current session.** Follow the
  task with `meta: reset_connection` or the camera checks later in the same play
  run against a stale login.
- **Restart the ROS daemon on change.** Any task touching the environment or
  `cyclonedds.xml` notifies a handler running `ros2 daemon stop && ros2 daemon
  start`. The daemon caches discovery state; skipping this makes a correct fix
  look like it did nothing.
- **`ansible.posix` must be ≥ 2.0.** The 1.x `synchronize` plugin warns on every
  run under modern ansible-core. Measured on the control node 2026-09-02:
  ansible-core **2.20.1**, `ansible.posix` **2.2.2**, `community.general`
  **12.1.0** in `~/.ansible/collections`.
- **`stdout_callback: yaml` is gone.** `community.general` 12 removed it; use
  `ansible.builtin.default` with `result_format: yaml`.
- **A template change does not invalidate the environment cache.** The shell
  snippet reads a cached copy of what sourcing ROS produces, keyed on the mtime
  of the underlay and of the workspace's `local_setup.bash`. Neither moves when
  the *snippet* changes — so editing the overlay path leaves every login shell
  sourcing a cache built against the old workspace, silently. Found here on
  2026-09-02 while forking the role, before it bit: `ros2_env` now deletes
  `~/.cache/ros2/<distro>-env.sh` whenever the snippet task reports changed.
- **`--check` output is not a preview of the file.** Check mode applies
  nothing, so a task that depends on an earlier task's effect sees the *old*
  state. The first dry run here showed a second `~/.profile` block being added,
  because the legacy-block removal ahead of it had not actually removed
  anything. The real apply removed, then added, leaving one. Read a check diff
  as "what this task would do given the state it sees".
- **Reachability is a prerequisite, not a task.** Every scripted SSH to the Pi
  in this repo carries `-o BatchMode=yes -o ConnectTimeout=5`; a bare ssh hangs
  ~2 minutes against a dead Wi-Fi link. Run `ansible robot -m ping` first.

## Running it

```bash
just provision-check    # ansible-playbook site.yml --check --diff  — read the diff
just provision          # apply
just gate-provision     # the P9 gate: applied twice, second run changed=0
```

The raw equivalents, so the recipes are not a black box:

```bash
cd ansible
ansible robot -m ping                       # reachability first
ansible-playbook site.yml --check --diff    # dry run
ansible-playbook site.yml                   # apply — sudo is passwordless on the Pi
ansible-playbook site.yml --tags env        # just re-sync the ROS environment
```

**Read the `--check --diff` output before the first real run**, particularly for
the `blockinfile` and template tasks — that is where you see exactly what will
change in `~/.profile`.

A role you cannot describe in shell is a role you cannot debug. The plain-`apt`
equivalent of what these roles install is kept visible in
[setup.md](setup.md#raspberry-pi).

## What stays manual

Automating these costs more than it saves:

- **Flashing the SD card and first boot.** Ansible needs SSH to exist first.
- **Camera calibration.** It is interactive by design — you hold a checkerboard
  in front of the lens.
- **The dev box's GPU stack.** The ONNX Runtime tarball and any CUDA toolkit
  install are dev-box-only, one-off, and version-sensitive; they are documented
  in [setup.md](setup.md#gpu) and are P4's problem, not a role.
- **The learning.** Ansible provisions the environment. The pipeline is the
  point of the project.
