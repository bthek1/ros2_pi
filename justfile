# pimesh — day-to-day commands.
#
# Add a recipe rather than documenting a long one-off command, and keep the
# recipes and the docs in agreement. Every phase of the bootstrap plan brings
# its own `gate-*` recipe: a command that exits 0 or non-zero and prints the
# number it asserted on.

# No `-u`: ROS's own setup.bash reads unbound variables, so nounset turns
# `source /opt/ros/.../setup.bash` into an error before anything runs.
set shell := ["bash", "-c"]

ws       := justfile_directory()
ros      := "/opt/ros/lyrical"
pi_ros   := "/opt/ros/jazzy"
pi_host  := "pi"
pi_ws    := "~/ros2_pi"
# What the Pi builds. It is a sensor head: this list grows by exactly one
# package (pimesh_camera, at P1) and then stops.
pi_pkgs  := "pimesh_msgs pimesh_camera"
# The camera's serial-keyed symlink — stable across replugs, unlike /dev/video0.
# Same value as ansible/group_vars/robot.yml's camera_device; the gate checks it.
camera_by_id := "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0"
# A bare ssh hangs ~2 minutes against a dead Wi-Fi link and wedges whatever
# trap it sits in. Never drop these two options.
ssh_opts := "-o BatchMode=yes -o ConnectTimeout=5"

# List the recipes.
default:
    @just --list

# ---------------------------------------------------------------- build ----

# colcon build on the dev box.
[group('build')]
build *args:
    #!/usr/bin/env bash
    set -eo pipefail
    # rosidl generates message code with PYTHON, so an interface package is NOT
    # immune to this box's `python3` being PlatformIO's venv — it fails with
    # `No module named 'em'`. Put the system interpreter first.
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    cd "{{ws}}"
    # The compiled packages export a compile database (see their CMakeLists);
    # colcon writes one per package, so they are merged into
    # build/compile_commands.json, which .vscode/c_cpp_properties.json reads.
    # Without it an editor guesses include paths and gets every ROS header wrong.
    #
    # RelWithDebInfo is NOT optional in this project. colcon's default build
    # type is EMPTY, which means no -O flag at all: every package compiles at
    # -O0. Measured 2026-09-07 while building P3 — the rotation estimator ran
    # 1.19 ms/frame at -O0 and 0.03 ms/frame optimised, a 40x difference, and
    # OpenCV's own cost did not move because that code is already optimised
    # inside libopencv. So the effect is invisible until you write numerics of
    # your own, and P5's TSDF and P6's marching cubes are exactly that.
    # WithDebInfo rather than plain Release so a crash still has a backtrace.
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo {{args}}
    python3 "{{ws}}/tools/merge_compile_commands.py" "{{ws}}/build"

# Never the build products — there is no ABI compatibility between Lyrical and
# Jazzy, so only source crosses.
#
# Copy the source tree to the Pi.
[group('build')]
sync-pi:
    #!/usr/bin/env bash
    set -eo pipefail
    rsync -a --delete \
        --exclude build --exclude install --exclude log \
        --exclude models --exclude bags --exclude meshes --exclude .git \
        -e "ssh {{ssh_opts}}" \
        "{{ws}}/" "{{pi_host}}:{{pi_ws}}/"
    echo "synced → {{pi_host}}:{{pi_ws}}"

# Build the Pi's two packages, on the Pi, from source.
[group('build')]
build-pi: sync-pi
    #!/usr/bin/env bash
    set -eo pipefail
    ssh {{ssh_opts}} {{pi_host}} "bash -lc '
        source {{pi_ros}}/setup.bash
        cd {{pi_ws}}
        colcon build --symlink-install --packages-select {{pi_pkgs}} \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -5
    '"

# A thin pointer at /usr/bin/python3, not a dependency sandbox: built with
# --system-site-packages, so it sees the system pytest and installs nothing.
# It exists because the editor needs ONE deterministic interpreter — this box
# has three python3.14s and only /usr/bin/python3 works for us. Idempotent.
#
# Create .venv, the interpreter the editor and `just test` both use.
[group('test')]
venv:
    #!/usr/bin/env bash
    set -eo pipefail
    if [ -x "{{ws}}/.venv/bin/python" ]; then
        echo ".venv exists → $("{{ws}}/.venv/bin/python" -c 'import sys; print(sys.base_prefix)')"
    else
        /usr/bin/python3 -m venv --system-site-packages "{{ws}}/.venv"
        echo ".venv created from /usr/bin/python3"
    fi
    "{{ws}}/.venv/bin/python" -c 'import pytest, sys; print(f"pytest {pytest.__version__} via {sys.executable}")'

# Two suites: the packages' own gtest via colcon, and pytest for the gate tools
# in tools/, which are not a ROS package and so are invisible to colcon.
#
# Run every test on the dev box.
[group('test')]
test *args:
    #!/usr/bin/env bash
    set -eo pipefail
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    cd "{{ws}}"
    rc=0
    echo "── colcon ──"
    colcon test {{args}} || rc=1
    colcon test-result || rc=1
    echo "── pytest (tools/) ──"
    # The same interpreter the editor's Testing sidebar uses, so a green
    # sidebar and a green `just test` cannot disagree. Falls back to the system
    # one when .venv is absent (a fresh clone), which is the same interpreter
    # the venv points at anyway.
    py="{{ws}}/.venv/bin/python"
    [ -x "${py}" ] || py=/usr/bin/python3
    echo "   interpreter: ${py}"
    "${py}" -m pytest tools/ -q || rc=1
    exit "${rc}"

# A test that has only ever run on Lyrical says nothing about the machine that
# actually runs the camera.
#
# Run the same tests on the Pi, under Jazzy.
[group('test')]
test-pi: build-pi
    #!/usr/bin/env bash
    set -eo pipefail
    ssh {{ssh_opts}} {{pi_host}} "bash -lc '
        source {{pi_ros}}/setup.bash
        cd {{pi_ws}}
        colcon test --packages-select {{pi_pkgs}} 2>&1 | tail -3
        colcon test-result
    '"

# ----------------------------------------------------------------- run -----

# The dev-box container (P2 onwards). Tears itself down on Ctrl-C.
[group('run')]
pipeline *args:
    #!/usr/bin/env bash
    set -eo pipefail
    trap 'pkill -f component_container_mt >/dev/null 2>&1 || true' EXIT
    source "{{ros}}/setup.bash"
    source "{{ws}}/install/setup.bash"
    ros2 launch pimesh_bringup pimesh.launch.py {{args}}

# A leaked camera process holds /dev/video0 exclusively and every later session
# then dies with "Device or resource busy".
#
# Sweep both machines for nodes an earlier session left running.
[group('run')]
stragglers:
    #!/usr/bin/env bash
    set -o pipefail
    patterns='component_container_mt|pimesh_|static_transform_publisher'
    local_hits=$(pgrep -af "${patterns}" | grep -v 'just stragglers' || true)
    if [ -z "${local_hits}" ]; then echo "dev box: clean"; else
        echo "dev box: STRAGGLERS"; echo "${local_hits}"; fi
    pi_hits=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -af '${patterns}'" 2>/dev/null || true)
    if [ -z "${pi_hits}" ]; then echo "pi:      clean"; else
        echo "pi:      STRAGGLERS"; echo "${pi_hits}"; fi

# ---------------------------------------------------------------- gates ----

# Asserts: both builds succeed, all five interface definitions are byte-identical
# across the two distros, and camera_link → camera_optical_frame resolves.
#
# P0 gate — the same source builds under two ROS distros.
[group('gate')]
gate-build:
    #!/usr/bin/env bash
    set -o pipefail
    fail=0
    out=$(mktemp -d)
    # Teardown: the launch is BOUNDED UP FRONT with `timeout -s INT` rather than
    # killed by a trap. SIGINT to `ros2 launch` shuts its children down in
    # order; killing the wrapper instead orphans the static_transform_publisher
    # grandchildren, which then hold the frame tree — measured on this gate's
    # own first run. The trap is only the temp-dir cleanup and a crash backstop.
    # The trap only cleans the temp dir. It deliberately does NOT pkill: a
    # pattern broad enough to catch the launch also matches any shell whose
    # command line merely mentions it — including the one running this gate,
    # which killed itself that way once. If the teardown assertion below fails,
    # the gate says so and `just stragglers` is the sweep.
    trap 'rm -rf "${out}"' EXIT

    echo "== P0 gate: cross-distro build =="

    echo "-- dev box ($(basename {{ros}})) --"
    t0=$(date +%s)
    if ! just build > "${out}/dev.log" 2>&1; then
        echo "FAIL: dev-box build"; tail -20 "${out}/dev.log"; fail=1
    fi
    dev_secs=$(( $(date +%s) - t0 ))

    echo "-- pi ($(basename {{pi_ros}})) --"
    t0=$(date +%s)
    if ! just build-pi > "${out}/pi.log" 2>&1; then
        echo "FAIL: pi build"; tail -20 "${out}/pi.log"; fail=1
    fi
    pi_secs=$(( $(date +%s) - t0 ))

    echo "-- interface parity --"
    source "{{ros}}/setup.bash"; source "{{ws}}/install/setup.bash"
    for iface in pimesh_msgs/msg/Keypoints pimesh_msgs/msg/PipelineStats \
                 pimesh_msgs/msg/MeshStats pimesh_msgs/srv/SaveMesh \
                 pimesh_msgs/srv/ResetMap; do
        name=$(echo "${iface}" | tr '/' '_')
        ros2 interface show "${iface}" > "${out}/${name}.dev" 2>&1
        ssh {{ssh_opts}} {{pi_host}} "bash -lc '
            source {{pi_ros}}/setup.bash
            source {{pi_ws}}/install/setup.bash
            ros2 interface show ${iface}'" > "${out}/${name}.pi" 2>&1
        if diff -q "${out}/${name}.dev" "${out}/${name}.pi" >/dev/null; then
            echo "  ok    ${iface}"
        else
            echo "  DIFF  ${iface}"; diff "${out}/${name}.dev" "${out}/${name}.pi" | head -20; fail=1
        fi
    done

    echo "-- frame tree --"
    # The launch runs in the FOREGROUND under `timeout -s INT`, with the probe
    # in the background. A shell without job control sets SIGINT to SIG_IGN for
    # background children, which can leave the launch un-interruptible and its
    # static_transform_publishers orphaned — measured on this gate.
    ( sleep 6
      timeout 8 ros2 run tf2_ros tf2_echo camera_link camera_optical_frame \
          > "${out}/tf.log" 2>&1 || true ) &
    probe_pid=$!
    # -k, always, on a `ros2 launch`: its SIGINT handler has a race that leaves
    # it hung forever roughly one run in three (measured 2026-09-04 while
    # building the P2 gate). Without the backstop this line can hang the gate.
    timeout -s INT -k 15 18 ros2 launch pimesh_bringup pimesh.launch.py \
        > "${out}/launch.log" 2>&1 || true
    wait "${probe_pid}" 2>/dev/null || true
    if grep -q 'Translation' "${out}/tf.log"; then
        echo "  ok    camera_link → camera_optical_frame"
        grep -m1 -A2 'Rotation: in Quaternion' "${out}/tf.log" | sed 's/^/        /'
    else
        echo "  FAIL  camera_link → camera_optical_frame not resolved"
        tail -10 "${out}/launch.log"; fail=1
    fi
    if grep -q 'component_container_mt' "${out}/launch.log"; then
        echo "  ok    pimesh_container up (decode_node since P2)"
    else
        echo "  FAIL  container did not start"; fail=1
    fi
    # The session must tear itself down: SIGINT at the timeout, then nothing
    # left holding a frame.
    sleep 1
    if pgrep -f '__node:=(map_to_odom_identity|base_to_camera|camera_to_optical)' >/dev/null; then
        echo "  FAIL  launch left stragglers behind"; fail=1
    else
        echo "  ok    session tore itself down"
    fi

    echo
    echo "dev-box build: ${dev_secs}s   pi build: ${pi_secs}s"
    if [ "${fail}" -eq 0 ]; then echo "P0 GATE PASS"; else echo "P0 GATE FAIL"; fi
    exit "${fail}"

# ----------------------------------------------------------- provisioning ---

# Read the diff before the first real apply — the blockinfile and template
# tasks are where you see exactly what changes in ~/.profile.
#
# Dry-run the playbook against the Pi.
[group('provision')]
provision-check *args:
    #!/usr/bin/env bash
    set -eo pipefail
    cd "{{ws}}/ansible"
    ansible-playbook site.yml --check --diff {{args}}

# Apply the playbook to the Pi. sudo is passwordless there.
[group('provision')]
provision *args:
    #!/usr/bin/env bash
    set -eo pipefail
    cd "{{ws}}/ansible"
    ansible-playbook site.yml {{args}}

# Asserts: the playbook is idempotent (second apply changed=0), the Pi's ROS
# environment EQUALS the dev box's, the build dependencies are present, the
# camera is where group_vars says, exactly one managed block owns ~/.profile,
# and the cross-distro build still passes.
#
# P9 gate — the Pi's configuration is re-assertable and matches this machine.
[group('gate')]
gate-provision:
    #!/usr/bin/env bash
    set -o pipefail
    fail=0
    out=$(mktemp -d)
    trap 'rm -rf "${out}"' EXIT
    cd "{{ws}}/ansible"

    echo "== P9 gate: provisioning =="

    echo "-- reachability --"
    if ansible robot -m ping > "${out}/ping.log" 2>&1; then
        echo "  ok    ansible robot -m ping"
    else
        echo "  FAIL  the Pi is not reachable"; tail -5 "${out}/ping.log"
        echo "P9 GATE FAIL"; exit 1
    fi

    echo "-- idempotence --"
    for run in 1 2; do
        ansible-playbook site.yml > "${out}/apply${run}.log" 2>&1 || true
        recap=$(grep -E '^pi +:' "${out}/apply${run}.log" | tail -1)
        eval "changed${run}=$(echo "${recap}" | sed -n 's/.*changed=\([0-9]*\).*/\1/p')"
        eval "failed${run}=$(echo "${recap}" | sed -n 's/.*failed=\([0-9]*\).*/\1/p')"
        eval "unreach${run}=$(echo "${recap}" | sed -n 's/.*unreachable=\([0-9]*\).*/\1/p')"
    done
    echo "  run 1: changed=${changed1} failed=${failed1} unreachable=${unreach1}"
    echo "  run 2: changed=${changed2} failed=${failed2} unreachable=${unreach2}"
    if [ "${failed1:-1}" -ne 0 ] || [ "${failed2:-1}" -ne 0 ] || \
       [ "${unreach1:-1}" -ne 0 ] || [ "${unreach2:-1}" -ne 0 ]; then
        echo "  FAIL  a task failed — see the log above"
        grep -B2 -A8 'fatal:' "${out}/apply2.log" | head -40; fail=1
    fi
    if [ "${changed2:-1}" -eq 0 ]; then
        echo "  ok    second apply changed nothing — the playbook is idempotent"
    else
        echo "  FAIL  second apply reported changed=${changed2}; idempotence is the whole claim"
        grep -E '^changed:' "${out}/apply2.log" | head -10; fail=1
    fi

    echo "-- environment parity --"
    # The dev box's own values, from a login shell — the same source the Pi's
    # come from. Equality is the assertion: a gate that only checked the Pi
    # would pass while this machine drifted, which is the failure that produces
    # silence instead of an error.
    for var in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; do
        dev=$(bash -lc "echo \${${var}:-UNSET}" 2>/dev/null | tail -1)
        pi=$(ssh {{ssh_opts}} {{pi_host}} "bash -lc 'echo \${${var}:-UNSET}'" 2>/dev/null | tail -1)
        printf "  %-20s dev=%-22s pi=%s\n" "${var}" "${dev}" "${pi}"
        if [ "${dev}" != "${pi}" ] || [ "${dev}" = "UNSET" ]; then
            echo "  FAIL  ${var} differs between the machines (or is unset)"; fail=1
        fi
    done

    echo "-- the Pi's own state --"
    ssh {{ssh_opts}} {{pi_host}} "bash -lc '
        printf \"dds_iface=%s\n\" \"\$(grep -o \"NetworkInterface name=\\\"[a-z0-9]*\\\"\" ~/.config/cyclonedds/cyclonedds.xml | head -1 | sed \"s/.*name=\\\"//;s/\\\"//\")\"
        printf \"profile_blocks=%s\n\" \"\$(grep -c \"BEGIN ANSIBLE MANAGED — ROS 2 environment\" ~/.profile)\"
        printf \"videodev2=%s\n\" \"\$([ -f /usr/include/linux/videodev2.h ] && echo yes || echo no)\"
        printf \"camera=%s\n\" \"\$([ -e {{camera_by_id}} ] && readlink -f {{camera_by_id}} || echo missing)\"
        printf \"gpp=%s\n\" \"\$(g++ -dumpfullversion)\"
        for p in v4l-utils build-essential cmake ros-jazzy-rclcpp-components ros-jazzy-image-transport ros-jazzy-camera-info-manager; do
            printf \"pkg:%s=%s\n\" \"\$p\" \"\$(dpkg-query -W -f=\"\\\${db:Status-Status}\" \$p 2>/dev/null || echo absent)\"
        done
    '" > "${out}/state.txt" 2>/dev/null
    get() { sed -n "s/^$1=//p" "${out}/state.txt" | tail -1; }

    if [ "$(get dds_iface)" = "wlan0" ]; then echo "  ok    cyclonedds.xml pins wlan0"
    else echo "  FAIL  cyclonedds.xml pins '$(get dds_iface)', expected wlan0"; fail=1; fi

    if [ "$(get profile_blocks)" = "1" ]; then echo "  ok    exactly one managed ROS block in ~/.profile"
    else echo "  FAIL  ~/.profile has $(get profile_blocks) managed ROS blocks — two means another tree still owns this host"; fail=1; fi

    if [ "$(get videodev2)" = "yes" ]; then echo "  ok    linux/videodev2.h present"
    else echo "  FAIL  linux/videodev2.h missing — pimesh_camera cannot compile"; fail=1; fi

    cam=$(get camera)
    if [ "${cam}" != "missing" ]; then echo "  ok    camera by-id → ${cam}"
    else echo "  FAIL  the C922 by-id symlink does not resolve"; fail=1; fi

    while IFS= read -r line; do
        pkg=${line#pkg:}; name=${pkg%%=*}; status=${pkg#*=}
        if [ "${status}" = "installed" ]; then echo "  ok    ${name}"
        else echo "  FAIL  ${name} is ${status}"; fail=1; fi
    done < <(grep '^pkg:' "${out}/state.txt")

    echo "-- the build still works --"
    if just gate-build > "${out}/build.log" 2>&1; then
        echo "  ok    just gate-build PASS"
    else
        echo "  FAIL  just gate-build failed after provisioning"
        tail -20 "${out}/build.log"; fail=1
    fi

    echo
    echo "g++ on the Pi: $(get gpp)   apply: changed ${changed1} → ${changed2}"
    if [ "${fail}" -eq 0 ]; then echo "P9 GATE PASS"; else echo "P9 GATE FAIL"; fi
    exit "${fail}"

# ---------------------------------------------------------------- camera ----

# Camera state PERSISTS inside the camera across processes and reboots, so this
# is machine state you inspect before blaming software for a black image.
#
# Print every V4L2 control, current vs default.
[group('camera')]
camera:
    #!/usr/bin/env bash
    set -eo pipefail
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'v4l2-ctl -d {{camera_by_id}} --list-ctrls; echo; v4l2-ctl -d {{camera_by_id}} --get-parm'"

# Restore the C922 to a known-good baseline: auto exposure on, the dynamic
# frame-rate thief off, auto focus and white balance back, the rest at default.
[group('camera')]
camera-reset:
    #!/usr/bin/env bash
    set -eo pipefail
    ssh {{ssh_opts}} {{pi_host}} "bash -lc '
        for c in auto_exposure=3 exposure_dynamic_framerate=0 \
                 focus_automatic_continuous=1 white_balance_automatic=1 \
                 gain=0 brightness=128 contrast=128 saturation=128 \
                 sharpness=128 zoom_absolute=100 pan_absolute=0 tilt_absolute=0; do
            v4l2-ctl -d {{camera_by_id}} --set-ctrl=\$c
        done
        echo camera baseline restored
        v4l2-ctl -d {{camera_by_id}} --get-ctrl=auto_exposure,exposure_time_absolute,exposure_dynamic_framerate
    '"

# `just cam 30` bounds it at 30 s; with no argument it runs until Ctrl-C.
# Either way it cannot outlive the session.
#
# Run the Pi's camera in the foreground.
[group('run')]
cam seconds='0' *args:
    #!/usr/bin/env bash
    set -eo pipefail
    bound=""
    # -k: `ros2 launch` can wedge in its own SIGINT handler (see gate-ipc), and
    # a wedged launch on the Pi holds /dev/video0 against every later session.
    if [ "{{seconds}}" != "0" ]; then bound="timeout -s INT -k 10 {{seconds}}"; fi
    ssh {{ssh_opts}} -tt {{pi_host}} "bash -lc '${bound} ros2 launch pimesh_camera camera.launch.py {{args}}'"

# The bag every later phase replays, so its numbers compare like for like.
#
# Records exactly what the CAMERA produced — the compressed stream and the
# intrinsics — and nothing the pipeline derived from it. A bag holding
# keypoints or depth would freeze one version of the algorithms into the
# fixture that is supposed to be judging them.
#
# The Pi camera is started here and bounded, so the recording cannot outlive
# the recipe.
#
# Record the camera's stream to bags/<name>, e.g. `just record desk1 60`.
[group('run')]
record name seconds='60':
    #!/usr/bin/env bash
    set -eo pipefail
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    source "{{ws}}/install/setup.bash"

    dest="{{ws}}/bags/{{name}}"
    if [ -e "${dest}" ]; then
        echo "bags/{{name}} already exists — refusing to overwrite a fixture"
        echo "every phase from P3 on compares against it; move it aside first"
        exit 1
    fi

    before=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    if [ "${before}" != "0" ]; then
        echo "${before} camera process(es) already on the Pi — run 'just stragglers'"
        exit 1
    fi

    # The camera outlives the recording by a margin at each end: starting it
    # first means the bag opens on a settled auto-exposure rather than on the
    # first few black frames, and ending it after means the bag is closed
    # before its publisher disappears.
    warmup=6
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT -k 10 $(({{seconds}} + 12)) ros2 launch pimesh_camera camera.launch.py > /tmp/record_{{name}}.log 2>&1'" &
    cam_pid=$!
    sleep "${warmup}"

    echo "recording {{seconds}} s to bags/{{name}} — SWEEP THE ROOM, do not leave it on the desk"
    mkdir -p "{{ws}}/bags"
    # `timeout -s INT` is what ENDS the recording. rosbag2's -d is the bag-SPLIT
    # duration, not a stop-after, and Lyrical has no --duration at all. SIGINT
    # rather than SIGTERM so rosbag2 closes the bag and writes metadata.yaml —
    # a bag killed hard replays as nothing.
    #
    # --topics, not positional arguments: `ros2 bag record <topic>` is gone in
    # Lyrical and fails with "unrecognized arguments" — AFTER the camera has
    # started and the person holding it has begun sweeping. Wasted take.
    #
    # --max-cache-size 0 writes straight through instead of buffering. At
    # ~9 MB/s that costs nothing and means a rough shutdown loses no frames.
    timeout -s INT -k 10 $(({{seconds}} + 3)) \
        ros2 bag record -o "${dest}" \
        --topics /image_raw/compressed /camera_info \
        --max-cache-size 0 || true

    wait "${cam_pid}" 2>/dev/null || true
    sleep 1

    echo "-- what landed in the bag --"
    ros2 bag info "${dest}"
    left=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    if [ "${left}" != "0" ]; then
        echo "WARNING: ${left} camera process(es) still on the Pi"
        exit 1
    fi

# Asserts: the keypoint stage keeps up with what decode hands it, one frame
# costs what it was budgeted, the matching finds about what the predecessor's
# did, and the stage gets through most of the stream. The pose-gate reject rate
# is PRINTED, not asserted — see tools/check_keypoints.py on both points.
#
# P3 gate — ORB keypoints, replayed off bags/desk1.
[group('gate')]
gate-keypoints bag='desk1':
    #!/usr/bin/env bash
    set -o pipefail
    fail=0
    out=$(mktemp -d)
    trap 'rm -rf "${out}"' EXIT
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    source "{{ws}}/install/setup.bash"

    echo "== P3 gate: keypoints =="

    # A bag, not the live camera. The point of the fixture is that P4 through
    # P8 measure themselves against the SAME frames — a live run would compare
    # today's room and today's light against last week's.
    bag="{{ws}}/bags/{{bag}}"
    if [ ! -d "${bag}" ]; then
        echo "  FAIL  bags/{{bag}} does not exist — record it with 'just record {{bag}} 60'"
        echo "P3 GATE FAIL"; exit 1
    fi
    echo "-- fixture --"
    ros2 bag info "${bag}" | sed -n '1,12p' | sed 's/^/  /'

    # The launch in the FOREGROUND under `timeout -s INT`, the replay and the
    # probe backgrounded: a shell without job control sets SIGINT to SIG_IGN
    # for background children, so a backgrounded launch is un-interruptible and
    # orphans its components. Measured on P0's gate.
    ( sleep 5
      # NOT `--once`. /pipeline/stats is one topic shared by every stage, keyed
      # by `stage`; `--once` returns whichever node published first, and
      # check_keypoints.py selects the keypoints record out of the stream.
      #
      # --full-length is REQUIRED, not cosmetic. `ros2 topic echo` silently
      # elides any string longer than 128 characters with a trailing "...",
      # and `detail` carries the pose-gate reject breakdown past that mark. The
      # gate printed "uncalibrated ?" for a run in which that number was
      # present all along — a truncated field looks exactly like a missing one.
      timeout -s INT 26 ros2 topic echo --full-length /pipeline/stats \
          > "${out}/stats.txt" 2>&1 || true ) &
    probe_pid=$!
    ( sleep 4
      # No --loop and no --rate: the clip is replayed once at the rate it was
      # captured, because the budget being judged is per-frame cost at the
      # camera's real rate.
      timeout -s INT -k 10 30 ros2 bag play "${bag}" > "${out}/play.log" 2>&1 || true ) &
    play_pid=$!

    # -k is required, not belt-and-braces: `ros2 launch` has a SIGINT race that
    # hangs it forever roughly one run in three, and `timeout` without a
    # backstop then waits forever too.
    timeout -s INT -k 15 40 ros2 launch pimesh_bringup pimesh.launch.py \
        > "${out}/run.log" 2>&1
    rc=$?
    wait "${probe_pid}" 2>/dev/null || true
    wait "${play_pid}" 2>/dev/null || true

    echo "-- assertions --"
    python3 "{{ws}}/tools/check_keypoints.py" --stats "${out}/stats.txt" || fail=1

    echo "-- teardown --"
    case "${rc}" in
      124) echo "  ok    the launch shut down on SIGINT" ;;
      137) echo "  warn  the launch needed the SIGKILL backstop — ros2 launch's"\
                " event-loop race, not the container" ;;
      *)   echo "  info  the launch exited ${rc}" ;;
    esac
    sleep 2
    left=$(pgrep -f '[c]omponent_container_mt' | wc -l | tr -d ' ')
    if [ "${left}" = "0" ]; then
        echo "  ok    dev box clean"
    else
        echo "  FAIL  ${left} container(s) still running"; fail=1
    fi

    echo
    if [ "${fail}" -eq 0 ]; then echo "P3 GATE PASS"; else echo "P3 GATE FAIL"; fi
    exit "${fail}"

# Asserts: the node loses nothing against raw v4l2 on the same camera, the
# stamp-to-receipt offset is under one frame interval AND stable across two
# separate launches (exactly what usb_cam 0.8.1 fails), the driver gives capture
# timestamps, and a camera it cannot open makes it exit non-zero instead of idle.
#
# P1 gate — capture, with honest timestamps.
[group('gate')]
gate-capture:
    #!/usr/bin/env bash
    set -o pipefail
    fail=0
    out=$(mktemp -d)
    trap 'rm -rf "${out}"' EXIT
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    source "{{ws}}/install/setup.bash"
    echo "== P1 gate: capture =="
    # A leaked camera process from an earlier session holds /dev/video0 and
    # would make every number below meaningless, so refuse to measure at all.
    before=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    if [ "${before}" != "0" ]; then
        echo "  FAIL  ${before} camera process(es) already running on the Pi — run 'just stragglers'"
        echo "P1 GATE FAIL"; exit 1
    fi
    echo "-- camera baseline --"
    just camera-reset > "${out}/reset.log" 2>&1 || true
    exposure=$(grep -E '^auto_exposure' "${out}/reset.log" | head -1)
    echo "  ${exposure:-auto_exposure: unknown}"
    echo "-- hardware ceiling (raw v4l2, no ROS in the loop) --"
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout 25 v4l2-ctl -d {{camera_by_id}} --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=60 --stream-mmap --stream-count=200 --stream-to=/dev/null 2>&1 | tail -3'" > "${out}/raw.txt" 2>&1
    # v4l2-ctl's LAST line is "Frame rate set to 60.000 fps" — the request
    # echoed back, not a measurement. The measured rate is on the progress
    # lines, which start with '<'. Taking the last match here silently compared
    # the node against the number we asked for, and called a working node a 50%
    # loss.
    hw_fps=$(grep '<' "${out}/raw.txt" | grep -oE '[0-9]+\.[0-9]+ fps' | tail -1 | cut -d' ' -f1)
    echo "  raw v4l2: ${hw_fps:-unknown} fps at 1280x720 MJPG, 60 requested"
    for run in 1 2; do
        echo "-- launch ${run} --"
        # Let the previous run's DDS discovery age out; two launches racing
        # each other's teardown is not what this gate is measuring.
        sleep 3
        ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT -k 10 45 ros2 launch pimesh_camera camera.launch.py > /tmp/cam${run}.log 2>&1'" &
        launch_pid=$!
        sleep 8
        # Stats first: one message, cheap, and running it last left it racing
        # the launch's own timeout and coming back empty.
        timeout -s INT 6 ros2 topic echo /pipeline/stats --once > "${out}/stats${run}.txt" 2>&1 || true
        # The stamp measurement that matters is taken ON THE PI, where the
        # publisher and the subscriber share a clock. Measured from here it
        # would carry the offset between two machines' clocks as well as the
        # frame, and that offset moves on its own — see tools/check_capture.py.
        ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT 8 ros2 topic delay /image_raw/compressed 2>&1 | tail -3'" > "${out}/pidelay${run}.txt" 2>&1 || true
        timeout -s INT 12 ros2 topic hz /image_raw/compressed > "${out}/hz${run}.txt" 2>&1 || true
        timeout -s INT 6 ros2 topic delay /image_raw/compressed > "${out}/delay${run}.txt" 2>&1 || true
        wait "${launch_pid}" 2>/dev/null || true
        eval "rate${run}=$(grep -oE 'average rate: [0-9.]+' "${out}/hz${run}.txt" | tail -1 | cut -d' ' -f3)"
        eval "delay${run}=$(grep -oE 'average delay: [0-9.]+' "${out}/delay${run}.txt" | tail -1 | cut -d' ' -f3)"
        eval "pidelay${run}=$(grep -oE 'average delay: [0-9.]+' "${out}/pidelay${run}.txt" | tail -1 | cut -d' ' -f3)"
        # The node's OWN capture rate, counted on the Pi. `ros2 topic hz` here
        # counts frames that arrived, so it charges the node for Wi-Fi loss —
        # it read 5.3% loss against the hardware for a node that was actually
        # losing 2.4%.
        eval "noderate${run}=$(grep -oE 'rate_hz: [0-9.]+' "${out}/stats${run}.txt" | tail -1 | cut -d' ' -f2 | xargs printf '%.2f' 2>/dev/null)"
        eval "echo \"  captured=\${noderate${run}:-none} Hz  delivered=\${rate${run}:-none} Hz  on-pi=\${pidelay${run}:-none} s  dev-box=\${delay${run}:-none} s\""
    done
    src=$(grep -oE 'detail: .*' "${out}/stats1.txt" | tail -1 | cut -d' ' -f2-)
    echo "-- assertions --"
    python3 "{{ws}}/tools/check_capture.py" \
        --hw-fps "${hw_fps:-0}" \
        --node-rate "${noderate1:-0}" "${noderate2:-0}" \
        --delivered-rate "${rate1:-0}" "${rate2:-0}" \
        --pi-delay "${pidelay1:--1}" "${pidelay2:--1}" \
        --dev-delay "${delay1:--1}" "${delay2:--1}" || fail=1
    echo "  timestamp source: ${src:-unknown}"
    case "${src}" in
      *CLOCK_MONOTONIC*) echo "  ok    stamps are capture times, not arrival times" ;;
      *) echo "  FAIL  driver is not giving capture timestamps"; fail=1 ;;
    esac
    echo "-- fails loudly --"
    t0=$(date +%s)
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT 10 ros2 run pimesh_camera camera_node --ros-args -p device:=/dev/video99 > /tmp/missing.log 2>&1'"
    rc=$?; took=$(( $(date +%s) - t0 ))
    if [ "${rc}" -ne 0 ] && [ "${took}" -le 3 ]; then
        echo "  ok    missing device → exit ${rc} in ${took}s"
    else
        echo "  FAIL  missing device → exit ${rc} after ${took}s (want non-zero within 2-3 s)"; fail=1
    fi
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT -k 10 20 ros2 launch pimesh_camera camera.launch.py > /tmp/holder.log 2>&1'" &
    holder=$!
    sleep 7
    t0=$(date +%s)
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT 8 ros2 run pimesh_camera camera_node > /tmp/busy.log 2>&1'"
    rc=$?; took=$(( $(date +%s) - t0 ))
    busy_msg=$(ssh {{ssh_opts}} {{pi_host}} "grep -ohE 'cannot start: .*' /tmp/busy.log | head -1" 2>/dev/null)
    wait "${holder}" 2>/dev/null || true
    if [ "${rc}" -ne 0 ] && [ "${took}" -le 3 ]; then
        echo "  ok    busy device → exit ${rc} in ${took}s"
        if [ -n "${busy_msg}" ]; then echo "        ${busy_msg}"; fi
    else
        echo "  FAIL  busy device → exit ${rc} after ${took}s"; fail=1
    fi
    echo "-- teardown --"
    sleep 2
    # '[c]amera_node', not 'camera_node': pgrep -f matches the command line of
    # the shell running it, and that shell's command line contains the pattern.
    # Without the bracket this reports a straggler that is its own query — it
    # did, and cost a gate run.
    left=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    if [ "${left}" = "0" ]; then
        echo "  ok    no camera process left on the Pi"
    else
        echo "  FAIL  ${left} camera process(es) still holding /dev/video0"; fail=1
    fi
    echo
    echo "hardware ${hw_fps:-?} Hz | captured ${noderate1:-?} / ${noderate2:-?} Hz | delivered ${rate1:-?} / ${rate2:-?} Hz | on-pi offset ${pidelay1:-?} / ${pidelay2:-?} s"
    if [ "${fail}" -eq 0 ]; then echo "P1 GATE PASS"; else echo "P1 GATE FAIL"; fi
    exit "${fail}"

# Asserts: every frame decode_node publishes arrives at the SAME address in the
# probe, the same run with intra-process OFF produces DIFFERENT addresses (so
# the check is capable of failing), exactly one subscriber reads the Pi's
# stream, and decode keeps up.
#
# P2 gate — the container is really zero-copy.
[group('gate')]
gate-ipc:
    #!/usr/bin/env bash
    set -o pipefail
    fail=0
    out=$(mktemp -d)
    trap 'rm -rf "${out}"' EXIT
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    source "{{ws}}/install/setup.bash"

    echo "== P2 gate: intra-process =="

    # A leaked camera from an earlier session holds /dev/video0 exclusively and
    # every number below would be measured against nothing.
    before=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    if [ "${before}" != "0" ]; then
        echo "  FAIL  ${before} camera process(es) already on the Pi — run 'just stragglers'"
        echo "P2 GATE FAIL"; exit 1
    fi

    # The decode stage needs real frames, so this is a two-machine gate. The Pi
    # camera is bounded up front and covers both container runs.
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT -k 10 70 ros2 launch pimesh_camera camera.launch.py > /tmp/gate_ipc_cam.log 2>&1'" &
    cam_pid=$!
    sleep 8

    # Run 1: the claim. Probe on, intra-process on.
    #
    # The launch runs in the FOREGROUND under `timeout -s INT` with the probes
    # backgrounded — a shell without job control sets SIGINT to SIG_IGN for
    # background children, which leaves a backgrounded launch un-interruptible
    # and its components orphaned. Measured on P0's gate.
    echo "-- run 1: intra_process:=true --"
    ( sleep 9
      # `topic info` does NOT create a subscriber; `topic hz` would, and would
      # then make this assertion fail by measuring it.
      timeout -s INT 6 ros2 topic info -v /image_raw/compressed > "${out}/info.txt" 2>&1 || true
      # NOT `--once`. /pipeline/stats is shared by every node, keyed by the
      # `stage` field, so `--once` returns whichever stage published first —
      # which is the camera, and this gate asserted the CAMERA's rate and
      # latency as decode's on its first run. Capture a few seconds of the
      # topic; check_ipc.py selects the decode record out of it.
      timeout -s INT 8 ros2 topic echo /pipeline/stats > "${out}/stats.txt" 2>&1 || true ) &
    probe_pid=$!
    # -k is not belt-and-braces, it is required. `ros2 launch` has a race in
    # its SIGINT handler: roughly one run in three it prints "This event loop
    # is already running", never signals its children, and hangs FOREVER —
    # measured 2026-09-04, 1 of 2 runs, same command. `timeout` without -k then
    # waits forever too, so the gate hangs rather than fails. The backstop
    # bounds it; which signal was needed is reported below.
    timeout -s INT -k 15 22 ros2 launch pimesh_bringup pimesh.launch.py \
        probe:=true > "${out}/on.log" 2>&1
    rc_on=$?
    wait "${probe_pid}" 2>/dev/null || true

    # Run 2: the control. Same everything, buffers serialised.
    echo "-- run 2: intra_process:=false (the control) --"
    sleep 3
    timeout -s INT -k 15 20 ros2 launch pimesh_bringup pimesh.launch.py \
        probe:=true intra_process:=false > "${out}/off.log" 2>&1
    rc_off=$?

    wait "${cam_pid}" 2>/dev/null || true

    subs=$(grep -c 'Endpoint type: SUBSCRIPTION' "${out}/info.txt" 2>/dev/null | tr -d ' ')
    stages=$(grep -oE '^stage: .*' "${out}/stats.txt" | sort -u | cut -d' ' -f2 | tr '\n' ' ')

    echo "-- assertions --"
    echo "  info  /pipeline/stats carried: ${stages:-nothing}"
    python3 "{{ws}}/tools/check_ipc.py" \
        --log-on "${out}/on.log" --log-off "${out}/off.log" \
        --subscribers "${subs:-0}" --stats "${out}/stats.txt" || fail=1

    echo "-- teardown --"
    # 124 = SIGINT was enough. 137 = the SIGKILL backstop had to fire, which
    # means `ros2 launch` wedged in the race described above. That is upstream,
    # not this pipeline, so it is REPORTED rather than failed — but the
    # straggler check below is a hard assertion either way.
    for pair in "run1:${rc_on}" "run2:${rc_off}"; do
        name=${pair%%:*}; rc=${pair#*:}
        case "${rc}" in
          124) echo "  ok    ${name} shut down on SIGINT" ;;
          137) echo "  warn  ${name} needed the SIGKILL backstop — ros2 launch's "\
                    "event-loop race, not the container" ;;
          *)   echo "  info  ${name} exited ${rc}" ;;
        esac
    done
    sleep 2
    left_pi=$(ssh {{ssh_opts}} {{pi_host}} "pgrep -f '[c]amera_node' | wc -l" 2>/dev/null | tr -d ' ')
    left_dev=$(pgrep -f '[c]omponent_container_mt' | wc -l | tr -d ' ')
    if [ "${left_pi}" = "0" ] && [ "${left_dev}" = "0" ]; then
        echo "  ok    both machines clean"
    else
        echo "  FAIL  ${left_dev} container(s) here, ${left_pi} camera(s) on the Pi"; fail=1
    fi

    echo
    echo "stages seen: ${stages:-none} | ${subs:-?} subscriber on the Pi's stream"
    if [ "${fail}" -eq 0 ]; then echo "P2 GATE PASS"; else echo "P2 GATE FAIL"; fi
    exit "${fail}"
