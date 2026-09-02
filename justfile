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
pi_pkgs  := "pimesh_msgs"
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
    colcon build --symlink-install {{args}}

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
        colcon build --symlink-install --packages-select {{pi_pkgs}} 2>&1 | tail -5
    '"

# colcon test on the dev box.
[group('test')]
test *args:
    #!/usr/bin/env bash
    set -eo pipefail
    export PATH="/usr/bin:${PATH}"
    source "{{ros}}/setup.bash"
    cd "{{ws}}"
    colcon test {{args}} && colcon test-result --verbose

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
    timeout -s INT 18 ros2 launch pimesh_bringup pimesh.launch.py \
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
        echo "  ok    pimesh_container up (empty until P2)"
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
