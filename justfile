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
    python3 -m pytest tools/ -q || rc=1
    exit "${rc}"

# The same tests under the OTHER distro and compiler. A test that has only ever
# run on Lyrical says nothing about the machine that actually runs the camera.
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
    if [ "{{seconds}}" != "0" ]; then bound="timeout -s INT {{seconds}}"; fi
    ssh {{ssh_opts}} -tt {{pi_host}} "bash -lc '${bound} ros2 launch pimesh_camera camera.launch.py {{args}}'"

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
        ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT 45 ros2 launch pimesh_camera camera.launch.py > /tmp/cam${run}.log 2>&1'" &
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
    ssh {{ssh_opts}} {{pi_host}} "bash -lc 'timeout -s INT 20 ros2 launch pimesh_camera camera.launch.py > /tmp/holder.log 2>&1'" &
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
