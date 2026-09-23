# ros2_pi — the commands you actually type. `just` with no arguments lists them.
#
# Deliberately short and meant to stay that way: this file is the *user-facing*
# surface of the workspace. Anything that is not a day-to-day action is a script
# in tools/, run as one — no recipe, no wrapper. The phase gates, each exiting
# non-zero and printing the number it asserted on:
#   bash tools/gates/{build,capture,ipc,keypoints,depth}.sh          P0-P4
#   bash tools/gates/{fusion,mesh,odom,dashboard}.sh                 P5-P8
#   bash tools/gates/{gpu-stack,calibration}.sh  the GPU toolchain; P9's intrinsics
#   bash tools/gates/{test,view-configs,justfile,teardown}.sh the workspace's own
# and the scripts they lean on — record-clip, fetch-{gpu-stack,model}, mesh-views,
# calibrate, camera-reset, test, stragglers, {sync,build,clean}-pi, clean.
#
# Recipe bodies stay one line each: `just` gives a body no way to share code, so
# inlined bash drifts and shellcheck cannot parse {{ }} to catch it.
# tools/just-lib.sh holds the prelude, the Pi's ssh and the kill patterns.

set shell := ["bash", "-euo", "pipefail", "-c"]

ws := justfile_directory()

# List the recipes
default:
    @just --list

# --- Build ------------------------------------------------------------------

# Build the workspace
[group('build')]
build *args:
    @bash "{{ ws }}/tools/build.sh" {{ args }}

# --- Things to watch --------------------------------------------------------

# The Pi's camera and the frame tree, in RViz. A viewer, not evidence
[group('run')]
view-camera seconds="600":
    @bash "{{ ws }}/tools/view-camera.sh" "{{ seconds }}"

# A recorded bag in RViz, looping. bag = a name under bags/, or a path to one
[group('run')]
replay bag seconds="600":
    @bash "{{ ws }}/tools/replay.sh" "{{ bag }}" "{{ seconds }}"

# ORB corners and the pose, in RViz. bag = optional, else the camera
[group('run')]
view-keypoints seconds="600" bag="":
    @bash "{{ ws }}/tools/view-keypoints.sh" "{{ seconds }}" "{{ bag }}"

# The room as a depth cloud, in RViz. bag = optional, else the camera
[group('run')]
view-depth seconds="600" bag="":
    @bash "{{ ws }}/tools/view-depth.sh" "{{ seconds }}" "{{ bag }}"

# The room as a triangle surface, in RViz. bag = optional, else the camera
[group('run')]
view-mesh seconds="600" bag="":
    @bash "{{ ws }}/tools/view-mesh.sh" "{{ seconds }}" "{{ bag }}"

# The camera's trajectory, in RViz. regime = sixdof (default) or rotation_only
[group('run')]
view-odom seconds="600" bag="" regime="sixdof":
    @bash "{{ ws }}/tools/view-odom.sh" "{{ seconds }}" "{{ bag }}" "{{ regime }}"

# The whole pipeline in a browser tab: http://localhost:8080. Not evidence
[group('run')]
dashboard seconds="600" bag="" port="8080":
    @bash "{{ ws }}/tools/dashboard.sh" "{{ seconds }}" "{{ bag }}" "{{ port }}"
