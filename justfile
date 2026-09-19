# ros2_pi — the commands you actually type. `just` with no arguments lists them.
#
# Deliberately short and meant to stay that way: this file is the *user-facing*
# surface of the workspace and nothing else. Anything that is not a day-to-day
# action is a script in tools/, run as one — no recipe, no wrapper:
#
# The phase gates, each exiting non-zero and printing the number it asserted on:
#   bash tools/gates/{build,capture,ipc,keypoints,depth,fusion,mesh,odom}.sh P0-P7
#   bash tools/gates/{gpu-stack,calibration}.sh  the GPU toolchain; P9's intrinsics
#   bash tools/gates/{test,view-configs,justfile}.sh          the workspace's own
#   bash tools/gates/hello-{build,talk,ipc,lan,clean}.sh      the scaffolding's own
# and the scripts they lean on:
#   bash tools/record-clip.sh desk1   record the reference clip P3 onwards replay
#   bash tools/fetch-{gpu-stack,model}.sh   the GPU stack and the depth weights
#   bash tools/mesh-views.sh <mesh.ply>     three offscreen renders — P6's evidence
#   bash tools/{calibrate,camera-reset}.sh  P9's intrinsics; the V4L2 controls
#   bash tools/test.sh   the unit tests here; tools/stragglers.sh sweeps both hosts
#   bash tools/{sync,build,clean}-pi.sh and tools/clean.sh   the Pi's trees, ours
# Recipe bodies stay one line each. `just` gives a body no way to share code with
# another, so inlined bash drifts and shellcheck cannot parse {{ }} to catch it;
# tools/just-lib.sh holds the prelude, the Pi's ssh invocation and the kill
# patterns, and tools/ is rsynced so it works on both distros.

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

# Hello world, here: both components in one container. seconds = how long to run
[group('run')]
hello-compose seconds="30":
    @bash "{{ ws }}/tools/hello-compose.sh" {{ seconds }}

# Hello world, across the LAN: talker on the Pi, listener here
[group('run')]
hello-lan seconds="20":
    @bash "{{ ws }}/tools/hello-lan.sh" {{ seconds }}

# The Pi's camera and the frame tree, in RViz. A viewer, not evidence
[group('run')]
view-camera seconds="600":
    @bash "{{ ws }}/tools/view-camera.sh" {{ seconds }}

# A recorded bag in RViz, looping. bag = a name under bags/, or a path to one
[group('run')]
replay bag seconds="600":
    @bash "{{ ws }}/tools/replay.sh" {{ bag }} {{ seconds }}

# ORB corners and the pose, in RViz. bag = optional, else the camera
[group('run')]
view-keypoints seconds="600" bag="":
    @bash "{{ ws }}/tools/view-keypoints.sh" {{ seconds }} {{ bag }}

# The room as a depth cloud, in RViz. bag = optional, else the camera
[group('run')]
view-depth seconds="600" bag="":
    @bash "{{ ws }}/tools/view-depth.sh" {{ seconds }} {{ bag }}

# The room as a triangle surface, in RViz. bag = optional, else the camera
[group('run')]
view-mesh seconds="600" bag="":
    @bash "{{ ws }}/tools/view-mesh.sh" {{ seconds }} {{ bag }}

# The camera's trajectory, in RViz. regime = sixdof (default) or rotation_only
[group('run')]
view-odom seconds="600" bag="" regime="sixdof":
    @bash "{{ ws }}/tools/view-odom.sh" {{ seconds }} {{ bag }} {{ regime }}
