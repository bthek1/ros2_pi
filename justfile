# ros2_pi — the commands you actually type. `just` with no arguments lists them.
#
# Deliberately short, and it is meant to stay that way. This file is the
# *user-facing* surface of the workspace and nothing else: build it, and run the
# two things there are to watch. Everything that is not a day-to-day action is a
# script in tools/ and is run as one — no recipe, no wrapper:
#
#   bash tools/gates/build.sh         P0: one source tree, two distros, same msgs
#   bash tools/gates/capture.sh       P1: 720p MJPEG on the LAN, stamped honestly
#   bash tools/gates/test.sh          the unit tests pass on both machines
#   bash tools/gates/view-configs.sh  every .rviz topic is one src/ publishes
#   bash tools/camera-reset.sh        clear the camera's persistent V4L2 controls
#   bash tools/test.sh                ...just run them here (tools/test-pi.sh there)
#
#   bash tools/gates/hello-build.sh   scaffolding: one real package builds
#   bash tools/gates/hello-talk.sh    P1: the talker honours its rate parameter
#   bash tools/gates/hello-ipc.sh     P2: one process, message handed over as a pointer
#   bash tools/gates/hello-lan.sh     P3: one source tree, two distros, over the LAN
#   bash tools/gates/hello-clean.sh   P4: Ctrl-C leaves nothing running, either machine
#   bash tools/gates/justfile.sh      this file's own shape
#   bash tools/stragglers.sh          assert nothing outlived its session
#   bash tools/sync-pi.sh             ship source to the Pi — source only
#   bash tools/build-pi.sh            ...and build it there, under Jazzy
#   bash tools/clean.sh               delete the colcon trees (tools/clean-pi.sh for the Pi's)
#
# Each gate exits non-zero and prints the number it asserted on. They are run
# often, but they are not what a person new to the workspace needs to see first,
# and burying `hello-compose` among seven of them was the reason this file got
# trimmed.
#
# The recipe bodies stay one line each regardless of how few there are. `just`
# gives a recipe body no way to share code with another, so inlined bash gets
# copy-pasted, drifts between the copies, and is invisible to shellcheck, which
# cannot parse {{ }}. tools/just-lib.sh holds the shared prelude, the one
# spelling of the Pi's ssh invocation, and the bracketed kill patterns — both
# things that must never be written differently twice.
#
# tools/ is rsynced to the Pi, so everything in it must work on both distros —
# the dev box is Lyrical, the Pi is Jazzy, and tools/ros-env.sh discovers which
# rather than naming one.

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
