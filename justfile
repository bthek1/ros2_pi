# ros2_pi — monocular visual SLAM: one webcam on the Pi, its pose and a mesh of
# the room on this box. `just` lists the one command; `just run` is the project.
#
# `run` builds, then brings up the SLAM pipeline (tracking, loop closure, the
# TSDF rebuilt at corrected poses) with RViz in front of it: `tools/session.sh`
# around `ros2 launch pimesh_bringup view.launch.py view:=run`. Anything after
# the bag is a pimesh.launch.py argument, e.g. `just run 600 desk1 loop_closure:=false`
# or `just run 600 "" dashboard:=true`.
#
# build, test and clean are [private]: they work, and are not listed. Gates are
# `bash tools/gates/<name>.sh`; recording a clip is `bash tools/record.sh`.

set shell := ["bash", "-euo", "pipefail", "-c"]

ws := justfile_directory()

# List the recipes
[private]
default:
    @just --list

# Build, then the SLAM pipeline in RViz. bag = a clip under bags/ (default: the Pi's camera); then any pimesh.launch.py name:=value
run seconds="600" bag="" *launch_args: build
    @bash "{{ ws }}/tools/session.sh" run {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=run seconds:="{{ seconds }}" bag:="{{ bag }}" {{ launch_args }}

[private]
build *args:
    @bash "{{ ws }}/tools/build.sh" {{ args }}

[private]
test *args:
    @bash "{{ ws }}/tools/test.sh" {{ args }}

[private]
clean:
    @bash "{{ ws }}/tools/clean.sh"
