# ros2_pi — the commands you actually type. `just` with no arguments lists them.
#
# The user-facing surface, kept short: build, test, record, and the viewers.
# Every viewer is `tools/session.sh` around `ros2 launch pimesh_bringup
# view.launch.py view:=<name>` — launch owns the processes, session.sh refuses to
# start beside another session and tears down the Pi, which launch cannot reach.
# What a view *is* lives in pimesh_bringup/config/views.yaml (#15).
# Gates are not recipes: `bash tools/gates/<name>.sh`, each exiting non-zero and
# printing the number it asserted on. Recipe bodies stay one line each.

set shell := ["bash", "-euo", "pipefail", "-c"]

ws := justfile_directory()

# List the recipes
default:
    @just --list

# Build the workspace
[group('build')]
build *args:
    @bash "{{ ws }}/tools/build.sh" {{ args }}

# Delete build/ install/ log/ here
[group('build')]
clean:
    @bash "{{ ws }}/tools/clean.sh"

# Build, run every unit test, and exit non-zero on any failure (colcon test alone does not)
[group('test')]
test *args:
    @bash "{{ ws }}/tools/test.sh" {{ args }}

# The Pi's camera and the frame tree, in RViz. A viewer, not evidence
[group('run')]
view-camera seconds="600":
    @bash "{{ ws }}/tools/session.sh" view-camera --pi -- ros2 launch pimesh_bringup view.launch.py view:=camera seconds:="{{ seconds }}"

# A recorded bag in RViz, looping. bag = a name under bags/, or a path to one
[group('run')]
replay bag seconds="600":
    @bash "{{ ws }}/tools/session.sh" replay --local -- ros2 launch pimesh_bringup view.launch.py view:=replay bag:="{{ bag }}" seconds:="{{ seconds }}"

# ORB corners and the pose, in RViz. bag = optional, else the camera
[group('run')]
view-keypoints seconds="600" bag="":
    @bash "{{ ws }}/tools/session.sh" view-keypoints {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=keypoints seconds:="{{ seconds }}" bag:="{{ bag }}"

# The room as a depth cloud, in RViz. bag = optional, else the camera
[group('run')]
view-depth seconds="600" bag="":
    @bash "{{ ws }}/tools/session.sh" view-depth {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=depth seconds:="{{ seconds }}" bag:="{{ bag }}"

# The room as a triangle surface, in RViz. bag = optional, else the camera
[group('run')]
view-mesh seconds="600" bag="":
    @bash "{{ ws }}/tools/session.sh" view-mesh {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=mesh seconds:="{{ seconds }}" bag:="{{ bag }}"

# The camera's trajectory, in RViz. odom_regime = sixdof (default) or rotation_only
[group('run')]
view-odom seconds="600" bag="" odom_regime="sixdof":
    @bash "{{ ws }}/tools/session.sh" view-odom {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=odom seconds:="{{ seconds }}" bag:="{{ bag }}" odom_regime:="{{ odom_regime }}"

# Map points and the trail, in RViz. local_ba = true (default) or false, the control
[group('run')]
view-map seconds="600" bag="" local_ba="true":
    @bash "{{ ws }}/tools/session.sh" view-map {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=map seconds:="{{ seconds }}" bag:="{{ bag }}" local_ba:="{{ local_ba }}"

# Record a clip from the Pi's camera into bags/<name>, counted from the first frame
[group('run')]
record name seconds="60":
    @bash "{{ ws }}/tools/session.sh" record --pi -- ros2 launch pimesh_bringup record.launch.py name:="{{ name }}" seconds:="{{ seconds }}"

# The whole pipeline in a browser tab: http://localhost:8080. Not evidence
[group('run')]
dashboard seconds="600" bag="" dashboard_port="8080":
    @bash "{{ ws }}/tools/session.sh" dashboard {{ if bag == "" { "--pi" } else { "--local" } }} -- ros2 launch pimesh_bringup view.launch.py view:=dashboard seconds:="{{ seconds }}" bag:="{{ bag }}" dashboard_port:="{{ dashboard_port }}"
