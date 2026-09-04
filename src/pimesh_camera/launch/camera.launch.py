"""
The Pi's whole side of the pipeline: one node, one device, one topic.

Three things here are deliberate and are why this file is not a three-liner:

  * `on_exit=Shutdown()` — if the camera node dies, the launch dies. A camera
    consumer that lingers after losing its device is the failure mode this
    project inherited: it looks identical to a healthy node publishing into a
    topic nobody reads.
  * `emulate_tty=True` — so log output is not buffered into invisibility when
    this runs under ssh, which is how it always runs.
  * The overrides are resolved in an OpaqueFunction. A LaunchConfiguration is
    an object, not a string, so it is ALWAYS truthy: the tempting
    `{'device': device} if device else {}` silently passes device='' and
    overrides the config file with nothing. Substitutions have to be performed
    against the launch context before they can be tested.

The frame tree is NOT published here. The Pi owns the camera, not the geometry;
the static transforms live in pimesh_bringup on the dev box, and one publisher
per TF edge is a rule this project holds to.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context, *args, **kwargs):
    share = get_package_share_directory('pimesh_camera')
    config = LaunchConfiguration('config').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    parameters = [config or os.path.join(share, 'config', 'camera.yaml')]

    # Only non-empty overrides become a parameter dict; an empty one would
    # shadow the config file's value with a blank.
    overrides = {}
    device = LaunchConfiguration('device').perform(context)
    if device:
        overrides['device'] = device
    framerate = LaunchConfiguration('framerate').perform(context)
    if framerate:
        overrides['framerate'] = int(framerate)
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='pimesh_camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            parameters=parameters,
            arguments=['--ros-args', '--log-level', log_level],
            # The launch exits when the node does, carrying its failure with it.
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='',
            description='parameter file, keyed by node name (empty = the '
                        "package's own config/camera.yaml)"),
        DeclareLaunchArgument(
            'device', default_value='',
            description='override the capture device. Used by the failure '
                        'gates: a device that does not exist must make this '
                        'exit non-zero rather than idle'),
        DeclareLaunchArgument(
            'framerate', default_value='',
            description='override the requested frame rate (empty = config)'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='node log level'),
        OpaqueFunction(function=_camera_node),
    ])
