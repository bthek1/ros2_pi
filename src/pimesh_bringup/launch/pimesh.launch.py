"""
The dev-box side of the pipeline: one process, one decode, zero copies.

Every dev-box stage is an rclcpp component loaded into a SINGLE container with
`use_intra_process_comms=True`. That is not a tidiness preference, it is the
architecture:

  * Exactly one subscriber to the Pi's stream. Five RELIABLE subscribers each
    pull their own unicast copy over the Pi's Wi-Fi and collapse the link — the
    predecessor measured ~2 frames/s per reader against 14.7 Hz for one.
  * One JPEG decode. Downstream components receive a shared_ptr to the same
    buffer, so a 2.7 MB frame is never copied or serialised between stages.

Splitting these into separate processes "for debugging" undoes both. If you
need to inspect a stage, subscribe its *output* from outside the container —
the previews are published for exactly that.

The dashboard and RViz stay OUTSIDE this container on purpose: they are
viewers, they can afford serialisation, and they must be able to die without
taking the pipeline with them.

As of P0 the container is empty. Each phase of the bootstrap plan adds its own
component to the list below, and the container is what makes the phase's
intra-process guarantee testable (`just gate-ipc`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    share = get_package_share_directory('pimesh_bringup')
    default_config = os.path.join(share, 'config', 'pimesh.yaml')

    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='parameter file, keyed by node name. A key that does '
                        'not match a node name applies NOTHING, silently'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'frames.launch.py'))),

        ComposableNodeContainer(
            name='pimesh_container',
            namespace='',
            package='rclcpp_components',
            # _mt = MultiThreadedExecutor. Each expensive component owns a
            # worker thread and a one-deep mailbox; the executor only ever
            # runs the cheap callbacks that hand work to them.
            executable='component_container_mt',
            composable_node_descriptions=[
                # P2: decode_node
                # P3: keypoint_node
                # P4: depth_node
                # P5: fusion_node
                # P6: mesh_node
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
