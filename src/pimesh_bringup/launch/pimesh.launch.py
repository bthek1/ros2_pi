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

As of P2 the container holds `decode_node`, plus an optional zero-copy probe
(`probe:=true`, which is what `just gate-ipc` launches). Each later phase adds
its own component to the list below, and the container is what makes the
phase's intra-process guarantee testable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('pimesh_bringup')
    default_config = os.path.join(share, 'config', 'pimesh.yaml')

    config = LaunchConfiguration('config')
    probe = LaunchConfiguration('probe')

    # One knob, not two. `probe:=true` has to switch on BOTH sides of the
    # comparison — decode_node logging what it published and the probe logging
    # what it received — because either alone proves nothing. A separate count
    # argument would let `probe:=true` silently log nothing at all.
    probe_addresses = PythonExpression(
        ["10 if '", probe, "'.lower() in ('true', '1') else 0"])

    # Set per COMPONENT, not once for the container. A component added later
    # without this line silently serialises while its neighbours do not, and
    # nothing about the launch looks wrong.
    #
    # It is a launch ARGUMENT rather than a hardcoded True so that the negative
    # can be demonstrated. `just gate-ipc` runs the container both ways: with it
    # on the published and received addresses must be equal, and with it off
    # they must DIFFER. An address check that has never been seen to fail is
    # not evidence that it can.
    intra_process = {
        'use_intra_process_comms': ParameterValue(
            LaunchConfiguration('intra_process'), value_type=bool),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='parameter file, keyed by node name. A key that does '
                        'not match a node name applies NOTHING, silently'),
        DeclareLaunchArgument(
            'intra_process', default_value='true',
            description='pass buffers between components instead of '
                        'serialising them. false is for gate-ipc, which proves '
                        'the check can fail, and for nothing else'),
        DeclareLaunchArgument(
            'probe', default_value='false',
            description='load ipc_probe_node, which logs the address of every '
                        'buffer it receives so gate-ipc can prove the '
                        'container is zero-copy. Off in normal operation'),

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
                ComposableNode(
                    package='pimesh_perception',
                    plugin='pimesh_perception::DecodeNode',
                    name='decode_node',
                    # The file first, then the override: later entries win, so
                    # the launch argument beats the YAML rather than the
                    # reverse. value_type is required — a substitution resolves
                    # to a string, and an untyped '10' would be declared as one
                    # and rejected against the node's integer parameter.
                    parameters=[
                        config,
                        {'log_buffer_addresses': ParameterValue(
                            probe_addresses, value_type=int)},
                    ],
                    extra_arguments=[intra_process],
                ),
                # P3: keypoint_node
                # P4: depth_node
                # P5: fusion_node
                # P6: mesh_node
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        # Loaded into the SAME container, or it would prove nothing: a probe in
        # its own process cannot receive an intra-process buffer, and the
        # address comparison would then fail for the wrong reason.
        LoadComposableNodes(
            condition=IfCondition(probe),
            target_container='pimesh_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pimesh_perception',
                    plugin='pimesh_perception::IpcProbeNode',
                    name='ipc_probe_node',
                    parameters=[
                        config,
                        {'frames': ParameterValue(probe_addresses, value_type=int)},
                    ],
                    extra_arguments=[intra_process],
                ),
            ],
        ),
    ])
