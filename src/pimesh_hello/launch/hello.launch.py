"""Compose the talker and the listener into one process.

This is the architectural bet of the whole rewrite, in miniature. The dev box
runs one container with intra-process comms on, because that turns a publish
into a `std::move` of a `unique_ptr` — the decode happens once and every
downstream stage gets the same buffer. Launching components as separate
processes "for debugging" is what breaks it, so this launch file exists to make
the composed form the easy one.

The `intra_process` argument is not a convenience. `gate-hello-ipc` runs this
file twice, once each way, because an address that matches proves nothing until
you have watched it fail to match with the mechanism switched off.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    intra_process = LaunchConfiguration('intra_process')

    params = PathJoinSubstitution(
        [FindPackageShare('pimesh_hello'), 'config', 'hello.yaml']
    )

    # ParameterValue with an explicit value_type: a LaunchConfiguration is a
    # string, and use_intra_process_comms wants a bool. Passing the raw
    # substitution sets a *string* parameter named use_intra_process_comms,
    # which the container ignores — the components then load without
    # intra-process comms and nothing anywhere says so.
    extra = [{'use_intra_process_comms': ParameterValue(intra_process, value_type=bool)}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'intra_process',
            default_value='true',
            description='Hand messages over as pointers inside the container.',
        ),
        ComposableNodeContainer(
            name='hello_container',
            namespace='',
            package='rclcpp_components',
            # _mt: a multi-threaded executor. Single-threaded would serialise
            # the timer and the subscription onto one thread, which works fine
            # here and would not once a callback costs milliseconds.
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='pimesh_hello',
                    plugin='pimesh_hello::HelloNode',
                    name='hello_node',
                    parameters=[params],
                    extra_arguments=extra,
                ),
                ComposableNode(
                    package='pimesh_hello',
                    plugin='pimesh_hello::EchoNode',
                    name='echo_node',
                    parameters=[params],
                    extra_arguments=extra,
                ),
            ],
            output='screen',
        ),
    ])
