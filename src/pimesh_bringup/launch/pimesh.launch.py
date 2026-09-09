"""The dev box's bringup: the frames, and the container everything will run in.

Empty of components today, and that is the point of it existing now — the
container and the frame tree are what every later stage plugs into, so they get
built and launched once, before there is anything to plug in. P2 adds the first
`ComposableNode` to the list below and changes nothing else about this file.

**One container, one process.** The Python predecessor ran each stage as its own
process and five of them subscribed the Pi's stream directly; each RELIABLE
subscriber pulls its own unicast copy over Wi-Fi, which collapsed the link to
~2 frames/s per reader against 14.7 Hz for a single one. Composing the dev-box
stages into one process makes a publish a `std::move` of a `unique_ptr`
instead: one network subscriber, one decode, and a pointer to every consumer.
Launching these as separate processes "for debugging" is precisely the thing
that breaks it.

There is deliberately no `intra_process` launch argument here yet.
`use_intra_process_comms` is a *per-component* option — it travels in a
`ComposableNode`'s `extra_arguments`, and a container with no components has
nowhere to put it. Declaring one now would be an argument that reads as if it
configured something and configured nothing, which is the same class of quiet
lie as a parameter key matching no node. It arrives in P2, with the first
component and the gate that proves the pointer was handed over.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node

# The three static edges, as node names that are also the keys in
# config/pimesh.yaml. Exactly one publisher per edge, all three of them here —
# a node that publishes its own base_link -> camera_link is the failure that
# makes a mesh smear and shows up in no single log.
STATIC_TRANSFORMS = ['map_to_odom', 'base_to_camera', 'camera_to_optical']


def _static_transform_args(entry: dict) -> list:
    """Turn one config/pimesh.yaml entry into static_transform_publisher's argv.

    The numbers live in the YAML because that is where this project's
    configuration lives, and they arrive as command-line flags because that is
    the only entry point the tool actually has. `static_transform_publisher`
    *declares* `frame_id`, `translation.x` and the rest as ROS parameters — they
    are visible in the binary — but its `main()` parses argv first and exits
    non-zero with "Frame id must not be empty" before a parameter file is ever
    read (measured 2026-09-09, both distros). Passing `parameters=[...]` to it
    produces three dead processes and a launch that carries on without a TF
    tree, which is worse than a hard failure because RViz then shows an empty
    Fixed Frame and blames itself.

    So: one source of truth in the YAML, translated here into the form the tool
    accepts, rather than the numbers written twice.
    """
    t = entry['translation']
    r = entry['rotation']
    return [
        '--frame-id', str(entry['frame_id']),
        '--child-frame-id', str(entry['child_frame_id']),
        '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
        '--qx', str(r['x']), '--qy', str(r['y']),
        '--qz', str(r['z']), '--qw', str(r['w']),
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('pimesh_bringup')
    params_path = os.path.join(share, 'config', 'pimesh.yaml')
    with open(params_path) as fh:
        params = yaml.safe_load(fh)

    transforms = []
    for name in STATIC_TRANSFORMS:
        # KeyError here is the right failure: a renamed key silently publishing
        # identity is the outcome this whole file is arranged to prevent.
        entry = params[f'/**/{name}']['ros__parameters']
        transforms.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=name,
                arguments=_static_transform_args(entry),
                output='screen',
            )
        )

    return LaunchDescription([
        *transforms,
        ComposableNodeContainer(
            name='pimesh_container',
            namespace='',
            package='rclcpp_components',
            # _mt: a multi-threaded executor. Single-threaded serialises every
            # callback onto one thread, which is invisible while the callbacks
            # are cheap and fatal once one of them costs 76 ms.
            executable='component_container_mt',
            # Empty until P2. The container is still worth starting: it holds
            # the process, the executor and the node namespace, so a stage can
            # be loaded into a running pipeline with `ros2 component load`.
            composable_node_descriptions=[],
            output='screen',
        ),
    ])
