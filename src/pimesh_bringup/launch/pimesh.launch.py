"""The dev box's bringup: the frames, and the container everything will run in.

The container held no components at all until P2; it was started anyway, because
the container and the frame tree are what every later stage plugs into. As of
2026-09-12 it holds `decode_node`, the one subscriber on the topic that crosses
Wi-Fi, and every stage added after this one goes in the same list beside it.

**One container, one process.** The Python predecessor ran each stage as its own
process and five of them subscribed the Pi's stream directly; each RELIABLE
subscriber pulls its own unicast copy over Wi-Fi, which collapsed the link to
~2 frames/s per reader against 14.7 Hz for a single one. Composing the dev-box
stages into one process makes a publish a `std::move` of a `unique_ptr`
instead: one network subscriber, one decode, and a pointer to every consumer.
Launching these as separate processes "for debugging" is precisely the thing
that breaks it.

**`intra_process` is an argument because a gate needs it to be.**
`use_intra_process_comms` is a *per-component* option — it travels in a
`ComposableNode`'s `extra_arguments` — so it could not exist here until there was
a component to carry it. Now that there is, it is a launch argument rather than a
hard-coded `True`, because an address that matches proves nothing until you have
watched it fail to match with the mechanism switched off: `tools/gates/ipc.sh`
runs this file both ways. It is `true` by default and normal operation never
changes it.

Note `ParameterValue(..., value_type=bool)` below. A `LaunchConfiguration` is a
string, and passing the raw substitution sets a *string* parameter named
`use_intra_process_comms`, which the container ignores while saying nothing — the
components then load serialising every frame and no log line mentions it.

**`log_payloads` is the other half of that gate**, and it is an argument for a
duller reason: `ros2 launch` has no way to override one node's parameter from the
command line. There is no `-p node.key:=value` — it parses its own arguments and
stops — so a parameter a gate needs to flip has to be reachable through a launch
argument or through a second YAML file. An argument threaded into
`decode_node`'s `parameters` list, after the YAML so it wins, is the smaller of
the two.

**`probe` loads the gate's instrument**, and it is a `LoadComposableNodes` rather
than a fourth entry in the list below, because `composable_node_descriptions` is
built when this file is evaluated and cannot be made conditional on an argument.
`LoadComposableNodes` can: it calls the running container's load service, which
is the same path `ros2 component load` takes.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

# The three static edges, as node names that are also the keys in
# config/pimesh.yaml. Exactly one publisher per edge, all three of them here —
# a node that publishes its own base_link -> camera_link is the failure that
# makes a mesh smear and shows up in no single log.
STATIC_TRANSFORMS = ['map_to_odom', 'base_to_camera', 'camera_to_optical']

# The components composed into the container, as (node name, plugin string). The
# node name is also the key in config/pimesh.yaml, and the plugin string is what
# the container looks up in the ament index at runtime — so a typo in either is a
# failure with no build error in front of it: a mis-keyed parameter silently
# applies nothing, and a mis-spelled plugin fails at launch.
#
# A list rather than literals inline, for the same reason STATIC_TRANSFORMS is one:
# test_transforms.py checks both halves of it against the YAML and the index, and
# it can only check what it can enumerate.
COMPONENTS = [
    # The container's one network subscriber. Everything else here reads its
    # output in-process, by pointer.
    ('decode_node', 'pimesh_perception::DecodeNode'),
    # ORB on every decoded frame, plus the rotation-only pose.
    ('keypoint_node', 'pimesh_perception::KeypointNode'),
]

# Loaded only with `probe:=true`: the instrument tools/gates/ipc.sh measures the
# pointer handover with. Separate from COMPONENTS because it is not part of the
# pipeline and must not be running when anything else is measured.
PROBE_COMPONENTS = [
    ('ipc_probe', 'pimesh_perception::IpcProbe'),
]


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


def _component(name: str, plugin: str, params_path: str, extra: list) -> ComposableNode:
    """One entry in the container's list.

    The parameters are the keyed YAML first and a single override second, and the
    order is load-bearing: rclcpp applies them in sequence, so the launch argument
    wins over the file while the file stays the one place the defaults live.

    The override is here at all because `ros2 launch` has no way to set one node's
    parameter from the command line — it parses its own arguments and stops — so a
    parameter a gate needs to flip has to be reachable through a launch argument.
    `log_payloads` is decode_node's and harmless elsewhere: a node that never
    declared it ignores it.
    """
    return ComposableNode(
        package='pimesh_perception',
        plugin=plugin,
        name=name,
        parameters=[
            params_path,
            {
                'log_payloads': ParameterValue(
                    LaunchConfiguration('log_payloads'), value_type=bool),
            },
        ],
        extra_arguments=extra,
    )


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

    intra_process = LaunchConfiguration('intra_process')

    # The option, in the form the container actually reads. One dict, shared by
    # every component: they are either all handing pointers to each other or none
    # of them are, and a mixture would be a pipeline with a copy somewhere in the
    # middle that no log mentions.
    extra = [{'use_intra_process_comms': ParameterValue(intra_process, value_type=bool)}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'intra_process',
            default_value='true',
            description='Hand frames between components as pointers. '
                        'false is the control run in tools/gates/ipc.sh.',
        ),
        DeclareLaunchArgument(
            'log_payloads',
            default_value='false',
            description="Log decode_node's published buffer address, once per "
                        'frame. 59 log lines a second; for gates, not for use.',
        ),
        DeclareLaunchArgument(
            'probe',
            default_value='false',
            description="Also load pimesh_perception's IpcProbe, the instrument "
                        'tools/gates/ipc.sh measures the pointer handover with.',
        ),
        *transforms,
        ComposableNodeContainer(
            name='pimesh_container',
            namespace='',
            package='rclcpp_components',
            # _mt: a multi-threaded executor. Single-threaded serialises every
            # callback onto one thread, which is invisible while the callbacks
            # are cheap and fatal once one of them costs 76 ms.
            executable='component_container_mt',
            composable_node_descriptions=[
                _component(name, plugin, params_path, extra) for name, plugin in COMPONENTS
            ],
            output='screen',
        ),
        LoadComposableNodes(
            target_container='pimesh_container',
            condition=IfCondition(LaunchConfiguration('probe')),
            composable_node_descriptions=[
                _component(name, plugin, params_path, extra)
                for name, plugin in PROBE_COMPONENTS
            ],
        ),
    ])
