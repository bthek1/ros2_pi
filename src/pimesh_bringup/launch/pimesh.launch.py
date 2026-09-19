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

**`probe` names a gate's instrument**, and each is loaded by its own
`LoadComposableNodes` rather than listed with the pipeline, because
`composable_node_descriptions` is built when this file is evaluated and cannot be
made conditional on an argument. `LoadComposableNodes` can: it calls the running
container's load service, which is the same path `ros2 component load` takes.
It is a *name* (`ipc_probe`, `depth_probe`, `none`) and not a flag now that there
are two of them, so that a run measuring one cannot quietly be loading the other.

**`pipeline:=false` brings up the frame tree and nothing else**, which is the
shape this file had before P2 put components in it, and it exists again because
a *looping bag* and a node publishing TF from that bag's stamps cannot both be
right. `ros2 bag play --loop` restarts at the beginning, so every header stamp
jumps ~60 s into the past; `keypoint_node` stamps `odom -> base_link` with the
frame's own stamp, as it must; and `tf2::BufferCore` refuses data older than the
newest it holds. Measured 2026-09-13: after the first wrap the edge froze at the
bag's final stamp and never moved again for the rest of the run, while every
listener logged TF_OLD_DATA at the frame rate. `tools/replay.sh` is the viewer
that wants this — see its header for why the warning is not merely noisy.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

# The three static edges, as node names that are also the keys in
# config/pimesh.yaml. Exactly one publisher per edge, all three of them here —
# a node that publishes its own base_link -> camera_link is the failure that
# makes a mesh smear and shows up in no single log.
STATIC_TRANSFORMS = ['map_to_odom', 'base_to_camera', 'camera_to_optical']

# The components composed into the container, as (node name, package, plugin).
# The node name is also the key in config/pimesh.yaml, and the package and plugin
# together are what the container looks up in the ament index at runtime — so a
# typo in any of the three is a failure with no build error in front of it: a
# mis-keyed parameter silently applies nothing, and a mis-spelled plugin fails at
# launch.
#
# **The package was a hard-coded 'pimesh_perception' until P5**, which was correct
# for exactly as long as one package supplied every component. It is in the tuple
# now because the alternative — deriving it from the plugin's namespace — is a
# convention nothing enforces, and the failure it would hide is a container
# looking up a class in the wrong package's index and reporting only that it could
# not find it.
#
# A list rather than literals inline, for the same reason STATIC_TRANSFORMS is one:
# test_transforms.py checks both halves of it against the YAML and the index, and
# it can only check what it can enumerate.
COMPONENTS = [
    # The container's one network subscriber. Everything else here reads its
    # output in-process, by pointer.
    ('decode_node', 'pimesh_perception', 'pimesh_perception::DecodeNode'),
    # ORB on every decoded frame, plus the rotation-only pose.
    ('keypoint_node', 'pimesh_perception', 'pimesh_perception::KeypointNode'),
    # Monocular depth on the GPU. **This is the pipeline's clock** — a frame costs
    # ~55 ms against a ~17 ms frame interval, measured at 17.42 Hz out of 59 Hz, so
    # it keeps roughly one frame in three and drops the rest through its own
    # one-slot mailbox. It is in this list
    # unconditionally, like everything else here, because the container is
    # everything or nothing: the components share one process precisely so that a
    # 2.7 MB frame reaches all three of them as a pointer.
    ('depth_node', 'pimesh_perception', 'pimesh_perception::DepthNode'),
    # The TSDF. **This is where the pipeline stops being a stream and starts
    # remembering** — and it is in this container rather than a process of its own
    # for a reason one size up from the frame handover: what it produces is a
    # volume of hundreds of megabytes whose only consumer is mesh_node, in this
    # same process. See pimesh_world/shared_volume.hpp.
    ('fusion_node', 'pimesh_world', 'pimesh_world::FusionNode'),
    # Marching cubes over a snapshot of that volume, on a timer. In this container
    # because it has to be: it reads the volume by pointer, and a mesh_node in
    # another process finds no volume at all and says so.
    ('mesh_node', 'pimesh_world', 'pimesh_world::MeshNode'),
]

# Nodes that run as their own process rather than in the container, as
# (node name, package, executable).
#
# **There is exactly one and it is deliberate.** Everything else on the dev box
# shares a process precisely so a 2.7 MB frame is handed on as a pointer, and
# `pipeline:=false` is the only granularity offered — everything or nothing.
# `dashboard_node` is the exception because its whole promise is the opposite
# one: `docs/info/dashboard.md` says *it must be able to die*, and a component in
# the container could not make that promise, since a crash there would take the
# TSDF with it. It subscribes to five small topics and holds a socket open to
# something outside this machine's control, and it has nothing to gain from the
# pointer handover.
#
# A list rather than a literal for the same reason the other two are lists:
# test_transforms.py checks it against the YAML and it can only check what it can
# enumerate.
STANDALONE_NODES = [
    ('dashboard_node', 'pimesh_dashboard', 'dashboard_node'),
]

# The gates' instruments, none of which is part of the pipeline.
#
# **`probe` names one of these rather than being a boolean, and the change of
# shape is the point.** With one probe a flag was enough; with two, a flag can no
# longer say the thing this list's old comment claimed — that a probe "must not be
# running when anything else is measured". `probe:=depth_probe` loading the IPC
# probe as well would put a third subscriber on the decoded topic during the very
# measurement it is there to take, and nothing in the output would mention it. A
# name makes that exclusion structural instead of a comment: `probe:=none` (the
# default) loads nothing, and no value loads two.
#
# The value **is the node name**, not a nickname mapped to one, so there is no
# second table to drift out of step with this one.
PROBE_COMPONENTS = [
    # tools/gates/ipc.sh: compares the buffer address decode_node published
    # against the one a subscriber received.
    ('ipc_probe', 'pimesh_perception', 'pimesh_perception::IpcProbe'),
    # tools/gates/depth.sh: the depth rate on its own steady clock, and whether
    # /depth/rgb is byte-identical to the frame each depth map was inferred on.
    ('depth_probe', 'pimesh_perception', 'pimesh_perception::DepthProbe'),
    # tools/gates/odom.sh: the trajectory as published, off /odom — the path
    # length, the net displacement, and above all the largest single step, which
    # is the number a mean hides.
    ('odom_probe', 'pimesh_perception', 'pimesh_perception::OdomProbe'),
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


def _component(
        name: str, package: str, plugin: str, params_path: str, extra: list
) -> ComposableNode:
    """One entry in the container's list.

    The parameters are the keyed YAML first and a single override second, and the
    order is load-bearing: rclcpp applies them in sequence, so the launch argument
    wins over the file while the file stays the one place the defaults live.

    The overrides are here at all because `ros2 launch` has no way to set one
    node's parameter from the command line — it parses its own arguments and stops
    — so a parameter a gate needs to flip has to be reachable through a launch
    argument. Both of these belong to one node and are harmless on the others: a
    node that never declared a parameter ignores it.

    `log_payloads` is decode_node's, for tools/gates/ipc.sh. `use_cuda` is
    depth_node's, and it is **the control run** tools/gates/depth.sh needs rather
    than a fallback anyone should choose: an 80 ms per-frame budget that the CPU
    path has never been watched to fail is a threshold nobody has seen exclude
    anything. `duration_s` is depth_probe's measuring window, which that gate sets
    from the clip's own metadata so that the window and the clip are the same
    seconds — comparing a 20 s window of a 60 s clip against that clip's average
    is a mistake this project has already made once, in P3, and it moved the
    answer further than a real regression would have.

    Note `value_type=` on every one of them. A `LaunchConfiguration` is a string,
    and the raw substitution would set a *string* parameter of the same name, which
    the node ignores while saying nothing at all.
    """
    return ComposableNode(
        package=package,
        plugin=plugin,
        name=name,
        parameters=[
            params_path,
            {
                'log_payloads': ParameterValue(
                    LaunchConfiguration('log_payloads'), value_type=bool),
                'use_cuda': ParameterValue(
                    LaunchConfiguration('use_cuda'), value_type=bool),
                'duration_s': ParameterValue(
                    LaunchConfiguration('probe_duration_s'), value_type=float),
                'align': ParameterValue(
                    LaunchConfiguration('align'), value_type=bool),
                'remesh_period_s': ParameterValue(
                    LaunchConfiguration('remesh_period_s'), value_type=float),
                'odometry': ParameterValue(
                    LaunchConfiguration('odom_regime'), value_type=str),
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

    # **Off by default, and that is the same reasoning as `probe`.** A dashboard
    # is a viewer, and a viewer attached to every run would be in every
    # measurement this workspace takes — it subscribes to two JPEG streams and a
    # 4 MB mesh across a process boundary. `bash tools/dashboard.sh` turns it on,
    # and so does tools/gates/dashboard.sh, which exists to prove that turning it
    # on costs the pipeline nothing.
    dashboards = [
        Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[
                params_path,
                {
                    # Note `value_type=int`. A LaunchConfiguration is a string,
                    # and the raw substitution would set a *string* parameter
                    # named `port`, which the node ignores while saying nothing —
                    # it would then listen on 8080 whatever was asked for, and the
                    # only symptom would be a page that does not load at the
                    # address the script printed.
                    'port': ParameterValue(
                        LaunchConfiguration('dashboard_port'), value_type=int),
                },
            ],
            condition=IfCondition(LaunchConfiguration('dashboard')),
            output='screen',
        )
        for name, package, executable in STANDALONE_NODES
    ]

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
            'pipeline',
            default_value='true',
            description='Compose the dev-box stages into the container. false '
                        'brings up the static frame tree alone, which is what a '
                        'looping bag replay wants: see the note at the top.',
        ),
        DeclareLaunchArgument(
            'probe',
            default_value='none',
            description='Also load one of the gates\' instruments into the '
                        'container, by node name: ipc_probe, depth_probe, or '
                        'none. A name rather than a flag so that two probes '
                        'cannot be measuring at once.',
        ),
        DeclareLaunchArgument(
            'probe_duration_s',
            default_value='60.0',
            description='Seconds depth_probe measures before printing its '
                        'summary. tools/gates/depth.sh sets it from the clip it '
                        'is replaying, so the window and the clip are the same '
                        'seconds.',
        ),
        DeclareLaunchArgument(
            'align',
            default_value='true',
            description="fusion_node's per-frame depth scale alignment. false is "
                        'the control run in tools/gates/fusion.sh.',
        ),
        DeclareLaunchArgument(
            'remesh_period_s',
            default_value='10.0',
            description="How often mesh_node extracts a surface. "
                        'tools/gates/mesh.sh sets it past the clip length for its '
                        'control run, so nothing meshes while the integrator is '
                        'measured.',
        ),
        DeclareLaunchArgument(
            'dashboard',
            default_value='false',
            description='Also start dashboard_node, in its own process, serving '
                        'http://localhost:8080. Off by default because a viewer '
                        'attached to every run would be in every measurement '
                        'this workspace takes.',
        ),
        DeclareLaunchArgument(
            'dashboard_port',
            default_value='8080',
            description='Port the dashboard listens on. tools/gates/hello-clean.sh '
                        'moves it off the default so a teardown test cannot '
                        'collide with a dashboard somebody has open.',
        ),
        DeclareLaunchArgument(
            'odom_regime',
            default_value='sixdof',
            description="keypoint_node's estimator: sixdof fits a rigid "
                        'transform to depth-backed landmarks, rotation_only '
                        'fits bearing rays and publishes zero translation. The '
                        'second is P3\'s estimator, kept as the control run in '
                        'tools/gates/odom.sh — a claim that translation improves '
                        'the surface needs a run without it.',
        ),
        DeclareLaunchArgument(
            'use_cuda',
            default_value='true',
            description="depth_node's execution provider. false forces the CPU, "
                        'which is the control run in tools/gates/depth.sh — a '
                        'budget nobody has watched fail is not an assertion.',
        ),
        *transforms,
        *dashboards,
        ComposableNodeContainer(
            name='pimesh_container',
            namespace='',
            # Everything downstream of capture, or nothing. There is no
            # half-measure here on purpose: the components share one process
            # precisely so that a frame is handed on as a pointer, and a
            # `pipeline:=false` run is asking for the frame tree on its own
            # rather than for a smaller pipeline.
            condition=IfCondition(LaunchConfiguration('pipeline')),
            package='rclcpp_components',
            # `component_container_isolated`, not `component_container_mt`, as of
            # 2026-09-12 — and the change is about two things at once.
            #
            # The occasion was a deprecation: on Lyrical, `_mt` logs "This
            # executable is deprecated and will be removed in M-turtle" at every
            # startup. The reason to prefer the isolated one anyway is the executor
            # model. `_mt` runs every component on **one shared multi-threaded
            # executor**; isolated gives **each component its own executor**, so a
            # callback that costs 76 ms — which is what depth will cost in P4 —
            # cannot delay another node's callbacks at all. One shared pool makes
            # that a question of how many threads happen to be free.
            #
            # Intra-process comms is unaffected: it is a property of the publisher,
            # the subscriber and their sharing a process, not of which executor
            # spins them. That is asserted rather than assumed — tools/gates/ipc.sh
            # was re-run after this change and still reports every address matching
            # with it on and none with it off.
            #
            # Available on both distros (checked 2026-09-12), which is the usual
            # precondition for anything in this workspace.
            executable='component_container_isolated',
            composable_node_descriptions=[
                _component(name, package, plugin, params_path, extra)
                for name, package, plugin in COMPONENTS
            ],
            output='screen',
        ),
        # One load action per probe, each conditional on `probe` naming it.
        #
        # A single action taking a filtered list cannot work: the list would have
        # to be filtered when this file is *evaluated*, which is before any launch
        # argument has a value. `IfCondition` is the mechanism that defers a
        # decision to launch time, and it decides for one action at a time — so
        # there is one action each, and `probe:=none` is simply a value that no
        # condition matches.
        #
        # `PythonExpression` rather than `LaunchConfigurationEquals`: the latter is
        # deprecated on Lyrical, and this workspace takes the spelling that exists
        # at both ends whenever two exist. The repr() is what quotes the name — a
        # value containing a quote would otherwise end the string and be evaluated
        # as Python.
        *[
            LoadComposableNodes(
                target_container='pimesh_container',
                condition=IfCondition(
                    PythonExpression(
                        [repr(name), ' == ', "'", LaunchConfiguration('probe'), "'"])),
                composable_node_descriptions=[
                    _component(name, package, plugin, params_path, extra),
                ],
            )
            for name, package, plugin in PROBE_COMPONENTS
        ],
    ])
