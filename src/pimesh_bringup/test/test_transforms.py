"""The static transforms, and the launch file that turns them into arguments.

`config/pimesh.yaml` is the only place in this project where geometry is written
down as literal numbers rather than computed, and nothing else checks it. A
mistyped quaternion still loads, still publishes, and still shows three frames
in RViz — it just puts every unprojected ray somewhere wrong, and the first
symptom is a mesh that does not look like the room, three milestones later.

These run under `colcon test` on both machines and need no hardware.
"""

import importlib.util
import math
import os

import pytest
import yaml

_HERE = os.path.dirname(__file__)
_CONFIG = os.path.join(_HERE, '..', 'config', 'pimesh.yaml')
_LAUNCH = os.path.join(_HERE, '..', 'launch', 'pimesh.launch.py')


def _load_launch_module():
    """Import launch/pimesh.launch.py by path.

    It is not on the Python path and is not meant to be — it is installed into
    share/, not onto sys.path — so a test that wants at its helpers has to go
    and get it. Doing that is worth it: the alternative is asserting on the
    YAML alone and never executing the code that reads it.
    """
    spec = importlib.util.spec_from_file_location('pimesh_launch', _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def config():
    with open(_CONFIG) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope='module')
def launch_module():
    return _load_launch_module()


def _entries(config, launch_module):
    return {
        name: config[f'/**/{name}']['ros__parameters']
        for name in launch_module.STATIC_TRANSFORMS
    }


def _rotate(q, v):
    """Rotate vector `v` by quaternion `q = (x, y, z, w)`.

    v' = v + 2 * qv x (qv x v + w * v), the standard form. Written out rather
    than pulled from a dependency: ten lines here is cheaper than making the
    Pi's test run depend on numpy, and the formula is the thing being trusted.
    """
    qx, qy, qz, qw = q

    def cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    qv = (qx, qy, qz)
    t = cross(qv, (v[0] + 0.0, v[1] + 0.0, v[2] + 0.0))
    t = (t[0] + qw * v[0], t[1] + qw * v[1], t[2] + qw * v[2])
    t = cross(qv, t)
    return tuple(v[i] + 2.0 * t[i] for i in range(3))


# --- The config and the launch file agree -----------------------------------

def test_every_transform_the_launch_file_wants_is_in_the_config(config, launch_module):
    """A key that matches no node is not an error in ROS 2 — the file loads and
    the parameter is silently never applied. Here it would be worse than silent:
    the launch file reads the YAML itself, so a renamed key is a KeyError at
    launch time. This test is what turns that into a test failure instead of a
    failed session."""
    for name in launch_module.STATIC_TRANSFORMS:
        assert f'/**/{name}' in config, f'launch wants {name}, config/pimesh.yaml has no such key'
        assert 'ros__parameters' in config[f'/**/{name}']


def test_config_has_no_keys_the_launch_file_ignores(config, launch_module):
    """The other direction, which is the one that rots quietly: a key nobody reads
    looks exactly like a key somebody does.

    A ROS 2 parameter file is not validated against anything — a key matching no
    node loads fine and the node runs on its code defaults — so an orphan here is
    invisible at runtime. It is how a parameter gets tuned for an hour in the wrong
    place. Every `/**/` key therefore has to be either a transform the launch file
    publishes or a component it composes."""
    keyed = {k[len('/**/'):] for k in config if k.startswith('/**/')}
    expected = set(launch_module.STATIC_TRANSFORMS)
    expected |= {name for name, _, _plugin in launch_module.COMPONENTS}
    expected |= {name for name, _, _plugin in launch_module.PROBE_COMPONENTS}
    expected |= {name for name, _, _exe in launch_module.STANDALONE_NODES}
    assert keyed == expected


def test_every_composed_component_has_a_config_key(launch_module, config):
    """And the same check from the other side, stated separately because it fails
    differently: a component with no key in the YAML runs entirely on its code
    defaults, which is a node that works and is not configured."""
    for name, _, _plugin in launch_module.COMPONENTS + launch_module.PROBE_COMPONENTS:
        assert f'/**/{name}' in config, f'{name} is composed but has no key in pimesh.yaml'
    for name, _, _exe in launch_module.STANDALONE_NODES:
        assert f'/**/{name}' in config, f'{name} is launched but has no key in pimesh.yaml'


def test_every_plugin_string_is_registered_in_the_ament_index(launch_module):
    """The plugin string is resolved at *runtime*, through the ament index, so a
    typo in it is not a build error — it is a container that comes up and then
    fails to load a component, with the rest of the launch carrying on around it.

    This is the cheapest check that catches it, and it is checking the real
    index: the same resource file the container reads, written by
    `rclcpp_components_register_nodes` at build time.

    **Each component is looked up in the index of the package the launch file
    names for it**, not in one fixed package's. The container does exactly that,
    and since P5 there are two packages supplying components — so a check against
    a single index would either pass over half of them or report a class as
    missing when it is merely elsewhere."""
    from ament_index_python.packages import get_package_prefix

    def registered_in(package):
        prefix = get_package_prefix(package)
        resource = os.path.join(prefix, 'share', 'ament_index', 'resource_index',
                                'rclcpp_components', package)
        assert os.path.isfile(resource), (
            f'{package} registers no components at all — is '
            f'rclcpp_components_register_nodes missing from its CMakeLists?')
        found = set()
        with open(resource) as handle:
            for line in handle:
                if line.strip():
                    # Each line is "<class>;<library path>".
                    found.add(line.split(';')[0].strip())
        return found

    for name, package, plugin in launch_module.COMPONENTS + launch_module.PROBE_COMPONENTS:
        registered = registered_in(package)
        assert plugin in registered, (
            f'{name} names {plugin}, which {package} does not register '
            f'(it registers {sorted(registered)})')


# --- The numbers are valid rotations ----------------------------------------

def test_quaternions_are_unit_norm(config, launch_module):
    """A non-unit quaternion is not a rotation: it scales as well as rotates, so
    the transform is no longer rigid and distances change as they cross the
    frame. tf2 will publish it without complaint."""
    for name, entry in _entries(config, launch_module).items():
        r = entry['rotation']
        norm = math.sqrt(r['x'] ** 2 + r['y'] ** 2 + r['z'] ** 2 + r['w'] ** 2)
        assert norm == pytest.approx(1.0, abs=1e-9), f'{name} quaternion has norm {norm}'


def test_the_frame_chain_is_the_one_rep_105_asks_for(config, launch_module):
    entries = _entries(config, launch_module)
    edges = {(e['frame_id'], e['child_frame_id']) for e in entries.values()}
    assert edges == {
        ('map', 'odom'),
        ('base_link', 'camera_link'),
        ('camera_link', 'camera_optical_frame'),
    }


def test_exactly_one_publisher_per_edge(config, launch_module):
    """Two publishers on one edge is the failure that makes a mesh smear and is
    not obvious from any single log. Here it would mean two entries with the
    same child, so it is cheap to rule out."""
    children = [e['child_frame_id'] for e in _entries(config, launch_module).values()]
    assert len(children) == len(set(children)), f'an edge is published twice: {children}'


# --- The optical rotation is the *right* rotation ---------------------------

def test_camera_to_optical_maps_optical_axes_onto_the_body_convention(config, launch_module):
    """The whole point of the camera_link -> camera_optical_frame edge.

    `camera_link` is REP-103 body convention: x forward, y left, z up.
    `camera_optical_frame` is the optical convention: z forward, x right,
    y down. The published rotation takes vectors expressed in the child
    (optical) frame into the parent (body) frame, so:

        optical z (forward) -> body  x  (forward)
        optical x (right)   -> body -y  (right, since body y is left)
        optical y (down)    -> body -z  (down, since body z is up)

    (-0.5, 0.5, -0.5, 0.5) is the standard value and it is easy to write down
    with a sign flipped — which produces a tree that still looks like three
    frames in RViz and puts every ray 90 degrees out.
    """
    entry = _entries(config, launch_module)['camera_to_optical']
    r = entry['rotation']
    q = (r['x'], r['y'], r['z'], r['w'])

    forward = _rotate(q, (0.0, 0.0, 1.0))
    right = _rotate(q, (1.0, 0.0, 0.0))
    down = _rotate(q, (0.0, 1.0, 0.0))

    assert forward == pytest.approx((1.0, 0.0, 0.0), abs=1e-9), 'optical z is not body forward'
    assert right == pytest.approx((0.0, -1.0, 0.0), abs=1e-9), 'optical x is not body right'
    assert down == pytest.approx((0.0, 0.0, -1.0), abs=1e-9), 'optical y is not body down'


def test_the_placeholder_edges_are_actually_identity(config, launch_module):
    """map -> odom stands in for a pose-graph backend that does not exist, and
    base_link -> camera_link for a mount that has not been measured. Both are
    documented as identity; a number quietly appearing in one of them would be a
    correction nobody knew was being applied."""
    for name in ('map_to_odom', 'base_to_camera'):
        entry = _entries(config, launch_module)[name]
        t = entry['translation']
        r = entry['rotation']
        assert (t['x'], t['y'], t['z']) == (0.0, 0.0, 0.0), f'{name} has a translation'
        assert (r['x'], r['y'], r['z'], r['w']) == (0.0, 0.0, 0.0, 1.0), f'{name} has a rotation'


# --- The conversion into the tool's actual interface ------------------------

def test_static_transform_args_emit_flags_not_parameters(config, launch_module):
    """`static_transform_publisher` declares frame_id and translation.x as ROS
    parameters and never reads them: its main() parses argv first and exits
    non-zero with "Frame id must not be empty". Passing parameters=[...] gives
    three dead processes and a launch that carries on with no TF tree, which
    reads as an RViz problem. This asserts the launch file emits the flag form
    that actually works, with every value present."""
    entry = _entries(config, launch_module)['camera_to_optical']
    args = launch_module._static_transform_args(entry)

    assert len(args) % 2 == 0, 'arguments must be flag/value pairs'
    flags = dict(zip(args[::2], args[1::2]))

    assert set(flags) == {
        '--frame-id', '--child-frame-id',
        '--x', '--y', '--z',
        '--qx', '--qy', '--qz', '--qw',
    }
    assert flags['--frame-id'] == 'camera_link'
    assert flags['--child-frame-id'] == 'camera_optical_frame'
    assert float(flags['--qx']) == pytest.approx(-0.5)
    assert float(flags['--qw']) == pytest.approx(0.5)
    # Every value has to survive as a string the tool can parse back.
    for flag, value in flags.items():
        assert value != '', f'{flag} is empty'
        if flag.startswith(('--x', '--y', '--z', '--q')):
            float(value)


def test_the_launch_description_actually_builds(launch_module):
    """A smoke test, and it earns its place: an earlier draft of this launch
    file passed `extra_arguments` to `ComposableNodeContainer`, which does not
    take it — `use_intra_process_comms` is a per-*component* option. That is the
    class of mistake that only shows up when somebody runs the thing."""
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node

    description = launch_module.generate_launch_description()
    actions = description.entities

    # Counted by kind rather than totalled, so the failure message says which part
    # of the launch file changed instead of just that the number moved.
    kinds = {}
    for action in actions:
        kinds[type(action)] = kinds.get(type(action), 0) + 1

    # The three static transform publishers plus the standalone nodes. The latter
    # are conditional — `dashboard:=false` by default — but a conditional action
    # is still built into the description; the condition decides at launch time,
    # which is what test_the_dashboard_is_opt_in checks.
    assert kinds.get(Node) == (
        len(launch_module.STATIC_TRANSFORMS) + len(launch_module.STANDALONE_NODES))
    assert kinds.get(ComposableNodeContainer) == 1, 'there is one container, always'
    # intra_process, log_payloads, probe, probe_duration_s, align,
    # remesh_period_s, dashboard, dashboard_port, odom_regime, use_cuda,
    # pipeline — each exists because something outside this file has to be able
    # to set it: the first ten for gates and viewers, the last for
    # tools/replay.sh.
    assert kinds.get(DeclareLaunchArgument) == 11
    # One per probe, loaded into the running container rather than listed in it,
    # because `composable_node_descriptions` is built when this file is evaluated
    # and cannot be made conditional on an argument. One action each rather than
    # one action over a filtered list, for the same reason: the filtering would
    # have to happen before `probe` has a value.
    assert kinds.get(LoadComposableNodes) == len(launch_module.PROBE_COMPONENTS)
    assert len(actions) == sum(kinds.values())


def test_at_most_one_probe_loads_and_none_by_default(launch_module):
    """`probe` is a gate's switch, and a probe is a consumer nobody asked for in an
    ordinary session.

    **This evaluates the conditions rather than checking their type**, which the
    earlier version of this test did not, and the difference matters now that there
    are two of them. An `isinstance(..., IfCondition)` assertion is satisfied by a
    condition that is always true, by a condition on the wrong configuration, and
    by two conditions that match the same value — and that last one is exactly the
    failure `probe` was turned from a flag into a name to prevent: a depth
    measurement taken with the IPC probe also subscribed, with nothing in the
    output saying so.

    So: for every value `probe` can take, count how many loads actually fire."""
    from launch import LaunchContext
    from launch_ros.actions import LoadComposableNodes

    description = launch_module.generate_launch_description()
    loads = [a for a in description.entities if isinstance(a, LoadComposableNodes)]
    assert len(loads) == len(launch_module.PROBE_COMPONENTS)

    names = [name for name, _, _plugin in launch_module.PROBE_COMPONENTS]
    for value, expected in [('none', 0), *[(n, 1) for n in names]]:
        context = LaunchContext()
        context.launch_configurations['probe'] = value
        fired = sum(
            1 for load in loads
            if load.condition is not None and load.condition.evaluate(context)
        )
        assert fired == expected, (
            f'probe:={value} loads {fired} probe(s), expected {expected} — '
            'two probes measuring at once is the thing this argument prevents'
        )


def test_the_dashboard_is_opt_in_and_runs_outside_the_container(launch_module):
    """Two claims, and both matter for a different reason.

    **Opt-in**, because a dashboard is a viewer: it subscribes to two JPEG
    streams and a 4 MB mesh across a process boundary, and one attached to every
    run would be inside every measurement this workspace takes — including the
    gate whose whole assertion is what attaching one costs.

    **Outside the container**, because `docs/info/dashboard.md` makes a promise
    the container cannot keep: *it must be able to die*. A component there would
    take the TSDF with it. Everything else on the dev box is composed precisely
    so a frame is handed on as a pointer; this node has nothing to gain from that
    and a crash to cost, so it is the one exception and the test is what stops it
    quietly becoming the rule.
    """
    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer, Node

    description = launch_module.generate_launch_description()
    names = {name for name, _, _exe in launch_module.STANDALONE_NODES}
    assert names, 'there is at least one standalone node; this test is about it'

    composed = {name for name, _, _plugin in launch_module.COMPONENTS}
    assert not (names & composed), (
        'a node cannot be both composed into the container and launched beside it')

    # Identified by *having a condition*, which the three static transform
    # publishers do not: a launch_ros Node does not expose its name before it has
    # been executed in a context, so reaching for a private attribute here would
    # be a test coupled to launch_ros internals rather than to this file.
    plain = [
        a for a in description.entities
        if isinstance(a, Node) and not isinstance(a, ComposableNodeContainer)
    ]
    unconditional = [a for a in plain if a.condition is None]
    standalone = [a for a in plain if a.condition is not None]
    assert len(unconditional) == len(launch_module.STATIC_TRANSFORMS), (
        'the frame tree is not optional')
    assert len(standalone) == len(names), (
        f'expected {len(names)} conditional standalone node action(s), got {len(standalone)}')

    for action in standalone:
        off = LaunchContext()
        off.launch_configurations['dashboard'] = 'false'
        assert not action.condition.evaluate(off), 'the default must not start a viewer'
        on = LaunchContext()
        on.launch_configurations['dashboard'] = 'true'
        assert action.condition.evaluate(on), 'dashboard:=true has to start it'


def test_the_dashboard_reads_the_topics_the_pipeline_publishes(config):
    """The panel draws what other nodes published, so the names have to match.

    **The failure is a page that loads.** A wrong topic name here is not an error
    anywhere: the dashboard subscribes to something nobody publishes, the row
    goes STALE after two seconds, and it looks exactly like the stage having
    stopped. That is the same shape as every other cross-key pair in this file,
    with the added sting that the dashboard is *where somebody would go to find
    out what stopped*.
    """
    dash = config['/**/dashboard_node']['ros__parameters']
    keypoints = config['/**/keypoint_node']['ros__parameters']
    depth = config['/**/depth_node']['ros__parameters']
    mesh = config['/**/mesh_node']['ros__parameters']

    assert dash['rgb_topic'] == '/keypoints/image/compressed'
    assert dash['depth_topic'] == depth['preview_topic']
    assert dash['mesh_topic'] == mesh['mesh_topic']
    assert dash['stats_topic'] == '/pipeline/stats'
    assert dash['odom_topic'] == '/odom'
    # The two image strips are the colour-mapped previews, not the raw topics: a
    # browser cannot map a 3.7 MB float depth image, and a fixed [0, max_range]
    # scale is what makes a colour mean a distance across frames.
    assert keypoints['preview_rate_hz'] > 0
    assert depth['preview_rate_hz'] > 0, (
        'depth_node is configured not to publish a preview, so the dashboard would '
        'show an empty panel that looks like a stalled GPU')
    # The dashboard may not ask for frames faster than they are produced, or the
    # cap it applies is not a cap at all.
    assert dash['rgb_rate_hz'] <= keypoints['preview_rate_hz']
    assert dash['depth_rate_hz'] <= depth['preview_rate_hz']


def test_the_container_can_be_left_out_but_is_there_by_default(launch_module):
    """`pipeline:=false` has to actually remove the container, and the default
    has to keep it.

    This is the assertion tools/replay.sh stands on. A looping bag replays header
    stamps ~60 s into the past at every wrap, keypoint_node stamps
    `odom -> base_link` with the frame's own stamp, and tf2 rejects every
    transform older than the newest it holds — so a replay that composes the
    pipeline freezes that edge after one pass and logs TF_OLD_DATA at the frame
    rate from inside the buffer's own mutex, which stalls every listener's
    lookups including RViz's render loop. Measured 2026-09-13.

    A condition that silently evaluated true would put that back without a word,
    which is why this evaluates it rather than only checking one is attached.
    """
    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer

    description = launch_module.generate_launch_description()
    containers = [
        a for a in description.entities if isinstance(a, ComposableNodeContainer)
    ]
    assert len(containers) == 1
    condition = containers[0].condition
    assert condition is not None, (
        'the container is unconditional — `pipeline:=false` would compose it anyway'
    )

    for value, expected in (('true', True), ('false', False), ('1', True), ('0', False)):
        context = LaunchContext()
        context.launch_configurations['pipeline'] = value
        assert condition.evaluate(context) is expected, f'pipeline:={value}'


# --- The parameter overrides the gates flip ----------------------------------
#
# `_component` threads three launch arguments into every component's parameter
# list, because `ros2 launch` has no way to set one node's parameter from the
# command line. Each belongs to one node and is ignored by the others.

def _override_values(launch_module):
    """The override dict `_component` builds, as {name: ParameterValue}.

    `ComposableNode` does not keep the dict as written: it normalises every key
    into a tuple of `TextSubstitution`, because a parameter *name* may itself be a
    substitution. So the keys are joined back into plain strings here. The values
    are left exactly as they are — whether they are `ParameterValue` at all is the
    thing being tested.
    """
    component = launch_module._component(
        'depth_node', 'pimesh_perception', 'x::Y', '/tmp/params.yaml', [])
    overrides = [p for p in component.parameters if isinstance(p, dict)]
    assert len(overrides) == 1, 'the override dict is the second parameters entry'

    def name_of(key):
        if isinstance(key, str):
            return key
        return ''.join(part.text for part in key)

    return {name_of(key): value for key, value in overrides[0].items()}


def test_every_launch_argument_override_declares_a_value_type(launch_module):
    """A `LaunchConfiguration` is a **string**, and a parameter set from the raw
    substitution is a string parameter — which a node that declared a bool or a
    double ignores, silently, while the launch reports nothing wrong.

    This is not hypothetical here. The same mistake with `use_intra_process_comms`
    is written up at the top of the launch file: the components load, every frame
    is serialised, and no log line mentions it. The cost of finding it was a gate
    that measured the wrong thing.

    So: every value in that override dict has to be a `ParameterValue` carrying an
    explicit `value_type`. The test is over the dict rather than over three names,
    so a fourth override added later is covered without anyone remembering to.
    """
    from launch_ros.descriptions import ParameterValue

    overrides = _override_values(launch_module)
    assert overrides, 'no overrides at all — has _component stopped threading them?'

    for name, value in overrides.items():
        assert isinstance(value, ParameterValue), (
            f'{name} is a raw substitution, so it would be set as a string '
            f'parameter and ignored by any node expecting another type')
        assert value.value_type is not None, (
            f'{name} has no value_type, so it resolves to a string')


def test_the_standalone_nodes_type_their_overrides_too(launch_module):
    """The same trap as the component overrides, one action type over.

    `dashboard_port` is threaded into the dashboard's `port` parameter, and a raw
    `LaunchConfiguration` there would set a *string* parameter of that name —
    which the node ignores in silence. It would then listen on 8080 whatever was
    asked for, and the only symptom would be a page that does not load at the
    address the script just printed.

    Written separately from the component test because the two build their
    overrides in different places, and a check that only covered one of them
    would have looked like it covered both.
    """
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch_ros.descriptions import ParameterValue

    description = launch_module.generate_launch_description()
    standalone = [
        a for a in description.entities
        if isinstance(a, Node) and not isinstance(a, ComposableNodeContainer) and
        a.condition is not None
    ]
    assert standalone, 'there is at least one standalone node'

    found = 0
    for action in standalone:
        for entry in action._Node__parameters or []:
            if not isinstance(entry, dict):
                continue
            for name, value in entry.items():
                found += 1
                assert isinstance(value, ParameterValue), (
                    f'{name} is a raw substitution, so it would be set as a string')
    assert found >= 1, 'no typed overrides on any standalone node'


def test_the_overrides_are_the_ones_the_gates_actually_pass(launch_module):
    """And that they are typed the way the receiving node declared them.

    `use_cuda` is a bool in depth_node, `duration_s` a double in depth_probe,
    `log_payloads` a bool in decode_node, `align` a bool in fusion_node,
    `remesh_period_s` a double in mesh_node, `odometry` a string in odometry_node.
    A `value_type` that disagrees with the declaration is the same silent no-op as
    having none.
    """
    expected = {
        'log_payloads': bool, 'use_cuda': bool, 'duration_s': float, 'align': bool,
        'remesh_period_s': float, 'odometry': str}
    overrides = _override_values(launch_module)

    assert set(overrides) == set(expected), (
        'the override set changed; update the gates that depend on it '
        '(tools/gates/ipc.sh, tools/gates/depth.sh, tools/gates/fusion.sh, '
        'tools/gates/mesh.sh, tools/gates/odom.sh) and this test together')
    for name, want in expected.items():
        assert overrides[name].value_type is want, (
            f'{name} is declared {want.__name__} by its node')


# --- The probe's config has to describe the pipeline it is measuring ---------

def test_depth_probe_watches_the_topics_the_pipeline_publishes(config):
    """`depth_probe` names its three topics itself, and if any of them drifts from
    what the pipeline publishes the probe simply measures nothing.

    That is not a loud failure. `tools/gates/depth.sh` would report
    `measured=0` and fail — which is the good case — but a drift in `source_topic`
    alone would leave the rate and the depth values intact while silently turning
    every `/depth/rgb` comparison into `rgb_unmatched`, the counter that means
    "could not check". The gate treats that as a failure precisely because it
    would otherwise look like a clean run with nothing to report.
    """
    decode = config['/**/decode_node']['ros__parameters']
    depth = config['/**/depth_node']['ros__parameters']
    probe = config['/**/depth_probe']['ros__parameters']

    assert probe['source_topic'] == decode['output_topic'], (
        'the probe is hashing a topic decode_node does not publish')
    assert probe['depth_topic'] == depth['depth_topic']
    assert probe['rgb_topic'] == depth['rgb_topic']
    assert depth['input_topic'] == decode['output_topic'], (
        'depth_node is subscribed to a topic nothing in this container publishes')


def test_the_probe_and_the_node_agree_on_the_clip(config):
    """`max_range_m` is written twice — once as the distance depth_node clips at,
    once as the bound depth_probe checks every sampled distance against.

    A comment in the YAML says they must match. This is that comment as an
    assertion, which is the difference between a rule and a hope: if the node's
    clip were raised and the probe's left behind, every frame would be reported as
    `out_of_range` and the gate would fail against perfectly good depth. If it
    went the other way the range check would pass over distances beyond the clip —
    a check that cannot fail, which is worse.
    """
    depth = config['/**/depth_node']['ros__parameters']
    probe = config['/**/depth_probe']['ros__parameters']
    assert probe['max_range_m'] == depth['max_range_m']


def test_depth_publishes_into_the_frame_the_static_tree_defines(config):
    """`depth_node` stamps its output `camera_optical_frame` by *name*, and the
    edge that defines that frame is published by `camera_to_optical`.

    The node is explicitly forbidden from re-deriving the optical rotation — it is
    a static edge, unit-tested above to actually be the optical convention. Naming
    a frame that nothing publishes would put the depth cloud nowhere, which in
    RViz looks exactly like a display that is switched off.
    """
    optical = config['/**/camera_to_optical']['ros__parameters']['child_frame_id']
    assert config['/**/depth_node']['ros__parameters']['optical_frame'] == optical


# --- The map's two nodes have to agree with each other and with depth ---------
#
# Every check below is a pair of YAML keys that must hold the same value, and
# every way of getting one wrong produces a pipeline that runs. There is no
# schema anywhere that relates them: they are two strings in a file, and the only
# thing that notices they disagree is this.

def test_fusion_subscribes_to_what_depth_publishes(config):
    """`depth_node` publishes the depth map and the exact colour frame it was
    inferred on; `fusion_node` pairs them **by stamp with no tolerance**.

    If either topic name drifts, fusion subscribes to something nobody publishes
    and integrates nothing at all — and an empty volume looks exactly like a
    camera that was never swept round the room. There is no error anywhere: a
    subscription to an unpublished topic is a perfectly ordinary thing to have.
    """
    depth = config['/**/depth_node']['ros__parameters']
    fusion = config['/**/fusion_node']['ros__parameters']
    assert fusion['depth_topic'] == depth['depth_topic']
    assert fusion['rgb_topic'] == depth['rgb_topic']
    assert depth['publish_rgb'] is True, (
        'depth_node is configured not to publish the colour twin, so every frame '
        'fusion integrates would be colourless and the mesh would come out grey')


def test_fusion_and_depth_agree_on_where_the_far_clip_is(config):
    """`max_range_m` is the model's "far away or no idea", and both nodes need the
    same number for it.

    Set fusion's higher than depth's and nothing happens, because no reading ever
    arrives past depth's clip. Set it *lower* and the far part of every room is
    silently discarded — a map that stops at four metres in a six-metre room, with
    a clean edge that looks like the end of what the camera saw.
    """
    depth = config['/**/depth_node']['ros__parameters']
    fusion = config['/**/fusion_node']['ros__parameters']
    assert fusion['max_range_m'] == depth['max_range_m'], (
        'fusion clips at {} m and depth at {} m'.format(
            fusion['max_range_m'], depth['max_range_m']))


def test_keypoints_reads_the_depth_topic_depth_publishes(config):
    """P7's estimator needs a depth map, and it finds one by name.

    **The failure is a pipeline that runs.** A mismatch here does not error, does
    not warn at startup and does not stop a single other stage: `keypoint_node`
    simply never sees a depth frame, holds its pose for the whole session, and
    publishes an `odom -> base_link` that never moves. That is indistinguishable
    from a camera sitting still — and the mesh it produces is exactly the
    rotation-only mesh milestone D already had, so even the surface looks
    unsurprising.

    The node does log a warning once per stats window when it is in `sixdof` and
    nothing has arrived, which is the runtime half of this; this is the half that
    runs with no hardware.
    """
    depth = config['/**/depth_node']['ros__parameters']
    odometry = config['/**/odometry_node']['ros__parameters']
    assert odometry['depth_topic'] == depth['depth_topic'], (
        'odometry_node reads {} and depth_node publishes {}'.format(
            odometry['depth_topic'], depth['depth_topic']))


def test_keypoints_and_depth_agree_on_where_the_far_clip_is(config):
    """The same pair as fusion's `max_range_m`, one stage over.

    Past the clip a reading is the model's "far away or no idea". A landmark
    placed there is a landmark on a surface that does not exist, and unlike a
    voxel it is not merely wrong in the map — it goes into the rigid fit and drags
    the *trajectory* with it. Set this higher than depth's clip and every frame
    contributes a cluster of landmarks at exactly 6 m that appear not to move.
    """
    depth = config['/**/depth_node']['ros__parameters']
    odometry = config['/**/odometry_node']['ros__parameters']
    assert odometry['max_depth_m'] == depth['max_range_m'], (
        'odometry_node accepts landmarks to {} m and depth clips at {} m'.format(
            odometry['max_depth_m'], depth['max_range_m']))
    assert odometry['min_depth_m'] < odometry['max_depth_m']


def test_odometry_reads_the_keypoints_topic_the_detector_publishes(config):
    """The pair the 2026-09-23 split created, and it fails the same quiet way.

    `keypoint_node` publishes corners and `odometry_node` subscribes to them by
    name. Nothing relates the two keys but this assertion: a mismatch leaves a
    node that never sees a corner, holds its pose forever, and publishes an
    `odom -> base_link` that never moves — which is indistinguishable from a
    camera sitting still, and produces exactly the mesh a working rotation-only
    run produces.

    The topic the detector publishes is a string literal in its source rather than
    a parameter, so this is asserted against the literal: making it configurable on
    one side only would be a third way for the two to disagree.
    """
    odometry = config['/**/odometry_node']['ros__parameters']
    assert odometry['keypoints_topic'] == '/keypoints', (
        'odometry_node reads {} and keypoint_node publishes /keypoints'.format(
            odometry['keypoints_topic']))


def test_the_matching_window_spans_the_gap_between_two_depth_frames(config):
    """`sixdof` correspondences come from the tracker's pooled track ids, so the
    window has to reach from one depth-backed frame to the next.

    Depth runs at ~17 Hz against a ~59 Hz camera, so consecutive depth maps are
    about three frames apart. A `match_window` under that leaves the 6-DoF
    estimator with no correspondences at all — and the symptom is the one this
    whole file is about: a node that holds its pose, forever, with nothing in any
    log saying which knob did it.

    Four rather than three, because "about three" is a ratio that moves with the
    room's lighting: the camera drops to ~45 Hz under a manual exposure and depth
    does not, which makes the gap smaller, but a slower GPU or a bigger model
    makes it larger.

    **It spans two nodes since the 2026-09-23 split**, which is what makes it one
    of the pairs this file exists for rather than a bound on one node's own
    config: the window belongs to the detector and the history it has to reach
    across belongs to the estimator, and nothing but this relates them.
    """
    keypoints = config['/**/keypoint_node']['ros__parameters']
    odometry = config['/**/odometry_node']['ros__parameters']
    if odometry['odometry'] != 'sixdof':
        return
    assert keypoints['match_window'] >= 4, (
        'match_window is {} — two depth frames are ~3 camera frames apart'.format(
            keypoints['match_window']))
    assert odometry['history_frames'] > keypoints['match_window'], (
        'odometry_node keeps {} frames of history and keypoint_node matches over '
        '{}'.format(odometry['history_frames'], keypoints['match_window']))


def test_the_odometry_regime_is_one_of_the_two_that_exist(config):
    """A typo here is refused at startup by the node — it throws rather than
    defaulting — so this is the hermetic half of the same check.

    Worth having twice because the loud version only fires on a machine with the
    container running, and the committed default is what every gate and every
    viewer picks up.
    """
    odometry = config['/**/odometry_node']['ros__parameters']
    assert odometry['odometry'] in ('sixdof', 'rotation_only')


def test_the_two_world_nodes_agree_on_the_volume_key(config):
    """**The sharpest of these, because it fails completely and says nothing.**

    `fusion_node` and `mesh_node` share the TSDF through a process-local registry
    keyed by this string — see `pimesh_world/shared_volume.hpp` for why it is
    shared memory rather than a topic. A mismatch is not a partial failure: it is
    two separate volumes, one of which is filled and never meshed and one of which
    is meshed and never filled. `/world/mesh` then stays empty for the life of the
    session, which is indistinguishable from a room nobody has pointed a camera at.
    """
    fusion = config['/**/fusion_node']['ros__parameters']
    mesh = config['/**/mesh_node']['ros__parameters']
    assert fusion['volume_key'] == mesh['volume_key'], (
        "fusion fills '{}' and mesh_node meshes '{}' — two volumes, and the "
        'published surface would be empty forever'.format(
            fusion['volume_key'], mesh['volume_key']))


def test_the_two_world_nodes_agree_on_the_frame_the_map_lives_in(config):
    """The volume is built in one frame and the Marker is stamped in another, and
    they are set separately. Disagreeing puts a correct surface in the wrong place
    — which in RViz is a room that has slid sideways, and looks like drift."""
    fusion = config['/**/fusion_node']['ros__parameters']
    mesh = config['/**/mesh_node']['ros__parameters']
    assert fusion['world_frame'] == mesh['world_frame']


def test_the_map_lives_in_a_frame_the_static_tree_publishes(config):
    """`map` is the root of the tree `map_to_odom` publishes. Naming a frame
    nothing publishes puts the mesh nowhere, which in RViz looks exactly like a
    display that is switched off — the same failure `depth_node`'s frame check
    above guards against, one stage on."""
    root = config['/**/map_to_odom']['ros__parameters']['frame_id']
    optical = config['/**/camera_to_optical']['ros__parameters']['child_frame_id']
    fusion = config['/**/fusion_node']['ros__parameters']
    assert fusion['world_frame'] == root
    assert fusion['optical_frame'] == optical


def test_the_mesher_does_not_mesh_below_the_volumes_own_noise_floor(config):
    """`mesh_min_weight` and `min_weight` answer different questions, and the
    mesher's has to be the stricter one.

    The volume's threshold is the noise floor for *ray-casting*: below it a voxel
    is "I have not confirmed this" and a ray passes through. Meshing asks
    something else — a voxel seen three times is real enough to stop a ray and
    thin enough to be one of the shingles a rotation-only sweep lays down. Set the
    mesher's *below* the volume's and it has no effect at all, because
    `march_cubes` takes the larger of the two; the knob would look configured and
    do nothing, which is this file's recurring theme.
    """
    fusion = config['/**/fusion_node']['ros__parameters']
    mesh = config['/**/mesh_node']['ros__parameters']
    assert mesh['mesh_min_weight'] >= fusion['min_weight'], (
        'mesh_min_weight {} is below the volume floor {}, so it changes '
        'nothing'.format(mesh['mesh_min_weight'], fusion['min_weight']))
    assert mesh['mesh_min_weight'] <= fusion['max_weight'], (
        'mesh_min_weight {} is above max_weight {}, so no voxel can ever reach '
        'it and the surface is empty forever'.format(
            mesh['mesh_min_weight'], fusion['max_weight']))


def test_the_marker_cap_is_a_cap_and_not_a_target(config):
    """The Marker is rebuilt and re-serialised on every publish; the saved PLY is
    not. A cap large enough to never bind would put tens of megabytes on the wire
    every ten seconds, and one small enough to bind on an empty room would
    decimate noise into fewer pieces of noise."""
    mesh = config['/**/mesh_node']['ros__parameters']
    assert 10000 <= mesh['max_triangles'] <= 1000000


# --- Every parameter in the YAML is one a node actually declares --------------

# Every package pimesh.launch.py composes a component from. A second package
# joined this list at P5 and the fixture below silently stopped covering half the
# parameters until it did — which is the very failure that fixture exists to
# catch, one level up.
_COMPONENT_SRC = [
    os.path.join(_HERE, '..', '..', 'pimesh_perception', 'src'),
    os.path.join(_HERE, '..', '..', 'pimesh_world', 'src'),
]


@pytest.fixture(scope='module')
def declared_parameters():
    """Every name passed to `declare_parameter("…"` in the component packages.

    Read out of the source rather than out of a running node, because the whole
    point is to stay hermetic: no container, no GPU, no camera, and it runs on the
    Pi. Every `declare_parameter` in this package takes a string literal, which is
    what makes a regex honest here — and if one ever does not, the assertion below
    fails loudly rather than quietly passing over it.
    """
    import re

    pattern = re.compile(r'declare_parameter\s*(?:<[^>]*>)?\(\s*"([^"]+)"')
    names = set()
    for directory in _COMPONENT_SRC:
        assert os.path.isdir(directory), (
            f'{directory} is missing, so this test would check less than it thinks')
        sources = [f for f in os.listdir(directory) if f.endswith('.cpp')]
        assert sources, f'no .cpp files in {directory} — not looking where it thinks'
        for filename in sources:
            with open(os.path.join(directory, filename)) as handle:
                names |= set(pattern.findall(handle.read()))
    assert names, 'found no declare_parameter calls at all'
    return names


def test_no_parameter_in_the_yaml_is_read_by_nobody(config, launch_module, declared_parameters):
    """**The trap this whole file exists for, one level deeper than it was.**

    `test_config_has_no_keys_the_launch_file_ignores` checks the `/**/<node>` keys.
    Nothing checked the parameter names *underneath* them — and a ROS 2 parameter
    file is not validated against anything, so a key no node declares loads
    cleanly, applies to nothing, and leaves the node on its code default. The file
    looks configured and is not.

    That is not hypothetical: on 2026-09-15 three keys for `depth_node`'s colour
    preview were in this YAML before the node declared any of them, which is a
    perfectly ordinary order to write things in and leaves no trace once it is
    wrong. This test is what makes the two halves have to arrive together.

    Static transform entries are excluded on purpose: `static_transform_publisher`
    parses `argv` and exits before it reads a parameter file at all, which
    `test_static_transform_args_emit_flags_not_parameters` covers separately.
    """
    components = {name for name, _, _plugin in launch_module.COMPONENTS}
    components |= {name for name, _, _plugin in launch_module.PROBE_COMPONENTS}

    orphans = {}
    for key, entry in config.items():
        node = key[len('/**/'):] if key.startswith('/**/') else key
        if node not in components:
            continue
        for parameter in entry['ros__parameters']:
            if parameter not in declared_parameters:
                orphans.setdefault(node, []).append(parameter)

    assert not orphans, (
        f'these parameters are set in config/pimesh.yaml and declared by no node, '
        f'so they silently apply nothing: {orphans}')


def test_the_keypoints_stream_is_deep_at_both_ends():
    """**A queue depth that is a correctness requirement, asserted because nothing
    else would notice it changing.**

    Every image topic in this workspace keeps 1 on purpose: the freshest frame is
    the only one anybody wants, and a backlog is how this pipeline dies.
    `/keypoints` is the exception. `odometry_node` looks each message up by its
    *exact stamp* to pair it with a depth map, and in `rotation_only` composes a
    rotation increment out of **every** one of them — the pairs in each message
    span the detector's previous frame, so the chain only composes correctly if
    none is skipped.

    A RELIABLE writer at `KEEP_LAST(1)` holds only the newest sample for
    retransmission, so a reader one frame behind loses that frame permanently.
    **The failure is silent and it is not a dropped-frame counter anywhere**: it is
    a rotation increment that never happened, and a pose that under-rotates by an
    amount nobody can attribute. Both ends have to be deep, and the two numbers
    live in two files with nothing relating them, which is the `volume_key` shape
    of bug — see CLAUDE.md on pairs that have to hold the same value.

    Read as text, like `test_dashboard_contract`: these are C++ literals in a
    constructor and there is no hermetic way to ask a node what QoS it used
    without standing one up, which would need a ROS graph this suite must not
    have.
    """
    import re

    src = os.path.join(_HERE, '..', '..', 'pimesh_perception', 'src')
    floor = 30

    def depths(filename, variable):
        with open(os.path.join(src, filename)) as handle:
            text = handle.read()
        found = re.findall(
            re.escape(variable) + r'\s*\(\s*rclcpp::KeepLast\s*\(\s*(\d+)\s*\)\s*\)', text)
        assert found, (
            f'{filename} no longer declares {variable} as a KeepLast QoS — this test '
            'cannot see the depth any more and is asserting nothing')
        return [int(n) for n in found]

    publisher = depths('keypoint_node.cpp', 'keypoints_qos')
    subscriber = depths('odometry_node.cpp', 'keypoints_qos')

    for depth in publisher:
        assert depth >= floor, (
            f'keypoint_node publishes /keypoints at KeepLast({depth}); a reader one '
            f'frame behind loses that frame for good (floor {floor})')
    for depth in subscriber:
        assert depth >= floor, (
            f'odometry_node subscribes to /keypoints at KeepLast({depth}); every '
            f'message it misses is a pose update that never happened (floor {floor})')


def test_the_detector_and_the_estimator_agree_on_the_landmark_clip(config):
    """`max_depth_m` bounds which landmarks the pose is fitted to, and
    `match_window` bounds how far back a correspondence can come from. Both belong
    to the tracking front end and they now live in two different nodes' parameter
    blocks, so this is the pair the 2026-09-23 split created alongside
    `keypoints_topic`.

    A window shorter than the gap between two depth frames leaves the estimator
    with no correspondences; that is asserted separately above. What this one adds
    is the other direction: the history has to outlast the window, or a depth map
    arrives to find the ORB output it needs already evicted — which shows up only
    as `depth_lost` climbing in a stats line nobody is reading.
    """
    keypoints = config['/**/keypoint_node']['ros__parameters']
    odometry = config['/**/odometry_node']['ros__parameters']
    assert odometry['history_frames'] >= 4 * keypoints['match_window'], (
        'history_frames={} is not comfortably longer than match_window={}'.format(
            odometry['history_frames'], keypoints['match_window']))
