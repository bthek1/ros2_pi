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
    assert keyed == expected


def test_every_composed_component_has_a_config_key(launch_module, config):
    """And the same check from the other side, stated separately because it fails
    differently: a component with no key in the YAML runs entirely on its code
    defaults, which is a node that works and is not configured."""
    for name, _, _plugin in launch_module.COMPONENTS + launch_module.PROBE_COMPONENTS:
        assert f'/**/{name}' in config, f'{name} is composed but has no key in pimesh.yaml'


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

    assert kinds.get(Node) == len(launch_module.STATIC_TRANSFORMS)
    assert kinds.get(ComposableNodeContainer) == 1, 'there is one container, always'
    # intra_process, log_payloads, probe, probe_duration_s, align,
    # remesh_period_s, use_cuda, pipeline — each of which exists because something
    # outside this file has to be able to set it: the first seven for gates, the
    # last for tools/replay.sh.
    assert kinds.get(DeclareLaunchArgument) == 8
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


def test_the_overrides_are_the_ones_the_gates_actually_pass(launch_module):
    """And that they are typed the way the receiving node declared them.

    `use_cuda` is a bool in depth_node, `duration_s` a double in depth_probe,
    `log_payloads` a bool in decode_node, `align` a bool in fusion_node,
    `remesh_period_s` a double in mesh_node. A
    `value_type` that disagrees with the declaration is the same silent no-op as
    having none.
    """
    expected = {
        'log_payloads': bool, 'use_cuda': bool, 'duration_s': float, 'align': bool,
        'remesh_period_s': float}
    overrides = _override_values(launch_module)

    assert set(overrides) == set(expected), (
        'the override set changed; update the gates that depend on it '
        '(tools/gates/ipc.sh, tools/gates/depth.sh, tools/gates/fusion.sh, '
        'tools/gates/mesh.sh) and this test together')
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
