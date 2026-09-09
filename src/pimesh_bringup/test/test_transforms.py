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


def test_config_has_no_transforms_the_launch_file_ignores(config, launch_module):
    """The other direction, which is the one that rots quietly: an entry nobody
    publishes looks exactly like an entry somebody does."""
    keyed = {k[len('/**/'):] for k in config if k.startswith('/**/')}
    assert keyed == set(launch_module.STATIC_TRANSFORMS)


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
    description = launch_module.generate_launch_description()
    actions = description.entities

    # Three transform publishers and one container.
    assert len(actions) == len(launch_module.STATIC_TRANSFORMS) + 1
