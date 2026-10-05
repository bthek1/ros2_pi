"""config/views.yaml against the files it names and the launch file it feeds.

Every check here is a mistake that does not announce itself. `view.launch.py`
refuses an unknown view and a missing model loudly; what it cannot refuse is a
view that is *well-formed and wrong*:

- a `launch:` key `pimesh.launch.py` does not declare is set as a launch
  configuration nobody reads, and the view runs on the default it meant to change;
- a YAML `true` arrives as the Python bool, `str()` makes it `'True'`, and whether
  that parses depends on which consumer reads it;
- a looping view without `pipeline: 'false'` freezes the pose at the first wrap and
  floods every TF listener — it looks like RViz being slow;
- `view.launch.py` redeclaring an argument the include already declares gives one
  value two defaults, and the outer one wins silently;
- a recipe naming a view the table does not have, or a view with no heading in
  docs/info/viewers.md, which is the page the launch's one log line points to.

Read as text and YAML, no launch service: the claims are about the files.
"""

import os
import re

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, '..')
_WS = os.path.join(_PKG, '..', '..')
_VIEWS = os.path.join(_PKG, 'config', 'views.yaml')
_VIEW_LAUNCH = os.path.join(_PKG, 'launch', 'view.launch.py')
_PIMESH_LAUNCH = os.path.join(_PKG, 'launch', 'pimesh.launch.py')
_JUSTFILE = os.path.join(_WS, 'justfile')
_VIEWERS_DOC = os.path.join(_WS, 'docs', 'info', 'viewers.md')

FIELDS = {'rviz', 'rviz_delay_s', 'needs_model', 'needs_bag', 'loop', 'launch'}

_DECLARED = re.compile(r"DeclareLaunchArgument\(\s*'([a-z_]+)'")


def _declared(path):
    with open(path) as fh:
        return _DECLARED.findall(fh.read())


@pytest.fixture(scope='module')
def views():
    with open(_VIEWS) as fh:
        return yaml.safe_load(fh)


def test_the_table_is_not_empty(views):
    # Every other test iterates it; an empty table passes all of them.
    assert views, 'config/views.yaml has no views'


def test_every_view_has_exactly_the_known_fields(views):
    for name, view in views.items():
        assert set(view) == FIELDS, (
            f'view {name}: fields {sorted(view)}, expected {sorted(FIELDS)} — '
            'a misspelled field is read as absent and its default applies')


def test_every_rviz_config_exists(views):
    for name, view in views.items():
        if view['rviz'] is None:
            continue
        path = os.path.join(_PKG, 'rviz', view['rviz'])
        assert os.path.isfile(path), f'view {name} names {view["rviz"]}, not in rviz/'


def test_every_launch_key_is_an_argument_pimesh_launch_declares(views):
    declared = set(_declared(_PIMESH_LAUNCH))
    assert len(declared) > 20, f'found {len(declared)} declared arguments — the regex is broken'
    for name, view in views.items():
        for key in view['launch']:
            assert key in declared, (
                f'view {name} sets {key}, which pimesh.launch.py does not declare: '
                'it would be set and read by nothing')


def test_every_launch_value_is_a_string(views):
    for name, view in views.items():
        for key, value in view['launch'].items():
            assert isinstance(value, str), (
                f"view {name}: {key}: {value!r} is a YAML {type(value).__name__}; "
                f"quote it ('{str(value).lower()}') so the launch sees what the file says")


def test_a_looping_view_brings_up_no_pipeline(views):
    for name, view in views.items():
        if view['loop']:
            assert view['launch'].get('pipeline') == 'false', (
                f'view {name} loops its bag under a live pipeline: the pose freezes at '
                'the first wrap and every TF listener floods')
            assert view['needs_bag'], f'view {name} loops a bag it does not require'


def test_view_launch_redeclares_nothing_the_include_declares():
    ours = set(_declared(_VIEW_LAUNCH))
    assert ours, 'view.launch.py declares nothing — the regex is broken'
    both = ours & set(_declared(_PIMESH_LAUNCH))
    assert not both, (
        f'view.launch.py redeclares {sorted(both)}: one value, two defaults, and the '
        'outer one wins without a word')


def test_every_recipe_names_a_view_that_exists_and_every_view_has_one(views):
    with open(_JUSTFILE) as fh:
        named = set(re.findall(r'view\.launch\.py view:=([a-z_]+)', fh.read()))
    assert named, 'no recipe runs view.launch.py — the regex is broken'
    assert named <= set(views), f'recipes name views {sorted(named - set(views))} the table lacks'
    assert set(views) <= named, f'views {sorted(set(views) - named)} have no recipe'


def test_every_view_has_a_heading_in_the_viewers_page(views):
    with open(_VIEWERS_DOC) as fh:
        headings = set(re.findall(r'^## (\S+)', fh.read(), re.M))
    for name in views:
        assert f'view-{name}' in headings, (
            f'docs/info/viewers.md has no "## view-{name}", and the launch points there')


# --- record.launch.py's after-recording check ----------------------------------
#
# The refusal a recording ends on, and the summary it prints. A version that
# summarised a directory without metadata.yaml would print a clip that was never
# finalised as though it were one; a rate taken over /camera_info instead of the
# image topic reads as a camera running at 1 Hz.

def _record_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'record_launch', os.path.join(_PKG, 'launch', 'record.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fake_bag(tmp_path, images=1200, infos=1200, seconds=20.0):
    bag = tmp_path / 'clip'
    bag.mkdir()
    meta = {'rosbag2_bagfile_information': {
        'duration': {'nanoseconds': int(seconds * 1e9)},
        'topics_with_message_count': [
            {'topic_metadata': {'name': '/camera_info'}, 'message_count': infos},
            {'topic_metadata': {'name': '/image_raw/compressed'}, 'message_count': images},
        ]}}
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(meta))
    (bag / 'clip_0.mcap').write_bytes(b'abc')
    return bag


def test_a_recording_without_metadata_is_refused(tmp_path):
    bag = tmp_path / 'unfinished'
    bag.mkdir()
    (bag / 'unfinished_0.mcap').write_bytes(b'')
    with pytest.raises(RuntimeError, match='metadata.yaml'):
        _record_module().summarise(bag)


def test_the_summary_rates_the_image_topic(tmp_path):
    lines = _record_module().summarise(_fake_bag(tmp_path, images=1200, infos=20))
    rate = [line for line in lines if 'image rate' in line]
    assert rate == ['  image rate                   60.0 Hz'], lines
    # sha256 of b'abc', the published reference vector — not one computed beside it.
    assert any('ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad' in line
               for line in lines), lines
