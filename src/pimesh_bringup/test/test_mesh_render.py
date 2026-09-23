"""The mesh gate's instrument, checked against geometry whose answer is known.

`tools/gates/mesh.sh` closes P6 by pointing at three PNG files and asserting what
fraction of each is surface. **That number is only worth something if the thing
drawing it is right**, and every way it can be wrong produces a picture:

- a renderer that ignored the azimuth would write three *identical* images, and
  the gate — which checks that three files exist and each has surface in it —
  would pass over it without a murmur;
- a channel swap paints the whole room the wrong colour, which on a grey scan of
  a grey room nobody would ever notice;
- a projection missing its distance scaling puts a plausible blob in the middle of
  the frame at the wrong size, and "coverage" is exactly the statistic that cannot
  tell the difference;
- geometry behind the camera, if it is not dropped, projects to a mirrored
  position somewhere on screen and adds surface that is not there.

So the instrument is checked here with no volume, no container and no saved mesh:
a plane and a cube built in memory, projected through the renderer, and the
picture required to be the one the geometry implies.

It lives in `pimesh_bringup` because that is where this workspace's Python tests
live, and it imports `tools/mesh_render.py` by path for the same reason
`test_straightness.py` imports `tools/calib_straightness.py` — the module is in
`tools/` because that is where this project's one-off tools live, and `tools/` is
rsynced to the Pi, so this test runs at both ends the way `gates/test.sh`
requires.
"""

import importlib.util
import pathlib
import struct

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


def _load_module():
    """Import tools/mesh_render.py by path.

    Three parents up from src/pimesh_bringup/test/test_mesh_render.py is `src/`,
    four is the workspace root. Resolved from __file__ rather than from the cwd
    because `colcon test` runs tests from the build tree.
    """
    root = pathlib.Path(__file__).resolve().parents[3]
    path = root / 'tools' / 'mesh_render.py'
    assert path.is_file(), f'{path} is missing, so this test would check nothing'
    spec = importlib.util.spec_from_file_location('mesh_render', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def render_module():
    return _load_module()


# --- Fixtures ----------------------------------------------------------------

def _square(half=0.5, colour=(200, 100, 50)):
    """A unit-ish square in the plane x = 0, facing down the world +x axis.

    Two triangles, four vertices, and every vertex the same colour. Face-on to a
    camera at azimuth 0 and elevation 0, which is what makes its projected size
    predictable.
    """
    vertices = np.array([
        [0.0, -half, -half],
        [0.0, half, -half],
        [0.0, half, half],
        [0.0, -half, half],
    ], dtype=np.float64)
    colours = np.tile(np.array(colour, dtype=np.float64), (4, 1))
    triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64)
    return vertices, colours, triangles


def _cube(half=0.5):
    """A closed box, so that turning the camera round it changes what is seen."""
    corners = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ], dtype=np.float64) * half
    faces = np.array([
        [0, 2, 1], [0, 3, 2],       # -z
        [4, 5, 6], [4, 6, 7],       # +z
        [0, 1, 5], [0, 5, 4],       # -y
        [3, 7, 6], [3, 6, 2],       # +y
        [0, 4, 7], [0, 7, 3],       # -x
        [1, 2, 6], [1, 6, 5],       # +x
    ], dtype=np.int64)
    # A different colour per face pair, so a view that never changes is obvious.
    colours = np.array([
        [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0],
        [255, 0, 255], [0, 255, 255], [128, 128, 128], [255, 255, 255],
    ], dtype=np.float64)
    return corners, colours, faces


def _ply_bytes(vertices, colours, triangles):
    """The exact binary layout `pimesh_mapping::write_ply` produces.

    Written out here from the format rather than produced by the C++ writer,
    because a Python test that shelled out to a C++ binary would be testing the
    build. What this pins is that the reader parses *this* layout — three floats
    and three bytes per vertex, a count byte and three int32 per face, no padding.
    `test_mesh_cleanup` pins the writer to the same layout from the other side.
    """
    header = (
        'ply\n'
        'format binary_little_endian 1.0\n'
        'comment written by pimesh_mapping\n'
        f'element vertex {len(vertices)}\n'
        'property float x\nproperty float y\nproperty float z\n'
        'property uchar red\nproperty uchar green\nproperty uchar blue\n'
        f'element face {len(triangles)}\n'
        'property list uchar int vertex_indices\n'
        'end_header\n'
    ).encode('ascii')

    body = bytearray()
    for (x, y, z), (r, g, b) in zip(vertices, colours):
        body += struct.pack('<fffBBB', x, y, z, int(r), int(g), int(b))
    for a, b_, c in triangles:
        body += struct.pack('<Biii', 3, int(a), int(b_), int(c))
    return header + bytes(body)


def _coverage(image):
    """Fraction of the frame that is surface rather than the 24-grey background."""
    return float((image.reshape(-1, 3).max(axis=1) > 24).mean())


# --- Reading -----------------------------------------------------------------

def test_the_reader_returns_what_the_writer_laid_down(render_module, tmp_path):
    vertices, colours, triangles = _square()
    path = tmp_path / 'square.ply'
    path.write_bytes(_ply_bytes(vertices, colours, triangles))

    points, read_colours, read_triangles = render_module.read_ply(str(path))
    np.testing.assert_allclose(points, vertices, atol=1e-6)
    np.testing.assert_array_equal(read_triangles, triangles)
    # Colours come back in the file's own order — red, green, blue — and the swap
    # to BGR happens once, where cv2 is handed the value.
    np.testing.assert_allclose(read_colours, colours, atol=0.5)


def test_a_file_that_is_not_this_ply_is_refused_rather_than_guessed_at(
        render_module, tmp_path):
    """A forgiving reader would make the renders a picture of something nobody
    wrote. Three ways to not be this format, and all three have to fail loudly."""
    not_ply = tmp_path / 'notply.ply'
    not_ply.write_bytes(b'this is not a ply file\n')
    with pytest.raises(ValueError):
        render_module.read_ply(str(not_ply))

    ascii_ply = tmp_path / 'ascii.ply'
    ascii_ply.write_bytes(
        b'ply\nformat ascii 1.0\nelement vertex 0\nelement face 0\nend_header\n')
    with pytest.raises(ValueError):
        render_module.read_ply(str(ascii_ply))

    truncated = tmp_path / 'truncated.ply'
    vertices, colours, triangles = _square()
    truncated.write_bytes(_ply_bytes(vertices, colours, triangles)[:-8])
    with pytest.raises(ValueError):
        render_module.read_ply(str(truncated))


# --- Projection ---------------------------------------------------------------

def test_a_face_on_square_projects_to_the_size_the_geometry_implies(render_module):
    """**The assertion that catches a projection with no distance in it.**

    The camera is placed at 2.4 x the bounding radius with a 50 degree horizontal
    field of view. For a 1 m square that is 1.697 m back and a focal length of
    1029 px, so the square spans ~606 px of a 960-px frame: a little over half the
    width, and about 0.53 of the area. A renderer that dropped the distance, or
    used a diameter where it wanted a radius, still draws a centred blob — and
    "coverage" is exactly the statistic that cannot tell the difference.
    """
    vertices, colours, triangles = _square(half=0.5)
    image = render_module.render(vertices, colours, triangles, 0.0, 0.0, 960, 720)

    coverage = _coverage(image)
    assert 0.45 < coverage < 0.62, f'the square covers {coverage:.3f} of the frame'

    # And it is centred: the camera looks at the mesh's centre, so the surface's
    # centroid on screen must be the image centre. A look-at that pointed at the
    # origin instead of at the geometry would put an off-origin mesh in a corner
    # or off the frame entirely.
    mask = image.reshape(720, 960, 3).max(axis=2) > 24
    rows, cols = np.nonzero(mask)
    assert abs(cols.mean() - 480) < 12, f'horizontal centroid {cols.mean():.1f}'
    assert abs(rows.mean() - 360) < 12, f'vertical centroid {rows.mean():.1f}'


def test_an_off_origin_mesh_is_still_framed(render_module):
    """The camera orbits the mesh's own centre, not the world origin. A scan's
    volume is centred wherever the camera started, which is never (0, 0, 0)."""
    vertices, colours, triangles = _square(half=0.5)
    moved = vertices + np.array([12.0, -7.0, 3.0])
    image = render_module.render(moved, colours, triangles, 0.0, 0.0, 960, 720)
    assert _coverage(image) > 0.4, 'a mesh 14 m from the origin fell out of frame'


def test_the_framing_keeps_every_vertex_in_front_of_the_camera(render_module):
    """**The guard against a mirrored projection is unreachable, and that is the
    thing worth pinning.**

    `render` drops any triangle with a vertex at z <= 0 in the camera frame,
    because such a vertex projects to a mirrored position somewhere on screen and
    paints surface that is not there. Trying to *reach* that guard shows it cannot
    be: the eye is placed at 2.4 x the bounding radius from the mesh's own centre,
    and no vertex is further than one radius from that centre, so every vertex is
    in front by at least 1.4 radii whatever the geometry. Adding a triangle far
    out simply grows the radius and pushes the camera further back with it.

    So what is asserted here is the framing rule rather than the guard: the vertex
    *nearest the camera* — sitting on the bounding sphere, on the view axis — must
    still be drawn. Set the 2.4 to anything under 1, and the near face of every
    mesh falls behind the eye and silently vanishes.
    """
    vertices, colours, triangles = _square(half=0.5)
    # A small white triangle on the +x face of the bounding sphere: the closest
    # geometry to a camera at azimuth 0, elevation 0.
    vertices = np.vstack([vertices, np.array([
        [1.0, -0.06, -0.06], [1.0, 0.06, -0.06], [1.0, 0.0, 0.06]])])
    colours = np.vstack([colours, np.tile(np.array([255.0, 255.0, 255.0]), (3, 1))])
    triangles = np.vstack([triangles, np.array([[4, 5, 6]], dtype=np.int64)])

    image = render_module.render(vertices, colours, triangles, 0.0, 0.0, 960, 720)
    white = (image.reshape(-1, 3) > 200).all(axis=1).sum()
    assert white > 50, (
        f'only {white} pixels of the nearest geometry were drawn — the camera is '
        f'inside the bounding sphere and the near surface is being dropped')


def test_the_three_fixed_views_are_three_different_pictures(render_module):
    """**The failure the gate structurally cannot see.**

    `mesh-views.sh` writes three files and `gates/mesh.sh` asserts each has
    surface in it. A renderer that ignored its azimuth would satisfy both, three
    times over, with three copies of one picture — and "three renders, emptiest
    0.09 surface" would read exactly as it does now.
    """
    vertices, colours, triangles = _cube()
    images = [
        render_module.render(vertices, colours, triangles, azimuth, 20.0, 480, 360)
        for azimuth in (0.0, 120.0, 240.0)
    ]
    for image in images:
        assert _coverage(image) > 0.05, 'a view of a cube came out empty'

    for first, second in ((0, 1), (0, 2), (1, 2)):
        difference = np.abs(
            images[first].astype(int) - images[second].astype(int)).mean()
        assert difference > 1.0, (
            f'views {first} and {second} differ by {difference:.3f} per channel — '
            f'the azimuth is not reaching the camera')


# --- Colour --------------------------------------------------------------------

def test_colour_reaches_the_image_in_the_order_cv2_writes_it(render_module):
    """A channel swap paints the whole room the wrong colour, and on a grey scan of
    a grey room nobody would notice. Red in, red out — which for cv2 means the
    *third* channel of the array."""
    for index, rgb in enumerate(((255, 0, 0), (0, 255, 0), (0, 0, 255))):
        vertices, colours, triangles = _square(half=0.5, colour=rgb)
        image = render_module.render(vertices, colours, triangles, 0.0, 0.0, 480, 360)
        mask = image.max(axis=2) > 24
        assert mask.any(), 'nothing was drawn'
        mean = image[mask].mean(axis=0)          # BGR, as cv2 stores it
        # cv2's channel order is the reverse of the file's, so red (index 0 in
        # the PLY) has to dominate channel 2 of the array.
        expected = 2 - index
        assert mean[expected] == mean.max(), (
            f'{rgb} rendered as BGR mean {mean.round(1)} — the channels are swapped')
        assert mean[expected] > 40, f'{rgb} rendered almost black: {mean.round(1)}'


# --- The empty case -------------------------------------------------------------

def test_an_empty_mesh_is_background_and_a_non_zero_exit(render_module, tmp_path, capsys):
    """`gates/mesh.sh` reads this script's exit status and its printed coverage. An
    empty mesh has to be a refusal, not three black rectangles and a success."""
    empty = np.zeros((0, 3))
    image = render_module.render(empty, empty, np.zeros((0, 3), dtype=np.int64),
                                 0.0, 20.0, 64, 48)
    assert _coverage(image) == 0.0

    path = tmp_path / 'empty.ply'
    path.write_bytes(_ply_bytes(np.zeros((0, 3)), np.zeros((0, 3)),
                                np.zeros((0, 3), dtype=np.int64)))
    import sys
    argv = sys.argv
    try:
        sys.argv = ['mesh_render.py', str(path), str(tmp_path / 'views')]
        assert render_module.main() == 1, 'an empty mesh exited 0'
    finally:
        sys.argv = argv


def test_the_renders_are_written_and_their_coverage_is_reported(
        render_module, tmp_path, capsys):
    """End to end, the way `tools/mesh-views.sh` calls it: three files with the
    names the gate greps for, and a coverage line per view."""
    import sys
    vertices, colours, triangles = _cube()
    path = tmp_path / 'cube.ply'
    path.write_bytes(_ply_bytes(vertices, colours, triangles))
    out_dir = tmp_path / 'views'

    argv = sys.argv
    try:
        sys.argv = ['mesh_render.py', str(path), str(out_dir), '--prefix', 'unit',
                    '--width', '320', '--height', '240']
        assert render_module.main() == 0
    finally:
        sys.argv = argv

    printed = capsys.readouterr().out
    for name in ('front', 'left', 'right'):
        written = out_dir / f'unit_{name}.png'
        assert written.is_file(), f'{written} was not written'
        # The gate parses exactly this shape; a change to it breaks the gate
        # silently, because a sed that matches nothing reads as a coverage of 0.
        assert f'render {name} {written} coverage=' in printed, printed
        image = cv2.imread(str(written))
        assert image is not None and _coverage(image) > 0.05
