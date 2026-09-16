#!/usr/bin/env python3
"""Render a saved PLY from fixed angles, offscreen, to PNG files.

**These images are P6's evidence.** The RViz window is a viewer for a person;
what closes the phase is a file somebody can look at later and a script can point
at. So this has to run with no display, no GL context and no window manager —
which rules out every renderer that goes through OpenGL, and rules out Open3D
twice over, since it is not installed and must not come back (it is what forced a
process boundary on the Python side).

So: a software rasteriser in numpy, writing with cv2. About 120 lines, no
dependency this workspace does not already have, and offscreen *by construction*
rather than by configuration — there is nothing here that could try to open a
window and fail on a headless machine.

**Painter's algorithm rather than a z-buffer**, and the trade is worth stating.
Triangles are drawn back to front and a nearer one simply overwrites a farther
one, which is exact for a surface and wrong where two triangles interpenetrate —
a thin sliver of the wrong surface at an intersection. A scan of a room has few
of those, and the alternative in pure numpy is either a per-pixel depth
comparison over a million triangles (slow) or a C extension (a dependency). What
this cannot do is be trusted for measurement; it is a picture, and the numbers
the gate asserts on come from the node.

Usage:
    mesh_render.py <mesh.ply> <output-dir> [--prefix NAME] [--width W] [--height H]
"""

import argparse
import os
import struct
import sys

import cv2
import numpy as np


def read_ply(path):
    """Read the binary little-endian PLY that pimesh_world writes.

    Deliberately narrow: it reads what `write_ply` produces and refuses anything
    else. A forgiving reader that silently mis-parses another dialect would make
    the renders a picture of something nobody wrote.
    """
    with open(path, 'rb') as handle:
        if handle.readline().strip() != b'ply':
            raise ValueError(f'{path} does not start with "ply"')
        n_vertices = n_faces = 0
        binary_le = False
        element = None
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f'{path} has no end_header')
            fields = line.split()
            if not fields:
                continue
            if fields[0] == b'format':
                binary_le = fields[1] == b'binary_little_endian'
            elif fields[0] == b'element':
                element = fields[1]
                if element == b'vertex':
                    n_vertices = int(fields[2])
                elif element == b'face':
                    n_faces = int(fields[2])
            elif fields[0] == b'end_header':
                break
        if not binary_le:
            raise ValueError(f'{path} is not binary_little_endian')

        # 3 floats + 3 uchar per vertex, packed, exactly as write_ply lays it out.
        vertex_dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                 ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        vertices = np.frombuffer(handle.read(vertex_dtype.itemsize * n_vertices),
                                 dtype=vertex_dtype, count=n_vertices)
        points = np.stack([vertices['x'], vertices['y'], vertices['z']], axis=1).astype(np.float64)
        colours = np.stack([vertices['r'], vertices['g'], vertices['b']], axis=1).astype(np.float64)

        face_dtype = np.dtype([('n', 'u1'), ('a', '<i4'), ('b', '<i4'), ('c', '<i4')])
        faces = np.frombuffer(handle.read(face_dtype.itemsize * n_faces),
                              dtype=face_dtype, count=n_faces)
        if n_faces and not np.all(faces['n'] == 3):
            raise ValueError(f'{path} has a face that is not a triangle')
        triangles = np.stack([faces['a'], faces['b'], faces['c']], axis=1).astype(np.int64)
    return points, colours, triangles


def look_at(eye, target, up=np.array([0.0, 0.0, 1.0])):
    """World-to-camera rotation, in the optical convention (z into the scene)."""
    forward = target - eye
    forward /= np.linalg.norm(forward)
    if abs(np.dot(forward, up)) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    # Rows are the camera axes in world coordinates: x right, y down, z forward.
    return np.stack([right, down, forward], axis=0)


def render(points, colours, triangles, azimuth_deg, elevation_deg, width, height):
    """One view. Returns a BGR image."""
    image = np.full((height, width, 3), 24, dtype=np.uint8)
    if len(triangles) == 0:
        return image

    centre = 0.5 * (points.min(axis=0) + points.max(axis=0))
    radius = float(np.linalg.norm(points - centre, axis=1).max())
    if radius <= 0:
        return image

    azimuth = np.radians(azimuth_deg)
    elevation = np.radians(elevation_deg)
    # Far enough out that the whole surface fits with margin at a ~50 degree
    # horizontal field of view.
    distance = radius * 2.4
    eye = centre + distance * np.array([
        np.cos(elevation) * np.cos(azimuth),
        np.cos(elevation) * np.sin(azimuth),
        np.sin(elevation)])
    rotation = look_at(eye, centre)

    camera = (points - eye) @ rotation.T
    focal = 0.5 * width / np.tan(np.radians(50.0) / 2.0)

    # Behind the camera: drop any triangle with a vertex there rather than letting
    # it project to a mirrored position somewhere on screen.
    z = camera[:, 2]
    valid = z > 1e-6
    safe_z = np.where(valid, z, 1.0)
    u = focal * camera[:, 0] / safe_z + width / 2.0
    v = focal * camera[:, 1] / safe_z + height / 2.0
    screen = np.stack([u, v], axis=1)

    tri = triangles
    keep = valid[tri].all(axis=1)
    tri = tri[keep]
    if len(tri) == 0:
        return image

    a = camera[tri[:, 0]]
    b = camera[tri[:, 1]]
    c = camera[tri[:, 2]]
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    good = lengths > 1e-12
    tri = tri[good]
    normals = normals[good] / lengths[good, None]
    if len(tri) == 0:
        return image

    # A headlight: the camera looks along +z, so the facing-ness of a triangle is
    # |n_z|. Absolute value rather than a backface cull, because a scan's winding
    # is consistent but a viewer coming at a wall from behind should still see the
    # wall rather than a hole.
    shade = 0.25 + 0.75 * np.abs(normals[:, 2])
    face_colour = colours[tri].mean(axis=1) * shade[:, None]
    face_colour = np.clip(face_colour, 0, 255).astype(np.uint8)

    depth = camera[tri, 2].mean(axis=1)
    order = np.argsort(-depth)          # farthest first
    corners = screen[tri].astype(np.int32)

    for index in order:
        cv2.fillConvexPoly(image, corners[index],
                           (int(face_colour[index][2]),
                            int(face_colour[index][1]),
                            int(face_colour[index][0])),
                           lineType=cv2.LINE_8)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh')
    parser.add_argument('out_dir')
    parser.add_argument('--prefix', default='mesh')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=720)
    args = parser.parse_args()

    points, colours, triangles = read_ply(args.mesh)
    if len(triangles) == 0:
        print(f'FAIL: {args.mesh} has no triangles', file=sys.stderr)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    # Three fixed angles, not chosen per mesh: the point of a fixed set is that two
    # runs are comparable. 120 degrees apart around the surface, from slightly
    # above, which is where a hand-held camera was.
    views = [('front', 0.0, 20.0), ('left', 120.0, 20.0), ('right', 240.0, 20.0)]
    written = []
    for name, azimuth, elevation in views:
        image = render(points, colours, triangles, azimuth, elevation,
                       args.width, args.height)
        path = os.path.join(args.out_dir, f'{args.prefix}_{name}.png')
        if not cv2.imwrite(path, image):
            print(f'FAIL: could not write {path}', file=sys.stderr)
            return 1
        # The fraction of the frame the surface covers. A render that is all
        # background is what an empty mesh, a camera inside the geometry, or a
        # projection sign error all produce, and all three look like a black
        # rectangle. The gate asserts on this rather than on the file existing.
        coverage = float((image.reshape(-1, 3).max(axis=1) > 24).mean())
        written.append((path, coverage))
        print(f'render {name} {path} coverage={coverage:.4f}')

    print(f'mesh {args.mesh} vertices={len(points)} triangles={len(triangles)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
