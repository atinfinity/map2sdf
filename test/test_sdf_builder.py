"""Tests for SDF world generation and STL export."""

import math
import struct
from xml.etree import ElementTree as ET

from map2sdf.contours import extract_regions
from map2sdf.map_io import MapData
from map2sdf.mesher import regions_to_triangles
from map2sdf.sdf_builder import build_world
from map2sdf.stl_writer import write_binary_stl

import numpy as np

import pytest


def single_cell_map():
    """One occupied cell at row 0, col 2 of a 3x4 grid, 0.5 m resolution."""
    grid = np.zeros((3, 4), dtype=bool)
    grid[0, 2] = True
    return MapData(occupied=grid, resolution=0.5, origin=(1.0, 2.0, 0.25))


def test_world_structure():
    map_data = single_cell_map()
    sdf = ET.fromstring(build_world(map_data, 'walls.stl'))
    world = sdf.find('world')
    assert world.get('name') == 'map_world'

    model = world.find("model[@name='map_walls']")
    assert model.find('static').text == 'true'
    assert model.find('pose').text.split() == ['1', '2', '0', '0', '0', '0.25']
    assert world.find("model[@name='ground_plane']") is not None


def test_shadows_disabled_by_default():
    map_data = single_cell_map()
    world = ET.fromstring(build_world(map_data, 'walls.stl')).find('world')
    assert world.find('scene/shadows').text == 'false'
    assert world.find("light[@name='sun']/cast_shadows").text == 'false'

    world = ET.fromstring(
        build_world(map_data, 'walls.stl', shadows=True)).find('world')
    assert world.find('scene/shadows').text == 'true'
    assert world.find("light[@name='sun']/cast_shadows").text == 'true'


def test_ground_plane_covers_map():
    # 4 cols x 3 rows at 0.5 m -> 2.0 x 1.5 m map at origin (1, 2), yaw 0.25.
    map_data = single_cell_map()
    world = ET.fromstring(build_world(map_data, 'walls.stl')).find('world')
    ground = world.find("model[@name='ground_plane']")
    c, s = math.cos(0.25), math.sin(0.25)
    cx = 1.0 + (2.0 * c - 1.5 * s) / 2.0
    cy = 2.0 + (2.0 * s + 1.5 * c) / 2.0
    pose = [float(v) for v in ground.find('pose').text.split()]
    assert pose[:2] == pytest.approx([cx, cy], abs=1e-3)
    size = [float(v) for v in ground.find('link/collision/geometry/plane/size').text.split()]
    assert size == pytest.approx(
        [2.0 * c + 1.5 * s + 20.0, 2.0 * s + 1.5 * c + 20.0], abs=1e-3)


def test_no_ground_option():
    map_data = single_cell_map()
    sdf = ET.fromstring(build_world(map_data, 'walls.stl', ground=False))
    assert sdf.find("world/model[@name='ground_plane']") is None


def test_mesh_world_references_uri():
    map_data = single_cell_map()
    sdf = ET.fromstring(build_world(map_data, 'walls.stl'))
    link = sdf.find("world/model[@name='map_walls']/link")
    assert link.find('collision/geometry/mesh/uri').text == 'walls.stl'
    assert link.find('visual/geometry/mesh/uri').text == 'walls.stl'
    assert len(link.findall('collision')) == 1


def test_mesh_requires_uri():
    with pytest.raises(ValueError, match='mesh_uri'):
        build_world(single_cell_map(), '')


def test_single_cell_mesh_bounds():
    grid = np.zeros((3, 4), dtype=bool)
    grid[0, 2] = True  # cell boundary: x in [1.0, 1.5], y in [1.0, 1.5]
    regions = extract_regions(grid)
    tris = regions_to_triangles(regions, rows=3, resolution=0.5, wall_height=2.0)
    assert tris.shape == (12, 3, 3)
    flat = tris.reshape(-1, 3)
    assert flat[:, 0].min() == pytest.approx(1.0)
    assert flat[:, 0].max() == pytest.approx(1.5)
    assert flat[:, 1].min() == pytest.approx(1.0)
    assert flat[:, 1].max() == pytest.approx(1.5)
    assert flat[:, 2].max() == pytest.approx(2.0)   # z in [0, wall_height]


def test_binary_stl_roundtrip(tmp_path):
    grid = np.zeros((2, 3), dtype=bool)
    grid[:, :] = True  # single 3x2-cell prism: 0.3 x 0.2 m footprint
    tris = regions_to_triangles(
        extract_regions(grid), rows=2, resolution=0.1, wall_height=1.0)
    path = tmp_path / 'walls.stl'
    write_binary_stl(path, tris)
    raw = path.read_bytes()
    count = struct.unpack('<I', raw[80:84])[0]
    assert count == len(tris)
    assert len(raw) == 84 + count * 50
    # Signed volume from the divergence theorem must equal the prism volume.
    record = np.frombuffer(raw[84:], dtype=[('n', '<f4', (3,)),
                                            ('v', '<f4', (3, 3)),
                                            ('attr', '<u2')])
    v = record['v'].astype(np.float64)
    volume = np.einsum('ij,ij->', np.cross(v[:, 0], v[:, 1]), v[:, 2]) / 6.0
    assert volume == pytest.approx(0.3 * 0.2 * 1.0, rel=0.01)
