"""Tests for polygon extrusion and triangulation."""

from pathlib import Path

import cv2

from map2sdf.contours import extract_regions
from map2sdf.map_io import load_map
from map2sdf.mesher import regions_to_triangles

import numpy as np

import pytest

SAMPLE_YAML = Path(__file__).parent / 'maps' / 'sample.yaml'


def mesh_volume(tris):
    """Signed volume via the divergence theorem; positive if outward."""
    v = tris.astype(np.float64)
    return np.einsum('ij,ij->', np.cross(v[:, 0], v[:, 1]), v[:, 2]) / 6.0


def cap_area(regions, rows, res):
    """Area of the wall footprint: outer polygons minus their holes."""
    def shoelace(ring):
        x, y = ring[:, 0] * res, ring[:, 1] * res
        return abs(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    return sum(shoelace(r['outer']) - sum(shoelace(h) for h in r['holes'])
               for r in regions)


def assert_watertight(tris):
    """Every undirected edge of a closed mesh appears exactly twice."""
    edges = {}
    for t in np.round(tris.astype(np.float64), 6):
        for k in range(3):
            edge = frozenset([tuple(t[k]), tuple(t[(k + 1) % 3])])
            edges[edge] = edges.get(edge, 0) + 1
    counts = set(edges.values())
    assert counts == {2}, f'edge multiplicities {counts} != {{2}}'


def fixture_grids():
    block = np.zeros((10, 10), dtype=bool)
    block[2:5, 2:7] = True

    ring = np.zeros((60, 60), dtype=bool)
    ring[5:55, 5:55] = True
    ring[10:50, 10:50] = False
    ring[28:33, 28:33] = True  # pillar in the room
    return {'block': block, 'ring_with_pillar': ring}


@pytest.mark.parametrize('name', ['block', 'ring_with_pillar'])
def test_mesh_is_watertight_prism(name):
    grid = fixture_grids()[name]
    regions = extract_regions(grid)
    tris = regions_to_triangles(regions, grid.shape[0], 0.5, 2.0)
    assert len(tris) > 0
    assert_watertight(tris)
    # For outward-oriented prisms, volume == footprint area * height.
    expected = cap_area(regions, grid.shape[0], 0.5) * 2.0
    assert mesh_volume(tris) == pytest.approx(expected, rel=1e-5)


def test_cap_covers_polygon():
    grid = fixture_grids()['ring_with_pillar']
    regions = extract_regions(grid)
    tris = regions_to_triangles(regions, grid.shape[0], 1.0, 2.0)
    top = tris[np.all(tris[:, :, 2] == 2.0, axis=1)][:, :, :2]

    scale = 4  # rasterize at 1/4-cell resolution
    size = grid.shape[0] * scale
    expected = np.zeros((size, size), np.uint8)
    for r in regions:
        y = grid.shape[0] - r['outer'][:, 1] - 0.5  # same y-up transform
        pts = np.column_stack([(r['outer'][:, 0] + 0.5), y]) * scale
        cv2.fillPoly(expected, [np.round(pts).astype(np.int32)], 1)
        for h in r['holes']:
            y = grid.shape[0] - h[:, 1] - 0.5
            pts = np.column_stack([(h[:, 0] + 0.5), y]) * scale
            cv2.fillPoly(expected, [np.round(pts).astype(np.int32)], 0)
    actual = np.zeros_like(expected)
    for t in top:
        cv2.fillPoly(actual, [np.round(t * scale).astype(np.int32)], 1)
    # Rasterization rounding leaves a thin band of mismatches along edges.
    mismatch = np.count_nonzero(actual ^ expected)
    assert mismatch < 0.05 * np.count_nonzero(expected)


def test_sample_map_mesh():
    data = load_map(SAMPLE_YAML)
    regions = extract_regions(data.occupied)
    tris = regions_to_triangles(
        regions, data.size_cells[0], data.resolution, 2.0)
    assert_watertight(tris)
    expected = cap_area(regions, data.size_cells[0], data.resolution) * 2.0
    assert mesh_volume(tris) == pytest.approx(expected, rel=1e-4)


def test_empty_regions():
    tris = regions_to_triangles([], 10, 0.05, 2.0)
    assert tris.shape == (0, 3, 3)
