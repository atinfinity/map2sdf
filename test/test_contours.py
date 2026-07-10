"""Tests for contour extraction from occupancy grids."""

from map2sdf.contours import extract_regions

import numpy as np


def area(ring):
    x, y = ring[:, 0], ring[:, 1]
    return abs(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def test_empty_grid():
    assert extract_regions(np.zeros((10, 10), dtype=bool)) == []


def test_single_cell_becomes_boundary_box():
    grid = np.zeros((5, 5), dtype=bool)
    grid[2, 3] = True
    regions = extract_regions(grid)
    assert len(regions) == 1
    outer = regions[0]['outer']
    assert sorted(outer[:, 0]) == [2.5, 2.5, 3.5, 3.5]  # cols 3 +- 0.5
    assert sorted(outer[:, 1]) == [1.5, 1.5, 2.5, 2.5]  # rows 2 +- 0.5


def test_block_offset_to_cell_boundaries():
    grid = np.zeros((10, 10), dtype=bool)
    grid[2:5, 2:5] = True  # 3x3 block: true boundary spans 1.5 .. 4.5
    regions = extract_regions(grid)
    assert len(regions) == 1
    outer = regions[0]['outer']
    assert outer[:, 0].min() >= 1.5 - 1e-6 and outer[:, 0].max() <= 4.5 + 1e-6
    # Offset grows the cell-center square (area 4) to the full boundary.
    assert 4.0 < area(outer) <= 9.0 + 1e-6


def test_ring_with_hole_and_pillar():
    grid = np.zeros((60, 60), dtype=bool)
    grid[5:55, 5:55] = True     # solid block
    grid[10:50, 10:50] = False  # carve a room -> ring walls
    grid[28:33, 28:33] = True   # pillar inside the room
    regions = extract_regions(grid)
    assert len(regions) == 2
    ring = max(regions, key=lambda r: area(r['outer']))
    pillar = min(regions, key=lambda r: area(r['outer']))
    assert len(ring['holes']) == 1
    assert pillar['holes'] == []
    # The hole boundary sits at the wall/room cell boundary (9.5), with
    # corners pulled slightly toward the wall by the clamped miter.
    hole = ring['holes'][0]
    assert 9.4 <= hole[:, 0].min() <= 9.6
    assert 49.4 <= hole[:, 0].max() <= 49.6


def test_speckles_below_tolerance_removed():
    grid = np.zeros((40, 40), dtype=bool)
    grid[5:35, 5:8] = True     # a real wall, kept
    grid[20, 20] = True        # 1-cell scan noise
    grid[30:32, 30:32] = True  # 2x2 noise blob
    regions = extract_regions(grid, tolerance_cells=3.0)
    assert len(regions) == 1
    assert extract_regions(grid)[0] is not None  # without tolerance: kept
    assert len(extract_regions(grid)) == 3


def test_tolerance_reduces_vertices():
    rng = np.random.default_rng(7)
    grid = np.zeros((30, 200), dtype=bool)
    for c in range(200):
        top = 10 + int(rng.integers(0, 2))
        grid[top:14, c] = True
    exact = extract_regions(grid)
    simplified = extract_regions(grid, tolerance_cells=2.0)
    assert len(simplified[0]['outer']) < len(exact[0]['outer'])
