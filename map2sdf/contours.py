"""Extract wall outlines from an occupancy grid as polygons with holes."""

import cv2

import numpy as np

_EPS = 1e-9


def extract_regions(occupied, tolerance_cells=0.0):
    """
    Trace occupied regions as simplified boundary polygons.

    Each region is a dict with an ``'outer'`` (N, 2) float array and a
    ``'holes'`` list of (M, 2) arrays, all in (col, row) pixel
    coordinates. Boundaries are approximated with Douglas-Peucker within
    ``tolerance_cells`` and pushed half a cell away from the occupied
    side so they follow cell boundaries rather than cell centers (a
    one-cell wall keeps its thickness). With a positive tolerance,
    features that fit within a tolerance-sized square are dropped, which
    removes scan-noise speckles.
    """
    img = np.asarray(occupied, dtype=np.uint8)
    contours, hierarchy = cv2.findContours(
        img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    parents = hierarchy[0][:, 3]

    depths = []
    for i in range(len(contours)):
        depth, p = 0, parents[i]
        while p >= 0:
            depth += 1
            p = parents[p]
        depths.append(depth)

    rings = {}
    for i, contour in enumerate(contours):
        ring = _contour_to_ring(contour, tolerance_cells,
                                is_hole=depths[i] % 2 == 1)
        if ring is not None:
            rings[i] = ring

    return [{'outer': rings[i],
             'holes': [rings[j] for j in rings if parents[j] == i]}
            for i in rings if depths[i] % 2 == 0]


def _contour_to_ring(contour, tolerance_cells, is_hole):
    """Simplify one contour and offset it off the occupied side."""
    _, _, w, h = cv2.boundingRect(contour)
    if tolerance_cells > 0:
        if w <= tolerance_cells and h <= tolerance_cells:
            return None  # feature smaller than the tolerance
        contour = cv2.approxPolyDP(contour, tolerance_cells, True)

    pts = contour.reshape(-1, 2).astype(np.float64)
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(pts, axis=0), axis=1) > _EPS
    pts = pts[keep]
    if len(pts) > 1 and np.linalg.norm(pts[0] - pts[-1]) <= _EPS:
        pts = pts[:-1]

    if len(pts) < 3:
        if is_hole:
            return None  # a degenerate hole is simply filled in
        # Single cells and straight one-cell-wide runs collapse to fewer
        # than 3 contour points; represent them by their cell-boundary box.
        x0, y0 = pts.min(axis=0) - 0.5
        x1, y1 = pts.max(axis=0) + 0.5
        return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

    return _offset_ring(pts, push_inside=is_hole)


def _offset_ring(pts, push_inside):
    """Move each vertex half a cell away from the occupied region."""
    d = np.roll(pts, -1, axis=0) - pts
    d = d / np.maximum(np.linalg.norm(d, axis=1), _EPS)[:, None]
    edge_n = np.column_stack([d[:, 1], -d[:, 0]])
    vert_n = edge_n + np.roll(edge_n, 1, axis=0)
    degenerate = np.linalg.norm(vert_n, axis=1) < _EPS
    vert_n[degenerate] = edge_n[degenerate]
    # Miter join: |n1 + n2| = 2 cos(theta/2); scaling by 1 / cos(theta/2)
    # keeps the offset edges exactly half a cell out (clamped for spikes).
    cos_half = np.maximum(np.linalg.norm(vert_n, axis=1) / 2.0, 0.25)
    vert_n = vert_n / np.maximum(np.linalg.norm(vert_n, axis=1), _EPS)[:, None]
    vert_n = vert_n / cos_half[:, None]

    # The occupied side is the ring interior for outer boundaries and the
    # exterior for holes; probe on which side the normals point.
    ring = pts.astype(np.float32).reshape(-1, 1, 2)
    sign = 1.0
    for k in range(len(pts)):
        probe = tuple(pts[k] + 0.25 * vert_n[k])
        side = cv2.pointPolygonTest(ring, probe, False)
        if side != 0:
            sign = 1.0 if (side > 0) == push_inside else -1.0
            break
    return pts + sign * 0.5 * vert_n
