"""Extrude wall polygons into a watertight triangle mesh."""

import numpy as np

_EPS = 1e-9


def regions_to_triangles(regions, rows, resolution, wall_height):
    """
    Convert regions from extract_regions into extruded mesh triangles.

    Polygons in (col, row) pixel coordinates are mapped to the map-local
    frame (lower-left corner of the image at the origin, y up) and
    extruded from z=0 to ``wall_height`` as prisms with triangulated top
    and bottom caps.

    :returns: (N, 3, 3) float32 array of triangles
    """
    tris = []
    for region in regions:
        outer = _ensure_winding(
            _to_map(region['outer'], rows, resolution), ccw=True)
        holes = [_ensure_winding(_to_map(h, rows, resolution), ccw=False)
                 for h in region['holes']]
        for ring in [outer] + holes:
            _side_walls(ring, wall_height, tris)
        for cap in _triangulate(outer, holes):
            tris.append(np.column_stack([cap, np.full(3, wall_height)]))
            tris.append(np.column_stack([cap[::-1], np.zeros(3)]))
    if not tris:
        return np.zeros((0, 3, 3), dtype=np.float32)
    return np.asarray(tris, dtype=np.float32)


def _to_map(pts, rows, resolution):
    """Map (col, row) pixel coordinates to map-local meters (y up)."""
    return np.column_stack([(pts[:, 0] + 0.5) * resolution,
                            (rows - pts[:, 1] - 0.5) * resolution])


def _signed_area(ring):
    x, y = ring[:, 0], ring[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _ensure_winding(ring, ccw):
    if (_signed_area(ring) > 0) != ccw:
        return ring[::-1]
    return ring


def _side_walls(ring, wall_height, out):
    """Append two triangles per edge; for CCW rings normals face right."""
    for k in range(len(ring)):
        a, b = ring[k], ring[(k + 1) % len(ring)]
        if np.linalg.norm(b - a) <= _EPS:
            continue
        a0, b0 = (*a, 0.0), (*b, 0.0)
        a1, b1 = (*a, wall_height), (*b, wall_height)
        out.append(np.array([a0, b0, b1]))
        out.append(np.array([a0, b1, a1]))


def _triangulate(outer, holes):
    """Triangulate a CCW polygon with CW holes via bridges + ear clipping."""
    return _ear_clip(_merge_holes(outer, holes))


def _merge_holes(outer, holes):
    """Connect each hole to the outer ring with a zero-width bridge."""
    poly = [tuple(p) for p in outer]
    pending = [[tuple(p) for p in h] for h in
               sorted(holes, key=lambda h: -float(np.max(h[:, 0])))]
    while pending:
        hole = pending.pop(0)
        hi = max(range(len(hole)), key=lambda k: hole[k][0])
        chains = [poly] + pending + [hole]
        order = sorted(range(len(poly)),
                       key=lambda j: (poly[j][0] - hole[hi][0]) ** 2 +
                                     (poly[j][1] - hole[hi][1]) ** 2)
        for j in order:
            if _segment_clear(chains, hole[hi], poly[j]):
                poly = poly[:j + 1] + hole[hi:] + hole[:hi + 1] + poly[j:]
                break
        # If no unobstructed bridge exists the hole is dropped.
    return np.array(poly)


def _segment_clear(chains, p, q):
    """Return True if segment p-q crosses no chain edge (endpoints allowed)."""
    for chain in chains:
        n = len(chain)
        for k in range(n):
            a, b = chain[k], chain[(k + 1) % n]
            if _close(a, p) or _close(a, q) or _close(b, p) or _close(b, q):
                continue
            if _segments_intersect(p, q, a, b):
                return False
    return True


def _close(a, b):
    return abs(a[0] - b[0]) <= _EPS and abs(a[1] - b[1]) <= _EPS


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_intersect(p, q, a, b):
    """Segment intersection test; touching counts as intersecting."""
    d1, d2 = _cross(p, q, a), _cross(p, q, b)
    d3, d4 = _cross(a, b, p), _cross(a, b, q)
    if ((d1 > _EPS) != (d2 > _EPS)) and ((d3 > _EPS) != (d4 > _EPS)) \
            and not (abs(d1) <= _EPS and abs(d2) <= _EPS):
        return True
    for o, s, t in ((a, p, q), (b, p, q), (p, a, b), (q, a, b)):
        if abs(_cross(s, t, o)) <= _EPS and _on_segment(s, t, o):
            return True
    return False


def _on_segment(s, t, o):
    return (min(s[0], t[0]) - _EPS <= o[0] <= max(s[0], t[0]) + _EPS
            and min(s[1], t[1]) - _EPS <= o[1] <= max(s[1], t[1]) + _EPS)


def _ear_clip(poly):
    """Ear-clip a simple CCW polygon (may contain bridge duplicates)."""
    n = len(poly)
    if n < 3:
        return []
    nxt = {i: (i + 1) % n for i in range(n)}
    prv = {i: (i - 1) % n for i in range(n)}
    tris, remaining, i, stuck = [], n, 0, 0
    while remaining > 3:
        a, b, c = poly[prv[i]], poly[i], poly[nxt[i]]
        if _is_ear(poly, prv, nxt, i):
            if _cross(a, b, c) > _EPS:
                tris.append(np.array([a, b, c]))
            nxt[prv[i]], prv[nxt[i]] = nxt[i], prv[i]
            i = prv[i]
            remaining -= 1
            stuck = 0
        else:
            i = nxt[i]
            stuck += 1
            if stuck > remaining:
                # Numerical degeneracy: fan-triangulate what is left.
                order = [i]
                j = nxt[i]
                while j != i:
                    order.append(j)
                    j = nxt[j]
                for k in range(1, len(order) - 1):
                    t = np.array([poly[order[0]], poly[order[k]],
                                  poly[order[k + 1]]])
                    if abs(_cross(t[0], t[1], t[2])) > _EPS:
                        tris.append(t)
                return tris
    a, b, c = poly[prv[i]], poly[i], poly[nxt[i]]
    if _cross(a, b, c) > _EPS:
        tris.append(np.array([a, b, c]))
    return tris


def _is_ear(poly, prv, nxt, i):
    a, b, c = poly[prv[i]], poly[i], poly[nxt[i]]
    cross = _cross(a, b, c)
    if abs(cross) <= _EPS:
        return True  # collinear vertex: removable, emits no triangle
    if cross < 0:
        return False  # reflex corner
    j = nxt[nxt[i]]
    while j != prv[i]:
        p = poly[j]
        if not (_close(p, a) or _close(p, b) or _close(p, c)) \
                and _cross(a, b, p) > _EPS and _cross(b, c, p) > _EPS \
                and _cross(c, a, p) > _EPS:
            return False
        j = nxt[j]
    return True
