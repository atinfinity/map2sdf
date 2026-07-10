"""Write wall mesh triangles as a binary STL file."""

import numpy as np


def write_binary_stl(path, triangles):
    """Write an (N, 3, 3) triangle array to ``path`` as binary STL."""
    triangles = np.asarray(triangles, dtype=np.float32)
    n = len(triangles)
    record = np.zeros(n, dtype=[('normal', '<f4', (3,)),
                                ('vertices', '<f4', (3, 3)),
                                ('attr', '<u2')])
    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edge1, edge2)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0.0] = 1.0
    record['normal'] = normals / lengths
    record['vertices'] = triangles
    with open(path, 'wb') as f:
        f.write(b'map2sdf binary STL'.ljust(80, b'\0'))
        f.write(np.uint32(n).tobytes())
        f.write(record.tobytes())
