"""Load occupancy grid maps in the nav2 map_server format (YAML + image)."""

from dataclasses import dataclass
from pathlib import Path

import cv2

import numpy as np

import yaml

VALID_MODES = ('trinary', 'scale', 'raw')


@dataclass
class MapData:
    """
    A binarized occupancy grid map with its metadata.

    ``occupied`` is a (rows, cols) bool array in image order: row 0 is the
    top of the image, which corresponds to the maximum y in the map frame.
    """

    occupied: np.ndarray
    resolution: float
    origin: tuple  # (x, y, yaw) of the lower-left corner in the map frame

    @property
    def size_cells(self):
        """Return the (rows, cols) shape of the grid."""
        return self.occupied.shape


def load_map(yaml_path, unknown_as='free', occupied_thresh=None):
    """
    Load a nav2 map_server YAML + image pair into a MapData.

    :param yaml_path: path to the map YAML file
    :param unknown_as: how to treat unknown cells: 'free' or 'occupied'
    :param occupied_thresh: overrides the YAML occupied_thresh when not None
    """
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    for key in ('image', 'resolution', 'origin'):
        if key not in meta:
            raise ValueError(f"map YAML is missing required key '{key}'")

    image_path = Path(meta['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f'failed to read map image: {image_path}')

    alpha = None
    if image.ndim == 3:
        if image.shape[2] == 4:
            alpha = image[:, :, 3]
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4
                             else cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float64)

    mode = meta.get('mode', 'trinary')
    if mode not in VALID_MODES:
        raise ValueError(f"unsupported map mode '{mode}' (expected one of {VALID_MODES})")
    negate = bool(meta.get('negate', 0))
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    if occupied_thresh is not None:
        occ_thresh = float(occupied_thresh)

    occupied, unknown = _binarize(image, alpha, mode, negate, occ_thresh, free_thresh)
    if unknown_as == 'occupied':
        occupied = occupied | unknown
    elif unknown_as != 'free':
        raise ValueError(f"unknown_as must be 'free' or 'occupied', got '{unknown_as}'")

    origin = meta['origin']
    if len(origin) < 3:
        origin = list(origin) + [0.0] * (3 - len(origin))

    return MapData(
        occupied=occupied,
        resolution=float(meta['resolution']),
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
    )


def _binarize(image, alpha, mode, negate, occ_thresh, free_thresh):
    """Return (occupied, unknown) bool arrays following nav2 semantics."""
    if mode == 'raw':
        # Pixel value is the occupancy probability in [0, 100]; 255 = unknown.
        occupied = (image >= occ_thresh * 100.0) & (image <= 100.0)
        unknown = image > 100.0
        return occupied, unknown

    # trinary / scale: darker pixels are more occupied unless negate is set.
    p = image / 255.0 if negate else (255.0 - image) / 255.0
    occupied = p > occ_thresh
    unknown = (p <= occ_thresh) & (p >= free_thresh)
    if mode == 'scale' and alpha is not None:
        unknown = unknown | (alpha < 255)
        occupied = occupied & (alpha == 255)
    return occupied, unknown
