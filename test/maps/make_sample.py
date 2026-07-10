"""Generate the sample occupancy grid map committed next to this script."""

from pathlib import Path

import cv2

import numpy as np

HERE = Path(__file__).parent

FREE, UNKNOWN, WALL = 254, 205, 0


def make_sample():
    """Write sample.pgm and sample.yaml: a 6x5 m room with walls and pillars."""
    grid = np.full((100, 120), FREE, dtype=np.uint8)  # rows x cols

    # Outer walls, 2 cells (0.1 m) thick.
    grid[:2, :] = WALL
    grid[-2:, :] = WALL
    grid[:, :2] = WALL
    grid[:, -2:] = WALL

    # Inner wall splitting the room vertically, with a door gap.
    grid[2:40, 58:62] = WALL
    grid[60:98, 58:62] = WALL

    # Two square pillars.
    grid[25:31, 25:31] = WALL
    grid[70:76, 90:96] = WALL

    # A patch of unexplored space in a corner.
    grid[80:98, 2:20] = UNKNOWN

    cv2.imwrite(str(HERE / 'sample.pgm'), grid)
    (HERE / 'sample.yaml').write_text(
        'image: sample.pgm\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-3.0, -2.5, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n')  # 0.196 keeps gray (205) in the unknown band


if __name__ == '__main__':
    make_sample()
    print(f'wrote {HERE / "sample.pgm"} and {HERE / "sample.yaml"}')
