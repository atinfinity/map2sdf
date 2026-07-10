"""Tests for OccupancyGrid message conversion."""

import math

from map2sdf.map_node import grid_to_map_data

from nav_msgs.msg import OccupancyGrid

import pytest


def make_msg():
    msg = OccupancyGrid()
    msg.info.width = 3
    msg.info.height = 2
    msg.info.resolution = 0.1
    msg.info.origin.position.x = 1.0
    msg.info.origin.position.y = 2.0
    msg.info.origin.orientation.z = math.sin(0.25)  # yaw = 0.5
    msg.info.origin.orientation.w = math.cos(0.25)
    # Row 0 starts at the origin (lowest y): occupied, free, unknown.
    msg.data = [100, 0, -1,
                0, 100, 65]
    return msg


def test_grid_flipped_to_image_order():
    data = grid_to_map_data(make_msg())
    # Message row 0 (lowest y) must become the last image row.
    assert data.occupied.tolist() == [[False, True, True],
                                      [True, False, False]]
    assert data.resolution == pytest.approx(0.1)


def test_origin_yaw_from_quaternion():
    data = grid_to_map_data(make_msg())
    assert data.origin == pytest.approx((1.0, 2.0, 0.5))


def test_unknown_and_threshold():
    data = grid_to_map_data(make_msg(), unknown_as='occupied')
    assert data.occupied.tolist() == [[False, True, True],
                                      [True, False, True]]
    data = grid_to_map_data(make_msg(), occupied_thresh=0.7)
    assert data.occupied.tolist()[0] == [False, True, False]  # 65 < 70
