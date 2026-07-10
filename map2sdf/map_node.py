"""ROS 2 node: generate an SDF world from a nav_msgs/OccupancyGrid topic."""

import math
from pathlib import Path

from map2sdf.contours import extract_regions
from map2sdf.map_io import MapData
from map2sdf.mesher import regions_to_triangles
from map2sdf.sdf_builder import build_world
from map2sdf.stl_writer import write_binary_stl

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def grid_to_map_data(msg, unknown_as='free', occupied_thresh=0.65):
    """
    Convert a nav_msgs/OccupancyGrid message into a MapData.

    Message rows start at the map origin (lowest y), while MapData uses
    image order (row 0 at the highest y), so the grid is flipped.
    """
    data = np.asarray(msg.data, dtype=np.int8).reshape(
        msg.info.height, msg.info.width)
    occupied = data >= int(round(occupied_thresh * 100.0))
    if unknown_as == 'occupied':
        occupied = occupied | (data < 0)
    q = msg.info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return MapData(
        occupied=np.flipud(occupied),
        resolution=float(msg.info.resolution),
        origin=(float(msg.info.origin.position.x),
                float(msg.info.origin.position.y), yaw),
    )


class MapToSdfNode(Node):
    """Subscribe to an occupancy grid topic and write SDF worlds."""

    def __init__(self):
        super().__init__('map2sdf_node')
        self.declare_parameter('out', '.')
        self.declare_parameter('world_name', 'map_world')
        self.declare_parameter('wall_height', 2.0)
        self.declare_parameter('simplify', 0.0)
        self.declare_parameter('ground', True)
        self.declare_parameter('shadows', False)
        self.declare_parameter('unknown_as', 'free')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('one_shot', True)

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self._sub = self.create_subscription(
            OccupancyGrid, 'map', self._on_map, qos)
        self.get_logger().info('waiting for an occupancy grid on /map ...')

    def _on_map(self, msg):
        p = {name: self.get_parameter(name).value
             for name in ('out', 'world_name', 'wall_height', 'simplify',
                          'ground', 'shadows', 'unknown_as',
                          'occupied_thresh', 'one_shot')}
        map_data = grid_to_map_data(msg, unknown_as=p['unknown_as'],
                                    occupied_thresh=p['occupied_thresh'])
        regions = extract_regions(
            map_data.occupied, p['simplify'] / map_data.resolution)
        triangles = regions_to_triangles(
            regions, map_data.size_cells[0], map_data.resolution,
            p['wall_height'])

        out_dir = Path(p['out'])
        out_dir.mkdir(parents=True, exist_ok=True)
        mesh_name = f"{p['world_name']}_walls.stl"
        write_binary_stl(out_dir / mesh_name, triangles)
        sdf_text = build_world(map_data, mesh_name,
                               world_name=p['world_name'],
                               ground=p['ground'], shadows=p['shadows'])
        world_path = out_dir / f"{p['world_name']}.sdf"
        world_path.write_text(sdf_text)
        self.get_logger().info(
            f'wrote {world_path} ({len(regions)} wall regions, '
            f'{len(triangles)} triangles)')
        if p['one_shot']:
            raise SystemExit(0)


def main(args=None):
    """Entry point for the ``map2sdf_node`` executable."""
    rclpy.init(args=args)
    node = MapToSdfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
