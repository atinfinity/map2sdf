"""Command-line interface: convert an occupancy grid map to an SDF world."""

import argparse
from pathlib import Path
import sys

from map2sdf.contours import extract_regions
from map2sdf.map_io import load_map
from map2sdf.mesher import regions_to_triangles
from map2sdf.sdf_builder import build_world
from map2sdf.stl_writer import write_binary_stl


def main(argv=None):
    """Entry point for the ``map2sdf`` executable."""
    parser = argparse.ArgumentParser(
        prog='map2sdf',
        description='Generate a Gazebo SDF world from a nav2-format '
                    'occupancy grid map (YAML + PGM/PNG).')
    parser.add_argument('--map', required=True, dest='map_yaml',
                        help='path to the map YAML file')
    parser.add_argument('-o', '--out', default='.',
                        help='output directory (default: current directory)')
    parser.add_argument('--wall-height', type=float, default=2.0,
                        help='wall height in meters (default: 2.0)')
    parser.add_argument('--world-name', default='map_world',
                        help='SDF world name, also used as the output file stem')
    parser.add_argument('--no-ground', action='store_true',
                        help='do not add a ground plane')
    parser.add_argument('--shadows', action='store_true',
                        help='enable shadow rendering (disabled by default '
                             'to keep rendering light)')
    parser.add_argument('--unknown-as', choices=('free', 'occupied'), default='free',
                        help='treat unknown map cells as free or occupied '
                             '(default: free)')
    parser.add_argument('--occupied-thresh', type=float, default=None,
                        help='override the occupied threshold from the map YAML')
    parser.add_argument('--simplify', type=float, default=0.0, metavar='TOL',
                        help='simplify walls before meshing: contours are '
                             'approximated within TOL meters, which greatly '
                             'reduces the triangle count for jagged SLAM maps; '
                             'features thinner than TOL may disappear '
                             '(default: 0 = off)')
    args = parser.parse_args(argv)

    try:
        map_data = load_map(args.map_yaml, unknown_as=args.unknown_as,
                            occupied_thresh=args.occupied_thresh)
    except (OSError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1

    regions = extract_regions(
        map_data.occupied, args.simplify / map_data.resolution)
    if not regions:
        print('warning: no occupied cells found in the map', file=sys.stderr)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    world_path = out_dir / f'{args.world_name}.sdf'

    mesh_name = f'{args.world_name}_walls.stl'
    triangles = regions_to_triangles(
        regions, map_data.size_cells[0], map_data.resolution, args.wall_height)
    write_binary_stl(out_dir / mesh_name, triangles)
    print(f'wrote {out_dir / mesh_name} ({len(triangles)} triangles)')

    sdf_text = build_world(
        map_data,
        mesh_name,  # resolved relative to the world file
        world_name=args.world_name,
        ground=not args.no_ground,
        shadows=args.shadows)
    world_path.write_text(sdf_text)

    rows, cols = map_data.size_cells
    print(f'wrote {world_path} ({rows}x{cols} cells -> {len(regions)} wall '
          f'regions, {len(triangles)} triangles)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
