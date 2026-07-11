"""Build a Gazebo (SDFormat 1.9) world XML from merged map rectangles."""

import math
from xml.dom import minidom
from xml.etree import ElementTree as ET

WALL_COLOR = '0.8 0.8 0.8 1'

# Standard gz-sim server systems, matching the worlds shipped with Gazebo.
_WORLD_PLUGINS = (
    ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
)


def build_world(map_data, mesh_uri, world_name='map_world',
                ground=True, shadows=False):
    """
    Return the SDF world document as a pretty-printed XML string.

    :param map_data: MapData with resolution, origin and grid shape
    :param mesh_uri: URI of the wall STL mesh, resolved relative to the
        world file
    :param shadows: enable shadow rendering (off by default to keep
        rendering light, e.g. under software rendering)
    """
    if not mesh_uri:
        raise ValueError('mesh_uri is required')

    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name=world_name)
    for filename, name in _WORLD_PLUGINS:
        ET.SubElement(world, 'plugin', filename=filename, name=name)

    _add_scene_and_sun(world, shadows)
    if ground:
        _add_ground_plane(world, map_data)

    _add_walls_model(world, map_data, mesh_uri, 'map_walls')
    raw = ET.tostring(sdf, encoding='unicode')
    return minidom.parseString(raw).toprettyxml(indent='  ')


def build_model(map_data, mesh_uri, model_name='map_walls'):
    """
    Return a standalone SDF model document as an XML string.

    The map origin pose is embedded in the model, so including it
    without an explicit pose keeps the walls aligned to the map frame.
    """
    if not mesh_uri:
        raise ValueError('mesh_uri is required')
    sdf = ET.Element('sdf', version='1.9')
    _add_walls_model(sdf, map_data, mesh_uri, model_name)
    raw = ET.tostring(sdf, encoding='unicode')
    return minidom.parseString(raw).toprettyxml(indent='  ')


def build_model_config(model_name, sdf_filename='model.sdf'):
    """Return the model.config XML string for a generated model."""
    root = ET.Element('model')
    ET.SubElement(root, 'name').text = model_name
    ET.SubElement(root, 'version').text = '1.0'
    ET.SubElement(root, 'sdf', version='1.9').text = sdf_filename
    ET.SubElement(root, 'description').text = \
        'Walls generated from an occupancy grid map by map2sdf.'
    raw = ET.tostring(root, encoding='unicode')
    return minidom.parseString(raw).toprettyxml(indent='  ')


def _add_walls_model(parent, map_data, mesh_uri, name):
    model = ET.SubElement(parent, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ox, oy, yaw = map_data.origin
    ET.SubElement(model, 'pose').text = f'{ox:g} {oy:g} 0 0 0 {yaw:g}'
    link = ET.SubElement(model, 'link', name='walls')
    _add_mesh_walls(link, mesh_uri)
    return model


def _add_mesh_walls(link, mesh_uri):
    """Add a single collision+visual pair referencing the wall mesh."""
    for tag in ('collision', 'visual'):
        elem = ET.SubElement(link, tag, name=f'walls_{tag}')
        geometry = ET.SubElement(elem, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = mesh_uri
    _add_material(link.find("visual[@name='walls_visual']"))


def _add_material(visual):
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = WALL_COLOR
    ET.SubElement(material, 'diffuse').text = WALL_COLOR


def _add_scene_and_sun(world, shadows):
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.4 0.4 0.4 1'
    ET.SubElement(scene, 'background').text = '0.7 0.8 1.0 1'
    ET.SubElement(scene, 'shadows').text = 'true' if shadows else 'false'

    sun = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(sun, 'cast_shadows').text = 'true' if shadows else 'false'
    ET.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(sun, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(sun, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(sun, 'direction').text = '-0.5 0.1 -0.9'


def _add_ground_plane(world, map_data, margin=10.0):
    rows, cols = map_data.size_cells
    width = cols * map_data.resolution
    height = rows * map_data.resolution
    ox, oy, yaw = map_data.origin
    # World-frame center and axis-aligned extent of the rotated map area.
    c, s = math.cos(yaw), math.sin(yaw)
    cx = ox + (width * c - height * s) / 2.0
    cy = oy + (width * s + height * c) / 2.0
    size_x = abs(width * c) + abs(height * s) + 2.0 * margin
    size_y = abs(width * s) + abs(height * c) + 2.0 * margin

    model = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{cx:g} {cy:g} 0 0 0 0'
    link = ET.SubElement(model, 'link', name='link')
    for tag in ('collision', 'visual'):
        elem = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(elem, 'geometry')
        plane = ET.SubElement(geometry, 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = f'{size_x:g} {size_y:g}'
    visual = link.find('visual')
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '0.6 0.6 0.6 1'
    ET.SubElement(material, 'diffuse').text = '0.6 0.6 0.6 1'
