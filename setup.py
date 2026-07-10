"""Setup script for the map2sdf package."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'map2sdf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('test/maps/sample.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jazzy',
    maintainer_email='dandelion1124@gmail.com',
    description='Generate Gazebo SDF world files from ROS 2 occupancy grid maps.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map2sdf = map2sdf.cli:main',
            'map2sdf_node = map2sdf.map_node:main',
        ],
    },
)
