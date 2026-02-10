from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_scout'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik',
    maintainer_email='user@todo.todo',
    description='4 scout drones simultaneous scan: coverage, fusion, central map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = multi_scout.fusion_node:main',
            'zone_boundaries_node = multi_scout.zone_boundaries_node:main',
            'fused_map_image_node = multi_scout.fused_map_image_node:main',
            'detected_targets_recorder_node = multi_scout.detected_targets_recorder_node:main',
        ],
    },
)
