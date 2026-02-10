#!/usr/bin/env python3
"""MRTA node launch: 6 worker drones target assignment and drop."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'task_allocation'
    config_path = os.path.join(get_package_share_directory(pkg), 'config', 'worker_drones.yaml')
    return LaunchDescription([
        Node(
            package=pkg,
            executable='mrta_node',
            name='mrta_node',
            namespace='',
            parameters=[config_path],
            output='screen',
        ),
    ])
