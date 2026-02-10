#!/usr/bin/env python3
"""
4 scout drones + central fusion launch.
- coverage_partitions.yaml is read for each drone's scan cell.
- 4x satellite_scout_node (scout_mission), 4x blue_target_mapper (map_object_detector), 1x fusion_node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, filename):
    pkg_share = get_package_share_directory(package_name)
    path = os.path.join(pkg_share, 'config', filename)
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = 'multi_scout'
    config_dir = os.path.join(get_package_share_directory(pkg), 'config')
    partitions_path = os.path.join(config_dir, 'coverage_partitions.yaml')
    fusion_path = os.path.join(config_dir, 'fusion_params.yaml')

    with open(partitions_path, 'r') as f:
        partitions = yaml.safe_load(f)
    drones = partitions.get('drones', [])

    ld = LaunchDescription()

    # File to save detected coords (empty = node default: ~/multi_uav_recon_ws/output/detected_final.yaml)
    default_detected_file = os.path.join(os.path.expanduser('~'), 'multi_uav_recon_ws', 'output', 'detected_final.yaml')
    ld.add_action(DeclareLaunchArgument(
        'detected_output_file', default_value=default_detected_file,
        description='YAML file path for saving detected targets',
    ))
    ld.add_action(DeclareLaunchArgument(
        'detected_save_interval_s', default_value='30',
        description='Save interval in seconds; 0 = only on node shutdown',
    ))

    # RViz "No tf data" fix: add world frame to tf tree (Gazebo/PX4 sometimes don't publish it)
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    ))

    # 4x satellite_scout_node — 50m climb, transit to zone centre, 80m, then hold (scan off)
    for i, d in enumerate(drones):
        vid = d['vehicle_id']
        params = {
            'vehicle_id': vid,
            'spawn_x_world': d['spawn_x_world'],
            'spawn_y_world': d['spawn_y_world'],
            'scan_zone_world_min_x': d['scan_zone_world_min_x'],
            'scan_zone_world_max_x': d['scan_zone_world_max_x'],
            'scan_zone_world_min_y': d['scan_zone_world_min_y'],
            'scan_zone_world_max_y': d['scan_zone_world_max_y'],
            'zone_center_world_x': d['zone_center_world_x'],
            'zone_center_world_y': d['zone_center_world_y'],
            'use_scan_zone': True,
            'transit_altitude_m': 50.0,
            'scan_enable': False,
            'patrol_mode': 'orbit',
            'orbit_scan_enable': False,
            'orbit_shape': 'lawnmower',
            'satellite_altitude': -80.0,
            'lawnmower_speed': 1.1,
            'orbit_control_mode': 'velocity',
        }
        ld.add_action(Node(
            package='scout_mission',
            executable='satellite_scout_node',
            name=f'satellite_scout_{vid}',
            namespace='',
            parameters=[params],
            output='screen',
        ))

    # 4x blue_target_mapper — each leader's camera + detection output
    leader_cameras = [
        '/leader_1/camera/image_raw',
        '/leader_2/camera/image_raw',
        '/leader_3/camera/image_raw',
        '/leader_4/camera/image_raw',
    ]
    drone_ids = ['px4_1', 'px4_2', 'px4_3', 'px4_4']
    detection_topics = ['/scout/detections_1', '/scout/detections_2', '/scout/detections_3', '/scout/detections_4']

    for i in range(4):
        d = drones[i]
        ld.add_action(Node(
            package='map_object_detector',
            executable='blue_target_mapper',
            name=f'blue_target_mapper_{i+1}',
            namespace='',
            parameters=[{
                'image_topic': leader_cameras[i],
                'drone_id': drone_ids[i],
                'target_altitude': 80.0,
                'detection_output_topic': detection_topics[i],
                'spawn_x_world': d['spawn_x_world'],
                'spawn_y_world': d['spawn_y_world'],
            }],
            output='screen',
        ))

    # 1x fusion_node
    ld.add_action(Node(
        package='multi_scout',
        executable='fusion_node',
        name='fusion_node',
        namespace='',
        parameters=[fusion_path],
        output='screen',
    ))

    # Zone boundaries viz (RViz: /scout/zone_boundaries -> MarkerArray)
    ld.add_action(Node(
        package='multi_scout',
        executable='zone_boundaries_node',
        name='zone_boundaries_node',
        namespace='',
        parameters=[{'frame_id': 'world'}],
        output='screen',
    ))

    # Fused map image: zones + fused targets -> 2D image (/scout/fused_map_image)
    ld.add_action(Node(
        package='multi_scout',
        executable='fused_map_image_node',
        name='fused_map_image_node',
        namespace='',
        output='screen',
    ))

    # Node that writes detected coords to file (if output_file is set)
    ld.add_action(Node(
        package='multi_scout',
        executable='detected_targets_recorder_node',
        name='detected_targets_recorder_node',
        namespace='',
        parameters=[{
            'fused_targets_topic': '/scout/fused_targets',
            'output_file': LaunchConfiguration('detected_output_file', default=''),
            'save_interval_s': LaunchConfiguration('detected_save_interval_s', default='30'),
            'save_on_shutdown': True,
        }],
        output='screen',
    ))

    return ld
