#!/usr/bin/env python3
"""
Perception launch file for Jackal Navigation:
Launches pointcloud_relay_node and FAST-LIVO2 SLAM independently.
Run in Terminal B before or alongside Navigation in Terminal A.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_share = get_package_share_directory('jackal_nav2_bringup')

    raw_lidar_topic = LaunchConfiguration('raw_lidar_topic')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_enable = LaunchConfiguration('image_enable')

    preflight = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'network_preflight.launch.py')
        )
    )

    relay_node = Node(
        package='jackal_nav2_bringup',
        executable='pointcloud_relay_node',
        name='pointcloud_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': raw_lidar_topic,
            'output_topic': lidar_pointcloud_topic,
            'input_depth': 10,
            'output_depth': 10,
        }],
    )

    fast_livo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fast_livo'),
            'launch', 'mapping_mid360.launch.py',
        ])),
        launch_arguments={
            'lidar_topic': lidar_pointcloud_topic,
            'imu_topic': imu_topic,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'image_enable': image_enable,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'publish_sensor_static_tf': 'false',
            'publish_lidar_to_imu_tf': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('raw_lidar_topic', default_value='/livox/lidar',
                              description='Raw LiDAR pointcloud topic from sensor'),
        DeclareLaunchArgument('lidar_pointcloud_topic', default_value='/livox/lidar_local',
                              description='Relayed local LiDAR pointcloud topic'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic for FAST-LIVO2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_enable', default_value='false'),

        LogInfo(msg='[perception.launch] Starting Network Preflight, PointCloud Relay, and FAST-LIVO2...'),
        preflight,
        relay_node,
        fast_livo,
    ])
