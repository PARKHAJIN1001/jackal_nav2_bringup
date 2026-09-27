#!/usr/bin/env python3

"""Launch Nav2 stack with waypoint loop node for shuttle navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('jackal_nav2_bringup')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_safety_params = os.path.join(package_share, 'config', 'nav2_safety.yaml')
    default_operator_params = os.path.join(package_share, 'config', 'operator_stop.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # ---- Loop-specific arguments ----
        DeclareLaunchArgument(
            'loop_count', default_value='5',
            description='Number of round-trips between the two waypoints'),
        DeclareLaunchArgument(
            'driving_policy', default_value='nav2',
            description='Driving policy: "nav2" uses Nav2 planner, '
                        '"other" waits for external cmd_vel'),
        DeclareLaunchArgument(
            'external_cmd_vel_topic', default_value='/external_cmd_vel',
            description='Topic for external cmd_vel when driving_policy=other'),

        # ---- Arguments forwarded to nav2.launch.py ----
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('safety_params_file', default_value=default_safety_params),
        DeclareLaunchArgument('operator_params_file', default_value=default_operator_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_map_patch', default_value='true'),
        DeclareLaunchArgument('launch_stability_monitor', default_value='false'),
        DeclareLaunchArgument('enable_motion', default_value='true'),
        DeclareLaunchArgument('stability_timeout', default_value='600.0'),
        DeclareLaunchArgument('stability_settle', default_value='2.0'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument(
            'launch_operator_stop',
            default_value=LaunchConfiguration('enable_motion')),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar_local'),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_camera_image', default_value='true'),
        DeclareLaunchArgument(
            'use_ui_overlays', default_value=LaunchConfiguration('use_rviz')),
        DeclareLaunchArgument('use_battery_gauge', default_value='true'),
        DeclareLaunchArgument('use_speed_display', default_value='true'),
        DeclareLaunchArgument(
            'battery_state_topic',
            default_value='/j100_0519/platform/bms/state'),
        DeclareLaunchArgument(
            'speed_odom_topic',
            default_value=LaunchConfiguration('nav_odom_topic')),
        DeclareLaunchArgument('use_pedestrian_figures', default_value='true'),
        DeclareLaunchArgument('use_pedestrian_traces', default_value='true'),
        DeclareLaunchArgument('tracks_topic', default_value='/ped_tracking'),
        DeclareLaunchArgument('traces_topic', default_value='/ped_traces'),

        # ---- Include the full Nav2 stack ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jackal_nav2_bringup'),
                'launch', 'nav2.launch.py',
            ])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'safety_params_file': LaunchConfiguration('safety_params_file'),
                'operator_params_file': LaunchConfiguration('operator_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_respawn': LaunchConfiguration('use_respawn'),
                'use_composition': LaunchConfiguration('use_composition'),
                'container_name': LaunchConfiguration('container_name'),
                'log_level': LaunchConfiguration('log_level'),
                'nav_odom_topic': LaunchConfiguration('nav_odom_topic'),
                'use_map_patch': LaunchConfiguration('use_map_patch'),
                'launch_stability_monitor': LaunchConfiguration('launch_stability_monitor'),
                'enable_motion': LaunchConfiguration('enable_motion'),
                'stability_timeout': LaunchConfiguration('stability_timeout'),
                'stability_settle': LaunchConfiguration('stability_settle'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'fast_livo_odom_topic': LaunchConfiguration('fast_livo_odom_topic'),
                'launch_operator_stop': LaunchConfiguration('launch_operator_stop'),
                'lidar_pointcloud_topic': LaunchConfiguration('lidar_pointcloud_topic'),
                'nav_cmd_vel_topic': LaunchConfiguration('nav_cmd_vel_topic'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_camera_image': LaunchConfiguration('use_camera_image'),
                'use_ui_overlays': LaunchConfiguration('use_ui_overlays'),
                'use_battery_gauge': LaunchConfiguration('use_battery_gauge'),
                'use_speed_display': LaunchConfiguration('use_speed_display'),
                'battery_state_topic': LaunchConfiguration('battery_state_topic'),
                'speed_odom_topic': LaunchConfiguration('speed_odom_topic'),
                'use_pedestrian_figures': LaunchConfiguration('use_pedestrian_figures'),
                'use_pedestrian_traces': LaunchConfiguration('use_pedestrian_traces'),
                'tracks_topic': LaunchConfiguration('tracks_topic'),
                'traces_topic': LaunchConfiguration('traces_topic'),
            }.items(),
        ),

        # ---- Waypoint Loop Node ----
        Node(
            package='jackal_nav2_bringup',
            executable='waypoint_loop_node.py',
            name='waypoint_loop_node',
            output='screen',
            parameters=[{
                'loop_count': ParameterValue(
                    LaunchConfiguration('loop_count'), value_type=int),
                'driving_policy': LaunchConfiguration('driving_policy'),
                'external_cmd_vel_topic': LaunchConfiguration(
                    'external_cmd_vel_topic'),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
