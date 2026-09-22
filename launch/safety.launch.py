#!/usr/bin/env python3
"""Independent raw LiDAR safety pipeline (Nav2 Humble / 1.1.x)."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def collision_parameters(guard):
    """Derive both monitor polygons from the guard's single source of truth."""
    params = {
        'base_frame_id': guard['base_frame'],
        'odom_frame_id': guard['odom_frame'],
        'cmd_vel_in_topic': '/nav2_cmd_vel_unstamped',
        'cmd_vel_out_topic': '/nav2/collision_checked_cmd_vel',
        'base_shift_correction': False,
        'transform_tolerance': 0.05,
        'source_timeout': guard['sensor_timeout'],
        'stop_pub_timeout': 2.0,
        'polygons': ['Stop', 'Slow'],
        'observation_sources': ['raw_lidar'],
        'raw_lidar.type': 'pointcloud',
        'raw_lidar.topic': '/nav2/safety_points',
        'raw_lidar.min_height': guard['min_height'],
        'raw_lidar.max_height': guard['max_height'],
    }
    for name, prefix, action in [
        ('Stop', 'stop', 'stop'),
        ('Slow', 'slow', 'slowdown'),
    ]:
        x, y = guard[f'{prefix}_half_x'], guard[f'{prefix}_half_y']
        if not all(math.isfinite(v) and v > 0 for v in (x, y)):
            raise ValueError('Invalid safety polygon')
        params.update(
            {
                f'{name}.type': 'polygon',
                f'{name}.action_type': action,
                f'{name}.points': [x, y, x, -y, -x, -y, -x, y],
                # Humble tests points > max_points, not >= min_points.
                f'{name}.max_points': guard['min_points'] - 1,
                f'{name}.visualize': True,
                f'{name}.polygon_pub_topic': f'/nav2/safety_{prefix}_polygon',
            }
        )
    params['Slow.slowdown_ratio'] = guard['slowdown_ratio']
    return params


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    with open(
        os.path.expanduser(value('safety_params_file')), encoding='utf-8'
    ) as stream:
        guard = yaml.safe_load(stream)['nav2_safety_guard']['ros__parameters']
    monitor = collision_parameters(guard)
    sim = boolean('use_sim_time')
    guard.update(
        {
            'use_sim_time': sim,
            'enable_motion': boolean('enable_motion'),
            'input_topic': value('lidar_pointcloud_topic'),
            'output_topic': value('nav_cmd_vel_topic'),
            'command_topic': monitor['cmd_vel_out_topic'],
            'safety_points_topic': monitor['raw_lidar.topic'],
        }
    )
    return [
        Node(
            package='jackal_nav2_bringup', executable='operator_stop.py',
            name='nav2_operator_stop', output='screen',
            condition=IfCondition(LaunchConfiguration('launch_operator_stop')),
            parameters=[value('operator_params_file'), {
                'use_sim_time': sim,
                'max_linear_x': guard['max_linear_x'],
                'max_angular_z': guard['max_angular_z'],
            }],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='nav2_safety_guard.py',
            name='nav2_safety_guard',
            output='screen',
            parameters=[guard],
        ),
        # Separate processes keep the output watchdog independent of Nav2's container.
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[monitor, {'use_sim_time': sim}],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='cmd_vel_safety_bridge.py',
            name='cmd_vel_safety_bridge',
            output='screen',
            parameters=[{
                'input_topic': value('nav_cmd_vel_topic'),
                'output_topic': '/j100_0519/cmd_vel',
                'forward_cmd_vel': boolean('enable_motion'),
                'max_linear_x': guard['max_linear_x'],
                'max_angular_z': guard['max_angular_z'],
                'timeout_sec': 0.5,
                'publish_rate_hz': 20.0,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_safety',
            output='screen',
            parameters=[
                {
                    'use_sim_time': sim,
                    'autostart': boolean('autostart'),
                    'node_names': ['collision_monitor'],
                    'bond_timeout': 0.0,
                }
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('jackal_nav2_bringup')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'safety_params_file',
                default_value=os.path.join(
                    share, 'config', 'nav2_safety.yaml'
                ),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('enable_motion', default_value='false'),
            DeclareLaunchArgument(
                'launch_operator_stop', default_value=LaunchConfiguration('enable_motion')),
            DeclareLaunchArgument('operator_params_file', default_value=os.path.join(
                share, 'config', 'operator_stop.yaml')),
            DeclareLaunchArgument(
                'lidar_pointcloud_topic', default_value='/livox/lidar_local'
            ),
            DeclareLaunchArgument(
                'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'network_preflight.launch.py'))),
            OpaqueFunction(function=_setup),
        ]
    )
