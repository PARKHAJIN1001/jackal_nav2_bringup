#!/usr/bin/env python3
"""Uninstalled test fixture for legacy synthetic Nav2 tests, never robot startup."""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _isolated_fixture(context):
    if os.environ.get('ROS_DOMAIN_ID') not in ('86', '188') or os.environ.get(
            'ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Test fixture requires isolated domain 86/188 and localhost only')
    for name, expected in (('nav_cmd_vel_topic', '/nav2_test/output'),
                           ('lidar_pointcloud_topic', '/nav2_test/raw')):
        if LaunchConfiguration(name).perform(context) != expected:
            raise RuntimeError('Test fixture requires ' + expected)
    # Synthetic fixture owns transport and readiness; standard entry points have no bypass.
    context._jackal_network_checked = True
    return []


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'fixture_navigation', root / 'launch/nav2.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    navigation = module.generate_launch_description()
    # Full stability is tested by controlled-clock tests, not these kinematic fixtures.
    actions = [action for action in navigation.entities
               if not (isinstance(action, Node) and
                       action.node_executable == 'stack_stability.py')]
    share = get_package_share_directory('jackal_nav2_bringup')
    return LaunchDescription([
        OpaqueFunction(function=_isolated_fixture),
        DeclareLaunchArgument('fixture_localization', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_ui_overlays', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            root / 'launch/localization.launch.py')),
            condition=IfCondition(LaunchConfiguration('fixture_localization'))),
        Node(package='moai_nav_viz', executable='pedestrian_figures_node',
             name='pedestrian_figures',
             condition=IfCondition(LaunchConfiguration('fixture_localization')),
             parameters=[str(Path(share) / 'config/pedestrian_viz.yaml'), {
                 'tracks_topic': '/ped_tracking', 'output_topic': '/nav2/pedestrian_figures',
                 'target_frame': 'odom'}]),
        *actions,
    ])
