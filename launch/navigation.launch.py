#!/usr/bin/env python3

"""Launch Nav2 with an isolated, stamped velocity output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


def _validate_params(context):
    path = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    if not os.path.isfile(path):
        raise RuntimeError(f'Nav2 parameter file does not exist: {path}')
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    for name in ('static_costmap', 'global_costmap', 'local_costmap'):
        costmap = config[name][name]['ros__parameters']
        if costmap.get('plugins') != ['static_layer', 'inflation_layer']:
            raise RuntimeError(f'{name} must use only StaticLayer + InflationLayer')
        if (costmap['static_layer'].get('plugin') != 'nav2_costmap_2d::StaticLayer' or
                costmap['inflation_layer'].get('plugin') != 'nav2_costmap_2d::InflationLayer' or
                'observation_sources' in yaml.safe_dump(costmap) or costmap.get('filters')):
            raise RuntimeError(f'{name} violates the prior-map-only policy')
    return []


def generate_launch_description():
    package_share = get_package_share_directory('jackal_nav2_bringup')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    nav_cmd_vel_topic = LaunchConfiguration('nav_cmd_vel_topic')
    use_map_patch = LaunchConfiguration('use_map_patch')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=None,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'odom_topic': nav_odom_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {
        'output': 'screen',
        'respawn': use_respawn,
        'respawn_delay': 2.0,
        'parameters': [configured_params],
        'arguments': ['--ros-args', '--log-level', log_level],
    }
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_map_patch', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('safety_params_file', default_value=os.path.join(
            package_share, 'config', 'nav2_safety.yaml')),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'),
        OpaqueFunction(function=_validate_params),
        Node(
            package='jackal_nav2_bringup',
            executable='static_costmap_node',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='map_patch_node.py',
            name='map_patch_node',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            condition=IfCondition(use_map_patch),
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_static_costmap',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['/static_costmap/static_costmap'],
                # Standalone Costmap2DROS does not create a lifecycle bond.
                'bond_timeout': 0.0,
            }],
        ),
        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    remappings=tf_remaps + [
                        ('cmd_vel', 'cmd_vel_nav'),
                    ],
                    **common,
                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
                    **common,
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    remappings=tf_remaps + [
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', 'nav2_cmd_vel_unstamped'),
                    ],
                    **common,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    }],
                ),
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=('/', container_name),
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=tf_remaps + [
                        ('cmd_vel', 'cmd_vel_nav'),
                    ],
                ),
                ComposableNode(
                    package='nav2_smoother',
                    plugin='nav2_smoother::SmootherServer',
                    name='smoother_server',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[configured_params],
                    remappings=(
                        tf_remaps + [('cmd_vel', 'cmd_vel_nav')]),
                ),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_waypoint_follower',
                    plugin='nav2_waypoint_follower::WaypointFollower',
                    name='waypoint_follower',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_velocity_smoother',
                    plugin='nav2_velocity_smoother::VelocitySmoother',
                    name='velocity_smoother',
                    parameters=[configured_params],
                    remappings=tf_remaps + [
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', 'nav2_cmd_vel_unstamped'),
                    ],
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    }],
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_share, 'launch', 'safety.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'enable_motion': LaunchConfiguration('enable_motion'),
                'safety_params_file': LaunchConfiguration('safety_params_file'),
                'lidar_pointcloud_topic': lidar_pointcloud_topic,
                'nav_cmd_vel_topic': nav_cmd_vel_topic,
            }.items(),
        ),
    ])
