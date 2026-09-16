#!/usr/bin/env python3

"""Launch FAST-LIVO2 odometry adaptation, map server, and AMCL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


def _validate_inputs(context):
    map_path = os.path.expanduser(LaunchConfiguration('map').perform(context))
    params_path = os.path.expanduser(
        LaunchConfiguration('params_file').perform(context))

    if not os.path.isabs(map_path):
        raise RuntimeError(f'map must be an absolute path: {map_path}')
    if not os.path.isfile(map_path):
        raise RuntimeError(f'map YAML file does not exist: {map_path}')
    if not os.path.isfile(params_path):
        raise RuntimeError(f'Nav2 parameter file does not exist: {params_path}')

    try:
        with open(map_path, encoding='utf-8') as stream:
            map_config = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f'failed to read map YAML {map_path}: {error}') from error

    if not isinstance(map_config, dict):
        raise RuntimeError(f'map YAML must contain a mapping: {map_path}')
    image_value = map_config.get('image')
    if not isinstance(image_value, str) or not image_value.strip():
        raise RuntimeError(f'map YAML has no valid image entry: {map_path}')

    image_path = os.path.expanduser(image_value)
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_path), image_path)
    image_path = os.path.normpath(image_path)
    if not os.path.isfile(image_path):
        raise RuntimeError(
            f'map image referenced by {map_path} does not exist: {image_path}')
    return []


def generate_launch_description():
    package_share = get_package_share_directory('jackal_nav2_bringup')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')
    fast_livo_odom_topic = LaunchConfiguration('fast_livo_odom_topic')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    use_scan_projection = LaunchConfiguration('use_scan_projection')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=None,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'odom_topic': nav_odom_topic,
                'scan_topic': scan_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = ['map_server', 'amcl']

    common = {
        'output': 'screen',
        'respawn': use_respawn,
        'respawn_delay': 2.0,
        'parameters': [configured_params],
        'arguments': ['--ros-args', '--log-level', log_level],
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the occupancy-grid map YAML'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_scan_projection', default_value='true'),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_amcl_quality_monitor', default_value='true',
                              description='Read-only AMCL covariance diagnostics; '
                                          'never injects an initial pose'),
        OpaqueFunction(function=_validate_inputs),
        Node(
            package='jackal_nav2_bringup',
            executable='fast_livo_odom_adapter.py',
            name='fast_livo_odom_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': fast_livo_odom_topic,
                'output_topic': nav_odom_topic,
                'expected_frame_id': 'odom',
                'expected_child_frame_id': 'base_link',
            }],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='amcl_quality_monitor.py',
            name='amcl_quality_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_amcl_quality_monitor')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    condition=IfCondition(use_scan_projection),
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=tf_remaps + [
                        ('cloud_in', lidar_pointcloud_topic),
                        ('scan', scan_topic),
                    ],
                ),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    remappings=tf_remaps,
                    **common,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
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
            condition=IfCondition(PythonExpression([
                "'", use_composition, "'.lower() == 'true' and '",
                use_scan_projection, "'.lower() == 'true'",
            ])),
            target_container=('/', container_name),
            composable_node_descriptions=[
                ComposableNode(
                    package='pointcloud_to_laserscan',
                    plugin=(
                        'pointcloud_to_laserscan::'
                        'PointCloudToLaserScanNode'),
                    name='pointcloud_to_laserscan',
                    parameters=[configured_params],
                    remappings=tf_remaps + [
                        ('cloud_in', lidar_pointcloud_topic),
                        ('scan', scan_topic),
                    ],
                ),
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=('/', container_name),
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='map_server',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_amcl',
                    plugin='nav2_amcl::AmclNode',
                    name='amcl',
                    parameters=[configured_params],
                    remappings=tf_remaps,
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_localization',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    }],
                ),
            ],
        ),
    ])
