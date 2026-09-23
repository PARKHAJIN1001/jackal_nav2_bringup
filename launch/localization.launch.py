#!/usr/bin/env python3

"""Launch pointcloud relay, FAST-LIVO2 odometry adaptation, map server, and AMCL."""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import yaml


CRITICAL = {
    'fastlivo_mapping', 'pointcloud_relay_node', 'nav2_safety_guard.py',
    'operator_stop.py', 'fast_livo_odom_adapter.py', 'stack_stability.py',
    'collision_monitor', 'controller_server', 'planner_server', 'bt_navigator',
    'velocity_smoother', 'amcl', 'map_server', 'static_costmap_node',
    'pointcloud_to_laserscan_node', 'component_container_isolated',
    'smoother_server', 'behavior_server', 'waypoint_follower', 'lifecycle_manager',
}


def _critical_exit(event, context):
    if context.is_shutdown or not event.cmd:
        return []
    names = [Path(value).name for value in event.cmd[:2]]
    executable = names[1] if names[0].startswith('python') and len(names) > 1 else names[0]
    if executable in CRITICAL or (executable == 'topic_ready_gate.py' and event.returncode):
        reason = f'{executable} exited ({event.returncode}); restart required'
        directory = os.environ.get('JACKAL_NAV_SESSION_DIR')
        if directory:
            path = Path(directory) / 'failure.json'
            try:
                with path.open('x') as stream:
                    json.dump({'reason': reason, 'monotonic_sec': time.monotonic(),
                               'wall_sec': time.time()}, stream)
            except OSError:
                pass
        return [Shutdown(reason=reason)]
    return []


def _network_preflight(context):
    if getattr(context, '_jackal_network_checked', False):
        return []
    result = subprocess.run(
        ['ros2', 'run', 'jackal_network_bringup', 'network_preflight.py', '--check'],
        capture_output=True, text=True, timeout=15, check=False)
    try:
        report = json.loads(result.stdout)
    except ValueError as error:
        raise RuntimeError('Rebuild jackal_network_bringup: network preflight unavailable; '
                           + result.stderr.strip()) from error
    if result.returncode != 0 or report.get('ready') is not True:
        raise RuntimeError('Network preflight failed: ' + json.dumps(report))
    context._jackal_network_checked = True
    return [RegisterEventHandler(OnProcessExit(on_exit=_critical_exit))]


critical_exit = _critical_exit
preflight = _network_preflight


def _validate_inputs(context):
    if context.launch_configurations.get('use_respawn', 'false').lower() != 'false':
        raise RuntimeError('Automatic respawn is disabled; use a fresh managed session')
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


def _configure_rviz(context):
    """Disable remote raw camera display in a temporary RViz profile."""
    path = LaunchConfiguration('rviz_config').perform(context)
    if (not IfCondition(LaunchConfiguration('use_rviz')).evaluate(context) or
            IfCondition(LaunchConfiguration('use_camera_image')).evaluate(context)):
        return [SetLaunchConfiguration('resolved_rviz_config', path)]
    with open(os.path.expanduser(path), encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)

    def disable_raw_image(displays):
        for display in displays:
            topic = display.get('Topic', {})
            value = topic.get('Value', '') if isinstance(topic, dict) else topic
            if (display.get('Class') == 'rviz_default_plugins/Image' and
                    value == '/camera/camera/color/image_raw'):
                display['Enabled'] = False
                if 'Value' in display:
                    display['Value'] = False
            disable_raw_image(display.get('Displays', []))

    disable_raw_image(profile['Visualization Manager']['Displays'])
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='nav_bringup_', suffix='.rviz', delete=False) as stream:
        yaml.safe_dump(profile, stream, sort_keys=False)
        generated = stream.name
    return [SetLaunchConfiguration('resolved_rviz_config', generated)]


def generate_launch_description():
    package_share = get_package_share_directory('jackal_nav2_bringup')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'jackal_nav2.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    raw_lidar_topic = LaunchConfiguration('raw_lidar_topic')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    use_pointcloud_relay = LaunchConfiguration('use_pointcloud_relay')
    fast_livo_odom_topic = LaunchConfiguration('fast_livo_odom_topic')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    use_scan_projection = LaunchConfiguration('use_scan_projection')
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
            'raw_lidar_topic', default_value='/livox/lidar',
            description='Raw LiDAR pointcloud topic from sensor'),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar_local',
            description='Relayed local LiDAR pointcloud topic'),
        DeclareLaunchArgument(
            'use_pointcloud_relay', default_value='true',
            description='Enable pointcloud_relay_node for Fast-DDS QoS buffering'),
        DeclareLaunchArgument(
            'fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_scan_projection', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_amcl_quality_monitor', default_value='true',
                              description='Read-only AMCL covariance diagnostics; '
                                          'never injects an initial pose'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic for FAST-LIVO2'),
        DeclareLaunchArgument('image_enable', default_value='false',
                              description='Enable camera image for FAST-LIVO2'),
        DeclareLaunchArgument('use_fast_livo', default_value='true',
                              description='Launch FAST-LIVO2 odometry'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 for localization and 2D pose estimate'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_camera_image', default_value='false',
                              description='Enable the remote raw camera display in RViz'),
        DeclareLaunchArgument('battery_state_topic',
                              default_value='/j100_0519/platform/bms/state'),
        OpaqueFunction(function=_network_preflight),
        OpaqueFunction(function=_validate_inputs),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fast_livo'),
                'launch', 'mapping_mid360.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('use_fast_livo')),
            launch_arguments={
                'lidar_topic': lidar_pointcloud_topic,
                'imu_topic': LaunchConfiguration('imu_topic'),
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'image_enable': LaunchConfiguration('image_enable'),
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'publish_sensor_static_tf': 'false',
                'publish_lidar_to_imu_tf': 'true',
            }.items(),
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='pointcloud_relay_node',
            name='pointcloud_relay',
            output='screen',
            condition=IfCondition(use_pointcloud_relay),
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': raw_lidar_topic,
                'output_topic': lidar_pointcloud_topic,
                'input_depth': 10,
                'output_depth': 10,
            }],
        ),
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
                        'bond_timeout': 0.0,
                    }],
                ),
            ],
        ),
        OpaqueFunction(function=_configure_rviz),
        Node(
            package='rviz2',
            executable='rviz2',
            name='jackal_nav2_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('resolved_rviz_config')],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='rviz_overlay_bridge.py',
            name='rviz_overlay_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'battery_topic': LaunchConfiguration('battery_state_topic'),
                'speed_topic': nav_odom_topic,
                'min_voltage': 24.0,
                'max_voltage': 29.4,
                'stale_timeout_sec': 5.0,
            }],
        ),
    ])
