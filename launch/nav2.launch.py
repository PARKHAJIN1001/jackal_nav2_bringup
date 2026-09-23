#!/usr/bin/env python3

"""Launch Nav2, Safety Guard, Collision Monitor, and RViz UI."""

import json
import math
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
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
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


def _validate_params(context):
    if context.launch_configurations.get('use_respawn', 'false').lower() != 'false':
        raise RuntimeError('Automatic respawn is disabled; use a fresh managed session')
    limit = float(context.launch_configurations.get('stability_timeout', '600'))
    hold = float(context.launch_configurations.get('stability_settle', '180'))
    if not all(math.isfinite(v) and v > 0 for v in (limit, hold)) or hold >= limit:
        raise RuntimeError('stability_settle must be positive and less than stability_timeout')
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
                f'{name}.max_points': guard['min_points'] - 1,
                f'{name}.visualize': True,
                f'{name}.polygon_pub_topic': f'/nav2/safety_{prefix}_polygon',
            }
        )
    params['Slow.slowdown_ratio'] = guard['slowdown_ratio']
    return params


def _setup_safety(context):
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
            'require_stability': boolean('launch_stability_monitor'),
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
    default_safety_params = os.path.join(package_share, 'config', 'nav2_safety.yaml')
    default_operator_params = os.path.join(package_share, 'config', 'operator_stop.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'jackal_nav2.rviz')
    default_ped_viz = os.path.join(package_share, 'config', 'pedestrian_viz.yaml')

    navigation_tree = os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
        'navigate_w_replanning_time.xml')
    if not os.path.isfile(navigation_tree):
        raise RuntimeError(f'Nav2 navigation behavior tree missing: {navigation_tree}')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    nav_cmd_vel_topic = LaunchConfiguration('nav_cmd_vel_topic')
    use_map_patch = LaunchConfiguration('use_map_patch')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    use_rviz = LaunchConfiguration('use_rviz')

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
    bt_parameters = [configured_params, {'default_nav_to_pose_bt_xml': navigation_tree}]
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
        # Nav2 configurations
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
        DeclareLaunchArgument('use_map_patch', default_value='false'),
        DeclareLaunchArgument('launch_stability_monitor', default_value='false'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('stability_timeout', default_value='600.0'),
        DeclareLaunchArgument('stability_settle', default_value='2.0'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument(
            'launch_operator_stop', default_value=LaunchConfiguration('enable_motion')),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar_local'),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'),

        # Visualization & Overlays
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_camera_image', default_value='false'),
        DeclareLaunchArgument(
            'use_ui_overlays', default_value=LaunchConfiguration('use_rviz')),
        DeclareLaunchArgument('use_battery_gauge', default_value='true'),
        DeclareLaunchArgument('use_speed_display', default_value='true'),
        DeclareLaunchArgument(
            'battery_state_topic',
            default_value='/j100_0519/platform/bms/state'),
        DeclareLaunchArgument('speed_odom_topic', default_value=nav_odom_topic),
        DeclareLaunchArgument('use_pedestrian_figures', default_value='false'),
        DeclareLaunchArgument('use_pedestrian_traces', default_value='false'),
        DeclareLaunchArgument('tracks_topic', default_value='/ped_tracking'),
        DeclareLaunchArgument('traces_topic', default_value='/ped_traces'),
        DeclareLaunchArgument('pedestrian_viz_params_file', default_value=default_ped_viz),

        OpaqueFunction(function=_network_preflight),
        OpaqueFunction(function=_validate_params),
        OpaqueFunction(function=_configure_rviz),

        # Stack Stability Monitor
        Node(
            package='jackal_nav2_bringup', executable='stack_stability.py',
            name='nav2_stack_stability', output='screen',
            condition=IfCondition(LaunchConfiguration('launch_stability_monitor')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'topic': LaunchConfiguration('scan_topic'), 'message_type': 'scan',
                'expected_frame': 'base_link', 'odom_topic': nav_odom_topic,
                'imu_topic': LaunchConfiguration('imu_topic'),
                'cloud_topic': lidar_pointcloud_topic,
                'fast_odom_topic': LaunchConfiguration('fast_livo_odom_topic'),
                'nav_cmd_topic': nav_cmd_vel_topic,
                'require_localization': True,
                'settle': 0.1, 'timeout': 600.0,
                'stability_timeout': ParameterValue(
                    LaunchConfiguration('stability_timeout'), value_type=float),
                'stability_settle': ParameterValue(
                    LaunchConfiguration('stability_settle'), value_type=float),
            }],
        ),

        # Costmap components
        Node(
            package='jackal_nav2_bringup',
            executable='static_costmap_node',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            condition=IfCondition(use_map_patch),
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
            condition=IfCondition(use_map_patch),
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['/static_costmap/static_costmap'],
                'bond_timeout': 0.0,
            }],
        ),

        # Nav2 Core Servers
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
                    **{**common, 'parameters': bt_parameters},
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
                        'bond_timeout': 0.0,
                    }],
                ),
            ],
        ),

        # Safety Stack (integrated from safety.launch.py)
        OpaqueFunction(function=_setup_safety),

        # Visualization & UI Overlays (integrated from nav_bringup.launch.py)
        Node(
            package='rviz2',
            executable='rviz2',
            name='jackal_nav2_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', LaunchConfiguration('resolved_rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='rviz_overlay_bridge.py',
            name='rviz_overlay_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ui_overlays')),
            parameters=[{
                'battery_input_topic': LaunchConfiguration('battery_state_topic'),
                'battery_output_topic': '/nav2/battery_percentage',
                'enable_battery_gauge': LaunchConfiguration('use_battery_gauge'),
                'speed_input_topic': LaunchConfiguration('speed_odom_topic'),
                'speed_output_topic': '/nav2/speed_overlay',
                'enable_speed_display': LaunchConfiguration('use_speed_display'),
            }],
        ),
        Node(
            package='moai_nav_viz',
            executable='pedestrian_figures_node',
            name='pedestrian_figures',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pedestrian_figures')),
            parameters=[LaunchConfiguration('pedestrian_viz_params_file'), {
                'use_sim_time': use_sim_time,
                'tracks_topic': LaunchConfiguration('tracks_topic'),
                'output_topic': '/nav2/pedestrian_figures',
                'target_frame': 'odom',
            }],
        ),
        Node(
            package='moai_nav_viz',
            executable='pedestrian_traces_node',
            name='pedestrian_traces',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pedestrian_traces')),
            parameters=[LaunchConfiguration('pedestrian_viz_params_file'), {
                'use_sim_time': use_sim_time,
                'tracks_topic': LaunchConfiguration('tracks_topic'),
                'traces_topic': LaunchConfiguration('traces_topic'),
                'output_topic': '/nav2/pedestrian_traces',
                'target_frame': 'odom',
            }],
        ),
    ])
