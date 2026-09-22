#!/usr/bin/env python3
"""
Unified staged launch: Relay -> FAST-LIVO2 -> Nav2.

Replaces the three-terminal manual procedure with readiness gates:
  Phase 1  Relay + RViz + visualizers       (immediate)
  Phase 2  FAST-LIVO2                       (after fresh cloud + IMU continuity)
  Phase 3  Nav2 localization + navigation   (after valid odometry continuity)
  Monitor  Full-stack continuity          (600s deadline, 180s continuous health)
  Ready    Stable + initialized navigation; continuously revoked on faults

Perception and initial pose remain separate steps.
All Nav2 nodes run as standalone processes (no composition container).
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name
import yaml


def _validate_timing(context):
    for timeout, settle in (('input_timeout', 'ready_settle'),
                            ('localization_timeout', 'ready_settle'),
                            ('stability_timeout', 'stability_settle')):
        limit, hold = (float(LaunchConfiguration(n).perform(context)) for n in (timeout, settle))
        if not all(math.isfinite(v) and v > 0 for v in (limit, hold)) or hold >= limit:
            raise RuntimeError(f'{settle} must be positive and less than {timeout}')
    return []


def _resolve_lidar(context):
    """Resolve and validate LiDAR topic."""
    output = LaunchConfiguration('lidar_pointcloud_topic').perform(context).strip()
    if not output:
        output = '/livox/lidar_local'
    output = expand_topic_name(output, 'nav_bringup', '/')
    validate_full_topic_name(output)
    return [
        SetLaunchConfiguration('lidar_pointcloud_topic', output),
        LogInfo(msg=f'[nav_bringup] LiDAR pointcloud input: {output}'),
    ]


def _configure_rviz(context):
    """Disable only the remote raw camera display in a temporary RViz profile."""
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


def _advance(event, context, phase, actions):
    """Prevent a cancelled gate or launch shutdown from starting another phase."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        reason = f'{phase} readiness failed (exit {event.returncode}); inspect gate logs'
        return [LogInfo(msg='[nav_bringup] ' + reason), Shutdown(reason=reason)]
    return actions


def generate_launch_description():
    nav2_share = get_package_share_directory('jackal_nav2_bringup')
    launch_dir = os.path.join(nav2_share, 'launch')
    default_params = os.path.join(nav2_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(nav2_share, 'rviz', 'jackal_nav2.rviz')

    # Frequently used substitutions
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    raw_lidar_topic = LaunchConfiguration('raw_lidar_topic')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    fast_livo_odom_topic = LaunchConfiguration('fast_livo_odom_topic')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    nav_cmd_vel_topic = LaunchConfiguration('nav_cmd_vel_topic')
    use_map_patch = LaunchConfiguration('use_map_patch')

    # ================================================================
    # Phase 1 — lightweight nodes, start immediately
    # ================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='jackal_nav2_rviz',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', LaunchConfiguration('resolved_rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pedestrian_figures = Node(
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
    )

    pedestrian_traces = Node(
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
    )

    battery_bridge = Node(
        package='jackal_nav2_bringup',
        executable='battery_percentage_bridge.py',
        name='battery_percentage_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_battery_gauge')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('battery_state_topic'),
            'output_topic': '/nav2/battery_percentage',
        }],
    )

    speed_overlay = Node(
        package='jackal_nav2_bringup',
        executable='speed_overlay.py',
        name='speed_overlay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_speed_display')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('speed_odom_topic'),
        }],
    )
    odom_gate = Node(
        package='jackal_nav2_bringup',
        executable='topic_ready_gate.py',
        name='_odom_gate',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic': fast_livo_odom_topic,
            'message_type': 'odom',
            'expected_frame': 'odom',
            'expected_child_frame': 'base_link',
            'max_position_norm': 0.0,
            'timeout': ParameterValue(LaunchConfiguration('input_timeout'), value_type=float),
            'settle': ParameterValue(LaunchConfiguration('ready_settle'), value_type=float),
        }],
    )

    # ================================================================
    # Phase 3 — Nav2 localization + navigation (deferred until odom ready)
    # All nodes run as standalone processes (use_composition:=false).
    # odom_adapter is launched cleanly by localization.launch.py.
    # ================================================================

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_respawn': 'false',
            'use_composition': 'false',
            'log_level': log_level,
            'fast_livo_odom_topic': fast_livo_odom_topic,
            'nav_odom_topic': nav_odom_topic,
            'use_scan_projection': 'true',
            'lidar_pointcloud_topic': lidar_pointcloud_topic,
            'scan_topic': scan_topic,
            'use_amcl_quality_monitor':
                LaunchConfiguration('use_amcl_quality_monitor'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_respawn': 'false',
            'use_composition': 'false',
            'log_level': log_level,
            'nav_odom_topic': nav_odom_topic,
            'nav_cmd_vel_topic': nav_cmd_vel_topic,
            'scan_topic': scan_topic,
            'imu_topic': LaunchConfiguration('imu_topic'),
            'fast_livo_odom_topic': fast_livo_odom_topic,
            'stability_timeout': LaunchConfiguration('stability_timeout'),
            'stability_settle': LaunchConfiguration('stability_settle'),
            'enable_motion': LaunchConfiguration('enable_motion'),
            'safety_params_file': LaunchConfiguration('safety_params_file'),
            'operator_params_file': LaunchConfiguration('operator_params_file'),
            'launch_operator_stop': LaunchConfiguration('launch_operator_stop'),
            'use_map_patch': use_map_patch,
            'lidar_pointcloud_topic': lidar_pointcloud_topic,
        }.items(),
    )

    # ================================================================
    # Phase transition handlers
    # ================================================================

    localization_gate = Node(
        package='jackal_nav2_bringup',
        executable='topic_ready_gate.py',
        name='_localization_gate',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic': scan_topic,
            'message_type': 'scan',
            'expected_frame': 'base_link',
            'odom_topic': nav_odom_topic,
            'require_localization': True,
            'timeout': ParameterValue(
                LaunchConfiguration('localization_timeout'), value_type=float),
            'settle': ParameterValue(LaunchConfiguration('ready_settle'), value_type=float),
        }],
    )

    perception_profiles = ExecuteProcess(
        cmd=['ros2', 'run', 'jackal_nav2_bringup', 'prepare_perception_config.py',
             '--lidar-topic', lidar_pointcloud_topic, '--use-sim-time', use_sim_time],
        output='screen',
    )

    def on_odom_ready(event, context):
        return _advance(event, context, 'FAST-LIVO2 odometry', [
            LogInfo(msg='[nav_bringup] Odometry active: starting Nav2 localization & navigation'),
            localization,
            navigation,
            localization_gate,
        ])

    def on_localization_ready(event, context):
        managed = IfCondition(LaunchConfiguration('managed_session')).evaluate(context)
        next_step = (
            'run ros2 run jackal_nav2_bringup nav_session.py perception '
            '--initial-pose-confirmed in a separately prepared terminal.'
            if managed else
            'run the printed perception command in a separate terminal '
            'with the same ROS/workspace/network environment.')
        return _advance(event, context, 'localization inputs', [
            LogInfo(
                msg='[nav_bringup] AMCL/map_server active; '
                    'fresh scan, odom and scan-time TF inputs ready (not motion readiness). '
                    'Full-stack stability monitor is still collecting evidence. '
                    'Wait for INITIAL_POSE_REQUIRED, then set initial pose '
                    'with RViz 2D Pose Estimate, '
                    'check scan/map alignment, then ' + next_step + ' '
                    'Keep the robot stationary during startup. After alignment, run '
                    'ros2 run jackal_nav2_bringup check_navigation_ready.py '
                    '(add --require-motion for an explicitly motion-enabled session).',
            ),
        ] + ([] if managed else [perception_profiles]))

    # ================================================================
    # Assemble launch description
    # ================================================================

    return LaunchDescription([
        # --- Arguments: Nav2 ---
        DeclareLaunchArgument(
            'map', description='Absolute path to occupancy-grid map YAML'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'use_camera_image', default_value='false',
            description='Enable the remote raw camera display in RViz'),
        DeclareLaunchArgument(
            'ready_settle', default_value='2.0',
            description='Seconds of continuous valid measurements required by each gate'),
        DeclareLaunchArgument('input_timeout', default_value='60.0'),
        DeclareLaunchArgument('localization_timeout', default_value='90.0'),
        DeclareLaunchArgument('stability_timeout', default_value='600.0'),
        DeclareLaunchArgument('stability_settle', default_value='2.0'),
        DeclareLaunchArgument('use_battery_gauge', default_value='true'),
        DeclareLaunchArgument('use_speed_display', default_value='true'),
        DeclareLaunchArgument(
            'battery_state_topic',
            default_value='/j100_0519/platform/bms/state'),
        DeclareLaunchArgument(
            'fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'speed_odom_topic', default_value=nav_odom_topic),
        DeclareLaunchArgument('use_map_patch', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument(
            'launch_operator_stop', default_value=LaunchConfiguration('enable_motion')),
        DeclareLaunchArgument('operator_params_file', default_value=os.path.join(
            get_package_share_directory('jackal_nav2_bringup'), 'config', 'operator_stop.yaml')),
        DeclareLaunchArgument(
            'managed_session', default_value='false',
            description='Use nav_session instructions; that tool owns perception profiles'),
        DeclareLaunchArgument('use_amcl_quality_monitor', default_value='true'),
        DeclareLaunchArgument('safety_params_file', default_value=os.path.join(
            nav2_share, 'config', 'nav2_safety.yaml')),
        DeclareLaunchArgument('use_pedestrian_figures', default_value='false'),
        DeclareLaunchArgument('use_pedestrian_traces', default_value='false'),
        DeclareLaunchArgument('tracks_topic', default_value='/ped_tracking'),
        DeclareLaunchArgument('traces_topic', default_value='/ped_traces'),
        DeclareLaunchArgument('pedestrian_viz_params_file',
                              default_value=os.path.join(
                                  nav2_share, 'config',
                                  'pedestrian_viz.yaml')),
        DeclareLaunchArgument('raw_lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='/livox/lidar_local'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'),

        # --- Arguments: FAST-LIVO2 ---
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('image_enable', default_value='false'),

        # --- Topic resolution ---
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('jackal_nav2_bringup'),
            'launch', 'network_preflight.launch.py'))),
        OpaqueFunction(function=_validate_timing),
        OpaqueFunction(function=_resolve_lidar),
        OpaqueFunction(function=_configure_rviz),

        # Register before any process can start or exit.
        RegisterEventHandler(OnProcessExit(
            target_action=odom_gate, on_exit=on_odom_ready)),
        RegisterEventHandler(OnProcessExit(
            target_action=localization_gate, on_exit=on_localization_ready)),

        # --- Phase 1: Immediate ---
        LogInfo(msg='[nav_bringup] Phase 1: visualizers + RViz + waiting for odometry from perception...'),
        rviz_node,
        pedestrian_figures,
        pedestrian_traces,
        battery_bridge,
        speed_overlay,
        odom_gate,

        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('enable_motion')),
            msg='[nav_bringup] Motion DISABLED.',
        ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('enable_motion')),
            msg='[nav_bringup] Motion ENABLED by operator override.',
        ),

    ])
