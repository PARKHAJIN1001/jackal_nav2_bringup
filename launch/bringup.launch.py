#!/usr/bin/env python3

"""Launch Jackal localization and navigation around external FAST-LIVO2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name


def _configure_lidar(context):
    """Resolve one cloud path before starting any node; reject relay loops."""
    enabled = IfCondition(LaunchConfiguration('use_lidar_relay')).evaluate(context)
    raw = LaunchConfiguration('raw_lidar_topic').perform(context).strip()
    output = LaunchConfiguration('lidar_pointcloud_topic').perform(context).strip()
    if not raw:
        raise RuntimeError('raw_lidar_topic must not be empty')
    output = output or ('/livox/lidar_local' if enabled else raw)
    raw = expand_topic_name(raw, 'pointcloud_relay', '/')
    output = expand_topic_name(output, 'pointcloud_relay', '/')
    validate_full_topic_name(raw)
    validate_full_topic_name(output)
    if enabled and raw == output:
        raise RuntimeError('relay input and output must be different topics')
    return [
        SetLaunchConfiguration('raw_lidar_topic', raw),
        SetLaunchConfiguration('lidar_pointcloud_topic', output),
        LogInfo(msg=(
            f'[jackal_nav2_bringup] Nav2 cloud input: {output}; '
            f'relay {"enabled" if enabled else "disabled"}. '
            'Configure external FAST-LIVO2/perception explicitly with this input.')),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('jackal_nav2_bringup')
    launch_dir = os.path.join(package_share, 'launch')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(package_share, 'rviz', 'jackal_nav2.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    fast_livo_odom_topic = LaunchConfiguration('fast_livo_odom_topic')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    nav_cmd_vel_topic = LaunchConfiguration('nav_cmd_vel_topic')
    use_map_patch = LaunchConfiguration('use_map_patch')
    use_scan_projection = LaunchConfiguration('use_scan_projection')
    use_lidar_relay = LaunchConfiguration('use_lidar_relay')
    raw_lidar_topic = LaunchConfiguration('raw_lidar_topic')
    lidar_pointcloud_topic = LaunchConfiguration('lidar_pointcloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    container_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=None,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'yaml_filename': map_yaml,
                'odom_topic': nav_odom_topic,
                'scan_topic': scan_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    shared_arguments = {
        'params_file': params_file,
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'use_respawn': use_respawn,
        'use_composition': use_composition,
        'container_name': container_name,
        'log_level': log_level,
        'nav_odom_topic': nav_odom_topic,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the occupancy-grid map YAML'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'fast_livo_odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_map_patch', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('use_amcl_quality_monitor', default_value='true'),
        DeclareLaunchArgument('safety_params_file', default_value=os.path.join(
            package_share, 'config', 'nav2_safety.yaml')),
        DeclareLaunchArgument('use_pedestrian_figures', default_value='true'),
        DeclareLaunchArgument('use_pedestrian_traces', default_value='true'),
        DeclareLaunchArgument('tracks_topic', default_value='/ped_tracking'),
        DeclareLaunchArgument('traces_topic', default_value='/ped_traces'),
        DeclareLaunchArgument('pedestrian_viz_params_file', default_value=os.path.join(
            package_share, 'config', 'pedestrian_viz.yaml')),
        DeclareLaunchArgument('use_scan_projection', default_value='true'),
        DeclareLaunchArgument('use_lidar_relay', default_value='true'),
        DeclareLaunchArgument(
            'raw_lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic', default_value='',
            description='Auto: relay output when enabled, raw_lidar_topic otherwise; '
                        'set explicitly for an external/custom relay'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/j100_0519/nav2_cmd_vel'),
        OpaqueFunction(function=_configure_lidar),
        ComposableNodeContainer(
            condition=IfCondition(use_composition),
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            # Child nodes (notably controller costmaps) inherit process-level
            # parameters from the component container.
            parameters=[container_params],
        ),
        Node(
            package='jackal_nav2_bringup',
            executable='pointcloud_relay_node',
            name='pointcloud_relay',
            output='screen',
            condition=IfCondition(use_lidar_relay),
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': raw_lidar_topic,
                'output_topic': lidar_pointcloud_topic,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'localization.launch.py')),
            launch_arguments={
                **shared_arguments,
                'map': map_yaml,
                'fast_livo_odom_topic': fast_livo_odom_topic,
                'use_scan_projection': use_scan_projection,
                'lidar_pointcloud_topic': lidar_pointcloud_topic,
                'scan_topic': scan_topic,
                'use_amcl_quality_monitor': LaunchConfiguration('use_amcl_quality_monitor'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={
                **shared_arguments,
                'nav_cmd_vel_topic': nav_cmd_vel_topic,
                'enable_motion': LaunchConfiguration('enable_motion'),
                'safety_params_file': LaunchConfiguration('safety_params_file'),
                'use_map_patch': use_map_patch,
                'lidar_pointcloud_topic': lidar_pointcloud_topic,
            }.items(),
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='jackal_nav2_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('enable_motion')),
            msg='[jackal_nav2_bringup] Motion DISABLED: Guard outputs zero commands.',
        ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('enable_motion')),
            msg='[jackal_nav2_bringup] Motion ENABLED by operator override; '
                'verify platform forwarding, E-stop and attended test area.',
        ),
        LogInfo(
            condition=IfCondition(use_rviz),
            msg=(
                '[jackal_nav2_bringup] Initial pose required: once the map is '
                'visible and /scan and /odom are arriving, select "2D Pose '
                'Estimate", click the Jackal position, and drag the arrow '
                'in the robot forward direction. Scan display in the map '
                'frame becomes available AFTER this initialization.'
            ),
        ),
        LogInfo(
            condition=UnlessCondition(use_rviz),
            msg=(
                '[jackal_nav2_bringup] Initial pose required, but RViz is '
                'disabled. Publish geometry_msgs/PoseWithCovarianceStamped '
                'on /initialpose before sending a navigation goal.'
            ),
        ),
    ])
