"""Static tests for launch, map validation, and navigation contracts."""

import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as element_tree

from launch import LaunchContext, LaunchDescription
import pytest
import yaml


os.environ.setdefault('ROS_LOG_DIR', '/tmp/jackal_nav2_bringup_test_logs')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'


def _load_launch(filename):
    path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_public_launch_files_generate_descriptions():
    for filename in (
            'bringup.launch.py',
            'nav_bringup.launch.py',
            'localization.launch.py',
            'navigation.launch.py', 'safety.launch.py'):
        description = _load_launch(filename).generate_launch_description()
        assert isinstance(description, LaunchDescription)


def test_audit_script_is_executable_for_symlink_install():
    assert os.access(PACKAGE_ROOT / 'scripts' / 'tf_localization_audit.py', os.X_OK)


def test_lidar_relay_is_default_and_routes_all_nav_consumers():
    bringup = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text(
        encoding='utf-8')
    navigation = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')

    assert "DeclareLaunchArgument('use_lidar_relay', default_value='true')" in bringup
    assert "executable='pointcloud_relay_node'" in bringup
    assert "'input_topic': raw_lidar_topic" in bringup
    assert "'output_topic': lidar_pointcloud_topic" in bringup
    assert "('cloud_in', lidar_pointcloud_topic)" in (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text(
            encoding='utf-8')
    assert "'lidar_pointcloud_topic': lidar_pointcloud_topic" in navigation
    assert "('/livox/lidar', lidar_pointcloud_topic)" not in navigation


def test_bringup_composes_nav2_by_default_and_forwards_container_settings():
    bringup = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text(
        encoding='utf-8')
    localization = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text(
            encoding='utf-8')
    navigation = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
            encoding='utf-8')

    assert "DeclareLaunchArgument('use_composition', default_value='true')" in (
        bringup)
    assert "executable='component_container_isolated'" in bringup
    assert 'parameters=[container_params]' in bringup
    assert "'use_composition': use_composition" in bringup
    assert "'container_name': container_name" in bringup
    assert 'LoadComposableNodes(' in localization
    assert 'LoadComposableNodes(' in navigation
    assert "plugin='nav2_amcl::AmclNode'" in localization
    assert "plugin='nav2_controller::ControllerServer'" in navigation


def test_nav2_frame_topic_and_plugin_contracts():
    with CONFIG_PATH.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    amcl = config['amcl']['ros__parameters']
    assert amcl['global_frame_id'] == 'map'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['base_frame_id'] == 'base_link'
    assert amcl['scan_topic'] == '/scan'
    assert amcl['tf_broadcast'] is True
    assert amcl['set_initial_pose'] is False
    assert amcl['always_reset_initial_pose'] is True
    assert amcl['max_beams'] == 120
    assert amcl['update_min_d'] == 0.05
    assert amcl['update_min_a'] == 0.05

    projection = config['pointcloud_to_laserscan']['ros__parameters']
    assert projection['target_frame'] == 'base_link'
    assert projection['min_height'] == 0.10
    assert projection['max_height'] == 1.80
    assert projection['angle_min'] < -3.14
    assert projection['angle_max'] > 3.14
    assert projection['range_min'] == 0.20
    assert projection['range_max'] == 30.0

    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']
    static = config['static_costmap']['static_costmap']['ros__parameters']
    map_patch = config['map_patch_node']['ros__parameters']
    assert local['global_frame'] == 'odom'
    assert local['rolling_window'] is True
    assert '\n' not in local['footprint']
    for costmap in (local, static, global_costmap):
        assert costmap['plugins'] == ['static_layer', 'inflation_layer']
        assert costmap['static_layer']['map_topic'] == '/map'
        assert costmap['static_layer']['map_subscribe_transient_local'] is True
        serialized = yaml.safe_dump(costmap)
        for forbidden in ('observation_sources', 'VoxelLayer', 'ObstacleLayer',
                          '/scan', '/livox', '/ped_detection', '/ped_tracking',
                          '/ped_traces', '/nav2/pedestrian'):
            assert forbidden not in serialized
    assert (local['width'], local['height'], local['resolution']) == (4, 4, 0.05)
    assert static['global_frame'] == 'map'
    assert static['rolling_window'] is False
    assert static['plugins'] == ['static_layer', 'inflation_layer']
    assert static['static_layer']['map_topic'] == '/map'
    assert static['static_layer']['map_subscribe_transient_local'] is True
    assert 'obstacle_layer' not in static
    assert 'voxel_layer' not in static
    assert global_costmap['global_frame'] == 'map'
    assert global_costmap['rolling_window'] is False
    assert '\n' not in global_costmap['footprint']
    assert global_costmap['plugins'] == ['static_layer', 'inflation_layer']
    assert global_costmap['static_layer']['map_topic'] == '/map'
    assert global_costmap['static_layer']['map_subscribe_transient_local'] is True
    assert 'obstacle_layer' not in global_costmap
    assert 'voxel_layer' not in global_costmap

    assert map_patch['input_topic'] == '/static_costmap/costmap'
    assert map_patch['output_topic'] == '/map_encoder/input'
    assert map_patch['map_frame'] == 'map'
    assert map_patch['base_frame'] == 'base_link'
    assert map_patch['patch_width'] == 10.0
    assert map_patch['patch_height'] == 10.0
    assert map_patch['resolution'] == 0.05
    assert map_patch['output_width'] == 200
    assert map_patch['output_height'] == 200
    assert map_patch['update_rate'] == 10.0

    controller = config['controller_server']['ros__parameters']
    assert controller['odom_topic'] == '/odom'
    assert controller['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert controller['FollowPath']['max_vel_x'] == 0.20
    assert controller['FollowPath']['max_vel_theta'] == 0.35
    assert config['planner_server']['ros__parameters']['GridBased']['plugin'] == (
        'nav2_navfn_planner/NavfnPlanner')

    plugin_names = config['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert 'nav2_round_robin_node_bt_node' in plugin_names
    assert 'nav2_path_expiring_timer_condition' in plugin_names
    assert not any('would_a_' in name for name in plugin_names)
    assert not any('error_codes' in name for name in plugin_names)


def test_navigation_launch_isolates_platform_command_output():
    text = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')
    assert "('cmd_vel', 'cmd_vel_nav')" in text
    assert "('cmd_vel_smoothed', 'nav2_cmd_vel_unstamped')" in text
    assert "default_value='/j100_0519/nav2_cmd_vel'" in text
    safety = (PACKAGE_ROOT / 'launch' / 'safety.launch.py').read_text()
    assert "'cmd_vel_in_topic': '/nav2_cmd_vel_unstamped'" in safety
    assert "'command_topic': monitor['cmd_vel_out_topic']" in safety
    assert "'output_topic': value('nav_cmd_vel_topic')" in safety
    assert "package='jackal_nav2_bringup'" in text
    assert "executable='nav2_twist_stamper.py'" not in text
    assert "executable='nav2_safety_guard.py'" in safety
    assert "DeclareLaunchArgument('enable_motion', default_value='false')" in safety
    assert 'jackal_network_bringup' not in text
    assert '/j100_0519/cmd_vel' not in text


def test_navigation_rejects_live_layers_in_override_file(tmp_path):
    config = yaml.safe_load(CONFIG_PATH.read_text())
    config['local_costmap']['local_costmap']['ros__parameters']['plugins'] = [
        'voxel_layer', 'inflation_layer']
    path = tmp_path / 'unsafe.yaml'
    path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(path)
    with pytest.raises(RuntimeError, match='only StaticLayer'):
        _load_launch('navigation.launch.py')._validate_params(context)


def test_navigation_accepts_prior_map_only_defaults():
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(CONFIG_PATH)
    assert _load_launch('navigation.launch.py')._validate_params(context) == []


def test_generated_perception_profiles_use_supported_yaml_interface(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('mid360_bringup'))
    spec = importlib.util.spec_from_file_location(
        'separate_perception_launch', share / 'launch' / 'perception.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    helper_spec = importlib.util.spec_from_file_location(
        'prepare_perception', PACKAGE_ROOT / 'scripts' / 'prepare_perception_config.py')
    helper = importlib.util.module_from_spec(helper_spec)
    helper_spec.loader.exec_module(helper)
    profile = helper.prepare(share, tmp_path / 'profile')
    context = LaunchContext()
    for name, filename in (
            ('lidar_preprocess_config', 'lidar_preprocess.yaml'),
            ('ped_yolo_config', 'ped_yolo.yaml'),
            ('extractor_config', 'mask_3d_extractor.yaml'),
            ('tracker_config', 'pedestrian_tracker.yaml')):
        context.launch_configurations[name] = str(share / 'config' / filename)
    context.launch_configurations.update(
        lidar_input_topic='__default__', tracking_frame='__default__')
    config = module._topic_configuration(context)
    assert config['lidar_input'] == '/livox/lidar'
    # Current upstream removed the convenience args. Test the real YAML inputs
    # instead of accepting an ignored lidar_input_topic/tracking_frame argument.
    original = yaml.safe_load((share / 'config' / 'mask_3d_extractor.yaml').read_text())
    assert original['mid360_mask_3d_extractor_node']['ros__parameters']['tracking_frame'] == 'base_link'
    context.launch_configurations.update(
        lidar_preprocess_config=str(profile / 'lidar_preprocess.yaml'),
        extractor_config=str(profile / 'mask_3d_extractor.yaml'))
    config = module._topic_configuration(context)
    assert config['lidar_input'] == '/livox/lidar_local'
    params = module._load_ros_parameters(
        str(profile / 'mask_3d_extractor.yaml'), 'mid360_mask_3d_extractor_node')
    assert params['tracking_frame'] == 'odom'
    assert params['use_latest_tf_fallback'] is False
    module._validate_topic_config(config, launch_ped_yolo=True)


def test_bringup_requires_operator_initial_pose_and_rviz_publishes_it():
    bringup = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text(
        encoding='utf-8')
    rviz = yaml.safe_load(
        (PACKAGE_ROOT / 'rviz' / 'jackal_nav2.rviz').read_text(
            encoding='utf-8'))

    assert "DeclareLaunchArgument('use_rviz', default_value='true')" in bringup
    assert 'Initial pose required' in bringup
    assert '/initialpose' in bringup

    tools = rviz['Visualization Manager']['Tools']
    initial_pose_tools = [
        tool for tool in tools
        if tool.get('Class') == 'rviz_default_plugins/SetInitialPose'
    ]
    assert len(initial_pose_tools) == 1
    initial_pose_tool = initial_pose_tools[0]
    assert initial_pose_tool['Topic']['Value'] == '/initialpose'
    assert initial_pose_tool['Covariance x'] == pytest.approx(0.25)
    assert initial_pose_tool['Covariance y'] == pytest.approx(0.25)
    assert initial_pose_tool['Covariance yaw'] == pytest.approx(
        0.06853891909122467)


def test_nav2_visualization_defaults_are_independent_of_perception_and_safety():
    text = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text()
    for name in ('figures', 'traces'):
        assert f"DeclareLaunchArgument('use_pedestrian_{name}', default_value='true')" in text
        assert f"executable='pedestrian_{name}_node'" in text
        assert f"'output_topic': '/nav2/pedestrian_{name}'" in text
        assert f"condition=IfCondition(LaunchConfiguration('use_pedestrian_{name}'))" in text
    assert "DeclareLaunchArgument('traces_topic', default_value='/ped_traces')" in text
    assert text.count("parameters=[LaunchConfiguration('pedestrian_viz_params_file'), {") == 2
    assert "'traces_topic': LaunchConfiguration('traces_topic')" in text
    assert 'perception.launch.py' not in text
    assert "DeclareLaunchArgument('enable_motion', default_value='false')" in text
    viz = yaml.safe_load((PACKAGE_ROOT / 'config' / 'pedestrian_viz.yaml').read_text())
    assert viz['pedestrian_figures']['ros__parameters'] == {
        'figure_scale': 0.5, 'label_height': 0.18}
    trace = viz['pedestrian_traces']['ros__parameters']
    assert trace['history_seconds'] == 3.0 and trace['max_points'] == 100
    assert trace['line_width'] == 0.03 and trace['ground_z'] == 0.02
    assert trace['input_timeout'] == 1.0 and trace['marker_lifetime'] == 0.3
    rviz = yaml.safe_load((PACKAGE_ROOT / 'rviz' / 'jackal_nav2.rviz').read_text())
    displays = {d['Name']: d for d in rviz['Visualization Manager']['Displays']}
    for name, topic in (('Pedestrian Figures', '/nav2/pedestrian_figures'),
                        ('Pedestrian Traces', '/nav2/pedestrian_traces')):
        assert displays[name]['Enabled'] is True
        assert displays[name]['Topic']['Value'] == topic
    assert displays['Debug Track Traces']['Enabled'] is False


def test_navigation_launch_starts_separated_costmap_pipeline():
    text = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')
    assert "executable='static_costmap_node'" in text
    assert "'/static_costmap/static_costmap'" in text
    assert "name='lifecycle_manager_static_costmap'" in text
    assert "'bond_timeout': 0.0" in text
    assert "executable='map_patch_node.py'" in text
    assert "DeclareLaunchArgument('use_map_patch', default_value='true')" in text
    assert "remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]" in text


def test_localization_launch_keeps_tf_at_root_and_adapter_tf_free():
    localization = (PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text(
        encoding='utf-8')
    adapter = (PACKAGE_ROOT / 'scripts' / 'fast_livo_odom_adapter.py').read_text(
        encoding='utf-8')

    assert "tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]" in localization
    assert "'expected_frame_id': 'odom'" in localization
    assert "'expected_child_frame_id': 'base_link'" in localization
    assert "package='pointcloud_to_laserscan'" in localization
    assert "executable='pointcloud_to_laserscan_node'" in localization
    assert "('cloud_in', lidar_pointcloud_topic)" in localization
    assert "('scan', scan_topic)" in localization
    assert "DeclareLaunchArgument('use_scan_projection', default_value='true')" in (
        localization)
    assert 'TransformBroadcaster' not in adapter
    assert 'sendTransform' not in adapter


def _validation_context(map_path, params_path=CONFIG_PATH):
    context = LaunchContext()
    context.launch_configurations['map'] = str(map_path)
    context.launch_configurations['params_file'] = str(params_path)
    return context


def test_map_validation_accepts_relative_image_path(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\x00')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        encoding='utf-8')
    localization = _load_launch('localization.launch.py')

    assert localization._validate_inputs(
        _validation_context(map_yaml)) == []


def test_map_validation_rejects_relative_yaml_path():
    localization = _load_launch('localization.launch.py')
    with pytest.raises(RuntimeError, match='absolute path'):
        localization._validate_inputs(
            _validation_context(Path('relative/map.yaml')))


def test_map_validation_rejects_missing_image(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: missing.pgm\n', encoding='utf-8')
    localization = _load_launch('localization.launch.py')

    with pytest.raises(RuntimeError, match='map image'):
        localization._validate_inputs(_validation_context(map_yaml))


def test_manifest_declares_runtime_and_test_dependencies():
    root = element_tree.parse(PACKAGE_ROOT / 'package.xml').getroot()
    runtime_dependencies = {
        element.text for element in root.findall('exec_depend')
    }
    assert {
        'geometry_msgs',
        'nav2_bringup',
        'nav_msgs',
        'pointcloud_to_laserscan',
        'python3-yaml',
        'rclpy',
        'rclcpp_components',
        'rviz2',
        'tf2_ros',
    }.issubset(runtime_dependencies)
    assert 'jackal_network_bringup' not in runtime_dependencies
    combined_dependencies = {
        element.text for tag in ('depend', 'exec_depend')
        for element in root.findall(tag)
    }
    assert {'nav2_costmap_2d', 'rclcpp', 'sensor_msgs'}.issubset(
        combined_dependencies)
    assert root.find("test_depend[.='ament_cmake_pytest']") is not None
