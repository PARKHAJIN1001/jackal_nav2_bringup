"""Readiness contracts: idle silence, stale inputs and actual bridge ownership."""

import os
from pathlib import Path
import sys

from geometry_msgs.msg import Transform
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
os.environ.setdefault('ROS_LOG_DIR', '/tmp/jackal_nav2_bringup_test_logs')
from check_navigation_ready import (  # noqa: E402,I100
    bridge_ready, fresh, safety_ready, valid_transform,
)


@pytest.mark.parametrize('stamp,receipt,now,mono,expected', [
    (10.0, 1.0, 10.1, 1.1, True),
    (10.0, 1.0, 10.31, 1.1, False),
    (10.0, 1.0, 10.1, 1.31, False),
    (10.0, 1.0, 9.8, 1.1, False),
    (10.0, 1.0, 10.1, 0.9, False),
    (float('nan'), 1.0, 10.1, 1.1, False),
    (0.0, 1.0, 0.1, 1.1, False),
])
def test_measurement_and_receipt_freshness(stamp, receipt, now, mono, expected):
    assert fresh(stamp, receipt, now, mono) is expected


@pytest.mark.parametrize('reason', [
    'sensor_missing', 'sensor_timestamp', 'sensor_receipt_timeout', 'independent_stop',
    'clock_regression', 'required_tf_missing_or_invalid', 'tf_stale:odom->base_link',
])
def test_idle_never_masks_safety_fault(reason):
    assert not safety_ready(reason, False)
    assert not safety_ready(reason, True)


@pytest.mark.parametrize('reason', ['command_missing_or_invalid', 'monitor_command_timeout'])
def test_command_silence_is_only_acceptable_while_idle(reason):
    assert safety_ready(reason, False)
    assert not safety_ready(reason, True)
    assert safety_ready('motion_disabled', False)


def test_parameter_value_alone_cannot_prove_forwarding():
    parameters = {'forward_cmd_vel': True, 'input_topic': '/nav', 'output_topic': '/platform'}
    assert bridge_ready(parameters, ['/bridge'], '/nav', '/platform', '/bridge')
    for writers in ([], ['/other'], ['/bridge', '/bridge'], ['/bridge', '/other']):
        assert not bridge_ready(parameters, writers, '/nav', '/platform', '/bridge')
    assert not bridge_ready(parameters, ['/bridge'], '/different', '/platform', '/bridge')
    parameters['forward_cmd_vel'] = False
    assert not bridge_ready(parameters, ['/bridge'], '/nav', '/platform', '/bridge')


def test_transform_validation():
    tf = Transform()
    tf.rotation.w = 0.0
    assert not valid_transform(tf)
    tf.rotation.w = 1.0
    assert valid_transform(tf)
    tf.translation.x = float('nan')
    assert not valid_transform(tf)


def test_native_goal_ui_and_default_bt_contract():
    import importlib.util
    import xml.etree.ElementTree as ET

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    import yaml

    root = Path(__file__).resolve().parents[1]
    profile = yaml.safe_load((root / 'rviz/jackal_nav2.rviz').read_text())
    assert any(p['Class'] == 'nav2_rviz_plugins/Navigation 2' for p in profile['Panels'])
    tools = profile['Visualization Manager']['Tools']
    assert sum(t['Class'] == 'nav2_rviz_plugins/GoalTool' for t in tools) == 1
    assert not any(t['Class'] == 'rviz_default_plugins/SetGoal' for t in tools)
    xml = ET.parse(Path(get_package_share_directory('nav2_rviz_plugins')) /
                   'plugins_description.xml')
    names = {c.attrib['name'] for c in xml.findall('class')}
    assert {'nav2_rviz_plugins/Navigation 2', 'nav2_rviz_plugins/GoalTool'} <= names
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', root / 'launch/nav2.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    # Resolve the real per-node override, including an alternate params file
    # that does not declare default_nav_to_pose_bt_xml.
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    context.launch_configurations.update(
        params_file=str(root / 'config/nav2_params.yaml'), use_sim_time='false',
        autostart='true', nav_odom_topic='/odom')
    nodes = [child for action in description.entities
             for child in action.get_sub_entities() if isinstance(child, Node)]
    navigator = next(n for n in nodes if n.node_executable == 'bt_navigator')
    parameters = evaluate_parameters(context, navigator._Node__parameters)
    tree = ET.parse(parameters[-1]['default_nav_to_pose_bt_xml'])
    tags = {e.tag for e in tree.iter()}
    assert {'ComputePathToPose', 'FollowPath'} <= tags
    assert not {'Spin', 'BackUp', 'RecoveryNode', 'ClearEntireCostmap'} & tags
    assert tree.find('.//RateController').attrib['hz'] == '1.0'


def test_missing_or_invalid_bridge_limits_cannot_pass_motion_check():
    from check_navigation_ready import speed_limits_match
    guard = {'max_linear_x': .2, 'max_angular_z': .35}
    controller = {'FollowPath.max_vel_x': .2, 'FollowPath.max_vel_theta': .35}
    smoother = {'max_velocity': [.2, 0., .35]}
    assert speed_limits_match(guard, guard, controller, smoother)
    for invalid in (None, True, float('nan'), -1, 0):
        bridge = {**guard, 'max_linear_x': invalid}
        assert not speed_limits_match(guard, bridge, controller, smoother)
    assert not speed_limits_match(guard, guard, controller, {'max_velocity': [.5, 0., 1.]})
