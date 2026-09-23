"""Navigation launch RViz configuration and legacy launch elimination tests."""

import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
import yaml


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_LOG_DIR', '/tmp/jackal_nav2_bringup_test_logs')
SPEC = importlib.util.spec_from_file_location(
    'navigation_launch', ROOT / 'launch/nav2.launch.py')
NAV = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV)


def test_rviz_camera_is_opt_in_without_modifying_original(tmp_path, monkeypatch):
    path = ROOT / 'rviz/jackal_nav2.rviz'
    before = path.read_bytes()
    monkeypatch.setattr(NAV.tempfile, 'tempdir', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(
        rviz_config=str(path), use_rviz='true', use_camera_image='false')
    for action in NAV._configure_rviz(context):
        action.execute(context)
    generated = Path(context.launch_configurations['resolved_rviz_config'])
    assert generated != path
    displays = yaml.safe_load(generated.read_text())['Visualization Manager']['Displays']
    camera = [d for d in displays if d.get('Class') == 'rviz_default_plugins/Image']
    assert camera and all(d['Enabled'] is False for d in camera)
    assert path.read_bytes() == before
    context.launch_configurations['use_camera_image'] = 'true'
    for action in NAV._configure_rviz(context):
        action.execute(context)
    assert context.launch_configurations['resolved_rviz_config'] == str(path)


def test_legacy_bringup_and_perception_launches_are_eliminated():
    assert not (ROOT / 'launch/bringup.launch.py').exists()
    assert not (ROOT / 'launch/nav_bringup.launch.py').exists()
    assert not (ROOT / 'launch/perception.launch.py').exists()
    assert not (ROOT / 'launch/safety.launch.py').exists()
    assert not (ROOT / 'launch/network_preflight.launch.py').exists()
