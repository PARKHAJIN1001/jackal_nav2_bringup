"""Staged launch cancellation, child exit, and RViz subscription regressions."""

import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import Shutdown
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_LOG_DIR', '/tmp/jackal_nav2_bringup_test_logs')
SPEC = importlib.util.spec_from_file_location('nav_bringup', ROOT / 'launch/nav_bringup.launch.py')
NAV = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV)


@pytest.mark.parametrize('code', [1, 130, -2, -9])
def test_failed_or_cancelled_gate_shuts_down_without_starting_next_phase(code):
    next_phase = object()
    result = NAV._advance(SimpleNamespace(returncode=code),
                          SimpleNamespace(is_shutdown=False), 'test', [next_phase])
    assert next_phase not in result
    assert any(isinstance(action, Shutdown) for action in result)


def test_success_during_shutdown_cannot_advance():
    event = SimpleNamespace(returncode=0)
    phase = [object()]
    assert NAV._advance(event, SimpleNamespace(is_shutdown=True), 'test', phase) == []
    assert NAV._advance(event, SimpleNamespace(is_shutdown=False), 'test', phase) == phase


@pytest.mark.parametrize('executable', ['fastlivo_mapping', 'pointcloud_relay_node'])
@pytest.mark.parametrize('code', [0, 1, -6])
def test_critical_child_exit_stops_stack(executable, code):
    event = SimpleNamespace(cmd=['/install/lib/pkg/' + executable], returncode=code)
    result = NAV._critical_process_exit(event, SimpleNamespace(is_shutdown=False))
    assert any(isinstance(action, Shutdown) for action in result)
    assert NAV._critical_process_exit(event, SimpleNamespace(is_shutdown=True)) == []


def test_noncritical_child_exit_does_not_stop_stack():
    event = SimpleNamespace(cmd=['/install/lib/pkg/topic_ready_gate.py'], returncode=0)
    assert NAV._critical_process_exit(event, SimpleNamespace(is_shutdown=False)) == []


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


def test_integrated_fast_livo_rejects_unimplemented_output_remap():
    context = LaunchContext()
    context.launch_configurations.update(
        raw_lidar_topic='/raw', lidar_pointcloud_topic='/local', fast_livo_odom_topic='/other')
    with pytest.raises(RuntimeError, match='requires fast_livo_odom_topic'):
        NAV._resolve_lidar(context)
