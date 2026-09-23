"""Offline process ownership and temporary kernel-setting lifecycle regressions."""

import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location('nav_session', ROOT / 'scripts/nav_session.py')
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


def test_pid_reuse_does_not_signal_unrelated_process(tmp_path, monkeypatch):
    current = SESSION.process_info(os.getpid())
    stale = {**current, 'start': '0'}
    SESSION.save_state(tmp_path, 'nav', {'owner': stale})
    signalled = []
    monkeypatch.setattr(SESSION.os, 'kill', lambda *args: signalled.append(args))
    SESSION.stop_role(tmp_path, 'nav')
    assert not signalled
    assert SESSION.same_process(current)
    assert not SESSION.same_process(stale)


@pytest.mark.parametrize('args,role', [
    (['python3', '/opt/ros/humble/bin/ros2', 'launch',
      'jackal_nav2_bringup', 'nav2.launch.py'], 'nav'),
    (['python3', '/opt/ros/humble/bin/ros2', 'launch',
      'mid360_bringup', 'perception.launch.py'], 'perception'),
    (['/install/fastlivo_mapping', '--ros-args'], 'nav'),
    (['python3', '/install/ped_yolo_node'], 'perception'),
    (['python3', '/install/nav_session.py', 'nav', '--map', '/map.yaml'], 'nav'),
    (['python3', '/install/nav_session.py', 'stop'], None),
    (['bash', '-c', 'echo ros2 launch jackal_nav2_bringup nav2.launch.py'], None),
    (['rg', 'fastlivo_mapping'], None),
])
def test_process_detection_uses_executable_not_search_text(args, role):
    # Search tools may have an executable name as their second argument.
    assert SESSION.stack_role(args) == role


def test_lock_rejects_second_session_before_launch(tmp_path, monkeypatch):
    monkeypatch.setattr(SESSION, 'require_environment', lambda: None)
    with (tmp_path / 'nav.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='already running'):
            SESSION.start(SimpleNamespace(action='nav'), tmp_path)


def test_failed_gate_does_not_launch_next_command_and_records_failure(tmp_path):
    log = tmp_path / 'logs'
    log.mkdir()
    marker = tmp_path / 'must_not_exist'
    code = SESSION.supervise('perception', [
        [sys.executable, '-c', 'raise SystemExit(7)'],
        [sys.executable, '-c', f'open({str(marker)!r}, "w").close()'],
    ], tmp_path, log)
    assert code == 7
    assert not marker.exists()
    assert json.loads((log / 'session.json').read_text())['returncode'] == 7


def test_dead_nav_parent_prevents_perception_start(tmp_path):
    log = tmp_path / 'logs'
    log.mkdir()
    parent = {**SESSION.process_info(os.getpid()), 'start': '0'}
    marker = tmp_path / 'must_not_exist'
    code = SESSION.supervise('perception', [
        [sys.executable, '-c', f'open({str(marker)!r}, "w").close()'],
    ], tmp_path, log, parent)
    assert code == 130
    assert not marker.exists()


def test_foreground_stop_reaps_owned_child_only(tmp_path):
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    log = tmp_path / 'logs'
    log.mkdir()
    runner = (
        f'import sys; sys.path.insert(0, {str(ROOT / "scripts")!r}); '
        'from pathlib import Path; import nav_session; '
        f'raise SystemExit(nav_session.supervise("perception", '
        f'[[sys.executable, "-c", "import time; time.sleep(60)"]], '
        f'Path({str(runtime)!r}), Path({str(log)!r})))')
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    process = subprocess.Popen([sys.executable, '-c', runner])
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = SESSION.read_state(runtime, 'perception')
            if state.get('child'):
                break
            time.sleep(0.05)
        assert state.get('child'), 'supervisor did not start its child'
        SESSION.stop_role(runtime, 'perception', timeout=5)
        process.wait(timeout=5)
        assert not SESSION.same_process(state['child'])
        assert unrelated.poll() is None
        assert SESSION.read_state(runtime, 'perception')['status'] == 'stopped'
    finally:
        for child in (process, unrelated):
            if child.poll() is None:
                child.send_signal(signal.SIGTERM)
            child.wait(timeout=5)


def test_nav_stop_orders_perception_before_nav(tmp_path):
    processes = []

    def launch(role, parent=None):
        log = tmp_path / role
        log.mkdir()
        command = (
            f'import sys; sys.path.insert(0, {str(ROOT / "scripts")!r}); '
            'from pathlib import Path; import nav_session; '
            f'raise SystemExit(nav_session.supervise({role!r}, '
            f'[[sys.executable, "-c", "import time; time.sleep(60)"]], '
            f'Path({str(tmp_path)!r}), Path({str(log)!r}), {parent!r}))')
        process = subprocess.Popen([sys.executable, '-c', command])
        processes.append(process)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = SESSION.read_state(tmp_path, role)
            if state.get('child'):
                return state
            time.sleep(0.05)
        pytest.fail(f'{role} did not start')

    try:
        nav = launch('nav')
        perception = launch('perception', nav['owner'])
        SESSION.stop_role(tmp_path, 'nav', timeout=8)
        for process in processes:
            process.wait(timeout=5)
        assert not SESSION.same_process(nav['child'])
        assert not SESSION.same_process(perception['child'])
        nav_end = SESSION.read_state(tmp_path, 'nav')
        perception_end = SESSION.read_state(tmp_path, 'perception')
        assert nav_end['status'] == perception_end['status'] == 'stopped'
        assert perception_end['ended_at'] <= nav_end['ended_at']
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=8)


def test_network_policy_is_consumed_without_sibling_import(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('JACKAL_NETWORK_ROLE', 'laptop')
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='{"ready":true}', stderr='')
    monkeypatch.setattr(SESSION.subprocess, 'run', run)
    SESSION.require_environment()
    assert calls[0] == [
        'ros2', 'run', 'jackal_network_bringup', 'network_preflight.py', '--check']
    monkeypatch.setattr(SESSION, 'network_status', lambda: {'ready': False})
    with pytest.raises(RuntimeError, match='Network policy'):
        SESSION.require_environment()


def test_motion_and_generated_profile_use_same_session_path(tmp_path):
    for name in ('nav2.yaml', 'safety.yaml', 'operator.yaml'):
        (tmp_path / name).write_text('{}')
    args = SimpleNamespace(map=tmp_path / 'map.yaml', enable_motion=True,
                           profile_dir=tmp_path, input_timeout=60, localization_timeout=90,
                           ready_settle=5, stability_timeout=600, stability_settle=180,
                           pedestrian_viz=False)
    command = SESSION.navigation_command(args)
    assert 'nav2.launch.py' in command
    assert 'enable_motion:=true' in command
    assert f'params_file:={tmp_path}/nav2.yaml' in command
    assert f'safety_params_file:={tmp_path}/safety.yaml' in command
    assert f'operator_params_file:={tmp_path}/operator.yaml' in command
    (tmp_path / 'safety.yaml').unlink()
    with pytest.raises(RuntimeError, match='profile missing'):
        SESSION.navigation_command(args)


def test_clean_launch_exit_with_critical_failure_marker_is_failure(tmp_path):
    log = tmp_path / 'logs'
    log.mkdir()
    (log / 'failure.json').write_text(json.dumps({'reason': 'guard exited'}))
    code = SESSION.supervise('perception', [[sys.executable, '-c', 'pass']], tmp_path, log)
    assert code == 1
    assert json.loads((log / 'session.json').read_text())['failure']['reason'] == 'guard exited'
