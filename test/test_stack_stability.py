"""Offline time-controlled tests; never start ROS nodes, drivers or robot processes."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from readiness_core import FreshWindow, StabilityWindow  # noqa: E402,I100
from operator_stop_core import StopHeartbeat  # noqa: E402,I100


def advance(window, start, end, healthy=True, pose=True):
    for tick in range(round(start * 10), round(end * 10) + 1):
        phase = window.update(healthy, pose, tick / 10)
    return phase


def test_full_180_seconds_then_pose_then_ready():
    window = StabilityWindow()
    assert advance(window, 0, 179.9, pose=False) == 'STABILIZING'
    assert window.update(True, False, 180) == 'INITIAL_POSE_REQUIRED'
    # Waiting for human initialpose is not a new transport timeout.
    assert advance(window, 180.1, 700, pose=False) == 'INITIAL_POSE_REQUIRED'
    assert window.update(True, True, 700.1) == 'READY'


def test_failure_restarts_continuous_hold_and_never_reuses_prior_credit():
    window = StabilityWindow()
    advance(window, 0, 179)
    assert window.update(False, True, 179.1) == 'INPUT_WAITING'
    assert advance(window, 179.2, 359.1) == 'STABILIZING'
    assert window.update(True, True, 359.2) == 'READY'
    assert window.resets == 1


def test_timeout_is_separate_from_settle_and_failed_is_latched():
    window = StabilityWindow()
    assert advance(window, 0, 599.9, healthy=False) == 'INPUT_WAITING'
    assert window.update(True, True, 600) == 'FAILED'
    assert advance(window, 600.1, 900) == 'FAILED'
    for timeout, settle in ((60, 180), (180, 180), (600, float('nan')), (0, 1)):
        with pytest.raises(ValueError):
            StabilityWindow(timeout, settle)


def test_recovery_has_new_600_second_deadline_and_pose_loss_revokes_ready():
    window = StabilityWindow()
    assert advance(window, 0, 800) == 'READY'
    assert window.update(True, False, 800.1) == 'INPUT_WAITING'
    assert advance(window, 800.2, 1399.9, healthy=False) == 'INPUT_WAITING'
    assert window.update(False, False, 1400.1) == 'FAILED'


def test_monitor_pause_and_backwards_time_revoke_ready():
    window = StabilityWindow()
    advance(window, 0, 180)
    assert window.update(True, True, 181) == 'INPUT_WAITING'
    assert window.update(True, True, 180.9) == 'INPUT_WAITING'
    heartbeat = StopHeartbeat()
    heartbeat.receive(False, 10)
    assert heartbeat.required(10.26)


def test_invalid_sample_between_ticks_is_not_lost():
    window = FreshWindow(.1, .3, .3, .05, 2)
    window.observe(10, 10, 0)
    window.observe(10.1, 10.1, .1)
    before = window.resets
    window.observe(10.11, 10.11, .11, 'invalid frame')
    window.observe(10.12, 10.12, .12)
    assert window.resets > before
    assert not window.ready(10.12, .12)


def load_launch(name):
    path = Path(__file__).resolve().parents[1] / 'launch' / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_preflight_failure_never_marks_context_checked(monkeypatch):
    module = load_launch('network_preflight.launch.py')
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        stdout=json.dumps({'ready': False}), stderr='', returncode=1))
    context = SimpleNamespace()
    with pytest.raises(RuntimeError, match='Network preflight failed'):
        module.preflight(context)
    assert not getattr(context, '_jackal_network_checked', False)


def test_preflight_once_per_context_and_critical_python_exit(monkeypatch):
    module = load_launch('network_preflight.launch.py')
    calls = []
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: (
        calls.append(a) or SimpleNamespace(stdout='{"ready":true}', returncode=0)))
    context = SimpleNamespace(is_shutdown=False)
    assert module.preflight(context)
    assert module.preflight(context) == []
    assert len(calls) == 1
    for name in ('operator_stop.py', 'nav2_safety_guard.py', 'stack_stability.py',
                 'fast_livo_odom_adapter.py'):
        assert module.critical_exit(SimpleNamespace(
            cmd=['/usr/bin/python3', '/pkg/' + name], returncode=0), context)


def test_stability_loss_latches_operator_and_bans_old_goal():
    from operator_stop import OperatorStop
    from operator_stop_core import OperatorState
    state = OperatorState()
    state.phase = 'RUNNING'
    state.active = {'old': 123}
    published = []
    node = SimpleNamespace(state=state, stability=StopHeartbeat(), cancel_confirmed=True,
                           output=SimpleNamespace(publish=published.append))
    OperatorStop.stack_ready(node, SimpleNamespace(data=False))
    assert state.phase == 'STOPPED'
    assert 'old' in state.banned
    assert not node.cancel_confirmed
    assert published[-1].data is True


def test_guard_discards_command_on_stability_block_and_recovery():
    from nav2_safety_guard import SafetyGuard
    zeroes = []
    node = SimpleNamespace(stability=StopHeartbeat(), publish_zero=lambda: zeroes.append(1))
    SafetyGuard.stack_ready(node, SimpleNamespace(data=False))
    SafetyGuard.stack_ready(node, SimpleNamespace(data=True))
    assert len(zeroes) == 2


def test_full_monitor_avoids_initialpose_deadlock_and_rechecks_lifecycle(monkeypatch):
    from builtin_interfaces.msg import Time
    import stack_stability as monitor

    clock = [0.0]
    output = []
    monkeypatch.setattr(monitor.time, 'monotonic', lambda: clock[0])
    lifecycle = {name: {'active_at': None} for name in monitor.LIFECYCLE_NODES}
    pose = [False]
    node = SimpleNamespace(
        lifecycle=lifecycle, lifecycle_signature=None, reset_counts=None,
        windows={'scan': SimpleNamespace(ready=lambda *a: True, resets=0, reason='valid')},
        settings={'odom_frame': 'odom'},
        graph_changed=False, window=StabilityWindow(), last_diagnostic=-float('inf'),
        last_phase=None, evidence=None, action=SimpleNamespace(server_is_ready=lambda: True),
        output=SimpleNamespace(publish=lambda msg: output.append(msg.data)),
        diagnostics=SimpleNamespace(publish=lambda msg: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int((clock[0] + 1000) * 1e9), to_msg=lambda: Time(sec=1000))),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, error=lambda *a: None),
        _localization_ready=lambda mono: True, pose_ready=lambda now: pose[0],
        inspect_graph=lambda mono: True, guard_inputs_ready=lambda *a: True,
        transform_ready=lambda *a: True,
    )
    for tick in range(1801):
        clock[0] = tick / 10
        for name in ('/map_server', '/amcl'):
            lifecycle[name]['active_at'] = clock[0]
        monitor.StackStability._tick(node)
    assert node.window.phase == 'INITIAL_POSE_REQUIRED'
    assert not any(output)
    # Initialpose allows remaining servers to activate; that transition must
    # restart the 180s evidence, not immediately publish a ready heartbeat.
    pose[0] = True
    for tick in range(1801, 3604):
        clock[0] = tick / 10
        for state in lifecycle.values():
            state['active_at'] = clock[0]
        monitor.StackStability._tick(node)
        if tick < 3602:
            assert not output[-1]
    assert output[-1]
    node.windows['scan'].resets += 1
    clock[0] += .1
    monitor.StackStability._tick(node)
    assert not output[-1]


def test_timing_rejected_before_process_creation():
    from launch import LaunchContext
    module = load_launch('nav_bringup.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        input_timeout='60', localization_timeout='90', ready_settle='180',
        stability_timeout='600', stability_settle='180')
    with pytest.raises(RuntimeError, match='ready_settle'):
        module._validate_timing(context)
    context.launch_configurations['ready_settle'] = '5'
    assert module._validate_timing(context) == []
    context.launch_configurations['stability_timeout'] = '180'
    with pytest.raises(RuntimeError, match='stability_settle'):
        load_launch('navigation.launch.py')._validate_params(context)


def test_uninstalled_fixture_rejects_robot_domain_and_platform_output(monkeypatch):
    from launch import LaunchContext
    path = Path(__file__).with_name('isolated_navigation.launch.py')
    spec = importlib.util.spec_from_file_location('isolated_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = LaunchContext()
    with pytest.raises(RuntimeError, match='isolated domain'):
        module._isolated_fixture(context)
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    context.launch_configurations.update(nav_cmd_vel_topic='/j100_0519/nav2_cmd_vel',
                                         lidar_pointcloud_topic='/nav2_test/raw')
    with pytest.raises(RuntimeError, match='/nav2_test/output'):
        module._isolated_fixture(context)
    assert not getattr(context, '_jackal_network_checked', False)


def test_critical_failure_evidence_preserves_first_reason(tmp_path, monkeypatch):
    module = load_launch('network_preflight.launch.py')
    monkeypatch.setenv('JACKAL_NAV_SESSION_DIR', str(tmp_path))
    context = SimpleNamespace(is_shutdown=False)
    module.critical_exit(
        SimpleNamespace(cmd=['/pkg/nav2_safety_guard.py'], returncode=-9), context)
    module.critical_exit(SimpleNamespace(cmd=['/pkg/collision_monitor'], returncode=0), context)
    record = json.loads((tmp_path / 'failure.json').read_text())
    assert 'nav2_safety_guard.py' in record['reason']


def test_failed_failure_log_still_shuts_down(tmp_path, monkeypatch):
    module = load_launch('network_preflight.launch.py')
    monkeypatch.setenv('JACKAL_NAV_SESSION_DIR', str(tmp_path / 'absent'))
    assert module.critical_exit(SimpleNamespace(cmd=['/pkg/nav2_safety_guard.py'], returncode=-9),
                                SimpleNamespace(is_shutdown=False))
