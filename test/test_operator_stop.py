"""Operator stop, stale heartbeat and new-goal-only arming contracts."""

import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from operator_stop_core import OperatorState, StopHeartbeat  # noqa: E402,I100
from prepare_navigation_profile import bounded_limits, generate  # noqa: E402,I100


def state():
    return OperatorState(0, 1, (2, 3), (0.0, 0.0), True)


def step(s, t, buttons=(0, 0, 0, 0), connected=True, ready=True):
    s.joy(buttons, [0.0, 0.0], t)
    s.connection(connected, t)
    s.odometry(0, 0, t)
    return s.tick(t, int(t * 1e9), ready)


def arm(s, start=1.0):
    for i in range(75):
        t = start + i * 0.05
        step(s, t, (0, int(i >= 25), 0, 0))
    assert s.phase == 'WAITING FOR NEW GOAL'


def test_reset_needs_hold_and_new_goal():
    s = state()
    assert step(s, 1.0)
    arm(s)
    assert step(s, 5.0)
    s.goals({'fresh': 6000000000})
    assert not step(s, 6.0)
    assert s.phase == 'RUNNING'
    assert step(s, 6.05, (1, 1, 0, 0))
    assert s.phase == 'STOPPED'
    s.goals({})
    arm(s, 7.0)
    s.goals({'fresh': 6000000000})
    assert step(s, 12.0)
    assert s.phase == 'STOPPED'


@pytest.mark.parametrize('fault', ['joy', 'bluetooth', 'odom', 'bridge', 'manual', 'clock'])
def test_fault_latches_until_explicit_reset(fault):
    s = state()
    arm(s)
    s.goals({'a': 6000000000})
    assert not step(s, 6.0)
    if fault == 'joy':
        assert s.tick(7.1, 7100000000, True)
    elif fault == 'bluetooth':
        assert step(s, 6.05, connected=False)
    elif fault == 'odom':
        s.joy([0] * 4, [0.0, 0.0], 7.1)
        s.connection(True, 7.1)
        assert s.tick(7.1, 7100000000, True)
    elif fault == 'bridge':
        assert step(s, 6.05, ready=False)
    elif fault == 'manual':
        assert step(s, 6.05, (0, 0, 1, 0))
    else:
        assert step(s, 5.9)
    assert step(s, 8.0)
    assert s.phase == 'STOPPED'


def test_unverified_and_invalid_mapping_never_arm():
    s = OperatorState()
    for i in range(100):
        assert step(s, 1 + i * 0.05, (0, 1, 0, 0))
    s = state()
    s.joy([0], [0.0], 1.0)
    assert s.phase == 'STOPPED'


def test_button_held_across_start_does_not_arm():
    s = state()
    for i in range(100):
        assert step(s, 1 + i * 0.05, (0, 1, 0, 0))
    assert s.phase == 'STOPPED'


def test_heartbeat_stale_and_recovery():
    h = StopHeartbeat()
    assert h.required(1.0)
    h.receive(False, 1.0)
    assert not h.required(1.2)
    assert h.required(1.26)
    assert h.required(0.9)
    h.receive(True, 1.3)
    assert h.required(1.3)


def test_bridge_profile_never_exceeds_real_limits():
    root = Path(__file__).resolve().parents[1]
    nav, safety, operator = [
        yaml.safe_load((root / 'config' / name).read_text())
        for name in ('nav2_params.yaml', 'nav2_safety.yaml', 'operator_stop.yaml')
    ]
    nav, safety, operator = generate(
        nav, safety, operator, {'max_linear_x': 0.2, 'max_angular_z': 0.35}
    )
    assert nav['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.2
    assert nav['velocity_smoother']['ros__parameters']['max_velocity'] == [0.2, 0.0, 0.35]
    assert safety['nav2_safety_guard']['ros__parameters']['max_angular_z'] == 0.35
    assert operator['nav2_operator_stop']['ros__parameters']['mapping_verified'] is True
    for value in (None, -1, 0, math.nan, True):
        with pytest.raises(ValueError):
            bounded_limits({'max_linear_x': value, 'max_angular_z': 0.35})


def test_stationary_observation_gap_restarts_hold_evidence():
    s = state()
    s.odometry(0, 0, 1.0)
    s.odometry(0, 0, 1.05)
    assert s.rest_since == 1.0
    s.odometry(0, 0, 3.0)
    assert s.rest_since == 3.0


def test_cancel_failure_cannot_unblock_output():
    from concurrent.futures import Future
    from types import SimpleNamespace

    from operator_stop import OperatorStop

    s = state()
    arm(s)
    s.goals({'active': 6000000000})
    step(s, 6.0)
    output = []
    fixture = SimpleNamespace(
        state=s, cancel_confirmed=True, output=SimpleNamespace(publish=output.append)
    )
    OperatorStop.joy(fixture, SimpleNamespace(buttons=[1, 0, 0, 0], axes=[0.0, 0.0]))
    assert output[-1].data is True
    assert not fixture.cancel_confirmed
    rejected = Future()
    rejected.set_exception(RuntimeError('cancel transport unavailable'))
    fixture.cancel_future, fixture.bridge_future = rejected, None
    fixture.cancel_at = fixture.bridge_at = 10.0
    fixture.cancel = SimpleNamespace(service_is_ready=lambda: False)
    fixture.bridge = SimpleNamespace(service_is_ready=lambda: False)
    OperatorStop.queries(fixture, 10.0)
    assert not fixture.cancel_confirmed
    assert step(s, 6.1)
    assert s.phase == 'STOPPED'
