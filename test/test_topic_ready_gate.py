"""Readiness regressions without starting ROS nodes or touching the robot."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location('gate', ROOT / 'scripts/topic_ready_gate.py')
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)


def window():
    return GATE.FreshWindow(0.8, 0.3, 0.3, 0.05, 5)


def feed(w, start=100.0, count=10):
    for i in range(count):
        now = start + i * 0.1
        w.observe(now - 0.02, now, now)
    return now


def test_publisher_discovery_and_wall_wait_are_not_readiness():
    w = window()
    assert not w.ready(100.0, 100.0)
    w.observe(100.0, 100.0, 100.0)
    assert not w.ready(100.1, 100.1)
    assert not w.ready(110.0, 110.0)


def test_continuous_fresh_measurements_pass_and_silence_revokes_readiness():
    w = window()
    now = feed(w)
    assert w.ready(now, now)
    assert not w.ready(now + 0.4, now + 0.4)
    now = feed(w, now + 0.5, count=3)
    assert not w.ready(now, now)


@pytest.mark.parametrize('fault', ['stale', 'future', 'duplicate', 'regression', 'invalid'])
def test_bad_sample_requires_a_new_continuous_window(fault):
    w = window()
    now = feed(w)
    assert w.ready(now, now)
    stamps = {'stale': now - 1.0, 'future': now + 1.0,
              'duplicate': w.stamp, 'regression': w.stamp - 0.1, 'invalid': now}
    w.observe(stamps[fault], now + 0.1, now + 0.1,
              error='malformed input' if fault == 'invalid' else '')
    assert not w.ready(now + 0.1, now + 0.1)
    now = feed(w, now + 0.2)
    assert w.ready(now, now)


def test_a_queued_burst_does_not_count_as_continuous_receipt():
    w = window()
    for i in range(100):
        w.observe(100.0 + i * 0.001, 100.1, 100.1)
    assert not w.ready(100.1, 100.1)


@pytest.mark.parametrize('message', [PointCloud2(), Imu(), Odometry(), LaserScan()])
def test_empty_messages_are_not_measurements(message):
    message.header.frame_id = 'sensor'
    assert GATE.message_error(message)


def test_odometry_contract_rejects_bad_frame_nonfinite_pose_and_quaternion():
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.pose.pose.orientation.w = 1.0
    assert not GATE.message_error(msg, 'odom', 'base_link')
    assert GATE.message_error(msg, 'map', 'base_link')
    assert GATE.message_error(msg, 'odom', 'sensor')
    msg.pose.pose.position.x = float('nan')
    assert GATE.message_error(msg, 'odom', 'base_link')
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.orientation.w = 2.0
    assert GATE.message_error(msg, 'odom', 'base_link')


def test_odometry_jump_and_origin_checks(monkeypatch):
    clock = SimpleNamespace(nanoseconds=100000000000)
    monkeypatch.setattr(GATE.time, 'monotonic', lambda: clock.nanoseconds / 1e9)
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        settings={'max_position_norm': 1.0, 'max_gap': 0.3,
                  'max_linear_speed': 2.0, 'max_angular_speed': 3.0},
        windows={'/odom': window()}, previous_poses={})
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    # Quaternion rounding must not look like rotation at every sample.
    msg.pose.pose.orientation.w = 0.999
    for i in range(10):
        clock.nanoseconds = 100000000000 + i * 100000000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(clock.nanoseconds, 10**9)
        GATE.ReadinessGate._receive(node, '/odom', msg, 'odom', 'base_link')
    assert node.windows['/odom'].ready(100.9, 100.9)
    clock.nanoseconds += 100000000
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(clock.nanoseconds, 10**9)
    msg.pose.pose.position.x = 0.9
    GATE.ReadinessGate._receive(node, '/odom', msg, 'odom', 'base_link')
    assert node.windows['/odom'].reason == 'odometry discontinuity'
    node.previous_poses.clear()
    msg.pose.pose.position.x = 10.0
    GATE.ReadinessGate._receive(node, '/odom', msg, 'odom', 'base_link')
    assert 'startup origin bound' in node.windows['/odom'].reason


def test_localization_gate_requires_active_nodes_map_and_scan_time_tf():
    calls = []
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp.sec = 'base_link', 100
    tf = SimpleNamespace(can_transform=lambda *args: calls.append(args) or True)

    def state():
        return {'future': None, 'sent': 100.0, 'active_at': 100.0}

    node = SimpleNamespace(
        tf_buffer=tf, settings={'odom_frame': 'odom', 'max_age': 0.3, 'future_tolerance': 0.05},
        scan_headers=[scan.header], get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100100000000)),
        lifecycle={'/amcl': state(), '/map_server': state()}, map_received=False)

    def ready():
        return GATE.ReadinessGate._localization_ready(node, 100.1)

    assert not ready()
    node.map_received = True
    node.lifecycle['/amcl']['active_at'] = None
    assert not ready()
    node.lifecycle['/amcl']['active_at'] = 100.0
    assert ready()
    assert calls[-1][:2] == ('odom', 'base_link')
    assert calls[-1][2].nanoseconds == 100000000000
    # A map->odom dependency would deadlock before manual initialpose.
    node.tf_buffer = SimpleNamespace(can_transform=lambda *args: False)
    assert not ready()
    # A newer scan may be awaiting TF; a still-fresh previous scan is sufficient.
    newer = LaserScan()
    newer.header.frame_id, newer.header.stamp.sec = 'base_link', 100
    newer.header.stamp.nanosec = 50000000
    node.scan_headers.append(newer.header)
    node.tf_buffer = SimpleNamespace(can_transform=lambda target, source, stamp:
                                     stamp.nanoseconds == 100000000000)
    assert ready()
    scan.header.stamp.sec = 99
    assert not ready()  # A matching but stale transform cannot open the gate.


def test_cancellation_returns_failure(monkeypatch):
    monkeypatch.setattr(GATE.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(GATE.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(GATE, 'ReadinessGate', lambda: SimpleNamespace(destroy_node=lambda: None))

    def interrupt(node):
        raise KeyboardInterrupt()

    monkeypatch.setattr(GATE.rclpy, 'spin', interrupt)
    assert GATE.main() == 130


def test_perception_gate_requires_map_tf_at_fresh_scan_time():
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp.sec = 'base_link', 100
    allowed = {('odom', 'base_link')}
    calls = []

    def can_transform(target, source, stamp):
        calls.append((target, source, stamp.nanoseconds))
        return (target, source) in allowed

    node = SimpleNamespace(
        tf_buffer=SimpleNamespace(can_transform=can_transform),
        settings={'odom_frame': 'odom', 'max_age': 0.3, 'future_tolerance': 0.05,
                  'require_map_to_odom': True},
        scan_headers=[scan.header], get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100100000000)),
        lifecycle={name: {'future': None, 'sent': 100.0, 'active_at': 100.0}
                   for name in ('/amcl', '/map_server')}, map_received=True)
    assert not GATE.ReadinessGate._localization_ready(node, 100.1)
    allowed.add(('map', 'odom'))
    assert GATE.ReadinessGate._localization_ready(node, 100.1)
    assert calls[-1] == ('map', 'odom', 100000000000)
    scan.header.stamp.sec = 99
    assert not GATE.ReadinessGate._localization_ready(node, 100.1)
