"""Validate battery percentage conversion, planar speed, and RViz overlay bridge."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock

from geometry_msgs.msg import Twist
import pytest
from sensor_msgs.msg import BatteryState


SPEC = importlib.util.spec_from_file_location(
    'rviz_overlay_bridge',
    Path(__file__).resolve().parents[1] / 'scripts' / 'rviz_overlay_bridge.py')
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


@pytest.mark.parametrize('fraction, expected', [(0.0, 0.0), (0.75, 75.0), (1.0, 100.0)])
def test_callback_publishes_percent(fraction, expected):
    node = Mock()
    BRIDGE.BatteryPercentageBridge._on_battery(node, BatteryState(percentage=fraction))
    message = node._publisher.publish.call_args.args[0]
    assert message.data == pytest.approx(expected)


@pytest.mark.parametrize('fraction', [float('nan'), float('inf'), -float('inf'), -1.0, 75.0])
def test_callback_never_publishes_unknown_or_invalid_values(fraction):
    node = Mock()
    BRIDGE.BatteryPercentageBridge._on_battery(node, BatteryState(percentage=fraction))
    node._publisher.publish.assert_not_called()
    node.get_logger().warning.assert_called_once()


@pytest.mark.parametrize('x, y, expected', [
    (0.0, 0.0, 0.0), (-0.7, 0.0, 0.7), (0.3, -0.4, 0.5),
])
def test_speed_ignores_direction_vertical_motion_and_yaw(x, y, expected):
    twist = Twist()
    twist.linear.x, twist.linear.y = x, y
    twist.linear.z = 3.0
    twist.angular.z = 2.0
    assert BRIDGE.planar_speed(twist) == pytest.approx(expected)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_invalid_speed(value):
    twist = Twist()
    twist.linear.x = value
    assert BRIDGE.planar_speed(twist) is None


def test_missing_data_is_not_displayed_as_zero_speed():
    assert BRIDGE.speed_text(None, None, 2.0) == 'Speed: -- m/s (waiting)'
    assert BRIDGE.speed_text(0.7, 2.1, 2.0) == 'Speed: -- m/s (stale)'
    assert BRIDGE.speed_text(None, 0.1, 2.0) == 'Speed: -- m/s (invalid)'
    assert BRIDGE.speed_text(0.0, 0.1, 2.0) == 'Speed: 0.00 m/s'


@pytest.mark.parametrize('fraction, expected', [(0.25, 25.0), (0.5, 50.0)])
def test_unified_bridge_battery_callback(fraction, expected):
    node = Mock()
    BRIDGE.RVizOverlayBridge._on_battery(node, BatteryState(percentage=fraction))
    message = node._battery_publisher.publish.call_args.args[0]
    assert message.data == pytest.approx(expected)
