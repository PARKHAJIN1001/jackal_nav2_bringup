"""Check percentage units and handling of unknown battery readings."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest
from sensor_msgs.msg import BatteryState


SPEC = importlib.util.spec_from_file_location(
    'battery_percentage_bridge',
    Path(__file__).resolve().parents[1] / 'scripts' / 'battery_percentage_bridge.py')
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
