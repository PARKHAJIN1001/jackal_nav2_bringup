"""Validate direction-independent planar speed and missing-data indication."""

import importlib.util
from pathlib import Path

from geometry_msgs.msg import Twist
import pytest


SPEC = importlib.util.spec_from_file_location(
    'speed_overlay', Path(__file__).resolve().parents[1] / 'scripts' / 'speed_overlay.py')
OVERLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OVERLAY)


@pytest.mark.parametrize('x, y, expected', [
    (0.0, 0.0, 0.0), (-0.7, 0.0, 0.7), (0.3, -0.4, 0.5),
])
def test_speed_ignores_direction_vertical_motion_and_yaw(x, y, expected):
    twist = Twist()
    twist.linear.x, twist.linear.y = x, y
    twist.linear.z = 3.0
    twist.angular.z = 2.0
    assert OVERLAY.planar_speed(twist) == pytest.approx(expected)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_invalid_speed(value):
    twist = Twist()
    twist.linear.x = value
    assert OVERLAY.planar_speed(twist) is None


def test_missing_data_is_not_displayed_as_zero_speed():
    assert OVERLAY.speed_text(None, None, 2.0) == 'Speed: -- m/s (waiting)'
    assert OVERLAY.speed_text(0.7, 2.1, 2.0) == 'Speed: -- m/s (stale)'
    assert OVERLAY.speed_text(None, 0.1, 2.0) == 'Speed: -- m/s (invalid)'
    assert OVERLAY.speed_text(0.0, 0.1, 2.0) == 'Speed: 0.00 m/s'
