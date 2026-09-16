"""Unit tests for the package-owned Nav2 Twist stamper."""

import importlib.util
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / 'scripts' / 'nav2_twist_stamper.py'
SPEC = importlib.util.spec_from_file_location(
    'nav2_twist_stamper', MODULE_PATH)
TWIST_STAMPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TWIST_STAMPER)


def test_make_stamped_twist_copies_all_components_and_header():
    twist = Twist()
    twist.linear.x = 0.2
    twist.linear.y = -0.1
    twist.linear.z = 0.3
    twist.angular.x = -0.4
    twist.angular.y = 0.5
    twist.angular.z = -0.35
    stamp = Time(sec=12, nanosec=345)

    result = TWIST_STAMPER.make_stamped_twist(
        twist, stamp, 'base_link')

    assert result.header.stamp == stamp
    assert result.header.frame_id == 'base_link'
    assert result.twist == twist


def test_make_stamped_twist_rejects_invalid_input():
    with pytest.raises(TypeError, match='Twist'):
        TWIST_STAMPER.make_stamped_twist(
            object(), Time(), 'base_link')
    with pytest.raises(ValueError, match='frame_id'):
        TWIST_STAMPER.make_stamped_twist(Twist(), Time(), '')
