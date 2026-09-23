"""Unit tests for planar velocity estimation."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ADAPTER_PATH = PACKAGE_ROOT / 'scripts' / 'fast_livo_odom_adapter.py'
SPEC = importlib.util.spec_from_file_location('fast_livo_odom_adapter', ADAPTER_PATH)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def _quaternion_for_yaw(yaw):
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(0.5 * yaw),
        w=math.cos(0.5 * yaw),
    )


def test_quaternion_to_yaw_normalizes_input():
    quaternion = _quaternion_for_yaw(1.2)
    quaternion.z *= 3.0
    quaternion.w *= 3.0
    assert ADAPTER.quaternion_to_yaw(quaternion) == pytest.approx(1.2)


@pytest.mark.parametrize(
    'quaternion',
    [
        SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        SimpleNamespace(x=math.nan, y=0.0, z=0.0, w=1.0),
    ],
)
def test_quaternion_to_yaw_rejects_invalid_input(quaternion):
    with pytest.raises(ValueError):
        ADAPTER.quaternion_to_yaw(quaternion)


def test_planar_twist_is_expressed_in_child_frame():
    previous = (0.0, 0.0, math.pi / 2.0)
    current = (0.0, 0.2, math.pi / 2.0)
    velocity = ADAPTER.estimate_planar_twist(previous, current, 0.1)
    assert velocity == pytest.approx((2.0, 0.0, 0.0), abs=1.0e-9)


def test_yaw_rate_uses_shortest_path_across_pi():
    previous = (0.0, 0.0, math.radians(179.0))
    current = (0.0, 0.0, math.radians(-179.0))
    velocity = ADAPTER.estimate_planar_twist(previous, current, 0.1)
    assert velocity[2] == pytest.approx(math.radians(20.0))


def test_estimator_first_sample_and_stationary_sample_are_zero():
    estimator = ADAPTER.PlanarTwistEstimator()
    twist, reason = estimator.update((1.0, 0.0, 0.0, 0.0, 0.0))
    assert twist == (0.0, 0.0, 0.0)
    assert reason == 'first_sample'

    twist, reason = estimator.update((1.1, 0.0, 0.0, 0.0, 0.0))
    assert twist == pytest.approx((0.0, 0.0, 0.0))
    assert reason is None


def test_estimator_applies_exponential_filter():
    estimator = ADAPTER.PlanarTwistEstimator(alpha=0.5)
    estimator.update((1.0, 0.0, 0.0, 0.0, 0.0))
    first, _ = estimator.update((1.1, 0.1, 0.0, 0.0, 0.0))
    second, _ = estimator.update((1.2, 0.4, 0.0, 0.0, 0.0))

    assert first == pytest.approx((1.0, 0.0, 0.0))
    assert second == pytest.approx((2.0, 0.0, 0.0))


@pytest.mark.parametrize(
    'sample',
    [
        (2.1, 0.1, 0.0, 0.0, 0.0),
        (1.1, 0.6, 0.0, 0.0, 0.0),
        (1.1, 0.0, 0.0, 0.0, 0.8),
    ],
)
def test_estimator_resets_on_forward_gap_or_pose_jump(sample):
    estimator = ADAPTER.PlanarTwistEstimator()
    estimator.update((1.0, 0.0, 0.0, 0.0, 0.0))

    twist, reason = estimator.update(sample)

    assert twist == (0.0, 0.0, 0.0)
    assert reason == 'discontinuity'
    assert estimator.filtered_twist is None


def test_estimator_reset_starts_a_new_baseline():
    estimator = ADAPTER.PlanarTwistEstimator()
    estimator.update((1.0, 0.0, 0.0, 0.0, 0.0))
    estimator.update((1.1, 0.1, 0.0, 0.0, 0.0))
    estimator.reset()

    twist, reason = estimator.update((5.0, 10.0, 10.0, 0.0, 1.0))

    assert twist == (0.0, 0.0, 0.0)
    assert reason == 'first_sample'


@pytest.mark.parametrize('stamp', [0.0, -1.0, math.nan, math.inf, 0.9, 1.0])
def test_rejected_timestamp_does_not_replace_valid_baseline(stamp):
    estimator = ADAPTER.PlanarTwistEstimator()
    baseline = (1.0, 0.0, 0.0, 0.0, 0.0)
    estimator.update(baseline)
    twist, reason = estimator.update((stamp, 100.0, 0.0, 0.0, 0.0))
    assert twist is None
    assert reason in ('invalid_timestamp', 'non_increasing_timestamp')
    assert estimator.previous == baseline
    twist, reason = estimator.update((1.1, 0.01, 0.0, 0.0, 0.0))
    assert reason is None
    assert twist == pytest.approx((0.1, 0.0, 0.0))


def _adapter_stub():
    published = []
    clock = SimpleNamespace(nanoseconds=10_050_000_000)
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        _last_ros_ns=None, _last_warning_ns=0,
        _max_message_age=0.60, _future_tolerance=0.20,
        _expected_frame='odom', _expected_child_frame='base_link',
        _estimator=ADAPTER.PlanarTwistEstimator(),
        _warn_throttled=lambda message: None,
        _zero_twist=ADAPTER.FastLivoOdomAdapter._zero_twist,
        _publisher=SimpleNamespace(publish=published.append),
    )
    return node, clock, published


def _deliver(node, stamp, x=0.0):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_link'
    sec, nanosec = divmod(round(stamp * 1e9), 1_000_000_000)
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    message.pose.pose.orientation.w = 1.0
    message.pose.pose.position.x = x
    ADAPTER.FastLivoOdomAdapter._odometry_callback(node, message)


@pytest.mark.parametrize('bad_stamp', [0.0, 9.0, 9.9, 10.0, 11.0])
def test_callback_drops_invalid_stale_future_and_non_increasing_time(bad_stamp):
    node, clock, published = _adapter_stub()
    _deliver(node, 10.0)
    _deliver(node, bad_stamp, 100.0)
    assert len(published) == 1
    clock.nanoseconds = 10_150_000_000
    _deliver(node, 10.1, 0.01)
    assert len(published) == 2
    assert published[-1].twist.twist.linear.x == pytest.approx(0.1)


def test_real_ros_clock_reset_accepts_new_epoch_with_zero_initial_twist():
    node, clock, published = _adapter_stub()
    _deliver(node, 10.0)
    clock.nanoseconds = 5_050_000_000
    _deliver(node, 5.0, 100.0)
    assert len(published) == 2
    assert published[-1].header.stamp.sec == 5
    assert published[-1].twist.twist.linear.x == 0.0
    clock.nanoseconds = 5_150_000_000
    _deliver(node, 5.1, 100.01)
    assert published[-1].twist.twist.linear.x == pytest.approx(0.1)
