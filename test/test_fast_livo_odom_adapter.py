"""Unit tests for planar velocity estimation."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

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
    estimator.update((0.0, 0.0, 0.0, 0.0, 0.0))
    first, _ = estimator.update((0.1, 0.1, 0.0, 0.0, 0.0))
    second, _ = estimator.update((0.2, 0.4, 0.0, 0.0, 0.0))

    assert first == pytest.approx((1.0, 0.0, 0.0))
    assert second == pytest.approx((2.0, 0.0, 0.0))


@pytest.mark.parametrize(
    'sample',
    [
        (1.0, 0.1, 0.0, 0.0, 0.0),
        (0.9, 0.1, 0.0, 0.0, 0.0),
        (1.6, 0.1, 0.0, 0.0, 0.0),
        (1.1, 0.6, 0.0, 0.0, 0.0),
        (1.1, 0.0, 0.0, 0.0, 0.8),
    ],
)
def test_estimator_resets_on_invalid_time_or_pose_jump(sample):
    estimator = ADAPTER.PlanarTwistEstimator()
    estimator.update((1.0, 0.0, 0.0, 0.0, 0.0))

    twist, reason = estimator.update(sample)

    assert twist == (0.0, 0.0, 0.0)
    assert reason == 'discontinuity'
    assert estimator.filtered_twist is None


def test_estimator_reset_starts_a_new_baseline():
    estimator = ADAPTER.PlanarTwistEstimator()
    estimator.update((0.0, 0.0, 0.0, 0.0, 0.0))
    estimator.update((0.1, 0.1, 0.0, 0.0, 0.0))
    estimator.reset()

    twist, reason = estimator.update((5.0, 10.0, 10.0, 0.0, 1.0))

    assert twist == (0.0, 0.0, 0.0)
    assert reason == 'first_sample'
