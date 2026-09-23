"""Safety contracts, including failures that Humble Collision Monitor ignores."""

import importlib.util
from pathlib import Path
import sys

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform, Twist
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from nav2_safety_core import (  # noqa: E402,I100
    cloud_xyz,
    filter_points,
    GuardState,
    make_stamped_twist,
    SafetyConfig,
    transform_xyz,
)


def healthy(points=None):
    state = GuardState(SafetyConfig(), True)
    assert state.observe(
        100.0,
        100.01,
        10.0,
        np.empty((0, 3)) if points is None else np.asarray(points),
    )
    assert state.receive_command([0.2, 0, 0, 0, 0, 0.35], 10.0)
    return state


def test_pass_clamp_and_already_slowed_monitor_command():
    state = healthy()
    assert state.decision(100.02, 10.02) == (
        0.2,
        0.35,
        'passing_collision_checked_command',
    )
    state.receive_command([0.06, 0, 0, 0, 0, 0.105], 10.02)
    assert state.decision(100.02, 10.02)[:2] == (0.06, 0.105)
    state.receive_command([-100, 0, 0, 0, 0, 100], 10.02)
    assert state.decision(100.02, 10.02)[:2] == (-0.5, 1.0)


def test_disabled_and_independent_stop_including_reverse_corner():
    state = healthy()
    state.enable_motion = False
    assert state.decision(100.02, 10.02)[2] == 'motion_disabled'
    state = healthy([[-0.60, -0.40, 0.2]] * 3)
    assert state.decision(100.02, 10.02)[2] == 'independent_stop'
    assert healthy([[0.5, 0.0, 0.2]] * 2).decision(100.02, 10.02)[0] == 0.2


@pytest.mark.parametrize(
    'values',
    [
        [float('nan'), 0, 0, 0, 0, 0],
        [0, 0, 0, float('inf'), 0, 0],
        [0, 0.1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, float('-inf')],
    ],
)
def test_invalid_commands(values):
    state = healthy()
    assert not state.receive_command(values, 10.0)
    assert state.decision(100.02, 10.02)[0:2] == (0.0, 0.0)


@pytest.mark.parametrize(
    'now,mono,tf,reason',
    [
        (100.31, 10.10, '', 'sensor_timestamp'),
        (100.10, 10.31, '', 'sensor_receipt_timeout'),
        (100.10, 10.26, '', 'monitor_command_timeout'),
        (99.9, 10.02, '', 'clock_regression'),
        (100.02, 10.02, 'required_tf_missing', 'required_tf_missing'),
    ],
)
def test_watchdogs(now, mono, tf, reason):
    state = healthy()
    assert state.decision(now, mono, tf) == (0.0, 0.0, reason)
    assert state.command is None


def test_sensor_order_future_invalid_and_recovery_need_new_command():
    state = healthy()
    assert not state.observe(100.0, 100.02, 10.02, np.empty((0, 3)))
    assert state.command is None
    assert not state.observe(101.0, 100.02, 10.02, np.empty((0, 3)))
    assert state.observe(100.03, 100.04, 10.04, np.empty((0, 3)))
    assert state.decision(100.05, 10.05)[0] == 0.0


def cloud(points):
    msg = PointCloud2(
        height=1, width=len(points), point_step=12, row_step=12 * len(points)
    )
    msg.fields = [
        PointField(name=name, offset=i * 4, datatype=7, count=1)
        for i, name in enumerate(('x', 'y', 'z'))
    ]
    msg.data = np.asarray(points, dtype='<f4').tobytes()
    return msg


def test_geometry_self_mask_height_and_valid_processed_empty():
    points = np.array(
        [
            [0, 0, 0.5],
            [1, 0, 0.05],
            [1, 0, 2],
            [0.5, 0.2, 0.5],
            [np.nan, 0, 0.5],
        ]
    )
    np.testing.assert_allclose(
        filter_points(points, SafetyConfig()), [[0.5, 0.2, 0.5]]
    )
    # A real scan whose points all lie outside the retained region is healthy empty.
    assert filter_points(
        cloud_xyz(cloud([[0, 0, 0.5]])), SafetyConfig()
    ).shape == (0, 3)
    with pytest.raises(ValueError):
        cloud_xyz(cloud([]))
    with pytest.raises(ValueError):
        cloud_xyz(cloud([[np.nan, 0, 0]]))


def test_cloud_padding_endian_and_transform_failure():
    msg = cloud([[1, 2, 3], [4, 5, 6]])
    msg.height, msg.width, msg.row_step = 2, 1, 16
    msg.is_bigendian = True
    msg.data = (
        np.asarray([1, 2, 3], dtype='>f4').tobytes()
        + bytes(4)
        + np.asarray([4, 5, 6], dtype='>f4').tobytes()
        + bytes(4)
    )
    np.testing.assert_allclose(cloud_xyz(msg), [[1, 2, 3], [4, 5, 6]])
    invalid = Transform()
    invalid.rotation.w = 0.0
    with pytest.raises(ValueError):
        transform_xyz(np.zeros((1, 3)), invalid)
    tf = Transform()
    tf.rotation.w, tf.translation.z = 1.0, 0.9
    np.testing.assert_allclose(
        transform_xyz(np.zeros((1, 3)), tf), [[0, 0, 0.9]]
    )


def test_monitor_guard_same_polygons_and_humble_threshold():
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', ROOT / 'launch/nav2.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = yaml.safe_load((ROOT / 'config/nav2_safety.yaml').read_text())[
        'nav2_safety_guard'
    ]['ros__parameters']
    params = module.collision_parameters(config)
    assert params['Stop.points'] == [
        0.6,
        0.4,
        0.6,
        -0.4,
        -0.6,
        -0.4,
        -0.6,
        0.4,
    ]
    assert (
        params['Stop.max_points']
        == params['Slow.max_points']
        == config['min_points'] - 1
    )
    assert params['Slow.slowdown_ratio'] == 0.7
    assert params['base_shift_correction'] is False
    assert params['raw_lidar.topic'] == '/nav2/safety_points'


@pytest.mark.parametrize(
    'kwargs',
    [
        {'stop_half_x': 0.1},
        {'sensor_timeout': 0},
        {'max_linear_x': 0.6},
        {'min_points': 2.5},
    ],
)
def test_bad_safety_config_fails_startup(kwargs):
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)


def test_make_stamped_twist_copies_all_components_and_header():
    twist = Twist()
    twist.linear.x = 0.2
    twist.linear.y = -0.1
    twist.linear.z = 0.3
    twist.angular.x = -0.4
    twist.angular.y = 0.5
    twist.angular.z = -0.35
    stamp = Time(sec=12, nanosec=345)

    result = make_stamped_twist(twist, stamp, 'base_link')

    assert result.header.stamp == stamp
    assert result.header.frame_id == 'base_link'
    assert result.twist == twist


def test_make_stamped_twist_rejects_invalid_input():
    with pytest.raises(TypeError, match='Twist'):
        make_stamped_twist(object(), Time(), 'base_link')
    with pytest.raises(ValueError, match='frame_id'):
        make_stamped_twist(Twist(), Time(), '')
