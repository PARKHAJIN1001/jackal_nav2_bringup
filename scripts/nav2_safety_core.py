"""Pure, fail-closed state and geometry helpers for the navigation output guard."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Twist, TwistStamped
import numpy as np


@dataclass(frozen=True)
class SafetyConfig:
    min_height: float = 0.10
    max_height: float = 1.80
    footprint_half_x: float = 0.254
    footprint_half_y: float = 0.215
    stop_half_x: float = 0.60
    stop_half_y: float = 0.40
    slow_half_x: float = 1.00
    slow_half_y: float = 0.50
    min_points: int = 3
    slowdown_ratio: float = 0.70
    sensor_timeout: float = 0.30
    command_timeout: float = 0.25
    future_tolerance: float = 0.05
    tf_timeout: float = 0.30
    map_tf_timeout: float = 0.50
    map_transform_tolerance: float = 1.0
    max_linear_x: float = 0.50
    max_angular_z: float = 1.0

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Safety limits must be finite and positive')
        if not isinstance(self.min_points, int) or self.min_points < 1:
            raise ValueError('min_points must be a positive integer')
        if not (
            self.min_height < self.max_height
            and self.footprint_half_x < self.stop_half_x <= self.slow_half_x
            and self.footprint_half_y < self.stop_half_y <= self.slow_half_y
        ):
            raise ValueError('Invalid height or nested safety polygons')
        if (
            self.slowdown_ratio > 1
            or self.max_linear_x > 0.50
            or self.max_angular_z > 1.0
        ):
            raise ValueError(
                'Safety speed limits may not exceed the requested profile ceiling'
            )


def cloud_xyz(message):
    """Decode XYZ without assuming packed rows, field ordering or byte order."""
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        raise ValueError('Empty or malformed raw observation')
    if message.row_step < message.width * message.point_step:
        raise ValueError('Invalid cloud row_step')
    if len(message.data) < message.row_step * message.height:
        raise ValueError('Truncated point cloud')
    fields = {f.name: f for f in message.fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ fields must be scalar FLOAT32 or FLOAT64')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('Invalid XYZ offset')
        formats.append(('>' if message.is_bigendian else '<') + f'f{size}')
        offsets.append(field.offset)
    dtype = np.dtype(
        {'names': ['x', 'y', 'z'], 'formats': formats,
         'offsets': offsets, 'itemsize': message.point_step}
    )
    rows = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=bytes(message.data),
        strides=(message.row_step, message.point_step),
    )
    xyz = np.column_stack([rows[name].ravel() for name in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('Raw observation contains no finite points')
    return xyz


def transform_xyz(xyz, transform):
    """Apply a validated rigid transform; never substitute identity on failure."""
    q = transform.rotation
    t = transform.translation
    values = [q.x, q.y, q.z, q.w, t.x, t.y, t.z]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite transform')
    norm = math.sqrt(sum(v * v for v in values[:4]))
    if abs(norm - 1.0) > 0.01:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = (v / norm for v in values[:4])
    rotation = np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ]
    )
    return xyz @ rotation.T + np.array([t.x, t.y, t.z])


def filter_points(xyz, config):
    finite = np.isfinite(xyz).all(axis=1)
    height = (xyz[:, 2] >= config.min_height) & (
        xyz[:, 2] <= config.max_height
    )
    own_body = (np.abs(xyz[:, 0]) <= config.footprint_half_x) & (
        np.abs(xyz[:, 1]) <= config.footprint_half_y
    )
    return xyz[finite & height & ~own_body].astype('<f4')


class GuardState:
    """No ROS dependencies; monotonic receipt and measurement ages both matter."""

    def __init__(self, config, enable_motion=False):
        self.config = config
        self.enable_motion = enable_motion
        self.sensor = None
        self.command = None
        self.last_ros = None
        self.sensor_error = 'sensor_missing'
        self.stop_count = 0

    def clock(self, now):
        backwards = self.last_ros is not None and now < self.last_ros
        self.last_ros = now
        if backwards:
            self.invalidate_sensor('clock_regression')
        return not backwards

    def invalidate_sensor(self, reason):
        self.sensor = None
        self.command = None
        self.sensor_error = reason

    def observe(self, stamp, now, received, points):
        if not self.clock(now):
            return False
        age = now - stamp
        if (
            not math.isfinite(stamp)
            or stamp <= 0
            or age < -self.config.future_tolerance
            or age > self.config.sensor_timeout
        ):
            self.invalidate_sensor('sensor_timestamp')
            return False
        if self.sensor is not None and stamp <= self.sensor[0]:
            self.invalidate_sensor('sensor_stamp_not_increasing')
            return False
        self.stop_count = int(
            np.count_nonzero(
                (np.abs(points[:, 0]) <= self.config.stop_half_x)
                & (np.abs(points[:, 1]) <= self.config.stop_half_y)
            )
        )
        self.sensor = (stamp, received)
        self.sensor_error = ''
        return True

    def receive_command(self, values, received):
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self.command = None
            return False
        # A differential-drive output must not silently reinterpret lateral motion.
        if any(abs(values[i]) > 1e-6 for i in (1, 2, 3, 4)):
            self.command = None
            return False
        self.command = (values[0], values[5], received)
        return True

    def decision(self, now, monotonic_now, tf_reason=''):
        if not self.clock(now):
            return 0.0, 0.0, 'clock_regression'
        reason = tf_reason
        if not reason and self.sensor is None:
            reason = self.sensor_error
        if not reason:
            age, receipt_age = (
                now - self.sensor[0],
                monotonic_now - self.sensor[1],
            )
            if not (
                -self.config.future_tolerance
                <= age
                <= self.config.sensor_timeout
            ):
                reason = 'sensor_timestamp'
            elif not (0 <= receipt_age <= self.config.sensor_timeout):
                reason = 'sensor_receipt_timeout'
        if reason:
            self.command = None  # Fresh command required after recovery.
            return 0.0, 0.0, reason
        if self.stop_count >= self.config.min_points:
            self.command = None
            return 0.0, 0.0, 'independent_stop'
        if not self.enable_motion:
            return 0.0, 0.0, 'motion_disabled'
        if self.command is None:
            return 0.0, 0.0, 'command_missing_or_invalid'
        x, yaw, received = self.command
        if not (0 <= monotonic_now - received <= self.config.command_timeout):
            self.command = None
            return 0.0, 0.0, 'monitor_command_timeout'
        return (
            max(-self.config.max_linear_x, min(self.config.max_linear_x, x)),
            max(
                -self.config.max_angular_z, min(self.config.max_angular_z, yaw)
            ),
            'passing_collision_checked_command',
        )


def make_stamped_twist(twist, stamp, frame_id):
    """Copy an unstamped Twist into a TwistStamped message."""
    if not isinstance(twist, Twist):
        raise TypeError('twist must be a geometry_msgs/msg/Twist')
    if not frame_id:
        raise ValueError('frame_id must not be empty')

    output = TwistStamped()
    output.header.stamp = stamp
    output.header.frame_id = frame_id
    output.twist.linear.x = twist.linear.x
    output.twist.linear.y = twist.linear.y
    output.twist.linear.z = twist.linear.z
    output.twist.angular.x = twist.angular.x
    output.twist.angular.y = twist.angular.y
    output.twist.angular.z = twist.angular.z
    return output
