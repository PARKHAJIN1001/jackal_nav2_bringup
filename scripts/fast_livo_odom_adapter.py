#!/usr/bin/env python3

"""Add a planar twist estimate to FAST-LIVO2 pose-only odometry."""

import copy
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def normalize_angle(angle):
    """Return an angle in [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    """Return yaw for a finite, non-zero quaternion or raise ValueError."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains a non-finite value')

    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError('quaternion has zero norm')

    x, y, z, w = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def stamp_to_seconds(stamp):
    """Convert builtin_interfaces/Time to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def estimate_planar_twist(previous, current, dt):
    """Estimate child-frame planar velocity between two (x, y, yaw) poses."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('dt must be finite and positive')

    dx = current[0] - previous[0]
    dy = current[1] - previous[1]
    delta_yaw = normalize_angle(current[2] - previous[2])
    midpoint_yaw = previous[2] + 0.5 * delta_yaw
    cosine = math.cos(midpoint_yaw)
    sine = math.sin(midpoint_yaw)

    velocity_odom_x = dx / dt
    velocity_odom_y = dy / dt
    velocity_base_x = cosine * velocity_odom_x + sine * velocity_odom_y
    velocity_base_y = -sine * velocity_odom_x + cosine * velocity_odom_y
    return velocity_base_x, velocity_base_y, delta_yaw / dt


class PlanarTwistEstimator:
    """Stateful, ROS-independent pose differentiator with EMA filtering."""

    def __init__(
            self, alpha=0.5, max_dt=0.5,
            max_translation_jump=0.5, max_yaw_jump=0.75):
        self.alpha = alpha
        self.max_dt = max_dt
        self.max_translation_jump = max_translation_jump
        self.max_yaw_jump = max_yaw_jump
        self.previous = None
        self.filtered_twist = None

    def reset(self):
        self.previous = None
        self.filtered_twist = None

    def update(self, sample):
        """Return (twist, reset_reason) for (stamp, x, y, z, yaw)."""
        if self.previous is None:
            self.previous = sample
            self.filtered_twist = None
            return (0.0, 0.0, 0.0), 'first_sample'

        dt = sample[0] - self.previous[0]
        dx = sample[1] - self.previous[1]
        dy = sample[2] - self.previous[2]
        dz = sample[3] - self.previous[3]
        translation = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_delta = abs(normalize_angle(sample[4] - self.previous[4]))
        discontinuity = (
            not math.isfinite(dt)
            or dt <= 0.0
            or dt > self.max_dt
            or translation > self.max_translation_jump
            or yaw_delta > self.max_yaw_jump
        )
        if discontinuity:
            self.previous = sample
            self.filtered_twist = None
            return (0.0, 0.0, 0.0), 'discontinuity'

        raw_twist = estimate_planar_twist(
            (self.previous[1], self.previous[2], self.previous[4]),
            (sample[1], sample[2], sample[4]),
            dt,
        )
        if self.filtered_twist is None:
            filtered_twist = raw_twist
        else:
            filtered_twist = tuple(
                self.alpha * raw + (1.0 - self.alpha) * old
                for raw, old in zip(raw_twist, self.filtered_twist)
            )
        self.previous = sample
        self.filtered_twist = filtered_twist
        return filtered_twist, None


class FastLivoOdomAdapter(Node):
    """Republish FAST-LIVO2 odometry with a filtered planar twist."""

    def __init__(self):
        super().__init__('fast_livo_odom_adapter')
        self.declare_parameter('input_topic', '/aft_mapped_to_init')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('expected_frame_id', 'odom')
        self.declare_parameter('expected_child_frame_id', 'base_link')
        self.declare_parameter('filter_alpha', 0.5)
        self.declare_parameter('max_dt_sec', 0.5)
        self.declare_parameter('max_translation_jump_m', 0.5)
        self.declare_parameter('max_yaw_jump_rad', 0.75)

        input_topic = self._required_string('input_topic')
        output_topic = self._required_string('output_topic')
        self._expected_frame = self._required_string('expected_frame_id')
        self._expected_child_frame = self._required_string(
            'expected_child_frame_id')
        self._alpha = self._bounded_double('filter_alpha', 0.0, 1.0)
        self._max_dt = self._positive_double('max_dt_sec')
        self._max_translation_jump = self._positive_double(
            'max_translation_jump_m')
        self._max_yaw_jump = self._positive_double('max_yaw_jump_rad')

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(Odometry, output_topic, qos)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._odometry_callback, qos)
        self._estimator = PlanarTwistEstimator(
            alpha=self._alpha,
            max_dt=self._max_dt,
            max_translation_jump=self._max_translation_jump,
            max_yaw_jump=self._max_yaw_jump,
        )
        self._last_warning_ns = 0

        self.get_logger().info(
            f'adapting pose-only odometry: {input_topic} -> {output_topic}; '
            'TF ownership remains with FAST-LIVO2')

    def _required_string(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'{name} must be a non-empty string')
        return value

    def _positive_double(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and greater than zero')
        return value

    def _bounded_double(self, name, minimum, maximum):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(
                f'{name} must be finite and in [{minimum}, {maximum}]')
        return value

    def _warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warning_ns >= 2_000_000_000:
            self.get_logger().warn(message)
            self._last_warning_ns = now_ns

    @staticmethod
    def _zero_twist(message):
        message.twist.twist.linear.x = 0.0
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0
        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = 0.0

    def _odometry_callback(self, incoming):
        if (incoming.header.frame_id != self._expected_frame or
                incoming.child_frame_id != self._expected_child_frame):
            self._warn_throttled(
                'dropping odometry with unexpected frames: '
                f'{incoming.header.frame_id} -> {incoming.child_frame_id}; '
                f'expected {self._expected_frame} -> '
                f'{self._expected_child_frame}')
            self._estimator.reset()
            return

        position = incoming.pose.pose.position
        coordinates = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in coordinates):
            self._warn_throttled('dropping odometry with a non-finite position')
            self._estimator.reset()
            return

        try:
            yaw = quaternion_to_yaw(incoming.pose.pose.orientation)
        except ValueError as error:
            self._warn_throttled(f'dropping invalid odometry: {error}')
            self._estimator.reset()
            return

        sample = (
            stamp_to_seconds(incoming.header.stamp),
            position.x,
            position.y,
            position.z,
            yaw,
        )
        outgoing = copy.deepcopy(incoming)

        filtered_twist, reset_reason = self._estimator.update(sample)
        if reset_reason == 'discontinuity':
            self._warn_throttled(
                'odometry discontinuity detected; resetting twist estimate')
        self._zero_twist(outgoing)
        outgoing.twist.twist.linear.x = filtered_twist[0]
        outgoing.twist.twist.linear.y = filtered_twist[1]
        outgoing.twist.twist.angular.z = filtered_twist[2]
        self._publisher.publish(outgoing)


def main(args=None):
    """Run the adapter until ROS shuts down."""
    rclpy.init(args=args)
    node = None
    try:
        node = FastLivoOdomAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(error)
    finally:
        if node is not None and rclpy.ok(context=node.context):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
