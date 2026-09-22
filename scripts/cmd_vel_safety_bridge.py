#!/usr/bin/env python3

"""Fail-closed bridge from stamped Nav2 commands to Clearpath Twist input."""

import math
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelSafetyBridge(Node):
    """Observe Nav2 commands and only forward them after explicit opt-in."""

    def __init__(self):
        super().__init__('cmd_vel_safety_bridge')
        self.declare_parameter('input_topic', '/j100_0519/nav2_cmd_vel')
        self.declare_parameter('output_topic', '/j100_0519/cmd_vel')
        self.declare_parameter('forward_cmd_vel', False)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_angular_z', 1.0)

        self._input_topic = self._string_parameter('input_topic')
        self._output_topic = self._string_parameter('output_topic')
        self._forward = self._bool_parameter('forward_cmd_vel')
        self._timeout = self._positive_double_parameter('timeout_sec')
        rate = self._positive_double_parameter('publish_rate_hz')
        self._max_linear_x = self._positive_double_parameter('max_linear_x')
        self._max_angular_z = self._positive_double_parameter('max_angular_z')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscription = self.create_subscription(
            TwistStamped, self._input_topic, self._command_callback, qos)
        self._publisher = None
        self._timer = None
        self._last_command = None
        self._last_received_monotonic = None
        self._stale_reported = False
        self._observed_count = 0

        if self._forward:
            self._publisher = self.create_publisher(
                Twist, self._output_topic, qos)
            self._timer = self.create_timer(1.0 / rate, self._publish_watchdog_output)
            self.get_logger().warn(
                'COMMAND FORWARDING ENABLED: '
                f'{self._input_topic} -> {self._output_topic}')
        else:
            self.get_logger().info(
                'monitor-only mode: commands are observed but never forwarded')

    def _string_parameter(self, name):
        value = self.get_parameter(name).get_parameter_value().string_value
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _bool_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _positive_double_parameter(self, name):
        value = self.get_parameter(name).get_parameter_value().double_value
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _command_callback(self, message):
        values = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('rejected command containing NaN or infinity')
            self._last_command = None
            self._last_received_monotonic = None
            if self._publisher is not None:
                self._publisher.publish(self._zero_command())
            return

        sanitized = Twist()
        sanitized.linear.x = _clamp(
            message.twist.linear.x, self._max_linear_x)
        sanitized.angular.z = _clamp(
            message.twist.angular.z, self._max_angular_z)
        self._last_command = sanitized
        self._last_received_monotonic = time.monotonic()
        self._stale_reported = False
        self._observed_count += 1

        if not self._forward and (
                self._observed_count == 1 or self._observed_count % 20 == 0):
            self.get_logger().info(
                'observed command without forwarding: '
                f'linear.x={sanitized.linear.x:.3f}, '
                f'angular.z={sanitized.angular.z:.3f}')

    def _zero_command(self):
        return Twist()

    def _publish_watchdog_output(self):
        now = time.monotonic()
        fresh = (
            self._last_command is not None
            and self._last_received_monotonic is not None
            and now - self._last_received_monotonic <= self._timeout
        )
        if fresh:
            self._publisher.publish(self._last_command)
            return

        self._publisher.publish(self._zero_command())
        if self._last_command is not None and not self._stale_reported:
            self.get_logger().warn(
                f'command input stale for more than {self._timeout:.3f}s; publishing zero')
            self._stale_reported = True


def main(args=None):
    """Run the fail-closed command bridge."""
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelSafetyBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
