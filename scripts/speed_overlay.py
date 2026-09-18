#!/usr/bin/env python3

"""Display planar odometry speed below the RViz battery gauge."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


def planar_speed(twist):
    """Return nonnegative ground speed in m/s, or None for invalid input."""
    speed = math.hypot(twist.linear.x, twist.linear.y)
    return speed if math.isfinite(speed) else None


def speed_text(speed, age, timeout):
    """Distinguish unavailable or stale telemetry from a stopped robot."""
    if age is None:
        return 'Speed: -- m/s (waiting)'
    if age > timeout:
        return 'Speed: -- m/s (stale)'
    if speed is None:
        return 'Speed: -- m/s (invalid)'
    return f'Speed: {speed:.2f} m/s'


class SpeedOverlay(Node):
    """Publish speed text at 10 Hz without using commanded velocity."""

    def __init__(self):
        super().__init__('speed_overlay')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('stale_timeout_sec', 2.0)
        self._timeout = self.get_parameter('stale_timeout_sec').value
        if not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('stale_timeout_sec must be finite and positive')
        self._speed = None
        self._received_at = None
        self._publisher = self.create_publisher(OverlayText, '/nav2/speed_overlay', 1)
        self._subscription = self.create_subscription(
            Odometry, self.get_parameter('input_topic').value,
            self._on_odometry, qos_profile_sensor_data)
        # Keep the stale indicator working even if a simulated clock pauses.
        self._timer = self.create_timer(
            0.1, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_odometry(self, message):
        self._speed = planar_speed(message.twist.twist)
        self._received_at = time.monotonic()

    def _publish(self):
        age = None if self._received_at is None else time.monotonic() - self._received_at
        message = OverlayText()
        message.action = OverlayText.ADD
        message.width = 250
        message.height = 40
        message.horizontal_alignment = OverlayText.LEFT
        message.vertical_alignment = OverlayText.TOP
        message.horizontal_distance = 16
        message.vertical_distance = 155
        message.text_size = 14.0
        message.font = 'DejaVu Sans'
        message.fg_color = ColorRGBA(r=0.95, g=0.95, b=0.95, a=1.0)
        message.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
        message.text = speed_text(self._speed, age, self._timeout)
        self._publisher.publish(message)


def main(args=None):
    """Run the speed overlay node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedOverlay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
