#!/usr/bin/env python3

"""Relay battery percentage and speed telemetry for RViz HUD display."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import BatteryState
from std_msgs.msg import ColorRGBA, Float32


def percentage_to_percent(percentage):
    """Return 0..100, or None for unknown/nonstandard BatteryState values."""
    if not math.isfinite(percentage) or not 0.0 <= percentage <= 1.0:
        return None
    return percentage * 100.0


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


class RVizOverlayBridge(Node):
    """Publish battery percentage and planar speed text overlays for RViz."""

    def __init__(self):
        super().__init__('rviz_overlay_bridge')

        # Battery gauge parameters
        self.declare_parameter('battery_input_topic', '/j100_0519/platform/bms/state')
        self.declare_parameter('battery_output_topic', '/nav2/battery_percentage')
        self.declare_parameter('enable_battery_gauge', True)

        # Speed display parameters
        self.declare_parameter('speed_input_topic', '/odom')
        self.declare_parameter('speed_output_topic', '/nav2/speed_overlay')
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('enable_speed_display', True)

        self._battery_enabled = self.get_parameter('enable_battery_gauge').value
        self._speed_enabled = self.get_parameter('enable_speed_display').value

        # Setup battery publisher and subscriber
        if self._battery_enabled:
            bat_in = self.get_parameter('battery_input_topic').value
            bat_out = self.get_parameter('battery_output_topic').value
            if not bat_in or not bat_out:
                raise ValueError('battery_input_topic and battery_output_topic must not be empty')
            if self.resolve_topic_name(bat_in) == self.resolve_topic_name(bat_out):
                raise ValueError('battery_input_topic and battery_output_topic must be different')
            self._battery_publisher = self.create_publisher(Float32, bat_out, 1)
            self._battery_subscription = self.create_subscription(
                BatteryState, bat_in, self._on_battery, qos_profile_sensor_data)
            self.get_logger().info(f'Battery gauge: {bat_in} -> {bat_out} (0..100 percent)')

        # Setup speed overlay publisher, subscriber, and 10Hz timer
        if self._speed_enabled:
            speed_in = self.get_parameter('speed_input_topic').value
            speed_out = self.get_parameter('speed_output_topic').value
            self._timeout = self.get_parameter('stale_timeout_sec').value
            if not math.isfinite(self._timeout) or self._timeout <= 0:
                raise ValueError('stale_timeout_sec must be finite and positive')
            self._speed = None
            self._received_at = None
            self._speed_publisher = self.create_publisher(OverlayText, speed_out, 1)
            self._speed_subscription = self.create_subscription(
                Odometry, speed_in, self._on_odometry, qos_profile_sensor_data)
            self._timer = self.create_timer(
                0.1, self._publish_speed, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(f'Speed overlay: {speed_in} -> {speed_out} (10 Hz)')

    def _on_battery(self, message):
        value = percentage_to_percent(message.percentage)
        if value is None:
            self.get_logger().warning(
                'Ignoring unknown/invalid battery percentage; expected a finite '
                'value in [0, 1]. The RViz gauge retains its last reading.',
                throttle_duration_sec=10.0)
            return
        self._battery_publisher.publish(Float32(data=value))

    def _on_odometry(self, message):
        self._speed = planar_speed(message.twist.twist)
        self._received_at = time.monotonic()

    def _publish_speed(self):
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
        self._speed_publisher.publish(message)


# Backward-compatible individual node wrappers
class BatteryPercentageBridge(Node):
    """Accept reliable or best-effort battery telemetry; publish reliable Float32."""

    def __init__(self):
        super().__init__('battery_percentage_bridge')
        self.declare_parameter('input_topic', '/j100_0519/platform/bms/state')
        self.declare_parameter('output_topic', '/nav2/battery_percentage')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must not be empty')
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError('input_topic and output_topic must be different')
        self._publisher = self.create_publisher(Float32, output_topic, 1)
        self._subscription = self.create_subscription(
            BatteryState, input_topic, self._on_battery, qos_profile_sensor_data)

    def _on_battery(self, message):
        value = percentage_to_percent(message.percentage)
        if value is None:
            self.get_logger().warning(
                'Ignoring unknown/invalid battery percentage; expected a finite '
                'value in [0, 1]. The RViz gauge retains its last reading.',
                throttle_duration_sec=10.0)
            return
        self._publisher.publish(Float32(data=value))


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
    """Run the unified RViz overlay bridge node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RVizOverlayBridge()
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
