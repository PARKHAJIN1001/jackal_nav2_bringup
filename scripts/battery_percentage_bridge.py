#!/usr/bin/env python3

"""Relay BatteryState's fraction as a percentage for the RViz pie chart."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


def percentage_to_percent(percentage):
    """Return 0..100, or None for unknown/nonstandard BatteryState values."""
    if not math.isfinite(percentage) or not 0.0 <= percentage <= 1.0:
        return None
    return percentage * 100.0


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
        self.get_logger().info(
            f'Battery gauge: {input_topic} -> {output_topic} (0..100 percent)')

    def _on_battery(self, message):
        value = percentage_to_percent(message.percentage)
        if value is None:
            self.get_logger().warning(
                'Ignoring unknown/invalid battery percentage; expected a finite '
                'value in [0, 1]. The RViz gauge retains its last reading.',
                throttle_duration_sec=10.0)
            return
        self._publisher.publish(Float32(data=value))


def main(args=None):
    """Run the battery display bridge."""
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryPercentageBridge()
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
