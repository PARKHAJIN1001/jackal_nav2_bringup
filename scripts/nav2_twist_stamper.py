#!/usr/bin/env python3

"""Stamp Nav2 Twist commands for the Jackal platform command topic."""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


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


class Nav2TwistStamper(Node):
    """Convert Nav2's internal Twist stream to a stamped platform stream."""

    def __init__(self):
        super().__init__('nav2_twist_stamper')
        self.declare_parameter('input_topic', '/nav2_cmd_vel_unstamped')
        self.declare_parameter('output_topic', '/j100_0519/nav2_cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must not be empty')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist, input_topic, self._twist_callback, 10)
        self.get_logger().info(
            f'Stamping Nav2 velocity commands from {input_topic} to '
            f'{output_topic} in frame {self._frame_id}')

    def _twist_callback(self, message):
        stamped = make_stamped_twist(
            message, self.get_clock().now().to_msg(), self._frame_id)
        self._publisher.publish(stamped)


def main(args=None):
    """Run the Nav2 twist stamper node."""
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2TwistStamper()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok(context=node.context):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
