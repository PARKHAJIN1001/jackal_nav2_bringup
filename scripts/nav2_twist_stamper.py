#!/usr/bin/env python3

"""Message-copy helper for Safety Guard; intentionally no executable node."""

from geometry_msgs.msg import Twist, TwistStamped


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
