#!/usr/bin/env python3
"""Read-only AMCL quality hints; covariance is not ground-truth pose accuracy."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def assess_covariance(covariance, frame, position_limit=0.8, yaw_limit=0.8):
    """Keep metres-squared and radians-squared thresholds separate."""
    if frame != 'map':
        return DiagnosticStatus.ERROR, 'unexpected_pose_frame'
    if len(covariance) != 36 or not all(math.isfinite(x) for x in covariance):
        return DiagnosticStatus.ERROR, 'invalid_covariance'
    x, y, yaw = (covariance[i] for i in (0, 7, 35))
    if min(x, y, yaw) < 0:
        return DiagnosticStatus.ERROR, 'invalid_covariance'
    if max(x, y) >= position_limit or yaw >= yaw_limit:
        return DiagnosticStatus.WARN, 'high_covariance_check_localization_manually'
    return DiagnosticStatus.OK, 'covariance_below_limits_not_accuracy_verification'


class AmclQualityMonitor(Node):
    """Publish only diagnostics. Never cache a pose for replay or command motion."""

    def __init__(self):
        super().__init__('amcl_quality_monitor')
        self.position_limit = self.declare_parameter('position_variance_limit', 0.8).value
        self.yaw_limit = self.declare_parameter('yaw_variance_limit', 0.8).value
        for value in (self.position_limit, self.yaw_limit):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('variance limits must be finite and positive')
        self.publisher = self.create_publisher(
            DiagnosticArray, '/nav2/localization_diagnostics', 1)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos_profile_sensor_data)
        self.last_warning = -math.inf

    def on_pose(self, msg):
        level, reason = assess_covariance(
            msg.pose.covariance, msg.header.frame_id, self.position_limit, self.yaw_limit)
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        status = DiagnosticStatus(
            name='amcl_quality', hardware_id='localization', level=level, message=reason)
        status.values = [KeyValue(key=key, value=str(value)) for key, value in (
            ('pose_stamp_sec', stamp),
            ('pose_age_sec', now.nanoseconds * 1e-9 - stamp),
            ('variance_x_m2', msg.pose.covariance[0]),
            ('variance_y_m2', msg.pose.covariance[7]),
            ('variance_yaw_rad2', msg.pose.covariance[35]),
            ('automatic_reinitialization', False),
        )]
        report = DiagnosticArray(status=[status])
        report.header.stamp = now.to_msg()
        self.publisher.publish(report)
        # AMCL poses are motion/event driven; TF freshness belongs to Guard/audit.
        mono = time.monotonic()
        if level != DiagnosticStatus.OK and mono - self.last_warning >= 5.0:
            self.get_logger().warning(reason + '; no automatic pose reset performed')
            self.last_warning = mono


def main(args=None):
    rclpy.init(args=args)
    node = AmclQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
