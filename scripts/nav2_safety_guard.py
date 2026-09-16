#!/usr/bin/env python3
"""Raw-cloud preprocessing and independent, fail-closed stamped output gate."""

from dataclasses import fields
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from nav2_safety_core import (
    cloud_xyz,
    filter_points,
    GuardState,
    SafetyConfig,
    transform_xyz,
)
from nav2_twist_stamper import make_stamped_twist
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SafetyGuard(Node):

    def __init__(self):
        super().__init__('nav2_safety_guard')
        descriptor = ParameterDescriptor(read_only=True)
        defaults = SafetyConfig()
        for field in fields(defaults):
            self.declare_parameter(
                field.name, getattr(defaults, field.name), descriptor
            )
        self.config = SafetyConfig(
            **{
                f.name: self.get_parameter(f.name).value
                for f in fields(defaults)
            }
        )
        for name, value in {
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'input_topic': '/livox/lidar_local',
            'safety_points_topic': '/nav2/safety_points',
            'command_topic': '/nav2/collision_checked_cmd_vel',
            'output_topic': '/j100_0519/nav2_cmd_vel',
            'enable_motion': False,
        }.items():
            self.declare_parameter(name, value, descriptor)
        self.base = self.get_parameter('base_frame').value
        self.odom = self.get_parameter('odom_frame').value
        self.map = self.get_parameter('map_frame').value
        for name in (
            'base_frame',
            'odom_frame',
            'map_frame',
            'input_topic',
            'safety_points_topic',
            'command_topic',
            'output_topic',
        ):
            if not self.get_parameter(name).value:
                raise ValueError(f'{name} must not be empty')
        self.state = GuardState(
            self.config, self.get_parameter('enable_motion').value
        )
        self.buffer = Buffer(cache_time=Duration(seconds=3.0), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.points_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('safety_points_topic').value,
            qos_profile_sensor_data,
        )
        self.output_pub = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 1
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/nav2/safety_diagnostics', 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_topic').value,
            self.cloud,
            qos_profile_sensor_data,
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.get_parameter('command_topic').value, self.command, 1
        )
        self.last_diag = 0.0
        self.last_count = 0
        self.last_raw_stamp = None
        self.last_raw_receipt = None
        self.pending = None
        # A paused ROS /clock must not pause the watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.05, self.tick, clock=self.steady_clock
        )

    def command(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.state.clock(now):
            return
        self.state.receive_command(
            [
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            ],
            time.monotonic(),
        )

    def cloud(self, message):
        # Keep only the newest observation. Nonblocking TF retry never re-stamps it.
        self.last_raw_stamp = seconds(message.header.stamp)
        self.last_raw_receipt = time.monotonic()
        self.pending = (message, self.last_raw_receipt)
        self.process_cloud()

    def process_cloud(self):
        if self.pending is None:
            return
        message, received = self.pending
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            if (
                not message.header.frame_id
                or seconds(message.header.stamp) <= 0
            ):
                raise ValueError('missing_frame_or_stamp')
            if time.monotonic() - received > self.config.sensor_timeout:
                raise ValueError('cloud_tf_timeout')
            xyz = cloud_xyz(message)
            if message.header.frame_id != self.base:
                transform = self.buffer.lookup_transform(
                    self.base,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                )
                xyz = transform_xyz(xyz, transform.transform)
            xyz = filter_points(xyz, self.config)
            self.pending = None
            if not self.state.observe(
                seconds(message.header.stamp), now, received, xyz
            ):
                return
            self.last_count = len(xyz)
            output = PointCloud2()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.base
            output.height, output.width = 1, len(xyz)
            output.fields = [
                PointField(
                    name=name,
                    offset=i * 4,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
                for i, name in enumerate(('x', 'y', 'z'))
            ]
            output.point_step, output.row_step = 12, 12 * len(xyz)
            output.is_dense = True
            output.data = xyz.tobytes()
            self.points_pub.publish(output)
        except TransformException:
            self.state.invalidate_sensor('cloud_tf_missing')
        except (ValueError, TypeError, OverflowError) as error:
            self.pending = None
            self.state.invalidate_sensor(f'invalid_cloud:{error}')

    def tf_health(self, now):
        try:
            for parent, child, timeout, allowance in (
                (self.odom, self.base, self.config.tf_timeout, 0.0),
                (
                    self.map,
                    self.odom,
                    self.config.map_tf_timeout,
                    self.config.map_transform_tolerance,
                ),
            ):
                transform = self.buffer.lookup_transform(parent, child, Time())
                stamp = seconds(transform.header.stamp)
                age = now - stamp + allowance
                if stamp <= 0 or not (
                    -self.config.future_tolerance <= age <= timeout
                ):
                    return f'tf_stale:{parent}->{child}'
                # Validate values even when only checking chain availability.
                transform_xyz(np.zeros((1, 3)), transform.transform)
        except (TransformException, ValueError):
            return 'required_tf_missing_or_invalid'
        return ''

    def tick(self):
        self.process_cloud()
        stamp = self.get_clock().now()
        now, received = stamp.nanoseconds * 1e-9, time.monotonic()
        x, yaw, reason = self.state.decision(
            now, received, self.tf_health(now)
        )
        output = Twist()
        output.linear.x, output.angular.z = x, yaw
        self.output_pub.publish(
            make_stamped_twist(output, stamp.to_msg(), self.base)
        )
        if received - self.last_diag >= 1.0:
            self.last_diag = received
            status = DiagnosticStatus(
                name='nav2_safety_guard', hardware_id='software_only'
            )
            status.level = (
                DiagnosticStatus.OK
                if reason.startswith('passing_')
                else DiagnosticStatus.WARN
            )
            status.message = reason
            values = {
                'enable_motion': self.state.enable_motion,
                'sensor_error': self.state.sensor_error,
                'raw_sensor_stamp_age': (
                    now - self.last_raw_stamp
                    if self.last_raw_stamp is not None else 'missing'),
                'raw_sensor_receipt_age': (
                    received - self.last_raw_receipt
                    if self.last_raw_receipt is not None else 'missing'),
                'sensor_stamp_age': (
                    now - self.state.sensor[0]
                    if self.state.sensor
                    else 'missing'
                ),
                'sensor_receipt_age': (
                    received - self.state.sensor[1]
                    if self.state.sensor
                    else 'missing'
                ),
                'command_receipt_age': (
                    received - self.state.command[2]
                    if self.state.command
                    else 'missing'
                ),
                'filtered_points': self.last_count,
                'stop_points': self.state.stop_count,
            }
            status.values = [
                KeyValue(key=k, value=str(v)) for k, v in values.items()
            ]
            diagnostic = DiagnosticArray(status=[status])
            diagnostic.header.stamp = stamp.to_msg()
            self.diag_pub.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyGuard()
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
