#!/usr/bin/env python3
"""Observe the PS4 and cancel Nav2 goals; never publish platform velocity commands."""

import math
import re
import subprocess
import threading
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from operator_stop_core import OperatorState, StopHeartbeat
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, ColorRGBA


class BluezProbe:
    """Bounded read-only SSH work runs outside the ROS executor."""

    def __init__(self, host, address):
        if not re.fullmatch(r'[A-Za-z0-9_.@-]+', host) or host.startswith('-'):
            raise ValueError('Invalid SSH host')
        if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', address):
            raise ValueError('Invalid controller Bluetooth address')
        self.command = [
            'ssh',
            '-o',
            'BatchMode=yes',
            '-o',
            'ConnectTimeout=1',
            '-o',
            'StrictHostKeyChecking=yes',
            host,
            'bluetoothctl info ' + address,
        ]
        self.result = (False, -math.inf)
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.done.is_set():
            try:
                result = subprocess.run(
                    self.command, capture_output=True, text=True, timeout=1.0, check=False
                )
                connected = result.returncode == 0 and any(
                    row.strip() == 'Connected: yes' for row in result.stdout.splitlines()
                )
            except (OSError, subprocess.TimeoutExpired):
                connected = False
            self.result = connected, time.monotonic()
            self.done.wait(0.2)

    def close(self):
        self.done.set()
        self.thread.join(timeout=1.5)


class OperatorStop(Node):

    def __init__(self):
        super().__init__('nav2_operator_stop')
        defaults = {
            'joy_topic': '/j100_0519/joy_teleop/joy',
            'odom_topic': '/j100_0519/platform/odom',
            'mapping_verified': True,
            'stop_button': 1,
            'reset_button': 3,
            'deadman_buttons': [4, 5],
            'neutral_axes': [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0],
            'nuc_host': 'administrator@192.168.50.2',
            'controller_address': 'A4:53:85:6E:27:41',
            'bridge_node': '/cmd_vel_safety_bridge',
            'max_linear_x': 0.5,
            'max_angular_z': 1.0,
            'footprint_confirmed': True,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.values = {key: self.get_parameter(key).value for key in defaults}
        self.state = OperatorState(
            **{
                k: self.values[k]
                for k in (
                    'stop_button',
                    'reset_button',
                    'deadman_buttons',
                    'neutral_axes',
                    'mapping_verified',
                )
            }
        )
        self.stability = StopHeartbeat()
        self.create_subscription(Bool, '/nav2/stack_ready', self.stack_ready, 1)
        self.probe = BluezProbe(self.values['nuc_host'], self.values['controller_address'])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Bool, '/nav2/operator_stop', qos)
        self.diag = self.create_publisher(DiagnosticArray, '/nav2/operator_stop_diagnostics', 10)
        self.overlay = self.create_publisher(OverlayText, '/nav2/operator_stop_overlay', 1)
        self.create_subscription(Joy, self.values['joy_topic'], self.joy, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.values['odom_topic'], self.odom, qos_profile_sensor_data
        )
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.goals, qos
        )
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.bridge = self.create_client(
            GetParameters, self.values['bridge_node'] + '/get_parameters'
        )
        self.cancel_future = self.bridge_future = None
        self.cancel_at = self.bridge_at = self.bridge_received = -math.inf
        self.cancel_confirmed = False
        self.cancel_detail = 'not queried'
        self.bridge_values = {}
        self.last_diag = -math.inf
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.clock)
        self.output.publish(Bool(data=True))

    def stack_ready(self, message):
        self.stability.receive(not message.data, time.monotonic())

    def joy(self, message):
        previous = self.state.phase
        now = time.monotonic()
        self.state.connection(True, now)
        self.state.joy(message.buttons, message.axes, now)
        if self.state.phase == 'STOPPED':
            if previous != 'STOPPED':
                self.cancel_confirmed = False
            self.output.publish(Bool(data=True))

    def odom(self, msg):
        t = msg.twist.twist
        self.state.odometry(math.hypot(t.linear.x, t.linear.y), t.angular.z, time.monotonic())

    def goals(self, message):
        active = {
            bytes(s.goal_info.goal_id.uuid).hex(): s.goal_info.stamp.sec * 1000000000
            + s.goal_info.stamp.nanosec
            for s in message.status_list
            if s.status in (1, 2, 3)
        }
        self.state.goals(active)
        if self.state.phase == 'STOPPED':
            self.output.publish(Bool(data=True))

    def queries(self, now):
        if self.cancel_future and self.cancel_future.done():
            try:
                response = self.cancel_future.result()
                self.cancel_confirmed = response.return_code == 0 and not response.goals_canceling
                self.cancel_detail = (
                    f'code={response.return_code}, pending={len(response.goals_canceling)}'
                )
            except Exception:
                self.cancel_confirmed = False
            self.cancel_future = None
        if self.cancel_future and now - self.cancel_at > 2:
            self.cancel.remove_pending_request(self.cancel_future)
            self.cancel_future = None
            self.cancel_confirmed = False
        if (
            self.state.phase == 'STOPPED'
            and not self.cancel_confirmed
            and now - self.cancel_at >= 0.5
            and self.cancel_future is None
        ):
            if self.cancel.service_is_ready():
                self.cancel_at = now
                self.cancel_future = self.cancel.call_async(CancelGoal.Request())
        if self.bridge_future and self.bridge_future.done():
            try:
                self.bridge_values = dict(
                    zip(
                        self.bridge_names,
                        map(parameter_value_to_python, self.bridge_future.result().values),
                    )
                )
                self.bridge_received = now
            except Exception:
                self.bridge_values = {}
            self.bridge_future = None
        if self.bridge_future and now - self.bridge_at > 2:
            self.bridge.remove_pending_request(self.bridge_future)
            self.bridge_future = None
            self.bridge_values = {}
        if (
            self.bridge_future is None
            and now - self.bridge_at >= 1
            and self.bridge.service_is_ready()
        ):
            self.bridge_names = ['max_linear_x', 'max_angular_z', 'forward_cmd_vel']
            self.bridge_at = now
            self.bridge_future = self.bridge.call_async(
                GetParameters.Request(names=self.bridge_names)
            )

    def tick(self):
        previous = self.state.phase
        now = time.monotonic()
        self.queries(now)
        connected, receipt = self.probe.result
        if 0 <= now - self.state.joy_at <= 1.0:
            connected = True
            receipt = now
        if receipt != self.state.link_at:
            self.state.connection(connected, receipt)
        bridge_active = self.bridge.service_is_ready()
        matched = (not bridge_active) or all(
            isinstance(self.bridge_values.get(k), (int, float))
            and math.isfinite(self.bridge_values[k])
            and 0 < self.values[k] <= self.bridge_values[k]
            for k in ('max_linear_x', 'max_angular_z')
        )
        bridge_ok = (
            not bridge_active
            or (
                matched
                and now - self.bridge_received <= 2.5
                and self.bridge_values.get('forward_cmd_vel') is True
            )
        )
        infrastructure = (
            self.values['footprint_confirmed']
            and bridge_ok
            and (self.cancel.service_is_ready() or not bridge_active or self.state.phase == 'WAITING FOR NEW GOAL')
        )
        blocked = self.state.tick(
            now,
            self.get_clock().now().nanoseconds,
            infrastructure,
        )
        if self.state.phase == 'STOPPED' and previous != 'STOPPED':
            self.cancel_confirmed = False
        self.output.publish(Bool(data=blocked))
        if now - self.last_diag >= 0.2:
            self.last_diag = now
            held = now - self.state.reset_since if self.state.reset_since is not None else 0.0
            link = 'connected' if connected and 0 <= now - receipt <= 3.0 else 'unavailable'
            values = {
                'phase': self.state.phase,
                'stack_ready': not self.stability.required(now),
                'reason': self.state.reason,
                'mapping_verified': self.state.verified,
                'connected': connected,
                'link_age': now - receipt,
                'joy_age': now - self.state.joy_at,
                'odom_age': now - self.state.odom_at,
                'cancel_confirmed': self.cancel_confirmed,
                'cancel_result': self.cancel_detail,
                'reset_hold_sec': held,
                'neutral': self.state.neutral,
                'bridge_limits_matched': matched,
                'footprint_confirmed': self.values['footprint_confirmed'],
                'active_goals': ','.join(self.state.active),
            }
            status = DiagnosticStatus(
                name='nav2_operator_stop',
                hardware_id='software_only',
                level=DiagnosticStatus.WARN if blocked else DiagnosticStatus.OK,
                message=self.state.phase,
                values=[KeyValue(key=k, value=str(v)) for k, v in values.items()],
            )
            diag = DiagnosticArray(status=[status])
            diag.header.stamp = self.get_clock().now().to_msg()
            self.diag.publish(diag)
            text = OverlayText()
            text.action, text.width, text.height = OverlayText.ADD, 420, 65
            text.horizontal_alignment, text.vertical_alignment = OverlayText.LEFT, OverlayText.TOP
            text.horizontal_distance, text.vertical_distance, text.text_size = 16, 200, 13.0
            text.fg_color = ColorRGBA(r=1.0, g=0.5 if blocked else 1.0, b=0.2, a=1.0)
            text.text = 'Nav2: ' + self.state.phase + '\n' + self.state.reason
            text.text += f'\nController link: {link} | Reset: {held:.1f}/2s'
            self.overlay.publish(text)

    def destroy_node(self):
        if self.context.ok():
            self.output.publish(Bool(data=True))
        self.probe.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = OperatorStop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
