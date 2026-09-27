#!/usr/bin/env python3
"""Collect two waypoints from RViz and shuttle between them N times via Nav2."""

import math
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_loop_core import LoopState


def heading_to_quaternion(yaw_rad):
    """Convert a 2D yaw angle in radians to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class WaypointLoopNode(Node):
    """ROS2 node that drives two-point shuttle navigation."""

    def __init__(self):
        super().__init__('waypoint_loop_node')
        self.declare_parameter('loop_count', 5)
        self.declare_parameter('driving_policy', 'nav2')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('external_cmd_vel_topic', '/external_cmd_vel')
        self.declare_parameter('joy_topic', '/j100_0519/joy_teleop/joy')
        self.declare_parameter('reset_button', 3)

        loop_count = self.get_parameter('loop_count').value
        driving_policy = self.get_parameter('driving_policy').value
        clicked_topic = self.get_parameter('clicked_point_topic').value
        external_topic = self.get_parameter('external_cmd_vel_topic').value
        joy_topic = self.get_parameter('joy_topic').value

        self.state = LoopState(loop_count=loop_count, driving_policy=driving_policy)
        self.goal_handle = None
        self._external_received_at = None
        self._last_diag = -1e9
        self._btn_pressed_since = None
        self._btn_triggered = False
        self._operator_phase = 'UNKNOWN'
        self._last_operator_diag = -1e9
        self._dispatch_timer = None
        self._pending_goal = None

        # --- Subscriptions ---
        self.create_subscription(
            PointStamped, clicked_topic, self._on_clicked_point, 10)
        self.create_subscription(
            Joy, joy_topic, self._on_joy, qos_profile_sensor_data)
        qos_diag = QoSProfile(depth=10)
        self.create_subscription(
            DiagnosticArray, '/nav2/operator_stop_diagnostics',
            self._on_operator_diag, qos_diag)

        if driving_policy == 'other':
            self.create_subscription(
                Twist, external_topic, self._on_external_cmd, qos_profile_sensor_data)

        # --- Action client ---
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # --- Diagnostics & Marker publishers ---
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/nav2/waypoint_loop_diagnostics', 10)
        qos_marker = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/nav2/loop_waypoints', qos_marker)

        # --- Periodic tick ---
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Waypoint loop node started: loop_count={loop_count}, '
            f'driving_policy={driving_policy}. '
            f'Use RViz "Publish Point" to click 2 waypoints.')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _cancel_active_goal(self):
        """Cancel current Nav2 goal if one is in flight."""
        if self._dispatch_timer is not None:
            self._dispatch_timer.cancel()
            self._dispatch_timer = None
        self._pending_goal = None
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel goal: {e}')
            self.goal_handle = None

    def _on_clicked_point(self, msg):
        """Collect waypoints from RViz Publish Point tool and publish markers."""
        was_active_nav = self.state.phase in ('NAVIGATING_TO_START', 'NAVIGATING')
        if was_active_nav:
            self.get_logger().info(
                'New waypoint clicked while navigating; cancelling current goal')
            self._cancel_active_goal()

        result = self.state.add_waypoint(msg.point.x, msg.point.y)
        self._btn_triggered = False
        self._btn_pressed_since = None
        self.get_logger().info(result)
        self._publish_markers()

    def _schedule_goal_dispatch(self, goal_tuple, delay_sec=0.25):
        """Schedule sending a goal after a short delay to avoid race with operator_stop."""
        if self._dispatch_timer is not None:
            self._dispatch_timer.cancel()
            self._dispatch_timer = None

        self._pending_goal = goal_tuple

        def _timer_cb():
            if self._dispatch_timer is not None:
                self._dispatch_timer.cancel()
                self._dispatch_timer = None
            if self._pending_goal is not None:
                goal = self._pending_goal
                self._pending_goal = None
                self._send_nav_goal(goal)

        self._dispatch_timer = self.create_timer(delay_sec, _timer_cb)

    def _on_joy(self, msg):
        """Monitor PS4 controller for Triangle button hold (2s)."""
        btn_idx = self.get_parameter('reset_button').value
        if len(msg.buttons) <= btn_idx:
            return
        if msg.buttons[btn_idx] == 1:
            now = time.monotonic()
            if self._btn_pressed_since is None:
                self._btn_pressed_since = now
            elif now - self._btn_pressed_since >= 2.0 and not self._btn_triggered:
                self._btn_triggered = True
                goal = self.state.operator_arm()
                if goal is not None:
                    stage = (
                        '1/2 -> Start'
                        if self.state.phase == 'NAVIGATING_TO_START'
                        else '2/2 -> Loop'
                    )
                    self.get_logger().info(
                        f'Operator armed via joy ({stage}); scheduling goal '
                        f'to ({goal[0]:.2f}, {goal[1]:.2f}, yaw={math.degrees(goal[2]):.1f}°)')
                    self._schedule_goal_dispatch(goal, delay_sec=0.25)
                    self._publish_markers()
        else:
            self._btn_pressed_since = None
            self._btn_triggered = False

    def _on_operator_diag(self, msg):
        """Detect operator arm from operator_stop diagnostics."""
        for status in msg.status:
            if status.name != 'nav2_operator_stop':
                continue
            self._operator_phase = status.message
            goal = self.state.operator_phase(status.message)
            if goal is not None and not self._btn_triggered:
                self._btn_triggered = True
                stage = (
                    '1/2 -> Start'
                    if self.state.phase == 'NAVIGATING_TO_START'
                    else '2/2 -> Loop'
                )
                self.get_logger().info(
                    f'Operator armed via diag ({stage}); scheduling goal '
                    f'to ({goal[0]:.2f}, {goal[1]:.2f}, yaw={math.degrees(goal[2]):.1f}°)')
                self._schedule_goal_dispatch(goal, delay_sec=0.25)
                self._publish_markers()

    def _on_external_cmd(self, msg):
        """Monitor external cmd_vel for the 'other' driving policy."""
        self._external_received_at = time.monotonic()

    # ------------------------------------------------------------------
    # Nav2 action helpers
    # ------------------------------------------------------------------

    def _send_nav_goal(self, goal_tuple):
        """Send a NavigateToPose goal to Nav2 with heading orientation."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            self.state.goal_failed('action_server_unavailable')
            return

        if len(goal_tuple) == 3:
            gx, gy, yaw = goal_tuple
        else:
            gx, gy = goal_tuple[:2]
            yaw = 0.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = heading_to_quaternion(yaw)

        self.get_logger().info(
            f'Sending Nav2 goal: ({gx:.2f}, {gy:.2f}, yaw={math.degrees(yaw):.1f}°) '
            f'[lap {self.state.completed_laps}/{self.state.loop_count}]')

        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """Handle goal acceptance or rejection."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 goal was rejected')
            self.state.goal_failed('goal_rejected')
            return
        self.get_logger().info('Nav2 goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _feedback_cb(self, feedback_msg):
        """Log navigation feedback periodically (every 10th message)."""
        pass  # Feedback logging can be added if needed

    def _goal_result_cb(self, future):
        """Handle goal completion, then send the next goal if needed."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            target = self.state.waypoints[self.state.current_target]
            self.get_logger().info(
                f'Reached waypoint ({target[0]:.2f}, {target[1]:.2f})')
            next_goal = self.state.goal_succeeded()
            if next_goal is not None:
                self._send_nav_goal(next_goal)
            elif self.state.phase == 'AT_START_WAITING_FOR_LOOP':
                self.get_logger().info(
                    f'Arrived at Start point ({target[0]:.2f}, {target[1]:.2f})! '
                    'Hold Triangle button for 2s to begin loop navigation.')
                self._btn_triggered = True  # Latch until released
                self._publish_markers()
            elif self.state.phase == 'COMPLETED':
                self.get_logger().info(
                    f'All {self.state.loop_count} laps completed!')
                self._publish_markers()
            elif self.state.phase == 'WAITING_EXTERNAL':
                self.get_logger().info(
                    'Waiting for external cmd_vel to drive to next waypoint...')
                self._publish_markers()
        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            action_str = 'canceled' if status == GoalStatus.STATUS_CANCELED else 'aborted'
            self.get_logger().warn(f'Nav2 goal was {action_str}')
            if self.state.phase == 'COLLECTING':
                return
            if self.state.phase == 'NAVIGATING_TO_START':
                self.state.phase = 'READY_FOR_START'
                self.state.reason = f'{action_str}_hold_triangle_to_retry'
            elif self.state.phase == 'NAVIGATING':
                self.state.phase = 'AT_START_WAITING_FOR_LOOP'
                self.state.reason = f'{action_str}_hold_triangle_to_retry'
            else:
                self.state.goal_failed(f'goal_{action_str}')
            self._publish_markers()
        else:
            self.get_logger().error(f'Nav2 goal failed with status {status}')
            self.state.goal_failed(f'status_{status}')

    # ------------------------------------------------------------------
    # Periodic tick
    # ------------------------------------------------------------------

    def _tick(self):
        """Publish diagnostics and visual markers periodically."""
        now = time.monotonic()
        if now - self._last_diag >= 1.0:
            self._last_diag = now
            self._publish_diagnostics()
            self._publish_markers()

    def _publish_markers(self):
        """Publish 3D markers in RViz for waypoints and route."""
        if not self.state.waypoints:
            return
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx, pt in enumerate(self.state.waypoints):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'waypoints'
            sphere.id = idx * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pt[0]
            sphere.pose.position.y = pt[1]
            sphere.pose.position.z = 0.2
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.5
            sphere.scale.y = 0.5
            sphere.scale.z = 0.5
            if idx == 0:
                sphere.color.r = 0.0
                sphere.color.g = 1.0
                sphere.color.b = 0.2
                sphere.color.a = 0.9
            else:
                sphere.color.r = 1.0
                sphere.color.g = 0.2
                sphere.color.b = 0.2
                sphere.color.a = 0.9
            msg.markers.append(sphere)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'waypoints'
            text.id = idx * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pt[0]
            text.pose.position.y = pt[1]
            text.pose.position.z = 0.7
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35
            if idx == 0:
                pos = f'({pt[0]:.2f}, {pt[1]:.2f})'
                if self.state.phase == 'AT_START_WAITING_FOR_LOOP':
                    label = f'1. START {pos} [ARRIVED! Hold Triangle 2s to Loop]'
                elif self.state.phase == 'READY_FOR_START':
                    label = f'1. START {pos} [Hold Triangle 2s to Move Here]'
                elif self.state.phase == 'NAVIGATING_TO_START':
                    label = f'1. START {pos} [Moving to Start...]'
                elif self.state.phase == 'COMPLETED':
                    label = f'1. START {pos} [COMPLETED]'
                elif self.state.phase == 'COLLECTING':
                    label = f'1. START {pos} [Click Goal with Publish Point]'
                else:
                    laps = f'{self.state.completed_laps}/{self.state.loop_count}'
                    label = f'1. START {pos} [Lap {laps}]'
            else:
                label = f'2. END ({pt[0]:.2f}, {pt[1]:.2f})'
            text.text = label
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            msg.markers.append(text)

        if len(self.state.waypoints) == 2:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'waypoints'
            line.id = 100
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.r = 1.0
            line.color.g = 0.85
            line.color.b = 0.0
            line.color.a = 0.9
            for pt in self.state.waypoints:
                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0.1
                line.points.append(p)
            msg.markers.append(line)
        else:
            for del_id in (2, 3, 100):
                del_marker = Marker()
                del_marker.header.frame_id = 'map'
                del_marker.header.stamp = now
                del_marker.ns = 'waypoints'
                del_marker.id = del_id
                del_marker.action = Marker.DELETE
                msg.markers.append(del_marker)

        self.marker_pub.publish(msg)

    def _publish_diagnostics(self):
        """Publish loop state as ROS diagnostics."""
        summary = self.state.summary()
        status = DiagnosticStatus(
            name='waypoint_loop_node',
            hardware_id='software_only',
        )
        terminal = self.state.phase in ('COMPLETED', 'ERROR')
        status.level = (
            DiagnosticStatus.OK if not terminal else
            DiagnosticStatus.ERROR if self.state.phase == 'ERROR' else
            DiagnosticStatus.OK
        )
        status.message = f'{self.state.phase}: {self.state.reason}'
        status.values = [
            KeyValue(key=k, value=str(v)) for k, v in summary.items()
        ]
        diag = DiagnosticArray(status=[status])
        diag.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(diag)

    def destroy_node(self):
        """Clean up pending timers and resources before destroying node."""
        if self._dispatch_timer is not None:
            self._dispatch_timer.cancel()
            self._dispatch_timer = None
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = WaypointLoopNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
