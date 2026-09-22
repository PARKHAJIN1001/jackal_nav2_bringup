"""
Opt-in real Nav2 goal tests with a kinematic base, never a platform command writer.

ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 pytest -s test/integration_goal_navigation.py
Run with the tested package overlay sourced. AMCL/map services are fixtures;
planner, controller, BT, smoother, collision monitor and guard are real nodes.
"""

import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Joy, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from record_navigation_validation import NavigationBag, Recording  # noqa: E402,I100
import operator_stop as operator_module  # noqa: E402,I100


class LocalLink:
    connected = True

    @property
    def result(self):
        return self.connected, time.monotonic()

    def close(self):
        pass


class GoalRig:
    """Integrate ONLY /nav2_test/output; never synthesize control commands."""

    def __init__(self, directory, enabled):
        assert os.environ.get('ROS_DOMAIN_ID') == '188', 'Dedicated test domain 188 required'
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Localhost isolation required'
        self.directory = directory
        self.enabled = enabled
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.command_at = -math.inf
        self.last_tick = time.monotonic()
        self.cloud_enabled = self.odom_tf_enabled = True
        self.map_tf_enabled = False
        self.blocked = False
        self.outputs, self.controller_commands, self.paths, self.scenarios = [], [], [], []
        self.bag = None
        rclpy.init(args=['--ros-args', '-p', 'mapping_verified:=true', '-p', 'stop_button:=0',
                         '-p', 'reset_button:=1', '-p', 'deadman_buttons:=[2,3]',
                         '-p', 'neutral_axes:=[0.0,0.0]', '-p', 'footprint_confirmed:=true'])
        self.node = Node('goal_test_base')
        self.stack_ready = self.node.create_publisher(Bool, '/nav2/stack_ready', 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.link = LocalLink()
        self.buttons = [0, 0, 0, 0]
        self.bridge = Node('cmd_vel_safety_bridge')
        self.bridge.declare_parameter('max_linear_x', 0.5)
        self.bridge.declare_parameter('max_angular_z', 1.0)
        self.bridge.declare_parameter('forward_cmd_vel', True)
        self.executor.add_node(self.bridge)
        original_probe = operator_module.BluezProbe
        try:
            operator_module.BluezProbe = lambda *args: self.link
            self.operator = operator_module.OperatorStop() if enabled else None
        finally:
            operator_module.BluezProbe = original_probe
        if self.operator:
            self.executor.add_node(self.operator)
        self.joy = self.node.create_publisher(
            Joy, '/j100_0519/joy_teleop/joy', qos_profile_sensor_data)
        self.platform_odom = self.node.create_publisher(
            Odometry, '/j100_0519/platform/odom', qos_profile_sensor_data)
        self.localization = []
        for name in ('amcl', 'map_server'):
            fixture = Node(name)
            fixture.create_service(GetState, '/' + name + '/get_state', self.active)
            self.executor.add_node(fixture)
            self.localization.append(fixture)
        self.broadcaster = TransformBroadcaster(self.node)
        self.map = self.node.create_publisher(
            OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = self.node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.scan = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.cloud = self.node.create_publisher(
            PointCloud2, '/nav2_test/raw', qos_profile_sensor_data)
        self.node.create_subscription(TwistStamped, '/nav2_test/output', self.on_output, 10)
        self.node.create_subscription(Twist, '/cmd_vel_nav', self.on_controller, 10)
        self.node.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.node.create_subscription(NavPath, '/local_plan', self.on_plan, 10)
        self.client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.node.create_timer(0.025, self.tick)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 160
        grid.info.resolution = 0.05
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [100 if x in (0, 159) or y in (0, 159) else 0
                     for y in range(160) for x in range(160)]
        self.map.publish(grid)
        self.grid = grid
        self.log = (directory / 'launch.log').open('w')
        prefix = (['--launch-prefix',
                   'gdb -batch -ex "handle SIGINT nostop pass" -ex run '
                   '-ex "thread apply all bt" --args',
                   '--launch-prefix-filter', 'static_costmap_node']
                  if os.environ.get('GOAL_TEST_GDB') == '1' else [])
        self.process = subprocess.Popen([
            'ros2', 'launch', str(ROOT / 'test/isolated_navigation.launch.py'),
            'use_composition:=false', 'use_map_patch:=false',
            'launch_operator_stop:=false',
            f'enable_motion:={str(enabled).lower()}',
            'lidar_pointcloud_topic:=/nav2_test/raw', 'nav_cmd_vel_topic:=/nav2_test/output',
        ] + prefix, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    @staticmethod
    def active(request, response):
        response.current_state.id, response.current_state.label = 3, 'active'
        return response

    def on_output(self, msg):
        self.v, self.w, self.command_at = msg.twist.linear.x, msg.twist.angular.z, time.monotonic()
        self.outputs.append((self.command_at, self.v, self.w))

    def on_controller(self, msg):
        self.controller_commands.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def on_plan(self, msg):
        self.paths.append(len(msg.poses))

    def tick(self):
        self.stack_ready.publish(Bool(data=True))
        mono = time.monotonic()
        dt, self.last_tick = min(mono - self.last_tick, 0.1), mono
        v, w = (self.v, self.w) if mono - self.command_at < 0.25 else (0.0, 0.0)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom', 'base_link', stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odom.publish(odom)
        self.platform_odom.publish(odom)
        self.joy.publish(Joy(header=odom.header, axes=[0.0, 0.0], buttons=self.buttons))
        transforms = []
        if self.odom_tf_enabled:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation = odom.pose.pose.orientation
            transforms.append(tf)
        if self.map_tf_enabled:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
            # Match AMCL's configured one-second transform dating allowance.
            tf.header.stamp.sec, tf.header.stamp.nanosec = stamp.sec + 1, stamp.nanosec
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        if transforms:
            self.broadcaster.sendTransform(transforms)
        if self.cloud_enabled:
            scan = LaserScan(angle_min=-1.0, angle_max=1.0, angle_increment=0.1,
                             range_min=0.1, range_max=30.0, ranges=[3.0] * 21)
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            self.scan.publish(scan)
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp = 'base_link', stamp
            points = [(0.4, 0.1, 0.5)] * 3 if self.blocked else [(2.0, 0.0, 0.5)]
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
            self.cloud.publish(cloud)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def until(self, predicate, timeout=20):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.pump(0.02)
            assert self.process.poll() is None, (self.directory / 'launch.log').read_text()[-6000:]
        assert predicate(), (
            f'timeout: pose={(self.x, self.y, self.yaw)}, output={(self.v, self.w)}')

    def goal(self, x=1.0, y=0.0, yaw=0.0):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        request = self.client.send_goal_async(msg)
        self.until(request.done, 5)
        handle = request.result()
        assert handle.accepted
        return handle, handle.get_result_async()

    def success(self, result, x, y, yaw=0):
        self.until(result.done, 35)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert math.hypot(self.x - x, self.y - y) <= 0.25
        assert abs(math.atan2(math.sin(self.yaw-yaw), math.cos(self.yaw-yaw))) <= 0.25
        self.stopped()

    def stopped(self):
        # Action completion and downstream deceleration are asynchronous.
        # Require continuous zero for 1.1s, starting no later than 1s from here.
        started, cursor, zero_since = time.monotonic(), len(self.outputs), None
        pose = None
        while time.monotonic() - started < 2.2:
            self.pump(0.025)
            for stamp, v, w in self.outputs[cursor:]:
                if abs(v) < 1e-6 and abs(w) < 1e-6:
                    if zero_since is None:
                        zero_since, pose = stamp, (self.x, self.y, self.yaw)
                    if stamp - zero_since >= 1.1:
                        assert zero_since - started <= 1.0
                        assert math.dist(pose, (self.x, self.y, self.yaw)) < 0.02
                        return
                else:
                    zero_since = None
            cursor = len(self.outputs)
        raise AssertionError('Output did not settle to continuous zero within the stop deadline')

    def cancel(self, handle, result):
        if not result.done():
            future = handle.cancel_goal_async()
            self.until(future.done, 5)
            self.until(result.done, 5)
        assert result.result().status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
        self.stopped()

    def reset(self):
        # Only between terminated actions; this is a fixture reset, not LIO behavior.
        self.stopped()
        self.x = self.y = self.yaw = 0.0
        self.pump(1.3)

    def mark(self, name):
        self.scenarios.append({'scenario': name, 'result': 'passed'})
        (self.directory / 'scenarios.json').write_text(json.dumps(self.scenarios, indent=2) + '\n')
        print('PASS:', name, flush=True)

    def corridor(self, enabled):
        self.grid.data = [100 if x in (0, 159) or y in (0, 159) or
                          (enabled and y in (64, 95)) else 0
                          for y in range(160) for x in range(160)]
        self.map.publish(self.grid)
        self.pump(1.5)

    def arm(self):
        self.buttons = [0, 0, 0, 0]
        self.pump(2)
        self.buttons = [0, 1, 0, 0]
        try:
            self.until(lambda: self.operator.state.phase == 'WAITING FOR NEW GOAL', 6)
        except AssertionError:
            print(vars(self.operator.state), self.operator.bridge_values,
                  self.operator.cancel_detail, flush=True)
            raise
        self.buttons = [0, 0, 0, 0]
        self.pump(0.2)

    def background(self, action):
        errors = []

        def run():
            try:
                action()
            except BaseException as error:
                errors.append(error)
        thread = threading.Thread(target=run)
        thread.start()
        self.until(lambda: not thread.is_alive(), 25)
        thread.join()
        if errors:
            raise errors[0]

    def readiness(self, expected, require_motion=False, timeout=8):
        output = self.directory / f'ready_{len(self.scenarios)}_{require_motion}.json'
        command = [sys.executable, str(ROOT / 'scripts/check_navigation_ready.py'),
                   '--timeout', str(timeout), '--settle', '0.5',
                   '--nav-cmd-topic', '/nav2_test/output', '--output', str(output)]
        if require_motion:
            command.append('--require-motion')
        with (self.directory / 'readiness.console').open('a') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            try:
                self.until(lambda: process.poll() is not None, timeout + 5)
                assert process.returncode == expected, stream.name
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
        report = json.loads(output.read_text())
        assert report['ready'] == (expected == 0)
        return report

    def close(self):
        if self.bag:
            self.bag.stop()
            self.bag = None
        forced = False
        try:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            forced = True
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=5)
        finally:
            self.log.close()
            self.executor.shutdown()
            for node in (*self.localization, self.bridge, self.node):
                node.destroy_node()
            if self.operator:
                self.operator.destroy_node()
            rclpy.try_shutdown()
        log = (self.directory / 'launch.log').read_text()
        assert not forced and self.process.returncode == 0, log[-5000:]
        assert not any(text in log for text in (
            'failed to terminate', 'exit code -6', 'exit code -11')), log[-5000:]


@pytest.mark.parametrize('enabled', [True, False])
def test_goal_to_final_velocity_and_kinematic_arrival(tmp_path, enabled):
    rig = GoalRig(tmp_path, enabled)
    try:
        if enabled:
            report = rig.readiness(1, timeout=4)
            assert not report['checks']['tf:map->odom']['ok']
            rig.mark('initialization_missing_is_not_ready')
        rig.map_tf_enabled = True
        rig.until(rig.client.server_is_ready, 35)
        rig.pump(2)
        if enabled:
            rig.arm()
        report = rig.readiness(0, timeout=15)
        assert report['perception'] == 'not_observed_or_loading'
        assert not report['motion_ready']  # No robot bridge exists in this isolated domain.
        rig.mark('idle_navigation_ready_without_perception_or_command')
        report = rig.readiness(1, require_motion=True, timeout=4)
        assert not report['checks']['motion_configuration']['ok']
        rig.mark('motion_check_rejects_missing_platform_bridge')
        if enabled:
            capture = tmp_path / 'capture'
            capture.mkdir()
            rig.bag = NavigationBag(Recording(capture, 12, False), '/nav2_test/output', '/odom')
            rig.background(rig.bag.start)
            recording_started = time.monotonic()
        handle, result = rig.goal()
        if not enabled:
            rig.until(lambda: any(v > 0.01 for _, v, _ in rig.controller_commands), 15)
            rig.pump(2)
            assert rig.x == rig.y == 0.0
            assert all(v == w == 0.0 for _, v, w in rig.outputs)
            rig.cancel(handle, result)
            rig.mark('disabled_guard_blocks_real_nav2_goal_commands')
            return
        rig.success(result, 1, 0)
        assert any(count > 1 for count in rig.paths)
        rig.mark('straight_goal_succeeded_and_stopped')
        _, result = rig.goal(rig.x, rig.y, math.pi/2)
        rig.success(result, rig.x, rig.y, math.pi/2)
        rig.mark('goal_heading_alignment')
        rig.pump(max(0, 13-(time.monotonic()-recording_started)))
        rig.background(rig.bag.stop)
        assert rig.bag.step['state'] == 'captured', rig.bag.step
        counts = rig.bag.recording.status['navigation_capture']['topic_message_counts']
        assert counts['/navigate_to_pose/_action/feedback'] > 0
        assert counts['/navigate_to_pose/_action/status'] > 0
        rig.bag = None
        rig.mark('navigation_rosbag_action_and_velocity_capture')
        rig.reset()
        rig.corridor(True)
        _, result = rig.goal()
        rig.success(result, 1, 0)
        rig.mark('mapped_corridor_goal_succeeded')
        rig.corridor(False)
        rig.reset()
        old, old_result = rig.goal(2, 0)
        rig.until(lambda: rig.x > 0.15)
        _, result = rig.goal(0.8, 0)
        rig.success(result, 0.8, 0)
        rig.until(old_result.done, 3)
        assert old_result.result().status != GoalStatus.STATUS_SUCCEEDED
        rig.mark('replacement_goal_preempts_old_goal')
        rig.reset()
        handle, result = rig.goal(2, 0)
        rig.until(lambda: rig.v > 0.05)
        rig.cancel(handle, result)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        rig.mark('cancel_terminates_goal_and_stops')
        rig.reset()
        _, result = rig.goal(2, 0)
        rig.until(lambda: rig.v > 0.05)
        cursor, pressed = len(rig.outputs), time.monotonic()
        rig.buttons = [1, 0, 0, 0]
        rig.until(lambda: any(v == w == 0 for _, v, w in rig.outputs[cursor:]), 0.5)
        first_zero = next(t for t, v, w in rig.outputs[cursor:] if v == w == 0)
        assert first_zero-pressed <= 0.1
        (tmp_path / 'operator_latency.json').write_text(json.dumps({
            'fixture_button_to_final_zero_sec': first_zero-pressed,
            'physical_button_to_stop_tested': False}))
        rig.until(result.done, 5)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        rig.stopped()
        rig.buttons = [0, 0, 0, 0]
        rig.pump(1)
        assert rig.operator.state.phase == 'STOPPED'
        rig.arm()
        rig.stopped()
        rig.mark('circle_stops_within_100ms_and_reset_does_not_replay_goal')
        rig.reset()
        _, result = rig.goal(2, 0)
        rig.until(lambda: rig.v > 0.05)
        rig.link.connected = False  # Joy continues at 40Hz despite disconnection.
        rig.stopped()
        rig.until(result.done, 5)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        rig.link.connected = True
        rig.pump(1)
        assert rig.operator.state.phase == 'STOPPED'
        rig.arm()
        rig.stopped()
        rig.mark('bluetooth_loss_with_repeated_joy_latches_stop')
        _, result = rig.goal(2, 0)
        rig.until(lambda: rig.v > 0.05)
        rig.executor.remove_node(rig.operator)
        rig.stopped()  # No heartbeat: Guard must stop without a cancel response.
        rig.operator.destroy_node()
        original_probe = operator_module.BluezProbe
        try:
            operator_module.BluezProbe = lambda *args: rig.link
            rig.operator = operator_module.OperatorStop()
        finally:
            operator_module.BluezProbe = original_probe
        rig.executor.add_node(rig.operator)
        rig.until(result.done, 5)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        rig.stopped()
        assert rig.operator.state.phase == 'STOPPED'
        _, stopped_result = rig.goal(2, 0)
        rig.until(stopped_result.done, 5)
        assert stopped_result.result().status == GoalStatus.STATUS_CANCELED
        rig.stopped()
        rig.arm()
        rig.mark('operator_loss_restart_and_goals_while_stopped')
        rig.reset()
        _, result = rig.goal()
        rig.until(lambda: rig.x > 0.1)
        rig.blocked = True
        rig.stopped()
        assert not result.done()
        rig.blocked = False
        rig.success(result, 1, 0)
        rig.mark('brief_obstacle_stop_then_resume')
        rig.reset()
        _, result = rig.goal()
        rig.until(lambda: rig.x > 0.1)
        rig.blocked = True
        rig.stopped()
        rig.until(result.done, 35)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        rig.blocked = False
        rig.stopped()
        rig.mark('persistent_obstacle_aborts_without_recovery_or_replay')
        rig.reset()
        _, result = rig.goal(100, 0)
        rig.until(result.done, 10)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        rig.stopped()
        rig.mark('unreachable_goal_aborts_and_stops')
        for fault in ('cloud_enabled', 'odom_tf_enabled'):
            rig.reset()
            handle, result = rig.goal()
            rig.until(lambda: rig.x > 0.1)
            setattr(rig, fault, False)
            rig.stopped()
            rig.cancel(handle, result)
            setattr(rig, fault, True)
            rig.pump(1.2)
            rig.mark(fault + '_loss_stops_output')
        rig.reset()
        handle, result = rig.goal()
        rig.until(lambda: rig.x > 0.1)
        rows = subprocess.check_output(['ps', '-eo', 'pid,pgid,args'], text=True).splitlines()[1:]
        monitors = [int(parts[0]) for row in rows if len(parts := row.split(None, 2)) == 3
                    and parts[1] == str(rig.process.pid)
                    and '/lib/nav2_collision_monitor/collision_monitor ' in parts[2]]
        assert len(monitors) == 1
        os.kill(monitors[0], signal.SIGINT)
        rig.stopped()
        rig.cancel(handle, result)
        rig.mark('lost_monitor_commands_stop_output')
        assert all(v >= -1e-6 and abs(v) <= 0.50 + 1e-6 and abs(w) <= 1.0 + 1e-6
                   for _, v, w in rig.outputs)
    finally:
        rig.close()
