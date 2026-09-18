#!/usr/bin/env python3
"""Inspect navigation readiness without sending goals, commands or state changes."""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState
from moai_nav_msgs.msg import Tracks
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from topic_ready_gate import message_error


LIFECYCLE_NODES = (
    '/map_server', '/amcl', '/planner_server', '/controller_server',
    '/bt_navigator', '/velocity_smoother', '/collision_monitor',
    '/local_costmap/local_costmap', '/global_costmap/global_costmap',
)
GUARD_PARAMETERS = (
    'enable_motion', 'output_topic', 'base_frame', 'odom_frame', 'map_frame',
    'sensor_timeout', 'tf_timeout', 'map_tf_timeout',
    'map_transform_tolerance', 'future_tolerance',
)
BRIDGE_PARAMETERS = ('forward_cmd_vel', 'input_topic', 'output_topic', 'timeout_sec')
IDLE_REASONS = {'command_missing_or_invalid', 'monitor_command_timeout'}


def seconds(stamp):
    """Convert a ROS timestamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(stamp, receipt, now, monotonic, timeout=0.3, future=0.05):
    """Require both valid measurement age and recent monotonic receipt."""
    return (math.isfinite(stamp) and stamp > 0 and
            -future <= now - stamp <= timeout and
            0 <= monotonic - receipt <= timeout)


def safety_ready(reason, active_goal):
    """Allow an idle controller to have no command; never mask a sensor/TF stop."""
    return (reason in {'motion_disabled', 'passing_collision_checked_command'} or
            (not active_goal and reason in IDLE_REASONS))


def bridge_ready(parameters, publishers, nav_topic, platform_topic, bridge_node):
    """Check startup configuration AND the actual output writer, not a bool alone."""
    return (parameters.get('forward_cmd_vel') is True and
            parameters.get('input_topic') == nav_topic and
            parameters.get('output_topic') == platform_topic and
            publishers == [bridge_node])


def valid_transform(transform):
    """Reject nonfinite and non-unit transforms even if their timestamps are fresh."""
    t, q = transform.translation, transform.rotation
    return (all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)) and
            abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) <= 0.01)


class NavigationReadiness(Node):
    """Use subscriptions, GetState/GetParameters and action discovery only."""

    def __init__(self, args):
        super().__init__('check_navigation_ready')
        self.args = args
        self.samples = {}
        self.scans = deque(maxlen=40)
        self.map_valid = False
        self.diagnostic = None
        self.active_goal = False
        self.perception_at = None
        self.last_ros = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(OccupancyGrid, args.map_topic, self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, args.scan_topic,
                                 lambda msg: self.on_sample('scan', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic,
                                 lambda msg: self.on_sample('odom', msg), qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/nav2/safety_diagnostics', self.on_safety, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Tracks, '/ped_tracking', self.on_perception,
                                 qos_profile_sensor_data)
        self.queries = {}
        for node in LIFECYCLE_NODES:
            self.add_query(node, GetState, 'get_state', GetState.Request())
        self.add_query('/nav2_safety_guard', GetParameters, 'get_parameters',
                       GetParameters.Request(names=list(GUARD_PARAMETERS)))
        self.add_query(args.bridge_node, GetParameters, 'get_parameters',
                       GetParameters.Request(names=list(BRIDGE_PARAMETERS)))
        self.create_timer(0.1, self.poll_queries)

    def add_query(self, node, service_type, service, request):
        """Create a bounded asynchronous read; absent services cannot block others."""
        self.queries[node] = {
            'client': self.create_client(service_type, node + '/' + service),
            'request': request, 'future': None, 'sent': -math.inf,
            'received': -math.inf, 'value': None,
        }

    def poll_queries(self):
        """Refresh observations and invalidate failed or timed-out queries."""
        now = time.monotonic()
        for query in self.queries.values():
            future = query['future']
            if future is not None and future.done():
                try:
                    query['value'] = future.result()
                    query['received'] = now
                except Exception:
                    query['value'] = None
                query['future'] = None
            elif future is not None and now - query['sent'] > 2:
                query['client'].remove_pending_request(future)
                future.cancel()
                query['future'], query['value'] = None, None
            if (query['future'] is None and now - query['sent'] >= 1 and
                    query['client'].service_is_ready()):
                query['sent'] = now
                query['future'] = query['client'].call_async(query['request'])

    def value(self, node, monotonic):
        """Return only a recent successful service response."""
        query = self.queries[node]
        return query['value'] if monotonic - query['received'] <= 2.5 else None

    def parameters(self, node, names, monotonic):
        """Decode declared parameters; unset or unavailable values remain missing."""
        response = self.value(node, monotonic)
        return dict(zip(names, map(parameter_value_to_python, response.values))) if response else {}

    def on_map(self, msg):
        """Accept a valid latched map; a static map need not have a recent stamp."""
        self.map_valid = (msg.header.frame_id == 'map' and
                          msg.info.width > 0 and msg.info.height > 0 and
                          math.isfinite(msg.info.resolution) and msg.info.resolution > 0 and
                          len(msg.data) == msg.info.width * msg.info.height)

    def on_sample(self, name, msg):
        """Track usable scan/odometry measurements without altering them."""
        receipt = time.monotonic()
        expected = 'base_link' if name == 'scan' else 'odom'
        error = message_error(msg, expected, 'base_link' if name == 'odom' else '')
        self.samples[name] = (seconds(msg.header.stamp), receipt, error)
        if name == 'scan' and not error:
            self.scans.append((msg.header, receipt))

    def on_safety(self, msg):
        """Keep the guard's decision and timestamp, not a generic diagnostic level."""
        for status in msg.status:
            if status.name == 'nav2_safety_guard':
                self.diagnostic = (seconds(msg.header.stamp), time.monotonic(), status.message)

    def on_status(self, msg):
        """Separate idle command silence from silence during an active goal."""
        self.active_goal = any(entry.status in (1, 2, 3) for entry in msg.status_list)

    def on_perception(self, msg):
        """Observe even empty tracking messages; perception is informational only."""
        self.perception_at = time.monotonic()

    def publishers(self, topic, message_type):
        """Identify typed writers by full node name, retaining duplicates."""
        return sorted((info.node_namespace.rstrip('/') + '/' + info.node_name)
                      for info in self.get_publishers_info_by_topic(topic)
                      if info.topic_type == message_type)

    def report(self):
        """Take a point-in-time report; this is not an automatic motion interlock."""
        now, mono = self.get_clock().now().nanoseconds * 1e-9, time.monotonic()
        if self.last_ros is not None and now < self.last_ros:
            self.samples.clear()
            self.scans.clear()
            self.diagnostic = None
        self.last_ros = now
        checks = {}

        def check(name, ok, detail):
            checks[name] = {'ok': bool(ok), 'detail': detail}

        check('map', self.map_valid, self.args.map_topic)
        for name in ('scan', 'odom'):
            sample = self.samples.get(name)
            ok = sample and not sample[2] and fresh(sample[0], sample[1], now, mono)
            check(name, ok, 'fresh valid measurements' if ok else 'missing, invalid or stale')
        inputs_ready = all(c['ok'] for c in checks.values())
        for node in LIFECYCLE_NODES:
            value = self.value(node, mono)
            check('lifecycle:' + node, value and value.current_state.id == 3,
                  value.current_state.label if value else 'no recent response')
        check('navigate_to_pose', self.action.server_is_ready(), 'action server discovery')
        guard = self.parameters('/nav2_safety_guard', GUARD_PARAMETERS, mono)
        check('guard_configuration', all(guard.get(k) is not None for k in GUARD_PARAMETERS),
              guard)
        for parent, child, timeout_key, allowance_key in (
                ('odom', 'base_link', 'tf_timeout', None),
                ('map', 'odom', 'map_tf_timeout', 'map_transform_tolerance')):
            ok, detail = False, 'missing TF or guard parameters'
            try:
                tf = self.buffer.lookup_transform(parent, child, Time())
                age = now - seconds(tf.header.stamp) + (guard[allowance_key] if allowance_key else 0)
                ok = (seconds(tf.header.stamp) > 0 and valid_transform(tf.transform) and
                      -guard['future_tolerance'] <= age <= guard[timeout_key])
                detail = f'measurement age after TF dating allowance: {age:.3f}s'
            except (TransformException, KeyError, TypeError):
                pass
            check('tf:' + parent + '->' + child, ok, detail)
        scan_tf = any(fresh(seconds(h.stamp), r, now, mono) and
                      self.buffer.can_transform('map', h.frame_id, Time.from_msg(h.stamp))
                      for h, r in self.scans)
        check('scan_time_tf', scan_tf, 'map -> scan frame at a recent scan timestamp')
        diag = self.diagnostic
        check('safety', diag and fresh(diag[0], diag[1], now, mono, timeout=2.5) and
              safety_ready(diag[2], self.active_goal), diag[2] if diag else 'no diagnostic')
        counts = Counter(ns.rstrip('/') + '/' + name
                         for name, ns in self.get_node_names_and_namespaces())
        required_nodes = (*LIFECYCLE_NODES, '/nav2_safety_guard')
        duplicates = [name for name in required_nodes if counts[name] != 1]
        check('node_ownership', not duplicates, duplicates)
        writers = self.publishers(self.args.nav_cmd_topic, 'geometry_msgs/msg/TwistStamped')
        check('guard_output', guard.get('output_topic') == self.args.nav_cmd_topic and
              writers == ['/nav2_safety_guard'], writers)
        monitor = self.publishers('/nav2/collision_checked_cmd_vel', 'geometry_msgs/msg/Twist')
        check('collision_monitor_output', monitor == ['/collision_monitor'], monitor)
        navigation_ready = all(c['ok'] for c in checks.values())
        bridge = self.parameters(self.args.bridge_node, BRIDGE_PARAMETERS, mono)
        platform_writers = self.publishers(self.args.platform_cmd_topic, 'geometry_msgs/msg/Twist')
        forwarding = (bridge_ready(bridge, platform_writers, self.args.nav_cmd_topic,
                                  self.args.platform_cmd_topic, self.args.bridge_node) and
                      counts[self.args.bridge_node] == 1)
        motion = {'enable_motion': guard.get('enable_motion'),
                  'bridge_parameters': bridge, 'platform_publishers': platform_writers,
                  'forwarding_observed': forwarding}
        motion_ready = navigation_ready and guard.get('enable_motion') is True and forwarding
        if self.args.require_motion:
            check('motion_configuration', motion_ready, motion)
        return {
            'ready': all(c['ok'] for c in checks.values()),
            'inputs_ready': inputs_ready, 'navigation_ready': navigation_ready,
            'motion_ready': motion_ready, 'checks': checks, 'motion': motion,
            'perception': ('messages_observed' if self.perception_at is not None and
                           mono - self.perception_at <= 2 else 'not_observed_or_loading'),
            'meaning': 'Read-only snapshot; manually confirm map/scan alignment. '
                       'Does not enable motion, prove pose accuracy or validate braking.',
        }


def main(argv=None):
    """Exit 0 when ready, 1 on readiness timeout, 2 on error, 130 on interrupt."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-motion', action='store_true')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--settle', type=float, default=2.0)
    parser.add_argument('--output', type=Path, help='optional new JSON file (never overwritten)')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--nav-cmd-topic', default='/j100_0519/nav2_cmd_vel')
    parser.add_argument('--platform-cmd-topic', default='/j100_0519/cmd_vel')
    parser.add_argument('--bridge-node', default='/cmd_vel_safety_bridge')
    args = parser.parse_args(argv)
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.settle)):
        parser.error('--timeout and --settle must be finite positive seconds')
    if args.settle >= args.timeout:
        parser.error('--settle must be less than --timeout')
    if args.output and args.output.exists():
        parser.error('--output already exists')
    node = None
    rclpy.init(args=[])
    try:
        node = NavigationReadiness(args)
        started, ready_since = time.monotonic(), None
        report, code = {}, 1
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            report = node.report()
            if report['ready']:
                if ready_since is None:
                    ready_since = time.monotonic()
                if time.monotonic() - ready_since >= args.settle:
                    code = 0
                    break
            else:
                ready_since = None
        report.update(ready=code == 0, settled=code == 0,
                      requested_settle_sec=args.settle,
                      elapsed_sec=time.monotonic() - started)
        output = json.dumps(report, indent=2, allow_nan=False) + '\n'
        print(output, end='')
        if args.output:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(output)
        return code
    except KeyboardInterrupt:
        return 130
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
