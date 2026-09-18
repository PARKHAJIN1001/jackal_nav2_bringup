#!/usr/bin/env python3
"""Require fresh, valid messages before advancing a staged launch."""

from collections import deque
import math
import sys
import time

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


MESSAGE_TYPES = {'cloud': PointCloud2, 'imu': Imu, 'odom': Odometry, 'scan': LaserScan}


def message_error(message, frame='', child_frame=''):
    """Validate the measurement contract without rewriting any sensor data."""
    header = message.header
    if not header.frame_id or (frame and header.frame_id != frame):
        return 'unexpected or empty frame_id'
    if not 0 <= header.stamp.nanosec < 1000000000:
        return 'invalid stamp'
    if isinstance(message, Odometry):
        if not message.child_frame_id or (child_frame and message.child_frame_id != child_frame):
            return 'unexpected or empty child_frame_id'
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return 'nonfinite odometry'
        if abs(sum(v * v for v in values[3:]) - 1.0) > 0.01:
            return 'invalid odometry quaternion'
    elif isinstance(message, PointCloud2):
        if (message.width * message.height == 0 or message.point_step == 0 or
                message.row_step < message.width * message.point_step or
                len(message.data) < message.row_step * message.height):
            return 'empty or malformed cloud'
        if not {'x', 'y', 'z'} <= {field.name for field in message.fields}:
            return 'cloud has no xyz fields'
    elif isinstance(message, Imu):
        a, w = message.linear_acceleration, message.angular_velocity
        if not all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)):
            return 'nonfinite IMU'
        if a.x * a.x + a.y * a.y + a.z * a.z < 1e-12:
            return 'zero IMU acceleration'
    elif isinstance(message, LaserScan):
        if (not math.isfinite(message.angle_increment) or message.angle_increment <= 0 or
                not any(math.isfinite(v) and message.range_min <= v <= message.range_max
                        for v in message.ranges)):
            return 'scan has no usable ranges'
    return ''


class FreshWindow:
    """A bounded state machine; publisher discovery is never readiness evidence."""

    def __init__(self, settle, max_age, max_gap, future_tolerance, min_messages):
        for name, value in [('settle', settle), ('max_age', max_age), ('max_gap', max_gap)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        if min_messages < 2:
            raise ValueError('min_messages must be at least 2')
        self.settle, self.max_age, self.max_gap = settle, max_age, max_gap
        self.future_tolerance, self.min_messages = future_tolerance, min_messages
        self.reset('no messages received')

    def reset(self, reason):
        self.first = self.last = self.stamp = None
        self.count = 0
        self.reason = reason

    def observe(self, stamp, ros_now, steady_now, error=''):
        if error:
            self.reset(error)
            return
        if not math.isfinite(stamp) or stamp <= 0:
            self.reset('invalid measurement time')
            return
        age = ros_now - stamp
        if not -self.future_tolerance <= age <= self.max_age:
            self.reset('stale or future measurement')
            return
        if self.last is not None and (
                steady_now - self.last > self.max_gap or
                not 0 < stamp - self.stamp <= self.max_gap):
            self.reset('measurement gap or timestamp regression')
        if self.first is None:
            self.first = steady_now
        self.last, self.stamp = steady_now, stamp
        self.count += 1
        self.reason = 'collecting continuous measurements'

    def ready(self, ros_now, steady_now):
        if self.last is None:
            return False
        if (steady_now - self.last > self.max_gap or
                not -self.future_tolerance <= ros_now - self.stamp <= self.max_age):
            self.reset('input stopped or became stale')
            return False
        return self.count >= self.min_messages and self.last - self.first >= self.settle


class ReadinessGate(Node):
    """Check sensor continuity and optionally AMCL lifecycle/map/scan-time TF."""

    def __init__(self):
        super().__init__('readiness_gate')
        defaults = {
            'topic': '', 'message_type': 'cloud', 'timeout': 45.0, 'settle': 3.0,
            'max_age': 0.3, 'max_gap': 0.3, 'future_tolerance': 0.05, 'min_messages': 5,
            'expected_frame': '', 'expected_child_frame': '', 'imu_topic': '',
            'odom_topic': '', 'require_localization': False, 'odom_frame': 'odom',
            'require_map_to_odom': False,
            'max_position_norm': 0.0, 'max_linear_speed': 2.0, 'max_angular_speed': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        p = self.settings
        if not p['topic'] or p['message_type'] not in MESSAGE_TYPES:
            raise ValueError('topic and a supported message_type are required')
        for name in ('timeout', 'max_linear_speed', 'max_angular_speed'):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(p['max_position_norm']) or p['max_position_norm'] < 0:
            raise ValueError('max_position_norm must be finite and nonnegative')
        if p['require_localization'] and p['message_type'] != 'scan':
            raise ValueError('require_localization requires a scan input')
        if p['require_map_to_odom'] and not p['require_localization']:
            raise ValueError('require_map_to_odom requires require_localization')
        self.started = time.monotonic()
        self.last_log = self.started
        self.windows, self.subscriptions_owned, self.previous_poses = {}, [], {}
        self.scan_headers = deque(maxlen=10)
        self.map_received = False
        self.lifecycle = {}
        self.tf_buffer = None
        if p['require_localization']:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.subscriptions_owned.append(self.create_subscription(
                OccupancyGrid, '/map', self._on_map, map_qos))
            for name in ('/map_server', '/amcl'):
                self.lifecycle[name] = {
                    'client': self.create_client(GetState, name + '/get_state'),
                    'future': None, 'sent': 0.0, 'active_at': None,
                }
        self._subscribe(p['topic'], p['message_type'], p['expected_frame'],
                        p['expected_child_frame'])
        if p['imu_topic']:
            self._subscribe(p['imu_topic'], 'imu')
        if p['odom_topic']:
            self._subscribe(p['odom_topic'], 'odom', p['odom_frame'], 'base_link')
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._tick, clock=self.timer_clock)
        self.get_logger().info('Waiting for continuous measurements: ' + ', '.join(self.windows))

    def _subscribe(self, topic, kind, frame='', child=''):
        if topic in self.windows:
            raise ValueError('gate inputs must use distinct topics')
        p = self.settings
        self.windows[topic] = FreshWindow(
            p['settle'], p['max_age'], p['max_gap'], p['future_tolerance'], p['min_messages'])
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned.append(self.create_subscription(
            MESSAGE_TYPES[kind], topic,
            lambda msg: self._receive(topic, msg, frame, child), qos))

    def _receive(self, topic, msg, frame, child):
        now, mono = self.get_clock().now().nanoseconds / 1e9, time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        error = message_error(msg, frame, child)
        if isinstance(msg, Odometry) and not error:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            position, quaternion = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
            norm = math.sqrt(sum(v * v for v in quaternion))
            quaternion = tuple(v / norm for v in quaternion)
            limit = self.settings['max_position_norm']
            if limit and math.dist(position, (0, 0, 0)) > limit:
                error = 'odometry outside startup origin bound; restart FAST-LIVO2'
            previous = self.previous_poses.get(topic)
            if previous and 0 < stamp - previous[0] <= self.settings['max_gap']:
                dt = stamp - previous[0]
                dot = min(1.0, abs(sum(a * b for a, b in zip(quaternion, previous[2]))))
                if (math.dist(position, previous[1]) / dt > self.settings['max_linear_speed'] or
                        2 * math.acos(dot) / dt > self.settings['max_angular_speed']):
                    error = 'odometry discontinuity'
            self.previous_poses[topic] = (stamp, position, quaternion)
        self.windows[topic].observe(stamp, now, mono, error)
        if isinstance(msg, LaserScan):
            if error:
                self.scan_headers.clear()
            else:
                self.scan_headers.append(msg.header)

    def _on_map(self, msg):
        self.map_received = bool(
            msg.header.frame_id == 'map' and msg.info.width > 0 and msg.info.height > 0 and
            msg.info.resolution > 0 and len(msg.data) == msg.info.width * msg.info.height)

    def _localization_ready(self, mono):
        if self.tf_buffer is None:
            return True
        for state in self.lifecycle.values():
            future = state['future']
            if future is not None and future.done():
                try:
                    response = future.result() if not future.cancelled() else None
                except Exception as error:
                    self.get_logger().warning(f'Lifecycle query failed: {error}')
                    response = None
                state['active_at'] = (
                    mono if response is not None and response.current_state.id == 3 else None)
                state['future'] = None
            elif future is not None and mono - state['sent'] > 2.0:
                future.cancel()
                state['future'], state['active_at'] = None, None
            if (state['future'] is None and mono - state['sent'] >= 1.0 and
                    state['client'].service_is_ready()):
                state['future'] = state['client'].call_async(GetState.Request())
                state['sent'] = mono
        active = all(s['active_at'] is not None and mono - s['active_at'] <= 2.0
                     for s in self.lifecycle.values())
        if not active or not self.map_received:
            return False
        # Initial startup must not require map->odom before initialpose.
        # The separate perception-session gate opts in after operator alignment.
        # Scan projection can finish before FAST-LIVO2 publishes the matching TF.
        # Check recent scan timestamps, still bounded by the same freshness limit.
        now = self.get_clock().now().nanoseconds / 1e9
        for header in reversed(self.scan_headers):
            stamp = header.stamp.sec + header.stamp.nanosec / 1e9
            if (-self.settings['future_tolerance'] <= now - stamp <= self.settings['max_age'] and
                    self.tf_buffer.can_transform(self.settings['odom_frame'], header.frame_id,
                                                 Time.from_msg(header.stamp)) and
                    (not self.settings.get('require_map_to_odom', False) or
                     self.tf_buffer.can_transform('map', self.settings['odom_frame'],
                                                  Time.from_msg(header.stamp)))):
                return True
        return False

    def _tick(self):
        mono, now = time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        localized = self._localization_ready(mono)
        ready = [window.ready(now, mono) for window in self.windows.values()]
        if all(ready) and localized:
            self.get_logger().info('Fresh measurement readiness checks passed')
            raise SystemExit(0)
        reasons = ', '.join(f'{topic}: {w.reason}' for topic, w in self.windows.items())
        if not localized:
            reasons += '; waiting for active AMCL/map_server, map and scan-time odom TF'
        if mono - self.started >= self.settings['timeout']:
            self.get_logger().error('Readiness timed out: ' + reasons)
            raise SystemExit(1)
        if mono - self.last_log >= 5.0:
            self.get_logger().info(reasons)
            self.last_log = mono


def main(args=None):
    """Exit nonzero on cancellation so shutdown cannot advance a launch phase."""
    rclpy.init(args=args)
    node, exit_code = None, 0
    try:
        node = ReadinessGate()
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = exc.code
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    except (ValueError, TypeError) as error:
        print(f'[readiness_gate] {error}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
