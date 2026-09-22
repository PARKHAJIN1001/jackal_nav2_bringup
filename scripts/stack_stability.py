#!/usr/bin/env python3
"""Continuously supervise full-stack inputs; publish a fail-closed readiness heartbeat."""

from collections import Counter
import json
import math
import os
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from readiness_core import fresh, StabilityWindow, valid_transform
from std_msgs.msg import Bool
from tf2_ros import TransformException
from topic_ready_gate import ReadinessGate


LIFECYCLE_NODES = (
    '/map_server', '/amcl', '/planner_server', '/controller_server',
    '/bt_navigator', '/velocity_smoother', '/collision_monitor',
    '/local_costmap/local_costmap', '/global_costmap/global_costmap',
    '/static_costmap/static_costmap', '/smoother_server', '/behavior_server',
    '/waypoint_follower',
)
PERCEPTION_NODES = {
    '/ped_yolo_node', '/mid360_mask_3d_extractor_node',
    '/mid360_lidar_accumulator_node', '/pedestrian_tracker_node',
    '/pedestrian_figures', '/pedestrian_traces',
}


class StackStability(ReadinessGate):
    extra_defaults = {
        'stability_timeout': 600.0, 'stability_settle': 180.0,
        'cloud_topic': '/livox/lidar_local', 'fast_odom_topic': '/aft_mapped_to_init',
        'nav_cmd_topic': '/j100_0519/nav2_cmd_vel',
    }

    def __init__(self):
        super().__init__()
        p = self.settings
        if not p['require_localization'] or not p['imu_topic'] or not p['odom_topic']:
            raise ValueError('stack monitor requires localization, IMU and odometry inputs')
        self.required_lifecycle = {'/map_server', '/amcl'}
        self.lifecycle_signature = None
        self.window = StabilityWindow(p['stability_timeout'], p['stability_settle'])
        self._subscribe(p['cloud_topic'], 'cloud')
        self._subscribe(p['fast_odom_topic'], 'odom', 'odom', 'base_link')
        for name in LIFECYCLE_NODES:
            if name not in self.lifecycle:
                self.lifecycle[name] = {
                    'client': self.create_client(GetState, name + '/get_state'),
                    'future': None, 'sent': 0.0, 'active_at': None,
                }
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.safety = None
        self.create_subscription(DiagnosticArray, '/nav2/safety_diagnostics', self.on_safety, 10)
        # Volatile: a new guard must never inherit a latched "ready" from a dead monitor.
        self.output = self.create_publisher(Bool, '/nav2/stack_ready', 1)
        self.diagnostics = self.create_publisher(
            DiagnosticArray, '/nav2/stability_diagnostics', 10)
        self.output.publish(Bool(data=False))
        self.reset_counts = None
        self.graph_at, self.graph_ok = -math.inf, False
        self.graph_changed = False
        self.perception_graph = None
        self.last_phase = None
        self.last_diagnostic = -math.inf
        directory = os.environ.get('JACKAL_NAV_SESSION_DIR')
        self.evidence = Path(directory) if directory else None

    def on_safety(self, message):
        for status in message.status:
            if status.name == 'nav2_safety_guard':
                self.safety = (message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                               time.monotonic(), {v.key: v.value for v in status.values})

    def guard_inputs_ready(self, now, mono):
        # Final guard reason includes operator stop and missing initial pose. Neither
        # is a transport failure. Inspect the independent raw-input evidence instead.
        if not self.safety or not fresh(*self.safety[:2], now, mono, timeout=2.5):
            return False
        values = self.safety[2]
        try:
            return (not values['sensor_error'] and all(
                -float(values['future_tolerance']) <= float(values[key]) <=
                float(values['sensor_timeout']) for key in
                ('sensor_stamp_age', 'sensor_receipt_age', 'raw_sensor_stamp_age',
                 'raw_sensor_receipt_age')))
        except (KeyError, ValueError, TypeError):
            return False

    def inspect_graph(self, mono):
        self.graph_changed = False
        if mono - self.graph_at < 1.0:
            return self.graph_ok
        self.graph_at = mono
        counts = Counter(ns.rstrip('/') + '/' + name
                         for name, ns in self.get_node_names_and_namespaces())
        required = (*LIFECYCLE_NODES, '/nav2_safety_guard', '/nav2_stack_stability',
                    '/fast_livo_odom_adapter')
        writers = sorted(info.node_namespace.rstrip('/') + '/' + info.node_name
                         for info in self.get_publishers_info_by_topic(
                             self.settings['nav_cmd_topic']))
        heartbeat_writers = self.get_publishers_info_by_topic('/nav2/stack_ready')
        self.graph_ok = (all(counts[name] == 1 for name in required)
                         and writers == ['/nav2_safety_guard']
                         and len(heartbeat_writers) == 1)
        graph = {name: counts[name] for name in PERCEPTION_NODES}
        self.graph_changed = self.perception_graph is not None and graph != self.perception_graph
        self.perception_graph = graph
        return self.graph_ok

    def transform_ready(self, parent, child, now, limit, allowance=0.0):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            values = self.safety[2] if self.safety else {}
            age = now - stamp + allowance
            return (valid_transform(tf.transform) and stamp > 0 and
                    -float(values['future_tolerance']) <= age <= float(values[limit]))
        except (TransformException, KeyError, TypeError, ValueError):
            return False

    def pose_ready(self, now):
        try:
            allowance = float(self.safety[2]['map_transform_tolerance'])
        except (TypeError, KeyError, ValueError):
            return False
        return self.transform_ready('map', self.settings['odom_frame'], now,
                                    'map_tf_timeout', allowance) and any(
            -self.settings['future_tolerance'] <=
            now - (h.stamp.sec + h.stamp.nanosec * 1e-9) <= self.settings['max_age'] and
            self.tf_buffer.can_transform('map', h.frame_id, Time.from_msg(h.stamp))
            for h in self.scan_headers)

    def _tick(self):
        mono, now = time.monotonic(), self.get_clock().now().nanoseconds * 1e-9
        localized = self._localization_ready(mono)
        active = tuple(name for name, state in self.lifecycle.items()
                       if state['active_at'] is not None and mono - state['active_at'] <= 5.0)
        lifecycle_changed = (self.lifecycle_signature is not None and
                             active != self.lifecycle_signature)
        self.lifecycle_signature = active
        pose = self.pose_ready(now)
        # Some Nav2 lifecycle transitions need initialpose. They cannot be required
        # to finish before asking for initialpose. Each transition restarts the hold.
        navigation_active = len(active) == len(self.lifecycle)
        inputs = [w.ready(now, mono) for w in self.windows.values()]
        resets = tuple(w.resets for w in self.windows.values())
        changed = self.reset_counts is not None and resets != self.reset_counts
        self.reset_counts = resets
        graph = self.inspect_graph(mono)
        guard = self.guard_inputs_ready(now, mono)
        healthy = (all(inputs) and localized and graph and guard
                   and self.transform_ready(self.settings['odom_frame'], 'base_link',
                                            now, 'tf_timeout')
                   and not changed and not self.graph_changed and not lifecycle_changed
                   and (not pose or (navigation_active and self.action.server_is_ready())))
        phase = self.window.update(healthy, pose and navigation_active, mono)
        self.output.publish(Bool(data=phase == 'READY'))
        if mono - self.last_diagnostic >= 0.2 or phase != self.last_phase:
            values = {
                'phase': phase, 'ready': phase == 'READY', 'inputs_healthy': healthy,
                'continuous_sec': 0.0 if self.window.since is None else mono - self.window.since,
                'settle_sec': self.window.settle, 'timeout_sec': self.window.timeout,
                'reset_count': self.window.resets, 'lifecycle_and_scan_tf': localized,
                'guard_inputs': guard, 'node_ownership': graph,
                'perception_changed': self.graph_changed,
                'navigation_lifecycle_active': navigation_active,
                'input_reasons': {topic: w.reason for topic, w in self.windows.items()},
                'monotonic_sec': mono, 'wall_sec': time.time(),
            }
            status = DiagnosticStatus(
                name='nav2_stack_stability', hardware_id='software_only',
                level=DiagnosticStatus.OK if phase == 'READY' else DiagnosticStatus.WARN,
                message=phase, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
            diagnostic = DiagnosticArray(status=[status])
            diagnostic.header.stamp = self.get_clock().now().to_msg()
            self.diagnostics.publish(diagnostic)
            if self.evidence:
                temporary = self.evidence / 'stability.json.tmp'
                temporary.write_text(json.dumps(values, indent=2) + '\n')
                temporary.replace(self.evidence / 'stability.json')
            if phase != self.last_phase:
                self.get_logger().info(
                    f'Stack state: {phase}; continuous={values["continuous_sec"]:.1f}s')
                if self.evidence:
                    with (self.evidence / 'stability_events.jsonl').open('a') as stream:
                        stream.write(json.dumps(values) + '\n')
            self.last_phase, self.last_diagnostic = phase, mono
        if phase == 'FAILED':
            self.get_logger().warn('Full-stack stability waiting for initial pose; continuing to monitor')
            self.window.started = mono
            self.phase = 'INPUT_WAITING'


def main():
    rclpy.init()
    node, code = None, 0
    try:
        node = StackStability()
        rclpy.spin(node)
    except SystemExit as error:
        code = error.code
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        if node:
            if rclpy.ok():
                node.output.publish(Bool(data=False))
            node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
