#!/usr/bin/env python3
"""Audit TF ownership, timing, and localization inputs without publishing."""

import argparse
from collections import deque
import functools
import json
import math
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from moai_nav_msgs.msg import Detections, Tracks
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def input_topics(require_perception=False):
    """Do not start remote image traffic during localization-only checks."""
    topics = [
        ('/scan', LaserScan),
        ('/odom', Odometry),
        ('/aft_mapped_to_init', Odometry),
        ('/amcl_pose', PoseWithCovarianceStamped),
        ('/livox/lidar_local', PointCloud2),
    ]
    if require_perception:
        topics.extend([
            ('/lidar/accumulated', PointCloud2),
            ('/camera/camera/color/image_raw', Image),
            ('/ped_detection', Detections),
            ('/ped_tracking', Tracks),
        ])
    return topics


def assess_edges(edges, max_age=0.5, map_future_tolerance=1.1):
    """Check root TF ownership; ignore independent namespaced TF topics."""
    issues = []
    roots = [edge for edge in edges if edge['topic'] in ('/tf', '/tf_static')]
    for parent, child, owner in (
        ('map', 'odom', '/amcl'),
        ('odom', 'base_link', '/laserMapping'),
    ):
        matches = [
            e for e in roots if e['parent'] == parent and e['child'] == child
        ]
        label = f'{parent} -> {child}'
        if not matches:
            issues.append(f'missing {label}')
            continue
        if any(e['topic'] == '/tf_static' for e in matches):
            issues.append(f'{label} must be dynamic')
        writers = {writer for e in matches for writer in e['writers']}
        if len(writers) != 1:
            issues.append(f'{label} has {len(writers)} DDS writers')
        owners = {name for e in matches for name in e['owners']}
        if owners != {owner}:
            issues.append(
                f'{label} expected {owner}; observed {sorted(owners)}'
            )
        future = map_future_tolerance if parent == 'map' else 0.1
        for edge in matches:
            if (
                edge['age_min_sec'] < -future
                or edge['age_max_sec'] > max_age
                or edge.get('latest_age_sec', 0) > max_age
            ):
                issues.append(f'{label} timestamp outside allowed age window')
    for child in {e['child'] for e in roots}:
        incoming = [e for e in roots if e['child'] == child]
        if len({e['parent'] for e in incoming}) > 1:
            issues.append(f'{child} has multiple parents on root TF topics')
        if len({w for e in incoming for w in e['writers']}) > 1:
            issues.append(f'{child} has multiple root TF writers')
        if any(
            e['invalid_quaternion'] or e['stamp_regressions'] for e in incoming
        ):
            issues.append(
                f'{child} has invalid quaternion or backwards timestamp'
            )
    parent_of = {e['child']: e['parent'] for e in roots}
    for child in parent_of:
        seen = set()
        current = child
        while current in parent_of:
            if current in seen:
                issues.append(f'cycle in root TF tree containing {current}')
                break
            seen.add(current)
            current = parent_of[current]
    return sorted(set(issues))


def stamp_seconds(stamp):
    """Convert a ROS timestamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def assess_planar_height(latest_tf, max_height=0.2):
    """Check this flat-floor setup for a stale vertical map/odom offset."""
    pose = latest_tf.get('map->base_link')
    if isinstance(pose, dict):
        height = pose['transform']['translation']['z']
        if not math.isfinite(height) or abs(height) > max_height:
            return [
                'map -> base_link height exceeds flat-floor tolerance; '
                'reinitialize AMCL after odometry restart and check mounts'
            ]
    return []


class ExactTimeChecks:
    """Count every received scan, with a bounded .5 s TF arrival grace period."""

    def __init__(self):
        self.pending = deque()
        self.phases = {}
        self.failures = []

    def add(self, scan, received, phase):
        self.pending.append(
            [scan.header, received, phase, {'odom', 'map'}]
        )

    def process(self, buffer, now):
        remaining = deque()
        for header, received, phase, targets in self.pending:
            for target in list(targets):
                success = (
                    stamp_seconds(header.stamp) > 0
                    and bool(header.frame_id)
                    and buffer.can_transform(
                        target, header.frame_id, Time.from_msg(header.stamp)
                    )
                )
                if not success and now - received < 0.5:
                    continue
                counts = self.phases.setdefault(phase, {}).setdefault(
                    target, {'checked_scans': 0, 'transformable_scans': 0}
                )
                counts['checked_scans'] += 1
                counts['transformable_scans'] += int(success)
                targets.remove(target)
                if not success and len(self.failures) < 64:
                    self.failures.append(
                        {'target': target, 'frame': header.frame_id,
                         'stamp_sec': stamp_seconds(header.stamp), 'phase': phase}
                    )
            if targets:
                remaining.append([header, received, phase, targets])
        self.pending = remaining

    def totals(self):
        return {
            target: {
                key: sum(
                    phase.get(target, {}).get(key, 0)
                    for phase in self.phases.values()
                )
                for key in ('checked_scans', 'transformable_scans')
            }
            for target in ('odom', 'map')
        }


class Audit(Node):
    """Collect a bounded window of inputs and exact-time TF lookups."""

    def __init__(self, warmup=3.0, require_perception=False):
        """Create read-only subscriptions."""
        super().__init__('tf_localization_audit')
        self.samples = {}
        self.scans = deque(maxlen=90)
        self.map = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.start = time.monotonic()
        self.warmup = warmup
        self.require_perception = require_perception
        self.restart_until = self.start
        self.phase_events = []
        self.last_ros = None
        self.accept_samples = True
        self.exact_checks = ExactTimeChecks()
        self.check_timer = self.create_timer(
            0.02,
            lambda: self.exact_checks.process(self.buffer, time.monotonic()),
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        for topic, msg_type in input_topics(require_perception):
            self.create_subscription(
                msg_type,
                topic,
                functools.partial(self.on_sample, topic),
                (
                    QoSProfile(
                        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                    )
                    if topic == '/amcl_pose'
                    else qos
                ),
            )

    def on_map(self, msg):
        """Keep the latest static map."""
        self.map = msg

    def on_sample(self, topic, msg):
        """Record timing and a bounded scan history."""
        if not self.accept_samples:
            return
        sample = self.samples.setdefault(topic, {'count': 0})
        received = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = stamp_seconds(msg.header.stamp)
        previous_stamp = sample.get('last_stamp_sec')
        interval = received - sample.get('last_receipt_monotonic', received)
        backwards_clock = self.last_ros is not None and now < self.last_ros
        restart_candidate = (
            topic == '/aft_mapped_to_init'
            and previous_stamp is not None
            and (previous_stamp - stamp > 0.5 or interval > 2.0)
        )
        if backwards_clock or restart_candidate:
            self.restart_until = received + self.warmup
            if len(self.phase_events) < 64:
                self.phase_events.append(
                    {
                        'receive_ros_sec': now,
                        'receive_elapsed_sec': received - self.start,
                        'reason': (
                            'clock_regression'
                            if backwards_clock
                            else 'inferred_restart_candidate'
                        ),
                    }
                )
        self.last_ros = now
        phase = (
            'initialization'
            if received - self.start < self.warmup
            else (
                'restart_candidate'
                if received < self.restart_until
                else 'steady'
            )
        )
        stats = sample.setdefault('phases', {}).setdefault(
            phase,
            {
                'count': 0,
                'stamp_regressions': 0,
                'age_min_sec': now - stamp,
                'age_max_sec': now - stamp,
                'interarrival_max_sec': 0.0,
            },
        )
        stats['count'] += 1
        stats['age_min_sec'] = min(stats['age_min_sec'], now - stamp)
        stats['age_max_sec'] = max(stats['age_max_sec'], now - stamp)
        stats['interarrival_max_sec'] = max(
            stats['interarrival_max_sec'], interval
        )
        if previous_stamp is not None and stamp < previous_stamp:
            stats['stamp_regressions'] += 1
            events = sample.setdefault('regression_events', [])
            if len(events) < 64:
                events.append(
                    {'previous_stamp_sec': previous_stamp,
                     'current_stamp_sec': stamp,
                     'regression_sec': previous_stamp - stamp,
                     'receive_ros_sec': now,
                     'receive_elapsed_sec': received - self.start,
                     'phase': phase}
                )
        sample['count'] += 1
        sample['frame'] = msg.header.frame_id
        sample['last_receipt_monotonic'] = time.monotonic()
        sample['last_stamp_sec'] = stamp_seconds(msg.header.stamp)
        sample[
            'last_age_sec'
        ] = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(
            msg.header.stamp
        )
        if topic == '/scan':
            ranges = [r for r in msg.ranges if math.isfinite(r)]
            sample['finite_ranges'] = len(ranges)
            sample['total_ranges'] = len(msg.ranges)
            sample['scan_time'] = msg.scan_time
            self.scans.append(msg)
            self.exact_checks.add(msg, received, phase)
        elif topic in ('/odom', '/aft_mapped_to_init', '/amcl_pose'):
            sample['latest'] = message_to_ordereddict(msg)

    def report(self, edges):
        """Summarize observations after the measurement window."""
        checks = self.exact_checks.totals()
        start_index = len(self.scans) // 2
        scans = list(self.scans)[start_index:-5]
        # The bounded recent scans below are used ONLY for endpoint alignment.
        latest_tf = {}
        for parent, child in (
            ('map', 'base_link'),
            ('odom', 'base_link'),
            ('base_link', 'livox_frame'),
        ):
            try:
                latest_tf[f'{parent}->{child}'] = message_to_ordereddict(
                    self.buffer.lookup_transform(parent, child, Time())
                )
            except TransformException as exc:
                latest_tf[f'{parent}->{child}'] = str(exc)
        issues = assess_edges(edges) + assess_planar_height(latest_tf)
        now = self.measurement_end_ros
        for topic, sample in self.samples.items():
            sample['latest_age_sec'] = now - sample['last_stamp_sec']
            sample['receipt_silence_sec'] = (
                self.measurement_end_monotonic
                - sample.pop('last_receipt_monotonic')
            )
            if (
                topic
                in (
                    '/scan',
                    '/odom',
                    '/aft_mapped_to_init',
                    '/livox/lidar_local',
                )
                and sample['receipt_silence_sec'] > 0.5
            ):
                issues.append(f'stale {topic} stream')
        for topic in ('/scan', '/odom', '/aft_mapped_to_init', '/amcl_pose'):
            if topic not in self.samples:
                issues.append(f'missing {topic} sample')
        required = (
            (
                '/lidar/accumulated',
                '/camera/camera/color/image_raw',
                '/ped_detection',
                '/ped_tracking',
            )
            if self.require_perception
            else ()
        )
        for topic in required:
            if (
                topic not in self.samples
                or self.samples[topic]['receipt_silence_sec'] > 1.0
            ):
                issues.append(
                    f'missing or stale full-perception input {topic}'
                )
        steady_checks = self.exact_checks.phases.get('steady', {})
        for target in ('odom', 'map'):
            check = steady_checks.get(
                target, {'checked_scans': 0, 'transformable_scans': 0}
            )
            if (
                not check['checked_scans']
                or check['transformable_scans'] / check['checked_scans'] < 0.99
            ):
                issues.append(
                    f'steady scan exact-time TF success below 99% to {target}'
                )
        if not self.buffer.can_transform('base_link', 'livox_frame', Time()):
            issues.append('missing base_link -> livox_frame sensor transform')
        return {
            'duration_sec': self.measurement_end_monotonic - self.start,
            'issues': issues,
            'edges': edges,
            'samples': self.samples,
            'scan_tf_checks': checks,
            'scan_tf_checks_by_phase': self.exact_checks.phases,
            'scan_tf_failure_examples': self.exact_checks.failures,
            'phase_events': self.phase_events,
            'phase_policy': ('warmup then steady; >.5 s odom regression or >2 s '
                             'gap is only an inferred restart'),
            'warmup_sec': self.warmup,
            'require_perception': self.require_perception,
            'alignment_window': ('recent bounded scans only; TF success counts '
                                 'cover entire interval'),
            'latest_tf': latest_tf,
            'scan_map_alignment': self.score_scans(scans),
        }

    def score_scans(self, scans):
        """Measure endpoint-to-wall consistency, NOT ground-truth pose error."""
        if self.map is None:
            return {'unavailable': 'no /map received'}
        grid = np.asarray(self.map.data).reshape(
            self.map.info.height, self.map.info.width
        )
        if not np.any(grid >= 65):
            return {'unavailable': 'no occupied cells'}
        distance = distance_transform_edt(grid < 65) * self.map.info.resolution
        origin = self.map.info.origin
        q = origin.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        distances = []
        total = 0
        unknown = 0
        for scan in scans:
            try:
                tf = self.buffer.lookup_transform(
                    self.map.header.frame_id,
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                ).transform
            except TransformException:
                continue
            ranges = np.asarray(scan.ranges)
            mask = (
                np.isfinite(ranges)
                & (ranges >= scan.range_min)
                & (ranges < scan.range_max)
            )
            angles = (
                scan.angle_min
                + np.arange(len(ranges))[mask] * scan.angle_increment
            )
            x, y = ranges[mask] * np.cos(angles), ranges[mask] * np.sin(angles)
            q = tf.rotation
            mx = (
                (1 - 2 * (q.y * q.y + q.z * q.z)) * x
                + 2 * (q.x * q.y - q.w * q.z) * y
                + tf.translation.x
                - origin.position.x
            )
            my = (
                2 * (q.x * q.y + q.w * q.z) * x
                + (1 - 2 * (q.x * q.x + q.z * q.z)) * y
                + tf.translation.y
                - origin.position.y
            )
            ix = np.floor(
                (math.cos(yaw) * mx + math.sin(yaw) * my)
                / self.map.info.resolution
            ).astype(int)
            iy = np.floor(
                (-math.sin(yaw) * mx + math.cos(yaw) * my)
                / self.map.info.resolution
            ).astype(int)
            valid = (
                (ix >= 0)
                & (ix < grid.shape[1])
                & (iy >= 0)
                & (iy < grid.shape[0])
            )
            total += len(ix)
            unknown += int(np.sum(grid[iy[valid], ix[valid]] < 0))
            distances.extend(distance[iy[valid], ix[valid]].tolist())
        if not distances:
            return {'unavailable': 'no transformable endpoints inside map'}
        return {
            'meaning': (
                'scan endpoint distance to nearest occupied map cell; '
                'NOT pose accuracy'
            ),
            'endpoints': total,
            'inside_map': len(distances),
            'unknown_cell_endpoints': unknown,
            'median_m': float(np.median(distances)),
            'p90_m': float(np.percentile(distances, 90)),
            'within_0_15m_fraction': float(
                np.sum(np.asarray(distances) <= 0.15 + 1e-6) / total
            ),
        }


def main():
    """Run the bounded audit and return nonzero for contract violations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=12.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--warmup', type=float, default=3.0)
    parser.add_argument('--require-perception', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration < 3:
        parser.error('--duration must be finite and at least 3 seconds')
    rclpy.init(args=ros_args)
    if not math.isfinite(args.warmup) or not 0 <= args.warmup < args.duration:
        parser.error(
            '--warmup must be finite, nonnegative and shorter than duration'
        )
    node = Audit(args.warmup, args.require_perception)
    executable = (
        Path(get_package_prefix('jackal_nav2_bringup'))
        / 'lib'
        / 'jackal_nav2_bringup'
        / 'tf_authority_probe'
    )
    probe = None
    try:
        probe = subprocess.Popen(
            [
                str(executable),
                *ros_args,
                '--ros-args',
                '-p',
                f'duration:={args.duration}',
                '-p',
                f'warmup:={args.warmup}',
            ],
            stdout=subprocess.PIPE,
            text=True,
        )
        until = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.accept_samples = False
        node.measurement_end_monotonic = time.monotonic()
        node.measurement_end_ros = node.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok() and node.exact_checks.pending:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.exact_checks.process(node.buffer, time.monotonic())
        probe_output, _ = probe.communicate(timeout=5)
        if probe.returncode:
            raise RuntimeError(
                f'TF authority probe failed: {probe.returncode}'
            )
        json_start = probe_output.find('[')
        json_end = probe_output.rfind(']')
        if json_start != -1 and json_end != -1 and json_end > json_start:
            probe_data = json.loads(probe_output[json_start:json_end + 1])
        else:
            probe_data = json.loads(probe_output)
        report = node.report(probe_data)
        rendered = json.dumps(report, indent=2, allow_nan=False)
        if args.output:
            args.output.write_text(rendered + '\n', encoding='utf-8')
        print(rendered)
        return 1 if report['issues'] else 0
    finally:
        if probe is not None and probe.poll() is None:
            probe.terminate()
            probe.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
