"""Bounded read-only observation of commands, pedestrian markers and costmaps."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from moai_nav_msgs.msg import Tracks
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray


class Observer(Node):
    def __init__(self):
        super().__init__('nav2_stationary_scene_observer')
        self.started = time.monotonic()
        self.stats = {}
        self.subs = []
        self.previous_grids = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, message_type, function in [
            ('/j100_0519/nav2_cmd_vel', TwistStamped, self.command),
            ('/ped_tracking', Tracks, self.tracks),
            ('/nav2/pedestrian_figures', MarkerArray, self.markers),
            ('/nav2/pedestrian_traces', MarkerArray, self.markers),
            ('/nav2/safety_diagnostics', DiagnosticArray, self.diagnostics),
            ('/nav2/lidar_relay_diagnostics', DiagnosticArray, self.diagnostics),
            ('/nav2/localization_diagnostics', DiagnosticArray, self.diagnostics),
        ]:
            self.subs.append(self.create_subscription(
                message_type, topic, lambda msg, t=topic, f=function: f(t, msg), qos))
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ['/map', '/static_costmap/costmap', '/global_costmap/costmap',
                      '/local_costmap/costmap']:
            self.subs.append(self.create_subscription(
                OccupancyGrid, topic, lambda msg, t=topic: self.grid(t, msg), map_qos))

    def receive(self, topic):
        now = time.monotonic()
        s = self.stats.setdefault(topic, {'messages': 0, 'max_gap_sec': 0})
        if 'last_received' in s:
            s['max_gap_sec'] = max(s['max_gap_sec'], now - s['last_received'])
        s['last_received'] = now
        s['messages'] += 1
        return s

    def command(self, topic, msg):
        s = self.receive(topic)
        v = [getattr(part, axis) for part in (msg.twist.linear, msg.twist.angular)
             for axis in 'xyz']
        s['nonfinite'] = s.get('nonfinite', 0) + int(not all(map(math.isfinite, v)))
        s['nonzero'] = s.get('nonzero', 0) + int(any(abs(x) > 1e-9 for x in v))

    def tracks(self, topic, msg):
        s = self.receive(topic)
        s.setdefault('frames', Counter())[msg.header.frame_id] += 1
        s['nonempty'] = s.get('nonempty', 0) + int(bool(msg.tracks))
        s['max_people'] = max(s.get('max_people', 0), sum(t.object_type == 0 for t in msg.tracks))
        s['latest_ids'] = [t.id for t in msg.tracks][:30]

    def markers(self, topic, msg):
        s = self.receive(topic)
        s['nonempty'] = s.get('nonempty', 0) + int(bool(msg.markers))
        adds = [m for m in msg.markers if m.action == 0]
        s['add_markers'] = s.get('add_markers', 0) + len(adds)
        s['delete_markers'] = s.get('delete_markers', 0) + sum(m.action in (2, 3) for m in msg.markers)
        s['duplicate_keys'] = s.get('duplicate_keys', 0) + int(
            len({(m.ns, m.id) for m in adds}) != len(adds))
        for m in adds:
            s.setdefault('frames', Counter())[m.header.frame_id] += 1
            s.setdefault('types', Counter())[str(m.type)] += 1
            if m.type == 2:
                s['sphere_top_z'] = m.pose.position.z + m.scale.z / 2
                s['sphere_diameter'] = m.scale.z
            if m.type == 4:
                s['max_trace_points'] = max(s.get('max_trace_points', 0), len(m.points))
        s['latest_add_count'] = len(adds)

    def diagnostics(self, topic, msg):
        s = self.receive(topic)
        for d in msg.status:
            s.setdefault('levels', Counter())[str(d.level)] += 1
            s.setdefault('reasons', Counter())[d.message] += 1
            s['latest_values'] = {v.key: v.value for v in d.values}

    def grid(self, topic, msg):
        s = self.receive(topic)
        digest = hashlib.sha256(bytes(msg.data)).hexdigest()
        origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        old = self.previous_grids.get(topic)
        if old:
            s['cell_changes'] = s.get('cell_changes', 0) + int(old[0] != digest)
            s['origin_changes'] = s.get('origin_changes', 0) + int(old[1] != origin)
        self.previous_grids[topic] = (digest, origin)
        s['latest_digest'] = digest
        s['shape'] = [msg.info.width, msg.info.height]
        s['latest_origin'] = origin

    def report(self):
        now = time.monotonic()
        result = {'duration_sec': now - self.started, 'topics': self.stats,
                  'read_only': True, 'motion_commands_published': False}
        for s in self.stats.values():
            s['receipt_silence_sec'] = now - s['last_received']
        return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=60)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 900:
        p.error('duration must be in [1, 900] seconds')
    with args.output.open('x') as stream:
        rclpy.init()
        node = Observer()
        try:
            until = time.monotonic() + args.duration
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            json.dump(node.report(), stream, indent=2, allow_nan=False)
            stream.write('\n')
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
