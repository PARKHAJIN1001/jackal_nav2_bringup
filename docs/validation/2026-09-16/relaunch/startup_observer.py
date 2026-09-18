"""Bounded read-only receipt/stamp and blocked-output evidence; no commands published."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu

p = argparse.ArgumentParser()
p.add_argument('--duration', type=float, default=90)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--raw-lidar', action='store_true')
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('nav2_relaunch_readonly_observer')
started = time.monotonic()
streams = {}
subscriptions = []
diagnostics = {}

def receive(topic, msg):
    x = streams.setdefault(topic, {'count': 0, 'regressions': 0, 'ages': [],
                                   'gaps': [], 'nonzero': 0, 'nonfinite': 0})
    now = time.monotonic()
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if 'last_receipt' in x:
        x['gaps'].append(now - x['last_receipt'])
        x['regressions'] += int(stamp < x['last_stamp'])
    else:
        x['first_receipt_after_start_sec'] = now - started
    x['last_receipt'], x['last_stamp'] = now, stamp
    x['count'] += 1
    x['frame'] = msg.header.frame_id
    x['ages'].append(time.time() - stamp)
    if isinstance(msg, TwistStamped):
        vals = [getattr(v, axis) for v in (msg.twist.linear, msg.twist.angular)
                for axis in ('x', 'y', 'z')]
        x['nonzero'] += int(any(abs(v) > 1e-8 for v in vals))
        x['nonfinite'] += int(not all(math.isfinite(v) for v in vals))

def diag(msg):
    for status in msg.status:
        x = diagnostics.setdefault(status.name, {'messages': {}, 'last_values': {}})
        x['messages'][status.message] = x['messages'].get(status.message, 0) + 1
        x['last_values'] = {v.key: v.value for v in status.values}

topics = [('/livox/lidar_local', PointCloud2), ('/livox/imu', Imu),
                    ('/aft_mapped_to_init', Odometry),
                    ('/j100_0519/nav2_cmd_vel', TwistStamped)]
if a.raw_lidar:
    topics.append(('/livox/lidar', PointCloud2))
for topic, kind in topics:
    subscriptions.append(n.create_subscription(kind, topic,
                         lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))
for topic in ['/nav2/lidar_relay_diagnostics', '/nav2/safety_diagnostics']:
    subscriptions.append(n.create_subscription(DiagnosticArray, topic, diag,
                                              qos_profile_sensor_data))
interrupted = False
try:
    while time.monotonic() - started < a.duration:
        rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    interrupted = True
finally:
    now = time.monotonic()
    for topic, x in streams.items():
        ages, gaps = sorted(x.pop('ages')), x.pop('gaps')
        x['age_min_median_max_sec'] = [ages[0], ages[len(ages)//2], ages[-1]]
        x['max_receipt_gap_sec'] = max(gaps, default=None)
        x['receipt_age_at_end_sec'] = now - x.pop('last_receipt')
        x['publishers_at_end'] = [i.node_namespace.rstrip('/') + '/' + i.node_name
                                  for i in n.get_publishers_info_by_topic(topic)]
    result = {'duration_sec': now-started, 'interrupted': interrupted,
              'topics': streams, 'diagnostics': diagnostics}
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    n.destroy_node()
    rclpy.shutdown()
