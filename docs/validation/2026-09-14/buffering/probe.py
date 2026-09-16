"""Read-only bounded ROS timing observer; no publishing or state changes."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import GetParameters

p = argparse.ArgumentParser()
p.add_argument('--output', required=True)
p.add_argument('--duration', type=float, default=25)
p.add_argument('--nuc', action='store_true')
a = p.parse_args()
rclpy.init()
n = Node('buffer_timing_probe_' + ('nuc' if a.nuc else 'laptop'))
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
streams, subscriptions, failures, diagnostics, params = {}, [], [], [], {}
def record(topic, stamp, size=0, frame=''):
    mono, wall = time.monotonic(), time.time()
    stream = streams.setdefault(topic, [])
    if len(stream) < 16000:
        stream.append([mono, wall, stamp, size, frame])
def callback(topic, data):
    try:
        endian = '<' if data[1] & 1 else '>'
        sec, ns, length = struct.unpack_from(endian + 'iII', data, 4)
        if not (0 <= ns < 1000000000) or not (1 <= length < 1024):
            raise ValueError('invalid CDR header')
        frame = data[16:16 + length - 1].decode('utf8')
        record(topic, sec + ns * 1e-9, len(data), frame)
    except Exception as exc:
        if len(failures) < 20:
            failures.append([topic, str(exc)])
def diag(msg):
    if len(diagnostics) < 300:
        diagnostics.append({'received': time.time(), 'status': [
            {'name': s.name, 'level': int.from_bytes(s.level, 'little')
             if isinstance(s.level, bytes) else int(s.level), 'message': s.message,
             'values': {v.key: v.value for v in s.values}} for s in msg.status]})
def tf(msg):
    for t in msg.transforms:
        if t.child_frame_id in ('base_link', 'odom'):
            record('/tf:' + t.header.frame_id + '->' + t.child_frame_id,
                   t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
topics = ['/livox/lidar', '/livox/imu'] if a.nuc else [
    '/livox/lidar_local', '/scan', '/aft_mapped_to_init', '/odom',
    '/lidar/accumulated', '/ped_detection', '/ped_tracking', '/ped_traces',
    '/ped_yolo/instance_mask', '/ped_yolo/detections2d', '/nav2/safety_points',
    '/j100_0519/nav2_cmd_vel']
started = time.monotonic()
while time.monotonic() - started < 3:
    rclpy.spin_once(n, timeout_sec=0.05)
types = dict(n.get_topic_names_and_types())
for topic in topics:
    if topic not in types:
        failures.append([topic, 'not discovered'])
        continue
    try:
        subscriptions.append(n.create_subscription(get_message(types[topic][0]), topic,
            lambda msg, topic=topic: callback(topic, msg), qos, raw=True))
    except Exception as exc:
        failures.append([topic, str(exc)])
if not a.nuc:
    subscriptions.append(n.create_subscription(DiagnosticArray, '/nav2/safety_diagnostics', diag, qos))
    subscriptions.append(n.create_subscription(TFMessage, '/tf', tf, qos))
requests = []
if not a.nuc:
    specs = {
        '/laserMapping': ['common/lid_topic', 'common/imu_topic', 'common/img_topic',
                         'common/img_en', 'common/lidar_en', 'use_sim_time'],
        '/nav2_safety_guard': ['enable_motion', 'input_topic', 'use_sim_time'],
        '/pointcloud_relay': ['input_topic', 'output_topic', 'input_depth', 'output_depth'],
        '/ped_yolo_node': ['device', 'use_sim_time'],
        '/mid360_lidar_accumulator_node': ['input_cloud_topic', 'output_cloud_topic', 'accumulation_frames'],
    }
    for target, names in specs.items():
        client = n.create_client(GetParameters, target + '/get_parameters')
        request = GetParameters.Request(names=names)
        requests.append((target, names, client, client.call_async(request)))
started = time.monotonic()
while time.monotonic() - started < a.duration:
    rclpy.spin_once(n, timeout_sec=0.05)
for target, names, client, future in requests:
    params[target] = {}
    if future.done() and future.result() is not None:
        for name, value in zip(names, future.result().values):
            params[target][name] = {'type': value.type, 'bool': value.bool_value,
                                  'integer': value.integer_value, 'double': value.double_value,
                                  'string': value.string_value}
    else:
        params[target]['error'] = 'parameter request timeout'
topology = {}
for topic in sorted(set(topics + ['/livox/lidar', '/camera/camera/color/image_raw'])):
    topology[topic] = {}
    for label, method in [('publishers', n.get_publishers_info_by_topic),
                          ('subscribers', n.get_subscriptions_info_by_topic)]:
        topology[topic][label] = [{
            'node': ep.node_namespace.rstrip('/') + '/' + ep.node_name,
            'type': ep.topic_type, 'reliability': str(ep.qos_profile.reliability),
            'depth': ep.qos_profile.depth,
        } for ep in method(topic)]
def stats(values):
    if not values:
        return None
    values = sorted(values)
    return {'min': values[0], 'p50': values[len(values)//2],
            'p95': values[min(len(values)-1, int(len(values)*.95))], 'max': values[-1]}
summary = {}
for topic, rows in streams.items():
    delta = rows[-1][0] - rows[0][0]
    age = [r[1]-r[2] for r in rows]
    summary[topic] = {
        'count': len(rows), 'hz': (len(rows)-1)/delta if delta else None,
        'age_sec': stats(age), 'first_age_sec': age[0], 'last_age_sec': age[-1],
        'interval_sec': stats([b[0]-a[0] for a,b in zip(rows,rows[1:])]),
        'stamp_delta_sec': stats([b[2]-a[2] for a,b in zip(rows,rows[1:])]),
        'stamp_regressions': sum(b[2]<a[2] for a,b in zip(rows,rows[1:])),
        'mean_serialized_bytes': sum(r[3] for r in rows)/len(rows),
        'frames': dict(Counter(r[4] for r in rows)),
    }
report = {'duration_sec': time.monotonic()-started, 'summary': summary,
          'topology': topology, 'parameters': params, 'diagnostics': diagnostics,
          'errors': failures, 'samples': streams,
          'limitations': 'Additional depth-1 best-effort probe; no remote cloud/image subscription on laptop.'}
Path(a.output).write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'diagnostics')}, indent=2))
print('diagnostic_messages', len(diagnostics))
n.destroy_node()
rclpy.shutdown()
