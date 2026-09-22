#!/usr/bin/env python3
"""Export time-aligned command stages and goal evidence from a navigation rosbag."""

import argparse
import csv
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STAGES = {
    '/cmd_vel_nav': 'controller',
    '/nav2_cmd_vel_unstamped': 'smoother',
    '/nav2/collision_checked_cmd_vel': 'collision_monitor',
    '/j100_0519/nav2_cmd_vel': 'guard',
    '/j100_0519/cmd_vel': 'bridge',
    '/j100_0519/platform/cmd_vel_unstamped': 'platform_command',
    '/j100_0519/platform/odom': 'platform_odom',
    '/odom': 'localization_odom',
}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def analyze(reader, output, nav_topic='/j100_0519/nav2_cmd_vel'):
    types = {entry.name: get_message(entry.type) for entry in reader.get_all_topics_and_types()}
    stages = {**STAGES, nav_topic: 'guard'}
    current, stamps, events = {}, {}, []
    goal_pose = None
    distance = angle = ''
    reason = phase = ''
    last_row = -math.inf
    names = list(dict.fromkeys(stages.values()))
    fields = [
        'time_sec',
        'distance_remaining',
        'path_endpoint_yaw_error',
        'guard_reason',
        'operator_phase',
    ]
    fields += [name + suffix for name in names for suffix in ('_v', '_w', '_age')]
    with (output / 'timeline.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            t = stamp * 1e-9
            msg = deserialize_message(data, types[topic])
            if topic in stages:
                twist = msg.twist if hasattr(msg, 'twist') else msg
                if hasattr(twist, 'twist'):
                    twist = twist.twist
                current[stages[topic]] = (twist.linear.x, twist.angular.z)
                stamps[stages[topic]] = t
            elif topic == '/plan' and msg.poses:
                goal_pose = msg.poses[-1].pose
            elif topic.endswith('/_action/feedback'):
                distance = float(msg.feedback.distance_remaining)
                pose = msg.feedback.current_pose
                if goal_pose is not None and pose.header.frame_id == 'map':
                    delta = yaw(goal_pose.orientation) - yaw(pose.pose.orientation)
                    angle = math.atan2(math.sin(delta), math.cos(delta))
            elif topic.endswith('/_action/status'):
                for status in msg.status_list:
                    event = {
                        'uuid': bytes(status.goal_info.goal_id.uuid).hex(),
                        'status': status.status,
                    }
                    if not any(
                        e.get('uuid') == event['uuid'] and e.get('status') == event['status']
                        for e in events
                    ):
                        events.append({'time_sec': t, **event})
            elif topic in ('/nav2/safety_diagnostics', '/nav2/operator_stop_diagnostics'):
                for status in msg.status:
                    if status.name == 'nav2_safety_guard':
                        reason = status.message
                    if status.name == 'nav2_operator_stop':
                        phase = status.message
            elif topic == '/rosout' and msg.level >= 30:
                events.append({'time_sec': t, 'node': msg.name, 'message': msg.msg})
            if t - last_row >= 0.05:
                row = {
                    'time_sec': t,
                    'distance_remaining': distance,
                    'path_endpoint_yaw_error': angle,
                    'guard_reason': reason,
                    'operator_phase': phase,
                }
                for name in names:
                    if name in current:
                        row.update(
                            {
                                name + '_v': current[name][0],
                                name + '_w': current[name][1],
                                name + '_age': t - stamps[name],
                            }
                        )
                writer.writerow(row)
                last_row = t
    report = {
        'events': events,
        'meaning': 'Time-aligned received evidence; stale stage samples have '
        'explicit ages. Path endpoint yaw is not independently measured goal accuracy. '
        'Physical button-to-stop timing requires synchronized video and platform odometry.',
    }
    (output / 'events.json').write_text(json.dumps(report, indent=2))
    return report


def main(argv=None):
    import rosbag2_py

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--nav-cmd-topic', default='/j100_0519/nav2_cmd_vel')
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    analyze(reader, args.output_dir, args.nav_cmd_topic)


if __name__ == '__main__':
    main()
