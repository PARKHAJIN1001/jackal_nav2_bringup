#!/usr/bin/env python3
"""Identify PS4 buttons with forwarding disabled and stationary platform odometry."""

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from prepare_navigation_profile import read_parameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
import yaml


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--config',
        type=Path,
        default=Path(get_package_share_directory('jackal_nav2_bringup'))
        / 'config/operator_stop.yaml',
    )
    parser.add_argument(
        '--footprint-confirmed',
        action='store_true',
        help='physical body and turning envelope fit stop half-width 0.40 m',
    )
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('Output exists; never overwrite a calibration')
    document = yaml.safe_load(args.config.read_text())
    params = document['nav2_operator_stop']['ros__parameters']
    rclpy.init(args=[])
    node = Node('calibrate_operator_stop')
    samples = {}

    def receive(key, msg):
        samples[key] = (time.monotonic(), msg)

    node.create_subscription(
        Joy, params['joy_topic'], lambda m: receive('joy', m), qos_profile_sensor_data
    )
    node.create_subscription(
        Odometry, params['odom_topic'], lambda m: receive('odom', m), qos_profile_sensor_data
    )

    def sample():
        bridge = read_parameters(node, params['bridge_node'], ['forward_cmd_vel'])
        guard = read_parameters(node, '/nav2_safety_guard', ['enable_motion'])
        if bridge.get('forward_cmd_vel') is not False or guard.get('enable_motion') is not False:
            raise RuntimeError('Require forward_cmd_vel=false AND enable_motion=false')
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if any(
            k not in samples or time.monotonic() - samples[k][0] > 0.3 for k in ('joy', 'odom')
        ):
            raise RuntimeError('Fresh Joy and platform odometry required')
        twist = samples['odom'][1].twist.twist
        if not (
            math.hypot(twist.linear.x, twist.linear.y) <= 0.02 and abs(twist.angular.z) <= 0.03
        ):
            raise RuntimeError('Platform is not stopped')
        return samples['joy'][1]

    try:
        input('Release all buttons, center the sticks; press Enter. ')
        neutral = sample()
        if any(neutral.buttons) or not all(math.isfinite(a) for a in neutral.axes):
            raise RuntimeError('Neutral sample is invalid')
        indices = []
        for label in ('Circle', 'Triangle', 'L1', 'R1'):
            input(f'Hold ONLY {label}, keep sticks centered, then press Enter. ')
            msg = sample()
            pressed = [i for i, value in enumerate(msg.buttons) if value]
            if (
                len(pressed) != 1
                or pressed[0] in indices
                or len(msg.axes) != len(neutral.axes)
                or any(abs(a - b) > 0.1 for a, b in zip(msg.axes, neutral.axes))
            ):
                raise RuntimeError('Expected one distinct button and neutral sticks')
            indices.append(pressed[0])
        params.update(
            stop_button=indices[0],
            reset_button=indices[1],
            deadman_buttons=indices[2:],
            neutral_axes=list(neutral.axes),
            mapping_verified=True,
            footprint_confirmed=args.footprint_confirmed,
        )
        with args.output.open('x') as stream:
            stream.write('# Observed ' + datetime.now(timezone.utc).isoformat() + '\n')
            yaml.safe_dump(document, stream, sort_keys=False)
        print('Saved observed mapping. Release all buttons. No motion enabled.')
        return 0
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt, EOFError) as exc:
        print(str(exc))
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
