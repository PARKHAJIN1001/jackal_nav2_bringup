#!/usr/bin/env python3
"""Read bridge limits and generate a new local profile; never enable forwarding or motion."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
import yaml


def bounded_limits(parameters):
    result = {}
    for name, requested in (('max_linear_x', 0.50), ('max_angular_z', 1.0)):
        value = parameters.get(name)
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Bridge limit unavailable or invalid: {name}')
        result[name] = min(requested, value)
    return result


def read_parameters(node, server, names, timeout=5.0):
    client = node.create_client(GetParameters, server.rstrip('/') + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Parameter service unavailable: {server}')
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Parameter query timed out: {server}')
        return dict(zip(names, map(parameter_value_to_python, future.result().values)))
    finally:
        node.destroy_client(client)


def generate(nav, safety, operator, bridge):
    limits = bounded_limits(bridge)
    v, w = limits['max_linear_x'], limits['max_angular_z']
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    follow.update(max_vel_x=v, max_speed_xy=v, max_vel_theta=w)
    nav['velocity_smoother']['ros__parameters'].update(
        max_velocity=[v, 0.0, w], min_velocity=[-v, 0.0, -w]
    )
    nav['behavior_server']['ros__parameters']['max_rotational_vel'] = w
    safety['nav2_safety_guard']['ros__parameters'].update(limits)
    operator['nav2_operator_stop']['ros__parameters'].update(limits)
    return nav, safety, operator


def main(argv=None):
    share = Path(get_package_share_directory('jackal_nav2_bringup'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument(
        '--operator-config', type=Path, default=share / 'config/operator_stop.yaml'
    )
    parser.add_argument('--bridge-node', default='/cmd_vel_safety_bridge')
    args = parser.parse_args(argv)
    if args.output_dir.exists():
        parser.error('--output-dir must be new')
    rclpy.init(args=[])
    node = Node('prepare_navigation_profile')
    try:
        bridge = read_parameters(node, args.bridge_node, ['max_linear_x', 'max_angular_z'])
        operator = yaml.safe_load(args.operator_config.read_text())
        operator['nav2_operator_stop']['ros__parameters']['bridge_node'] = args.bridge_node
        data = generate(
            yaml.safe_load((share / 'config/nav2_params.yaml').read_text()),
            yaml.safe_load((share / 'config/nav2_safety.yaml').read_text()),
            operator,
            bridge,
        )
        args.output_dir.mkdir(parents=True, exist_ok=False)
        for filename, document in zip(('nav2.yaml', 'safety.yaml', 'operator.yaml'), data):
            (args.output_dir / filename).write_text(yaml.safe_dump(document, sort_keys=False))
        (args.output_dir / 'bridge_snapshot.json').write_text(
            json.dumps(
                {
                    'node': args.bridge_node,
                    'parameters': bridge,
                    'captured_utc': datetime.now(timezone.utc).isoformat(),
                },
                indent=2,
            )
        )
        print('Profile written. No motion enabled. Use the standard session wrapper:')
        import shlex
        print('ros2 run jackal_nav2_bringup nav_session.py nav --map /absolute/map.yaml '
              '--profile-dir ' + shlex.quote(str(args.output_dir.resolve())))
        print('Add --enable-motion only for an attended motion-enabled session.')
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(str(exc))
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
