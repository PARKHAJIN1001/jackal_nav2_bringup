#!/usr/bin/env python3
"""Read the network owner's report before creating ROS processes."""

import json
import os
from pathlib import Path
import subprocess
import time

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


CRITICAL = {
    'fastlivo_mapping', 'pointcloud_relay_node', 'nav2_safety_guard.py',
    'operator_stop.py', 'fast_livo_odom_adapter.py', 'stack_stability.py',
    'collision_monitor', 'controller_server', 'planner_server', 'bt_navigator',
    'velocity_smoother', 'amcl', 'map_server', 'static_costmap_node',
    'pointcloud_to_laserscan_node', 'component_container_isolated',
    'smoother_server', 'behavior_server', 'waypoint_follower', 'lifecycle_manager',
}


def critical_exit(event, context):
    if context.is_shutdown or not event.cmd:
        return []
    names = [Path(value).name for value in event.cmd[:2]]
    executable = names[1] if names[0].startswith('python') and len(names) > 1 else names[0]
    if executable in CRITICAL or (executable == 'topic_ready_gate.py' and event.returncode):
        reason = f'{executable} exited ({event.returncode}); restart required'
        directory = os.environ.get('JACKAL_NAV_SESSION_DIR')
        if directory:
            path = Path(directory) / 'failure.json'
            # This log directory belongs to the fresh managed session, not a global marker.
            try:
                with path.open('x') as stream:
                    json.dump({'reason': reason, 'monotonic_sec': time.monotonic(),
                               'wall_sec': time.time()}, stream)
            except OSError:
                pass  # Preserve the first failure; logging failure must not prevent shutdown.
        return [Shutdown(reason=reason)]
    return []


def preflight(context):
    # Private context attribute cannot be bypassed by a launch CLI argument.
    if getattr(context, '_jackal_network_checked', False):
        return []
    result = subprocess.run(
        ['ros2', 'run', 'jackal_network_bringup', 'network_preflight.py', '--check'],
        capture_output=True, text=True, timeout=15, check=False)
    try:
        report = json.loads(result.stdout)
    except ValueError as error:
        raise RuntimeError('Rebuild jackal_network_bringup: network preflight unavailable; '
                           + result.stderr.strip()) from error
    if result.returncode != 0 or report.get('ready') is not True:
        raise RuntimeError('Network preflight failed: ' + json.dumps(report))
    context._jackal_network_checked = True
    return [RegisterEventHandler(OnProcessExit(on_exit=critical_exit))]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=preflight)])
