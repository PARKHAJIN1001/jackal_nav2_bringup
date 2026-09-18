#!/usr/bin/env python3
"""
Exercise real DDS gate subscriptions in domain 187 on localhost only.

Run manually with the ROS/workspace sourced. No hardware nodes, navigation
goals, initial poses or velocity commands are created. Fake lifecycle services
stand in for AMCL/map_server; this does not validate either implementation.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def main():
    os.environ['ROS_DOMAIN_ID'] = '187'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(key, None)
    logs = Path(tempfile.mkdtemp(prefix='nav_gate_integration_'))
    os.environ['ROS_LOG_DIR'] = str(logs)
    rclpy.init()
    node = Node('fake_gate_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    processes, files = [], []
    script = Path(__file__).resolve().parents[1] / 'scripts/topic_ready_gate.py'

    def start(name, **params):
        path = logs / (name + '.log')
        stream = path.open('w')
        files.append(stream)
        command = [sys.executable, str(script), '--ros-args']
        for key, value in dict(settle=0.4, timeout=8.0, **params).items():
            command.extend(['-p', f'{key}:={str(value).lower()}'])
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def pump(seconds, publish=lambda: None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publish()
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.03)

    def finish(process, code, publish=lambda: None):
        deadline = time.monotonic() + 10.0
        while process.poll() is None and time.monotonic() < deadline:
            pump(0.05, publish)
        assert process.poll() == code, f'exit {process.poll()}, expected {code}; logs: {logs}'

    try:
        cloud_pub = node.create_publisher(PointCloud2, '/test_cloud', qos_profile_sensor_data)
        imu_pub = node.create_publisher(Imu, '/test_imu', qos_profile_sensor_data)
        cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12)
        cloud.header.frame_id = 'livox_frame'
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = bytes(12)
        imu = Imu()
        imu.header.frame_id = 'livox_imu_frame'
        imu.linear_acceleration.z = 9.81

        # Graph endpoints exist for the entire timeout, but send no messages.
        empty = start('publisher_only', topic='/test_cloud', message_type='cloud')
        finish(empty, 1)
        assert 'no messages received' in (logs / 'publisher_only.log').read_text()
        print('PASS: publisher-only input times out', flush=True)

        sensors = start('sensors', topic='/test_cloud', message_type='cloud',
                        imu_topic='/test_imu')

        def send_sensors(stale=False, include_imu=True):
            stamp = node.get_clock().now().to_msg()
            if stale:
                stamp.sec -= 2
            cloud.header.stamp = imu.header.stamp = stamp
            cloud_pub.publish(cloud)
            if include_imu:
                imu_pub.publish(imu)

        pump(1.5, lambda: send_sensors(include_imu=False))
        assert sensors.poll() is None
        pump(1.0, lambda: send_sensors(stale=True))
        assert sensors.poll() is None
        finish(sensors, 0, send_sensors)
        print('PASS: both sensors and fresh timestamps required', flush=True)

        state_id = [1]

        def get_state(request, response):
            response.current_state.id = state_id[0]
            return response

        node.create_service(GetState, '/amcl/get_state', get_state)
        node.create_service(GetState, '/map_server/get_state', get_state)
        map_pub = node.create_publisher(
            OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 1
        grid.info.resolution = 0.1
        grid.data = [0]
        map_pub.publish(grid)
        scan_pub = node.create_publisher(LaserScan, '/test_scan', qos_profile_sensor_data)
        odom_pub = node.create_publisher(Odometry, '/test_odom', qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(node)
        localization = start('localization', topic='/test_scan', message_type='scan',
                             expected_frame='base_link', odom_topic='/test_odom',
                             require_localization=True)

        def send_localization(with_tf=False):
            stamp = node.get_clock().now().to_msg()
            scan = LaserScan(angle_increment=0.1, range_min=0.1, range_max=5.0, ranges=[1.0])
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            if with_tf:
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
                # Model TF computation lag: the newest scan never has TF yet.
                delayed_ns = stamp.sec * 10**9 + stamp.nanosec - 50000000
                tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(delayed_ns, 10**9)
                tf.transform.rotation.w = 1.0
                broadcaster.sendTransform(tf)
            scan_pub.publish(scan)
            odom_pub.publish(odom)

        pump(1.5, lambda: send_localization(with_tf=True))
        assert localization.poll() is None  # Lifecycle nodes are still unconfigured.
        state_id[0] = 3
        pump(1.2, send_localization)
        assert localization.poll() is None  # No TF for current scan timestamps.
        finish(localization, 0, lambda: send_localization(with_tf=True))
        print('PASS: active lifecycle, map and scan-time TF; no map->odom needed', flush=True)
        perception = start(
            'before_perception', topic='/test_scan', message_type='scan',
            expected_frame='base_link', odom_topic='/test_odom',
            require_localization=True, require_map_to_odom=True)
        pump(1.5, lambda: send_localization(with_tf=True))
        assert perception.poll() is None  # Valid odom alone is not initialized AMCL.

        def send_initialized():
            send_localization(with_tf=True)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)

        finish(perception, 0, send_initialized)
        print('PASS: separate perception gate waits for scan-time map->odom', flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for stream in files:
            stream.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f'Logs: {logs}', flush=True)


if __name__ == '__main__':
    main()
