"""
Opt-in isolated ROS integration: NEVER publish on a platform command topic.

Run explicitly with ROS_DOMAIN_ID=86 and the workspace sourced. This is not
collected by the default unit-test glob. All child processes are scoped here.
"""

import math
import os
import signal
import subprocess
import time

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from moai_nav_msgs.msg import Track, Tracks
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def test_static_costmaps_figures_and_independent_safety(tmp_path):
    assert (
        os.environ.get('ROS_DOMAIN_ID') == '86'
    ), 'Use the dedicated isolated test domain 86'
    assert (
        os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    ), 'Test must be restricted to localhost'
    grid = np.full((100, 100), 254, dtype=np.uint8)
    grid[[0, -1], :] = 0
    grid[:, [0, -1]] = 0
    (tmp_path / 'map.pgm').write_bytes(b'P5\n100 100\n255\n' + grid.tobytes())
    (tmp_path / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [-2.5, -2.5, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
    )
    log = (tmp_path / 'launch.log').open('w')
    process = subprocess.Popen(
        [
            'ros2',
            'launch',
            'jackal_nav2_bringup',
            'bringup.launch.py',
            f'map:={tmp_path / "map.yaml"}',
            'use_rviz:=false',
            'use_map_patch:=false',
            'use_scan_projection:=false',
            'enable_motion:=true',
            'lidar_pointcloud_topic:=/nav2_test/raw',
            'nav_cmd_vel_topic:=/nav2_test/output',
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = Node('isolated_navigation_test')
    tf = TransformBroadcaster(node)
    raw = node.create_publisher(
        PointCloud2, '/nav2_test/raw', qos_profile_sensor_data
    )
    cmd = node.create_publisher(Twist, '/nav2_cmd_vel_unstamped', 1)
    tracks = node.create_publisher(Tracks, '/ped_tracking', 1)
    outputs, figures, costmaps = [], [], {}
    node.create_subscription(
        TwistStamped,
        '/nav2_test/output',
        lambda msg: outputs.append((time.monotonic(), msg.twist.linear.x)),
        10,
    )
    node.create_subscription(
        MarkerArray,
        '/nav2/pedestrian_figures',
        lambda msg: figures.append((time.monotonic(), msg)),
        10,
    )
    for topic in (
        '/static_costmap/costmap',
        '/global_costmap/costmap',
        '/local_costmap/costmap',
    ):
        node.create_subscription(
            OccupancyGrid,
            topic,
            lambda msg, name=topic: costmaps.update({name: list(msg.data)}),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def run(
        seconds,
        points=((2.0, 0.0, 0.5),),
        send_command=True,
        send_sensor=True,
        send_tracks=True,
        track_x=1.0,
        frame='odom',
        yaw=0.0,
        old_stamp=False,
        send_tf=True,
        speed=0.2,
    ):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            now = node.get_clock().now()
            if send_tf:
                transforms = []
                for parent, child, future in [
                    ('map', 'odom', 1.0),
                    ('odom', 'base_link', 0.0),
                ]:
                    transform = TransformStamped()
                    transform.header.frame_id, transform.child_frame_id = (
                        parent,
                        child,
                    )
                    transform.header.stamp = (
                        now + rclpy.duration.Duration(seconds=future)
                    ).to_msg()
                    transform.transform.rotation.w = (
                        math.cos(yaw / 2) if child == 'base_link' else 1.0
                    )
                    transform.transform.rotation.z = (
                        math.sin(yaw / 2) if child == 'base_link' else 0.0
                    )
                    transforms.append(transform)
                tf.sendTransform(transforms)
            if send_sensor:
                message = PointCloud2(
                    height=1,
                    width=len(points),
                    point_step=12,
                    row_step=12 * len(points),
                )
                message.header.frame_id = 'base_link'
                message.header.stamp = (
                    now - rclpy.duration.Duration(seconds=2.0)
                    if old_stamp
                    else now
                ).to_msg()
                message.fields = [
                    PointField(name=n, offset=i * 4, datatype=7, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))
                ]
                message.data = np.asarray(points, dtype='<f4').tobytes()
                raw.publish(message)
            if send_command:
                command = Twist()
                command.linear.x = speed
                cmd.publish(command)
            if send_tracks:
                observation = Tracks()
                observation.header.frame_id, observation.header.stamp = (
                    frame,
                    now.to_msg(),
                )
                person = Track(id=2**40, object_type=0)
                person.pose.position.x = track_x
                observation.tracks.append(person)
                tracks.publish(observation)
            # Drain all subscriptions: one spin per publish leaves test-observer
            # callbacks queued and would compare old commands with new inputs.
            spin_until = time.monotonic() + 0.05
            while time.monotonic() < spin_until:
                rclpy.spin_once(node, timeout_sec=0.002)

    def latest_output():
        assert outputs and time.monotonic() - outputs[-1][0] < 0.3
        return outputs[-1][1]

    def latest_add():
        return [
            m
            for _, msg in figures[-5:]
            for m in msg.markers
            if m.action == Marker.ADD
        ]

    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            run(1.0)
            if (
                len(costmaps) == 3
                and outputs
                and outputs[-1][1] > 0.19
                and latest_add()
            ):
                break
            assert process.poll() is None, (
                tmp_path / 'launch.log'
            ).read_text()[-8000:]
        assert len(costmaps) == 3, (tmp_path / 'launch.log').read_text()[
            -8000:
        ]
        assert latest_output() == pytest.approx(0.2)
        baseline = {name: list(cells) for name, cells in costmaps.items()}
        for x in (0.8, 0.7, 0.9):
            run(0.5, points=((x, 0.0, 0.5),) * 3, track_x=x)
            assert latest_output() == pytest.approx(0.06)
            assert (
                costmaps == baseline
            ), 'Live points must not change any costmap cell'
        # Runtime subscriptions must also exclude all live obstacle inputs.
        for name, namespace in [
            ('local_costmap', '/local_costmap'),
            ('global_costmap', '/global_costmap'),
            ('static_costmap', '/static_costmap'),
        ]:
            subscriptions = node.get_subscriber_names_and_types_by_node(
                name, namespace
            )
            assert not any(
                topic
                in (
                    '/nav2_test/raw',
                    '/nav2/safety_points',
                    '/scan',
                    '/ped_tracking',
                    '/ped_detection',
                )
                for topic, _ in subscriptions
            )
        run(0.5, points=((0.4, 0.1, 0.5),) * 3)
        assert latest_output() == 0.0
        assert costmaps == baseline
        run(0.5, points=((0.4, 0.1, 0.5),) * 3, send_tracks=False)
        figure_stop = time.monotonic()
        run(1.3, points=((0.4, 0.1, 0.5),) * 3, send_tracks=False)
        assert latest_output() == 0.0
        recent = [
            (received, msg)
            for received, msg in figures
            if received > figure_stop + 0.4
        ]
        assert any(
            m.action == Marker.DELETE for _, msg in recent for m in msg.markers
        )
        run(0.5)
        assert latest_output() == pytest.approx(0.2)
        run(0.5, send_sensor=False)
        assert latest_output() == 0.0
        run(0.5, old_stamp=True)
        assert latest_output() == 0.0
        run(
            0.5, points=((0.0, 0.0, 0.5),)
        )  # Valid scan, only robot-body points -> healthy empty.
        assert latest_output() == pytest.approx(0.2)
        run(0.5, points=())  # Empty raw input is not a healthy observation.
        assert latest_output() == 0.0
        run(0.5)
        run(0.5, send_command=False)
        assert latest_output() == 0.0
        run(0.5, speed=float('nan'))
        assert latest_output() == 0.0
        run(0.5)
        run(0.6, send_tf=False)
        assert latest_output() == 0.0
        # Bad measurement frame must delete figures, never use latest TF or embedded odom.
        run(0.5, frame='missing_camera_frame')
        assert any(
            m.action == Marker.DELETE
            for _, msg in figures[-5:]
            for m in msg.markers
        )
        # The guard's stop remains effective even if CM's output bypasses its algorithm:
        # direct fault-injection on an ISOLATED test-domain intermediate topic only.
        unchecked = node.create_publisher(
            Twist, '/nav2/collision_checked_cmd_vel', 1
        )
        run(0.5, points=((0.4, 0.1, 0.5),) * 3, send_command=False)
        bad = Twist()
        bad.linear.x = 0.2
        for _ in range(10):
            unchecked.publish(bad)
            run(0.06, points=((0.4, 0.1, 0.5),) * 3, send_command=False)
        assert latest_output() == 0.0
        node.destroy_publisher(unchecked)
        run(0.5)
        assert latest_output() == pytest.approx(0.2)
        # Kill only the monitor child owned by this test launch process group.
        rows = subprocess.check_output(
            ['ps', '-eo', 'pid,pgid,args'], text=True
        ).splitlines()
        monitor_pids = [
            int(row.split(None, 2)[0])
            for row in rows[1:]
            if len(row.split(None, 2)) == 3
            and row.split(None, 2)[1] == str(process.pid)
            and '/lib/nav2_collision_monitor/collision_monitor ' in row
        ]
        assert len(monitor_pids) == 1
        os.kill(monitor_pids[0], signal.SIGINT)
        run(0.7)
        assert latest_output() == 0.0
        print(
            'PASS: static cell invariance/subscriptions, figure expiry/TF failure, '
            'CM slowdown/stop, independent stop, stale/empty sensor, lost TF, '
            'NaN, monitor command timeout and monitor process exit; '
            'output=/nav2_test/output only'
        )
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
