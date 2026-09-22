"""Exercise published heading maps and stale/missing TF in isolated domain 190."""

import math
import os
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_patch_node import MapPatchNode  # noqa: E402,I100


def test_heading_map_stamp_and_stale_tf():
    assert os.environ.get('ROS_DOMAIN_ID') == '190'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init(args=[])
    patch = MapPatchNode()
    fixture = Node('map_patch_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(patch)
    executor.add_node(fixture)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps, diagnostics = [], []
    fixture.create_subscription(OccupancyGrid, '/map_encoder/input', maps.append, qos)
    fixture.create_subscription(
        DiagnosticArray, '/nav2/map_patch_diagnostics', diagnostics.append, 10
    )
    publisher = fixture.create_publisher(OccupancyGrid, '/static_costmap/costmap', qos)
    tf = TransformBroadcaster(fixture)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 200
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 40000
    grid.data[100 * 200 + 120] = 100  # world (1.025, 0.025)
    publisher.publish(grid)

    def pump(seconds, heading=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if heading is not None:
                msg = TransformStamped()
                msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
                msg.header.stamp = fixture.get_clock().now().to_msg()
                msg.transform.rotation.z = math.sin(heading / 2)
                msg.transform.rotation.w = math.cos(heading / 2)
                tf.sendTransform(msg)
            executor.spin_once(timeout_sec=0.01)

    try:
        pump(0.5)
        assert not maps  # no TF must never synthesize a normal patch
        for heading in (0, math.pi / 2, -math.pi / 2, math.pi):
            pump(0.5, heading)
            msg = maps[-1]
            assert msg.header.frame_id == 'base_link'
            assert (msg.info.width, msg.info.height) == (200, 200)
            assert msg.header.stamp.sec > 0
            indices = [i for i, value in enumerate(msg.data) if value == 100]
            assert len(indices) == 1
            x, y = (indices[0] % 200 + 0.5) * 0.05 - 5, (indices[0] // 200 + 0.5) * 0.05 - 5
            world_x = math.cos(heading) * x - math.sin(heading) * y
            world_y = math.sin(heading) * x + math.cos(heading) * y
            assert math.hypot(world_x - 1.025, world_y - 0.025) <= 0.05
        last_stamp = maps[-1].header.stamp
        pump(0.5)
        count = len(maps)
        pump(0.4)
        assert len(maps) == count
        assert maps[-1].header.stamp == last_stamp or (
            maps[-1].header.stamp.sec * 10**9 + maps[-1].header.stamp.nanosec
            <= fixture.get_clock().now().nanoseconds - 300000000
        )
        assert diagnostics[-1].status[0].message == 'stale_or_future_robot_pose'
    finally:
        executor.shutdown()
        patch.destroy_node()
        fixture.destroy_node()
        rclpy.try_shutdown()
