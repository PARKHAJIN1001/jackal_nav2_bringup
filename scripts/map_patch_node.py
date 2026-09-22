#!/usr/bin/env python3

"""Publish a fixed-size robot-centric patch of the static Nav2 costmap."""

from dataclasses import dataclass
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def pose_stamp_usable(stamp, now, maximum_age=0.3):
    return math.isfinite(stamp) and stamp > 0 and -0.05 <= now-stamp <= maximum_age


@dataclass(frozen=True)
class GridGeometry:
    """Describe a two-dimensional, row-major occupancy grid."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def validate(self):
        """Raise ValueError when the geometry cannot describe a grid."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError('grid width and height must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError('grid resolution must be finite and positive')
        if not all(math.isfinite(value) for value in (
                self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('grid origin must be finite')


def quaternion_to_yaw(x, y, z, w):
    """Return yaw from a finite, non-zero quaternion."""
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion must be finite')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be non-zero')
    x_n, y_n, z_n, w_n = (value / norm for value in values)
    return math.atan2(
        2.0 * (w_n * z_n + x_n * y_n),
        1.0 - 2.0 * (y_n * y_n + z_n * z_n),
    )


def extract_robot_centric_patch(
        source_data, source_geometry, robot_x, robot_y, robot_yaw,
        output_geometry, unknown_value=-1):
    """Nearest-neighbor sample a static grid into a +x-forward robot grid."""
    source_geometry.validate()
    output_geometry.validate()
    if len(source_data) != source_geometry.width * source_geometry.height:
        raise ValueError('source data length does not match its geometry')
    if not all(math.isfinite(value) for value in (
            robot_x, robot_y, robot_yaw)):
        raise ValueError('robot pose must be finite')
    if not -1 <= int(unknown_value) <= 100:
        raise ValueError('unknown value must fit nav_msgs/OccupancyGrid')

    robot_cos = math.cos(robot_yaw)
    robot_sin = math.sin(robot_yaw)
    source_cos = math.cos(source_geometry.origin_yaw)
    source_sin = math.sin(source_geometry.origin_yaw)
    output = [int(unknown_value)] * (
        output_geometry.width * output_geometry.height)

    for output_row in range(output_geometry.height):
        local_y = (
            output_geometry.origin_y
            + (output_row + 0.5) * output_geometry.resolution
        )
        for output_col in range(output_geometry.width):
            local_x = (
                output_geometry.origin_x
                + (output_col + 0.5) * output_geometry.resolution
            )

            map_x = robot_x + robot_cos * local_x - robot_sin * local_y
            map_y = robot_y + robot_sin * local_x + robot_cos * local_y

            delta_x = map_x - source_geometry.origin_x
            delta_y = map_y - source_geometry.origin_y
            source_x = source_cos * delta_x + source_sin * delta_y
            source_y = -source_sin * delta_x + source_cos * delta_y
            source_col = math.floor(source_x / source_geometry.resolution)
            source_row = math.floor(source_y / source_geometry.resolution)

            if (0 <= source_col < source_geometry.width
                    and 0 <= source_row < source_geometry.height):
                output_index = output_row * output_geometry.width + output_col
                source_index = source_row * source_geometry.width + source_col
                output[output_index] = int(source_data[source_index])

    return output


class MapPatchNode(Node):
    """Crop and rotate the static costmap around the current robot pose."""

    def __init__(self):
        super().__init__('map_patch_node')

        self.declare_parameter(
            'input_topic', '/static_costmap/costmap')
        self.declare_parameter('output_topic', '/map_encoder/input')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('patch_width', 10.0)
        self.declare_parameter('patch_height', 10.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('output_width', 200)
        self.declare_parameter('output_height', 200)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('transform_tolerance', 0.2)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('max_pose_age', 0.3)
        self._max_pose_age = float(self.get_parameter('max_pose_age').value)
        if not math.isfinite(self._max_pose_age) or self._max_pose_age <= 0:
            raise ValueError('max_pose_age must be positive and finite')
        self._diagnostics = self.create_publisher(
            DiagnosticArray, '/nav2/map_patch_diagnostics', 10)

        self._input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self._map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        patch_width = self.get_parameter('patch_width').value
        patch_height = self.get_parameter('patch_height').value
        resolution = self.get_parameter('resolution').value
        output_width = self.get_parameter('output_width').value
        output_height = self.get_parameter('output_height').value
        update_rate = self.get_parameter('update_rate').value
        transform_tolerance = self.get_parameter(
            'transform_tolerance').value
        self._unknown_value = self.get_parameter('unknown_value').value

        if update_rate <= 0.0:
            raise ValueError('update_rate must be positive')
        if transform_tolerance < 0.0:
            raise ValueError('transform_tolerance must be non-negative')
        if not math.isclose(
                patch_width, output_width * resolution,
                rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError(
                'patch_width must equal output_width * resolution')
        if not math.isclose(
                patch_height, output_height * resolution,
                rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError(
                'patch_height must equal output_height * resolution')

        self._output_geometry = GridGeometry(
            width=int(output_width),
            height=int(output_height),
            resolution=float(resolution),
            origin_x=-float(patch_width) / 2.0,
            origin_y=-float(patch_height) / 2.0,
        )
        self._output_geometry.validate()
        self._transform_timeout = Duration(seconds=float(transform_tolerance))
        self._latest_costmap = None
        self._last_warning = {}

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid, self._output_topic, transient_qos)
        self._subscription = self.create_subscription(
            OccupancyGrid,
            self._input_topic,
            self._costmap_callback,
            transient_qos,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0 / float(update_rate), self._publish)

        self.get_logger().info(
            f'Publishing {output_width}x{output_height} robot-centric static '
            f'patches from {self._input_topic} to {self._output_topic}')

    def _warn_throttled(self, key, message):
        now = time.monotonic()
        if now - self._last_warning.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self._last_warning[key] = now

    def _costmap_callback(self, message):
        if message.header.frame_id != self._map_frame:
            self._warn_throttled(
                'frame',
                f'Ignoring static costmap in frame '
                f'{message.header.frame_id!r}; expected {self._map_frame!r}',
            )
            return
        self._latest_costmap = message

    def _publish(self):
        source = self._latest_costmap
        if source is None:
            self._warn_throttled(
                'costmap', f'Waiting for {self._input_topic}')
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if not pose_stamp_usable(stamp, now, self._max_pose_age):
                raise ValueError('stale_or_future_robot_pose')
            robot_yaw = quaternion_to_yaw(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            source_yaw = quaternion_to_yaw(
                source.info.origin.orientation.x,
                source.info.origin.orientation.y,
                source.info.origin.orientation.z,
                source.info.origin.orientation.w,
            )
            source_geometry = GridGeometry(
                width=int(source.info.width),
                height=int(source.info.height),
                resolution=float(source.info.resolution),
                origin_x=float(source.info.origin.position.x),
                origin_y=float(source.info.origin.position.y),
                origin_yaw=source_yaw,
            )
            patch_data = extract_robot_centric_patch(
                source.data,
                source_geometry,
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                robot_yaw,
                self._output_geometry,
                self._unknown_value,
            )
        except (TransformException, ValueError) as error:
            self._warn_throttled(
                'extract', f'Cannot publish static map patch: {error}')
            self._report(False, str(error))
            return

        output = OccupancyGrid()
        output.header.stamp = transform.header.stamp
        output.header.frame_id = self._base_frame
        output.info.map_load_time = source.info.map_load_time
        output.info.resolution = self._output_geometry.resolution
        output.info.width = self._output_geometry.width
        output.info.height = self._output_geometry.height
        output.info.origin.position.x = self._output_geometry.origin_x
        output.info.origin.position.y = self._output_geometry.origin_y
        output.info.origin.orientation.w = 1.0
        output.data = patch_data
        self._publisher.publish(output)
        self._report(True, 'fresh_heading_patch')

    def _report(self, ok, reason):
        msg = DiagnosticArray(status=[DiagnosticStatus(
            name='map_patch',
            level=DiagnosticStatus.OK if ok else DiagnosticStatus.WARN, message=reason)])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics.publish(msg)


def main(args=None):
    """Run the map patch node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MapPatchNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok(context=node.context):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
