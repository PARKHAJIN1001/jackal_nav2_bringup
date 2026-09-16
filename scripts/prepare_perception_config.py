#!/usr/bin/env python3
"""Generate Nav2 input profiles for separately launched perception; never launch it."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shlex
import tempfile

import yaml


def nav2_profile(document, node, overrides):
    """Preserve installed tuning; fail rather than silently invent a node config."""
    result = deepcopy(document)
    if (not isinstance(result, dict) or not isinstance(result.get(node), dict)
            or not isinstance(result[node].get('ros__parameters'), dict)):
        raise ValueError(f'missing {node}.ros__parameters')
    params = result[node]['ros__parameters']
    for key, value in overrides.items():
        if key not in params:
            raise ValueError(f'unsupported installed parameter: {node}.{key}')
        params[key] = value
    return result


def prepare(share, output, lidar_topic='/livox/lidar_local'):
    """Write a new, versioned profile directory without touching sibling packages."""
    from rclpy.exceptions import InvalidTopicNameException
    from rclpy.validate_full_topic_name import validate_full_topic_name
    try:
        validate_full_topic_name(lidar_topic)
    except InvalidTopicNameException as exc:
        raise ValueError(f'invalid absolute LiDAR topic: {lidar_topic}') from exc
    specs = (
        ('lidar_preprocess.yaml', 'mid360_lidar_accumulator_node',
         {'input_cloud_topic': lidar_topic}),
        ('mask_3d_extractor.yaml', 'mid360_mask_3d_extractor_node',
         {'tracking_frame': 'odom', 'use_latest_tf_fallback': False}),
    )
    profiles, manifest = {}, {'source_share': str(share), 'sources': {}}
    for filename, node, overrides in specs:
        source = share / 'config' / filename
        raw = source.read_bytes()
        profiles[filename] = nav2_profile(yaml.safe_load(raw), node, overrides)
        manifest['sources'][filename] = {
            'sha256': hashlib.sha256(raw).hexdigest(), 'overrides': overrides}
    # Validate everything before creating files, and never overwrite old evidence.
    if output is None:
        output = Path(tempfile.mkdtemp(prefix='nav2_perception_'))
    else:
        output = output.expanduser().absolute()
        output.mkdir(parents=True, exist_ok=False)
    for filename, document in profiles.items():
        (output / filename).write_text(yaml.safe_dump(document, sort_keys=False))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return output


def perception_command(directory):
    return [
        'ros2', 'launch', 'mid360_bringup', 'perception.launch.py',
        f'lidar_preprocess_config:={directory / "lidar_preprocess.yaml"}',
        f'extractor_config:={directory / "mask_3d_extractor.yaml"}',
        'publish_sensor_static_tf:=true', 'publish_base_to_lidar_tf:=false',
        'publish_lidar_to_imu_tf:=false', 'publish_lidar_to_camera_tf:=true',
        'launch_rviz:=false', 'launch_detection_markers:=false',
        'launch_track_markers:=false', 'launch_trace_markers:=false',
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--share', type=Path, help='mid360_bringup share directory')
    parser.add_argument('--output-dir', type=Path, help='must not already exist')
    parser.add_argument('--lidar-topic', default='/livox/lidar_local')
    args = parser.parse_args(argv)
    if args.share is None:
        from ament_index_python.packages import get_package_share_directory
        args.share = Path(get_package_share_directory('mid360_bringup'))
    try:
        directory = prepare(args.share, args.output_dir, args.lidar_topic)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(f'Profiles saved to {directory}; no ROS nodes were started.')
    print('After fixing the FAST-LIVO2 frame/timestamp contract, run separately:')
    print(shlex.join(perception_command(directory)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
