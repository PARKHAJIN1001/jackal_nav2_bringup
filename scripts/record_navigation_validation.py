#!/usr/bin/env python3
"""Record localization/perception and optional navigation evidence; never command motion."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory


def capture(command, destination, timeout=10):
    """Record both command output and failure, without changing ROS state."""
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True,
                               start_new_session=True)
    try:
        output, _ = process.communicate(timeout=timeout)
        destination.write_text(output, encoding='utf-8')
        return process.returncode
    except subprocess.TimeoutExpired:
        signal_group(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            signal_group(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=3)
        destination.write_text(output + '\nTimed out: ' + ' '.join(command) + '\n',
                               encoding='utf-8')
        return 124
    except KeyboardInterrupt:
        # ros2 run does not reliably forward a signal sent only to its PID.
        # Stop the isolated group, including the audit's TF probe, on operator cancel.
        signal_group(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            signal_group(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=3)
        destination.write_text(output + '\nInterrupted by operator; incomplete capture.\n',
                               encoding='utf-8')
        raise


def signal_group(pid, sig):
    """Ignore a subprocess exiting between a timeout and the cleanup signal."""
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        pass


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class Recording:
    """Persist progress and failures, even when the operator cancels acquisition."""

    def __init__(self, directory, duration, require_perception):
        self.directory = directory
        self.status = {
            'state': 'recording', 'started_utc': utc_now(),
            'mode': 'full_perception' if require_perception else 'localization_only',
            'requested_duration_sec': duration, 'steps': [], 'metadata_errors': [],
            'meaning': 'Evidence capture status, NOT pilot approval or localization acceptance',
        }
        self.save()

    def save(self):
        temporary = self.directory / 'recording_status.json.tmp'
        temporary.write_text(json.dumps(self.status, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.directory / 'recording_status.json')

    def run(self, command, filename, timeout=10):
        step = {'command': command, 'file': filename, 'state': 'running',
                'started_utc': utc_now()}
        self.status['steps'].append(step)
        self.save()
        print(f'Capturing {filename} (timeout {timeout:g}s)', flush=True)
        started = time.monotonic()
        try:
            code = capture(command, self.directory / filename, timeout)
            step.update(returncode=code, state='captured' if code == 0 else 'failed')
            return code
        except KeyboardInterrupt:
            step['state'] = 'interrupted'
            raise
        except Exception as exc:
            step.update(state='error', error=str(exc))
            raise
        finally:
            step['elapsed_sec'] = time.monotonic() - started
            self.save()

    def finish(self, state):
        self.status.update(state=state, finished_utc=utc_now())
        self.save()


def parameter_nodes(require_perception):
    nodes = ['amcl', 'laserMapping', 'nav2_safety_guard', 'collision_monitor',
             'pointcloud_to_laserscan', 'local_costmap/local_costmap',
             'global_costmap/global_costmap', 'static_costmap/static_costmap']
    if require_perception:
        nodes += ['pedestrian_figures', 'pedestrian_traces',
                  'ped_yolo_node',
                  'mid360_lidar_accumulator_node', 'mid360_mask_3d_extractor_node',
                  'pedestrian_tracker_node']
    return nodes


def audit_command(duration, directory, require_perception, lidar_topic='/livox/lidar_local'):
    command = ['ros2', 'run', 'jackal_nav2_bringup', 'tf_localization_audit.py',
               '--duration', str(duration), '--warmup', '3',
               '--output', str(directory / 'audit.json'), '--lidar-topic', lidar_topic]
    if require_perception:
        command.append('--require-perception')
    return command


def summarize_audit(recording, code):
    """Do not equate a zero exit, partial window or failed metadata dump with acceptance."""
    try:
        report = json.loads((recording.directory / 'audit.json').read_text())
        duration = report['duration_sec']
        issues = report['issues']
        if (not isinstance(duration, (int, float)) or not math.isfinite(duration)
                or duration < recording.status['requested_duration_sec'] - 0.1
                or not isinstance(issues, list) or code not in (0, 1)):
            raise ValueError('incomplete duration, malformed report or failed audit process')
        recording.status['audit_duration_sec'] = duration
        recording.status['audit_issues'] = issues
    except (OSError, ValueError, TypeError, KeyError) as exc:
        recording.status['audit_error'] = str(exc)
        return 'incomplete', 2
    if recording.status['metadata_errors'] or any(
            step['state'] != 'captured' for step in recording.status['steps']
            if step['file'] != 'audit.console'):
        return 'incomplete', 2
    return ('recorded_with_findings', 1) if issues or code else ('recorded', 0)


def navigation_topics(nav_cmd_topic='/j100_0519/nav2_cmd_vel', odom_topic='/odom'):
    """Record the action and complete command chain, without raw clouds or images."""
    return [
        '/navigate_to_pose/_action/status', '/navigate_to_pose/_action/feedback',
        '/goal_pose', '/initialpose', '/plan', '/local_plan', '/tf', '/tf_static',
        odom_topic, '/cmd_vel_nav', '/nav2_cmd_vel_unstamped',
        '/nav2/collision_checked_cmd_vel', nav_cmd_topic,
        '/j100_0519/cmd_vel', '/j100_0519/platform/cmd_vel_unstamped',
        '/j100_0519/platform/odom', '/nav2/safety_diagnostics',
    ]


def inspect_navigation_bag(directory, duration, nav_cmd_topic, odom_topic):
    """Reject short/incomplete captures; message counts never imply goal success."""
    metadata = yaml.safe_load((directory / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    elapsed = metadata['duration']['nanoseconds'] * 1e-9
    counts = {entry['topic_metadata']['name']: entry['message_count']
              for entry in metadata['topics_with_message_count']}
    required = ('/tf', odom_topic, nav_cmd_topic, '/nav2/safety_diagnostics')
    missing = [topic for topic in required if counts.get(topic, 0) <= 0]
    if not math.isfinite(elapsed) or elapsed < duration - 0.5 or missing:
        raise ValueError(f'incomplete navigation capture: duration={elapsed}, missing={missing}')
    files = metadata['relative_file_paths']
    if not files or any(not (directory / path).is_file() for path in files):
        raise ValueError('navigation bag storage files missing')
    return {'duration_sec': elapsed, 'topic_message_counts': counts,
            'meaning': 'Capture integrity only; action terminal states and actual stopping '
                       'must be assessed separately. Native RViz goals use the action, '
                       'so /goal_pose may be absent. Log intended test poses separately.'}


class NavigationBag:
    """Own one recorder process group and finalize its evidence on every exit path."""

    def __init__(self, recording, nav_cmd_topic, odom_topic):
        self.recording = recording
        self.nav_cmd_topic, self.odom_topic = nav_cmd_topic, odom_topic
        self.directory = recording.directory / 'navigation_bag'
        self.process = self.stream = None
        command = ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--output',
                   str(self.directory), '--include-hidden-topics'] + navigation_topics(
                       nav_cmd_topic, odom_topic)
        self.step = {'command': command, 'file': 'navigation.console',
                     'state': 'running', 'started_utc': utc_now()}

    def start(self):
        """Wait for storage creation before telling the operator to send test goals."""
        self.recording.status['steps'].append(self.step)
        self.recording.save()
        self.stream = (self.recording.directory / 'navigation.console').open('w')
        self.process = subprocess.Popen(self.step['command'], stdout=self.stream,
                                        stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('navigation bag recorder exited during startup')
            if list(self.directory.glob('*.db3')):
                print('Navigation recording active. Send goals only after readiness and '
                      'physical alignment checks.', flush=True)
                return
            time.sleep(0.1)
        raise RuntimeError('navigation bag recorder startup timed out')

    def stop(self):
        """Flush rosbag and preserve incomplete/interrupted acquisition as a failure."""
        try:
            if self.process:
                if self.process.poll() is None:
                    signal_group(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    signal_group(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
                self.step['returncode'] = self.process.returncode
                if self.process.returncode not in (0, -signal.SIGINT, 130):
                    raise RuntimeError(f'bag recorder exited {self.process.returncode}')
            else:
                raise RuntimeError('bag recorder did not start')
            summary = inspect_navigation_bag(
                self.directory, self.recording.status['requested_duration_sec'],
                self.nav_cmd_topic, self.odom_topic)
            self.recording.status['navigation_capture'] = summary
            self.step['state'] = 'captured'
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
            self.step.update(state='failed', error=str(error))
        finally:
            if self.stream:
                self.stream.close()
            self.step['finished_utc'] = utc_now()
            self.recording.save()


def collect(recording, duration, require_perception, lidar_topic='/livox/lidar_local',
            navigation=False, nav_cmd_topic='/j100_0519/nav2_cmd_vel', odom_topic='/odom'):
    directory = recording.directory
    recording.status['lidar_topic'] = lidar_topic
    recording.status['navigation'] = navigation
    recording.save()
    environment = {name: os.environ.get(name, '') for name in (
        'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
        'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
        'LD_LIBRARY_PATH')}
    (directory/'ros_environment.json').write_text(
        json.dumps(environment, indent=2)+'\n', encoding='utf-8')
    versions = {}
    for package in ('jackal_nav2_bringup', 'moai_nav_viz', 'mid360_bringup',
                    'mid360_perception', 'fast_livo', 'nav2_collision_monitor',
                    'nav2_amcl', 'nav2_costmap_2d', 'tf2_ros', 'rmw_fastrtps_cpp'):
        try:
            share = Path(get_package_share_directory(package))
            versions[package] = {
                'version': ET.parse(share/'package.xml').findtext('version'),
                'share': str(share), 'config_sha256': {}}
            for source in (share/'config').glob('*.yaml'):
                data = source.read_bytes()
                versions[package]['config_sha256'][source.name] = hashlib.sha256(data).hexdigest()
                saved = directory/'config'/package/source.name
                saved.parent.mkdir(parents=True, exist_ok=True)
                saved.write_bytes(data)
        except (LookupError, OSError, ET.ParseError) as exc:
            recording.status['metadata_errors'].append({'package': package, 'error': str(exc)})
            recording.save()
    (directory/'versions.json').write_text(json.dumps(versions, indent=2)+'\n')
    recording.run(['dpkg-query', '-W', 'ros-humble-nav2-*', 'ros-humble-tf2*'],
                  'system_versions.txt')
    recording.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'], 'nodes.txt')
    # Optional helpers: record actual values when present, not a hard dependency.
    try:
        present = set((directory / 'nodes.txt').read_text().splitlines())
    except OSError as exc:
        present = set()
        recording.status['metadata_errors'].append({'file': 'nodes.txt', 'error': str(exc)})
        recording.save()
    for node in ('pointcloud_relay', 'amcl_quality_monitor'):
        if '/' + node in present:
            recording.run(['ros2', 'param', 'dump', '/' + node], node + '.yaml')
    for node in parameter_nodes(require_perception):
        recording.run(['ros2', 'param', 'dump', '/'+node], node.replace('/', '_')+'.yaml')
    if navigation:
        for node in ('bt_navigator', 'planner_server', 'controller_server', 'velocity_smoother'):
            recording.run(['ros2', 'param', 'dump', '/' + node], node + '.yaml')
        if '/cmd_vel_safety_bridge' in present:
            recording.run(['ros2', 'param', 'dump', '/cmd_vel_safety_bridge'], 'bridge.yaml')
        # Preserve the actual installed default tree, not a hardcoded source filename.
        try:
            params = yaml.safe_load((directory / 'bt_navigator.yaml').read_text())
            entry = params.get('/bt_navigator', params.get('bt_navigator', {}))
            tree = Path(entry['ros__parameters']['default_nav_to_pose_bt_xml'])
            (directory / 'navigation_tree.xml').write_bytes(tree.read_bytes())
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            recording.status['metadata_errors'].append({'navigation_tree': str(error)})
    bag = NavigationBag(recording, nav_cmd_topic, odom_topic) if navigation else None
    try:
        if bag:
            bag.start()
        code = recording.run(audit_command(duration, directory, require_perception, lidar_topic),
                             'audit.console', timeout=duration+45)
    finally:
        if bag:
            bag.stop()
    return summarize_audit(recording, code)


def main(argv=None):
    """Create a new evidence directory; never overwrite a previous measurement."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=600.0)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--lidar-topic', default='/livox/lidar_local',
                        help='required cloud input; match bringup lidar_pointcloud_topic')
    parser.add_argument('--localization-only', action='store_true',
                        help='omit perception inputs and parameter queries; '
                        'default is full perception')
    parser.add_argument('--navigation', action='store_true',
                        help='also capture navigation action/TF/odom/velocity evidence in rosbag2; '
                        'does not send goals or enable motion')
    parser.add_argument('--nav-cmd-topic', default='/j100_0519/nav2_cmd_vel')
    parser.add_argument('--odom-topic', default='/odom')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 12:
        parser.error('--duration must be at least 12 seconds')
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=False)
        directory = args.output_dir
    else:
        directory = Path(tempfile.mkdtemp(prefix='nav2_validation_'))
    print(f'Validation artifacts: {directory}', flush=True)
    recording = Recording(directory, args.duration, not args.localization_only)
    try:
        state, code = collect(
            recording, args.duration, not args.localization_only, args.lidar_topic,
            args.navigation, args.nav_cmd_topic, args.odom_topic)
    except KeyboardInterrupt:
        state, code = 'interrupted', 130
    except Exception as exc:
        recording.status['error'] = str(exc)
        state, code = 'incomplete', 2
    recording.finish(state)
    print(f'Recording {state}, exit {code}; saved to {directory}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
