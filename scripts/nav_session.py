#!/usr/bin/env python3
"""Run the validated localization workflow in separately owned terminal sessions."""

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


PROC = Path('/proc')
STOP_REQUESTED = False


def process_info(pid):
    """Read a PID identity including boot and start time; zombies are not live."""
    try:
        folder = PROC / str(pid)
        fields = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {
            'pid': int(pid), 'start': fields[19], 'group': int(fields[2]),
            'boot': (PROC / 'sys/kernel/random/boot_id').read_text().strip(),
            'args': (folder / 'cmdline').read_bytes().decode().rstrip('\0').split('\0'),
        }
    except (OSError, ValueError, IndexError, UnicodeError):
        return None


def same_process(identity):
    """Avoid signalling a PID reused after shutdown or reboot."""
    current = process_info(identity['pid']) if identity else None
    return bool(current and all(current[k] == identity[k] for k in ('pid', 'start', 'boot')))


def stack_role(args):
    """Identify local stack processes by executable/launch tokens, never substrings."""
    names = [Path(a).name for a in args]
    executable_names = names[:2] if names and names[0].startswith('python') else names[:1]
    if 'nav_session.py' in executable_names:
        index = names.index('nav_session.py') + 1
        if len(args) > index and args[index] in ('nav', 'perception'):
            return args[index]
    if 'ros2' in executable_names and 'launch' in args:
        i = args.index('launch')
        pair = args[i + 1:i + 3]
        if pair == ['mid360_bringup', 'perception.launch.py']:
            return 'perception'
        if (len(pair) == 2 and pair[0] == 'jackal_nav2_bringup' and
                pair[1] in ('localization.launch.py', 'nav2.launch.py')):
            return 'nav'
        if pair == ['fast_livo', 'mapping_mid360.launch.py']:
            return 'nav'
    if any(n in executable_names for n in (
            'fastlivo_mapping', 'pointcloud_relay_node', 'amcl', 'controller_server',
            'planner_server', 'bt_navigator', 'nav2_safety_guard.py',
            'stack_stability.py', 'operator_stop.py', 'rviz_overlay_bridge.py',
            'fast_livo_odom_adapter.py', 'collision_monitor', 'velocity_smoother',
            'static_costmap_node', 'map_server', 'pointcloud_to_laserscan_node')):
        return 'nav'
    if any(n in executable_names for n in (
            'ped_yolo_node', 'mid360_mask_3d_extractor_node',
            'mid360_lidar_accumulator_node', 'pedestrian_tracker_node')):
        return 'perception'
    return None


def stack_processes():
    """Inspect local processes without contacting ROS or stopping anything."""
    result = []
    for path in PROC.iterdir():
        if path.name.isdigit():
            info = process_info(path.name)
            if info and stack_role(info['args']):
                result.append({**info, 'role': stack_role(info['args'])})
    return result


def runtime_directory():
    """Use a fixed per-user lock location, independent of the log directory."""
    path = Path('/tmp') / f'jackal-nav2-session-{os.getuid()}'
    path.mkdir(mode=0o700, exist_ok=True)
    stat = path.lstat()
    if path.is_symlink() or stat.st_uid != os.getuid() or stat.st_mode & 0o077:
        raise RuntimeError(f'Unsafe runtime directory: {path}')
    return path


def read_state(runtime, role):
    """Read the last state, including stopped sessions."""
    try:
        return json.loads((runtime / f'{role}.json').read_text())
    except FileNotFoundError:
        return {}


def save_state(runtime, role, state):
    """Atomically preserve ownership and the durable log path."""
    temporary = runtime / f'{role}.json.tmp'
    temporary.write_text(json.dumps(state, indent=2) + '\n')
    temporary.replace(runtime / f'{role}.json')
    if state.get('log_dir'):
        durable = Path(state['log_dir']) / 'session.json'
        temporary = durable.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(durable)


def network_status():
    """Read the owning package's policy without importing its source or changing sysctls."""
    result = subprocess.run(
        ['ros2', 'run', 'jackal_network_bringup', 'network_preflight.py', '--check'],
        capture_output=True, text=True, timeout=10, check=False)
    try:
        report = json.loads(result.stdout)
    except (ValueError, TypeError) as error:
        raise RuntimeError('Install/rebuild jackal_network_bringup; network status unavailable: '
                           + result.stderr.strip()) from error
    if result.returncode not in (0, 1) or not isinstance(report.get('ready'), bool):
        raise RuntimeError(f'Network status failed: {report}')
    return report


def require_environment():
    """Reject a different network or an unprepared kernel before starting nodes."""
    if os.environ.get('ROS_DOMAIN_ID') != '1' or os.environ.get('JACKAL_NETWORK_ROLE') != 'laptop':
        raise RuntimeError('Source the workspace and network_env.sh laptop first (domain 1).')
    report = network_status()
    if not report['ready']:
        raise RuntimeError('Network policy is not ready: ' + json.dumps(report) +
                           '; inspect the network-owned report before changing settings.')


def stop_role(runtime, role, timeout=35.0):
    """Request shutdown only from our matching supervisor; never kill by name."""
    state = read_state(runtime, role)
    owner = state.get('owner')
    if not same_process(owner):
        return
    os.kill(owner['pid'], signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while same_process(owner) and time.monotonic() < deadline:
        time.sleep(0.1)
    if same_process(owner):
        raise RuntimeError(f'{role} has not stopped; inspect {state.get("log_dir")}')


def group_alive(group):
    """Check our own child session, including children left after launch exits."""
    for path in PROC.iterdir():
        if path.name.isdigit():
            info = process_info(path.name)
            if info and info['group'] == group:
                return True
    return False


def stop_child(child):
    """Let launch forward SIGINT once, then reap any remaining owned processes."""
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
    for delay, sig in ((12.0, signal.SIGTERM), (3.0, signal.SIGKILL)):
        deadline = time.monotonic() + delay
        while group_alive(child.pid) and time.monotonic() < deadline:
            child.poll()
            time.sleep(0.1)
        if not group_alive(child.pid):
            break
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
    child.wait(timeout=5)
    if group_alive(child.pid):
        raise RuntimeError(f'Owned process group {child.pid} still has live children')


def initial_pose_gate():
    """Reuse the localization gate, additionally requiring map TF after initial pose."""
    return [
        'ros2', 'run', 'jackal_nav2_bringup', 'topic_ready_gate.py', '--ros-args',
        '-p', 'topic:=/scan', '-p', 'message_type:=scan',
        '-p', 'expected_frame:=base_link', '-p', 'odom_topic:=/odom',
        '-p', 'require_localization:=true', '-p', 'require_map_to_odom:=true',
        '-p', 'timeout:=60.0', '-p', 'settle:=5.0',
    ]


def supervise(role, commands, runtime, log_dir, parent=None):
    """Own one foreground session; perception depends on the exact Nav2 session."""
    global STOP_REQUESTED
    STOP_REQUESTED = False

    def request_stop(*_):
        global STOP_REQUESTED
        STOP_REQUESTED = True

    old_handlers = {s: signal.signal(s, request_stop)
                    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    state = {'role': role, 'owner': process_info(os.getpid()), 'log_dir': str(log_dir),
             'status': 'starting', 'commands': commands,
             'started_at': datetime.now().astimezone().isoformat()}
    save_state(runtime, role, state)
    from runtime_resource_audit import ResourceRecorder
    resources = ResourceRecorder(
        log_dir / 'resources.jsonl', lambda: [p['pid'] for p in stack_processes()],
        log_dir.name, log_dir).start()
    child = None
    code = 0
    try:
        for index, command in enumerate(commands):
            if STOP_REQUESTED or (parent and not same_process(parent)):
                code = 130
                break
            state.update(status='running', stage=index + 1, command=command)
            save_state(runtime, role, state)
            print(f'[{role}] {shlex.join(command)}', flush=True)
            print(f'[{role}] logs: {log_dir}; running does not mean ready.', flush=True)
            env = {**os.environ, 'ROS_LOG_DIR': str(log_dir / 'ros'),
                   'OVERRIDE_LAUNCH_PROCESS_OUTPUT': 'both',
                   'JACKAL_NAV_SESSION_DIR': str(log_dir)}
            # Preserve launch child stdout as well as rcl logging after terminal loss.
            child = subprocess.Popen(command, env=env, start_new_session=True)
            state['child'] = process_info(child.pid)
            save_state(runtime, role, state)
            while child.poll() is None:
                if STOP_REQUESTED or (parent and not same_process(parent)):
                    code = 130
                    break
                time.sleep(0.2)
            if role == 'nav':
                stop_role(runtime, 'perception')
            stop_child(child)
            code = code or child.returncode
            if (log_dir / 'failure.json').is_file():
                state['failure'] = json.loads((log_dir / 'failure.json').read_text())
                code = code or 1
            child = None
            if code:
                break
        return code
    except BaseException:
        code = 1
        raise
    finally:
        cleanup_error = None
        try:
            if role == 'nav':
                stop_role(runtime, 'perception')
        except (OSError, RuntimeError) as error:
            cleanup_error = error
        try:
            if child is not None:
                stop_child(child)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            cleanup_error = error
        try:
            resources.close()
            state['resource_audit_error'] = resources.error
            state.update(status='cleanup_failed' if cleanup_error else 'stopped', returncode=code)
            state['ended_at'] = datetime.now().astimezone().isoformat()
            if cleanup_error:
                state['cleanup_error'] = str(cleanup_error)
            save_state(runtime, role, state)
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
        if cleanup_error:
            raise cleanup_error


def navigation_command(args):
    """One explicit profile/motion path for both stationary and attended sessions."""
    command = [
        'ros2', 'launch', 'jackal_nav2_bringup', 'nav2.launch.py',
        f'map:={args.map.expanduser().resolve()}',
        f'enable_motion:={str(args.enable_motion).lower()}', 'managed_session:=true',
        f'input_timeout:={args.input_timeout}',
        f'localization_timeout:={args.localization_timeout}',
        f'ready_settle:={args.ready_settle}',
        f'stability_timeout:={args.stability_timeout}',
        f'stability_settle:={args.stability_settle}',
        f'use_pedestrian_figures:={str(args.pedestrian_viz).lower()}',
        f'use_pedestrian_traces:={str(args.pedestrian_viz).lower()}',
    ]
    if args.profile_dir:
        for key, filename in (('params_file', 'nav2.yaml'), ('safety_params_file', 'safety.yaml'),
                              ('operator_params_file', 'operator.yaml')):
            path = args.profile_dir.expanduser().resolve() / filename
            if not path.is_file():
                raise RuntimeError(f'Navigation profile missing: {path}')
            command.append(f'{key}:={path}')
    return command


def start(args, runtime):
    """Lock, inspect conflicts, then start the unchanged validated launch commands."""
    require_environment()
    role = args.action
    with (runtime / f'{role}.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f'{role} session is already running') from error
        conflicts = [p for p in stack_processes() if p['pid'] != os.getpid() and
                     (p['role'] == role or role == 'nav')]
        if conflicts:
            raise RuntimeError(f'Existing stack processes; stop in their original terminals: '
                               f'{[(p["pid"], p["role"]) for p in conflicts]}')
        parent = None
        if role == 'perception':
            parent = read_state(runtime, 'nav').get('owner')
            if not same_process(parent):
                raise RuntimeError('Start Nav2 with nav_session.py nav first.')
            if not args.initial_pose_confirmed:
                raise RuntimeError('Set initial pose, check scan/map alignment, then use '
                                   '--initial-pose-confirmed.')
        elif not args.map.expanduser().is_file():
            raise RuntimeError(f'Map YAML does not exist: {args.map}')
        log_dir = args.log_root.expanduser().resolve() / (
            datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + role)
        log_dir.mkdir(parents=True, exist_ok=False)
        if role == 'nav':
            commands = [navigation_command(args)]
        else:
            from ament_index_python.packages import get_package_share_directory
            from prepare_perception_config import perception_command, prepare
            profile = prepare(Path(get_package_share_directory('mid360_bringup')),
                              log_dir / 'perception')
            commands = [initial_pose_gate(), perception_command(profile)]
        return supervise(role, commands, runtime, log_dir, parent)


def main(argv=None):
    """Expose status, separate foreground launch sessions and ordered shutdown."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'nav', 'perception', 'status', 'stop'])
    parser.add_argument('--map', type=Path)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--profile-dir', type=Path, help='prepared nav2/safety/operator YAMLs')
    parser.add_argument('--pedestrian-viz', action='store_true')
    parser.add_argument('--input-timeout', type=float, default=60.0)
    parser.add_argument('--localization-timeout', type=float, default=90.0)
    parser.add_argument('--ready-settle', type=float, default=5.0)
    parser.add_argument('--stability-timeout', type=float, default=600.0)
    parser.add_argument('--stability-settle', type=float, default=3.0)
    parser.add_argument('--initial-pose-confirmed', action='store_true')
    parser.add_argument('--log-root', type=Path, default=Path.home() / '.ros/nav_sessions')
    args = parser.parse_args(argv)
    if args.action == 'nav' and args.map is None:
        parser.error('nav requires --map')
    import math
    for limit, hold in ((args.input_timeout, args.ready_settle),
                        (args.localization_timeout, args.ready_settle),
                        (args.stability_timeout, args.stability_settle)):
        if not all(math.isfinite(v) and v > 0 for v in (limit, hold)) or hold >= limit:
            parser.error('each settle time must be positive and less than its timeout')
    if args.enable_motion and args.action != 'nav':
        parser.error('--enable-motion applies only to nav')
    try:
        runtime = runtime_directory()
        if args.action in ('check', 'status'):
            print(json.dumps({
                'network': network_status(),
                'network_namespace': os.readlink('/proc/self/ns/net'),
                'sessions': {r: {**read_state(runtime, r),
                                 'owner_alive': same_process(read_state(runtime, r).get('owner'))}
                             for r in ('nav', 'perception')},
                'stack_processes': stack_processes(),
            }, indent=2))
            if args.action == 'check':
                require_environment()
            return 0
        if args.action == 'stop':
            for role in ('perception', 'nav'):
                stop_role(runtime, role)
            remaining = stack_processes()
            if remaining:
                raise RuntimeError('Unmanaged processes remain; stop their original terminals: '
                                   f'{[p["pid"] for p in remaining]}')
            print('Managed stack stopped. Kernel settings were not changed.')
            return 0
        return start(args, runtime)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f'nav_session: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
