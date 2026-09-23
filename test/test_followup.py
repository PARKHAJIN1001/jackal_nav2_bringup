"""Offline regressions for the September 14 buffering follow-up."""

import ast
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_LOG_DIR', '/tmp/jackal_nav2_bringup_test_logs')


def load(name, directory='scripts'):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), ROOT / directory / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


QUALITY = load('amcl_quality_monitor.py')
PROFILE = load('prepare_perception_config.py')
RESOURCE = load('runtime_resource_audit.py')


def test_legacy_alias_is_completely_eliminated():
    assert not (ROOT / 'launch/bringup.launch.py').exists()


def test_quality_never_replays_pose_even_after_many_degraded_updates():
    published = []
    node = SimpleNamespace(
        position_limit=0.8, yaw_limit=0.8, last_warning=-float('inf'),
        publisher=SimpleNamespace(publish=published.append),
        get_logger=lambda: SimpleNamespace(warning=lambda text: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1000000000, to_msg=lambda: PoseWithCovarianceStamped().header.stamp)))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    original = deepcopy(msg)
    QUALITY.AmclQualityMonitor.on_pose(node, msg)
    assert msg == original
    msg.pose.covariance[0] = 10.0
    for _ in range(100):
        QUALITY.AmclQualityMonitor.on_pose(node, msg)
    assert len(published) == 101
    assert published[-1].status[0].level == DiagnosticStatus.WARN
    assert not hasattr(node, 'last_good_pose')
    # Enforce the only publisher's message type and no legacy /initialpose writer.
    tree = ast.parse((ROOT / 'scripts/amcl_quality_monitor.py').read_text())
    pubs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_publisher']
    assert len(pubs) == 1 and pubs[0].args[0].id == 'DiagnosticArray'
    assert not (ROOT / 'scripts/amcl_recovery_monitor.py').exists()
    launch = (ROOT / 'launch/localization.launch.py').read_text()
    assert "executable='amcl_recovery_monitor.py'" not in launch
    assert "'use_sim_time': use_sim_time" in launch


@pytest.mark.parametrize('index,value,level', [
    (0, 0.1, DiagnosticStatus.OK), (7, 0.9, DiagnosticStatus.WARN),
    (35, 0.9, DiagnosticStatus.WARN), (35, -0.1, DiagnosticStatus.ERROR),
    (3, float('nan'), DiagnosticStatus.ERROR), (0, float('inf'), DiagnosticStatus.ERROR),
])
def test_quality_covariance(index, value, level):
    cov = [0.0] * 36
    cov[index] = value
    assert QUALITY.assess_covariance(cov, 'map')[0] == level
    assert QUALITY.assess_covariance(cov, 'odom')[0] == DiagnosticStatus.ERROR


def test_profile_preserves_tuning_and_refuses_missing_parameter():
    source = {'node': {'ros__parameters': {'a': 1, 'b': {'nested': 2}}}}
    before = deepcopy(source)
    assert PROFILE.nav2_profile(source, 'node', {'a': 5})['node']['ros__parameters'] == {
        'a': 5, 'b': {'nested': 2}}
    assert source == before
    with pytest.raises(ValueError, match='unsupported'):
        PROFILE.nav2_profile(source, 'node', {'missing': 1})
    with pytest.raises(ValueError, match='ros__parameters'):
        PROFILE.nav2_profile({}, 'node', {})


def test_profile_writes_manifest_never_overwrites(tmp_path):
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    for filename, node, params in (
        ('lidar_preprocess.yaml', 'mid360_lidar_accumulator_node', {'input_cloud_topic': '/raw'}),
        ('mask_3d_extractor.yaml', 'mid360_mask_3d_extractor_node',
         {'tracking_frame': 'base_link', 'use_latest_tf_fallback': True}),
    ):
        (share / 'config' / filename).write_text(
            yaml.safe_dump({node: {'ros__parameters': params}}))
    output = PROFILE.prepare(share, tmp_path / 'output')
    manifest = json.loads((output / 'manifest.json').read_text())
    assert len(manifest['sources']['lidar_preprocess.yaml']['sha256']) == 64
    assert 'lidar_input_topic:=' not in ' '.join(PROFILE.perception_command(output))
    assert 'use_sim_time:=false' in PROFILE.perception_command(output)
    assert 'use_sim_time:=true' in PROFILE.perception_command(output, use_sim_time=True)
    with pytest.raises(FileExistsError):
        PROFILE.prepare(share, output)
    for invalid in ('relative/topic', '/topic with spaces', ''):
        with pytest.raises(ValueError, match='invalid absolute'):
            PROFILE.prepare(share, tmp_path / 'invalid', invalid)
        assert not (tmp_path / 'invalid').exists()


def test_resource_counters_and_pid_identity(tmp_path):
    text = 'Udp: InDatagrams InErrors RcvbufErrors\nUdp: 120 5 4\n'
    assert RESOURCE.udp_counters(text)['RcvbufErrors'] == 4
    assert RESOURCE.counter_delta({'RcvbufErrors': 10}, {'RcvbufErrors': 13}) == {
        'RcvbufErrors': 3}
    assert RESOURCE.counter_delta({'RcvbufErrors': 10}, {'RcvbufErrors': 2}) == {
        'RcvbufErrors': None}
    process = tmp_path / '123'
    process.mkdir()
    process.joinpath('stat').write_text('123 (name with ) parens) S ' + '0 ' * 18 + '777 0 0')
    process.joinpath('status').write_text('VmRSS: 1024 kB\nVmSwap: 512 kB\n')
    assert RESOURCE.process_snapshot(123, tmp_path) == {
        'pid': 123, 'start_ticks': 777, 'memory_kib': {'VmRSS': 1024, 'VmSwap': 512}}
    tmp_path.joinpath('net').mkdir()
    tmp_path.joinpath('net/snmp').write_text(text)
    tmp_path.joinpath('meminfo').write_text('MemAvailable: 2000 kB\n')
    assert not RESOURCE.sample([123], {}, tmp_path)['errors']
    assert 'PID reused' in RESOURCE.sample([123], {123: 55}, tmp_path)['errors'][0]['error']


def test_socket_drop_queues_and_host_session_join(tmp_path):
    socket = RESOURCE.udp_sockets(
        'sl local_address rem_address st tx_queue rx_queue tr tm->when '
        'retrnsmt uid timeout inode\n'
        '7: 0100007F:1CE8 00000000:0000 07 00000010:00000020 00:00000000 '
        '00000000 1000 0 555 2 0000000000000000 9\n')[0]
    assert socket['inode'] == 555 and socket['drops'] == 9
    assert socket['rx_queue_bytes'] == 32 and socket['tx_queue_bytes'] == 16
    paths = [tmp_path / 'laptop.jsonl', tmp_path / 'nuc.jsonl']
    for path in paths:
        rows = [{'session_id': 'trial-1', 'wall_sec': i, 'udp': {'InErrors': 10 + i}}
                for i in range(2)]
        path.write_text('\n'.join(json.dumps(row) for row in rows))
    result = RESOURCE.compare_hosts(paths)
    assert all(host['udp_delta']['InErrors'] == 1 for host in result['hosts'])
    paths[1].write_text(paths[1].read_text().replace('trial-1', 'another-trial'))
    with pytest.raises(ValueError, match='Session IDs differ'):
        RESOURCE.compare_hosts(paths)
