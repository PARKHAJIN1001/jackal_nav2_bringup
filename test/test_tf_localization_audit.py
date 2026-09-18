"""Tests for the read-only TF ownership and timestamp checks."""

import copy
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped
import pytest
from sensor_msgs.msg import LaserScan


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'tf_localization_audit', ROOT / 'scripts' / 'tf_localization_audit.py'
)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
RECORD_SPEC = importlib.util.spec_from_file_location(
    'record_navigation_validation', ROOT / 'scripts' / 'record_navigation_validation.py'
)
RECORD = importlib.util.module_from_spec(RECORD_SPEC)
RECORD_SPEC.loader.exec_module(RECORD)


def test_localization_only_audit_does_not_subscribe_remote_camera():
    topics = dict(AUDIT.input_topics())
    assert '/livox/lidar_local' in topics
    assert '/scan' in topics
    assert '/camera/camera/color/image_raw' not in topics
    assert '/ped_tracking' not in topics
    full = dict(AUDIT.input_topics(require_perception=True))
    assert set(topics) < set(full)
    assert '/camera/camera/color/image_raw' in full
    assert '/ped_tracking' in full


@pytest.mark.parametrize('needs_kill', [False, True])
def test_cancelled_capture_stops_its_process_group(monkeypatch, tmp_path, needs_kill):
    process = Mock(pid=43210)
    replies = [KeyboardInterrupt()]
    if needs_kill:
        replies.append(subprocess.TimeoutExpired(['probe'], 3))
    replies.append(('partial evidence', None))
    process.communicate.side_effect = replies
    monkeypatch.setattr(RECORD.subprocess, 'Popen', lambda *a, **k: process)
    signals = []
    monkeypatch.setattr(RECORD.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    path = tmp_path / 'interrupted.console'
    with pytest.raises(KeyboardInterrupt):
        RECORD.capture(['probe'], path)
    assert signals[0] == (43210, signal.SIGINT)
    assert len(signals) == (2 if needs_kill else 1)
    if needs_kill:
        assert signals[1] == (43210, signal.SIGKILL)
    assert 'partial evidence' in path.read_text()
    assert 'incomplete capture' in path.read_text()


def test_recording_modes_do_not_query_or_subscribe_perception_in_baseline(tmp_path):
    base = RECORD.parameter_nodes(False)
    full = RECORD.parameter_nodes(True)
    assert 'amcl' in base and 'laserMapping' in base
    assert not any('pedestrian' in node or 'mid360' in node for node in base)
    assert 'pedestrian_traces' in full and 'pedestrian_tracker_node' in full
    assert '--require-perception' not in RECORD.audit_command(600, tmp_path, False)
    assert '--require-perception' in RECORD.audit_command(600, tmp_path, True)


@pytest.mark.parametrize('returncode', [0, 124])
def test_recording_progress_and_command_failure_persist(monkeypatch, tmp_path, returncode):
    monkeypatch.setattr(RECORD, 'capture', lambda *a: returncode)
    recording = RECORD.Recording(tmp_path, 600, False)
    assert recording.run(['fake-command'], 'sample.txt') == returncode
    status = json.loads((tmp_path / 'recording_status.json').read_text())
    assert status['mode'] == 'localization_only'
    step = status['steps'][0]
    assert step['returncode'] == returncode
    assert step['state'] == ('captured' if returncode == 0 else 'failed')
    assert step['elapsed_sec'] >= 0


def test_recording_cancel_marks_incomplete_step_and_main(monkeypatch, tmp_path):
    def cancel(*args):
        raise KeyboardInterrupt()

    monkeypatch.setattr(RECORD, 'capture', cancel)
    recording = RECORD.Recording(tmp_path, 600, False)
    with pytest.raises(KeyboardInterrupt):
        recording.run(['fake-command'], 'sample.txt')
    assert recording.status['steps'][0]['state'] == 'interrupted'
    monkeypatch.setattr(RECORD, 'collect', cancel)
    directory = tmp_path / 'new_recording'
    assert RECORD.main(['--localization-only', '--output-dir', str(directory)]) == 130
    status = json.loads((directory / 'recording_status.json').read_text())
    assert status['state'] == 'interrupted'
    assert status['finished_utc']


@pytest.mark.parametrize('code,duration,issues,metadata,state,exit_code', [
    (0, 600., [], False, 'recorded', 0),
    (1, 600., ['TF fault'], False, 'recorded_with_findings', 1),
    (0, 600., ['TF fault'], False, 'recorded_with_findings', 1),
    (0, 600., [], True, 'incomplete', 2),
    (0, 20., [], False, 'incomplete', 2),
    (124, 600., [], False, 'incomplete', 2),
    (0, float('nan'), [], False, 'incomplete', 2),
])
def test_recording_does_not_equate_partial_or_failed_capture_with_success(
        tmp_path, code, duration, issues, metadata, state, exit_code):
    recording = RECORD.Recording(tmp_path, 600, True)
    if metadata:
        recording.status['steps'].append({'file': 'amcl.yaml', 'state': 'failed'})
    (tmp_path / 'audit.json').write_text(json.dumps({'duration_sec': duration, 'issues': issues}))
    assert RECORD.summarize_audit(recording, code) == (state, exit_code)


def test_missing_audit_and_missing_package_are_explicit(monkeypatch, tmp_path):
    recording = RECORD.Recording(tmp_path, 600, False)
    assert RECORD.summarize_audit(recording, 0) == ('incomplete', 2)

    def missing(package):
        raise LookupError('not installed')

    monkeypatch.setattr(RECORD, 'get_package_share_directory', missing)
    monkeypatch.setattr(RECORD, 'capture', lambda *args: 0)
    (tmp_path / 'audit.json').write_text(json.dumps({'duration_sec': 600, 'issues': []}))
    assert RECORD.collect(recording, 600, False) == ('incomplete', 2)
    assert len([e for e in recording.status['metadata_errors'] if 'package' in e]) == 10
    assert any(e.get('file') == 'nodes.txt' for e in recording.status['metadata_errors'])


def test_cleanup_handles_process_exiting_before_signal(monkeypatch):
    def already_gone(*args):
        raise ProcessLookupError()

    monkeypatch.setattr(RECORD.os, 'killpg', already_gone)
    RECORD.signal_group(12345, signal.SIGINT)


def edge(parent, child, owner, writer, topic='/tf', age=0.03):
    return {
        'parent': parent,
        'child': child,
        'owners': [owner],
        'writers': [writer],
        'topic': topic,
        'age_min_sec': age,
        'age_max_sec': age,
        'latest_age_sec': age,
        'invalid_quaternion': 0,
        'stamp_regressions': 0,
    }


@pytest.fixture
def edges():
    return [
        edge('map', 'odom', '/amcl', 'amcl', age=-0.9),
        edge('odom', 'base_link', '/laserMapping', 'fast'),
        edge(
            'base_link',
            'livox_frame',
            '/mount',
            'mount',
            topic='/tf_static',
            age=10000,
        ),
    ]


def test_expected_tree_accepts_future_amcl_and_old_static(edges):
    assert AUDIT.assess_edges(edges) == []


def test_platform_namespace_is_independent(edges):
    edges.append(edge('odom', 'base_link', '/ekf', 'ekf', '/j100_0519/tf'))
    assert AUDIT.assess_edges(edges) == []


def test_duplicate_writers_with_same_node_name_are_detected(edges):
    edges[1]['writers'].append('fast_duplicate')
    assert any('2 DDS writers' in issue for issue in AUDIT.assess_edges(edges))


@pytest.mark.parametrize(
    'field,value',
    [
        ('age_min_sec', -0.2),
        ('age_max_sec', 0.8),
        ('latest_age_sec', 3.0),
    ],
)
def test_fast_timing_faults_are_detected(edges, field, value):
    edges[1][field] = value
    assert any('timestamp' in issue for issue in AUDIT.assess_edges(edges))


def test_missing_and_wrong_owner(edges):
    assert any(
        'missing map -> odom' in i for i in AUDIT.assess_edges(edges[1:])
    )
    edges[1]['owners'] = ['/ekf']
    assert any(
        'expected /laserMapping' in i for i in AUDIT.assess_edges(edges)
    )


def test_static_dynamic_collision_and_multiple_parents(edges):
    collision = copy.deepcopy(edges[1])
    collision['topic'] = '/tf_static'
    edges.append(collision)
    assert any('must be dynamic' in i for i in AUDIT.assess_edges(edges))
    edges.append(edge('map', 'base_link', '/other', 'other'))
    assert any('multiple parents' in i for i in AUDIT.assess_edges(edges))


def test_cycles_invalid_quaternions_and_regressions(edges):
    edges.append(edge('livox_frame', 'map', '/bad', 'bad'))
    edges[0]['invalid_quaternion'] = 1
    edges[1]['stamp_regressions'] = 1
    issues = AUDIT.assess_edges(edges)
    assert any('cycle' in i for i in issues)
    assert any('backwards timestamp' in i for i in issues)


def test_wall_score_handles_rotated_map_origin_and_outside_endpoints():
    half_sqrt = 0.5**0.5
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=half_sqrt, w=half_sqrt)
    grid = SimpleNamespace(
        data=[0, 0, 100, 0, 0, 0, 0, 0, 0],
        header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(
            width=3,
            height=3,
            resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=-1.0), orientation=quaternion
            ),
        ),
    )
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.5, y=0.0), rotation=quaternion
        )
    )
    dummy = SimpleNamespace(
        map=grid,
        buffer=SimpleNamespace(lookup_transform=lambda *args: transform),
    )
    # One endpoint lies exactly on a wall; the second is outside the map.
    scan = LaserScan(
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        ranges=[1.1, 9.0, float('inf')],
    )
    score = AUDIT.Audit.score_scans(dummy, [scan])
    assert score['endpoints'] == 2
    assert score['inside_map'] == 1
    assert score['median_m'] == 0.0
    assert score['within_0_15m_fraction'] == 0.5


def test_wall_score_reports_missing_map():
    assert 'unavailable' in AUDIT.Audit.score_scans(
        SimpleNamespace(map=None), []
    )


@pytest.mark.parametrize(
    'height,expected_count',
    [
        (0.001, 0),
        (0.95, 1),
        (-0.95, 1),
        (float('nan'), 1),
    ],
)
def test_planar_height_detects_stale_odom_origin(height, expected_count):
    transform = {
        'map->base_link': {'transform': {'translation': {'z': height}}}
    }
    assert len(AUDIT.assess_planar_height(transform)) == expected_count


def test_exact_time_checks_count_entire_run_not_final_ring():
    checks = AUDIT.ExactTimeChecks()
    available = SimpleNamespace(can_transform=lambda *args: True)
    for i in range(1000):
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp.sec = i + 1
        checks.add(scan, float(i), 'initialization' if i < 30 else 'steady')
        checks.process(available, float(i))
    assert checks.totals()['odom']['checked_scans'] == 1000
    assert checks.phases['steady']['map']['transformable_scans'] == 970
    assert not checks.pending


def test_exact_time_tf_wait_failure_and_zero_stamp_are_counted():
    checks = AUDIT.ExactTimeChecks()
    scan = LaserScan()
    scan.header.frame_id = 'base_link'
    scan.header.stamp.sec = 10
    checks.add(scan, 0.0, 'steady')
    missing = SimpleNamespace(can_transform=lambda *args: False)
    checks.process(missing, 0.4)
    assert checks.pending
    checks.process(missing, 0.5)
    assert checks.totals()['odom'] == {
        'checked_scans': 1, 'transformable_scans': 0}
    scan.header.stamp.sec = 0
    checks.add(scan, 1.0, 'restart_candidate')
    checks.process(SimpleNamespace(can_transform=lambda *args: True), 1.6)
    assert checks.totals()['map'] == {
        'checked_scans': 2, 'transformable_scans': 0}


def _healthy_samples(lidar_topic='/livox/lidar_local'):
    samples = {
        topic: {
            'count': 20, 'last_stamp_sec': 99.9, 'latest_age_sec': 0.1,
            'receipt_silence_sec': 0.01, 'last_receipt_monotonic': 49.99,
        }
        for topic, _ in AUDIT.input_topics(lidar_topic=lidar_topic)
    }
    # A latched AMCL pose may be much older than the continuous streams.
    samples['/amcl_pose'].update(last_stamp_sec=1.0, latest_age_sec=99.0)
    return samples


def test_event_driven_pose_does_not_require_continuous_republication():
    assert AUDIT.assess_input_streams(_healthy_samples()) == []


@pytest.mark.parametrize('age', [2.0, -0.2, float('nan')])
def test_recent_receipt_cannot_hide_invalid_measurement_age(age):
    samples = _healthy_samples()
    samples['/scan']['latest_age_sec'] = age
    issues = AUDIT.assess_input_streams(samples)
    assert any('/scan measurement timestamp' in issue for issue in issues)


def test_missing_required_cloud_and_custom_cloud_routing():
    samples = _healthy_samples('/custom/cloud')
    assert AUDIT.assess_input_streams(samples, lidar_topic='/custom/cloud') == []
    assert 'missing /livox/lidar_local sample' in AUDIT.assess_input_streams(samples)
    command = RECORD.audit_command(60, ROOT, False, '/custom/cloud')
    assert command[command.index('--lidar-topic') + 1] == '/custom/cloud'


def test_recovered_stream_still_reports_faults_during_steady_measurement():
    samples = _healthy_samples()
    samples['/odom']['phases'] = {'steady': {
        'age_min_sec': 0.01, 'age_max_sec': 2.0,
        'interarrival_max_sec': 1.0, 'stamp_regressions': 1,
    }}
    issues = AUDIT.assess_input_streams(samples)
    assert any('/odom measurement age' in issue for issue in issues)
    assert any('/odom receipt gap' in issue for issue in issues)
    assert any('/odom has backwards timestamp' in issue for issue in issues)


def test_full_perception_rejects_old_measurements_with_fresh_delivery():
    samples = _healthy_samples()
    for topic, _ in AUDIT.input_topics(require_perception=True):
        samples.setdefault(topic, dict(samples['/scan']))
    samples['/ped_tracking']['latest_age_sec'] = 2.0
    assert any('/ped_tracking measurement timestamp' in issue
               for issue in AUDIT.assess_input_streams(samples, require_perception=True))


def test_report_requires_map_even_with_successful_tf_and_fresh_inputs(edges):
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    counts = {target: {'checked_scans': 20, 'transformable_scans': 20}
              for target in ('map', 'odom')}
    node = SimpleNamespace(
        map=None, scans=[], samples=_healthy_samples(),
        measurement_end_ros=100.0, measurement_end_monotonic=50.0, start=38.0,
        warmup=3.0, require_perception=False, lidar_topic='/livox/lidar_local',
        phase_events=[],
        exact_checks=SimpleNamespace(
            totals=lambda: counts, phases={'steady': counts}, failures=[]),
        buffer=SimpleNamespace(
            lookup_transform=lambda *args: transform, can_transform=lambda *args: True),
        score_scans=lambda scans: {'unavailable': 'no /map received'},
    )
    report = AUDIT.Audit.report(node, edges)
    assert report['issues'] == ['missing /map sample']


def test_tf_fault_identifies_edge_instead_of_ambiguous_child_name(edges):
    edges[0]['stamp_regressions'] = 1
    assert 'map -> odom has invalid quaternion or backwards timestamp' in (
        AUDIT.assess_edges(edges))
