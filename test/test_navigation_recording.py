"""Evidence acquisition integrity; a complete bag is not a successful goal."""

import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import record_navigation_validation as record  # noqa: E402,I100


def bag_metadata(directory, duration=15.0, missing=None):
    directory.mkdir(exist_ok=True)
    (directory / 'data.db3').write_bytes(b'test fixture')
    topics = ['/tf', '/odom', '/test/output', '/nav2/safety_diagnostics']
    metadata = {'rosbag2_bagfile_information': {
        'duration': {'nanoseconds': int(duration * 1e9)},
        'relative_file_paths': ['data.db3'],
        'topics_with_message_count': [{'topic_metadata': {'name': topic}, 'message_count': 10}
                                      for topic in topics if topic != missing],
    }}
    (directory / 'metadata.yaml').write_text(yaml.safe_dump(metadata))


def test_capture_integrity_does_not_require_a_goal_or_claim_success(tmp_path):
    bag_metadata(tmp_path)
    result = record.inspect_navigation_bag(tmp_path, 15.0, '/test/output', '/odom')
    assert result['duration_sec'] == pytest.approx(15.0)
    assert 'Capture integrity only' in result['meaning']
    topics = record.navigation_topics('/test/output')
    assert '/navigate_to_pose/_action/status' in topics
    assert '/navigate_to_pose/_action/feedback' in topics
    assert '/test/output' in topics
    assert '/livox/lidar_local' not in topics


@pytest.mark.parametrize('duration,missing', [
    (0, None), (13, None), (15, '/odom'), (15, '/test/output'),
])
def test_short_or_missing_required_stream_is_incomplete(tmp_path, duration, missing):
    bag_metadata(tmp_path, duration, missing)
    with pytest.raises(ValueError, match='incomplete'):
        record.inspect_navigation_bag(tmp_path, 15.0, '/test/output', '/odom')


def test_failed_navigation_step_prevents_recorded_state(tmp_path):
    recording = record.Recording(tmp_path, 15, False)
    (tmp_path / 'audit.json').write_text(json.dumps({'duration_sec': 15, 'issues': []}))
    recording.status['steps'].append({'file': 'navigation.console', 'state': 'failed'})
    assert record.summarize_audit(recording, 0) == ('incomplete', 2)


def test_bag_stop_sends_interrupt_and_validates_final_storage(tmp_path, monkeypatch):
    recording = record.Recording(tmp_path, 15, False)
    bag = record.NavigationBag(recording, '/test/output', '/odom')
    bag_metadata(bag.directory)
    signals = []
    monkeypatch.setattr(record, 'signal_group', lambda pid, sig: signals.append((pid, sig)))
    bag.process = SimpleNamespace(
        pid=99999, poll=lambda: None, wait=lambda timeout: 0, returncode=0)
    bag.stop()
    assert signals == [(99999, signal.SIGINT)]
    assert bag.step['state'] == 'captured'
    (bag.directory / 'data.db3').unlink()
    bag.stop()
    assert bag.step['state'] == 'failed'
