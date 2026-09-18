"""Regression coverage for preflight failures without contacting a robot."""

import importlib.util
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('pilot_preflight', ROOT / 'tools/pilot_preflight.py')
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


def test_local_commands_support_source_and_keep_failure_status(tmp_path):
    setup = tmp_path / 'setup with spaces.bash'
    setup.write_text('PREFLIGHT_TEST_VALUE=ready\n')
    code, output, _ = PREFLIGHT.run_local_cmd(
        f'source {shlex.quote(str(setup))} && printf "%s" "$PREFLIGHT_TEST_VALUE"')
    assert (code, output) == (0, 'ready')
    code, _, _ = PREFLIGHT.run_local_cmd('false | cat')
    assert code != 0


@pytest.mark.parametrize('code,output,expected', [
    (127, '', 'FAIL'),
    (124, '/livox/lidar\n', 'FAIL'),
    (0, '', 'FAIL'),
    (0, '/livox/lidar_local\n/livox/lidar_extra\n', 'FAIL'),
    (0, 'Configured network\n0\n', 'FAIL'),
    (0, '/livox/imu\n/livox/lidar\n', 'PASS'),
])
def test_dds_discovery_requires_success_and_exact_topic(monkeypatch, code, output, expected):
    monkeypatch.setattr(PREFLIGHT, 'run_local_cmd', lambda cmd: (code, output, 'query failed'))
    assert PREFLIGHT.dds_discovery()['status'] == expected


@pytest.mark.parametrize('code,stratum,leap,reference,expected', [
    (0, '2', 'Normal', 'C0A83201', True),
    (0, '2', 'Insert second', 'C0A83201', True),
    (0, '0', 'Not synchronised', '00000000', False),
    (0, '2', 'Not synchronised', 'C0A83201', False),
    (0, '16', 'Normal', 'C0A83201', False),
    (0, '', 'Normal', 'C0A83201', False),
    (0, '2', 'Normal', '', False),
    (0, '2', 'Normal', '00000000', False),
    (1, '2', 'Normal', 'C0A83201', False),
])
def test_chrony_rejects_unsynchronized_or_failed_tracking(
        code, stratum, leap, reference, expected):
    output = (f'Reference ID : {reference}\nStratum : {stratum}\n'
              f'System time : 0.0 seconds fast of NTP time\nLeap status : {leap}\n')
    assert PREFLIGHT.chrony_is_synchronized(code, output) is expected


@pytest.mark.parametrize('remote_code,remote_output,expected', [
    (0, b'yes\n', 'PASS'), (1, b'yes\n', 'FAIL'), (0, b'no\n', 'FAIL'),
])
def test_remote_time_sync_checks_command_status(
        monkeypatch, remote_code, remote_output, expected):
    local = 'Reference ID : C0A83201\nStratum : 2\nLeap status : Normal\n'
    monkeypatch.setattr(PREFLIGHT, 'run_local_cmd', lambda cmd: (0, local, ''))
    stdout = SimpleNamespace(
        read=lambda: remote_output,
        channel=SimpleNamespace(recv_exit_status=lambda: remote_code))
    client = SimpleNamespace(exec_command=lambda *a, **k: (None, stdout, None))
    assert PREFLIGHT.chrony_status_check(client)['status'] == expected


@pytest.mark.parametrize('fix', [False, True])
def test_shm_presence_is_not_a_failure_and_fix_never_deletes(monkeypatch, fix):
    monkeypatch.setattr(PREFLIGHT.glob, 'glob', lambda pattern: ['/dev/shm/fastrtps_live'])

    def forbidden(command):
        pytest.fail(f'SHM inspection must not execute a command: {command}')

    monkeypatch.setattr(PREFLIGHT, 'run_local_cmd', forbidden)
    result = PREFLIGHT.dds_shm_cleanup(fix)
    assert result['count'] == 1
    assert result['status'] == 'INFO'
