#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
연결 후 검증 및 런타임 환경 진단 스크립트.

ROS2 의존성 없이 stdlib 및 paramiko만 사용하여 시스템 상태를 점검합니다.
"""

import argparse
import glob
import json
import os
import socket
import subprocess
import sys
import time

try:
    import paramiko
except ImportError:
    print("paramiko 패키지가 필요합니다. 'pip install paramiko'를 실행하세요.")
    sys.exit(1)

NUC_IP = '192.168.50.2'
SSH_USERS = ['administrator', 'jackal']
SSH_KEY_FILE = os.path.expanduser('~/.ssh/id_ed25519_jackal_nuc')
SSH_PASSWORDS = ['clearpath', 'jackal']
WIFI_IFACE = 'wlp5s0'
DDS_MULTICAST = '239.255.0.1'
DDS_PORT = 7400


class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    RESET = '\033[0m'


def color_print(text, color, json_mode):
    if not json_mode:
        print(f'{color}{text}{Colors.RESET}')


def run_local_cmd(cmd):
    """Run Bash setup commands with a deadline and preserve failures."""
    try:
        res = subprocess.run(
            ['bash', '-o', 'pipefail', '-c', cmd],
            capture_output=True, text=True, timeout=15,
            env={**os.environ, 'LC_ALL': 'C'})
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, '', 'command timed out after 15 seconds'
    except Exception as e:
        return 1, '', str(e)


def dds_shm_cleanup(fix=False):
    """Report SHM presence without assuming that existing segments are stale."""
    files = glob.glob('/dev/shm/fastrtps_*')
    count = len(files)
    status = 'PASS' if count == 0 else 'INFO'
    msg = f'/dev/shm/fastrtps_* 파일 수: {count}'

    if count > 0:
        msg += ' (사용 중일 수 있음; 파일 존재만으로 장애 판정하지 않음)'
    if fix:
        msg += ' --fix로 SHM을 삭제하지 않음'

    return {'name': 'DDS SHM Inspection', 'status': status, 'msg': msg, 'count': count}


def nuc_connectivity():
    """Check NUC connectivity."""
    start = time.time()
    status = 'FAIL'
    msg = 'Connection failed'
    rtt = None

    try:
        with socket.create_connection((NUC_IP, 22), timeout=2.0):
            end = time.time()
            rtt = (end - start) * 1000
            status = 'PASS'
            msg = f'SSH 포트 개방 확인 (RTT: {rtt:.2f}ms)'
    except Exception as e:
        msg = f'접속 불가: {e}'

    return {'name': 'NUC Connectivity', 'status': status, 'msg': msg, 'rtt_ms': rtt}


def get_ssh_client():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Try with SSH key first
    key_path = SSH_KEY_FILE if os.path.exists(SSH_KEY_FILE) else None
    for user in SSH_USERS:
        try:
            if key_path:
                client.connect(NUC_IP, username=user, key_filename=key_path, timeout=3.0)
                return client
            client.connect(NUC_IP, username=user, timeout=3.0)
            return client
        except Exception:
            pass

    for user in SSH_USERS:
        for pwd in SSH_PASSWORDS:
            try:
                client.connect(NUC_IP, username=user, password=pwd, timeout=3.0)
                return client
            except Exception:
                continue

    return None


def nuc_ssh_connection():
    """Verify NUC SSH connection."""
    client = get_ssh_client()
    if client:
        client.close()
        return {'name': 'NUC SSH', 'status': 'PASS', 'msg': 'SSH 연결 성공'}
    return {'name': 'NUC SSH', 'status': 'FAIL', 'msg': 'SSH 연결 실패'}


def clock_offset_measurement(client):
    """Measure clock offset between laptop and NUC."""
    if not client:
        return {'name': 'Clock Offset', 'status': 'FAIL', 'msg': 'SSH 연결 필요'}

    offsets = []
    rtts = []
    try:
        for _ in range(10):
            t1 = time.time()
            stdin, stdout, stderr = client.exec_command('date +%s.%N')
            nuc_time_str = stdout.read().decode().strip()
            t4 = time.time()

            if not nuc_time_str:
                continue

            nuc_time = float(nuc_time_str)
            rtt = t4 - t1
            # NUC 응답 시간은 왕복 시간의 중간이라고 가정
            t_mid = t1 + rtt / 2
            offset = nuc_time - t_mid

            offsets.append(offset)
            rtts.append(rtt)

        if not offsets:
            return {'name': 'Clock Offset', 'status': 'FAIL', 'msg': '시간 측정 실패'}

        avg_offset = (sum(offsets) / len(offsets)) * 1000  # ms
        avg_rtt = (sum(rtts) / len(rtts)) * 1000

        # 20ms 기준 (Connected_Validation 문서 참고)
        status = 'PASS' if abs(avg_offset) < 20.0 else 'FAIL'
        msg = f'NUC-Laptop 오프셋: {avg_offset:.2f}ms (평균 RTT: {avg_rtt:.2f}ms)'
        return {'name': 'Clock Offset', 'status': status, 'msg': msg, 'offset_ms': avg_offset}

    except Exception as e:
        return {'name': 'Clock Offset', 'status': 'FAIL', 'msg': str(e)}


def chrony_is_synchronized(returncode, output):
    """Require a synchronized leap state and a valid NTP stratum."""
    if returncode != 0:
        return False
    fields = {}
    for line in output.splitlines():
        key, separator, value = line.partition(':')
        if separator:
            fields[key.strip()] = value.strip()
    try:
        stratum = int(fields.get('Stratum', ''))
    except ValueError:
        return False
    reference = fields.get('Reference ID', '').split()
    return (
        fields.get('Leap status') in ('Normal', 'Insert second', 'Delete second')
        and 1 <= stratum <= 15
        and bool(reference)
        and reference[0] not in ('00000000', '0.0.0.0')
    )


def chrony_status_check(client):
    """Check actual local and remote synchronization states."""
    laptop_rc, laptop_out, laptop_error = run_local_cmd('chronyc -n tracking')
    laptop_synced = chrony_is_synchronized(laptop_rc, laptop_out)

    nuc_synced = False
    nuc_msg = 'Unknown'
    if client:
        try:
            _, stdout, _ = client.exec_command(
                'LC_ALL=C timedatectl show --property=NTPSynchronized --value', timeout=5)
            out = stdout.read().decode().strip()
            nuc_synced = stdout.channel.recv_exit_status() == 0 and out == 'yes'
            nuc_msg = 'NUC Clock Synchronized (OK)' if nuc_synced else (
                'NUC Clock NOT Synchronized')
        except Exception as error:
            nuc_msg = f'NUC time sync query failed: {error}'

    status = 'PASS' if laptop_synced and nuc_synced else 'FAIL'
    laptop_msg = 'Laptop Chrony OK' if laptop_synced else 'Laptop Chrony NOT Tracking'
    if laptop_rc != 0:
        laptop_msg += f' (exit {laptop_rc}: {laptop_error})'

    return {'name': 'Time Sync Status', 'status': status, 'msg': f'{laptop_msg}, {nuc_msg}'}


def wifi_power_management(fix=False):
    """Check WiFi power management status."""
    rc, out, _ = run_local_cmd(f'iwconfig {WIFI_IFACE} 2>/dev/null')
    if rc != 0:
        return {
            'name': 'WiFi Power Management',
            'status': 'SKIP',
            'msg': f'{WIFI_IFACE} 장치를 찾을 수 없음',
        }

    is_on = 'Power Management:on' in out
    status = 'WARN' if is_on else 'PASS'
    msg = (
        '절전 모드 켜짐 (NTP 지터 유발 가능, sudo iwconfig wlp5s0 power off 권장)'
        if is_on else '절전 모드 꺼짐 (양호)'
    )

    return {'name': 'WiFi Power Management', 'status': status, 'msg': msg}


def nuc_network_buffers(client, fix=False):
    """Check NUC network buffers."""
    if not client:
        return {'name': 'NUC Network Buffers', 'status': 'FAIL', 'msg': 'SSH 연결 필요'}

    try:
        cmd = 'sysctl -n net.core.rmem_max net.ipv4.ipfrag_high_thresh'
        stdin, stdout, stderr = client.exec_command(cmd)
        lines = stdout.read().decode().strip().split()
        if len(lines) >= 2:
            rmem = int(lines[0])
            ipfrag = int(lines[1])
            rmem_ok = rmem >= 26214400
            ipfrag_ok = ipfrag >= 8388608
            status = 'PASS' if rmem_ok and ipfrag_ok else 'WARN'
            msg = f'rmem_max={rmem} (>=25MB: {rmem_ok}), ipfrag={ipfrag} (>=8MB: {ipfrag_ok})'
        else:
            status = 'FAIL'
            msg = 'sysctl 조회 실패'

        return {'name': 'NUC Network Buffers', 'status': status, 'msg': msg}
    except Exception as e:
        return {'name': 'NUC Network Buffers', 'status': 'FAIL', 'msg': str(e)}


def nuc_ros_process_check(client):
    """Check ROS processes on NUC."""
    if not client:
        return {'name': 'NUC ROS Process', 'status': 'FAIL', 'msg': 'SSH 연결 필요'}

    try:
        stdin, stdout, stderr = client.exec_command('ps aux | grep "[r]os2"')
        out = stdout.read().decode().strip()

        running = len(out) > 0
        msg = 'ROS2 프로세스 실행 중' if running else 'ROS2 프로세스 없음'

        return {
            'name': 'NUC ROS Process',
            'status': 'PASS' if running else 'INFO',
            'msg': msg,
            'running': running,
        }
    except Exception as e:
        return {'name': 'NUC ROS Process', 'status': 'FAIL', 'msg': str(e)}


def dds_discovery():
    """Check exact topic discovery; this does not establish message delivery."""
    rc, out, error = run_local_cmd(
        'source /opt/ros/humble/setup.bash >/dev/null && '
        'source /home/parkhajin/moai_navigation_ws/install/setup.bash >/dev/null && '
        'source /home/parkhajin/moai_navigation_ws/install/jackal_network_bringup/share/'
        'jackal_network_bringup/config/network_env.sh laptop >/dev/null && '
        'exec ros2 topic list --no-daemon --spin-time 3'
    )
    if rc != 0:
        return {'name': 'DDS Topic Discovery', 'status': 'FAIL',
                'msg': f'DDS 조회 실패 (exit {rc}): {error}'}
    if '/livox/lidar' in {line.strip() for line in out.splitlines()}:
        return {'name': 'DDS Topic Discovery', 'status': 'PASS',
                'msg': '/livox/lidar 발견 (메시지 수신 여부는 별도 확인 필요)'}
    return {'name': 'DDS Topic Discovery', 'status': 'FAIL',
            'msg': '/livox/lidar 토픽 미발견 (네트워크·센서 서비스 확인 필요)'}


def main():
    parser = argparse.ArgumentParser(description='Jackal Nav2 Pilot Preflight Script')
    parser.add_argument('--fix', action='store_true',
                        help='이전 호출과의 호환성 옵션; SHM 자동 삭제는 수행하지 않음')
    parser.add_argument('--json', action='store_true', help='JSON 형식 출력')
    args = parser.parse_args()

    results = []

    # a) DDS SHM
    results.append(dds_shm_cleanup(args.fix))

    # b) NUC Connectivity
    results.append(nuc_connectivity())

    # c) NUC SSH
    ssh_res = nuc_ssh_connection()
    results.append(ssh_res)

    client = None
    if ssh_res['status'] == 'PASS':
        client = get_ssh_client()

    # d) Clock Offset
    results.append(clock_offset_measurement(client))

    # e) Chrony Status
    results.append(chrony_status_check(client))

    # f) WiFi PM
    results.append(wifi_power_management(args.fix))

    # g) NUC Buffers
    results.append(nuc_network_buffers(client, args.fix))

    # h) NUC ROS Process
    results.append(nuc_ros_process_check(client))

    # i) DDS Discovery
    results.append(dds_discovery())

    if client:
        client.close()

    # j) Summary
    if args.json:
        print(json.dumps(results, indent=2, ensure_ascii=False))
    else:
        print(f"\n{'='*40}")
        print('Jackal Nav2 Preflight Summary')
        print(f"{'='*40}")
        for r in results:
            if r['status'] == 'PASS':
                color = Colors.GREEN
            elif r['status'] == 'FAIL':
                color = Colors.RED
            else:
                color = Colors.YELLOW
            status_text = f"[{r['status']}]"
            print(f"{color}{status_text:<7} {r['name']:<25}: {r['msg']}{Colors.RESET}")
        print(f"{'='*40}")

    # 결정적 실패가 있는지 확인
    fail_count = sum(
        1 for r in results if r['status'] == 'FAIL' and r['name'] != 'NUC ROS Process'
    )
    if fail_count > 0:
        if not args.json:
            print(f'{Colors.RED}{fail_count}개의 항목 실패. 확인이 필요합니다.{Colors.RESET}')
        sys.exit(1)
    else:
        if not args.json:
            print(f'{Colors.GREEN}모든 중요 점검 항목 통과!{Colors.RESET}')
        sys.exit(0)


if __name__ == '__main__':
    main()
