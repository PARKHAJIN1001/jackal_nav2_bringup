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
import struct
import subprocess
import time
import sys

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
        print(f"{color}{text}{Colors.RESET}")

def run_local_cmd(cmd):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except Exception as e:
        return 1, "", str(e)

def dds_shm_cleanup(fix=False):
    """(a) DDS SHM 정리 (로컬)"""
    files = glob.glob('/dev/shm/fastrtps_*')
    count = len(files)
    status = 'PASS' if count == 0 else 'FAIL'
    msg = f"/dev/shm/fastrtps_* 파일 수: {count}"
    
    if count > 0 and fix:
        rc, _, err = run_local_cmd("rm -f /dev/shm/fastrtps_*")
        if rc == 0:
            msg += " -> 삭제 성공"
            status = 'PASS'
        else:
            msg += f" -> 삭제 실패 ({err})"
            
    return {"name": "DDS SHM Cleanup", "status": status, "msg": msg, "count": count}

def nuc_connectivity():
    """(b) NUC Connectivity"""
    start = time.time()
    status = 'FAIL'
    msg = 'Connection failed'
    rtt = None
    
    try:
        with socket.create_connection((NUC_IP, 22), timeout=2.0) as sock:
            end = time.time()
            rtt = (end - start) * 1000
            status = 'PASS'
            msg = f"SSH 포트 개방 확인 (RTT: {rtt:.2f}ms)"
    except Exception as e:
        msg = f"접속 불가: {e}"
        
    return {"name": "NUC Connectivity", "status": status, "msg": msg, "rtt_ms": rtt}

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
    """(c) NUC SSH 연결 검증"""
    client = get_ssh_client()
    if client:
        client.close()
        return {"name": "NUC SSH", "status": "PASS", "msg": "SSH 연결 성공"}
    return {"name": "NUC SSH", "status": "FAIL", "msg": "SSH 연결 실패"}

def clock_offset_measurement(client):
    """(d) Clock Offset Measurement"""
    if not client:
        return {"name": "Clock Offset", "status": "FAIL", "msg": "SSH 연결 필요"}
        
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
            return {"name": "Clock Offset", "status": "FAIL", "msg": "시간 측정 실패"}
            
        avg_offset = (sum(offsets) / len(offsets)) * 1000  # ms
        avg_rtt = (sum(rtts) / len(rtts)) * 1000
        
        status = 'PASS' if abs(avg_offset) < 20.0 else 'FAIL'  # 20ms 기준 (Connected_Validation 문서 참고)
        msg = f"NUC-Laptop 오프셋: {avg_offset:.2f}ms (평균 RTT: {avg_rtt:.2f}ms)"
        return {"name": "Clock Offset", "status": status, "msg": msg, "offset_ms": avg_offset}
        
    except Exception as e:
        return {"name": "Clock Offset", "status": "FAIL", "msg": str(e)}

def chrony_status_check(client):
    """(e) Chrony Status Check"""
    laptop_rc, laptop_out, _ = run_local_cmd('chronyc tracking')
    laptop_synced = 'System time' in laptop_out or 'Reference ID' in laptop_out

    nuc_synced = False
    nuc_msg = 'Unknown'
    if client:
        stdin, stdout, stderr = client.exec_command('timedatectl status')
        out = stdout.read().decode()
        if 'synchronized: yes' in out or 'NTPSynchronized=yes' in out:
            nuc_synced = True
            nuc_msg = 'NUC Clock Synchronized (OK)'
        else:
            nuc_msg = 'NUC Clock NOT Synchronized'

    status = 'PASS' if laptop_synced and nuc_synced else 'FAIL'
    laptop_msg = 'Laptop Chrony OK' if laptop_synced else 'Laptop Chrony NOT Tracking'

    return {'name': 'Time Sync Status', 'status': status, 'msg': f'{laptop_msg}, {nuc_msg}'}


def wifi_power_management(fix=False):
    """(f) WiFi Power Management"""
    rc, out, _ = run_local_cmd(f'iwconfig {WIFI_IFACE} 2>/dev/null')
    if rc != 0:
        return {'name': 'WiFi Power Management', 'status': 'SKIP', 'msg': f'{WIFI_IFACE} 장치를 찾을 수 없음'}

    is_on = 'Power Management:on' in out
    status = 'WARN' if is_on else 'PASS'
    msg = '절전 모드 켜짐 (NTP 지터 유발 가능, sudo iwconfig wlp5s0 power off 권장)' if is_on else '절전 모드 꺼짐 (양호)'

    return {'name': 'WiFi Power Management', 'status': status, 'msg': msg}


def nuc_network_buffers(client, fix=False):
    """(g) NUC Network Buffers"""
    if not client:
        return {'name': 'NUC Network Buffers', 'status': 'FAIL', 'msg': 'SSH 연결 필요'}

    try:
        stdin, stdout, stderr = client.exec_command('sysctl -n net.core.rmem_max net.ipv4.ipfrag_high_thresh')
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
    """(h) NUC ROS Process Check"""
    if not client:
        return {'name': 'NUC ROS Process', 'status': 'FAIL', 'msg': 'SSH 연결 필요'}

    try:
        stdin, stdout, stderr = client.exec_command('ps aux | grep "[r]os2"')
        out = stdout.read().decode().strip()

        running = len(out) > 0
        msg = 'ROS2 프로세스 실행 중' if running else 'ROS2 프로세스 없음'

        return {'name': 'NUC ROS Process', 'status': 'PASS' if running else 'INFO', 'msg': msg, 'running': running}
    except Exception as e:
        return {'name': 'NUC ROS Process', 'status': 'FAIL', 'msg': str(e)}


def dds_discovery():
    """(i) DDS / ROS2 Discovery"""
    rc, out, _ = run_local_cmd(
        'source /opt/ros/humble/setup.bash && '
        'source /home/parkhajin/moai_navigation_ws/install/setup.bash && '
        'source /home/parkhajin/moai_navigation_ws/install/jackal_network_bringup/share/'
        'jackal_network_bringup/config/network_env.sh laptop && '
        'timeout 3 ros2 topic list 2>/dev/null | grep -c "/livox/lidar" || true'
    )
    if rc == 0 and out.strip() != '0':
        return {'name': 'DDS Topic Discovery', 'status': 'PASS', 'msg': 'ROS2 토픽 연결 확인 (/livox/lidar 수신 중)'}
    return {'name': 'DDS Topic Discovery', 'status': 'WARN', 'msg': 'ROS2 토픽 미수신 (센서 서비스 확인 필요)'}

def main():
    parser = argparse.ArgumentParser(description="Jackal Nav2 Pilot Preflight Script")
    parser.add_argument('--fix', action='store_true', help="발견된 문제 자동 수정 시도")
    parser.add_argument('--json', action='store_true', help="JSON 형식 출력")
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
        print("Jackal Nav2 Preflight Summary")
        print(f"{'='*40}")
        for r in results:
            color = Colors.GREEN if r['status'] == 'PASS' else Colors.RED if r['status'] == 'FAIL' else Colors.YELLOW
            status_text = f"[{r['status']}]"
            print(f"{color}{status_text:<7} {r['name']:<25}: {r['msg']}{Colors.RESET}")
        print(f"{'='*40}")
        
    # 결정적 실패가 있는지 확인
    fail_count = sum(1 for r in results if r['status'] == 'FAIL' and r['name'] != 'NUC ROS Process')
    if fail_count > 0:
        if not args.json:
            print(f"{Colors.RED}{fail_count}개의 항목 실패. 확인이 필요합니다.{Colors.RESET}")
        sys.exit(1)
    else:
        if not args.json:
            print(f"{Colors.GREEN}모든 중요 점검 항목 통과!{Colors.RESET}")
        sys.exit(0)

if __name__ == '__main__':
    main()
