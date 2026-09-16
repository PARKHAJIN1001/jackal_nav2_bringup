#!/usr/bin/env bash
# ============================================================================
# pilot_quickfix.sh — 즉시 블로커 해소 스크립트
# ============================================================================
# 사용법:
#   1) 랩톱에서:  sudo bash pilot_quickfix.sh laptop
#   2) NUC에서:   sudo bash pilot_quickfix.sh nuc
#
# 각 단계는 독립적으로 실행되며, 실패해도 다음 단계를 시도합니다.
# --dry-run 옵션으로 실제 변경 없이 확인만 가능합니다.
# ============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

DRY_RUN=false
TARGET="${1:-}"

if [[ "$TARGET" == "--dry-run" ]]; then
    DRY_RUN=true
    TARGET="${2:-}"
fi

if [[ -z "$TARGET" ]] || [[ "$TARGET" != "laptop" && "$TARGET" != "nuc" ]]; then
    echo -e "${RED}사용법: sudo bash $0 [--dry-run] <laptop|nuc>${NC}"
    exit 1
fi

run_or_dry() {
    if $DRY_RUN; then
        echo -e "  ${YELLOW}[DRY-RUN]${NC} $*"
    else
        eval "$@"
    fi
}

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN} Jackal Nav2 Quick Fix — $TARGET${NC}"
echo -e "${CYAN} $(date '+%Y-%m-%d %H:%M:%S %Z')${NC}"
echo -e "${CYAN}============================================${NC}"

# ──────────────────────────────────────────────
# [1] FastDDS 공유메모리 잔여 정리
# ──────────────────────────────────────────────
echo -e "\n${GREEN}[1/5] FastDDS SHM 잔여 정리${NC}"
SHM_COUNT=$(ls /dev/shm/ 2>/dev/null | grep -c 'fastrtps' || true)
if [[ "$SHM_COUNT" -gt 0 ]]; then
    echo "  발견: ${SHM_COUNT}개 fastrtps SHM 파일"
    run_or_dry "rm -f /dev/shm/fastrtps_*"
    AFTER=$(ls /dev/shm/ 2>/dev/null | grep -c 'fastrtps' || true)
    echo -e "  ${GREEN}✅ 정리 완료 (남은: ${AFTER}개)${NC}"
else
    echo -e "  ${GREEN}✅ SHM 잔여 없음${NC}"
fi

if [[ "$TARGET" == "laptop" ]]; then
    # ──────────────────────────────────────────────
    # [2] WiFi 파워세이브 비활성화
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[2/5] WiFi 파워세이브 비활성화${NC}"
    WIFI_IF="wlp5s0"
    if iwconfig "$WIFI_IF" 2>/dev/null | grep -q "Power Management:on"; then
        echo "  현재: Power Management ON"
        run_or_dry "iwconfig $WIFI_IF power off"
        echo -e "  ${GREEN}✅ 파워세이브 비활성화 완료${NC}"
        echo "  복원: sudo iwconfig $WIFI_IF power on"
    elif iwconfig "$WIFI_IF" 2>/dev/null | grep -q "Power Management:off"; then
        echo -e "  ${GREEN}✅ 이미 비활성화 상태${NC}"
    else
        echo -e "  ${YELLOW}⚠️ $WIFI_IF 인터페이스를 찾을 수 없음${NC}"
        # 다른 WiFi 인터페이스 검색
        for iface in $(iwconfig 2>/dev/null | grep 'IEEE 802.11' | awk '{print $1}'); do
            echo "  대안 발견: $iface"
            if iwconfig "$iface" 2>/dev/null | grep -q "Power Management:on"; then
                run_or_dry "iwconfig $iface power off"
                echo -e "  ${GREEN}✅ $iface 파워세이브 비활성화${NC}"
            fi
        done
    fi

    # ──────────────────────────────────────────────
    # [3] Chrony 설정 확인 및 maxupdateskew 적용
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[3/5] Chrony 설정 확인${NC}"
    if command -v chronyc &>/dev/null; then
        echo "  chrony 설치 확인 ✅"
        
        # maxupdateskew 런타임 설정
        CURRENT_SKEW=$(chronyc tracking 2>/dev/null | grep 'Skew' | head -1 | awk '{print $NF}')
        echo "  현재 Skew: ${CURRENT_SKEW:-N/A}"
        
        # maxupdateskew 1000 적용 (런타임)
        run_or_dry "chronyc maxupdateskew 1000"
        echo -e "  ${GREEN}✅ maxupdateskew 1000 적용${NC}"
        
        # 설정 파일 확인
        CONF="/etc/chrony/chrony.conf"
        if [[ -f "$CONF" ]]; then
            if grep -q 'maxupdateskew 1000' "$CONF"; then
                echo -e "  ${GREEN}✅ 설정 파일에 maxupdateskew 1000 반영됨${NC}"
            else
                CURRENT_MUS=$(grep 'maxupdateskew' "$CONF" 2>/dev/null | head -1 || echo "없음")
                echo -e "  ${YELLOW}⚠️ 설정 파일: $CURRENT_MUS (런타임만 1000)${NC}"
                
                # jackal_network_bringup에 업데이트된 설정이 있으면 설치
                NETWORK_CONF="$HOME/moai_navigation_ws/src/jackal_network_bringup/config/time_sync/chrony-laptop.conf"
                if [[ -f "$NETWORK_CONF" ]] && grep -q 'maxupdateskew 1000' "$NETWORK_CONF"; then
                    echo "  소스 설정 파일 발견: $NETWORK_CONF"
                    run_or_dry "install -o root -g root -m 0644 '$NETWORK_CONF' '$CONF'"
                    echo -e "  ${GREEN}✅ 영구 설정 설치 완료${NC}"
                fi
            fi
        fi
        
        # tracking 상태 출력
        echo ""
        echo "  --- chronyc tracking ---"
        chronyc tracking 2>/dev/null | grep -E 'Reference|Stratum|System time|Root|Skew|Freq' | sed 's/^/  /'
    else
        echo -e "  ${RED}❌ chrony 미설치. 설치: sudo apt install chrony${NC}"
    fi

    # ──────────────────────────────────────────────
    # [4] 시간 오프셋 측정
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[4/5] NUC-랩톱 시간 오프셋 측정${NC}"
    if timeout 2 bash -c "echo | nc -w1 192.168.50.2 22 > /dev/null 2>&1"; then
        echo "  NUC SSH 포트 열림 ✅"
        echo "  10회 SSH 기반 측정..."
        
        python3 -c "
import time, subprocess, statistics

offsets = []
rtts = []
for i in range(10):
    t1 = time.time()
    r = subprocess.run(
        ['ssh', '-o', 'ConnectTimeout=2', '-o', 'StrictHostKeyChecking=no',
         '-o', 'BatchMode=yes', 'jackal@192.168.50.2', 'date +%s.%N'],
        capture_output=True, text=True, timeout=5
    )
    t4 = time.time()
    if r.returncode == 0 and r.stdout.strip():
        nuc_t = float(r.stdout.strip())
        mid = (t1 + t4) / 2
        rtt = t4 - t1
        offset = nuc_t - mid
        offsets.append(offset * 1000)
        rtts.append(rtt * 1000)

if offsets:
    avg = statistics.mean(offsets)
    std = statistics.stdev(offsets) if len(offsets) > 1 else 0
    print(f'  평균 오프셋: {avg:+.1f}ms ± {std:.1f}ms')
    print(f'  범위: [{min(offsets):+.1f}ms, {max(offsets):+.1f}ms]')
    print(f'  RTT: avg={statistics.mean(rtts):.1f}ms')
    if abs(avg) <= 5:
        print('  ✅ 기준 충족 (≤5ms)')
    elif abs(avg) <= 20:
        print('  🟡 허용 범위 (≤20ms) 내, 이상적 기준(≤5ms) 초과')
    else:
        print('  🔴 기준 초과 (>20ms)!')
else:
    print('  SSH 측정 실패 — 키 인증 확인 필요')
" 2>/dev/null || echo -e "  ${YELLOW}⚠️ SSH 키 인증 필요 — 수동 측정 권장${NC}"
    else
        echo -e "  ${YELLOW}⚠️ NUC 연결 불가${NC}"
    fi

    # ──────────────────────────────────────────────
    # [5] 요약
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[5/5] 요약${NC}"
    echo "  다음 단계:"
    echo "  1. NUC에서도 실행: ssh jackal sudo bash pilot_quickfix.sh nuc"
    echo "  2. 시간 오프셋 재확인: chronyc tracking"
    echo "  3. ROS2 시작: ros2 launch jackal_nav2_bringup bringup.launch.py \\"
    echo "       map:=\$(ros2 pkg prefix jackal_nav2_bringup)/share/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"

elif [[ "$TARGET" == "nuc" ]]; then
    # ──────────────────────────────────────────────
    # [2] 네트워크 버퍼 튜닝
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[2/5] 네트워크 버퍼 튜닝${NC}"
    
    RMEM_MAX=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo "0")
    IPFRAG=$(cat /proc/sys/net/ipv4/ipfrag_high_thresh 2>/dev/null || echo "0")
    echo "  현재 rmem_max: $RMEM_MAX"
    echo "  현재 ipfrag_high_thresh: $IPFRAG"
    
    # rmem_max: 25MB (DDS UDP 수신 버퍼 확대)
    TARGET_RMEM=26214400
    if [[ "$RMEM_MAX" -lt "$TARGET_RMEM" ]]; then
        run_or_dry "sysctl -w net.core.rmem_max=$TARGET_RMEM"
        run_or_dry "sysctl -w net.core.rmem_default=2097152"
        echo -e "  ${GREEN}✅ rmem_max → $TARGET_RMEM${NC}"
    else
        echo -e "  ${GREEN}✅ rmem_max 이미 충분${NC}"
    fi
    
    # ipfrag_high_thresh: 8MB (IP 재조립 실패 방지)
    TARGET_FRAG=8388608
    if [[ "$IPFRAG" -lt "$TARGET_FRAG" ]]; then
        run_or_dry "sysctl -w net.ipv4.ipfrag_high_thresh=$TARGET_FRAG"
        run_or_dry "sysctl -w net.ipv4.ipfrag_low_thresh=6291456"
        echo -e "  ${GREEN}✅ ipfrag_high_thresh → $TARGET_FRAG${NC}"
    else
        echo -e "  ${GREEN}✅ ipfrag_high_thresh 이미 충분${NC}"
    fi

    # ──────────────────────────────────────────────
    # [3] timesyncd 시간원 확인
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[3/5] 시간 동기화 확인${NC}"
    
    if systemctl is-active --quiet systemd-timesyncd 2>/dev/null; then
        echo "  timesyncd: active ✅"
        timedatectl show 2>/dev/null | grep -E 'NTP|Server' | sed 's/^/  /' || true
    else
        echo -e "  ${YELLOW}⚠️ timesyncd 비활성${NC}"
    fi
    
    # NTP 시간원 확인
    NTP_CONF="/etc/systemd/timesyncd.conf.d/90-jackal-lan-time.conf"
    if [[ -f "$NTP_CONF" ]]; then
        echo "  LAN NTP 설정: $NTP_CONF ✅"
        grep 'NTP=' "$NTP_CONF" 2>/dev/null | sed 's/^/  /' || true
    else
        echo -e "  ${YELLOW}⚠️ LAN NTP 설정 없음${NC}"
        # 기본으로 laptop을 시간원으로 설정
        if [[ ! -d "/etc/systemd/timesyncd.conf.d" ]]; then
            run_or_dry "mkdir -p /etc/systemd/timesyncd.conf.d"
        fi
        echo "  laptop(192.168.50.1)을 NTP 시간원으로 설정..."
        run_or_dry "cat > '$NTP_CONF' << 'TIMECONF'
[Time]
NTP=192.168.50.1
FallbackNTP=
TIMECONF"
        run_or_dry "systemctl restart systemd-timesyncd"
        echo -e "  ${GREEN}✅ LAN NTP 설정 완료${NC}"
    fi

    # ──────────────────────────────────────────────
    # [4] sysctl 영구 설정
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[4/5] sysctl 영구 설정${NC}"
    SYSCTL_CONF="/etc/sysctl.d/99-jackal-nav2.conf"
    if [[ ! -f "$SYSCTL_CONF" ]]; then
        run_or_dry "cat > '$SYSCTL_CONF' << 'SYSCONF'
# Jackal Nav2 — DDS/ROS2 네트워크 최적화
# UDP 수신 버퍼 (DDS 대용량 포인트클라우드 전송)
net.core.rmem_max = 26214400
net.core.rmem_default = 2097152
# IP 재조립 (대형 UDP 패킷 fragmentation 방지)
net.ipv4.ipfrag_high_thresh = 8388608
net.ipv4.ipfrag_low_thresh = 6291456
SYSCONF"
        echo -e "  ${GREEN}✅ $SYSCTL_CONF 생성 완료 (재부팅 후에도 유지)${NC}"
    else
        echo -e "  ${GREEN}✅ 이미 설정됨${NC}"
        cat "$SYSCTL_CONF" | sed 's/^/  /'
    fi

    # ──────────────────────────────────────────────
    # [5] 요약
    # ──────────────────────────────────────────────
    echo -e "\n${GREEN}[5/5] 요약${NC}"
    echo "  NUC 설정 완료. 다음 단계:"
    echo "  1. 센서 서비스 재시작: sudo systemctl start jackal-sensors.service"
    echo "  2. 랩톱에서 시간 오프셋 재확인"
    echo "  3. ROS2 bringup 시작"
fi

echo -e "\n${CYAN}============================================${NC}"
echo -e "${CYAN} 완료 — $(date '+%H:%M:%S')${NC}"
echo -e "${CYAN}============================================${NC}"
