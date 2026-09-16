# FAST-LIVO2 배포 후 정지 실기 예비 검증 — 미통과

2026-09-14, frontier_10F. 사용자 정지·수동 조작·비상정지 가능, 지도 표시와
2D Pose Estimate 입력 확인 후 실행했다. **10분 안정화 및 주행 검증은 하지 않았다.**
증거는 [stationary_pilot](validation/2026-09-14/stationary_pilot/README.md)에 보관했다.

## 실행 조건

- Nav2 composition, local LiDAR relay, `enable_motion=false`.
- NUC `cmd_vel_safety_bridge.forward_cmd_vel=false`를 시작 전에 조회했다. 변경하지 않았다.
- FAST 단일 LIO-only: `/livox/lidar_local`, `image_enable=false`.
- 별도 perception: 같은 LiDAR 입력, tracking `odom`, latest-TF fallback false.
- NUC base→LiDAR, FAST LiDAR→IMU, perception LiDAR→camera 소유권 유지.
- ROS 실행 중 시계 step, TF tolerance 확대, 측정 stamp 덮어쓰기, 주행 목표 발행 없음.
- 시작 전 중복 Nav2 잔여 프로세스 정리는 [preflight](validation/2026-09-14/stationary_preflight/README.md)에 별도 기록했다.

## 측정 결과

| 항목 | 실측 | 판정 / 한계 |
| --- | --- | --- |
| root `map→odom` 소유자 | `/amcl`, writer 1개 | 소유권 정상 |
| root `odom→base_link` 소유자 | `/laserMapping`, writer 1개 | 소유권 정상 |
| 전체 perception 60초, steady scan→odom | 833/833 (100%) | 해당 구간 통과 |
| 같은 구간 scan→map | 743/833 (89.2%) | 99% 목표 미달 |
| AMCL TF 역행 | 1회, 0.066192초 | 역행 0 목표 미달 |
| FAST TF 역행 | 0회, 수신 age 약 13–38 ms | 해당 구간 정상 |
| `/livox/lidar_local` / `/ped_tracking` | 약 15 Hz, topic stamp 역행 0 | tracking은 `odom` |
| 계측 카메라 구독을 뺀 후속 30초 | steady scan→map 및 odom 각각 404/404, 역행 0 | 최초 실패를 상쇄하지 않음 |
| 별도 scene 60초 최종 속도 | 1,192개, nonzero 0, nonfinite 0 | 주행 활성화 시험 아님 |
| FAST RSS, 150초 | 156,816→157,012 KiB (약 153 MiB) | LIO-only에서 거의 일정, 장시간/LIVO 누수 해결 보장 아님 |
| Nav2 container RSS, 150초 | 170,928→171,016 KiB | 해당 구간 거의 일정 |
| 시스템 UDP RcvbufErrors, 150초 | **+151,344** | 실제 증가, 과거 누적값만 인용한 것이 아님 |

60초 recorder의 `incomplete`는 figure/trace parameter dump 실패 때문이기도 하다.
TF audit 자체는 60.02초를 완료하고 별도로 위 두 가지 이슈를 검출했다.
`/j100_0519/tf`의 플랫폼 EKF는 독립 namespace의 관측이며 root TF 중복으로 계산하지 않았다.
AMCL의 미래 stamp 정책은 그대로 두었다. `age<0` 자체를 오류로 판정하지 않는다.

### 시간 동기화

- 시작 전 최소 RTT 표본: NUC−laptop 추정 −0.059 ms, RTT 0.810 ms.
- 종료 전 최소 RTT 표본: 추정 −0.0018 ms, RTT 0.298 ms.
- 후속 10개 표본 추정 offset은 약 −0.041~+0.020 ms. RTT/2 규모의 불확실성이 있다.
- Laptop chrony: time.bora.net 선택, Leap Normal, 남은 보정 약 0.007 ms.
- NUC: 계속 192.168.50.1 사용. 후속 응답 하나는 `Ignored=yes`였지만,
  그것만으로 시계 불일치를 단정하지 않고 위 직접 OS 시각 비교를 함께 기록했다.

이번 측정은 1 ms 미만 시각 차이와 일치한다. **이번 TF 문제의 원인이 시계 오차라는 근거는 없다.**
다음 부팅 이후의 동기화 유지까지 검증한 것은 아니다.

### UDP 버퍼와 TF 지연

Laptop/NUC 모두 `rmem_max=rmem_default=212992` bytes였다.
Laptop FastDDS XML은 receiveBufferSize 26,214,400 bytes를 요청하지만 실제 ROS UDP 소켓은
`ss -m`에서 `rb425984`였다. 큰 버퍼 요청이 커널 한도에 제한된 상태와 일치한다.

심한 손실이 관측된 두 소켓의 소유자는 **Nav2 container PID 267349**다:

```text
192.168.50.1:7666  fd=15  rb425984  drops=731248
192.168.50.1:7667  fd=21  rb425984  drops=452218
```

소켓 drops는 시점 누적값이며 시스템 150초 증가량과 혼동하지 않는다.
후속 30초는 perception 자체를 끈 것이 아니라 **audit의 추가 camera/accumulated/
detection/tracking 구독을 생략**한 비교다. 간헐적 문제 또는 관측 부하 민감성을 시사하지만,
동일 조건 반복/A-B 순서 교차 시험이 아니므로 원인 확정은 아니다.

수신 손실이 TF 지연의 기여 요인일 가능성이 있다(추론). AMCL 내부 message-filter의
처리 순서 역전, DDS 재전송/스케줄링, 계측 부하는 아직 분리하지 못했다.
버퍼만 크게 하면 해결된다고 단정하지 않는다. 깊어진 큐는 지연과 메모리 사용을 늘릴 수 있다.

### Costmap / 사람 / 정지 경로

- 실행 중 세 costmap parameter dump: `StaticLayer + InflationLayer`, observation_sources 비어 있음.
  local은 odom rolling 4×4 m, 0.05 m. 실시간 subscription graph 전체 검증은 미완료다.
- scene 구간 static/global costmap: 각 40/39개 full grid에서 cell hash 변화 0.
- local: 100개 full grid 중 cell hash 변화 7회, origin 변화 3회.
  odom drift/rolling window 및 지도 투영에 따른 변화 가능성이 있다(추론).
  이것만으로 사람 영향이 없다는 고정 TF 통합 시험까지 통과했다고 주장하지 않는다.
- tracking 859개 모두 `odom`, 모두 nonempty, 최대 5개 human track.
  실제 인원 수 정답이나 ID 지속성은 평가하지 않았다.
- Guard 진단: `independent_stop` 55회, `tf_stale:map->odom` 3회.
  `enable_motion=false`; monitor 명령 없음. 유효 명령 통과/감속/센서 단절 시험은 미실시.
- scan–map 끝점 거리: median 0.05 m, p90 0.35 m, 0.15 m 이내 약 80.5%.
  이는 **절대 localization 정확도**가 아니다. AMCL의 x 분산 약 0.258 m²도 잔여 문제다.

## Figure/trace 실행 등록 복원

두 노드가 시작 직후 `StopIteration`으로 종료됐다. 구현 파일과 예전 executable은 남아 있지만,
`moai_nav_viz/setup.py` 및 build egg-info의 console_scripts에는 두 항목이 없었다.
소스 함수 단위 회귀만으로 배포 상태까지 검증하지 못한 사례다.

승인을 받아 **`moai_nav_viz/setup.py`에 등록 두 줄만 복원**했다:

```python
'pedestrian_figures_node = moai_nav_viz.pedestrian_figures_node:main',
'pedestrian_traces_node = moai_nav_viz.pedestrian_traces_node:main',
```

기존 bbox 노드 변경 없음. figure/trace 순수 회귀 **41개 통과**.
workspace 재빌드는 자동 승인 도구의 `Selected model is at capacity` 오류로 실행되지 않았다.
승인 실패를 수동 entry_points 편집이나 `python -m` 실행으로 우회하지 않았다.
**소스 복원 완료 / 설치 등록 복원 대기 / 실기 figure·trace 확인 미완료**다.

## 종료 상태와 추가 잔여 문제

- Nav2/RViz 및 별도 perception launch에 Ctrl-C를 전달했고 두 실행 세션 종료를 확인했다.
- Perception 자식은 clean exit. Nav2 `static_costmap_node`는 종료 중 -6,
  container는 launch의 SIGINT/SIGTERM timeout 후 SIGKILL(-9). 정상 종료 개선도 후속 대상이다.
- Nav2와 함께 relay가 먼저 종료돼 사용자 FAST에는 LiDAR가 끊기고 IMU만 도착했다.
  약 17초 뒤 IMU buffer cap으로 mapper가 fail-fast 종료했다. **측정 중 오류가 아니라 종료 구간 사건**이다.
  안전한 센서 단절 시험 완료로 간주하지 않는다.
- 사용자 FAST launch/IMU static publisher는 Ctrl-C로 정리해야 한다.
  후속 PID 조회는 자동 승인 용량 오류로 거절되어 잔여 상태를 확인하지 못했다.
- NUC 센서/플랫폼 서비스와 `forward_cmd_vel=false`는 그대로 뒀다.
- 현재 README의 Nav2-first 종료 절차는 relay도 중단한다. 명령 차단을 먼저 확보한 다음
  perception/FAST도 신속히 정리하도록 lifecycle/종료 절차를 개선해야 한다.
  전체 프로세스 kill, SHM 전체 삭제, 무조건적인 종료 timeout 확대로 해결하지 않는다.

## 다음 실행 전 사용자 단계

먼저 사용자 FAST 터미널에서 **Ctrl-C**. 이후 laptop에서 아래 두 작업을 수행한다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PATH=/usr/bin:$PATH colcon build --symlink-install --executor sequential \
  --packages-select moai_nav_viz
source install/setup.bash
```

sudo는 사용자가 직접 실행한다. 우선 **laptop 한 대의 수신 한도만** 기존 FastDDS 요청과 맞추는
임시 A/B 시험이며 NUC/default/send buffer/영구 sysctl 파일은 변경하지 않는다.
`rmem_max`는 수신 socket buffer 상한이다. [Linux 커널 문서](https://www.kernel.org/doc/html/v5.19/admin-guide/sysctl/net.html#rmem-max)

```bash
sudo sysctl -w net.core.rmem_max=26214400
```

한도 변경 후 새 ROS 소켓의 실제 `ss -m` 크기를 다시 확인한다. 기존 소켓이 자동 확장됐다고
가정하지 않는다. 재부팅 전 임시 시험이다. 되돌릴 경우 테스트 스택 종료 후
`sudo sysctl -w net.core.rmem_max=212992`로 이 세션에서 확인한 이전 한도를 복원한다.
메모리/UDP drops/센서 age를 함께 감시해 큐가 지연을 감추는지 확인한다. 시각 변경은 하지 않는다.

그다음 Nav2/relay → FAST → 별도 perception → 사용자 초기 pose를 새로 진행한다.
marker executable 및 실제 발행 확인, 60초 full-perception 반복이 양호할 때만 10분 측정을 수행한다.
최종 기준은 TF 중복·역행 0 및 steady scan exact-time 성공률 99% 이상이며 현재 미통과다.
