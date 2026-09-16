# 실기 정지 검증 준비 — FAST 실행 승인 오류로 측정 대기

이후 사용자가 FAST를 실행하여 [실기 예비 계측](../../../Stationary_Pilot_2026-09-14.md)을 수행했다.
아래는 실행 승인 대기 시점의 기록이다. 최신 결과는 60초 기준 미달이며 10분 검증은 아직이다.

2026-09-14 사용자 요청으로 실기 안정화 검증을 시작했다. 사용자는 Jackal 정지 상태와
수동 조작/비상정지 사용 가능을 확인했다. **60초/10분 안정화 측정은 아직 시작하지 못했다.**

## 확인한 상태

- NUC 연결 정상, `jackal-sensors.service`와 `clearpath-platform.service` active.
- NUC `cmd_vel_safety_bridge.forward_cmd_vel=false`를 조회했다. 설정 변경 없음.
- Laptop chrony: 외부 time.bora.net 선택, Leap Normal, 남은 보정 약 0.06 ms.
- NUC timesyncd: server 192.168.50.1, PacketCount 285 → 290, 최신 응답 `Ignored=no`.
- 직접 OS-to-OS 10회 측정: 최소 RTT 0.810 ms 표본에서 NUC−laptop 추정 −0.059 ms.
  RTT/2 수준의 불확실성을 포함해 해석한다. adjtime pending은 양쪽 0이고 chrony 잔여 보정도 별도 확인했다.
- D455: NUC `lsusb -t`에서 video 인터페이스 5000M (USB 3).
- Livox 원본 `/livox/lidar` publisher 1개, RELIABLE. Nav2 실행 전 subscriber 0개.
- Laptop 시작 시 MemAvailable 약 9.8 GiB, swap 사용 약 1.7 GiB. 기존 swap이 남아 있는 값이며 새 누수 측정 결과가 아니다.

## 프로세스 정리와 실행

기존 root ROS graph에 `nav2_container`와 `fast_livo_odom_adapter`가 각각 3개 존재했다.
정확한 명령·PID·ROS domain을 확인한 뒤 해당 6개만 SIGINT로 종료하고 소멸을 확인했다.
종료 PID: `224838, 224842, 232313, 232317, 232529, 232533`.
이전 실행의 잔여 프로세스로 판단하지만, 어떤 사용자가 어떤 명령으로 남겼는지는 단정하지 않는다.
NUC 센서·플랫폼 서비스는 유지했다.

새로 실행한 Nav2는 frontier_10F, composition, relay, RViz, figure/trace를 사용하며
`enable_motion=false`다. 로그에서 Motion DISABLED 및 collision monitor 활성화를 확인했다.
이 시점의 실측 최종 속도 토픽 검사와 localization 검증은 아직 아니다.

FAST-LIVO2 실행 요청은 자동 승인 도구의 `Selected model is at capacity` 오류로
두 차례 거절됐다. 승인 실패를 다른 도구/실행 경로로 우회하지 않았다.
FAST와 perception은 아직 실행하지 않았으므로 `odom`/`map` TF 대기와 AMCL/RViz
message-filter queue 경고는 이 시작 단계에서는 예상된다. 이를 정상 운용 중 버퍼링으로
분류하거나 TF tolerance 확대의 근거로 사용하지 않는다.
RViz의 GLSL link 오류도 로그에 있다. 초기 pose 입력 전 사용자 화면에서 map 가시성을 확인해야 한다.

## 재개 정보

- 실행 세션: Nav2 PTY `14087` (다음 세션에서 존속 여부를 다시 확인할 것).
- 활성 로그/프로파일: `/tmp/nav2_stationary_20260914.1xmPBa/`.
- `nav2_before_fast.log`는 대기 시점 스냅샷. 전체 실행 로그는 위 임시 디렉터리에서 계속 기록된다.
- FAST를 단일 LIO-only (`image_enable=false`)로 실행해야 한다.
- 이후 별도 perception은 해당 임시 디렉터리의 `perception_profile` YAML을 사용한다.
  LiDAR `/livox/lidar_local`, tracking `odom`, latest-TF fallback false,
  base→lidar 및 lidar→imu static TF false, camera bridge만 true.
- 사용자 2D Pose Estimate → 60초 full-perception/리소스 예비 계측 → 결과 양호 시
  10분 누적 계측. 새 FAST 시작 뒤 이전 초기 pose/track history를 재사용하지 않는다.
- 목표는 TF 중복·역행 0, steady scan exact-time 성공률 99% 이상이다. 현재는 미검증이다.
- 주행 목표, motion enable, NUC forwarding 변경은 하지 않았다.
