# 2026-09-16 재현 → 완전 종료 → 별도 터미널 순차 기동 검증

정상 실행 방법을 찾는 경우 [순차 기동 절차와 확인 조건](Normal_Startup_2026-09-16.md)을 먼저 참고한다.
아래는 앞선 실패 시험의 역사적 기록이며 정상 실행 안내를 대신하지 않는다.
후속 원본 영상 표시 OFF 시험에서는 FAST/perception 동작이 회복됐지만 AMCL TF 안정성은 미달이다.

## 판정

**요청한 재현 시험을 수행했으며, 결과는 정상 기동 실패다.**
실행 순서 또는 composition 설정만으로 해결되지 않았다. 아래 커맨드는
주행을 차단한 **오류 재현용**이며, 정상 운용·goal 주행 승인 커맨드가 아니다.
이 기록은 [앞선 10분 정지 검증](Connected_Validation_2026-09-16.md)의 후속이다.

원본 실행 자료: `/tmp/nav2_relaunch_20260916.IIFOxF/`.
보관 자료: [validation/relaunch](validation/2026-09-16/relaunch/).
노트북의 독립 PTY 터미널 세션에서 각 launch를 실행했다. NUC 서비스는 재시작하지 않았다.
시스템 설정, 원본 DDS XML, TF tolerance, sensor stamp, IMU buffer 한도를 변경하지 않았다.

## 사전 확인

- 관련 launch 잔여 없음. NUC sensors/platform 서비스 모두 active.
- NUC `cmd_vel_safety_bridge.forward_cmd_vel=false` 확인.
- laptop chrony 정상, 남은 보정 약 0.026 ms. 최소 RTT 0.987 ms 표본에서
  NUC−laptop 약 −0.070 ms 추정. 정확도에는 RTT/2의 불확실성이 있다.
- laptop `rmem_max=26214400` 유지. sudo나 시계 step 없음.
- 모든 Nav2 실행은 `enable_motion=false`, FAST는 LIO-only, perception tracking은 odom.

## 비교 결과

| 시험 | 기동 조건 | 관측 결과 |
| --- | --- | --- |
| A | 원본 DDS, composition=false; Nav2 → FAST → perception | FAST의 IMU buffer limit 보호 종료 재현. relay 관측 수신 간격 최대 27.807초. AMCL은 이번에는 active. |
| B | A의 모든 프로세스 종료 확인 후 원본 DDS, composition=true; Nav2 단독 확인 → FAST | AMCL active. Nav2 단독 초기 relay 공백 후 회복했으나 FAST 추가 후 다시 수신 공백·동일 보호 종료. perception은 추가하지 않고 중단. |
| C | B 완전 종료 후 임시 UDP-only DDS, composition=true; Nav2 → FAST → perception 각각 별도 세션 | SHM 제외 후에도 초기 relay 공백과 동일 FAST 보호 종료. 전체 스택 순차 재기동에서도 미해결. |

A의 두 root TF 초기화 구간에서 사용자의 RViz 초기 pose 입력이 기록됐다.
C에서도 FAST 기동 무렵 `initialPoseReceived`가 기록됐다. 에이전트가 이전 pose를
재발행하거나 임의의 초기 pose를 넣지 않았다. 정상 운용 10분 TF 시험으로 해석하지 않는다.

### 직접 실패 원인

세 시험의 FAST 로그에 다음 예외와 exit code 1이 있다.

```text
IMU application buffer limit reached; stop and restart localization/perception after diagnosis
```

relay에서 LiDAR가 끊기는 동안 IMU가 계속 들어오며 2,000개 한도에 도달했다.
예를 들어 B에서는 FAST IMU buffer가 213 → 599 → 994 → 1380 → 1773으로 증가한 뒤 종료됐다.
`IMU and LiDAR not synced`의 큰 차이는 마지막 LiDAR stamp와 계속 들어오는 IMU stamp의
차이이므로, 그 수치만으로 NUC–laptop 시계가 수십 초 어긋났다고 해석하지 않는다.

이전 사용자 실행에서는 map_server bond timeout으로 AMCL이 inactive인 문제도 있었다.
이번 A/B/C에서는 AMCL active를 확인했으므로 **그 lifecycle 오류까지 매번 재현된 것은 아니다.**
AMCL active와 지도 표시만으로 odom·센서 준비 완료를 판단하면 안 된다.

### 수신 계측의 범위

- A 70초 observer: local cloud 37개, 최대 간격 27.807초. 마지막 약 9초는 종료 구간이다.
- B Nav2-only 20초 observer: local cloud 수신 0개, IMU 3500개.
- B 추가 raw-vs-relay 20초: 원본 297개 / local 275개, 각 최대 간격 0.125 / 0.130초로 회복.
  새 원본 구독의 영향과 단순 시간 경과를 분리하지 못했으므로 인과관계를 단정하지 않는다.
- B FAST 단계 35초: local cloud 최대 간격 12.933초, odom 64개 수신 후 FAST 종료.
- C Nav2-only 25초: local cloud는 마지막 부분에 7개만 수신, relay 누적 최대 간격 26.665초.
- C full 65초: local cloud 283개 / IMU 11997개, local 최대 간격 17.671초.
  이 observer는 FAST fatal 이후 시작했으므로 FAST 실패 원인은 console log와 함께 판단한다.
- 위 여섯 관측 창의 최종 속도 합계 4,414개 모두 zero, nonfinite 0.
  이는 주행 차단 확인이지 유효 명령 통과·실제 제동 시험이 아니다.

전체 launch 종료 후 원본만 다시 읽은 15초에서는 LiDAR 224개(약 15 Hz), IMU 2991개
(약 200 Hz), stamp 역행 0회였다. LiDAR 최대 수신 간격 0.071초, 중앙 age 0.073초.
이 표본은 전체 스택 부하 상태의 정상 수신을 보장하지 않는다.

### 통신 가설과 미확정 사항

B의 Nav2 container UDP 수신 메모리는 약 52.4 MB 한도까지 차고 drop이 증가했다.
같은 snapshot의 relay 수신 큐에는 큰 backlog가 없었다. NUC Livox의 discovery socket에도
많은 **누적** drop이 있었지만, 해당 시험만의 증가량으로 환산할 시작 표본은 없다.

따라서 DDS discovery/수신 처리와 LiDAR 전달 경로를 추가 분리할 필요가 있다.
**컨테이너 backlog가 relay 단절을 일으켰다는 인과관계는 아직 추론**이며 확인되지 않았다.
C는 SHM을 제외해도 재현됐으므로 SHM 단독 원인·UDP-only 해결책이라는 결론도 지지하지 않는다.
NUC 센서 자체 고장, ROS clock 문제, 특정 consumer 한 개의 문제로 단정하지 않는다.

임시 UDP-only 파일은 원본과 동일한 interface/peer/buffer 설정에서 SHM descriptor와
user transport만 제외했다. [Fast DDS transport 구성 방식](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/udp/udp.html)을 사용한 비교이며
원본 networking 패키지나 기본값에 배포하지 않았다.

## 재현 커맨드 — 원본 DDS, 주행 차단

세 터미널 **각각**에서 먼저 실행한다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

다른 FAST/Nav2/perception이 실행 중이면 중복 실행하지 않는다. 시간 동기화 확인과
`ros2 param get /cmd_vel_safety_bridge forward_cmd_vel`의 False 확인을 먼저 한다.

### 터미널 A: Nav2 + relay + RViz

```bash
ros2 launch jackal_nav2_bringup bringup.launch.py \
  map:=$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml \
  use_sim_time:=false \
  use_composition:=false \
  use_lidar_relay:=true \
  raw_lidar_topic:=/livox/lidar \
  lidar_pointcloud_topic:=/livox/lidar_local \
  use_rviz:=true \
  use_pedestrian_figures:=true \
  use_pedestrian_traces:=true \
  enable_motion:=false
```

위 `false`는 A 실패 조건이다. `true`로 바꾼 B에서도 실패했으므로 이것을 해결책으로
제시하지 않는다. 다음 명령은 다른 점검 터미널에서 실행한다.

```bash
ros2 lifecycle get /amcl
ros2 topic echo /nav2/lidar_relay_diagnostics
```

AMCL `active [3]`와 relay의 지속적인 `fresh`를 확인하고 echo만 Ctrl-C로 끝낸다.
`receipt_timeout`이나 수신 공백이면 정상 검증 절차에서는 다음 단계를 진행하지 않는다.
한 번의 fresh 표본은 이후 지속성을 보장하지 않는다.

### 터미널 B: 단일 FAST-LIVO2

```bash
ros2 launch fast_livo mapping_mid360.launch.py \
  lidar_topic:=/livox/lidar_local \
  imu_topic:=/livox/imu \
  odom_frame:=odom \
  base_frame:=base_link \
  image_enable:=false \
  use_sim_time:=false \
  rviz:=false \
  publish_sensor_static_tf:=false \
  publish_lidar_to_imu_tf:=true
```

FAST의 생존뿐 아니라 `/odom`의 지속적인 수신을 확인한다. buffers의 IMU 증가,
LiDAR 대기, FATAL 발생 시 시험을 중지한다. buffer 상한을 늘려서 계속 실행하지 않는다.

### 터미널 C: perception

```bash
nav2_perception_dir="$(mktemp -d /tmp/nav2_perception_repro.XXXXXX)"

ros2 run jackal_nav2_bringup prepare_perception_config.py \
  --output-dir "$nav2_perception_dir/profile" &&
ros2 launch mid360_bringup perception.launch.py \
  lidar_preprocess_config:="$nav2_perception_dir/profile/lidar_preprocess.yaml" \
  extractor_config:="$nav2_perception_dir/profile/mask_3d_extractor.yaml" \
  use_sim_time:=false \
  publish_sensor_static_tf:=true \
  publish_base_to_lidar_tf:=false \
  publish_lidar_to_imu_tf:=false \
  publish_lidar_to_camera_tf:=true \
  launch_rviz:=false \
  launch_detection_markers:=false \
  launch_track_markers:=false \
  launch_trace_markers:=false
```

이 상태에서 AMCL active, 신선한 `/scan`·`/odom`, 정상 FAST를 확인한 뒤에만
2D Pose Estimate를 입력한다. FAST를 재시작했다면 perception도 재시작하고
새 초기 pose를 입력한다. **이 재현 시험에서는 Nav2 Goal을 보내지 않는다.**

## 종료 및 다음 진단

주행 차단 상태에서 세 launch 터미널에 Ctrl-C를 신속히 전달한다.
Nav2 relay만 먼저 끄고 FAST를 오래 남기면 IMU-only 누적이 다시 생길 수 있다.
이번에는 각 시험 사이와 마지막에 로그에서 추출한 정확한 launch/자식 PID를 확인했다.
검증 프로세스는 모두 종료했으며 NUC sensors/platform 서비스는 active로 유지했다.
기존 ROS CLI daemon은 재현 launch 대상이 아니므로 종료하지 않았다.

RViz -11, static costmap -6, 일부 container 종료 timeout/-9는 별도 잔여 문제다.
프로세스가 남지 않은 것과 clean shutdown은 다르다.

다음 진단은 센서→relay만의 기준선에서 scan projection, Nav2 lifecycle/TF 소비자,
perception을 단계적으로 추가하고, 각 단계의 양쪽 UDP counter **증가량**과
수신 주기·queue를 동시 측정하는 것이다. 임의의 timestamp 보정, TF tolerance 확대,
IMU buffer 확대, NUC sudo 변경을 이번 시험에 섞지 않았다.
