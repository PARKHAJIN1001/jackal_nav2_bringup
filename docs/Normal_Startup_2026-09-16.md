# Nav2 정상 기동을 위한 순서와 확인 조건

이 문서는 **오류 재현 명령이 아니라 정상 기동을 확인하며 진행하는 절차**다.
다만 현재 모든 실행에서 안정적인 정상화를 보장하는 수정은 완료되지 않았다.
아래 명령으로 FAST 처리·perception·초기 pose 수신·TF 연결은 확인했지만,
AMCL의 scan 시각 TF 유지 기준은 미달했다. **주행 활성화 절차가 아니다.**

## 이번에 확인한 조건

- 원본 네트워크 프로필과 composition=true 사용. 시계·DDS·buffer 한도 변경 없음.
- RViz의 `/camera/camera/color/image_raw` Image 표시만 끈 설정 사용.
  원래 RViz 파일과 사용자가 추가한 속도·배터리 표시는 수정하지 않았다.
  시험 설정은 [nav2_no_raw_image.rviz](validation/2026-09-16/normal_start/nav2_no_raw_image.rviz)에 보관했다.
  시험 중 RViz가 저장한 파일에는 Fixed Frame=base_link 및 화면 배치 변경이 있어
  `rviz_saved_during_trial.rviz`로 따로 보존했다. 실행 안내용 파일은 원본의
  Fixed Frame=map을 유지하고 Image 활성화 두 항목만 false로 바꾼 설정이다.
- Nav2 단독 20초: LiDAR 292개, relay 진단 fresh 40/40.
- 첫 FAST는 IMU 수신 전에 LiDAR buffer 한도에 도달해 종료했다.
  해당 launch를 완전히 종료하고 한 번 다시 실행한 뒤 정상 처리됐다.
  따라서 **Image 표시를 끄면 첫 기동부터 반드시 성공한다는 결론은 아니다.**
- FAST 재기동 후 30초: LiDAR/odometry 각각 449개, stamp 역행 0,
  odometry 최대 수신 간격 0.082초. IMU buffer는 누적 증가하지 않았다.
- perception 포함 60초: local cloud 886개, odometry 855개,
  수신 시작 이후 최대 간격 각각 0.158/0.170초. relay 진단 fresh 118/118.
  새 관측자의 odometry 최초 수신은 약 2.7초 뒤였다.
  perception 기동 직후 relay 누적 최대 간격 0.750초는 별도로 기록됐다.
- track/figure/trace 발행 확인. figure 머리 상단 0.85 m. 이번 시각적 위치 정확도 평가는 하지 않았다.
- 사용자 초기 pose 수신(14:29:42 KST) 이후 30초 audit:
  `map → odom`은 AMCL, `odom → base_link`는 FAST 단일 writer,
  역행 0회. 정상 구간 scan 시각 TF는 odom 365/365, **map 284/365 (77.81%)**.
  직전 60초 audit는 초기 pose 재입력을 포함하며 AMCL 역행 2회였다.
- 최종 속도는 full-stack 관측 1,142개와 별도 scene 관측 1,202개 모두 0.
  두 관측 구간은 겹치므로 고유 메시지 수로 합산하지 않는다.

원본 영상 표시가 추가된 시점 및 표시 OFF 후 센서 연속성 회복은 원인 후보를
지지하지만, 통제된 반복 ON/OFF 시험을 하지 않아 인과관계는 **추론**이다.
AMCL TF 지연이 남아 있어 전체 localization 정상화·주행 준비 완료로 표시하지 않는다.
기존 10분 검증 미통과 결과도 여전히 유효하다.

## 0. 공통 환경 — 각 터미널에서 실행

Jackal은 정지, 수동 조작·비상정지 가능 상태를 유지한다. 기존 Nav2/FAST/perception과
중복 실행하지 않는다. ROS 가동 중 시계를 강제로 맞추지 않는다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

점검 터미널에서 시간 동기화와 NUC 속도 전달 차단 상태를 확인한다.

```bash
chronyc tracking
ssh jackal 'timedatectl timesync-status'
ros2 param get /cmd_vel_safety_bridge forward_cmd_vel
```

NUC time server는 `192.168.50.1`, `forward_cmd_vel`은 False여야 한다.
시각 동기화가 안 됐거나 전달 차단 상태를 확인할 수 없으면 진행하지 않는다.
동기화 설정·sudo가 필요하면 별도 조치한다. 이번 명령에는 sudo가 없다.

## 1. 터미널 A — Nav2 + relay + RViz

```bash
ros2 launch jackal_nav2_bringup bringup.launch.py \
  map:=$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml \
  use_sim_time:=false use_composition:=true \
  use_lidar_relay:=true raw_lidar_topic:=/livox/lidar \
  lidar_pointcloud_topic:=/livox/lidar_local \
  use_rviz:=true \
  rviz_config:=$HOME/moai_navigation_ws/src/jackal_nav2_bringup/docs/validation/2026-09-16/normal_start/nav2_no_raw_image.rviz \
  use_pedestrian_figures:=true use_pedestrian_traces:=true \
  enable_motion:=false
```

명시한 RViz 설정 파일이 중요하다. 기본 RViz 파일에는 raw Image 표시가 켜져 있다.
위 파일은 소스 경로로 직접 읽으므로 이번 절차에 재빌드는 필요 없다.
RViz에서 Image를 다시 켜거나 다른 원본 영상 viewer를 추가하지 않는다.

점검 터미널:

```bash
ros2 lifecycle get /amcl
ros2 topic echo /nav2/lidar_relay_diagnostics
```

AMCL `active [3]`, relay `fresh`가 연속 유지되는지 약 20초 확인하고 **echo만** Ctrl-C로 끝낸다.
`receipt_timeout`이 반복되면 다음 단계로 진행하지 않는다.
지도만 보이거나 active라고 해서 FAST가 준비된 것은 아니다.
아직 map→odom이 없어 scan이 지도 위에 안 보이는 것은 초기 pose 전에는 가능하다.

## 2. 터미널 B — FAST-LIVO2 하나만 실행

```bash
ros2 launch fast_livo mapping_mid360.launch.py \
  lidar_topic:=/livox/lidar_local imu_topic:=/livox/imu \
  odom_frame:=odom base_frame:=base_link \
  image_enable:=false use_sim_time:=false rviz:=false \
  publish_sensor_static_tf:=false publish_lidar_to_imu_tf:=true
```

점검 터미널:

```bash
ros2 topic hz /odom
```

약 15 Hz로 연속 수신되는지 20–30초 확인하고 **hz만** Ctrl-C로 끝낸다.
FAST 로그에서 LiDAR 처리와 bounded buffer 유지도 함께 본다.

`process has died` 또는 `application buffer limit reached`가 나오면 **초기 pose를
반복 입력해도 해결되지 않는다.** FAST가 없으므로 odom→base_link를 제공하지 못한다.
해당 FAST launch에 Ctrl-C로 잔여 static TF까지 종료하고 LiDAR/IMU 수신을 진단한다.
이번에는 한 번 재기동해 회복했지만 무한 재시작·buffer 한도 확대를 해결책으로 삼지 않는다.
실패가 반복되면 perception/goal을 시작하지 않는다.

## 3. 터미널 C — 별도 perception

```bash
nav2_perception_dir="$(mktemp -d /tmp/nav2_perception.XXXXXX)"

ros2 run jackal_nav2_bringup prepare_perception_config.py \
  --output-dir "$nav2_perception_dir/profile" &&
ros2 launch mid360_bringup perception.launch.py \
  lidar_preprocess_config:="$nav2_perception_dir/profile/lidar_preprocess.yaml" \
  extractor_config:="$nav2_perception_dir/profile/mask_3d_extractor.yaml" \
  use_sim_time:=false \
  publish_sensor_static_tf:=true \
  publish_base_to_lidar_tf:=false publish_lidar_to_imu_tf:=false \
  publish_lidar_to_camera_tf:=true \
  launch_rviz:=false launch_detection_markers:=false \
  launch_track_markers:=false launch_trace_markers:=false
```

입력은 `/livox/lidar_local`, tracking은 `odom`, latest-TF fallback은 false가 된다.
현재 perception launch는 위 YAML override를 사용한다. 예전의
`lidar_input_topic:=... tracking_frame:=...`만 지정하는 명령으로 돌아가지 않는다.

## 4. RViz — 여기서 초기 pose 지정

1. FAST가 살아 있고 `/odom`이 수신되며 AMCL이 active인지 확인한다.
   RViz Global Options의 Fixed Frame은 `map`으로 유지한다.
2. `2D Pose Estimate`로 실제 위치를 클릭하고 전방으로 드래그해 방향을 지정한다.
3. AMCL의 `initialPoseReceived` / `Setting pose`, scan과 지도의 정합을 확인한다.
4. 점군이 지도 위에 표시됐다는 사실만으로 안정성 합격으로 판단하지 않는다.

FAST를 재시작했다면 perception도 재시작하고 **그 후 새 초기 pose**를 입력한다.
이전 odom 원점의 pose를 자동 재사용하지 않는다.

## 5. 점검 — 초기화와 TF 유지를 구분해서 확인

```bash
ros2 run tf2_ros tf2_echo map base_link
```

연속 갱신을 확인하고 Ctrl-C로 끝낸다. 필요하면 측정 구간 전체를 audit한다.

```bash
nav2_check_dir="$(mktemp -d /tmp/nav2_startup_check.XXXXXX)"
ros2 run jackal_nav2_bringup tf_localization_audit.py \
  --duration 60 --warmup 3 --output "$nav2_check_dir/tf.json"
```

perception은 실행한 채 audit에는 `--require-perception`을 붙이지 않아
추가 raw 영상 구독을 피한다. 이것은 perception 전체 항목 audit는 아니다.
`issues`가 비어 있지 않다면 미해결이다. 현재 실측은 map exact-time TF 기준 미달이므로,
**이 절차를 실행했다는 이유만으로 Nav2 Goal을 보내지 않는다.**

## 종료

주 launch를 **C (perception) → B (FAST) → A (Nav2/relay)** 순서로 Ctrl-C 종료한다.
relay만 먼저 끄고 FAST를 오래 남겨두지 않는다. NUC 센서·플랫폼 서비스는
이 절차에서 종료하지 않는다.

이번 검증 launch와 확인된 자식 PID는 모두 종료됐고 NUC의 두 서비스는 active였다.
다만 종료 시 RViz -11, static costmap -6, container timeout 후 -9가 기록됐다.
잔여 프로세스가 없는 것과 clean shutdown은 다르며 이 종료 오류도 남은 과제다.

실측 자료: [normal_start](validation/2026-09-16/normal_start/).
원본 로그: `/tmp/nav2_normal_start_20260916.2eecof/`.
