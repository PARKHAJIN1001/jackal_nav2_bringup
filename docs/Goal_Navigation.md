# RViz 목표 지정 주행

권장 진입점은 `nav_session.py nav`다. [전체 기동 안정화 기준](Startup_Stability.md)을 먼저 적용한다. RViz **2D Pose Estimate**로 초기 위치를 정하고,
**Nav2 Goal**로 목표 위치와 방향을 클릭·드래그한다. **Navigation 2** 패널에서 진행 상태,
결과, Cancel을 사용한다. 기본 motion은 비활성이며 실기 목표 도달 검증은 아직 완료하지 않았다.

## 적용과 초기 준비

현재 실행을 원래 터미널에서 정상 종료한 뒤 패키지를 반영한다. 기존 미커밋 변경과 외부
패키지는 보존한다. 이 패키지는 NUC 브리지나 플랫폼 설정을 변경하지 않는다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jackal_network_bringup jackal_nav2_bringup --symlink-install
source install/setup.bash
source install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

실제 ROS 호스트에서 시각 동기화와 `sysctl -n net.ipv4.ipfrag_high_thresh`를 확인한다.
현재 network 패키지 정책은 **최소 16MiB·ipfrag_time=3초**다. 재부팅하면 임시 설정을 다시 확인한다. 기존
[apply/restore 도구](Operations.md)를 사용하며 `/etc/sysctl.d` 영구 변경은 하지 않는다.
격리된 도구 환경에서 읽은 커널 값은 실제 호스트 값의 증거가 아니다.

NUC의 Clearpath 플랫폼·MID-360가 준비되어 있고 기존 Nav2/FAST/perception이 종료된 상태에서:

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"
```

이 launch는 relay → FAST → Nav2를 순서대로 기동한다. `AMCL/map_server active`는
**초기 위치 지정 전 입력 준비**이며 안정화 완료는 아니다. `INITIAL_POSE_REQUIRED`를 기다린 뒤 로봇을 정지시키고 2D Pose Estimate로 위치·방향을
지정한 뒤 scan이 올바른 지도 벽과 겹치는지 확인한다. 반복된 복도에서는 TF와 작은 covariance만으로
올바른 위치를 증명할 수 없다. FAST나 AMCL을 재기동했다면 초기 위치를 다시 지정한다.

이후 `nav_session.py perception --initial-pose-confirmed`를 별도 준비된 터미널에서 실행한다. Perception 프로세스 변화 후에도 안정화 시간을 다시 확인한다.
managed `nav_session.py`와 직접 launch를 혼용하지 않는다. Perception/YOLO는 별도 로딩 상태이며,
빈 tracking 메시지도 정상 수신으로 취급한다. Perception은 주행용 costmap이나 정지 판정의 입력이 아니다.

## 준비 점검

```bash
ros2 run jackal_nav2_bringup check_navigation_ready.py
# 실제 주행 설정까지 확인할 때만:
ros2 run jackal_nav2_bringup check_navigation_ready.py --require-motion \
  --output /tmp/navigation_ready_new.json
```

점검기는 목표·속도·초기 위치를 발행하지 않고 lifecycle이나 파라미터도 바꾸지 않는다.
기본 15초 동안 확인하며 2초 연속 준비 조건을 요구한다. 종료 코드: **0 준비**, **1 준비 시간 초과**,
**2 인자/실행 오류**, **130 중단**. 출력 파일은 덮어쓰지 않는다.

- `inputs_ready`: 지도 및 유효하고 신선한 scan/odom.
- `navigation_ready`: 필수 lifecycle active, NavigateToPose 서버, 최근 동적 TF와 scan 시각 TF,
  안전 진단, 단일 Guard/Collision Monitor 출력 경로까지 확인.
- `motion_ready`: 위 조건과 Guard `enable_motion=true`, NUC 브리지의 시작 설정 및 실제 출력 writer 확인.
- 목표가 없는 동안의 command silence는 정상 대기다. 센서·TF 오류나 장애물 정지는 준비 완료가 아니다.
- `perception=not_observed_or_loading`은 별도 정보다. YOLO 준비를 기다리느라 navigation을 실패시키지 않는다.

`--map-topic`, `--scan-topic`, `--odom-topic`, `--nav-cmd-topic`, `--platform-cmd-topic`,
`--bridge-node`로 경로를 맞출 수 있다. 프레임은 이 패키지의 `map/odom/base_link` 계약을 따른다.
기본 점검은 motion이 비활성이어도 통과할 수 있다. 이는 특정 시점의 관측이며 자동 주행 허가 장치나
계속 실행되는 건강 감시기가 아니다. 지도·scan 정합은 사용자가 확인한다.

## 실제 주행 모드 기동

첫 실기는 사용자 입회, 비상정지 사용 가능, 수동 조작 우선권과 빈 시험 구역 확인 후 진행한다.
기존 NUC 브리지는 `forward_cmd_vel`을 **시작할 때만 읽는다**. 실행 중 `ros2 param set`을 사용하면
표시 파라미터와 실제 동작이 달라질 수 있으므로 사용하지 않는다. Guard의 `enable_motion`도 시작 전용이다.

1. 활성 목표를 취소하고 로봇 정지를 확인한다. 기존 Nav2/FAST와 별도 perception을 종료한다.
2. NUC에서 기존 센서 service를 중지하고 동일 센서 launch를 foreground로 시작한다.
   플랫폼 service는 유지하며, 다른 센서 launch가 남아 있으면 그 원래 터미널에서 먼저 종료한다.
3. 새 센서 세션 위에 노트북 `nav_session.py nav`를 생성 프로파일과 `--enable-motion`으로 시작한다.
4. 정지 상태에서 새 초기 위치·지도 정합을 확인하고, perception을 시작하고,
   `check_navigation_ready.py --require-motion`을 통과한 뒤 **새 목표**를 보낸다.

NUC의 현재 설치본을 사용하는 명령은 아래와 같다. NUC 파일은 수정하지 않는다.

```bash
# NUC, 플랫폼은 유지하고 sensor launch만 새로 시작한다.
sudo systemctl stop jackal-sensors.service
source /opt/ros/humble/setup.bash
export COLCON_CURRENT_PREFIX=/home/administrator/ws_livox/install/livox_ros_driver2
source "$COLCON_CURRENT_PREFIX/share/livox_ros_driver2/package.bash"
unset COLCON_CURRENT_PREFIX
source /home/administrator/moai_navigation_ws/install/setup.bash
source /home/administrator/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh nuc
export LD_LIBRARY_PATH="/home/administrator/ws_livox/install/livox_sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ros2 launch jackal_network_bringup robot.launch.py \
  launch_platform:=false launch_d455:=true launch_mid360:=true \
  launch_mid360_scan:=false launch_nav2:=false launch_network_probe:=true \
  forward_cmd_vel:=true
```

```bash
# 노트북, 앞의 공통 ROS/workspace/network 환경을 source한 터미널
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml" \
  --profile-dir /tmp/jackal_motion_profile --enable-motion
```

기존 local `jackal_network_bringup`와 실제 NUC 설치본에는 차이가 있다. local 소스를 NUC에
덮어쓰지 않는다. 실제 NUC 설정이 위와 다르면 출력 토픽·service·watchdog을 먼저 확인한다.
현재 브리지 `timeout_sec=0.5`, 플랫폼 `cmd_vel_timeout=0.5`, mux external priority=1이며
수동 joy/RC와 E-stop의 우선순위가 더 높다. 이는 확인된 설정이지 제동거리 실측 결과가 아니다.

## 목표·막힘·취소 동작

기본 단일 목표 BT는 설치된 Nav2의 `navigate_w_replanning_time.xml`이며 1Hz로 재계획한다.
NavFn/DWB와 세 costmap의 지도 전용 정책을 유지한다. 지도에 없는 물체·사람은 독립 LiDAR 경로에서
감속·정지시키며 자동 우회하지 않는다. 전후 감지 거리와 timeout을 유지하고 측면 영역을 조정한다. 속도는 확인된 브리지 상한에 맞춘다.

- 속도는 준비 프로파일의 브리지 제한 이하이며 요청 상한은 0.50m/s·1.0rad/s다. 목표 오차는 0.25m·0.25rad다.
- 목표 변경은 현재 목표를 교체한다. 이전 목표를 큐에 넣어 나중에 실행하지 않는다.
- 짧은 장애물 정지 후 액션이 여전히 실행 중이면 통로가 열렸을 때 이어간다.
- progress checker는 30초 동안 필요한 0.1m 진행을 못 하면 실패시킨다.
  이는 장애물 검출 순간부터 정확히 30초를 재는 타이머가 아니다.
- 실패는 `ABORTED`, 취소는 `CANCELED`로 끝나며 새 목표를 기다린다. 자동 복구 회전·후진·costmap
  초기화는 없다. 경로 추종·목표 방향 정렬용 회전은 정상 동작이다.

Navigation 2 패널은 **자신의 Nav2 Goal 도구로 보낸 목표**를 Cancel한다. 호환 `/goal_pose`
토픽이나 다른 액션 클라이언트의 목표는 해당 클라이언트로 취소한다. 단일 운영자 환경에서 모든
NavigateToPose 목표를 취소해야 하는 진단용 명령은 다음과 같다.

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
```

패널의 Pause/Reset은 lifecycle/localization에도 영향을 줄 수 있으므로 목표 취소·비상정지 버튼으로
사용하지 않는다. Reset 후에는 초기 위치를 다시 지정한다. 이번 완료 기준은 단일 목표이며,
패널의 waypoint 모드나 사용자 지정 BT는 이 정책의 검증 대상이 아니다.

## 기록과 실기 인수

```bash
ros2 run jackal_nav2_bringup record_navigation_validation.py \
  --navigation --duration 180 --output-dir /tmp/navigation_trial_new
# Perception 없이 검증하는 경우 --localization-only를 함께 사용한다.
```

파라미터 수집 후 `Navigation recording active` 안내를 기다린다. `navigation_bag/`에는
액션 UUID·status·feedback, 경로, TF, odom, 속도 전달 각 단계, safety 진단을 기록한다.
원본 cloud/image는 추가 구독하지 않는다. `navigation_tree.xml`에 실제 기본 BT를 복사한다.
Native Goal 도구는 액션을 사용하므로 `/goal_pose`가 기록되지 않을 수 있다. 의도한 목표 좌표·방향과
물리적 위치 표식, 사용자 관찰·시험 이벤트 시각은 별도 시험 메모에 기록한다.

기록기가 목표를 보내거나 로봇을 활성화하지 않는다. 기본 localization/perception 기록은 그대로다.
짧은 녹화, 필수 TF/odom/최종 명령/safety 스트림 누락, metadata 실패는 `incomplete`이며,
`recorded`도 **기록 무결성** 상태일 뿐 목표 성공이나 실제 정지의 증거를 대신하지 않는다.

실기는 약 1m 직진 → 방향 정렬 → 목표 변경·Cancel → 장애물 감속·정지 순으로 확장한다.
각 정상 목표에서 `SUCCEEDED`, 추정 위치 오차 ≤0.25m, 방향 오차 ≤0.25rad, 최종 명령 0과
관측 속도 ≤0.02m/s가 1초 이상 유지됨을 확인한다. 사용자가 물리적 도착·정지도 별도로 확인한다.
취소·실패, 수동 조작 우선권, E-stop, watchdog 응답은 각각 결과와 시간을 기록한다.

NUC 브리지는 현재 명령 timestamp를 검증하지 않는다. 지연 도착한 명령을 새 명령으로 취급할
가능성은 코드에서 추론한 한계다. 이 변경에서는 수정하지 않았으며 통신 단절·재연결 안전성을
검증했다고 주장하지 않는다. 링크 이상 시 현장에서 정지시키고 새 세션 준비 절차를 따른다.

## 종료와 비전달 모드 복귀

1. 패널 Cancel 후 액션 종료와 최종 명령 0, 실제 정지를 확인한다.
2. 노트북 Nav2/FAST를 원래 터미널에서 Ctrl+C로 종료하고 별도 perception도 종료한다.
3. NUC foreground sensor launch를 Ctrl+C로 종료한 다음 기본 service를 복구한다.

```bash
# NUC, foreground sensor launch 종료 후
sudo systemctl start jackal-sensors.service
ros2 param get /cmd_vel_safety_bridge forward_cmd_vel
ros2 node info /cmd_vel_safety_bridge
```

기본 service의 시작 인자는 `forward_cmd_vel=false`여야 하고 브리지의 `/j100_0519/cmd_vel`
publisher가 없어야 한다. 파라미터 값만으로 비전달을 단정하지 않는다.
마지막으로 활성 Nav2/FAST/perception이 없음을 확인하고 network 패키지의 커널 도구로 restore한다.
이전 세션의 목표·초기 위치·perception tracks를 재사용하지 않는다.

## 2026-09-21 패치 운용 순서

이 절차가 위의 기존 직접 motion 실행 예제보다 우선한다. NUC 설정을 자동 변경하지 않는다.
통신·주행 변경의 실제 정지 성능은 사용자 입회 실측 전까지 미검증이다.

1. network 패키지 apply/status로 실제 ROS 호스트의 정책을 확인한다. 기존 미커밋 설정인
   ASYNCHRONOUS publication mode와 NUC 2MiB send buffer를 유지한다. NUC 재배포는 별도다.
2. NUC forwarding=false, Nav2 enable_motion=false 상태에서 아래 버튼 보정 도구를 실행한다.
   ○·△·L1·R1을 안내대로 하나씩 확인한다. 스틱을 움직이지 않는다.
3. 차체·장착물 및 회전 외곽이 정지 영역 전후 반폭 0.60m·좌우 0.40m 안에 들어오는지
   현장에서 확인한 경우에만 `--footprint-confirmed`를 준다. 확인하지 않으면 재무장되지 않는다.

```bash
ros2 run jackal_nav2_bringup calibrate_operator_stop.py \
  --output /tmp/jackal_operator_verified.yaml --footprint-confirmed
ros2 run jackal_nav2_bringup prepare_navigation_profile.py \
  --operator-config /tmp/jackal_operator_verified.yaml --output-dir /tmp/jackal_motion_profile
```

프로파일 생성은 브리지 파라미터를 읽고 새 로컬 파일만 만든다. 입력 파일의 nuc_host와
controller_address가 현재 연결된 컨트롤러와 일치해야 한다. SSH는 known_hosts를 검증하며
비대화형 공개키 인증을 사용한다. 모르는 호스트 키를 자동 승인하거나 비밀번호를 저장하지 않는다.
브리지 실제 상한이 0.20m/s·0.35rad/s면 모든 출력 단계를 그 값으로 맞춘다.
표시 파라미터를 실행 중 변경한 브리지는 시작 시 읽은 값과 다를 수 있으므로 재시작 후 확인한다.

기존 스택 정상 종료와 NUC forwarding 기동 절차를 마친 뒤 새 프로파일을 사용한다.

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml" \
  --profile-dir /tmp/jackal_motion_profile --enable-motion
```

초기 위치와 정합 확인 후 버튼을 모두 놓고, 실제 정지 상태에서 △를 2초간 누른다.
`WAITING FOR NEW GOAL` 확인 → `check_navigation_ready.py --require-motion` → 새 목표 순서다.
○ 또는 L1/R1은 Nav2를 정지·취소한다. 연결 복구·버튼 해제만으로 재개하지 않는다.
재무장에는 전체 스택 `READY` heartbeat, 정상 Joy/BlueZ 상태, 중립, 정지 1초, 활성 목표 없음이 필요하다. 안정화 실패·heartbeat 만료는 정지를 래치하고 과거 목표를 금지한다.
BlueZ 조회 실패나 관리 노드 중단도 정지한다. Bluetooth 자체 단절 감지 지연과 통신 장애 중
정지는 정상 통신의 0.5초 목표와 구분하며, 물리 비상정지를 대체하지 않는다.

현재 시험 설정: progress 0.1m/30s, local/global inflation 0.45m/scaling 5,
감속 반폭 1.00m/0.50m·배율 0.70, 정지 반폭 0.60m/0.40m. 목표 허용 오차는 유지한다.

지도 데이터는 `/map_encoder/input`을 사용한다. base_link 기준 10×10m, 200×200셀,
전방 +x이며 pose가 0.3초 이상 오래되면 새 patch를 발행하지 않는다. 구독자는 보관된
transient-local 메시지의 timestamp와 `/nav2/map_patch_diagnostics`를 확인해야 한다.
RViz의 `Robot Heading Map Patch`를 켜서 확인한다. 제어용 local costmap은 odom 축을 유지한다.

```bash
ros2 run jackal_nav2_bringup analyze_navigation_bag.py /path/to/navigation_bag \
  --output-dir /tmp/jackal_navigation_analysis
```

`timeline.csv`는 단계별 명령과 실제 odom, 각 샘플의 age, 거리·경로 끝 방향 오차를 제공한다.
`events.json`은 액션 UUID/상태와 경고·실패 로그를 제공한다. 경로 끝 방향 오차는 요청한
목표에 대한 독립적인 ground truth가 아니다. 물리 버튼 입력부터 0.5초 정지는 영상과 플랫폼
odom으로 별도 측정한다. 소프트웨어 기준은 Joy 수신부터 명령 0까지 0.1초, 최종 정지 기준은
선속도 0.02m/s·각속도 0.03rad/s 이하 1초다. 직진·회전·곡선 각 10회 통과한 속도만 승인한다.
