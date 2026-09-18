# Jackal localization 실행 절차

현재 기준은 [2026-09-18 검증](validation/2026-09-18/z_axis_test_1351/REPORT.md)의 설정이다. 이 절차는 기존 staged launch를 세션 도구로 감싸 기동·로그·종료를 관리한다. AMCL, FAST, TF, 센서 장착 변환, costmap 및 제어 파라미터는 변경하지 않는다. 자동주행은 다음 단계이며 현재 도구는 `enable_motion=false`로 실행한다.

## 처음 한 번: 패키지 반영

기존 수동 launch가 실행 중이면 해당 터미널에서 perception → Nav2/FAST 순서로 종료한 다음 전환한다. 새 도구는 이미 실행 중인 프로세스를 인계받거나 임의로 종료하지 않는다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jackal_nav2_bringup --symlink-install
source install/setup.bash
```

새 도구를 설치하기 전에도 패키지 소스 경로의 `scripts/nav_session.py`와 `scripts/ipfrag_session.py`를 `python3`로 실행할 수 있다. 단, 수정된 `topic_ready_gate.py`와 프로파일 생성기도 함께 반영되어 있어야 한다.

## 모든 ROS 터미널의 공통 환경

```bash
source /opt/ros/humble/setup.bash
source ~/moai_navigation_ws/install/setup.bash
source ~/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

Jackal 연결, NUC의 플랫폼/MID-360 서비스, 기존 시각 동기화가 준비된 상태에서 시작한다. 도구는 domain 1과 laptop 네트워크 환경을 확인하지만, 시각 동기화나 외부 서비스 자체를 재설정하지 않는다. 실제 ROS가 실행되는 호스트 터미널에서 명령을 실행한다. 격리 환경의 `/proc/sys`는 호스트 값과 다를 수 있다.

## 1. 임시 128MiB 설정

```bash
sudo python3 "$(ros2 pkg prefix jackal_nav2_bringup)/lib/jackal_nav2_bringup/ipfrag_session.py" apply
ros2 run jackal_nav2_bringup nav_session.py check
```

`ipfrag_session.py`는 `net.ipv4.ipfrag_high_thresh`만 변경한다. 변경 전 값을 root 소유 `/run/jackal-nav2-ipfrag/state.json`에 저장한다. 반복 apply는 저장된 원래 값을 덮어쓰지 않는다. `/etc/sysctl.d`, 다른 커널 설정, CPU/RAM 정책은 건드리지 않는다.

이 설정은 명시적인 restore 또는 재부팅까지 유지된다. **터미널 종료에 연결된 자동 복원은 하지 않는다.** 따라서 Codex/터미널 중단만으로 실행 중인 스택의 한도가 4MiB로 돌아가지 않는다. 이미 128MiB 이상이고 이 도구의 저장값이 없으면 `already_sufficient_unmanaged`를 출력하고 변경·소유권 인계를 하지 않는다.

이전 `hold_ipfrag.sh`가 설정을 유지 중이라면 스택 종료 후 그 스크립트로 먼저 복원하고, 다음 기동부터 새 도구를 사용한다. 원래 값을 모르면 추측해서 복원하지 않는다. 영구 적용은 이번 변경에 포함하지 않는다.

## 2. 터미널 A: Nav2/FAST 기동

로봇을 정지시킨 채 실행한다.

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"
```

동일 사용자의 세션 잠금과 로컬 프로세스 검사를 사용해 중복 기동을 거부한다. 직접 실행한 Nav2·FAST·perception이 발견돼도 기존 프로세스를 죽이지 않고 PID를 알려 준다. 프로세스 검사는 보수적이며 같은 호스트의 다른 로봇 실험도 충돌로 표시할 수 있다.

내부 순서는 기존과 같다.

1. relay·RViz 기동, cloud/IMU 연속 수신 확인.
2. FAST-LIVO2 기동, 유효한 odometry 연속 수신 확인.
3. Nav2 기동, scan/odom·AMCL/map_server active·map·scan 시각의 odom TF 확인.
4. `AMCL/map_server active` 안내 후 사용자가 RViz **2D Pose Estimate**로 위치·방향을 지정하고 지도·scan 정합을 확인.

첫 기동 gate는 초기 위치 지정 전이므로 `map → odom`을 요구하지 않는다. 현재 도구는 이미지 기반 LIVO나 임의 토픽 구성을 노출하지 않고 검증된 기본 구성을 사용한다. 고급 인자는 직접 launch 경로에서 사용한다.

## 3. 터미널 B: 별도 perception

동일 환경을 source하고, 초기 위치와 정합 확인을 마친 뒤 실행한다.

```bash
ros2 run jackal_nav2_bringup nav_session.py perception --initial-pose-confirmed
```

이 플래그는 사용자가 눈으로 정합을 확인했다는 표시다. 도구가 임의 초기 위치를 발행하지 않는다. 이 단계에서는 기존 localization gate에 `require_map_to_odom=true`를 추가해 초기 위치 지정 후의 scan 시각 map TF도 확인한다. 실패·취소·Nav2 세션 종료 시 perception으로 진행하지 않는다.

프로파일은 해당 세션 로그 폴더 안에 생성한다. 입력 `/livox/lidar_local`, tracking frame `odom`, 기존 static TF 분담을 유지한다. 설치된 upstream YAML은 수정하지 않는다.

YOLO cold start를 고려해 upstream 연결 검사 시간은 60초로 지정한다. 이 검사는 **토픽 연결 수**를 보는 것이며 검출 메시지 수신이나 localization 정확도를 보장하지 않는다. 노드 프로세스가 실행 중이라는 `status`와 준비 완료를 혼동하지 않는다. 실제 흐름이 필요할 때 기존 관측 도구를 사용한다.

```bash
timeout 15 ros2 topic hz /ped_yolo/detections2d
timeout 15 ros2 topic hz /ped_tracking
```

사람이 검출되지 않더라도 빈 검출 배열 메시지가 갱신되는지 확인한다. 위 명령은 이미지 구독을 추가하지 않으며, timeout 종료 코드 124는 지정한 관측 시간이 끝났다는 뜻이다.

terminal B에서 Ctrl+C 하면 perception만 종료한다. terminal A의 Nav2 세션이 끝나면 종속된 managed perception도 종료한다.

## 4. 상태·로그 확인

```bash
ros2 run jackal_nav2_bringup nav_session.py status
```

- 세션 소유자 PID·시작 시각·boot ID, 실제 자식 명령, 실행/종료 상태, 로그 위치, 커널 값과 네트워크 namespace를 표시한다.
- 로그 기본 경로는 `~/.ros/nav_sessions/<날짜시간>_<nav|perception>/`이다. `--log-root /원하는/경로`로 바꿀 수 있다. 각 터미널의 로그 폴더는 별도이며 `session.json`과 `ros/`를 보존한다.
- ROS launch/node 출력은 터미널과 ROS 로그로 남는다. `session.json`은 시작 및 상태 변경 때 저장해 정상 종료 전에도 근거를 보존한다. 원본 cloud/image를 자동 녹화하지 않는다.
- 세션 잠금과 현재 상태는 `/tmp/jackal-nav2-session-<uid>/`에 둔다. 재부팅 후 사라질 수 있으므로 기록의 원본은 로그 폴더다. 오래된 PID는 boot ID와 프로세스 시작 시각까지 맞아야 종료 요청 대상으로 인정한다.

## 5. 종료 후 커널 값 복원

준비된 별도 ROS 터미널에서:

```bash
ros2 run jackal_nav2_bringup nav_session.py stop
sudo python3 "$(ros2 pkg prefix jackal_nav2_bringup)/lib/jackal_nav2_bringup/ipfrag_session.py" restore
```

stop은 **managed perception → managed Nav2** 순서로 종료를 요청한다. 각 세션은 먼저 launch에 SIGINT를 한 번 보내 정상 종료를 기다린다. 남은 자식은 해당 세션이 만든 프로세스 그룹에 한해 TERM/KILL로 정리한다. `pkill` 같은 이름 기반 강제 종료는 사용하지 않는다.

관리되지 않은 프로세스가 남으면 stop이 실패 상태와 PID를 알려 준다. 원래 실행 터미널에서 종료한 뒤 다시 확인한다. 강제 종료(SIGKILL)·호스트 중단으로 세션 관리자만 사라진 경우에도 남은 프로세스를 자동 인계하지 않는다. `status`의 잔여 프로세스를 확인한다.

restore는 활성 스택이 남아 있거나, 저장값이 없거나, 외부에서 커널 값을 다른 값으로 변경했다면 복원을 거부한다. ROS stop 자체는 커널 값을 바꾸지 않는다. 재부팅하면 런타임 설정과 `/run` 기록이 사라지므로 다음 부팅에서 현재 값을 확인하고 apply부터 진행한다.

## 직접 launch 및 기존 도구와의 관계

- `nav_bringup.launch.py`: 기존 staged launch. 현재 권장 세션 도구가 이 launch를 실행한다. 직접 실행도 가능하지만 세션의 중복 방지·종속 종료 관리에는 포함되지 않는다.
- `bringup.launch.py`: 별도 FAST 프로세스를 전제로 하는 기존 수동/고급 경로. managed staged 세션과 동시에 사용하지 않는다.
- `prepare_perception_config.py`: 기존 단독 프로파일 생성 기능을 유지한다. session 도구도 같은 구현을 사용한다.
- `pilot_preflight.py`: 상세 진단용 기존 도구. `pilot_quickfix.sh`는 WiFi/chrony 등 별도 변경을 포함하므로 위 표준 기동 절차의 일부가 아니다.

다음 단계는 기존 `/goal_pose` 수신부터 실제 명령 전달·목표 도달·정지까지 연결하는 작업이다. 이 실행 정리는 자율주행 활성화나 검증 완료를 뜻하지 않는다.

## 수동 조이스틱이 움직이지 않을 때

`enable_motion=false`는 Nav2 자동주행 출력 제한이다. 현재 로봇 bridge의 `forward_cmd_vel=false`는 Nav2 명령을 관측만 하며, 로봇 `/cmd_vel`로 전달하지 않는다. 수동 주행 장애를 해결하려고 이 설정을 켜지 않는다.

9월 18일에는 조이스틱 표시등과 `/dev/input/ps4` 장치, Joy 반복 메시지가 존재해도 실제 Bluetooth 연결이 끊겨 버튼·축 값이 모두 0인 상태가 발생했다. 사용자가 PS 버튼으로 재연결한 후 수동 주행이 복구됐다. [진단·복구 기록](validation/2026-09-18/manual_joystick/REPORT.md).

동일 증상에서는 로봇의 Bluetooth `Connected` 상태와 DS4 서비스 로그, 실제 Joy 버튼 변화를 먼저 확인한다. 조이스틱 재연결 시 스틱을 중앙에 두고 주행 활성화 버튼을 놓는다. 연결이 정상인데도 움직이지 않으면 비상정지 상태와 joystick → teleop → twist_mux → platform 경로를 읽기 전용으로 확인한다.
