# Jackal localization 실행 절차

표준 진입점은 정지 관측·실주행 모두 `nav_session.py nav`다. 기본은 motion 비활성이며,
생성 프로파일은 `--profile-dir`, 명시적 실주행은 `--enable-motion`으로 전달한다.
기동·안정화·재검증의 최신 기준은 [Startup_Stability.md](Startup_Stability.md)를 따른다.
이번 변경은 오프라인 검증만 했으며, 장비 기동·커널 설정·주행 검증은 사용자가 수행한다.

## 처음 한 번: 패키지 반영

기존 수동 launch가 실행 중이면 해당 터미널에서 perception → Nav2/FAST 순서로 종료한 다음 전환한다. 새 도구는 이미 실행 중인 프로세스를 인계받거나 임의로 종료하지 않는다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jackal_network_bringup jackal_nav2_bringup --symlink-install
source install/setup.bash
```

두 패키지를 함께 반영한다. 새로운 network preflight와 stability monitor가 설치되지 않은 혼합 설치본은 준비 완료로 인정하지 않는다. 이전 설치 디렉터리에 남은 `pilot_quickfix.sh`는 실행하지 않는다.

## 모든 ROS 터미널의 공통 환경

```bash
source /opt/ros/humble/setup.bash
source ~/moai_navigation_ws/install/setup.bash
source ~/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

Jackal 연결, NUC의 플랫폼/MID-360 서비스, 기존 시각 동기화가 준비된 상태에서 시작한다. 도구는 domain 1과 laptop 네트워크 환경을 확인하지만, 시각 동기화나 외부 서비스 자체를 재설정하지 않는다. 실제 ROS가 실행되는 호스트 터미널에서 명령을 실행한다. 격리 환경의 `/proc/sys`는 호스트 값과 다를 수 있다.

## 1. network 패키지가 소유하는 임시 통신 설정

```bash
sudo python3 "$(ros2 pkg prefix jackal_network_bringup)/lib/jackal_network_bringup/ipfrag_session.py" apply
ros2 run jackal_nav2_bringup nav_session.py check
```

`jackal_network_bringup`이 `ipfrag_high_thresh` 최소 16MiB와 `ipfrag_time=3` 정책을 소유한다.
세션과 직접 launch는 `ros2 run jackal_network_bringup network_preflight.py --check`를 호출한다.
이 도구는 ipfrag 상태, XML 요청값과 rmem/wmem 한도, publication mode를 읽기 전용으로 확인한다.
`ipfrag_session.py apply`는 DDS socket 한도를 수정하지 않는다. 실패 시 보고서의 해당 항목을 진단한다.
메모리 한도가 이미 크면 줄이지 않으며 실제 바꾼 항목만 원래 값으로 복원한다.
명시적 restore 또는 재부팅까지 유지하고 터미널 종료 시 자동 복원하지 않는다.
현재 정책은 새로운 시험 기준이며 과거 128MiB 검증 결과와 구분한다.

이전 `/run/jackal-nav2-ipfrag/state.json`이 있으면 모든 ROS 스택 종료 후 network 도구의
`restore --legacy-nav2`로 먼저 복원한다. boot/namespace/현재 값이 다르면 자동 인계하지 않는다.
128MiB 상태가 이미 충분해도 시간값만 변경될 수 있으므로 두 값을 모두 확인한다.
상태 디렉터리는 root만 쓸 수 있고 일반 사용자는 읽을 수 있다. 이전 비공개 상태를 읽지 못하면
status --check는 오류(2)로 종료한다. sudo로 기존 상태를 복원한 뒤 새 apply를 사용한다.
커널 쓰기는 명시적인 sudo apply/restore로만 수행하며 Nav2 launch는 변경하지 않는다.

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
4. 전체 프로세스 기동 후 최대 600초 동안 180초 연속 정상 구간을 확인한다. 실패하면 연속 시간을 초기화한다.
5. `INITIAL_POSE_REQUIRED`에서 RViz **2D Pose Estimate**로 위치·방향을 지정하고 지도·scan 정합을 확인한다.
6. 초기 위치로 Nav2 lifecycle 전환이 추가로 완료되면 다시 180초 확인 후 `READY`가 된다. Perception 시작/종료도 재확인 대상이다.

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
- 로그 기본 경로는 `~/.ros/nav_sessions/<날짜시간>_<nav|perception>/`이다. `--log-root /원하는/경로`로 바꿀 수 있다. 각 터미널의 로그 폴더는 별도이며 `session.json`, `resources.jsonl`, `ros/`를 보존한다. Nav 세션은 `stability.json`·`stability_events.jsonl`도 기록한다.
- ROS launch/node 출력은 터미널과 ROS 로그로 남는다. `session.json`은 시작 및 상태 변경 때 저장해 정상 종료 전에도 근거를 보존한다. 원본 cloud/image를 자동 녹화하지 않는다.
- 세션 잠금과 현재 상태는 `/tmp/jackal-nav2-session-<uid>/`에 둔다. 재부팅 후 사라질 수 있으므로 기록의 원본은 로그 폴더다. 오래된 PID는 boot ID와 프로세스 시작 시각까지 맞아야 종료 요청 대상으로 인정한다.

## 5. 종료 후 커널 값 복원

준비된 별도 ROS 터미널에서:

```bash
ros2 run jackal_nav2_bringup nav_session.py stop
sudo python3 "$(ros2 pkg prefix jackal_network_bringup)/lib/jackal_network_bringup/ipfrag_session.py" restore
```

stop은 **managed perception → managed Nav2** 순서로 종료를 요청한다. 각 세션은 먼저 launch에 SIGINT를 한 번 보내 정상 종료를 기다린다. 남은 자식은 해당 세션이 만든 프로세스 그룹에 한해 TERM/KILL로 정리한다. `pkill` 같은 이름 기반 강제 종료는 사용하지 않는다.

관리되지 않은 프로세스가 남으면 stop이 실패 상태와 PID를 알려 준다. 원래 실행 터미널에서 종료한 뒤 다시 확인한다. 강제 종료(SIGKILL)·호스트 중단으로 세션 관리자만 사라진 경우에도 남은 프로세스를 자동 인계하지 않는다. `status`의 잔여 프로세스를 확인한다.

restore는 활성 스택이 남아 있거나, 저장값이 없거나, 외부에서 커널 값을 다른 값으로 변경했다면 복원을 거부한다. ROS stop 자체는 커널 값을 바꾸지 않는다. 재부팅하면 런타임 설정과 `/run` 기록이 사라지므로 다음 부팅에서 현재 값을 확인하고 apply부터 진행한다.

## 직접 launch 및 기존 도구와의 관계

- `nav_bringup.launch.py`: 기존 staged launch. 현재 권장 세션 도구가 이 launch를 실행한다. 직접 실행도 가능하지만 세션의 중복 방지·종속 종료 관리에는 포함되지 않는다.
- `bringup.launch.py`: staged launch의 별칭. FAST도 기동하므로 이전 외부 FAST 절차와 혼용하지 않는다. 과거 relay/composition 옵션은 오류로 거부한다.
- `prepare_perception_config.py`: 기존 단독 프로파일 생성 기능을 유지한다. session 도구도 같은 구현을 사용한다.
- `pilot_preflight.py`: 상세 진단용 기존 도구. `pilot_quickfix.sh`는 설치 대상에서 제외한 과거 자료다. 정책·변경·복원 도구는 network 패키지만 소유한다.

RViz 기본 도구는 이제 `Nav2 Goal`과 `Navigation 2` 패널이다. 목표 지정 전 초기 위치·정합 확인과 `check_navigation_ready.py`를 사용한다. 기본 motion 비활성 세션은 목표를 보내도 비영 속도를 전달하지 않는다. 실주행도 동일 세션 도구에서 명시적으로 선택한다. 실주행·기록·완료 판정은 [목표 주행 절차](Goal_Navigation.md)를 따른다.

## 수동 조이스틱이 움직이지 않을 때

`enable_motion=false`는 Nav2 자동주행 출력 제한이다. 현재 로봇 bridge의 `forward_cmd_vel=false`는 Nav2 명령을 관측만 하며, 로봇 `/cmd_vel`로 전달하지 않는다. 수동 주행 장애를 해결하려고 이 설정을 켜지 않는다.

9월 18일에는 조이스틱 표시등과 `/dev/input/ps4` 장치, Joy 반복 메시지가 존재해도 실제 Bluetooth 연결이 끊겨 버튼·축 값이 모두 0인 상태가 발생했다. 사용자가 PS 버튼으로 재연결한 후 수동 주행이 복구됐다. [진단·복구 기록](validation/2026-09-18/manual_joystick/REPORT.md).

동일 증상에서는 로봇의 Bluetooth `Connected` 상태와 DS4 서비스 로그, 실제 Joy 버튼 변화를 먼저 확인한다. 조이스틱 재연결 시 스틱을 중앙에 두고 주행 활성화 버튼을 놓는다. 연결이 정상인데도 움직이지 않으면 비상정지 상태와 joystick → teleop → twist_mux → platform 경로를 읽기 전용으로 확인한다.
