# 기동 통일·연속 안정화 (2026-09-21)

이 변경은 로컬 소스 구현과 오프라인 테스트 결과다. Jackal/ROS 스택 기동, NUC 접속,
커널·chrony·DDS XML 변경, 패키지 배포, 실주행 시험은 수행하지 않았다.
아래 명령은 사용자가 장비 연결 후 실행하고, 생성 기록을 바탕으로 진단한다.

## 변경한 계약

- 표준은 `nav_session.py nav` → `nav_bringup.launch.py`다. 정지 관측과 실주행 모두 같은 래퍼를
  쓰며 `--profile-dir`로 생성된 `nav2.yaml`, `safety.yaml`, `operator.yaml`을 전달한다.
  `--enable-motion`을 명시하지 않으면 motion=false다. `bringup.launch.py`는 staged 별칭이다.
  예전 외부 FAST 전제·relay/composition 인자는 거부한다. 별도 FAST를 동시에 시작하지 않는다.
- 세션과 직접 launch, 하위 localization/navigation/safety launch는 network 패키지의
  `network_preflight.py --check`를 공유한다. 한 launch context에서는 한 번 검사한다.
  환경·XML·커널 한도 불일치나 도구 미설치는 노드 기동 전에 실패한다.
- network 도구는 기존 ipfrag 정책과 XML의 **활성 UDP transport 요청값**을 읽는다.
  rmem/wmem 한도를 XML 요청과 비교하고 domain/role/RMW/publication mode를 확인한다.
  수정·복원은 계속 network 패키지 책임이며 Nav2는 설정을 쓰지 않는다.
  `ipfrag_session.py apply`는 socket buffer 한도까지 수정하는 도구가 아니다.
- `network_preflight.py --pid PID`는 프로세스가 받은 transport 환경변수와 현재 디스크의 XML,
  매핑된 DDS 라이브러리를 별도 기록한다. 이는 실제 초기화 때 읽힌 XML 내용의 증명이 아니다.
  `xml_load_verified=false`를 명시하며 실제 load 입증에는 별도 시작 추적이 필요하다.
- `nav2_twist_stamper.py`는 `make_stamped_twist()`만 남긴 import 전용 파일이다.
  `pilot_quickfix.sh`는 설치에서 제외한다. 기존 install에 남아 있다면 실행하지 않는다.
  `amcl_recovery_monitor.py`의 diagnostics 별칭은 호출자 전환 후 **2026-10-31 제거 예정**이다.
  Python 캐시는 삭제하고 ignore 처리했다.
- pedestrian figures/traces는 기본 off다. perception을 쓸 세션은 필요에 따라
  `--pedestrian-viz`를 지정한다. 배터리·속도 표시는 유지한다.
- Collision Monitor와 Guard, 세 costmap과 `/map_encoder/input`, 기존 안전 timeout은 유지한다.
  smoother/behavior/waypoint 서버 축소와 static costmap 조건부 제거는 측정 후 별도 변경한다.

## 시간과 상태

| 용도 | launch 인자 | 세션 인자 | 기본 |
|---|---|---|---|
| relay/FAST 각 단계 대기 | `input_timeout` | `--input-timeout` | 60초 |
| localization 입력 대기 | `localization_timeout` | `--localization-timeout` | 90초 |
| 단계별 연속 확인 | `ready_settle` | `--ready-settle` | 5초 |
| 전체 안정화 획득/복구 한도 | `stability_timeout` | `--stability-timeout` | 600초 |
| 전체 연속 정상 구간 | `stability_settle` | `--stability-settle` | 180초 |

각 연속 확인 시간은 해당 대기 한도보다 작아야 한다. 시간은 monotonic 기준이다.
`ready_settle=180`만 올리는 잘못된 구성은 시작 전에 거부한다.

전체 Nav2 프로세스 생성 시 continuous monitor를 시작한다. 로컬 raw cloud, IMU,
FAST odometry, scan, adapted odometry의 freshness/연속성·frame·시각, TF,
map/AMCL 및 navigation lifecycle, Guard 입력 상태, 최종 출력 소유권을 확인한다.
센서 gap·잘못된 메시지·clock regression·TF/lifecycle 문제는 연속 구간을 취소한다.
감시 루프 자체가 0.3초 넘게 멈춰도 그동안의 시간을 정상 구간으로 계산하지 않는다.

상태는 `/nav2/stability_diagnostics`와 세션의 `stability.json`에 기록된다.

1. `INPUT_WAITING`: 필수 입력이나 상태가 부족함. 출력 차단.
2. `STABILIZING`: 정상 수신을 연속 확인하는 중. 출력 차단.
3. `INITIAL_POSE_REQUIRED`: 초기 입력이 180초 유지됨. 초기 위치와 scan/map 정합을 직접 확인.
4. `READY`: 초기 위치 TF와 navigation lifecycle까지 준비되고 연속 확인 완료.
5. `FAILED`: 600초 내 연속 구간 확보 실패. launch 종료, 새 세션 필요.

초기 위치 없이는 일부 Nav2 lifecycle 전환이 끝나지 않을 수 있으므로, 초기 위치 입력 전에는
map/AMCL active와 생성된 navigation 노드를 확인한다. 위치 입력 후 lifecycle 전환으로
통신 상태가 바뀔 수 있어 **다시 180초 확인할 수 있다**. 이때 초기 위치를 다시 찍으라는 뜻은 아니다.
초기 위치의 수동 정합 대기에는 600초 제한을 적용하지 않는다. `READY` 또는 초기 안정화 이후
장애가 발생하면 새로운 600초 복구 창을 연다. perception 노드 구성 변화도 연속 시간을 초기화한다.

monitor는 종료하지 않고 `/nav2/stack_ready` heartbeat를 계속 발행한다. Guard와 operator stop은
false·누락·0.25초 만료 시 차단한다. operator stop은 기존 목표를 금지하고 취소하며,
다시 안정화돼도 **명시적 재무장과 새 목표**가 필요하다. `READY` 자체가 재무장이나 주행 시작은 아니다.
CLI `check_navigation_ready.py`의 2초 settle은 이미 180초 검사를 수행한 monitor의 READY도 요구한다.
TF 존재·READY는 지도 정합 정확도나 제동 성능의 증명이 아니다.

핵심 프로세스(Guard, operator stop, odom adapter, stability monitor, relay, FAST, 주요 Nav2 서버,
projection, lifecycle manager 등)가 정상/비정상 종료하면 전체 launch를 종료한다.
자동 respawn은 거부한다. 세션의 PID·boot/start identity와 소유 process group 기반 종료를 유지한다.
직접 launch도 사전 점검과 종료 감시는 적용하지만 세션 잠금·종속 perception 관리는 제공하지 않는다.

## 사용자 실행 순서

먼저 두 패키지를 함께 빌드한다. 여기서는 빌드·설치를 실행하지 않았다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jackal_network_bringup jackal_nav2_bringup --symlink-install
source install/setup.bash
source install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
ros2 run jackal_network_bringup network_preflight.py --check
```

실패하면 JSON의 실패 항목을 먼저 진단한다. ipfrag apply가 모든 네트워크 문제를 고치는 것은 아니다.
D455 제거·discovery 변경·버퍼 추가 확대를 이 패치의 필수 조치로 하지 않는다.
기존 network 도구의 apply/restore 사용법은 [Operations](Operations.md)를 따른다.

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"
```

`INITIAL_POSE_REQUIRED`에서 초기 위치·정합을 확인한다. perception이 필요하면 별도 준비된
터미널에서 아래 명령을 실행하고, 모든 추가 기동이 끝난 뒤 `READY`를 기다린다.

```bash
ros2 run jackal_nav2_bringup nav_session.py perception --initial-pose-confirmed
ros2 run jackal_nav2_bringup check_navigation_ready.py --timeout 15 --settle 2
```

실주행은 [Goal_Navigation](Goal_Navigation.md)의 브리지·컨트롤러 확인과 프로파일 생성 후
동일한 nav 명령에 `--profile-dir /tmp/jackal_motion_profile --enable-motion`을 추가한다.
스택 READY → 버튼 해제·정지 확인 → 기존 재무장 → readiness CLI → 새 목표 순서를 지킨다.

## 세션 기록과 두 호스트 비교

Nav 세션 로그 디렉터리에는 `session.json`, `resources.jsonl`, `stability.json`,
`stability_events.jsonl`, `ros/`가 남는다. 핵심 프로세스 실패는 `failure.json`에 최초 원인을 남기고 세션 종료 코드도 실패로 표시한다. 자원 샘플은 ROS 노드를 추가하지 않고 1초 간격으로
RSS/swap, UDP 오류 증가량, UDP/UDP6 소켓별 드롭·queue·관측 가능한 소유 PID를 기록한다.
프로세스 시작 identity, DDS 환경 및 라이브러리 경로도 남는다. 권한 때문에 소켓 소유자를
못 읽으면 빈 owners로 남기며, 소켓 드롭 0을 통신 전체 무손실로 해석하지 않는다.

기존 `record_navigation_validation.py --navigation`도 같은 자원 기록과 network 프로세스 보고서를
수집하고 stability heartbeat/diagnostics를 bag에 넣는다. installed DDS 패키지 버전도 기록한다.
기록 폴더의 `source_session_dir`로 초기 기동 기록과 연결한다.

NUC는 자동 접속하지 않는다. 사용자가 NUC에서 같은 세션 ID(노트북 nav 로그 폴더 이름)를 지정해
`runtime_resource_audit.py --session-id <ID> --duration 600 --output <새파일>`로 수집한다.
필요한 PID는 `--pid`로 반복 지정한다. 이 standalone 파일은 ROS import 없이도 실행할 수 있다.
종료 후 NUC 파일을 직접 복사한 다음 아래처럼 합친다.

```bash
ros2 run jackal_nav2_bringup runtime_resource_audit.py \
  --compare /path/to/nav-session/resources.jsonl /path/to/nuc_resources.jsonl \
  --output /path/to/new_two_host_summary.json
```

세션 ID가 다르면 거부한다. 양쪽 UDP 증가량·소켓 기록·안정화 전환 시점을 한 JSON에 묶으며,
호스트 간 wall-clock 비교에는 별도의 시간 동기화 기록이 필요하다. 프로세스 XML 환경은
`network_preflight.py --role nuc --pid <PID>`로 NUC에서 별도 확보할 수 있다.

## 연결 후 반복 판정

NUC 재기동 직후와 장시간 켜둔 상태 각각 **3회 이상** 수행하고, 아래 항목을 남긴다.

| 구분 | 기록/판정 |
|---|---|
| 기동 | boot ID, uptime, 환경/XML hash, 양쪽 커널 값과 network 보고서 |
| 안정화 | 최초 180초 완료 시간, 초기 위치 후 READY 시간, reset 횟수, 600초 timeout 여부 |
| 통신 | 양쪽 UDP 오류 증가량, 소켓 드롭, 프로세스/DDS 버전, 카메라·perception 실행 상태 |
| 장애/재무장 | 입력 중단 후 READY 해제와 0 출력, 복구 후 이전 목표가 재개되지 않음 |
| 종료 | managed perception → nav 종료, 잔여 stack PID 없음, 자원 기록/상태 파일 보존 |
| 재기동 | 새 초기 위치·안정화·재무장·새 목표 필요, 과거 목표 재사용 없음 |

장애 주입·주행·재기동은 사용자가 수행한다. 정지 관측 세션을 먼저 통과한 뒤 attended motion
시험을 진행한다. shutdown 로그의 정상 종료만으로 플랫폼 정지를 입증하지 않는다.
P2 서버 축소는 현재 구성과 동일 조건에서 트래픽·안정화 성공률/시간을 측정한 후 결정한다.

기존 opt-in synthetic integration은 표준 network 검사를 우회하는 운영 옵션을 추가하지 않고,
**설치되지 않는 test 전용 launch**로 분리했다. localhost와 domain 86/188,
`/nav2_test/output`만 허용하며 준비 heartbeat는 fixture가 공급한다. 이들은 전체 180초 안정화
실증이 아니다. 이번에는 어떤 ROS integration도 실행하지 않았다.
