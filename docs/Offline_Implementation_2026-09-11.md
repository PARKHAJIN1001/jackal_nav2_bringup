# 2026-09-11 — Jackal 미연결 상태의 사전 구현

기준 문서: [어제 인계 기록](Session_Handoff_2026-09-10.md).
오늘 범위는 **소스 구현과 ROS 노드를 실행하지 않는 단위·정적 검사**다.
Jackal 접속, Nav2/FAST-LIVO2/perception/RViz 실행, 시각 변경, sudo 시스템 변경,
파일럿 주행 및 실기 검증은 하지 않았다. 어제의 현장 결과는 그대로 보존한다.

## 구현한 내용

### 1. Human figure 기본 크기 0.5배

`moai_nav_viz/pedestrian_figures_node.py`에 시작 파라미터를 추가했다.

- `figure_scale=0.5`: 머리+몸통 높이 0.85 m, 몸통 0.69 m,
  머리 반지름 0.08 m, 어깨 반지름 0.125 m.
- `label_height=0.18`: 글자는 별도 크기로 유지하고 위치는 축소된 figure 위로 조정.
- 바닥 `odom.z=0`, 머리/몸통 접점, track ID와 색상, 실제 XY 위치는 유지.
- `figure_scale=1.0`으로 이전 크기를 선택할 수 있다. 유효하지 않은 치수는 거부한다.
- detection/tracking 크기, 메시지 정의, footprint, 안전 영역은 바꾸지 않았다.

### 2. 현재 추적 중인 사람의 trace

`moai_nav_viz/pedestrian_traces_node.py`를 새로 추가했다.
기존 bbox용 `traces_viz_node.py`는 변경하지 않았다.

```text
/ped_traces    (기존 tracker 이력) ─┐
                                 ├─ 전용 trace 노드 → /nav2/pedestrian_traces
/ped_tracking (현재 사람 ID 확인) ─┘                    RViz MarkerArray
```

- `object_type=0`이며 현재 Tracks에 있는 ID만 LINE_STRIP으로 표시한다.
  figure와 같은 ID별 색상, namespace `pedestrian_trace/<int64 ID>`, marker ID 0 사용.
- 기본 상한: 최근 3초, 마지막 100점. 선 두께 0.03 m, `odom.z=0.02 m`.
  점이 2개 미만이면 선을 그리지 않는다.
- 각 `pose_trace` 항목의 header frame과 **그 항목의 측정 시각**으로 변환한다.
  최신 TF나 내장 odometry 대체, 별도 위치 예측/ID 재추정은 없다.
- Tracks와 Traces 양쪽 모두 측정 시각·monotonic 수신 시각 기준 1초 freshness 검사.
  빈 배열/ID 소멸/잘못된 이력/TF 실패 시 DELETE. 10 Hz 검사와 0.3초 lifetime 적용.
  새 envelope header로 오래된 이력을 다시 보내도 계속 표시하지 않는다.
- timestamp가 뒤섞인 이력은 거부한다. 늦게 도착한 메시지가 순서 기준을 초기화하지
  않으며, ROS clock 역행은 양쪽 입력을 초기화한다.
- 이 노드는 FAST-LIVO2 재시작을 완전히 자동 식별하지 않는다. odom 원점이 바뀌면
  perception과 시각화도 재시작하고 2D Pose Estimate를 다시 입력해야 한다.

**이력 길이 주의:** 기존 tracker의 `trace_window=10`, `trace_down_sample=1`은
그대로다. 표시 시간 3초는 상한이며 원본 10개 표본보다 긴 이력을 새로 만들지 않는다.
더 긴 이력이 필요하면 연결 후 실제 갱신율을 확인하고 별도 perception launch의
기존 `trace_window`/`trace_down_sample` 옵션을 조절한다.
`tracking_frame:=odom` 절차도 유지한다. upstream에서 잘못 만든 좌표를 시각화가
사후 복원할 수는 없으므로 tracker 입력/이력의 좌표 품질은 연결 후 확인 대상이다.

### 3. Nav2 설정과 RViz 연결

- [pedestrian_viz.yaml](../config/pedestrian_viz.yaml)을 두 시각화 노드의 설정 파일로 추가.
- 공개 옵션 `use_pedestrian_traces=true`, `traces_topic=/ped_traces`,
  `pedestrian_viz_params_file` 추가. 기존 figure 옵션과 독립적으로 끌 수 있다.
- RViz `Pedestrian Traces` 기본 활성화. 이전 bbox trace는 비활성 debug display 유지.
- Nav2는 perception을 실행하지 않는다. 기존 별도 perception 명령의
  `launch_trace_markers:=false`를 유지해도 `/ped_traces`는 발행된다.
- 세 costmap의 정적 전용 정책 및 LiDAR 기반 독립 정지 경로는 유지한다.
  trace 입력도 costmap 설정에 들어가지 못하도록 정적 계약 테스트를 확장했다.

### 4. 다음 localization 계측의 모드·불완전 기록 구분

[record_navigation_validation.py](../scripts/record_navigation_validation.py):

- `--localization-only`를 추가했다. 기본값은 기존 full-perception 모드 그대로다.
  baseline에서는 perception/viz parameter 조회와 perception 입력 구독을 제외한다.
  로컬 설치 패키지의 버전/설정 스냅샷은 두 모드에서 모두 저장한다.
- full-perception 스냅샷에는 `pedestrian_traces` 파라미터도 포함한다.
- `recording_status.json`에 모드, 요청 시간, 단계별 명령·종료 코드·진행 상태,
  metadata 실패와 취소/종료 상태를 기록한다. 명령 시작 전에 진행 상태를 저장한다.
- timeout/취소 때 자식 프로세스 그룹을 종료하는 기존 처리를 유지하고,
  종료 직전 프로세스가 이미 사라진 경우의 cleanup race도 처리했다.
- 결과는 `recorded`, `recorded_with_findings`, `incomplete`, `interrupted`로 구분.
  report 누락/짧은 측정/조회 실패를 성공으로 표기하지 않는다.
- **기록 완료는 안정화 완료가 아니다.** 기존 audit 판정 외에 무중단 정상 구간,
  odom 발산, 입력 공백 및 현장 정합 확인은 여전히 필요하다.

## 오프라인 검사 결과와 적용 상태

- 기존 Nav2 pytest + figure/trace 순수 메시지·상태 테스트 **131개 통과**.
- 변경 Python 8개 파일 `ament_flake8` 통과.
- 검사 내용: 배율/바닥/접점/위치 불변, int64 ID/색상/중복 방지,
  현재 사람 ID 제한, 이력 제한, 삭제·양쪽 입력 단절·clock 역행,
  잘못된 header/NaN/TF 실패와 원래 수신 시각을 유지한 재시도,
  로봇 회전의 합성 transform을 각 이력 시각에 적용한 정지 보행자 좌표 불변,
  costmap·launch 계약, 계측 모드·timeout·취소·불완전 기록 분류.
- 합성 transform 검사는 순수 함수 테스트다. DDS/실제 TF buffer/RViz 렌더링이나
  실제 perception을 켠 통합 시험을 수행한 것은 아니다.
- 이번에는 colcon 빌드/설치 및 통합·실기·파일럿 검증을 수행하지 않았다.
  새 console entry point와 설정 파일은 **다음 실행 전에 재빌드**해야 한다.

다음 적용 때 사용할 빌드 명령(오늘은 실행하지 않음):

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
source ~/moai_navigation_ws/install/setup.bash
colcon build --symlink-install --packages-select moai_nav_viz jackal_nav2_bringup
source ~/moai_navigation_ws/install/setup.bash
```

위 명령은 나머지 의존 패키지가 기존처럼 설치되어 있다는 전제다.
빌드 후 기본 Nav2 bringup이 축소 figure와 trace를 함께 실행한다.
배율·선 두께·시간 상한은 설정 파일에서 수정한 뒤 해당 시각화 노드를 재시작한다.

## 연결 후로 남긴 항목

1. **지속 시각 동기화:** chrony/NTP 서버 선택과 NUC의 실제 경로·기존 서비스 확인,
   부팅/재연결 이후 offset·pending slew 계측. 영구 설정은 네트워크 패키지 책임이며
   필요한 sudo는 사용자에게 직접 요청한다. 오늘 시각 문제를 해결했다고 보지 않는다.
2. **Localization:** perception 없는 기준선 → 카메라/perception 부하 추가 순서로
   LiDAR/IMU/scan/odom/TF 지연·순서와 UDP/IP 오류를 비교한다. AMCL tuning,
   DDS/커널 버퍼 변경, timestamp 덮어쓰기, TF tolerance 확대는 하지 않았다.
3. **TF writer 추적:** 종료된 publisher 이름이 최종 조회에서 `unknown`이 되는 문제와
   정상 종료/재시작·SHM 잔여 자원 문제는 아직 남아 있다.
4. **표시의 실제 품질:** 절반 크기/라벨 가독성, 실제 trace 길이와 흔들림,
   로봇 회전/입력 단절/FAST-LIVO2 재시작에서 잔상 제거를 RViz에서 확인한다.
5. **무중단 실기 계측:** 초기화 이후 10분 동안 root TF 중복·역행 0회,
   scan exact-time TF 성공률 odom/map 각각 99% 이상과 입력 건강 상태를 확인한다.
   어제의 중단 포함 600초 로그를 이 시험의 성공으로 간주하지 않는다.

`enable_motion=false`와 NUC `forward_cmd_vel=false`를 그대로 유지한다.
주행 및 제동 검증은 사용자 입회 별도 단계다. 보행자 우회 planner는 이번 범위 밖이다.
