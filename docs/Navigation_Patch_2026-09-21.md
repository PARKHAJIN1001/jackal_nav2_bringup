# Jackal 주행 패치 구현·검증 — 2026-09-21

두 로컬 패키지의 구현과 설치 갱신을 완료했다. 실기 목표 도달과 물리 버튼 입력부터 0.5초 정지는 아직 검증하지 않았다. 기본 실행은 `enable_motion=false`이며, 버튼 매핑·차체 확인·실행 중인 브리지 제한을 확인하기 전에는 재무장할 수 없다.

## 변경 범위

기존 미커밋 변경을 유지했다. `jackal_network_bringup`의 ASYNCHRONOUS publication mode와 NUC DDS 2MiB 송신 버퍼 변경을 보존했으며, NUC 배포·플랫폼·브리지·mux는 수정하지 않았다.

- **통신 책임:** network의 `ipfrag_session.py`가 두 커널 항목의 정책, 변경, 복원, 테스트를 소유한다. 항목별 원본·적용 검증·복원 진행 상태를 저장한다. 외부에서 원래 값으로 되돌린 경우도 외부 변경으로 판정한다. 부분 복원 실패는 저장 상태를 유지하며 이어서 복원할 수 있다. legacy Nav2 상태는 `restore --legacy-nav2`로만 처리한다.
- **상태 소비:** [nav_session.py](../scripts/nav_session.py)는 network 상태 CLI만 호출한다. 형제 패키지 Python 소스를 불러오거나 커널 정책을 중복 정의하지 않는다. `status --check`는 0 준비 / 1 미준비 / 2 오류이며 읽기 전용이다. `checks`, `ownership`, `recovery_pending`으로 각 항목을 설명한다.
- **자동주행 정지:** [operator_stop.py](../scripts/operator_stop.py), [operator_stop_core.py](../scripts/operator_stop_core.py)를 추가했다. ○·L1/R1·연결 상실·재시작은 정지를 유지하고 목표를 취소한다. 취소 통신 실패가 출력을 다시 허용하지 않는다. △ 2초 재무장에는 연결 정상, 중립, 정지 1초, 활성 목표 없음, 취소 확인이 필요하다. 재무장 뒤 새 목표가 오기 전까지 출력은 차단한다.
- **출력 경로:** 최종 속도는 기존 Guard만 발행한다. `/nav2/operator_stop`의 true·미수신·0.25초 만료는 Guard 차단 사유다. 관리 노드는 20Hz heartbeat, 진단, RViz 상태를 발행한다. BlueZ는 별도 스레드에서 제한 시간 있는 읽기 전용 SSH로 확인한다.
- **운용 도구:** [calibrate_operator_stop.py](../scripts/calibrate_operator_stop.py)는 forwarding과 motion이 모두 false일 때 실제 버튼을 관측해 새 YAML을 저장한다. [prepare_navigation_profile.py](../scripts/prepare_navigation_profile.py)는 브리지 제한을 읽어 Controller·Smoother·Guard·관리 노드에 같은 상한을 만든다. 어느 도구도 motion을 켜지 않는다.
- **heading 지도:** [map_patch_node.py](../scripts/map_patch_node.py)는 실제 사용한 TF 시각으로 발행한다. pose가 0.3초 넘게 오래되거나 TF가 없으면 새 patch를 발행하지 않는다. RViz에는 `Robot Heading Map Patch`를 추가했다. 기존 `base_link`, 로봇 중심, 전방 +x, 10×10m, 200×200셀 계약을 유지했다. 제어용 local costmap은 `odom` 기준이다.
- **기록·분석:** 기록기에 실제 DDS XML·해시, publication mode, 커널 두 값, network 상태, Joy·정지 진단을 추가했다. [analyze_navigation_bag.py](../scripts/analyze_navigation_bag.py)는 명령 단계별 속도·odom·샘플 age·feedback·경로 끝 방향 오차와 액션 UUID/종료 로그를 CSV/JSON으로 연결한다. 경로 끝 오차는 요청 목표의 독립 측정값이 아니므로 의도한 목표도 별도로 기록해야 한다.

## 시험 프로파일

| 항목 | 값 |
|---|---|
| 진행 판정 | 0.1m / 30초 |
| 목표 허용 오차 | 0.25m / 0.25rad 유지 |
| local/global inflation | 0.45m / scaling 5.0 |
| 정지 반폭 | 전후 0.60m / 좌우 0.40m |
| 감속 반폭·배율 | 전후 1.00m / 좌우 0.50m / 0.70, 중복 감속 없음 |
| 속도 상한 | 요청 0.50m/s·1.0rad/s와 확인된 브리지 제한 중 작은 값 |
| 커널 정책 | 최소 16MiB / ipfrag_time=3초, 기존 큰 메모리 한도 유지 |

센서·TF·명령 timeout, DWB·NavFn, 지도 전용 costmap, 복구 회전·후진 없는 BT, localization/TF 소유권은 유지했다. 차체·장착물·회전 외곽 확인은 `footprint_confirmed`로 기록하며 기본은 false다.

## 검증 결과

| 검사 | 결과 |
|---|---|
| Nav2 Python 단위 검사 | 256개 통과 |
| Network Python 단위 검사 | 36개 통과, 그중 커널 도구 16개 |
| C++ 검사 | AMCL motion model 4개 + relay health 3개 통과 |
| 실제 Nav2 목표 통합 | motion 활성·비활성 2개 통과 |
| 기존 static/safety 통합 | 1개 통과, 감속 0.70과 지도 불변 확인 |
| heading 지도 노드 실행 | 1개 통과, 0°·±90°·180° / 한 셀 이내 / TF 없음·만료 |
| 설치된 관리 노드 실행 | 1개 통과, 기본 정지·20Hz·정상 종료·재시작 |
| 신규·관련 변경 코드 검사 | flake8, pep257 및 변경 C++ 스타일/정적 검사 통과 |
| 설치 | `/tmp/jackal_patch_build` 검증 후 실제 workspace의 두 패키지 갱신 |

최종 목표 통합은 `ROS_DOMAIN_ID=188`, `ROS_LOCALHOST_ONLY=1`, `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`에서 실제 설치본을 사용했다. 실제 Nav2 출력 `/nav2_test/output`으로 차동구동 모형을 움직였다. 이 시험은 중간 속도 명령을 주입하지 않았다. 기존 static/safety 시험의 별도 fault injection과 구분한다.

직진·방향 정렬·목표 변경·일반 취소·지도 복도·○ 정지·Bluetooth 단절 중 Joy 반복·관리 노드 중단/재시작·정지 중 새 목표·짧은 막힘 재개·30초 이상 막힘 실패·도달 불가·LiDAR/TF/명령 단절·motion 비활성을 통과했다. 목표 성공은 액션 SUCCEEDED, 모형 위치·방향 오차와 연속 최종 정지를 함께 검사했다.

최종 비동기 DDS 시험에서 **모형 버튼 상태 입력 → 최종 명령 0은 단회 16.9ms**였다. 이는 실제 Bluetooth 버튼 입력, 네트워크 센서 부하, 구동계 제동을 포함하지 않으며 물리적 0.5초 기준의 검증 결과가 아니다.

추가로 standalone static costmap 종료 시 SIGSEGV를 재현했다. 역추적과 수정 전후 결과를 근거로 layer 라이브러리 해제와 callback 수명 순서가 원인이라고 추론했다. 노드 소멸까지 라이브러리를 유지하도록 [static_costmap_node.cpp](../src/static_costmap_node.cpp)를 수정했고 최종 별도 프로세스 통합 시험에서 정상 종료했다.

전체 `colcon test`가 모든 모드에서 통과했다고 주장하지 않는다. 과거 검증용 Python 파일·기존 preflight 코드의 전체 lint 오류를 보존했고, xmllint의 원격 ROS XSD 접근도 실패했다. 두 manifest는 `catkin_pkg`로 구문·의미 검증했다. 기능 회귀와 변경 코드 lint를 따로 실행했다. 기존 `use_composition=true` 경로는 종료 시 컨테이너 강제 종료가 재현되어 완료로 인정하지 않았다. 최종 통합은 권장 staged launch와 같은 `use_composition=false` 구성이다.

원시 로그·CSV·시나리오 목록: [검증 자료](validation/2026-09-21/navigation_patch/). 큰 rosbag은 `/tmp/jackal_final_goal_run/test_goal_to_final_velocity_an0/capture/navigation_bag`에 있다.

## 실제 호스트 상태와 남은 실기 절차

읽기 전용 조회에서 실제 호스트의 커널 값은 **4MiB·30초**, 저장된 복원 소유 상태는 없었고 `status --check`는 1을 반환했다. 이번 구현 작업에서 실제 sysctl을 적용하지 않았다. NUC BlueZ는 연결 상태를 반환했으나 조회 시 브리지 노드를 찾지 못해 실행 중인 제한/forwarding을 확인하지 못했다. 소스의 기본 제한을 실제 실행 값으로 간주하지 않았다.

[최신 운용 절차](Goal_Navigation.md#2026-09-21-패치-운용-순서)에 따라 다음을 진행한다.

1. 스택 종료 상태에서 network 정책 적용·점검. 과거 128MiB 실측을 새 설정의 증거로 사용하지 않는다.
2. motion=false·forwarding=false로 실제 ○/△/L1/R1 매핑과 중립을 보정하고 차체 외곽을 확인한다.
3. 브리지 상한을 조회해 새 프로파일을 생성한다. NUC 스택을 중복 없이 정리하고 명시적 forwarding 기동 절차를 따른다.
4. 초기 위치·지도/scan 정합 확인 후 △ 2초 재무장, 준비 점검, 새 목표 순으로 진행한다.
5. 사용자 입회하에 정지 → 개방 공간 → 복도 → 장애물 순으로 검증한다. 물리 버튼 영상과 플랫폼 odom을 함께 기록하고 적용 속도의 직진·회전·곡선 각 10회를 검사한다.

정지 기준은 노트북 Joy 수신→최종 명령 0 ≤0.1초, 정상 통신 중 물리 ○ 입력→차체 정지 ≤0.5초, 이후 선속도 ≤0.02m/s·각속도 ≤0.03rad/s가 1초 이상 유지되는 것이다. 이 기준을 넘는 속도 프로파일은 승인하지 않는다. 본 기능은 Nav2 자동주행 정지이며, 수동 조이스틱·RC 전체 차단이나 통신 단절 중 버튼 전달을 보장하지 않는다. NUC 브리지의 명령 timestamp 미검증 한계도 남아 있다.

## 비교 자료

[Clearpath 공식 Humble J100 Nav2 설정](https://github.com/clearpathrobotics/clearpath_nav2_demos/blob/humble/config/j100/nav2.yaml)도 DWB·NavFn과 odom 기준 local costmap을 사용한다. 그 설정의 높은 속도·가속도와 동적 obstacle layer를 이번 지도 전용 운용에 복사하지 않았다. [Jackal ROS 2 Humble 이식 사례](https://github.com/r-shima/jackal_ros2_humble)는 플랫폼·PS4 설정 참고 자료로 확인했다. 이들 사례가 현재 장비의 제동 성능을 입증하지는 않는다.
