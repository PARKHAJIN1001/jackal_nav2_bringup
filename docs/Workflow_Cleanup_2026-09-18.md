# 2026-09-18 실행 절차 정리

## 변경 결과

오늘 검증된 localization 설정을 유지하면서 기동·종료를 관리하는 도구와 [운영 절차](Operations.md)를 추가했다. 기존 launch 직접 실행도 유지한다.

- `scripts/nav_session.py`: `check`, `nav`, `perception`, `status`, `stop`. 별도 foreground 터미널, 고정된 사용자별 잠금, 기존 프로세스 탐지, PID/시작 시각/boot ID를 통한 종료 대상 확인을 제공한다. Nav2 종료 시 종속된 managed perception을 먼저 종료한다. 중단·실패 시 다음 단계로 진행하지 않는다.
- `scripts/ipfrag_session.py`: 임시 128MiB apply/status/restore. 원래 값·boot ID·network namespace를 root 소유 `/run`에 보존한다. 다른 실행의 설정을 인계하거나 원래 값을 추측하지 않는다. 활성 스택·외부 변경·다른 namespace에서는 복원을 거부한다. 적용/복원 뒤 실제 값을 읽어 확인한다.
- 기존 `topic_ready_gate.py`: 기본 동작은 그대로 두고 `require_map_to_odom=false` 옵션을 추가했다. 별도 perception 세션만 true로 지정해 초기 위치 지정 후 scan 시각의 map TF를 확인한다.
- `prepare_perception_config.py`: 기존 토픽·TF 프로파일을 유지하며 upstream 연결 검사 시간만 10초에서 60초로 늘렸다. 토픽 연결 검사와 실제 검출 메시지 수신은 별개임을 안내한다.
- `nav_bringup.launch.py`: `managed_session=false`가 기본이다. wrapper가 true로 실행할 때만 별도 managed perception 명령을 안내하고, 프로파일 생성은 그 세션에 맡긴다. 직접 launch에서는 기존 프로파일 출력 절차를 유지한다.
- launch 자식 출력은 터미널과 ROS 로그 양쪽에 남기며, `session.json`에 명령·소유자·시작/종료 시각·상태·로그 위치를 저장한다. 기본 로그 경로는 `~/.ros/nav_sessions`다.
- README의 과거 상태 요약을 역사적 기록으로 분리하고 현재 운영 절차를 연결했다. 기존 진단·검증 산출물은 보존했다.

## 유지한 동작

relay → FAST → Nav2 준비 검사, 수동 초기 위치 지정, 별도 perception 순서를 유지했다. AMCL/FAST 추정 파라미터, 센서 토픽, 장착 변환, TF 소유권, costmap, 제어·정지 관련 파라미터는 이번 정리에서 변경하지 않았다. 새 wrapper의 자동주행 출력은 false로 고정했다. CPU/RAM 보호 정책이나 영구 커널 설정은 추가하지 않았다.

128MiB 설정은 이제 터미널 생존에 연결하지 않고 **apply → 스택 기동/검증 → 스택 종료 → restore**로 관리한다. 따라서 터미널이 닫힐 때 스택보다 먼저 한도가 복원되는 문제를 피한다. 이전 임시 hold 스크립트가 관리하던 실행은 그 절차로 종료·복원한 후 새 방식으로 전환한다.

## 검증

- workspace 환경을 source한 전체 Python 회귀 검사: **213 passed**. 최초 전체 검사에서 workspace source 누락으로 2개 패키지 조회 테스트가 실패했고, 환경 수정 후 모두 통과했다.
- 새 프로세스 관리 테스트는 실제 가짜 자식 프로세스를 사용해 종료 후 회수, 관련 없는 프로세스 보존, perception → Nav2 종료 순서, PID 재사용 방지, 중복 잠금, gate 실패 후 후속 실행 차단을 확인했다. 로봇 프로세스는 사용하지 않았다.
- 커널 도구는 임시 일반 파일로 apply/restore/idempotence와 실행 중 복원 거부, 외부 값 변경, boot/namespace 불일치를 검사했다. 실제 커널 설정을 변경하는 테스트는 수행하지 않았다.
- CMake build/install 성공. 실제 workspace 대신 `/tmp/nav_workflow_20260918_build`, `/tmp/nav_workflow_20260918_install`을 사용했다. 설치된 두 도구의 `ros2 run ... --help`와 launch `--show-args`를 확인했다.
- CTest의 AMCL motion model, relay health, readiness gate, session 관리 **4개 대상 통과**.
- localhost-only ROS domain 187 통합 검사: publisher-only 입력 차단, fresh cloud+IMU 요구, 초기 기동의 map TF 비요구, 별도 perception 기동의 scan-time map TF 요구 통과. 로그 `/tmp/nav_gate_integration_9b2tnwk0`. 이 테스트는 가짜 lifecycle·센서·TF를 사용하며 실제 AMCL/로봇 검증이 아니다.
- 변경한 Python 파일의 ament flake8/pep257 및 diff whitespace 검사 통과.

## 적용 범위와 다음 확인

실행 중인 Nav2/FAST/perception을 재기동하거나 종료하지 않았다. 실제 호스트 커널 설정도 변경하지 않았다. 주 workspace의 새 실행 파일 설치는 다음 정상 종료 후 패키지 재빌드로 반영한다. 소스 변경 일부는 기존 symlink-install에 연결되므로 다음 실행부터 읽힐 수 있지만, 이미 실행 중인 Python 프로세스를 다시 불러오지는 않았다.

운영 도구가 표시하는 `running`은 프로세스 상태이며, 모든 노드의 건강 상태나 정합의 정확성을 뜻하지 않는다. SIGKILL 등으로 관리자만 종료돼 남은 프로세스는 자동 인계·일괄 삭제하지 않고 원래 터미널과 `status`로 확인한다.

다음 실기 확인은 기존 스택과 임시 설정의 정상 종료·복원, 재빌드, 새 절차를 통한 초기 위치·perception 기동, 순서 있는 종료·복원이다. 그 후 [합의한 다음 목표](Session_Handoff_2026-09-18.md)의 2D Goal Pose 자율주행 연결로 진행한다.
