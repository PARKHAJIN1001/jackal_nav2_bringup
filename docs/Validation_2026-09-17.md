# 2026-09-17 패치 실행 검증 및 연결 진단

구현, 직접 실행한 테스트, ROS 메시지 및 프로세스 로그를 근거로 작성했다.
전체 판정은 **미통과**다. 단위 테스트와 초기 연결 검증은 통과했으나 종료 충돌,
간헐적인 메시지 공백, lifecycle 응답 유실이 남았다.

검증 빌드와 설치는 `/tmp/jackal_nav2_validation_20260917_KXP5m6/{build,install}`에
별도로 생성했다. 기존 workspace의 build/install은 교체하지 않았다.
원본 실행 로그는 같은 임시 디렉터리에, 주요 증거 사본은
[patch_verification](validation/2026-09-17/patch_verification)에 보존했다.

| 검사 | 관측 결과 |
|---|---|
| CMake configure/build/install | 성공 |
| Python 회귀 테스트 | 162개 통과 |
| C++ RelayHealth | 3개 통과 |
| cppcheck / cpplint / lint_cmake / uncrustify | 통과. 새 C++ 서식 오류 1건 수정 후 재검사 |
| XML | 샌드박스에서는 외부 스키마 조회 실패. 호스트에서 package.xml 검사 통과 |
| flake8 / pep257 | 각각 153건 / 13건 실패. tools/pilot_preflight.py와 기존 검증 스크립트 등의 문제 포함 |
| 격리 안전 통합 테스트 | 기능 단언 통과 후 종료 검증 실패 |
| 최초 전체 스택 35초 감사 | issues=[]; 안정 구간 scan 445/445가 odom·map TF 조회 성공 |
| 최종 비합성 모드 40초 감사 | 안정 구간 scan 415/415가 odom·map TF 조회 성공. 0.3초 수신 공백 기준은 실패 |

**1. 종료 패치는 아직 완료로 판정할 수 없다.**

새 빌드의 독립 static costmap에서 preshutdown, deactivate, cleanup 로그가 나온 뒤
SIGSEGV(exit -11)가 발생했다. GDB 호출 스택은 `rclcpp::CallbackGroup::~CallbackGroup`
→ `NodeBase::~NodeBase` → `LifecycleNode::~LifecycleNode` → costmap 소멸 경로다.
노드 소멸과 콜백 정리 과정의 문제로 추정하지만 메모리 오류의 근본 원인은 확정하지 않았다.
Nav2 `component_container_isolated`도 SIGINT 후 종료하지 않아 SIGTERM, SIGKILL(exit -9)이
필요했다. 실환경 재기동 때도 컨테이너 종료 실패가 재현됐다.

근거: [격리 실행 로그](validation/2026-09-17/patch_verification/clean_integration/test_static_costmaps_figures_a0/launch.log),
[GDB 실행 로그](validation/2026-09-17/patch_verification/gdb_integration/test_static_costmaps_figures_a0/launch.log).
격리 도메인은 86, localhost 전용이며 속도 출력은 `/nav2_test/output`이었다.
기능 단언에는 정적 costmap 불변성, 감속·정지, 센서/TF/명령 단절, NaN,
collision monitor 종료 시 정지 등이 포함된다. 전체 통합 테스트 성공을 의미하지 않는다.
기존 설치본을 잘못 참조한 첫 시도와 두 실행이 겹친 시도는 유효한 패치 검증에서 제외했다.
별도 비합성 통합 시도에서는 costmap 기준값 비교도 실패해 초기화 조건의 추가 확인이 필요하다.

**2. LAN 단절 후 입력 재개만으로 위치 추정이 정상 복구되지 않았다.**

사용자가 LAN 분리·재연결을 알린 120초 기록에서 LiDAR 입력 공백은 최대 5.61초였다.
이후 FAST-LIVO의 odom→base_link 위치는 약 (13189.8, 4754.6, -23161.4)m로 발산했다.
map→odom 타임스탬프 역행 1건(약 66ms)도 관측됐으며 quaternion 오류 수는 0이었다.
이 구간을 정상 수동 주행 성공으로 계산하지 않았다. FAST-LIVO 내부에서 단절을 처리한
정확한 수치적 경로는 아직 조사하지 않았다. 단절 후 재초기화와 동작 허용 조건의 연동이 필요하다.

근거: [LAN 단절 포함 감사](validation/2026-09-17/patch_verification/manual_motion_audit.json).

**3. 재기동 이후 컨테이너 실행에서 scan·AMCL 갱신 지연이 관측됐다.**

첫 재기동에서는 LiDAR·odom·인지 입력이 이어지는 동안 scan이 최대 28.37초 끊겼다.
동일 입력의 동시 비교에서 기존 `/scan`은 약 9.63Hz, 최대 공백 1.27초였고,
별도 프로세스의 변환 출력은 약 14.58Hz, 최대 공백 0.186초였다.
scan 컴포넌트를 unload하고 같은 설정의 별도 프로세스로 교체했으나,
AMCL map→odom TF에는 종료 시점 기준 8.63초의 지연이 남았다.
컨테이너 실행 경로가 지연에 관여한다고 추정한다. executor 문제와 DDS 문제를
이 관측만으로 분리하거나 원인을 확정할 수는 없다.

최종 실행에서는 `use_composition=false`로 전환했다. 안정 구간의 시각 일치 TF 조회는
415/415 성공했고, scan의 최대 수신 공백은 0.439초, odom은 0.431초였다.
따라서 위치 추정 연결은 회복됐지만 0.3초 연속성 기준은 여전히 실패다.
정적 지도 endpoint 거리의 90백분위는 약 0.15m이며 이는 실제 pose 정확도의 증명이 아니다.

근거: [동시 비교](validation/2026-09-17/patch_verification/restart_01/projection_comparison.json),
[컨테이너 AMCL 지연](validation/2026-09-17/patch_verification/restart_01/recovered_audit.json),
[최종 비합성 실행 감사](validation/2026-09-17/patch_verification/restart_02/audit.json).

**4. lifecycle 서비스 응답 유실도 실제로 발생했다.**

최종 재기동의 static costmap은 configure를 수행했지만 change_state 응답 전송이
timeout으로 실패했다. 관리자는 configure 단계에서 기다렸고 노드는 inactive에 남았다.
이후 원래 autostart가 요청해야 할 activate 전이를 직접 요청했고, 성공 응답과
active [3] 상태를 확인했다. 이는 자동 기동 절차가 통과한 것으로 계산하지 않는다.

근거: [노드 실행 로그](validation/2026-09-17/patch_verification/restart_02/nav2.log),
[복구 전 상태](validation/2026-09-17/patch_verification/restart_02/static_costmap_state.log),
[직접 활성화 후 상태](validation/2026-09-17/patch_verification/restart_02/static_costmap_state_after.log).

**5. preflight의 시계 오프셋 판정에는 측정 오차 문제가 남았다.**

preflight는 약 +37.07ms 오프셋, 평균 RTT 75.96ms로 실패했다.
지속 SSH 연결을 사용한 별도 측정에서는 10개 표본이 약 -0.205~+0.250ms,
최저 RTT 표본은 -0.141ms / RTT 0.937ms였다. 양쪽 시간 동기 상태도 정상이었다.
따라서 preflight 실패는 매번 SSH 채널을 열어 측정하는 방식의 비대칭 지연에 의한
오판으로 추정된다. 시간 설정을 변경하지 않았다.

NUC rmem_max=212992 경고와 노트북 Wi-Fi 절전 활성화 경고도 관측됐다.
노트북의 별도 30초 자원 관측에서는 UDP RcvbufErrors가 291 증가했다.
이는 호스트 전체 카운터이며 특정 토픽 손실의 원인으로 확정하지 않는다.

근거: [preflight](validation/2026-09-17/patch_verification/preflight.json),
[지속 연결 시계 측정](validation/2026-09-17/patch_verification/clock_probe.json),
[자원 기록](validation/2026-09-17/patch_verification/connected_resources_active.jsonl).

현재 실행은 Nav2, FAST-LIVO2, `mid360_bringup/perception.launch.py`, 별도 scan 변환의
네 터미널 세션이다. FAST-LIVO2는 `/livox/lidar_local`과 `/livox/imu`를 사용하며
영상 입력은 비활성화했다. perception은 MID360·YOLO detection/tracking을 수행한다.
Nav2는 비합성 모드이며 자체 scan 변환은 꺼서 중복 발행을 피했다.
FAST-LIVO2/perception의 외부 기동은 정상 구성의 전제이며 패키지 누락 결함이 아니다.

`enable_motion=false`, NUC `forward_cmd_vel=false`를 확인했다.
최종 15초 관측에서 Nav2 속도 출력 301개는 모두 0이었다.
자율주행 목표나 비영 속도 명령을 실차에 발행하지 않았다.
현재 런타임 우회 설정을 소스 launch의 기본값에 반영하지 않았다.
다음 수정 우선순위는 종료 수명주기, DDS/lifecycle 응답·입력 공백 처리,
단절 후 위치 추정 재초기화, preflight 오프셋 측정 순이다.
