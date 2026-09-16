# 정적 전용 costmap·figure·독립 정지 감시

최종 사용자 확인 및 다음 작업은 [2026-09-10 인계 문서](Session_Handoff_2026-09-10.md)를
참고한다. 사용자는 오늘 Nav2 RViz의 보행자 figure 표시 성공을 확인했다.
아래 빈 track 표본은 앞선 특정 측정 구간의 기록이며 최종 표시 성공과 구분한다.
localization 안정화와 지속 시각 동기화는 여전히 미완료다.

## 구현 계약

- static/global/local 모두 `/map`의 StaticLayer + InflationLayer만 사용한다.
  local은 `odom`, 4×4 m rolling window, 0.05 m 해상도와 기존 footprint를 유지한다.
  사람이든 새로 놓인 박스든 실시간 costmap obstacle로 추가하지 않는다.
  launch도 sensor layer/filter를 활성화한 대체 파라미터 파일을 거부한다.
- AMCL `/scan`과 수동 2D Pose Estimate는 유지한다. 사람 점 제거는 하지 않는다.
- 별도 perception은 `/livox/lidar_local`, `tracking_frame:=odom`으로 실행한다.
  메시지 정의와 tracker 알고리즘은 바꾸지 않는다. Nav2가 perception을 시작하지 않는다.
- figure 입력 `/ped_tracking` (`moai_nav_msgs/Tracks`), 출력
  `/nav2/pedestrian_figures` (`visualization_msgs/MarkerArray`). object_type 0만,
  ID별 namespace `pedestrian/<int64 id>`와 marker ID 0/1/2를 사용한다.
  몸통은 아래가 뾰족한 TRIANGLE_LIST, 머리는 구, 라벨은 원본 ID다.
  몸통 1.38 m, 어깨 반지름 .25 m, 머리 반지름 .16 m, 전체 1.7 m이며
  바닥을 `odom.z=0`에 고정한다. 원본 frame/측정 stamp의 TF만 사용한다.
  빈 배열/ID 소멸/TF 실패/1 s freshness 만료 시 DELETE, lifetime .3 s.
  추가 예측과 detection 중복 표시는 없다.
- 기본 RViz figure는 켜져 있고 bbox/trace/안전 영역은 선택적인 debug display다.
  bbox/trace는 별도 perception에서 해당 marker 발행도 켜야 보인다.

## 출력 경로와 한계

```text
Velocity Smoother -> Collision Monitor -> Guard -> /j100_0519/nav2_cmd_vel
                    Twist                 TwistStamped

raw LiDAR -> Guard 전처리 -> /nav2/safety_points -> Collision Monitor
                     └──── 독립 최소 정지 영역 검사
```

전처리는 base_link 변환, 비정상 점 제거, footprint 마스킹, z=.10–1.80 m
제한만 한다. 유효한 raw cloud의 모든 점이 필터링되어 빈 결과가 되는 것은
센서 고장이 아니다. 반면 empty/truncated/all-NaN raw cloud는 고장으로 처리한다.
측정 시각을 덮어쓰거나 TF 실패를 identity로 대체하지 않는다.

`config/nav2_safety.yaml`의 한 설정에서 Monitor와 Guard의 정지 영역을 생성한다.
Stop ±.60/±.50 m, Slow ±1.00/±.80 m, 3점 이상, 감속 비율 .3이다.
Humble 1.1.20은 `max_points`보다 **많은** 점일 때 동작하므로 값은 2다.
`base_shift_correction=false`, 최종 출력 상한 .20 m/s/.35 rad/s를 적용한다.

Guard는 20 Hz steady-clock timer로 원본 sensor stamp와 monotonic 수신 age
각각 .30 s, 명령 수신 age .25 s를 검사한다. clock 역행, 필수 TF 누락/노후,
비정상 명령, Monitor 종료 시 0을 출력한다. map→odom의 AMCL 미래 stamp 1 s는
정상 정책으로 유지하며, freshness 검사에서 그 allowance를 명시적으로 반영한다.
`enable_motion=false`가 기본이고 실행 중 파라미터 변경으로 우회할 수 없다.
기존 stamper의 변환 함수를 재사용하지만 직접 stamper 출력 launch는 제거했다.

이는 인증된 안전장치가 아니며 보행자 우회 경로를 만들지 않는다. 영역은 현장
제동 검증 전 시험값이다. NUC의 `forward_cmd_vel=false`, 플랫폼의 .5 s watchdog,
수동 조작/E-stop 우선권은 자동으로 변경하지 않는다. 실주행 전 별도 확인이 필요하다.

## 재현 가능한 검증

워크스페이스를 source한 뒤:

```bash
cd ~/moai_navigation_ws/src/jackal_nav2_bringup
python3 -m pytest test ../moai_nav_viz/test/test_pedestrian_figures.py -q

# 실제 플랫폼과 분리된 localhost 전용 도메인. 출력도 테스트 토픽만 사용.
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 \
  FASTRTPS_DEFAULT_PROFILES_FILE= FASTDDS_DEFAULT_PROFILES_FILE= \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 -m pytest test/integration_static_safety.py -v -s
```

격리 시험은 실제 Nav2 세 costmap, Collision Monitor, Guard, figure를 실행한다.
움직이는 track/cloud에 대한 costmap cell 불변성과 런타임 구독 목록을 검사하고,
감속·정지·독립 stop·empty/stale sensor·TF 단절·NaN·명령 단절·Monitor 실제
프로세스 종료를 검사한다. `/nav2_test/output` 외의 최종 명령 토픽은 사용하지 않는다.
로봇 회전 시 정지 보행자의 odom 위치가 유지되는 것은 별도 geometry 회귀 테스트로 검증한다.

실기는 README의 순서대로 Nav2/relay → FAST-LIVO2 → 별도 perception → 사용자
2D Pose Estimate를 완료한 뒤 로봇을 정지 상태로 유지하고 다음을 실행한다.

```bash
ros2 run jackal_nav2_bringup record_navigation_validation.py --duration 600
```

새 `/tmp/nav2_validation_*` 디렉터리에 실제 파라미터 dump, 설정 사본/해시,
ROS 패키지 버전, 노드 목록, full-perception audit JSON/console을 저장한다.
기존 기록 디렉터리는 덮어쓰지 않는다. 10분 TF 중복·역행 0회, 정상 구간의
scan exact-time 변환 성공률 99% 이상이 목표다. `--require-perception`이 적용되어
camera/accumulated cloud/detection/tracking 미수신도 실패로 기록한다.

TF audit는 매 scan을 .5 s까지 기다려 exact-time TF 성공/실패를 누적한다.
정상 구간과 초기 3초/추정 재시작 구간을 분리하고, 실패를 누락하지 않는다.
TF 역행은 직전/현재 stamp·크기·ROS/경과 수신 시각·writer GID를 기록한다.
`.5 s 초과 odom 역행` 또는 `2 s 초과 gap`은 **재시작 후보라는 추론**이며
실제 프로세스 재시작을 확정하는 증거는 아니다. scan–map 거리 점수만 마지막
bounded scan window를 사용하며 이는 절대 localization 정확도가 아니다.

## 재시작 및 환경 오류

FAST-LIVO2 재시작 시 perception도 반드시 재시작하고 AMCL 초기 pose를 다시 입력한다.
sudo가 필요한 NTP·시스템 변경은 사용자에게 요청한다. TF tolerance를 늘리거나
측정 timestamp를 현재 시각으로 바꿔 오류를 숨기지 않는다.

2026-09-10 perception에서 tf2_ros Buffer 심볼 로딩 오류가 보고되어
`mid360_perception`을 현재 호스트 라이브러리로 clean-cache 재빌드했다.
정확한 실패 시점의 로딩 환경은 확보되지 않아 이전 오류의 원인을 단정하지 않는다.
재빌드 후 `ldd -r`에서 미해결 심볼이 없음을 확인했다.

Nav2 강제 종료 이후 여러 노드가 SHM 초기화에서 멈추는 현상도 재현했다.
임시 UDP-only profile은 즉시 시작됐고, 원본 profile은 미사용 Fast DDS SHM
자원 정리 후 단일 노드는 시작됐으나, lock 파일 없이 남은 포트/세마포어에서
전체 launch 지연이 재발했다. 활성 DDS lock이 없는 상태를 확인하고 해당
46개 파일을 `/tmp/nav2_shm_recovery.ONByX5`로 이동 보관한 뒤 전체 노드 생성과
Collision Monitor 활성화가 복구됐다. 사용자 파일은 삭제하지 않았고
네트워크 패키지도 수정하지 않았다. 이 자원 복구를 launch에 자동 추가하지 않는다.
공식 정리 도구는 [Fast DDS SHM CLI](https://fast-dds.docs.eprosima.com/en/2.6.x/fastddscli/cli/cli.html)에 설명되어 있다.
다른 작업의 ROS 프로세스를 임의 종료하거나 `/dev/shm` 전체를 삭제하지 않는다.

## 검증 상태

- 단위·회귀 테스트: 초기 80개, 마지막 진단 보강 후 83개 통과.
- `colcon test --packages-select jackal_nav2_bringup`: 등록 테스트 13개 통과.
  cppcheck는 설치된 2.7의 알려진 성능 문제 때문에 ament가 자동 생략함.
- 격리 통합 테스트: 통과. 최종 로그:
  `/tmp/nav2_isolated_integration_final/test_static_costmaps_figures_a0/launch.log`.
- 기존 static costmap shutdown abort와 Nav2 container 종료 timeout이 관찰됨.
  운용 중 검증 통과와 별개인 잔여 종료 안정성 문제이며 이번에 해결됐다고 주장하지 않는다.
- 복구 후 12초 사전 점검: `/tmp/nav2_validation_recovered_preinit/audit.json`.
  scan exact-time TF 정상 구간은 odom/map 각각 108/108 성공, TF 역행 0회였으나
  NUC 시각이 노트북보다 약 9.6초 빠름(5회 SSH 왕복 측정)을 확인했다.
  LiDAR stamp 약 9.4초 미래, AMCL TF 약 10.4초 미래로 **시각 정책 실패**다.
  perception도 미실행 상태여서 이 결과를 정상 운용/10분 검증 통과로 보지 않는다.
- full-perception의 중단 없는 10분 정상 운용 검증 및 사용자 입회 제동 검증은 미완료다.
  시각 보정을 위해 이번 Nav2/FAST-LIVO2 테스트 프로세스는 종료했다.
  sudo 시각 변경 및 `jackal-sensors.service` 재시작은 사용자에게 요청하며
  자동으로 실행하지 않는다. NUC `forward_cmd_vel=false`도 그대로 유지했다.
- 지속 NTP, mount 실측, 절대 pose 정확도, 동적 우회 planner: 후속 현장 작업.

### 시각 동기화 후 추가 사전 점검 (2026-09-10)

- `/tmp/nav2_synced_preinit_audit.json`: 12초 계측에서 LiDAR/scan 각각 171개,
  FAST-LIVO2 odometry 171개를 수신했다. 정상 구간 scan exact-time TF는
  odom/map 각각 135/135 성공했고 root TF 역행/중복 writer는 없었다.
  root `/tf`의 map→odom은 `/amcl`, odom→base_link는 `/laserMapping`이 담당했다.
  플랫폼 별도 `/j100_0519/tf`와 구분한 결과다.
- 그러나 odom TF는 수신 시각보다 약 .11–.14초 미래여서 audit는 실패했다.
  SSH 연결 초기 시간을 제외한 동일 연결 왕복 측정에서도 NUC가 .146초,
  후속 측정에서 .228초 앞섰다. 단일 SSH 실행의 중간 시각만으로 작은
  오차를 추정하면 접속 지연이 섞이므로 이 방식은 정밀 확인에 사용하지 않는다.
- 양쪽 커널을 `adjtimex(modes=0)` 및 `adjtime(NULL, &remaining)`으로
  읽기 전용 조회한 결과, 노트북에 -.149초의 pending slew가 남아 있었고
  NUC에는 없었다. `init_time` 종료 후에도 노트북의 점진 보정이 계속된
  상태에서 NUC 시각을 복사해 차이가 다시 벌어진 것으로 판단한다.
  ROS 정지 후 사용자에게 `sudo ntpdate -b time.bora.net`으로 즉시 보정하고
  NUC를 다시 동기화하도록 요청했다. TF 허용 시간/측정 stamp는 바꾸지 않았다.
- D455는 NUC 드라이버에서 USB 2.1로 인식되며 frame timeout과 커널의
  `usb 3-7 ... UVC control ... -32` 오류가 반복됐다. 이는 관측된 입력 장애이며
  케이블/포트의 구체적 고장 원인은 아직 확정하지 않았다. 사용자에게 USB 3
  포트 재연결을 요청했다. 카메라 입력 복구 전에는 full-perception 통과를
  주장하지 않는다.
- 이번 Nav2/FAST-LIVO2도 후속 시각 보정을 위해 종료했다. 주행은 켜지 않았고
  NUC bridge의 `forward_cmd_vel: false`와 0 속도 비전달 로그를 재확인했다.

### Full-perception 재개 및 통신 계층 점검 (2026-09-10)

- 사용자 즉시 시각 보정 후 양쪽 pending slew가 0이 됐고, 동일 SSH 연결
  왕복 측정에서 NUC−노트북은 약 -6 ms였다. D455는 USB 3.2/5000 Mbps로
  다시 인식됐다. 임시 시각 서버는 종료했다.
- `/tmp/nav2_ready_preinit_audit.json`의 15초 점검에서 camera 112개,
  local LiDAR/scan 각각 101개를 실제 수신했다. 측정 시각은 미래가 아니었고
  root TF writer도 기대한 AMCL/FAST-LIVO2 단일 소유였다. 그러나 약 1.8초
  수신 공백과 scan exact-time TF odom 99/101, map 97/101로 기준 미달이었다.
- 사용자 별도 perception의 extractor/tracker 프로세스는 심볼 로딩 오류 없이
  유지됐고, 실행 파라미터 `tracking_frame: odom`을 확인했다.
- 전체 구성의 결과·설정·버전 저장 위치:
  `/tmp/nav2_validation_full_perception_20260910_1950`.
  AMCL, pointcloud_to_laserscan, global costmap의 parameter dump는 timeout으로
  끝났으므로 이 세 노드의 파일은 유효한 실행 파라미터 사본이 아니다.
  설정 YAML 사본과 실제 수집 성공한 다른 node dump를 구분해서 해석해야 한다.
- `runtime_observer.json`의 첫 60초에서 tracking 86개는 모두 odom frame의
  빈 배열이었고 figure는 없었다. 최종 출력 1,145개는 전부 유한한 0 속도였다.
  Guard는 주로 odom TF 노후로 차단했고 독립 stop 판정도 관측됐다.
  `enable_motion=false` 상태의 관측으로, enabled 상태의 실기 제동 통과를
  의미하지 않는다.
- 링크는 양쪽 1 Gbps였으나, 약 12초 표본의 노트북 유선 수신량은 487 Mbps였다.
  `network_delta.json`의 별도 12.31초 표본에서는 노트북 UDP RcvbufErrors가
  45,995, IP ReasmFails가 217,596 증가했다. NUC RcvbufErrors도 14,500 증가했다.
  원인은 단순 대역폭 부족으로 확정할 수 없지만 수신 버퍼/조각 재조립 손실이
  실제 발생했다는 증거다. 당시 rmem_default/rmem_max는 양쪽 212,992 bytes,
  노트북 ipfrag_high_thresh는 4 MiB, ipfrag_time은 30초였다.
- [ROS 2 Humble DDS tuning 원문](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/DDS-tuning.rst)은
  IP 조각 손실과 재조립 버퍼 포화가 긴 수신 정체를 일으킬 수 있음을 설명한다.
  현재 환경에서도 관련 가능성을 추론하지만, 설정 변경 전후 재계측 없이
  단일 원인으로 확정하거나 해결됐다고 주장하지 않는다. 시스템 전역 kernel
  buffer 변경은 별도 사용자 sudo 실행이 필요하고 아직 적용하지 않았다.

### Localization 발산 및 사용자 요청 재시작

- 사람이 시야에 있는 추가 표본 `tracking_chain_probe.json`에서 detection
  429개 중 421개는 1~2명을 포함했지만 tracking 467개는 전부 비어 있었다.
  당시 odom/detection 좌표가 수십만 미터로 발산했고 연속 detection 좌표도
  수백 미터씩 달라졌다. tracker의 1~1.5 m association gate와 확인 3회 조건을
  만족하지 못한 것으로 판단한다. 확인 조건을 완화하거나 detection으로
  figure를 대신 그리는 수정은 하지 않았다.
- 사용자 요청으로 기존 Nav2, FAST-LIVO2, 별도 perception을 종료하고
  perception 없이 Nav2/FAST-LIVO2만 재시작했다. 기록 parent 중단 시 audit
  child가 남는 문제가 확인되어 종료 처리도 보강했다. 기존 audit child는
  자체 600초를 끝내 JSON을 남겼지만, 끝부분에 의도적인 스택 종료 구간이
  포함되므로 **중단 없는 10분 정상 운용 검증으로 취급하지 않는다**.
  해당 JSON에는 AMCL TF 역행 7회, steady scan TF odom 2095/2107,
  map 1992/2107이 기록됐다. 종료 후 writer 이름 조회가 unknown으로 남은
  것은 별도로 구분해야 한다.
- `/tmp/nav2_localization_recovery_audit.json`: 재시작 후 20초 단독 점검에서
  odom 위치는 (-.0048, -.0046, .00037) m로 새 원점 주변에 머물렀고
  root TF 소유자는 AMCL/FAST-LIVO2 각각 하나, 역행 0회였다. 사용자 초기 pose
  수신도 확인됐다. scan–map endpoint 거리 중앙값 .05 m는 정합 지표이며
  절대 위치 정확도는 아니다. steady scan TF odom 236/236, map 224/236으로
  map 변환 성공률은 여전히 99% 목표 미달이다.
- 진단 도구의 기본 점검이 카메라 영상을 불필요하게 구독하던 부분을 수정했다.
  이제 `--require-perception`일 때만 camera/accumulated/detection/tracking을
  구독한다. 전체 구성 검증은 이 옵션을 유지한다. timestamp/TF tolerance는
  변경하지 않았다. 계측 취소 시 별도 subprocess group을 함께 종료하고
  불완전 기록임을 남기도록 수정했다.
- 수정 후 단위·회귀 83개 통과, 수정 Python 3개 파일 flake8 통과.
  재시작 시 perception은 종료했으나 인계 직전 사용자 별도 터미널에서 다시
  실행된 것을 확인했다. localization 단독 안정화 동안 중지를 요청했다.
  실제 주행은 계속 비활성 상태다.

### 일일 종료 및 사용자 최종 확인

- 사용자가 Nav2 RViz에서 보행자 figure가 잘 표시되는 것을 확인했다.
  다음 작업은 반복적인 NUC–laptop 시각 불일치와 localization 안정화,
  figure 시각적 크기 약 0.5배 및 tracking human trace 시각화다.
- 사용자 종료 요청에 따라 오늘 실행한 Nav2/FAST-LIVO2/perception/RViz,
  계측과 laptop ROS CLI daemon의 종료를 확인했다. 기본 NUC 플랫폼·센서
  서비스는 유지했다. 로그/설정의 주요 표본은 `docs/validation/2026-09-10/`에 복사했다.
