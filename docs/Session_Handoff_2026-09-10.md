# 2026-09-10 작업 마무리 및 다음 task

## 최종 상태

**사용자가 Nav2 RViz에서 사람이 3D figure로 잘 표시되는 것을 확인했다.**
표시 기능의 실기 연결은 확인됐지만, localization의 지속 안정성과 NUC–laptop의
반복적인 시각 불일치는 아직 해결되지 않았다. 사용자는 figure가 커 보인다고
평가했고, 다음 작업으로 약 절반 크기와 tracking human의 trace 표시를 요청했다.

이 문서는 오늘의 인계 기록이다. 아래 후속 기능과 시스템 설정 변경은 아직
구현·적용하지 않았다. 오늘 띄운 Nav2, FAST-LIVO2, perception, RViz, 계측,
임시 시각 서버, laptop ROS CLI daemon은 모두 종료 확인했다. NUC의 기본
플랫폼·센서 시스템 서비스는 유지했고, 실제 주행 출력은 활성화하지 않았다.

## 오늘 완료·확인한 내용

- **정적 전용 costmap:** static/global/local 모두 사전 지도 기반
  StaticLayer + InflationLayer. 센서·detection·tracking 장애물 입력을 제거하고
  local의 odom rolling 4×4 m / 0.05 m 해상도 및 footprint는 유지했다.
  새로 놓인 물체도 costmap에 추가하지 않는 정책이다. MapEncoder의 정적 정책도 유지했다.
- **보행자 figure:** `/ped_tracking` → `/nav2/pedestrian_figures` 경로,
  머리 구 + 역원뿔 몸통 + ID 라벨 구현. odom 기준 바닥 정렬, track ID 유지,
  stale/소멸/TF 실패 처리 및 제한된 lifetime을 적용했다. 실제 RViz 표시 성공은
  사용자의 최종 확인이다. 앞선 자동 계측에서 빈 track 배열만 보였던 구간을
  덮어쓰거나 해당 계측도 표시 성공이었던 것으로 해석하지 않는다.
- **독립 정지 감시:** raw LiDAR 전처리 → Collision Monitor → Guard →
  기존 TwistStamped 출력. Guard의 독립 stop/freshness/TF/명령/clock 검사를
  추가하고 직접 stamper 출력 우회 경로를 제거했다. 실제 정지 모드에서 0 출력과
  차단 사유를 확인했지만, 주행 중 제동 성능 검증은 하지 않았다.
- **TF 책임:** 정상 점검 구간에서 root `map→odom`은 AMCL,
  `odom→base_link`는 FAST-LIVO2 각각 단일 writer였다. 별도 플랫폼
  `/j100_0519/tf`는 합치지 않는다. 구조가 맞는 것과 시간적으로 안정적인 것은 별개다.
- **perception 연결:** 사용자가 별도 실행하며 `/livox/lidar_local`,
  `tracking_frame:=odom`을 사용한다. base→LiDAR와 LiDAR→IMU static TF 중복을
  막고 perception은 기존 calibration의 LiDAR→camera 연결만 담당한다.
- **환경 복구:** `mid360_perception` 재빌드 후 tf2 Buffer 심볼 오류 없이
  실행됨을 확인했다. 실패 당시의 정확한 라이브러리 조합은 확보하지 못했다.
  D455는 재연결 후 USB 3.2/5000 Mbps 인식 및 영상 수신을 확인했다.
- **진단 개선:** 전체 측정 구간의 scan exact-time TF 성공률, TF 역행 크기·stamp·writer,
  단계별 지연을 기록한다. 단독 localization 점검은 이제 카메라를 구독하지 않는다.
  전체 perception 점검에만 `--require-perception`을 사용한다. 계측 취소 시
  자식 프로세스 그룹이 남지 않도록 종료 처리를 보강했다.

## 검증 수준과 아직 남은 문제

- 최신 단위·회귀 **83개 통과**, 수정 Python 3개 파일 flake8 통과.
  초기 구현 기준 colcon 등록 테스트 13개 및 격리 통합 시험도 통과했다.
  마지막 진단 수정 후 검증은 직접 pytest/flake8이며 colcon 전체 재실행과 구분한다.
  설치된 cppcheck 2.7은 ament가 자동 생략했으므로 정적 분석 통과로 세지 않는다.
- 격리 시험에서는 움직이는 객체에도 costmap cell 불변, figure 형상/삭제/timeout,
  독립 stop 및 sensor/TF/명령/Monitor 장애 처리를 검증했다.
- **안정화된 10분 실기 시험은 미완료다.** 저장된 600초 audit는 사용자 재시작 요청에
  따른 종료 구간을 포함한다. 그 안의 AMCL TF 역행 7회, map 변환 기준 미달과
  종료 후 writer 이름 `unknown` 등을 정상 운용만의 결과로 단정하면 안 된다.
- Full perception 중 FAST-LIVO2 odom 좌표가 수십만 미터로 발산했다.
  같은 시기에 odom으로 변환한 detection 좌표도 급변하고 track 배열은 비었다.
  tracker의 association 조건을 만족하지 못한 것으로 **추론**한다. 이것을
  figure 노드의 표시 실패나 AMCL 설정만의 문제로 단정하지 않는다.
- 재시작 후 단독 20초 점검에서 odom은 새 원점 근처로 돌아왔고 TF 역행은 없었다.
  steady scan TF는 odom 236/236, map 224/236이었다. map은 99% 목표 미달이다.
  scan–map endpoint 중앙 거리 .05 m는 정합 지표이며 절대 pose 정확도가 아니다.
- **시각 문제:** NUC가 약 9.6초 앞선 경우가 있었고, `init_time` 종료 후에도
  laptop에 pending slew가 남아 두 시각이 다시 벌어지는 것을 관측했다.
  ROS 정지 상태에서 사용자가 즉시 보정하고 NUC를 다시 맞춘 뒤 약 6 ms 차이까지
  줄었지만, 지속 동기화와 재부팅 후 재현성은 확보되지 않았다.
- **통신 문제:** 1 Gbps 링크에서도 12.31초 표본에 laptop UDP 수신 버퍼 오류
  45,995회, IP 재조립 실패 217,596회가 증가했다. NUC 수신 오류도 증가했다.
  시각 불일치와 별도로 실제 전송 손실이 있었다. 이것이 발산에 기여했을 가능성은
  있으나 유일한 원인으로 확정하지 않는다. kernel buffer 변경은 아직 안 했다.
- **종료 문제:** Nav2 container 종료 지연/강제 종료와 static costmap shutdown abort,
  이후 Fast DDS SHM 잔여 자원 문제를 관측했다. 무조건적인 SHM 삭제나 자동 청소는
  추가하지 않았다. 개별 종료 원인과 재시작 안정성은 별도 개선이 필요하다.

## 다음 task — 우선순위와 완료 조건

### 1. NUC–laptop 지속 시각 동기화

- [ ] 매번 수동 `init_time`/시각 복사를 반복하지 않는 공통 시각 기준을 구성한다.
  NUC의 외부 인터넷 경로 제약을 확인하고, 접근 가능한 LAN 시각 서버를 포함한
  chrony/NTP 구성을 검토한다. 기존 시간 서비스와 충돌하지 않도록 한다.
- [ ] 부팅·NUC 재부팅·네트워크 재연결 이후에도 오차와 동기화 상태를 확인한다.
  오차 허용 기준과 복구 시간을 정하고 운용 중의 누적 drift도 측정한다.
- [ ] 시작 전 동기화 상태를 확인하고, 불량하면 주행 차단 이유를 명확히 표시한다.
  센서 timestamp를 현재 시각으로 덮어쓰거나 ROS `/clock`으로 대신하지 않는다.
- [ ] sudo가 필요한 설치·서비스·시스템 설정 변경은 사용자에게 직접 요청한다.
  영구 설정과 일회성 시험을 구분하고 복원 방법을 남긴다.

### 2. Localization 안정화 및 입력 지연 원인 분리

- [ ] 먼저 perception 없이 odometry/AMCL 기준선을 확보한 후 camera/perception
  부하를 단계적으로 추가한다. 불필요한 진단 구독이 입력 부하를 바꾸지 않게 한다.
- [ ] LiDAR/IMU 원본, relay, scan, odometry, AMCL TF의 지연·수신 공백·순서 역전과
  UDP/IP 오류를 함께 비교한다. 커널 버퍼·DDS 전송/구독 설정은 측정에 근거해
  하나씩 시험하고 전후 결과를 남긴다. 네트워크 설정 책임은 networking 패키지에 둔다.
- [ ] FAST-LIVO2 발산 발생 조건과 재시작 복구를 재현한다. 먼저 입력 문제를
  구분하고 AMCL/TF tolerance 또는 tracker gate 확대만으로 증상을 감추지 않는다.
- [ ] 전체 perception을 켠 **중단 없는 초기화 이후 10분** 동안 root TF 중복·역행 0회,
  scan exact-time TF 성공률 odom/map 각각 99% 이상을 검증한다. 센서 freshness와
  odom 발산 여부도 따로 판단한다. 수동 pose 입력·재시작은 정상 구간과 분리한다.
- [ ] 정상 종료·재실행에서 고아 프로세스/SHM 초기화 정체가 재발하지 않는지 검증한다.

### 3. 3D human figure 크기 약 절반으로 축소

사용자 요청의 "절반"은 **시각적 선형 치수 0.5배**로 해석한 다음 구현 제안이다.
위치 좌표나 실제 사람의 detection/tracking 크기를 절반으로 만드는 것이 아니다.
머리 반지름은 유지하되, 몸통 높이만 줄이고자 한다.

| 치수 | 현재 | 0.5배 제안 |
| --- | ---: | ---: |
| 머리+몸통 전체 높이 | 1.70 m | 1.00 m |
| 몸통 높이 | 1.38 m | 0.69 m |
| 머리 반지름 | 0.16 m | 0.16 m |
| 어깨 반지름 | 0.25 m | 0.125 m |

- [ ] figure 노드의 시각화 배율 파라미터로 조절하고 기본값을 0.5로 한다.
  바닥 `odom.z=0`, 구/몸통 접점, track ID와 색상은 유지한다.
- [ ] 라벨 위치는 작은 figure에 맞추되 글자 크기는 별도 조절해 읽기 쉽게 한다.
- [ ] 실제 track 위치·메시지 정의·costmap·footprint·Collision Monitor/Guard 영역은
  바꾸지 않는다. 배율·바닥 높이·형상 비율 회귀 테스트를 추가한다.

### 4. Tracking human trace 시각화

- 이미 `/ped_traces` (`moai_nav_msgs/Traces`)와 `moai_nav_viz/traces_viz_node.py`,
  `/ped_trace_markers`, RViz의 비활성 Debug Track Traces display가 있다.
  오늘 Nav2용 perception 명령은 `launch_trace_markers:=false`였다.
- 기존 노드는 과거 bbox를 표시하고, 마지막 입력을 timer로 계속 재발행한다.
  코드상 freshness 검사, 빈 `pose_trace` 보호, 안정적인 ID 정책을 보강할 필요가 있다.
  int64 trace ID를 Marker int32 ID에 직접 넣는 부분도 점검 대상이다.
  따라서 기존 표시를 켜는 것만으로 새 trace 요구의 완료로 보지 않는다.
- [ ] 기존 tracker의 이력을 우선 활용하고 같은 객체의 새 ID를 추정하지 않는다.
  현재 추적 중인 사람(object_type=0)만 대상으로 figure와 ID/색상을 맞춘다.
- [ ] **제안:** odom 기준의 얇은 LINE_STRIP으로 최근 이동 이력을 표시한다.
  궤적 길이/시간 범위/선 두께는 조절 가능하게 하고 bbox trace는 디버그용으로 분리한다.
  구체적인 시간 범위와 display 구성은 다음 task에서 확정한다.
- [ ] 각 이력의 frame/측정 시각을 올바르게 처리하고, 로봇 회전 때문에 정지한
  사람의 궤적이 따라 움직이지 않는지 확인한다. 최신 TF/내장 odometry로 임의 대체하지 않는다.
- [ ] 빈 이력, ID 소멸, 입력 단절/오래된 이력, TF 실패, clock/FAST-LIVO2 재시작 시
  잔상을 정리한다. namespace에 int64 ID를 사용하고 marker lifetime도 제한한다.
- [ ] trace는 RViz 전용이다. costmap 장애물 입력이나 보행자 우회 planner에는 연결하지 않는다.

## 다음 실행 시 유지할 운영 원칙

1. 시스템 시각·센서 상태를 먼저 확인한다. 오늘 USB 3 복구 상태도 재확인한다.
2. Nav2/relay → FAST-LIVO2 → 사용자 별도 perception → 수동 2D Pose Estimate 순서.
   localization 단독 진단 단계에서는 perception을 켜지 않는다.
3. FAST-LIVO2를 재시작하면 perception도 재시작하고 초기 pose를 새로 입력한다.
4. `enable_motion=false`, NUC `forward_cmd_vel=false`를 유지한다.
   실주행은 E-stop/수동 우선권/watchdog 확인 후 사용자 입회 별도 단계다.
5. 안전·시각화 코드는 navigation/viz 패키지, 네트워크/시각 통신 설정은
   `jackal_network_bringup`의 책임으로 구분한다. Nav2가 perception을 중복 실행하지 않는다.

## 보관 자료

주요 자료는 `/tmp` 삭제에도 인계 내용이 남도록 레포 안으로 복사했다.

- [상세 구현·검증 기록](Static_Safety_Figures_Validation.md)
- [중단 구간 포함 full-perception audit](validation/2026-09-10/full_perception/audit.json)
- [재시작 후 localization 단독 audit](validation/2026-09-10/localization_recovery_audit.json)
- [detection/track/odom 비교 표본](validation/2026-09-10/full_perception/tracking_chain_probe.json)
- [UDP/IP 오류 증가량](validation/2026-09-10/full_perception/network_delta.json)
- [실행 패키지 버전·설정 해시](validation/2026-09-10/full_perception/versions.json)

같은 폴더에 실행 환경, 설정 사본, Guard/Monitor 파라미터와 부분 perception 로그도
보관했다. `amcl.yaml`, `pointcloud_to_laserscan.yaml`,
`global_costmap_global_costmap.yaml`은 조회 timeout 기록이므로 유효한 parameter dump가 아니다.
전체 ROS 로그/이미지/rosbag을 영구 보관한 것은 아니며 사용자 최종 figure 확인 시점의
별도 화면 캡처도 없다. 종료 이후 실제 주행 시험이나 새 시스템 설정 적용은 하지 않았다.
