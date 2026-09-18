# 2026-09-16 정지 실기 재검증 — 10분 미통과, 종료 완료

이 문서는 [9월 14일 미통과 예비 검증](Stationary_Pilot_2026-09-14.md)의 후속이다.
**600초 full-perception 계측은 완료했으나 TF 안정화 기준 미달**이다. 주행 검증은 하지 않았다.
증거 디렉터리: `docs/validation/2026-09-16/stationary_buffer_trial/`.
활성 원본: `/tmp/nav2_stationary_20260916.W3M8X4/`.

이후 별도 요청으로 [완전 종료·순차 재기동 재현 시험](Relaunch_Reproduction_2026-09-16.md)을 수행했다.
아래 10분 결과와 다른 실행 구간이며, 후속 시험에서는 LiDAR 수신 공백에 따른
FAST IMU buffer 보호 종료가 재현됐다. composition false/true만으로 해결되지 않았다.

그 이후 [정상 기동 절차 확인](Normal_Startup_2026-09-16.md)에서 raw Image 표시 OFF,
FAST 한 차례 재기동 후 센서·perception·초기 pose 수신은 회복됐다.
map 시각 TF 유지 기준은 여전히 미달이며, 아래 10분 합격 판정을 변경하지 않는다.

## 사전 조건과 변경 범위

- 사용자: 정지 상태, 수동 조작·비상정지 가능 확인.
- 사용자가 `moai_nav_viz`를 재빌드했다. 설치 metadata에 figure/trace entry point가 있고 실제 import도 성공했다.
- 사용자가 laptop `net.core.rmem_max=26214400`을 임시 적용했다. `rmem_default=212992`는 유지.
  NUC는 `rmem_max=212992` 유지. 에이전트의 sudo/시스템 설정 변경 없음.
- 새 laptop DDS 소켓 `ss -m`: `rb52428800`. 이전 시험의 `rb425984`에서 변경 반영을 확인했다.
- Laptop chrony Leap Normal, 남은 보정 약 0.008 ms. NUC 서버 192.168.50.1, 최신 응답 Ignored=no.
- 직접 OS 시각 10회 비교: 최소 RTT 0.922 ms 표본에서 NUC−laptop +0.180 ms 추정.
  각 표본 offset +0.125~+0.433 ms, RTT/2 불확실성과 함께 해석. 시계 step 없음.
- 기존 관련 laptop launch 잔여 없음 확인. NUC sensors/platform 서비스 active.
- NUC `forward_cmd_vel=false`를 실행 중 parameter로 확인했다.
- FAST mapper 소스 SHA-256은 배포 확인했던 `b19076f19c2028b9832b8ae7f8f000adab6c0e638f2c462c077471a03fc835fc`와 일치.

## 실행과 카메라 복구

Nav2/relay/RViz → 단일 FAST LIO-only → 별도 perception 순서로 실행했다.
Nav2 `enable_motion=false`와 figure `figure_scale=0.5`를 runtime parameter로 확인했다.
Perception은 새로 생성한 YAML: `/livox/lidar_local`, tracking `odom`, latest-TF fallback false.
TF 소유권은 NUC base→LiDAR, FAST LiDAR→IMU, perception LiDAR→camera를 유지한다.

AMCL 초기 pose 수신은 13:11:52 및 FAST 시작 이후 13:12:42 KST에 기록됐다.
후자의 요청 시각이 최신 odom보다 약 43 ms 미래라는 경고가 있었지만 초기 pose 설정은 이어졌다.
이 초기화 사건과 정상 운용 계측을 구분한다. 자동 pose 재입력이나 stamp 변경은 하지 않았다.

D455는 USB 3로 열거됐지만, 시작 전부터 Protocol error / No such device가 발생해
image와 camera_info 발행자가 없었다. USB 3 인식만으로 영상 정상이라고 판단하지 않았다.
사용자 재연결 후 영상·검출이 복구됐다. 카메라 서비스/NUC 서비스 재시작은 하지 않았다.
초기 kernel/service 로그에는 반복 disconnect 및 장치 정보 읽기 실패가 있다.
케이블/포트/전원/장치 중 원인이 무엇인지는 확정하지 않았다.

## 완료된 예비 계측

| 항목 | 결과 |
| --- | --- |
| localization-only audit 60초 | issues 없음; steady scan→map 및 odom 각각 852/852 |
| full-perception audit 60초 | issues 없음; steady scan→map 및 odom 각각 852/852 |
| full60 root TF 소유권 | map→odom: AMCL 1 writer, odom→base_link: FAST 1 writer |
| full60 TF/topic stamp 역행 | 0회 |
| full60 camera 수신 age | 정상 구간 약 34–51 ms |
| full60 tracking 수신 age | 정상 구간 약 84–124 ms, odom frame |
| 예비 resource 150초 | UDP RcvbufErrors 증가 0, FAST RSS 152644→153128 KiB |

localization-only는 **수집기의 입력 범위**다. 별도 perception 프로세스는 실행 중이었고,
카메라 재연결/복구가 그 전후에 있었으므로 perception을 완전히 끈 대조군으로 해석하지 않는다.
두 audit 모두 초기 listener 구간의 map TF 실패 13개를 별도 집계하고, steady는 100%다.

### 표시 / costmap / 속도 60초

- 최종 TwistStamped 1,196개: nonzero 0, nonfinite 0. 관측 최대 간격 약 0.304초.
- tracking 897개 모두 odom, nonempty, 최대 1 human. 마지막 ID 1.
- figure: 구/삼각형 몸통/ID 라벨, top z=0.85 m, 머리 지름=0.16 m, 중복 marker key 0.
- trace: odom LINE_STRIP, 최대 10 points, 중복 key 0. 상위 tracker의 trace_window=10이 현재 경로 길이를 제한한다.
- 사용자가 RViz에서 figure와 trace 둘 다 보인다고 확인했다. 실제 궤적 정확도 평가와는 별개다.
- static/global/local full costmap 각각 39/39/101개에서 cell hash 변화 0.
- 세 costmap 실제 node-info 구독에는 map/footprint/parameter_events만 있고 LiDAR/scan/detection/tracking 없음.
  TF listener는 별도 노드로 구성될 수 있다. costmap subscription 결과와 root TF audit를 함께 해석한다.
- Guard 진단 independent_stop 59회; enable_motion=false. 유효 명령 통과/제동/센서 단절 시험을 한 것은 아니다.

## 장기 계측과 잔여 관찰

full-perception 600초 audit, 720초 11-PID 자원 기록, 690초 표시/출력 기록을 시작했다.
설정 수집을 포함하므로 각 측정의 시작/끝 구간은 정확히 같지 않다.

장기 recorder의 parameter 조회 구간에 UDP RcvbufErrors가 +67,743 증가했다.
소켓 스냅샷에서 Nav2 container 포트 7666의 drops 증가가 일치했다.
과거와 현재 snapshot 간에는 포트 7667 증가도 있으나 동일 계측 구간이라고 가정하지 않는다.
새 participant/설정 조회 부하와의 시간적 연관은 있으나 패킷 내용이나 원인은 아직 확정하지 않았다.
**버퍼 상향으로 모든 UDP 손실이 해결됐다고 주장하지 않는다.**
수신 버퍼 설정 외에 TF tolerance나 timestamp를 변경하지 않고 끝까지 측정했다.

## 10분 최종 판정

`full600/recording_status.json`: `recorded_with_findings`, 수집 600.035초 완료.
이는 recorder 실패/자료 누락이 아니라 아래 안정화 기준 실패다.

| 항목 | 실측 | 판정 |
| --- | --- | --- |
| steady scan→odom | 8496/8496 = 100% | 통과 |
| steady scan→map | 8145/8496 = **95.869%** | 목표 99% 미달 |
| FAST odom→base_link | writer 1, 잘못된 quaternion 0, timestamp 역행 0 | 해당 구간 통과 |
| AMCL map→odom | writer 1, 잘못된 quaternion 0, **timestamp 역행 5회** | 역행 0 목표 미달 |
| 역행 크기 | 65.87–67.91 ms | 약 한 scan 주기 규모 |
| relay/FAST odometry/scan/tracking 입력 stamp 역행 | 각 0회 | 해당 관측 구간 정상 |
| scan 수신 최대 간격 | 약 1.073초 | 지연/누락 잔여 |
| FAST odometry 수신 age | 정상 구간 약 13–86 ms | 현재 mapper에서 장시간 backlog 미관측 |
| tracking 수신 age | 정상 구간 약 84–145 ms | 영상 복구 후 입력 확인 |

audit의 `odom has invalid quaternion or backwards timestamp`는 child 이름 `odom`, 즉
**AMCL map→odom의 역행**을 가리킨다. FAST `/odom` topic 문제나 quaternion 오류로 읽으면 안 된다.
TF probe가 4개 역행을 `restart_candidate`로 표시했지만, 이는 >2초 수신 공백으로 추정한 label이다.
계측 중 노드를 재시작하지 않았고 topic 수집기의 `phase_events=[]`다.
이 4회를 초기화 예외로 빼지 않고 5회 모두 실패에 포함했다.
초기 listener 구간의 map TF 실패 36개도 별도 보관했다.

### 장기 메모리·네트워크·시각

- 720초 자원 기록: FAST RSS 153984→156708 KiB (150.4→153.0 MiB),
  Nav2 container 172576→173028 KiB, YOLO 1687396→1685824 KiB.
- 11개 검증 PID의 VmSwap 최대 0. 시스템 가용 메모리 최저 약 4.1 GiB.
  시스템 swap 사용은 별도로 증가했으므로 모든 애플리케이션의 메모리 안정성을 주장하지 않는다.
  이 구간에서 검증 대상의 과거와 같은 RSS 폭증은 재현되지 않았다. 장시간/이동/LIVO 누수 해결 보장은 아니다.
- 시스템 UDP RcvbufErrors는 +67,743, 초반 약 40초까지 증가하고 나머지 기록에서는 동일했다.
- Nav2 포트 7667의 후속 120초 표본: 수신 메모리 441600–11997440 bytes,
  drops=1275 고정. 별도 중간 snapshot에는 약 17.8 MB 대기도 있었다.
  증가한 한도 안에서도 큐가 지속적으로 비지 않는 표본이 관측돼, drop 감소와 저지연 달성은 별개다.
  이 소켓 데이터 중 어떤 topic이 지배적인지는 아직 분리하지 못했다.
- 종료 직전 OS 시각 최소 RTT 표본: NUC−laptop −0.127 ms, RTT 0.376 ms.
  표본 offset −0.551~−0.091 ms, chrony 남은 보정 약 0.292 ms, Leap Normal.
  현재 시계 오차가 TF 역행의 원인이라는 근거는 없다. 재부팅/재연결 전체 안정성을 보장하지 않는다.

### 표시·costmap·출력 690초

- 최종 속도 메시지 **13,801개 모두 0**, nonfinite 0, 수신 최대 간격 약 0.097초.
- figure 17,206개 MarkerArray: odom, top z 0.85 m, 머리 지름 0.16 m,
  중복 key 0, DELETE 54개. trace 6,900개: odom, 최대 10 points, 중복 key 0, DELETE 23개.
- tracking은 10,306개 모두 odom/nonempty, 최대 5 human track. 인원 정답·ID 지속성은 평가하지 않았다.
- static/global/local full grid 각각 441/445/1150개: **cell hash와 origin 변화 모두 0**.
  구독 계약과 함께 정적 전용 정책의 이번 정지 운용 결과로 기록한다.
  모든 배치/로봇 이동 상황의 동적 객체 미반영을 일반화하거나 고정 TF 합성 시험을 대체하지 않는다.
- Guard 진단: independent_stop 602회, map→odom stale 69회, odom→base_link stale 1회.
  별도 TF consumer의 지연이 존재했다. `enable_motion=false`였으므로 유효 주행 명령 하의
  정지/감속/제동 또는 센서 고장 안전 시험을 통과했다고 해석하지 않는다.
- scan–map 끝점 거리 median 0 m, p90 약 0.30 m, 0.15 m 이내 약 85.5%.
  절대 위치 정확도는 아니다. AMCL x/y/yaw 분산은 약 0.238/0.091 m², 0.058 rad²로
  별도 실측 위치 비교 및 이동 수렴 시험이 필요하다.

## 해석과 다음 진단 순서

1. 이번 관측에서는 FAST와 relay stamp는 단조 증가하는 반면 AMCL TF만 한 scan 주기 정도 역행했다.
   **AMCL의 TF/message-filter 처리 순서 또는 해당 consumer의 수신 지연** 쪽으로 진단 범위를
   좁힐 근거다. AMCL 내부 callback 순서와 TF 발행 순서를 아직 계측하지 않아 원인 확정은 아니다.
2. 설치 Nav2 AMCL은 1.1.20. [공식 소스](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/amcl_node.cpp)에서
   scan→odom message filter와 scan stamp에 tolerance를 더하는 TF 발행을 확인했다.
   정상적인 미래 stamp 정책은 유지하며 tolerance 증가, stamp 덮어쓰기, 오래된 pose 재주입으로 감추지 않는다.
3. 다음 정지 A/B는 기존 `use_composition:=false` 경로로 AMCL/scan projection을 별도 프로세스로
   분리해 동일 map·perception·시간·속도 차단 조건에서 60초 → 600초를 비교한다.
   더 많은 DDS participant가 생기는 영향도 같이 계측해야 하므로 개선된다고 미리 단정하지 않는다.
   이번에는 A/B를 실행하거나 기본값을 바꾸지 않았다.
4. 차이가 확인되면 container 공유 수신 경로/RTPS 큐와 AMCL message-filter callback의 측정 stamp,
   실행 순서, TF publish 순서를 추가 계측한다. 실제 재현에 근거한 최소 변경만 적용한다.
5. trace가 짧으면 현재 upstream tracker `trace_window=10`과 viz의 3초/100점 상한을 구분한다.
   이번에는 upstream 추적 알고리즘/기본 설정을 변경하지 않았다.
6. D455 USB 연결 안정화, 실제 mount/절대 pose 정확도, 주행·감속·제동·센서 단절 시험은 남아 있다.

## 종료 확인

사용자는 검증 후 노트북 프로세스 종료를 선택했다. 주행 차단 상태에서 세 launch에 거의 동시에
Ctrl-C를 보내 relay 종료 후 FAST의 IMU-only 누적을 피했다. FAST와 perception은 clean exit.
Nav2 static costmap은 종료 중 -6, RViz는 -11, container는 SIGINT/SIGTERM timeout 후
launch가 SIGKILL(-9)했다. **프로세스 종료 완료와 정상 종료 성공은 다르며** lifecycle/RViz 종료 개선도 남아 있다.

launch/자식 PID 24개를 명시해 `ps` 조회한 결과 잔여 없음. NUC
`jackal-sensors.service`, `clearpath-platform.service`는 모두 active로 유지했다.
노트북의 사용자 적용 rmem_max=26214400은 되돌리지 않았다(임시 설정). 주행 목표·sudo·시각 변경 없음.
이번 단계의 패키지 변경은 검증 문서/증거 보관이며 런타임 알고리즘 수정은 하지 않았다.

## 사용자 추가 요청 — 보행자–지도 정합 및 다음 주행 검증

아래는 이번 종료 이후 추가한 **후속 계획**이다. 새 프로세스 실행, frame 설정 변경,
주행 활성화 또는 goal 전송은 하지 않았다. 사용자가 원하는 다음 실기 목표는
**이동 중 TF 유지와 Nav2 goal까지의 자율 이동**이다.

### 1. 보행자를 map 기준 실제 위치와 일치시키기

목표는 실제 보행자와 지도상 figure/trace의 상대 위치가 일치하는 것이다.
이를 위해 tracker 자체의 기준 frame을 반드시 `map`으로 바꿔야 하는 것은 아니다.
현재 `rviz/jackal_nav2.rviz`의 Fixed Frame은 이미 `map`이고,
`launch/bringup.launch.py`는 figure/trace의 `target_frame`을 `odom`으로 지정한다.
따라서 올바른 TF가 있으면 odom 좌표의 보행자도 map 화면에 표시할 수 있다.
연속적인 추적은 odom에 유지하고 전역 위치/표시를 map으로 변환하는 역할 분리를 우선한다.
이는 odom의 연속성과 map localization 보정의 불연속성을 구분하는
[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)에 따른 설계다.

측정 시각 `t`에서의 좌표 관계는 다음과 같다.

```text
p_map(t) = T_map_odom(t) × p_odom(t)
p_odom(t) = T_odom_sensor(t) × p_sensor(t)
```

현재 figure/trace 코드에는 `frame_locked=True`가 있다. Humble RViz의
[MarkerBase 구현](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/marker/markers/marker_base.cpp)은
이 경우 표시 변환에 time zero(최신 TF)를 사용한다. 입력을 측정 시각에 odom으로
변환하는 것과, RViz가 이를 map에 그릴 때 적용하는 시각 정책은 서로 다르다.
따라서 현재 화면을 측정 시각의 map 좌표 정합 검증 완료로 간주하지 않는다.
이 차이가 실제 위치 불일치의 원인인지는 아직 **미확인 가설**이다.

- [ ] 현장 기준점에 사람이 정지한 상태에서 figure의 지도상 XY와 실제 위치를 비교한다.
  로봇 정지, 수동 저속 직진, 회전을 분리해 위치 오차·지연·trace 흔들림을 기록한다.
  사람 검출 중심과 발 위치의 차이, 카메라–LiDAR–base 외부 보정, map 해상도 및
  AMCL pose 오차도 함께 점검한다. 단순히 frame 이름 문제로 단정하지 않는다.
- [ ] **map 기준 figure/trace 출력 옵션**을 검토·구현한다. tracking은 odom에 유지하고,
  표시 전용 경계에서 좌표를 실제로 변환한다. `header.frame_id`만 map으로 덮어쓰지 않는다.
  현재 launch는 target을 odom으로 고정하므로 YAML 값 변경만으로 완료됐다고 하지 않는다.
- [ ] 명시적 map 출력에서는 figure를 입력 측정 시각으로, trace는 각 이력점의 측정 시각으로
  변환한다. 최신 TF/identity 대체 없이 TF 누락과 stale 입력을 처리한다.
  기존 odom frame-locked 표시와 측정 시각 map 표시를 비교해 AMCL 보정 시의
  전체 이동과 과거 궤적 불연속을 구분한다. 최종 표시 정책과 기본값은 검증 후 확정한다.
- [ ] map 출력의 바닥 높이와 figure–trace 접점을 확인한다. 현재 odom.z=0 가정을
  map에서도 무조건 적용하지 않는다. figure 배율 0.5는 시각 크기에만 적용한다.
- [ ] 로봇 회전 + 정지 보행자 합성 시험, AMCL 보정 변화, TF 단절, 오래된 입력,
  ID 소멸 시험을 추가한다. 실측 오차 기준은 현장 기준점 정확도와 함께 정하고 보고한다.

map으로 출력하더라도 잘못된 localization이나 센서 보정 오차가 사라지는 것은 아니다.
보행자를 costmap에 넣지 않는 정책과 독립 LiDAR 정지 경로는 그대로 유지한다.

### 2. 다음 실기 — 이동 중 TF 유지와 Nav2 goal 도달

아래 단계를 순서대로 진행한다. **현재 10분 AMCL 기준 미달 상태에서 바로 자율 goal을
실행하지 않는다.** 각 단계의 실패는 기록하고 다음 단계로 넘어가지 않는다.

1. **정지 기준선 회복:** 앞 절의 composition A/B 등으로 AMCL 시간 역행을 진단한다.
   전체 perception을 켠 초기화 이후 10분 동안 root TF writer 중복·역행 0회,
   scan exact-time odom/map 성공률 각각 99% 이상을 다시 확인한다.
   시각 동기화, D455 영상, raw LiDAR freshness와 메모리/큐도 확인한다.
2. **출력·정지 기능 사전 확인:** 실제 플랫폼으로 전달하지 않는 격리 명령 경로에서
   감속/정지, LiDAR·TF·monitor 출력 단절, NaN 및 timeout 회귀를 확인한다.
   현장에서는 E-stop, 수동 우선권, 플랫폼 0.5초 watchdog 및 빈 시험 구역을 확인한다.
   정상적인 independent_stop도 원인을 확인하며, 통과를 위해 보호 영역을 줄이거나 우회하지 않는다.
3. **사용자 수동 저속 이동:** Nav2 `enable_motion=false`를 유지한 채 사용자 수동 조작으로
   짧은 직진·회전을 수행한다. TF 연속성/역행/지연, scan–map 정합, 보행자 map 위치와
   trace를 계측한다. 수동 경로가 Nav2 Guard를 통과한다고 가정하지 않는다.
4. **사용자 입회 저속 안전 시험:** 앞 단계 통과 후 사용자와 시점·영역을 확인하고서만
   Nav2 motion 및 NUC 전달을 명시적으로 활성화한다. 0.20 m/s, 0.35 rad/s 상한을
   유지하고 더 낮은 시험 속도로 접근·감속·정지 및 입력 단절을 확인한다.
   사람을 시험 장애물로 세우지 않고 안전한 시험 물체와 충분한 여유 공간을 사용한다.
5. **짧은 Nav2 goal:** 통제된 빈 공간의 도달 가능한 짧은 직선 goal부터 시작하고,
   성공 후 회전이 포함된 goal로 확장한다. RViz goal은 `map` 기준이다.
   action 결과, 실제 최종 위치·방향, 실행 중 goal checker 허용치, TF 통계,
   Guard 차단 사유, 속도·자원·통신 로그를 함께 남긴다.
   action `SUCCEEDED`와 실제 목표 위치 도달을 별도로 확인한다.
6. **중단·복구:** TF 끊김/역행, localization 급변, 센서 단절, 예상치 못한 움직임 시
   즉시 수동 정지/E-stop, goal 취소, 전달 차단 후 원인을 확인한다.
   운용 중 시계 step이나 pose 재입력으로 감추지 않는다. FAST 재시작 시에는
   perception 재시작·초기 pose 재입력 후 정지 기준선부터 다시 확인한다.

이번 정책은 사람이나 새 물체를 우회하는 planner가 아니다. 경로가 막히면 감속·정지하거나
goal을 중단할 수 있고, 이를 우회 주행 성공으로 기대하지 않는다. 주행 검증이 끝나면
motion/전달 차단을 복원하고 프로세스 종료 여부를 사용자와 확인한다.
