# TF / Localization 실기 검증 — 2026-09-10

## 검증 범위와 기준

Jackal + MID-360 + FAST-LIVO2 (LiDAR/IMU 모드), ROS 2 Humble,
`frontier_10F` 지도에서 직접 launch하여 확인했다. 자동 주행 goal 또는
속도 명령은 보내지 않았다. 사용자가 RViz에서 초기 pose를 지정했고,
수동 이동 뒤 정지 상태를 알려주어 그 시점부터 정지 기준 측정으로 분리했다.

판정 기준은 다음과 같다.

- 루트 `/tf`에서 `map -> odom`은 `/amcl` 단독 발행.
- 루트 `/tf`에서 `odom -> base_link`는 `/laserMapping` 단독 발행.
- `base_link -> livox_frame`은 NUC의 `/base_to_mid360_static_tf` 단독 발행.
- `/j100_0519/tf`의 wheel/EKF 트리는 루트 TF에 합치지 않는다.
- scan 측정 시각으로 `odom`, `map`에 변환할 수 있어야 한다.
- local costmap의 sensor origin이 voxel 높이 범위 안에 있어야 한다.

`tf_localization_audit.py`는 C++ 수집기가 읽은 DDS writer GID를 endpoint의
노드 이름에 대응시킨다. 노드 이름이 같더라도 writer가 둘이면 중복으로
판정한다. AMCL의 `transform_tolerance: 1.0`에 따른 미래 TF stamp와
과거의 static TF stamp는 정상으로 취급한다. 기본 age 허용치는
0.5초이고, AMCL의 미래 허용치는 1.1초다. 이는 현재 실기 설정의 진단
기준이며 임의의 로봇/TF 설정 모두에 대한 일반 기준은 아니다.
현재 평탄한 층의 지도에서는 `map -> base_link` 높이도 ±0.2 m 안인지
검사한다. 이는 재시작 전 odom에 대한 AMCL 보정이 남아 높이가 어긋나는
경우를 잡는다. 경사로·다층 3D 지도에서는 이 평면 기준을 재검토해야 한다.

## 발견 및 수정

### 1. 시각 동기화

노트북의 `init_time`은 `sudo ntpdate time.bora.net`이며 노트북만 바꾼다.
NUC에는 외부 인터넷 경로가 없어 이 실행만으로 NUC가 동기화되지 않았다.
사용자가 sudo로 NUC를 노트북 시각에 맞춘 뒤 실측을 진행했다.
시각 전달에 사용한 임시 HTTP 서버는 종료했다.

실기는 `use_sim_time: false`로 통일한다. `/clock`을 일부 노드에만 넣거나
TF 허용 시간을 늘리는 것으로 OS/센서 timestamp 차이를 해결하지 않는다.
장시간 운용에는 NUC가 접근 가능한 지속적인 chrony/NTP 구성이 별도로 필요하다.

### 2. FAST-LIVO2의 IMU 원점과 base_link 원점

수정 전 `LIVMapper::set_posestamp()`는 IMU 위치를 그대로 쓰고 회전만
−30° 보정했다. 내부 LiDAR 변환은 다음 식을 사용한다.

```text
p_odom = R_odom_imu * (R_imu_lidar * p_lidar + t_imu_lidar) + t_odom_imu
```

따라서 단순 frame 이름 변경이나 회전 보정만으로는 base pose가 되지 않는다.
형제 패키지 `FAST-LIVO2_ROS2`에서 전체 rigid transform을 적용했다.

```text
T_imu_base  = T_imu_lidar * inverse(T_base_lidar)
T_odom_base = T_odom_imu  * T_imu_base
```

MID360 launch는 실제 static TF와 동일한 `base_to_lidar_*` mount 값을
estimator 출력에도 전달한다. 현재 mount는 `(0, 0, 0.9 m)`, pitch `+30°`다.
**이 값은 설정 일치를 확인한 것이지 실측 재보정한 값은 아니다.**

또한 기존 internal calibration `T_imu_lidar`의 translation
`(+0.04165, +0.02326, -0.0284 m)`을 유지하면서 정적 `T_lidar_imu`는
그 역변환 `(-0.04165, -0.02326, +0.0284 m)`으로 일치시켰다.
MID360 launch의 `lidar_to_imu_*` override도 이제 이 static TF 방향이며,
내부 calibration은 그 역변환에서 계산한다. 예전의 양수 override를
그대로 복사하지 않는다. Legacy `base_*_correction_deg`는 full mount가
있는 MID360 launch에서는 superseded되며 시작 로그에 이를 알린다.

첫 IMU 초기화가 끝난 시점, world map을 만들기 전에 내부 상태의 원점을
초기 base 위치로 정한다. 따라서 TF와 모든 world cloud/marker가 함께
옮겨지고, base가 `odom.z ≈ 0`에서 시작한다. 이 처리가 없으면 원점
변환 자체는 맞더라도 base가 약 −0.9 m에 놓여 Nav2 voxel 높이 필터와
raytracing이 깨진다. 실제 실행에서 이 경고를 확인하고 수정했다.

### 3. FAST-LIVO2 timestamp

`/aft_mapped_to_init`, `odom -> base_link`, pose/path 출력은 처리 완료의
`now()` 대신 `LidarMeasures.last_lio_update_time`을 사용한다.
이는 해당 state의 LiDAR update 시각이며, 비동기 수신 버퍼의 최신 header
시각과도 구분된다. 이 변경은 이동 중 시각 정합을 바로잡는 수정이지,
정지 상태의 절대 localization 정확도 개선을 입증한 것은 아니다.

`/LIVO2/imu_propagate`는 별도의 IMU-rate 출력이며 이 Nav2 구성에서는
사용하지 않는다. `/odom` adapter는 TF를 발행하지 않는다.

### 4. AMCL 및 초기화 안내

| 설정 | 이전 | 변경 |
| --- | ---: | ---: |
| max_beams | 60 | 120 |
| update_min_d | 0.25 m | 0.05 m |
| update_min_a | 0.2 rad | 0.05 rad |

작은 이동에도 보정을 수행하도록 조정했다. `alpha1..5`, `sigma_hit`,
particle 수, height band 등은 아직 보정 자료가 없어 유지했다.
`set_initial_pose: false`, `always_reset_initial_pose: true`도 유지한다.
launch 후 **사용자가 2D Pose Estimate를 지정해야 한다.**
RViz fixed frame이 map일 때 scan 표시가 초기 pose보다 먼저 나와야 한다는
잘못된 대기 조건을 README/launch 안내에서 제거했다.

## 정합 지표 해석

수정 전 정지 측정에서 scan endpoint 23,442개의 지도 벽까지 거리 중앙값은
0.05 m, 90백분위는 0.10 m였고, 약 95.2%가 0.15 m 이내였다.
검사한 TF의 중복·역행 timestamp는 없었고 buffered scan 변환은 통과했다.
초기 별도 TF 측정에서도 scan 80/80개가 map/odom 양쪽으로 변환 가능했다.

이 지표는 **각 scan endpoint와 가장 가까운 occupied map cell의 거리**다.
map 해상도, 가구/사람, 높이 projection, 반복 복도에 영향을 받는다.
지도 밖 endpoint는 15 cm 정합 비율의 분모에 남기며, 중앙값/백분위는
지도 안 endpoint만 사용한다. 미지 영역에 떨어진 점 수도 별도 표시한다.
정확한 corridor 선택이나 ground-truth 위치 오차를 증명하지 않는다.
재시작 후 수동 초기 pose와 다른 실시간 scan을 사용하므로 전후 수치는
동일 데이터·동일 seed의 통제된 A/B 실험이 아니다.

## 수정 후 실기 결과

최종 FAST-LIVO2 재시작 후 사용자가 초기 pose를 다시 입력했다.
실행 중인 AMCL에서 `max_beams=120`, `update_min_d=0.05`,
`update_min_a=0.05`가 적용됐고 AMCL/controller lifecycle은 active였다.

안정 구간의 12초 audit (`steady_audit.json`) 결과:

- `map -> odom`: `/amcl` 단독, timestamp 역행 0회.
- `odom -> base_link`: `/laserMapping` 단독, timestamp 역행 0회.
- 검사한 scan은 map/odom 모두 40/40개 변환 가능.
- FAST-LIVO2 TF의 수신 시각 − 측정 시각은 약 38–58 ms.
  기존 처리 시각 stamp의 1 ms 수준 age를 더 정확한 저지연 출력으로
  오해하면 안 된다. 변경 후 수치는 실제 처리·전달 지연을 포함한다.
- `odom -> base_link`와 `map -> base_link`의 z는 약 0 m로 정상화.
- 새 odometry 구간에서 local costmap sensor-origin 높이 경고가 사라짐.
- 관측된 FAST-LIVO2 TF 주기는 대략 12–15 Hz였다(수집기 discovery 구간
  포함). 이 구성을 50–100 Hz odometry로 간주하지 않는다.
- scan-map 거리 중앙값 0.10 m, 90백분위 0.15 m,
  약 96.3%가 0.15 m 이내. 수정 전보다 중앙값은 커졌으므로 이 결과로
  **정확도가 향상됐다고 주장하지 않는다.**

앞선 `final_audit.json`에서는 AMCL TF timestamp 역행 1회를 검출해
audit가 exit 1을 반환했다. RViz/AMCL/local costmap에도 간헐적인
message-filter drop 로그가 있었다. 이후 안정 구간에서는 재현되지
않았지만, 원인과 장시간 재현성은 아직 확정하지 못했다. 이것을 무시하도록
진단 기준을 완화하지 않았다. 이동 중 검증 시 timestamp/queue 상태를
계속 기록해야 한다.

마지막으로 `/request_nomotion_update`를 **한 번만** 호출했다. 이후
`refined_audit.json`은 exit 0, scan map/odom 변환 40/40개, 거리 중앙값
0.05 m / 90백분위 0.10 m / 15 cm 이내 약 97.5%였다. 사용자 정지 입력
주변에서 정합이 유지됨을 확인했지만, 이것도 독립적인 위치 오차 측정은
아니다. 자동 반복 업데이트를 기본 launch에 추가하지 않았다.

최종 Python 테스트 49개가 통과했고, 새 진단 코드의 ament flake8 및
uncrustify 검사도 통과했다. FAST-LIVO2 `base_pose_test`는 CTest에서
통과했다. 기존 FAST-LIVO2 소스의 deprecated-header/기타 compiler 경고는
남아 있으며 빌드 실패는 없었다.

카메라의 RealSense static tree는 별도로 존재한다. 이번 실행은
`image_enable:=false`이며 카메라 extrinsic 연결·visual 모드 정확도는
검증 범위에 포함하지 않았다.

## 재현과 다음 검증

README의 순서대로 두 패키지를 빌드/실행한 뒤 초기 pose를 지정하고:

```bash
ros2 run jackal_nav2_bringup tf_localization_audit.py \
  --duration 12 --output /tmp/jackal_tf_audit.json
```

실기 launch/JSON 원본은 이번 세션의 `/tmp/nav2_refine_baseline` 및
`/tmp/nav2_refine_after`에 있다. `/tmp`는 영구 보관 위치가 아니다.

회귀 검증에는 Python TF contract/정합 지표 테스트와 FAST-LIVO2의
회전·lever-arm 변환 101개 및 초기 base 원점 100개 수치 케이스가 있다.

남은 정확도 평가는 별도 안전 구역에서 실측 landmark를 기준으로 해야 한다.
정지 반복 초기화, 직선 왕복, 회전, 반복 복도 재진입을 동일 경로로 기록하고
위치/방향 오차와 재초기화 실패율을 비교한다. 작은 covariance 또는 좋은
벽 겹침만으로 주행 준비 완료를 판단하지 않는다.
