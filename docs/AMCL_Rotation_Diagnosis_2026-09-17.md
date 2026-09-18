**진단 결론**

이 문서는 운동모델 변경 전 기록의 진단이다. 이후 적용과 오프라인 검증은 [회전 패치 결과](AMCL_Rotation_Patch_2026-09-17.md)에 정리했다.

현재 가장 유력한 원인은 `DifferentialMotionModel`과 FAST-LIVO2가 출력하는 회전 중 작은 평면 이동, 그리고 `alpha1=alpha4=0.2`의 조합이다. TF 연결 관계나 FAST-LIVO2의 yaw 부호 오류보다 이 조합이 관측된 AMCL 분포 확산과 잘 맞는다. 이는 저장된 odometry에 설치된 AMCL 구현의 수식을 적용한 추론이다. 내부 particle cloud, 참 위치, 정확한 초기 입자 상태가 없어 단독 원인으로 확정하거나 개선 효과를 검증한 것은 아니다.

사용자 요청에 따라 다운 전 기록만 분석했다. ROS launch, 로봇 구동, 파라미터 변경, CPU/RAM 보호 조치는 수행하지 않았다. 작성한 파일은 이 보고서와 오프라인 분석 자료뿐이다.

**분석 범위와 한계**

- 기록: 2026-09-17 11:40:22–11:41:36 KST, 약 74초. 제자리 양방향 회전과 정지 포함.
- FAST odometry 963개, scan 861개, raw cloud 648개, AMCL pose 49개 중 기록 이전의 latched pose 1개 제외.
- 소프트웨어: `nav2_amcl 1.1.20`, `pointcloud_to_laserscan 2.0.1`.
- 실행에 사용한 임시 install의 `nav2_params.yaml`과 소스 YAML은 동일했다. 다운 전 parameter service 조회는 실패했으므로 live parameter dump를 확보했다고 주장하지 않는다.
- 기록 시작 때 수신한 과거 AMCL pose는 이미 `var_x=4.71 m²`, `var_yaw=1.22 rad²`였다. 이번 기록은 정상 수렴 상태에서 최초 이탈까지 포착한 실험이 아니다. 회전 중 추가 악화와 잘못된 위치로의 이동은 기록됐다.
- bag의 수신 지연은 기록기 관점이다. 모든 노드가 같은 지연을 겪었다고 볼 수 없다.

**1. AMCL 운동모델과 파라미터: 가장 강한 원인 후보**

현재 설정은 `robot_model_type: nav2_amcl::DifferentialMotionModel`, `alpha1..5: 0.2`, `update_min_a: 0.05`, `update_min_d: 0.05`이다.

설치 버전의 DifferentialMotionModel은 평면 이동이 **1cm 미만**이면 제자리 회전으로 취급해 첫 회전량을 0으로 둔다. 1cm 이상이면 이동 방향으로 `rot1`을 계산하고, 나머지를 `rot2`로 나눈다. 옆 방향으로 조금 움직이면서 회전하면 실제 yaw 변화가 작아도 두 회전량이 각각 크게 나올 수 있다. `alpha1`은 이 값들로 yaw 잡음을, `alpha4`는 이동 잡음을 키운다. `alpha5`는 이 모델의 업데이트 계산에 사용되지 않는다. [해당 버전 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_amcl/src/motion_model/differential_motion_model.cpp)

기록된 AMCL pose 시각을 업데이트 경계로 보고, FAST pose를 그 시각에 보간해 47개 구간을 계산했다. 실제 내부 업데이트 경계와 같다는 가정이 들어간 재구성이다.

| 항목 | 재구성 결과 |
|---|---:|
| 업데이트 사이 평면 이동 중앙값 | 1.03cm |
| 평면 이동 ≥1cm인 구간 | 25 / 47 |
| 현재 모델의 이동 잡음 표준편차 | 중앙값 0.403m, 최대 0.969m |
| 현재 모델의 yaw 잡음 표준편차 | 중앙값 23.1°, 최대 55.5° |

예를 들어 `t=30.533s`의 구간에서는 이동이 1.37cm, 실제 yaw 변화가 4.43°인데, 이동 방향으로 계산한 첫 회전량의 절댓값이 약 92.3°다. 현재 설정으로는 이 한 업데이트에 이동 잡음 약 0.97m, yaw 잡음 약 55.5°가 계산된다. **이는 로봇의 실측 오차가 아니라 모델이 입자에 주입하는 잡음의 계산값이다.**

관측된 AMCL은 yaw 보정값이 한 구간에서 112.3° 바뀌었고, 위치가 한 구간에서 4.83m 이동했다. 큰 잡음으로 분포가 퍼진 뒤 지도에서 다른 위치 가설을 선택하는 패턴과 일치한다는 추론이다.

수식 비교만 하면 모든 alpha를 0.01로 낮춰도 최대 이동 잡음은 0.217m, yaw 잡음은 12.4°다. 단순히 alpha를 조금 내리는 것만으로 충분하다고 말하기 어렵다. 동일 입력과 alpha=0.2를 OmniMotionModel 수식에 넣으면 최대 0.072m·4.14°다. 이는 운동모델 선택을 비교할 근거이며, Omni 설정의 주행 성능이나 안전성을 검증한 결과는 아니다. [Omni 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_amcl/src/motion_model/omni_motion_model.cpp)

[잡음 비교 그래프](validation/2026-09-17/rotation_diagnosis/motion_noise.png) · [계산 결과](validation/2026-09-17/rotation_diagnosis/motion_model_summary.json)

**2. FAST-LIVO2: yaw 발산 증거는 약하지만 회전 중심 문제는 남아 있다**

회전 중 FAST yaw rate와 Livox IMU 및 플랫폼 IMU의 상관계수는 각각 약 0.989, 0.988이다. 정지 구간 바이어스를 제거한 회전 속도 차이의 중앙값은 약 0.004–0.005rad/s다. Livox IMU는 기록된 30° 장착 회전을 적용해 base 방향으로 비교했다. Livox IMU는 FAST의 입력이므로 완전히 독립된 검증은 아니며, 별도 플랫폼 IMU와도 일치한다는 점을 함께 사용했다.

FAST의 roll/pitch는 대략 ±1.7° 범위, z 변화 폭은 약 1.7cm였다. 이번 회전 기록에서 과거 LAN 단절 때와 같은 대규모 LIO 위치 발산은 보이지 않는다. `imu_gyr_cov`, `imu_acc_cov` 등을 우선 변경해야 한다는 근거는 부족하다.

반면 제자리 회전 중 FAST `base_link`의 x/y 변화 폭은 각각 18.2/24.3cm다. 회전 중심이 고정됐다고 가정해 원호를 맞추면 등가 반경은 약 **13.8cm**, 잔차 중앙값은 1.56cm다. 현재 `base_link → livox_frame`의 수평 이동 설정은 `(0, 0)`이다.

이 결과는 실제 센서 장착 위치와 설정 차이, base_link와 실제 회전 중심의 차이, 스키드 이동, LIO 위치 오차 중 하나 이상을 의심하게 한다. **13.8cm를 그대로 외부 파라미터 보정값으로 사용하면 안 된다.** 단일 제자리 회전 기록으로 이를 분리할 수 없다. 이 작은 위치 변화는 1번의 AMCL 잡음 증폭 조건을 만들 수 있다.

[회전 비교 그래프](validation/2026-09-17/rotation_diagnosis/rotation_diagnostic.png) · [수치](validation/2026-09-17/rotation_diagnosis/summary.json)

**3. TF 구성: 값의 불일치보다 전달 지연을 분리해서 봐야 한다**

구현과 기록에서 확인한 주요 연결은 다음과 같다.

```text
MID360 /livox/lidar → relay /livox/lidar_local
  ├─ FAST-LIVO2 + /livox/imu → /aft_mapped_to_init + odom→base_link TF
  │                            └─ adapter → /odom (twist 추가)
  └─ pointcloud_to_laserscan → /scan → AMCL + /map → map→odom TF

base_link → livox_frame → livox_imu_frame
```

AMCL 운동 업데이트는 scan 시각의 **odom→base_link TF**를 사용한다. adapter가 추가하는 `/odom.twist`나 그 필터 계수를 바꿔도 이 운동모델 문제를 직접 해결하지 않는다. [AMCL 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_amcl/src/amcl_node.cpp)

- 기록된 동일 시각 FAST pose와 odom→base_link TF 517쌍은 위치·quaternion 성분 차이가 0이었다.
- 관측된 주요 dynamic TF에서 timestamp 역행과 잘못된 quaternion은 없었다. 동일 child가 다른 parent로 연결되는 징후도 이 기록에는 없었다. 동일한 값을 내는 중복 publisher까지 배제한 것은 아니다.
- static TF는 `base→LiDAR: (0,0,0.9m), pitch +30°`, `LiDAR→IMU: (-0.04165,-0.02326,0.0284m), 회전 0`이었다. FAST의 IMU→base 계산과 방향이 일치한다.
- 해당 TF로 raw cloud를 다시 투영한 35,104개 대응 range의 오차 중앙값은 약 2×10⁻⁸m였다. 투영에서 장착 변환을 누락했거나 이중 적용했다는 증거는 없다. 물리적 장착 치수가 정확하다는 보증은 아니다.

별도로 bag의 `/tf` 수신 경로에는 회전 도중부터 약 7–8초 지연과 누락 구간이 기록됐다. 하지만 같은 시각 AMCL pose의 수신 지연은 중앙값 약 0.16초였다. AMCL이 scan 시각의 TF로 pose를 계산했다는 구현과 함께 보면, **bag의 TF 지연을 그대로 AMCL 내부 TF 지연이라고 해석할 수 없다.** DDS/기록기/구독자별 정체를 구분해야 한다.

`map→odom` timestamp가 scan보다 1초 미래인 것은 `transform_tolerance=1.0`의 의도된 동작이다. 이 값을 실제 측정 지연 보정이나 deskew로 해석해서는 안 된다.

[전달 지연 그래프](validation/2026-09-17/rotation_diagnosis/timing.png) · [TF 대조](validation/2026-09-17/rotation_diagnosis/geometry_summary.json)

**4. scan/지도 정합: deskew 부재는 실제 문제지만 이번 이탈 전체를 설명하지는 못한다**

현재 scan은 FAST의 deskew 결과가 아니라 raw cloud에서 만들어진다. 변환 노드는 cloud 전체를 한 번 좌표 변환하고 각 angle bin의 최소 range를 취하며, 점별 timestamp를 사용하지 않고 `time_increment=0`을 출력한다. [pointcloud_to_laserscan 구현](https://raw.githubusercontent.com/ros-perception/pointcloud_to_laserscan/humble/src/pointcloud_to_laserscan_node.cpp)

raw cloud의 측정 시간 폭은 약 66.8ms였다. 샘플링한 회전 cloud 내부의 회전량은 최대 약 1.60°로, 시간 왜곡의 기하학적 크기는 10m에서 약 0.28m, 2m에서 약 0.056m 수준이다. `sigma_hit=0.15m`와 비교해 먼 벽의 정합에는 영향을 줄 수 있다. 그러나 4.83m의 위치 가설 이동을 이것 하나로 단정하기는 어렵다. `scan_time=0.10`을 실제 주기에 맞추는 것만으로 점별 시간 보정이 생기지는 않는다.

AMCL 최소 거리 0.4m를 적용한 유효 range 중 1m 이내 비율은 중앙값 약 56.6%다. 그림을 확인하면 상당 부분이 가까운 복도 벽이다. 높이 하한을 0.1→0.5m로 바꾼 오프라인 투영에서도 이 비율은 거의 줄지 않았다. 이를 전부 바닥·차체 노이즈로 간주하거나 range_min을 크게 올리는 처방은 근거가 부족하다.

지도에 scan을 AMCL pose로 겹쳐 보면 `t=27.33s`에는 사용 빔의 약 74.5%가 지도 장애물에서 0.15m 이내였다. 회전 중 정합이 악화된 뒤 `t=40.13s`에는 다른 위치로 4.83m 이동했는데도 약 71.4%가 그 기준을 만족한다. 반복되는 구조와 지도 일부의 조밀한 장애물 영역이 잘못된 가설에도 점수를 줄 수 있다는 추론이다. 이 비율은 단순 endpoint 거리 분석이며 AMCL의 전체 likelihood를 재현한 값은 아니다.

FAST의 `/cloud_registered`는 보정된 cloud 후보지만 현재 구현은 `header.stamp=now_stamp()`를 사용한다. 해당 topic으로 scan 입력만 바꾸는 방식은 측정 시각·좌표계 문제를 새로 만들 수 있다. 이를 사용하려면 cloud의 기준 시각과 odom/base 변환을 함께 설계해야 한다.

[scan 형상](validation/2026-09-17/rotation_diagnosis/scan_geometry.png) · [지도 겹침](validation/2026-09-17/rotation_diagnosis/map_alignment.png)

**5. 멈춘 뒤 제 위치를 못 찾는 이유**

현재 설정의 AMCL은 움직임 임계값을 넘을 때 업데이트한다. 기록에서도 마지막 pose 업데이트는 `t=43.67s`였고 이후 약 30초 동안 scan은 계속 기록됐다. 정지했다고 측정 업데이트를 반복하며 다시 수렴하는 구조는 아니다.

또한 YAML에 `recovery_alpha_slow/fast`가 없고 설치 버전 기본값은 둘 다 0이다. 실행 중 별도 변경이 없었다면 random pose injection 기반 회복이 비활성인 설정이다. `always_reset_initial_pose=true`는 고공분산 때 자동 재초기화하는 옵션이 아니다. 이 조건들은 이미 잘못된 가설에 들어간 뒤 회복이 어려운 상황과 맞는다. [설치 버전 AMCL 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_amcl/src/amcl_node.cpp)

현재 `likelihood_field`에서는 `z_short/z_max`를 사용하지 않으며 `do_beamskip`만 켜도 이 모델에서 beam skipping이 작동하지 않는다. 이 항목들을 먼저 조절하는 것은 핵심 원인에 대응하지 못한다. [센서 모델 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp)

**후속 수정·검증 우선순위 — 이번에는 적용하지 않음**

1. 센서와 base_link 및 실제 회전 중심의 수평 위치를 확인한다. 기존 회전 궤적의 13.8cm 등가 반경은 점검 단서로 사용한다.
2. 같은 기록에 대해 AMCL 운동모델과 alpha를 함께 비교한다. Differential 계수 조정과 LIO의 횡방향 이동을 다루는 모델을 후보로 두고, 정상 초기 pose에서 회전 전후 분포·위치 연속성을 비교해야 한다. 이번 수식 분석은 전체 AMCL 재생 검증을 대체하지 않는다.
3. 측정 시각이 일치하는 deskew scan 경로를 마련한다. 그 다음 `sigma_hit`, 센서 모델의 outlier 처리, 복구 설정을 검토한다.
4. TF/DDS 전달 정체는 별도 구성 문제로 추적한다. 11:41:48경의 controller heartbeat 실패와 FAST IMU/LiDAR sync 경고 폭증은 회전 이탈 뒤에 나타난다. 다운 이후 extrapolation 로그를 회전 이탈의 단독 원인으로 묶지 않는다. OOM의 직접 원인 및 자원 보호 정책은 이번 분석 범위에 넣지 않았다.

**자료와 재현**

분석 스크립트·JSON·그림·설정 스냅샷은 [rotation_diagnosis](validation/2026-09-17/rotation_diagnosis/)에 저장했다. 원본 bag의 위치·크기·SHA-256은 [manifest](validation/2026-09-17/rotation_diagnosis/manifest.json)에 있다. 약 238MB(227MiB) 원본 bag은 `/tmp/jackal_nav2_validation_20260917_KXP5m6/rotation/bag/bag_0.db3`에 남겨 두었으며 저장소에 복제하지 않았다.

스크립트는 ROS 메시지 역직렬화와 SQLite 읽기, NumPy/SciPy 계산만 수행한다. 노드를 기동하지 않는다. `analyze.py → inspect_geometry.py → timing_and_map.py → motion_model_analysis.py` 순서이며, 스크립트의 `P`는 원본 기록 디렉터리를 가리킨다.
