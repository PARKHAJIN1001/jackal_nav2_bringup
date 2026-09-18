# FAST-LIVO2 오프라인 후속 수정 및 workspace 배포 확인 (2026-09-14)

## 상태

최신 현장 결과: [2026-09-16 10분 정지 검증](Connected_Validation_2026-09-16.md).

후속 실기 예비 측정은 [정지 검증 기록](Stationary_Pilot_2026-09-14.md)을 참고한다.
60초 full-perception에서 TF 기준 미달과 시각화 실행 등록 누락을 확인했고 10분 검증은 미실시다.
아래는 그보다 앞선 오프라인/배포 확인 시점의 기록이다.

**사용자가 패치 적용과 두 패키지의 workspace 재빌드를 완료했고, 이후 소스·설치본·오프라인 회귀를 직접 확인했다. 실기 검증은 아직 수행하지 않았다.** 최초 원본 쓰기 요청은 자동 승인 도구의 모델 용량 오류로 거절됐으며, 이를 우회하지 않고 사용자가 제공된 명령으로 적용했다. 이번 확인에서도 로봇 launch·주행·sudo·시각 변경은 하지 않았다.

- Nav2 새 executable의 설치 경로와 relay의 build/install 해시 일치를 확인했다.
- FAST-LIVO2 수정은 `/tmp/fast_livo_followup.dYwWPy/staged`, 수정 전 스냅샷은 같은 경로의 `before`에 있다.
- 영구 보관한 [적용용 패치](patches/fast_livo_nav2_followup_2026-09-14.patch)는 FAST-LIVO2에만 적용하는 18개 파일 변경이다. Nav2 변경을 재적용하는 패치가 아니다.
- 원본에서 패치 dry-run 성공. 복사본에 실제 변경을 적용한 결과가 테스트한 staged 소스와 일치하고, 역방향 dry-run도 성공했다.
- 적용된 원본 18개 파일 모두 staged 소스와 바이트 단위로 일치하고 역방향 dry-run도 성공했다. 현재 `src/LIVMapper.cpp` SHA-256은 `b19076f19c2028b9832b8ae7f8f000adab6c0e638f2c462c077471a03fc835fc`다.

이 문서는 [최신 소스 리뷰](Upstream_Review_2026-09-14.md)의 후속 기록이다. 이전 실측 결과를 이번 수정본의 현장 검증 결과로 취급하지 않는다.

## 수정 내용

### TF와 측정 시각

- 실제 `publish_odometry()`가 `LidarMeasures.last_lio_update_time`을 사용한다. odometry와 TF가 같은 stamp와 pose를 사용하며, 처리 완료 시각으로 덮어쓰지 않는다.
- 실제 pose 경로에 `T_odom_base = T_odom_imu * T_imu_lidar * inverse(T_base_lidar)`를 연결했다. 회전뿐 아니라 IMU–base 사이 translation도 적용한다.
- IMU 초기화 직후, world map 생성 전에 한 번만 base 원점을 맞춘다. 기존 map 생성 후 좌표계를 이동하지 않는다.
- MID360 launch의 `lidar_to_imu_*`는 static `T_lidar_imu` 방향이다. 내부 extrinsic에는 역변환을 전달한다. 기본 static translation은 `(-0.04165, -0.02326, +0.0284)` m다.
- MID360 launch는 전체 base mount를 넘기며, 이 경우 legacy orientation 보정을 중복 곱하지 않는다. 전체 mount 없이 직접 실행하는 legacy 경로는 경고를 남긴다.
- NUC의 `base_link → livox_frame`, FAST의 `livox_frame → livox_imu_frame`, 별도 perception의 camera bridge 소유권을 유지한다. mount 값 자체를 새로 실측한 것은 아니다.

### 확인된 누수·누적 경로

1. `updateVisualMapPoints()`의 no-add 분기에서 패치 배열을 할당하고 해제하지 않는 경로를 수정했다. 추가가 결정됐을 때만 할당하고 Feature에 소유권을 넘긴다.
2. RGB 결과가 0점이면 `pcl_wait_pub`가 계속 남던 경로를 수정했다. 설정한 발행 window를 소비했으면 RGB 출력 유무와 무관하게 비운다.
3. ONLY_LIO에서도 `measures`에 과거 구간과 IMU 포인터가 계속 쌓이던 경로를 수정했다. 소비자가 사용하는 현재 구간 하나만 보관한다.
4. DDS depth 기본값을 LiDAR 4 / IMU 512 / image 2로 제한하고, image 비활성 시 구독 자체를 만들지 않는다. image callback의 불필요한 ROS message 전체 복사도 제거했다.
5. application buffer 기본 상한은 LiDAR 30 frame / IMU 2000 sample / image 8 frame이다. path는 최신 2000 pose만 보관한다. 2초마다 buffer·visual voxel 수를 기록한다.

세 경로는 코드에서 확인된 문제다. **이전 16.7 MiB/s RSS 증가가 전부 이 때문이라는 결론은 아직 내릴 수 없다.** visual map, 관측이 보유한 image, 선택적 PCD 저장 등의 전체 메모리 상한을 보장하는 변경이 아니다.

### 결손 처리

- application buffer가 상한에 도달하면 estimator를 오류 종료한다. IMU 적분 이력을 조용히 버리면서 pose를 계속 만들지 않는다.
- 작은 timestamp 역전·중복은 해당 입력을 거부하고 paired data/time queue를 유지한다. 0.5초 초과 역행은 재시작이 필요한 오류로 처리한다.
- nonfinite/0 이하 입력 stamp는 거부한다. odometry 출력 stamp는 유효하고 증가해야 한다.
- IMU stamp를 반올림해 맞추던 `ros_driver_bug_fix=true`는 거부한다. 명시적인 calibrated IMU/image offset은 유지한다.
- 이것은 정지 안전 인증이 아니다. 실패 시 독립 Guard와 플랫폼 watchdog이 실제로 차단하는지는 별도 정지 시험이 필요하다.

## 오프라인 검증

- 시스템 CMake 3.22.1, ROS Humble, Debug(`-g0`), 단일 build worker로 전체 C++ 빌드 및 격리 install 성공. 사용자 경로의 CMake 4.0에서는 MPI 탐지가 실패했으며, 시스템 설정/패키지는 변경하지 않았다.
- CTest 기능 묶음 **5/5 통과**: base transform/origin, buffer 보관, stamp 처리, 실제 VIO 함수의 patch 할당, launch/production wiring.
- base helper: 101 transform/lever-arm 및 100 초기 원점 사례. buffer helper: 빈 RGB 결과 10만 회, 측정 구간 교체 10만 회.
- launch/production wiring 검사 **6개 통과**: 실제 launch callback을 node 실행 없이 평가하고, 회전이 있는 IMU TF도 역변환 관계를 확인했다. 소스 wiring 검사는 실시간 TF 통신 검증을 대체하지 않는다.
- VIO 대조 시험: 기존 함수 본문을 executable에 연결한 경우 no-add 1000회에 패치 크기 배열 **1000개 추가 할당**, 수정본은 **0개**. 수정본의 add 경로는 배열 1개와 Feature 1개를 정상 추가했다. 라이브러리의 다른 크기 배열/백그라운드 할당은 이 계수에 포함하지 않는다.
- 테스트 시 기존 overlay `libvio.so`가 먼저 로드될 수 있어, CTest가 just-built library를 우선하도록 지정했다. 단순히 테스트 executable만 새로 만들었다고 수정본 검증으로 판단하지 않는다.
- Nav2 Python 회귀 **107개 통과**, 새 Python contract test flake8 통과. FAST 전체 기존 lint가 모두 통과했다는 의미는 아니다.
- 격리 install의 `--show-args` 성공. ROS 노드는 실행하지 않았다.

로그는 [validation/2026-09-14/fast_livo_followup](validation/2026-09-14/fast_livo_followup/README.md)에 보관한다.

### 사용자 적용 이후 workspace 배포 검증

- workspace CMake cache: 시스템 `/usr/bin/cmake`, Debug, `-g0`, `BUILD_TESTING=ON` 확인.
- `ros2 pkg prefix`가 `~/moai_navigation_ws/install/{fast_livo,jackal_nav2_bringup}`를 가리킨다.
- `fastlivo_mapping`의 `ldd -r`에서 workspace 설치본의 mapper/IMU/preprocess/VIO/LIO 라이브러리를 로드하며, 미해결 symbol이나 누락 library는 없었다. 실제 ROS node를 실행한 검사는 아니다.
- `libvio.so` build/install SHA-256은 둘 다 `75d652bedb1a7ebbbe1ab505c7d069daaf96a13c9c9a3728b6aa24dbf8dd7097`, relay executable은 둘 다 `8c79fc3fbbb33545c444a3a8a8cb17345fc9e0132827a4215375b98ba0874d38`이다.
- workspace build의 FAST C++ 테스트 4개를 직접 실행해 통과했다. VIO 테스트는 **설치된 libvio.so**를 로드하며 no-add 1000회 패치 할당 0개, add 경로 1개를 확인했다.
- 실제 원본의 launch/source 계약 6개, Nav2 Python 107개, relay C++ 3개를 재검사해 통과했다. workspace build 디렉터리에 CTest 결과를 덮어쓰지 않고 각 테스트를 직접 실행했다.
- 두 패키지 설치본 `--show-args`, 설치된 perception profile helper의 `--help` 성공. `enable_motion` 기본값 false 확인.
- 새 로그는 `validation/2026-09-14/fast_livo_followup/workspace_deployment/`에 별도로 보관한다. 기존 격리 로그와 구분한다.

## 사용자가 완료한 적용·재빌드 명령 (이력)

**이미 적용했으므로 아래 patch 명령을 다시 실행하지 않는다.** 이 절차를 다른 수정 전 복사본에서 사용할 때에는 관련 프로세스를 정지하고 먼저 dry-run을 확인한다. 실패하면 강제로 적용하거나 파일 전체를 덮어쓰지 말고 변경점을 다시 검토한다. sudo는 필요하지 않다.

Git metadata는 최초 확인에서 `.git`이 가리키는 `../../.git/modules/src/FAST-LIVO2_ROS2`가 없어 사용할 수 없었다. 이번에는 metadata 복구를 수행하거나 재확인하지 않았다. 이 작업에서 `.git`을 삭제하거나 새 저장소로 초기화하지 않았다.

```bash
cd ~/moai_navigation_ws/src/FAST-LIVO2_ROS2
patch --dry-run --forward --fuzz=0 -p1 < ../jackal_nav2_bringup/docs/patches/fast_livo_nav2_followup_2026-09-14.patch
# 위 검사가 성공한 경우에만 실행
patch --forward --fuzz=0 -p1 < ../jackal_nav2_bringup/docs/patches/fast_livo_nav2_followup_2026-09-14.patch
```

사용자가 실행 완료했다고 보고한 workspace 배포 명령이다. 이후 배포 결과와 테스트를 확인했으며, 빌드 명령 자체는 에이전트가 재실행하지 않았다. 단일 worker와 시스템 CMake를 사용한다.

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PATH=/usr/bin:$PATH MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --symlink-install --executor sequential \
  --packages-select fast_livo jackal_nav2_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG=-g0 -DBUILD_TESTING=ON
source install/setup.bash
```

실제 로드 라이브러리와 새 executable 설치 확인까지 완료했다. 격리 install의 실행파일이 아닌 workspace 배포본으로 검사했다.

## 다음 검증과 남은 문제

1. 원본 적용/재빌드 확인은 완료했다. 다음은 시각 동기 확인 → 주행 차단(`enable_motion=false`) 상태로 Nav2/relay → 단일 FAST(LIO-only, `image_enable=false`) → 별도 perception 순서의 정지 검증이다. FULL_STACK의 identity odometry fallback은 사용하지 않는다.
2. FAST 재시작 때 perception/trace history도 재시작하고 AMCL 초기 pose를 수동 재입력한다. 이전 odom 원점의 pose를 재주입하지 않는다.
3. 우선 60초 RSS/swap·buffer 수·센서 age·TF exact-time을 기록한다. 정상일 때만 10분 누적 계측을 한다. TF 중복/역행 0회, scan exact-time 성공률 99% 이상은 **아직 미검증 목표**다.
4. LIVO 비교 전 추가 리뷰가 필요하다. 이번 빌드에서 기존 `updateReferencePatch()`의 `ref_mean`/`other_mean`이 cached mean 분기에서 초기화되지 않는 코드를 확인했다. 이번 patch에는 NCC/reference-selection 알고리즘 변경을 섞지 않았으며, **image-enabled 안정화 완료로 표시하지 않는다.** 먼저 해당 경로의 수치·반복 호출 회귀 검증을 보완한다.
5. `/cloud_registered`/debug image stamp와 별도 IMU propagation pose는 이번 Nav2 base-pose 계약의 범위 밖이다. 해당 출력을 사용할 때에는 별도 검증한다.
6. 최신 perception의 실제 GPU 사용, TF 지연, figure/trace, 동적 costmap 미반영과 독립 정지 경로, 센서 단절 시험은 배포 후 확인한다. 이번 오프라인 결과로 통과 처리하지 않는다.
7. 주행은 비상정지·수동 조작·플랫폼 forwarding/watchdog을 확인한 사용자 입회 단계로 남긴다. 절대 localization 정확도와 mount 재실측도 후속 현장 과제다.
