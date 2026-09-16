# 최신 소스 리뷰 및 Nav2 버퍼링 후속 구현 (2026-09-14)

## 결론

최신 현장 결과: [정지 실기 예비 검증 — 미통과](Stationary_Pilot_2026-09-14.md).
아래 리뷰/배포 문단의 미실시 표기는 각각 작성 당시의 상태다.

후속: [FAST-LIVO2 수정·workspace 배포 확인](FAST_LIVO_Followup_2026-09-14.md).
사용자가 TF/timestamp·누수/누적 패치를 적용하고 재빌드했다. 원본 18개 파일의 일치, 설치 라이브러리 로딩과 오프라인 회귀까지 확인했다. 실기 검증은 아직이다. **아래 내용은 패치 적용 전 최초 리뷰 시점의 기록이다.**

Nav2 범위의 개선은 구현했지만 **가져온 FAST-LIVO2를 현재 상태로 실기 재검증할 수는 없다**. 이전 실행에서 확인한 측정 시각/센서 lever-arm 처리와 현재 소스가 다르다. 메모리 증가도 해결됐다는 근거가 없다. 이 문서는 현재 파일을 읽은 결과이며, Git 이력 접근이 도구 승인 단계에서 실패하여 특정 커밋의 변경이라고 단정하지 않는다.

실기 launch·sudo·시각 변경·네트워크 변경은 하지 않았다. 외부 패키지 소스와 기존 install은 수정하지 않았다.

## 외부 패키지 리뷰: 후속 수정이 필요한 순서

### P1 — FAST-LIVO2 측정 timestamp 및 base pose 계약

- `FAST-LIVO2_ROS2/src/LIVMapper.cpp:1392`의 `publish_odometry()`는 odometry와 TF stamp에 `now_stamp()`를 넣는다. 이전 실측에 사용한 측정 시각 `LidarMeasures.last_lio_update_time`과 다른 동작이다. 처리 지연이 생겨도 age가 작게 보여 TF/scan 시각 비교를 왜곡할 수 있다.
- 같은 함수의 translation은 `_state.pos_end`를 그대로 사용한다. `set_posestamp()`도 position은 그대로 두고 quaternion에 고정 회전만 곱한다. 현재 readParameters에 base mount translation을 이용하는 변환이 없다.
- LiDAR world 변환은 `R * (extR * p_lidar + extT) + p`인데, 발표하는 base TF에는 IMU→base lever arm이 없다. 이 설치에서는 센서와 base_link가 떨어져 있으므로 orientation 보정만으로 일관된 `odom → base_link → livox_frame`을 만들 수 없다.
- 기존 `include/base_pose.h`와 `test/test_base_pose.cpp`는 남아 있지만, 현재 LIVMapper와 CMake에서 해당 helper/test를 사용하는 연결을 찾지 못했다. 파일이 존재하거나 helper 단독 테스트가 통과하는 것만으로 실제 publish 경로의 수정 완료를 판단하면 안 된다.

**필요 조치:** FAST-LIVO2 내부에서 측정 시각과 전체 rigid transform을 복원하고 실제 publish 함수에 연결한다. Nav2 adapter에서 stamp를 덮어쓰거나 알 수 없는 지연을 빼서 해결하지 않는다. Nav2 쪽에서 두 번째 TF writer도 추가하지 않았다.

### P1 — 큰 구독 큐와 미확정 메모리 증가

- 현재 FAST-LIVO2 cloud/IMU/image 구독에 `keep_last(200000)`이 남아 있다.
- `img_buffer`/LiDAR deque의 명시적 길이 제한을 찾지 못했다. image 비활성일 때 callback은 반환하지만 image 구독 생성 자체는 조건부가 아니다.
- 이전 실측 RSS는 252초 동안 4.26 → 8.38 GiB, 약 16.7 MiB/s 증가했다. 이는 현재 가져온 소스에 대한 새 측정이 아니며, 어떤 큐/visual map/메모리 할당이 원인인지는 미확정이다.

**필요 조치:** timestamp/base pose 수정 후 정지·주행 차단 상태에서 짧은 LIO/LIVO A/B와 실제 queue/map 크기 계측을 한다. bounded queue, 비활성 image subscription 제거, visual map 소유권/해제는 근거를 보고 수정한다. 전체 프로세스 CPU 100% 포화나 특정 메모리 누수라고 단정하지 않는다.

### P1 — perception 입력과 추적 frame override 유실

- 현재 `mid360_bringup/launch/perception.launch.py`는 `lidar_input_topic`·`tracking_frame`을 선언/사용하지 않는다. 기존 Nav2 회귀 테스트가 이 지점에서 실패했다(수정 전 89 pass / 1 fail).
- 기본 accumulator 입력은 `/livox/lidar`, extractor의 `tracking_frame`은 `base_link`, `use_latest_tf_fallback`은 true다. tracker는 detection header frame의 위치를 그대로 추적하므로 입력을 `odom`으로 공급해야 한다.
- `lidar_preprocess_config`와 `extractor_config` YAML 경로 override는 현재도 지원된다.

**이번 Nav2 대응:** `prepare_perception_config.py`로 설치본 YAML을 복사·검증하고 필요한 세 필드만 변경한다. 결과는 별도로 실행하는 perception 명령으로 전달한다. standalone 기본값과 detection/tracking 메시지는 유지한다. 재시작 때에는 perception track history도 리셋해야 한다.

### P1 — full_stack의 Nav2 부적합 기본값

- 현재 `use_fast_livo=false`이므로 `full_stack.launch.py launch_rviz:=false`만으로는 odometry가 시작되지 않는다.
- `use_fast_livo=true`로 바꾸면 `fast_livo_odom_fallback=true`가 기본이다. fallback은 real odometry가 오기 전 identity odometry/TF를 발행한다. 실기 Nav2에 적합한 결손 처리 방식이 아니다.
- FAST-LIVO2용 lidar argument를 perception 입력으로 전달하는 연결도 없다. 외부 launch를 선택한다고 relay 사용이 자동 통일되지 않는다.
- perception 기본 `publish_base_to_lidar_tf=true`는 NUC의 동일 static TF와 충돌할 수 있다.

**이번 절차:** Nav2/relay → 단일 FAST-LIVO2 → 별도 perception → 사용자 초기 pose. perception은 base→lidar와 lidar→imu를 발행하지 않고 camera bridge만 제공한다.

### 확인한 개선/유지 항목

- YOLO 기본 설정은 `inference_device: cuda:0`으로 되어 있다. 이전 실측의 CPU 설정과는 다르지만, 실제 GPU 사용 성공이나 지연 개선은 이번에 확인하지 않았다.
- perception camera TF는 calibration을 역변환하고 RealSense optical 하위 트리를 유지하는 bridge 방식이 남아 있다.
- Nav2 figure의 0.5 scale과 측정 시각 TF 기반 trace 코드는 남아 있다. 시각화와 costmap 입력은 계속 분리한다.

## 이번 Nav2 변경

1. `enable_motion=false`를 최상위 기본값으로 복원하고 활성/차단 상태를 launch에서 알린다. 플랫폼 forwarding 설정은 건드리지 않는다.
2. AMCL 자동 pose 재주입을 제거했다. `amcl_quality_monitor.py`는 covariance 진단만 발행한다. 기존 executable 이름도 진단 전용으로 전환하여 수동 실행 시 과거 pose가 재주입되지 않는다.
3. relay on/off에 따라 빈 `lidar_pointcloud_topic` 기본값을 자동 결정한다. 입력과 출력이 상대/절대 이름이나 ROS remapping으로 같은 토픽이 되면 loop를 거부한다. 모든 Nav2 소비자에 같은 결과를 전달한다.
4. relay는 원본 stamp·frame·points를 유지한다. `/nav2/lidar_relay_diagnostics`에 measurement age, receipt silence, 수신 간격, timestamp/clock 역행 횟수를 발행한다. relay 자체가 downstream queue를 제한하거나 stale cloud를 조용히 폐기한다고 주장하지 않는다.
5. perception YAML 생성기는 원본 hash/변경 필드를 manifest에 보관하고 기존 출력 덮어쓰기를 거부한다. Nav2가 perception을 자동 launch하는 옵션은 추가하지 않았다.
6. `runtime_resource_audit.py`는 ROS 구독 없이 PID/start-time을 묶어 RSS/swap/UDP 오류 delta를 JSONL로 기록한다. PID 재사용/프로세스 종료 시 수집을 중단한다. 자동 kill/sudo/네트워크 수정은 없다.
7. 기존 recorder는 실행 중인 relay/quality monitor가 있으면 파라미터도 저장하고, full-perception 모드에서 YOLO 파라미터를 추가 저장한다.

AMCL noise/laser model/tolerance, scan 높이, costmap plugin, 안전 polygon, 속도 제한은 이번에 바꾸지 않았다. 증거 없는 파라미터 튜닝으로 문제를 가리지 않는다.

## 검증 및 재개 조건

- Python 회귀 **148개 통과**(Nav2 107 + figure/trace 41), C++ relay health **3개 통과**. 수정한 Python 8개 파일 flake8와 C++ 3개 파일 cpplint/uncrustify 검사도 통과했다.
- 전체 Nav2 C++ 빌드 및 격리 install 성공. 설치 경로의 새 CLI `--help`와 bringup `--show-args`에서 주행 기본값 false를 확인했다. 이는 ROS 노드 실행 검증이 아니다.
- CTest 기능 테스트 묶음 8/8 통과. 테스트가 단독 실행될 때에도 사용자 ROS 로그 디렉터리에 쓰지 않도록 테스트 로그 경로를 지정했다. YAML 생성 CLI와 자체 PID 대상 0.2초 리소스 기록 smoke test도 성공했다.
- Nav2 회귀 테스트: 주행 기본값, relay on/off/custom/loop, 진단 전용 publisher, 반복 high covariance 입력, perception YAML의 실제 upstream 소비 경로, 원본 보존, 리소스 PID/오류 delta를 검증한다.
- C++ relay health: 수신이 계속되는 stale 입력, ROS time 정지 중 실제 수신 단절, future/역행/invalid header, 10만 회 관측의 고정 크기 상태를 검사한다.
- 빌드는 `/tmp`의 격리 prefix를 사용한다. workspace install에 배포하거나 실기 노드를 실행한 것이 아니다.
- 적용 전 workspace에서 `colcon build --symlink-install --packages-select jackal_nav2_bringup` 후 `source install/setup.bash`가 필요하다. 새 executable/C++ relay는 기존 install에 아직 배포되지 않았다. 외부 패키지는 P1 수정 후 별도로 재빌드·검증한다.
- 새 외부 소스를 포함한 end-to-end 실행, GPU/메모리 개선, scan map exact-time 99% 목표, 주행 안전은 **미검증**이다.
- 외부 FAST-LIVO2 P1 수정이 선행되어야 한다. 이후 60초 RSS/TF 비교 → 정상이라면 10분 누적 계측 → 별도 사용자 입회 주행 순서로 진행한다.
- socket buffer/sysctl 변경이 필요하면 실제 적용 크기·UDP 증가량을 근거로 사용자에게 sudo를 요청한다. 이전 큰 누적 오류값만으로 변경하지 않는다.

## 리뷰 대상 fingerprint (SHA-256)

```text
FAST-LIVO2_ROS2/src/LIVMapper.cpp
2a98f250b95befabb3acf48016d82786a06aaa55a5de34622e39ff7fc1101aca
mid360_bringup/launch/perception.launch.py
148622f88b58a888a07d8f1831cb0b4173f00e92b58bbabf9836435918668c2f
mid360_bringup/launch/full_stack.launch.py
c1adc30aa387db20c8a77f9a8bf42881940d445615aa96a2c138f8812671a603
mid360_perception/src/reprojection_fusion_node.cpp
e7da93ec7b2e4b2c19d14cc2d8ecc624b2f381b9d72d9ffd0101b8f43bbd7b87
```

직전 실행의 원자료는 [버퍼링 진단](validation/2026-09-14/buffering/README.md)에 보존한다. 현재 가져온 소스에 대한 검증 결과와 혼동하지 않는다.
