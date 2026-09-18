**적용 결과**

AMCL 기본 운동모델을 `nav2_amcl::DifferentialMotionModel`에서 `nav2_amcl::OmniMotionModel`로 변경했다. 제자리 회전 중 LIO odometry가 보고하는 작은 횡방향 이동이 Differential 모델의 1cm 분기에서 큰 잡음으로 증폭되는 현상에 대응한다. alpha1–5는 0.2로 유지해 모델 변경 효과를 분리했다. Omni에서는 alpha5도 횡방향 잡음 계산에 사용된다. 이 값들은 아직 현장 보정이 끝난 계수가 아니다.

이 설정은 odometry의 확률적 오차 모델이다. Jackal의 controller 횡속도 제한은 여전히 `min_vel_y=max_vel_y=0`이다. 장착 위치, FAST-LIVO2 IMU 계수, scan 투영, 자동 회복 설정은 이번 패치에서 변경하지 않았다.

**구현 변경**

- [nav2_params.yaml](../config/nav2_params.yaml): 운동모델 선택과 설명을 수정했다. 검증되지 않은 센서 계수를 calibrated라고 표현하던 주석도 정정했다.
- [test_amcl_motion_model.cpp](../test/test_amcl_motion_model.cpp): 설정 YAML의 플러그인을 실제로 로드해 입자 업데이트를 검사한다. ROS node/DDS participant는 만들지 않는다.
- CMake/package.xml: 위 테스트의 Nav2 AMCL, pluginlib, YAML 의존성과 테스트 대상을 등록했다.

**검증 결과**

회귀 테스트 4개가 통과했다. 1cm 경계 전후의 잡음 연속성, 정지 시 입자 보존, 무잡음 전·후진/횡이동 및 ±π 방향 경계, 14cm 회전 중심 오프셋 궤적을 검사했다. 기존 설정·launch 테스트는 17개가 통과했다. 전체 패키지 빌드와 임시 install이 성공했고, 새 C++ 파일의 cpplint/uncrustify 및 CMake lint가 통과했다.

추가로 저장된 지도·scan·FAST pose를 설치된 **AMCL C++ 라이브러리**에 입력했다. 단순 수식 비교에 이어 motion model, likelihood-field sensor model, particle filter resampling 및 최고 가중치 cluster 선택까지 실행했다.

- 설치 버전: nav2_amcl 1.1.20.
- 입력: 기존 회전 기록의 scan 494개와 보간한 FAST pose. 각 실행에서 필터 업데이트 49회.
- 비교: 모델별 난수 seed 1–10, 총 20회. 두 모델의 alpha와 센서 계수는 동일.
- 공통 초기 평균: 기록 내 첫 AMCL pose `(-3.797, -0.196, -2.998rad)`.
- 공통 초기 공분산: `(0.25m², 0.25m², 0.06854rad²)`로 가정. 기록 시작 때 이미 커져 있던 공분산을 그대로 사용한 재생은 아니다.
- `/tf` 수신 지연, lifecycle, executor, DDS는 재생하지 않았다. ROS 노드를 재기동하지 않았다.

아래 값은 **각 모델의 10회 실행 전체에서 가장 큰 값**이다.

| 항목 | Differential | Omni |
|---|---:|---:|
| 연속 pose 사이 최대 위치 점프 | 7.039m | 0.060m |
| LIO 대비 yaw 보정값의 최대 단일 변화 | 125.35° | 1.41° |
| 초기 pose에서 LIO를 적분한 기준 대비 최대 위치 차이 | 7.180m | 0.442m |
| 전체 입자 분포의 최대 x/y 분산 | 12.135m² | 0.370m² |
| 전체 입자 분포의 최대 yaw 분산 | 6.162rad² | 0.06595rad² |

동일 입력에서 운동모델만 바꿨을 때 큰 pose 점프와 분포 확산이 줄었다. 따라서 앞선 진단의 운동모델/잡음 증폭 가설을 더 강하게 뒷받침한다. 다만 초기 위치는 참 위치로 검증되지 않았고 비교 기준도 LIO에서 얻었다. **이 수치는 실제 위치 오차나 현장 성공률이 아니다.** Omni에도 기준 대비 최대 0.44m 차이가 남아 있으므로 현장 정확도 검증이 필요하다.

[10회씩의 비교 그래프](validation/2026-09-17/rotation_patch/replay_comparison.png) · [전체 수치](validation/2026-09-17/rotation_patch/replay_summary.json) · [회귀 테스트 결과](validation/2026-09-17/rotation_patch/motion_tests.xml)

**현재 배포 상태**

소스 설정에 패치를 적용했고, 다음 확인에 사용할 install을 `/tmp/jackal_amcl_rotation_patch_20260917/install`에 만들었다. 그 안의 설정과 소스 설정이 동일함을 확인했다. 기존 workspace install은 덮어쓰지 않았다. ROS 프로세스 재기동, 실제 로봇 움직임, CPU/RAM 보호 조치는 수행하지 않았다.

향후 현장 확인에서는 실제로 로드된 `robot_model_type`이 Omni인지 확인한 뒤, 정상 초기 pose에서 제자리 양방향 회전을 비교해야 한다. 실제 센서 수평 장착 위치와 회전 중심 확인, deskew scan, TF 전달 정체, 정지 후 회복 설정은 후속 항목으로 남아 있다. 이번 테스트는 기존에 발견된 shutdown/lifecycle 문제의 해결을 검증하지 않는다.

**오프라인 비교 재현**

[자료 디렉터리](validation/2026-09-17/rotation_patch/)에 입력 JSON 압축본, 결과, 사용한 C++ 어댑터와 추출 스크립트를 저장했다. 추출 스크립트는 원본 기록 경로를 받으며, 압축된 입력으로 재실행할 때에는 원본 bag이 필요 없다. `.cpp.txt`, `.py.txt`는 실행에 사용한 검증 도구의 소스 사본이다.

ROS Humble와 해당 버전 AMCL이 설치된 환경에서 다음은 로봇 노드 없이 AMCL 코어 계산만 실행한다.

```bash
source /opt/ros/humble/setup.bash
replay_artifacts=/home/parkhajin/moai_navigation_ws/src/jackal_nav2_bringup/docs/validation/2026-09-17/rotation_patch
replay_work=$(mktemp -d /tmp/jackal_amcl_replay.XXXXXX)
cp "$replay_artifacts/core_replay.cpp.txt" "$replay_work/core_replay.cpp"
cp "$replay_artifacts/replay_CMakeLists.txt" "$replay_work/CMakeLists.txt"
gzip -dc "$replay_artifacts/replay_input.json.gz" > "$replay_work/input.json"
cmake -S "$replay_work" -B "$replay_work/build"
cmake --build "$replay_work/build" --parallel 2
"$replay_work/build/core_replay" "$replay_work/input.json" "$replay_work/results.json"
```

입력의 hash, 실행 수, 임시 install 위치는 [manifest](validation/2026-09-17/rotation_patch/manifest.json)에 있다. 위 코어 어댑터는 이번 설정(`likelihood_field`, 매 업데이트 resampling, recovery alpha=0)에 맞춘 검증 도구이며 범용 ROS bag 재생기를 대체하지 않는다.
