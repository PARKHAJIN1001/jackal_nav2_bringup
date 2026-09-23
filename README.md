# Jackal Nav2 Bringup (`jackal_nav2_bringup`)

Clearpath Jackal 로봇을 위한 고성능 AMCL 2D 사전 지도 위치 추정(Localization) 및 Navigation 2 (Nav2) 완전 통합 자율주행 패키지입니다.  
FAST-LIVO2 (LiDAR-Inertial Odometry)의 고정밀 3D 오도메트리를 기반으로 AMCL의 2D 점유격자 지도(Occupancy Grid Map) 정합을 융합하며, 2계층 분리 아키텍처(2-Terminal Decoupled Architecture)를 통해 높은 안정성과 확장성을 제공합니다.

---

## 1. 패키지 목적 및 아키텍처 (Purpose & Architecture)

### 1.1 시스템 아키텍처
```text
[ 하드웨어 센서 (NUC) ]
  ├── Livox MID-360 LiDAR (/livox/lidar)
  └── RealSense D455 Camera (/camera/camera/color/image_raw)
          │
          ▼ (LAN via Fast-DDS)
[ 터미널 A: Localization 시스템 (`localization.launch.py`) ]
  ├── pointcloud_relay_node       : MID-360 포인트클라우드 릴레이 (/livox/lidar_local)
  ├── fast_livo (외부 알고리즘)     : 3D 라이다-관성 오도메트리 생성 (/aft_mapped_to_init)
  ├── fast_livo_odom_adapter.py   : /aft_mapped_to_init -> odom -> base_link TF & /odom 변환
  ├── pointcloud_to_laserscan     : 3D PointCloud -> 2D LaserScan (/scan)
  ├── map_server                  : 사전 점유격자 지도(Prior Map) 로드 및 /map 발행
  ├── amcl                        : 2D 레이저 스캔 기반 map -> odom TF 추정
  └── rviz2                       : 통합 시각화 UI
          │
          ▼ (TF & Scan & Costmap 공유)
[ 터미널 B: Nav2 내비게이션 시스템 (`nav2.launch.py`) ]
  ├── Nav2 Controller Server      : DWB 로컬 경로 추종 및 속도 명령 생성
  ├── Nav2 Planner Server         : NavFn 전역 경로 계획
  ├── Nav2 Smoother Server        : 경로 곡률 최적화 및 평활화
  ├── Nav2 Behavior Server        : 장애물 회피 및 복구 행동 트리
  ├── Nav2 BT Navigator           : 네비게이션 행동 트리(Behavior Tree) 실행
  ├── Velocity Smoother           : 가감속 한계 기반 속도 스무딩
  ├── Collision Monitor           : 기구학적 다각형 충돌 예측
  └── Safety Guard                : 실시간 라이다 점군 기반 안전 감시 및 비상 정지
```

### 1.2 TF 트리 소유권 (TF Frame Ownership)
```text
map (지도 절대 좌표계)
 └── odom (오도메트리 좌표계)             : AMCL이 map -> odom 브로드캐스팅
      └── base_link (로봇 중심)           : fast_livo_odom_adapter가 odom -> base_link 브로드캐스팅
           ├── livox_frame (LiDAR)        : static_transform_publisher
           └── camera_link (Camera)       : static_transform_publisher
```

- **Clearpath 기본 TF 분리**: Clearpath 기본 바퀴 EKF는 `/j100_0519/tf` 네임스페이스를 사용하며, 이 패키지의 글로벌 TF 트리 (`/tf`, `/tf_static`)와 충돌하지 않도록 격리되어 있습니다.

---

## 2. 패키지 의존성 (Dependencies)

### 2.1 ROS 2 Humble 필수 패키지
- `ros-humble-navigation2` 및 `ros-humble-nav2-bringup`
- `ros-humble-nav2-amcl`
- `ros-humble-nav2-map-server`
- `ros-humble-nav2-controller`
- `ros-humble-nav2-planner`
- `ros-humble-nav2-smoother`
- `ros-humble-nav2-behaviors`
- `ros-humble-nav2-bt-navigator`
- `ros-humble-nav2-collision-monitor`
- `ros-humble-nav2-velocity-smoother`
- `ros-humble-nav2-rviz-plugins`
- `ros-humble-pointcloud-to-laserscan`
- `ros-humble-bondcpp`
- `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs`

### 2.2 외부 SLAM 패키지
- `fast_livo` (FAST-LIVO2 ROS 2 패키지, `~/moai_navigation_ws` 또는 `~/ws_livox`)

### 2.3 시스템 라이브러리 및 Python 모듈
- Python 3.10+
- `python3-numpy`, `python3-scipy`, `python3-yaml`, `python3-pytest`

---

## 3. Git Clone 후 빌드 및 초기 설정 (Initial Setup)

### 3.1 패키지 의존성 일괄 설치
```bash
cd ~/moai_navigation_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 패키지 빌드
```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select jackal_nav2_bringup
source install/setup.bash
```

### 3.3 사전 지도(Prior Map) 확인
기본 제공 지도는 `maps/frontier_10F/frontier_10F.yaml` (포함 파일: `.pgm`)입니다.  
사용자 환경에 맞추어 다른 지도를 사용할 경우 절대 경로를 launch 파라미터(`map:=...`)로 전달할 수 있습니다.

---

## 4. 패키지 운용 방법 (Step-by-Step Operation Guide)

운용 전 항상 NUC와 유선 LAN이 연결되어 있고 센서 스트림이 정상 동작하는지 확인하십시오 (`ping -c 3 192.168.50.2`).

### 4.1 터미널 A: Localization 시스템 기동
새 터미널을 열고 `jackal` 환경을 불러온 뒤 위치 추정 스택을 실행합니다.
```bash
jackal
ros2 launch jackal_nav2_bringup localization.launch.py
```
- 실행 내용: FAST-LIVO2, Odom 어댑터, PointCloud-to-LaserScan, Map Server, AMCL, RViz2
- RViz2 창이 자동으로 실행되며 지도(Map) 레이어가 표시됩니다.

### 4.2 RViz2 초기 위치(Initial Pose) 지정 및 정합 확인
1. RViz2 상단 툴바의 **2D Pose Estimate** 버튼을 클릭합니다.
2. 지도의 실제 로봇 위치를 클릭하고, 로봇이 바라보는 방향(헤딩)으로 드래그하여 녹색 화살표를 배치합니다.
3. **정합 검증**:
   - 빨간색 점으로 표시되는 2D LaserScan (`/scan`) 데이터가 지도의 검은색 벽면 라인과 정확히 겹치는지 육안으로 확인합니다.
   - 터미널이나 에코를 통해 AMCL의 위치 추정 신뢰도와 TF 브로드캐스트를 확인합니다:
     ```bash
     ros2 topic echo --once /amcl_pose
     ros2 run tf2_ros tf2_echo map odom
     ```

### 4.3 터미널 B: Nav2 내비게이션 기동
위치 정합이 확인되면 또 다른 새 터미널을 열고 Nav2 코어를 실행합니다.

#### A. 기본 안전 모드 (모터 구동 차단, 시뮬레이션 및 경로 확인용)
```bash
jackal
ros2 launch jackal_nav2_bringup nav2.launch.py enable_motion:=false
```

#### B. 실제 자율주행 모드 (모터 속도 명령 전달 활성화)
```bash
jackal
ros2 launch jackal_nav2_bringup nav2.launch.py enable_motion:=true
```

- 실행 내용: Nav2 7대 서버 노드, 속도 평활화(Velocity Smoother), 충돌 모니터(Collision Monitor), 안전 가드(Safety Guard).
- `enable_motion:=true`일 때만 `/j100_0519/nav2_cmd_vel`로 실제 모터 구동 명령이 전송됩니다.

### 4.4 목표 지점 주행 및 안전 정지
1. **Nav2 Goal 명령 전달**:
   - RViz2 상단 툴바의 **Nav2 Goal** 버튼을 클릭합니다.
   - 로봇이 도달할 목표 위치를 클릭하고 원하는 도착 헤딩 방향으로 드래그합니다.
   - 전역 경로(녹색 선) 및 로컬 경로가 생성되며 로봇이 주행을 시작합니다.
2. **비상 정지 (Emergency Stop)**:
   - 비상 상황 시 터미널에서 즉시 `Ctrl-C`로 `nav2.launch.py`를 종료하거나, Jackal 본체의 물리 e-stop 버튼을 누릅니다.
   - 소프트웨어 비상 정지 모니터 도구를 활용할 수도 있습니다:
     ```bash
     ros2 run jackal_nav2_bringup operator_stop.py
     ```

---

## 5. Launch 파라미터 가이드 (Launch Arguments)

### `localization.launch.py`
| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `map` | `$(find jackal_nav2_bringup)/maps/frontier_10F/frontier_10F.yaml` | 점유격자 지도 YAML 절대 경로 |
| `use_rviz` | `true` | RViz2 시각화 창 실행 여부 |
| `raw_lidar_topic` | `/livox/lidar` | 원본 MID-360 포인트클라우드 토픽 |
| `lidar_pointcloud_topic` | `/livox/lidar_local` | 릴레이된 로컬 포인트클라우드 토픽 |

### `nav2.launch.py`
| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `enable_motion` | `false` | 모터 속도 명령 출력 허용 여부 (`true` 시 주행 활성화) |
| `use_rviz` | `false` | RViz2 실행 여부 (터미널 A에서 이미 띄웠으므로 기본값 false) |
| `launch_stability_monitor`| `false` | 연속 180초 안정성 검증 모니터링 노드 기동 여부 |
| `use_map_patch` | `false` | 로컬 맵 패치 노드 기동 여부 |
| `use_collision_monitor` | `true` | 기구학적 다각형 충돌 방지 노드 실행 여부 |
| `use_safety_guard` | `true` | 실시간 점군 기반 속도 감속 및 비상 정지 가드 실행 여부 |

---

## 6. 단위 및 통합 테스트 (Unit & Integration Tests)

본 패키지는 지속적인 품질 관리를 위해 450여 개의 단위 테스트 및 통합 테스트를 포함하고 있습니다.
```bash
cd ~/moai_navigation_ws
colcon test --packages-select jackal_nav2_bringup
colcon test-result --verbose
```
- 모든 테스트가 0 failure, 0 error로 통과해야 합니다.
- 정적 코드 분석:
  ```bash
  ament_flake8 --config .flake8 .
  ```
