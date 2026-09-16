# Static–Dynamic Separated Costmap Policy Implementation Plan

> 2026-09-10 구현 정책 개정: static/global/local은 모두 사전 지도 전용이다.
> 아래 prediction/Social Encoder 항목은 후속 연구 설계이며 이번 구현이 아니다.
> 현재 구현·검증 범위는 [정적 지도·figure·독립 정지 감시](Static_Safety_Figures_Validation.md)를 따른다.

## 1. 목적

본 설계의 목적은 로봇 내비게이션에서 **정적 환경 정보(static physical context)​**와 **동적 객체 정보(dynamic social context)​**를 명시적으로 분리하는 것이다.

기본 정책은 다음과 같다.

```text
Prior SLAM Map
      │
      ▼
Static Costmap
      │
      ▼
Map Encoder


Human Tracking / Prediction
      │
      ▼
Social Encoder
```

즉,

- 벽, 기둥, 고정 장애물 등의 **정적 환경 정보**는 사전 SLAM map에서만 획득한다.
- 사람과 같은 **동적 객체는 static costmap에 포함하지 않는다.**
- 동적 객체의 위치 및 미래 trajectory는 tracking / prediction pipeline을 통해 별도로 처리한다.
- 실제 로봇 운용상의 collision safety는 별도의 live obstacle representation으로 처리한다.

이를 통해 모델 내부에서

```text
Physical context = Static map
Social context   = Dynamic agents
```

라는 명확한 representation separation을 유지한다.

---

# 2. 시스템 전제 조건

본 정책은 다음 조건을 전제로 한다.

## 2.1 Prior SLAM map

사전에 생성된 occupancy map이 존재해야 한다.

```text
/map
```

또는 이에 대응하는 occupancy grid가 제공되어야 한다.

Map에는 가급적 다음 요소만 포함되어야 한다.

- 벽
- 기둥
- 고정된 구조물
- 변경 빈도가 낮은 장애물

사람 및 이동 물체가 SLAM map에 영구적인 obstacle로 포함되는 것은 피한다.

---

## 2.2 Localization

사전 map 기준 localization이 가능해야 한다.

TF 구조:

```text
map
 │
 │ localization
 ▼
odom
```

Localization module은

```text
map → odom
```

transform을 제공한다.

---

## 2.3 Odometry

FAST-LIVO2를 이용하여 연속적인 local odometry를 제공한다.

```text
odom
 │
 │ FAST-LIVO2
 ▼
base_link
```

따라서 전체 TF tree는 다음과 같이 구성한다.

```text
map
 │
 │ localization
 ▼
odom
 │
 │ FAST-LIVO2
 ▼
base_link
 │
 ├── lidar
 ├── camera
 └── imu
```

---

# 3. Costmap 정책

전체 cost representation을 하나의 costmap으로 처리하지 않고 목적에 따라 분리한다.

```text
                    Navigation Representation
                              │
              ┌───────────────┴───────────────┐
              │                               │
              ▼                               ▼
       Static Map Context              Live Safety Context
              │                               │
       Prior SLAM Map                    MID-360
              │                               │
       Static Costmap                Independent LiDAR stop monitor
              │                               │
        Map Encoder                  Collision checking
```

---

# 4. Static Costmap

## 4.1 역할

Static costmap은 다음 목적으로 사용한다.

- Global planning
- Map Encoder input
- Static collision constraint
- Physical environment representation

반대로 다음 정보는 포함하지 않는다.

- 사람
- 이동 중인 카트
- 이동 로봇
- 기타 temporary dynamic obstacle

---

## 4.2 생성 방법

Nav2 `StaticLayer`를 이용하여 prior SLAM occupancy map으로부터 costmap을 생성한다.

```text
Prior SLAM Map
      │
      ▼
StaticLayer
      │
      ▼
InflationLayer
      │
      ▼
Static Costmap
```

구성 예시:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:

      global_frame: map
      robot_base_frame: base_link

      rolling_window: false

      plugins:
        - static_layer
        - inflation_layer

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

        inflation_radius: 0.45
        cost_scaling_factor: 3.0
```

중요한 점은 다음 sensor-based layer를 넣지 않는 것이다.

```text
ObstacleLayer
VoxelLayer
STVL
```

즉 Static Costmap에서는 현재 LiDAR observation을 사용하지 않는다.

---

# 5. Local Static Map Extraction

전체 SLAM map을 Map Encoder에 입력하지 않고 로봇 중심 local patch를 추출한다.

예:

```text
Global SLAM map

┌──────────────────────────────┐
│                              │
│                obstacles     │
│                              │
│           ┌─────────┐        │
│           │  local  │        │
│           │  patch  │        │
│           │    R    │        │
│           └─────────┘        │
│                              │
└──────────────────────────────┘
```

Localization으로 얻은 현재 pose

\[
T^{map}_{base}
\]

를 이용하여 local map을 추출한다.

예:

```text
input map size:
10 m × 10 m

resolution:
0.05 m

output:
200 × 200 occupancy grid
```

---

# 6. Robot-Centric Map Representation

Map Encoder에는 global 좌표보다 robot-centric representation을 사용하는 것을 기본 정책으로 한다.

```text
                   robot forward
                        ↑
                        │
                ┌───────────────┐
                │               │
                │       R       │
                │               │
                └───────────────┘
```

현재 robot pose를

```text
(x_r, y_r, yaw_r)
```

라고 하면 map point를

\[
p_{robot}
=
R(-yaw_r)
(p_{map}-p_r)
\]

로 변환한다.

이렇게 하면 Map Encoder 입력에서 로봇은 항상

```text
position = center
heading  = +x or +y
```

방향으로 정렬된다.

---

# 7. Map Encoder 입력

Map Encoder 입력은 다음 중 하나로 구현한다.

## Option A — Binary Occupancy

```text
0 = free
1 = occupied
```

가장 단순한 baseline.

---

## Option B — Nav2 Costmap Value

```text
0     = free
1~252 = inflated cost
253   = inscribed
254   = lethal
255   = unknown
```

Inflation 정보를 직접 모델에 제공할 수 있다.

---

## Option C — Distance Transform / SDF

Static occupancy map으로부터 obstacle distance를 계산한다.

예:

```text
Wall
████████████

distance
0 1 2 3 4 5 ...
```

또는 signed distance:

\[
SDF(p)
\]

로 표현한다.

모델이 obstacle boundary 자체보다는

```text
"obstacle에서 얼마나 떨어져 있는가"
```

를 이해하기 쉬우므로 향후 Map Encoder 입력으로 우선적으로 고려할 수 있다.

---

# 8. Social Context Pipeline

동적 사람 정보는 static costmap에 넣지 않는다.

별도의 pipeline을 유지한다.

```text
Camera / LiDAR
      │
      ▼
Human Detection / Tracking
      │
      ▼
Observed trajectories
      │
      ▼
Trajectory Prediction
      │
      ▼
Social Encoder
```

예를 들어 사람 \(i\)의 trajectory:

\[
H_i =
\{
(x_i^{t-N},y_i^{t-N}),
...,
(x_i^t,y_i^t)
\}
\]

를 Social Encoder에 제공한다.

따라서 모델 수준에서는

```text
Map Encoder
    ↓
Static physical geometry


Social Encoder
    ↓
Dynamic interaction
```

으로 역할이 분리된다.

---

# 9. Navigation Model Integration

현재 trajectory generation architecture에 다음처럼 연결한다.

```text
             Human trajectories
                    │
                    ▼
              Social Encoder
                    │
                    ▼
             Social Context
                    │
                    │
BERT Future Query ──┤
                    │
                    ▼
                 Fusion
                    ▲
                    │
             Static Map Encoder
                    ▲
                    │
             Static Costmap
                    ▲
                    │
              Prior SLAM Map
```

또는 현재 설계한 sequential cross-attention 구조에서는

```text
BERT future queries
       │
       │
       ▼
Social Context
       │
       ▼
Cross Attention
       │
       ▼
      Q¹
       │
       │
Static Map Encoder
       │
       ▼
Cross Attention
       │
       ▼
      Q²
       │
       ▼
Goal Conditioning
       │
       ▼
      CVAE
       │
       ▼
τ¹ τ² ... τᴷ
```

와 같이 구성한다.

---

# 10. 독립 LiDAR 감속·정지 감시

세 costmap은 사전 지도만 사용하므로 사람과 새로 놓인 물체를 모두 반영하지 않는다.
이들을 우회하는 경로를 만들지 않으며, 별도의 raw LiDAR 경로에서 감속·정지만 수행한다.

```text
Raw MID-360 -> relay -> base_link 변환 / finite / self-mask / 높이 제한
                                  |
                         /nav2/safety_points
                                  |
Velocity Smoother -> Collision Monitor -> independent Guard -> TwistStamped
```

Guard는 동일한 최소 정지 영역을 직접 검사하고 센서/명령의 freshness,
필수 TF, 비정상 명령, clock 역행을 검사한다. 사람 점 제거는 하지 않는다.
`enable_motion=false`가 기본이며 NUC 출력 전달 설정은 자동 변경하지 않는다.
현장 제동 거리와 watchdog/E-stop 검증 전에는 주행을 허용하지 않는다.

# 11. 정적 Costmap과 별도 정지 감시의 역할

| 입력/목적 | static/global/local costmap | 독립 LiDAR 감시 | 보행자 figure |
|---|---|---|---|
| 사전 지도 | O | X | X |
| 현재 LiDAR 장애물 입력 | X | O | perception 경유 |
| 사람·미등록 물체의 cell 생성 | X | cell 생성 없음 | cell 생성 없음 |
| 경로 계획 | 정적 경로만 | 감속·정지만 | RViz 표시만 |
| Map Encoder | static costmap만 | X | X |

소프트웨어 정지 기능은 인증된 안전장치를 대체하지 않는다.

---

# 12. Dynamic Object의 Map Encoder 유입 방지

본 정책의 핵심 invariant를 다음과 같이 정의한다.

> **Map Encoder에는 현재 sensor observation으로 생성된 obstacle을 직접 입력하지 않는다.**

즉 금지:

```text
MID-360
   ↓
ObstacleLayer
   ↓
Costmap
   ↓
Map Encoder
```

대신:

```text
Prior SLAM Map
   ↓
StaticLayer
   ↓
Static Costmap
   ↓
Map Encoder
```

을 사용한다.

이를 통해 사람이 동시에

```text
Human trajectory
→ Social Encoder

Human pointcloud
→ Map Encoder
```

에 중복 표현되는 문제를 방지한다.

---

# 13. 환경 변화 처리 정책

Prior map과 실제 환경이 달라질 가능성이 있으므로 세 가지 obstacle class를 개념적으로 구분한다.

```text
Environment Object
        │
        ├── Prior Static
        │
        ├── New Static
        │
        └── Dynamic
```

예:

```text
Prior Static
- wall
- pillar

New Static
- 새로 놓인 box
- 이동 후 고정된 desk

Dynamic
- human
- cart
- mobile robot
```

초기 구현에서는 Map Encoder에

```text
Prior Static
```

만 제공한다.

`New Static`은 safety layer에서는 장애물로 처리하지만 Map Encoder에는 반영하지 않는다.

---

# 14. 향후 Dynamic / New-Static 분리

필요한 경우 다음 확장을 추가한다.

```text
MID-360
   │
FAST-LIVO2
   │
   ▼
PointCloud registration
   │
   ▼
Prior-map consistency
   │
   ▼
Temporal consistency
   │
   ├──── Dynamic
   │
   └──── New Static
```

판정 정책:

```text
Prior map matched
        ↓
     STATIC


Prior map unmatched
        ↓
Temporal observation
    ┌────────┴────────┐
    ▼                 ▼
position changes    persistent
    │                 │
 DYNAMIC          NEW STATIC
```

이 기능은 초기 구현에는 포함하지 않고 Phase 3 이후 확장으로 둔다.

---

# 15. ROS2 Topic 구조

권장 topic structure:

```text
/map
 │
 └── Prior SLAM occupancy grid


/tf
 ├── map → odom
 └── odom → base_link


/static_costmap/costmap
 │
 └── static environment only


/map_encoder/input
 │
 └── robot-centric static map patch


/livox/lidar
 │
 └── current PointCloud2


/local_costmap/costmap
 │
 └── prior-map-only rolling window in odom


/ped_tracking
 │
 └── tracked pedestrian trajectories


/human_predictions
 │
 └── future pedestrian trajectories
```

---

# 16. ROS Node 구성

초기 implementation:

```text
                       map_server
                           │
                          /map
                           │
                           ▼
                 nav2 static_costmap
                           │
                           ▼
                 /static_costmap
                           │
                           ▼
                  map_patch_node
                           │
                           ▼
                 /map_encoder/input
                           │
                           ▼
                     Map Encoder
```

별도:

```text
MID-360
   │
   ▼
Raw LiDAR preprocessing
   │
   ▼
Collision Monitor + independent Guard
   │
   ▼
Guarded velocity output
```

그리고:

```text
Human tracker
     │
     ▼
Trajectory history
     │
     ▼
Prediction model
     │
     ▼
Social Encoder
```

---

# 17. `map_patch_node`

별도 ROS2 node를 구현하여 Map Encoder에 필요한 local static representation을 생성한다.

## Input

```text
/static_costmap/costmap
/tf
```

또는 직접:

```text
/map
/tf
```

## Output

```text
/map_encoder/input
```

Message type 후보:

```text
nav_msgs/OccupancyGrid
```

초기 구현에서는 OccupancyGrid 사용을 권장한다.

향후 neural network inference pipeline에서는 tensor로 직접 변환할 수 있다.

---

# 18. `map_patch_node` 처리 과정

```text
1. /map 수신

2. TF 조회
   map → base_link

3. robot position 계산

4. local map ROI 결정

5. occupancy grid crop

6. robot heading 기준 회전

7. resize / normalization

8. /map_encoder/input publish
```

pseudo pipeline:

```python
map = receive_global_map()

pose = lookup_transform(
    target="map",
    source="base_link"
)

patch = crop(
    map,
    center=pose.position,
    size=(10.0, 10.0)
)

patch = rotate(
    patch,
    angle=-pose.yaw
)

publish(patch)
```

---

# 19. Update Frequency

Static map 자체는 거의 변하지 않으므로 map을 매 cycle 다시 생성할 필요는 없다.

그러나 robot-centric patch는 로봇이 움직이므로 주기적으로 갱신해야 한다.

초기 권장:

```text
Static map:
on-change only

Map patch:
5–10 Hz

Trajectory generation:
10 Hz
```

현재 navigation model이 10 Hz target이라면 Map Encoder patch 역시 최대 10 Hz면 충분하다.

---

# 20. Localization Error 처리

Map patch의 품질은 localization 성능에 직접적으로 영향을 받는다.

예를 들어 localization error가 발생하면:

```text
Real

wall
██████

    Robot


Model input

wall
██████

       Robot
```

처럼 static obstacle 위치가 잘못 보일 수 있다.

따라서 다음을 로깅한다.

```text
localization covariance
map→odom transform
pose discontinuity
yaw discontinuity
```

Localization confidence가 크게 떨어질 경우 Map Encoder input을 일시적으로 freeze하거나 invalid 처리하는 정책도 향후 고려한다.

---

# 21. 초기 구현 단계

## Phase 1 — Static Costmap Baseline

목표:

```text
Prior SLAM map
→ Nav2 StaticLayer
→ static costmap
```

구현:

- [ ] map_server 실행
- [ ] localization 실행
- [ ] FAST-LIVO2 odometry 연결
- [ ] `map → odom → base_link` TF 검증
- [ ] global costmap에서 `StaticLayer`만 활성화
- [ ] LiDAR observation layer 제거
- [ ] 실제 사람이 지나가도 static costmap이 변하지 않는지 확인

### 검증

사람이 로봇 앞을 통과해도

```text
/static_costmap
```

의 해당 위치 cost 값이 변하지 않아야 한다.

---

# 22. Phase 2 — Map Encoder Input

목표:

```text
Static Costmap
→ Robot-centered crop
→ Map Encoder
```

구현:

- [ ] `map_patch_node` 구현
- [ ] local ROI crop
- [ ] robot-centric rotation
- [ ] fixed resolution 적용
- [ ] fixed input size 적용
- [ ] occupancy normalization
- [ ] Map Encoder 연결

초기 parameter 후보:

```yaml
map_patch:
  width: 10.0
  height: 10.0

  resolution: 0.05

  output_width: 200
  output_height: 200

  update_rate: 10.0
```

---

# 23. Phase 3 — 독립 LiDAR 정지 감시

- 세 costmap의 StaticLayer + InflationLayer 계약 유지
- raw LiDAR 전처리 -> Collision Monitor -> 독립 Guard 연결
- 동일 stop rectangle, 3점 threshold, .30 s sensor / .25 s command timeout
- 합성 입력 및 테스트 명령 토픽에서 감속·정지·센서 단절 검증
- 실제 플랫폼은 사용자 입회와 watchdog/E-stop 확인 후 별도 검증

---

# 24. Phase 4 — Social Pipeline Integration

목표:

```text
Human tracks
→ Social Encoder

Static map
→ Map Encoder

두 representation fusion
→ CVAE trajectory generation
```

검증해야 할 핵심은 다음이다.

### Social only

```text
Human context = ON
Static map    = OFF
```

### Map only

```text
Human context = OFF
Static map    = ON
```

### Social + Map

```text
Human context = ON
Static map    = ON
```

이를 통해 두 representation의 역할을 명확하게 분석한다.

---

# 25. Phase 5 — Environment Change Handling

향후 필요할 경우 구현한다.

```text
Current LiDAR
     +
Prior SLAM Map
     +
FAST-LIVO2
     │
     ▼
Map Consistency Filter
     │
     ▼
Temporal Consistency
     │
 ┌───┴──────────┐
 ▼              ▼
Dynamic      New Static
```

이 단계에서는 Dynablox와 같은 detection-free dynamic filtering 계열 방법을 참고할 수 있다.

---

# 26. 평가 항목

## A. Costmap correctness

### Static preservation

고정 장애물이 항상 동일 위치에 존재하는지 평가한다.

### Dynamic exclusion

사람이 static costmap에 나타나지 않는지 평가한다.

---

## B. Map alignment

Localization된 robot pose 기준으로 local map patch의 obstacle 위치가 실제 LiDAR와 일치하는지 평가한다.

---

## C. Model performance

기존 trajectory prediction 평가:

```text
ADE
FDE
```

Navigation 평가:

```text
Collision Rate
ITR
PTTC
Goal success
Time to Goal
```

Static obstacle collision은 별도로 기록한다.

---

# 27. Ablation

최종적으로 다음 ablation을 구성한다.

| Experiment | Social | Static Map | Live obstacle safety |
|---|---:|---:|---:|
| BERT baseline | O | X | O |
| Map only | X | O | O |
| Social + Map | O | O | O |
| Social + sensor costmap | O | Dynamic 포함 | O |

특히 마지막 비교를 통해

```text
clean static representation
```

과

```text
dynamic-contaminated costmap
```

의 차이를 확인할 수 있다.

---

# 28. 핵심 설계 원칙

본 구현에서는 다음 세 가지 representation을 의도적으로 구분한다.

```text
1. Static Physical Context
   Prior SLAM map
       ↓
   Static Costmap
       ↓
   Map Encoder


2. Dynamic Social Context
   Human tracking
       ↓
   Human prediction
       ↓
   Social Encoder


3. Reactive Safety Context
   Current LiDAR
       ↓
   Independent LiDAR stop monitor
       ↓
   Collision safety
```

따라서 최종 architecture는 다음과 같다.

```text
                         Navigation System

        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼

  Static Physical       Dynamic Social       Reactive Safety

  Prior SLAM Map        Human Tracking          MID-360
        │                     │                     │
        ▼                     ▼                     ▼
 Static Costmap         Prediction          LiDAR stop monitor
        │                     │                     │
        ▼                     ▼                     │
   Map Encoder          Social Encoder             │
        │                     │                     │
        └─────────┬───────────┘                     │
                  ▼                                 │
          Trajectory Generator                      │
                  │                                 │
             K trajectories                         │
                  │                                 │
                  ▼                                 │
              Selector                              │
                  │                                 │
                  ▼                                 │
            Tracking / MPC ◄────────────────────────┘
                  │
                  ▼
                Robot
```

---

# 29. 구현 우선순위

가장 먼저 다음 최소 구성을 완성한다.

```text
[1]
Prior SLAM Map
      ↓
Static Costmap

[2]
Static Costmap
      ↓
Robot-centric crop
      ↓
Map Encoder

[3]
Human trajectory
      ↓
Social Encoder

[4]
MID-360
      ↓
Independent LiDAR stop monitor
      ↓
Safety / Collision checking
```

이후 환경 변화 문제가 실제 실험에서 유의미하게 발생하는 것이 확인될 경우에만

```text
prior-map consistency
+
temporal dynamic filtering
```

을 추가한다.

초기 단계부터 dynamic filtering을 복잡하게 구현하지 않는 것이 전체 시스템의 원인 분석과 ablation 측면에서 유리하다.

---

# 30. 최종 정책

본 시스템에서 costmap의 역할을 다음과 같이 정의한다.

> **Static costmap은 현재 센서가 관측한 모든 장애물을 표현하는 지도가 아니라, navigation model이 사용할 정적 물리 환경 prior를 표현한다.**

따라서:

```text
Prior SLAM map
      ↓
Static physical constraint
      ↓
Map Encoder
```

와

```text
Human observations
      ↓
Dynamic social constraint
      ↓
Social Encoder
```

를 분리한다.

실시간 센서 기반 obstacle representation은 별도의 safety layer로 유지하며, trajectory generation model의 static map representation과 혼합하지 않는다.
