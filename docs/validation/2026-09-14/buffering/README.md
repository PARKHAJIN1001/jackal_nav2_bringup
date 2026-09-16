# 실행 중 버퍼링 진단 — 2026-09-14

## 범위와 상태

- 약 16:01–16:08 KST에 실행 중 프로세스, 설정, 로그, ROS endpoint, 메시지 timestamp를 읽기 전용으로 조사했다.
- 런타임 설정·시각·소스 코드를 변경하지 않았고 재시작이나 주행 명령을 수행하지 않았다.
- 사용자 종료 후 기존 Nav2/full_stack/FAST-LIVO2/YOLO/복구 노드/RViz PID가 사라진 것을 확인했다. 전체 NUC 서비스 종료를 의미하지는 않는다.
- 종료 후 RAM available 12,148 MiB. swap 사용량 2,147 MiB는 남아 있었다. swap을 강제로 비우지 않았다.

## 핵심 결론

하나의 relay backlog로 설명할 수 없다. **FAST-LIVO2의 지속적인 메모리 증가**와 **scan/AMCL TF 공급 단절**이 함께 확인됐다. 원본 cloud/odometry의 수초 지연은 짧은 관측 구간에서는 재현되지 않았다. 메모리의 정확한 할당 주체와 TF 단절의 최초 원인은 아직 확정하지 않았다.

## 1. 메모리 증가 — 확인됨

FAST-LIVO2 PID 188609에 대한 순차적인 ps 관측:

| 프로세스 경과 시간 | RSS (KiB) | RSS (GiB) |
|---|---:|---:|
| 255초 | 4,471,792 | 4.26 |
| 429초 | 7,451,308 | 7.11 |
| 507초 | 8,783,052 | 8.38 |

252초 동안 약 4.11 GiB 증가했다(약 16.7 MiB/s). 호스트 swap도 약 44 MiB에서 1,566 MiB로 늘었다. FAST-LIVO2 CPU는 약 225–229%, YOLO는 약 202%였다. 멀티코어 합산값이며 전체 CPU 100% 포화를 뜻하지 않는다.

실행 중 FAST-LIVO2는 `common/img_en=1`인 영상 사용 모드였다. 소스에 cloud/IMU/image 구독 `keep_last(200000)`와 명시적 상한 없는 영상 버퍼가 있다. 이들은 조사 대상이지만, RSS만으로 DDS backlog/영상 큐/visual map/누수 중 어느 것이 주원인인지 단정할 수 없다. `Waiting LiDAR data` 로그 역시 프레임 사이의 빈 큐 확인에서 출력될 수 있어 센서 단절의 증거로 사용하지 않는다.

## 2. 메시지 신선도와 TF — 확인됨

`local_timing.json`: depth 1 BEST_EFFORT 관측자, 약 20초. age는 수신 시 wall time에서 원래 header stamp를 뺀 값이다. 실제 발행률이 아니라 관측자 수신률이며 관측자 자체의 유실 가능성도 있다.

| 입력 | 수신률 | age 중앙값 | age p95 | 최대 수신 간격 |
|---|---:|---:|---:|---:|
| `/livox/lidar_local` | 14.98 Hz | 80 ms | 87 ms | 133 ms |
| `/aft_mapped_to_init` | 15.00 Hz | 76 ms | 86 ms | 120 ms |
| `/lidar/accumulated` | 15.02 Hz | 94 ms | 100 ms | 73 ms |
| `/ped_tracking` | 15.00 Hz | 117 ms | 123 ms | 84 ms |
| `/ped_yolo/instance_mask` | 10.54 Hz | 427 ms | 460 ms | 110 ms |
| `/nav2/safety_points` | 14.94 Hz | 92 ms | 100 ms | 136 ms |
| `/scan` | 5.67 Hz | 124 ms | 147 ms | 1.47초 |
| `/tf:map->odom` | 0.76 Hz | 아래 주석 참고 | 아래 주석 참고 | 5.88초 |

AMCL의 미래 timestamp는 `transform_tolerance=1.0`에 따른 정상 정책이다. 음수 age 자체를 시각 오류로 판단하지 않았다. Tracking은 cloud 기반 stamp이므로 표의 117 ms를 카메라 detection부터의 전체 지연으로 해석하면 안 된다.

별도의 15초 `tf_localization_audit.py` 측정은 처음 2초를 워밍업으로 분리하고 각 scan에 최대 0.5초의 TF 도착 유예를 주었다. 정상 구간 누적 결과:

- scan → odom: **43/43 (100%)**.
- scan → map: **10/43 (23.3%)**.
- global `/tf`의 `map -> odom` writer는 `/amcl`, `odom -> base_link` writer는 `/laserMapping`으로 각각 하나였다. 이 구간에 timestamp 역행은 없었다.
- 감사 종료 시 map→odom의 최신 stamp age는 약 1.53초였다.
- NUC EKF의 odom→base_link는 `/j100_0519/tf`에 별도로 관측됐다. 별도 topic에 존재한다는 사실만으로 Nav2 `/tf`의 중복 writer라고 판단하지 않았다.
- RViz queue-full 로그와 AMCL의 scan TF cache/queue 오류가 반복됐다. **map TF를 기다리며 화면 메시지 큐가 차는 것이 버퍼링의 유력한 직접 경로**다. 메모리 압박·메시지 손실·callback scheduling 중 최초 원인은 추가 분리가 필요하다.

## 3. 자동 AMCL 재초기화 — 확인됨

`amcl_recovery_monitor`는 약 8분 39초 동안 `/initialpose`를 **38회** 자동 발행했다. 최단 간격은 약 5.81초였다. 낮은 covariance의 과거 pose를 보관한 뒤 covariance 임계값을 넘으면 현재 stamp로 다시 발행하는 구현이다.

반복 초기화는 정상 수렴을 방해할 가능성이 있고, 로봇이 이동했을 때 과거 pose를 현재 위치로 재사용할 위험이 있다. 다만 이것이 TF 단절이나 메모리 증가의 단독 원인이라는 인과 관계는 입증하지 않았다. 수동 초기 pose 원칙과 함께 우선 재검토할 대상이다.

## 4. Relay 경로와 네트워크 — 확인됨

실제 ROS graph 및 GetParameters 기준:

- `/livox/lidar` 직접 구독: `pointcloud_relay`, `laserMapping`, `mid360_lidar_accumulator_node`.
- `/livox/lidar_local` 구독: scan projection, safety guard(관측 중 진단 노드 포함).
- 따라서 **FAST-LIVO2와 perception은 현재 relay를 우회**했다. relay가 모든 소비자의 네트워크 수신을 통합하는 구성이 아니었다.
- FAST-LIVO2와 accumulator의 실제 입력 파라미터도 `/livox/lidar`였다. 실행 중 프로세스의 값이므로 이후 소스 기본값 변경 여부와 구분해야 한다.
- 두 시스템 관측 사이 UDP RcvbufErrors가 5,026,743 → 5,067,801로 **41,058 증가**했다. 기존 누적값만 보고 손실을 추정한 것이 아니다. 모든 오류를 특정 토픽에 귀속할 수는 없다.
- FastDDS XML의 receive buffer 요청은 26,214,400 bytes였으나 kernel `net.core.rmem_max`는 212,992 bytes였고, 관측된 DDS socket rb는 425,984였다. 요청한 크기가 실제 적용됐다고 볼 수 없는 상태다.
- 동일 구간 IP reassembly failure 누적값은 증가하지 않았다. 과거 누적 fragmentation 오류를 현재의 주원인이라고 주장하지 않는다.

## 5. 시각과 안전

- laptop chrony: 남은 보정 약 59 μs, Last offset 약 -72 μs, Leap normal.
- NUC timesyncd: 선택 서버 192.168.50.1, Offset 약 +88 μs, Jitter 약 214 μs.
- 이 NTP 통계는 양호하다. 이번 증거는 과거의 수초 시각 불일치 재발을 지지하지 않는다. 동시 직접 시각 비교를 수행한 것은 아니다.
- Guard 실제 `enable_motion=true`. 최종 출력에 NUC bridge 구독자가 존재했다. 플랫폼의 전달 차단 여부는 이 조사에서 확정하지 않았다.
- 20초 동안 Guard 진단 19건 중 17건이 `tf_stale:map->odom`, 2건이 `independent_stop`이었다. 최종 Twist의 각 성분을 따로 검증하지 않았으므로 실제 정지 명령 검증 완료를 의미하지 않는다.

## 후속 조치 제안 — 이번에는 미적용

1. 다음 실험은 `enable_motion:=false`로 제한한다. 자동 recovery의 반복 재초기화를 먼저 분리하고, 사용자 초기 pose 한 번으로 비교한다.
2. 영상 비활성 LIO / 영상 활성 LIVO를 동일 조건으로 짧게 비교하고 RSS 증가율을 기록한다. 그 뒤 의심되는 실제 큐 길이와 visual map 보유량을 계측하여 수정 대상을 특정한다. 큐를 임의로 크게 늘리지 않는다.
3. FAST-LIVO2와 perception의 LiDAR 입력을 의도한 relay 경로로 맞추고 수신률·지연·UDP 오류 증가량을 다시 비교한다. 이것만으로 메모리 문제가 해결된다고 가정하지 않는다.
4. socket buffer와 DDS transport 조정은 실제 한계/손실을 근거로 적용하고, sudo가 필요한 항목은 사용자에게 별도 요청한다.
5. scan projection/AMCL callback과 TF 수신 경로를 분리 계측한다. tolerance 또는 timestamp 변경으로 결손을 숨기지 않는다.
6. 메모리 증가와 TF 결손을 해결한 뒤에만 장시간 TF/신선도 검증 및 사용자 입회 주행을 진행한다.

## 증거 파일

- `local_timing.json`: 샘플, 요약, 실제 파라미터, graph, Guard 진단.
- `tf_audit.json`: writer, TF 시각, 전체 구간 scan exact-time 성공률.
- `probe.py`: 이번에 사용한 읽기 전용 관측자.
- `amcl_recovery_monitor.log`, `nav2_container.log`, `rviz.log`, `fast_livo.log`: 종료 후 복사한 관련 로그.

원본은 `/tmp/nav2_buffer_diagnosis_20260914.olrlEs`에서 수집했고, 후속 구현 시 이 저장소로 복사하여 보존했다. 측정 당시 실행본에 대한 자료이며 이후 가져온 외부 소스의 검증 결과가 아니다.
