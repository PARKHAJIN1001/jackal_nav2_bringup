# 2026-09-16 정지 실기 비교 계측

**10분 TF 안정화 미통과 / 주행 미실시 / 검증용 laptop 프로세스 종료 확인.**
해석·제약·후속 진단은 [전체 기록](../../../Connected_Validation_2026-09-16.md).

- `localization60/`, `full60/`: 각각 60초, recorder `recorded`, steady TF 100%, 역행 0.
- `full600/`: 600초, recorder `recorded_with_findings`; scan→map 8145/8496,
  scan→odom 8496/8496, AMCL TF 역행 5회. 실제 parameter/config/버전 포함.
- `scene60.json`, `scene690.json`: 별도 read-only marker/costmap/진단/최종 속도 관측.
  사용한 관측 코드는 [9월 14일 observer](../../2026-09-14/stationary_pilot/scene_observer.py)와 동일하다.
- `resources_localization150.jsonl`, `resources_full720.jsonl`: 11개 PID RSS/swap와 시스템 UDP.
- `nav2_udp_queue120.jsonl`: Nav2 socket 7667의 10초 간격 읽기 전용 표본.
- `dds_sockets_*.txt`: 서로 다른 시점의 socket memory/drop snapshot. 누적값과 구간 증가를 구별한다.
- `costmap_node_info.txt`: 실제 세 costmap node의 subscription graph.
- `clock_preflight.json`, `clock_end_window.json`, `chrony_end_window.txt`: 시각 비교, 보정 없음.
- `nuc_usb_kernel.log`: 초기 카메라 오류 근거 일부. 모든 재연결 로그를 담은 것은 아니다.
- `launch_logs.tar.gz`: Nav2/FAST/perception 전체 로그, 정상 운용과 종료 오류를 구분한다.

각 recorder/observer의 측정 구간은 완전히 같지 않다. raw pointcloud/image/전체 rosbag는 저장하지 않았다.
장비 주소, 로컬 경로, PID가 포함되므로 외부 공유 전 검토한다.
