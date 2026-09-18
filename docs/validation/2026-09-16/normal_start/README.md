# 정상 기동 절차 확인 자료

판정·명령은 [Normal_Startup_2026-09-16.md](../../../Normal_Startup_2026-09-16.md)를 참고한다.

- nav2_only: Image OFF, FAST 전 20초.
- nav2_fast: 첫 FAST가 IMU 미수신으로 LiDAR buffer 보호 종료된 35초. 실패 표본도 보존.
- nav2_fast_retry: 단일 FAST 재기동 후 30초, 처리 정상.
- full_stack / full_scene: perception 포함 60초. 관측 구간은 겹침.
- localization: 60초 TF audit, 사용자 pose 재입력 포함. map 99% 기준 미달.
- after_pose: pose 입력 후 별도 30초. odom 100%, map 77.81%, 역행 0. 안정성 미합격.
- nav2_no_raw_image.rviz: 명령 안내용 map Fixed Frame 설정. 기존 원본에서 Image 활성화 두 항목만 OFF.
- rviz_saved_during_trial.rviz: 실행 중 RViz에서 저장된 화면 배치 및 base_link Fixed Frame 상태.
  실행 안내용으로 쓰지 않는다. 런타임 전체의 UI 설정 이력은 기록하지 못했다.
- launch_logs.tar.gz: console/ROS 로그와 생성한 perception YAML 원본.

2026-09-16 14:35 KST 최종 확인: 이번 launch/자식 PID 잔여 없음.
NUC jackal-sensors.service 및 clearpath-platform.service 모두 active.
TF tolerance·sensor stamp·buffer 상한·원본 네트워크/기본 RViz 설정 변경 없음.
