# 재현 시험 증거

[판정·재현 커맨드](../../../Relaunch_Reproduction_2026-09-16.md)

- A_false: 원본 DDS + 분리 실행. IMU buffer 보호 종료 재현.
- B_sequential: 완전 종료 후 원본 DDS + composition 순차 실행. FAST 단계에서 실패하여 perception 미기동.
- C_udp: 완전 종료 후 임시 UDP-only + composition, 세 launch 별도 세션 순차 실행. 동일 실패.
- `launch_logs.tar.gz`: 세 시험 console/ROS 로그와 생성한 perception 설정. 원본 센서 bag는 아님.
- 각 JSON: 별도 관측 창이며, 시작/끝과 초기화/종료 포함 여부는 상세 기록을 따른다.
- `post_stop_raw.json`: launch 종료 후 원본 센서 수신 비교. 전체 부하 상태의 성공 증거가 아님.
- `startup_observer.py`: 사용한 bounded read-only 관측기. 명령/초기 pose를 발행하지 않음.
- `fastdds_laptop_original.xml`, `fastdds_laptop_udp_only.xml`: 원본과 임시 비교 설정. 시스템에 배포하지 않음.
- `shutdown_verification.json`: 로그에서 추출한 각 launch/자식 PID를 호스트 /proc와 대조한 종료 결과.

기존 ROS CLI daemon, NUC 서비스는 종료 대상이 아니다. 종료 오류(-6/-11/-9)는
정상적인 clean shutdown으로 집계하지 않는다. 주행이나 장기 TF 안정화 성공 자료가 아니다.
