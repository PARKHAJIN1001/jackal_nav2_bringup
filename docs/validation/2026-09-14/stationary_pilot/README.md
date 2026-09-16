# 정지 실기 예비 계측 증거

판정: **미통과 / 10분 및 주행 검증 미실시**.
상세 해석과 다음 단계는 [Stationary_Pilot_2026-09-14.md](../../../Stationary_Pilot_2026-09-14.md).

- `pilot60/`: 원본 recorder, 실제 params/config/버전, full-perception audit 60초.
  recorder state=incomplete: figure/trace 노드 없음. TF audit에도 별도 두 가지 findings가 있다.
- `tf_light30.json`: perception은 계속 실행, audit의 추가 perception 입력 구독만 생략한 후속 30초.
  측정 순서와 구간이 달라 엄격한 부하 A/B 인과 검증은 아니다.
- `resources_pilot150.jsonl`: 5개 PID RSS/swap, 시스템 메모리/UDP 증가량. perception PID 시계열은 없음.
- `scene60.json`, `scene_observer.py`: 별도 read-only 60초 관측과 당시 임시 코드.
  final speed, tracking, full costmap hash, 진단. figure/trace 메시지 미수신.
- `clock_preflight.json`, `clock_after_pilot.json`: 양쪽 OS 시각 10회 비교, 시각 수정 없음.
- `pilot60_recorder.log`: recorder 단계/종료 상태.
- `launch_logs.tar.gz`: Nav2, FAST, perception 전체 로그. 초기화·정상·종료 구간을 구분할 것.
  FAST FATAL은 relay 종료 후 IMU buffer cap 사건, Nav2 -6/-9는 종료 단계다.

원래 세션 디렉터리: `/tmp/nav2_stationary_20260914.1xmPBa`.
로그에는 당시 장비 주소와 PID가 들어 있으므로 외부 공유 전 검토한다.
