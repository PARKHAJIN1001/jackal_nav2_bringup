# 2026-09-18 수동 조이스틱 무응답 진단

새 운영 절차 재현 후 수동 주행이 안 된다는 사용자 제보를 읽기 전용으로 조사했다.

- Nav2는 `enable_motion=false`로 기동 중이다. 이 설정은 Nav2 출력 제한이다.
- 로봇 bridge의 실제 `forward_cmd_vel=false` 파라미터 응답을 확인했다. 구현상 이 모드는 robot `/cmd_vel` publisher 자체를 생성하지 않는다.
- 실제 twist_mux 설정에서 joystick 우선순위 10, 외부 cmd_vel 우선순위 1이었다. 비상정지 메시지는 false였다.
- joy 노드는 DS4DRV 가상 장치 `/dev/input/js0`를 열고 있으며 Joy 메시지를 반복 발행했다. 그러나 모든 축·버튼 값이 0이었고, 사용자가 L1을 눌렀다 뗀 관측 구간에도 버튼 변화가 기록되지 않았다. joystick cmd_vel도 수신되지 않았다.
- DS4 서비스 로그에 14:27:35 이후 신호 저하, 14:27:55 Disconnected가 기록됐다.
- Bluetooth 실제 조회 결과: 기존 컨트롤러 `Paired: yes`, `Trusted: yes`, `Blocked: no`, **`Connected: no`**.

이번 수동 입력 장애의 직접 확인된 문제는 컨트롤러와 로봇 사이 Bluetooth 연결 해제다. 장치 파일과 Joy 반복 발행만으로 실제 컨트롤러 연결을 확인할 수 없었다. Nav2 정지 설정을 해제해 해결할 문제라는 근거는 없다. 사용자에게 스틱 중앙·활성화 버튼 해제 상태에서 PS 버튼으로 재연결을 요청했다. 이 기록 시점에서는 복구 확인 전이다.

이 조사에서 이동 명령, 비상정지 해제, 파라미터 변경, 서비스 재시작을 수행하지 않았다. Bluetooth가 끊어진 원인은 확정하지 않았다.

## 복구 확인

사용자가 PS 버튼으로 재연결한 후 실제 수동 주행이 정상 동작한다고 확인했다. 로봇 Bluetooth 조회도 `Connected: yes`, DS4 서비스 로그도 14:51:37 Connected를 기록했다. Nav2/bridge 파라미터나 비상정지 설정 변경 없이 복구됐다. 따라서 이번 장애는 Nav2의 `enable_motion=false`에 의한 수동 주행 차단이 아니라 컨트롤러 연결 해제에 따른 입력 단절이었다.
