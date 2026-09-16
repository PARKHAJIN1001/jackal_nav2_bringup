# FAST-LIVO2 격리 검증 로그

이 디렉터리 최상위 로그는 최초 `/tmp/fast_livo_followup.dYwWPy/staged`의
오프라인 결과다. 이후 사용자가 원본 패치 적용과 workspace 재빌드를 완료했다.
배포 확인 로그는 `workspace_deployment/`에 별도로 추가했다.
어느 검사에서도 실기 센서 데이터, ROS 노드 실행, 이동 명령은 사용하지 않았다.

- `configure.log`, `build.log`, `install.log`: 시스템 CMake 격리 빌드/install.
  최종 build log는 증분 빌드 기록이다.
- `ctest.log`: 기능 테스트 5/5. 전체 upstream lint 실행 결과는 아니다.
- `launch_contract.log`: node 실행 없는 launch/production wiring 6개 검사.
- `launch_args.log`: 격리 install에서 `--show-args` 실행.
- `nav2_pytest.log`: Nav2 Python 회귀 107개.
- `vio_fixed.log`: 실제 수정 VIO 함수 호출, no-add 1000회 할당 0, add 경로 할당 1.
- `vio_original_negative.log`: 동일 test에 기존 함수 본문을 연결한 대조군.
  no-add 1000회 할당 1000으로 exit 1이 예상 결과다. 실패를 통과로 간주한 것이 아니다.

VIO 계수는 64-float patch 크기의 배열 할당만 대상으로 한다. 프로세스 전체
RSS나 모든 메모리 누수를 측정한 결과가 아니다. 초기 라이브러리 warmup 후
측정했으며, fixed 라이브러리를 우선 로드하도록 지정했다.

자세한 변경/한계/적용 절차: [후속 기록](../../../FAST_LIVO_Followup_2026-09-14.md).
