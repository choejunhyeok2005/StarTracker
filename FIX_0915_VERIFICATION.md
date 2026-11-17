# 2026-09-15 검증 결과

- Python unittest: 40개 통과, 실패/오류 0.
- Python 소스 AST 문법 검사: 28개 파일 통과.
- 검증 환경: Windows / Python 3.14, GPIO·카메라·BLE·GNSS 대역 사용.
- 새 회귀 테스트: 실제 BLE 클릭 핸들러의 시작 요청/중지 분기,
  BLE 종료 이벤트 미설정, 위치만 수신한 경우 GNSS 준비 거절,
  위치+UTC 준비, fix_quality=0 거절, 세션 시작/중복 요청 차단,
  작업 스레드 종료 대기, 연결 해제, 시작 예외, 종료 중 시작 작업 정리.
- 기존 IMU, 모터 방향, 촬영 시퀀스, 추적 종료, JSON 기록 테스트 포함.
- Raspberry Pi 실기기, 실제 solve-field 및 별 사진, 실제 노출 시간,
  연속 15회 보정과 18장 촬영의 현장 검증은 미실시.

실행 명령:

```bash
python -m unittest discover -s tests -v
```

원본의 .git 파일은 다른 컴퓨터의 worktree를 참조하므로 복사본에서 제외했습니다.
원본 프로젝트 파일은 수정하지 않았습니다.
