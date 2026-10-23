# 2026-09-19 추가 수정 검증

## 반영 내용

- IMU 정렬 직후 ALT 0°에서 수동 하강이 차단되던 추적용 소프트 제한 분리
- 수동 ALT 제한 사용 여부와 범위를 `config.py`에서 제어
- 장노출 시간, 횟수, 프레임 사이 대기시간을 `config.py`에서 제어
- 장노출 시퀀스 도중 설정된 프레임 수마다 plate-solving drift correction 실행
- stdout과 stderr 전체를 터미널과 `data/logs/console_*.log`에 동시 저장
- 최종 결과가 합성본이 아닌 개별 장노출 프레임 목록임을 문서화

## 자동 검증

Windows의 번들 Python으로 다음 명령을 실행했습니다.

```text
python -m compileall -q .
python -m unittest discover -s tests -v
```

결과:

- Python 문법 검사 성공
- 자동 테스트 47개 실행
- 실패 0, 오류 0

## 하드웨어 검증 필요 항목

자동 테스트는 실제 Raspberry Pi GPIO, TMC2209, 카메라와 astrometry.net을
구동하지 않습니다. 현장에서는 다음을 확인해야 합니다.

1. 수동 ALT 양방향 이동과 기계적 끝점 사이의 안전거리
2. `MANUAL_ALT_SOFT_LIMITS_ENABLED=True` 사용 시 실제 안전 각도
3. 장노출 3장마다 plate-solving 보정 사진이 촬영되는지
4. 보정 뒤 모터가 20 Hz 추적 루프의 제한 스텝으로 나누어 이동하는지
5. `data/logs/console_*.log`에 시작부터 종료까지 출력이 남는지
6. 최종 개별 JPG 수가 `FINAL_FRAME_COUNT`와 일치하는지

기본값 10초 × 18장은 노출 적분시간만 180초입니다. 카메라 재설정·안정화,
프레임 간 대기시간, 보정용 5초 촬영과 solve-field 실행 시간이 추가되므로 실제
시퀀스 경과시간은 180초보다 깁니다.
