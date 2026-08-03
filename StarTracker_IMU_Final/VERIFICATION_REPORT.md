# StarTracker IMU Final Verification Report

검증일: 2026-08-03

## 확정된 동작

- 고정 Base MPU6050(`0x68`)과 이동 Upper MPU6050(`0x69`)을 사용한다.
- 각 pitch는 11개 가속도 샘플의 중앙값으로 계산한다.
- Upper와 Base의 보정된 pitch 차이를 `±0.2°` 안에 3회 연속 유지해야
  성공한다.
- 초기 보정 중에는 Upper가 연결된 ALT 모터만 움직이고 AZ는 움직이지 않는다.
- 240스텝 probe로 ALT DIR 극성을 판별하고, 판별값을 보정 이후의 수동 조작과
  자동 추적에도 유지한다.
- timeout, 반복 수, 상대 이동량, 관측 불가능한 probe, 연속 오차 증가 조건에서
  ALT 0°를 확정하지 않고 중단한다.
- 성공한 물리 위치만 ALT 0°로 확정한다.
- 기본 최종 촬영은 ALT/AZ 마운트의 시야 회전을 고려한 18×10초 시퀀스다.

## 자동 검증 결과

- `python -B -m unittest discover -s tests`: 31 tests, 0 failures
- 전체 Python AST 파싱: 22 files, 0 syntax errors
- 임시 작업 표식, 구형 `motorCtrl`, 구형 `smbus` import 검색: 0 unresolved
- 핵심 설정 불변식: tolerance 0.2°, stable reads 3, median samples 11,
  travel 15°, final capture 18×10초
- 다운로드 원본 비교: baseline 15 files의 SHA-256이 모두 일치

## 장비에서 남은 검증

개발 PC에는 Raspberry Pi GPIO, 실제 MPU6050 두 개, 모터, 카메라가 연결되어
있지 않으므로 물리 방향, 센서 장착 offset, 실제 수렴 정밀도와 별상 품질은 자동
검증 범위에 포함되지 않는다. Raspberry Pi에서는
`HARDWARE_ACCEPTANCE_CHECKLIST.md`를 위에서 아래 순서로 완료해야 한다.
