# ADR-0005: Base·Upper MPU6050 기반 ALT 영점 정렬 및 EL 추적 제어

- **상태:** Accepted
- **프로젝트:** StarTracker
- **기록 정리일:** 2026-11-14
- **주의:** ADR 상태는 의사결정 상태이며 구현/실물 검증 완료를 뜻하지 않는다.

## 배경 / Context

스텝 기반 ALT 위치추정 전 시스템의 0° 물리 기준을 정의해야 하며 센서 배치 편차와 모터 방향 오류를 다뤄야 한다.

## 검토한 대안 / Alternatives

- 수동 영점 지정: 구현은 쉽지만 시작 때마다 사람의 개입과 재현성 문제가 있다.
- 스텝 카운터만으로 영점 추정: 기계적 실제 위치를 확인할 수 없다.
- 고정 Base와 이동 Upper의 MPU6050 pitch 차이로 영점 정렬: 2개 센서·I²C와 초기화/안전 검증이 필요하다.

## 결정 / Decision

Base(0x68)·Upper(0x69) IMU의 pitch 차이를 사용해 ALT축 초기 정렬을 수행한다. 11개 샘플 중앙값, ±0.2° 이내 3회 연속 시 ALT=0°로 확정한다. 이 과정에 AZ축을 구동하지 않으며 보정 실패 시 추적을 시작하지 않는다. 현재 EL 제어는 IMU 추적 모드를 기본으로 사용하고 PID 튜닝을 진행한다.

## 선택 근거 / Rationale

- 서로 다른 프레임을 측정해 실제 상대 고도각 기준을 얻는다.
- 240스텝 probe로 ALT 구동 방향을 판별하고 실패 조건을 별도로 정의한다.

## 영향과 트레이드오프 / Consequences

- 2026-08-03 문서 기준 소프트웨어 31개 테스트 통과; 2026-10-31 EL PID 튜닝 로그가 있으나 다양한 실기 조건에서 일반 성능을 입증한 것은 아니다.
- 실물 센서 오프셋, 배선, 방향, 양방향 수렴, 최종 촬영 품질은 하드웨어 수락 체크리스트에 따라 확인해야 한다.
- 물리 리미트 스위치가 없어 최초 시험은 하중 제거·저속·비상 차단 준비가 필요하다.

## 근거 / Evidence

- 현재 기본 README: https://github.com/choejunhyeok2005/StarTracker/blob/main/README.md
- 검증 기록: https://github.com/choejunhyeok2005/StarTracker/blob/main/VERIFICATION_REPORT.md
- 실물 수락 체크리스트: https://github.com/choejunhyeok2005/StarTracker/blob/main/HARDWARE_ACCEPTANCE_CHECKLIST.md

## 후속 검토

- 구현 현황, 측정 자료와 팀 승인 여부를 구분해 갱신한다.
- 설계가 변경되면 과거 내용을 지우지 않고 새로운 ADR에서 변경 사유 및 이전 번호를 연결한다.
