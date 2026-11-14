# ADR-0004: 엔코더 기반 2축 위치 폐루프 추적 모드

- **상태:** Proposed
- **프로젝트:** StarTracker
- **기록 정리일:** 2026-11-14
- **주의:** ADR 상태는 의사결정 상태이며 구현/실물 검증 완료를 뜻하지 않는다.

## 배경 / Context

스텝 명령 누적만으로 실제 ALT/AZ를 추정하면 스텝 손실·백래시와 외란을 정확히 알 수 없다. 실제 출력축 위치를 관측하는 대안이 필요하다.

## 검토한 대안 / Alternatives

- 스텝 명령 누적 추정: 별도 센서가 없지만 실제 이동 오차를 직접 감지하지 못한다.
- IMU·Plate Solving 보정 중심: 현재 센서 및 영상 자산을 이용하지만 빠른 2축 연속 계측과는 다르다.
- 엔코더를 통한 2축 각도 피드백·PID: 센서/배선/주기 및 실물 검증 비용이 추가된다.

## 결정 / Decision

[선택형 시제품, 기본 모드 아님] 엔코더 측정각을 제어 피드백으로 사용하고, 위치 PID→속도·가속도·스텝 제한→TMC2209로 연결하는 소프트웨어 모드를 실험한다. ENCODER_ENABLED 기본값은 False이며 IMU 추적 모드가 현재 기본이다.

## 선택 근거 / Rationale

- 보낸 스텝을 곧 실제 위치로 간주하지 않도록 하여 오차 관측을 분리한다.
- AZ 0°/360° 경계, 센서 미응답, stale 데이터와 제어 포화를 명시적으로 처리한다.

## 영향과 트레이드오프 / Consequences

- 2026-10-12 문서는 자동 테스트 77개 통과, 60초 시뮬레이션 마지막 ALT 약 0.00922°/AZ 약 0.00969° 오차를 기록했다. 이는 실물 정확도 증명이 아니다.
- 실제 엔코더 모델·전압·출력축 해상도·방향·기어 백래시·GPIO 지터·하드웨어 안정성은 미검증이다.
- 장치 검증과 팀 합의 후에만 기본 운용 모드로 승격할 수 있다.

## 근거 / Evidence

- 선택형 구현: https://github.com/choejunhyeok2005/StarTracker/blob/main/StarTracker_IMU_Final/StarTracker_EL_Tuning_v1/CONTROL_PROJECT.md
- 소프트웨어 검증: https://github.com/choejunhyeok2005/StarTracker/blob/main/StarTracker_IMU_Final/StarTracker_EL_Tuning_v1/CONTROL_VERIFICATION.md

## 후속 검토

- 구현 현황, 측정 자료와 팀 승인 여부를 구분해 갱신한다.
- 설계가 변경되면 과거 내용을 지우지 않고 새로운 ADR에서 변경 사유 및 이전 번호를 연결한다.
