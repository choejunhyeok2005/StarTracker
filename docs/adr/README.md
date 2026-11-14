# Architecture Decision Records (ADR)

StarTracker의 기술 선택 이유, 대안, 영향 및 검증 상태를 분리해 보관합니다. **Accepted는 구현/실물 정밀도 검증 완료를 뜻하지 않습니다.**

통합 Notion 보드: [ADR | 설계 의사결정 기록](https://app.notion.com/p/4181024c8713812db8f6e2e5ca15be73)

| 번호 | 제목 | 상태 | Notion |
| --- | --- | --- | --- |
| ADR-0004 | [선택형 엔코더 기반 2축 위치 폐루프 제어](0004-optional-encoder-closed-loop-tracking.md) | Proposed | [기록](https://app.notion.com/p/4181024c87138179ba70c626a001148d) |
| ADR-0005 | [Base·Upper MPU6050 ALT 영점 및 EL 제어](0005-dual-imu-altitude-zeroing-and-el-control.md) | Accepted | [기록](https://app.notion.com/p/4181024c8713811594f1c4bda2815681) |

기본 구현은 IMU EL 제어이고 엔코더 모드는 선택형 소프트웨어 시제품입니다. 실물 배선·성능 증거 없이 엔코더 기본 운용 완료로 표기하지 않습니다.
