# StarTracker 엔코더 폐루프 제어 확장

> 현재 기본 모드는 [IMU EL 제어](IMU_CONTROL.md)입니다. 이 문서는 이전 엔코더
> 선택형 구현을 설명합니다. 엔코더 모드를 선택하려면 `IMU_TRACKING_ENABLED=False`와
> `ENCODER_ENABLED=True`를 함께 설정해야 합니다.

**Encoder 및 영상 피드백 기반 2축 StarTracker 정밀 위치제어 시스템**의
1차 소프트웨어 구현입니다. 기존 IMU 초기 정렬·천체 좌표 변환·Plate Solving에
엔코더 피드백을 연결했습니다. ALT는 EL과 같은 고도축입니다.
엔코더 모델은 미정이며 기본값은 `ENCODER_ENABLED=False`입니다.

## PC에서 실행

프로젝트 루트에서 Python 3.10 이상으로 실행합니다. 데모는 표준 라이브러리만
사용하며 GPIO·카메라·천문 패키지를 불러오지 않습니다.

```console
python simulate_control.py
python -m unittest discover -s tests -v
```

- `data/control_demo/control.csv`: 목표각, 실제각, 오차, 속도 명령, 스텝 명령
- `data/control_demo/report.html`: 브라우저에서 여는 오프라인 그래프 (CDN 없음)
- `data/control_demo/summary.json`: 시뮬레이션 조건과 마지막 오차

60초를 실제 대기 없이 계산합니다. 15% 부족한 모터 응답, AZ 359.8°→0.2°
경계 통과, 천천히 변하는 목표, 20초 시점의 ALT −0.15° / AZ +0.12° 외란을
넣고 40초 이후 목표를 고정합니다. **실물 정확도나 분해능 측정이 아닙니다.**
기어 탄성·실제 백래시·GPIO 지터·센서 잡음은 이 단순 모델에 없습니다.

실물 추적 CSV도 같은 도구로 그립니다.

```console
python -m services.control_telemetry data/logs/control_실행시각.csv data/logs/report.html
```

## 제어 경로와 구현 범위

```text
RA/Dec → 기존 Astronomy → 시간에 따른 목표 ALT/AZ
                              ↓
              위치 PID + 속도/가속도/스텝 수 제한
                              ↓
               MotorDriver → TMC2209 → 실제 축
                              ↑           ↓
                        실제각 ← Encoder

IMU 초기 정렬 → ALT 기준 설정
Plate Solving → 엔코더 좌표와 하늘 좌표 사이 오프셋 보정
```

`MotorController.update()`가 기존 방식 또는 엔코더 PID를 선택합니다.
엔코더 모드에서는 보낸 스텝 수를 실제 이동으로 누적하지 않습니다.
PID 출력은 °/s이며 실제 주기와 스텝각으로 변환합니다. 한 스텝보다 작은
명령은 다음 주기로 이월합니다. 기존 50ms 주기·최대 5스텝 제한을 유지합니다.
Python 소프트 실시간 구현이며 별도 시간 최적 궤적 생성기는 아닙니다.

축별 계수·데드밴드·출력 포화·적분 누적 제한·측정값 미분을 지원합니다.
초기값은 **P 제어** (`Ki=Kd=0`)이며 실물 튜닝값이 아닙니다.
AZ 오차와 미분은 0/360°를 고려합니다. 데드밴드 진입 시 명령을 0으로
만들므로 가속도 제한은 이동 명령 구간에 적용됩니다.

Plate Solving 보정은 카운터를 덮어쓰지 않고 좌표 오프셋을 바꾸어
다음 센서 읽기에서도 유지됩니다. 기존 IMU는 초기 ALT 정렬에 사용합니다.
**연속 IMU/Compass 센서 융합과 다점 마운트 모델은 이번 범위에 없습니다.**

## 엔코더 선정 후 설정

최종 출력축을 측정하면 기어 이후 움직임을 관측할 수 있습니다. 모터축
엔코더도 환산은 가능하지만 하류 백래시는 직접 관측하지 못합니다.
제공된 입력 모듈은 **A/B상 증분형 x4 카운터**입니다. 실제 센서를 가정하지
않도록 핀·해상도는 미설정입니다. `config.py`에서 다음을 설정합니다.

| 설정 | 의미 |
| --- | --- |
| `ENCODER_ENABLED` | 배선·방향·영점 검증 후 True |
| `ENCODER_AXES[축]['pins']` | A/B BCM 핀 번호 |
| `counts_per_rev` | 최종 출력축 1회전당 x4 디코딩 카운트 |
| `sign` | 각도 증가 방향에 따라 +1 또는 −1 |
| `POSITION_PID` | 축별 Kp/Ki/Kd, 최대 속도/가속도, 허용오차 |
| `ENCODER_STALL_*` | 최소 관측 이동량과 무응답 판단 조건 |

제조사 PPR/CPR 표기의 x1/x4를 구분하고, 모터축 장착이면 기어비를 반영하세요.
마이크로스텝 수를 엔코더 해상도로 사용하면 안 됩니다. 기본 허용오차 0.01°와
무응답 최소 이동량 0.005°는 센서의 출력축 분해능·잡음에 맞게 바꾸세요.

신호 전압·출력 방식·레벨 변환·풀업은 선정 센서의 데이터시트에 따릅니다.
코드는 풀업을 임의로 켜지 않습니다. 모터/I²C/UART 예약 핀 충돌은 검사하지만
전체 확장장치 배선을 자동으로 알 수는 없습니다. Pi 5 GPIO 백엔드와 엣지
콜백 성능은 실물 검증이 필요합니다. 높은 빈도에서는 외부 카운터/MCU를 검토하세요.

SPI/I²C 절대형은 `EncoderSample(alt, az, timestamp)`를 반환하는 `read()`와
`close()`를 구현하여 `MotorController(encoder=센서객체)`에 연결할 수 있습니다.
각도 단위는 출력축 °, timestamp는 **측정 시점의 `time.monotonic()`**입니다.
ALT는 연속적인 부호 있는 각도여야 합니다. 캐시값에 새 timestamp를 붙이면
오래된 데이터 검사를 우회하게 됩니다. 실제 장치 팩토리 `configured_encoder()`도 바꾸세요.

## 기준 설정과 실패 처리

증분형은 재시작 후 절대각을 모릅니다. IMU 정렬 완료에서 ALT=0 기준을 잡고,
첫 Plate Solving에서 하늘 좌표를 맞춥니다. 기존 IMU 비활성 벤치 모드에서는
사용자가 물리적 ALT 영점을 맞춰야 합니다. 현재 보정은 단순 좌표 오프셋이며
축 비직교나 설치 기울기의 다점 보정은 아닙니다.

읽기 오류, NaN/Inf, 오래되거나 미래인 timestamp, 불법 A/B 전이, 기준 미설정,
추적 ALT 범위 이탈, 제어 간격 초과, 일정 명령량 후 축 무응답은 추적을
정지시키고 오류를 유지합니다. 오류 후 `resume()`를 거부하므로 원인을 점검하고
프로그램을 재시작하여 기준을 다시 잡아야 합니다. 스텝 추정값으로 대체하지 않습니다.

증분형 정지 카운터만으로 정지와 단선을 즉시 구분할 수 없습니다.
무응답 검사는 이동 명령량과 시간 조건이 만족된 뒤에 작동합니다.
수동 모드는 PID 위치 유지가 아닌 측정각 갱신·짧은 이동 제한이고,
ALT 수동 범위는 `MANUAL_MIN/MAX_ALTITUDE`입니다. 초기 IMU 정렬은 기존
상대 이동 제한을 사용합니다. 추적 ALT 제한은 보정된 좌표의 소프트 제한으로
하드웨어 리미트 스위치와 동등하지 않습니다. AZ 케이블 감김 제한은 없습니다.

실물 확인 순서:

1. 모터 구동 없이 축을 손으로 움직여 카운트·방향·해상도 확인
2. 기준점에서 영점 확인
3. 작은 저속 명령으로 모터/엔코더 방향 일치 확인
4. P 게인부터 조절하고 목표각 계단응답·반전·외란 복귀 CSV 수집
5. 센서 실패 정지와 촬영 중 보정 확인 후 긴 추적으로 확장

## 파일 안내

| 파일 | 역할 |
| --- | --- |
| `controllers/position_pid.py` | 축별 PID |
| `controllers/encoder_feedback.py` | 기준·피드백·무응답 검사 |
| `controllers/motor_controller.py` | 기존 추적·수동·정렬과 연결 |
| `hardware/encoder.py` | 센서 인터페이스·A/B상 입력 |
| `services/control_telemetry.py` | CSV·HTML 그래프 |
| `simulate_control.py` | 하드웨어 없는 데모 |
| `tests/test_encoder_control.py` | 폐루프 및 실패 조건 검사 |

기존 checksum/verification 문서는 업로드 당시 기록입니다.
이번 검증은 `CONTROL_VERIFICATION.md`를 참고하세요.
