# StarTracker

**현재 기본 제어: MPU6050 가속도계·자이로 칼만 필터 → EL 폐루프 제어**.
설정·실행·제약은 [IMU_CONTROL.md](IMU_CONTROL.md)를 참고하세요.
`python simulate_imu_control.py` → `data/kalman_demo/report.html`에서 PC 시뮬레이션을 확인합니다.
AZ는 기존 스텝 추정과 Plate Solving 보정을 유지합니다.

이전 엔코더 선택형 확장: [CONTROL_PROJECT.md](CONTROL_PROJECT.md).
하드웨어 없이 `python simulate_control.py`를 실행하면
`data/control_demo/report.html`에 목표각/실제각/오차 그래프가 생성됩니다.
엔코더 모델이 미정이므로 `ENCODER_ENABLED=False`가 기본값입니다.

현재 기본 촬영 흐름은 [PIPELINED_CAPTURE.md](PIPELINED_CAPTURE.md)를 참고하세요.
최초 목표 plate solving 후 즉시 연속 장노출을 시작하고, 이전 완료 사진으로
다음 촬영 중 병렬 보정합니다. 보정 15회 후 18장 촬영 흐름은
`PIPELINED_CAPTURE_ENABLED=False`인 기존 모드 설명입니다.

Raspberry Pi 5, HQ Camera, GNSS, BLE 조이스틱, ALT/AZ 스테퍼 모터와 두 개의
MPU6050을 사용하는 최종 제어 코드입니다. 전원을 켜면 고정 베이스와 움직이는
상단판의 pitch를 먼저 맞추고, 성공한 경우에만 카메라·BLE·천체 추적 단계로
넘어갑니다.

## 확정된 하드웨어

- Raspberry Pi 5
- Raspberry Pi HQ Camera + 16 mm 렌즈
- 17HM19-2004S 0.9° 스테퍼 모터 2개
- TMC2209, 1/32 microstep
- 최종 감속비 27:1
- GY-NEO6MV2 GNSS
- ESP32-C3 BLE 조이스틱
- MPU6050 2개

### MPU6050 배선

두 센서의 SDA/SCL/GND는 같은 I²C 버스를 공유합니다.

| 센서 | 장착 위치 | 주소 | AD0 |
| --- | --- | --- | --- |
| Base IMU | 움직이지 않는 하단 베이스 | `0x68` | LOW 또는 GND |
| Upper IMU | ALT 모터로 움직이는 상단판 | `0x69` | HIGH 또는 3.3 V |

두 센서는 같은 축 방향으로 단단히 고정해야 합니다. Base IMU는 측정만 하며,
초기 보정 중에는 Upper IMU가 달린 ALT축만 움직입니다. AZ축은 움직이지 않습니다.

## Raspberry Pi 준비

I²C, UART와 카메라를 활성화한 후 다음 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-picamera2 \
  python3-rpi-lgpio \
  astrometry.net \
  i2c-tools

python3 -m pip install -r requirements.txt
```

두 IMU 주소를 먼저 확인합니다.

```bash
i2cdetect -y 1
```

표에 `68`과 `69`가 모두 보여야 합니다. 하나만 보이면 프로그램을 실행하지 말고
AD0 배선과 전원을 확인하십시오.

Astrometry.net 인덱스 파일은 실제 시야각에 맞게 별도 설치해야 합니다.
현재 `config.py`는 HQ Camera 전체 센서와 16 mm 렌즈의 약 22° 가로 시야각을
기준으로 `PLATE_SCALE_LOW=15`, `PLATE_SCALE_HIGH=30`을 사용합니다.

## 실행

프로젝트 루트에서 실행합니다.

```bash
python3 main.py
```

개발 PC에서 하드웨어 없는 자동 테스트를 실행하려면 다음 명령을 사용합니다.

```bash
python -m unittest discover -s tests -v
```

## 초기화와 추적 시작

1. 콘솔 파일 로그와 GNSS 수신을 시작합니다. 나침반 초기화 및 설정에 따른 회전 보정을 수행합니다. 나침반 보정 실패 시에도 다음 단계로 진행합니다.
2. 모터 드라이버를 초기화하고 시스템 상태를 `ALIGN`으로 전환합니다.
3. `0x68`, `0x69` MPU6050의 `WHO_AM_I`와 I²C 응답을 확인합니다.
4. 각 센서에서 11개 가속도 샘플을 읽고 median pitch를 계산합니다.
5. `error = corrected upper pitch - corrected base pitch`를 계산합니다.
6. 보정이 필요하면 ALT `direction=True`로 240스텝 시험 이동해 실제 회전
   방향을 자동 판별합니다.
7. 판별한 ALT 전기 방향을 저장한 뒤, 오차에 비례한 스텝을 계산해 상단 ALT
   모터만 이동합니다. 이 방향 매핑은 이후 수동 조작과 자동 추적에도 유지됩니다.
8. pitch 차이가 ±0.2° 이내로 3회 연속 측정되면 그 물리 위치를 ALT 0°로
   확정합니다.
9. IMU 버스를 닫고 카메라를 초기화합니다.
10. GNSS 위치·UTC 또는 아래의 대체 설정으로 관측자 좌표를 등록하고 `MANUAL`로 전환합니다.
11. BLE 조이스틱으로 촬영 구도를 수동 조정합니다.
12. 버튼을 누르면 사진을 plate solving하여 화면 중심의 RA/Dec를 목표로
    저장합니다.
13. 20 Hz를 목표로 하는 tracking loop에서 현재 UTC의 목표 Alt/Az를 계산하고 두 모터를
    구동합니다.
14. 기본 모드에서는 즉시 110초 노출 17장 촬영을 시작합니다. 두 번째 촬영부터
    이전 완료 사진을 별도 스레드에서 plate solving하여 현재 모터 위치를 보정합니다.
    목표 RA/Dec는 유지하며 추적 루프가 보정 이동을 수행합니다.
15. 마지막 촬영 직전에는 진행 중인 분석을 기다립니다. 마지막 노출 중에는 추가 분석을
    하지 않지만 목표를 따라가는 모터 추적은 계속합니다.
16. 사진과 로그를 저장하고 모터를 정지합니다. 정상 촬영 완료 후 기본 설정
    `EXIT_AFTER_CAPTURE=True`에 따라 프로그램을 종료합니다.

버튼 클릭으로 세션을 시작하면 미리보기와 수동 이동을 중지합니다. 촬영 중 다시
클릭하면 중단을 요청하며, 진행 중인 노출·파일 저장 후 다음 프레임을 생략합니다.
BLE가 연결되어 있고 종료 중이 아니면 중단 후 미리보기와 수동 조작으로 복귀합니다.
BLE 연결 해제 시에는 시스템을 종료합니다.

### GNSS와 웹 미리보기

- 위치 수신은 최대 `GNSS_FIX_TIMEOUT=300`초, 위치 확보 후 UTC 수신은 최대
  `GNSS_UTC_TIMEOUT=5`초 기다립니다.
- 위치가 없으면 위도 37.5°, 경도 127.0°, 고도 0 m와 시스템 UTC를 사용합니다.
  `GNSS_FALLBACK_*` 값을 실제 관측지로 설정하세요.
- 위치만 수신하면 해당 위치와 시스템 UTC를, 위치와 UTC 모두 수신하면 GNSS 값을 사용합니다.
- 웹 서버는 모든 인터페이스의 8000 포트에서 실행합니다. 같은 네트워크에서
  `http://<라즈베리파이 IP>:8000/`에 접속합니다. 촬영 세션 중에는 미리보기가 중지됩니다.

## IMU 보정 기준과 설정

주요 값은 `config.py`에 있습니다.

- 허용오차: `IMU_LEVEL_TOLERANCE_DEG = 0.2`
- 완료 조건: `IMU_STABLE_READS_REQUIRED = 3`
- 필터 샘플: `IMU_FILTER_SAMPLES = 11`
- 전체 제한시간: `IMU_ALIGNMENT_TIMEOUT_SECONDS = 60.0`
- 최대 반복: `IMU_MAX_ALIGNMENT_ITERATIONS = 80`
- 시작점 기준 최대 상대 이동: `IMU_MAX_RELATIVE_TRAVEL_DEG = 30.0`

초기 오차가 이미 허용 범위 안이면 불필요한 probe 이동을 하지 않습니다. 이 경우
자동 방향 판별도 생략되므로 `MOTOR_ALT_POSITIVE_DIRECTION`은 실제 배선에 맞는
값이어야 합니다. 보정 probe가 실행되면 이 값은 판별 결과로 런타임에 갱신됩니다.

센서 장착 편차가 있으면 두 판을 물리적으로 평행하게 고정한 상태에서 각 센서의
pitch 차이를 측정해 다음 값으로 보정합니다.

```python
IMU_BASE_PITCH_OFFSET_DEG = 0.0
IMU_UPPER_PITCH_OFFSET_DEG = 0.0
```

계산식은 `측정 pitch - 해당 센서 offset`입니다. 두 센서 축 방향이 반대로
장착된 문제는 offset으로 해결할 수 없으므로 장착 방향부터 맞춰야 합니다.

## 보정 실패 조건

다음 중 하나라도 발생하면 ALT 0°를 확정하지 않고 프로그램을 종료합니다.

- 센서 주소 또는 `WHO_AM_I` 오류
- I²C 읽기 오류, 0 벡터 또는 유효하지 않은 측정값
- 시험 이동 후 pitch 변화가 0.08° 미만
- 보정 후 오차가 0.1° 이상 연속 3회 증가
- 60초 또는 80회 반복 초과
- 시작 위치 기준 ±30° 상대 이동 범위 초과

`IMU_REQUIRED=True`가 실제 운용 기본값입니다. `False`는 모터를 분리한 벤치
검사용이며, 자동 보정 실패 상태에서 실제 장비를 운용하는 용도가 아닙니다.

## 수동 ALT 이동 범위

IMU 정렬 직후 소프트웨어 ALT는 0°입니다. 추적용 `MIN_ALTITUDE=0.0`을 수동
조작에도 적용하면 하강 방향이 잠기는 문제가 있으므로 수동 범위를 분리했습니다.

```python
MANUAL_ALT_SOFT_LIMITS_ENABLED = False
MANUAL_MIN_ALTITUDE = -30.0
MANUAL_MAX_ALTITUDE = 90.0
```

기본값은 수동 소프트 제한 해제입니다. 제한을 사용하려면 첫 값을 `True`로 바꾸고
실제 기구가 안전하게 움직일 수 있는 최소·최대 각도를 현장에서 확인해 설정하십시오.
자동 추적에는 기존 `MIN_ALTITUDE`와 `MAX_ALTITUDE`가 계속 적용됩니다.

## 촬영 모드와 저장 파일

### 기본: 촬영과 보정을 병렬 실행

```python
PIPELINED_CAPTURE_ENABLED = True
PIPELINED_EXPOSURE_SECONDS = 110.0
PIPELINED_FRAME_COUNT = 17
PIPELINED_SKIP_FINAL_CORRECTION = True
EXIT_AFTER_CAPTURE = True
```

설정상 총 노출 시간은 1,870초(31분 10초)이며 초기 목표 촬영·분석, 파일 저장,
분석 대기 시간은 별도입니다. 실제 노출 시간은 카메라 메타데이터로 확인해야 합니다.
사진은 `data/images/final_sequence/final_<시각>_<번호>.jpg`에 개별 저장되며
자동 합성하지 않습니다. 최초 목표 사진도 `data/images/plate_<시각>.jpg`로 보존합니다.

기본 설정에서는 최대 15번의 병렬 분석 기회가 있습니다. 분석이 노출보다 오래 걸리면
오래된 프레임을 대기열에 쌓지 않고 다음 기회에 최신 완료 사진을 분석합니다.
완료 조건은 성공 보정 횟수가 아닌 저장 장수이며, 연속 보정 실패 3회면 중단합니다.
`CORRECTION_INTERVAL`과 `MAX_CORRECTION_COUNT`는 기본 모드의 촬영 시작 조건이 아닙니다.

### 기존: 주기 보정 후 촬영

`PIPELINED_CAPTURE_ENABLED=False`이면 120초 간격으로 별도 보정 사진을 촬영하고,
성공 보정 15회 후 아래 설정으로 최종 촬영합니다.

```python
FINAL_CAPTURE_MODE = "sequence"
FINAL_SUBEXPOSURE_SECONDS = 10.0
FINAL_FRAME_COUNT = 18
FINAL_FRAME_INTERVAL_SECONDS = 0.0
FINAL_DRIFT_CORRECTION_EVERY_FRAMES = 3
```

총 노출 시간은 180초이며 JPG 18장을 개별 저장합니다. 3장마다 다음 프레임 전에
별도 plate-solving 사진으로 보정하며, 보정 사진은 18장에 포함하지 않습니다.
`FINAL_CAPTURE_MODE="single"`이면 `LONG_EXPOSURE_SECONDS=180.0` 단일 노출을 사용합니다.

### 시야 회전과 현장 확인

ALT/AZ 2축 구조는 목표 중심을 따라가도 시야 회전을 제거하지 못합니다.
노출 중 모터 보정도 별 흔들림을 만들 수 있으므로 110초 노출의 별 모양과
plate solving 성공률은 현장에서 확인해야 합니다. 별 정렬·시야 회전 보정·스태킹은
후처리 프로그램에서 수행합니다.

## EL PID 튜닝 모드

`python tune_el.py`로 목표각 유지·시작각 복귀 실험을 시뮬레이션하고 CSV/HTML 그래프를 저장합니다.
실물 실행은 `--hardware`와 실제 초기각 `--start-alt`가 필요합니다.
P/I/D 지정 방법과 실물 준비 절차는 [EL_TUNING.md](EL_TUNING.md)를 확인하세요.

## 세션 로그

각 세션은 `data/logs`에 다음 파일을 저장합니다.

- `tracking_*.json`: IMU 보정 결과, 관측자, 목표, 보정 오차, 상태 이벤트,
  최종 이미지 경로
- `platesolving_*.json`: 최초 목표와 성공한 보정의 plate-solving 결과
- `console_*.log`: 프로그램 실행 중 stdout과 stderr에 출력된 전체 콘솔 로그

## 안전 주의사항

- 현재 장비에는 물리 리미트 스위치가 없습니다. 전원을 켤 때 상단판을 기계적
  끝점에서 충분히 떨어뜨려 놓아야 합니다.
- 최초 실기 시험은 벨트 또는 카메라 하중을 제거하고 저속으로 수행하십시오.
- 자동 추적의 ALT 0~90° 제한은 IMU 보정이 성공한 위치를 기준으로 합니다.
- 기본 설정에서는 수동 ALT 소프트 제한이 꺼져 있습니다. 물리 리미트 스위치가
  없으므로 첫 시험에서는 기계적 끝점에 닿기 전에 조이스틱을 놓아야 합니다.
- BLE가 끊기거나 추적 오류가 발생하면 모터 추적을 중단합니다.
- 버튼은 TARGET_CAPTURE, TRACKING, DRIFT_CORRECTION, CAPTURE 중에도 중단
요청으로 동작합니다. 진행 중인 카메라 노출은 끝난 후 다음 프레임부터 취소됩니다.

실기 시험은 [HARDWARE_ACCEPTANCE_CHECKLIST.md](HARDWARE_ACCEPTANCE_CHECKLIST.md)의
순서를 따르십시오.
