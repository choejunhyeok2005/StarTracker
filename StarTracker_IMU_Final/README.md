# StarTracker IMU Final

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

## 최종 전체 알고리즘

1. GNSS reader thread를 시작해 위치와 UTC 수신을 병행합니다.
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
10. GNSS 위치와 UTC가 유효해지면 관측자 좌표를 등록하고 `MANUAL`로 전환합니다.
11. BLE 조이스틱으로 촬영 구도를 수동 조정합니다.
12. 버튼을 누르면 사진을 plate solving하여 화면 중심의 RA/Dec를 목표로
    저장합니다.
13. 20 Hz tracking loop에서 현재 GNSS UTC의 목표 Alt/Az를 계산하고 두 모터를
    구동합니다.
14. 120초마다 새 사진을 plate solving하여 실제 카메라 방향으로 소프트웨어
    위치를 동기화합니다. 연속 실패 3회면 세션을 종료합니다.
15. 성공한 보정 15회를 완료하면 tracking을 유지한 채 10초 프레임 18장을
    촬영합니다.
16. 이미지 경로, IMU 보정값, plate solving 결과와 drift 수치를 JSON으로
    저장하고 `MANUAL`로 복귀합니다.

상태 전이는 다음과 같습니다.

```text
INIT -> ALIGN -> MANUAL -> TARGET_CAPTURE -> TRACKING
                                      TRACKING <-> DRIFT_CORRECTION
                                      TRACKING -> CAPTURE -> MANUAL
```

## IMU 보정 기준과 설정

주요 값은 `config.py`에 있습니다.

- 허용오차: `IMU_LEVEL_TOLERANCE_DEG = 0.2`
- 완료 조건: `IMU_STABLE_READS_REQUIRED = 3`
- 필터 샘플: `IMU_FILTER_SAMPLES = 11`
- 전체 제한시간: `IMU_ALIGNMENT_TIMEOUT_SECONDS = 60.0`
- 최대 반복: `IMU_MAX_ALIGNMENT_ITERATIONS = 80`
- 시작점 기준 최대 상대 이동: `IMU_MAX_RELATIVE_TRAVEL_DEG = 15.0`

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
- 시작 위치 기준 ±15° 상대 이동 범위 초과

`IMU_REQUIRED=True`가 실제 운용 기본값입니다. `False`는 모터를 분리한 벤치
검사용이며, 자동 보정 실패 상태에서 실제 장비를 운용하는 용도가 아닙니다.

## 최종 촬영과 시야 회전

ALT/AZ 2축 구조는 목표 중심을 따라가도 장노출 중 화면 회전을 제거하지 못합니다.
따라서 기본값은 다음과 같습니다.

```python
FINAL_CAPTURE_MODE = "sequence"
FINAL_SUBEXPOSURE_SECONDS = 10.0
FINAL_FRAME_COUNT = 18
```

총 적분시간은 약 180초이며, 결과 프레임은 후처리 프로그램에서 정렬·회전 보정·
스태킹해야 합니다. `FINAL_CAPTURE_MODE="single"`과 180초 단일 노출은 실험
모드로 남아 있지만 화면 외곽 별의 선명도를 보장하지 않습니다.

## 로그

각 세션은 `data/logs`에 두 JSON 파일을 저장합니다.

- `tracking_*.json`: IMU 보정 결과, 관측자, 목표, 보정 오차, 상태 이벤트,
  최종 이미지 경로
- `platesolving_*.json`: 최초 목표와 모든 drift-correction solve 결과

## 안전 주의사항

- 현재 장비에는 물리 리미트 스위치가 없습니다. 전원을 켤 때 상단판을 기계적
  끝점에서 충분히 떨어뜨려 놓아야 합니다.
- 최초 실기 시험은 벨트 또는 카메라 하중을 제거하고 저속으로 수행하십시오.
- 프로그램의 ALT 0~90° 제한은 IMU 보정이 성공한 위치를 기준으로 합니다.
- BLE가 끊기거나 추적 오류가 발생하면 모터 추적을 중단합니다.
- 버튼은 TARGET_CAPTURE, TRACKING, DRIFT_CORRECTION, CAPTURE 중에도 중단
요청으로 동작합니다. 진행 중인 카메라 노출은 끝난 후 다음 프레임부터 취소됩니다.

실기 시험은 [HARDWARE_ACCEPTANCE_CHECKLIST.md](HARDWARE_ACCEPTANCE_CHECKLIST.md)의
순서를 따르십시오.
