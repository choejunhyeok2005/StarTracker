# 🌌 StarTracker

### GNSS · Dual IMU · Camera · Plate Solving 기반 2축 자동 천체 추적 시스템

Raspberry Pi 5를 중심으로 **GNSS, Dual MPU6050 IMU, HQ Camera, BLE 조이스틱, TMC2209 스테퍼 모터**를 통합하여 천체의 위치를 계산하고 카메라를 목표 방향으로 정렬·추적하는 2축 Star Tracker를 개발했습니다.

단순한 모터 제어를 넘어 **센서 측정 → 자세 보정 → 위치/시간 계산 → 천체 좌표 계산 → 모터 제어 → 영상 기반 위치 보정 → 촬영**까지 하나의 시스템으로 연결했습니다.

> **Project Goal**
> 센서와 천문 계산, 영상 분석을 결합하여 실제 하드웨어가 목표 천체를 찾아가고 추적할 수 있는 시스템을 구현하는 것

---

## 📸 Project Overview

### 핵심 기능

- 📍 GNSS 기반 위도 / 경도 / 고도 / UTC 획득
- 🧭 Dual MPU6050 기반 ALT축 초기 정렬
- ⚙️ TMC2209 + 17HM19-2004S 스테퍼 모터 제어
- 🎮 ESP32-C3 BLE 조이스틱을 이용한 수동 제어
- 📷 Raspberry Pi HQ Camera를 이용한 천체 이미지 획득
- ⭐ Astrometry.net 기반 Plate Solving
- 🌍 관측 위치와 UTC 기반 천체 위치 계산
- 🔭 ALT/AZ 2축 자동 Tracking
- 🔄 주기적인 Plate Solving 기반 Drift Correction
- 📊 IMU / Tracking / Plate Solving 결과 JSON 로그 저장
- 🛑 BLE 연결 끊김 및 Tracking 오류 발생 시 모터 정지

---

## 🧩 System Architecture

```text
                       ┌─────────────────┐
                       │      GNSS       │
                       │ Latitude        │
                       │ Longitude       │
                       │ Altitude / UTC  │
                       └────────┬────────┘
                                │
                                ▼
┌────────────────┐      ┌─────────────────────┐
│ Dual MPU6050   │─────▶│  Tracking Manager   │
│ Base  0x68     │      │                     │
│ Upper 0x69     │      │ Target Alt / Az     │
└────────────────┘      │ Motor Control       │
                        │ Plate Solving       │
                        │ Drift Correction    │
                        └──────────┬──────────┘
                                   │
                         ┌─────────┴─────────┐
                         ▼                   ▼
                 ┌─────────────┐     ┌─────────────┐
                 │ AZ Motor    │     │ ALT Motor   │
                 │ TMC2209     │     │ TMC2209     │
                 └─────────────┘     └─────────────┘
                         │                   │
                         └─────────┬─────────┘
                                   ▼
                           ┌─────────────┐
                           │ HQ Camera   │
                           └──────┬──────┘
                                  │
                                  ▼
                        ┌───────────────────┐
                        │ Astrometry.net     │
                        │ Plate Solving     │
                        └─────────┬─────────┘
                                  │
                                  ▼
                           Drift Correction

        ┌───────────────────┐
        │ ESP32-C3 BLE      │
        │ Joystick          │
        └─────────┬─────────┘
                  ▼
          Manual AZ / ALT Control
```

---

## 🔧 Hardware

| Component | Specification | Purpose |
|---|---|---|
| Controller | Raspberry Pi 5 | Main controller |
| Camera | Raspberry Pi HQ Camera + 16 mm Lens | 천체 이미지 획득 |
| Motor | 17HM19-2004S ×2, 0.9°/full step | ALT / AZ 구동 |
| Motor Driver | TMC2209, 1/32 microstep | Stepper Motor 제어 |
| Gear | 27:1 | 출력축 감속 |
| GNSS | GY-NEO6MV2 | 위치 및 UTC 획득 |
| IMU | MPU6050 ×2 | Dual IMU 기반 ALT 정렬 |
| Controller | ESP32-C3 BLE Joystick | 수동 조작 |

---

## 🧭 Dual IMU Alignment

두 개의 MPU6050을 서로 다른 위치에 장착합니다.

```text
       ┌────────────────────┐
       │    Upper Plate     │
       │    MPU6050 0x69    │
       └─────────┬──────────┘
                 │
              ALT Axis
                 │
       ┌─────────┴──────────┐
       │     Base Plate     │
       │    MPU6050 0x68    │
       └────────────────────┘
```

Base IMU는 움직이지 않는 기준면을 측정하고 Upper IMU는 ALT축과 함께 움직이는 상단판의 pitch를 측정합니다.

```text
error = corrected_upper_pitch - corrected_base_pitch
```

오차가 허용 범위 안으로 들어오도록 ALT 모터를 자동 보정합니다.

### Alignment 설정

- 허용 오차: **±0.2°**
- 안정 판정: **3회 연속**
- 필터 샘플: **11 samples**
- 최대 보정 시간: **60초**
- 최대 반복: **80회**
- 시작점 기준 최대 상대 이동: **±15°**

초기 probe 이동을 통해 ALT 모터의 실제 회전 방향과 소프트웨어 좌표계의 +방향이 일치하는지도 확인합니다.

---

## ⚙️ Motor Control

17HM19-2004S 모터와 TMC2209를 사용합니다.

```text
0.9° / Full Step
       ↓
1 / 32 Microstep
       ↓
27 : 1 Gear Reduction
       ↓
≈ 0.00104167° / output step
       ↓
≈ 960 steps / degree
```

### GPIO

```text
AZ
STEP : BCM 22
DIR  : BCM 27

ALT
STEP : BCM 10
DIR  : BCM 9
```

Tracking Loop는 **20 Hz** 주기로 목표 위치를 업데이트하며, 한 번의 업데이트에서 이동하는 스텝 수를 제한합니다.

---

## 📍 GNSS

GY-NEO6MV2에서 다음 정보를 획득합니다.

```text
Latitude
Longitude
Altitude
UTC
```

유효한 GNSS Fix를 확보한 후 관측자 위치를 Tracking 시스템에 등록하고 GNSS UTC를 천문 계산의 시간 기준으로 사용합니다.

---

## ⭐ Plate Solving

카메라 이미지에서 현재 카메라가 실제로 바라보는 하늘의 위치를 확인하기 위해 **Astrometry.net Plate Solving**을 사용합니다.

```text
Camera
  ↓
Image Capture
  ↓
Astrometry.net
  ↓
RA / Dec
  ↓
Current Camera Position
  ↓
Drift Correction
```

현재 설정:

- Plate Solving 간격: **120초**
- 연속 실패 허용: **3회**
- 성공 보정 횟수: **15회**
- Plate Solving timeout: **60초**
- 검색 시야각 범위: **15° ~ 30°**

---

## 🔭 Automatic Tracking

GNSS 위치와 UTC를 이용하여 현재 시각의 목표 천체 위치를 계산하고 ALT/AZ 좌표로 변환합니다.

```text
Observer Position
      +
Current UTC
      +
Target RA / Dec
      ↓
Astronomical Calculation
      ↓
Target Alt / Az
      ↓
Motor Controller
      ↓
ALT / AZ Movement
```

Tracking Loop는 약 **20 Hz**로 실행됩니다.

---

## 🔄 Drift Correction

기계적인 오차와 초기 정렬 오차가 누적되는 문제를 줄이기 위해 Tracking 중 주기적으로 카메라 이미지를 촬영하고 Plate Solving을 다시 수행합니다.

```text
Tracking
   ↓
Camera Capture
   ↓
Plate Solving
   ↓
Actual RA / Dec
   ↓
Position Error
   ↓
Drift Correction
   ↓
Tracking
```

이를 통해 계산상 목표 위치와 실제 카메라가 바라보는 위치의 차이를 주기적으로 보정합니다.

---

## 📷 Final Capture

Tracking이 **15회의 성공적인 보정**을 완료하면 Tracking을 유지하면서 최종 촬영 단계로 넘어갑니다.

기본 설정:

```text
Sequence Capture
10 seconds × 18 frames
≈ 180 seconds total integration
```

ALT/AZ 방식의 마운트에서 발생할 수 있는 시야 회전을 고려하여 단일 180초 노출 대신 여러 sub-frame을 촬영하고 후처리에서 정렬·회전 보정·스태킹하는 방식을 사용합니다.

---

## 🎮 BLE Manual Control

ESP32-C3 기반 BLE Joystick으로 자동 Tracking 전에 카메라 방향을 수동 조정할 수 있습니다.

```text
Joystick X      → AZ
Joystick Y      → ALT
Joystick Click  → Tracking Start / Stop
```

BLE 연결이 끊어지면 안전을 위해 Tracking을 중단합니다.

---

## 🧠 Software Architecture

```text
main.py
 │
 ├── SensorManager
 │      └── GNSS
 │
 ├── TrackingManager
 │      ├── IMU Alignment
 │      ├── Motor Control
 │      ├── Astronomical Tracking
 │      ├── Plate Solving
 │      └── Drift Correction
 │
 └── BLEManager
        └── BLE Joystick
```

기능별 Manager로 역할을 분리하고 `main.py`에서 전체 실행 흐름과 시스템 상태를 관리합니다.

---

## 🔄 System State

```text
INIT
 │
 ▼
ALIGN
 │
 ▼
MANUAL
 │
 ▼
TARGET_CAPTURE
 │
 ▼
TRACKING
 │
 ├───────────────┐
 │               │
 ▼               │
DRIFT_CORRECTION─┘
 │
 ▼
CAPTURE
 │
 ▼
MANUAL
```

---

## 🛡️ Safety & Failure Handling

실제 하드웨어를 움직이는 시스템이므로 오류 상황에 대한 처리를 포함했습니다.

### IMU

- I²C 통신 오류
- `WHO_AM_I` 오류
- 유효하지 않은 측정값
- Alignment timeout
- 허용 이동 범위 초과

→ ALT 0° 확정을 실패시키고 운용을 진행하지 않습니다.

### GNSS

GNSS Fix를 제한 시간 안에 획득하지 못하면 Tracking을 시작하지 않습니다.

### Plate Solving

연속 Plate Solving 실패가 설정 횟수에 도달하면 Tracking 세션을 종료합니다.

### BLE

BLE 연결이 끊어지면 모터 Tracking을 중단합니다.

### Mechanical Safety

현재 장비에는 물리적 Limit Switch가 없습니다. 실제 시험에서는 기계적 끝점에서 충분히 떨어진 위치에서 시작해야 합니다.

---

## 📊 Logging

각 Tracking 세션의 결과를 JSON으로 저장합니다.

```text
data/logs/
├── tracking_*.json
└── platesolving_*.json
```

Tracking Log에는 IMU 보정 결과, 관측자 정보, 목표, 오차, 상태 이벤트와 이미지 경로를 기록하고, Plate Solving Log에는 최초 목표와 Drift Correction 결과를 기록합니다.

---

## 💻 Tech Stack

### Hardware

`Raspberry Pi 5` · `Raspberry Pi HQ Camera` · `GY-NEO6MV2` · `MPU6050 ×2` · `17HM19-2004S ×2` · `TMC2209 ×2` · `ESP32-C3`

### Software

`Python` · `asyncio` · `I²C` · `UART` · `BLE` · `GPIO` · `Picamera2` · `Astrometry.net`

### Engineering Topics

`Embedded Linux` · `Sensor Integration` · `Stepper Motor Control` · `Hardware Calibration` · `Astronomical Coordinate Calculation` · `Plate Solving` · `Feedback / Drift Correction` · `Asynchronous Programming` · `Failure Handling` · `Data Logging`

---

## 📁 Project Structure

```text
StarTracker_IMU_Final/
│
├── main.py
├── config.py
├── README.md
├── HARDWARE_ACCEPTANCE_CHECKLIST.md
├── VERIFICATION_REPORT.md
│
├── camera/
├── communication/
├── controllers/
├── hardware/
└── docs/
```

`StarTracker_IMU_Final`은 현재 최종 하드웨어 구성과 전체 Tracking 알고리즘을 포함하는 핵심 디렉터리입니다.

---

## 🧪 Verification

하드웨어 검증 절차는 `HARDWARE_ACCEPTANCE_CHECKLIST.md`에 정리되어 있으며, 하드웨어 없이 실행할 수 있는 unittest 기반 테스트도 제공합니다.

```bash
python -m unittest discover -s tests -v
```

실행 환경에서는 다음과 같이 시작합니다.

```bash
python3 main.py
```

---

## 🚧 주요 개발 과제

### 1. Dual IMU 기반 초기 정렬

두 IMU의 pitch 차이를 이용해 상단판의 초기 기준점을 자동 보정했습니다.

### 2. 실제 모터 방향과 좌표계 정합

ALT축의 실제 회전 방향과 소프트웨어의 +방향이 일치하도록 초기 probe 기반 방향 판별을 적용했습니다.

### 3. 계산값과 실제 카메라 방향의 차이

천체 위치 계산만으로 발생할 수 있는 기계적 오차와 정렬 오차를 줄이기 위해 Plate Solving 기반 Drift Correction을 Tracking에 통합했습니다.

### 4. 장노출 촬영

ALT/AZ 마운트에서 발생할 수 있는 시야 회전을 고려하여 10초 sub-frame 18장을 촬영하고 후처리에서 정렬·회전 보정·스태킹하는 방식을 적용했습니다.

---

## 🎯 What I Built

이 프로젝트에서 핵심적으로 다룬 부분은 **센서와 모터를 각각 동작시키는 것이 아니라 서로 다른 하드웨어의 데이터를 하나의 제어 루프로 통합하는 것**입니다.

```text
GNSS
  ↓
Position / Time
  ↓
Astronomical Calculation
  ↓
Target Alt / Az
  ↓
Motor Control
  ↓
Camera
  ↓
Plate Solving
  ↓
Drift Correction
  ↓
Continuous Tracking
```

하드웨어 제어, 센서 보정, 천문 계산, 영상 기반 피드백을 하나의 Embedded System으로 통합하는 것을 목표로 개발했습니다.

---

## 📌 Repository

**GitHub:** `choejunhyeok2005/StarTracker`

최종 구현은 `StarTracker_IMU_Final` 디렉터리를 중심으로 확인할 수 있습니다.

---

## License

This project is developed as a personal embedded-system and astronomical tracking project.
