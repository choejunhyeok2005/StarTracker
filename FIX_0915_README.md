# total_fix_0915 수정본

## 실행

이 폴더 전체를 Raspberry Pi에 복사하고 해당 폴더에서 실행합니다.

```bash
cd ~/total_fix_0915
python main.py
```

기존에 사용하던 Raspberry Pi Python 환경, GPIO/I2C/UART/BLE 권한,
Picamera2/libcamera, solve-field 실행 파일과 Astrometry 인덱스가 필요합니다.
사진과 JSON은 실행 폴더의 data/images, data/logs에 저장됩니다.
requirements.txt는 Python 의존성 목록이며 Pi 시스템 패키지와 인덱스를 설치하지 않습니다.

## 동작

GNSS 수신과 IMU 정렬 후 유효한 위치/UTC를 기다립니다.
MANUAL에서 조이스틱으로 구도를 잡고 클릭하면 영상이 중단되고 target plate solving을 시작합니다.
성공하면 고정 RA/Dec를 추적하고, config의 `CORRECTION_INTERVAL`마다 보정합니다.
보정 성공 횟수가 `MAX_CORRECTION_COUNT`에 도달하면 추적을 유지하며
`FINAL_SUBEXPOSURE_SECONDS`, `FINAL_FRAME_INTERVAL_SECONDS`, `FINAL_FRAME_COUNT`
설정으로 촬영합니다. 촬영 중에도 `FINAL_DRIFT_CORRECTION_EVERY_FRAMES`장마다
plate solving으로 drift를 보정합니다.
세션이 끝나면 작업 스레드 종료를 확인하고 영상 및 조이스틱을 복구합니다.
최초 solving 실패 시에도 MANUAL로 복귀합니다.
자동 세션 중 클릭은 중지 요청입니다. 이미 진행 중인 사진/solve-field는 반환될 때까지 기다립니다.
영상 복귀 후 브라우저의 기존 MJPEG 연결은 새로고침이 필요할 수 있습니다.

## 주요 변경

- main.py / communication/ble_manager.py: 클릭 뒤 무조건 180초 촬영/종료하던 경로 제거.
- controllers/session_controller.py: 비동기 세션 관리, 중복 시작 차단, BLE 연결 유지,
  작업 종료 확인 후 수동 조작/영상 재개, 종료 시 진행 중 작업 정리.
- managers/tracking_manager.py: 수동 이동 차단 플래그, 예정 시각 기준 20 Hz 목표 루프,
  GNSS UTC 필수, 사진별 고유 경로/촬영 호출 시작·종료 및 solving 종료 UTC 기록,
  IMU offset 기록, 중지 로그 마감을 작업 종료 이후로 지연.
- managers/sensor_manager.py: 위치 fix와 UTC를 동일 timeout 안에서 모두 기다림.
- config.py: IMU 방향 시험 이동 240스텝.
- camera/camera_manager.py: 영상 재개 시 자동 노출 복원, cv2 지연 import.
- hardware/imu.py: 센서 주소 및 WHO_AM_I 출력.
- requirements.txt: Flask 및 OpenCV 명시.
- controllers/motor_controller.py: 추적용 0° 제한 때문에 발생하던 수동 ALT 한쪽 방향
  잠금을 제거하고, 별도 수동 제한을 config에서 선택할 수 있게 변경.
- camera/camera_manager.py / managers/tracking_manager.py: 장노출 프레임 시간·간격·횟수와
  촬영 중 drift correction 주기를 config로 제어.
- services/console_logger.py / main.py: stdout·stderr 전체를 `console_*.log`로 동시 저장.

## 검증 범위와 현장 확인

이 수정본의 자동 테스트는 하드웨어 대역을 사용합니다.
Windows에서 실제 Pi 모터, 노출 시간, 별 사진 solving, 30분 이상 추적은 검증하지 않았습니다.
실제 20 Hz 달성 여부는 좌표 계산 및 GPIO 처리 시간에 따라 달라집니다.
기존 drift 계산은 solving 완료 시각을 사용합니다. 기록된 촬영 시각은 capture_plate 호출의
시작/종료 시각이며 센서의 정확한 노출 중앙 시각은 아닙니다.
긴 solving 동안의 모터 이동/촬영 시각 보상은 별도 정밀도 검증이 필요합니다.
기존 GPIO 핀과 기어 설정은 보존했습니다. config.py에는 GEAR_RATIO=27로 STEP_ANGLE을
계산한 뒤 나침반 설정부에서 GEAR_RATIO=54를 재정의하는 기존 구조가 있으므로
실제 추적축 감속비와 960 steps/degree의 일치를 확인하세요.
COMPASS_CALIBRATION_ENABLED는 원본 파일의 True를 유지했습니다
(첨부 실행 로그에서는 나침반 보정이 비활성 상태였습니다).

## 첫 현장 확인

클릭 후 TARGET_CAPTURE -> Plate image saved -> PlateSolver 결과를 확인합니다.
성공하면 목표 RA/Dec 및 TRACKING이 출력되어야 합니다.
세션 중 클릭 후 MANUAL 복귀와 재시작을 먼저 확인하고 전체 15회 보정을 시험하세요.
최종 결과는 `data/images/final_sequence` 아래의 개별 프레임들입니다. 합성본이나
마지막 한 장만 저장하는 구조가 아닙니다. `data/logs`에는 JSON과 전체 콘솔 로그가
저장됩니다.

기존 README, VERIFICATION_REPORT, 체크리스트와 해시 파일은 원본 참고 자료입니다.
이번 수정본의 변경/제한은 이 파일, 검증 결과는 FIX_0915_VERIFICATION.md가 기준입니다.
