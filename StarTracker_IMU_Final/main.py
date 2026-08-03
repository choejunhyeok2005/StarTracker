"""
main.py

StarTracker Main Program

전체 실행 흐름:

1. GNSS reader 시작
2. Motor 초기화 및 dual-IMU ALT zero 보정
3. Camera 초기화
4. GNSS 위치 Fix 및 UTC 획득
5. 실제 위도 / 경도 / 고도와 GNSS clock 등록
6. BLE 연결 및 조이스틱 수동 조작
7. 클릭으로 target Plate Solving 및 Tracking 시작/중단
8. 2분마다 Plate Solving drift 보정
9. 15회 성공 후 Tracking을 유지하며 10초 subframe 18장 촬영
10. IMU/Tracking/Plate Solving 로그 저장 후 Manual Mode 복귀
"""

import asyncio

import config

from communication.ble_manager import BLEManager
from managers.sensor_manager import SensorManager
from managers.tracking_manager import TrackingManager


async def run_star_tracker() -> None:

    sensor = SensorManager()
    tracking = TrackingManager()
    ble = BLEManager(tracking)

    try:

        # ==================================================
        # Program Start
        # ==================================================

        print()
        print("========================================")
        print("             STAR TRACKER")
        print("========================================")
        print()

        # ==================================================
        # 1. GNSS Initialize
        # ==================================================

        print("[Main] GNSS initialization...")

        sensor_initialized = sensor.initialize()

        if not sensor_initialized:

            print("[Main] GNSS initialization failed.")
            return

        # ==================================================
        # 2. Motor + IMU Alignment + Camera Initialize
        # ==================================================

        print("[Main] Hardware initialization and IMU alignment...")

        alignment_result = tracking.initialize_hardware()

        if alignment_result is not None:
            print(
                "[Main] ALT zero established: "
                f"error={alignment_result.final_error_deg:.3f} deg"
            )

        # ==================================================
        # 3. Wait For GNSS Fix
        # ==================================================

        fix_success = sensor.wait_for_fix(
            timeout=config.GNSS_FIX_TIMEOUT
        )

        if not fix_success:

            print("[Main] GNSS Fix acquisition failed.")
            print("[Main] Move the system outdoors and restart.")
            return

        location = sensor.get_location()

        if location is None:

            print("[Main] GNSS location data is unavailable.")
            return

        latitude, longitude, altitude = location

        print()
        print("[Main] GNSS Location")
        print(f"[Main] Latitude  : {latitude:.6f}")
        print(f"[Main] Longitude : {longitude:.6f}")
        print(f"[Main] Altitude  : {altitude:.1f} m")
        print()

        # ==================================================
        # 4. GNSS UTC Provider + Observer
        # ==================================================

        tracking.set_time_provider(
            sensor.get_utc
        )

        tracking.configure_observer(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
        )

        # ==================================================
        # 5. BLE Connect
        # ==================================================

        print("[Main] BLE connection...")

        ble_connected = await ble.connect()

        if not ble_connected:

            print("[Main] BLE connection failed.")
            return

        # ==================================================
        # 6. System Ready
        # ==================================================

        print()
        print("========================================")
        print("              SYSTEM READY")
        print("========================================")
        print()
        print("[Control]")
        print("Joystick X : AZ movement")
        print("Joystick Y : ALT movement")
        print("Joystick Click : Tracking Start / Stop")
        print("Ctrl+C : Program Exit")
        print()

        # ==================================================
        # 7. Main Loop
        # ==================================================

        while True:

            if not ble.running:

                print(
                    "[Main] BLE connection lost. "
                    "Stopping the system for safety."
                )

                break

            await asyncio.sleep(0.5)

    except asyncio.CancelledError:

        print("[Main] Program cancellation requested.")
        raise

    except Exception as error:

        print(f"[Main] Unexpected Error: {error}")

    finally:

        # ==================================================
        # System Shutdown
        # ==================================================

        print()
        print("[Main] System shutdown started...")

        try:

            await ble.disconnect()

        except Exception as error:

            print(
                f"[Main] BLE Shutdown Warning: {error}"
            )

        try:

            tracking.shutdown()

        except Exception as error:

            print(
                f"[Main] Tracking Shutdown Warning: {error}"
            )

        try:

            sensor.shutdown()

        except Exception as error:

            print(
                f"[Main] Sensor Shutdown Warning: {error}"
            )

        print("[Main] System shutdown complete.")


def main() -> None:

    try:

        asyncio.run(
            run_star_tracker()
        )

    except KeyboardInterrupt:

        print()
        print("[Main] Program terminated by user.")


if __name__ == "__main__":

    main()
