"""
main.py

StarTracker Main Program (Compass Integrated)
"""

import asyncio
import threading
import time

import config

from communication.ble_manager import BLEManager
from managers.compass_manager import CompassManager
from managers.lcd_manager import install_print_hook, lcd_manager, restore_print_hook
from managers.sensor_manager import SensorManager
from managers.tracking_manager import TrackingManager
from services.web_server import create_streaming_app


async def run_star_tracker() -> None:

    sensor = SensorManager()
    tracking = TrackingManager()
    ble = BLEManager(tracking)
    compass = CompassManager()

    install_print_hook()

    try:

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
        # 2. Hardware + Compass Calibration + IMU Alignment
        # ==================================================

        print("[Main] Hardware initialization & Compass calibration...")

        if compass.initialize():
            cal_success = await compass.calibrate_async(sample_divisions=25)
            if not cal_success:
                print(
                    "[Main] Compass calibration failed. Proceeding with caution."
                )

        alignment_result = tracking.initialize_hardware()

        if alignment_result is not None:
            print(
                "[Main] ALT zero established: "
                f"error={alignment_result.final_error_deg:.3f} deg"
            )

        # ==================================================
        # 3. Wait For GNSS Fix
        # ==================================================

        fix_success = sensor.wait_for_fix(timeout=config.GNSS_FIX_TIMEOUT)

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

        initial_azimuth = compass.get_heading()
        if initial_azimuth is not None:
            print(f"[Main] Initial True North Azimuth: {initial_azimuth:.2f}°")

        # ==================================================
        # 4. GNSS UTC Provider + Observer
        # ==================================================

        tracking.set_time_provider(sensor.get_utc)

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
        # 6. Start Streaming
        # ==================================================

        print("[Main] Starting camera streaming...")

        camera = tracking.camera
        camera.start_streaming()

        print(
            f"[Main] Web stream available at http://"
            f"{config.STREAMING_SERVER_HOST}:"
            f"{config.STREAMING_SERVER_PORT}/"
        )

        # Flask 서버를 별도 스레드에서 실행
        flask_app = create_streaming_app(camera)

        def run_flask():
            flask_app.run(
                host=config.STREAMING_SERVER_HOST,
                port=config.STREAMING_SERVER_PORT,
                debug=False,
                use_reloader=False,
                threaded=True,
            )

        flask_thread = threading.Thread(
            target=run_flask,
            name="FlaskStreamingServer",
            daemon=True,
        )
        flask_thread.start()

        print("[Main] Streaming server started in background")

        # ==================================================
        # 7. System Ready
        # ==================================================

        print()
        print("========================================")
        print("              SYSTEM READY")
        print("========================================")
        print()
        print("[Control]")
        print("Joystick X : AZ movement")
        print("Joystick Y : ALT movement")
        print("Joystick Click : Manual Control Finished → Long Exposure Start")
        print("Ctrl+C : Program Exit")
        print()

        # ==================================================
        # 8. Main Loop
        # ==================================================

        while True:

            # 사용자가 조이스틱을 클릭하면 ble._stop_event가 설정된다
            if ble._stop_event.is_set():
                print("[Main] Joystick click detected → Manual control finished")
                break

            if not ble.running:
                print(
                    "[Main] BLE connection lost. "
                    "Stopping the system for safety."
                )
                break

            current_az = compass.get_heading()
            if current_az is not None:
                pass

            await asyncio.sleep(0.5)

        # ==================================================
        # 9. Stop Streaming + Long Exposure
        # ==================================================

        print("[Main] Stopping streaming...")
        camera.stop_streaming()

        print("[Main] Waiting for camera stabilization...")
        time.sleep(0.5)

        print("[Main] Starting long exposure capture...")
        camera.capture_long_exposure()

        print("[Main] Long exposure complete")

    except asyncio.CancelledError:
        print("[Main] Program cancellation requested.")
        raise

    except Exception as error:
        print(f"[Main] Unexpected Error: {error}")

    finally:

        print()
        print("[Main] System shutdown started...")

        try:
            camera.stop_streaming()
        except Exception as error:
            print(f"[Main] Camera Streaming Shutdown Warning: {error}")

        try:
            await ble.disconnect()
        except Exception as error:
            print(f"[Main] BLE Shutdown Warning: {error}")

        try:
            tracking.shutdown()
        except Exception as error:
            print(f"[Main] Tracking Shutdown Warning: {error}")

        try:
            compass.shutdown()
        except Exception as error:
            print(f"[Main] Compass Shutdown Warning: {error}")

        try:
            sensor.shutdown()
        except Exception as error:
            print(f"[Main] Sensor Shutdown Warning: {error}")

        print("[Main] System shutdown complete.")
        # Prevent further prints from being enqueued to the LCD while we display shutdown text
        try:
            restore_print_hook()
        except Exception:
            pass
        try:
            lcd_manager.clear()
            lcd_manager.write_text("Star Tracker System shutdown...", end="\n")
            time.sleep(5.0)
        except Exception:
            pass
        lcd_manager.clear()
        lcd_manager.set_backlight(False)
        lcd_manager.close()


def main() -> None:

    try:
        asyncio.run(run_star_tracker())
    except KeyboardInterrupt:
        print()
        print("[Main] Program terminated by user.")
        try:
            restore_print_hook()
        except Exception:
            pass
        try:
            lcd_manager.clear()
            lcd_manager.write_text("Star Tracker System shutdown...", end="\n")
            time.sleep(5.0)
        except Exception:
            pass
        lcd_manager.clear()
        lcd_manager.set_backlight(False)
        lcd_manager.close()


if __name__ == "__main__":

    main()