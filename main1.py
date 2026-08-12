"""
main.py

StarTracker Main Program (Compass Integrated with LCD)
"""

import asyncio
from typing import List

import config

from communication.ble_manager import BLEManager
from managers.compass_manager import CompassManager
from managers.sensor_manager import SensorManager
from managers.tracking_manager import TrackingManager

# RPLCD 라이브러리 로드
try:
    from RPLCD.i2c import CharLCD
    HAS_LCD = True
except ImportError:
    HAS_LCD = False


class LCDPrinter:
    """터미널 print와 I2C LCD 출력을 동시에 처리하는 클래스"""
    def __init__(self, cols: int = 20, rows: int = 4):
        self.cols = cols
        self.rows = rows
        self.lines: List[str] = []
        self.lcd = None

        if HAS_LCD:
            try:
                self.lcd = CharLCD(
                    i2c_expander='PCF8574',
                    address=0x27,
                    port=1,
                    cols=self.cols,
                    rows=self.rows
                )
                self.lcd.clear()
            except Exception as e:
                print(f"[LCD Init Warning] Could not initialize LCD: {e}")
                self.lcd = None

    def log(self, text: str = "") -> None:
        """터미널에 출력하고 LCD 화면을 스크롤하며 업데이트합니다."""
        # 1. 터미널 출력
        print(text)

        if not self.lcd:
            return

        # 빈 줄 처리
        if not text.strip():
            return

        # 2. LCD 출력용 문자열 처리 (최대 열 길이에 맞춰 잘라냄)
        formatted_text = text[:self.cols].ljust(self.cols)

        # 3. 줄 버퍼 업데이트 (최대 4줄 유지)
        self.lines.append(formatted_text)
        if len(self.lines) > self.rows:
            self.lines.pop(0)

        # 4. LCD 화면 갱신
        try:
            self.lcd.clear()
            for i, line in enumerate(self.lines):
                self.lcd.cursor_pos = (i, 0)
                self.lcd.write_string(line)
        except Exception as e:
            print(f"[LCD Write Error] {e}")

    def close(self) -> None:
        """LCD 백라이트를 끄고 리소스를 해제합니다."""
        if self.lcd:
            try:
                self.lcd.clear()
                self.lcd.backlight_enabled = False
                self.lcd.close()
            except Exception:
                pass


async def run_star_tracker() -> None:

    # LCD 프린터 객체 생성
    printer = LCDPrinter(cols=20, rows=4)

    sensor = SensorManager()
    tracking = TrackingManager()
    ble = BLEManager(tracking)
    compass = CompassManager()

    try:

        printer.log()
        printer.log("====================")
        printer.log("    STAR TRACKER    ")
        printer.log("====================")
        printer.log()

        # ==================================================
        # 1. GNSS Initialize
        # ==================================================

        printer.log("[Main] GNSS init...")

        sensor_initialized = sensor.initialize()

        if not sensor_initialized:
            printer.log("[Main] GNSS failed.")
            return

        # ==================================================
        # 2. Hardware + Compass Calibration + IMU Alignment
        # ==================================================

        printer.log("[Main] HW & Calib...")

        if compass.initialize():
            cal_success = await compass.calibrate_async(sample_divisions=25)
            if not cal_success:
                printer.log("[Main] Calib failed.")

        alignment_result = tracking.initialize_hardware()

        if alignment_result is not None:
            printer.log(f"ALT err:{alignment_result.final_error_deg:.2f} deg")

        # ==================================================
        # 3. Wait For GNSS Fix
        # ==================================================

        printer.log("[Main] Wait GNSS Fix")

        fix_success = sensor.wait_for_fix(timeout=config.GNSS_FIX_TIMEOUT)

        if not fix_success:
            printer.log("[Main] Fix failed.")
            printer.log("Restart outdoors.")
            return

        location = sensor.get_location()

        if location is None:
            printer.log("[Main] No Location.")
            return

        latitude, longitude, altitude = location

        printer.log(f"Lat: {latitude:.4f}")
        printer.log(f"Lon: {longitude:.4f}")
        printer.log(f"Alt: {altitude:.1f} m")

        initial_azimuth = compass.get_heading()
        if initial_azimuth is not None:
            printer.log(f"Azimuth: {initial_azimuth:.1f} deg")

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

        printer.log("[Main] BLE connect...")

        ble_connected = await ble.connect()

        if not ble_connected:
            printer.log("[Main] BLE failed.")
            return

        # ==================================================
        # 6. System Ready
        # ==================================================

        printer.log("====================")
        printer.log("    SYSTEM READY    ")
        printer.log("====================")

        # ==================================================
        # 7. Main Loop
        # ==================================================

        while True:

            if not ble.running:
                printer.log("[Main] BLE lost.")
                break

            current_az = compass.get_heading()
            if current_az is not None:
                pass

            await asyncio.sleep(0.5)

    except asyncio.CancelledError:
        printer.log("[Main] Cancelled.")
        raise

    except Exception as error:
        printer.log(f"Err: {error}")

    finally:

        printer.log("[Main] Shutdown...")

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

        printer.log("Shutdown Complete")
        await asyncio.sleep(1)
        
        # LCD 전원 off 및 닫기
        printer.close()


def main() -> None:

    try:
        asyncio.run(run_star_tracker())
    except KeyboardInterrupt:
        print()
        print("[Main] Program terminated by user.")


if __name__ == "__main__":

    main()