"""
main.py

StarTracker Main Program (Compass Integrated with Async LCD Queue)
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
    """비동기 큐(Queue) 기반으로 실시간 LCD 출력을 보장하는 클래스"""
    def __init__(self, cols: int = 20, rows: int = 4):
        self.cols = cols
        self.rows = rows
        self.lines: List[str] = [" " * self.cols for _ in range(self.rows)]
        self.lcd = None
        self.queue: asyncio.Queue = asyncio.Queue()
        self.worker_task: asyncio.Task = None

        if HAS_LCD:
            try:
                self.lcd = CharLCD(
                    i2c_expander='PCF8574',
                    address=0x27,
                    port=1,
                    cols=self.cols,
                    rows=self.rows,
                    auto_linebreaks=False
                )
                self.lcd.clear()
            except Exception as e:
                print(f"[LCD Init Warning] Could not initialize LCD: {e}", flush=True)
                self.lcd = None

    def start_worker(self) -> None:
        """비동기 이벤트 루프 내에서 LCD 출력 전용 백그라운드 워커 실행"""
        if self.lcd and self.worker_task is None:
            self.worker_task = asyncio.create_task(self._lcd_worker())

    async def _lcd_worker(self) -> None:
        """큐에서 메시지를 대기했다가 수신 순서대로 LCD 화면을 스크롤하며 표출"""
        while True:
            try:
                text = await self.queue.get()

                # 20자 길이에 맞춰 좌측 정렬 및 공백 채움
                formatted_text = str(text)[:self.cols].ljust(self.cols)
                
                # 스크롤 버퍼 업데이트
                self.lines.pop(0)
                self.lines.append(formatted_text)

                # 각 줄 위치를 명시적으로 지정하여 빠르게 덮어쓰기
                for row_idx, line_str in enumerate(self.lines):
                    self.lcd.cursor_pos = (row_idx, 0)
                    self.lcd.write_string(line_str)

                self.queue.task_done()
                await asyncio.sleep(0.01)  # I2C 통신 안정화를 위한 미세 지연

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[LCD Worker Error] {e}", flush=True)

    def log(self, text: str = "") -> None:
        """터미널 즉시 출력 및 비동기 큐 삽입"""
        # 1. 터미널 출력을 즉시 수행
        print(text, flush=True)

        if not self.lcd:
            return

        # 2. 비동기 큐에 출력 문장 전달
        try:
            self.queue.put_nowait(text)
        except Exception:
            pass

    def close(self) -> None:
        """백그라운드 태스크 취소, LCD 화면 소등 및 백라이트 Off"""
        if self.worker_task:
            self.worker_task.cancel()

        if self.lcd:
            try:
                self.lcd.clear()
                self.lcd.backlight_enabled = False
                self.lcd.close()
            except Exception:
                pass


async def run_star_tracker() -> None:

    # 1. LCD 프린터 생성 및 백그라운드 워커 시작
    printer = LCDPrinter(cols=20, rows=4)
    printer.start_worker()

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
            print(f"[Main] BLE Shutdown Warning: {error}", flush=True)

        try:
            tracking.shutdown()
        except Exception as error:
            print(f"[Main] Tracking Shutdown Warning: {error}", flush=True)

        try:
            compass.shutdown()
        except Exception as error:
            print(f"[Main] Compass Shutdown Warning: {error}", flush=True)

        try:
            sensor.shutdown()
        except Exception as error:
            print(f"[Main] Sensor Shutdown Warning: {error}", flush=True)

        printer.log("Shutdown Complete")
        await asyncio.sleep(1)

        # LCD 워커 종료 및 전원 off
        printer.close()


def main() -> None:

    try:
        asyncio.run(run_star_tracker())
    except KeyboardInterrupt:
        print()
        print("[Main] Program terminated by user.", flush=True)


if __name__ == "__main__":

    main()