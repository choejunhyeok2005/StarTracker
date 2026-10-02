"""
managers/compass_manager.py

QMC5883L 지자기 센서 및 TMC2209 스텝모터 캘리브레이션 관리 모듈
"""

import asyncio
import math
import time
import RPi.GPIO as GPIO
import smbus

import config


class CompassManager:

    def __init__(self):
        self.bus_num = config.COMPASS_I2C_BUS
        self.addr = config.COMPASS_I2C_ADDR
        self.bus = None
        self.declination = config.MAGNETIC_DECLINATION

        self.step_pin = config.COMPASS_STEP_PIN
        self.dir_pin = config.COMPASS_DIR_PIN
        self.en_pin = config.COMPASS_EN_PIN

        self.output_steps_per_rev = (
            config.MOTOR_STEPS_PER_REV * config.MICROSTEP * config.GEAR_RATIO
        )

        self.x_offset = 0.0
        self.y_offset = 0.0
        self.x_scale = 1.0
        self.y_scale = 1.0
        self.is_calibrated = False

    def initialize(self) -> bool:
        """I2C 센서 및 GPIO 초기화"""
        try:
            self.bus = smbus.SMBus(self.bus_num)
            self.bus.write_byte_data(self.addr, 0x0B, 0x01)
            time.sleep(0.01)
            self.bus.write_byte_data(self.addr, 0x09, 0x1D)
            time.sleep(0.1)

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.setup(self.dir_pin, GPIO.OUT)
            GPIO.setup(self.en_pin, GPIO.OUT)

            GPIO.output(self.en_pin, GPIO.LOW)
            print("[Compass] Hardware initialized successfully.")
            return True
        except Exception as e:
            print(f"[Compass] Initialization failed: {e}")
            return False

    def disable_motor(self) -> None:
        """모터 발열 방지를 위한 Disable 처리"""
        try:
            GPIO.output(self.en_pin, GPIO.HIGH)
        except Exception:
            pass

    def enable_motor(self) -> None:
        """모터 재활성화"""
        try:
            GPIO.output(self.en_pin, GPIO.LOW)
        except Exception:
            pass

    def _read_xyz(self):
        """I2C 데이터 읽기"""
        if not self.bus:
            return None

        status = self.bus.read_byte_data(self.addr, 0x06)
        if not (status & 0x01):
            return None

        data = self.bus.read_i2c_block_data(self.addr, 0x00, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]

        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536

        return x, y, z

    def _rotate_motor_steps(
        self, steps: int, direction: int = 1, step_delay: float = 0.0004
    ):
        """스텝 펄스 출력"""
        if direction > 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)

        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(step_delay)

    async def calibrate_async(self, sample_divisions: int = 36) -> bool:
        """비동기 360도 회전 캘리브레이션"""
        self.enable_motor()
        x_min = y_min = 99999
        x_max = y_max = -99999

        steps_per_sample = self.output_steps_per_rev // sample_divisions

        print("\n[Compass] Starting Motor Calibration...")

        for i in range(sample_divisions):
            await asyncio.to_thread(
                self._rotate_motor_steps, steps_per_sample
            )
            await asyncio.sleep(0.05)

            xyz = self._read_xyz()
            if xyz:
                x, y, _ = xyz
                x_min, x_max = min(x_min, x), max(x_max, x)
                y_min, y_max = min(y_min, y), max(y_max, y)

                print(
                    f"[Compass] Calibrating {i+1:2d}/{sample_divisions} | "
                    f"X:[{x_min}, {x_max}] Y:[{y_min}, {y_max}]"
                )

        print("\n[Compass] Calibration Scan Complete.")

        if x_max == x_min or y_max == y_min:
            print("[Compass] Calibration Error: Insufficient sensor variation.")
            return False

        self.x_offset = (x_max + x_min) / 2
        self.y_offset = (y_max + y_min) / 2
        self.x_scale = (x_max - x_min) / 2
        self.y_scale = (y_max - y_min) / 2
        self.is_calibrated = True

        print(
            f"[Compass] Calibration Parameters: Offset({self.x_offset:.1f}, {self.y_offset:.1f}), "
            f"Scale({self.x_scale:.1f}, {self.y_scale:.1f})"
        )

        self.disable_motor()
        return True

    def get_heading(self) -> float | None:
        """보정된 진북 방위각 반환 (0~360°)"""
        xyz = self._read_xyz()
        if not xyz:
            return None

        x, y, _ = xyz

        if not self.is_calibrated:
            x_cal, y_cal = x, y
        else:
            x_cal = (x - self.x_offset) / self.x_scale
            y_cal = (y - self.y_offset) / self.y_scale

        heading_mag = math.degrees(math.atan2(y_cal, x_cal))
        if heading_mag < 0:
            heading_mag += 360

        heading_true = (heading_mag + self.declination) % 360
        heading_cw = (360 - heading_true) % 360
        return heading_cw

    def shutdown(self) -> None:
        """종료 시 전원 해제"""
        self.disable_motor()
        try:
            GPIO.cleanup()
        except Exception:
            pass
        print("[Compass] Hardware Shutdown Complete.")