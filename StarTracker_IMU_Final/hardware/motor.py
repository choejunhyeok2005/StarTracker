"""
hardware/motor.py

Raspberry Pi Step Motor Driver

실제 동작 확인된 BLE 조이스틱 모터 테스트를 기준으로 작성.

축 구성:
- AZ  : STEP BCM 22 / DIR BCM 27
- ALT : STEP BCM 10 / DIR BCM 9

기능:
- 한 축 단독 구동
- ALT/AZ 독립 스레드 동시 수동 구동
- Tracking용 두 축 비율 구동
"""

import threading
import time
from typing import Optional

import RPi.GPIO as GPIO

import config


class MotorDriver:
    """STEP/DIR 신호만 담당하는 저수준 모터 드라이버."""

    def __init__(self):

        self.initialized = False

        # 수동 제어 시 두 축이 각각 독립적으로 작동한다.
        self._alt_lock = threading.Lock()
        self._az_lock = threading.Lock()

        self._configured_pins: list[int] = []

    # =====================================================
    # Initialize
    # =====================================================

    def initialize(self) -> None:

        if self.initialized:
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        motor_pins = [
            config.MOTOR_BASE_STEP_PIN,
            config.MOTOR_BASE_DIR_PIN,
            config.MOTOR_MOUNT_STEP_PIN,
            config.MOTOR_MOUNT_DIR_PIN,
        ]

        for pin in motor_pins:

            GPIO.setup(
                pin,
                GPIO.OUT,
                initial=GPIO.LOW,
            )

            self._configured_pins.append(pin)

        # Enable 핀을 실제로 연결한 경우에만 설정
        enable_pin = getattr(
            config,
            "MOTOR_ENABLE_PIN",
            None,
        )

        if enable_pin is not None:

            GPIO.setup(
                enable_pin,
                GPIO.OUT,
                initial=GPIO.HIGH,
            )

            self._configured_pins.append(enable_pin)

        # 마이크로스텝 핀을 실제로 연결한 경우에만 설정
        ms1_pin = getattr(
            config,
            "MOTOR_MS1_PIN",
            None,
        )

        ms2_pin = getattr(
            config,
            "MOTOR_MS2_PIN",
            None,
        )

        for pin in (ms1_pin, ms2_pin):

            if pin is not None:

                GPIO.setup(
                    pin,
                    GPIO.OUT,
                    initial=GPIO.LOW,
                )

                self._configured_pins.append(pin)

        self.initialized = True

        print("[MotorDriver] GPIO Initialized")

    # =====================================================
    # Enable / Disable
    # =====================================================

    def enable(self) -> None:
        """
        Enable 핀이 연결되어 있으면 드라이버를 활성화한다.

        Enable 핀이 None이면 아무것도 하지 않는다.
        """

        self._require_initialized()

        enable_pin = getattr(
            config,
            "MOTOR_ENABLE_PIN",
            None,
        )

        if enable_pin is None:
            return

        active_low = getattr(
            config,
            "MOTOR_ENABLE_ACTIVE_LOW",
            True,
        )

        GPIO.output(
            enable_pin,
            GPIO.LOW if active_low else GPIO.HIGH,
        )

    def disable(self) -> None:

        if not self.initialized:
            return

        enable_pin = getattr(
            config,
            "MOTOR_ENABLE_PIN",
            None,
        )

        if enable_pin is None:
            return

        active_low = getattr(
            config,
            "MOTOR_ENABLE_ACTIVE_LOW",
            True,
        )

        GPIO.output(
            enable_pin,
            GPIO.HIGH if active_low else GPIO.LOW,
        )

    # =====================================================
    # Optional Microstep Setting
    # =====================================================

    def set_microstep(
        self,
        ms1: int,
        ms2: int,
    ) -> None:
        """
        MS1/MS2 핀 값을 직접 설정한다.

        실제 드라이버 모듈의 마이크로스텝 표를 확인한 값만 넣는다.
        핀이 연결되지 않았다면 설정하지 않는다.
        """

        self._require_initialized()

        ms1_pin = getattr(
            config,
            "MOTOR_MS1_PIN",
            None,
        )

        ms2_pin = getattr(
            config,
            "MOTOR_MS2_PIN",
            None,
        )

        if ms1_pin is None or ms2_pin is None:

            print(
                "[MotorDriver] Microstep pins are not configured."
            )

            return

        GPIO.output(
            ms1_pin,
            GPIO.HIGH if ms1 else GPIO.LOW,
        )

        GPIO.output(
            ms2_pin,
            GPIO.HIGH if ms2 else GPIO.LOW,
        )

    def set_mode(self, mode=None) -> None:
        """
        기존 MotorController 코드와의 호환용 함수.

        DEFAULT_MOTOR_MODE가 None이면 아무것도 하지 않는다.
        마이크로스텝 비트 조합을 확인한 뒤 config의
        MICROSTEP_MODE_TABLE에 직접 등록해서 사용한다.
        """

        if mode is None:
            return

        mode_table = getattr(
            config,
            "MICROSTEP_MODE_TABLE",
            {},
        )

        if mode not in mode_table:

            raise ValueError(
                f"Unknown or unverified microstep mode: {mode}"
            )

        ms1, ms2 = mode_table[mode]

        self.set_microstep(
            ms1=ms1,
            ms2=ms2,
        )

    # =====================================================
    # Axis Mapping
    # =====================================================

    @staticmethod
    def _get_axis_pins(
        axis: str,
    ) -> tuple[int, int]:

        axis = axis.upper()

        if axis == "AZ":

            return (
                config.MOTOR_BASE_STEP_PIN,
                config.MOTOR_BASE_DIR_PIN,
            )

        if axis in ("ALT", "EL"):

            return (
                config.MOTOR_MOUNT_STEP_PIN,
                config.MOTOR_MOUNT_DIR_PIN,
            )

        raise ValueError(
            "Axis must be 'ALT', 'EL', or 'AZ'."
        )

    def _get_axis_lock(
        self,
        axis: str,
    ) -> threading.Lock:

        axis = axis.upper()

        if axis == "AZ":
            return self._az_lock

        if axis in ("ALT", "EL"):
            return self._alt_lock

        raise ValueError(
            "Axis must be 'ALT', 'EL', or 'AZ'."
        )

    # =====================================================
    # STEP Pulse
    # =====================================================

    @staticmethod
    def _pulse(
        step_pin: int,
        pulse_delay: float,
    ) -> None:

        GPIO.output(
            step_pin,
            GPIO.HIGH,
        )

        time.sleep(pulse_delay)

        GPIO.output(
            step_pin,
            GPIO.LOW,
        )

        time.sleep(pulse_delay)

    # =====================================================
    # Single Axis Drive
    # =====================================================

    def drive(
        self,
        axis: str,
        steps: int,
        direction: bool,
        pulse_delay: Optional[float] = None,
    ) -> None:
        """
        지정한 축을 정해진 스텝만큼 움직인다.

        BLE 수동 제어에서는 보통 steps=1로 반복 호출한다.
        """

        self._require_initialized()

        steps = int(steps)

        if steps <= 0:
            return

        if pulse_delay is None:

            pulse_delay = config.MOTOR_PULSE_DELAY

        if pulse_delay <= 0:

            raise ValueError(
                "pulse_delay must be greater than zero."
            )

        step_pin, dir_pin = self._get_axis_pins(
            axis
        )

        axis_lock = self._get_axis_lock(
            axis
        )

        with axis_lock:

            GPIO.output(
                dir_pin,
                GPIO.HIGH if direction else GPIO.LOW,
            )

            for _ in range(steps):

                self._pulse(
                    step_pin=step_pin,
                    pulse_delay=pulse_delay,
                )

    # =====================================================
    # Dual Axis Tracking Drive
    # =====================================================

    def drive_dual(
        self,
        alt_steps: int,
        alt_direction: bool,
        az_steps: int,
        az_direction: bool,
        pulse_delay: Optional[float] = None,
    ) -> None:
        """
        Tracking용 ALT/AZ 두 축 비율 구동.

        두 축의 STEP 신호를 같은 시간 구간에 분산한다.

        예:
            ALT 5스텝
            AZ  2스텝

        두 AZ 스텝이 전체 ALT 동작 구간에 고르게 들어간다.
        """

        self._require_initialized()

        alt_steps = max(
            0,
            int(alt_steps),
        )

        az_steps = max(
            0,
            int(az_steps),
        )

        if alt_steps == 0 and az_steps == 0:
            return

        if pulse_delay is None:

            pulse_delay = config.MOTOR_PULSE_DELAY

        if pulse_delay <= 0:

            raise ValueError(
                "pulse_delay must be greater than zero."
            )

        alt_step_pin, alt_dir_pin = (
            self._get_axis_pins("ALT")
        )

        az_step_pin, az_dir_pin = (
            self._get_axis_pins("AZ")
        )

        # 항상 ALT → AZ 순서로 Lock을 잡아 교착상태 방지
        with self._alt_lock:

            with self._az_lock:

                GPIO.output(
                    alt_dir_pin,
                    GPIO.HIGH
                    if alt_direction
                    else GPIO.LOW,
                )

                GPIO.output(
                    az_dir_pin,
                    GPIO.HIGH
                    if az_direction
                    else GPIO.LOW,
                )

                total_ticks = max(
                    alt_steps,
                    az_steps,
                )

                alt_accumulator = 0
                az_accumulator = 0

                for _ in range(total_ticks):

                    pulse_alt = False
                    pulse_az = False

                    alt_accumulator += alt_steps
                    az_accumulator += az_steps

                    if (
                        alt_accumulator
                        >= total_ticks
                    ):

                        alt_accumulator -= total_ticks
                        pulse_alt = True

                    if (
                        az_accumulator
                        >= total_ticks
                    ):

                        az_accumulator -= total_ticks
                        pulse_az = True

                    if pulse_alt:

                        GPIO.output(
                            alt_step_pin,
                            GPIO.HIGH,
                        )

                    if pulse_az:

                        GPIO.output(
                            az_step_pin,
                            GPIO.HIGH,
                        )

                    time.sleep(pulse_delay)

                    if pulse_alt:

                        GPIO.output(
                            alt_step_pin,
                            GPIO.LOW,
                        )

                    if pulse_az:

                        GPIO.output(
                            az_step_pin,
                            GPIO.LOW,
                        )

                    time.sleep(pulse_delay)

    # =====================================================
    # Validation
    # =====================================================

    def _require_initialized(self) -> None:

        if not self.initialized:

            raise RuntimeError(
                "MotorDriver is not initialized. "
                "Call initialize() first."
            )

    # =====================================================
    # Cleanup
    # =====================================================

    def cleanup(self) -> None:

        if not self.initialized:
            return

        self.disable()

        # STEP과 DIR을 LOW로 정리
        for pin in (
            config.MOTOR_BASE_STEP_PIN,
            config.MOTOR_BASE_DIR_PIN,
            config.MOTOR_MOUNT_STEP_PIN,
            config.MOTOR_MOUNT_DIR_PIN,
        ):

            try:
                GPIO.output(pin, GPIO.LOW)

            except Exception:
                pass

        time.sleep(0.05)

        try:

            GPIO.cleanup(
                self._configured_pins
            )

        except Exception as error:

            print(
                f"[MotorDriver] GPIO Cleanup Warning: "
                f"{error}"
            )

        self._configured_pins.clear()
        self.initialized = False

        print("[MotorDriver] GPIO Cleanup Complete")


        