"""
controllers/motor_controller.py

High Level Motor Controller

- Manual Control
- GOTO
- ALT/AZ Tracking
- Drift Correction

모든 모터 명령은 이 클래스를 통해 실행한다.
"""

import threading
from enum import Enum, auto

import config


class MotorMode(Enum):

    STOP = auto()
    ALIGN = auto()
    MANUAL = auto()
    TRACKING = auto()


class MotorController:

    def __init__(self, driver=None):

        if driver is None:
            # Import lazily so hardware-free controller tests do not require
            # Raspberry Pi GPIO libraries.
            from hardware.motor import MotorDriver

            driver = MotorDriver()

        self.driver = driver

        self.mode = MotorMode.STOP

        # 소프트웨어가 추정하는 현재 출력축 각도
        self.current_alt = 0.0
        self.current_az = 0.0

        # 목표 출력축 각도
        self.target_alt = 0.0
        self.target_az = 0.0

        self._alt_positive_direction = bool(
            getattr(config, "MOTOR_ALT_POSITIVE_DIRECTION", True)
        )
        self._az_positive_direction = bool(
            getattr(config, "MOTOR_AZ_POSITIVE_DIRECTION", True)
        )

        # Startup IMU alignment uses a relative coordinate because the
        # absolute ALT position is unknown until alignment succeeds.
        self.alignment_relative_steps = 0
        self._alignment_max_relative_steps = 0

        self.step_angle = config.STEP_ANGLE
        self.steps_per_degree = config.STEPS_PER_DEGREE

        # Tracking Thread와 Correction Thread의 동시 접근 방지
        self._motor_lock = threading.Lock()
        # 수동 조작 시 두 축을 독립적으로 움직이기 위한 Lock
        self._manual_alt_lock = threading.Lock()
        self._manual_az_lock = threading.Lock()

    # =====================================================
    # Initialize
    # =====================================================

    def initialize(self):

        self.driver.initialize()

        self.driver.set_mode(
            config.DEFAULT_MOTOR_MODE
        )

        self.driver.enable()

        self.mode = MotorMode.STOP

        print("[MotorController] Ready")

    # =====================================================
    # Manual Control
    # =====================================================

    def move_manual(
        self,
        axis: str,
        direction: bool,
        steps: int,
        pulse_delay: float = None,
    ):

        if self.mode == MotorMode.ALIGN:
            raise RuntimeError(
                "Manual movement is disabled during IMU alignment."
            )

        if steps <= 0:
            return

        axis = axis.upper()

        if axis == "ALT":
            axis_lock = self._manual_alt_lock

        elif axis == "AZ":
            axis_lock = self._manual_az_lock

        else:
            raise ValueError(
                "Axis must be 'ALT' or 'AZ'"
            )

        # ALT와 AZ는 서로 다른 Lock을 사용하므로
        # 조이스틱 대각선 입력 시 두 축이 동시에 움직일 수 있다.
        with axis_lock:

            # 소프트웨어 위치만 제한하면 실제 축은 0/90도 경계를 넘어
            # 계속 움직일 수 있다. 따라서 모터를 구동하기 전에 남은
            # 이동 범위로 스텝 수를 제한한다.
            if axis == "ALT":

                if direction:
                    remaining_angle = (
                        config.MAX_ALTITUDE
                        - self.current_alt
                    )
                else:
                    remaining_angle = (
                        self.current_alt
                        - config.MIN_ALTITUDE
                    )

                allowed_steps = int(
                    max(0.0, remaining_angle)
                    / self.step_angle
                )

                steps = min(steps, allowed_steps)

                if steps <= 0:
                    return

            self.mode = MotorMode.MANUAL

            self.driver.drive(
                axis=axis,
                direction=self._to_driver_direction(
                    axis,
                    direction,
                ),
                steps=steps,
                pulse_delay=pulse_delay,
            )

            moved_angle = self.steps_to_angle(
                steps
            )

            if axis == "ALT":

                if direction:
                    self.current_alt += moved_angle
                else:
                    self.current_alt -= moved_angle

                self.current_alt = (
                    self._clamp_altitude(
                        self.current_alt
                    )
                    )
            

            else:

                if direction:
                    self.current_az += moved_angle
                else:
                    self.current_az -= moved_angle

                self.current_az = (
                    self._normalize_azimuth(
                        self.current_az
                    )
                )

    # =====================================================
    # Startup IMU Alignment
    # =====================================================

    def begin_alignment(self, max_relative_steps: int) -> None:
        """Enter bounded relative ALT movement before absolute zero exists."""

        max_relative_steps = int(max_relative_steps)

        if max_relative_steps <= 0:
            raise ValueError("max_relative_steps must be positive.")

        with self._motor_lock:
            self.mode = MotorMode.ALIGN
            self.alignment_relative_steps = 0
            self._alignment_max_relative_steps = max_relative_steps

    def move_alignment(
        self,
        direction: bool,
        steps: int,
        pulse_delay: float = None,
    ) -> None:
        """Move only the upper ALT axis within the relative safety bound."""

        if self.mode != MotorMode.ALIGN:
            raise RuntimeError("MotorController is not in ALIGN mode.")

        steps = int(steps)

        if steps <= 0:
            return

        signed_steps = steps if direction else -steps
        next_relative_steps = (
            self.alignment_relative_steps + signed_steps
        )

        if (
            abs(next_relative_steps)
            > self._alignment_max_relative_steps
        ):
            raise RuntimeError(
                "ALT alignment relative-travel limit exceeded."
            )

        with self._manual_alt_lock:
            self.driver.drive(
                axis="ALT",
                direction=bool(direction),
                steps=steps,
                pulse_delay=pulse_delay,
            )
            self.alignment_relative_steps = next_relative_steps

    def finish_alignment(
        self,
        positive_direction_is_true: bool = None,
    ) -> None:
        """Commit the aligned physical position as the ALT zero reference."""

        if self.mode != MotorMode.ALIGN:
            raise RuntimeError("MotorController is not in ALIGN mode.")

        with self._motor_lock:
            if positive_direction_is_true is not None:
                self._alt_positive_direction = bool(
                    positive_direction_is_true
                )
            self.current_alt = 0.0
            self.target_alt = 0.0
            self.target_az = 0.0
            self.alignment_relative_steps = 0
            self._alignment_max_relative_steps = 0
            self.mode = MotorMode.STOP

    def abort_alignment(self) -> None:
        """Leave ALIGN mode without claiming a valid physical ALT zero."""

        with self._motor_lock:
            self.alignment_relative_steps = 0
            self._alignment_max_relative_steps = 0
            self.mode = MotorMode.STOP

    # =====================================================
    # Tracking Target
    # =====================================================

    def set_target(
        self,
        alt: float,
        az: float,
    ):

        if self.mode == MotorMode.ALIGN:
            raise RuntimeError(
                "Tracking target cannot be set during IMU alignment."
            )

        self.mode = MotorMode.TRACKING

        self.target_alt = self._clamp_altitude(alt)
        self.target_az = self._normalize_azimuth(az)

    # 이전 코드와의 호환용
    def goto(
        self,
        alt: float,
        az: float,
    ):

        self.set_target(alt, az)

    # =====================================================
    # Current Position Synchronization
    # =====================================================

    def set_current_position(
        self,
        alt: float,
        az: float,
    ):

        with self._motor_lock:

            self.current_alt = self._clamp_altitude(alt)
            self.current_az = self._normalize_azimuth(az)

    # =====================================================
    # Tracking Update
    # =====================================================

    def update(self):

        if self.mode != MotorMode.TRACKING:
            return

        with self._motor_lock:

            alt_error = (
                self.target_alt
                - self.current_alt
            )

            # 0/360도 경계를 고려한 최단 AZ 오차
            az_error = self._shortest_azimuth_error(
                target_az=self.target_az,
                current_az=self.current_az,
            )

            alt_steps = 0
            az_steps = 0

            alt_direction = alt_error >= 0
            az_direction = az_error >= 0

            if abs(alt_error) >= config.ANGLE_TOLERANCE:

                requested_alt_steps = self.angle_to_steps(
                    abs(alt_error)
                )

                alt_steps = min(
                    requested_alt_steps,
                    config.MAX_STEP_PER_UPDATE,
                )

            if abs(az_error) >= config.ANGLE_TOLERANCE:

                requested_az_steps = self.angle_to_steps(
                    abs(az_error)
                )

                az_steps = min(
                    requested_az_steps,
                    config.MAX_STEP_PER_UPDATE,
                )

            if alt_steps == 0 and az_steps == 0:
                return

            self.driver.drive_dual(
                alt_steps=alt_steps,
                alt_direction=self._to_driver_direction(
                    "ALT",
                    alt_direction,
                ),
                az_steps=az_steps,
                az_direction=self._to_driver_direction(
                    "AZ",
                    az_direction,
                ),
            )

            self._apply_step_position_update(
                alt_steps=alt_steps,
                alt_direction=alt_direction,
                az_steps=az_steps,
                az_direction=az_direction,
            )

    # =====================================================
    # Internal Position Update
    # =====================================================

    def _apply_step_position_update(
        self,
        alt_steps: int,
        alt_direction: bool,
        az_steps: int,
        az_direction: bool,
    ):

        if alt_steps > 0:

            moved_alt = self.steps_to_angle(alt_steps)

            if alt_direction:
                self.current_alt += moved_alt
            else:
                self.current_alt -= moved_alt

            self.current_alt = self._clamp_altitude(
                self.current_alt
            )

        if az_steps > 0:

            moved_az = self.steps_to_angle(az_steps)

            if az_direction:
                self.current_az += moved_az
            else:
                self.current_az -= moved_az

            self.current_az = self._normalize_azimuth(
                self.current_az
            )

    # =====================================================
    # Target Check
    # =====================================================

    def reached_target(self):

        alt_error = abs(
            self.target_alt
            - self.current_alt
        )

        az_error = abs(
            self._shortest_azimuth_error(
                target_az=self.target_az,
                current_az=self.current_az,
            )
        )

        return (
            alt_error < config.ANGLE_TOLERANCE
            and
            az_error < config.ANGLE_TOLERANCE
        )

    # =====================================================
    # Stop
    # =====================================================

    def stop(self):

        self.mode = MotorMode.STOP

    # =====================================================
    # Angle / Step Conversion
    # =====================================================

    def angle_to_steps(
        self,
        angle: float,
    ) -> int:

        if angle <= 0:
            return 0

        return round(
            angle * self.steps_per_degree
        )

    def steps_to_angle(
        self,
        steps: int,
    ) -> float:

        return steps * self.step_angle

    # =====================================================
    # Angle Utilities
    # =====================================================

    @staticmethod
    def _normalize_azimuth(az: float) -> float:

        return az % 360.0

    @staticmethod
    def _clamp_altitude(alt: float) -> float:

        return max(
            config.MIN_ALTITUDE,
            min(config.MAX_ALTITUDE, alt),
        )

    @staticmethod
    def _shortest_azimuth_error(
        target_az: float,
        current_az: float,
    ) -> float:

        return (
            target_az
            - current_az
            + 180.0
        ) % 360.0 - 180.0

    def _to_driver_direction(
        self,
        axis: str,
        positive_direction: bool,
    ) -> bool:
        """Map logical angle direction to the electrical DIR pin value."""

        axis = axis.upper()

        if axis == "ALT":
            electrical_positive = self._alt_positive_direction
        elif axis == "AZ":
            electrical_positive = self._az_positive_direction
        else:
            raise ValueError("Axis must be 'ALT' or 'AZ'.")

        return (
            electrical_positive
            if positive_direction
            else not electrical_positive
        )

    # =====================================================
    # Cleanup
    # =====================================================

    def cleanup(self):

        self.mode = MotorMode.STOP
        self.alignment_relative_steps = 0
        self._alignment_max_relative_steps = 0

        self.driver.cleanup()
