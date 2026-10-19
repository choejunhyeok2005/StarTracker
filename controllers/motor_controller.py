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
import math
import time
from enum import Enum, auto

import config


class MotorMode(Enum):

    STOP = auto()
    ALIGN = auto()
    MANUAL = auto()
    TRACKING = auto()


class MotorController:

    def __init__(self, driver=None, encoder=None, telemetry=None, clock=time.monotonic):

        if driver is None:
            # Import lazily so hardware-free controller tests do not require
            # Raspberry Pi GPIO libraries.
            from hardware.motor import MotorDriver

            driver = MotorDriver()

        self.driver = driver
        self.feedback = None
        self.imu_feedback = None
        self._encoder_pending = encoder is None and getattr(config, "ENCODER_ENABLED", False)
        self._feedback_clock, self._feedback_telemetry = clock, telemetry
        if encoder is not None:
            self._create_feedback(encoder)

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

        if self._encoder_pending:
            from hardware.encoder import configured_encoder
            self._create_feedback(configured_encoder())
            self._encoder_pending = False
        self.driver.initialize()

        self.driver.set_mode(
            config.DEFAULT_MOTOR_MODE
        )

        self.driver.enable()

        self.mode = MotorMode.STOP

        print("[MotorController] Ready")

    def _create_feedback(self, encoder):
        from controllers.encoder_feedback import EncoderFeedback
        telemetry = self._feedback_telemetry
        try:
            if telemetry is None and config.CONTROL_TELEMETRY_ENABLED:
                from datetime import datetime
                from services.control_telemetry import ControlTelemetry
                telemetry = ControlTelemetry(
                    f"data/logs/control_{datetime.now():%Y%m%d_%H%M%S_%f}.csv")
            self.feedback = EncoderFeedback(encoder, telemetry, self._feedback_clock)
        except Exception:
            encoder.close()
            if telemetry:
                telemetry.close()
            raise

    def enable_imu_tracking(self, imu, sensor=None):
        """Attach the already-initialized upper MPU6050 after startup alignment."""
        if self.feedback or self._encoder_pending:
            raise ValueError("Select either encoder feedback or IMU EL feedback")
        if self.imu_feedback:
            return
        if config.IMU_FILTER_MODE not in ("kalman", "lowpass"):
            raise ValueError("IMU_FILTER_MODE must be kalman or lowpass")
        from controllers.imu_tracking_controller import IMUTrackingController
        from hardware.imu import IMUAltitudeSensor
        selected_sensor = sensor or IMUAltitudeSensor(imu, self._feedback_clock)
        initial_bias = 0.0
        try:
            if config.IMU_FILTER_MODE == "kalman":
                initial_bias = selected_sensor.calibrate().bias
        except Exception:
            self.stop()
            raise
        telemetry = self._feedback_telemetry
        if telemetry is None and config.CONTROL_TELEMETRY_ENABLED:
            from datetime import datetime
            from services.control_telemetry import ControlTelemetry
            telemetry = ControlTelemetry(f"data/logs/imu_control_{datetime.now():%Y%m%d_%H%M%S_%f}.csv")
        try:
            feedback = IMUTrackingController(
                selected_sensor, telemetry, self._feedback_clock, initial_bias=initial_bias)
            feedback.reference(self.current_alt)
        except Exception:
            if telemetry:
                telemetry.close()
            self.stop()
            raise
        self.imu_feedback = feedback

    def _imu_position(self):
        try:
            return self.imu_feedback.position()
        except Exception:
            self.stop()
            raise

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

            if self.imu_feedback:
                with self._motor_lock:
                    self._move_manual_imu(axis, direction, steps, pulse_delay)
                return
            if self.feedback:
                with self._motor_lock:
                    self._move_manual_feedback(axis, direction, steps, pulse_delay)
                return

            manual_alt_limits_enabled = bool(
                getattr(
                    config,
                    "MANUAL_ALT_SOFT_LIMITS_ENABLED",
                    True,
                )
            )

            # 소프트웨어 위치만 제한하면 실제 축은 0/90도 경계를 넘어
            # 계속 움직일 수 있다. 따라서 모터를 구동하기 전에 남은
            # 이동 범위로 스텝 수를 제한한다.
            if axis == "ALT" and manual_alt_limits_enabled:

                if direction:
                    remaining_angle = (
                        config.MANUAL_MAX_ALTITUDE
                        - self.current_alt
                    )
                else:
                    remaining_angle = (
                        self.current_alt
                        - config.MANUAL_MIN_ALTITUDE
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

                if manual_alt_limits_enabled:
                    self.current_alt = max(
                        config.MANUAL_MIN_ALTITUDE,
                        min(
                            config.MANUAL_MAX_ALTITUDE,
                            self.current_alt,
                        ),
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

            if self.feedback:
                self.feedback.reference(0.0, self.current_az)

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

        if not all(math.isfinite(v) for v in (alt, az)):
            raise ValueError("Target angles must be finite")
        with self._motor_lock:
            if self.feedback and self.mode != MotorMode.TRACKING:
                self.feedback.reset()
            if self.imu_feedback and self.mode != MotorMode.TRACKING:
                self.imu_feedback.reset()
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

            if self.imu_feedback:
                try:
                    self.imu_feedback.reference(alt)
                except Exception:
                    self.stop()
                    raise
            if self.feedback:
                self.feedback.reference(alt, az)
            self.current_alt = self._clamp_altitude(alt)
            self.current_az = self._normalize_azimuth(az)

    # =====================================================
    # Tracking Update
    # =====================================================

    def apply_position_offset(self, alt_offset: float, az_offset: float):
        """Apply measured pointing error without losing subsequent tracking steps."""
        with self._motor_lock:
            if self.imu_feedback:
                self.imu_feedback.shift_reference(alt_offset)
                self.current_alt = self._imu_position()
                self.current_az = self._normalize_azimuth(self.current_az + az_offset)
                return
            if self.feedback:
                self.feedback.shift_reference(alt_offset, az_offset)
                self.current_alt, self.current_az = self.feedback.position()
                return
            self.current_alt = self._clamp_altitude(self.current_alt + alt_offset)
            self.current_az = self._normalize_azimuth(self.current_az + az_offset)

    def update(self):
        try:
            self._update_position()
        except Exception as error:
            if self.imu_feedback:
                self.imu_feedback.fault = str(error)
                self.stop()
            raise

    def _update_position(self):

        if self.mode != MotorMode.TRACKING:
            return

        with self._motor_lock:

            if self.feedback:
                try:
                    self.feedback.update(self)
                except Exception as error:
                    self.feedback.fault = str(error)
                    self.stop()
                    raise
                return

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
            imu_speed = 0.0

            alt_direction = alt_error >= 0
            az_direction = az_error >= 0

            if self.imu_feedback:
                signed_steps, imu_speed = self.imu_feedback.steps(self)
                alt_steps, alt_direction = abs(signed_steps), signed_steps >= 0
            elif abs(alt_error) >= config.ANGLE_TOLERANCE:

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

            if self.imu_feedback:
                self.imu_feedback.record(self, alt_steps if alt_direction else -alt_steps,
                                         imu_speed, az_steps if az_direction else -az_steps)
            if self.mode != MotorMode.TRACKING:
                return
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
                alt_steps=0 if self.imu_feedback else alt_steps,
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

        if self.imu_feedback:
            with self._motor_lock:
                self.current_alt = self._imu_position()
        if self.feedback:
            with self._motor_lock:
                self.current_alt, self.current_az = self.feedback.position()

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
            alt_error <= (self.feedback.pid["ALT"].tolerance if self.feedback
                          else self.imu_feedback.pid.tolerance if self.imu_feedback
                          else config.ANGLE_TOLERANCE)
            and
            az_error < (self.feedback.pid["AZ"].tolerance if self.feedback
                        else config.ANGLE_TOLERANCE)
        )

    # =====================================================
    # Stop
    # =====================================================

    def resume(self):
        if self.imu_feedback:
            if self.imu_feedback.fault:
                raise RuntimeError("IMU fault is latched; inspect hardware and restart")
            self.imu_feedback.reset()
        if self.feedback:
            if self.feedback.fault:
                raise RuntimeError("Encoder fault is latched; inspect hardware and restart")
            self.feedback.reset()
        resume_driver = getattr(self.driver, "resume", None)
        if resume_driver is not None:
            resume_driver()

        self.mode = MotorMode.STOP

    def stop(self):

        self.mode = MotorMode.STOP

        stop_driver = getattr(self.driver, "stop", None)
        if stop_driver is not None:
            stop_driver()

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

        try:
            self.driver.cleanup()
        finally:
            if self.feedback:
                self.feedback.close()
            if self.imu_feedback:
                self.imu_feedback.close()

    def _move_manual_imu(self, axis, direction, steps, pulse_delay):
        """Manual jog uses measured EL and existing step-based AZ, without holding a target."""
        self.current_alt = self._imu_position()
        steps = min(int(steps), config.MAX_STEP_PER_UPDATE)
        if axis == "ALT":
            bound = (config.MANUAL_MAX_ALTITUDE - self.current_alt if direction
                     else self.current_alt - config.MANUAL_MIN_ALTITUDE)
            steps = min(steps, int(max(0, bound) / self.step_angle))
        if steps <= 0:
            return
        self.mode = MotorMode.MANUAL
        self.imu_feedback.reset()
        try:
            self.driver.drive(axis=axis, direction=self._to_driver_direction(axis, direction),
                              steps=steps, pulse_delay=pulse_delay)
        except Exception as error:
            self.imu_feedback.fault = str(error)
            self.stop()
            raise
        if axis == "AZ":
            self.current_az = (self.current_az + self.steps_to_angle(steps) * (1 if direction else -1)) % 360

    def _move_manual_feedback(self, axis, direction, steps, pulse_delay):
        """Bound manual bursts using fresh encoder readings; no PID target hold."""
        try:
            self.current_alt, self.current_az = self.feedback.position()
            steps = min(int(steps), config.MAX_STEP_PER_UPDATE)
            if axis == "ALT":
                bound = (config.MANUAL_MAX_ALTITUDE - self.current_alt if direction
                         else self.current_alt - config.MANUAL_MIN_ALTITUDE)
                steps = min(steps, int(max(0, bound) / self.step_angle))
            if steps <= 0:
                return
            self.mode = MotorMode.MANUAL
            self.feedback.reset()
            self.driver.drive(axis=axis, direction=self._to_driver_direction(axis, direction),
                              steps=steps, pulse_delay=pulse_delay)
            self.current_alt, self.current_az = self.feedback.position()
        except Exception as error:
            self.feedback.fault = str(error)
            self.stop()
            raise
