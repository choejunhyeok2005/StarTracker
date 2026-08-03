"""Closed-loop initial alignment for the moving upper ALT plate."""

from __future__ import annotations

import time
from dataclasses import dataclass
from collections.abc import Callable


class IMUAlignmentError(RuntimeError):
    """Raised when startup alignment cannot be completed safely."""

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class AlignmentSettings:
    base_pitch_offset_deg: float
    upper_pitch_offset_deg: float
    filter_samples: int
    sample_interval_seconds: float
    tolerance_deg: float
    stable_reads_required: int
    direction_probe_steps: int
    min_probe_response_deg: float
    proportional_gain: float
    min_correction_steps: int
    max_correction_steps: int
    motor_pulse_delay: float
    settle_seconds: float
    timeout_seconds: float
    max_iterations: int
    max_relative_travel_steps: int
    max_worsening_count: int
    worsening_margin_deg: float
    steps_per_degree: float

    @classmethod
    def from_config(cls) -> "AlignmentSettings":
        """Build settings from the project's validated configuration."""

        import config

        return cls(
            base_pitch_offset_deg=(
                config.IMU_BASE_PITCH_OFFSET_DEG
            ),
            upper_pitch_offset_deg=(
                config.IMU_UPPER_PITCH_OFFSET_DEG
            ),
            filter_samples=config.IMU_FILTER_SAMPLES,
            sample_interval_seconds=(
                config.IMU_SAMPLE_INTERVAL_SECONDS
            ),
            tolerance_deg=config.IMU_LEVEL_TOLERANCE_DEG,
            stable_reads_required=(
                config.IMU_STABLE_READS_REQUIRED
            ),
            direction_probe_steps=(
                config.IMU_DIRECTION_PROBE_STEPS
            ),
            min_probe_response_deg=(
                config.IMU_MIN_PROBE_RESPONSE_DEG
            ),
            proportional_gain=config.IMU_PROPORTIONAL_GAIN,
            min_correction_steps=(
                config.IMU_MIN_CORRECTION_STEPS
            ),
            max_correction_steps=(
                config.IMU_MAX_CORRECTION_STEPS
            ),
            motor_pulse_delay=config.IMU_MOTOR_PULSE_DELAY,
            settle_seconds=config.IMU_SETTLE_SECONDS,
            timeout_seconds=(
                config.IMU_ALIGNMENT_TIMEOUT_SECONDS
            ),
            max_iterations=config.IMU_MAX_ALIGNMENT_ITERATIONS,
            max_relative_travel_steps=round(
                config.IMU_MAX_RELATIVE_TRAVEL_DEG
                * config.STEPS_PER_DEGREE
            ),
            max_worsening_count=config.IMU_MAX_WORSENING_COUNT,
            worsening_margin_deg=(
                config.IMU_WORSENING_MARGIN_DEG
            ),
            steps_per_degree=config.STEPS_PER_DEGREE,
        )


@dataclass(frozen=True)
class AlignmentResult:
    final_error_deg: float
    iterations: int
    movement_count: int
    cumulative_steps: int
    positive_direction_increases_pitch: bool | None
    elapsed_seconds: float


class IMULevelingController:
    """Make upper pitch match fixed-base pitch using ALT movement only."""

    def __init__(
        self,
        imu,
        motor,
        *,
        settings: AlignmentSettings,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.imu = imu
        self.motor = motor
        self.settings = settings
        self._clock = clock
        self._sleep = sleep

    def align(self) -> AlignmentResult:
        """Run alignment and commit ALT zero only after stable convergence."""

        started_at = self._clock()
        stable_reads = 0
        final_error = float("inf")
        movement_count = 0
        cumulative_steps = 0
        positive_direction_increases_pitch = None
        probe_baseline_error = None
        correction_baseline_error = None
        worsening_count = 0

        self.motor.begin_alignment(
            self.settings.max_relative_travel_steps
        )

        try:
            for iteration in range(1, self.settings.max_iterations + 1):
                if self._clock() - started_at > self.settings.timeout_seconds:
                    raise IMUAlignmentError(
                        "timeout",
                        "IMU alignment exceeded its time limit.",
                    )

                base_pitch, upper_pitch = (
                    self.imu.read_filtered_pitches(
                        sample_count=self.settings.filter_samples,
                        sample_interval=(
                            self.settings.sample_interval_seconds
                        ),
                    )
                )
                final_error = (
                    upper_pitch
                    - self.settings.upper_pitch_offset_deg
                    - (
                        base_pitch
                        - self.settings.base_pitch_offset_deg
                    )
                )

                if probe_baseline_error is not None:
                    probe_delta = final_error - probe_baseline_error

                    if (
                        abs(probe_delta)
                        < self.settings.min_probe_response_deg
                    ):
                        raise IMUAlignmentError(
                            "probe_unobservable",
                            "ALT direction probe produced no observable "
                            "pitch response.",
                        )

                    positive_direction_increases_pitch = (
                        probe_delta > 0.0
                    )
                    probe_baseline_error = None

                if correction_baseline_error is not None:
                    if (
                        abs(final_error)
                        > correction_baseline_error
                        + self.settings.worsening_margin_deg
                    ):
                        worsening_count += 1
                    else:
                        worsening_count = 0

                    correction_baseline_error = None

                    if (
                        worsening_count
                        >= self.settings.max_worsening_count
                    ):
                        raise IMUAlignmentError(
                            "worsening_error",
                            "Pitch error increased repeatedly after "
                            "ALT corrections.",
                        )

                if abs(final_error) <= self.settings.tolerance_deg:
                    stable_reads += 1

                    if stable_reads >= self.settings.stable_reads_required:
                        self.motor.finish_alignment(
                            positive_direction_is_true=(
                                positive_direction_increases_pitch
                            )
                        )
                        return AlignmentResult(
                            final_error_deg=final_error,
                            iterations=iteration,
                            movement_count=movement_count,
                            cumulative_steps=cumulative_steps,
                            positive_direction_increases_pitch=(
                                positive_direction_increases_pitch
                            ),
                            elapsed_seconds=self._clock() - started_at,
                        )

                    if self.settings.settle_seconds:
                        self._sleep(self.settings.settle_seconds)
                    continue

                stable_reads = 0

                if positive_direction_increases_pitch is None:
                    probe_baseline_error = final_error
                    steps = self.settings.direction_probe_steps
                    direction = True

                else:
                    requested_steps = round(
                        abs(final_error)
                        * self.settings.steps_per_degree
                        * self.settings.proportional_gain
                    )
                    steps = max(
                        self.settings.min_correction_steps,
                        min(
                            requested_steps,
                            self.settings.max_correction_steps,
                        ),
                    )

                    pitch_must_increase = final_error < 0.0
                    direction = (
                        pitch_must_increase
                        == positive_direction_increases_pitch
                    )

                    correction_baseline_error = abs(final_error)

                signed_steps = steps if direction else -steps

                if (
                    abs(cumulative_steps + signed_steps)
                    > self.settings.max_relative_travel_steps
                ):
                    raise IMUAlignmentError(
                        "travel_limit",
                        "Requested ALT correction exceeds the relative "
                        "alignment travel limit.",
                    )

                self.motor.move_alignment(
                    direction=direction,
                    steps=steps,
                    pulse_delay=self.settings.motor_pulse_delay,
                )
                movement_count += 1
                cumulative_steps += signed_steps

                if self.settings.settle_seconds:
                    self._sleep(self.settings.settle_seconds)

            raise IMUAlignmentError(
                "max_iterations",
                "IMU alignment reached its iteration limit.",
            )

        except Exception:
            self.motor.abort_alignment()
            raise
