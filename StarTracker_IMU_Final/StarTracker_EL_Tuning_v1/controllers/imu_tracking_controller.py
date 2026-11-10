"""Continuous upper-IMU EL feedback. No IMU-derived AZ is invented."""
import math
import time

import config
from hardware.imu import IMUReadDeadlineError
from controllers.position_pid import PositionPID, angular_error


class IMUTrackingController:
    def __init__(self, sensor, telemetry=None, clock=time.monotonic, initial_bias=0.0):
        self.sensor, self.telemetry, self.clock = sensor, telemetry, clock
        self.pid = PositionPID(**config.IMU_TRACKING_PID)
        self.fault = None
        self.recovery_started = None
        self.recovery_attempts = 0
        self.recovery_good = 0
        self.deadline_events = 0
        self.recovery_successes = 0
        self.recovery_failures = 0
        self.recovery_total_seconds = 0.0
        self.recovery_callback = None
        self.offset = None
        self.last_raw = self.filtered = None
        self.last_sample_time = None
        self.last_sample = None
        self.filter_mode = config.IMU_FILTER_MODE
        if self.filter_mode not in ("kalman", "lowpass"):
            raise ValueError("IMU_FILTER_MODE must be kalman or lowpass")
        self.kalman = None
        if self.filter_mode == "kalman":
            from controllers.angle_kalman import AngleBiasKalman
            self.kalman = AngleBiasKalman(config.IMU_KALMAN_Q_ANGLE, config.IMU_KALMAN_Q_BIAS,
                                         config.IMU_KALMAN_R_ANGLE, config.IMU_KALMAN_INITIAL_BIAS_VARIANCE)
            self.kalman.bias = initial_bias
        self.start_time = clock()
        self.reset()

    def reset(self):
        self.pid.reset()
        self.last_control_time = None
        self.remainder = 0.0
        self.watch = None

    def position(self, *, recover_deadline=False):
        if self.fault:
            raise RuntimeError(self.fault)
        try:
            sample = self.sensor.read()
            now = self.clock()
            if not all(math.isfinite(v) for v in (sample.angle, sample.timestamp)):
                raise RuntimeError("Nonfinite IMU sample")
            if self.kalman and (sample.gyro_rate is None or not math.isfinite(sample.gyro_rate)
                                or abs(sample.gyro_rate) > config.IMU_GYRO_MAX_RATE_DPS):
                raise RuntimeError("Kalman control requires a valid gyro rate")
            if not 0 <= now - sample.timestamp <= config.IMU_TRACKING_MAX_READ_SECONDS:
                raise RuntimeError("Stale or future IMU sample")
            if self.last_sample_time is not None and sample.timestamp < self.last_sample_time:
                raise RuntimeError("IMU sample clock moved backwards")
            elapsed = None if self.last_sample_time is None else sample.timestamp - self.last_sample_time
            recovering = self.recovery_started is not None
            if recovering and self.last_raw is not None:
                if sample.timestamp <= self.last_sample_time:
                    raise RuntimeError("Recovery requires a new IMU sample")
                if abs(angular_error(sample.angle, self.last_raw)) > config.IMU_TRACKING_MAX_JUMP_DEG:
                    raise RuntimeError("IMU tilt jumped unexpectedly during recovery")
            if recovering or self.last_raw is None or elapsed > config.IMU_TRACKING_MAX_GAP:
                # After manual/idle movement, start a fresh filter state.
                raw = sample.angle if self.last_raw is None else self.last_raw + angular_error(sample.angle, self.last_raw)
                self.filtered = raw
                if self.kalman:
                    # No gyro integration across an unobserved idle interval.
                    # Keep the learned bias, reset angle/covariance from gravity.
                    self.kalman.initialize(raw, self.kalman.bias)
            else:
                delta = angular_error(sample.angle, self.last_raw)
                if abs(delta) > config.IMU_TRACKING_MAX_JUMP_DEG:
                    raise RuntimeError("IMU tilt jumped unexpectedly")
                raw = self.last_raw + delta
                if self.kalman:
                    if elapsed > 0:
                        self.filtered = self.kalman.update(raw, sample.gyro_rate, elapsed)
                    # Identical timestamps do not add duplicate evidence.
                else:
                    weight = elapsed / (config.IMU_TRACKING_FILTER_TAU + elapsed) if elapsed > 0 else 0.0
                    self.filtered += weight * (raw - self.filtered)
            self.last_raw, self.last_sample_time = raw, sample.timestamp
            self.last_sample = sample
            if self.offset is None:
                raise RuntimeError("IMU EL reference is not established")
            return self.filtered + self.offset
        except IMUReadDeadlineError as error:
            if not recover_deadline:
                self.fault = str(error)
            raise
        except Exception as error:
            self.fault = str(error)
            raise

    def reference(self, alt):
        if not math.isfinite(alt):
            raise ValueError("IMU reference must be finite")
        # Re-anchor the coordinate mapping, never integrate commanded steps.
        self.offset = 0.0
        # A plate solution changes only the coordinate offset. Preserve the
        # learned gyro bias and covariance during a running observation.
        angle = self.position()
        self.offset = alt - angle
        self.reset()

    def shift_reference(self, delta):
        if self.offset is None or not math.isfinite(delta):
            raise ValueError("Invalid IMU reference shift")
        self.offset += delta
        self.reset()

    def steps(self, motor):
        now = self.clock()
        dt = config.TRACKING_INTERVAL if self.last_control_time is None else now - self.last_control_time
        recovering = self.recovery_started is not None
        if recovering:
            if self.clock() - self.recovery_started > config.IMU_DEADLINE_RECOVERY_MAX_SECONDS:
                raise RuntimeError("IMU deadline recovery time limit exceeded")
            self.recovery_attempts += 1
        if not recovering and not 0 < dt <= config.IMU_TRACKING_MAX_GAP:
            raise RuntimeError("IMU control loop deadline exceeded")
        self.last_control_time = now
        self.control_dt = dt
        try:
            alt = self.position(recover_deadline=True)
        except IMUReadDeadlineError as error:
            if not recovering:
                self.recovery_started = self.clock()
                self.recovery_attempts = 0
                self.deadline_events += 1
                print(f"[IMU] Deadline #{self.deadline_events}; both axes paused: {error}")
                self._emit_recovery("started", error=str(error))
            self.recovery_good = 0
            try:
                self._check_recovery_budget()
            except RuntimeError as failure:
                self._finish_recovery(False, str(failure))
                raise
            return None, 0.0
        motor.current_alt = alt
        margin = config.IMU_TRACKING_LIMIT_MARGIN_DEG
        if not config.MIN_ALTITUDE - margin <= alt <= config.MAX_ALTITUDE + margin:
            raise RuntimeError("IMU EL outside tracking limits")
        if recovering:
            self.recovery_good += 1
            if self.clock() - self.recovery_started > config.IMU_DEADLINE_RECOVERY_MAX_SECONDS:
                failure = "IMU deadline recovery time limit exceeded"
                self._finish_recovery(False, failure)
                raise RuntimeError(failure)
            if self.recovery_good >= config.IMU_DEADLINE_RECOVERY_GOOD_SAMPLES:
                paused = self.clock() - self.recovery_started
                watch = self.watch
                self.reset()
                # Exclude the deliberate pause, but retain accumulated stall evidence.
                if watch:
                    since, initial, commanded = watch
                    self.watch = (since + paused, initial, commanded)
                self.recovery_successes += 1
                attempts = self.recovery_attempts
                self.recovery_total_seconds += paused
                self.recovery_started = None
                message = (f"recovered in {paused:.3f}s after {attempts} retry reads; "
                           f"successes={self.recovery_successes}, failures={self.recovery_failures}")
                print(f"[IMU] Deadline #{self.deadline_events} {message}; resumes next update")
                self._emit_recovery("succeeded", duration_seconds=paused,
                                    retry_reads=attempts, message=message)
            else:
                try:
                    self._check_recovery_budget()
                except RuntimeError as failure:
                    self._finish_recovery(False, str(failure))
                    raise
            # Never send pulses on a recovery sample, including the final one.
            return None, 0.0
        error = motor.target_alt - alt
        self.pid.max_speed = min(config.IMU_TRACKING_PID["max_speed"],
                                 config.MAX_STEP_PER_UPDATE * motor.step_angle / dt,
                                 motor.step_angle / (2 * config.MOTOR_PULSE_DELAY))
        speed = self.pid.update(error, alt, dt)
        if abs(error) <= self.pid.tolerance:
            self.remainder = 0.0
        demand = speed * dt / motor.step_angle + self.remainder
        steps = max(-config.MAX_STEP_PER_UPDATE, min(config.MAX_STEP_PER_UPDATE, math.trunc(demand)))
        self.remainder = demand - steps
        bound = config.MAX_ALTITUDE - alt if steps > 0 else alt - config.MIN_ALTITUDE
        steps = (1 if steps >= 0 else -1) * min(abs(steps), int(max(0, bound) / motor.step_angle))
        if self.watch:
            since, initial, commanded = self.watch
            if abs(alt - initial) >= config.IMU_TRACKING_STALL_MOVEMENT_DEG:
                self.watch = None
            elif now - since >= config.IMU_TRACKING_STALL_SECONDS:
                if commanded >= config.IMU_TRACKING_STALL_COMMAND_DEG:
                    raise RuntimeError("IMU EL did not respond to motor commands")
                self.watch = None
        if steps:
            since, initial, commanded = self.watch or (now, alt, 0.0)
            self.watch = (since, initial, commanded + abs(steps) * motor.step_angle)
        return steps, speed

    def _emit_recovery(self, outcome, **details):
        if self.recovery_callback:
            try:
                self.recovery_callback({"outcome": outcome,
                    "deadline_event": self.deadline_events,
                    "retry_reads": self.recovery_attempts,
                    "recovery_successes": self.recovery_successes,
                    "recovery_failures": self.recovery_failures, **details})
            except Exception as error:
                print(f"[IMU] Recovery log warning: {error}")

    def _finish_recovery(self, succeeded, reason):
        if self.recovery_started is None:
            return
        duration = max(0.0, self.clock() - self.recovery_started)
        self.recovery_total_seconds += duration
        if not succeeded:
            self.recovery_failures += 1
        print(f"[IMU] Deadline #{self.deadline_events} recovery FAILED after "
              f"{duration:.3f}s and {self.recovery_attempts} retry reads: {reason}")
        self._emit_recovery("failed", duration_seconds=duration, reason=reason)
        self.recovery_started = None

    def _check_recovery_budget(self):
        if (self.recovery_attempts >= config.IMU_DEADLINE_RECOVERY_ATTEMPTS
                or self.clock() - self.recovery_started > config.IMU_DEADLINE_RECOVERY_MAX_SECONDS):
            raise RuntimeError("IMU deadline recovery failed; retry budget exhausted")

    def record(self, motor, alt_steps, alt_speed, az_steps):
        if self.telemetry:
            diagnostics = {"accel_alt": self.last_raw + self.offset}
            if self.kalman:
                diagnostics.update(gyro_rate=self.last_sample.gyro_rate, gyro_bias=self.kalman.bias,
                                   innovation_deg=self.kalman.innovation,
                                   angle_variance=self.kalman.p00, kalman_gain=self.kalman.gain)
            self.telemetry.record(time_s=self.clock() - self.start_time,
                                  target_alt=motor.target_alt, actual_alt=motor.current_alt,
                                  error_alt=motor.target_alt - motor.current_alt,
                                  target_az=motor.target_az, actual_az=motor.current_az,
                                  error_az=angular_error(motor.target_az, motor.current_az),
                                  speed_alt=alt_speed, speed_az=az_steps * motor.step_angle / self.control_dt,
                                  steps_alt=alt_steps, steps_az=az_steps, **diagnostics)

    def close(self):
        if self.telemetry:
            self.telemetry.close()
