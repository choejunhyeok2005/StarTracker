"""Continuous upper-IMU EL feedback. No IMU-derived AZ is invented."""
import math
import time

import config
from controllers.position_pid import PositionPID, angular_error


class IMUTrackingController:
    def __init__(self, sensor, telemetry=None, clock=time.monotonic, initial_bias=0.0):
        self.sensor, self.telemetry, self.clock = sensor, telemetry, clock
        self.pid = PositionPID(**config.IMU_TRACKING_PID)
        self.fault = None
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

    def position(self):
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
            if self.last_raw is None or elapsed > config.IMU_TRACKING_MAX_GAP:
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
        if not 0 < dt <= config.IMU_TRACKING_MAX_GAP:
            raise RuntimeError("IMU control loop deadline exceeded")
        self.last_control_time = now
        self.control_dt = dt
        alt = self.position()
        motor.current_alt = alt
        margin = config.IMU_TRACKING_LIMIT_MARGIN_DEG
        if not config.MIN_ALTITUDE - margin <= alt <= config.MAX_ALTITUDE + margin:
            raise RuntimeError("IMU EL outside tracking limits")
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
