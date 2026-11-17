"""Encoder feedback for the existing synchronous STEP/DIR tracking loop."""
import math
import time

import config
from controllers.position_pid import PositionPID, angular_error


class EncoderFeedback:
    def __init__(self, encoder, telemetry=None, clock=time.monotonic):
        self.encoder, self.telemetry, self.clock = encoder, telemetry, clock
        self.offset = None
        self.last_time = None
        self.last_sample_time = None
        self.start_time = clock()
        self.fault = None
        self.pid = {a: PositionPID(**config.POSITION_PID[a]) for a in ("ALT", "AZ")}
        self.reset()

    def reset(self):
        for pid in self.pid.values():
            pid.reset()
        self.last_time = None
        self.remainder = {"ALT": 0.0, "AZ": 0.0}
        self.watch = {"ALT": None, "AZ": None}

    def sample(self):
        if self.fault:
            raise RuntimeError(self.fault)
        try:
            sample = self.encoder.read()
            now = self.clock()
            if not all(math.isfinite(v) for v in (sample.alt, sample.az, sample.timestamp)):
                raise RuntimeError("Nonfinite encoder sample")
            if not 0 <= now - sample.timestamp <= config.ENCODER_MAX_SAMPLE_AGE:
                raise RuntimeError("Stale or future encoder timestamp")
            if self.last_sample_time is not None and sample.timestamp < self.last_sample_time:
                raise RuntimeError("Encoder clock moved backwards")
            self.last_sample_time = sample.timestamp
            return sample
        except Exception as error:
            self.fault = str(error)
            raise

    def reference(self, alt, az):
        if not all(math.isfinite(v) for v in (alt, az)):
            raise ValueError("Reference must be finite")
        sample = self.sample()
        self.offset = (alt - sample.alt, az - sample.az)
        self.reset()

    def shift_reference(self, alt, az):
        if self.offset is None:
            raise RuntimeError("Encoder reference has not been established")
        if not all(math.isfinite(v) for v in (alt, az)):
            raise ValueError("Reference offset must be finite")
        self.offset = (self.offset[0] + alt, self.offset[1] + az)
        self.reset()

    def position(self):
        sample = self.sample()
        if self.offset is None:
            raise RuntimeError("Set encoder reference using alignment/plate solving first")
        return sample.alt + self.offset[0], (sample.az + self.offset[1]) % 360.0

    def update(self, motor):
        now = self.clock()
        dt = config.TRACKING_INTERVAL if self.last_time is None else now - self.last_time
        if dt <= 0:
            return
        if dt > config.ENCODER_MAX_CONTROL_GAP:
            raise RuntimeError("Encoder control loop deadline exceeded")
        self.last_time = now
        alt, az = self.position()
        motor.current_alt, motor.current_az = alt, az
        if not config.MIN_ALTITUDE <= alt <= config.MAX_ALTITUDE:
            raise RuntimeError("Measured ALT outside tracking limits")
        errors = {"ALT": motor.target_alt - alt, "AZ": angular_error(motor.target_az, az)}
        positions = {"ALT": alt, "AZ": az}
        speeds, steps = {}, {}
        # Do not accumulate integration against a lower-level step-rate limit.
        speed_cap = min(config.MAX_STEP_PER_UPDATE * motor.step_angle / dt,
                        motor.step_angle / (2 * config.MOTOR_PULSE_DELAY))
        for axis in ("ALT", "AZ"):
            pid = self.pid[axis]
            pid.max_speed = min(config.POSITION_PID[axis]["max_speed"], speed_cap)
            speed = pid.update(errors[axis], positions[axis], dt, circular=axis == "AZ")
            if abs(errors[axis]) <= pid.tolerance:
                self.remainder[axis] = 0.0
            demand = speed * dt / motor.step_angle + self.remainder[axis]
            count = math.trunc(demand)
            count = max(-config.MAX_STEP_PER_UPDATE, min(config.MAX_STEP_PER_UPDATE, count))
            self.remainder[axis] = demand - count
            # Keep pulse rounding and integral braking inside ALT limits.
            if axis == "ALT":
                bound = config.MAX_ALTITUDE - alt if count > 0 else alt - config.MIN_ALTITUDE
                count = (1 if count >= 0 else -1) * min(abs(count), int(max(0, bound) / motor.step_angle))
            speeds[axis], steps[axis] = speed, count
            watch = self.watch[axis]
            if watch is not None:
                since, initial, commanded = watch
                moved = abs(angular_error(positions[axis], initial) if axis == "AZ"
                            else positions[axis] - initial)
                if moved >= config.ENCODER_STALL_MIN_MOVEMENT:
                    watch = None
                elif now - since >= config.ENCODER_STALL_TIMEOUT:
                    if commanded >= config.ENCODER_STALL_COMMAND_DEG:
                        raise RuntimeError(f"{axis} encoder did not respond to commanded movement")
                    watch = None
            if count:
                since, initial, commanded = watch or (now, positions[axis], 0.0)
                watch = (since, initial, commanded + abs(count) * motor.step_angle)
            self.watch[axis] = watch
        if motor.mode.name != "TRACKING":
            return
        motor.driver.drive_dual(
            alt_steps=abs(steps["ALT"]),
            alt_direction=motor._to_driver_direction("ALT", steps["ALT"] >= 0),
            az_steps=abs(steps["AZ"]),
            az_direction=motor._to_driver_direction("AZ", steps["AZ"] >= 0))
        # Do not add commanded steps to measured position. A later sample must
        # observe movement, including lost steps/backlash, independently.
        if self.telemetry:
            self.telemetry.record(time_s=now - self.start_time,
                                  target_alt=motor.target_alt, actual_alt=alt, error_alt=errors["ALT"],
                                  target_az=motor.target_az, actual_az=az, error_az=errors["AZ"],
                                  speed_alt=speeds["ALT"], speed_az=speeds["AZ"],
                                  steps_alt=steps["ALT"], steps_az=steps["AZ"])

    def close(self):
        try:
            self.encoder.close()
        finally:
            if self.telemetry:
                self.telemetry.close()
