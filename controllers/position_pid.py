"""Bounded position PID: degrees in, degrees/second out."""
import math


def angular_error(target, actual):
    return (target - actual + 180.0) % 360.0 - 180.0


class PositionPID:
    def __init__(self, kp, ki, kd, max_speed, max_acceleration, tolerance):
        values = (kp, ki, kd, max_speed, max_acceleration, tolerance)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("PID settings must be finite and nonnegative.")
        if max_speed == 0 or max_acceleration == 0:
            raise ValueError("Speed and acceleration must be positive.")
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_speed, self.max_acceleration = max_speed, max_acceleration
        self.tolerance = tolerance
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = None
        self.speed = 0.0

    def update(self, error, measurement, dt, circular=False):
        if not all(math.isfinite(v) for v in (error, measurement, dt)) or dt <= 0:
            raise ValueError("PID requires finite inputs and positive dt.")
        delta = 0.0 if self.previous_measurement is None else (
            angular_error(measurement, self.previous_measurement) if circular
            else measurement - self.previous_measurement)
        self.previous_measurement = measurement
        if abs(error) <= self.tolerance:
            self.integral = self.speed = 0.0
            return 0.0
        candidate = self.integral + error * dt
        raw = self.kp * error + self.ki * candidate - self.kd * delta / dt
        # Conditional integration also accounts for acceleration saturation.
        low = self.speed - self.max_acceleration * dt
        high = self.speed + self.max_acceleration * dt
        # A newly reduced hard speed limit takes priority over the slew limit.
        speed = max(-self.max_speed, min(self.max_speed, max(low, min(high, raw))))
        if raw == speed or (raw - speed) * error < 0:
            self.integral = candidate
        self.speed = speed
        return speed
