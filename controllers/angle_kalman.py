"""Two-state Kalman estimator: angle [deg], gyro bias [deg/s].

F = [[1, -dt], [0, 1]], H = [1, 0]. Gyro is the input; gravity tilt
is the measurement. Joseph covariance update preserves symmetry/PSD.
"""
import math


class AngleBiasKalman:
    def __init__(self, q_angle, q_bias, r_angle, initial_bias_variance):
        values = (q_angle, q_bias, r_angle, initial_bias_variance)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Kalman variances must be finite and positive")
        self.q_angle, self.q_bias, self.r_angle = q_angle, q_bias, r_angle
        self.initial_bias_variance = initial_bias_variance
        self.angle = None
        self.bias = 0.0
        self.innovation = self.gain = 0.0

    def initialize(self, angle, bias=0.0):
        if not all(math.isfinite(v) for v in (angle, bias)):
            raise ValueError("Invalid Kalman initial state")
        self.angle, self.bias = angle, bias
        self.p00, self.p01, self.p11 = self.r_angle, 0.0, self.initial_bias_variance
        self.innovation = self.gain = 0.0

    def update(self, angle_measurement, gyro_rate, dt):
        if self.angle is None:
            raise RuntimeError("Initialize Kalman estimator before updating")
        if not all(math.isfinite(v) for v in (angle_measurement, gyro_rate, dt)) or dt <= 0:
            raise ValueError("Kalman update needs finite values and positive dt")
        predicted = self.angle + dt * (gyro_rate - self.bias)
        # Continuous angle noise plus integrated bias random walk.
        a = self.p00 - 2 * dt * self.p01 + dt * dt * self.p11
        a += self.q_angle * dt + self.q_bias * dt ** 3 / 3
        b = self.p01 - dt * self.p11 - self.q_bias * dt * dt / 2
        c = self.p11 + self.q_bias * dt
        innovation = (angle_measurement - predicted + 180) % 360 - 180
        k0, k1 = a / (a + self.r_angle), b / (a + self.r_angle)
        self.angle = predicted + k0 * innovation
        self.bias += k1 * innovation
        u = 1 - k0
        self.p00 = u * u * a + k0 * k0 * self.r_angle
        self.p01 = u * (b - k1 * a) + k0 * k1 * self.r_angle
        self.p11 = c - 2 * k1 * b + k1 * k1 * (a + self.r_angle)
        self.innovation, self.gain = innovation, k0
        if not all(math.isfinite(v) for v in (self.angle, self.bias, self.p00, self.p01, self.p11)):
            raise RuntimeError("Kalman estimator became nonfinite")
        return self.angle
