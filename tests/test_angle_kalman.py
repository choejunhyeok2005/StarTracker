import math
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import config
from controllers.angle_kalman import AngleBiasKalman
from controllers.motor_controller import MotorController
from hardware.imu import IMUAltitudeSensor, IMUReadError, MPU6050Pair, AltitudeSample
from simulate_control import SimulatedMount
from simulate_imu_control import SimulatedIMU, run_demo
from tests.test_imu_hardware import FakeSMBus, encode_acceleration


class KalmanMathTests(unittest.TestCase):
    def make_filter(self):
        result = AngleBiasKalman(.01, .0001, .09, .01)
        result.initialize(0)
        return result

    def test_unknown_constant_bias_converges_without_angle_drift(self):
        k = self.make_filter()
        for i in range(2400):
            k.update(.1 * math.sin(i * .7), .2, .05)
        self.assertLess(abs(k.bias - .2), .01)
        self.assertLess(abs(k.angle), .03)

    def test_moving_angle_tracks_with_biased_gyro_and_noisy_acceleration(self):
        k = self.make_filter()
        errors, accel_errors = [], []
        for i in range(1, 1601):
            truth = i * .05 * .3
            measured = truth + .4 * math.sin(i * 1.3)
            estimate = k.update(measured, .3 + .12, .05)
            if i > 400:
                errors.append((estimate - truth) ** 2)
                accel_errors.append((measured - truth) ** 2)
        self.assertLess(sum(errors), sum(accel_errors) * .1)
        self.assertAlmostEqual(k.bias, .12, delta=.015)

    def test_wrap_has_small_innovation(self):
        k = self.make_filter()
        k.initialize(179.9)
        self.assertAlmostEqual(k.update(-179.9, 2, .1), 180.1)
        self.assertAlmostEqual(k.innovation, 0)

    def test_covariance_remains_positive_semidefinite_with_variable_dt(self):
        k = self.make_filter()
        for i in range(3000):
            k.update(math.sin(i / 50), .1, (.001, .02, .2)[i % 3])
            self.assertGreaterEqual(k.p00, 0)
            self.assertGreaterEqual(k.p11, 0)
            self.assertGreaterEqual(k.p00 * k.p11 - k.p01 ** 2, -1e-12)

    def test_invalid_parameters_and_time_rejected(self):
        with self.assertRaises(ValueError):
            AngleBiasKalman(.01, .001, 0, .01)
        for dt in (0, -.1, math.nan):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                self.make_filter().update(0, 0, dt)


class GyroHardwareTests(unittest.TestCase):
    def pair(self, blocks):
        bus = FakeSMBus({0x68: [], 0x69: blocks})
        pair = MPU6050Pair(bus_factory=lambda _: bus, sleep=lambda _: None, max_retries=2)
        pair.initialize()
        return pair, bus

    def test_burst_decodes_signed_values_and_skips_temperature(self):
        burst = encode_acceleration(100, -200, 16384) + [0xAA, 0xBB] + encode_acceleration(-131, 262, -393)
        pair, bus = self.pair([burst])
        accel, gyro = pair.read_motion(0x69)
        self.assertEqual(accel, (100, -200, 16384))
        self.assertEqual(gyro, (-131, 262, -393))
        self.assertIn((0x69, 0x1B, 0), bus.writes)
        self.assertIn(("block", 0x69, 0x3B, 14), bus.reads)

    def test_truncated_burst_retries_and_fails_closed(self):
        pair, bus = self.pair([[0] * 6, [0] * 8])
        with self.assertRaises(IMUReadError):
            pair.read_motion(0x69)
        self.assertEqual(sum(r[0] == "block" for r in bus.reads), 2)

    def test_adapter_converts_x_gyro_and_applies_sign(self):
        pair = SimpleNamespace(upper_address=0x69,
                               read_motion=Mock(return_value=((0, 0, 16384), (-131, 0, 0))))
        sensor = IMUAltitudeSensor(pair, clock=lambda: 0)
        self.assertEqual(sensor.read().gyro_rate, -1)
        with patch.object(config, "IMU_GYRO_X_SIGN", -1):
            self.assertEqual(sensor.read().gyro_rate, 1)

    def test_excessive_gyro_and_bad_gravity_rejected(self):
        for accel, gyro in (((0, 0, 16384), (32767, 0, 0)), ((0, 0, 0), (0, 0, 0))):
            pair = SimpleNamespace(upper_address=0x69, read_motion=Mock(return_value=(accel, gyro)))
            with self.assertRaises(IMUReadError):
                IMUAltitudeSensor(pair, clock=lambda: 0).read()

    def calibration_sensor(self, angles, rates):
        samples = [AltitudeSample(angle, i * .01, rate) for i, (angle, rate) in enumerate(zip(angles, rates))]
        sensor = IMUAltitudeSensor(None, sleep=lambda _: None)
        sensor.read = Mock(side_effect=samples)
        return sensor

    def test_stationary_calibration_and_angle_wrap(self):
        sensor = self.calibration_sensor([179.99, -179.99, 179.98, -179.98], [.12, .121, .119, .12])
        with patch.object(config, "IMU_GYRO_CALIBRATION_SAMPLES", 4):
            result = sensor.calibrate()
        self.assertAlmostEqual(result.bias, .12)
        self.assertEqual(result.samples, 4)

    def test_calibration_rejects_motion_and_excessive_bias(self):
        with patch.object(config, "IMU_GYRO_CALIBRATION_SAMPLES", 4):
            for angles, rates in (([0, 1, 2, 3], [0] * 4), ([0] * 4, [0, 2, -2, 0]),
                                   ([0] * 4, [6] * 4)):
                with self.subTest(angles=angles, rates=rates), self.assertRaises(IMUReadError):
                    self.calibration_sensor(angles, rates).calibrate()


class KalmanControlTests(unittest.TestCase):
    def setUp(self):
        flag = patch.object(config, "CONTROL_TELEMETRY_ENABLED", False)
        flag.start()
        self.addCleanup(flag.stop)
        self.plant = SimulatedMount()
        self.sensor = SimulatedIMU(self.plant)
        self.motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.motor.current_alt = self.plant.alt
        self.motor.enable_imu_tracking(None, self.sensor)

    def test_kalman_default_is_used_and_calibrated_bias_reaches_estimator(self):
        self.assertEqual(config.IMU_FILTER_MODE, "kalman")
        self.assertAlmostEqual(self.motor.imu_feedback.kalman.bias, .12)

    def test_plate_solution_preserves_bias_and_covariance(self):
        k = self.motor.imu_feedback.kalman
        k.bias = .123
        k.p00, k.p01, k.p11 = .02, -.001, .003
        self.motor.set_current_position(50, 30)
        self.assertAlmostEqual(k.bias, .123)
        self.assertEqual((k.p00, k.p01, k.p11), (.02, -.001, .003))
        self.assertAlmostEqual(self.motor.imu_feedback.position(), 50)

    def test_missing_gyro_does_not_silently_fall_back_to_accel(self):
        self.sensor.read = lambda: AltitudeSample(47, self.plant.time)
        self.motor.set_target(41, 0)
        self.plant.time += .05
        with self.assertRaisesRegex(RuntimeError, "gyro"):
            self.motor.update()
        self.assertTrue(self.plant.stopped)
        with self.assertRaises(RuntimeError):
            self.motor.resume()

    def test_idle_gap_reanchors_angle_and_preserves_bias(self):
        self.motor.imu_feedback.kalman.bias = .123
        self.plant.time += 10
        self.plant.alt += 2
        angle = self.motor.imu_feedback.position()
        self.assertAlmostEqual(angle, 42, delta=.02)
        self.assertAlmostEqual(self.motor.imu_feedback.kalman.bias, .123)

    def test_failed_calibration_stops_before_any_drive(self):
        motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.sensor.calibrate = Mock(side_effect=IMUReadError("mount moved"))
        with self.assertRaises(IMUReadError):
            motor.enable_imu_tracking(None, self.sensor)
        self.assertTrue(self.plant.stopped)
        self.assertIsNone(motor.imu_feedback)

    def test_closed_loop_demo_converges_and_records_filter_diagnostics(self):
        from pathlib import Path
        import csv
        with tempfile.TemporaryDirectory() as directory:
            result = run_demo(directory)
            self.assertLess(abs(result["final_physical_alt_error_deg"]), .18)
            self.assertAlmostEqual(result["estimated_gyro_bias_dps"], .12, delta=.02)
            with (Path(directory) / "imu_control.csv").open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 1600)
            self.assertTrue(all(row["gyro_bias"] and row["accel_alt"] for row in rows))


if __name__ == "__main__":
    unittest.main()
