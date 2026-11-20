"""Sidereal feedforward and feedback hold during exposures."""
import unittest

from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch

import config
from controllers.motor_controller import MotorController, MotorMode
from hardware.imu import AltitudeSample
from simulate_control import SimulatedMount
from simulate_imu_control import SimulatedIMU
from tests.test_tracking_runtime import make_runtime_manager

# Roughly sidereal: 0.004 deg/s at the 0.05 s tracking interval.
RATE = 0.004 * config.TRACKING_INTERVAL


class StepOnlyFeedforwardTests(unittest.TestCase):
    """No IMU: both axes are step-estimated."""

    def setUp(self):
        flag = patch.object(config, "CONTROL_TELEMETRY_ENABLED", False)
        flag.start()
        self.addCleanup(flag.stop)
        self.plant = SimulatedMount()
        self.plant.efficiency = 1.0
        self.motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.motor.current_alt, self.motor.current_az = 40.0, 10.0
        self.plant.alt, self.plant.az = 40.0, 10.0

    def run_ramp(self, ticks, alt0=40.0, az0=10.0):
        for tick in range(ticks):
            self.plant.time = tick * config.TRACKING_INTERVAL
            self.motor.set_target(alt0 + tick * RATE, az0 + tick * RATE)
            self.motor.update()
        return alt0 + (ticks - 1) * RATE, az0 + (ticks - 1) * RATE

    def test_slow_target_is_followed_within_one_step_below_tolerance(self):
        target_alt, target_az = self.run_ramp(200)  # 10 s, 0.04 deg
        step = config.STEP_ANGLE
        # Without feedforward the 0.03 deg bang-bang would leave up to 0.03 deg.
        self.assertLess(abs(target_alt - self.plant.alt), 1.01 * step)
        self.assertLess(abs(target_az - self.plant.az), 1.01 * step)

    def test_feedforward_disabled_restores_bang_bang(self):
        with patch.object(config, "TRACKING_FEEDFORWARD_ENABLED", False):
            target_alt, _ = self.run_ramp(100)  # 0.02 deg, below 0.03 tolerance
        self.assertEqual(self.plant.alt, 40.0)
        self.assertGreater(target_alt - self.plant.alt, 0.019)

    def test_large_target_jump_is_not_fed_forward(self):
        self.motor.set_target(40.0, 10.0)
        self.motor.set_target(40.0 + 2 * config.TRACKING_FEEDFORWARD_MAX_STEP_DEG, 10.0)
        self.assertEqual(self.motor._ff_alt, 0.0)
        self.assertEqual(self.motor._ff_az, 0.0)

    def test_first_target_after_stop_is_not_fed_forward(self):
        self.motor.set_target(40.0, 10.0)
        self.motor.stop()
        self.motor.set_target(40.01, 10.01)
        self.assertEqual(self.motor._ff_alt, 0.0)

    def test_hold_drives_only_feedforward(self):
        # A 1 deg pointing error would normally be corrected immediately.
        self.motor.current_alt, self.motor.current_az = 39.0, 9.0
        self.motor.set_exposure_hold(True)
        target_alt, target_az = self.run_ramp(200)
        moved = (self.plant.alt - 40.0, self.plant.az - 10.0)
        self.assertAlmostEqual(moved[0], target_alt - 40.0, delta=1.01 * config.STEP_ANGLE)
        self.assertAlmostEqual(moved[1], target_az - 10.0, delta=1.01 * config.STEP_ANGLE)
        self.motor.set_exposure_hold(False)
        self.plant.time += config.TRACKING_INTERVAL
        self.motor.set_target(target_alt, target_az)
        self.motor.update()
        self.assertGreater(self.plant.alt - 40.0, moved[0])  # correction resumes

    def test_cap_keeps_excess_feedforward_owed(self):
        self.motor.current_alt = 39.0  # feedback alone already asks for the cap
        self.motor.set_target(40.0, 10.0)
        self.motor.set_target(40.0 + 3 * config.STEP_ANGLE, 10.0)
        self.motor.update()
        self.assertEqual(self.motor.current_alt, 39.0 + config.MAX_STEP_PER_UPDATE * config.STEP_ANGLE)
        self.assertAlmostEqual(self.motor._ff_alt, 3.0)


class IMUFeedforwardHoldTests(unittest.TestCase):
    def setUp(self):
        for name, value in (("IMU_FILTER_MODE", "lowpass"), ("CONTROL_TELEMETRY_ENABLED", False)):
            patcher = patch.object(config, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.plant = SimulatedMount()
        self.plant.efficiency = 1.0
        self.sensor = SimulatedIMU(self.plant)
        self.motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.motor.current_alt, self.motor.current_az = self.plant.alt, self.plant.az
        self.motor.enable_imu_tracking(None, self.sensor)

    def tick(self, alt, az):
        self.plant.time += config.TRACKING_INTERVAL
        self.motor.set_target(alt, az)
        self.motor.update()

    def test_hold_ignores_imu_error_but_keeps_feedforward(self):
        self.motor.set_exposure_hold(True)
        alt0 = 41.0  # 1 deg EL error: the PID would move at max speed
        for tick in range(200):
            self.tick(alt0 + tick * RATE, 0.0)
        self.assertAlmostEqual(self.plant.alt - 40.0, 199 * RATE, delta=1.01 * config.STEP_ANGLE)
        self.motor.set_exposure_hold(False)
        before = self.plant.alt
        for tick in range(200, 220):
            self.tick(alt0 + tick * RATE, 0.0)
        self.assertGreater(self.plant.alt - before, 20 * RATE + 0.01)

    def test_hold_still_stops_on_imu_read_failure(self):
        self.motor.set_exposure_hold(True)
        self.tick(40.0, 359.8)
        self.sensor.read = Mock(side_effect=OSError("I2C disconnected"))
        with self.assertRaises(OSError):
            self.tick(40.0 + RATE, 359.8 + RATE)
        self.assertEqual(self.motor.mode, MotorMode.STOP)

    def test_hold_still_checks_altitude_jump(self):
        self.motor.set_exposure_hold(True)
        self.tick(40.0, 359.8)
        self.sensor.read = lambda: AltitudeSample(70, self.plant.time)
        with self.assertRaisesRegex(RuntimeError, "jumped"):
            self.tick(40.0, 359.8)


class ExposureHoldSequenceTests(unittest.TestCase):
    def test_hold_wraps_each_exposure_and_clears_on_failure(self):
        manager, camera, motor, _logger = make_runtime_manager()
        manager._get_observation_time = lambda: 100.0
        holds = []
        motor.set_exposure_hold = holds.append
        manager._solve_exposure = Mock(return_value=True)
        manager._wait_for_capture_settle = Mock(return_value=True)

        def capture(*, filename, exposure_time):
            self.assertEqual(holds[-1], True)
            if len(holds) == 3:
                raise OSError("camera failed")
            return filename

        camera.capture_long_exposure = capture
        with TemporaryDirectory() as directory, patch.object(
                config, "FINAL_SEQUENCE_DIRECTORY", directory):
            manager.pipelined_capture_loop()
        self.assertEqual(holds, [True, False, True, False])
        self.assertEqual(_logger.finished[-1]["status"], "final_capture_failed")


if __name__ == "__main__":
    unittest.main()


class PerAxisStepRatioTests(unittest.TestCase):
    """AZ_STEP_RATIO / ALT_STEP_RATIO from calibrate_steps.py."""

    def make(self, alt_ratio=1.0, az_ratio=1.0):
        patcher = patch.multiple(config, ALT_STEP_RATIO=alt_ratio, AZ_STEP_RATIO=az_ratio,
                                 CONTROL_TELEMETRY_ENABLED=False)
        patcher.start()
        self.addCleanup(patcher.stop)
        plant = SimulatedMount()
        plant.efficiency = 1.0
        motor = MotorController(driver=plant, clock=lambda: plant.time)
        return plant, motor

    def test_default_ratio_keeps_old_step_angle(self):
        _, motor = self.make()
        self.assertEqual(motor.step_angle, config.STEP_ANGLE)
        self.assertEqual(motor.az_step_angle, config.STEP_ANGLE)
        self.assertEqual(motor.angle_to_steps(1.0, "AZ"), motor.angle_to_steps(1.0))

    def test_az_ratio_two_halves_az_steps_only(self):
        _, motor = self.make(az_ratio=2.0)
        self.assertEqual(motor.angle_to_steps(1.0, "AZ") * 2, motor.angle_to_steps(1.0, "ALT"))
        self.assertAlmostEqual(motor.steps_to_angle(10, "AZ"), 20 * config.STEP_ANGLE)
        self.assertEqual(motor.step_angle, config.STEP_ANGLE)

    def test_az_feedforward_matches_real_axis_with_ratio(self):
        plant, motor = self.make(az_ratio=2.0)
        plant.efficiency = 1.0
        az_gain = 2.0  # the real AZ axis moves twice the nominal step

        def drive(alt_steps, alt_direction, az_steps, az_direction):
            plant.az += az_steps * config.STEP_ANGLE * az_gain * (1 if az_direction else -1)

        plant.drive_dual = drive
        motor.current_alt, motor.current_az = 40.0, 10.0
        plant.az = 10.0
        for tick in range(200):
            plant.time = tick * config.TRACKING_INTERVAL
            motor.set_target(40.0, 10.0 + tick * RATE)
            motor.update()
        target = 10.0 + 199 * RATE
        self.assertAlmostEqual(plant.az, target, delta=1.01 * 2 * config.STEP_ANGLE)
        self.assertAlmostEqual(motor.current_az, plant.az, places=9)

    def test_manual_az_jog_uses_az_ratio(self):
        plant, motor = self.make(az_ratio=2.0)
        plant.drive = Mock()
        motor.current_az = 100.0
        motor.move_manual("AZ", True, 10)
        self.assertAlmostEqual(motor.current_az, 100.0 + 20 * config.STEP_ANGLE)

    def test_invalid_ratio_rejected(self):
        with self.assertRaises(ValueError):
            self.make(az_ratio=0.0)
