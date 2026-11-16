import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import config
from controllers.motor_controller import MotorController, MotorMode
from controllers.position_pid import PositionPID
from hardware.imu import AltitudeSample, IMUAltitudeSensor, IMUReadError
from simulate_control import SimulatedMount
from simulate_imu_control import SimulatedIMU, run_demo
from tests.test_tracking_startup import (FakeCamera, FakeMotor, FakeIMU,
                                         FakeAstronomy, fake_leveler_factory)
from managers.tracking_manager import TrackingManager


class IMUFeedbackTests(unittest.TestCase):
    def setUp(self):
        mode = patch.object(config, "IMU_FILTER_MODE", "lowpass")
        mode.start()
        self.addCleanup(mode.stop)
        flag = patch.object(config, "CONTROL_TELEMETRY_ENABLED", False)
        flag.start()
        self.addCleanup(flag.stop)
        self.plant = SimulatedMount()
        self.sensor = SimulatedIMU(self.plant)
        self.motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.motor.current_alt = self.plant.alt
        self.motor.current_az = self.plant.az
        self.motor.enable_imu_tracking(None, self.sensor)

    def tick(self, count=1):
        for _ in range(count):
            self.plant.time += config.TRACKING_INTERVAL
            self.motor.update()

    def test_el_closes_loop_while_az_keeps_original_steps(self):
        self.motor.set_target(41, .2)
        self.tick(3)
        self.assertGreater(self.plant.alt, 40)
        self.assertGreater(self.motor.current_az, 359.8)
        self.assertLessEqual(self.motor.current_alt, self.plant.alt + .02)
        # AZ remains a step estimate, so deliberately lost steps are observable
        # only in the simulated plant, not fed back into the AZ controller.
        self.assertGreater(self.motor.current_az, self.plant.az)

    def test_pid_keeps_motion_state_between_exact_thresholds(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                pid = PositionPID(**config.IMU_TRACKING_PID)
                for error, stopped in (
                    (.150, True), (.151, False), (.150, False),
                    (.140, False), (.135, True), (.140, True),
                    (.150, True), (.151, False),
                ):
                    speed = pid.update(sign * error, 0.0, config.TRACKING_INTERVAL)
                    self.assertEqual(pid.stopped, stopped, msg=f"error={sign * error}")
                    self.assertEqual(speed == 0.0, stopped)
                pid.reset()
                self.assertEqual(pid.update(sign * .140, 0.0, config.TRACKING_INTERVAL), 0.0)
                self.assertTrue(pid.stopped)

    def test_fractional_steps_reset_when_pid_stops(self):
        controller = self.motor.imu_feedback
        # Keep the sensor angle fixed so we can exercise both states at 0.14 deg.
        controller.position = Mock(return_value=40.0)
        motor = SimpleNamespace(target_alt=40.0, step_angle=1.0)

        def update(error):
            self.plant.time += config.TRACKING_INTERVAL
            motor.target_alt = 40.0 + error
            return controller.steps(motor)

        self.assertGreater(update(.16)[1], 0)
        previous_remainder = controller.remainder
        self.assertGreater(previous_remainder, 0)
        self.assertGreater(update(.14)[1], 0)
        self.assertGreater(controller.remainder, previous_remainder)
        self.assertEqual(update(.13), (0, 0.0))
        self.assertTrue(controller.pid.stopped)
        self.assertEqual(controller.remainder, 0.0)
        self.assertEqual(update(.14), (0, 0.0))
        self.assertEqual(controller.remainder, 0.0)
        self.assertGreater(update(.16)[1], 0)

    def test_no_commanded_el_steps_are_added_to_measurement(self):
        self.plant.efficiency = 0
        self.sensor.read = lambda: AltitudeSample(47, self.plant.time)
        self.motor.set_target(41, 0)
        self.tick(10)
        self.assertEqual(self.motor.current_alt, 40)

    def test_stall_stops_and_refuses_resume(self):
        self.plant.efficiency = 0
        self.sensor.read = lambda: AltitudeSample(47, self.plant.time)
        self.motor.set_target(41, 0)
        with self.assertRaisesRegex(RuntimeError, "did not respond"):
            self.tick(200)
        self.assertTrue(self.plant.stopped)
        with self.assertRaises(RuntimeError):
            self.motor.resume()

    def test_read_failure_stops_both_axes(self):
        self.sensor.read = Mock(side_effect=OSError("I2C disconnected"))
        self.motor.set_target(41, 0)
        with self.assertRaises(OSError):
            self.tick()
        self.assertEqual(self.plant.alt, 40)
        self.assertEqual(self.plant.az, 359.8)
        self.assertEqual(self.motor.mode, MotorMode.STOP)

    def test_stale_data_stops(self):
        self.sensor.read = lambda: AltitudeSample(47, -1)
        self.motor.set_target(41, 0)
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            self.tick()
        self.assertTrue(self.plant.stopped)

    def test_jump_rejected_before_motion(self):
        self.sensor.read = lambda: AltitudeSample(70, self.plant.time)
        self.motor.set_target(41, 0)
        with self.assertRaisesRegex(RuntimeError, "jumped"):
            self.tick()

    def test_nan_rejected(self):
        self.sensor.read = lambda: AltitudeSample(math.nan, self.plant.time)
        self.motor.set_target(41, 0)
        with self.assertRaisesRegex(RuntimeError, "Nonfinite"):
            self.tick()

    def test_control_deadline_stops_without_catchup(self):
        self.motor.set_target(41, 0)
        self.tick(3)
        before = self.plant.alt
        self.plant.time += 1
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            self.motor.update()
        self.assertEqual(before, self.plant.alt)

    def test_plate_correction_survives_subsequent_sensor_read(self):
        self.motor.apply_position_offset(.3, -.1)
        self.motor.set_target(40.3, 359.7)
        self.tick()
        self.assertAlmostEqual(self.motor.current_alt, 40.3, delta=.02)
        self.assertAlmostEqual(self.motor.current_az, 359.7)

    def test_absolute_plate_reference_resets_mapping(self):
        self.motor.set_current_position(50, 20)
        self.motor.set_target(50, 20)
        self.tick()
        self.assertAlmostEqual(self.motor.current_alt, 50, delta=.02)

    def test_normal_resume_restarts_control_timing(self):
        self.motor.set_target(41, 0)
        self.tick(3)
        self.motor.stop()
        self.plant.time += 10
        self.motor.resume()
        self.motor.set_target(41, 0)
        self.tick()
        self.assertFalse(self.plant.stopped)

    def test_manual_limits_use_measured_altitude(self):
        self.motor.driver.drive = Mock()
        self.motor.set_current_position(config.MANUAL_MAX_ALTITUDE, 0)
        self.motor.move_manual("ALT", True, 100)
        self.motor.driver.drive.assert_not_called()


    def test_ble_jog_skips_imu_read_on_both_axes(self):
        self.sensor.read = Mock(side_effect=OSError("must not read IMU"))
        self.motor.driver.drive = Mock()
        for axis in ("ALT", "AZ"):
            self.motor.move_manual(axis, True, 1, use_imu=False)
        self.sensor.read.assert_not_called()
        self.assertEqual(self.motor.driver.drive.call_count, 2)

    def test_ble_jog_keeps_estimated_altitude_bounds(self):
        self.sensor.read = Mock(side_effect=OSError("must not read IMU"))
        self.motor.driver.drive = Mock()
        for angle, direction in ((config.MANUAL_MAX_ALTITUDE, True),
                                 (config.MANUAL_MIN_ALTITUDE, False)):
            self.motor.current_alt = angle
            self.motor.move_manual("ALT", direction, 10, use_imu=False)
        self.motor.driver.drive.assert_not_called()
        self.sensor.read.assert_not_called()

    def test_ble_jog_does_not_clear_tracking_fault(self):
        self.motor.imu_feedback.fault = "IMU read exceeded control deadline"
        self.motor.driver.drive = Mock()
        self.motor.move_manual("AZ", True, 1, use_imu=False)
        self.assertEqual(self.motor.imu_feedback.fault,
                         "IMU read exceeded control deadline")
        with self.assertRaisesRegex(RuntimeError, "latched"):
            self.motor.resume()

    def test_default_manual_still_checks_imu(self):
        self.sensor.read = Mock(side_effect=OSError("I2C disconnected"))
        self.motor.driver.drive = Mock()
        with self.assertRaises(OSError):
            self.motor.move_manual("AZ", True, 1)
        self.motor.driver.drive.assert_not_called()

    def test_preview_forwards_ble_option_and_keeps_state_gate(self):
        manager = TrackingManager.__new__(TrackingManager)
        manager.manual_enabled = True
        manager.state = Mock()
        manager.state.is_state.return_value = True
        manager.motor = Mock()
        manager.preview_move("ALT", True, 1, use_imu=False)
        manager.motor.move_manual.assert_called_once_with(
            axis="ALT", direction=True, steps=1, pulse_delay=None, use_imu=False)
        manager.motor.reset_mock()
        manager.state.is_state.return_value = False
        manager.preview_move("ALT", True, 1, use_imu=False)
        manager.motor.move_manual.assert_not_called()

    def test_simulated_external_disturbance_recovers(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_demo(directory)
            self.assertLess(abs(summary["final_physical_alt_error_deg"]), .18)
            self.assertIn("센서 측정 아님", (Path(directory) / "report.html").read_text(encoding="utf-8"))


class IMUAdapterTests(unittest.TestCase):
    def setUp(self):
        mode = patch.object(config, "IMU_FILTER_MODE", "lowpass")
        mode.start()
        self.addCleanup(mode.stop)

    def sensor(self, xyz):
        pair = SimpleNamespace(upper_address=0x69, read_acceleration=Mock(return_value=xyz))
        return IMUAltitudeSensor(pair, clock=lambda: 1.0)

    def test_valid_tilt_uses_only_upper_sensor(self):
        sensor = self.sensor((0, 8192, math.sqrt(3) * 8192))
        self.assertAlmostEqual(sensor.read().angle, 30)
        sensor.pair.read_acceleration.assert_called_once_with(0x69)

    def test_zero_dynamic_and_unobservable_gravity_rejected(self):
        for xyz in ((0, 0, 0), (0, 0, 32000), (16384, 0, 0), (math.nan, 0, 0)):
            with self.subTest(xyz=xyz), self.assertRaises(IMUReadError):
                self.sensor(xyz).read()

    def test_slow_i2c_read_rejected(self):
        sensor = self.sensor((0, 0, 16384))
        sensor.clock = Mock(side_effect=[0, 1])
        with self.assertRaisesRegex(IMUReadError, "deadline"):
            sensor.read()


class IMULifecycleTests(unittest.TestCase):
    def make_manager(self):
        self.events = []
        motor = FakeMotor(self.events, "motor")
        motor.enable_imu_tracking = lambda imu: self.events.append("imu.attach")
        return TrackingManager(camera=FakeCamera(self.events, "camera"), motor=motor,
                               imu=FakeIMU(self.events), astronomy=FakeAstronomy(self.events),
                               platesolver=object(), logger=object(),
                               leveling_controller_factory=fake_leveler_factory(self.events))

    def test_continuous_mode_keeps_bus_open_after_alignment(self):
        manager = self.make_manager()
        manager.initialize_hardware()
        self.assertEqual(self.events, ["motor.initialize", "imu.initialize", "leveler.align",
                                       "imu.attach", "camera.initialize"])

    def test_attach_failure_closes_imu_and_stops_motor(self):
        manager = self.make_manager()
        manager.motor.enable_imu_tracking = Mock(side_effect=RuntimeError("bad sample"))
        with self.assertRaises(RuntimeError):
            manager.initialize_hardware()
        self.assertIn("imu.close", self.events)
        self.assertIn("motor.stop", self.events)
        self.assertNotIn("camera.initialize", self.events)

    def test_conflicting_modes_rejected_before_motor_initialization(self):
        manager = self.make_manager()
        with patch.object(config, "ENCODER_ENABLED", True), self.assertRaises(ValueError):
            manager.initialize_hardware()
        self.assertEqual(self.events, [])


if __name__ == "__main__":
    unittest.main()
