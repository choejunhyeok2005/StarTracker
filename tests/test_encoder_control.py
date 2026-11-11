import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import config
from controllers.motor_controller import MotorController, MotorMode
from controllers.position_pid import PositionPID, angular_error
from hardware.encoder import EncoderSample, QuadratureEncoder, configured_encoder
from simulate_control import SimulatedMount, run_demo


class EncoderControlTests(unittest.TestCase):
    def setUp(self):
        self.telemetry_patch = patch.object(config, "CONTROL_TELEMETRY_ENABLED", False)
        self.telemetry_patch.start()
        self.addCleanup(self.telemetry_patch.stop)
        self.make_motor()

    def make_motor(self):
        self.plant = SimulatedMount()
        self.motor = MotorController(driver=self.plant, encoder=self.plant,
                                     clock=lambda: self.plant.time)
        self.motor.set_current_position(40, 359.8)

    def tick(self, count=1):
        for _ in range(count):
            self.plant.time += config.TRACKING_INTERVAL
            self.motor.update()

    def test_wrap_takes_short_route(self):
        self.motor.set_target(40, .2)
        self.tick(3)
        self.assertGreater(self.plant.az, 359.8)
        self.assertAlmostEqual(angular_error(.2, 359.8), .4)

    def test_missed_steps_do_not_become_measured_movement(self):
        self.plant.efficiency = 0
        self.motor.set_target(41, 1)
        self.tick(5)
        self.assertEqual(self.motor.current_alt, 40)
        self.assertEqual(self.motor.current_az, 359.8)

    def test_stall_stops_and_latches(self):
        self.plant.efficiency = 0
        self.motor.set_target(41, 1)
        with self.assertRaisesRegex(RuntimeError, "did not respond"):
            self.tick(100)
        self.assertTrue(self.plant.stopped)
        self.assertEqual(self.motor.mode, MotorMode.STOP)
        with self.assertRaises(RuntimeError):
            self.motor.resume()

    def test_stale_nan_and_io_fault_stop_before_pulses(self):
        for sample in (EncoderSample(40, 0, -1), EncoderSample(math.nan, 0, 0)):
            with self.subTest(sample=sample):
                self.make_motor()
                self.plant.read = lambda: sample
                self.motor.set_target(41, 1)
                with self.assertRaises(RuntimeError):
                    self.tick()
                self.assertTrue(self.plant.stopped)
                self.assertEqual(self.plant.alt, 40)
        self.make_motor()
        self.plant.read = Mock(side_effect=OSError("disconnected"))
        self.motor.set_target(41, 1)
        with self.assertRaises(OSError):
            self.tick()
        self.assertTrue(self.plant.stopped)

    def test_deadline_fault_prevents_catchup_burst(self):
        self.motor.set_target(41, 1)
        self.tick()
        before = self.plant.alt
        self.plant.time += 1
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            self.motor.update()
        self.assertEqual(self.plant.alt, before)

    def test_plate_offset_survives_next_encoder_read(self):
        self.motor.apply_position_offset(.2, -.1)
        self.plant.alt += .01
        alt, az = self.motor.feedback.position()
        self.assertAlmostEqual(alt, 40.21)
        self.assertAlmostEqual(az, 359.7)

    def test_reference_after_motion_is_absolute(self):
        self.plant.alt += 1
        self.motor.set_current_position(50, 10)
        self.plant.alt += .1
        self.assertAlmostEqual(self.motor.feedback.position()[0], 50.1)

    def test_measured_limit_violation_stops(self):
        self.plant.alt = 91
        self.motor.set_target(90, 0)
        with self.assertRaisesRegex(RuntimeError, "outside tracking limits"):
            self.tick()
        self.assertTrue(self.plant.stopped)

    def test_nonfinite_target_rejected(self):
        with self.assertRaises(ValueError):
            self.motor.set_target(math.nan, 0)

    def test_normal_stop_and_resume_do_not_catch_up_old_time(self):
        self.motor.set_target(41, 1)
        self.tick(10)
        self.motor.stop()
        before = self.plant.alt
        self.plant.time += 10
        self.motor.update()
        self.assertEqual(self.plant.alt, before)
        self.motor.resume()
        self.motor.set_target(41, 1)
        self.tick(3)
        self.assertGreater(self.plant.alt, before)

    def test_maximum_steps_and_driver_polarity(self):
        driver = Mock()
        self.motor.driver = driver
        self.motor._alt_positive_direction = False
        self.motor.set_target(80, 100)
        self.tick(30)
        self.assertGreater(len(driver.drive_dual.call_args_list), 0)
        for call in driver.drive_dual.call_args_list:
            self.assertLessEqual(call.kwargs["alt_steps"], config.MAX_STEP_PER_UPDATE)
            self.assertLessEqual(call.kwargs["az_steps"], config.MAX_STEP_PER_UPDATE)
            self.assertFalse(call.kwargs["alt_direction"])

    def test_low_speed_fractional_steps_accumulate(self):
        for pid in self.motor.feedback.pid.values():
            pid.kp = .02
            pid.tolerance = 0
        self.motor.set_target(40.02, 359.8)
        self.tick(100)
        self.assertGreater(self.plant.alt, 40)

    def test_initial_alignment_sets_encoder_reference(self):
        self.motor.feedback.offset = None
        self.motor.begin_alignment(100)
        self.motor.finish_alignment()
        self.assertEqual(self.motor.feedback.position()[0], 0)
        self.plant.alt += .1
        self.assertAlmostEqual(self.motor.feedback.position()[0], .1)

    def test_no_reference_refuses_tracking(self):
        self.motor.feedback.offset = None
        self.motor.set_target(40, 0)
        with self.assertRaisesRegex(RuntimeError, "reference"):
            self.tick()
        self.assertTrue(self.plant.stopped)

    def test_simulated_disturbance_recovers_and_report_exists(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_demo(directory)
            self.assertLess(abs(result["final_error"]["alt_deg"]), .011)
            self.assertLess(abs(result["final_error"]["az_deg"]), .011)
            self.assertEqual(len((Path(directory) / "control.csv").read_text().splitlines()), 1201)
            self.assertIn("<canvas", (Path(directory) / "report.html").read_text(encoding="utf-8"))

    def test_pid_antiwindup_and_acceleration_bound(self):
        pid = PositionPID(2, 1, 0, .1, .2, .01)
        self.assertAlmostEqual(pid.update(10, 0, .05), .01)
        for _ in range(100):
            self.assertLessEqual(abs(pid.update(10, 0, .05)), .1)
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.update(0, 0, .05), 0)

    def test_derivative_uses_wrapped_measurement_not_target_step(self):
        pid = PositionPID(0, 0, .01, 100, 10000, 0)
        pid.update(1, 359.9, .1, circular=True)
        self.assertAlmostEqual(pid.update(100, .1, .1, circular=True), -.02)

    def test_reduced_speed_limit_is_always_obeyed(self):
        pid = PositionPID(2, 0, 0, 1, 1, .01)
        pid.update(10, 0, 1)
        pid.max_speed = .01
        self.assertLessEqual(pid.update(10, 0, .01), .01)


class QuadratureTests(unittest.TestCase):
    def make_encoder(self):
        gpio = Mock()
        self.levels = {4: 0, 5: 0, 6: 0, 7: 0}
        gpio.input.side_effect = lambda p: self.levels[p]
        axes = {"ALT": {"pins": (4, 5), "counts_per_rev": 400, "sign": 1},
                "AZ": {"pins": (6, 7), "counts_per_rev": 400, "sign": -1}}
        return QuadratureEncoder(axes, gpio=gpio, clock=lambda: 1)

    def test_forward_reverse_x4_and_sign(self):
        encoder = self.make_encoder()
        for state in (1, 3, 2, 0):
            self.levels[4], self.levels[5] = state >> 1, state & 1
            self.levels[6], self.levels[7] = state >> 1, state & 1
            encoder._edge("ALT")
            encoder._edge("AZ")
        self.assertAlmostEqual(encoder.read().alt, 3.6)
        self.assertAlmostEqual(encoder.read().az, -3.6)
        for state in (2, 3, 1, 0):
            self.levels[4], self.levels[5] = state >> 1, state & 1
            encoder._edge("ALT")
        self.assertEqual(encoder.read().alt, 0)
        encoder.close()
        self.assertEqual(encoder.gpio.remove_event_detect.call_count, 4)

    def test_illegal_transition_latches_fault(self):
        encoder = self.make_encoder()
        self.levels[4] = self.levels[5] = 1
        encoder._edge("ALT")
        with self.assertRaisesRegex(RuntimeError, "reference lost"):
            encoder.read()

    def test_unconfigured_hardware_fails_before_gpio_import(self):
        with self.assertRaises(ValueError):
            configured_encoder()


if __name__ == "__main__":
    unittest.main()
