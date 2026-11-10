import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import config
from controllers.motor_controller import MotorController, MotorMode
from hardware.imu import AltitudeSample, IMUAltitudeSensor, IMUReadDeadlineError, IMUReadError
from simulate_control import SimulatedMount


class DeadlineRecoveryTests(unittest.TestCase):
    def setUp(self):
        flag = patch.object(config, "CONTROL_TELEMETRY_ENABLED", False)
        flag.start()
        self.addCleanup(flag.stop)
        self.plant = SimulatedMount()
        self.sensor = Mock()
        self.sensor.calibrate.return_value = SimpleNamespace(bias=0.12)
        self.sensor.read.side_effect = self.fresh
        self.motor = MotorController(driver=self.plant, clock=lambda: self.plant.time)
        self.motor.current_alt, self.motor.current_az = self.plant.alt, self.plant.az
        self.motor.enable_imu_tracking(None, self.sensor)
        self.motor.set_target(41, 0.2)
        self.drive = Mock(wraps=self.plant.drive_dual)
        self.plant.drive_dual = self.drive

    def fresh(self):
        return AltitudeSample(47, self.plant.time, 0.12)

    def tick(self):
        self.plant.time += 0.05
        self.motor.update()

    def slow(self):
        self.plant.time += 1.0
        raise IMUReadDeadlineError("IMU read exceeded control deadline (1000 > 150 ms)")

    def begin_recovery(self):
        self.sensor.read.side_effect = self.slow
        self.tick()
        self.drive.assert_not_called()
        self.assertIsNone(self.motor.imu_feedback.fault)
        self.assertEqual(self.motor.mode, MotorMode.TRACKING)

    def test_two_fresh_samples_then_next_update_resumes_both_axes(self):
        self.begin_recovery()
        self.sensor.read.side_effect = self.fresh
        self.tick()
        self.drive.assert_not_called()
        self.tick()
        self.drive.assert_not_called()
        self.assertIsNone(self.motor.imu_feedback.recovery_started)
        self.tick()
        self.drive.assert_called_once()
        self.assertLessEqual(abs(self.drive.call_args.kwargs["alt_steps"]), config.MAX_STEP_PER_UPDATE)
        self.assertGreater(self.drive.call_args.kwargs["az_steps"], 0)
        self.assertIsNone(self.motor.imu_feedback.fault)

    def test_repeated_deadlines_exhaust_three_retries_and_latch(self):
        self.begin_recovery()
        self.tick()
        self.tick()
        with self.assertRaisesRegex(RuntimeError, "budget exhausted"):
            self.tick()
        self.drive.assert_not_called()
        self.assertEqual(self.motor.mode, MotorMode.STOP)
        with self.assertRaisesRegex(RuntimeError, "latched"):
            self.motor.resume()

    def test_good_samples_must_be_consecutive_and_budget_is_total(self):
        self.begin_recovery()
        self.sensor.read.side_effect = self.fresh
        self.tick()
        self.sensor.read.side_effect = self.slow
        self.tick()
        self.sensor.read.side_effect = self.fresh
        with self.assertRaisesRegex(RuntimeError, "budget exhausted"):
            self.tick()
        self.drive.assert_not_called()

    def test_non_deadline_failure_during_recovery_stops_immediately(self):
        self.begin_recovery()
        self.sensor.read.side_effect = IMUReadError("invalid gravity")
        with self.assertRaisesRegex(IMUReadError, "invalid gravity"):
            self.tick()
        self.drive.assert_not_called()
        self.assertEqual(self.motor.mode, MotorMode.STOP)

    def test_jump_after_long_gap_is_not_accepted_as_recovery(self):
        self.begin_recovery()
        self.sensor.read.side_effect = lambda: AltitudeSample(70, self.plant.time, 0.12)
        with self.assertRaisesRegex(RuntimeError, "jumped"):
            self.tick()
        self.drive.assert_not_called()

    def test_recovery_preserves_altitude_limits(self):
        self.begin_recovery()
        self.motor.imu_feedback.offset = 100
        self.sensor.read.side_effect = self.fresh
        with self.assertRaisesRegex(RuntimeError, "outside tracking limits"):
            self.tick()
        self.drive.assert_not_called()

    def test_recovery_time_budget_prevents_another_read(self):
        self.begin_recovery()
        count = self.sensor.read.call_count
        self.plant.time += config.IMU_DEADLINE_RECOVERY_MAX_SECONDS + 1
        with self.assertRaisesRegex(RuntimeError, "time limit"):
            self.tick()
        self.assertEqual(self.sensor.read.call_count, count)
        self.drive.assert_not_called()

    def test_recovery_cannot_resume_external_stop(self):
        self.begin_recovery()
        self.motor.stop()
        self.sensor.read.side_effect = self.fresh
        self.tick()
        self.drive.assert_not_called()
        self.assertEqual(self.motor.mode, MotorMode.STOP)

    def test_stall_evidence_and_reference_are_preserved(self):
        f = self.motor.imu_feedback
        f.watch = (0.0, 40.0, 0.4)
        offset = f.offset
        self.begin_recovery()
        self.sensor.read.side_effect = self.fresh
        self.tick()
        self.tick()
        self.assertEqual(f.watch[1:], (40.0, 0.4))
        self.assertEqual(f.offset, offset)
        self.assertEqual(f.kalman.bias, 0.12)
        self.assertEqual(f.pid.integral, 0.0)

    def test_stale_sample_during_recovery_stops(self):
        self.begin_recovery()
        self.sensor.read.side_effect = lambda: AltitudeSample(47, self.plant.time - 1, 0.12)
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            self.tick()
        self.drive.assert_not_called()

    def test_adapter_produces_typed_deadline(self):
        pair = SimpleNamespace(upper_address=0x69,
            read_motion=Mock(return_value=((0, 0, 16384), (0, 0, 0))))
        sensor = IMUAltitudeSensor(pair, clock=Mock(side_effect=[0, 1]))
        with self.assertRaises(IMUReadDeadlineError):
            sensor.read()

    def test_reference_deadline_still_latches_without_auto_recovery(self):
        self.sensor.read.side_effect = self.slow
        with self.assertRaises(IMUReadDeadlineError):
            self.motor.set_current_position(40, 0)
        self.assertTrue(self.motor.imu_feedback.fault)
        self.drive.assert_not_called()


if __name__ == "__main__":
    unittest.main()
