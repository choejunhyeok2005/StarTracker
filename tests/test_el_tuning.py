import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import config
from controllers.motor_controller import MotorController
from simulate_control import SimulatedMount
from simulate_imu_control import SimulatedIMU
from tune_el import ELOnlyDriver, parser, run, run_sequence, validate


class TuningTests(unittest.TestCase):
    def test_invalid_input_rejected_before_initializing_hardware(self):
        cases = [["--hardware"], ["--start-alt", "90", "--step", "1"],
                 ["--ki", "nan"], ["--kd", "-1"], ["--cycles", "0"],
                 ["--hold", "0"], ["--step", "6"], ["--step", "0"]]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                validate(parser().parse_args(case))

    def test_default_is_simulation_and_negative_step_allowed(self):
        args = parser().parse_args(["--step", "-1"])
        validate(args)
        self.assertFalse(args.hardware)
        self.assertEqual(args.start_alt, 40)

    def test_az_drive_is_rejected(self):
        driver = Mock()
        guarded = ELOnlyDriver(driver)
        with self.assertRaises(RuntimeError):
            guarded.drive_dual(az_steps=1)
        with self.assertRaises(RuntimeError):
            guarded.drive(axis="AZ", steps=1)
        driver.drive.assert_not_called()
        driver.drive_dual.assert_not_called()

    def test_real_controller_cycles_return_and_never_pulse_az(self):
        plant = SimulatedMount()
        plant.alt, plant.az = 40, 0
        calls = []
        original = plant.drive_dual
        def drive(**kwargs):
            calls.append(kwargs)
            original(**kwargs)
        plant.drive_dual = drive
        with patch.object(config, "CONTROL_TELEMETRY_ENABLED", False):
            motor = MotorController(driver=ELOnlyDriver(plant), clock=lambda: plant.time)
            motor.current_alt = 40
            motor.enable_imu_tracking(None, SimulatedIMU(plant))
            args = parser().parse_args(["--cycles", "2"])
            validate(args)
            events = []
            result = run_sequence(motor, args, lambda: plant.time,
                                  lambda dt: setattr(plant, "time", plant.time + dt), events)
        self.assertTrue(result["returned_within_tolerance"])
        self.assertEqual([e["phase"] for e in events], ["baseline", "step_1", "return_1", "step_2", "return_2"])
        self.assertTrue(plant.stopped)
        self.assertTrue(calls)
        self.assertTrue(all(c["az_steps"] == 0 for c in calls))

    def test_interruption_and_sensor_error_stop_without_return_commands(self):
        for error in (KeyboardInterrupt(), OSError("sensor failed")):
            motor = Mock(current_az=0)
            motor.imu_feedback.start_time = 0
            motor.update.side_effect = error
            args = parser().parse_args([])
            validate(args)
            with self.subTest(error=type(error).__name__), self.assertRaises(type(error)):
                run_sequence(motor, args, clock=lambda: 0)
            motor.stop.assert_called_once()
            motor.set_target.assert_called_once_with(40, 0)

    def test_run_saves_gains_phase_times_and_graph_without_changing_config(self):
        previous = dict(config.IMU_TRACKING_PID)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            args = parser().parse_args(["--cycles", "1", "--ki", "0.01", "--kd", "0.02",
                                       "--output", str(output)])
            self.assertEqual(run(args), 0)
            settings = json.loads((output / "settings.json").read_text())
            self.assertEqual(settings["ki"], .01)
            self.assertEqual(settings["kd"], .02)
            self.assertTrue(settings["simulation_only"])
            self.assertEqual(len(json.loads((output / "events.json").read_text())), 3)
            self.assertTrue((output / "report.html").exists())
            self.assertTrue((output / "control.csv").exists())
        self.assertEqual(previous, config.IMU_TRACKING_PID)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            args = parser().parse_args(["--output", directory])
            with self.assertRaises(FileExistsError):
                run(args)

    def test_partial_artifacts_survive_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failed"
            args = parser().parse_args(["--output", str(output)])
            with patch("tune_el.run_sequence", side_effect=OSError("test sensor failure")):
                self.assertEqual(run(args), 1)
            result = json.loads((output / "summary.json").read_text())
            self.assertEqual(result["status"], "failed")
            self.assertIn("test sensor failure", result["error"])
            self.assertTrue((output / "control.csv").exists())


if __name__ == "__main__":
    unittest.main()
