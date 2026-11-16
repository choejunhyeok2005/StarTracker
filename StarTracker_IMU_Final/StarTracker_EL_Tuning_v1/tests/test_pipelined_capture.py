import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch

import config
from tests.test_tracking_runtime import make_runtime_manager


class PipelinedCaptureTests(unittest.TestCase):
    def test_solves_previous_frame_during_next_exposure_and_skips_final(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager.is_tracking = True
        manager.target_ra, manager.target_dec = 1, 2
        manager._get_observation_time = lambda: 100.0
        manager.astronomy = Mock()
        manager.astronomy.radec_to_altaz.side_effect = [(30.1, 359.9), (30, 0)]
        motor.apply_position_offset = Mock()
        camera.capture_plate = Mock(side_effect=AssertionError("Extra plate photo"))
        exposure_two = threading.Event()
        corrected = threading.Event()
        paths, solved = [], []

        def solve(path):
            self.assertTrue(exposure_two.wait(2), "Solve must overlap exposure 2")
            solved.append(Path(path).name)
            return {"ra": 1.1, "dec": 2.1}

        manager.platesolver = Mock()
        manager.platesolver.solve.side_effect = solve
        motor.apply_position_offset.side_effect = lambda *args: corrected.set()

        def capture(*, filename, exposure_time):
            paths.append(filename)
            if len(paths) == 2:
                exposure_two.set()
                self.assertTrue(corrected.wait(2), "Correction should run during capture")
            if len(paths) == 3:
                self.assertEqual(len(solved), 1, "Last exposure must skip correction")
            Path(filename).write_bytes(b"test-image")
            return filename

        camera.capture_long_exposure = capture
        with TemporaryDirectory() as directory, patch.multiple(
            config, PIPELINED_WAIT_FOR_SETTLE=False, PIPELINED_FRAME_COUNT=3,
            PIPELINED_EXPOSURE_SECONDS=110,
            PIPELINED_SKIP_FINAL_CORRECTION=True, FINAL_SEQUENCE_DIRECTORY=directory,
        ):
            manager.pipelined_capture_loop()
            self.assertEqual(len(list(Path(directory).glob("*.jpg"))), 3)
        self.assertEqual(len(solved), 1)
        self.assertTrue(solved[0].endswith("_001.jpg"))
        self.assertAlmostEqual(motor.apply_position_offset.call_args.args[0], 0.1)
        self.assertAlmostEqual(motor.apply_position_offset.call_args.args[1], -0.1)
        self.assertEqual(logger.finished[-1]["final_images"], paths)
        self.assertTrue(manager.capture_completed)
        camera.capture_plate.assert_not_called()

    def test_capture_failure_preserves_already_saved_paths_and_prevents_success_exit(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager.is_tracking = True
        manager._get_observation_time = lambda: 100.0
        camera.capture_long_exposure = Mock(side_effect=["first.jpg", OSError("disk full")])
        manager._solve_exposure = Mock()
        with TemporaryDirectory() as directory, patch.multiple(
            config, PIPELINED_WAIT_FOR_SETTLE=False, PIPELINED_FRAME_COUNT=3,
            FINAL_SEQUENCE_DIRECTORY=directory,
        ):
            manager.pipelined_capture_loop()
        self.assertEqual(logger.finished[-1]["status"], "final_capture_failed")
        self.assertEqual(logger.finished[-1]["final_images"], ["first.jpg"])
        self.assertFalse(manager.capture_completed)
        self.assertTrue(manager.stop_event.is_set())
        self.assertEqual(motor.stop_count, 1)

    def test_cancel_during_solve_does_not_apply_late_result(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager.platesolver = Mock()
        def solve(path):
            manager.stop_event.set()
            return {"ra": 1, "dec": 2}
        manager.platesolver.solve.side_effect = solve
        motor.apply_position_offset = Mock()
        manager._solve_exposure(("first.jpg", 100, 1))
        motor.apply_position_offset.assert_not_called()


class SettledCaptureTests(unittest.TestCase):
    def test_waits_between_six_exposures(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager._get_observation_time = lambda: 100.0
        order = []

        manager.target_ra, manager.target_dec = 1, 2
        manager.astronomy = Mock()
        manager.astronomy.radec_to_altaz.return_value = (40.0, 100.0)

        def capture(*, filename, exposure_time):
            order.append(("photo", exposure_time))
            Path(filename).write_bytes(b"test-image")
            order.append("saved")
            return filename

        def solve(path):
            self.assertTrue(Path(path).is_file())
            order.append("solve")
            return {"ra": 1, "dec": 2}

        manager.platesolver = Mock()
        manager.platesolver.solve.side_effect = solve
        motor.apply_position_offset = Mock(side_effect=lambda *args: order.append("correct"))
        manager._wait_for_capture_settle = Mock(side_effect=lambda *args: order.append(("settle", args)) or True)
        camera.capture_long_exposure = capture
        with TemporaryDirectory() as directory, patch.object(config, "FINAL_SEQUENCE_DIRECTORY", directory):
            manager.pipelined_capture_loop()
        exposure = [("photo", 10.0), "saved"]
        correction = ["solve", "correct", ("settle", (30.0, 5))]
        self.assertEqual(order, (exposure + correction) * 5 + exposure)
        self.assertTrue(manager.capture_completed)
        self.assertEqual(motor.stop_count, 1)
        saved_paths = logger.finished[-1]["final_images"]
        self.assertEqual(len(saved_paths), 6)
        self.assertEqual([call.args[0] for call in manager.platesolver.solve.call_args_list],
                         saved_paths[:-1])

    def test_solver_failure_preserves_the_first_frame(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager._get_observation_time = lambda: 100.0
        camera.capture_long_exposure = Mock(return_value="first.jpg")
        manager._solve_exposure = Mock(return_value=False)
        manager._wait_for_capture_settle = Mock()
        with TemporaryDirectory() as directory, patch.object(config, "FINAL_SEQUENCE_DIRECTORY", directory):
            manager.pipelined_capture_loop()
        camera.capture_long_exposure.assert_called_once()
        manager._wait_for_capture_settle.assert_not_called()
        self.assertFalse(manager.capture_completed)
        self.assertEqual(motor.stop_count, 1)
        self.assertEqual(logger.finished[-1]["status"], "inter_exposure_solve_failed")
        self.assertEqual(logger.finished[-1]["final_images"], ["first.jpg"])

    def test_settle_timeout_continues_with_next_frame(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager._get_observation_time = lambda: 100.0
        camera.capture_long_exposure = Mock(side_effect=lambda *, filename, exposure_time: filename)
        manager._solve_exposure = Mock(return_value=True)
        manager._wait_for_capture_settle = Mock(side_effect=[False, True, True, True, True])
        with TemporaryDirectory() as directory, patch.object(config, "FINAL_SEQUENCE_DIRECTORY", directory):
            manager.pipelined_capture_loop()
        self.assertTrue(manager.capture_completed)
        self.assertEqual(logger.finished[-1]["status"], "completed")
        self.assertEqual(len(logger.finished[-1]["final_images"]), 6)
        self.assertEqual(camera.capture_long_exposure.call_count, 6)
        self.assertEqual(manager._solve_exposure.call_count, 5)
        settle = [event["details"] for event in logger.events if event.get("name") == "inter_exposure_settle"]
        self.assertEqual([(event["after_frame"], event["settled"]) for event in settle],
                         [(1, False), (2, True), (3, True), (4, True), (5, True)])
        self.assertEqual(motor.stop_count, 1)

    def test_stopping_during_settle_does_not_start_next_frame(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager._get_observation_time = lambda: 100.0
        camera.capture_long_exposure = Mock(return_value="first.jpg")
        manager._solve_exposure = Mock(return_value=True)

        def cancel(*args):
            manager.stop_event.set()
            return False

        manager._wait_for_capture_settle = Mock(side_effect=cancel)
        with TemporaryDirectory() as directory, patch.object(config, "FINAL_SEQUENCE_DIRECTORY", directory):
            manager.pipelined_capture_loop()
        camera.capture_long_exposure.assert_called_once()
        self.assertFalse(manager.capture_completed)
        self.assertEqual(logger.finished[-1]["status"], "capture_cancelled")
        self.assertEqual(logger.finished[-1]["final_images"], ["first.jpg"])

    def test_settle_requires_consecutive_valid_samples_on_both_axes(self):
        manager, motor_camera, motor, logger = make_runtime_manager()
        motor.current_alt, motor.current_az = 40.0, 359.99
        result = []
        waiter = threading.Thread(target=lambda: result.append(manager._wait_for_capture_settle(2.0, 5)))
        waiter.start()
        try:
            self.assertTrue(manager._capture_settling.wait(1))
            for _ in range(4):
                manager._record_capture_settle_sample(40.0, 0.01)
            self.assertEqual(manager._capture_settle_streak, 4)
            manager._record_capture_settle_sample(40.0, 0.1)
            self.assertEqual(manager._capture_settle_streak, 0)
            manager._record_capture_settle_sample(40.0, 0.01)
            manager._record_capture_settle_sample(40.0, 0.01, fresh=False)
            self.assertEqual(manager._capture_settle_streak, 0)
            for _ in range(5):
                manager._record_capture_settle_sample(40.0, 0.01)
            waiter.join(1)
            self.assertEqual(result, [True])
        finally:
            manager.stop_event.set()
            waiter.join(2)


if __name__ == "__main__":
    unittest.main()
