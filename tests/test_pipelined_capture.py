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
            config, PIPELINED_FRAME_COUNT=3, PIPELINED_EXPOSURE_SECONDS=110,
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
            config, PIPELINED_FRAME_COUNT=3, FINAL_SEQUENCE_DIRECTORY=directory,
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


if __name__ == "__main__":
    unittest.main()
