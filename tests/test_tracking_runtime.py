import unittest
from unittest.mock import patch

import config
from managers.state_manager import SystemState
from managers.tracking_manager import TrackingManager


class RuntimeCamera:
    def __init__(self):
        self.sequence_calls = []

    def capture_sequence(self, **kwargs):
        self.sequence_calls.append(kwargs)
        return ["frame_001.jpg", "frame_002.jpg"]

    def capture_long_exposure(self, **kwargs):
        raise AssertionError("Default capture mode must use a sequence.")

    def stop(self):
        pass


class RuntimeMotor:
    def __init__(self):
        self.stop_count = 0

    def stop(self):
        self.stop_count += 1

    def cleanup(self):
        pass


class RuntimeLogger:
    def __init__(self):
        self.events = []
        self.finished = []

    def record_event(self, **kwargs):
        self.events.append(kwargs)

    def finish_session(self, **kwargs):
        self.finished.append(kwargs)

    def record_plate_solution(self, **kwargs):
        self.events.append({"plate_solution": kwargs})

    def record_correction(self, correction):
        self.events.append({"correction": correction})


class RuntimeIMU:
    def close(self):
        pass


class RuntimeAstronomy:
    def get_current_time(self, utc_datetime=None):
        return utc_datetime if utc_datetime is not None else "system-utc"


def make_runtime_manager():
    camera = RuntimeCamera()
    motor = RuntimeMotor()
    logger = RuntimeLogger()
    manager = TrackingManager(
        camera=camera,
        motor=motor,
        astronomy=object(),
        platesolver=object(),
        logger=logger,
        imu=RuntimeIMU(),
        leveling_controller_factory=lambda *args, **kwargs: None,
    )
    manager._current_time_iso = lambda: "2026-08-03T00:00:00Z"
    return manager, camera, motor, logger


class TrackingRuntimeTests(unittest.TestCase):
    """Catch infinite solve retries and fallback to a single long exposure."""

    def test_three_consecutive_correction_failures_stop_session(self):
        manager, _camera, motor, _logger = make_runtime_manager()
        manager.is_tracking = True
        manager.state.set_state(SystemState.TRACKING)

        self.assertFalse(manager._record_correction_failure())
        self.assertFalse(manager._record_correction_failure())
        self.assertTrue(manager._record_correction_failure())

        self.assertFalse(manager.is_tracking)
        self.assertTrue(manager.stop_event.is_set())
        self.assertEqual(manager.state.get_state(), SystemState.PREVIEW)
        self.assertEqual(motor.stop_count, 1)

    def test_observation_time_uses_system_utc_without_gnss_time(self):
        manager, _camera, _motor, _logger = make_runtime_manager()
        manager.astronomy = RuntimeAstronomy()

        manager.set_time_provider(None)
        self.assertEqual(manager._get_observation_time(), "system-utc")

        manager.set_time_provider(lambda: None)
        self.assertEqual(manager._get_observation_time(), "system-utc")

    def test_observation_time_prefers_available_gnss_utc(self):
        manager, _camera, _motor, _logger = make_runtime_manager()
        manager.astronomy = RuntimeAstronomy()
        manager.set_time_provider(lambda: "gnss-utc")

        self.assertEqual(manager._get_observation_time(), "gnss-utc")

    def test_finish_sequence_uses_subframes_and_logs_every_path(self):
        manager, camera, motor, logger = make_runtime_manager()
        manager.is_tracking = True
        manager.state.set_state(SystemState.TRACKING)

        manager.finish_sequence()

        self.assertEqual(len(camera.sequence_calls), 1)
        call = camera.sequence_calls[0]
        self.assertEqual(call["frame_count"], config.FINAL_FRAME_COUNT)
        self.assertEqual(
            call["exposure_time"],
            config.FINAL_SUBEXPOSURE_SECONDS,
        )
        self.assertIs(call["cancel_event"], manager.stop_event)
        self.assertEqual(
            logger.finished[0]["final_images"],
            ["frame_001.jpg", "frame_002.jpg"],
        )
        self.assertIsNone(logger.finished[0]["final_image"])
        self.assertFalse(manager.is_tracking)
        self.assertTrue(manager.stop_event.is_set())
        self.assertEqual(motor.stop_count, 1)
        self.assertEqual(manager.state.get_state(), SystemState.PREVIEW)

    def test_final_capture_corrects_drift_at_configured_frame_interval(self):
        """Catch running the whole long-exposure sequence without feedback."""

        manager, camera, _motor, _logger = make_runtime_manager()
        manager.is_tracking = True
        manager.state.set_state(SystemState.TRACKING)
        solve_frames = []
        correction_frames = []

        def capture_sequence(**kwargs):
            after_frame = kwargs.get("after_frame")
            for frame_number in range(1, 6):
                if after_frame is not None:
                    manager._test_active_frame = frame_number
                    after_frame(frame_number, f"frame_{frame_number:03d}.jpg")
            return [f"frame_{number:03d}.jpg" for number in range(1, 7)]

        camera.capture_sequence = capture_sequence

        def capture_and_solve():
            solve_frames.append(manager._test_active_frame)
            return {"ra": 1.0, "dec": 2.0}

        def correct(result, observation_time=None):
            correction_frames.append(manager._test_active_frame)
            return {"altitude_error": 0.1, "azimuth_error": -0.1}

        manager.capture_and_solve = capture_and_solve
        manager.drift_correction = correct

        with patch.object(
            config,
            "FINAL_DRIFT_CORRECTION_EVERY_FRAMES",
            3,
            create=True,
        ):
            manager.finish_sequence()

        self.assertEqual(solve_frames, [3])
        self.assertEqual(correction_frames, [3])


if __name__ == "__main__":
    unittest.main()
