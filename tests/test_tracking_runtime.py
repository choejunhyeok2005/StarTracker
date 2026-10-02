import unittest

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


class RuntimeIMU:
    def close(self):
        pass


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


if __name__ == "__main__":
    unittest.main()
