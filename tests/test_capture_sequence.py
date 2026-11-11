import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import config
from camera.camera_manager import CameraManager


class CaptureSequenceTests(unittest.TestCase):
    """Catch missing frames, duplicate paths, and ignored cancellation."""

    def test_sequence_captures_requested_number_of_unique_frames(self):
        manager = CameraManager(camera_factory=lambda: None)
        calls = []

        def fake_capture(*, filename, exposure_time):
            calls.append((filename, exposure_time))
            return filename

        manager.capture_long_exposure = fake_capture

        with tempfile.TemporaryDirectory() as directory:
            paths = manager.capture_sequence(
                directory=directory,
                frame_count=3,
                exposure_time=10.0,
                cancel_event=threading.Event(),
            )

        self.assertEqual(len(paths), 3)
        self.assertEqual(len(set(paths)), 3)
        self.assertTrue(Path(paths[0]).name.endswith("_001.jpg"))
        self.assertTrue(Path(paths[2]).name.endswith("_003.jpg"))
        self.assertEqual([call[1] for call in calls], [10.0, 10.0, 10.0])

    def test_pre_cancelled_sequence_captures_no_frames(self):
        manager = CameraManager(camera_factory=lambda: None)
        calls = []
        manager.capture_long_exposure = lambda **kwargs: calls.append(kwargs)
        cancel_event = threading.Event()
        cancel_event.set()

        paths = manager.capture_sequence(
            directory="unused",
            frame_count=3,
            exposure_time=10.0,
            cancel_event=cancel_event,
        )

        self.assertEqual(paths, [])
        self.assertEqual(calls, [])

    def test_sequence_waits_configured_interval_between_frames(self):
        """Catch ignoring the configurable long-exposure cadence."""

        class RecordingCancelEvent:
            def __init__(self):
                self.waits = []

            def is_set(self):
                return False

            def wait(self, timeout):
                self.waits.append(timeout)
                return False

        manager = CameraManager(camera_factory=lambda: None)
        manager.capture_long_exposure = (
            lambda *, filename, exposure_time: filename
        )
        cancel_event = RecordingCancelEvent()

        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(
                config,
                "FINAL_FRAME_INTERVAL_SECONDS",
                2.5,
                create=True,
            ),
        ):
            manager.capture_sequence(
                directory=directory,
                frame_count=3,
                exposure_time=10.0,
                cancel_event=cancel_event,
            )

        self.assertEqual(cancel_event.waits, [2.5, 2.5])

    def test_sequence_reports_completed_frames_before_the_last_frame(self):
        """Catch losing the hook used for between-frame drift correction."""

        manager = CameraManager(camera_factory=lambda: None)
        manager.capture_long_exposure = (
            lambda *, filename, exposure_time: filename
        )
        completed = []

        with tempfile.TemporaryDirectory() as directory:
            try:
                manager.capture_sequence(
                    directory=directory,
                    frame_count=4,
                    exposure_time=10.0,
                    after_frame=lambda number, path: completed.append(
                        (number, Path(path).name)
                    ),
                    cancel_event=threading.Event(),
                )
            except TypeError as error:
                self.fail(f"capture_sequence rejected after_frame: {error}")

        self.assertEqual(
            [number for number, _path in completed],
            [1, 2, 3],
        )
        self.assertTrue(completed[0][1].endswith("_001.jpg"))


if __name__ == "__main__":
    unittest.main()
