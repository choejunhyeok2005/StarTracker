import tempfile
import threading
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
