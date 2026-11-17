import unittest
from tempfile import TemporaryDirectory
from pathlib import Path
from unittest.mock import Mock

from camera.camera_manager import CameraManager


class LongExposureCameraTests(unittest.TestCase):
    def test_same_exposure_keeps_camera_running_and_saves_distinct_files(self):
        camera = Mock()
        camera.started = True
        def capture(path):
            Path(path).write_bytes(b"test-image")
            return {"ExposureTime": 110_000_000}
        camera.capture_file.side_effect = capture
        manager = CameraManager(camera_factory=lambda: camera)
        manager.camera, manager.initialized = camera, True
        with TemporaryDirectory() as directory:
            for number in range(3):
                manager.capture_long_exposure(
                    filename=str(Path(directory) / f"{number}.jpg"), exposure_time=110,
                )
            self.assertEqual(len(list(Path(directory).glob("*.jpg"))), 3)
        camera.configure.assert_called_once()
        camera.start.assert_called_once()
        camera.stop.assert_called_once()
        self.assertEqual(camera.capture_file.call_count, 3)
        settings = camera.create_still_configuration.call_args.kwargs
        self.assertFalse(settings["queue"])
        self.assertEqual(settings["controls"]["ExposureTime"], 110_000_000)
        self.assertEqual(settings["controls"]["FrameDurationLimits"],
                         (110_000_000, 110_000_000))


if __name__ == "__main__":
    unittest.main()
