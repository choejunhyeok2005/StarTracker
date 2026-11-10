import unittest
from tempfile import TemporaryDirectory
from pathlib import Path
from unittest.mock import Mock

from camera.camera_manager import CameraManager


class LongExposureCameraTests(unittest.TestCase):
    def test_same_exposure_keeps_camera_running_and_saves_distinct_files(self):
        camera = Mock()
        camera.started = True
        requests = []
        def capture():
            request = Mock()
            request.get_metadata.return_value = {"ExposureTime": 110_000_000}
            request.save.side_effect = lambda stream, path: Path(path).write_bytes(b"jpg")
            request.save_dng.side_effect = lambda path: Path(path).write_bytes(b"raw")
            requests.append(request)
            return request
        camera.capture_request.side_effect = capture
        manager = CameraManager(camera_factory=lambda: camera)
        manager.camera, manager.initialized = camera, True
        with TemporaryDirectory() as directory:
            for number in range(3):
                manager.capture_long_exposure(
                    filename=str(Path(directory) / f"{number}.jpg"), exposure_time=110,
                )
            self.assertEqual(len(list(Path(directory).glob("*.jpg"))), 3)
            self.assertEqual(len(list(Path(directory).glob("*.dng"))), 3)
        camera.configure.assert_called_once()
        camera.start.assert_called_once()
        camera.stop.assert_called_once()
        self.assertEqual(camera.capture_request.call_count, 3)
        camera.capture_file.assert_not_called()
        for request in requests:
            request.save.assert_called_once()
            request.save_dng.assert_called_once()
            request.release.assert_called_once()
            jpg = Path(request.save.call_args.args[1])
            self.assertEqual(Path(request.save_dng.call_args.args[0]), jpg.with_suffix(".dng"))
        self.assertEqual(manager.last_exposure_metadata, {"ExposureTime": 110_000_000})
        settings = camera.create_still_configuration.call_args.kwargs
        self.assertEqual(settings["raw"], {})
        self.assertFalse(settings["queue"])
        self.assertEqual(settings["controls"]["ExposureTime"], 110_000_000)
        self.assertEqual(settings["controls"]["FrameDurationLimits"],
                         (110_000_000, 110_000_000))


    def test_save_failure_releases_frame_and_propagates(self):
        for method in ("save", "save_dng"):
            with self.subTest(method=method), TemporaryDirectory() as directory:
                camera = Mock()
                request = camera.capture_request.return_value
                getattr(request, method).side_effect = OSError("disk full")
                manager = CameraManager()
                manager.camera, manager.initialized = camera, True
                with self.assertRaisesRegex(OSError, "disk full"):
                    manager.capture_long_exposure(str(Path(directory) / "frame.jpg"), 1)
                request.release.assert_called_once()

    def test_plate_capture_saves_same_frame_raw(self):
        from unittest.mock import patch
        camera = Mock()
        manager = CameraManager()
        manager.camera, manager.initialized = camera, True
        with TemporaryDirectory() as directory, patch("camera.camera_manager.time.sleep"):
            path = str(Path(directory) / "plate.jpg")
            self.assertEqual(manager.capture_plate(path), path)
            request = camera.capture_request.return_value
            request.save.assert_called_once_with("main", path)
            request.save_dng.assert_called_once_with(str(Path(path).with_suffix(".dng")))
            request.release.assert_called_once()
            self.assertEqual(camera.create_still_configuration.call_args.kwargs["raw"], {})


if __name__ == "__main__":
    unittest.main()
