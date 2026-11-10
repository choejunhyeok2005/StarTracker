import json
import tempfile
import unittest
from pathlib import Path

import config
from services.tracking_logger import TrackingLogger


class TrackingLoggerTests(unittest.TestCase):
    """Catch loss of alignment evidence and multi-frame output paths."""

    def test_session_records_alignment_and_final_images(self):
        original_directory = config.TRACKING_LOG_DIRECTORY

        with tempfile.TemporaryDirectory() as directory:
            config.TRACKING_LOG_DIRECTORY = directory

            try:
                logger = TrackingLogger()
                logger.start_session(
                    observer={"latitude": 37.5},
                    target={"ra": 10.0, "dec": 20.0},
                    started_at_utc="2026-08-03T00:00:00Z",
                )
                logger.record_alignment(
                    {
                        "final_error_deg": 0.05,
                        "iterations": 4,
                    }
                )
                logger.finish_session(
                    status="completed",
                    ended_at_utc="2026-08-03T00:03:00Z",
                    final_images=["frame_001.jpg", "frame_002.jpg"],
                )

                tracking_path = next(
                    Path(directory).glob("tracking_*.json")
                )
                data = json.loads(tracking_path.read_text(encoding="utf-8"))
            finally:
                config.TRACKING_LOG_DIRECTORY = original_directory

        self.assertEqual(data["alignment"]["final_error_deg"], 0.05)
        self.assertEqual(
            data["final_images"],
            ["frame_001.jpg", "frame_002.jpg"],
        )
        self.assertIsNone(data["final_image"])


if __name__ == "__main__":
    unittest.main()
