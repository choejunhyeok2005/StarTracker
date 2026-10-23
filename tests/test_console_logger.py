import importlib
import io
import sys
import tempfile
import unittest
from pathlib import Path


class ConsoleLoggerTests(unittest.TestCase):
    def test_stdout_and_stderr_are_tee_saved_to_one_session_file(self):
        """Catch console messages disappearing when file logging is enabled."""

        try:
            console_logger = importlib.import_module(
                "services.console_logger"
            )
        except ModuleNotFoundError as error:
            self.fail(f"console logger module is missing: {error}")

        stdout = io.StringIO()
        stderr = io.StringIO()

        with tempfile.TemporaryDirectory() as directory:
            session = console_logger.start_console_file_logging(
                directory=directory,
                stdout=stdout,
                stderr=stderr,
            )
            try:
                print("stdout message")
                print("stderr message", file=sys.stderr)
            finally:
                session.close()

            saved = Path(session.path).read_text(encoding="utf-8")

        self.assertIn("stdout message", stdout.getvalue())
        self.assertIn("stderr message", stderr.getvalue())
        self.assertIn("stdout message", saved)
        self.assertIn("stderr message", saved)


if __name__ == "__main__":
    unittest.main()
