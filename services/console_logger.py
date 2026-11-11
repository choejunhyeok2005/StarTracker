"""Mirror process console output to a timestamped UTF-8 log file."""

import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, TextIO


class _TeeStream:
    def __init__(
        self,
        console: TextIO,
        logfile: TextIO,
        lock: threading.Lock,
    ) -> None:
        self._console = console
        self._logfile = logfile
        self._lock = lock

    @property
    def encoding(self):
        return getattr(self._console, "encoding", "utf-8")

    @property
    def errors(self):
        return getattr(self._console, "errors", None)

    def write(self, text: str) -> int:
        with self._lock:
            written = self._console.write(text)
            self._logfile.write(text)
            self._logfile.flush()
        return written

    def flush(self) -> None:
        with self._lock:
            self._console.flush()
            self._logfile.flush()

    def isatty(self) -> bool:
        return bool(getattr(self._console, "isatty", lambda: False)())

    def fileno(self) -> int:
        return self._console.fileno()


class ConsoleLogSession:
    def __init__(
        self,
        path: Path,
        logfile: TextIO,
        stdout: TextIO,
        stderr: TextIO,
    ) -> None:
        self.path = str(path)
        self._logfile = logfile
        self._stdout = stdout
        self._stderr = stderr
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return

        if isinstance(sys.stdout, _TeeStream):
            sys.stdout = self._stdout
        if isinstance(sys.stderr, _TeeStream):
            sys.stderr = self._stderr

        self._logfile.flush()
        self._logfile.close()
        self._closed = True


def start_console_file_logging(
    directory: str,
    *,
    stdout: Optional[TextIO] = None,
    stderr: Optional[TextIO] = None,
) -> ConsoleLogSession:
    """Start one process-wide console tee and return its lifecycle handle."""

    output_directory = Path(directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S_%fZ"
    )
    path = output_directory / f"console_{timestamp}.log"
    logfile = path.open("w", encoding="utf-8", buffering=1)
    original_stdout = stdout if stdout is not None else sys.stdout
    original_stderr = stderr if stderr is not None else sys.stderr
    lock = threading.Lock()

    sys.stdout = _TeeStream(original_stdout, logfile, lock)
    sys.stderr = _TeeStream(original_stderr, logfile, lock)

    return ConsoleLogSession(
        path=path,
        logfile=logfile,
        stdout=original_stdout,
        stderr=original_stderr,
    )
