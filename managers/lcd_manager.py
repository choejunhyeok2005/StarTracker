from __future__ import annotations

import sys
import threading
import time
from collections import deque
from typing import Deque, Optional

try:
    from RPLCD.i2c import CharLCD
except ImportError:  # pragma: no cover
    CharLCD = None


class LCDManager:
    def __init__(
        self,
        i2c_expander: str = "PCF8574",
        address: int = 0x27,
        port: int = 1,
        cols: int = 20,
        rows: int = 4,
        backlight_enabled: bool = True,
        rotation_interval: float = 4.0, #LCD 줄 바꿈 시간
        max_queue_size: int = 50,
    ) -> None:
        self.cols = cols
        self.rows = rows
        self._buffer: Deque[str] = deque(maxlen=rows)
        self._current_line: str = ""
        self._lock = threading.RLock()
        self._enabled = False
        self._long_exposure = False
        self._backlight_enabled = backlight_enabled
        self._lcd: Optional[CharLCD] = None
        self._init_args = {
            "i2c_expander": i2c_expander,
            "address": address,
            "port": port,
            "cols": cols,
            "rows": rows,
        }

        # Pages queue and worker
        self.rotation_interval = float(rotation_interval)
        # each page is a list[str] with up to `rows` items
        self.pages: deque[list[str]] = deque(maxlen=max_queue_size)
        self._worker_stop = threading.Event()
        self._worker_event = threading.Event()
        self._worker_thread = threading.Thread(target=self._rotation_worker, daemon=True)

        self.initialize()
        self._worker_thread.start()

    def initialize(self) -> None:
        if CharLCD is None:
            self._enabled = False
            return

        try:
            self._lcd = CharLCD(**self._init_args)
            self._enabled = True
            self.clear()
            self.set_backlight(self._backlight_enabled)
        except Exception:  # pragma: no cover
            self._enabled = False

    def is_enabled(self) -> bool:
        return self._enabled

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()
            self._current_line = ""
            if self._enabled and self._lcd is not None:
                try:
                    self._lcd.clear()
                except Exception:
                    pass

    def set_backlight(self, enabled: bool) -> None:
        with self._lock:
            self._backlight_enabled = enabled
            if not self._enabled or self._lcd is None:
                return
            if self._long_exposure and enabled:
                return
            try:
                self._lcd.backlight_enabled = enabled
            except Exception:
                pass

    def set_long_exposure(self, active: bool) -> None:
        with self._lock:
            self._long_exposure = active
            if not self._enabled or self._lcd is None:
                return
            try:
                self._lcd.backlight_enabled = False if active else self._backlight_enabled
            except Exception:
                pass

    def close(self) -> None:
        # Stop worker thread cleanly
        try:
            self._worker_stop.set()
            self._worker_event.set()
            if self._worker_thread.is_alive():
                self._worker_thread.join(timeout=1.0)
        except Exception:
            pass

        with self._lock:
            if not self._enabled or self._lcd is None:
                self._enabled = False
                self._lcd = None
                return
            try:
                self._lcd.close()
            except Exception:
                pass
            self._enabled = False
            self._lcd = None

    def drain_and_show_remaining(self, per_page_delay: float | None = None, timeout: float | None = None) -> None:
        """
        Synchronously display any queued pages one-by-one (honoring per_page_delay)
        and then leave the last page visible. Intended to be called at shutdown to
        ensure recent messages are shown on the LCD before clearing.

        - per_page_delay: seconds to wait between pages (defaults to rotation_interval)
        - timeout: overall timeout in seconds for the whole drain operation (optional)
        """
        if per_page_delay is None:
            per_page_delay = self.rotation_interval

        deadline = None if timeout is None else (time.time() + timeout)

        pages_to_show: list[list[str]] = []
        with self._lock:
            while self.pages:
                pages_to_show.append(self.pages.popleft())

        for page in pages_to_show:
            # check timeout
            if deadline is not None and time.time() > deadline:
                break
            self._display_page(page)
            # ensure at least small pause so human can read
            time.sleep(per_page_delay)

        # If no pages but we already have a last page displayed by worker, keep it.

    def write_text(self, text: str, end: str = "\n") -> None:
        """
        Enqueue logical lines for rotation display. Each logical line longer than
        `cols` is split into chunks of `cols` and enqueued in order.
        """
        if not self._enabled:
            return

        raw = (self._current_line or "") + text
        lines = []
        while "\n" in raw:
            line, raw = raw.split("\n", 1)
            lines.append(line)
        self._current_line = raw

        if "\n" in end:
            lines.append(self._current_line)
            self._current_line = ""

        # Split long lines into chunks of width `cols`
        chunks: list[str] = []
        for line in lines:
            if line == "":
                chunks.append("")
                continue
            for i in range(0, len(line), self.cols):
                chunks.append(line[i : i + self.cols])

        if not chunks:
            return

        # If carriage-return style update requested, treat as immediate overwrite
        if "\r" in end:
            # join chunks into a single line (progress updates typically single-line)
            single = "".join(chunks)
            page = [single]
            with self._lock:
                # replace existing pages with this single immediate page
                self.pages.clear()
                self.pages.append(page)
                self._worker_event.set()
            # reset current line buffer
            self._current_line = ""
            return

        # Group chunks into pages (each page up to `rows` lines)
        new_pages: list[list[str]] = []
        page: list[str] = []
        for chunk in chunks:
            page.append(chunk)
            if len(page) >= self.rows:
                new_pages.append(page)
                page = []
        if page:
            new_pages.append(page)

        if not new_pages:
            return

        with self._lock:
            # Prioritize new pages: discard any existing pages and replace with new content
            self.pages.clear()
            for p in new_pages:
                self.pages.append(p)
            # Wake worker to show new content immediately
            self._worker_event.set()

    def _display_page(self, page: list[str]) -> None:
        # Render up to `rows` lines for a page; pad with blanks
        if not self._enabled or self._lcd is None:
            return
        try:
            self._lcd.clear()
            for r in range(self.rows):
                line = page[r] if r < len(page) else ""
                self._lcd.cursor_pos = (r, 0)
                self._lcd.write_string(line.ljust(self.cols)[: self.cols])
        except Exception:
            pass

    def _rotation_worker(self) -> None:
        while not self._worker_stop.is_set():
            try:
                # Wait until there's a page or stop requested
                if not self.pages:
                    # block until event or timeout
                    self._worker_event.wait(timeout=0.5)
                    self._worker_event.clear()
                    continue

                if self._long_exposure:
                    # Do not show anything during long exposures; wait for event
                    self._worker_event.wait(timeout=0.5)
                    self._worker_event.clear()
                    continue

                with self._lock:
                    if not self.pages:
                        continue
                    # Peek at leftmost page without removing
                    current_pages_count = len(self.pages)
                    page = self.pages[0]

                # Display current page
                self._display_page(page)

                if current_pages_count <= 1:
                    # Only one page -> display fixed until new content arrives
                    # wait until new pages arrive or stop requested
                    self._worker_event.wait()
                    self._worker_event.clear()
                    continue

                # Rotate: move leftmost page to right end
                with self._lock:
                    try:
                        moved = self.pages.popleft()
                        self.pages.append(moved)
                    except IndexError:
                        pass

                # Wait but wake early if new content arrives
                self._worker_event.wait(timeout=self.rotation_interval)
                self._worker_event.clear()
            except Exception:
                # Swallow exceptions in worker to keep it alive
                time.sleep(0.5)


lcd_manager = LCDManager()

_original_print = None


def install_print_hook() -> None:
    global _original_print
    if _original_print is not None:
        return

    _original_print = __builtins__["print"] if isinstance(__builtins__, dict) else __builtins__.print
    builtins_print = _original_print

    def lcd_print(*args, sep=" ", end="\n", file=None, flush=False, **kwargs):
        builtins_print(*args, sep=sep, end=end, file=file, flush=flush, **kwargs)
        if file is None or file is sys.stdout:
            try:
                message = sep.join(str(arg) for arg in args)
                lcd_manager.write_text(message, end=end)
            except Exception:
                pass

    if isinstance(__builtins__, dict):
        __builtins__["print"] = lcd_print
    else:
        __builtins__.print = lcd_print


def restore_print_hook() -> None:
    global _original_print
    if _original_print is None:
        return
    if isinstance(__builtins__, dict):
        __builtins__["print"] = _original_print
    else:
        __builtins__.print = _original_print
    _original_print = None
