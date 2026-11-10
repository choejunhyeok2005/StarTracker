"""
services/tracking_logger.py

Tracking 세션 로그와 Plate Solving 결과를 JSON으로 저장한다.
"""

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import config


class TrackingLogger:
    """한 번의 Target Tracking 세션 기록을 관리한다."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active = False
        self._tracking_data: dict = {}
        self._plate_data: dict = {}
        self._tracking_path: Optional[Path] = None
        self._plate_path: Optional[Path] = None

    def start_session(
        self,
        observer: dict,
        target: dict,
        started_at_utc: str,
    ) -> None:
        """새 Tracking 세션을 만들고 초기 상태를 저장한다."""

        session_id = datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S_%fZ"
        )

        output_dir = Path(config.TRACKING_LOG_DIRECTORY)

        with self._lock:
            self._tracking_path = output_dir / (
                f"tracking_{session_id}.json"
            )
            self._plate_path = output_dir / (
                f"platesolving_{session_id}.json"
            )

            self._tracking_data = {
                "session_id": session_id,
                "status": "tracking",
                "started_at_utc": started_at_utc,
                "ended_at_utc": None,
                "observer": observer,
                "target": target,
                "alignment": None,
                "corrections": [],
                "events": [],
                "final_image": None,
                "final_images": [],
            }

            self._plate_data = {
                "session_id": session_id,
                "target": target,
                "solutions": [],
            }

            self._active = True
            self._write_locked()

    def record_event(
        self,
        name: str,
        timestamp_utc: str,
        details: Optional[dict] = None,
    ) -> None:
        """Tracking 상태 변화나 오류를 기록한다."""

        with self._lock:
            if not self._active:
                return

            event = {
                "name": name,
                "timestamp_utc": timestamp_utc,
            }

            if details:
                event["details"] = details

            self._tracking_data["events"].append(event)
            self._write_locked()

    def record_alignment(self, alignment: dict) -> None:
        """Store the completed startup IMU alignment result."""

        with self._lock:
            if not self._active:
                return

            self._tracking_data["alignment"] = dict(alignment)
            self._write_locked()

    def record_plate_solution(
        self,
        stage: str,
        result: dict,
        timestamp_utc: str,
        correction_number: Optional[int] = None,
    ) -> None:
        """Target 또는 Drift Correction Plate Solving 결과를 저장한다."""

        with self._lock:
            if not self._active:
                return

            solution = {
                "stage": stage,
                "timestamp_utc": timestamp_utc,
                "result": result,
            }

            if correction_number is not None:
                solution["correction_number"] = correction_number

            self._plate_data["solutions"].append(solution)
            self._write_locked()

    def record_correction(self, correction: dict) -> None:
        """한 번의 Drift Correction 계산값을 기록한다."""

        with self._lock:
            if not self._active:
                return

            self._tracking_data["corrections"].append(correction)
            self._write_locked()

    def finish_session(
        self,
        status: str,
        ended_at_utc: str,
        final_image: Optional[str] = None,
        final_images: Optional[list[str]] = None,
    ) -> None:
        """세션을 종료하고 두 JSON 파일을 최종 저장한다."""

        with self._lock:
            if not self._active:
                return

            self._tracking_data["status"] = status
            self._tracking_data["ended_at_utc"] = ended_at_utc
            self._tracking_data["final_image"] = final_image
            self._tracking_data["final_images"] = list(
                final_images or []
            )

            self._write_locked()
            self._active = False

            print(
                "[TrackingLog] Saved: "
                f"{self._tracking_path}"
            )
            print(
                "[TrackingLog] Plate results saved: "
                f"{self._plate_path}"
            )

    def _write_locked(self) -> None:
        """Lock을 잡은 상태에서 JSON 파일을 원자적으로 갱신한다."""

        if self._tracking_path is None or self._plate_path is None:
            return

        self._write_json(
            self._tracking_path,
            self._tracking_data,
        )
        self._write_json(
            self._plate_path,
            self._plate_data,
        )

    @staticmethod
    def _write_json(path: Path, data: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)

        temporary_path = path.with_suffix(
            path.suffix + ".tmp"
        )

        temporary_path.write_text(
            json.dumps(
                data,
                ensure_ascii=False,
                indent=2,
                default=str,
            ),
            encoding="utf-8",
        )

        temporary_path.replace(path)
