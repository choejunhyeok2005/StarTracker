"""
camera/camera_manager.py

Raspberry Pi HQ Camera Manager

담당 기능:
- 카메라 초기화
- Plate Solving용 이미지 촬영
- 최종 장노출 촬영
- 카메라 자원 종료
"""

import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

import cv2

import config


class CameraManager:
    """Raspberry Pi HQ Camera의 촬영 작업을 관리한다."""

    def __init__(
        self,
        camera_factory: Optional[Callable[[], Any]] = None,
    ):

        self.camera: Optional[Any] = None
        self._camera_factory = camera_factory
        self.initialized = False

        # 두 스레드가 카메라를 동시에 사용하는 것을 방지
        self._camera_lock = threading.Lock()

        # 스트리밍 상태 플래그
        self._streaming = False

    # ==================================================
    # Initialize
    # ==================================================

    def initialize(self) -> None:
        """카메라를 생성하고 Plate Solving 촬영 설정으로 시작한다."""

        with self._camera_lock:

            if self.initialized:
                return

            if self._camera_factory is None:
                try:
                    from picamera2 import Picamera2
                except ImportError as error:
                    raise RuntimeError(
                        "Picamera2 is required on the Raspberry Pi."
                    ) from error

                camera_factory = Picamera2
            else:
                camera_factory = self._camera_factory

            self.camera = camera_factory()

            plate_config = self.camera.create_still_configuration(
                main={
                    "size": (
                        config.CAMERA_IMAGE_WIDTH,
                        config.CAMERA_IMAGE_HEIGHT,
                    ),
                    "format": config.CAMERA_FORMAT,
                },
                buffer_count=2,
            )

            self.camera.configure(plate_config)
            self.camera.start()

            time.sleep(config.CAMERA_WARMUP_SECONDS)

            self.initialized = True

            print("[Camera] Ready")

    # ==================================================
    # Streaming (MJPEG Web Feed)
    # ==================================================

    def start_streaming(self) -> None:
        """Preview 설정으로 스트리밍을 시작한다."""

        with self._camera_lock:

            if self._streaming:
                return

            self._ensure_camera()

            # 스트리밍용 preview 설정
            stream_config = self.camera.create_preview_configuration(
                main={
                    "size": config.STREAMING_RESOLUTION,
                    "format": config.CAMERA_FORMAT,
                },
            )

            self.camera.configure(stream_config)
            self.camera.start()

            time.sleep(config.CAMERA_WARMUP_SECONDS)

            self._streaming = True

            print("[Camera] Streaming started")

    def stop_streaming(self) -> None:
        """스트리밍을 중지한다."""

        with self._camera_lock:

            if not self._streaming or self.camera is None:
                return

            try:
                if self.camera.started:
                    self.camera.stop()
            except Exception as error:
                print(f"[Camera] Streaming stop warning: {error}")

            self._streaming = False

            print("[Camera] Streaming stopped")

    def generate_frames(self):
        """Flask MJPEG 스트림용 제너레이터."""

        if not self._streaming:
            return

        while self._streaming:
            try:
                with self._camera_lock:
                    if self.camera is None or not self._streaming:
                        break

                    frame = self.camera.capture_array()

                ret, buf = cv2.imencode(".jpg", frame, [
                    cv2.IMWRITE_JPEG_QUALITY,
                    config.STREAMING_JPEG_QUALITY
                ])

                if not ret:
                    continue

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + buf.tobytes()
                    + b"\r\n"
                )

            except Exception as error:
                print(f"[Camera] Frame generation error: {error}")
                break

    # ==================================================
    # Plate Solving Capture
    # ==================================================

    def capture_plate(
        self,
        filename: Optional[str] = None,
    ) -> str:
        """Plate Solving에 사용할 이미지를 촬영한다."""

        self._require_initialized()

        if filename is None:
            filename = config.PLATE_IMAGE_PATH

        filepath = Path(filename)
        filepath.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        with self._camera_lock:

            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "ExposureTime": int(
                        config.PLATE_EXPOSURE_SECONDS
                        * 1_000_000
                    ),
                    "AnalogueGain": config.PLATE_ANALOGUE_GAIN,
                }
            )

            # 설정 반영 대기
            time.sleep(config.CAMERA_CONTROL_SETTLE_SECONDS)

            self.camera.capture_file(str(filepath))

        print(f"[Camera] Plate image saved: {filepath}")

        return str(filepath)

    # ==================================================
    # Field-Rotation-Safe Capture Sequence
    # ==================================================

    def capture_sequence(
        self,
        directory: Optional[str] = None,
        frame_count: Optional[int] = None,
        exposure_time: Optional[float] = None,
        cancel_event=None,
    ) -> list[str]:
        """Capture short subframes, stopping cleanly between frames."""

        if directory is None:
            directory = config.FINAL_SEQUENCE_DIRECTORY
        if frame_count is None:
            frame_count = config.FINAL_FRAME_COUNT
        if exposure_time is None:
            exposure_time = config.FINAL_SUBEXPOSURE_SECONDS

        frame_count = int(frame_count)
        exposure_time = float(exposure_time)

        if frame_count <= 0:
            raise ValueError("frame_count must be greater than zero.")
        if exposure_time <= 0:
            raise ValueError("exposure_time must be greater than zero.")

        if cancel_event is not None and cancel_event.is_set():
            return []

        output_directory = Path(directory)
        output_directory.mkdir(parents=True, exist_ok=True)
        sequence_id = datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S_%fZ"
        )
        completed_paths = []

        for frame_number in range(1, frame_count + 1):
            if cancel_event is not None and cancel_event.is_set():
                break

            filepath = output_directory / (
                f"final_{sequence_id}_{frame_number:03d}.jpg"
            )
            completed_paths.append(
                self.capture_long_exposure(
                    filename=str(filepath),
                    exposure_time=exposure_time,
                )
            )

        return completed_paths

    # ==================================================
    # Long Exposure Capture
    # ==================================================

    def capture_long_exposure(
        self,
        filename: Optional[str] = None,
        exposure_time: Optional[float] = None,
    ) -> str:
        """최종 결과용 장노출 이미지를 촬영한다."""

        self._require_initialized()

        if filename is None:
            filename = config.FINAL_IMAGE_PATH

        if exposure_time is None:
            exposure_time = config.LONG_EXPOSURE_SECONDS

        if exposure_time <= 0:
            raise ValueError(
                "exposure_time must be greater than zero."
            )

        filepath = Path(filename)
        filepath.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        exposure_microseconds = int(
            exposure_time * 1_000_000
        )

        with self._camera_lock:

            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "ExposureTime": exposure_microseconds,
                    "AnalogueGain": config.LONG_EXPOSURE_ANALOGUE_GAIN,
                }
            )

            time.sleep(config.CAMERA_CONTROL_SETTLE_SECONDS)

            print(
                "[Camera] Long exposure started: "
                f"{exposure_time:.1f} sec"
            )

            # capture_file은 노출 및 파일 저장이 끝날 때까지 대기
            self.camera.capture_file(str(filepath))

        print(f"[Camera] Final image saved: {filepath}")

        return str(filepath)

    # ==================================================
    # Stop
    # ==================================================

    def stop(self) -> None:
        """카메라를 안전하게 정지한다."""

        with self._camera_lock:

            if self.camera is None:
                return

            try:
                if self.camera.started:
                    self.camera.stop()

            except Exception as error:
                print(f"[Camera] Stop warning: {error}")

            finally:
                try:
                    self.camera.close()
                except Exception:
                    pass

                self.camera = None
                self.initialized = False

        print("[Camera] Stopped")

    # ==================================================
    # Internal Validation
    # ==================================================

    def _require_initialized(self) -> None:

        if not self.initialized or self.camera is None:
            raise RuntimeError(
                "CameraManager is not initialized. "
                "Call initialize() first."
            )

    def _ensure_camera(self) -> None:
        """카메라 객체가 없으면 생성한다."""

        if self.camera is None:
            if self._camera_factory is None:
                try:
                    from picamera2 import Picamera2
                except ImportError as error:
                    raise RuntimeError(
                        "Picamera2 is required on the Raspberry Pi."
                    ) from error

                camera_factory = Picamera2
            else:
                camera_factory = self._camera_factory

            self.camera = camera_factory()
