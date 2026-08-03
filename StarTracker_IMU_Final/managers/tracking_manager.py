"""
managers/tracking_manager.py

StarTracker Tracking Manager

담당 기능:
- Preview 수동 이동
- 최초 Plate Solving 및 목표 RA/Dec 저장
- Alt-Az 기반 연속 추적
- 주기적 Plate Solving
- Drift Correction
- 최종 장노출 촬영
"""

import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from typing import Callable, Optional

import config

from controllers.imu_leveling_controller import (
    AlignmentSettings,
    IMULevelingController,
)
from hardware.imu import MPU6050Pair
from managers.state_manager import StateManager, SystemState


class TrackingManager:
    """StarTracker의 전체 추적 흐름을 관리한다."""

    def __init__(
        self,
        *,
        camera=None,
        motor=None,
        astronomy=None,
        platesolver=None,
        logger=None,
        imu=None,
        leveling_controller_factory=None,
    ):

        # GNSS UTC를 받아오는 함수
        self._time_provider: Optional[Callable] = None

        # 하드웨어 없는 테스트가 가능한 지점에서 실제 구현을 지연 로드한다.
        if camera is None:
            from camera.camera_manager import CameraManager

            camera = CameraManager()
        if motor is None:
            from controllers.motor_controller import MotorController

            motor = MotorController()
        if astronomy is None:
            from services.astronomy import Astronomy

            astronomy = Astronomy()
        if platesolver is None:
            from services.platesolver import PlateSolver

            platesolver = PlateSolver()
        if logger is None:
            from services.tracking_logger import TrackingLogger

            logger = TrackingLogger()
        if imu is None:
            imu = MPU6050Pair(
                bus_number=config.IMU_I2C_BUS,
                base_address=config.IMU_BASE_ADDRESS,
                upper_address=config.IMU_UPPER_ADDRESS,
            )

        self.camera = camera
        self.motor = motor
        self.astronomy = astronomy
        self.platesolver = platesolver
        self.logger = logger
        self.imu = imu
        self._leveling_controller_factory = (
            leveling_controller_factory or IMULevelingController
        )
        self.state = StateManager()
        self.alignment_result = None
        self._hardware_initialized = False
        self._observer_configured = False

        self.observer = {
            "latitude": None,
            "longitude": None,
            "altitude": None,
        }

        # 추적 대상의 고정 천구 좌표
        self.target_ra: Optional[float] = None
        self.target_dec: Optional[float] = None

        # 추적 상태
        self.is_tracking = False
        self.correction_count = 0
        self.consecutive_correction_failures = 0

        # 스레드 종료 및 대기 제어
        self.stop_event = threading.Event()

        # 최초 Plate Solving 중의 시작 작업 취소 제어
        self._start_cancel_event = threading.Event()

        self.tracking_thread: Optional[threading.Thread] = None
        self.correction_thread: Optional[threading.Thread] = None

        # 중복 시작 및 상태 변경 방지
        self._state_lock = threading.Lock()

    # ==================================================
    # Initialize
    # ==================================================

    def initialize(
        self,
        latitude: float,
        longitude: float,
        altitude: float = 0.0,
    ) -> None:
        """호환용 전체 초기화: 하드웨어 보정 후 관측자를 설정한다."""

        self.initialize_hardware()
        self.configure_observer(latitude, longitude, altitude)

    def initialize_hardware(self):
        """모터와 IMU를 보정한 뒤에만 카메라를 초기화한다."""

        if self._hardware_initialized:
            return self.alignment_result

        self.motor.initialize()
        self.state.set_state(SystemState.ALIGN)

        try:
            if config.IMU_ENABLED:
                try:
                    self.imu.initialize()
                    leveler = self._leveling_controller_factory(
                        self.imu,
                        self.motor,
                        settings=AlignmentSettings.from_config(),
                    )
                    self.alignment_result = leveler.align()
                finally:
                    self.imu.close()
            else:
                print(
                    "[IMU] Alignment disabled; operator must establish "
                    "the physical ALT zero."
                )
                if hasattr(self.motor, "set_current_position"):
                    self.motor.set_current_position(alt=0.0, az=0.0)

        except Exception as error:
            self.state.set_state(SystemState.INIT)
            self.motor.stop()

            if config.IMU_REQUIRED:
                raise RuntimeError(
                    f"Required IMU alignment failed: {error}"
                ) from error

            print(f"[IMU] Bench override after alignment failure: {error}")
            if hasattr(self.motor, "set_current_position"):
                self.motor.set_current_position(alt=0.0, az=0.0)

        self.camera.initialize()
        self._hardware_initialized = True
        self.state.set_state(SystemState.INIT)

        if self.alignment_result is not None:
            print(
                "[IMU] Alignment complete: "
                f"error={self.alignment_result.final_error_deg:.3f} deg, "
                f"iterations={self.alignment_result.iterations}"
            )

        return self.alignment_result

    def configure_observer(
        self,
        latitude: float,
        longitude: float,
        altitude: float = 0.0,
    ) -> None:
        """Apply a validated GNSS position and enable MANUAL operation."""

        if not self._hardware_initialized:
            raise RuntimeError(
                "Hardware must be initialized before observer setup."
            )

        self.astronomy.set_location(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
        )

        self.observer = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        }

        self._observer_configured = True
        self.state.set_state(SystemState.PREVIEW)

        print("[Tracking] Ready")

    # ==================================================
    # Preview Manual Move
    # ==================================================

    def preview_move(
        self,
        axis: str,
        direction: bool,
        steps: int,
        pulse_delay: float = None,
    ) -> None:
        """PREVIEW 상태에서 조이스틱 수동 이동을 처리한다."""

        if not self.state.is_state(SystemState.PREVIEW):
            return

        self.motor.move_manual(
            axis=axis,
            direction=direction,
            steps=steps,
            pulse_delay=pulse_delay,
        )

    # ==================================================
    # Capture + Plate Solve
    # ==================================================

    def capture_and_solve(self) -> Optional[dict]:
        """Plate Solving용 사진을 촬영하고 결과를 반환한다."""

        try:
            image_path = self.camera.capture_plate()
            result = self.platesolver.solve(image_path)

            if result is None:
                print("[Tracking] Plate Solving returned no result.")
                return None

            return result

        except Exception as error:
            print(f"[Tracking] Plate Solving Failed: {error}")
            return None

    # ==================================================
    # Start Tracking
    # ==================================================

    def start_tracking(self) -> bool:
        """
        현재 화면 중심을 Plate Solving하여 목표 RA/Dec로 저장한 뒤
        Tracking Thread와 Correction Thread를 시작한다.
        """

        with self._state_lock:

            if self.is_tracking:
                print("[Tracking] Already running.")
                return False

            if not self.state.is_state(SystemState.PREVIEW):
                print("[Tracking] Start rejected: system is not in PREVIEW.")
                return False

            # Plate Solving 중 수동 이동 차단
            self._start_cancel_event.clear()
            self.state.set_state(SystemState.TARGET_CAPTURE)

        print("[Tracking] Capturing target field...")

        result = self.capture_and_solve()

        if self._start_cancel_event.is_set():
            print("[Tracking] Start cancelled.")
            return False

        if result is None:
            self.state.set_state(SystemState.PREVIEW)
            return False

        self.target_ra = float(result["ra"])
        self.target_dec = float(result["dec"])

        # 최초 Plate Solving 시각을 기준으로 실제 현재 Alt/Az 계산
        observation_time = self._get_observation_time()

        initial_alt, initial_az = self.astronomy.radec_to_altaz(
            ra=self.target_ra,
            dec=self.target_dec,
            observation_time=observation_time,
        )

        try:
            self._validate_target_altitude(initial_alt)
        except RuntimeError as error:
            print(f"[Tracking] Start rejected: {error}")
            self.state.set_state(SystemState.PREVIEW)
            return False

        # 현재 카메라 방향과 소프트웨어 모터 위치 동기화
        self.motor.set_current_position(
            alt=initial_alt,
            az=initial_az,
        )

        self.motor.set_target(
            alt=initial_alt,
            az=initial_az,
        )

        print(f"[Tracking] Target RA  : {self.target_ra:.6f} deg")
        print(f"[Tracking] Target DEC : {self.target_dec:.6f} deg")
        print(f"[Tracking] Initial ALT: {initial_alt:.6f} deg")
        print(f"[Tracking] Initial AZ : {initial_az:.6f} deg")

        # Plate Solving 결과 처리 중 연결 해제나 종료 요청이 들어올 수
        # 있으므로 실제 Tracking 전환은 상태 Lock 안에서 다시 확인한다.
        with self._state_lock:

            if (
                self._start_cancel_event.is_set()
                or not self.state.is_state(
                    SystemState.TARGET_CAPTURE
                )
            ):
                print("[Tracking] Start cancelled.")
                return False

            self.correction_count = 0
            self.consecutive_correction_failures = 0
            self.stop_event.clear()
            self.is_tracking = True

            self.state.set_state(SystemState.TRACKING)

        started_at_utc = self._time_to_iso(
            observation_time
        )

        self._safe_log(
            self.logger.start_session,
            observer=self.observer.copy(),
            target={
                "ra": self.target_ra,
                "dec": self.target_dec,
            },
            started_at_utc=started_at_utc,
        )

        if self.alignment_result is not None:
            self._safe_log(
                self.logger.record_alignment,
                asdict(self.alignment_result),
            )

        self._safe_log(
            self.logger.record_plate_solution,
            stage="target_capture",
            result=result,
            timestamp_utc=started_at_utc,
        )

        self.tracking_thread = threading.Thread(
            target=self.tracking_loop,
            name="TrackingLoop",
            daemon=True,
        )

        self.correction_thread = threading.Thread(
            target=self.correction_loop,
            name="CorrectionLoop",
            daemon=True,
        )

        self.tracking_thread.start()
        self.correction_thread.start()

        print("[Tracking] Started")

        return True

    # ==================================================
    # Tracking Loop
    # ==================================================

    def tracking_loop(self) -> None:
        """
        고정된 목표 RA/Dec를 현재 시각의 Alt/Az로 계속 변환하고
        MotorController에 전달한다.
        """

        while not self.stop_event.is_set():

            try:
                if self.target_ra is None or self.target_dec is None:
                    raise RuntimeError("Tracking target is not configured.")

                observation_time = self._get_observation_time()

                target_alt, target_az = self.astronomy.radec_to_altaz(
                    ra=self.target_ra,
                    dec=self.target_dec,
                    observation_time=observation_time,
                )

                self._validate_target_altitude(target_alt)

                self.motor.set_target(
                    alt=target_alt,
                    az=target_az,
                )

                self.motor.update()

            except Exception as error:
                print(f"[Tracking] Tracking Loop Error: {error}")
                self.stop_event.set()
                self.is_tracking = False
                self.motor.stop()
                self.state.set_state(SystemState.PREVIEW)

                self._safe_log(
                    self.logger.record_event,
                    name="tracking_error",
                    timestamp_utc=self._current_time_iso(),
                    details={"error": str(error)},
                )
                self._safe_log(
                    self.logger.finish_session,
                    status="tracking_error",
                    ended_at_utc=self._current_time_iso(),
                )
                break

            self.stop_event.wait(config.TRACKING_INTERVAL)

        print("[Tracking] Tracking Loop Ended")

    # ==================================================
    # Correction Loop
    # ==================================================

    def correction_loop(self) -> None:
        """설정된 주기마다 Plate Solving으로 실제 방향을 측정한다."""

        next_correction_time = (
            time.monotonic()
            + config.CORRECTION_INTERVAL
        )

        while not self.stop_event.is_set():

            remaining = max(
                0.0,
                next_correction_time - time.monotonic(),
            )

            # Tracking 시작 시각을 기준으로 120초 주기를 유지한다.
            stopped = self.stop_event.wait(
                remaining
            )

            if stopped:
                break

            self.state.set_state(
                SystemState.DRIFT_CORRECTION
            )

            result = self.capture_and_solve()

            # Plate Solving 도중 Stop 버튼이 눌렸다면
            # 보정하지 않고 즉시 종료
            if self.stop_event.is_set():
                break

            if result is None:
                print("[Tracking] Correction skipped.")

                self._safe_log(
                    self.logger.record_event,
                    name="correction_skipped",
                    timestamp_utc=self._current_time_iso(),
                )

                if self._record_correction_failure():
                    break

            else:

                observation_time = self._get_observation_time()
                observation_time_utc = self._time_to_iso(
                    observation_time
                )
                correction_number = self.correction_count + 1

                self._safe_log(
                    self.logger.record_plate_solution,
                    stage="drift_correction",
                    result=result,
                    timestamp_utc=observation_time_utc,
                    correction_number=correction_number,
                )

                try:
                    correction = self.drift_correction(
                        result,
                        observation_time=observation_time,
                    )

                except Exception as error:
                    print(
                        "[Tracking] Drift Correction Error: "
                        f"{error}"
                    )

                    self._safe_log(
                        self.logger.record_event,
                        name="correction_error",
                        timestamp_utc=observation_time_utc,
                        details={"error": str(error)},
                    )

                    if self._record_correction_failure():
                        break

                else:
                    self.consecutive_correction_failures = 0
                    self.correction_count = correction_number
                    correction["correction_number"] = (
                        self.correction_count
                    )

                    self._safe_log(
                        self.logger.record_correction,
                        correction,
                    )

                    print(
                        "[Tracking] Correction Count: "
                        f"{self.correction_count}/"
                        f"{config.MAX_CORRECTION_COUNT}"
                    )

                    if (
                        self.correction_count
                        >= config.MAX_CORRECTION_COUNT
                    ):
                        self.finish_sequence()
                        break

            if self.stop_event.is_set():
                break

            self.state.set_state(SystemState.TRACKING)

            # Solving 시간이 길어도 밀린 보정을 연속 실행하지 않고
            # 다음 미래 주기로 이동한다.
            next_correction_time += config.CORRECTION_INTERVAL
            now = time.monotonic()

            while next_correction_time <= now:
                next_correction_time += config.CORRECTION_INTERVAL

        print("[Tracking] Correction Loop Ended")

    def _record_correction_failure(self) -> bool:
        """Stop a session that cannot obtain reliable correction feedback."""

        self.consecutive_correction_failures += 1

        print(
            "[Tracking] Consecutive correction failures: "
            f"{self.consecutive_correction_failures}/"
            f"{config.MAX_CONSECUTIVE_CORRECTION_FAILURES}"
        )

        if (
            self.consecutive_correction_failures
            < config.MAX_CONSECUTIVE_CORRECTION_FAILURES
        ):
            return False

        self.stop_tracking(reason="correction_failures")
        return True

    # ==================================================
    # Drift Correction
    # ==================================================

    def drift_correction(
        self,
        result: dict,
        observation_time=None,
    ) -> dict:
        """
        Plate Solving으로 측정한 실제 카메라 방향을 MotorController에
        동기화한다.

        실제 보정 이동은 Tracking Loop의 motor.update()가 담당한다.
        """

        current_ra = float(result["ra"])
        current_dec = float(result["dec"])

        # 실제 위치와 목표 위치를 정확히 같은 시각으로 변환
        if observation_time is None:
            observation_time = self._get_observation_time()

        current_alt, current_az = self.astronomy.radec_to_altaz(
            ra=current_ra,
            dec=current_dec,
            observation_time=observation_time,
        )

        target_alt, target_az = self.astronomy.radec_to_altaz(
            ra=self.target_ra,
            dec=self.target_dec,
            observation_time=observation_time,
        )

        alt_error = target_alt - current_alt
        az_error = self._shortest_azimuth_error(
            target_az=target_az,
            current_az=current_az,
        )

        print(f"[Drift] Current ALT : {current_alt:.6f} deg")
        print(f"[Drift] Target ALT  : {target_alt:.6f} deg")
        print(f"[Drift] ALT Error   : {alt_error:.6f} deg")
        print(f"[Drift] Current AZ  : {current_az:.6f} deg")
        print(f"[Drift] Target AZ   : {target_az:.6f} deg")
        print(f"[Drift] AZ Error    : {az_error:.6f} deg")

        # Plate Solving 결과를 실제 현재 위치로 동기화
        self.motor.set_current_position(
            alt=current_alt,
            az=current_az,
        )

        # 목표 위치를 다시 전달
        self.motor.set_target(
            alt=target_alt,
            az=target_az,
        )

        # motor.update()는 Tracking Loop에서 계속 실행된다.
        print("[Tracking] Drift Position Synchronized")

        return {
            "timestamp_utc": self._time_to_iso(
                observation_time
            ),
            "current_alt": current_alt,
            "current_az": current_az,
            "target_alt": target_alt,
            "target_az": target_az,
            "altitude_error": alt_error,
            "azimuth_error": az_error,
        }

    # ==================================================
    # User Stop
    # ==================================================

    def stop_tracking(
        self,
        reason: str = "user_stopped",
    ) -> None:
        """사용자가 버튼을 눌러 추적을 중지하고 PREVIEW로 복귀한다."""

        stopped_active_tracking = False

        with self._state_lock:

            if self.state.is_state(SystemState.TARGET_CAPTURE):
                self._start_cancel_event.set()
                self.state.set_state(SystemState.PREVIEW)
                print("[Tracking] Target capture cancelled.")
                return

            if not self.is_tracking:
                return

            self.is_tracking = False
            self.stop_event.set()
            self.motor.stop()

            self.state.set_state(SystemState.PREVIEW)
            stopped_active_tracking = True

        if stopped_active_tracking:
            self._safe_log(
                self.logger.finish_session,
                status=reason,
                ended_at_utc=self._current_time_iso(),
            )

            print(f"[Tracking] Stopped: {reason}")

    # ==================================================
    # Automatic Finish Sequence
    # ==================================================

    def finish_sequence(self) -> None:
        """
        보정 횟수가 완료되면 Tracking을 유지하면서 최종 장노출을
        촬영하고, 촬영 후 모터를 정지한다.
        """

        if not self.is_tracking:
            return

        self.state.set_state(SystemState.CAPTURE)

        print("[Tracking] Correction cycle completed.")
        print("[Camera] Final capture started.")

        self._safe_log(
            self.logger.record_event,
            name="final_capture_started",
            timestamp_utc=self._current_time_iso(),
        )

        final_image = None
        final_images = []
        final_status = "completed"

        try:
            # 최종 촬영 중에도 Tracking Loop는 계속 실행된다.
            capture_mode = config.FINAL_CAPTURE_MODE.lower()

            if capture_mode == "sequence":
                final_images = self.camera.capture_sequence(
                    directory=config.FINAL_SEQUENCE_DIRECTORY,
                    frame_count=config.FINAL_FRAME_COUNT,
                    exposure_time=config.FINAL_SUBEXPOSURE_SECONDS,
                    cancel_event=self.stop_event,
                )

                if self.stop_event.is_set():
                    final_status = "capture_cancelled"

                print(
                    "[Camera] Final subframes saved: "
                    f"{len(final_images)}"
                )

            elif capture_mode == "single":
                final_image = self.camera.capture_long_exposure(
                    filename=config.FINAL_IMAGE_PATH,
                    exposure_time=config.LONG_EXPOSURE_SECONDS,
                )
                print(f"[Camera] Final image saved: {final_image}")

            else:
                raise ValueError(
                    "FINAL_CAPTURE_MODE must be 'sequence' or 'single'."
                )

        except Exception as error:
            print(f"[Camera] Final Capture Failed: {error}")
            final_status = "final_capture_failed"

            self._safe_log(
                self.logger.record_event,
                name="final_capture_error",
                timestamp_utc=self._current_time_iso(),
                details={"error": str(error)},
            )

        finally:
            self.is_tracking = False
            self.stop_event.set()
            self.motor.stop()

            self._safe_log(
                self.logger.finish_session,
                status=final_status,
                ended_at_utc=self._current_time_iso(),
                final_image=final_image,
                final_images=final_images,
            )

            # 촬영과 로그 저장이 끝나면 시스템을 종료하지 않고
            # 조이스틱 수동 제어가 가능한 MANUAL 상태로 복귀한다.
            self.state.set_state(SystemState.PREVIEW)

            print(
                "[Tracking] Automatic Sequence Finished; "
                "Manual Mode Ready"
            )

    # ==================================================
    # Shutdown
    # ==================================================

    def shutdown(self) -> None:
        """전체 시스템을 안전하게 종료한다."""

        was_tracking = self.is_tracking
        self.is_tracking = False
        self._start_cancel_event.set()
        self.stop_event.set()
        self.motor.stop()

        if was_tracking:
            self._safe_log(
                self.logger.finish_session,
                status="shutdown",
                ended_at_utc=self._current_time_iso(),
            )

        # 현재 스레드가 아닌 경우에만 짧게 종료를 기다린다.
        current_thread = threading.current_thread()

        if (
            self.tracking_thread is not None
            and self.tracking_thread.is_alive()
            and self.tracking_thread is not current_thread
        ):
            self.tracking_thread.join(timeout=2.0)

        if (
            self.correction_thread is not None
            and self.correction_thread.is_alive()
            and self.correction_thread is not current_thread
        ):
            self.correction_thread.join(timeout=2.0)

        try:
            self.camera.stop()
        except Exception as error:
            print(f"[Camera] Stop Error: {error}")

        try:
            self.imu.close()
        except Exception as error:
            print(f"[IMU] Close Error: {error}")

        self.motor.cleanup()
        self._hardware_initialized = False
        self._observer_configured = False

        print("[Tracking] Shutdown Complete")

    # ==================================================
    # Utility
    # ==================================================

    @staticmethod
    def _shortest_azimuth_error(
        target_az: float,
        current_az: float,
    ) -> float:
        """0/360도 경계를 고려한 AZ 최단 오차를 반환한다."""

        return (
            target_az
            - current_az
            + 180.0
        ) % 360.0 - 180.0

    @staticmethod
    def _validate_target_altitude(altitude: float) -> None:
        """Reject unsafe target elevations instead of silently clamping."""

        if not config.MIN_ALTITUDE <= altitude <= config.MAX_ALTITUDE:
            raise RuntimeError(
                "Target altitude is outside the configured safe range: "
                f"{altitude:.3f} deg."
            )
    
    # ==================================================
    # Time Provider
    # ==================================================

    def set_time_provider(
        self,
        provider: Callable,
    ) -> None:
        """
        GNSS UTC를 반환하는 함수를 등록한다.

        사용 예:
            tracking.set_time_provider(sensor.get_utc)
        """

        self._time_provider = provider


    def _get_observation_time(self):
        """
        GNSS UTC가 있으면 사용하고,
        없으면 시스템 시간을 사용한다.
        """

        if self._time_provider is not None:

            try:

                utc_datetime = self._time_provider()

                if utc_datetime is not None:

                    return self.astronomy.get_current_time(
                        utc_datetime
                    )

            except Exception as error:

                print(
                    "[Tracking] GNSS Time Warning: "
                    f"{error}"
                )

        return self.astronomy.get_current_time()

    @staticmethod
    def _time_to_iso(observation_time) -> str:
        """Astropy Time을 UTC ISO 8601 문자열로 변환한다."""

        try:
            return f"{observation_time.utc.isot}Z"
        except Exception:
            return datetime.now(timezone.utc).isoformat()

    def _current_time_iso(self) -> str:
        return self._time_to_iso(
            self._get_observation_time()
        )

    @staticmethod
    def _safe_log(operation, *args, **kwargs):
        """로그 저장 실패가 모터 추적을 중단시키지 않도록 격리한다."""

        try:
            return operation(*args, **kwargs)
        except Exception as error:
            print(f"[TrackingLog] Warning: {error}")
            return None
