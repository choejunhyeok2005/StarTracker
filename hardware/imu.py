"""MPU6050 쌍 접근 및 피치(Pitch) 측정 유틸리티"""

from __future__ import annotations

import math
import statistics
import time
from collections.abc import Callable
from types import TracebackType
from typing import Any


class IMUError(RuntimeError):
    """IMU 초기화 및 측정 실패 시 발생하는 기본 예외"""


class IMUInitializationError(IMUError):
    """MPU6050 초기화 또는 식별 실패 시 발생"""


class IMUReadError(IMUError):
    """가속도계 샘플에서 유효한 피치를 생성할 수 없을 때 발생"""


def signed_int16(high: int, low: int) -> int:
    """Big-endian 2의 보수 16비트 정수 디코딩"""
    value = ((int(high) & 0xFF) << 8) | (int(low) & 0xFF)
    return value - 0x10000 if value & 0x8000 else value


def acceleration_to_pitch(
    x: int, y: int, z: int, *, permissive: bool = True
) -> float:
    """설정된 센서 축 방향에 대한 피치(도 단위) 반환"""
    values = (float(x), float(y), float(z))

    if not all(math.isfinite(value) for value in values):
        if permissive:
            return 0.0
        raise IMUReadError("가속도계 샘플에 유효하지 않은 값이 포함되어 있습니다.")

    if values == (0.0, 0.0, 0.0):
        if permissive:
            return 0.0
        raise IMUReadError("가속도계 샘플이 올 zero 벡터입니다.")

    pitch = math.degrees(
        math.atan2(-values[0], math.hypot(values[1], values[2]))
    )

    if not math.isfinite(pitch):
        if permissive:
            return 0.0
        raise IMUReadError("계산된 피치 값이 유효하지 않습니다.")

    return pitch


class MPU6050Pair:
    """고정 베이스 MPU6050 및 상부 MPU6050 읽기 클래스"""

    WHO_AM_I = 0x75
    POWER_MANAGEMENT_1 = 0x6B
    ACCELEROMETER_CONFIG = 0x1C
    DIGITAL_LOW_PASS_CONFIG = 0x1A
    ACCELEROMETER_START = 0x3B

    def __init__(
        self,
        *,
        bus_number: int = 1,
        base_address: int = 0x68,
        upper_address: int = 0x69,
        base_offset: float = 0.0,
        upper_offset: float = 0.0,
        max_delta: float = 5.0,  # 두 센서 간 허용 오차 범위(도 단위)
        permissive: bool = True,
        max_retries: int = 3,
        bus_factory: Callable[[int], Any] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if base_address == upper_address:
            raise ValueError("베이스 및 상부 IMU 주소는 서로 달라야 합니다.")

        self.bus_number = int(bus_number)
        self.base_address = int(base_address)
        self.upper_address = int(upper_address)
        self.base_offset = float(base_offset)
        self.upper_offset = float(upper_offset)
        self.max_delta = float(max_delta)
        self.permissive = permissive
        self.max_retries = max(1, max_retries)
        self._bus_factory = bus_factory
        self._sleep = sleep
        self._bus: Any | None = None
        self.initialized = False

    def __enter__(self) -> MPU6050Pair:
        self.initialize()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()

    def initialize(self) -> None:
        """I2C 버스를 열고 두 장치를 검증 및 설정합니다."""
        if self.initialized:
            return

        if self._bus_factory is None:
            try:
                from smbus2 import SMBus
            except ImportError as error:
                raise IMUInitializationError(
                    "MPU6050 접근을 위해 smbus2 패키지가 필요합니다."
                ) from error

            bus_factory = SMBus
        else:
            bus_factory = self._bus_factory

        try:
            self._bus = bus_factory(self.bus_number)

            for address in (self.base_address, self.upper_address):
                identity = int(
                    self._bus.read_byte_data(address, self.WHO_AM_I)
                )

                if not self.permissive and (identity & 0x7E not in (0x68, 0x70)):
                    raise IMUInitializationError(
                        f"I2C 주소 0x{address:02X}에서 예상치 못한 WHO_AM_I 값(0x{identity:02X})을 수신했습니다."
                    )

                self._bus.write_byte_data(address, self.POWER_MANAGEMENT_1, 0x00)
                self._bus.write_byte_data(address, self.ACCELEROMETER_CONFIG, 0x00)
                self._bus.write_byte_data(address, self.DIGITAL_LOW_PASS_CONFIG, 0x03)

            self._sleep(0.05)
            self.initialized = True

        except IMUInitializationError:
            self.close()
            raise
        except Exception as error:
            self.close()
            if self.permissive:
                self.initialized = True
            else:
                raise IMUInitializationError(
                    f"MPU6050 쌍 초기화 실패: {error}"
                ) from error

    def read_pitch(self, address: int) -> float:
        """I2C 재시도 로직을 포함하여 피치 각도를 읽어옵니다."""
        if not self.initialized or self._bus is None:
            if not self.permissive:
                raise IMUReadError("MPU6050Pair가 초기화되지 않았습니다.")

        if address not in (self.base_address, self.upper_address):
            raise ValueError(f"알 수 없는 IMU 주소: 0x{address:02X}")

        data = None
        last_error = None

        for attempt in range(self.max_retries):
            try:
                if self._bus is not None:
                    data = self._bus.read_i2c_block_data(
                        address, self.ACCELEROMETER_START, 6
                    )
                    if len(data) == 6:
                        break
            except Exception as error:
                last_error = error
                self._sleep(0.005)

        if data is None or len(data) != 6:
            if self.permissive:
                return 0.0
            raise IMUReadError(
                f"0x{address:02X} 주소 읽기 실패: {last_error}"
            )

        x = signed_int16(data[0], data[1])
        y = signed_int16(data[2], data[3])
        z = signed_int16(data[4], data[5])

        raw_pitch = acceleration_to_pitch(x, y, z, permissive=self.permissive)
        offset = self.base_offset if address == self.base_address else self.upper_offset
        return raw_pitch - offset

    def verify_pitch_difference(
        self, base_pitch: float, upper_pitch: float, max_delta: float | None = None
    ) -> bool:
        """두 센서 간의 피치 차이가 허용 범위(max_delta) 이내인지 검증합니다."""
        limit = self.max_delta if max_delta is None else max_delta
        return abs(upper_pitch - base_pitch) <= limit

    def read_filtered_pitches(
        self,
        *,
        sample_count: int = 10,
        sample_interval: float = 0.01,
        noise_threshold: float = 3.0,
    ) -> tuple[float, float]:
        """
        독립적으로 필터링된 (베이스 피치, 상부 피치) 2개 값만 반환합니다.
        (메인 코드 언패킹 에러를 방지하도록 표준화됨)
        """
        if sample_count <= 0:
            raise ValueError("sample_count는 양수여야 합니다.")
        if sample_interval < 0:
            raise ValueError("sample_interval은 음수일 수 없습니다.")

        base_samples = []
        upper_samples = []

        for index in range(sample_count):
            base_samples.append(self.read_pitch(self.base_address))
            upper_samples.append(self.read_pitch(self.upper_address))

            if index + 1 < sample_count and sample_interval:
                self._sleep(sample_interval)

        base_med = statistics.median(base_samples)
        upper_med = statistics.median(upper_samples)

        clean_base = [x for x in base_samples if abs(x - base_med) <= noise_threshold]
        clean_upper = [x for x in upper_samples if abs(x - upper_med) <= noise_threshold]

        final_base = float(statistics.median(clean_base or base_samples))
        final_upper = float(statistics.median(clean_upper or upper_samples))

        return final_base, final_upper

    def close(self) -> None:
        """I2C 버스를 안전하게 닫습니다."""
        bus = self._bus
        self._bus = None
        self.initialized = False

        if bus is None:
            return

        try:
            bus.close()
        except Exception:
            pass