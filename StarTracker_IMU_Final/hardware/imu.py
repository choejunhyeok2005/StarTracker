"""MPU6050 pair access and pitch measurement utilities."""

from __future__ import annotations

import math
import statistics
import time
from collections.abc import Callable
from typing import Any


class IMUError(RuntimeError):
    """Base exception for IMU initialization and measurement failures."""


class IMUInitializationError(IMUError):
    """Raised when an MPU6050 cannot be initialized or identified."""


class IMUReadError(IMUError):
    """Raised when an accelerometer sample cannot produce a valid pitch."""


def signed_int16(high: int, low: int) -> int:
    """Decode a big-endian two's-complement 16-bit integer."""

    value = ((int(high) & 0xFF) << 8) | (int(low) & 0xFF)
    return value - 0x10000 if value & 0x8000 else value


def acceleration_to_pitch(x: int, y: int, z: int) -> float:
    """Return pitch in degrees for the configured sensor axis orientation."""

    values = (float(x), float(y), float(z))

    if not all(math.isfinite(value) for value in values):
        raise IMUReadError("Accelerometer sample contains a non-finite value.")

    if values == (0.0, 0.0, 0.0):
        raise IMUReadError("Accelerometer sample is an all-zero vector.")

    pitch = math.degrees(
        math.atan2(-values[0], math.hypot(values[1], values[2]))
    )

    if not math.isfinite(pitch):
        raise IMUReadError("Calculated pitch is not finite.")

    return pitch


class MPU6050Pair:
    """Read a fixed base MPU6050 and a moving upper MPU6050."""

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
        bus_factory: Callable[[int], Any] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if base_address == upper_address:
            raise ValueError("Base and upper IMU addresses must be different.")

        self.bus_number = int(bus_number)
        self.base_address = int(base_address)
        self.upper_address = int(upper_address)
        self._bus_factory = bus_factory
        self._sleep = sleep
        self._bus = None
        self.initialized = False

    def initialize(self) -> None:
        """Open I2C, validate both devices, and configure accelerometers."""

        if self.initialized:
            return

        if self._bus_factory is None:
            try:
                from smbus2 import SMBus
            except ImportError as error:
                raise IMUInitializationError(
                    "smbus2 is required for MPU6050 access."
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

                # MPU6050 reports 0x68. Some compatible parts expose AD0
                # in bit 0, so mask that bit while validating the device ID.
                if identity & 0x7E != 0x68:
                    raise IMUInitializationError(
                        f"Unexpected WHO_AM_I 0x{identity:02X} "
                        f"at I2C address 0x{address:02X}."
                    )

                self._bus.write_byte_data(
                    address,
                    self.POWER_MANAGEMENT_1,
                    0x00,
                )
                self._bus.write_byte_data(
                    address,
                    self.ACCELEROMETER_CONFIG,
                    0x00,
                )
                self._bus.write_byte_data(
                    address,
                    self.DIGITAL_LOW_PASS_CONFIG,
                    0x03,
                )

            self._sleep(0.05)
            self.initialized = True

        except IMUInitializationError:
            self.close()
            raise
        except Exception as error:
            self.close()
            raise IMUInitializationError(
                f"Failed to initialize MPU6050 pair: {error}"
            ) from error

    def read_pitch(self, address: int) -> float:
        """Read one coherent acceleration block and convert it to pitch."""

        if not self.initialized or self._bus is None:
            raise IMUReadError("MPU6050Pair is not initialized.")

        if address not in (self.base_address, self.upper_address):
            raise ValueError(f"Unknown IMU address: 0x{address:02X}")

        try:
            data = self._bus.read_i2c_block_data(
                address,
                self.ACCELEROMETER_START,
                6,
            )
        except Exception as error:
            raise IMUReadError(
                f"Failed to read IMU at 0x{address:02X}: {error}"
            ) from error

        if len(data) != 6:
            raise IMUReadError(
                f"Expected 6 acceleration bytes from 0x{address:02X}; "
                f"received {len(data)}."
            )

        x = signed_int16(data[0], data[1])
        y = signed_int16(data[2], data[3])
        z = signed_int16(data[4], data[5])
        return acceleration_to_pitch(x, y, z)

    def read_filtered_pitches(
        self,
        *,
        sample_count: int,
        sample_interval: float,
    ) -> tuple[float, float]:
        """Return independently median-filtered base and upper pitch."""

        if sample_count <= 0:
            raise ValueError("sample_count must be positive.")
        if sample_interval < 0:
            raise ValueError("sample_interval cannot be negative.")

        base_samples = []
        upper_samples = []

        for index in range(sample_count):
            base_samples.append(self.read_pitch(self.base_address))
            upper_samples.append(self.read_pitch(self.upper_address))

            if index + 1 < sample_count and sample_interval:
                self._sleep(sample_interval)

        return (
            float(statistics.median(base_samples)),
            float(statistics.median(upper_samples)),
        )

    def close(self) -> None:
        """Close the I2C bus safely, including partial initialization."""

        bus = self._bus
        self._bus = None
        self.initialized = False

        if bus is None:
            return

        try:
            bus.close()
        except Exception:
            pass
