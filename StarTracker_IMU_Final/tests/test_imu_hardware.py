import math
import unittest

from hardware.imu import (
    IMUReadError,
    IMUInitializationError,
    MPU6050Pair,
    acceleration_to_pitch,
    signed_int16,
)


def encode_acceleration(x, y, z):
    result = []
    for value in (x, y, z):
        unsigned = value & 0xFFFF
        result.extend(((unsigned >> 8) & 0xFF, unsigned & 0xFF))
    return result


class FakeSMBus:
    def __init__(self, samples, identities=None):
        self.samples = {
            address: list(values)
            for address, values in samples.items()
        }
        self.identities = identities or {0x68: 0x68, 0x69: 0x68}
        self.writes = []
        self.reads = []
        self.closed = False

    def read_byte_data(self, address, register):
        self.reads.append(("byte", address, register))
        return self.identities[address]

    def write_byte_data(self, address, register, value):
        self.writes.append((address, register, value))

    def read_i2c_block_data(self, address, register, length):
        self.reads.append(("block", address, register, length))
        return self.samples[address].pop(0)

    def close(self):
        self.closed = True


class IMUMathTests(unittest.TestCase):
    """Catch broken signed conversion and sensor-axis pitch math."""

    def test_signed_int16_decodes_positive_and_negative_limits(self):
        self.assertEqual(signed_int16(0x7F, 0xFF), 32767)
        self.assertEqual(signed_int16(0x80, 0x00), -32768)
        self.assertEqual(signed_int16(0xFF, 0xFF), -1)

    def test_level_acceleration_has_zero_pitch(self):
        self.assertAlmostEqual(
            acceleration_to_pitch(0, 0, 16384),
            0.0,
            places=7,
        )

    def test_positive_x_tilts_pitch_negative(self):
        self.assertAlmostEqual(
            acceleration_to_pitch(16384, 0, 16384),
            -45.0,
            places=7,
        )

    def test_zero_acceleration_is_rejected(self):
        with self.assertRaises(IMUReadError):
            acceleration_to_pitch(0, 0, 0)

    def test_non_finite_acceleration_is_rejected(self):
        with self.assertRaises(IMUReadError):
            acceleration_to_pitch(math.inf, 0, 1)


class MPU6050PairTests(unittest.TestCase):
    """Catch address mixups, split reads, and broken median filtering."""

    def test_initialize_validates_and_configures_both_sensors(self):
        bus = FakeSMBus({0x68: [], 0x69: []})
        pair = MPU6050Pair(
            base_address=0x68,
            upper_address=0x69,
            bus_factory=lambda _bus_number: bus,
            sleep=lambda _seconds: None,
        )

        pair.initialize()

        self.assertIn((0x68, 0x6B, 0x00), bus.writes)
        self.assertIn((0x69, 0x6B, 0x00), bus.writes)
        self.assertIn((0x68, 0x1C, 0x00), bus.writes)
        self.assertIn((0x69, 0x1A, 0x03), bus.writes)

    def test_initialize_rejects_wrong_sensor_identity(self):
        bus = FakeSMBus(
            {0x68: [], 0x69: []},
            identities={0x68: 0x68, 0x69: 0x00},
        )
        pair = MPU6050Pair(
            base_address=0x68,
            upper_address=0x69,
            bus_factory=lambda _bus_number: bus,
            sleep=lambda _seconds: None,
        )

        with self.assertRaises(IMUInitializationError):
            pair.initialize()

    def test_filtered_pitch_returns_median_for_each_address(self):
        bus = FakeSMBus(
            {
                0x68: [
                    encode_acceleration(0, 0, 16384),
                    encode_acceleration(-16384, 0, 16384),
                    encode_acceleration(0, 0, 16384),
                ],
                0x69: [
                    encode_acceleration(16384, 0, 16384),
                    encode_acceleration(0, 0, 16384),
                    encode_acceleration(16384, 0, 16384),
                ],
            }
        )
        pair = MPU6050Pair(
            base_address=0x68,
            upper_address=0x69,
            bus_factory=lambda _bus_number: bus,
            sleep=lambda _seconds: None,
        )
        pair.initialize()

        base_pitch, upper_pitch = pair.read_filtered_pitches(
            sample_count=3,
            sample_interval=0.0,
        )

        self.assertAlmostEqual(base_pitch, 0.0, places=7)
        self.assertAlmostEqual(upper_pitch, -45.0, places=7)
        block_reads = [entry for entry in bus.reads if entry[0] == "block"]
        self.assertEqual(len(block_reads), 6)
        self.assertTrue(all(entry[2:] == (0x3B, 6) for entry in block_reads))

    def test_close_is_idempotent(self):
        bus = FakeSMBus({0x68: [], 0x69: []})
        pair = MPU6050Pair(
            base_address=0x68,
            upper_address=0x69,
            bus_factory=lambda _bus_number: bus,
            sleep=lambda _seconds: None,
        )
        pair.initialize()

        pair.close()
        pair.close()

        self.assertTrue(bus.closed)


if __name__ == "__main__":
    unittest.main()
