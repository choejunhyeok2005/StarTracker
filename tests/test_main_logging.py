import importlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

import config


class MainLoggingTests(unittest.TestCase):
    def load_main(self):
        fake_bleak = types.ModuleType("bleak")
        fake_bleak.BleakClient = object
        fake_bleak.BleakScanner = object
        fake_rpi = types.ModuleType("RPi")
        fake_gpio = types.ModuleType("RPi.GPIO")
        fake_rpi.GPIO = fake_gpio
        fake_smbus = types.ModuleType("smbus")
        fake_pynmea2 = types.ModuleType("pynmea2")
        fake_serial = types.ModuleType("serial")
        fake_serial.Serial = object
        fake_serial.SerialException = OSError
        fake_flask = types.ModuleType("flask")
        fake_flask.Flask = object
        fake_flask.Response = object

        with patch.dict(
            sys.modules,
            {
                "bleak": fake_bleak,
                "RPi": fake_rpi,
                "RPi.GPIO": fake_gpio,
                "smbus": fake_smbus,
                "pynmea2": fake_pynmea2,
                "serial": fake_serial,
                "flask": fake_flask,
            },
        ):
            return importlib.import_module("main")

    def test_main_opens_and_closes_configured_console_log(self):
        """Catch defining the logger without enabling it for real runs."""

        session = Mock()
        main = self.load_main()

        with (
            patch.object(config, "CONSOLE_LOG_ENABLED", True, create=True),
            patch.object(
                config,
                "CONSOLE_LOG_DIRECTORY",
                "test-console-logs",
                create=True,
            ),
            patch.object(
                main,
                "start_console_file_logging",
                return_value=session,
                create=True,
            ) as start_logging,
            patch.object(
                main.asyncio,
                "run",
                side_effect=lambda coroutine: coroutine.close(),
            ),
        ):
            main.main()

        start_logging.assert_called_once_with("test-console-logs")
        session.close.assert_called_once_with()

    def test_gnss_startup_selects_all_three_fallback_modes(self):
        main = self.load_main()

        class FakeSensor:
            def __init__(self, location, utc):
                self.location = location
                self.utc = utc
                self.position_wait_calls = 0
                self.utc_wait_calls = 0

            def wait_for_position(self, timeout):
                self.position_wait_calls += 1
                return self.location is not None

            def wait_for_utc(self, timeout):
                self.utc_wait_calls += 1
                return self.utc is not None

            def has_fix(self):
                return self.location is not None

            def get_location(self):
                return self.location

            def get_utc(self):
                return self.utc

        location = (12.0, 34.0, 56.0)
        utc = object()

        sensor = FakeSensor(location, utc)
        selected, provider, status = main._resolve_gnss_startup(sensor, True)
        self.assertEqual(selected, location)
        self.assertIs(provider.__self__, sensor)
        self.assertEqual(status, "[GNSS] Position and UTC available from GNSS")
        self.assertEqual(sensor.position_wait_calls, 1)
        self.assertEqual(sensor.utc_wait_calls, 1)

        sensor = FakeSensor(location, None)
        selected, provider, status = main._resolve_gnss_startup(sensor, True)
        self.assertEqual(selected, location)
        self.assertIsNone(provider)
        self.assertEqual(status, "[GNSS] Position fix available; using system UTC")
        self.assertEqual(sensor.utc_wait_calls, 1)

        sensor = FakeSensor(None, None)
        selected, provider, status = main._resolve_gnss_startup(sensor, False)
        self.assertEqual(
            selected,
            (
                config.GNSS_FALLBACK_LATITUDE,
                config.GNSS_FALLBACK_LONGITUDE,
                config.GNSS_FALLBACK_ALTITUDE,
            ),
        )
        self.assertIsNone(provider)
        self.assertEqual(status, "[GNSS] No position fix: fallback location + system UTC")
        self.assertEqual(sensor.position_wait_calls, 0)
        self.assertEqual(sensor.utc_wait_calls, 0)


if __name__ == "__main__":
    unittest.main()
