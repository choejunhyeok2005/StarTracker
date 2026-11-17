"""Hardware-free checks for the original broken click path and UTC gating."""
import importlib
import sys
import threading
import unittest
from datetime import datetime, timezone
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from managers.state_manager import StateManager, SystemState


class ButtonTests(unittest.TestCase):
    def test_manual_click_requests_tracking_without_ending_ble(self):
        bleak = ModuleType("bleak")
        bleak.BleakClient = bleak.BleakScanner = object
        with patch.dict(sys.modules, {"bleak": bleak}):
            module = importlib.import_module("communication.ble_manager")
        tracking = SimpleNamespace(state=StateManager(), stop_tracking=Mock())
        tracking.state.set_state(SystemState.MANUAL)
        ble = module.BLEManager(tracking)
        ble.on_start = Mock()
        ble._handle_click()
        ble.on_start.assert_called_once()
        self.assertFalse(ble._stop_event.is_set())
        tracking.state.set_state(SystemState.TRACKING)
        ble._handle_click()
        tracking.stop_tracking.assert_called_once()
        self.assertFalse(ble._stop_event.is_set())


class GNSSTests(unittest.TestCase):
    def make_sensor(self):
        serial = ModuleType("serial")
        serial.Serial = object
        serial.SerialException = OSError
        with patch.dict(sys.modules, {"serial": serial, "pynmea2": ModuleType("pynmea2")}):
            module = importlib.import_module("managers.sensor_manager")
        return module.SensorManager()

    def test_position_without_utc_is_not_ready(self):
        sensor = self.make_sensor()
        sensor.latitude, sensor.longitude, sensor.altitude = 37.5, 127., 10.
        sensor._fix_event.set()
        self.assertFalse(sensor.wait_for_fix(timeout=0))

    def test_position_wait_does_not_require_utc(self):
        sensor = self.make_sensor()
        sensor.latitude, sensor.longitude, sensor.altitude = 37.5, 127., 10.
        sensor._fix_event.set()

        self.assertTrue(sensor.wait_for_position(timeout=0))
        self.assertIsNone(sensor.get_utc())

    def test_valid_position_and_utc_are_ready(self):
        sensor = self.make_sensor()
        import time
        sensor.latitude, sensor.longitude, sensor.altitude = 37.5, 127., 10.
        sensor.utc = datetime.now(timezone.utc)
        sensor._utc_reference_monotonic = time.monotonic()
        sensor._fix_event.set()
        sensor._utc_event.set()
        self.assertTrue(sensor.wait_for_fix(timeout=0))

    def test_no_fix_quality_does_not_pass_satellite_threshold(self):
        sensor = self.make_sensor()
        sensor._process_nmea(SimpleNamespace(sentence_type="GGA", gps_qual="0",
                            num_sats="8", latitude=37.5, longitude=127., altitude=10.))
        self.assertFalse(sensor.has_fix())


if __name__ == "__main__":
    unittest.main()
