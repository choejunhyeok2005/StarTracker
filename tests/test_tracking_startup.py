import unittest

from controllers.imu_leveling_controller import AlignmentResult
from managers.state_manager import SystemState
from managers.tracking_manager import TrackingManager


class RecordingComponent:
    def __init__(self, events, name):
        self.events = events
        self.name = name


class FakeCamera(RecordingComponent):
    def initialize(self):
        self.events.append("camera.initialize")

    def stop(self):
        self.events.append("camera.stop")


class FakeMotor(RecordingComponent):
    def initialize(self):
        self.events.append("motor.initialize")

    def stop(self):
        self.events.append("motor.stop")

    def cleanup(self):
        self.events.append("motor.cleanup")


class FakeIMU(RecordingComponent):
    def __init__(self, events, failure=None):
        super().__init__(events, "imu")
        self.failure = failure

    def initialize(self):
        self.events.append("imu.initialize")
        if self.failure is not None:
            raise self.failure

    def close(self):
        self.events.append("imu.close")


class FakeAstronomy:
    def __init__(self, events):
        self.events = events
        self.location = None

    def set_location(self, *, latitude, longitude, altitude):
        self.location = (latitude, longitude, altitude)
        self.events.append("astronomy.set_location")


class FakeLeveler:
    def __init__(self, events, **kwargs):
        self.events = events

    def align(self):
        self.events.append("leveler.align")
        return AlignmentResult(
            final_error_deg=0.05,
            iterations=3,
            movement_count=0,
            cumulative_steps=0,
            positive_direction_increases_pitch=None,
            elapsed_seconds=0.2,
        )


def fake_leveler_factory(events):
    def factory(*args, **kwargs):
        return FakeLeveler(events, **kwargs)

    return factory


class TrackingStartupTests(unittest.TestCase):
    """Catch camera/manual startup before a required alignment succeeds."""

    def make_manager(self, events, *, imu_failure=None):
        return TrackingManager(
            camera=FakeCamera(events, "camera"),
            motor=FakeMotor(events, "motor"),
            astronomy=FakeAstronomy(events),
            platesolver=object(),
            logger=object(),
            imu=FakeIMU(events, failure=imu_failure),
            leveling_controller_factory=fake_leveler_factory(events),
        )

    def test_hardware_startup_aligns_before_camera_initialization(self):
        events = []
        manager = self.make_manager(events)

        result = manager.initialize_hardware()

        self.assertEqual(
            events,
            [
                "motor.initialize",
                "imu.initialize",
                "leveler.align",
                "imu.close",
                "camera.initialize",
            ],
        )
        self.assertAlmostEqual(result.final_error_deg, 0.05)
        self.assertEqual(manager.state.get_state(), SystemState.INIT)

    def test_observer_configuration_enters_manual_after_hardware(self):
        events = []
        manager = self.make_manager(events)
        manager.initialize_hardware()

        manager.configure_observer(37.5, 127.0, 42.0)

        self.assertEqual(
            manager.astronomy.location,
            (37.5, 127.0, 42.0),
        )
        self.assertEqual(manager.state.get_state(), SystemState.PREVIEW)

    def test_required_imu_failure_never_initializes_camera(self):
        events = []
        manager = self.make_manager(
            events,
            imu_failure=RuntimeError("I2C unavailable"),
        )

        with self.assertRaises(RuntimeError):
            manager.initialize_hardware()

        self.assertNotIn("camera.initialize", events)
        self.assertIn("imu.close", events)
        self.assertIn("motor.stop", events)
        self.assertEqual(manager.state.get_state(), SystemState.INIT)


if __name__ == "__main__":
    unittest.main()
