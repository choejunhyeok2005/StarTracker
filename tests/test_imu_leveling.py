import unittest
from dataclasses import replace

from controllers.imu_leveling_controller import (
    AlignmentSettings,
    IMUAlignmentError,
    IMULevelingController,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SequenceIMU:
    def __init__(self, readings):
        self.readings = list(readings)
        self.calls = 0

    def read_filtered_pitches(self, *, sample_count, sample_interval):
        self.calls += 1
        if len(self.readings) > 1:
            return self.readings.pop(0)
        return self.readings[0]


class FakeAlignmentMotor:
    def __init__(self):
        self.moves = []
        self.started_with_limit = None
        self.finished = False
        self.aborted = False
        self.positive_direction_is_true = None

    def begin_alignment(self, max_relative_steps):
        self.started_with_limit = max_relative_steps

    def move_alignment(self, direction, steps, pulse_delay=None):
        self.moves.append((direction, steps, pulse_delay))

    def finish_alignment(self, positive_direction_is_true=None):
        self.finished = True
        self.positive_direction_is_true = positive_direction_is_true

    def abort_alignment(self):
        self.aborted = True


class SimulatedPairIMU:
    def __init__(
        self,
        *,
        base_pitch,
        upper_pitch,
        positive_direction_increases_pitch,
        steps_per_degree=960.0,
    ):
        self.base_pitch = float(base_pitch)
        self.upper_pitch = float(upper_pitch)
        self.positive_direction_increases_pitch = bool(
            positive_direction_increases_pitch
        )
        self.steps_per_degree = float(steps_per_degree)

    def read_filtered_pitches(self, *, sample_count, sample_interval):
        return self.base_pitch, self.upper_pitch

    def apply_alt_move(self, direction, steps):
        sign = 1.0 if direction else -1.0
        if not self.positive_direction_increases_pitch:
            sign *= -1.0
        self.upper_pitch += sign * steps / self.steps_per_degree


class CoupledAlignmentMotor(FakeAlignmentMotor):
    def __init__(self, imu):
        super().__init__()
        self.imu = imu

    def move_alignment(self, direction, steps, pulse_delay=None):
        super().move_alignment(direction, steps, pulse_delay)
        self.imu.apply_alt_move(direction, steps)


class WorseningAlignmentMotor(CoupledAlignmentMotor):
    def move_alignment(self, direction, steps, pulse_delay=None):
        if not self.moves:
            super().move_alignment(direction, steps, pulse_delay)
            return

        FakeAlignmentMotor.move_alignment(
            self,
            direction,
            steps,
            pulse_delay,
        )
        self.imu.upper_pitch += 0.5


def test_settings():
    return AlignmentSettings(
        base_pitch_offset_deg=0.0,
        upper_pitch_offset_deg=0.0,
        filter_samples=11,
        sample_interval_seconds=0.0,
        tolerance_deg=0.2,
        stable_reads_required=3,
        direction_probe_steps=240,
        min_probe_response_deg=0.08,
        proportional_gain=0.65,
        min_correction_steps=8,
        max_correction_steps=960,
        motor_pulse_delay=0.001,
        settle_seconds=0.0,
        timeout_seconds=60.0,
        max_iterations=80,
        max_relative_travel_steps=14400,
        max_worsening_count=3,
        worsening_margin_deg=0.1,
        steps_per_degree=960.0,
    )


class IMULevelingStableReadTests(unittest.TestCase):
    """Catch accidental movement and early success on noisy in-range data."""

    def test_settings_from_config_derives_relative_travel_steps(self):
        settings = AlignmentSettings.from_config()

        self.assertEqual(settings.stable_reads_required, 3)
        self.assertEqual(settings.tolerance_deg, 0.2)
        self.assertEqual(
            settings.max_relative_travel_steps,
            round(15.0 * 960.0),
        )

    def test_already_aligned_requires_three_stable_reads_without_movement(self):
        imu = SequenceIMU(
            [(1.0, 1.1), (1.0, 1.05), (1.0, 1.02)]
        )
        motor = FakeAlignmentMotor()
        clock = FakeClock()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        result = controller.align()

        self.assertEqual(imu.calls, 3)
        self.assertEqual(motor.moves, [])
        self.assertTrue(motor.finished)
        self.assertFalse(motor.aborted)
        self.assertAlmostEqual(result.final_error_deg, 0.02, places=7)
        self.assertEqual(result.movement_count, 0)


class IMULevelingDirectionTests(unittest.TestCase):
    """Catch wrong probe polarity and corrections that increase pitch error."""

    def test_positive_error_converges_when_true_increases_pitch(self):
        imu = SimulatedPairIMU(
            base_pitch=0.0,
            upper_pitch=2.0,
            positive_direction_increases_pitch=True,
        )
        motor = CoupledAlignmentMotor(imu)
        clock = FakeClock()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        result = controller.align()

        self.assertLessEqual(abs(result.final_error_deg), 0.2)
        self.assertTrue(result.positive_direction_increases_pitch)
        self.assertTrue(motor.positive_direction_is_true)
        self.assertGreater(result.movement_count, 1)
        self.assertTrue(motor.finished)
        self.assertFalse(motor.aborted)


class IMULevelingFailureTests(unittest.TestCase):
    """Catch unsafe success when feedback or convergence is unavailable."""

    def test_unobservable_direction_probe_aborts(self):
        imu = SequenceIMU([(0.0, 2.0)])
        motor = FakeAlignmentMotor()
        clock = FakeClock()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaises(IMUAlignmentError) as caught:
            controller.align()

        self.assertEqual(caught.exception.reason, "probe_unobservable")
        self.assertTrue(motor.aborted)
        self.assertFalse(motor.finished)

    def test_three_worsening_corrections_abort(self):
        imu = SimulatedPairIMU(
            base_pitch=0.0,
            upper_pitch=2.0,
            positive_direction_increases_pitch=True,
        )
        motor = WorseningAlignmentMotor(imu)
        clock = FakeClock()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaises(IMUAlignmentError) as caught:
            controller.align()

        self.assertEqual(caught.exception.reason, "worsening_error")
        self.assertTrue(motor.aborted)
        self.assertFalse(motor.finished)

    def test_timeout_aborts_before_stable_success(self):
        clock = FakeClock()

        class SlowIMU(SequenceIMU):
            def read_filtered_pitches(self, **kwargs):
                result = super().read_filtered_pitches(**kwargs)
                clock.sleep(61.0)
                return result

        imu = SlowIMU([(0.0, 0.1)])
        motor = FakeAlignmentMotor()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaises(IMUAlignmentError) as caught:
            controller.align()

        self.assertEqual(caught.exception.reason, "timeout")
        self.assertTrue(motor.aborted)

    def test_relative_travel_limit_rejects_probe(self):
        imu = SequenceIMU([(0.0, 2.0)])
        motor = FakeAlignmentMotor()
        clock = FakeClock()
        settings = replace(
            test_settings(),
            max_relative_travel_steps=100,
        )
        controller = IMULevelingController(
            imu,
            motor,
            settings=settings,
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaises(IMUAlignmentError) as caught:
            controller.align()

        self.assertEqual(caught.exception.reason, "travel_limit")
        self.assertEqual(motor.moves, [])
        self.assertTrue(motor.aborted)

    def test_iteration_limit_aborts(self):
        imu = SimulatedPairIMU(
            base_pitch=0.0,
            upper_pitch=5.0,
            positive_direction_increases_pitch=True,
        )
        motor = CoupledAlignmentMotor(imu)
        clock = FakeClock()
        settings = replace(test_settings(), max_iterations=2)
        controller = IMULevelingController(
            imu,
            motor,
            settings=settings,
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaises(IMUAlignmentError) as caught:
            controller.align()

        self.assertEqual(caught.exception.reason, "max_iterations")
        self.assertTrue(motor.aborted)

    def test_negative_error_converges_when_true_decreases_pitch(self):
        imu = SimulatedPairIMU(
            base_pitch=0.0,
            upper_pitch=-2.0,
            positive_direction_increases_pitch=False,
        )
        motor = CoupledAlignmentMotor(imu)
        clock = FakeClock()
        controller = IMULevelingController(
            imu,
            motor,
            settings=test_settings(),
            clock=clock,
            sleep=clock.sleep,
        )

        result = controller.align()

        self.assertLessEqual(abs(result.final_error_deg), 0.2)
        self.assertFalse(result.positive_direction_increases_pitch)
        self.assertIs(motor.positive_direction_is_true, False)
        self.assertGreater(result.movement_count, 1)
        self.assertTrue(motor.finished)
        self.assertFalse(motor.aborted)


if __name__ == "__main__":
    unittest.main()
