import unittest

from controllers.motor_controller import MotorController, MotorMode


class FakeMotorDriver:
    def __init__(self):
        self.drives = []
        self.initialized = False
        self.cleaned = False

    def initialize(self):
        self.initialized = True

    def set_mode(self, mode):
        self.mode = mode

    def enable(self):
        self.enabled = True

    def drive(self, *, axis, direction, steps, pulse_delay=None):
        self.drives.append(
            {
                "axis": axis,
                "direction": direction,
                "steps": steps,
                "pulse_delay": pulse_delay,
            }
        )

    def drive_dual(self, **kwargs):
        raise AssertionError("ALIGN mode must not use dual-axis movement.")

    def cleanup(self):
        self.cleaned = True


class MotorAlignmentModeTests(unittest.TestCase):
    """Catch AZ movement, unbounded travel, and invalid zero commits."""

    def setUp(self):
        self.driver = FakeMotorDriver()
        self.controller = MotorController(driver=self.driver)
        self.controller.initialize()

    def test_alignment_moves_only_alt_and_blocks_manual_control(self):
        self.controller.begin_alignment(max_relative_steps=300)

        with self.assertRaises(RuntimeError):
            self.controller.move_manual(
                axis="AZ",
                direction=True,
                steps=1,
            )

        self.controller.move_alignment(
            direction=False,
            steps=120,
            pulse_delay=0.002,
        )

        self.assertEqual(
            self.driver.drives,
            [
                {
                    "axis": "ALT",
                    "direction": False,
                    "steps": 120,
                    "pulse_delay": 0.002,
                }
            ],
        )
        self.assertEqual(self.controller.alignment_relative_steps, -120)

    def test_alignment_refuses_relative_travel_beyond_limit(self):
        self.controller.begin_alignment(max_relative_steps=300)
        self.controller.move_alignment(direction=True, steps=250)

        with self.assertRaises(RuntimeError):
            self.controller.move_alignment(direction=True, steps=51)

        self.assertEqual(len(self.driver.drives), 1)

    def test_finish_alignment_commits_altitude_zero(self):
        self.controller.current_alt = 42.0
        self.controller.target_alt = 12.0
        self.controller.target_az = 87.0
        self.controller.begin_alignment(max_relative_steps=300)
        self.controller.move_alignment(direction=True, steps=100)

        self.controller.finish_alignment()

        self.assertEqual(self.controller.mode, MotorMode.STOP)
        self.assertEqual(self.controller.current_alt, 0.0)
        self.assertEqual(self.controller.target_alt, 0.0)
        self.assertEqual(self.controller.target_az, 0.0)
        self.assertEqual(self.controller.alignment_relative_steps, 0)

    def test_abort_does_not_claim_a_new_altitude_zero(self):
        self.controller.current_alt = 17.0
        self.controller.begin_alignment(max_relative_steps=300)

        self.controller.abort_alignment()

        self.assertEqual(self.controller.mode, MotorMode.STOP)
        self.assertEqual(self.controller.current_alt, 17.0)

    def test_discovered_alt_polarity_is_used_by_manual_control(self):
        self.controller.begin_alignment(max_relative_steps=300)
        self.controller.finish_alignment(
            positive_direction_is_true=False
        )

        self.controller.move_manual(
            axis="ALT",
            direction=True,
            steps=10,
        )

        self.assertFalse(self.driver.drives[-1]["direction"])
        self.assertGreater(self.controller.current_alt, 0.0)


if __name__ == "__main__":
    unittest.main()
