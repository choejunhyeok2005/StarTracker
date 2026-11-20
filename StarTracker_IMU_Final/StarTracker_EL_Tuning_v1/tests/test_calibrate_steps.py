import json
import tempfile
import unittest
from pathlib import Path

import config
from calibrate_steps import SimulatedRig, StepCalibration, main, summarize_axis


def run(rig, axis="ALT", steps=960, repeats=2):
    procedure = StepCalibration(rig.move, rig.measure, steps=steps, repeats=repeats,
                                sleep=lambda s: None, log=lambda *a: None)
    return summarize_axis(procedure.run_axis(axis), steps), procedure


class StepCalibrationTests(unittest.TestCase):
    def test_recovers_response_and_backlash(self):
        rig = SimulatedRig(response=(0.40, 0.25), backlash=(0.05, 0.0), noise=0.0)
        alt, _ = run(rig, "ALT")
        self.assertAlmostEqual(alt["ratio"], 0.40, places=6)
        self.assertAlmostEqual(alt["backlash_deg"], 0.05, places=6)
        az, _ = run(rig, "AZ")
        self.assertAlmostEqual(az["ratio"], 0.25, places=6)
        self.assertTrue(any("마이크로스텝" in n for n in az["diagnosis"]))
        self.assertAlmostEqual(az["equivalent_microstep"], config.MICROSTEP * 4)

    def test_correct_setup_reported_as_normal(self):
        summary, _ = run(SimulatedRig(response=(1.0, 1.0), backlash=(0, 0), noise=0.0))
        self.assertTrue(any("정상" in n for n in summary["diagnosis"]))

    def test_azimuth_wraps_through_zero(self):
        rig = SimulatedRig(response=(1.0, 1.0), backlash=(0, 0), noise=0.0)
        rig.az = 359.5
        summary, _ = run(rig, "AZ")
        self.assertAlmostEqual(summary["ratio"], 1.0, places=6)

    def test_net_movement_per_repeat_is_zero(self):
        _, procedure = run(SimulatedRig(noise=0.0))
        self.assertEqual(sum(r["steps"] for r in procedure.rows), 0)

    def test_alt_limit_refuses_before_moving(self):
        rig = SimulatedRig()
        rig.alt = config.MAX_ALTITUDE - 6
        moves = []
        procedure = StepCalibration(lambda a, s: moves.append(s), rig.measure, steps=960,
                                    repeats=1, sleep=lambda s: None, log=lambda *a: None)
        with self.assertRaisesRegex(RuntimeError, "room"):
            procedure.run_axis("ALT")
        self.assertEqual(moves, [])

    def test_rejects_oversized_moves(self):
        with self.assertRaises(ValueError):
            StepCalibration(None, None, steps=10_000, repeats=1)

    def test_simulation_cli_writes_results(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(main(["--output", directory, "--repeats", "1"]), 0)
            data = json.loads((Path(directory) / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(set(data["axes"]), {"ALT", "AZ"})
            self.assertEqual(data["settings"]["mode"], "sim")
            self.assertTrue((Path(directory) / "moves.csv").exists())

    def test_hardware_requires_location(self):
        with self.assertRaises(SystemExit):
            main(["--hardware", "--yes"])


if __name__ == "__main__":
    unittest.main()
