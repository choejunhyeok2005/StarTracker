"""Hardware-free demo. Run: python simulate_control.py --output data/control_demo"""
import argparse
import json
from pathlib import Path

import config
from controllers.motor_controller import MotorController
from hardware.encoder import EncoderSample
from services.control_telemetry import ControlTelemetry, render_report


class SimulatedMount:
    """A deterministic output encoder and motor plant with missed steps."""
    def __init__(self):
        self.time = 0.0
        self.alt = 40.0
        self.az = 359.8
        self.efficiency = 0.85
        self.stopped = False

    def read(self):
        return EncoderSample(self.alt, self.az, self.time)

    def drive_dual(self, alt_steps, alt_direction, az_steps, az_direction):
        if self.stopped:
            return
        self.alt += alt_steps * config.STEP_ANGLE * self.efficiency * (1 if alt_direction else -1)
        self.az += az_steps * config.STEP_ANGLE * self.efficiency * (1 if az_direction else -1)

    def stop(self):
        self.stopped = True

    def resume(self):
        self.stopped = False

    def cleanup(self):
        self.stop()

    def close(self):
        pass


def run_demo(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "control.csv"
    plant = SimulatedMount()
    motor = MotorController(driver=plant, encoder=plant,
                            telemetry=ControlTelemetry(csv_path), clock=lambda: plant.time)
    motor.set_current_position(40.0, 359.8)
    final_error = None
    try:
        for tick in range(1200):
            plant.time = tick * config.TRACKING_INTERVAL
            # 0–360 crossing, a ramp target, then a fixed target.
            drift = min(plant.time, 40.0) * 0.004
            motor.set_target(40.35 + drift, (0.2 + drift) % 360)
            if tick == 400:
                plant.alt -= 0.15  # external disturbance at t=20s
                plant.az += 0.12
            motor.update()
        alt, az = motor.feedback.position()
        final_error = {"alt_deg": motor.target_alt - alt,
                       "az_deg": motor._shortest_azimuth_error(motor.target_az, az)}
    finally:
        motor.cleanup()
    render_report(csv_path, output / "report.html",
                  label="시뮬레이션 · 실물 측정값 아님 · 15% 응답 손실 / 20초 외란")
    summary = {"simulation_only": True, "duration_s": 60,
               "motor_response_fraction": 0.85, "disturbance_time_s": 20,
               "final_error": final_error}
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Report: {(output / 'report.html').resolve()}")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="data/control_demo")
    run_demo(parser.parse_args().output)
