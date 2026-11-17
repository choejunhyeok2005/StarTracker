"""PC-only IMU EL closed-loop demo; AZ remains step-estimated."""
import argparse
import json
import math
from pathlib import Path

import config
from controllers.motor_controller import MotorController
from hardware.imu import AltitudeSample, GyroCalibration
from services.control_telemetry import ControlTelemetry, render_report
from simulate_control import SimulatedMount


class SimulatedIMU:
    def __init__(self, plant):
        self.plant = plant
        self.last_alt = plant.alt
        self.last_time = plant.time
        self.last_rate = 0.0
        self.gyro_bias = .12

    def calibrate(self):
        # A synthetic stationary calibration; real hardware collects 100 reads.
        return GyroCalibration(self.gyro_bias, .003, config.IMU_GYRO_CALIBRATION_SAMPLES)

    def read(self):
        # Tilt in degrees; an arbitrary mounting offset and deterministic noise.
        dt = self.plant.time - self.last_time
        if dt > 0:
            self.last_rate = (self.plant.alt - self.last_alt) / dt
            self.last_alt, self.last_time = self.plant.alt, self.plant.time
        return AltitudeSample(self.plant.alt + 7.0 + .02 * math.sin(self.plant.time * 7),
                              self.plant.time, self.last_rate + self.gyro_bias + .003 * math.sin(self.plant.time * 11))


def run_demo(output="data/kalman_demo"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    plant = SimulatedMount()
    csv_path = output / "imu_control.csv"
    motor = MotorController(driver=plant, telemetry=ControlTelemetry(csv_path), clock=lambda: plant.time)
    motor.current_alt, motor.current_az = plant.alt, plant.az
    motor.enable_imu_tracking(None, sensor=SimulatedIMU(plant))
    try:
        for tick in range(1600):
            plant.time = tick * config.TRACKING_INTERVAL
            motor.set_target(41.0, (359.8 + .004 * plant.time) % 360)
            if tick == 600:
                plant.alt -= .7
            motor.update()
        result = {"simulation_only": True, "duration_s": 80,
                  "disturbance_s": 30, "disturbance_alt_deg": -.7,
                  "final_measured_alt_error_deg": motor.target_alt - motor.current_alt,
                  "final_physical_alt_error_deg": motor.target_alt - plant.alt,
                  "filter_mode": config.IMU_FILTER_MODE,
                  "estimated_gyro_bias_dps": motor.imu_feedback.kalman.bias if motor.imu_feedback.kalman else None,
                  "az_source": "step_estimate_not_IMU"}
    finally:
        motor.cleanup()
    render_report(csv_path, output / "report.html",
                  label="IMU EL 필터 시뮬레이션 · 실물 측정값 아님 · 30초에 −0.7° 외란",
                  az_is_estimate=True)
    (output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="data/kalman_demo")
    run_demo(parser.parse_args().output)
