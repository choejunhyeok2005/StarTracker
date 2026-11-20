"""PC-only 6-frame sequence simulation that drives the REAL MotorController.

Run: python simulate_tracking_sequence.py
Compares the old behaviour (flags off) with feedforward and with
feedforward + exposure hold, using IMU noise measured on the Pi
(raw tilt std ~0.12 deg, gyro std ~0.06 deg/s). Simulation only.
"""
import argparse
import math
import random
import statistics
from unittest.mock import patch

import config
from controllers.motor_controller import MotorController
from hardware.imu import AltitudeSample, GyroCalibration
from simulate_control import SimulatedMount

PIXEL_DEG = 20.1 / 3600          # plate solve 2026-11-06
LAT, DEC = 37.6, 54.87           # observer and target of session 20261106T132811
SIDEREAL_DPS = 360 / 86164.0905
GYRO_BIAS = 0.12


class NoisyIMU:
    """Upper MPU6050 seen through measured noise levels."""

    def __init__(self, plant, rng, tilt_std=0.12, gyro_std=0.06):
        self.plant, self.rng = plant, rng
        self.tilt_std, self.gyro_std = tilt_std, gyro_std
        self.last_alt, self.last_time, self.rate = plant.alt, plant.time, 0.0

    def calibrate(self):
        return GyroCalibration(GYRO_BIAS + self.rng.gauss(0, 0.01), self.gyro_std, 100)

    def read(self):
        dt = self.plant.time - self.last_time
        if dt > 0:
            self.rate = (self.plant.alt - self.last_alt) / dt
            self.last_alt, self.last_time = self.plant.alt, self.plant.time
        return AltitudeSample(self.plant.alt + 7.0 + self.rng.gauss(0, self.tilt_std),
                              self.plant.time,
                              self.rate + GYRO_BIAS + self.rng.gauss(0, self.gyro_std))


def altaz(hour_angle):
    h, d, p = map(math.radians, (hour_angle, DEC, LAT))
    alt = math.asin(math.sin(d) * math.sin(p) + math.cos(d) * math.cos(p) * math.cos(h))
    az = math.atan2(-math.sin(h) * math.cos(d),
                    math.sin(d) * math.cos(p) - math.cos(d) * math.sin(p) * math.cos(h))
    return math.degrees(alt), math.degrees(az) % 360


def wrap(x):
    return (x + 180) % 360 - 180


def run_sequence(seed, efficiency, frames=6, exposure=10.0, solve=8.0):
    rng = random.Random(seed)
    dt = config.TRACKING_INTERVAL
    ha0 = rng.uniform(-30, 30)
    plant = SimulatedMount()
    plant.efficiency = efficiency
    alt0, az0 = altaz(ha0)
    plant.alt, plant.az = alt0 + rng.gauss(0, 0.05), az0 + rng.gauss(0, 0.05)
    motor = MotorController(driver=plant, clock=lambda: plant.time)
    motor.current_alt, motor.current_az = alt0, az0      # first plate solve
    motor.enable_imu_tracking(None, NoisyIMU(plant, rng))

    blur, settle_times, timeouts, fault = [], [], 0, None
    phase, phase_t, frame, streak, track, now = "expose", 0.0, 1, 0, [], 0.0
    motor.set_exposure_hold(True)
    try:
        while frame <= frames:
            now += dt
            phase_t += dt
            plant.time = now
            target_alt, target_az = altaz(ha0 + SIDEREAL_DPS * now)
            motor.set_target(target_alt, target_az)
            motor.update()
            star = (target_alt - plant.alt,
                    wrap(target_az - plant.az) * math.cos(math.radians(target_alt)))
            if phase == "expose":
                track.append(star)
                if phase_t >= exposure:
                    motor.set_exposure_hold(False)
                    blur.append(max(math.dist(a, b) for a in track for b in track[::4]))
                    mid = track[len(track) // 2]
                    correction = (mid[0] + rng.gauss(0, 0.005),
                                  mid[1] / math.cos(math.radians(target_alt)) + rng.gauss(0, 0.005))
                    track, frame, phase, phase_t = [], frame + 1, "solve", 0.0
            elif phase == "solve" and phase_t >= solve:
                motor.apply_position_offset(-correction[0], -correction[1])
                phase, phase_t, streak = "settle", 0.0, 0
            elif phase == "settle":
                ok = (abs(target_alt - motor.current_alt) <= config.IMU_TRACKING_PID["tolerance"]
                      and abs(wrap(target_az - motor.current_az)) <= config.ANGLE_TOLERANCE)
                streak = streak + 1 if ok else 0
                if streak >= config.PIPELINED_SETTLE_SAMPLES or phase_t >= 30:
                    settle_times.append(phase_t)
                    timeouts += streak < config.PIPELINED_SETTLE_SAMPLES
                    motor.set_exposure_hold(True)
                    phase, phase_t = "expose", 0.0
    except RuntimeError as error:
        fault = str(error)
    return blur, settle_times, timeouts, fault


def summarize(name, efficiency, seeds, flags):
    with patch.multiple(config, CONTROL_TELEMETRY_ENABLED=False, **flags):
        runs = [run_sequence(seed, efficiency) for seed in seeds]
    blur = sorted(b / PIXEL_DEG for r in runs for b in r[0])
    settle = [s for r in runs for s in r[1]]
    faults = [r[3] for r in runs if r[3]]
    p90 = blur[int(0.9 * (len(blur) - 1))]
    print(f"{name:<22} {efficiency:>5.2f} {statistics.median(blur):>8.2f} {p90:>8.2f} "
          f"{blur[-1]:>8.2f} {statistics.median(settle):>8.2f} "
          f"{sum(r[2] for r in runs):>5} {len(faults):>6}")
    for fault in sorted(set(faults)):
        print(f"{'':<22} fault: {fault}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=20)
    args = parser.parse_args()
    seeds = range(1, args.seeds + 1)
    variants = (
        ("old (flags off)", dict(TRACKING_FEEDFORWARD_ENABLED=False,
                                 TRACKING_HOLD_FEEDBACK_DURING_EXPOSURE=False)),
        ("feedforward only", dict(TRACKING_FEEDFORWARD_ENABLED=True,
                                  TRACKING_HOLD_FEEDBACK_DURING_EXPOSURE=False)),
        ("feedforward + hold", dict(TRACKING_FEEDFORWARD_ENABLED=True,
                                    TRACKING_HOLD_FEEDBACK_DURING_EXPOSURE=True)),
    )
    print("SIMULATION ONLY - real Python controller, synthetic plant and IMU noise")
    print(f"{'variant':<22} {'resp':>5} {'blur50':>8} {'blur90':>8} {'blurMax':>8} "
          f"{'settle_s':>8} {'tmo':>5} {'faults':>6}   (blur in px, 10 s frames)")
    for efficiency in (0.85, 0.40):
        for name, flags in variants:
            summarize(name, efficiency, seeds, flags)


if __name__ == "__main__":
    main()
