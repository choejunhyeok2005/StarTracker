#!/usr/bin/env python3
"""Indoor AZ yaw measurement using two MPU6050 sensors. No stars required.

Place this file alongside config.py and hardware/imu.py in StarTracker_EL_Tuning_v1.
Default: rotate the AZ stage manually after calibration. Optional --motor sends
bounded STEP pulses; verify wiring, direction, microstepping and travel first.
This is a short-term gyro estimate, NOT an absolute heading or encoder.
"""
import argparse
import csv
import math
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path

import config
from hardware.imu import MPU6050Pair


def yaw_rate(imu, address):
    """Body angular rate projected on the local vertical axis, in deg/s.

    Valid for approximately vertical yaw axes with low acceleration.
    Sign follows the sensed gravity direction, which may be inverted.
    """
    acc, gyro = imu.read_motion(address)
    norm = math.sqrt(sum(v * v for v in acc))
    if not 0.75 * 16384 <= norm <= 1.25 * 16384:
        raise RuntimeError(f"Bad gravity magnitude at 0x{address:02x}: {norm / 16384:.2f} g")
    return sum(g * a for g, a in zip(gyro, acc)) / (131.0 * norm)


def read_pair(imu):
    # Read sequentially; take timestamp at midpoint for integration.
    t0 = time.monotonic()
    base = yaw_rate(imu, imu.base_address)
    upper = yaw_rate(imu, imu.upper_address)
    return (t0 + time.monotonic()) / 2, base, upper


def main():
    p = argparse.ArgumentParser(description="StarTracker AZ relative yaw test (dual MPU6050)")
    p.add_argument("--motor", action="store_true", help="automatically move AZ motor (default: manual rotation)")
    p.add_argument("--steps", type=int, default=960, help="AZ pulses when --motor (1..9600)")
    p.add_argument("--pulse-delay", type=float, default=0.005, help="HIGH and LOW duration in seconds (>=0.005)")
    p.add_argument("--duration", type=float, default=12, help="measurement window in manual mode, seconds")
    p.add_argument("--interval", type=float, default=0.02, help="nominal sensor interval, seconds")
    p.add_argument("--bias-seconds", type=float, default=3, help="stationary gyro calibration duration")
    args = p.parse_args()
    if not 1 <= args.steps <= 9600 or args.pulse_delay < 0.005:
        p.error("--steps must be 1..9600 and --pulse-delay >=0.005")
    if not 3 <= args.duration <= 120 or not 0.01 <= args.interval <= 0.1:
        p.error("--duration must be 3..120 and --interval 0.01..0.1")
    if not 2 <= args.bias_seconds <= 20:
        p.error("--bias-seconds must be 2..20")

    expected_step_angle = config.MOTOR_FULL_STEP_ANGLE / config.MICROSTEP / config.GEAR_RATIO
    expected = args.steps * expected_step_angle
    out = Path("data/az_imu_tests") / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out.mkdir(parents=True, exist_ok=True)
    samples = []
    stop = threading.Event()
    driver = None
    worker = None
    motor_error = []
    issued = [0]
    completed = False
    interrupted = False
    failure = None

    print("\n[AZ MPU6050 relative-angle test]")
    print(f"IMUs: BASE=0x{config.IMU_BASE_ADDRESS:02X}, UPPER=0x{config.IMU_UPPER_ADDRESS:02X}")
    print(f"Motor mode: {args.motor}; expected for {args.steps} pulses: {expected:.4f} deg")
    print("IMPORTANT: Upper IMU must rotate with AZ, base IMU must remain fixed.")
    print("Keep the mount COMPLETELY STILL during initial bias calibration.")
    if args.motor:
        print("WARNING: physical AZ motion. Stop main.py; verify clear path and real TMC2209 mode.")
        print("This script will not configure MS1/MS2 and cannot detect mechanical end stops.")
        if input("Type MOVE to enable motor test: ").strip() != "MOVE":
            print("Cancelled.")
            return

    try:
        with MPU6050Pair(bus_number=config.IMU_I2C_BUS,
                         base_address=config.IMU_BASE_ADDRESS,
                         upper_address=config.IMU_UPPER_ADDRESS) as imu:
            print(f"Calibrating stationary bias for {args.bias_seconds:.1f} seconds...")
            bias_data = []
            start = time.monotonic()
            while time.monotonic() - start < args.bias_seconds:
                _, b, u = read_pair(imu)
                bias_data.append((b, u))
                time.sleep(args.interval)
            if len(bias_data) < 30:
