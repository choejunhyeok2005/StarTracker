"""EL step-response experiment. Simulation is default; --hardware drives the mount."""
import argparse
import json
import math
from datetime import datetime
from pathlib import Path
import time

import config
from controllers.motor_controller import MotorController
from services.control_telemetry import ControlTelemetry, render_report


class ELOnlyDriver:
    """Reject any AZ pulse request in this standalone experiment."""
    def __init__(self, driver):
        self.driver = driver

    def __getattr__(self, name):
        return getattr(self.driver, name)

    def drive(self, *, axis, **kwargs):
        if axis not in ("ALT", "EL"):
            raise RuntimeError("EL tuning must not drive AZ")
        return self.driver.drive(axis=axis, **kwargs)

    def drive_dual(self, *, az_steps, **kwargs):
        if az_steps != 0:
            raise RuntimeError("EL tuning must not drive AZ")
        return self.driver.drive_dual(az_steps=0, **kwargs)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--hardware", action="store_true", help="Actually drive Raspberry Pi EL hardware")
    p.add_argument("--start-alt", type=float, help="Known physical initial EL; required for hardware")
    p.add_argument("--step", type=float, default=1.0, help="Signed change from initial EL, at most 5 degrees")
    p.add_argument("--hold", type=float, default=30.0, help="Seconds per target/return phase, including travel")
    p.add_argument("--baseline", type=float, default=5.0)
    p.add_argument("--cycles", type=int, default=3)
    for name in ("kp", "ki", "kd", "tolerance"):
        p.add_argument("--" + name, type=float, default=config.IMU_TRACKING_PID[name])
    p.add_argument("--output", help="New output directory; existing paths are not overwritten")
    return p


def validate(args):
    if args.hardware and args.start_alt is None:
        raise ValueError("Hardware requires --start-alt: independently establish the physical starting EL first")
    if args.start_alt is None:
        args.start_alt = 40.0
    values = (args.start_alt, args.step, args.hold, args.baseline, args.kp, args.ki, args.kd, args.tolerance)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("All numeric settings must be finite")
    if not 0 < abs(args.step) <= 5 or not 1 <= args.cycles <= 20:
        raise ValueError("Require 0 < abs(step) <= 5 degrees and 1..20 cycles")
    if not .1 <= args.hold <= 300 or not .1 <= args.baseline <= 300:
        raise ValueError("Phase durations must be 0.1..300 seconds")
    if min(args.kp, args.ki, args.kd, args.tolerance) < 0:
        raise ValueError("Gains and tolerance must be nonnegative")
    for angle in (args.start_alt, args.start_alt + args.step):
        if not config.MIN_ALTITUDE <= angle <= config.MAX_ALTITUDE:
            raise ValueError("Initial and target EL must be inside configured tracking limits")
    if config.ENCODER_ENABLED or not config.IMU_TRACKING_ENABLED:
        raise ValueError("EL tuning requires IMU tracking enabled and encoder feedback disabled")


def run_sequence(motor, args, clock=time.monotonic, sleep=time.sleep, events=None):
    """Fixed-duration phases. Stop on every exit, including interruption/fault."""
    events = events if events is not None else []
    phases = [("baseline", args.start_alt, args.baseline)]
    for cycle in range(1, args.cycles + 1):
        phases += [(f"step_{cycle}", args.start_alt + args.step, args.hold),
                   (f"return_{cycle}", args.start_alt, args.hold)]
    fixed_az = motor.current_az
    try:
        for name, target, duration in phases:
            motor.set_target(target, fixed_az)
            event = dict(phase=name, target_alt=target,
                         start_time_s=clock() - motor.imu_feedback.start_time,
                         duration_s=duration)
            events.append(event)
            print(f"[Tuning] {name}: EL={target:.3f} deg, duration={duration:.1f}s")
            deadline = clock() + duration
            next_tick = clock()
            while clock() < deadline:
                motor.update()
                next_tick += config.TRACKING_INTERVAL
                sleep(max(0.0, next_tick - clock()))
                if next_tick < clock() - config.TRACKING_INTERVAL:
                    next_tick = clock()
            # A phase ending does not prove the requested angle was reached.
            actual = motor._imu_position()
            event.update(end_time_s=clock() - motor.imu_feedback.start_time,
                         end_actual_alt=actual, end_error_deg=target - actual,
                         within_tolerance=abs(target - actual) <= args.tolerance)
        return {"returned_within_tolerance": events[-1]["within_tolerance"],
                "final_error_deg": events[-1]["end_error_deg"]}
    finally:
        motor.stop()


def run(args):
    validate(args)  # Never initialize GPIO for invalid input.
    output = Path(args.output or f"data/tuning/{datetime.now():%Y%m%d_%H%M%S_%f}")
    output.mkdir(parents=True, exist_ok=False)
    settings = dict(vars(args), simulation_only=not args.hardware,
                    filter_mode=config.IMU_FILTER_MODE, interval_s=config.TRACKING_INTERVAL,
                    max_speed=config.IMU_TRACKING_PID["max_speed"],
                    max_acceleration=config.IMU_TRACKING_PID["max_acceleration"],
                    max_steps_per_update=config.MAX_STEP_PER_UPDATE,
                    kalman_q_angle=config.IMU_KALMAN_Q_ANGLE, kalman_q_bias=config.IMU_KALMAN_Q_BIAS,
                    kalman_r_angle=config.IMU_KALMAN_R_ANGLE)
    (output / "settings.json").write_text(json.dumps(settings, indent=2), encoding="utf-8")
    telemetry = ControlTelemetry(output / "control.csv")
    motor = imu = None
    result, events = {"status": "initializing"}, []
    exit_code = 1
    try:
        if args.hardware:
            from hardware.imu import MPU6050Pair
            from hardware.motor import MotorDriver
            clock, sleep = time.monotonic, time.sleep
            imu = MPU6050Pair(bus_number=config.IMU_I2C_BUS,
                              base_address=config.IMU_BASE_ADDRESS, upper_address=config.IMU_UPPER_ADDRESS)
            motor = MotorController(driver=ELOnlyDriver(MotorDriver()), telemetry=telemetry)
            motor.initialize()
            imu.initialize()
            motor.current_alt, motor.current_az = args.start_alt, 0.0
            motor.enable_imu_tracking(imu)
        else:
            from simulate_control import SimulatedMount
            from simulate_imu_control import SimulatedIMU
            plant = SimulatedMount()
            plant.alt, plant.az = args.start_alt, 0.0
            clock = lambda: plant.time
            def sleep(seconds):
                plant.time += seconds
            motor = MotorController(driver=ELOnlyDriver(plant), telemetry=telemetry, clock=clock)
            motor.current_alt, motor.current_az = args.start_alt, 0.0
            motor.enable_imu_tracking(None, SimulatedIMU(plant))
        # Only this process's controller instance is changed; config.py stays intact.
        pid = motor.imu_feedback.pid
        pid.kp, pid.ki, pid.kd, pid.tolerance = args.kp, args.ki, args.kd, args.tolerance
        result.update(run_sequence(motor, args, clock, sleep, events))
        result["status"] = "completed" if result["returned_within_tolerance"] else "return_outside_tolerance"
        exit_code = 0 if result["returned_within_tolerance"] else 2
    except KeyboardInterrupt:
        result.update(status="interrupted", error="Operator interrupted; stopped without automatic return")
        exit_code = 130
    except Exception as error:
        result.update(status="failed", error=str(error))
    finally:
        # Stop before report generation, and preserve partial records on failure.
        operations = ([motor.stop, motor.cleanup] if motor else [])
        operations += ([imu.close] if imu else []) + [telemetry.close]
        for operation in operations:
            try:
                operation()
            except Exception as error:
                result.setdefault("cleanup_errors", []).append(str(error))
                result["status"], exit_code = "failed", 1
        (output / "events.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
        try:
            render_report(output / "control.csv", output / "report.html",
                          label=("실물" if args.hardware else "시뮬레이션") +
                          f" EL 튜닝 · Kp={args.kp}, Ki={args.ki}, Kd={args.kd}", az_is_estimate=True)
        except Exception as error:
            result["report_error"] = str(error)
        result["simulation_only"] = not args.hardware
        (output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"Saved: {output.resolve()}")
    return exit_code


if __name__ == "__main__":
    cli = parser()
    args = cli.parse_args()
    try:
        validate(args)
    except ValueError as error:
        cli.error(str(error))
    raise SystemExit(run(args))
