"""Measure real degrees per motor step with plate solving (camera = ruler).

Without --hardware this runs a PC simulation (no GPIO, camera or solver).

    python calibrate_steps.py                       # simulation
    python calibrate_steps.py --hardware --lat 37.6 --lon 126.9

Sequence per axis (the mount is stationary while each photo is taken):
    solve -> preload +N/4 -> solve, then per repeat: +N, +N, -N, -N with a
    solve after every move. Net movement per repeat is zero.
The solved RA/Dec is converted to Alt/Az at the photo time, so sky rotation
cancels out: the Alt/Az change between two photos is the axis movement.
"""
import argparse
import csv
import json
import math
import random
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import config

MAX_STEPS = 2880            # 3 deg nominal per move
ALT_MARGIN_DEG = 5.0


def wrap(x):
    return (x + 180.0) % 360.0 - 180.0


class StepCalibration:
    """Hardware-independent procedure. mover(axis, signed_steps); measure() -> (alt, az, meta)."""

    def __init__(self, mover, measure, *, steps, repeats, settle=2.0, sleep=time.sleep,
                 log=print):
        if not 0 < steps <= MAX_STEPS:
            raise ValueError(f"steps must be 1..{MAX_STEPS}")
        if repeats < 1:
            raise ValueError("repeats must be >= 1")
        self.mover, self.measure = mover, measure
        self.steps, self.repeats, self.settle = steps, repeats, settle
        self.sleep, self.log = sleep, log
        self.rows = []

    def plan(self):
        preload = max(1, self.steps // 4)
        moves = [+self.steps, +self.steps, -self.steps, -self.steps] * self.repeats
        return preload, moves

    def check_limits(self, axis, alt):
        if axis != "ALT":
            return
        preload, _ = self.plan()
        reach = (preload + 2 * self.steps) * config.STEP_ANGLE * 1.2
        low, high = config.MIN_ALTITUDE + ALT_MARGIN_DEG, config.MAX_ALTITUDE - ALT_MARGIN_DEG
        if not low <= alt <= high - reach:
            raise RuntimeError(
                f"ALT {alt:.2f} deg leaves too little room: need {low:.1f}..{high - reach:.1f} deg "
                f"for +{reach:.2f} deg of travel. Point lower or use fewer --steps.")

    def _move(self, axis, signed):
        self.mover(axis, signed)
        self.sleep(self.settle)

    def run_axis(self, axis):
        preload, moves = self.plan()
        start = self.measure()
        self.log(f"[{axis}] start ALT={start[0]:.4f} AZ={start[1]:.4f}")
        self.check_limits(axis, start[0])
        self._move(axis, +preload)            # take up slack in the + direction
        previous = self.measure()
        last_direction = +1
        for index, signed in enumerate(moves, 1):
            self._move(axis, signed)
            current = self.measure()
            direction = 1 if signed > 0 else -1
            nominal = signed * config.STEP_ANGLE
            if axis == "ALT":
                measured, cross = current[0] - previous[0], wrap(current[1] - previous[1])
            else:
                measured, cross = wrap(current[1] - previous[1]), current[0] - previous[0]
            row = {
                "axis": axis, "move": index, "steps": signed,
                "reversal": direction != last_direction,
                "nominal_deg": nominal, "measured_deg": measured,
                "ratio": measured / nominal, "cross_axis_deg": cross,
                "alt": current[0], "az": current[1], **current[2],
            }
            self.rows.append(row)
            self.log(f"[{axis}] move {index}: {signed:+d} steps nominal {nominal:+.4f} "
                     f"measured {measured:+.4f} ratio {row['ratio']:.3f}"
                     f"{' (reversal)' if row['reversal'] else ''}")
            previous, last_direction = current, direction
        return [r for r in self.rows if r["axis"] == axis]


def summarize_axis(rows, steps):
    same = [r["ratio"] for r in rows if not r["reversal"]]
    rev = [r["ratio"] for r in rows if r["reversal"]]
    fwd = [r["ratio"] for r in rows if r["steps"] > 0 and not r["reversal"]]
    back = [r["ratio"] for r in rows if r["steps"] < 0 and not r["reversal"]]
    ratio = statistics.median(same)
    spread = statistics.pstdev(same) / abs(ratio) if len(same) > 1 and ratio else 0.0
    backlash = ((statistics.mean(same) - statistics.mean(rev)) * steps * config.STEP_ANGLE
                if rev else None)
    result = {
        "ratio": ratio, "ratio_spread": spread,
        "ratio_forward": statistics.median(fwd) if fwd else None,
        "ratio_backward": statistics.median(back) if back else None,
        "backlash_deg": backlash,
        "equivalent_microstep": config.MICROSTEP / ratio if ratio else None,
        "equivalent_gear_ratio": config.GEAR_RATIO / ratio if ratio else None,
        "moves": len(rows),
    }
    result["diagnosis"] = diagnose(result)
    return result


def diagnose(s):
    notes = []
    ratio, spread = s["ratio"], s["ratio_spread"]
    if ratio <= 0:
        return ["축이 반대로 움직임: MOTOR_*_POSITIVE_DIRECTION 또는 결선 확인"]
    if spread > 0.05:
        notes.append(f"배율 편차 {spread:.0%}: 스텝 누락 의심 (펄스 간격, 드라이버 전류, 부하)")
    if abs(ratio - 1) <= 0.03:
        notes.append("배율 약 1.0: 모터/기어 설정 정상. 불일치가 있었다면 IMU 쪽을 의심")
    else:
        for value in (2, 4, 8, 0.5, 0.25, 0.125):
            if abs(ratio / value - 1) <= 0.03 and spread <= 0.05:
                notes.append(f"배율이 {value:g}배에 가까움: 마이크로스텝 설정(MS1/MS2) 의심. "
                             f"실제 마이크로스텝 = {s['equivalent_microstep']:.0f}")
                break
        else:
            if spread <= 0.05:
                notes.append(f"배율 {ratio:.3f}로 일정: 기어비/마이크로스텝 값 확인, "
                             f"안 되면 config의 축별 STEP_RATIO로 보정 가능")
    fwd, back = s["ratio_forward"], s["ratio_backward"]
    if fwd and back and abs(fwd - back) / max(abs(fwd), abs(back)) > 0.05:
        notes.append(f"정방향 {fwd:.3f} / 역방향 {back:.3f} 차이: 축 균형(무게중심) 또는 부하 방향 영향")
    if s["backlash_deg"] is not None and s["backlash_deg"] > 0.02:
        notes.append(f"백래시 약 {s['backlash_deg']:.3f} deg: 방향 전환 시 손실")
    return notes


def save(output, rows, summary, settings):
    output.mkdir(parents=True, exist_ok=True)
    if rows:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        with (output / "moves.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(
        {"settings": settings, "axes": summary}, indent=2, ensure_ascii=False), encoding="utf-8")


# ---------------------------------------------------------------- simulation
class SimulatedRig:
    """Plant with per-axis response, backlash and plate-solve noise."""

    def __init__(self, response=(0.40, 0.40), backlash=(0.03, 0.03), noise=0.002, seed=1):
        self.alt, self.az = 40.0, 120.0
        self.response = dict(ALT=response[0], AZ=response[1])
        self.backlash = dict(ALT=backlash[0], AZ=backlash[1])
        self.last_dir = dict(ALT=+1, AZ=+1)
        self.rng, self.noise = random.Random(seed), noise

    def move(self, axis, signed):
        direction = 1 if signed > 0 else -1
        travel = abs(signed) * config.STEP_ANGLE * self.response[axis]
        if direction != self.last_dir[axis]:
            travel = max(0.0, travel - self.backlash[axis])
        self.last_dir[axis] = direction
        if axis == "ALT":
            self.alt += direction * travel
        else:
            self.az = (self.az + direction * travel) % 360

    def measure(self):
        return (self.alt + self.rng.gauss(0, self.noise),
                (self.az + self.rng.gauss(0, self.noise)) % 360, {"source": "simulation"})


# ------------------------------------------------------------------ hardware
def build_hardware(args, output):
    from astropy.time import Time
    from camera.camera_manager import CameraManager
    from controllers.motor_controller import MotorController
    from hardware.motor import MotorDriver
    from services.astronomy import Astronomy
    from services.platesolver import PlateSolver

    astronomy = Astronomy()
    astronomy.set_location(args.lat, args.lon, args.elevation)
    camera = CameraManager()
    camera.initialize()
    solver = PlateSolver()
    driver = MotorDriver()
    motor = MotorController(driver=driver)
    motor.initialize()
    counter = {"n": 0}

    def mover(axis, signed):
        driver.drive(axis, abs(signed), motor._to_driver_direction(axis, signed > 0),
                     pulse_delay=args.pulse_delay)

    def measure():
        for attempt in (1, 2):
            counter["n"] += 1
            path = str(output / f"plate_{counter['n']:03d}.jpg")
            started = datetime.now(timezone.utc)
            camera.capture_plate(filename=path)
            finished = datetime.now(timezone.utc)
            result = solver.solve(path)
            if result:
                midpoint = started + (finished - started) / 2
                alt, az = astronomy.radec_to_altaz(float(result["ra"]), float(result["dec"]),
                                                   observation_time=Time(midpoint))
                return alt, az, {"image": path, "time_utc": midpoint.isoformat(),
                                 "ra": result["ra"], "dec": result["dec"]}
            print(f"[Calibrate] plate solve failed ({attempt}/2): {path}")
        raise RuntimeError("Plate solving failed twice; stopping")

    def cleanup():
        try:
            driver.stop()
        finally:
            motor.cleanup()
            stop_camera = getattr(camera, "stop", None)
            if stop_camera:
                stop_camera()

    return mover, measure, cleanup


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hardware", action="store_true", help="move real motors and use the camera")
    p.add_argument("--axis", choices=("alt", "az", "both"), default="both")
    p.add_argument("--steps", type=int, default=960, help="steps per move (960 = 1 deg nominal)")
    p.add_argument("--repeats", type=int, default=2, help="each repeat is +N,+N,-N,-N")
    p.add_argument("--settle", type=float, default=2.0, help="seconds to wait after each move")
    p.add_argument("--pulse-delay", type=float, default=None,
                   help=f"seconds per pulse half-period (default config {config.MOTOR_PULSE_DELAY})")
    p.add_argument("--lat", type=float, help="observer latitude (hardware mode)")
    p.add_argument("--lon", type=float, help="observer longitude (hardware mode)")
    p.add_argument("--elevation", type=float, default=0.0)
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p.add_argument("--output", default=None)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    axes = ["ALT", "AZ"] if args.axis == "both" else [args.axis.upper()]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "hw" if args.hardware else "sim"
    output = Path(args.output or f"data/calibration/{mode}_{stamp}")
    cleanup = lambda: None
    if args.hardware:
        if args.lat is None or args.lon is None:
            raise SystemExit("--hardware needs --lat and --lon")
        preload = max(1, args.steps // 4)
        travel = (preload + 2 * args.steps) * config.STEP_ANGLE
        print(f"Axes {axes}, {args.steps} steps/move ({args.steps * config.STEP_ANGLE:.3f} deg nominal), "
              f"{args.repeats} repeat(s), up to {travel:.2f} deg nominal travel per axis.")
        print("Point at a clear star field, keep cables free. Ctrl+C stops the motors.")
        if not args.yes and input("Move the motors now? [y/N] ").strip().lower() != "y":
            raise SystemExit("Cancelled; motors were not moved.")
        output.mkdir(parents=True, exist_ok=False)
        mover, measure, cleanup = build_hardware(args, output)
        sleep = time.sleep
    else:
        print("SIMULATION ONLY - true response ALT/AZ 0.40, backlash 0.03 deg")
        rig = SimulatedRig()
        mover, measure, sleep = rig.move, rig.measure, (lambda s: None)
    procedure = StepCalibration(mover, measure, steps=args.steps, repeats=args.repeats,
                                settle=args.settle, sleep=sleep)
    summary, status = {}, "completed"
    settings = {"mode": mode, "steps": args.steps, "repeats": args.repeats,
                "step_angle": config.STEP_ANGLE, "microstep": config.MICROSTEP,
                "gear_ratio": config.GEAR_RATIO,
                "pulse_delay": args.pulse_delay or config.MOTOR_PULSE_DELAY}
    try:
        for axis in axes:
            rows = procedure.run_axis(axis)
            summary[axis] = summarize_axis(rows, args.steps)
            # Ratio is relative to the uncorrected config.STEP_ANGLE, so it
            # goes into config.py as-is regardless of the current setting.
            summary[axis]["config_line"] = f"{axis}_STEP_RATIO = {summary[axis]['ratio']:.4f}"
    except KeyboardInterrupt:
        status = "cancelled"
    except Exception as error:
        status = f"failed: {error}"
        print(f"[Calibrate] {status}")
    finally:
        cleanup()
        settings["status"] = status
        save(output, procedure.rows, summary, settings)
    print(f"\nStatus: {status}. Saved to {output}")
    for axis, s in summary.items():
        print(f"{axis}: ratio {s['ratio']:.4f} (spread {s['ratio_spread']:.1%}), "
              f"backlash {s['backlash_deg'] if s['backlash_deg'] is None else round(s['backlash_deg'], 4)} deg")
        for note in s["diagnosis"]:
            print(f"   - {note}")
        if "config_line" in s:
            print(f"   - 소프트웨어 보정 시 config.py: {s['config_line']}")
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
