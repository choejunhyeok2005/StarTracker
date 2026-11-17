"""Run existing EL tuning unchanged, adding read and motor-update timings."""
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import config
from hardware.imu import MPU6050Pair
from controllers.motor_controller import MotorController
from check_imu_timing import TimedBus
import tune_el


def main():
    config.IMU_GYRO_CALIBRATION_MAX_TILT_RANGE = 0.8
    config.IMU_TRACKING_PID['max_speed'] = 0.05
    args = tune_el.parser().parse_args()
    tune_el.validate(args)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    if args.output is None:
        args.output = f'data/tuning/{stamp}'
    output = Path('data/diagnostics') / ('loaded_' + stamp)
    output.mkdir(parents=True, exist_ok=False)
    read_rows, update_rows = [], []
    start = time.monotonic()
    original_read = MPU6050Pair.read_motion
    original_update = MotorController.update
    previous_update = None

    def read_motion(pair, address):
        if not isinstance(pair._bus, TimedBus):
            pair._bus = TimedBus(pair._bus)
        bus = pair._bus
        bus.attempts.clear()
        error = ''
        acceleration = gyro = None
        began = time.monotonic()
        try:
            acceleration, gyro = original_read(pair, address)
            return acceleration, gyro
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            elapsed = time.monotonic() - began
            read_rows.append((began-start, hex(address), elapsed*1000,
                              list(bus.attempts), acceleration, gyro, error))

    def update(motor):
        nonlocal previous_update
        began = time.monotonic()
        gap = None if previous_update is None else (began-previous_update)*1000
        previous_update = began
        error = ''
        try:
            return original_update(motor)
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            update_rows.append((began-start, (time.monotonic()-began)*1000,
                                gap, error))

    # Buffer diagnostics in memory to avoid per-tick disk writes.
    MPU6050Pair.read_motion = read_motion
    MotorController.update = update
    result = 1
    try:
        result = tune_el.run(args)
    finally:
        MPU6050Pair.read_motion = original_read
        MotorController.update = original_update
        with (output/'reads.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'address', 'total_read_ms', 'attempts_json',
                             'accel_xyz', 'gyro_xyz', 'error'])
            for row in read_rows:
                writer.writerow([*row[:3], json.dumps(row[3]), *row[4:]])
        with (output/'updates.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'update_ms', 'start_to_start_ms', 'error'])
            writer.writerows(update_rows)
        report = dict(tuning_output=str(Path(args.output).resolve()),
                      exit_code=result, reads=len(read_rows),
                      read_limit_ms=config.IMU_TRACKING_MAX_READ_SECONDS*1000,
                      reads_over_limit=sum(r[2] > config.IMU_TRACKING_MAX_READ_SECONDS*1000 for r in read_rows),
                      reads_with_retry=sum(len(r[3]) > 1 for r in read_rows),
                      read_errors=sum(bool(r[6]) for r in read_rows),
                      max_read_ms=max((r[2] for r in read_rows), default=None),
                      max_update_ms=max((r[1] for r in update_rows), default=None),
                      max_update_gap_ms=max((r[2] for r in update_rows if r[2] is not None), default=None),
                      update_errors=[{'time_s': r[0], 'error': r[3]} for r in update_rows if r[3]],
                      calibration_tilt_limit_deg=config.IMU_GYRO_CALIBRATION_MAX_TILT_RANGE)
        (output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        print('Diagnostic saved:', output.resolve())
    return result


if __name__ == '__main__':
    raise SystemExit(main())
