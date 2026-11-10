"""Loaded IMU deadline diagnostic. ALT oscillates +/-120 relative steps; no camera/GNSS/BLE."""
import argparse
import csv
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import config
from controllers.motor_controller import MotorController
from hardware.imu import IMUAltitudeSensor, MPU6050Pair

class TimedBus:
    def __init__(self, bus):
        self.bus = bus
        self.attempts = []
    def __getattr__(self, name):
        return getattr(self.bus, name)
    def read_i2c_block_data(self, *args, **kwargs):
        started = time.monotonic()
        error = ""
        try:
            result = self.bus.read_i2c_block_data(*args, **kwargs)
            return result
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.attempts.append({"elapsed_ms": (time.monotonic()-started)*1000,
                                  "error": error})

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 180:
        raise SystemExit('--seconds must be between 5 and 180')
    out = Path('data/diagnostics') / ('imu_loaded_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True, exist_ok=False)
    motor = MotorController()
    stop = threading.Event()
    worker_error = []
    durations = []
    failures = []
    motor_started = False
    try:
        pair = MPU6050Pair(bus_number=config.IMU_I2C_BUS,
                           base_address=config.IMU_BASE_ADDRESS,
                           upper_address=config.IMU_UPPER_ADDRESS)
        for attempt in range(1, 4):
            try:
                pair.initialize()
                break
            except Exception as exc:
                print(f"IMU initialization attempt {attempt}/3 failed: {exc}")
                if attempt == 3:
                    raise
                time.sleep(0.25)
        sensor = IMUAltitudeSensor(pair)
        timed_bus = TimedBus(pair._bus)
        pair._bus = timed_bus
        motor.initialize()
        motor.begin_alignment(max_relative_steps=120)
        motor_started = True
        def oscillate_alt():
            direction = True
            try:
                while not stop.is_set():
                    motor.move_alignment(direction=direction, steps=120,
                                         pulse_delay=config.MOTOR_PULSE_DELAY)
                    direction = not direction
                    stop.wait(0.75)
            except Exception as exc:
                worker_error.append(f"{type(exc).__name__}: {exc}")
                stop.set()
        worker = threading.Thread(target=oscillate_alt, name='ALT_IMU_Diagnostic', daemon=True)
        worker.start()
        started = time.monotonic()
        deadline = config.IMU_TRACKING_MAX_READ_SECONDS
        with (out/'reads.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_s','read_ms','i2c_attempts_json','angle_deg','gyro_dps','error'])
            while time.monotonic()-started < args.seconds and not stop.is_set():
                tick = time.monotonic()
                timed_bus.attempts.clear()
                angle = gyro = ''
                error = ''
                try:
                    sample = sensor.read()
                    angle, gyro = sample.angle, sample.gyro_rate
                except Exception as exc:
                    error = f"{type(exc).__name__}: {exc}"
                elapsed = (time.monotonic()-tick)*1000
                durations.append(elapsed)
                writer.writerow([tick-started, elapsed, json.dumps(timed_bus.attempts), angle, gyro, error])
                if error:
                    failures.append({"elapsed_s": tick-started, "read_ms": elapsed, "error": error,
                                     "i2c_attempts": timed_bus.attempts.copy()})
                    f.flush()
                    print('IMU_READ_FAILURE:', error)
                    stop.set()
                    break
                time.sleep(max(0, 0.05-(time.monotonic()-tick)))
        stop.set()
        worker.join(timeout=2)
        motor.stop()
        motor.abort_alignment()
        motor.driver.cleanup()
        motor_started = False
        pair.close()
        ordered = sorted(durations)
        p95 = ordered[min(len(ordered)-1, int(0.95*(len(ordered)-1)))] if ordered else None
        summary = {"duration_requested_s": args.seconds, "reads": len(durations),
                   "deadline_ms": deadline*1000, "over_deadline": sum(x>deadline*1000 for x in durations),
                   "max_read_ms": max(durations) if durations else None, "p95_read_ms": p95,
                   "failures": failures, "motor_worker_error": worker_error,
                   "motor_axis": "ALT", "motor_relative_bound_steps": 120}
        (out/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps(summary, indent=2))
        print('Saved:', out.resolve())
    finally:
        stop.set()
        if motor_started:
            try:
                motor.stop()
                motor.abort_alignment()
                motor.driver.cleanup()
            except Exception:
                pass
        if 'pair' in locals():
            pair.close()

if __name__ == '__main__':
    main()
