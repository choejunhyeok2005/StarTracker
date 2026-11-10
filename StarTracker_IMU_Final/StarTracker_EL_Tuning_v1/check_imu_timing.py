"""Measure upper IMU read latency and individual I2C attempts; no motor GPIO."""
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import config
from hardware.imu import MPU6050Pair


class TimedBus:
    def __init__(self, bus):
        self.bus = bus
        self.attempts = []

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def read_i2c_block_data(self, *args, **kwargs):
        started = time.monotonic()
        error = ''
        size = None
        try:
            result = self.bus.read_i2c_block_data(*args, **kwargs)
            size = len(result)
            return result
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            self.attempts.append((time.monotonic() - started, size, error))


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values)-1, round((len(values)-1)*fraction))]


def main():
    output = Path('data/diagnostics') / ('timing_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)
    durations = []
    retry_reads = failures = deadline_exceeded = attempt_errors = 0
    status = 'completed'
    deadline = config.IMU_TRACKING_MAX_READ_SECONDS
    started = time.monotonic()
    print('IMU timing test: 180 seconds, no motor commands. Keep mount still.')
    try:
        with MPU6050Pair(bus_number=config.IMU_I2C_BUS,
                         base_address=config.IMU_BASE_ADDRESS,
                         upper_address=config.IMU_UPPER_ADDRESS) as imu:
            bus = TimedBus(imu._bus)
            imu._bus = bus
            time.sleep(2)
            with (output / 'reads.csv').open('w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(['time_s', 'total_read_ms', 'attempt_count',
                                 'attempts_json', 'accel_xyz', 'gyro_xyz', 'error'])
                started = time.monotonic()
                next_tick = started
                next_report = 30
                while time.monotonic() - started < 180:
                    bus.attempts.clear()
                    acceleration = gyro = None
                    error = ''
                    read_start = time.monotonic()
                    try:
                        acceleration, gyro = imu.read_motion(imu.upper_address)
                    except Exception as exc:
                        error = f'{type(exc).__name__}: {exc}'
                    elapsed = time.monotonic() - read_start
                    durations.append(elapsed)
                    failures += bool(error)
                    retry_reads += len(bus.attempts) > 1
                    attempt_errors += sum(bool(a[2]) for a in bus.attempts)
                    deadline_exceeded += elapsed > deadline
                    writer.writerow([read_start-started, elapsed*1000, len(bus.attempts),
                                     json.dumps(bus.attempts), acceleration, gyro, error])
                    if elapsed > deadline or error:
                        print(f'SLOW/ERROR at {read_start-started:.2f}s: '
                              f'{elapsed*1000:.2f} ms, attempts={len(bus.attempts)}, {error}')
                        file.flush()
                    if time.monotonic()-started >= next_report:
                        print(f'{next_report}/180 s; reads={len(durations)}, '
                              f'over_limit={deadline_exceeded}, retries={retry_reads}')
                        next_report += 30
                        file.flush()
                    next_tick += config.TRACKING_INTERVAL
                    now = time.monotonic()
                    if next_tick < now:
                        next_tick = now + config.TRACKING_INTERVAL
                    time.sleep(max(0, next_tick-time.monotonic()))
    except KeyboardInterrupt:
        status = 'interrupted'
    except Exception as exc:
        status = f'failed: {type(exc).__name__}: {exc}'
    finally:
        result = dict(status=status, motor_initialized=False, samples=len(durations),
                      deadline_ms=deadline*1000, over_deadline=deadline_exceeded,
                      reads_with_retry=retry_reads, i2c_attempt_errors=attempt_errors,
                      failed_reads=failures,
                      note='Standalone upper IMU reads; does not reproduce motor load. '
                           'Elapsed timings include OS scheduling delays.')
        if durations:
            result.update(median_ms=percentile(durations, .5)*1000,
                          p95_ms=percentile(durations, .95)*1000,
                          p99_ms=percentile(durations, .99)*1000,
                          maximum_ms=max(durations)*1000)
        (output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2))
        print('Saved:', output.resolve())


if __name__ == '__main__':
    main()
