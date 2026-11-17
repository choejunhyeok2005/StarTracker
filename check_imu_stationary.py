"""Record both stationary IMUs for 30 s. No motor/GPIO initialization."""
import csv
import json
import math
from pathlib import Path
import statistics
import time
from datetime import datetime

import config
from hardware.imu import MPU6050Pair


def main():
    output = Path('data/diagnostics') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    samples = {}
    errors = {}
    print('Stationary IMU test: no motor commands. Keep the mount still for 30 s.')
    with MPU6050Pair(bus_number=config.IMU_I2C_BUS,
                     base_address=config.IMU_BASE_ADDRESS,
                     upper_address=config.IMU_UPPER_ADDRESS) as imu:
        addresses = (imu.base_address, imu.upper_address)
        samples = {address: [] for address in addresses}
        errors = {address: 0 for address in addresses}
        time.sleep(2)
        with (output / 'stationary.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['time_s', 'address', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                             'gravity_g', 'tilt_deg', 'yz_projection_g', 'error'])
            start = time.monotonic()
            for index in range(600):
                for address in addresses:
                    elapsed = time.monotonic() - start
                    try:
                        acceleration, gyro = imu.read_motion(address)
                        x, y, z = acceleration
                        gravity = math.sqrt(x*x + y*y + z*z) / 16384
                        tilt = math.degrees(math.atan2(y, z))
                        projection = math.hypot(y, z) / 16384
                        samples[address].append((elapsed, gravity, tilt, gyro[0]/131, projection))
                        writer.writerow([elapsed, hex(address), *acceleration, *gyro,
                                         gravity, tilt, projection, ''])
                    except Exception as error:
                        errors[address] += 1
                        writer.writerow([elapsed, hex(address)] + ['']*9 + [str(error)])
                if (index+1) % 100 == 0:
                    print(f'{index+1}/600 samples per sensor')
                time.sleep(max(0, start + (index+1)*0.05 - time.monotonic()))
    report = {}
    for address, rows in samples.items():
        result = {'samples': len(rows), 'read_errors': errors[address]}
        if rows:
            gravity = [r[1] for r in rows]
            tilt = [(r[2]-rows[0][2]+180)%360-180 for r in rows]
            gyro = [r[3] for r in rows]
            result.update(gravity_min=min(gravity), gravity_max=max(gravity),
                          gravity_outside_limits=sum(not config.IMU_TRACKING_MIN_GRAVITY_G <= g <= config.IMU_TRACKING_MAX_GRAVITY_G for g in gravity),
                          tilt_range_deg=max(tilt)-min(tilt),
                          tilt_std_deg=statistics.pstdev(tilt),
                          gyro_x_mean_dps=statistics.mean(gyro),
                          gyro_x_std_dps=statistics.pstdev(gyro),
                          yz_projection_min_g=min(r[4] for r in rows))
            first = [t for t, r in zip(tilt, rows) if r[0] < 5]
            last = [t for t, r in zip(tilt, rows) if r[0] >= 25]
            if first and last:
                result['last_minus_first_5s_mean_deg'] = statistics.mean(last)-statistics.mean(first)
        report[hex(address)] = result
    (output / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    print('Saved:', output.resolve())


if __name__ == '__main__':
    main()
