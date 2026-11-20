

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
                raise RuntimeError("Not enough calibration samples")
            base_bias = statistics.mean(x[0] for x in bias_data)
            upper_bias = statistics.mean(x[1] for x in bias_data)
            base_std = statistics.pstdev(x[0] for x in bias_data)
            upper_std = statistics.pstdev(x[1] for x in bias_data)
            print(f"Base gyro bias: {base_bias:+.4f} deg/s (std {base_std:.4f})")
            print(f"Upper gyro bias: {upper_bias:+.4f} deg/s (std {upper_std:.4f})")
            if max(base_std, upper_std) > 0.5:
                raise RuntimeError("IMU moved or is too noisy during stationary calibration")

            if args.motor:
                # The current repository uses GPIO 1/0 as MS pin numbers, not HIGH/LOW.
                # Do not touch them during this calibration; the physical driver
                # must already be correctly strapped/configured.
                config.MOTOR_MS1_PIN = None
                config.MOTOR_MS2_PIN = None
                from hardware.motor import MotorDriver
                driver = MotorDriver()
                driver.initialize()
                driver.enable()

                def run_motor():
                    try:
                        # Count each completed driver call separately. A call
                        # may still be interrupted mid-pulse by a signal.
                        while issued[0] < args.steps and not stop.is_set():
                            driver.drive("AZ", 1, bool(config.MOTOR_AZ_POSITIVE_DIRECTION),
                                         pulse_delay=args.pulse_delay)
                            if stop.is_set():
                                break
                            issued[0] += 1
                    except Exception as exc:
                        motor_error.append(str(exc))
                        stop.set()

            print("Starting capture...")
            if not args.motor:
                print("Rotate the AZ platform by hand NOW, then keep it still.")
            # baseline samples are taken during calibration; begin motion only here
            tprev, bprev, uprev = read_pair(imu)
            a_base = a_upper = 0.0
            start = time.monotonic()
            if args.motor:
                worker = threading.Thread(target=run_motor, daemon=True)
                worker.start()
            end_time = start + (max(args.duration, args.steps * 2 * args.pulse_delay + 2)
                                if args.motor else args.duration)
            while time.monotonic() < end_time:
                t, b, u = read_pair(imu)
                dt = t - tprev
                if dt <= 0 or dt > 0.30:
                    raise RuntimeError(f"Sensor sampling gap too large: {dt:.3f} sec")
                a_base += ((bprev + b) / 2 - base_bias) * dt
                a_upper += ((uprev + u) / 2 - upper_bias) * dt
                samples.append((t - start, b, u, a_base, a_upper, a_upper-a_base))
                tprev, bprev, uprev = t, b, u
                if motor_error:
                    raise RuntimeError("Motor error: " + motor_error[0])
                time.sleep(max(0, args.interval - (time.monotonic() - t)))
            if worker:
                worker.join(timeout=2)
                if worker.is_alive():
                    raise RuntimeError("Motor thread did not stop as expected")
            completed = not args.motor or issued[0] == args.steps
    except KeyboardInterrupt:
        interrupted = True
        print("\nInterrupted. Motor stop requested.")
    except Exception as exc:
        failure = str(exc)
        print(f"\n[ERROR] {failure}")
    finally:
        stop.set()
        if driver is not None:
            driver.stop()
            if worker is not None:
                worker.join(timeout=3)
                if worker.is_alive():
                    print("[WARNING] Motor thread remains active; check motor power and controller.")
            driver.cleanup()
        if samples:
            filename = out / "az_imu.csv"
            with filename.open("w", newline="", encoding="utf-8") as f:
                wr = csv.writer(f)
                wr.writerow(["time_s", "base_rate_dps", "upper_rate_dps", "base_angle_deg",
                             "upper_angle_deg", "relative_az_deg"])
                wr.writerows(samples)
            measured = samples[-1][-1]
            valid = completed and not interrupted and failure is None
            print(f"\n[{'RESULT' if valid else 'INCOMPLETE'}] Relative AZ gyro estimate: {measured:+.3f} deg")
            if args.motor:
                print(f"Requested pulses: {args.steps}; completed calls: {issued[0]}")
                if valid:
                    print(f"Expected rotation: {expected:.3f} deg")
                    print(f"Measured/expected ratio: {abs(measured) / expected:.3f}")
                else:
                    print("Incomplete test: do not use the angle ratio for calibration")


if __name__ == "__main__":
    main()
