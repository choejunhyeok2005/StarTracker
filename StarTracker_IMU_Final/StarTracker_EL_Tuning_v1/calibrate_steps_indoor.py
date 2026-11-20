
"""StarTracker indoor motor-angle calibration.

Run without --hardware for calculation only.
Hardware mode moves the selected axis; clear its travel path first.
"""

import argparse
import time

import config

STEPS_PER_DEGREE = 960.0
MAX_TEST_STEPS = 9600


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--axis", choices=["AZ", "ALT"], default="AZ")
    parser.add_argument("--steps", type=int, default=960)
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--pulse-delay", type=float, default=0.003)
    args = parser.parse_args()

    if not 1 <= args.steps <= MAX_TEST_STEPS:
        parser.error("steps must be 1..9600")

    if args.pulse_delay < 0.003:
        parser.error("pulse delay must be >= 0.003 s")

    nominal = args.steps / STEPS_PER_DEGREE

    print(f"\nAxis: {args.axis}")
    print(f"Pulses: {args.steps}")
    print(f"Expected rotation: {nominal:.6f} degrees")

    if args.hardware:
        print("\nWARNING: The motor will physically move.")
        print("Confirm power, wiring, clearance and direction.")
        print("Disconnect any obstructing load if necessary.")
        print("Stop main.py before running this test.")
        print("MS1/MS2 hardware configuration must be verified.")

        if input("Type MOVE to proceed: ").strip() != "MOVE":
            print("Cancelled.")
            return

        # Avoid accidentally driving GPIO 1/0 as MS pins.
        # This does not configure the TMC2209 microstep mode.
        config.MOTOR_MS1_PIN = None
        config.MOTOR_MS2_PIN = None

        from hardware.motor import MotorDriver

        driver = MotorDriver()
        try:
            driver.initialize()
            driver.enable()
            driver.drive(
                axis=args.axis,
                steps=args.steps,
                direction=True,
                pulse_delay=args.pulse_delay,
            )
        except KeyboardInterrupt:
            print("\nInterrupted.")
            return
        finally:
            driver.cleanup()
    else:
        print("Calculation-only mode: no motor movement.")

    try:
        measured = float(
            input("\nMeasured signed angle in degrees: ")
        )
    except (ValueError, EOFError):
        print("Measurement skipped.")
        return

    ratio = abs(measured) / nominal

    print("\n--- Calibration result ---")
    print(f"Nominal: {nominal:.6f} deg")
    print(f"Measured: {measured:.6f} deg")
    print(f"Ratio: {ratio:.4f}")

    if ratio > 0:
        print(f"Equivalent gear ratio: {27 / ratio:.3f}:1")
        print(f"Equivalent microstep: 1/{32 / ratio:.3f}")
        print(f"Suggested {args.axis}_STEP_RATIO = {ratio:.4f}")

    if measured < 0:
        print("Check motor direction polarity.")

    print("Equivalent values are diagnostic alternatives,")
    print("not proof of the actual hardware configuration.")


if __name__ == "__main__":
    main()
