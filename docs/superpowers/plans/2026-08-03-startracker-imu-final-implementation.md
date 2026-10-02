# StarTracker IMU Final Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a complete Raspberry Pi star-tracker project that automatically aligns the moving upper ALT plate to the fixed base using two MPU6050 sensors, then runs the verified GNSS/BLE/plate-solving/tracking/capture state machine.

**Architecture:** A hardware-only `MPU6050Pair` supplies filtered pitch measurements. A hardware-independent `IMULevelingController` closes the loop through a bounded ALIGN mode on `MotorController`. `TrackingManager` owns startup sequencing and only enables manual/tracking operation after alignment and GNSS configuration succeed.

**Tech Stack:** Python 3.11+, Raspberry Pi 5, `smbus2`, `RPi.GPIO`, Picamera2, Astropy, Astrometry.net, Bleak, PySerial, `unittest`.

## Global Constraints

- Fixed base IMU is read-only at I2C `0x68`; moving upper IMU is I2C `0x69`.
- Only the upper ALT motor moves during alignment; AZ receives zero alignment commands.
- Alignment succeeds at absolute corrected pitch error `<= 0.2 deg` for exactly three consecutive filtered readings.
- Default filter uses the median of 11 samples at 0.01-second intervals.
- Direction is discovered by a 240-step positive probe with minimum response `0.08 deg`.
- Corrections use gain `0.65`, minimum 8 steps, maximum 960 steps.
- Alignment is bounded by 60 seconds, 80 iterations, and 15 degrees net relative travel.
- Default final capture is 18 ten-second subframes; the 180-second single exposure remains experimental.
- Original `C:\Users\이승민\Downloads\CODE_3_algorithm_updated (1)\code` is read-only and remains unchanged.
- The source project has no Git repository; checkpoint commits are replaced by test results and SHA-256 manifests.

---

## File Map

- Create `hardware/imu.py`: MPU6050 register I/O, signed conversion, pitch math, median filtering.
- Create `controllers/imu_leveling_controller.py`: filtered adaptive alignment loop and result/error types.
- Modify `controllers/motor_controller.py`: injectable driver and bounded ALIGN movement API.
- Modify `config.py`: exact IMU, alignment, correction-failure, and capture-sequence defaults.
- Modify `managers/tracking_manager.py`: hardware startup split, alignment integration, correction failure policy, safe capture modes.
- Modify `main.py`: concurrent GNSS acquisition and IMU-first hardware startup order.
- Modify `camera/camera_manager.py`: cancellable subframe sequence capture.
- Modify `communication/ble_manager.py`: stop requests during target capture and final capture.
- Modify `services/tracking_logger.py`: alignment metadata and multiple final image paths.
- Modify `requirements.txt`: add `smbus2`.
- Modify `README.md`: wiring, offsets, state machine, operation, field-rotation limit, acceptance steps.
- Create `tests/test_imu_hardware.py`: register conversion, pitch, median, identity validation.
- Create `tests/test_imu_leveling.py`: convergence, direction, filtering gate, and failure bounds.
- Create `tests/test_motor_alignment.py`: ALIGN-only movement, ALT-only behavior, relative-travel limit, zero commit.

---

### Task 1: Create the Isolated Deliverable and Baseline

**Files:**
- Copy: original project source files into `outputs/StarTracker_IMU_Final/`
- Preserve: `outputs/StarTracker_IMU_Final/docs/`
- Create: `outputs/StarTracker_IMU_Final/baseline-sha256.txt`

**Interfaces:**
- Consumes: source directory named in Global Constraints.
- Produces: runnable project root containing `main.py`, `config.py`, package directories, requirements, README, and design/plan documents.

- [ ] **Step 1: Copy the source tree without deleting existing docs**

Use PowerShell `Copy-Item -Recurse -Force` for every item under the original `code` directory into the deliverable root. Do not copy the three loose attachment files and do not create a root `hardware.py`.

- [ ] **Step 2: Record the copied baseline**

Generate SHA-256 entries for every copied source file, using paths relative to the deliverable root, sorted by path, and write them to `baseline-sha256.txt`.

- [ ] **Step 3: Verify baseline syntax**

Run:

```powershell
python -c "import ast,pathlib; files=list(pathlib.Path('.').rglob('*.py')); [ast.parse(p.read_text(encoding='utf-8'), filename=str(p)) for p in files]; print(len(files))"
```

Expected: all copied Python files parse without an exception.

- [ ] **Step 4: Record checkpoint**

Save the parse result and baseline manifest; Git commit is unavailable by project definition.

### Task 2: MPU6050 Pair Driver

**Files:**
- Create: `hardware/imu.py`
- Create: `tests/test_imu_hardware.py`
- Modify: `requirements.txt`

**Interfaces:**
- Produces: `MPU6050Pair(bus_factory=None, sleep=time.sleep)`, `initialize() -> None`, `read_pitch(address: int) -> float`, `read_filtered_pitches(sample_count: int, sample_interval: float) -> tuple[float, float]`, `close() -> None`.
- Produces pure helpers: `signed_int16(high: int, low: int) -> int` and `acceleration_to_pitch(x: int, y: int, z: int) -> float`.

- [ ] **Step 1: Write failing signed conversion and pitch tests**

```python
class IMUMathTests(unittest.TestCase):
    def test_signed_int16(self):
        self.assertEqual(signed_int16(0x7F, 0xFF), 32767)
        self.assertEqual(signed_int16(0x80, 0x00), -32768)

    def test_level_pitch_is_zero(self):
        self.assertAlmostEqual(acceleration_to_pitch(0, 0, 16384), 0.0)
```

- [ ] **Step 2: Run the tests and confirm missing-module failure**

Run `python -m unittest tests.test_imu_hardware -v`.
Expected: FAIL because `hardware.imu` does not exist.

- [ ] **Step 3: Implement pure conversion and pitch functions**

Use a 16-bit two's-complement conversion and `degrees(atan2(-x, hypot(y, z)))`. Reject an all-zero vector and non-finite results with `IMUReadError`.

- [ ] **Step 4: Add a fake-bus initialization/filter test**

The fake bus must return `0x68`/`0x69` from `WHO_AM_I`, record wake/config writes, and supply six-byte acceleration blocks. Verify 11 noisy samples return their median and both addresses are sampled.

- [ ] **Step 5: Implement lazy SMBus initialization and median filtering**

Import `SMBus` from `smbus2` only when no injected factory is supplied. Read block register `0x3B` with length 6. Validate identities against each configured address, apply pitch offsets only in the alignment controller, and make `close()` idempotent.

- [ ] **Step 6: Run the driver tests**

Run `python -m unittest tests.test_imu_hardware -v`.
Expected: PASS.

- [ ] **Step 7: Add `smbus2` dependency and record checkpoint**

Append exactly `smbus2` to `requirements.txt` if absent, rerun the test, and retain the output.

### Task 3: Bounded Closed-Loop Alignment

**Files:**
- Create: `controllers/imu_leveling_controller.py`
- Create: `tests/test_imu_leveling.py`
- Create: `tests/test_motor_alignment.py`
- Modify: `controllers/motor_controller.py`
- Modify: `config.py`

**Interfaces:**
- Produces `AlignmentSettings.from_config() -> AlignmentSettings` with every numeric default in Global Constraints.
- Produces `AlignmentResult(final_error_deg, iterations, movement_count, cumulative_steps, positive_direction_increases_pitch, elapsed_seconds)`.
- Produces `IMULevelingController(imu, motor, settings=None, clock=time.monotonic, sleep=time.sleep).align() -> AlignmentResult`.
- Produces motor methods `begin_alignment(max_relative_steps: int)`, `move_alignment(direction: bool, steps: int, pulse_delay: float | None = None)`, `finish_alignment(positive_direction_is_true: bool | None = None)`, and `abort_alignment()`.

- [ ] **Step 1: Write a failing already-aligned test**

```python
def test_already_aligned_requires_three_stable_reads(self):
    imu = SequenceIMU([(1.00, 1.10), (1.00, 1.05), (1.00, 1.02)])
    motor = FakeAlignmentMotor(imu)
    result = controller_for(imu, motor).align()
    self.assertEqual(motor.moves, [])
    self.assertLessEqual(abs(result.final_error_deg), 0.2)
    self.assertTrue(motor.finished)
```

- [ ] **Step 2: Run the leveling test and confirm missing-module failure**

Run `python -m unittest tests.test_imu_leveling -v`.
Expected: FAIL because the controller module does not exist.

- [ ] **Step 3: Implement result, settings, errors, and stable-read loop**

Use corrected error `(upper - upper_offset) - (base - base_offset)`. Only call `finish_alignment()` after three consecutive in-tolerance results; call `abort_alignment()` on every raised alignment error.

- [ ] **Step 4: Add direction and proportional convergence tests**

Simulate both mechanical polarities. Each fake motor move adjusts only upper pitch by `steps / STEPS_PER_DEGREE` with the configured polarity. Assert both positive and negative starting errors converge and no AZ movement API exists in the fake call log.

- [ ] **Step 5: Implement probe and correction logic**

Probe 240 steps with `direction=True`, require error delta at least 0.08 degree, infer the direction sign, then compute `round(abs(error) * steps_per_degree * 0.65)` clamped to 8..960. Set direction so upper pitch moves opposite the error.

- [ ] **Step 6: Add failure tests**

Cover unobservable probe, three worsening measurements above the 0.1-degree margin, elapsed time above 60 seconds, 80 iterations, and requested travel beyond 15 degrees. Each test must assert `abort_alignment()` and no `finish_alignment()`.

- [ ] **Step 7: Implement all failure bounds and progress accounting**

Check the deadline before measurement and movement, track cumulative signed alignment steps, reset stable count after any out-of-tolerance reading, and include a stable reason code in every `IMUAlignmentError`.

- [ ] **Step 8: Write failing motor ALIGN-mode tests**

Inject a fake low-level driver into `MotorController`. Assert normal `move_manual()` is rejected in ALIGN mode, alignment movement invokes only `axis='ALT'`, the configured relative bound is enforced, and `finish_alignment()` sets `current_alt`, `target_alt`, and `target_az` to zero.

- [ ] **Step 9: Implement injectable driver and ALIGN motor mode**

Move the `MotorDriver` import inside the constructor when no driver is supplied. Add `ALIGN` to `MotorMode`; guard all modes under the existing locks; never route alignment through the unknown absolute 0..90-degree clamp.

- [ ] **Step 10: Run all IMU and motor tests**

Run `python -m unittest tests.test_imu_hardware tests.test_imu_leveling tests.test_motor_alignment -v`.
Expected: PASS.

### Task 4: Runtime Integration and Field-Rotation-Safe Capture

**Files:**
- Modify: `managers/tracking_manager.py`
- Modify: `managers/state_manager.py`
- Modify: `main.py`
- Modify: `camera/camera_manager.py`
- Modify: `communication/ble_manager.py`
- Modify: `services/tracking_logger.py`
- Create: `tests/test_capture_sequence.py`

**Interfaces:**
- Produces `TrackingManager.initialize_hardware() -> AlignmentResult | None` and `configure_observer(latitude, longitude, altitude) -> None`.
- Produces `CameraManager.capture_sequence(directory=None, frame_count=None, exposure_time=None, cancel_event=None) -> list[str]`.
- Extends `TrackingLogger.finish_session(..., final_image=None, final_images=None)` without breaking single-image callers.

- [ ] **Step 1: Write failing capture-sequence tests**

Use an uninitialized object with a stubbed `capture_long_exposure` method. Verify exactly three deterministic `.jpg` paths for three requested frames and verify a pre-set cancellation event produces zero frames.

- [ ] **Step 2: Implement cancellable capture sequence**

Create the configured directory, generate unique UTC/frame-number filenames, check cancellation before every frame, call `capture_long_exposure`, and return only successfully completed paths.

- [ ] **Step 3: Integrate startup alignment**

`initialize_hardware()` must initialize motor, set state ALIGN, initialize the IMU pair, run alignment, close IMU in `finally`, initialize camera only after alignment success, and leave state INIT until observer configuration. With required alignment failure, re-raise after motor abort so `main` reaches cleanup.

- [ ] **Step 4: Split observer configuration and reorder main startup**

Start GNSS reading first, run `initialize_hardware()`, wait for valid position/UTC, register the time provider, call `configure_observer()`, then connect BLE. Preserve the existing `finally` shutdown sequence.

- [ ] **Step 5: Add correction and range failure policy**

Track consecutive solve failures and stop the session after three. Reset the counter on success. Reject target altitude outside `MIN_ALTITUDE..MAX_ALTITUDE` before setting motor targets instead of relying on silent clamping.

- [ ] **Step 6: Integrate default sequence capture and cancellation**

In `finish_sequence()`, choose sequence mode by default, keep the tracking thread active during subframes, store all returned paths, and finish logging once. Extend BLE stoppable states to TARGET_CAPTURE, TRACKING, DRIFT_CORRECTION, and CAPTURE.

- [ ] **Step 7: Extend logger schema compatibly**

Store `alignment`, `final_image`, and `final_images` keys. Existing calls supplying only `final_image` remain valid. Record alignment result before the first target capture.

- [ ] **Step 8: Run focused and complete tests**

Run `python -m unittest discover -s tests -v`.
Expected: all tests PASS.

### Task 5: Documentation, Static Analysis, and Acceptance Package

**Files:**
- Modify: `README.md`
- Create: `HARDWARE_ACCEPTANCE_CHECKLIST.md`
- Create: `final-sha256.txt`

**Interfaces:**
- Produces exact install/run/configuration instructions and a safe Raspberry Pi commissioning order.

- [ ] **Step 1: Rewrite README runtime documentation**

Document AD0 wiring for `0x69`, I2C enablement, `i2cdetect -y 1`, `smbus2` installation, offset semantics, ALIGN state, three-read criterion, direction probe, all abort bounds, GNSS/BLE flow, capture sequence, experimental single exposure, and field rotation.

- [ ] **Step 2: Write the hardware acceptance checklist**

The checklist order is: disconnected-motor I2C detection, sensor orientation check, hand-supported low-speed probe, small positive offset convergence, small negative offset convergence, emergency shutdown, GNSS fix/UTC, BLE manual axes, initial plate solve, one drift correction, three-frame short capture, then full 18-frame run.

- [ ] **Step 3: Run syntax and placeholder scans**

Parse every Python file using `ast.parse`. Search source/docs for temporary work-marker patterns, unresolved `motorCtrl`, root-level `hardware.py`, and stale README text claiming IMU is absent. Expected: no unresolved result.

- [ ] **Step 4: Run the complete automated suite**

Run `python -m unittest discover -s tests -v`.
Expected: all tests PASS with no Raspberry Pi hardware imports required.

- [ ] **Step 5: Validate configuration invariants**

Run a Python assertion script that checks `STEP_ANGLE > 0`, `STEPS_PER_DEGREE == 1 / STEP_ANGLE`, odd filter sample count, tolerance `0.2`, stable count `3`, minimum steps <= maximum steps, and unique IMU addresses.

- [ ] **Step 6: Generate final manifest and compare scope**

Write sorted SHA-256 entries for the complete deliverable to `final-sha256.txt`. Compare source paths against the baseline and list every created/modified file in the final handoff.

- [ ] **Step 7: Record final checkpoint**

Retain exact test, parse, scan, and invariant outputs. Git commit is unavailable; the manifests provide immutable before/after evidence.
