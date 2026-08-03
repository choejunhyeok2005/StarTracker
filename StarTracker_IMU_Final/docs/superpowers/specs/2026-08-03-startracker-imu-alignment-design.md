# StarTracker IMU Alignment and Final Control Design

Date: 2026-08-03

## 1. Goal

Integrate two MPU6050 sensors into the existing Raspberry Pi ALT/AZ star-tracker so that startup automatically makes the moving upper plate parallel to the fixed base. Only the upper ALT motor may move during alignment.

Alignment succeeds when the corrected pitch difference is within `+/-0.2 deg` for three consecutive filtered measurements. After success, the aligned ALT position becomes the software `0 deg` reference. Any alignment failure stops startup before camera, BLE, or astronomical tracking begins.

The existing GNSS, BLE joystick, offline plate solving, ALT/AZ tracking, drift correction, logging, and safe shutdown behavior remain part of the final system.

## 2. Physical Assumptions and Limits

- Base MPU6050: fixed to the non-moving base, I2C address `0x68`.
- Upper MPU6050: fixed to the ALT-moving upper plate, I2C address `0x69`; its AD0 pin must be HIGH.
- Both sensors use matching axis orientation. Small mounting bias is represented by per-sensor pitch offset settings.
- The upper plate starts away from mechanical end stops. The current hardware has no limit switch, so software cannot prove that a physical end stop is clear.
- Initial alignment moves only ALT. AZ never moves during alignment.
- Leveling establishes a relative zero between base and upper plate; it does not find geographic north or the celestial pole.
- ALT/AZ tracking keeps the target centered but does not remove field rotation. Therefore the default final capture is a sequence of short subframes. A single 180-second exposure remains an explicit experimental mode, not the default guaranteed output.

## 3. Selected Approach

Use a filtered adaptive closed loop rather than a raw proportional loop or full accelerometer/gyroscope fusion.

Each pitch measurement is the median of 11 accelerometer-derived pitch samples. The controller performs one small positive-direction probe to discover the actual mechanical direction. It then applies proportional corrections whose step count is clamped to safe minimum and maximum values. Filtering, repeat-success gating, direction discovery, bounded travel, progress checks, timeouts, and fail-fast behavior make the result deterministic enough for startup alignment without adding unnecessary gyro drift handling.

## 4. Components

### `hardware/imu.py`

`MPU6050Pair` owns the I2C bus and sensor register access.

- Opens SMBus lazily during `initialize()` so importing the module on a development PC is safe.
- Validates both sensors using `WHO_AM_I`.
- Wakes each sensor and configures the accelerometer to +/-2 g and the low-pass filter.
- Reads each XYZ acceleration sample as one six-byte block to avoid mixing bytes from different sensor updates.
- Converts signed 16-bit values and calculates pitch using `atan2(-x, sqrt(y^2 + z^2))`.
- Returns median-filtered base and upper pitch values.
- Applies no motor logic.
- Closes the SMBus idempotently.

### `controllers/imu_leveling_controller.py`

`IMULevelingController` owns the alignment state machine. It depends only on an object that supplies filtered pitch pairs and a motor object that supplies bounded ALT alignment movement. This boundary permits hardware-free tests.

The controller returns an `AlignmentResult` containing final error, iteration count, movement count, cumulative signed steps, discovered direction, and elapsed time. Expected alignment failures raise `IMUAlignmentError` with a stable reason string.

### `controllers/motor_controller.py`

Add `MotorMode.ALIGN` and three explicit operations:

- `begin_alignment()`: prevents normal tracking/manual movement and resets relative alignment travel.
- `move_alignment(direction, steps)`: moves only ALT, bypasses the unknown absolute 0-90 degree estimate, and enforces a separate maximum relative travel bound.
- `finish_alignment()`: commits the current physical position as ALT `0 deg`, clears targets, and returns to STOP.

`abort_alignment()` returns the controller to STOP without declaring a valid ALT zero. The main program exits after such a failure.

### `managers/tracking_manager.py`

Split hardware startup from observer configuration:

- `initialize_hardware()`: initialize motor, enter `ALIGN`, initialize IMUs, align the upper plate, close the IMU bus after the one-time alignment attempt, then initialize the camera only after success.
- `configure_observer(latitude, longitude, altitude)`: configure astronomy and transition to MANUAL only after a valid GNSS fix.

The manager owns IMU cleanup and includes alignment results in logs/console output.

### `main.py`

Startup order becomes:

1. Start the GNSS reader thread.
2. Initialize motor and run IMU alignment while GNSS data accumulates.
3. Initialize the camera after alignment succeeds.
4. Wait for valid GNSS position and UTC.
5. Configure the astronomical observer and GNSS time provider.
6. Connect BLE and enter MANUAL operation.

The existing `finally` path always disconnects BLE, shuts down tracking hardware, and closes GNSS resources.

### `config.py`

The final defaults are:

| Setting | Default | Purpose |
| --- | ---: | --- |
| `IMU_ENABLED` | `True` | Run startup alignment |
| `IMU_REQUIRED` | `True` | Abort startup on alignment failure |
| `IMU_I2C_BUS` | `1` | Raspberry Pi I2C bus |
| `IMU_BASE_ADDRESS` | `0x68` | Fixed base sensor |
| `IMU_UPPER_ADDRESS` | `0x69` | Moving upper sensor |
| `IMU_BASE_PITCH_OFFSET_DEG` | `0.0` | Measured base mounting bias |
| `IMU_UPPER_PITCH_OFFSET_DEG` | `0.0` | Measured upper mounting bias |
| `IMU_FILTER_SAMPLES` | `11` | Median filter sample count |
| `IMU_SAMPLE_INTERVAL_SECONDS` | `0.01` | Delay between samples |
| `IMU_LEVEL_TOLERANCE_DEG` | `0.2` | Accepted absolute pitch error |
| `IMU_STABLE_READS_REQUIRED` | `3` | Consecutive successful measurements |
| `IMU_DIRECTION_PROBE_STEPS` | `240` | Approximately 0.25 degree probe |
| `IMU_MIN_PROBE_RESPONSE_DEG` | `0.08` | Minimum observable probe response |
| `IMU_PROPORTIONAL_GAIN` | `0.65` | Fraction of estimated error corrected per cycle |
| `IMU_MIN_CORRECTION_STEPS` | `8` | Avoid a zero-step stall near tolerance |
| `IMU_MAX_CORRECTION_STEPS` | `960` | At most approximately 1 degree per cycle |
| `IMU_SETTLE_SECONDS` | `0.25` | Mechanical settling time after movement |
| `IMU_ALIGNMENT_TIMEOUT_SECONDS` | `60.0` | Absolute alignment deadline |
| `IMU_MAX_ALIGNMENT_ITERATIONS` | `80` | Secondary loop bound |
| `IMU_MAX_RELATIVE_TRAVEL_DEG` | `15.0` | Maximum net travel from startup position |
| `IMU_MAX_WORSENING_COUNT` | `3` | Abort after repeated error growth |
| `IMU_WORSENING_MARGIN_DEG` | `0.1` | Ignore smaller noise-level changes |
| `FINAL_CAPTURE_MODE` | `"sequence"` | Field-rotation-safe default |
| `FINAL_SUBEXPOSURE_SECONDS` | `10.0` | Duration of each default subframe |
| `FINAL_FRAME_COUNT` | `18` | Approximately 180 seconds total integration |

## 5. Alignment Algorithm

Let:

```text
base  = measured_base_pitch  - base_pitch_offset
upper = measured_upper_pitch - upper_pitch_offset
error = upper - base
```

The fixed base is measurement-only. A positive or negative correction always commands the upper ALT motor.

When `IMU_ENABLED` is `True`, `IMU_REQUIRED=True` is the supported production configuration. Setting it to `False` is a bench-only override: alignment failure is printed prominently and the operator must establish ALT zero manually before any motion. The delivered default never continues from a failed automatic alignment.

1. Initialize and validate both IMUs.
2. Enter motor ALIGN mode and take a filtered baseline measurement.
3. If `abs(error) <= 0.2`, increment the stable counter, wait, and measure again. Do not move.
4. If alignment is needed and motor direction is unknown, move ALT `direction=True` by 240 steps, settle, and measure again.
5. Let `probe_delta = new_error - old_error`. If `abs(probe_delta) < 0.08`, abort because the sensor/motor response is not observable. Otherwise its sign defines whether `direction=True` increases or decreases upper pitch.
   Persist this discovered ALT direction mapping for subsequent manual control and tracking commands.
6. For each correction, compute:

```text
requested_steps = round(abs(error) * STEPS_PER_DEGREE * 0.65)
steps = clamp(requested_steps, 8, 960)
```

7. Select the direction that moves upper pitch opposite the sign of `error`.
8. Refuse a movement that would exceed the configured relative travel bound.
9. Move, settle, filter a new pitch pair, and update progress counters.
10. Reset the stable counter whenever error leaves tolerance.
11. Abort if the error grows by more than 0.1 degree for three correction cycles, the 60-second deadline expires, 80 iterations are reached, I2C fails, or travel is exhausted.
12. When three consecutive filtered errors are within +/-0.2 degree, call `finish_alignment(positive_direction_is_true=...)`, preserve the discovered electrical direction mapping, and define the physical position as ALT `0 deg`.

If the initial error is already within tolerance for all three reads, no probe is performed. In that no-movement case, the configured `MOTOR_ALT_POSITIVE_DIRECTION` remains authoritative.

## 6. Full Runtime State Machine

```text
INIT
  -> ALIGN
  -> MANUAL
  -> TARGET_CAPTURE
  -> TRACKING
  <-> DRIFT_CORRECTION
  -> CAPTURE
  -> MANUAL
```

- `INIT`: objects exist, GNSS reader starts, but no user motion is accepted.
- `ALIGN`: only bounded IMU-driven ALT movement is accepted.
- `MANUAL`: BLE joystick controls ALT/AZ framing.
- `TARGET_CAPTURE`: manual motion is blocked while the first plate image is solved.
- `TRACKING`: target RA/Dec is fixed; current GNSS UTC is converted to target Alt/Az at 20 Hz and the motor moves in bounded increments.
- `DRIFT_CORRECTION`: tracking continues while a fresh image is plate-solved; solved camera Alt/Az replaces the software position estimate and the tracking loop removes the measured drift.
- `CAPTURE`: tracking continues while short subframes are captured. A BLE click requests cancellation; the current subframe completes and remaining frames are skipped.
- After successful capture or user stop, return to MANUAL. Hardware or startup failures go to shutdown rather than MANUAL.

Three consecutive failed drift-correction solves abort the active session instead of retrying forever. Target altitude leaving the configured safe range also ends tracking rather than silently clamping the commanded angle.

## 7. Final Capture Policy

The default capture saves 18 sequential 10-second images while motor tracking remains active. Filenames include a frame number and UTC-compatible timestamp. The logger records every output path. These subframes are intended for later alignment, field derotation, and stacking outside this control program.

`FINAL_CAPTURE_MODE = "single"` may still request the existing long exposure for experiments, but a two-axis ALT/AZ mount cannot guarantee sharp field edges during a 180-second exposure.

## 8. Error Handling and Cleanup

- I2C, WHO_AM_I, invalid sample, non-finite angle, unobservable probe, non-convergence, worsening error, timeout, and travel-limit failures use explicit alignment exceptions.
- Alignment failure calls `abort_alignment()`, stops motor commands, and prevents camera/BLE startup.
- Shutdown is idempotent even when only some components initialized.
- BLE disconnection stops tracking and causes the main loop to enter global cleanup.
- Logging failure remains isolated from motor control.
- Capture and correction loops check cancellation events between blocking hardware operations.

## 9. Verification

Automated `unittest` coverage uses fake IMU and fake motor objects for:

- already-aligned startup with no movement;
- positive and negative initial error;
- automatic discovery of either wiring direction;
- noisy measurements that require median filtering and three stable reads;
- probe response too small;
- repeated worsening error;
- timeout/iteration/travel failures;
- exact ALT zero commit on success;
- no AZ command during alignment;
- signed accelerometer conversion and pitch math.

Static parsing and import-oriented tests run on the development PC without GPIO/I2C. Raspberry Pi acceptance testing then verifies both addresses, direction probing at low speed, convergence from small known offsets, emergency shutdown, GNSS acquisition, BLE manual control, plate solving, correction, and subframe capture.

## 10. Deliverable and Compatibility

The original Downloads folder remains unchanged. The completed project is delivered as `outputs/StarTracker_IMU_Final` with source, tests, this design, configuration documentation, Raspberry Pi installation instructions, and a hardware acceptance checklist.

The original project is not a Git repository, so this design cannot be committed there. The written specification is retained inside the delivered project instead.
