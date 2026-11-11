"""Encoder boundary: output-axis degrees, monotonic acquisition timestamp.

Drivers for SPI/I2C encoders can implement the same read()/close() interface.
Incremental counters require a reference after every power cycle.
"""
from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class EncoderSample:
    alt: float
    az: float
    timestamp: float


class QuadratureEncoder:
    # Legal Gray-code transitions; each edge is one count (x4 decoding).
    TRANSITIONS = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
                   (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}

    def __init__(self, axes, gpio=None, clock=time.monotonic):
        # axes = {"ALT": {"pins": (a,b), "counts_per_rev": ..., "sign": 1}, ...}
        self.axes = axes
        pins = []
        for axis in ("ALT", "AZ"):
            spec = axes[axis]
            if (len(spec["pins"]) != 2 or any(type(p) is not int or p < 0 or p > 27
                                            for p in spec["pins"])):
                raise ValueError("Configure two BCM encoder pins per axis first.")
            cpr = spec["counts_per_rev"]
            if not isinstance(cpr, (int, float)) or not math.isfinite(cpr) or cpr <= 0:
                raise ValueError("counts_per_rev must be positive x4 counts/output revolution.")
            if spec["sign"] not in (-1, 1):
                raise ValueError("Encoder sign must be -1 or 1.")
            pins.extend(spec["pins"])
        if len(set(pins)) != 4:
            raise ValueError("Encoder pins must be distinct.")
        if gpio is None:
            import RPi.GPIO as gpio
        self.gpio, self.clock = gpio, clock
        self.lock = threading.Lock()
        self.counts = {"ALT": 0, "AZ": 0}
        self.states = {}
        self.fault = None
        self.closed = False
        self.registered = []
        gpio.setmode(gpio.BCM)
        try:
            for axis, spec in axes.items():
                for pin in spec["pins"]:
                    gpio.setup(pin, gpio.IN)  # Electrical pull-ups depend on the encoder.
                self.states[axis] = self._state(axis)
            for axis, spec in axes.items():
                for pin in spec["pins"]:
                    gpio.add_event_detect(pin, gpio.BOTH,
                                          callback=lambda channel, a=axis: self._edge(a))
                    self.registered.append(pin)
        except Exception:
            self.close()
            raise

    def _state(self, axis):
        a, b = self.axes[axis]["pins"]
        return (int(self.gpio.input(a)) << 1) | int(self.gpio.input(b))

    def _edge(self, axis):
        with self.lock:
            try:
                old, new = self.states[axis], self._state(axis)
                if old != new and (old, new) not in self.TRANSITIONS:
                    self.fault = f"{axis} invalid quadrature transition; reference lost"
                self.counts[axis] += self.TRANSITIONS.get((old, new), 0)
                self.states[axis] = new
            except Exception as error:
                self.fault = str(error)

    def read(self):
        with self.lock:
            if self.closed or self.fault:
                raise RuntimeError(self.fault or "Encoder closed")
            angles = {a: self.counts[a] * 360.0 * self.axes[a]["sign"] /
                      self.axes[a]["counts_per_rev"] for a in ("ALT", "AZ")}
            return EncoderSample(angles["ALT"], angles["AZ"], self.clock())

    def close(self):
        self.closed = True
        for pin in self.registered:
            self.gpio.remove_event_detect(pin)
        self.registered.clear()


def configured_encoder():
    import config
    axes = config.ENCODER_AXES
    motor_pins = {config.MOTOR_BASE_STEP_PIN, config.MOTOR_BASE_DIR_PIN,
                  config.MOTOR_MOUNT_STEP_PIN, config.MOTOR_MOUNT_DIR_PIN,
                  config.MOTOR_ENABLE_PIN, config.MOTOR_MS1_PIN, config.MOTOR_MS2_PIN,
                  config.COMPASS_EN_PIN, 2, 3, 14, 15}
    if any(p is not None and p in motor_pins for s in axes.values() for p in s["pins"]):
        raise ValueError("Encoder GPIO conflicts with motor/I2C/UART GPIO.")
    return QuadratureEncoder(axes)
