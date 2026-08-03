"""
communication/ble_manager.py

ESP32-C3 BLE Joystick Manager

실제 동작 확인된 테스트 코드를 기준으로 구성한다.

BLE 패킷:
- bit 7   : Joystick Click
- bit 6   : Reserved
- bit 5~3 : X (-3 ~ 3)
- bit 2~0 : Y (-3 ~ 3)

수동 조작:
- X → AZ 모터
- Y → ALT 모터
- AZ / ALT 독립 스레드
- 대각선 입력 시 두 축 동시 회전 가능

클릭:
- PREVIEW  → Tracking Start
- TRACKING → Tracking Stop
"""

import asyncio
import threading
import time
from typing import Optional

from bleak import BleakClient, BleakScanner

from managers.state_manager import SystemState


# =========================================================
# BLE 설정
# =========================================================

DEVICE_NAME = "ESP32-C3-SuperMini-Controller"

CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"


# =========================================================
# BLE 패킷 설정
# =========================================================

# 클릭 신호를 bit 7에 넣는 기준
BUTTON_MASK = 0x80


# =========================================================
# 조이스틱 속도
#
# 실제 테스트에서 작동 확인된 값
# 값이 작을수록 빠름
# =========================================================

STEP_DELAY = {
    0: None,
    1: 0.003,
    2: 0.0015,
    3: 0.0007,
}


class BLEManager:
    """BLE 조이스틱 입력과 수동 모터 이동을 관리한다."""

    def __init__(self, tracking):

        self.tracking = tracking

        self.client: Optional[BleakClient] = None

        # 현재 조이스틱 값
        self.joy_x = 0
        self.joy_y = 0

        # 현재 버튼 상태
        self.button_pressed = False
        self.previous_button = False

        # CODE_3: 첫 Plate Solving 시작 작업의 중복 생성 방지
        self._tracking_starting = False

        # 실행 상태
        self.running = False

        # 입력값 보호
        self._input_lock = threading.Lock()

        # 종료 신호
        self._stop_event = threading.Event()

        # 버튼 채터링 방지
        self._last_click_time = 0.0
        self._button_debounce_seconds = 0.25

        # 독립 스레드
        self._az_thread: Optional[threading.Thread] = None
        self._alt_thread: Optional[threading.Thread] = None
        self._button_thread: Optional[threading.Thread] = None

    # =====================================================
    # Signed 3-bit
    # =====================================================

    @staticmethod
    def signed3bit(value: int) -> int:
        """
        3비트 2의 보수 값을 정수로 변환한다.

        000 → 0
        001 → 1
        010 → 2
        011 → 3
        100 → -4
        101 → -3
        110 → -2
        111 → -1
        """

        return value - 8 if value & 0x04 else value

    # =====================================================
    # BLE Notify
    # =====================================================

    def handle_notify(self, sender, data) -> None:
        """
        BLE Callback에서는 모터를 움직이지 않는다.

        BLE 데이터 해석 후 현재 조이스틱 상태만 갱신한다.
        """

        if not data:
            return

        value = int(data[0])

        # 실제 테스트에서 확인된 비트 구조
        x_raw = self.signed3bit(
            (value >> 3) & 0x07
        )

        y_raw = self.signed3bit(
            value & 0x07
        )

        # 3비트에서 나올 수 있는 -4는 사용하지 않음
        x_raw = max(-3, min(3, x_raw))
        y_raw = max(-3, min(3, y_raw))

        # 클릭 신호: bit 7
        button = bool(value & BUTTON_MASK)

        with self._input_lock:

            self.joy_x = x_raw
            self.joy_y = y_raw
            self.button_pressed = button

    # =====================================================
    # AZ Independent Thread
    # =====================================================

    def _az_motor_loop(self) -> None:
        """X 입력을 이용해 AZ축을 연속 제어한다."""

        while not self._stop_event.is_set():

            with self._input_lock:
                x = self.joy_x

            # 수동 제어는 PREVIEW 상태에서만 허용
            if (
                x != 0
                and
                self.tracking.state.is_state(
                    SystemState.PREVIEW
                )
            ):

                direction = x > 0
                pulse_delay = STEP_DELAY[abs(x)]

                try:

                    self.tracking.preview_move(
                        axis="AZ",
                        direction=direction,
                        steps=1,
                        pulse_delay=pulse_delay,
                    )

                except Exception as error:

                    print(
                        f"[BLE] AZ Motor Warning: {error}"
                    )

                    time.sleep(0.01)

            else:

                time.sleep(0.001)

        print("[BLE] AZ Thread Ended")

    # =====================================================
    # ALT Independent Thread
    # =====================================================

    def _alt_motor_loop(self) -> None:
        """Y 입력을 이용해 ALT축을 연속 제어한다."""

        while not self._stop_event.is_set():

            with self._input_lock:
                y = self.joy_y

            if (
                y != 0
                and
                self.tracking.state.is_state(
                    SystemState.PREVIEW
                )
            ):

                direction = y > 0
                pulse_delay = STEP_DELAY[abs(y)]

                try:

                    self.tracking.preview_move(
                        axis="ALT",
                        direction=direction,
                        steps=1,
                        pulse_delay=pulse_delay,
                    )

                except Exception as error:

                    print(
                        f"[BLE] ALT Motor Warning: {error}"
                    )

                    time.sleep(0.01)

            else:

                time.sleep(0.001)

        print("[BLE] ALT Thread Ended")

    # =====================================================
    # Button Thread
    # =====================================================

    def _button_loop(self) -> None:
        """조이스틱 클릭의 상승 에지만 인식한다."""

        while not self._stop_event.is_set():

            with self._input_lock:
                button = self.button_pressed

            # False → True가 되는 순간만 클릭으로 처리
            rising_edge = (
                button
                and not self.previous_button
            )

            if rising_edge:

                current_time = time.monotonic()

                if (
                    current_time
                    - self._last_click_time
                    >= self._button_debounce_seconds
                ):

                    self._last_click_time = current_time
                    self._handle_click()

            self.previous_button = button

            time.sleep(0.01)

        print("[BLE] Button Thread Ended")

    # =====================================================
    # Click Action
    # =====================================================

    def _handle_click(self) -> None:
        """현재 시스템 상태에 따라 추적을 시작하거나 중지한다."""

        current_state = self.tracking.state.get_state()

        if current_state == SystemState.PREVIEW:

            # CODE_3의 BLE 변경사항: Plate Solving 시작 스레드가 이미
            # 실행 중이면 같은 버튼 입력으로 중복 생성하지 않는다.
            if self._tracking_starting:
                print("[BLE] Tracking Start Already Running")
                return

            self._tracking_starting = True

            print("[BLE] Joystick Click → Tracking Start")

            # Plate Solving이 오래 걸릴 수 있으므로
            # 버튼 스레드를 막지 않고 별도 스레드에서 실행
            start_thread = threading.Thread(
                target=self._start_tracking_worker,
                name="BLETrackingStart",
                daemon=True,
            )

            start_thread.start()

        elif current_state in (
            SystemState.TARGET_CAPTURE,
            SystemState.TRACKING,
            SystemState.DRIFT_CORRECTION,
            SystemState.CAPTURE,
        ):

            print("[BLE] Joystick Click → Tracking Stop")

            self.tracking.stop_tracking()

        else:

            print(
                "[BLE] Click Ignored: "
                f"Current State = {current_state.name}"
            )

    def _start_tracking_worker(self) -> None:

        try:

            success = self.tracking.start_tracking()

            if not success:
                print("[BLE] Tracking Start Failed")

        except Exception as error:

            print(
                f"[BLE] Tracking Start Error: {error}"
            )

        finally:
            # 실패하거나 예외가 발생해도 다음 시작 입력을 받을 수 있다.
            self._tracking_starting = False

    # =====================================================
    # Start Manual Threads
    # =====================================================

    def _start_control_threads(self) -> None:

        self._az_thread = threading.Thread(
            target=self._az_motor_loop,
            name="BLE_AZ_Control",
            daemon=True,
        )

        self._alt_thread = threading.Thread(
            target=self._alt_motor_loop,
            name="BLE_ALT_Control",
            daemon=True,
        )

        self._button_thread = threading.Thread(
            target=self._button_loop,
            name="BLE_Button_Control",
            daemon=True,
        )

        self._az_thread.start()
        self._alt_thread.start()
        self._button_thread.start()

    # =====================================================
    # Connect
    # =====================================================

    def _handle_unexpected_disconnect(self, client) -> None:
        """예기치 않은 BLE 연결 해제 시 모터와 제어 스레드를 멈춘다."""

        if not self.running:
            return

        print("[BLE] Unexpected disconnection detected.")

        self.running = False
        self._stop_event.set()

        with self._input_lock:
            self.joy_x = 0
            self.joy_y = 0
            self.button_pressed = False

        try:
            self.tracking.stop_tracking(
                reason="ble_disconnected"
            )
        except Exception as error:
            print(f"[BLE] Fail-safe stop warning: {error}")

    async def connect(self) -> bool:

        if self.running:
            print("[BLE] Already Connected")
            return True

        print("[BLE] Scanning...")

        devices = await BleakScanner.discover(
            timeout=5.0
        )

        target = None

        for device in devices:

            if device.name == DEVICE_NAME:

                target = device
                break

        if target is None:

            print("[BLE] Device Not Found")
            return False

        print(
            f"[BLE] Connecting: {target.address}"
        )

        try:

            self.client = BleakClient(
                target.address,
                disconnected_callback=(
                    self._handle_unexpected_disconnect
                ),
            )

            await self.client.connect()

            await self.client.start_notify(
                CHAR_UUID,
                self.handle_notify,
            )

        except Exception as error:

            print(
                f"[BLE] Connection Failed: {error}"
            )

            self.client = None
            return False

        self._stop_event.clear()

        with self._input_lock:

            self.joy_x = 0
            self.joy_y = 0
            self.button_pressed = False
            self.previous_button = False

        self.running = True

        self._start_control_threads()

        print("[BLE] Connected")
        print("[BLE] Joystick Input Ready")

        return True

    # =====================================================
    # Disconnect
    # =====================================================

    async def disconnect(self) -> None:

        if not self.running and self.client is None:
            return

        print("[BLE] Disconnecting...")

        self.running = False
        self._stop_event.set()

        with self._input_lock:

            self.joy_x = 0
            self.joy_y = 0
            self.button_pressed = False

        threads = [
            self._az_thread,
            self._alt_thread,
            self._button_thread,
        ]

        for thread in threads:

            if (
                thread is not None
                and thread.is_alive()
                and thread is not threading.current_thread()
            ):

                thread.join(timeout=2.0)

        if self.client is not None:

            try:

                if self.client.is_connected:

                    try:

                        await self.client.stop_notify(
                            CHAR_UUID
                        )

                    except Exception:
                        pass

                    await self.client.disconnect()

            except Exception as error:

                print(
                    f"[BLE] Disconnect Warning: {error}"
                )

            finally:

                self.client = None

        print("[BLE] Disconnected")

    # =====================================================
    # Standalone Run
    # =====================================================

    async def run(self) -> None:

        connected = await self.connect()

        if not connected:
            return

        try:

            while self.running:

                await asyncio.sleep(0.1)

        finally:

            await self.disconnect()
