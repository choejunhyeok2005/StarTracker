import asyncio
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from controllers.session_controller import SessionController
from managers.state_manager import StateManager, SystemState


class SessionTests(unittest.IsolatedAsyncioTestCase):
    async def test_imu_fault_keeps_motor_and_manual_control_stopped(self):
        session, tracking, ble = self.make_session()
        tracking.motor.imu_feedback = SimpleNamespace(fault="IMU read failed")
        ble.on_start()
        await session.run_pending()
        tracking.motor.resume.assert_not_called()
        tracking.camera.start_streaming.assert_not_called()
        self.assertFalse(tracking.manual_enabled)
        self.assertFalse(session.busy)

    def make_session(self):
        tracking = SimpleNamespace(
            camera=SimpleNamespace(stop_streaming=Mock(), start_streaming=Mock()),
            motor=SimpleNamespace(stop=Mock(), resume=Mock()), state=StateManager(),
            manual_enabled=True, tracking_thread=None, correction_thread=None,
            start_tracking=Mock(return_value=False), stop_tracking=Mock(),
            logger=SimpleNamespace(finish_session=Mock()),
            _safe_log=lambda operation, **kw: operation(**kw),
            _current_time_iso=lambda: "2026-09-15T00:00:00Z",
        )
        ble = SimpleNamespace(running=True, _input_lock=threading.Lock(), joy_x=2, joy_y=1)
        return SessionController(tracking, ble), tracking, ble

    async def test_request_runs_solver_path_and_restores_manual(self):
        session, tracking, ble = self.make_session()
        ble.on_start()
        ble.on_start()
        self.assertFalse(tracking.manual_enabled)
        await session.run_pending()
        tracking.start_tracking.assert_called_once()
        tracking.camera.stop_streaming.assert_called_once()
        tracking.camera.start_streaming.assert_called_once()
        tracking.motor.resume.assert_called_once()
        self.assertEqual(tracking.state.get_state(), SystemState.MANUAL)
        self.assertTrue(tracking.manual_enabled)
        self.assertEqual((ble.joy_x, ble.joy_y), (0, 0))

    async def test_busy_session_ignores_new_request_until_workers_exit(self):
        session, tracking, ble = self.make_session()
        release = threading.Event()
        tracking.correction_thread = threading.Thread(target=release.wait)
        tracking.correction_thread.start()
        ble.on_start()
        task = asyncio.create_task(session.run_pending())
        try:
            await asyncio.sleep(0.05)
            ble.on_start()
            self.assertFalse(session.requested)
            self.assertFalse(tracking.manual_enabled)
            tracking.camera.start_streaming.assert_not_called()
        finally:
            release.set()
            await task
        self.assertTrue(tracking.manual_enabled)

    async def test_disconnect_does_not_capture_or_restart_preview(self):
        session, tracking, ble = self.make_session()
        ble.on_start()
        ble.running = False
        await session.run_pending()
        tracking.start_tracking.assert_not_called()
        tracking.camera.start_streaming.assert_not_called()
        tracking.motor.resume.assert_not_called()

    async def test_start_exception_returns_to_manual(self):
        session, tracking, ble = self.make_session()
        tracking.start_tracking.side_effect = RuntimeError("solver error")
        ble.on_start()
        await session.run_pending()
        tracking.stop_tracking.assert_called_once_with(reason="session_error")
        self.assertTrue(tracking.manual_enabled)

    async def test_completed_capture_exits_without_restoring_manual(self):
        session, tracking, ble = self.make_session()
        tracking.capture_completed = True
        ble.on_start()
        await session.run_pending()
        tracking.camera.start_streaming.assert_not_called()
        tracking.motor.resume.assert_not_called()
        self.assertFalse(tracking.manual_enabled)

    async def test_cancellation_waits_for_startup_before_returning(self):
        session, tracking, ble = self.make_session()
        entered, release = threading.Event(), threading.Event()
        def start():
            entered.set()
            release.wait(2)
        tracking.start_tracking.side_effect = start
        tracking.stop_tracking.side_effect = lambda **kw: release.set()
        ble.on_start()
        task = asyncio.create_task(session.run_pending())
        await asyncio.to_thread(entered.wait, 1)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        tracking.camera.start_streaming.assert_not_called()
        self.assertTrue(release.is_set())


if __name__ == "__main__":
    unittest.main()
