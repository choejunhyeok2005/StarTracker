"""Keep the BLE event loop alive while a complete tracking session runs."""
import asyncio
import config


class SessionController:
    def __init__(self, tracking, ble):
        self.tracking = tracking
        self.ble = ble
        self.busy = False
        ble.on_start = self.request_start
        self.requested = False

    def request_start(self):
        if not self.busy and not self.requested:
            self.requested = True
            self.tracking.manual_enabled = False

    async def run_pending(self):
        if not self.requested or self.busy:
            return
        self.requested = False
        self.busy = True
        tracking = self.tracking
        starting = None
        closing = False
        try:
            await asyncio.to_thread(tracking.camera.stop_streaming)
            if self.ble.running:
                starting = asyncio.create_task(asyncio.to_thread(tracking.start_tracking))
                await asyncio.shield(starting)
            while any(t is not None and t.is_alive() for t in
                      (tracking.tracking_thread, tracking.correction_thread)):
                if not self.ble.running:
                    tracking.stop_tracking(reason="ble_disconnected")
                await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            closing = True
            tracking.stop_tracking(reason="shutdown")
            if starting is not None:
                try:
                    await starting
                except Exception as error:
                    print(f"[Session] Startup ended during shutdown: {error}")
            tracking.stop_tracking(reason="shutdown")
            for worker in (tracking.tracking_thread, tracking.correction_thread):
                if worker is not None and worker.ident is not None:
                    await asyncio.to_thread(worker.join)
            raise
        except Exception as error:
            print(f"[Session] Failed: {error}")
            tracking.stop_tracking(reason="session_error")
            for worker in (tracking.tracking_thread, tracking.correction_thread):
                if worker is not None and worker.ident is not None:
                    await asyncio.to_thread(worker.join)
        finally:
            tracking.motor.stop()
            tracking._safe_log(tracking.logger.finish_session,
                               status=getattr(tracking, "stop_reason", "session_ended"),
                               ended_at_utc=tracking._current_time_iso())
            control_fault = any(getattr(getattr(tracking.motor, name, None), "fault", None)
                                for name in ("imu_feedback", "feedback"))
            if control_fault:
                tracking.manual_enabled = False
                print("[Session] Sensor fault latched; motor remains stopped until restart.")
            if (self.ble.running and not closing and not control_fault
                    and not (getattr(tracking, "capture_completed", False)
                             and config.EXIT_AFTER_CAPTURE)):
                await asyncio.to_thread(tracking.camera.start_streaming)
                with self.ble._input_lock:
                    self.ble.joy_x = self.ble.joy_y = 0
                tracking.motor.resume()
                from managers.state_manager import SystemState
                tracking.state.set_state(SystemState.MANUAL)
                tracking.manual_enabled = True
            self.busy = False
