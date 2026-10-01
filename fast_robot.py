"""One Pi command owner; bounded logging and status consumers stay off its path."""
from __future__ import annotations

import csv
import json
import logging
import queue
import sys
import threading
import time
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from crsf import encode_flight_mode, encode_rpm_telemetry, encode_temperature_telemetry
from fast_hat import FastConfig, FastHat, FastHatError, FastStatus, REQUIRED_CAPABILITIES
from protocol_defs import ConfigResult, HatState, HoldFlag, Profile, Reason, StopState, FaultCode
from robot_main import ArmingGate, CRSFReader, DriveRequest, Settings, channel, drive_request, radio_healthy, wheel_targets
from robot_radio import ProfileSelector

LOG = logging.getLogger("robot.fast")
STATIONARY_RPM = 2
TELEMETRY_QUEUE_ROWS = 32


from robot_config import hat_configuration as fast_config

def _telemetry_header() -> list[str]:
    fields = ["wall_time", "monotonic_s", "radio_age_ms", "link_quality", "armed", "throttle", "steering",
              "requested_profile", "applied_profile", "profile_valid", "hat_state", "hat_fault", "fault_wheel",
              "stop_state", "hold_flags", "reason_flags", "boost_remaining_ms", "boost_capacity_ms",
              "boost_refill_remaining_ms", "cooldown_remaining_ms", "hat_sweep_us", "hat_command_age_ms",
              "hat_status_age_ms", "hat_ack_seq", "hat_applied_seq", "pi_command_seq", "missed_pi_deadlines",
              "hat_last_rtt_ms", "hat_max_rtt_ms", "hat_status_interval_ms", "pi_control_interval_ms",
              "telemetry_queue_depth", "telemetry_dropped_rows", "telemetry_storage_error", "radio_tx_error",
              "boot_id", "host_session", "firmware_build_id", "hat_config_id"]
    for name in ("requested_rpm", "target_rpm", "actual_rpm", "effective_cap_ma", "hold_cap_ma", "reported_current_ma",
                 "temperature_c", "motor_error", "motor_feedback_age_ms", "temperature_age_ms", "position_raw",
                 "validity", "wheel_reason_flags"):
        fields.extend(f"{name}_{mid}" for mid in range(1, 5))
    return fields


def unique_session_path(path: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return path.with_name(f"{path.stem}-{stamp}-{uuid.uuid4().hex[:8]}{path.suffix or '.csv'}")


class AsyncCsvTelemetry:
    """Drop newest rows on overflow; disk problems are visible and nonfatal.

    Metadata, events and CSV share one bounded queue. No disk operation or
    diagnostic logger is called by the command loop. A stalled worker is a
    daemon; shutdown spends at most 100 ms waiting for ordinary log work.
    """
    def __init__(self, path: Path, *, max_rows: int = TELEMETRY_QUEUE_ROWS,
                 metadata: dict[str, Any] | None = None):
        self.path = unique_session_path(path)
        self._metadata = metadata or {}
        self._rows: queue.Queue[Any] = queue.Queue(max_rows)
        self._ready = threading.Event()
        self._done = threading.Event()
        self._stop = threading.Event()
        self._error: Exception | None = None
        self._thread: threading.Thread | None = None
        self._closed = False
        self.dropped_rows = 0
        self.dropped_events = 0

    @property
    def storage_error(self) -> str | None:
        return None if self._error is None else f"{type(self._error).__name__}: {self._error}"

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Telemetry writer is already started")
        self._thread = threading.Thread(target=self._run, name="robot-telemetry", daemon=True)
        self._thread.start()
        self._ready.wait(0.1)

    def _run(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("x", newline="", encoding="utf-8", buffering=65536) as file, \
                    self.path.with_suffix(".events.jsonl").open("x", encoding="utf-8") as events:
                self.path.with_suffix(".session.json").write_text(json.dumps(self._metadata, sort_keys=True, indent=2) + "\n")
                writer = csv.writer(file)
                writer.writerow(_telemetry_header())
                last_flush = time.monotonic()
                self._ready.set()
                while not self._stop.is_set() or not self._rows.empty():
                    try:
                        kind, data = self._rows.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if kind == "row":
                        writer.writerow(data)
                    else:
                        events.write(json.dumps(data, sort_keys=True) + "\n")
                    now = time.monotonic()
                    if now - last_flush >= 1:
                        file.flush(); events.flush()
                        last_flush = now
                events.write(json.dumps({"event": "logging_closed", "dropped_rows": self.dropped_rows,
                                         "dropped_events": self.dropped_events}) + "\n")
        except Exception as exc:
            self._error = exc
        finally:
            self._ready.set()
            self._done.set()

    def check(self) -> str | None:
        return self.storage_error

    def _submit(self, kind: str, data: Any) -> bool:
        if self._closed or self._error is not None:
            if kind == "row": self.dropped_rows += 1
            else: self.dropped_events += 1
            return False
        try:
            self._rows.put_nowait((kind, data))
            return True
        except queue.Full:
            if kind == "row": self.dropped_rows += 1
            else: self.dropped_events += 1
            return False

    def submit(self, row: list[Any], report: Any = None) -> bool:
        return self._submit("row", row)

    def event(self, event: str, **data: Any) -> bool:
        return self._submit("event", {"wall_time": time.time(), "monotonic_s": time.monotonic(), "event": event, **data})

    def queue_depth(self) -> int:
        return self._rows.qsize()

    def close(self) -> None:
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.1)


class OperatorStop(RuntimeError):
    """An operator transition should stop in this pass, not crash the service."""


class FastRobot:
    def __init__(self, settings: Settings, radio: CRSFReader, *, telemetry_path: Path, live_status: bool = True):
        if settings.motor_backend != "fast":
            raise ValueError("FastRobot requires motor_backend=fast")
        self.settings, self.radio, self.telemetry_path = settings, radio, telemetry_path
        self.config = fast_config(settings)
        self.gate = ArmingGate(settings)
        self.selector = ProfileSelector(settings.profiles)
        self._running = True
        self._shutdown = threading.Event()
        self._owner: int | None = None
        self._hat: FastHat | None = None
        self._telemetry: AsyncCsvTelemetry | None = None
        self._boot_id = self._session_id = 0
        self._last_ack_seq: int | None = None
        self._last_ack_at = time.monotonic()
        self._last_status_at: float | None = None
        self._last_request: DriveRequest | None = None
        self._last_targets = (0.0, 0.0, 0.0, 0.0)
        self._last_command_seq: int | None = None
        self._last_control_started_at: float | None = None
        self._last_control_interval_ms: float | None = None
        self._missed_deadlines = 0
        self._stop_at = 0.0
        self._stop_requested = True
        self._inspection_fault: str | None = None
        self._last_transition: Any = None
        self._operator_profile = Profile.GENTLE
        self._peaks = [{"current_ma": 0, "temperature_c": None, "rpm": 0} for _ in range(4)]
        self._last_inhibition: str | None = None
        self._temperature_samples: list[tuple[float, float] | None] = [None] * 4
        self._temperature_trends = [0.0] * 4
        self._view: dict[str, Any] = {}
        self._view_lock = threading.Lock()
        self._live_status = live_status
        self._display_stop = threading.Event()
        self._display_thread: threading.Thread | None = None
        self._warned_at: dict[str, float] = {}

    def snapshot(self) -> dict[str, Any]:
        """Status consumers cannot issue commands or touch the UART."""
        with self._view_lock:
            return json.loads(json.dumps(self._view))

    def request_shutdown(self, *_args: Any) -> None:
        self._shutdown.set()
        self._running = False

    def _assert_owner(self) -> None:
        if self._owner is not None and self._owner != threading.get_ident():
            raise RuntimeError("Only the Pi supervisor may write motor commands")

    def _event(self, name: str, **details: Any) -> None:
        if self._telemetry is not None:
            self._telemetry.event(name, **details)

    def _clear_requests(self) -> None:
        self.gate.invalidate()
        self.selector.reset()
        self._last_request = None
        self._last_targets = (0.0, 0.0, 0.0, 0.0)

    def _raw_status(self) -> FastStatus:
        if self._hat is None:
            raise FastHatError("Fast HAT serial link is not open")
        status = self._hat.snapshot()
        if status is None or not 0 <= time.monotonic() - status.received_at <= self.settings.fast_status_timeout_s:
            raise FastHatError("HAT feedback became stale")
        if len(status.wheels) != 4:
            raise FastHatError("HAT did not report four wheels")
        return status

    def _status(self, *, require_armed: bool = False) -> FastStatus:
        status = self._raw_status()
        if status.capabilities & REQUIRED_CAPABILITIES != REQUIRED_CAPABILITIES:
            raise FastHatError("HAT protocol capabilities mismatch")
        if self._boot_id and (status.boot_id != self._boot_id or status.host_session != self._session_id):
            raise FastHatError("HAT boot/session identity changed; fresh handshake required")
        if status.config_id != self.config.configuration_id or status.config_result != ConfigResult.APPLIED:
            raise FastHatError("HAT loaded configuration does not match the validated candidate")
        now = time.monotonic()
        if status.ack_seq != self._last_ack_seq:
            self._last_ack_seq, self._last_ack_at = status.ack_seq, now
        if require_armed:
            if not status.motion_enabled:
                raise OperatorStop(f"HAT inhibited motion: fault={status.fault_code}, state={status.state.name}")
            if now - self._last_ack_at > self.settings.fast_status_timeout_s:
                raise FastHatError("HAT has not acknowledged a recent command")
            if not status.feedback_fresh(self.config.feedback_timeout_ms):
                raise OperatorStop("Motor feedback unavailable; stop and wait for recovery")
        return status

    def _wait_disarmed(self, timeout_s: float, *, requested_at: float | None = None,
                       stop_sequence: int | None = None) -> FastStatus:
        """Stop confirmation is independent from fault severity and config state."""
        deadline = time.monotonic() + timeout_s
        last_query = 0.0
        while time.monotonic() < deadline:
            try:
                status = self._raw_status()
                after_request = (requested_at is None or
                                 (status.received_at >= requested_at and
                                  all(w.age_ms is not None and
                                      status.received_at - w.age_ms / 1000 >= requested_at
                                      for w in status.wheels)))
                ack_matches = stop_sequence is None or status.accepted_seq == stop_sequence
                if (after_request and ack_matches and status.stop_confirmed and
                        status.stationary(STATIONARY_RPM, self.config.feedback_timeout_ms) and
                        all(w.mode in (1, 2) for w in status.wheels)):
                    return status
            except FastHatError:
                pass
            if self._hat is not None and time.monotonic() - last_query >= 0.1:
                self._hat.request_status()
                last_query = time.monotonic()
            time.sleep(0.02)
        raise FastHatError("Stop unconfirmed: four fresh stationary motor readings are required")

    def _safe_stop(self, *, wait_ack: bool) -> None:
        self._assert_owner()
        self._clear_requests()
        if self._hat is None:
            return
        self._stop_requested = True
        self._event("stop_requested")
        self._stop_at = time.monotonic()
        self._last_command_seq = self._hat.stop()
        # Receipt cannot prove stopped. Continue queries even if ACK fails/faults.
        if wait_ack:
            try:
                status = self._wait_disarmed(max(2, self.config.stop_verify_ms / 1000 + 0.5),
                                              requested_at=self._stop_at,
                                              stop_sequence=self._last_command_seq)
                self._event("stop_confirmed", fault=status.fault_code)
            except Exception as exc:
                self._inspection_fault = f"STOP UNCONFIRMED: {exc}"
                self._event("stop_unconfirmed", reason=str(exc))
                raise

    def _verify_radio(self, *, neutral: bool = False) -> DriveRequest:
        if self._shutdown.is_set() or not self._running:
            raise OperatorStop("Shutdown requested")
        snap = self.radio.snapshot()
        if not radio_healthy(snap, time.monotonic(), self.settings):
            raise OperatorStop("Radio frames or link statistics stale")
        assigned = self.settings.channels
        if channel(snap, assigned.arm) <= 0.5 or channel(snap, assigned.stop) > 0.5:
            raise OperatorStop("Arm released or operator stop requested")
        request = drive_request(snap, self.settings, self.selector)
        if neutral and (request.throttle != 0 or request.steering != 0):
            raise OperatorStop("Throttle or steering changed during arming; return to neutral and rearm")
        return request

    def _establish_session(self) -> None:
        self._assert_owner()
        self._clear_requests()
        if self._hat is None:
            raise FastHatError("HAT is unavailable")
        for attempt in range(3):
            if self._shutdown.is_set():
                raise OperatorStop("Shutdown requested")
            try:
                seq = self._hat.hello()
                self._hat.wait_ack(seq, timeout=1.5)
                stopped = self._wait_disarmed(3)
                if stopped.fault_code:
                    raise FastHatError(f"HAT fault {stopped.fault_code} prevents session configuration")
                self._boot_id, self._session_id = stopped.boot_id, stopped.host_session
                seq = self._hat.send_config(self.config)
                self._hat.wait_ack(seq, timeout=1)
                self._status()
                self._event("session_ready", boot_id=stopped.boot_id, session=stopped.host_session,
                            firmware_build_id=stopped.build_id, config_id=self.config.configuration_id)
                return
            except FastHatError as exc:
                self._event("recovery_attempt", attempt=attempt + 1, reason=str(exc))
                if attempt == 2:
                    raise
                time.sleep(0.2 * (attempt + 1))

    def _inhibit_reason(self, status: FastStatus) -> str | None:
        if self._inspection_fault:
            return self._inspection_fault
        if status.stop_state == StopState.UNCONFIRMED:
            self._inspection_fault = "STOP UNCONFIRMED: inspect missing wheel feedback and use independent cutoff"
            return self._inspection_fault
        if status.fault_code:
            reason = f"HAT fault {status.fault_code} wheel {status.fault_wheel}"
            # Motor error, stall, current and unconfirmed stop need inspection;
            # command loss, feedback loss and thermal recovery clear to readiness.
            if status.fault_code in (FaultCode.MOTOR_FAULT, FaultCode.OVERSPEED,
                                     FaultCode.STALL, FaultCode.ABNORMAL_CURRENT, FaultCode.CONFIGURATION_FAULT,
                                     FaultCode.CONTROL_PROGRESS):
                self._inspection_fault = reason
            return reason
        for mid, wheel in enumerate(status.wheels, 1):
            if wheel.error:
                self._inspection_fault = f"Motor {mid} error 0x{wheel.error:02X}; inspect wheel"
                return self._inspection_fault
        return None

    def _record(self, status: FastStatus, radio: Any) -> None:
        now = time.monotonic()
        request = self._last_request
        telemetry = self._telemetry
        transmitter = getattr(self.radio, "transmitter", None)
        stats = self._hat.stats() if self._hat else None
        radio_age = None if radio.channels_at is None else round((now - radio.channels_at) * 1000)
        view = {"armed": bool(self.gate.armed and status.motion_enabled),
                "requested_profile": self._operator_profile.name,
                "applied_profile": status.applied_profile.name,
                "profile_valid": self.selector.valid,
                "profile_reason": self.selector.reason,
                "hat_state": status.state.name, "fault": status.fault_code,
                "fault_wheel": status.fault_wheel,
                "stop": StopState.REQUESTED.name if self._stop_requested and status.stop_state == StopState.NONE else status.stop_state.name,
                "hat_stop_state": status.stop_state.name, "stop_requested": self._stop_requested,
                "hold": int(status.hold_flags), "reasons": [flag.name for flag in Reason if flag & status.reason_flags],
                "boost_remaining_ms": status.boost_remaining_ms, "boost_capacity_ms": status.boost_capacity_ms,
                "boost_refill_remaining_ms": status.boost_refill_remaining_ms, "cooldown_ms": status.cooldown_remaining_ms,
                "radio_age_ms": radio_age, "command_age_ms": status.command_age_ms,
                "status_age_ms": round((now - status.received_at) * 1000),
                "logging_dropped_rows": telemetry.dropped_rows if telemetry else 0,
                "logging_dropped_events": telemetry.dropped_events if telemetry else 0,
                "storage_error": telemetry.storage_error if telemetry else None,
                "radio_tx_error": transmitter.error if transmitter else None,
                "radio_tx_dropped": transmitter.dropped_batches if transmitter else 0,
                "inspection_fault": self._inspection_fault,
                "wheels": [asdict(w) for w in status.wheels]}
        for index, wheel in enumerate(status.wheels):
            if wheel.temperature_valid and wheel.temp_age_ms is not None:
                measured_at = now - wheel.temp_age_ms / 1000
                previous = self._temperature_samples[index]
                if previous is None or measured_at - previous[1] > 0.1:
                    if previous is not None:
                        self._temperature_trends[index] = (wheel.temp_c - previous[0]) / (measured_at - previous[1])
                    self._temperature_samples[index] = (wheel.temp_c, measured_at)
            view["wheels"][index]["temperature_trend_c_s"] = self._temperature_trends[index]
        view["condition"] = "STOP REQUESTED" if view["stop"] in ("REQUESTED", "IN_PROGRESS", "UNCONFIRMED") else "STOP CONFIRMED" if status.stop_confirmed else "DERATING" if status.applied_profile != status.requested_profile or status.reason_flags & (Reason.FIRMWARE_CEILING | Reason.THERMAL_DERATE) else "WARNING" if status.reason_flags or not status.feedback_fresh(self.config.feedback_timeout_ms) or not status.temperature_fresh(self.config.temp_stop_stale_ms) else "WITHIN LIMITS"
        with self._view_lock:
            self._view = view
        for peak, wheel in zip(self._peaks, status.wheels):
            if wheel.current_valid: peak["current_ma"] = max(peak["current_ma"], abs(wheel.current_ma))
            if wheel.speed_valid: peak["rpm"] = max(peak["rpm"], abs(wheel.rpm))
            if wheel.temperature_valid: peak["temperature_c"] = max(peak["temperature_c"] if peak["temperature_c"] is not None else -40, wheel.temp_c)
        transition = (view["armed"], view["hat_state"], view["fault"], view["stop"], view["applied_profile"], int(status.reason_flags))
        if transition != self._last_transition:
            self._event("state_transition", **view, peaks=self._peaks)
            self._last_transition = transition
        if telemetry is None or self._last_status_at == status.received_at:
            return
        self._last_status_at = status.received_at
        row = [time.time(), now, radio_age, radio.link_quality, int(view["armed"]),
               request.throttle if request else 0, request.steering if request else 0,
               view["requested_profile"], view["applied_profile"], view["profile_valid"], status.state.name,
               status.fault_code, status.fault_wheel, status.stop_state.name, int(status.hold_flags), int(status.reason_flags),
               status.boost_remaining_ms, status.boost_capacity_ms, status.boost_refill_remaining_ms,
               status.cooldown_remaining_ms, status.sweep_us, status.command_age_ms, view["status_age_ms"],
               status.ack_seq, status.applied_seq, self._last_command_seq, self._missed_deadlines,
               getattr(stats, "last_rtt_ms", None), getattr(stats, "max_rtt_ms", None),
               getattr(stats, "last_status_interval_ms", None), self._last_control_interval_ms,
               telemetry.queue_depth(), telemetry.dropped_rows, telemetry.storage_error, view["radio_tx_error"],
               status.boot_id, status.host_session, status.build_id, status.config_id]
        row.extend(self._last_targets)
        for attr in ("target_rpm", "rpm", "effective_cap_ma", "hold_cap_ma", "current_ma", "temp_c", "error", "age_ms",
                     "temp_age_ms", "position_raw", "validity", "reason_flags"):
            row.extend(getattr(w, attr) for w in status.wheels)
        telemetry.submit(row)
        if hasattr(self.radio, "publish_telemetry"):
            summary = f"{status.state.name} {view['requested_profile'][0]}>{status.applied_profile.name[0]} B{status.boost_remaining_ms // 1000}s {status.stop_state.name}"
            if status.fault_code:
                summary += f" F{status.fault_code}W{status.fault_wheel}"
            if not self.selector.valid: summary += " SWITCH?"
            elif status.reason_flags:
                summary += " " + "/".join(view["reasons"])
            if not status.feedback_fresh(self.config.feedback_timeout_ms): summary += " SENSOR INVALID"
            frames = [encode_flight_mode(summary)]
            if status.feedback_fresh(self.config.feedback_timeout_ms):
                frames.extend(encode_rpm_telemetry((w.rpm,), source=index) for index, w in enumerate(status.wheels))
            for mid, wheel in enumerate(status.wheels):
                if wheel.temperature_valid and wheel.temp_age_ms is not None and wheel.temp_age_ms <= self.config.temp_stop_stale_ms:
                    frames.append(encode_temperature_telemetry((wheel.temp_c,), source=mid + 4))
            self.radio.publish_telemetry(tuple(frames))

    def _display(self) -> None:
        """Console work can block only this status consumer, never commands."""
        while not self._display_stop.wait(1 / self.settings.live_status_hz):
            view = self.snapshot()
            if not view:
                continue
            wheels = " ".join(f"W{i}: {w['target_rpm']:+.2f}/{w['rpm']:+d}rpm {w['current_ma']/1000:+.2f}/{w['effective_cap_ma']/1000:.2f}A T={w['temp_c']}C/{w['temp_age_ms']}ms trend={w['temperature_trend_c_s']:+.2f}C/s valid={w['validity']} fb={w['age_ms']}ms err={w['error']} hold={w['hold_cap_ma']}mA"
                              for i, w in enumerate(view["wheels"], 1))
            LOG.info("%s %s requested=%s applied=%s stop=%s hold=%s Boost=%.1fs/refill=%.1fs radio=%sms cmd=%sms %s reasons=%s dropped=%s storage=%s",
                     view["condition"], view["hat_state"], view["requested_profile"], view["applied_profile"], view["stop"], view["hold"],
                     view["boost_remaining_ms"] / 1000, view["boost_refill_remaining_ms"] / 1000,
                     view["radio_age_ms"], view["command_age_ms"], wheels, view["reasons"], view["logging_dropped_rows"], view["storage_error"])
            warnings = []
            if view["inspection_fault"]: warnings.append(view["inspection_fault"])
            if view["fault"]: warnings.append(f"Protection stop wheel {view['fault_wheel']}, fault {view['fault']}; inspect status")
            if view["stop"] in ("UNCONFIRMED", "IN_PROGRESS", "REQUESTED"): warnings.append("Stop unconfirmed; use independent motor-power cutoff if needed")
            if "HOLD_LIMITED" in view["reasons"]: warnings.append("Holding limited; secure chassis against drift")
            for i, w in enumerate(view["wheels"], 1):
                if w["temp_c"] is not None and w["temp_c"] >= self.config.temp_warn_c:
                    warnings.append(f"Wheel {i}: {w['temp_c']} C, trend {w['temperature_trend_c_s']:+.2f} C/s; reduce load and monitor temperature")
                if int(w["reason_flags"]) & int(Reason.SPEED_ERROR | Reason.STALL_WARNING):
                    warnings.append(f"Wheel {i}: current cap reached with low speed; inspect obstruction or reduce demand")
            if not view["profile_valid"]: warnings.append(f"Profile switch: {view['profile_reason']}; verify SB/channel calibration")
            if view["storage_error"]: warnings.append(f"Logging storage error: {view['storage_error']}")
            for warning in warnings:
                key = warning.split(":")[0]
                if time.monotonic() - self._warned_at.get(key, 0) >= 5:
                    LOG.warning("%s", warning)
                    self._warned_at[key] = time.monotonic()

    def run(self) -> None:
        self._owner = threading.get_ident()
        period = self.settings.fast_period_ms / 1000
        self._telemetry = AsyncCsvTelemetry(self.telemetry_path, max_rows=self.settings.telemetry_queue_rows,
            metadata={"configuration": self.settings.to_dict(), "configuration_identity": self.settings.configuration_identity,
                      "firmware_configuration": asdict(self.config), "firmware_configuration_id": self.config.configuration_id,
                      "protocol_version": 2, "pi_build": _pi_build_identity()})
        self._telemetry.start()
        self.telemetry_path = self._telemetry.path
        if self._live_status:
            self._display_thread = threading.Thread(target=self._display, name="robot-live-status", daemon=True)
            self._display_thread.start()
        with FastHat(self.settings.motor_port, baudrate=self.settings.hat_baud, timeout=self.settings.motor_timeout_s) as hat:
            self._hat = hat
            try:
                self._establish_session()
                while self._running and not self._shutdown.is_set():
                    began = time.monotonic()
                    if self._last_control_started_at is not None:
                        self._last_control_interval_ms = (began - self._last_control_started_at) * 1000
                    self._last_control_started_at = began
                    snap = self.radio.snapshot()
                    if radio_healthy(snap, time.monotonic(), self.settings):
                        intent = drive_request(snap, self.settings, self.selector)
                        self._operator_profile = Profile(intent.profile)
                    else:
                        self.selector.reset()
                        self._operator_profile = Profile.GENTLE
                    try:
                        status = self._status()
                    except FastHatError as exc:
                        self._clear_requests()
                        self._event("communication_inhibition", reason=str(exc))
                        self._safe_stop(wait_ack=False)
                        self._establish_session()
                        status = self._status()
                    reason = self._inhibit_reason(status)
                    if reason:
                        self._clear_requests()
                        if reason != self._last_inhibition:
                            self._event("motion_inhibited", reason=reason, wheels=[asdict(w) for w in status.wheels])
                        self._last_inhibition = reason
                        # Preserve the HAT verification deadline. A pending stop
                        # needs observational queries, not a new stop generation.
                        if status.motion_enabled or status.stop_state == StopState.NONE:
                            self._safe_stop(wait_ack=False)
                        elif not status.stop_confirmed:
                            self._stop_requested = True
                            hat.request_status()
                    else:
                        if self._last_inhibition is not None:
                            self._event("readiness_recovered", previous_reason=self._last_inhibition, deliberate_rearm_required=True)
                        self._last_inhibition = None
                        state = self.gate.observe(snap, time.monotonic())
                        try:
                            if state in ("link_lost", "stop_requested", "disarmed") or (state == "stop_held" and status.motion_enabled):
                                self._safe_stop(wait_ack=False)
                            elif state == "newly_armed":
                                if not status.stop_confirmed or not status.stationary(STATIONARY_RPM, self.config.feedback_timeout_ms):
                                    self.gate.invalidate()
                                else:
                                    self._verify_radio(neutral=True)
                                    self._assert_owner()
                                    seq = hat.arm()
                                    hat.wait_ack(seq, timeout=0.5)
                                    self._verify_radio(neutral=True)
                                    self._last_command_seq = seq
                                    self._stop_requested = False
                                    status = self._status(require_armed=True)
                                    self._event("deliberate_arm")
                            elif state == "armed":
                                status = self._status(require_armed=True)
                                request = self._verify_radio()
                                targets = wheel_targets(request, self.settings)
                                signed = tuple(targets[w.motor_id] * w.polarity for w in sorted(self.settings.wheels, key=lambda w: w.motor_id))
                                self._assert_owner()
                                if self._shutdown.is_set(): raise OperatorStop("Shutdown requested")
                                self._last_command_seq = hat.targets(signed, Profile(request.profile))
                                self._last_request, self._last_targets = request, signed
                                # A shutdown delivered inside a pending transport call is
                                # observed before another ARM/TARGETS can be submitted.
                                if self._shutdown.is_set(): raise OperatorStop("Shutdown requested")
                        except OperatorStop as exc:
                            self._event("operator_stop", reason=str(exc))
                            self._safe_stop(wait_ack=False)
                        except FastHatError as exc:
                            self._event("motion_command_rejected", reason=str(exc))
                            self._safe_stop(wait_ack=False)
                    status = self._raw_status()
                    self._record(status, snap)
                    delay = period - (time.monotonic() - began)
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        self._missed_deadlines += 1
            finally:
                failure = sys.exc_info()[0] is not None
                try:
                    self._safe_stop(wait_ack=True)
                except Exception:
                    if not failure: raise
                finally:
                    self._hat = None
                    self._display_stop.set()
                    self._event("session_peaks", wheels=self._peaks)
                    self._telemetry.close()
                    if self._display_thread is not None:
                        self._display_thread.join(timeout=0.1)


def _pi_build_identity() -> str:
    """Runtime source identity without a shell command or network dependency."""
    import hashlib
    digest = hashlib.sha256()
    for filename in ("fast_robot.py", "robot_main.py", "robot_config.py", "robot_radio.py", "crsf.py", "fast_hat.py", "protocol_defs.py"):
        digest.update(filename.encode()); digest.update(Path(__file__).with_name(filename).read_bytes())
    return digest.hexdigest()
