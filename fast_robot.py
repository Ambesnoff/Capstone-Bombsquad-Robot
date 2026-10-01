"""Pi-side supervisor for the fast four-wheel HAT firmware.

The Pi reads the XR4, mixes requested wheel speeds, and supervises the HAT.
The HAT closes each motor's speed/current loop and stops if Pi commands stop.
No serial ports or motors are touched merely by importing this module.
"""

from __future__ import annotations

import csv
import logging
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any

from fast_hat import FastConfig, FastHat, FastHatError, FastStatus
from robot_main import ArmingGate, CRSFReader, DriveRequest, Settings, channel, drive_request, radio_healthy, wheel_targets


LOG = logging.getLogger("robot.fast")
HAT_DISARMED = 1
HAT_ARMED = 2
HAT_FAULT = 3
STATIONARY_RPM = 2
TELEMETRY_QUEUE_ROWS = 32


def fast_config(settings: Settings) -> FastConfig:
    """Convert conservative Pi settings into HAT's integer wire units."""
    return FastConfig(
        max_rpm=settings.max_rpm,
        max_current_ma=round(settings.max_current_a * 1000),
        neutral_brake_ma=round(settings.neutral_braking_current_a * 1000),
        accel_rpm_s=round(settings.acceleration_rpm_s),
        decel_rpm_s=round(settings.deceleration_rpm_s),
        kp_ma_per_rpm=round(settings.kp_a_per_rpm * 1000),
        ki_ma_per_rpm_s=round(settings.ki_a_per_rpm_s * 1000),
        ff_ma_per_rpm_s=settings.ff_ma_per_rpm_s,
        watchdog_ms=settings.fast_watchdog_ms,
        control_period_ms=settings.fast_period_ms,
        stall_time_ms=settings.stall_time_ms,
        temp_limit_c=settings.temp_limit_c,
    )


def _telemetry_header() -> list[str]:
    fields = [
        "wall_time", "monotonic_s", "radio_age_ms", "link_quality", "armed",
        "throttle", "steering", "current_cap_ma", "hat_state", "hat_fault",
        "hat_sweep_us", "hat_command_age_ms", "hat_status_age_ms",
        "hat_ack_seq", "pi_command_seq", "missed_pi_deadlines",
        "hat_last_rtt_ms", "hat_max_rtt_ms", "hat_mean_rtt_ms",
        "hat_status_interval_ms", "hat_max_status_interval_ms", "hat_mean_status_interval_ms",
        "pi_control_interval_ms", "pi_max_control_interval_ms", "telemetry_queue_depth",
        "hat_status_frames", "hat_tx_frames", "hat_bad_crc",
        "hat_bad_status", "hat_old_status", "hat_discarded_bytes",
    ]
    for name in ("requested_rpm", "actual_rpm", "reported_current_ma", "temperature_c", "motor_error", "motor_feedback_age_ms"):
        fields.extend(f"{name}_{mid}" for mid in range(1, 5))
    return fields


class AsyncCsvTelemetry:
    """Write bounded telemetry outside the control loop.

    A full queue or failed writer is a control fault: the caller can then
    stop the motors without waiting for a stalled disk operation.
    """

    def __init__(self, path: Path, *, max_rows: int = TELEMETRY_QUEUE_ROWS):
        self.path = path
        self._rows: queue.Queue[tuple[list[Any], tuple[Any, ...] | None] | None] = queue.Queue(max_rows)
        self._ready = threading.Event()
        self._done = threading.Event()
        self._error: Exception | None = None
        self._thread: threading.Thread | None = None
        self._closed = False

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Telemetry writer is already started")
        self._thread = threading.Thread(target=self._run, name="robot-telemetry", daemon=True)
        self._thread.start()
        if not self._ready.wait(2.0):
            raise RuntimeError("Telemetry file did not open promptly")
        self.check()

    def _run(self) -> None:
        try:
            with self.path.open("w", newline="", encoding="utf-8", buffering=65536) as file:
                writer = csv.writer(file)
                writer.writerow(_telemetry_header())
                last_flush = time.monotonic()
                self._ready.set()
                while True:
                    item = self._rows.get()
                    if item is None:
                        break
                    row, report = item
                    writer.writerow(row)
                    now = time.monotonic()
                    if now - last_flush >= 1.0:
                        file.flush()
                        last_flush = now
                    if report is not None:
                        LOG.info("HAT received %.1f reports/s, sweep %.1fms, missed Pi deadlines %d; wheel RPM %s",
                                 *report)
        except Exception as exc:
            self._error = exc
        finally:
            self._ready.set()
            self._done.set()

    def check(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"Telemetry writer failed: {self._error}") from self._error
        if self._done.is_set() and not self._closed:
            raise RuntimeError("Telemetry writer stopped unexpectedly")

    def submit(self, row: list[Any], report: tuple[Any, ...] | None = None) -> None:
        self.check()
        if self._closed:
            raise RuntimeError("Telemetry writer is closed")
        try:
            self._rows.put_nowait((row, report))
        except queue.Full as exc:
            raise RuntimeError("Telemetry writer backlog is full") from exc

    def queue_depth(self) -> int:
        return self._rows.qsize()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._thread is None:
            return
        if not self._done.is_set():
            try:
                self._rows.put(None, timeout=0.5)
            except queue.Full as exc:
                raise RuntimeError("Telemetry writer did not drain after stop") from exc
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError("Telemetry writer did not finish after stop")
        if self._error is not None:
            raise RuntimeError(f"Telemetry writer failed: {self._error}") from self._error


class FastRobot:
    def __init__(self, settings: Settings, radio: CRSFReader, *, telemetry_path: Path):
        if settings.motor_backend != "fast":
            raise ValueError("FastRobot requires motor_backend=fast")
        self.settings = settings
        self.radio = radio
        self.telemetry_path = telemetry_path
        self.gate = ArmingGate(settings)
        self._running = True
        self._hat: FastHat | None = None
        self._last_status_at: float | None = None
        self._last_ack_seq: int | None = None
        self._last_ack_at = time.monotonic()
        self._last_log_at = time.monotonic()
        self._last_log_count = 0
        self._status_count = 0
        self._missed_deadlines = 0
        self._last_request: DriveRequest | None = None
        self._last_targets = (0, 0, 0, 0)
        self._last_command_seq: int | None = None
        self._last_control_started_at: float | None = None
        self._last_control_interval_ms: float | None = None
        self._max_control_interval_ms: float | None = None

    def request_shutdown(self, *_args: Any) -> None:
        self._running = False

    def _status(self, *, require_armed: bool = False) -> FastStatus:
        if self._hat is None:
            raise RuntimeError("Fast HAT serial link is not open")
        status = self._hat.snapshot()
        now = time.monotonic()
        if status is None or now - status.received_at > self.settings.fast_status_timeout_s:
            raise RuntimeError("HAT feedback became stale")
        if status.fault_code or status.state == HAT_FAULT:
            raise RuntimeError(f"HAT fault {status.fault_code}; inspect all four wheels before restarting")
        if require_armed and status.state != HAT_ARMED:
            raise RuntimeError(f"HAT stopped or disarmed itself (state {status.state})")
        if len(status.wheels) != 4:
            raise RuntimeError("HAT did not report all four wheels")
        for mid, wheel in enumerate(status.wheels, 1):
            if wheel.error:
                raise RuntimeError(f"Motor {mid} fault 0x{wheel.error:02X}")
            if require_armed and (wheel.age_ms is None or
                                  wheel.age_ms >= self.settings.fast_status_timeout_s * 1000):
                raise RuntimeError(f"Motor {mid} feedback became stale")
            if wheel.temp_c is not None and wheel.temp_c >= self.settings.temp_limit_c:
                raise RuntimeError(f"Motor {mid} winding temperature reached {wheel.temp_c} C")
        if status.ack_seq != self._last_ack_seq:
            self._last_ack_seq = status.ack_seq
            self._last_ack_at = now
        if require_armed and now - self._last_ack_at > self.settings.fast_status_timeout_s:
            raise RuntimeError("HAT has not acknowledged a recent command")
        return status

    def _wait_disarmed(self, timeout_s: float) -> FastStatus:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                status = self._status()
            except RuntimeError as exc:
                if "feedback became stale" not in str(exc):
                    raise
            else:
                if status.state == HAT_DISARMED and status.stationary(
                    STATIONARY_RPM, min(200, round(self.settings.fast_status_timeout_s * 1000))
                ):
                    return status
            time.sleep(0.02)
        raise RuntimeError("HAT did not confirm four stopped, fresh motors")

    def _safe_stop(self, *, wait_ack: bool) -> None:
        if self._hat is None:
            return
        try:
            seq = self._hat.stop()
            self._last_command_seq = seq
            if wait_ack:
                self._hat.wait_ack(seq, timeout=max(0.5, self.settings.fast_watchdog_ms / 1000 + 0.2))
                self._wait_disarmed(3.0)
        except Exception as exc:
            LOG.error("Could not confirm stop through HAT: %s", exc)
            if wait_ack:
                raise

    def _radio_still_armed(self) -> DriveRequest:
        snapshot = self.radio.snapshot()
        now = time.monotonic()
        if not radio_healthy(snapshot, now, self.settings):
            raise RuntimeError("Radio frames or link statistics became stale")
        assigned = self.settings.channels
        if channel(snapshot, assigned.arm) <= 0.5 or channel(snapshot, assigned.stop) > 0.5:
            raise RuntimeError("Arm switch released or stop switch requested")
        return drive_request(snapshot, self.settings)

    def _record(self, telemetry: AsyncCsvTelemetry, status: FastStatus, snapshot: Any) -> None:
        if status.received_at == self._last_status_at:
            return
        self._last_status_at = status.received_at
        self._status_count += 1
        now = time.monotonic()
        request = self._last_request
        stats = self._hat.stats() if self._hat is not None else None
        row: list[Any] = [
            time.time(), now,
            round((now - snapshot.channels_at) * 1000) if snapshot.channels_at is not None else "",
            snapshot.link_quality if snapshot.link_quality is not None else "",
            int(self.gate.armed),
            round(request.throttle, 4) if request else 0,
            round(request.steering, 4) if request else 0,
            round(request.current_cap_a * 1000) if request else 0,
            status.state, status.fault_code, status.sweep_us, status.command_age_ms,
            round((now - status.received_at) * 1000), status.ack_seq,
            self._last_command_seq if self._last_command_seq is not None else "",
            self._missed_deadlines,
            round(stats.last_rtt_ms, 2) if stats and stats.last_rtt_ms is not None else "",
            round(stats.max_rtt_ms, 2) if stats and stats.max_rtt_ms is not None else "",
            round(stats.mean_rtt_ms, 2) if stats and stats.mean_rtt_ms is not None else "",
            round(stats.last_status_interval_ms, 2) if stats and stats.last_status_interval_ms is not None else "",
            round(stats.max_status_interval_ms, 2) if stats and stats.max_status_interval_ms is not None else "",
            round(stats.mean_status_interval_ms, 2) if stats and stats.mean_status_interval_ms is not None else "",
            round(self._last_control_interval_ms, 2) if self._last_control_interval_ms is not None else "",
            round(self._max_control_interval_ms, 2) if self._max_control_interval_ms is not None else "",
            telemetry.queue_depth(),
            stats.status_frames if stats else "",
            stats.tx_frames if stats else "",
            stats.bad_crc if stats else "",
            stats.bad_status if stats else "",
            stats.old_status if stats else "",
            stats.discarded_bytes if stats else "",
        ]
        row.extend(self._last_targets)
        row.extend(w.rpm for w in status.wheels)
        row.extend(w.current_ma for w in status.wheels)
        row.extend("" if w.temp_c is None else w.temp_c for w in status.wheels)
        row.extend(w.error for w in status.wheels)
        row.extend("" if w.age_ms is None else w.age_ms for w in status.wheels)
        report = None
        if now - self._last_log_at >= 1.0:
            elapsed = now - self._last_log_at
            received = stats.status_frames if stats else self._status_count
            reported_hz = (received - self._last_log_count) / elapsed
            self._last_log_at = now
            self._last_log_count = received
            report = (reported_hz, status.sweep_us / 1000, self._missed_deadlines,
                      ", ".join(str(w.rpm) for w in status.wheels))
        telemetry.submit(row, report)

    def run(self) -> None:
        period = self.settings.fast_period_ms / 1000
        with FastHat(self.settings.motor_port, baudrate=self.settings.hat_baud,
                     timeout=self.settings.motor_timeout_s) as hat:
            self._hat = hat
            telemetry: AsyncCsvTelemetry | None = None
            try:
                for attempt in range(3):
                    hello_seq = hat.hello()
                    try:
                        hat.wait_ack(hello_seq, timeout=1.5)
                        break
                    except FastHatError:
                        if attempt == 2 or hat.stats().reader_error or not self._running:
                            raise
                        LOG.warning("HAT is still starting; retrying safe HELLO")
                        time.sleep(0.2)
                self._wait_disarmed(5.0)
                config_seq = hat.send_config(fast_config(self.settings))
                hat.wait_ack(config_seq, timeout=1.0)
                LOG.info("Fast HAT configured. Radio must be healthy, throttle zero, then arm switch low -> high.")

                telemetry = AsyncCsvTelemetry(self.telemetry_path)
                telemetry.start()
                while self._running:
                    began = time.monotonic()
                    if self._last_control_started_at is not None:
                        interval_ms = max(0.0, (began - self._last_control_started_at) * 1000)
                        self._last_control_interval_ms = interval_ms
                        self._max_control_interval_ms = max(self._max_control_interval_ms or 0.0,
                                                            interval_ms)
                    self._last_control_started_at = began
                    telemetry.check()
                    radio_snapshot = self.radio.snapshot()
                    # The reader can timestamp a new frame between began
                    # and snapshot(); compare freshness with a later time.
                    state = self.gate.observe(radio_snapshot, time.monotonic())
                    status = self._status(require_armed=state == "armed")

                    if state in ("link_lost", "stop_requested", "disarmed"):
                        self._safe_stop(wait_ack=True)
                        LOG.warning("%s; requesting stop for all four motors", state)
                        self._last_request = None
                        self._last_targets = (0, 0, 0, 0)
                        status = self._status()
                    elif state == "newly_armed":
                        if status.state != HAT_DISARMED or not status.stationary(
                            STATIONARY_RPM,
                            min(200, round(self.settings.fast_status_timeout_s * 1000)),
                        ):
                            self.gate.invalidate()
                            LOG.warning("HAT has not confirmed four stationary wheels; repeat arm cycle")
                        else:
                            arm_seq = hat.arm()
                            hat.wait_ack(arm_seq, timeout=0.5)
                            self._last_command_seq = arm_seq
                            self._last_ack_at = time.monotonic()
                            status = self._status(require_armed=True)
                            LOG.info("Armed four-wheel drive")
                    elif state == "armed":
                        request = self._radio_still_armed()
                        targets = wheel_targets(request, self.settings)
                        signed = tuple(round(targets[w.motor_id] * w.polarity)
                                       for w in sorted(self.settings.wheels,
                                                       key=lambda wheel: wheel.motor_id))
                        current_ma = round(request.current_cap_a * 1000)
                        self._last_command_seq = hat.targets(signed, current_ma)
                        self._last_request = request
                        self._last_targets = signed

                    self._record(telemetry, status, radio_snapshot)
                    delay = period - (time.monotonic() - began)
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        self._missed_deadlines += 1
            finally:
                error_in_flight = sys.exc_info()[0] is not None
                cleanup_error: Exception | None = None
                try:
                    self._safe_stop(wait_ack=True)
                except Exception as exc:
                    if not error_in_flight:
                        cleanup_error = exc
                try:
                    if telemetry is not None:
                        telemetry.close()
                except Exception as exc:
                    if not error_in_flight and cleanup_error is None:
                        cleanup_error = exc
                finally:
                    self._hat = None
                if cleanup_error is not None:
                    raise cleanup_error
