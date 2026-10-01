"""Binary Raspberry Pi link for the four-wheel robot HAT firmware.

The HAT owns the motor-side speed loops and watchdog. This module only sends
high-level targets and receives validated, timestamped four-wheel reports.
Creating a FastHat never arms the motors or sends a motion command.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping, Sequence


SYNC = b"\xa5\x5a"
VERSION = 1
MAX_PAYLOAD = 64
STATUS_LENGTH = 36
MAX_STATUS_BACKLOG = 256
_CONFIG_FORMAT = "<11HB"
_STATUS_PREFIX = struct.Struct("<HBBHH")
_WHEEL_FORMAT = struct.Struct("<hhbBB")


class FastHatError(RuntimeError):
    """The host cannot trust or complete a HAT operation."""


class FrameType(IntEnum):
    HELLO = 0x01
    CONFIG = 0x02
    ARM = 0x03
    TARGETS = 0x04
    STOP = 0x05
    STATUS_REQ = 0x06
    STATUS = 0x80


class HatState(IntEnum):
    STOPPING = 0
    DISARMED = 1
    ARMED = 2
    FAULT = 3


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(frame_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= int(frame_type) <= 255 or not 0 <= sequence <= 65535:
        raise ValueError("Invalid frame type or sequence")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("Frame payload exceeds 64 bytes")
    body = struct.pack("<BBHB", VERSION, int(frame_type), sequence, len(payload)) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass(frozen=True)
class Frame:
    kind: int
    sequence: int
    payload: bytes


class FrameParser:
    """Incremental parser that recovers after noise, bad lengths, or bad CRCs."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames = 0
        self.bad_crc = 0
        self.bad_version = 0
        self.bad_length = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        result: list[Frame] = []
        while True:
            marker = self.buffer.find(SYNC)
            if marker < 0:
                # Retain a trailing A5; the next read may begin with 5A.
                keep = 1 if self.buffer.endswith(SYNC[:1]) else 0
                self.discarded_bytes += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = SYNC[:1]
                else:
                    self.buffer.clear()
                break
            if marker:
                self.discarded_bytes += marker
                del self.buffer[:marker]
            if len(self.buffer) < 7:
                break
            if self.buffer[2] != VERSION:
                self.bad_version += 1
                self.discarded_bytes += 1
                del self.buffer[:1]
                continue
            length = self.buffer[6]
            if length > MAX_PAYLOAD:
                self.bad_length += 1
                self.discarded_bytes += 1
                del self.buffer[:1]
                continue
            total = 9 + length
            if len(self.buffer) < total:
                break
            expected = struct.unpack_from("<H", self.buffer, total - 2)[0]
            actual = crc16_ccitt_false(self.buffer[2:total - 2])
            if expected != actual:
                self.bad_crc += 1
                self.discarded_bytes += 1
                del self.buffer[:1]
                continue
            kind = self.buffer[3]
            sequence = struct.unpack_from("<H", self.buffer, 4)[0]
            result.append(Frame(kind, sequence, bytes(self.buffer[7:total - 2])))
            self.frames += 1
            del self.buffer[:total]
        return result


@dataclass(frozen=True)
class FastWheel:
    rpm: int
    current_ma: int
    temp_c: int | None
    error: int
    age_ms: int | None


@dataclass(frozen=True)
class FastStatus:
    ack_seq: int
    state: HatState
    fault_code: int
    sweep_us: int
    command_age_ms: int
    wheels: tuple[FastWheel, FastWheel, FastWheel, FastWheel]
    received_at: float

    def feedback_fresh(self, max_age_ms: int) -> bool:
        return all(wheel.age_ms is not None and wheel.age_ms <= max_age_ms for wheel in self.wheels)

    def stationary(self, max_abs_rpm: int = 2, max_age_ms: int = 100) -> bool:
        return self.feedback_fresh(max_age_ms) and all(abs(wheel.rpm) <= max_abs_rpm for wheel in self.wheels)


def decode_status(payload: bytes, received_at: float | None = None) -> FastStatus:
    if len(payload) != STATUS_LENGTH:
        raise ValueError(f"STATUS payload is {len(payload)} bytes, expected 36")
    ack, raw_state, fault, sweep, command_age = _STATUS_PREFIX.unpack_from(payload)
    try:
        state = HatState(raw_state)
    except ValueError as exc:
        raise ValueError(f"Unknown HAT state {raw_state}") from exc
    records: list[FastWheel] = []
    for offset in range(_STATUS_PREFIX.size, STATUS_LENGTH, _WHEEL_FORMAT.size):
        rpm, current, temperature, error, age = _WHEEL_FORMAT.unpack_from(payload, offset)
        records.append(FastWheel(rpm, current, None if temperature == 127 else temperature,
                                 error, None if age == 255 else age))
    return FastStatus(ack, state, fault, sweep, command_age,
                      (records[0], records[1], records[2], records[3]),
                      time.monotonic() if received_at is None else received_at)


def _uint16(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 65535:
        raise ValueError(f"{name} must be an integer within 0..65535")
    return value


@dataclass(frozen=True)
class FastConfig:
    max_rpm: int
    max_current_ma: int
    neutral_brake_ma: int
    accel_rpm_s: int
    decel_rpm_s: int
    kp_ma_per_rpm: int
    ki_ma_per_rpm_s: int
    ff_ma_per_rpm_s: int
    watchdog_ms: int
    control_period_ms: int
    stall_time_ms: int
    temp_limit_c: int

    def payload(self) -> bytes:
        values = (
            self.max_rpm, self.max_current_ma, self.neutral_brake_ma,
            self.accel_rpm_s, self.decel_rpm_s, self.kp_ma_per_rpm,
            self.ki_ma_per_rpm_s, self.ff_ma_per_rpm_s, self.watchdog_ms,
            self.control_period_ms, self.stall_time_ms,
        )
        names = (
            "max_rpm", "max_current_ma", "neutral_brake_ma", "accel_rpm_s",
            "decel_rpm_s", "kp_ma_per_rpm", "ki_ma_per_rpm_s",
            "ff_ma_per_rpm_s", "watchdog_ms", "control_period_ms", "stall_time_ms",
        )
        for name, value in zip(names, values):
            _uint16(value, name)
        if (isinstance(self.temp_limit_c, bool) or not isinstance(self.temp_limit_c, int)
                or not 0 <= self.temp_limit_c <= 255):
            raise ValueError("temp_limit_c must be an integer within 0..255")
        # Match the v1 firmware's compiled bounds so invalid configuration
        # cannot needlessly trip its latched configuration fault.
        limits = (
            ("max_rpm", self.max_rpm, 1, 200),
            ("max_current_ma", self.max_current_ma, 1, 1200),
            ("accel_rpm_s", self.accel_rpm_s, 1, 5000),
            ("decel_rpm_s", self.decel_rpm_s, 1, 5000),
            ("kp_ma_per_rpm", self.kp_ma_per_rpm, 0, 1000),
            ("ki_ma_per_rpm_s", self.ki_ma_per_rpm_s, 0, 1000),
            ("ff_ma_per_rpm_s", self.ff_ma_per_rpm_s, 0, 1000),
            ("watchdog_ms", self.watchdog_ms, 100, 1000),
            ("control_period_ms", self.control_period_ms, 10, 100),
            ("stall_time_ms", self.stall_time_ms, 100, 5000),
            ("temp_limit_c", self.temp_limit_c, 40, 70),
        )
        for name, value, minimum, maximum in limits:
            if not minimum <= value <= maximum:
                raise ValueError(f"{name} must be within {minimum}..{maximum} for HAT firmware v1")
        if self.neutral_brake_ma > self.max_current_ma:
            raise ValueError("neutral_brake_ma cannot exceed max_current_ma")
        return struct.pack(_CONFIG_FORMAT, *values, self.temp_limit_c)


@dataclass(frozen=True)
class FastStats:
    tx_frames: int
    rx_frames: int
    status_frames: int
    bad_crc: int
    bad_version: int
    bad_length: int
    bad_status: int
    old_status: int
    discarded_bytes: int
    last_rtt_ms: float | None
    max_rtt_ms: float | None
    mean_rtt_ms: float | None
    last_status_interval_ms: float | None
    max_status_interval_ms: float | None
    mean_status_interval_ms: float | None
    reader_error: str | None


class FastHat:
    """Threaded link to the HAT; caller keeps control of arming and targets.

    The receive thread never issues commands. STOP sets a shared stop flag
    before waiting for the serial write lock, so pending TARGETS writes fail.
    A command that has already begun writing may finish before STOP; the HAT
    also has an independent target watchdog.
    """

    def __init__(self, port: str, baudrate: int = 230400, timeout: float = 0.15,
                 *, serial_port: Any = None) -> None:
        if not 0 < timeout <= 5:
            raise ValueError("timeout must be within 0..5 seconds")
        if not 9600 <= baudrate <= 1000000:
            raise ValueError("baudrate must be within 9600..1000000")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        if serial_port is None:
            if not port:
                raise ValueError("Supply the HAT UART port")
            try:
                import serial
            except ImportError as exc:
                raise FastHatError("Install pyserial: python3 -m pip install -r requirements.txt") from exc
            device = serial.Serial(port=None, baudrate=baudrate, timeout=min(timeout, 0.02),
                                   write_timeout=timeout)
            device.dtr = False
            device.rts = False
            device.port = port
            device.open()
            self.serial = device
        else:
            self.serial = serial_port
            self.serial.timeout = min(timeout, 0.02)
            self.serial.write_timeout = timeout
        self._write_lock = threading.Lock()
        self._condition = threading.Condition()
        self._stop_flag = threading.Event()
        self._stop_flag.set()  # Motion is forbidden until ARM is acknowledged.
        self._reader_stop = threading.Event()
        self._closed = False
        self._next_seq = 1
        self._status: FastStatus | None = None
        self._last_status_seq: int | None = None
        self._reader_error: str | None = None
        self._parser = FrameParser()
        self._sent_at: dict[int, float] = {}
        self._sent_kind: dict[int, FrameType] = {}
        self._rtt_recorded: set[int] = set()
        self._rtt_count = 0
        self._rtt_sum_ms = 0.0
        self._last_rtt_ms: float | None = None
        self._max_rtt_ms: float | None = None
        self._tx_frames = 0
        self._status_frames = 0
        self._status_interval_count = 0
        self._status_interval_sum_ms = 0.0
        self._last_status_interval_ms: float | None = None
        self._max_status_interval_ms: float | None = None
        self._bad_status = 0
        self._old_status = 0
        try:
            # Data left by a prior host session must not count as live status.
            self.serial.reset_input_buffer()
        except Exception as exc:
            self.serial.close()
            raise FastHatError(f"Could not clear HAT input: {exc}") from exc
        self._reader = threading.Thread(target=self._read_loop, name="fast-hat-reader", daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        while not self._reader_stop.is_set():
            try:
                pending = int(getattr(self.serial, "in_waiting", 0) or 0)
                if pending > MAX_STATUS_BACKLOG:
                    self.serial.reset_input_buffer()
                    with self._condition:
                        self._reader_error = "HAT status backlog discarded"
                        self._stop_flag.set()
                        self._condition.notify_all()
                    return
                waiting = max(1, pending)
                chunk = self.serial.read(waiting)
            except Exception as exc:
                if not self._reader_stop.is_set():
                    with self._condition:
                        self._reader_error = str(exc)
                        self._stop_flag.set()
                        self._condition.notify_all()
                return
            if not chunk:
                self._reader_stop.wait(0.001)
                continue
            with self._condition:
                for frame in self._parser.feed(chunk):
                    if frame.kind != FrameType.STATUS:
                        continue
                    try:
                        status = decode_status(frame.payload, time.monotonic())
                    except ValueError:
                        self._bad_status += 1
                        continue
                    if self._last_status_seq is not None:
                        advance = (frame.sequence - self._last_status_seq) & 0xFFFF
                        if not 0 < advance < 0x8000:
                            self._old_status += 1
                            continue
                    if self._status is not None:
                        interval_ms = max(0.0, (status.received_at - self._status.received_at) * 1000)
                        self._last_status_interval_ms = interval_ms
                        self._max_status_interval_ms = max(self._max_status_interval_ms or 0.0,
                                                           interval_ms)
                        self._status_interval_sum_ms += interval_ms
                        self._status_interval_count += 1
                    self._last_status_seq = frame.sequence
                    self._status = status
                    self._status_frames += 1
                    sent_at = self._sent_at.get(status.ack_seq)
                    if sent_at is not None and status.ack_seq not in self._rtt_recorded:
                        rtt_ms = max(0.0, (status.received_at - sent_at) * 1000)
                        self._last_rtt_ms = rtt_ms
                        self._max_rtt_ms = max(self._max_rtt_ms or 0.0, rtt_ms)
                        self._rtt_sum_ms += rtt_ms
                        self._rtt_count += 1
                        self._rtt_recorded.add(status.ack_seq)
                    self._condition.notify_all()

    def _send(self, kind: FrameType, payload: bytes = b"") -> int:
        with self._write_lock:
            if self._closed:
                raise FastHatError("HAT port is closed")
            with self._condition:
                if self._reader_error is not None and kind != FrameType.STOP:
                    raise FastHatError(f"HAT reader failed: {self._reader_error}")
            if kind == FrameType.TARGETS and self._stop_flag.is_set():
                raise FastHatError("HAT is stopped; ARM must be acknowledged before TARGETS")
            seq = self._next_seq
            self._next_seq = (seq + 1) & 0xFFFF
            frame = encode_frame(kind, seq, payload)
            sent_at = time.monotonic()
            with self._condition:
                self._sent_at[seq] = sent_at
                self._sent_kind[seq] = kind
                self._rtt_recorded.discard(seq)
                # Keep a short history to resolve waits without unbounded growth.
                if len(self._sent_at) > 512:
                    for old in tuple(self._sent_at)[:128]:
                        del self._sent_at[old]
                        self._sent_kind.pop(old, None)
                        self._rtt_recorded.discard(old)
            try:
                written = self.serial.write(frame)
            except Exception as exc:
                self._stop_flag.set()
                raise FastHatError(f"HAT serial write failed: {exc}") from exc
            if written != len(frame):
                self._stop_flag.set()
                raise FastHatError(f"Incomplete HAT write: {written} of {len(frame)} bytes")
            with self._condition:
                self._tx_frames += 1
            return seq

    def hello(self) -> int:
        """Force safe HAT disarm and begin a new sequence session."""
        self._stop_flag.set()
        with self._condition:
            self._status = None
            self._last_status_seq = None
        return self._send(FrameType.HELLO)

    def send_config(self, config: FastConfig | Mapping[str, int]) -> int:
        """Send a complete configuration while the HAT is disarmed."""
        if isinstance(config, Mapping):
            config = FastConfig(**config)
        if not isinstance(config, FastConfig):
            raise TypeError("config must be FastConfig or a field mapping")
        return self._send(FrameType.CONFIG, config.payload())

    def arm(self) -> int:
        """Request arming; TARGETS stay blocked until this command is ACKed."""
        return self._send(FrameType.ARM)

    def targets(self, rpm_by_id: Sequence[int], current_cap_ma: int) -> int:
        """Send four signed RPM targets in motor-ID order 1, 2, 3, 4."""
        if len(rpm_by_id) != 4:
            raise ValueError("Exactly four RPM targets are required")
        for rpm in rpm_by_id:
            if isinstance(rpm, bool) or not isinstance(rpm, int) or not -32768 <= rpm <= 32767:
                raise ValueError("Each target RPM must be a signed 16-bit integer")
        _uint16(current_cap_ma, "current_cap_ma")
        return self._send(FrameType.TARGETS, struct.pack("<hhhhH", *rpm_by_id, current_cap_ma))

    def stop(self) -> int:
        """Block new targets immediately, then send a disarming STOP frame."""
        self._stop_flag.set()
        return self._send(FrameType.STOP)

    def request_status(self) -> int:
        return self._send(FrameType.STATUS_REQ)

    def snapshot(self) -> FastStatus | None:
        with self._condition:
            return self._status

    def wait_ack(self, sequence: int, timeout: float | None = None) -> FastStatus:
        """Wait for an exact post-send STATUS acknowledgment, or fail closed."""
        limit = self.timeout if timeout is None else timeout
        if limit <= 0:
            raise ValueError("ACK timeout must be positive")
        deadline = time.monotonic() + limit
        with self._condition:
            if sequence not in self._sent_at:
                raise FastHatError(f"Sequence {sequence} was not sent by this HAT session")
            sent_at = self._sent_at[sequence]
            kind = self._sent_kind[sequence]
            while True:
                if self._reader_error is not None:
                    self._stop_flag.set()
                    raise FastHatError(f"HAT reader failed: {self._reader_error}")
                status = self._status
                if status is not None and status.received_at >= sent_at:
                    if status.state == HatState.FAULT and kind not in (FrameType.STOP, FrameType.HELLO):
                        self._stop_flag.set()
                        raise FastHatError(f"HAT fault {status.fault_code} while waiting for {kind.name}")
                    if status.ack_seq == sequence:
                        if kind in (FrameType.ARM, FrameType.TARGETS) and status.state != HatState.ARMED:
                            self._stop_flag.set()
                            raise FastHatError(f"HAT ACKed {kind.name} without entering ARMED state")
                        if kind == FrameType.CONFIG and status.state != HatState.DISARMED:
                            self._stop_flag.set()
                            raise FastHatError("HAT ACKed CONFIG while not DISARMED")
                        if kind in (FrameType.HELLO, FrameType.STOP) and status.state == HatState.ARMED:
                            self._stop_flag.set()
                            raise FastHatError(f"HAT ACKed {kind.name} while still ARMED")
                        if kind == FrameType.ARM:
                            self._stop_flag.clear()
                        return status
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._stop_flag.set()
                    raise FastHatError(f"No HAT ACK for {kind.name} sequence {sequence}")
                self._condition.wait(remaining)

    def stats(self) -> FastStats:
        with self._condition:
            parser = self._parser
            return FastStats(
                tx_frames=self._tx_frames, rx_frames=parser.frames,
                status_frames=self._status_frames, bad_crc=parser.bad_crc,
                bad_version=parser.bad_version, bad_length=parser.bad_length,
                bad_status=self._bad_status, old_status=self._old_status,
                discarded_bytes=parser.discarded_bytes,
                last_rtt_ms=self._last_rtt_ms, max_rtt_ms=self._max_rtt_ms,
                mean_rtt_ms=(self._rtt_sum_ms / self._rtt_count if self._rtt_count else None),
                last_status_interval_ms=self._last_status_interval_ms,
                max_status_interval_ms=self._max_status_interval_ms,
                mean_status_interval_ms=(self._status_interval_sum_ms / self._status_interval_count
                                         if self._status_interval_count else None),
                reader_error=self._reader_error,
            )

    def close(self) -> None:
        """Best-effort STOP before closing; HAT watchdog covers link failure."""
        if self._closed:
            return
        self._stop_flag.set()
        try:
            self.stop()
        except FastHatError:
            pass
        with self._write_lock:
            self._closed = True
            self._reader_stop.set()
            self.serial.close()
        self._reader.join(timeout=min(0.2, self.timeout + 0.05))

    def __enter__(self) -> "FastHat":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
