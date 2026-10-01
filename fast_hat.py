"""Binary Raspberry Pi link for the four-wheel robot HAT firmware.

The HAT owns the motor-side speed loops and watchdog. This module only sends
high-level targets and receives validated, timestamped four-wheel reports.
Creating a FastHat never arms the motors or sends a motion command.
"""

from __future__ import annotations

import struct
import math
import secrets
import zlib
import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


from protocol_defs import (
    SYNC, VERSION, MAX_PAYLOAD, STATUS_LENGTH, FIRMWARE_MAX_CURRENT_MA,
    FrameType, HatState, Profile, StopState, ConfigResult, Reason, Capability,
    Validity, HoldFlag, FaultCode, CONFIG_FIELDS, CONFIG_SPEC, CONFIG_STRUCT, validate_config,
    STATUS_HEADER_FIELDS, STATUS_HEADER_STRUCT, WHEEL_FIELDS, WHEEL_STRUCT,
    CONFIG_PREFIX, ARM_STRUCT, TARGETS_STRUCT,
)

MAX_STATUS_BACKLOG = 4 * (STATUS_LENGTH + 9)
REQUIRED_CAPABILITIES = (Capability.PROFILES | Capability.SESSION | Capability.HOLDING |
                         Capability.TEMPERATURE_AGE | Capability.STOP_CONFIRMATION |
                         Capability.CONFIG_ID | Capability.RESET_WATCHDOG | Capability.POSITION)


class FastHatError(RuntimeError):
    """The host cannot trust or complete a HAT operation."""


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(frame_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if (isinstance(frame_type, bool) or not isinstance(frame_type, int)
            or not 0 <= frame_type <= 255 or isinstance(sequence, bool)
            or not isinstance(sequence, int) or not 0 <= sequence <= 65535):
        raise ValueError("Invalid frame type or sequence")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"Frame payload exceeds {MAX_PAYLOAD} bytes")
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
    # rpm is the motor's whole-RPM measurement, not the finer target encoding.
    rpm: int
    current_ma: int
    temp_c: int | None
    error: int
    age_ms: int | None
    target_rpm: float = 0.0
    position_raw: int | None = None
    effective_cap_ma: int = 0
    hold_cap_ma: int = 0
    temp_age_ms: int | None = None
    mode: int = 255
    validity: Validity = Validity.RPM | Validity.CURRENT | Validity.POSITION | Validity.TEMPERATURE
    reason_flags: Reason = Reason(0)

    @property
    def speed_valid(self) -> bool:
        return bool(self.validity & Validity.RPM)

    @property
    def current_valid(self) -> bool:
        return bool(self.validity & Validity.CURRENT)

    @property
    def position_valid(self) -> bool:
        return bool(self.validity & Validity.POSITION) and self.position_raw is not None

    @property
    def temperature_valid(self) -> bool:
        return bool(self.validity & Validity.TEMPERATURE) and self.temp_c is not None


@dataclass(frozen=True)
class FastStatus:
    ack_seq: int
    state: HatState
    fault_code: int
    sweep_us: int
    command_age_ms: int
    wheels: tuple[FastWheel, FastWheel, FastWheel, FastWheel]
    received_at: float
    boot_id: int = 0
    host_session: int = 0
    capabilities: Capability = Capability(0)
    build_id: int = 0
    config_id: int = 0
    applied_seq: int = 0
    accepted_ms: int = 0
    applied_ms: int = 0
    stop_state: StopState = StopState.NONE
    requested_profile: Profile = Profile.GENTLE
    applied_profile: Profile = Profile.GENTLE
    config_result: ConfigResult = ConfigResult.NONE
    reason_flags: Reason = Reason(0)
    boost_remaining_ms: int = 0
    boost_capacity_ms: int = 0
    boost_refill_remaining_ms: int = 0
    cooldown_remaining_ms: int = 0
    hold_flags: HoldFlag = HoldFlag(0)
    fault_wheel: int = 0
    hello_nonce: int = 0
    config_ack_seq: int = 0

    @property
    def accepted_seq(self) -> int:
        return self.ack_seq

    @property
    def motion_enabled(self) -> bool:
        return (self.state in (HatState.ARMED, HatState.SETTLING, HatState.HOLDING)
                and not self.hold_flags & HoldFlag.DISARMED and self.fault_code == 0
                and self.stop_state == StopState.NONE)

    @property
    def stop_confirmed(self) -> bool:
        return self.stop_state == StopState.CONFIRMED

    def feedback_fresh(self, max_age_ms: int) -> bool:
        return all(w.speed_valid and w.current_valid and w.age_ms is not None
                   and w.age_ms <= max_age_ms for w in self.wheels)

    def temperature_fresh(self, max_age_ms: int) -> bool:
        return all(w.temperature_valid and w.temp_age_ms is not None
                   and w.temp_age_ms <= max_age_ms for w in self.wheels)

    def stationary(self, max_abs_rpm: float = 2, max_age_ms: int = 100) -> bool:
        return self.feedback_fresh(max_age_ms) and all(abs(w.rpm) <= max_abs_rpm for w in self.wheels)


def decode_status(payload: bytes, received_at: float | None = None) -> FastStatus:
    if len(payload) != STATUS_LENGTH:
        raise ValueError(f"STATUS payload is {len(payload)} bytes, expected {STATUS_LENGTH}")
    values = dict(zip(STATUS_HEADER_FIELDS, STATUS_HEADER_STRUCT.unpack_from(payload)))
    for field, enum in (("state", HatState), ("stop_state", StopState),
                        ("requested_profile", Profile), ("applied_profile", Profile),
                        ("config_result", ConfigResult), ("reason_flags", Reason),
                        ("capabilities", Capability), ("hold_flags", HoldFlag),
                        ("fault_code", FaultCode)):
        values[field] = enum(values[field])
    if values["fault_wheel"] > 4:
        raise ValueError("Invalid fault identity in STATUS")
    if values["boost_remaining_ms"] > values["boost_capacity_ms"]:
        raise ValueError("Invalid Boost budget in STATUS")
    records = []
    for index in range(4):
        raw = dict(zip(WHEEL_FIELDS, WHEEL_STRUCT.unpack_from(payload, STATUS_HEADER_STRUCT.size + index * WHEEL_STRUCT.size)))
        valid = Validity(raw.pop("validity"))
        if int(valid) & ~15:
            raise ValueError("Unknown wheel validity flags")
        temperature = raw["temp_c"] if valid & Validity.TEMPERATURE else None
        if temperature is not None and not -40 <= temperature <= 125:
            raise ValueError("Invalid motor temperature")
        if raw["effective_cap_ma"] > FIRMWARE_MAX_CURRENT_MA or raw["hold_cap_ma"] > FIRMWARE_MAX_CURRENT_MA:
            raise ValueError("Motor cap exceeds firmware hard ceiling")
        if abs(raw["target_centi_rpm"]) > 20000:
            raise ValueError("Invalid wheel target")
        raw["target_rpm"] = raw.pop("target_centi_rpm") / 100.0
        raw["temp_c"] = temperature
        raw["position_raw"] = raw["position_raw"] if valid & Validity.POSITION else None
        raw["age_ms"] = None if raw["age_ms"] == 65535 else raw["age_ms"]
        raw["temp_age_ms"] = None if raw["temp_age_ms"] == 65535 else raw["temp_age_ms"]
        raw["validity"] = valid
        raw["reason_flags"] = Reason(raw["reason_flags"])
        records.append(FastWheel(**raw))
    values["ack_seq"] = values.pop("accepted_seq")
    return FastStatus(**values, wheels=tuple(records),
                      received_at=time.monotonic() if received_at is None else received_at)


def _uint16(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 65535:
        raise ValueError(f"{name} must be an integer within 0..65535")
    return value


@dataclass(frozen=True)
class FastConfig:
    # Profiles are bounded by the independently enforced firmware ceiling.
    max_rpm: int = 40
    max_current_ma: int = 2700
    neutral_brake_ma: int = 300
    accel_rpm_s: int = 120
    decel_rpm_s: int = 180
    kp_ma_per_rpm: int = 20
    ki_ma_per_rpm_s: int = 4
    ff_ma_per_rpm_s: int = 0
    watchdog_ms: int = 300
    control_period_ms: int = 15
    stall_time_ms: int = 1000
    temp_limit_c: int = 65
    gentle_current_ma: int = 800
    normal_current_ma: int = 1500
    boost_current_ma: int = 2500
    boost_capacity_ms: int = 20000
    boost_refill_ms: int = 60000
    temp_poll_ms: int = 500
    temp_boost_stale_ms: int = 750
    temp_stop_stale_ms: int = 1500
    cooldown_ms: int = 3000
    cap_ramp_ma_s: int = 1000
    hold_current_ma: int = 300
    hold_kp_ma_per_degree: int = 2
    hold_ki_ma_per_degree_s: int = 1
    hold_damping_ma_per_rpm: int = 20
    neutral_settle_ms: int = 300
    feedback_timeout_ms: int = 150
    stall_target_centi_rpm: int = 800
    stall_speed_centi_rpm: int = 200
    stall_current_ma: int = 250
    abnormal_current_ma: int = 2700
    abnormal_current_ms: int = 200
    abnormal_margin_ma: int = 400
    saturation_warn_ms: int = 500
    stop_verify_ms: int = 1500
    temp_warn_c: int = 50
    temp_derate_c: int = 55
    temp_release_c: int = 45
    temp_hysteresis_c: int = 3
    hold_enabled: bool = True
    disarmed_hold_enabled: bool = False
    stall_enabled: bool = True
    hold_temp_c: int = 55
    encoder_counts_per_rev: int = 65536

    def payload(self) -> bytes:
        validate_config({name: getattr(self, name) for name in CONFIG_FIELDS})
        return CONFIG_STRUCT.pack(*(getattr(self, name) for name in CONFIG_FIELDS))

    @property
    def configuration_id(self) -> int:
        return zlib.crc32(self.payload()) or 1


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
        self._generation = 0
        self._stop_pending = True
        self._stop_barrier_sequence: int | None = None
        self._boot_id = 0
        self._host_session = 0
        self._config_id = 0
        self._config: FastConfig | None = None
        self._pending_hello_nonce: int | None = None
        self._identity_error: str | None = None
        self._status: FastStatus | None = None
        self._last_status_seq: int | None = None
        self._reader_error: str | None = None
        self._parser = FrameParser()
        self._sent_at: dict[int, float] = {}
        self._sent_kind: dict[int, FrameType] = {}
        self._sent_generation: dict[int, int] = {}
        self._sent_config: dict[int, FastConfig] = {}
        self._sent_nonce: dict[int, int] = {}
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
                    if self._pending_hello_nonce is not None:
                        # Only this HELLO's nonce/sequence can establish a new identity.
                        if (status.hello_nonce != self._pending_hello_nonce
                                or self._sent_kind.get(status.ack_seq) != FrameType.HELLO
                                or self._sent_nonce.get(status.ack_seq) != status.hello_nonce):
                            self._old_status += 1
                            continue
                        if not status.boot_id or not status.host_session:
                            self._bad_status += 1
                            continue
                        self._boot_id, self._host_session = status.boot_id, status.host_session
                        self._pending_hello_nonce = None
                        self._last_status_seq = None
                        self._identity_error = None
                    elif self._boot_id and (status.boot_id != self._boot_id
                                            or status.host_session != self._host_session):
                        self._identity_error = "HAT boot/session changed; HELLO and configuration are required"
                        self._stop_flag.set()
                        self._config_id = 0
                        self._condition.notify_all()
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
                    if status.fault_code or not status.motion_enabled:
                        self._stop_flag.set()
                    if status.stop_confirmed and status.ack_seq == self._stop_barrier_sequence:
                        self._stop_pending = False
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

    def _send(self, kind: FrameType, payload: bytes = b"", *, generation: int | None = None,
              config: FastConfig | None = None, nonce: int | None = None) -> int:
        with self._write_lock:
            if self._closed:
                raise FastHatError("HAT port is closed")
            with self._condition:
                if self._reader_error is not None and kind != FrameType.STOP:
                    raise FastHatError(f"HAT reader failed: {self._reader_error}")
                if kind in (FrameType.ARM, FrameType.TARGETS):
                    if generation != self._generation:
                        raise FastHatError("STOP superseded the pending motion command")
                    if self._identity_error:
                        raise FastHatError(self._identity_error)
                    if kind == FrameType.ARM and self._stop_pending:
                        raise FastHatError("STOP is pending; confirmed stop feedback is required before ARM")
            if kind == FrameType.TARGETS and self._stop_flag.is_set():
                raise FastHatError("HAT is stopped; ARM must be acknowledged before TARGETS")
            seq = self._next_seq
            self._next_seq = (seq + 1) & 0xFFFF
            frame = encode_frame(kind, seq, payload)
            sent_at = time.monotonic()
            with self._condition:
                self._sent_at[seq] = sent_at
                self._sent_kind[seq] = kind
                if kind in (FrameType.STOP, FrameType.HELLO):
                    self._stop_barrier_sequence = seq
                self._sent_generation[seq] = self._generation
                if config is not None:
                    self._sent_config[seq] = config
                if nonce is not None:
                    self._sent_nonce[seq] = nonce
                self._rtt_recorded.discard(seq)
                # Keep a short history to resolve waits without unbounded growth.
                if len(self._sent_at) > 512:
                    for old in tuple(self._sent_at)[:128]:
                        del self._sent_at[old]
                        self._sent_kind.pop(old, None)
                        self._sent_generation.pop(old, None)
                        self._sent_config.pop(old, None)
                        self._sent_nonce.pop(old, None)
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
        """Inhibit motion and request a fresh HAT-issued session token."""
        self._stop_flag.set()
        with self._condition:
            self._generation += 1
            self._stop_pending = True
            self._status = None
            self._config_id = 0
            self._config = None
            nonce = secrets.randbits(64) or 1
            self._pending_hello_nonce = nonce
        return self._send(FrameType.HELLO, struct.pack("<Q", nonce), nonce=nonce)

    def _session(self) -> tuple[int, int, int]:
        with self._condition:
            if self._pending_hello_nonce is not None or not self._boot_id or not self._host_session:
                raise FastHatError("HELLO must be acknowledged before session-bound commands")
            if self._identity_error:
                raise FastHatError(self._identity_error)
            return self._boot_id, self._host_session, self._generation

    def send_config(self, config: FastConfig | Mapping[str, int]) -> int:
        """Send a complete configuration while the HAT is disarmed."""
        if isinstance(config, Mapping):
            config = FastConfig(**config)
        if not isinstance(config, FastConfig):
            raise TypeError("config must be FastConfig or a field mapping")
        body = config.payload()
        boot, session, _ = self._session()
        status = self.snapshot()
        if status is None or status.motion_enabled or not status.stop_confirmed or not status.stationary(max_age_ms=config.feedback_timeout_ms):
            raise FastHatError("Configuration requires fresh confirmed stopped feedback")
        self._stop_flag.set()
        return self._send(FrameType.CONFIG, CONFIG_PREFIX.pack(boot, session, config.configuration_id) + body,
                          config=config)

    def arm(self) -> int:
        """Request arming; TARGETS stay blocked until this command is ACKed."""
        boot, session, generation = self._session()
        if not self._config_id:
            raise FastHatError("CONFIG must be freshly acknowledged before ARM")
        status = self.snapshot()
        if status is None or status.fault_code or not status.stop_confirmed or not status.stationary(
                max_age_ms=self._config.feedback_timeout_ms if self._config else 150):
            raise FastHatError("ARM requires fresh fault-free confirmed stopped feedback")
        return self._send(FrameType.ARM, ARM_STRUCT.pack(boot, session, self._config_id), generation=generation)

    def targets(self, rpm_by_id: Sequence[float], profile: Profile) -> int:
        """Send four RPM targets at 0.01 RPM precision and a requested profile."""
        if len(rpm_by_id) != 4:
            raise ValueError("Exactly four RPM targets are required")
        encoded = []
        max_rpm = self._config.max_rpm if self._config else 200
        for rpm in rpm_by_id:
            if (isinstance(rpm, bool) or not isinstance(rpm, (int, float))
                    or not math.isfinite(rpm) or abs(rpm) > max_rpm):
                raise ValueError(f"Each target RPM must be finite within +/-{max_rpm}")
            encoded.append(round(rpm * 100))
        if isinstance(profile, bool) or not isinstance(profile, (Profile, int)):
            raise ValueError("profile must be Gentle, Normal, or Boost")
        profile = Profile(profile)
        boot, session, generation = self._session()
        return self._send(FrameType.TARGETS, TARGETS_STRUCT.pack(boot, session, *encoded, profile), generation=generation)

    def stop(self) -> int:
        """Block new targets immediately, then send a disarming STOP frame."""
        self._stop_flag.set()
        with self._condition:
            # This is independent of the serial lock and invalidates pending ARM ACKs.
            self._generation += 1
            self._stop_pending = True
            self._condition.notify_all()
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
                if kind in (FrameType.ARM, FrameType.TARGETS) and self._sent_generation[sequence] != self._generation:
                    self._stop_flag.set()
                    raise FastHatError("STOP superseded the pending motion acknowledgment")
                if self._reader_error is not None:
                    self._stop_flag.set()
                    raise FastHatError(f"HAT reader failed: {self._reader_error}")
                if self._identity_error and kind not in (FrameType.HELLO, FrameType.STOP):
                    self._stop_flag.set()
                    raise FastHatError(self._identity_error)
                status = self._status
                if status is not None and status.received_at >= sent_at:
                    if status.fault_code and kind not in (FrameType.STOP, FrameType.HELLO):
                        self._stop_flag.set()
                        raise FastHatError(f"HAT fault {status.fault_code} while waiting for {kind.name}")
                    if status.ack_seq == sequence or (kind == FrameType.CONFIG and status.config_ack_seq == sequence):
                        if kind in (FrameType.ARM, FrameType.TARGETS) and not status.motion_enabled:
                            self._stop_flag.set()
                            raise FastHatError(f"HAT ACKed {kind.name} without entering ARMED state")
                        if kind == FrameType.HELLO:
                            if status.hello_nonce != self._sent_nonce.get(sequence):
                                raise FastHatError("HELLO nonce mismatch")
                            if status.capabilities & REQUIRED_CAPABILITIES != REQUIRED_CAPABILITIES:
                                raise FastHatError("HAT is missing required protocol v2 capabilities")
                        if kind == FrameType.CONFIG:
                            config = self._sent_config[sequence]
                            if status.config_result != ConfigResult.APPLIED:
                                self._stop_flag.set()
                                raise FastHatError(f"HAT rejected CONFIG ({status.config_result.name})")
                            if (status.config_ack_seq != sequence or status.motion_enabled or not status.stop_confirmed
                                    or status.applied_seq != sequence
                                    or status.config_id != config.configuration_id):
                                self._stop_flag.set()
                                raise FastHatError("HAT did not apply the exact stopped CONFIG")
                            self._config_id = config.configuration_id
                            self._config = config
                        if kind in (FrameType.HELLO, FrameType.STOP) and status.motion_enabled:
                            self._stop_flag.set()
                            raise FastHatError(f"HAT ACKed {kind.name} while still ARMED")
                        if kind == FrameType.ARM:
                            if status.applied_seq != sequence:
                                self._stop_flag.set()
                                raise FastHatError("HAT ARM acknowledgment does not confirm application")
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
