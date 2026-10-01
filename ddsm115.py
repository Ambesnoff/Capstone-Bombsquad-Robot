"""Python control of DDSM115 motors through Waveshare DDSM Driver HAT (A).

Connect through the Pi's 40-pin GPIO UART or the HAT's ESP32-USB port, with
the HAT switch at ESP32. This module sends newline-terminated JSON at
115200 baud, NOT raw RS485 packets.
The optional DDSM115Raw class instead uses DDSM-USB with the switch at USB
and supports all documented motor packets, including the electric brake.
Requires Python 3.9+ and pyserial: python -m pip install pyserial

All 20 command types in Waveshare's published ddsm_example firmware are
wrapped below. Firmware reference: 79bfd6f21f9f8f15c231797184a3a93bf881c8a7
https://github.com/waveshareteam/ddsm_example
The robot setup and test procedure is in README.md.

Opening a connection selects DDSM115 decoding but does not command motion.
Use one controlling program; do not also control the HAT through its web UI.
Software/transport tests passed; real hardware validation is still required.
"""

from __future__ import annotations

import json
import math
import re
import threading
import time
import warnings
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Union


class HatError(RuntimeError):
    """Communication, firmware response, or motor-state error."""


class FeedbackTimeout(HatError):
    """No matching reply arrived. No motion command is automatically retried."""


class MotorFault(HatError):
    """A motor returned nonzero error bits; feedback is attached."""

    def __init__(self, feedback: "Feedback"):
        self.feedback = feedback
        super().__init__(f"Motor {feedback.motor_id}: error 0x{feedback.error:02X}")


class Mode(IntEnum):
    CURRENT = 1
    SPEED = 2
    POSITION = 3


class WiFiMode(IntEnum):
    OFF = 0
    AP = 1
    STATION = 2
    AP_STATION = 3


class Command(IntEnum):
    """Every incoming command type handled by the referenced HAT firmware."""

    ZERO_SETPOINT = 10000
    CONTROL = 10010
    SET_ID = 10011
    SET_MODE = 10012
    GET_ID = 10031
    GET_FEEDBACK = 10032
    HEARTBEAT = 11001
    MOTOR_TYPE = 11002
    WIFI_BOOT_MODE = 10401
    WIFI_AP = 10402
    WIFI_STATION = 10403
    WIFI_AP_STATION = 10404
    WIFI_INFO = 10405
    WIFI_SAVE_CURRENT = 10406
    WIFI_SAVE_CONFIG = 10407
    WIFI_DISCONNECT = 10408
    REBOOT = 600
    FREE_FLASH = 601
    RESET_WIFI = 603
    CLEAR_NVS = 604


def _integer(value: int, name: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return int(value)


def motor_id(value: int) -> int:
    """Validate an addressed motor ID; 200 is reserved here for ID discovery."""
    value = _integer(value, "motor ID", 1, 253)
    if value == 200:
        raise ValueError("ID 200 is used by the motor's single-motor ID query")
    return value


def _seconds(value: float, name: str, allow_zero: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
        raise ValueError(f"{name} must be {'nonnegative' if allow_zero else 'positive'}")
    return value


@dataclass(frozen=True)
class Feedback:
    """One DDSM115 report. Position counts are retained without guessing scale."""

    motor_id: int
    mode: Mode
    rpm: int
    torque_current_raw: int
    error: int
    temperature_c: Optional[int] = None
    position_raw: Optional[int] = None
    position_u8: Optional[int] = None
    raw: Dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    @property
    def torque_current_a_estimate(self) -> float:
        """Documented count-to-current estimate; NOT battery/input current."""
        return self.torque_current_raw * 8.0 / 32767.0

    @property
    def position_degrees_coarse(self) -> Optional[float]:
        """Convert query's documented 0..255 position scale to 0..360 degrees."""
        return None if self.position_u8 is None else self.position_u8 * 360.0 / 255.0

    @property
    def error_bits(self) -> tuple:
        """Indices of the set fault bits; see the guide for known meanings."""
        return tuple(bit for bit in range(8) if self.error & (1 << bit))

    @classmethod
    def from_json(cls, data: Mapping[str, Any]) -> "Feedback":
        try:
            if data.get("T") != 20010 or data.get("typ") != 115:
                raise ValueError("Expected DDSM115 motor feedback")
            mid = motor_id(data["id"])
            mode = Mode(_integer(data["mode"], "mode", 1, 3))
            rpm = _integer(data["spd"], "feedback rpm", -32768, 32767)
            current = _integer(data["tor"], "torque-current count", -32768, 32767)
            error = _integer(data["err"], "error", 0, 255)
            temp = _integer(data["temp"], "temperature", 0, 255) if "temp" in data else None
            pos = _integer(data["pos"], "position", 0, 65535) if "pos" in data else None
            u8 = _integer(data["u8"], "coarse position", 0, 255) if "u8" in data else None
            if not ((temp is not None and u8 is not None) or pos is not None):
                raise ValueError("Missing position/temperature fields")
            return cls(mid, mode, rpm, current, error, temp, pos, u8, dict(data))
        except (KeyError, TypeError, ValueError) as exc:
            raise HatError("Malformed DDSM115 feedback") from exc


Event = Union[dict, str]


class DDSMHat:
    """Synchronous JSON driver for the ESP32 interface of DDSM Driver HAT (A).

    port: a serial device such as '/dev/serial0', '/dev/ttyUSB0', or 'COM5'.
    timeout: deadline for each expected reply, in seconds (default 0.5).
    startup_delay: let the ESP32 finish booting after serial opens.
    stop_on_close: best-effort zero-speed commands for motors commanded here.
    serial_port: injectable serial-compatible object, mainly for testing.

    No background keepalive is started. A timed-out transaction faults this
    connection: use emergency_stop/close, make the mechanism safe, then reopen.
    """

    def __init__(self, port: Optional[str] = None, *, timeout: float = 0.5,
                 startup_delay: float = 2.0, stop_on_close: bool = True,
                 serial_port: Any = None):
        self.timeout = _seconds(timeout, "timeout")
        startup_delay = _seconds(startup_delay, "startup_delay", True)
        self.stop_on_close = stop_on_close
        self._lock = threading.RLock()
        self._rx = bytearray()
        self._last_write = 0.0
        self._modes: Dict[int, Mode] = {}
        self._active_ids = set()
        self._heartbeat_ms: Optional[int] = None
        self._motor_type = 115
        self._faulted = False
        self._closed = False
        self.last_stop_errors = []
        if serial_port is None:
            if not port:
                raise ValueError("Supply a serial port, for example /dev/serial0 or COM5")
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Install pyserial: python -m pip install pyserial") from exc
            # Set control lines before opening to reduce unwanted ESP32 resets.
            self.serial = serial.Serial(port=None, baudrate=115200, bytesize=8,
                                        parity="N", stopbits=1, timeout=0.02,
                                        write_timeout=self.timeout)
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.port = port
            self.serial.open()
        else:
            self.serial = serial_port
            self.serial.timeout = min(0.02, self.timeout)
            self.serial.write_timeout = self.timeout
        try:
            time.sleep(startup_delay)
            self.serial.reset_input_buffer()
            self.set_motor_type(115)
        except BaseException:
            self.serial.close()
            self._closed = True
            raise

    def _ready(self) -> None:
        if self._closed:
            raise HatError("Connection is closed")
        if self._faulted:
            raise HatError("Connection faulted; stop/close, make the mechanism safe, then reconnect")

    def _require_115(self) -> None:
        if self._motor_type != 115:
            raise HatError("DDSM115 helpers require set_motor_type(115)")

    def _write(self, payload: Mapping[str, Any]) -> None:
        if self._closed:
            raise HatError("Connection is closed")
        data = (json.dumps(dict(payload), separators=(",", ":"),
                           ensure_ascii=True, allow_nan=False) + "\n").encode("ascii")
        # Do not enqueue multiple commands together: the firmware clears its
        # motor RX buffer at the beginning of each JSON command handler.
        gap = 0.015 - (time.monotonic() - self._last_write)
        if gap > 0:
            time.sleep(gap)
        try:
            count = self.serial.write(data)
            self._last_write = time.monotonic()
            if count != len(data):
                raise HatError("Incomplete serial write")
        except Exception as exc:
            self._faulted = True
            if isinstance(exc, HatError):
                raise
            raise HatError("Serial write failed") from exc

    def _pop_event(self) -> Optional[Event]:
        """Parse mixed diagnostics/JSON, including objects without a newline."""
        while self._rx[:1] in (b"\r", b"\n"):
            del self._rx[:1]
        if not self._rx:
            return None
        if self._rx[0] == ord("{"):
            depth, quoted, escaped = 0, False, False
            for index, value in enumerate(self._rx):
                if quoted:
                    if escaped:
                        escaped = False
                    elif value == 92:
                        escaped = True
                    elif value == 34:
                        quoted = False
                elif value == 34:
                    quoted = True
                elif value == 123:
                    depth += 1
                elif value == 125:
                    depth -= 1
                    if depth == 0:
                        packet = bytes(self._rx[:index + 1])
                        del self._rx[:index + 1]
                        try:
                            return json.loads(packet.decode("utf-8"))
                        except (ValueError, UnicodeError):
                            return packet.decode("utf-8", errors="replace")
                elif value == 10:
                    # A broken one-line object is diagnostic text, not feedback.
                    packet = bytes(self._rx[:index + 1])
                    del self._rx[:index + 1]
                    return packet.decode("utf-8", errors="replace").strip()
            return None
        ends = [i for i in (self._rx.find(b"\n"), self._rx.find(b"{")) if i >= 0]
        if not ends:
            return None
        end = min(ends)
        packet = bytes(self._rx[:end])
        del self._rx[:end]
        return packet.decode("utf-8", errors="replace").strip()

    def _event_until(self, deadline: float) -> Optional[Event]:
        while time.monotonic() < deadline:
            event = self._pop_event()
            if event is not None:
                if event == "":
                    continue
                return event
            try:
                waiting = getattr(self.serial, "in_waiting", 0)
                chunk = self.serial.read(max(1, min(waiting, 4096)))
            except Exception as exc:
                self._faulted = True
                raise HatError("Serial read failed") from exc
            self._rx.extend(chunk)
            if len(self._rx) > 16384:
                self._faulted = True
                raise HatError("Unframed serial data; check ESP32 port, switch, and firmware")
        return None

    def _request(self, payload: Mapping[str, Any], match: Callable[[Event], bool],
                 timeout: Optional[float] = None) -> Event:
        self._ready()
        timeout = self.timeout if timeout is None else _seconds(timeout, "timeout")
        # Old bytes cannot satisfy a new request. Late replies after a failed
        # request are handled by faulting the session, not retrying motion.
        try:
            self.serial.reset_input_buffer()
        except Exception as exc:
            self._faulted = True
            raise HatError("Could not clear the serial input") from exc
        self._rx.clear()
        self._write(payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            event = self._event_until(deadline)
            if event is None:
                break
            if isinstance(event, dict) and event.get("crc") == 0:
                self._faulted = True
                raise HatError("HAT reported a motor-packet CRC error")
            if match(event):
                return event
        self._faulted = True
        raise FeedbackTimeout(f"No matching reply for command {payload['T']}; no retry was sent")

    def _motor_request(self, payload: dict, mid: Optional[int], *, info: bool,
                       require_mode: Optional[Mode] = None) -> Feedback:
        self._require_115()
        def match(event: Event) -> bool:
            return (isinstance(event, dict) and event.get("T") == 20010
                    and (mid is None or event.get("id") == mid)
                    and (("temp" in event and "u8" in event) if info else "pos" in event))
        data = self._request(payload, match)
        try:
            fb = Feedback.from_json(data)
            if require_mode is not None and fb.mode != require_mode:
                raise HatError("Returned motor mode differs from the requested mode")
        except HatError:
            self._faulted = True
            raise
        self._modes[fb.motor_id] = fb.mode
        return fb

    def get_feedback(self, mid: int) -> Feedback:
        """T=10032. Read mode, RPM, current counts, temperature, position, errors."""
        with self._lock:
            mid = motor_id(mid)
            return self._motor_request({"T": Command.GET_FEEDBACK, "id": mid}, mid, info=True)

    def get_id(self) -> Feedback:
        """T=10031. Discover ONE connected motor; do not call on a multi-motor bus."""
        with self._lock:
            return self._motor_request({"T": Command.GET_ID}, None, info=False)

    def set_id(self, new_id: int) -> Feedback:
        """T=10011 then T=10031. ONE motor only; persistent, once per power cycle."""
        with self._lock:
            self._ready()
            self._require_115()
            new_id = motor_id(new_id)
            self._write({"T": Command.SET_ID, "id": new_id})
            time.sleep(0.08)  # Firmware sends five raw ID-setting packets.
            self._modes.clear()
            fb = self.get_id()
            if fb.motor_id != new_id:
                self._faulted = True
                raise HatError("ID change was not verified; power-cycle the single motor and recheck")
            return fb

    def set_mode(self, mid: int, mode: Union[Mode, int]) -> Feedback:
        """T=10012. Select current/speed/position, then verify with feedback.

        Already-selected modes are not reselected (position entry can re-zero).
        Position mode requires <10 RPM and explicitly disabled HAT heartbeat.
        """
        with self._lock:
            mid = motor_id(mid)
            mode = Mode(_integer(mode, "mode", 1, 3))
            if mode == Mode.POSITION and self._heartbeat_ms != -1:
                raise HatError("Call set_heartbeat(-1) before position mode; stock watchdog sends position zero")
            fb = self.get_feedback(mid)
            if fb.error:
                raise MotorFault(fb)
            if fb.mode == mode:
                return fb
            if mode == Mode.POSITION and abs(fb.rpm) >= 10:
                raise HatError("Stop the motor before entering position mode (<10 RPM required)")
            self._active_ids.add(mid)
            self._write({"T": Command.SET_MODE, "id": mid, "mode": int(mode)})
            fb = self.get_feedback(mid)
            if fb.mode != mode:
                self._faulted = True
                raise HatError("Motor mode change was not verified")
            if fb.error:
                raise MotorFault(fb)
            return fb

    def control(self, mid: int, value: int, acceleration: int = 100) -> Feedback:
        """T=10010. Send a setpoint in the CURRENT mode; units depend on mode."""
        with self._lock:
            self._ready()
            self._require_115()
            mid = motor_id(mid)
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            mode = self._modes.get(mid)
            if mode is None:
                fb = self.get_feedback(mid)
                if fb.error:
                    raise MotorFault(fb)
                mode = fb.mode
            limits = {Mode.SPEED: (-330, 330), Mode.CURRENT: (-32767, 32767),
                      Mode.POSITION: (0, 32767)}
            value = _integer(value, "setpoint", *limits[mode])
            if mode == Mode.POSITION and self._heartbeat_ms != -1:
                raise HatError("Call set_heartbeat(-1) before position commands")
            self._active_ids.add(mid)
            fb = self._motor_request({"T": Command.CONTROL, "id": mid, "cmd": value,
                                      "act": acceleration}, mid, info=False, require_mode=mode)
            if fb.error:
                raise MotorFault(fb)
            return fb

    def _ensure_mode(self, mid: int, mode: Mode) -> None:
        if self._modes.get(mid) != mode:
            self.set_mode(mid, mode)

    def set_speed(self, mid: int, rpm: int, acceleration: int = 100) -> Feedback:
        """Select speed mode as needed and request signed integer RPM (-330..330)."""
        with self._lock:
            mid = motor_id(mid)
            rpm = _integer(rpm, "rpm", -330, 330)
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            self._ensure_mode(mid, Mode.SPEED)
            return self.control(mid, rpm, acceleration)

    def set_speeds(self, speeds: Mapping[int, int], acceleration: int = 100) -> dict:
        """Command several motors sequentially; failure attempts to stop all of them."""
        with self._lock:
            checked = {motor_id(mid): _integer(rpm, "rpm", -330, 330)
                       for mid, rpm in speeds.items()}
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            try:
                return {mid: self.set_speed(mid, rpm, acceleration) for mid, rpm in checked.items()}
            except BaseException:
                self.emergency_stop(checked)
                raise

    def set_current_raw(self, mid: int, counts: int) -> Feedback:
        """Select current mode; signed counts -32767..32767 (encoded +/-8 A scale)."""
        with self._lock:
            mid = motor_id(mid)
            counts = _integer(counts, "current counts", -32767, 32767)
            self._ensure_mode(mid, Mode.CURRENT)
            return self.control(mid, counts, 0)

    def set_position(self, mid: int, degrees: float) -> Feedback:
        """Select position mode; target 0..360 degrees, single-turn shortest path."""
        with self._lock:
            mid = motor_id(mid)
            degrees = _seconds(degrees, "degrees", True)
            if degrees > 360:
                raise ValueError("Position must be between 0 and 360 degrees")
            if self._heartbeat_ms != -1:
                raise HatError("Call set_heartbeat(-1) before position commands")
            self._ensure_mode(mid, Mode.POSITION)
            return self.control(mid, round(degrees * 32767 / 360), 0)

    def zero_setpoint(self, mid: int) -> Feedback:
        """T=10000 exactly: zero RPM, zero current, OR move to position zero."""
        with self._lock:
            mid = motor_id(mid)
            self._active_ids.add(mid)
            fb = self._motor_request({"T": Command.ZERO_SETPOINT, "id": mid}, mid, info=False)
            if fb.error:
                raise MotorFault(fb)
            return fb

    def stop(self, mid: int) -> Feedback:
        """Select SPEED then send T=10000. This is zero-speed control, NOT a brake bit."""
        with self._lock:
            self._ready()
            self._require_115()
            mid = motor_id(mid)
            self._active_ids.add(mid)
            self._write({"T": Command.SET_MODE, "id": mid, "mode": int(Mode.SPEED)})
            fb = self._motor_request({"T": Command.ZERO_SETPOINT, "id": mid}, mid,
                                     info=False, require_mode=Mode.SPEED)
            if fb.error:
                raise MotorFault(fb)
            return fb

    def emergency_stop(self, ids: Optional[Iterable[int]] = None) -> list:
        """Best-effort zero-speed writes, even after a timeout. Returns write errors.

        Sends mode 2 then zero to each ID twice, without awaiting feedback.
        This invalidates the session; reconnect before further control.
        It does not cut power, set the electric brake bit, or prove a stop.
        """
        with self._lock:
            mids = sorted(self._active_ids) if ids is None else list(dict.fromkeys(motor_id(i) for i in ids))
            failures = []
            for _ in range(2):
                for mid in mids:
                    for payload in ({"T": Command.SET_MODE, "id": mid, "mode": 2},
                                    {"T": Command.ZERO_SETPOINT, "id": mid}):
                        try:
                            self._write(payload)
                        except Exception as exc:
                            failures.append(f"Motor {mid}: {type(exc).__name__}")
            self._faulted = True
            self.last_stop_errors = failures
            return failures

    def set_heartbeat(self, milliseconds: int) -> None:
        """T=11001. -1 disables; positive ms enables stock watchdog for IDs 1..4."""
        with self._lock:
            milliseconds = _integer(milliseconds, "heartbeat milliseconds", -1, 2147483647)
            if milliseconds == 0:
                raise ValueError("Use -1 to disable or a positive number of milliseconds")
            if milliseconds > 0 and Mode.POSITION in self._modes.values():
                raise HatError("Leave position mode before enabling the stock heartbeat")
            self._ready()
            self._write({"T": Command.HEARTBEAT, "time": milliseconds})
            self._heartbeat_ms = milliseconds

    def set_motor_type(self, model: int = 115) -> None:
        """T=11002. HAT-wide decoder selector: 115 or 210. Helpers target 115 only."""
        with self._lock:
            model = _integer(model, "model", 115, 210)
            if model not in (115, 210):
                raise ValueError("This firmware supports model selectors 115 and 210")
            self._ready()
            if self._active_ids:
                raise HatError("Close/stop the current session before changing motor type")
            self._write({"T": Command.MOTOR_TYPE, "type": model})
            self._motor_type = model
            self._modes.clear()

    def read_messages(self, seconds: float = 0.2) -> list:
        """Collect JSON dictionaries and diagnostic strings without sending anything.

        Wi-Fi replies can contain passwords; nothing is automatically printed.
        A complete JSON object is accepted even if firmware omits its newline.
        """
        with self._lock:
            if self._closed:
                raise HatError("Connection is closed")
            deadline = time.monotonic() + _seconds(seconds, "seconds")
            events = []
            while time.monotonic() < deadline:
                event = self._event_until(deadline)
                if event is None:
                    break
                events.append(event)
            return events

    def send_command(self, payload: Mapping[str, Any], *, collect_seconds: float = 0.2) -> list:
        """Advanced: send any JSON command, then collect replies without interpreting.

        Return values are received messages, not proof of execution. This clears
        mode/watchdog assumptions. Use only when you understand the command.
        Raw motor writes are tracked for best-effort stopping on close.
        """
        with self._lock:
            self._ready()
            payload = dict(payload)
            _integer(payload.get("T"), "T", 0, 2147483647)
            collect_seconds = _seconds(collect_seconds, "collect_seconds")
            if payload["T"] in (10000, 10010, 10012):
                self._active_ids.add(motor_id(payload.get("id")))
            self._modes.clear()
            self._heartbeat_ms = None
            if payload["T"] == 11002:
                self._motor_type = payload.get("type")
            self._write(payload)
            return self.read_messages(collect_seconds)

    def _admin(self, command: Command, **fields: Any) -> None:
        self._ready()
        if self._active_ids:
            raise HatError("Use a fresh, stationary session for Wi-Fi/system settings")
        self._write({"T": command, **fields})

    @staticmethod
    def _network(ssid: str, password: str, *, ap: bool) -> None:
        if not isinstance(ssid, str) or not 1 <= len(ssid.encode("utf-8")) <= 32:
            raise ValueError("SSID must contain 1..32 UTF-8 bytes")
        if not isinstance(password, str) or len(password.encode("utf-8")) > 63:
            raise ValueError("Password must be a string of at most 63 UTF-8 bytes")
        if ap and password and len(password.encode("utf-8")) < 8:
            raise ValueError("An AP password must be empty or at least 8 bytes")

    def set_wifi_boot_mode(self, mode: Union[WiFiMode, int]) -> None:
        """T=10401. Save boot Wi-Fi mode: off/AP/station/both = 0/1/2/3."""
        with self._lock:
            self._admin(Command.WIFI_BOOT_MODE, cmd=_integer(mode, "Wi-Fi mode", 0, 3))

    def wifi_ap(self, ssid: str, password: str) -> None:
        """T=10402. Configure the HAT's hotspot; no structured acknowledgement."""
        with self._lock:
            self._network(ssid, password, ap=True)
            self._admin(Command.WIFI_AP, ssid=ssid, password=password)

    def wifi_connect(self, ssid: str, password: str) -> None:
        """T=10403. Start joining a Wi-Fi network; connection can take seconds."""
        with self._lock:
            self._network(ssid, password, ap=False)
            self._admin(Command.WIFI_STATION, ssid=ssid, password=password)

    def wifi_ap_station(self, ap_ssid: str, ap_password: str,
                        sta_ssid: str, sta_password: str) -> None:
        """T=10404. Configure a hotspot and join an existing network together."""
        with self._lock:
            self._network(ap_ssid, ap_password, ap=True)
            self._network(sta_ssid, sta_password, ap=False)
            self._admin(Command.WIFI_AP_STATION, ap_ssid=ap_ssid, ap_password=ap_password,
                        sta_ssid=sta_ssid, sta_password=sta_password)

    def get_wifi_info(self) -> dict:
        """T=10405. Return Wi-Fi info; the returned object can contain passwords."""
        with self._lock:
            return self._request({"T": Command.WIFI_INFO},
                                 lambda e: isinstance(e, dict) and "ip" in e and "rssi" in e)

    def save_wifi_current(self) -> None:
        """T=10406. Save current Wi-Fi settings into the HAT's wifiConfig.json."""
        with self._lock:
            self._admin(Command.WIFI_SAVE_CURRENT)

    def save_wifi_config(self, mode: Union[WiFiMode, int], ap_ssid: str, ap_password: str,
                         sta_ssid: str, sta_password: str) -> None:
        """T=10407. Apply AP+station settings and save the supplied boot mode."""
        with self._lock:
            self._network(ap_ssid, ap_password, ap=True)
            self._network(sta_ssid, sta_password, ap=False)
            self._admin(Command.WIFI_SAVE_CONFIG, mode=_integer(mode, "Wi-Fi mode", 0, 3),
                        ap_ssid=ap_ssid, ap_password=ap_password,
                        sta_ssid=sta_ssid, sta_password=sta_password)

    def wifi_disconnect(self) -> None:
        """T=10408. Disconnect station Wi-Fi; stock firmware need not disable AP/radio."""
        with self._lock:
            self._admin(Command.WIFI_DISCONNECT)

    def reboot(self) -> None:
        """T=600. Restart ESP32, then close this host connection. Does not cut motor power."""
        with self._lock:
            self._admin(Command.REBOOT)
            self.close()

    def get_free_flash(self) -> int:
        """T=601. Parse the stock serial text response; return free LittleFS bytes."""
        with self._lock:
            pattern = re.compile(r"free flash memory:\s*(\d+)\s*bytes", re.I)
            event = self._request({"T": Command.FREE_FLASH},
                                 lambda e: (isinstance(e, str) and bool(pattern.search(e)))
                                 or (isinstance(e, dict) and "free" in e and "total" in e))
            return int(pattern.search(event).group(1)) if isinstance(event, str) else int(event["free"])

    def reset_wifi_settings(self) -> None:
        """T=603. Delete saved wifiConfig.json; does not reflash firmware or motor IDs."""
        with self._lock:
            self._admin(Command.RESET_WIFI)

    def clear_nvs(self) -> None:
        """T=604. Erase ESP32 NVS configuration; separate from deleting wifiConfig.json."""
        with self._lock:
            self._admin(Command.CLEAR_NVS)

    def close(self) -> list:
        """Attempt to stop motors commanded here, close serial, return stop-write errors."""
        with self._lock:
            if self._closed:
                return self.last_stop_errors
            try:
                if self.stop_on_close and self._active_ids:
                    self.emergency_stop()
            finally:
                self.serial.close()
                self._closed = True
            return self.last_stop_errors

    def __enter__(self) -> "DDSMHat":
        self._ready()
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        errors = self.close()
        if errors:
            warnings.warn("Stop writes failed; use the physical power cutoff", RuntimeWarning)
        return False


class DDSM115Raw:
    """Direct motor protocol through the HAT's DDSM-USB port, switch at USB.

    Use this alternative when the actual electric-brake bit is needed. The
    ESP32 JSON/Wi-Fi/watchdog commands are unavailable on this connection.
    The five documented raw protocol families and all three modes are covered.
    """

    def __init__(self, port: Optional[str] = None, *, timeout: float = 0.15,
                 stop_on_close: bool = True, serial_port: Any = None):
        self.timeout = _seconds(timeout, "timeout")
        self.stop_on_close = stop_on_close
        self._lock = threading.RLock()
        self._modes: Dict[int, Mode] = {}
        self._active_ids = set()
        self._closed = self._faulted = False
        self.last_stop_errors = []
        if serial_port is None:
            if not port:
                raise ValueError("Supply the HAT's DDSM-USB serial port")
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Install pyserial: python -m pip install pyserial") from exc
            self.serial = serial.Serial(port, 115200, bytesize=8, parity="N", stopbits=1,
                                        timeout=0.01, write_timeout=self.timeout)
        else:
            self.serial = serial_port
            self.serial.timeout = min(0.01, self.timeout)
            self.serial.write_timeout = self.timeout
        try:
            time.sleep(0.05)
            self.serial.reset_input_buffer()
        except BaseException:
            self.serial.close()
            self._closed = True
            raise

    @staticmethod
    def _crc8(data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
        return crc

    @classmethod
    def _frame(cls, data: bytes) -> bytes:
        if len(data) != 9:
            raise ValueError("Expected nine bytes before the CRC")
        return data + bytes([cls._crc8(data)])

    def _ready(self) -> None:
        if self._closed or self._faulted:
            raise HatError("Raw connection closed/faulted; stop, make safe, and reconnect")

    def _send(self, frame: bytes) -> None:
        if self._closed:
            raise HatError("Raw connection is closed")
        try:
            if self.serial.write(frame) != len(frame):
                raise HatError("Incomplete raw serial write")
        except Exception as exc:
            self._faulted = True
            raise HatError("Raw serial write failed") from exc

    def _exchange(self, frame: bytes, mid: Optional[int], *, info: bool = False) -> Feedback:
        self._ready()
        try:
            self.serial.reset_input_buffer()
            self._send(frame)
            deadline = time.monotonic() + self.timeout
            pending = bytearray()
            while time.monotonic() < deadline:
                pending.extend(self.serial.read(10 - len(pending)))
                if len(pending) != 10:
                    continue
                response = bytes(pending)
                if response == frame:  # Some USB adapters echo the command.
                    pending.clear()
                    continue
                if self._crc8(response[:9]) != response[9]:
                    raise HatError("Raw feedback CRC mismatch")
                if mid is not None and response[0] != mid:
                    raise HatError("Raw feedback motor ID mismatch")
                data = {"T": 20010, "typ": 115, "id": response[0], "mode": response[1],
                        "tor": int.from_bytes(response[2:4], "big", signed=True),
                        "spd": int.from_bytes(response[4:6], "big", signed=True),
                        "err": response[8]}
                if info:
                    data.update(temp=response[6], u8=response[7])
                else:
                    data["pos"] = int.from_bytes(response[6:8], "big")
                fb = Feedback.from_json(data)
                self._modes[fb.motor_id] = fb.mode
                return fb
            raise FeedbackTimeout("Raw feedback timeout; no motion retry was sent")
        except Exception as exc:
            self._faulted = True
            if isinstance(exc, HatError):
                raise
            raise HatError("Raw serial exchange failed") from exc

    def get_feedback(self, mid: int) -> Feedback:
        """0x74: temperature and coarse position together with mode/RPM/current/errors."""
        with self._lock:
            mid = motor_id(mid)
            return self._exchange(self._frame(bytes([mid, 0x74]) + bytes(7)), mid, info=True)

    def get_id(self) -> Feedback:
        """0xC8/0x64: find the ONE connected motor's ID."""
        with self._lock:
            return self._exchange(self._frame(bytes([200, 0x64]) + bytes(7)), None)

    def set_id(self, new_id: int) -> Feedback:
        """AA 55 53: set ONE motor's persistent ID and verify; once per power cycle."""
        with self._lock:
            self._ready()
            new_id = motor_id(new_id)
            frame = self._frame(bytes([0xAA, 0x55, 0x53, new_id]) + bytes(5))
            for _ in range(5):
                self._send(frame)
                time.sleep(0.01)
            time.sleep(0.05)
            self._modes.clear()
            fb = self.get_id()
            if fb.motor_id != new_id:
                self._faulted = True
                raise HatError("Raw ID change unverified; power-cycle the single motor and recheck")
            return fb

    def set_mode(self, mid: int, mode: Union[Mode, int]) -> Feedback:
        """0xA0: select current=1/speed=2/position=3; mode byte replaces the CRC."""
        with self._lock:
            mid = motor_id(mid)
            mode = Mode(_integer(mode, "mode", 1, 3))
            fb = self.get_feedback(mid)
            if fb.error:
                raise MotorFault(fb)
            if fb.mode == mode:
                return fb
            if mode == Mode.POSITION and abs(fb.rpm) >= 10:
                raise HatError("Stop the motor before entering position mode (<10 RPM required)")
            self._active_ids.add(mid)
            self._send(bytes([mid, 0xA0]) + bytes(7) + bytes([mode]))
            time.sleep(0.005)
            fb = self.get_feedback(mid)
            if fb.mode != mode:
                self._faulted = True
                raise HatError("Raw mode change was not verified")
            if fb.error:
                raise MotorFault(fb)
            return fb

    def control(self, mid: int, value: int, acceleration: int = 100,
                *, brake: bool = False) -> Feedback:
        """0x64: mode-dependent setpoint, acceleration byte, and electric-brake bit."""
        with self._lock:
            self._ready()
            mid = motor_id(mid)
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            if not isinstance(brake, bool):
                raise ValueError("brake must be True or False")
            mode = self._modes.get(mid)
            if mode is None:
                fb = self.get_feedback(mid)
                if fb.error:
                    raise MotorFault(fb)
                mode = fb.mode
            if brake and mode != Mode.SPEED:
                raise HatError("The electric brake bit requires speed mode")
            bounds = {Mode.SPEED: (-330, 330), Mode.CURRENT: (-32767, 32767),
                      Mode.POSITION: (0, 32767)}
            value = _integer(value, "setpoint", *bounds[mode])
            frame = self._frame(bytes([mid, 0x64]) + value.to_bytes(2, "big", signed=True)
                                + bytes([0, 0, acceleration, 255 if brake else 0, 0]))
            self._active_ids.add(mid)
            fb = self._exchange(frame, mid)
            if fb.mode != mode:
                self._faulted = True
                raise HatError("Unexpected raw feedback mode")
            if fb.error:
                raise MotorFault(fb)
            return fb

    def _ensure_mode(self, mid: int, mode: Mode) -> None:
        if self._modes.get(mid) != mode:
            self.set_mode(mid, mode)

    def set_speed(self, mid: int, rpm: int, acceleration: int = 100) -> Feedback:
        """Select speed mode as needed; signed RPM also sets rotation direction."""
        with self._lock:
            mid = motor_id(mid)
            rpm = _integer(rpm, "rpm", -330, 330)
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            self._ensure_mode(mid, Mode.SPEED)
            return self.control(mid, rpm, acceleration)

    def set_speeds(self, speeds: Mapping[int, int], acceleration: int = 100) -> dict:
        """Command motors sequentially, with a stop attempt for all on failure."""
        with self._lock:
            checked = {motor_id(mid): _integer(rpm, "rpm", -330, 330) for mid, rpm in speeds.items()}
            acceleration = _integer(acceleration, "acceleration", 0, 255)
            try:
                return {mid: self.set_speed(mid, rpm, acceleration) for mid, rpm in checked.items()}
            except BaseException:
                self.emergency_stop(checked)
                raise

    def set_current_raw(self, mid: int, counts: int) -> Feedback:
        """Select current mode; use signed protocol counts (-32767..32767)."""
        with self._lock:
            mid = motor_id(mid)
            counts = _integer(counts, "current counts", -32767, 32767)
            self._ensure_mode(mid, Mode.CURRENT)
            return self.control(mid, counts, 0)

    def set_position(self, mid: int, degrees: float) -> Feedback:
        """Select position mode; 0..360-degree single-turn shortest-path target."""
        with self._lock:
            mid = motor_id(mid)
            degrees = _seconds(degrees, "degrees", True)
            if degrees > 360:
                raise ValueError("Position must be between 0 and 360 degrees")
            self._ensure_mode(mid, Mode.POSITION)
            return self.control(mid, round(degrees * 32767 / 360), 0)

    def stop(self, mid: int) -> Feedback:
        """Select speed mode and request zero RPM without the electric-brake flag."""
        return self.set_speed(mid, 0, 1)

    def brake(self, mid: int) -> Feedback:
        """Select speed mode and send the actual 0xFF electric-brake flag."""
        with self._lock:
            mid = motor_id(mid)
            self._ensure_mode(mid, Mode.SPEED)
            return self.control(mid, 0, 1, brake=True)

    def emergency_stop(self, ids: Optional[Iterable[int]] = None) -> list:
        """Best-effort mode-2 + zero/brake writes; fault this session afterwards."""
        with self._lock:
            mids = sorted(self._active_ids) if ids is None else list(dict.fromkeys(motor_id(i) for i in ids))
            errors = []
            for _ in range(2):
                for mid in mids:
                    frames = (bytes([mid, 0xA0]) + bytes(7) + b"\x02",
                              self._frame(bytes([mid, 0x64, 0, 0, 0, 0, 1, 255, 0])))
                    for frame in frames:
                        try:
                            self._send(frame)
                        except Exception as exc:
                            errors.append(f"Motor {mid}: {type(exc).__name__}")
                        time.sleep(0.01)
            self._faulted = True
            self.last_stop_errors = errors
            return errors

    def close(self) -> list:
        """Attempt zero/brake for commanded motors and close the serial connection."""
        with self._lock:
            if not self._closed:
                try:
                    if self.stop_on_close and self._active_ids:
                        self.emergency_stop()
                finally:
                    self.serial.close()
                    self._closed = True
            return self.last_stop_errors

    def __enter__(self) -> "DDSM115Raw":
        self._ready()
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if self.close():
            warnings.warn("Stop writes failed; use the physical power cutoff", RuntimeWarning)
        return False


if __name__ == "__main__":
    print("Import DDSMHat from this file. See README.md for robot setup.\n"
          "No serial connection was opened and no motor commands were sent.")
