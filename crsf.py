"""Read ExpressLRS CRSF receiver frames from a serial port.

The normal ExpressLRS receiver output is non-inverted 420000 baud, 8N1.
Only validated 0x16 RC and 0x14 link-statistics frames update snapshots.
The reader never decides whether controls are fresh enough to drive motors;
callers must compare ``channels_at`` against ``time.monotonic()``.

CRSF framing: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
ExpressLRS UART default: https://www.expresslrs.org/quick-start/webui/
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


CRSF_SYNC = 0xC8
RC_CHANNELS_PACKED = 0x16
LINK_STATISTICS = 0x14
MAX_FRAME_SIZE = 64
RC_CHANNEL_COUNT = 16
RC_PAYLOAD_SIZE = 22
LINK_PAYLOAD_SIZE = 10
MAX_RECEIVE_BACKLOG = 256


def crc8(data: bytes) -> int:
    """CRSF CRC-8, polynomial 0xD5, initial value 0 (type + payload)."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 0xFF
    return crc


@dataclass(frozen=True)
class CRSFFrame:
    frame_type: int
    payload: bytes


class CRSFParser:
    """Incrementally parse a receiver's serial byte stream.

    ExpressLRS emits 0xC8 as its serial sync byte. A bad length or CRC is
    discarded one byte at a time so the next valid frame can resynchronize.
    ``feed`` returns only complete frames with valid framing and CRC.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[CRSFFrame]:
        self._buffer.extend(data)
        frames: list[CRSFFrame] = []
        while self._buffer:
            sync_index = self._buffer.find(CRSF_SYNC)
            if sync_index < 0:
                self._buffer.clear()
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 2:
                break
            length = self._buffer[1]
            if not 2 <= length <= MAX_FRAME_SIZE - 2:
                del self._buffer[0]
                continue
            total = length + 2
            if len(self._buffer) < total:
                break
            body = self._buffer[2 : total - 1]
            if crc8(body) != self._buffer[total - 1]:
                del self._buffer[0]
                continue
            frames.append(CRSFFrame(body[0], bytes(body[1:])))
            del self._buffer[:total]
        return frames


def decode_channels(payload: bytes) -> tuple[int, ...]:
    """Decode the first 22 bytes of a 0x16 payload to 16 raw 11-bit values.

    CRSF permits later protocol versions to append fields, so trailing
    payload bytes are ignored after a valid enclosing frame.
    """
    if len(payload) < RC_PAYLOAD_SIZE:
        raise ValueError("RC frame needs 22 payload bytes")
    packed = int.from_bytes(payload[:RC_PAYLOAD_SIZE], "little")
    return tuple((packed >> (11 * index)) & 0x7FF for index in range(RC_CHANNEL_COUNT))


def _signed_byte(value: int) -> int:
    return value - 256 if value >= 128 else value


@dataclass(frozen=True)
class LinkStatistics:
    uplink_rssi_1: int
    uplink_rssi_2: int
    uplink_link_quality: int
    uplink_snr: int
    active_antenna: int
    rf_mode: int
    uplink_rf_power: int
    downlink_rssi: int
    downlink_link_quality: int
    downlink_snr: int


def decode_link_statistics(payload: bytes) -> LinkStatistics:
    """Decode the first ten fields of a 0x14 link statistics payload."""
    if len(payload) < LINK_PAYLOAD_SIZE:
        raise ValueError("link statistics frame needs 10 payload bytes")
    return LinkStatistics(
        uplink_rssi_1=payload[0],
        uplink_rssi_2=payload[1],
        uplink_link_quality=payload[2],
        uplink_snr=_signed_byte(payload[3]),
        active_antenna=payload[4],
        rf_mode=payload[5],
        uplink_rf_power=payload[6],
        downlink_rssi=payload[7],
        downlink_link_quality=payload[8],
        downlink_snr=_signed_byte(payload[9]),
    )


@dataclass(frozen=True)
class CRSFSnapshot:
    """Latest validated values; absent timestamps mean no such frame arrived."""

    channels: tuple[int, ...] | None = None
    channels_at: float | None = None
    link_quality: int | None = None
    link_at: float | None = None
    error: str | None = None


class CRSFReader:
    """Background serial reader. Use ``start``/``close`` or a context manager.

    ``snapshot()`` is safe to call from the control loop. Its timestamps use
    ``time.monotonic()``. ``serial_port`` allows an already-open serial-like
    object to be supplied for tests. Closing the reader closes that object.
    """

    def __init__(
        self,
        port: str | None,
        baudrate: int = 420000,
        *,
        serial_port: Any = None,
    ) -> None:
        if not isinstance(baudrate, int) or isinstance(baudrate, bool) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if not port and serial_port is None:
            raise ValueError("port is required unless serial_port is supplied")
        self.port = port
        self.baudrate = baudrate
        self._serial = serial_port
        self._parser = CRSFParser()
        self._lock = threading.Lock()
        self._snapshot = CRSFSnapshot()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False

    def start(self) -> CRSFReader:
        if self._closed:
            raise RuntimeError("CRSF reader is closed")
        if self._thread is not None:
            return self
        if self._serial is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Install pyserial: python -m pip install pyserial") from exc
            self._serial = serial.Serial(
                self.port, baudrate=self.baudrate, bytesize=8, parity="N", stopbits=1,
                timeout=0.01,
            )
        self._thread = threading.Thread(target=self._run, name="crsf-reader", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        serial_port = self._serial
        try:
            while not self._stop.is_set():
                # A paused reader can otherwise stamp old kernel-buffered RC
                # frames as newly arrived. Drop a large backlog and wait for
                # genuinely new frames from the receiver.
                waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
                if waiting > MAX_RECEIVE_BACKLOG:
                    serial_port.reset_input_buffer()
                    self._parser.reset()
                    with self._lock:
                        self._snapshot = CRSFSnapshot(error="Receiver input backlog discarded")
                    continue
                # pyserial read(64) waits until all 64 bytes arrive or its
                # timeout expires. A normal RC frame is only 26 bytes, so
                # that needlessly delays every freshly received command.
                # Block for one byte when idle, then drain what is available
                # without waiting for another complete buffer.
                data = serial_port.read(min(64, max(1, waiting)))
                if not data:
                    continue
                for frame in self._parser.feed(data):
                    now = time.monotonic()
                    if frame.frame_type == RC_CHANNELS_PACKED:
                        try:
                            channels = decode_channels(frame.payload)
                        except ValueError:
                            continue
                        with self._lock:
                            self._snapshot = CRSFSnapshot(
                                channels=channels,
                                channels_at=now,
                                link_quality=self._snapshot.link_quality,
                                link_at=self._snapshot.link_at,
                                error=None,
                            )
                    elif frame.frame_type == LINK_STATISTICS:
                        try:
                            stats = decode_link_statistics(frame.payload)
                        except ValueError:
                            continue
                        with self._lock:
                            self._snapshot = CRSFSnapshot(
                                channels=self._snapshot.channels,
                                channels_at=self._snapshot.channels_at,
                                link_quality=stats.uplink_link_quality,
                                link_at=now,
                                error=None,
                            )
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    current = self._snapshot
                    self._snapshot = CRSFSnapshot(
                        current.channels, current.channels_at,
                        current.link_quality, current.link_at,
                        f"{type(exc).__name__}: {exc}",
                    )
        finally:
            self._stop.set()

    def snapshot(self) -> CRSFSnapshot:
        with self._lock:
            return self._snapshot

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._serial is not None:
            self._serial.close()
        if self._thread is not None and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)

    def __enter__(self) -> CRSFReader:
        return self.start()

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()
