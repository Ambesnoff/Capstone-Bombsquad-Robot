"""Focused CRSF framing, decoding, and reader tests using fixed wire frames."""

from __future__ import annotations

import queue
import threading
import time
import unittest

from crsf import (
    CRSFParser,
    CRSFReader,
    decode_channels,
    decode_link_statistics,
    crc8,
)


# 16 channels with alternating standard low, center, and high values.
RC_VALUES = (
    172, 992, 1811, 992, 172, 1811, 172, 992,
    1811, 172, 992, 1811, 172, 992, 1811, 992,
)
RC_FRAME = bytes.fromhex(
    "c8 18 16 ac 00 df c4 c1 c7 8a 89 b3 02 7c 13 67 05 f8 26 ce 0a f0 4d 1c 7c ce"
)
LINK_FRAME = bytes.fromhex("c8 0c 14 50 51 41 f9 01 03 04 5a 19 02 a2")


class FakeSerial:
    def __init__(self) -> None:
        self.chunks: queue.Queue[bytes | Exception] = queue.Queue()
        self.closed = False

    def read(self, size: int) -> bytes:
        try:
            item = self.chunks.get(timeout=0.01)
        except queue.Empty:
            return b""
        if isinstance(item, Exception):
            raise item
        return item

    def close(self) -> None:
        self.closed = True


class BacklogSerial:
    def __init__(self) -> None:
        self.buffer = bytearray(RC_FRAME * 12)
        self.reset_count = 0
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return len(self.buffer)

    def reset_input_buffer(self) -> None:
        self.buffer.clear()
        self.reset_count += 1

    def read(self, size: int) -> bytes:
        if not self.buffer:
            time.sleep(0.005)
            return b""
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self) -> None:
        self.closed = True


class StrictSizeSerial:
    """Like pyserial, wait for the requested count instead of returning early."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.sizes: list[int] = []
        self.condition = threading.Condition()
        self.closed = False

    @property
    def in_waiting(self) -> int:
        with self.condition:
            return len(self.buffer)

    def inject(self, data: bytes) -> None:
        with self.condition:
            self.buffer.extend(data)
            self.condition.notify_all()

    def read(self, size: int) -> bytes:
        with self.condition:
            self.sizes.append(size)
            self.condition.wait_for(lambda: len(self.buffer) >= size or self.closed, timeout=0.01)
            chunk = bytes(self.buffer[:size])
            del self.buffer[:size]
            return chunk

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class CRSFTests(unittest.TestCase):
    def test_fixed_rc_frame_crc_and_channel_values(self) -> None:
        self.assertEqual(len(RC_FRAME), 26)
        self.assertEqual(crc8(RC_FRAME[2:-1]), RC_FRAME[-1])
        frames = CRSFParser().feed(RC_FRAME)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].frame_type, 0x16)
        self.assertEqual(decode_channels(frames[0].payload), RC_VALUES)

    def test_newer_rc_frame_with_extra_field(self) -> None:
        # The CRSF spec permits trailing fields beyond a known payload.
        extended = bytes.fromhex(
            "c8 19 16 ac 00 df c4 c1 c7 8a 89 b3 02 7c 13 67 05 f8 26 ce 0a f0 4d 1c 7c 99 9b"
        )
        frame = CRSFParser().feed(extended)[0]
        self.assertEqual(decode_channels(frame.payload), RC_VALUES)

    def test_fragmentation_noise_and_corrupt_crc_resynchronize(self) -> None:
        parser = CRSFParser()
        bad = bytearray(RC_FRAME)
        bad[-1] ^= 1
        self.assertEqual(parser.feed(b"\x55\xff" + RC_FRAME[:7]), [])
        self.assertEqual(parser.feed(RC_FRAME[7:] + bad + LINK_FRAME[:4])[0].frame_type, 0x16)
        frames = parser.feed(LINK_FRAME[4:])
        self.assertEqual([frame.frame_type for frame in frames], [0x14])

    def test_invalid_length_is_rejected_and_next_frame_found(self) -> None:
        parser = CRSFParser()
        frames = parser.feed(bytes.fromhex("c8 00 c8 3f") + RC_FRAME)
        self.assertEqual([frame.frame_type for frame in frames], [0x16])

    def test_link_statistics_and_short_payloads(self) -> None:
        frame = CRSFParser().feed(LINK_FRAME)[0]
        stats = decode_link_statistics(frame.payload)
        self.assertEqual(stats.uplink_rssi_1, 80)
        self.assertEqual(stats.uplink_link_quality, 65)
        self.assertEqual(stats.uplink_snr, -7)
        self.assertEqual(stats.downlink_link_quality, 25)
        with self.assertRaises(ValueError):
            decode_channels(b"\x00" * 21)
        with self.assertRaises(ValueError):
            decode_link_statistics(b"\x00" * 9)

    def test_reader_exposes_monotonic_snapshots(self) -> None:
        serial_port = FakeSerial()
        before = time.monotonic()
        with CRSFReader(None, serial_port=serial_port) as reader:
            serial_port.chunks.put(RC_FRAME[:9])
            serial_port.chunks.put(RC_FRAME[9:] + LINK_FRAME)
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                snapshot = reader.snapshot()
                if snapshot.channels is not None and snapshot.link_quality is not None:
                    break
                time.sleep(0.005)
            else:
                self.fail("reader did not receive both test frames")
            self.assertEqual(snapshot.channels, RC_VALUES)
            self.assertEqual(snapshot.link_quality, 65)
            self.assertGreaterEqual(snapshot.channels_at, before)
            self.assertGreaterEqual(snapshot.link_at, snapshot.channels_at)
            self.assertIsNone(snapshot.error)
        self.assertTrue(serial_port.closed)

    def test_reader_does_not_wait_for_64_bytes_to_publish_rc_frame(self) -> None:
        serial_port = StrictSizeSerial()
        with CRSFReader(None, serial_port=serial_port) as reader:
            serial_port.inject(RC_FRAME + LINK_FRAME)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                snapshot = reader.snapshot()
                if snapshot.channels is not None and snapshot.link_quality == 65:
                    break
                time.sleep(0.001)
            else:
                self.fail("reader waited for more bytes than the completed frames")
        self.assertNotIn(64, serial_port.sizes)

    def test_reader_reports_serial_error(self) -> None:
        serial_port = FakeSerial()
        with CRSFReader(None, serial_port=serial_port) as reader:
            serial_port.chunks.put(OSError("port unplugged"))
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline and reader.snapshot().error is None:
                time.sleep(0.005)
            self.assertIn("port unplugged", reader.snapshot().error or "")

    def test_reader_discards_old_serial_backlog_before_accepting_new_frames(self) -> None:
        serial_port = BacklogSerial()
        with CRSFReader(None, serial_port=serial_port) as reader:
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline and serial_port.reset_count == 0:
                time.sleep(0.005)
            self.assertEqual(serial_port.reset_count, 1)
            self.assertIsNone(reader.snapshot().channels)
            serial_port.buffer.extend(RC_FRAME + LINK_FRAME)
            while time.monotonic() < deadline:
                if reader.snapshot().channels is not None and reader.snapshot().link_quality == 65:
                    break
                time.sleep(0.005)
            else:
                self.fail("new RC and link frames were not accepted after dropping backlog")


if __name__ == "__main__":
    unittest.main()
