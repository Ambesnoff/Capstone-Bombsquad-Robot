"""Fast HAT link framing and failure behavior, with no motor hardware."""

import struct
import threading
import time
import unittest

from fast_hat import (
    FastConfig, FastHat, FastHatError, FrameParser, FrameType, HatState,
    crc16_ccitt_false, decode_status, encode_frame,
)


def status_payload(ack: int, state: HatState = HatState.DISARMED,
                   *, wheel_age: int = 4, fault: int = 0) -> bytes:
    header = struct.pack("<HBBHH", ack, state, fault, 18000, 8)
    wheels = b"".join(struct.pack("<hhbBB", rpm, current, 25, 0, wheel_age)
                      for rpm, current in ((-8, -180), (8, 180), (8, 180), (-8, -180)))
    return header + wheels


class FakeSerial:
    def __init__(self, *, auto_ack=False):
        self.timeout = 0.01
        self.write_timeout = 0.15
        self.auto_ack = auto_ack
        self.writes: list[bytes] = []
        self._input = bytearray()
        self._condition = threading.Condition()
        self.closed = False
        self.read_error = False
        self.status_sequence = 41

    @property
    def in_waiting(self):
        with self._condition:
            return len(self._input)

    def reset_input_buffer(self):
        with self._condition:
            self._input.clear()

    def inject(self, data: bytes):
        with self._condition:
            self._input.extend(data)
            self._condition.notify_all()

    def read(self, size: int):
        with self._condition:
            self._condition.wait_for(lambda: self._input or self.closed or self.read_error,
                                     timeout=self.timeout)
            if self.read_error:
                raise OSError("receiver unplugged")
            if not self._input:
                return b""
            chunk = bytes(self._input[:size])
            del self._input[:size]
            return chunk

    def write(self, data: bytes):
        self.writes.append(data)
        if self.auto_ack:
            frame = FrameParser().feed(data)[0]
            state = HatState.ARMED if frame.kind in (FrameType.ARM, FrameType.TARGETS) else HatState.DISARMED
            self.inject(encode_frame(FrameType.STATUS, self.status_sequence,
                                     status_payload(frame.sequence, state)))
            self.status_sequence = (self.status_sequence + 1) & 0xFFFF
        return len(data)

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()


class FrameTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_fragmented_corrupt_and_noisy_frames_resynchronize(self):
        parser = FrameParser()
        good = encode_frame(FrameType.HELLO, 314)
        damaged = bytearray(encode_frame(FrameType.STATUS_REQ, 7))
        damaged[-1] ^= 0x80
        invalid_length = b"\xa5\x5a\x01\x80\x01\x00\xff"
        self.assertEqual(parser.feed(b"noise\xa5"), [])
        self.assertEqual(parser.feed(good[1:5]), [])
        self.assertEqual(parser.feed(good[5:]), [FrameParser().feed(good)[0]])
        self.assertEqual(parser.feed(bytes(damaged) + invalid_length + good),
                         [FrameParser().feed(good)[0]])
        self.assertEqual(parser.bad_crc, 1)
        self.assertEqual(parser.bad_length, 1)
        self.assertGreater(parser.discarded_bytes, 0)

    def test_status_fields_and_unavailable_markers(self):
        payload = bytearray(status_payload(22, HatState.ARMED, wheel_age=255))
        payload[8 + 4] = 127  # first wheel temperature
        result = decode_status(bytes(payload), received_at=10.0)
        self.assertEqual(result.ack_seq, 22)
        self.assertEqual(result.state, HatState.ARMED)
        self.assertEqual(result.wheels[0].rpm, -8)
        self.assertEqual(result.wheels[0].current_ma, -180)
        self.assertIsNone(result.wheels[0].temp_c)
        self.assertIsNone(result.wheels[0].age_ms)
        self.assertFalse(result.stationary())
        self.assertEqual(result.received_at, 10.0)


class FastHatTests(unittest.TestCase):
    def test_invalid_config_is_rejected_before_serial_write(self):
        fake = FakeSerial()
        with FastHat("test", serial_port=fake) as hat:
            invalid = FastConfig(201, 1000, 300, 120, 180, 20, 4, 1, 400, 20, 900, 65)
            with self.assertRaisesRegex(ValueError, "max_rpm"):
                hat.send_config(invalid)
            self.assertEqual(fake.writes, [])

    def test_config_and_targets_wire_layout_with_ack(self):
        fake = FakeSerial(auto_ack=True)
        with FastHat("test", timeout=0.2, serial_port=fake) as hat:
            self.assertEqual(fake.writes, [])
            hello = hat.hello()
            self.assertEqual(hat.wait_ack(hello).state, HatState.DISARMED)
            config = FastConfig(40, 1000, 300, 120, 180, 20, 4, 1, 400, 20, 900, 65)
            config_seq = hat.send_config(config)
            self.assertEqual(hat.wait_ack(config_seq).ack_seq, config_seq)
            config_frame = FrameParser().feed(fake.writes[-1])[0]
            self.assertEqual(config_frame.kind, FrameType.CONFIG)
            self.assertEqual(config_frame.payload, struct.pack("<11HB", 40, 1000, 300, 120,
                                                               180, 20, 4, 1, 400, 20, 900, 65))
            with self.assertRaises(FastHatError):
                hat.targets((-10, 10, 10, -10), 300)
            arm_seq = hat.arm()
            self.assertEqual(hat.wait_ack(arm_seq).state, HatState.ARMED)
            target_seq = hat.targets((-10, 10, 10, -10), 300)
            self.assertEqual(hat.wait_ack(target_seq).state, HatState.ARMED)
            target_frame = FrameParser().feed(fake.writes[-1])[0]
            self.assertEqual(target_frame.payload, struct.pack("<hhhhH", -10, 10, 10, -10, 300))
            self.assertIsNotNone(hat.stats().last_rtt_ms)
            stop_seq = hat.stop()
            self.assertEqual(hat.wait_ack(stop_seq).state, HatState.DISARMED)
            with self.assertRaises(FastHatError):
                hat.targets((0, 0, 0, 0), 0)
        self.assertTrue(fake.closed)
        self.assertEqual(FrameParser().feed(fake.writes[-1])[0].kind, FrameType.STOP)

    def test_bad_status_does_not_satisfy_ack(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.08, serial_port=fake) as hat:
            sequence = hat.hello()
            fake.inject(encode_frame(FrameType.STATUS, 1, b"invalid"))
            with self.assertRaisesRegex(FastHatError, "No HAT ACK"):
                hat.wait_ack(sequence)
            self.assertEqual(hat.stats().bad_status, 1)

    def test_arm_ack_requires_armed_state(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.1, serial_port=fake) as hat:
            sequence = hat.arm()
            fake.inject(encode_frame(FrameType.STATUS, 5, status_payload(sequence, HatState.DISARMED)))
            with self.assertRaisesRegex(FastHatError, "without entering ARMED"):
                hat.wait_ack(sequence)
            with self.assertRaises(FastHatError):
                hat.targets((1, 1, 1, 1), 100)

    def test_corrupt_status_is_ignored_then_fresh_status_accepted(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.2, serial_port=fake) as hat:
            sequence = hat.hello()
            bad = bytearray(encode_frame(FrameType.STATUS, 1, status_payload(sequence)))
            bad[-1] ^= 1
            fake.inject(bytes(bad) + encode_frame(FrameType.STATUS, 2, status_payload(sequence)))
            status = hat.wait_ack(sequence)
            self.assertEqual(status.ack_seq, sequence)
            self.assertEqual(hat.stats().bad_crc, 1)

    def test_duplicate_or_old_status_does_not_refresh_feedback(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.2, serial_port=fake) as hat:
            sequence = hat.hello()
            fake.inject(encode_frame(FrameType.STATUS, 100, status_payload(sequence)))
            first = hat.wait_ack(sequence)
            fake.inject(encode_frame(FrameType.STATUS, 100, status_payload(sequence)) +
                        encode_frame(FrameType.STATUS, 99, status_payload(sequence)))
            deadline = time.monotonic() + 0.2
            while hat.stats().old_status < 2 and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertEqual(hat.stats().old_status, 2)
            self.assertEqual(hat.snapshot().received_at, first.received_at)

    def test_status_interval_metrics_measure_accepted_reports(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.2, serial_port=fake) as hat:
            sequence = hat.hello()
            fake.inject(encode_frame(FrameType.STATUS, 100, status_payload(sequence)))
            hat.wait_ack(sequence)
            time.sleep(0.005)
            fake.inject(encode_frame(FrameType.STATUS, 101, status_payload(sequence)))
            deadline = time.monotonic() + 0.2
            while hat.stats().status_frames < 2 and time.monotonic() < deadline:
                time.sleep(0.001)
            stats = hat.stats()
            self.assertEqual(stats.status_frames, 2)
            self.assertIsNotNone(stats.last_status_interval_ms)
            self.assertGreater(stats.last_status_interval_ms, 0)
            self.assertEqual(stats.last_status_interval_ms, stats.mean_status_interval_ms)
            self.assertEqual(stats.last_status_interval_ms, stats.max_status_interval_ms)

    def test_hello_can_restart_status_freshness_after_hat_reboot(self):
        fake = FakeSerial(auto_ack=True)
        with FastHat("test", timeout=0.2, serial_port=fake) as hat:
            first = hat.hello()
            self.assertEqual(hat.wait_ack(first).ack_seq, first)
            fake.status_sequence = 0  # ESP32 restarted while the Pi remained open.
            second = hat.hello()
            self.assertEqual(hat.wait_ack(second).ack_seq, second)

    def test_reader_failure_still_allows_best_effort_stop(self):
        fake = FakeSerial()
        with FastHat("test", timeout=0.1, serial_port=fake) as hat:
            fake.read_error = True
            fake.inject(b"x")
            deadline = time.monotonic() + 0.2
            while hat.stats().reader_error is None and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertIsNotNone(hat.stats().reader_error)
            with self.assertRaises(FastHatError):
                hat.hello()
            seq = hat.stop()
            self.assertEqual(FrameParser().feed(fake.writes[-1])[0].sequence, seq)

    def test_large_status_backlog_faults_instead_of_retimestamping_old_feedback(self):
        class BacklogSerial(FakeSerial):
            overfull = False

            @property
            def in_waiting(self):
                return 300 if self.overfull else super().in_waiting

        fake = BacklogSerial()
        with FastHat("test", timeout=0.1, serial_port=fake) as hat:
            fake.overfull = True
            deadline = time.monotonic() + 0.2
            while hat.stats().reader_error is None and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertIn("backlog", hat.stats().reader_error)
            hat.stop()


if __name__ == "__main__":
    unittest.main()
