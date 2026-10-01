"""Host and HAT integration behavior without serial hardware."""

import csv
import json
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from crsf import CRSFSnapshot
from fast_hat import FastHatError, FastStatus, FastWheel, HatState
from fast_robot import AsyncCsvTelemetry, FastRobot
from robot_main import Settings


CONFIG = Path(__file__).resolve().parents[1] / "config.example.json"


def settings() -> Settings:
    data = json.loads(CONFIG.read_text())
    data["motor_port"] = "/dev/test-hat"
    data["radio_port"] = "/dev/test-radio"
    return Settings.from_dict(data)


def radio_frame(*, arm=172, throttle=172, stop=172, lq=80) -> CRSFSnapshot:
    values = [992] * 16
    values[0] = 992
    values[2] = throttle
    values[4] = arm
    values[5] = 172
    values[6] = stop
    values[9] = 1811
    now = time.monotonic()
    return CRSFSnapshot(tuple(values), now, lq, now, None)


class PhaseRadio:
    def __init__(self, phases):
        self.phases = phases
        self.index = 0
        self.history = []

    def snapshot(self):
        self.history.append(self.index)
        return radio_frame(**self.phases[self.index])


class FakeFastHat:
    def __init__(self):
        self.commands = []
        self.sequence = 0
        self.state = HatState.DISARMED
        self.fault = 0
        self.ages = (0, 0, 0, 0)
        self.temps = (25, 25, 25, 25)
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def _send(self, name, *values):
        self.sequence += 1
        self.commands.append((name, *values))
        return self.sequence

    def hello(self):
        self.state = HatState.DISARMED
        return self._send("hello")

    def send_config(self, config):
        self.config = config
        return self._send("config")

    def arm(self):
        self.state = HatState.ARMED
        return self._send("arm")

    def targets(self, rpm, cap):
        return self._send("targets", tuple(rpm), cap)

    def stop(self):
        self.state = HatState.DISARMED
        self.ages = (0, 0, 0, 0)
        return self._send("stop")

    def wait_ack(self, seq, timeout=None):
        assert seq == self.sequence
        return self.snapshot()

    def snapshot(self):
        wheels = tuple(FastWheel(0, 0, self.temps[i], 0, self.ages[i]) for i in range(4))
        return FastStatus(self.sequence, self.state, self.fault, 12000, 0,
                          wheels, time.monotonic())

    def stats(self):
        return SimpleNamespace(last_rtt_ms=4.0, max_rtt_ms=5.0,
                               mean_rtt_ms=3.0, last_status_interval_ms=20.1,
                               max_status_interval_ms=22.0, mean_status_interval_ms=20.0,
                               status_frames=4, tx_frames=4,
                               bad_crc=0, bad_status=0, old_status=0,
                               discarded_bytes=0, reader_error=None)


class FastRobotTests(unittest.TestCase):
    def test_telemetry_writer_bounds_disk_backlog_without_blocking_control(self):
        writing = threading.Event()
        release = threading.Event()

        class SlowWriter:
            def writerow(self, row):
                if row and row[0] != "wall_time":
                    writing.set()
                    release.wait(1.0)

        with tempfile.TemporaryDirectory() as tmp:
            telemetry = AsyncCsvTelemetry(Path(tmp) / "data.csv", max_rows=1)
            with patch("fast_robot.csv.writer", return_value=SlowWriter()):
                telemetry.start()
                try:
                    telemetry.submit([1])
                    self.assertTrue(writing.wait(0.5))
                    telemetry.submit([2])
                    before = time.monotonic()
                    with self.assertRaisesRegex(RuntimeError, "backlog is full"):
                        telemetry.submit([3])
                    self.assertLess(time.monotonic() - before, 0.1)
                finally:
                    release.set()
                    telemetry.close()

    def test_telemetry_writer_failure_is_visible_to_control(self):
        class FailingWriter:
            def writerow(self, row):
                if row and row[0] != "wall_time":
                    raise OSError("disk disconnected")

        with tempfile.TemporaryDirectory() as tmp:
            telemetry = AsyncCsvTelemetry(Path(tmp) / "data.csv", max_rows=1)
            with patch("fast_robot.csv.writer", return_value=FailingWriter()):
                telemetry.start()
                telemetry.submit([1])
                self.assertTrue(telemetry._done.wait(0.5))
                with self.assertRaisesRegex(RuntimeError, "disk disconnected"):
                    telemetry.check()
                with self.assertRaisesRegex(RuntimeError, "disk disconnected"):
                    telemetry.close()

    def test_four_wheel_targets_stop_and_telemetry(self):
        phases = [
            {"arm": 172},
            {"arm": 1811},
            {"arm": 1811, "throttle": 1811},
            {"arm": 1811, "stop": 1811},
            {"arm": 1811},
            {"arm": 1811, "throttle": 1811},
        ]
        radio = PhaseRadio(phases)
        hat = FakeFastHat()
        with tempfile.TemporaryDirectory() as tmp:
            cfg = settings()
            robot = FastRobot(replace(cfg, wheels=tuple(reversed(cfg.wheels))), radio,
                              telemetry_path=Path(tmp) / "data.csv")
            steps = 0

            def advance(_duration):
                nonlocal steps
                steps += 1
                if steps > 20:
                    raise AssertionError(f"Host did not finish expected phases: {hat.commands}")
                if radio.index == len(phases) - 1:
                    robot.request_shutdown()
                else:
                    radio.index += 1

            with patch("fast_robot.FastHat", return_value=hat), patch("fast_robot.time.sleep", side_effect=advance):
                robot.run()
            with (Path(tmp) / "data.csv").open(newline="") as file:
                rows = list(csv.DictReader(file))

        self.assertEqual([cmd[0] for cmd in hat.commands].count("arm"), 1,
                         f"commands={hat.commands}, radio phases={radio.history}")
        self.assertIn(("targets", (-40, 40, 40, -40), 1000), hat.commands)
        self.assertGreaterEqual([cmd[0] for cmd in hat.commands].count("stop"), 2)
        self.assertTrue(hat.closed)
        self.assertEqual(rows[2]["requested_rpm_1"], "-40")
        self.assertIn("hat_sweep_us", rows[2])
        self.assertIn("hat_last_rtt_ms", rows[2])
        self.assertIn("hat_status_interval_ms", rows[2])
        self.assertIn("pi_control_interval_ms", rows[2])

    def test_missing_motor_feedback_stops_host_and_throws(self):
        radio = PhaseRadio([
            {"arm": 172}, {"arm": 1811},
            {"arm": 1811, "throttle": 1811},
            {"arm": 1811, "throttle": 1811},
        ])
        hat = FakeFastHat()
        original_targets = hat.targets

        def lose_feedback(rpm, cap):
            seq = original_targets(rpm, cap)
            hat.ages = (None, 0, 0, 0)
            return seq

        hat.targets = lose_feedback
        with tempfile.TemporaryDirectory() as tmp:
            robot = FastRobot(settings(), radio, telemetry_path=Path(tmp) / "data.csv")
            steps = 0

            def advance(_duration):
                nonlocal steps
                steps += 1
                if steps > 20:
                    raise AssertionError(f"Host did not detect stale feedback: {hat.commands}")
                radio.index = min(radio.index + 1, len(radio.phases) - 1)

            with patch("fast_robot.FastHat", return_value=hat), patch("fast_robot.time.sleep", side_effect=advance):
                with self.assertRaisesRegex(RuntimeError, "Motor 1 feedback became stale"):
                    robot.run()
        self.assertIn("stop", [cmd[0] for cmd in hat.commands])
        self.assertTrue(hat.closed)

    def test_link_loss_stops_and_requires_new_arm_cycle(self):
        phases = [
            {"arm": 172}, {"arm": 1811},
            {"arm": 1811, "throttle": 1811},
            {"arm": 1811, "lq": 0},
            {"arm": 1811, "throttle": 1811},
            {"arm": 172}, {"arm": 1811},
            {"arm": 1811, "throttle": 1811},
        ]
        radio = PhaseRadio(phases)
        hat = FakeFastHat()
        with tempfile.TemporaryDirectory() as tmp:
            robot = FastRobot(settings(), radio, telemetry_path=Path(tmp) / "data.csv")
            steps = 0

            def advance(_duration):
                nonlocal steps
                steps += 1
                if steps > 20:
                    raise AssertionError(f"Host did not finish expected phases: {hat.commands}")
                if radio.index == len(phases) - 1:
                    robot.request_shutdown()
                else:
                    radio.index += 1

            with patch("fast_robot.FastHat", return_value=hat), patch("fast_robot.time.sleep", side_effect=advance):
                robot.run()

        kinds = [command[0] for command in hat.commands]
        self.assertEqual(kinds.count("arm"), 2)
        self.assertEqual(kinds.count("targets"), 2)
        self.assertGreaterEqual(kinds.count("stop"), 2)
        first_target = kinds.index("targets")
        second_target = kinds.index("targets", first_target + 1)
        self.assertIn("stop", kinds[first_target + 1:second_target])

    def test_startup_retries_hello_after_hat_boot_delay(self):
        radio = PhaseRadio([{"arm": 172}])
        hat = FakeFastHat()
        original_ack = hat.wait_ack
        first = True

        def wait_ack(seq, timeout=None):
            nonlocal first
            if first and hat.commands[-1][0] == "hello":
                first = False
                raise FastHatError("No HAT ACK for HELLO")
            return original_ack(seq, timeout)

        hat.wait_ack = wait_ack
        with tempfile.TemporaryDirectory() as tmp:
            robot = FastRobot(settings(), radio, telemetry_path=Path(tmp) / "data.csv")
            sleeps = 0

            def advance(_duration):
                nonlocal sleeps
                sleeps += 1
                if sleeps >= 2:
                    robot.request_shutdown()

            with patch("fast_robot.FastHat", return_value=hat), patch("fast_robot.time.sleep", side_effect=advance):
                robot.run()
        kinds = [command[0] for command in hat.commands]
        self.assertEqual(kinds.count("hello"), 2)
        self.assertEqual(kinds.count("config"), 1)
        self.assertIn("stop", kinds)


if __name__ == "__main__":
    unittest.main()
