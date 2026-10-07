"""Pivot turning (zero throttle, stick deflected) must reach the motors through the real drive loops.

The mixing maths is covered elsewhere; these tests run FastRobot.run() and the legacy Robot.run()
against fake HATs and check what is actually commanded at the motor boundary.
Wheel polarity is applied only there, so expected values are computed from the settings:
logical +p * max_rpm on the left side and -p * max_rpm on the right side, times each wheel's polarity.
"""
import json
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from crsf import CRSFSnapshot
from ddsm115 import Feedback, Mode
from fast_hat import FastStatus, FastWheel, REQUIRED_CAPABILITIES
from fast_robot import FastRobot
from protocol_defs import ConfigResult, HatState, Profile, Reason, StopState
from robot_main import Robot, Settings

ROOT = Path(__file__).resolve().parents[1]
FAST_CONFIG = ROOT / "config.example.json"
LEGACY_CONFIG = ROOT / "config.legacy.example.json"

LOW, NEUTRAL, HIGH = 172, 992, 1811


def fast_settings(pivot_gain):
    data = json.loads(FAST_CONFIG.read_text())
    data.update(motor_port="/dev/test-hat", radio_port="/dev/test-radio", max_rpm=40)
    # replace() raises TypeError until Settings has a pivot_gain field: the expected red.
    return replace(Settings.from_dict(data), pivot_gain=pivot_gain)


def legacy_settings(pivot_gain):
    data = json.loads(LEGACY_CONFIG.read_text())
    data.update(motor_port="/dev/test-hat", radio_port="/dev/test-radio")
    return replace(Settings.from_dict(data), pivot_gain=pivot_gain)


def fast_frame(*, arm=LOW, throttle=LOW, steering=NEUTRAL, reverse=LOW, stop=LOW, profile=LOW, lq=80):
    """Fast layout: steering CH1, throttle CH3, arm CH5, profile CH6, stop CH7, reverse CH8."""
    values = [NEUTRAL] * 16
    values[0] = steering
    values[2] = throttle
    values[4] = arm
    values[5] = profile
    values[6] = stop
    values[7] = reverse
    now = time.monotonic()
    return CRSFSnapshot(tuple(values), now, lq, now, None)


def legacy_frame(*, arm=LOW, throttle=LOW, steering=NEUTRAL, reverse=LOW, stop=LOW, dial=HIGH, lq=80):
    """Legacy layout: steering CH1, throttle CH3, arm CH5, reverse CH6, stop CH7, current dial CH10."""
    values = [NEUTRAL] * 16
    values[0] = steering
    values[2] = throttle
    values[4] = arm
    values[5] = reverse
    values[6] = stop
    values[9] = dial
    now = time.monotonic() - 0.001
    return CRSFSnapshot(tuple(values), now, lq, now, None)


class PhaseRadio:
    def __init__(self, phases, frame):
        self.phases, self.frame, self.index = phases, frame, 0

    def snapshot(self):
        return self.frame(**self.phases[self.index])


class FakeFastHat:
    """Records ("targets", (w1, w2, w3, w4), profile) and reports a consistent armed/disarmed status."""

    def __init__(self):
        self.commands = []
        self.sequence = 0
        self.state = HatState.DISARMED
        self.stop_state = StopState.CONFIRMED
        self.closed = False
        self.config = None
        self.profile = Profile.GENTLE
        self.targets_rpm = (0, 0, 0, 0)
        self.boot, self.session = 123, 456

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def _send(self, name, *values):
        self.sequence += 1
        self.commands.append((name, *values))
        return self.sequence

    def hello(self):
        self.state, self.stop_state, self.config = HatState.DISARMED, StopState.CONFIRMED, None
        self.session += 1
        return self._send("hello")

    def send_config(self, config):
        self.config = config
        return self._send("config")

    def arm(self):
        self.state, self.stop_state = HatState.ARMED, StopState.NONE
        return self._send("arm")

    def targets(self, rpm, profile):
        self.targets_rpm, self.profile = tuple(rpm), profile
        return self._send("targets", tuple(rpm), profile)

    def stop(self):
        self.state, self.stop_state = HatState.DISARMED, StopState.CONFIRMED
        self.targets_rpm = (0, 0, 0, 0)
        return self._send("stop")

    def request_status(self):
        return self._send("status")

    def wait_ack(self, seq, timeout=None):
        return self.snapshot()

    def snapshot(self):
        armed = self.state == HatState.ARMED
        wheels = tuple(
            FastWheel(0, 0, 25, 0, 0, target_rpm=self.targets_rpm[i], position_raw=0,
                      effective_cap_ma=800 if armed else 0, temp_age_ms=0, mode=1)
            for i in range(4)
        )
        return FastStatus(
            self.sequence, self.state, 0, 12000, 0, wheels, time.monotonic(), boot_id=self.boot,
            host_session=self.session, capabilities=REQUIRED_CAPABILITIES, build_id=1122,
            config_id=self.config.configuration_id if self.config else 0,
            config_result=ConfigResult.APPLIED if self.config else ConfigResult.NONE,
            stop_state=self.stop_state, requested_profile=self.profile, applied_profile=self.profile,
            boost_capacity_ms=20000, reason_flags=Reason(0), fault_wheel=0,
        )

    def stats(self):
        return SimpleNamespace(last_rtt_ms=4, max_rtt_ms=5, last_status_interval_ms=20, reader_error=None)


def expected_pivot_targets(cfg, direction=1):
    """Polarity-applied targets, ordered by motor id. direction=+1 is stick right: left wheels forward."""
    p = cfg.pivot_gain * cfg.max_rpm
    logical = {w.motor_id: (direction * p if w.side == "left" else -direction * p) for w in cfg.wheels}
    return tuple(logical[w.motor_id] * w.polarity for w in sorted(cfg.wheels, key=lambda w: w.motor_id))


class FastLoopPivotTests(unittest.TestCase):
    # arm low; arm high (the arming pass); arm high, sticks neutral (first armed pass)
    NEUTRAL_PHASES = [{"arm": LOW}, {"arm": HIGH}, {"arm": HIGH}]

    def run_fast(self, cfg, final_phase):
        phases = self.NEUTRAL_PHASES + [final_phase]
        hat, radio = FakeFastHat(), PhaseRadio(phases, fast_frame)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        robot = FastRobot(cfg, radio, telemetry_path=Path(tmp.name) / "data.csv", live_status=False)

        def advance(_duration):
            if radio.index == len(phases) - 1:
                robot.request_shutdown()
            else:
                radio.index += 1

        with patch("fast_robot.FastHat", return_value=hat), patch("fast_robot.time.sleep", side_effect=advance):
            robot.run()
        return hat

    def target_commands(self, hat):
        return [c for c in hat.commands if c[0] == "targets"]

    def assertTargets(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=6)

    def test_L1_full_right_stick_at_zero_throttle_commands_polarity_applied_pivot(self):
        cfg = fast_settings(0.5)
        hat = self.run_fast(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH})
        self.assertEqual([c[0] for c in hat.commands].count("arm"), 1)
        commands = self.target_commands(hat)
        self.assertTrue(commands, "no targets command reached the HAT")
        # Neutral sticks after arming command zero; the final pass commands the pivot.
        self.assertTargets(commands[0][1], (0, 0, 0, 0))
        self.assertTargets(commands[-1][1], expected_pivot_targets(cfg))
        self.assertEqual(commands[-1][2], Profile.GENTLE)

    def test_L1b_pivot_is_logically_left_forward_right_backward(self):
        cfg = fast_settings(0.5)
        hat = self.run_fast(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH})
        rpm = self.target_commands(hat)[-1][1]
        by_id = {w.motor_id: rpm[i] * w.polarity for i, w in enumerate(sorted(cfg.wheels, key=lambda w: w.motor_id))}
        for wheel in cfg.wheels:
            if wheel.side == "left":
                self.assertGreater(by_id[wheel.motor_id], 0)
            else:
                self.assertLess(by_id[wheel.motor_id], 0)
            self.assertAlmostEqual(abs(by_id[wheel.motor_id]), 0.5 * cfg.max_rpm, places=6)

    def test_L2_pivot_gain_zero_sends_all_zero_targets_for_pivot_phase(self):
        cfg = fast_settings(0.0)
        hat = self.run_fast(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH})
        commands = self.target_commands(hat)
        self.assertTrue(commands, "the loop must still command (zero) targets while armed")
        self.assertEqual(len(commands), 2)  # one per armed pass: neutral, then the pivot-phase pass
        for command in commands:
            self.assertTargets(command[1], (0, 0, 0, 0))

    def test_L4_reverse_switch_does_not_change_pivot_direction(self):
        cfg = fast_settings(0.5)
        hat = self.run_fast(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH, "reverse": HIGH})
        self.assertTargets(self.target_commands(hat)[-1][1], expected_pivot_targets(cfg))

    def test_L5_full_left_stick_pivots_the_other_way(self):
        cfg = fast_settings(0.5)
        hat = self.run_fast(cfg, {"arm": HIGH, "throttle": LOW, "steering": LOW})
        self.assertTargets(self.target_commands(hat)[-1][1], expected_pivot_targets(cfg, direction=-1))

    def test_L6_pivot_gain_only_changes_targets_never_the_firmware_configuration(self):
        on = self.run_fast(fast_settings(0.5), {"arm": HIGH, "steering": HIGH})
        off = self.run_fast(fast_settings(0.0), {"arm": HIGH, "steering": HIGH})
        self.assertEqual(on.config, off.config)


class FakeLegacyHat:
    def __init__(self):
        self.commands = []
        self.stopped = []
        self.closed = False

    def set_current_raw(self, mid, counts):
        self.commands.append((mid, counts))
        return Feedback(mid, Mode.CURRENT, 0, 0, 0, position_raw=0)

    def emergency_stop(self, ids):
        self.stopped.append(tuple(ids))
        return []

    def close(self):
        self.closed = True


class LegacyLoopPivotTests(unittest.TestCase):
    def run_legacy(self, cfg, final_phase):
        phases = [{"arm": LOW}, {"arm": HIGH}, {"arm": HIGH}, final_phase]
        radio = PhaseRadio(phases, legacy_frame)
        hats = []

        class HarnessRobot(Robot):
            def open_hat(self):
                self.hat = FakeLegacyHat()
                hats.append(self.hat)

            def prepare_current_mode(self):
                for control in self.controls.values():
                    control.last_feedback_at = time.monotonic()

        robot = HarnessRobot(cfg, radio)

        def advance(_seconds):
            if radio.index == len(phases) - 1:
                robot.request_shutdown()
            else:
                radio.index += 1

        with patch("robot_main.time.sleep", side_effect=advance):
            robot.run()
        return [command for hat in hats for command in hat.commands]

    def test_L3_pivot_on_commands_opposite_logical_drive_on_left_and_right(self):
        cfg = legacy_settings(0.5)
        self.assertEqual(cfg.motor_backend, "legacy")
        commands = self.run_legacy(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH})
        self.assertTrue(commands, "the real drive() must command the motors at zero throttle with pivot on")
        polarity = {w.motor_id: w.polarity for w in cfg.wheels}
        side = {w.motor_id: w.side for w in cfg.wheels}
        # Counts are motor-signed (polarity already applied); multiply back to the logical direction.
        logical = {mid: counts * polarity[mid] for mid, counts in commands[-len(cfg.wheels):]}
        self.assertEqual(set(logical), {1, 2, 3, 4})
        for mid, value in logical.items():
            if side[mid] == "left":
                self.assertGreater(value, 0, f"left wheel {mid} must drive forward")
            else:
                self.assertLess(value, 0, f"right wheel {mid} must drive backward")

    def test_L3b_pivot_gain_zero_commands_no_drive_at_zero_throttle(self):
        cfg = legacy_settings(0.0)
        commands = self.run_legacy(cfg, {"arm": HIGH, "throttle": LOW, "steering": HIGH})
        self.assertTrue(commands)
        self.assertTrue(all(counts == 0 for _mid, counts in commands))


if __name__ == "__main__":
    unittest.main()
