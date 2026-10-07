"""Robot behavior tests with no connected motors or radio hardware."""

import json
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from crsf import CRSFSnapshot
from ddsm115 import Feedback, HatError, Mode, MotorFault
from robot_main import (
    ArmingGate,
    DriveRequest,
    RadioLost,
    Robot,
    Settings,
    ShutdownRequested,
    WheelControl,
    drive_request,
    radio_healthy,
    wheel_targets,
)


CONFIG = Path(__file__).resolve().parents[1] / "config.example.json"


def settings() -> Settings:
    data = json.loads(CONFIG.read_text())
    data["motor_port"] = "/dev/test-hat"
    data["radio_port"] = "/dev/test-radio"
    # Existing controller/protection cases deliberately exercise a 40-rpm config.
    data["max_rpm"] = 40
    return Settings.from_dict(data)


def snapshot(*, throttle=172, steering=992, dial=1811, arm=172,
             reverse=172, stop=172, lq=80, now=None) -> CRSFSnapshot:
    channels = [992] * 16
    channels[2] = throttle
    channels[0] = steering
    channels[9] = dial
    channels[4] = arm
    channels[5] = reverse
    channels[6] = stop
    now = time.monotonic() if now is None else now
    return CRSFSnapshot(tuple(channels), now, lq, now, None)


class FakeRadio:
    def __init__(self, value: CRSFSnapshot):
        self.value = value

    def snapshot(self) -> CRSFSnapshot:
        return self.value


class FakeHat:
    def __init__(self, radio=None, *, fault=False):
        self.radio = radio
        self.fault = fault
        self.commands = []
        self.stopped = []
        self.closed = False

    def set_current_raw(self, mid, counts):
        self.commands.append((mid, counts))
        if self.radio and len(self.commands) == 1:
            self.radio.value = snapshot(arm=172)
        return Feedback(mid, Mode.CURRENT, 0, 0, 1 if self.fault else 0, position_raw=0)

    def emergency_stop(self, ids):
        self.stopped.append(tuple(ids))
        return []

    def close(self):
        self.closed = True


class RobotTests(unittest.TestCase):
    def test_example_has_four_clockwise_ids_and_polarities(self):
        self.assertEqual(
            [(wheel.motor_id, wheel.side, wheel.polarity) for wheel in settings().wheels],
            [(1, "left", -1), (2, "right", 1), (3, "right", 1), (4, "left", -1)],
        )

    def test_fast_config_rejects_uart_rate_or_rpm_the_firmware_cannot_use(self):
        data = json.loads(CONFIG.read_text())
        data["hat_baud"] = 115200
        with self.assertRaisesRegex(ValueError, "230400 baud"):
            Settings.from_dict(data)
        data["hat_baud"] = 230400
        data["max_rpm"] = 251
        with self.assertRaisesRegex(ValueError, "max_rpm"):
            Settings.from_dict(data)

    def test_radio_requires_fresh_channels_and_positive_fresh_link(self):
        cfg = settings()
        now = time.monotonic()
        fresh = snapshot(now=now)
        self.assertTrue(radio_healthy(fresh, now, cfg))
        self.assertFalse(radio_healthy(replace(fresh, channels_at=now - 0.3), now, cfg))
        self.assertFalse(radio_healthy(replace(fresh, link_quality=0), now, cfg))
        self.assertFalse(radio_healthy(replace(fresh, link_quality=255), now, cfg))
        self.assertFalse(radio_healthy(replace(fresh, link_at=None), now, cfg))
        self.assertFalse(radio_healthy(replace(fresh, error="serial unplugged"), now, cfg))

    def test_arm_requires_neutral_low_then_new_high_edge(self):
        cfg = settings()
        gate = ArmingGate(cfg)
        now = time.monotonic()
        self.assertNotEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        gate.observe(snapshot(arm=172, now=now), now)
        self.assertNotEqual(
            gate.observe(snapshot(arm=1811, throttle=1811, now=now), now), "newly_armed"
        )
        self.assertNotEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        gate.observe(snapshot(arm=172, now=now), now)
        self.assertEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        self.assertEqual(gate.observe(snapshot(arm=1811, now=now), now), "armed")

    def test_radio_loss_and_stop_latch_need_new_arm_cycle(self):
        cfg = settings()
        gate = ArmingGate(cfg)
        now = time.monotonic()
        gate.observe(snapshot(arm=172, now=now), now)
        self.assertEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        self.assertEqual(gate.observe(snapshot(arm=1811, lq=0, now=now), now), "link_lost")
        self.assertNotEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        gate.observe(snapshot(arm=172, now=now), now)
        self.assertEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")
        self.assertEqual(gate.observe(snapshot(arm=1811, stop=1811, now=now), now), "stop_requested")
        self.assertNotEqual(gate.observe(snapshot(arm=1811, now=now), now), "newly_armed")

    def test_skid_steering_and_reverse_use_all_four_ids(self):
        cfg = settings()
        request = drive_request(snapshot(throttle=1811, steering=1811), cfg)
        targets = wheel_targets(request, cfg)
        self.assertEqual(set(targets), {1, 2, 3, 4})
        self.assertGreater(targets[1], targets[2])
        self.assertEqual(targets[1], targets[4])
        self.assertEqual(targets[2], targets[3])
        backward = wheel_targets(replace(request, reverse=True, steering=0), cfg)
        self.assertTrue(all(value < 0 for value in backward.values()))
        zero_dial = wheel_targets(replace(request, current_cap_a=0), cfg)
        self.assertTrue(all(value == 0 for value in zero_dial.values()))

    def test_current_output_respects_dial_cap_and_slew_limit(self):
        cfg = settings()
        control = WheelControl()
        now = time.monotonic()
        counts = control.command(40, 0.4, now, cfg)
        self.assertLessEqual(abs(counts), round(0.4 * 32767 / 8))
        self.assertLessEqual(control.target_rpm, cfg.acceleration_rpm_s * cfg.loop_period_s)
        self.assertGreaterEqual(counts, 0)
        control.accept(Feedback(1, Mode.CURRENT, 60, 0, 0, position_raw=0), now)
        counts = control.command(0, 0, now + cfg.loop_period_s, cfg)
        self.assertLess(counts, 0)
        self.assertLessEqual(abs(counts), round(cfg.neutral_braking_current_a * 32767 / 8))

    def test_neutral_does_not_restart_a_stopped_wheel(self):
        cfg = settings()
        control = WheelControl(last_rpm=0, target_rpm=40, updated_at=time.monotonic())
        counts = control.command(0, 0, control.updated_at + cfg.loop_period_s, cfg)
        self.assertEqual(counts, 0)
        self.assertEqual(control.target_rpm, 0)

    def test_reversal_uses_deceleration_rate(self):
        cfg = replace(settings(), acceleration_rpm_s=1000, deceleration_rpm_s=10)
        control = WheelControl(target_rpm=10, updated_at=time.monotonic())
        control.command(-40, 1.0, control.updated_at + 0.1, cfg)
        self.assertAlmostEqual(control.target_rpm, 9.0)

    def test_overspeed_feedback_is_a_fault(self):
        robot = Robot(settings(), FakeRadio(snapshot()))
        with self.assertRaises(HatError):
            robot._check_feedback(Feedback(1, Mode.CURRENT, 61, 0, 0, position_raw=0))

    def test_mid_cycle_disarm_prevents_next_motor_command_and_stops_all(self):
        cfg = settings()
        radio = FakeRadio(snapshot(arm=1811, throttle=1811))
        robot = Robot(cfg, radio)
        hat = FakeHat(radio)
        robot.hat = hat
        for control in robot.controls.values():
            control.last_feedback_at = time.monotonic()
        request = drive_request(radio.snapshot(), cfg)
        with self.assertRaises(RadioLost):
            robot.drive(request)
        self.assertEqual(len(hat.commands), 1)
        robot.stop_all()
        self.assertEqual(hat.stopped, [(1, 2, 3, 4)])
        self.assertTrue(hat.closed)

    def test_run_stops_every_motor_if_feedback_reports_fault(self):
        cfg = settings()

        class SequenceRadio:
            def __init__(self):
                self.calls = 0

            def snapshot(self):
                self.calls += 1
                return snapshot(
                    arm=172 if self.calls == 1 else 1811,
                    throttle=172 if self.calls < 5 else 1811,
                    now=time.monotonic() - 0.001,
                )

        class TestRobot(Robot):
            def open_hat(self):
                self.hat = fake_hat

            def prepare_current_mode(self):
                for control in self.controls.values():
                    control.last_feedback_at = time.monotonic()

        fake_hat = FakeHat(fault=True)
        robot = TestRobot(cfg, SequenceRadio())
        with self.assertRaises(MotorFault):
            robot.run()
        self.assertEqual(fake_hat.stopped, [(1, 2, 3, 4)])
        self.assertTrue(fake_hat.closed)

    def test_shutdown_during_motor_command_skips_remaining_wheels(self):
        cfg = settings()
        radio = FakeRadio(snapshot(arm=1811, throttle=1811))
        robot = Robot(cfg, radio)

        class InterruptingHat(FakeHat):
            def set_current_raw(self, mid, counts):
                feedback = super().set_current_raw(mid, counts)
                robot.request_shutdown()
                return feedback

        hat = InterruptingHat()
        robot.hat = hat
        for control in robot.controls.values():
            control.last_feedback_at = time.monotonic()
        with self.assertRaises(ShutdownRequested):
            robot.drive(drive_request(radio.snapshot(), cfg))
        self.assertEqual([mid for mid, _ in hat.commands], [1])
        robot.stop_all()
        self.assertEqual(hat.stopped, [(1, 2, 3, 4)])

    def test_full_run_stops_and_requires_rearm_after_stop_and_link_loss(self):
        cfg = settings()
        phases = [
            {"arm": 172},                # first low observation
            {"arm": 1811},               # arm
            {"arm": 1811, "throttle": 1811},
            {"arm": 1811, "stop": 1811}, # stop switch
            {"arm": 1811},               # still high: cannot rearm
            {"arm": 172},
            {"arm": 1811},               # deliberate rearm
            {"arm": 1811, "throttle": 1811},
            {"arm": 1811, "lq": 0},     # RF loss
            {"arm": 1811},               # RF back, but still high
            {"arm": 172},
            {"arm": 1811},               # another deliberate rearm
            {"arm": 1811, "throttle": 1811},
        ]

        class PhaseRadio:
            index = 0

            def snapshot(self):
                return snapshot(now=time.monotonic() - 0.001, **phases[self.index])

        class TestRobot(Robot):
            def __init__(self, cfg, radio):
                super().__init__(cfg, radio)
                self.hats = []
                self.drive_phases = []

            def open_hat(self):
                self.hat = FakeHat()
                self.hats.append(self.hat)

            def prepare_current_mode(self):
                pass

            def drive(self, request):
                self.drive_phases.append(self.radio.index)

        radio = PhaseRadio()
        robot = TestRobot(cfg, radio)

        def advance(_seconds):
            if radio.index == len(phases) - 1:
                robot.request_shutdown()
            else:
                radio.index += 1

        with patch("robot_main.time.sleep", side_effect=advance):
            robot.run()
        self.assertEqual(robot.drive_phases, [2, 7, 12])
        self.assertEqual(len(robot.hats), 3)
        self.assertTrue(all(hat.stopped == [(1, 2, 3, 4)] for hat in robot.hats))


if __name__ == "__main__":
    unittest.main()


class ConfigurationAndProfileTests(unittest.TestCase):
    def test_unknown_boolean_and_nonfinite_configuration_are_rejected(self):
        for update in ({"max_rpm": True}, {"steering_gain": float("nan")}, {"typo_current_a": 2.5},
                       {"radio_telemetry_enabled": 1}):
            data = json.loads(CONFIG.read_text()); data.update(update)
            with self.subTest(update=update), self.assertRaises(ValueError): Settings.from_dict(data)

    def test_full_candidate_uses_shared_firmware_validator(self):
        data = json.loads(CONFIG.read_text());data["protection"]["temp_poll_ms"] = 800
        with self.assertRaisesRegex(ValueError, "temp_poll_ms"): Settings.from_dict(data)
        data = json.loads(CONFIG.read_text());data["profiles"]["boost_capacity_s"] = 21
        with self.assertRaisesRegex(ValueError, "boost_capacity_ms"): Settings.from_dict(data)

    def test_identity_reproducible_and_complete(self):
        cfg = settings(); rebuilt = Settings.from_dict(cfg.to_dict())
        self.assertEqual(cfg.configuration_identity, rebuilt.configuration_identity)
        self.assertNotEqual(cfg.configuration_identity, replace(cfg, radio_timeout_s=.3).configuration_identity)

    def test_example_channels_map_mode6_arm5_stop7_reverse8(self):
        cfg = settings()
        self.assertEqual((cfg.channels.profile, cfg.channels.arm, cfg.channels.stop, cfg.channels.reverse), (6,5,7,8))

    def test_three_position_debounce_invalid_and_frame_replay(self):
        from robot_radio import Profile, ProfileSelector
        selector=ProfileSelector(settings().profiles)
        self.assertEqual(selector.observe(1811,10),Profile.GENTLE)
        self.assertEqual(selector.observe(1811,10),Profile.GENTLE)
        self.assertEqual(selector.observe(1811,10.2),Profile.BOOST)
        self.assertEqual(selector.observe(1200,10.3),Profile.GENTLE)
        self.assertFalse(selector.valid)
        self.assertEqual(selector.observe(992,10.4),Profile.GENTLE)
        self.assertEqual(selector.observe(992,10.6),Profile.NORMAL)
        self.assertEqual(selector.observe(172,10.7),Profile.GENTLE)

    def test_legacy_retains_dial_and_original_channel_mapping(self):
        cfg=Settings.from_dict(json.loads(CONFIG.with_name("config.legacy.example.json").read_text()))
        self.assertEqual((cfg.motor_backend,cfg.channels.current_dial,cfg.channels.arm,cfg.channels.reverse),("legacy",10,5,6))
        self.assertAlmostEqual(drive_request(snapshot(),cfg).current_cap_a,1.0)


class NeutralArmingTests(unittest.TestCase):
    def test_steering_must_be_neutral_and_unsafe_high_edge_is_consumed(self):
        cfg=settings();gate=ArmingGate(cfg);now=time.monotonic()
        gate.observe(snapshot(arm=172,now=now),now)
        self.assertNotEqual(gate.observe(snapshot(arm=1811,steering=1811,now=now),now),"newly_armed")
        self.assertNotEqual(gate.observe(snapshot(arm=1811,steering=992,now=now),now),"newly_armed")
        gate.observe(snapshot(arm=172,now=now),now)
        self.assertEqual(gate.observe(snapshot(arm=1811,now=now),now),"newly_armed")
