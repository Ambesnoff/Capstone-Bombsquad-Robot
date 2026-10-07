"""Shared 250-rpm requirements using in-memory transport substitutes."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROBOT_PROJECT = Path(__file__).resolve().parents[1]

from fast_hat import FastConfig, FastHat, FrameType, decode_status
from protocol_defs import (
    CONFIG_FIELDS, CONFIG_STRUCT, HatState, Profile, STATUS_HEADER_FIELDS,
    STATUS_HEADER_STRUCT, StopState, TARGETS_STRUCT, WHEEL_FIELDS, WHEEL_STRUCT,
)
from robot_config import Settings, hat_configuration
from robot_main import DriveRequest, wheel_targets


def configured_settings(max_rpm):
    data = json.loads((ROBOT_PROJECT / "config.example.json").read_text())
    data["motor_port"] = "/dev/test-hat"
    data["radio_port"] = "/dev/test-radio"
    data["max_rpm"] = max_rpm
    return Settings.from_dict(data)


def status_payload(target_centi_rpm, *, actual_rpm=0):
    header = dict.fromkeys(STATUS_HEADER_FIELDS, 0)
    header.update(
        boot_id=11, host_session=22, capabilities=255,
        build_id=1, accepted_seq=1, applied_seq=1,
        state=HatState.ARMED, stop_state=StopState.NONE,
        sweep_us=10000, command_age_ms=8,
        boost_capacity_ms=20000, boost_remaining_ms=12500,
    )
    wheel = dict(
        target_centi_rpm=target_centi_rpm, rpm=actual_rpm, current_ma=0,
        position_raw=12000, temp_c=25, effective_cap_ma=800,
        hold_cap_ma=300, age_ms=4, temp_age_ms=400,
        error=0, mode=1, validity=15, reason_flags=0,
    )
    return (
        STATUS_HEADER_STRUCT.pack(*(header[name] for name in STATUS_HEADER_FIELDS))
        + b"".join(
            WHEEL_STRUCT.pack(*(wheel[name] for name in WHEEL_FIELDS))
            for _ in range(4)
        )
    )


class Shared250RpmRequirementTests(unittest.TestCase):
    def test_supplied_fast_configuration_uses_shared_250_rpm_limit(self):
        data = json.loads((ROBOT_PROJECT / "config.example.json").read_text())
        self.assertEqual(data["max_rpm"], 250)

    def test_settings_and_wire_config_accept_250(self):
        settings = configured_settings(250)
        config = hat_configuration(settings)
        wire = dict(zip(CONFIG_FIELDS, CONFIG_STRUCT.unpack(config.payload())))
        self.assertEqual(settings.max_rpm, 250)
        self.assertEqual(wire["max_rpm"], 250)

    def test_settings_and_wire_config_reject_above_250(self):
        for value in (251, 330):
            with self.subTest(value=value), self.assertRaises(ValueError):
                configured_settings(value)
            with self.subTest(wire_value=value), self.assertRaises(ValueError):
                replace(FastConfig(), max_rpm=value).payload()

    def test_full_throttle_has_same_250_targets_in_every_profile(self):
        settings = configured_settings(250)
        for profile in Profile:
            request = DriveRequest(
                throttle=1.0, steering=0.0, reverse=False,
                current_cap_a=getattr(settings.profiles, f"{profile.name.lower()}_a"),
                profile=profile,
            )
            with self.subTest(profile=profile):
                logical = wheel_targets(request, settings)
                self.assertEqual(logical, {1: 250, 2: 250, 3: 250, 4: 250})
                self.assertEqual(
                    tuple(logical[w.motor_id] * w.polarity for w in settings.wheels),
                    (-250, 250, 250, -250),
                )
                self.assertEqual(
                    wheel_targets(replace(request, reverse=True), settings),
                    {1: -250, 2: -250, 3: -250, 4: -250},
                )

    def test_target_encoder_accepts_signed_250_in_every_profile(self):
        config = replace(FastConfig(), max_rpm=250)
        config.payload()  # The target ceiling must be a valid wire configuration.
        for profile in Profile:
            fake = SimpleNamespace(
                _config=config, _session=lambda: (11, 22, 33), _send=Mock(return_value=7),
            )
            with self.subTest(profile=profile):
                self.assertEqual(
                    FastHat.targets(fake, (-250, 250, 250, -250), profile), 7,
                )
                args, kwargs = fake._send.call_args
                self.assertEqual(args[0], FrameType.TARGETS)
                self.assertEqual(
                    TARGETS_STRUCT.unpack(args[1]),
                    (11, 22, -25000, 25000, 25000, -25000, int(profile)),
                )
                self.assertEqual(kwargs, {"generation": 33})

    def test_target_encoder_rejects_outside_signed_250_before_send(self):
        fake = SimpleNamespace(
            _config=replace(FastConfig(), max_rpm=250),
            _session=lambda: (11, 22, 33), _send=Mock(),
        )
        for value in (-250.01, 250.01):
            with self.subTest(value=value), self.assertRaises(ValueError):
                FastHat.targets(fake, (value, 0, 0, 0), Profile.GENTLE)
        fake._send.assert_not_called()

    def test_status_decoder_accepts_signed_250_targets(self):
        for value in (-25000, 25000):
            with self.subTest(value=value):
                status = decode_status(status_payload(value))
                self.assertTrue(all(w.target_rpm == value / 100 for w in status.wheels))

    def test_status_decoder_rejects_targets_beyond_signed_250(self):
        for value in (-25001, 25001):
            with self.subTest(value=value), self.assertRaises(ValueError):
                decode_status(status_payload(value))

    def test_status_keeps_above_ceiling_actual_speed_visible(self):
        # A real overspeed measurement is diagnostic data, not an invalid target.
        for value in (-313, 313):
            with self.subTest(value=value):
                status = decode_status(status_payload(0, actual_rpm=value))
                self.assertTrue(all(w.rpm == value for w in status.wheels))

    def test_explicit_40_rpm_configuration_keeps_original_target_behavior(self):
        settings = configured_settings(40)
        request = DriveRequest(1.0, 0.0, False, 0.8, Profile.GENTLE)
        self.assertEqual(wheel_targets(request, settings), {1: 40, 2: 40, 3: 40, 4: 40})

    def test_raising_hard_ceiling_does_not_bypass_a_lower_configured_ceiling(self):
        fake = SimpleNamespace(
            _config=replace(FastConfig(), max_rpm=40),
            _session=lambda: (11, 22, 33), _send=Mock(return_value=7),
        )
        self.assertEqual(FastHat.targets(fake, (-40, 40, 40, -40), Profile.BOOST), 7)
        fake._send.reset_mock()
        for value in (-40.01, 40.01):
            with self.subTest(value=value), self.assertRaises(ValueError):
                FastHat.targets(fake, (value, 0, 0, 0), Profile.BOOST)
        fake._send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
