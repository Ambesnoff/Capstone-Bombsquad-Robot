"""Spec tests: faster ramps, pivot turning and the steering convention.

Written test-first.  Features exercised here:

* config.example.json ramps and neutral braking values (gains unchanged)
* the optional ``pivot_gain`` config key (0..1, default 0.0, Pi-only)
* ``robot_main.wheel_targets`` pivot mixing
* the steering convention: stick right turns the robot to ITS right
"""

import copy
import json
import time
import unittest
from dataclasses import replace
from pathlib import Path

import robot_config
from crsf import CRSFSnapshot
from robot_main import DriveRequest, Settings, drive_request, wheel_targets


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config.example.json"
LEGACY = ROOT / "config.legacy.example.json"
WHEELS = (1, 2, 3, 4)


def example_data() -> dict:
    return json.loads(EXAMPLE.read_text())


def example_settings() -> Settings:
    return Settings.from_dict(example_data())


def with_pivot(settings: Settings, value: float) -> Settings:
    # TypeError here until Settings grows a pivot_gain field (expected red).
    return replace(settings, pivot_gain=value)


def request(throttle, steering, reverse=False, cap=0.8) -> DriveRequest:
    return DriveRequest(throttle=throttle, steering=steering, reverse=reverse,
                        current_cap_a=cap)


def todays_targets(req: DriveRequest, settings: Settings) -> dict:
    """Reference copy of the pre-feature wheel_targets formula."""
    throttle = req.throttle if req.current_cap_a > 0.05 else 0.0
    linear = throttle * (-1 if req.reverse else 1)
    turn = req.steering * throttle * settings.steering_gain
    left, right = linear + turn, linear - turn
    scale = max(1, abs(left), abs(right))
    return {
        w.motor_id: settings.max_rpm * ((left if w.side == "left" else right) / scale)
        for w in settings.wheels
    }


class PivotTestCase(unittest.TestCase):
    def assertTargets(self, actual: dict, expected: dict, msg=None):
        self.assertEqual(set(actual), set(expected), msg)
        for motor_id, value in expected.items():
            self.assertAlmostEqual(actual[motor_id], value, places=9,
                                   msg=f"{msg or ''} wheel {motor_id}")


class ConfigTests(PivotTestCase):
    def test_c1_example_config_values(self):
        s = example_settings()
        self.assertEqual(s.acceleration_rpm_s, 500)
        self.assertEqual(s.deceleration_rpm_s, 1000)
        self.assertEqual(s.neutral_braking_current_a, 1.0)
        self.assertEqual(s.pivot_gain, 0.5)
        # Gains must remain exactly as before.
        self.assertEqual(s.kp_a_per_rpm, 0.02)
        self.assertEqual(s.ki_a_per_rpm_s, 0.02)
        self.assertEqual(s.ff_ma_per_rpm_s, 0)

    def test_c1_raw_json_values(self):
        data = example_data()
        self.assertEqual(data["acceleration_rpm_s"], 500)
        self.assertEqual(data["deceleration_rpm_s"], 1000)
        self.assertEqual(data["neutral_braking_current_a"], 1.0)
        self.assertEqual(data["pivot_gain"], 0.5)
        self.assertEqual(data["kp_a_per_rpm"], 0.02)
        self.assertEqual(data["ki_a_per_rpm_s"], 0.02)
        self.assertEqual(data["ff_ma_per_rpm_s"], 0)

    def test_legacy_example_unchanged(self):
        data = json.loads(LEGACY.read_text())
        self.assertNotIn("pivot_gain", data)

    def test_c2_missing_pivot_gain_defaults_to_zero(self):
        data = example_data()
        del data["pivot_gain"]
        self.assertEqual(Settings.from_dict(data).pivot_gain, 0.0)

    def test_c2_legacy_config_loads_with_pivot_off(self):
        data = json.loads(LEGACY.read_text())
        self.assertEqual(Settings.from_dict(data).pivot_gain, 0.0)

    def test_c3_valid_values_load(self):
        for value in (0, 0.5, 1):
            with self.subTest(value=value):
                data = example_data()
                data["pivot_gain"] = value
                self.assertEqual(Settings.from_dict(data).pivot_gain, value)

    def test_c3_invalid_values_rejected(self):
        for value in (-0.01, 1.01, "0.5", True, None):
            with self.subTest(value=value):
                data = example_data()
                data["pivot_gain"] = value
                with self.assertRaises(ValueError) as ctx:
                    Settings.from_dict(data)
                # Must be rejected as a bad value, not as an unknown key.
                self.assertNotIn("Unknown configuration", str(ctx.exception))
                self.assertIn("pivot_gain", str(ctx.exception))

    def test_c4_to_dict_includes_pivot_gain(self):
        for value in (0.0, 0.5, 1.0):
            with self.subTest(value=value):
                data = example_data()
                data["pivot_gain"] = value
                self.assertEqual(Settings.from_dict(data).to_dict()["pivot_gain"], value)

    def test_c5_hat_payload_independent_of_pivot_gain(self):
        base = example_data()
        payloads = []
        for value in (0.0, 0.5):
            data = copy.deepcopy(base)
            data["pivot_gain"] = value
            payloads.append(robot_config.hat_configuration(Settings.from_dict(data)).payload())
        self.assertEqual(payloads[0], payloads[1])


class MixingTests(PivotTestCase):
    def setUp(self):
        self.base = example_settings()

    def test_example_wheel_layout(self):
        sides = {w.motor_id: w.side for w in self.base.wheels}
        self.assertEqual(sides, {1: "left", 2: "right", 3: "right", 4: "left"})
        self.assertEqual(self.base.max_rpm, 250)
        self.assertEqual(self.base.steering_gain, 1.0)

    def test_m1_backward_compatible_when_pivot_off(self):
        s = with_pivot(self.base, 0.0)
        for t in (0, 0.25, 0.5, 1):
            for st in (-1, -0.5, 0, 0.5, 1):
                for rev in (False, True):
                    with self.subTest(t=t, s=st, reverse=rev):
                        req = request(t, st, rev)
                        self.assertTargets(wheel_targets(req, s), todays_targets(req, s))

    def test_m1_backward_compatible_when_cap_gated(self):
        s = with_pivot(self.base, 0.0)
        req = request(0.5, 1, False, cap=0.05)
        self.assertTargets(wheel_targets(req, s), todays_targets(req, s))

    def test_m2_pivot_right(self):
        s = with_pivot(self.base, 0.5)
        self.assertTargets(wheel_targets(request(0, 1), s),
                           {1: 125, 2: -125, 3: -125, 4: 125})

    def test_m3_pivot_left(self):
        s = with_pivot(self.base, 0.5)
        self.assertTargets(wheel_targets(request(0, -1), s),
                           {1: -125, 2: 125, 3: 125, 4: -125})

    def test_m4_reverse_switch_does_not_change_pivot(self):
        s = with_pivot(self.base, 0.5)
        forward = wheel_targets(request(0, 1, reverse=False), s)
        reverse = wheel_targets(request(0, 1, reverse=True), s)
        self.assertTargets(reverse, forward)
        self.assertTargets(reverse, {1: 125, 2: -125, 3: -125, 4: 125})

    def test_m5_half_stick(self):
        s = with_pivot(self.base, 0.5)
        self.assertTargets(wheel_targets(request(0, 0.5), s),
                           {1: 62.5, 2: -62.5, 3: -62.5, 4: 62.5})

    def test_m6_blend_at_half_throttle(self):
        s = with_pivot(self.base, 0.5)
        # turn 0.75, left 1.25, right -0.25, scale 1.25
        self.assertTargets(wheel_targets(request(0.5, 1), s),
                           {1: 250.0, 2: -50.0, 3: -50.0, 4: 250.0})

    def test_m7_full_throttle_ignores_pivot_gain(self):
        for pivot in (0, 0.5):
            with self.subTest(pivot_gain=pivot):
                s = with_pivot(self.base, pivot)
                self.assertTargets(wheel_targets(request(1, 1), s),
                                   {1: 250, 2: 0, 3: 0, 4: 250})

    def test_m8_current_cap_gate(self):
        s = with_pivot(self.base, 0.5)
        for cap in (0.05, 0.0):
            with self.subTest(cap=cap):
                self.assertTargets(wheel_targets(request(0, 1, cap=cap), s),
                                   {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})

    def test_m8_current_cap_gate_with_throttle(self):
        s = with_pivot(self.base, 0.5)
        self.assertTargets(wheel_targets(request(0.5, 1, cap=0.05), s),
                           {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})

    def test_m9_no_creep_at_neutral(self):
        s = with_pivot(self.base, 0.5)
        self.assertTargets(wheel_targets(request(0, 0), s),
                           {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})


class ConventionTests(PivotTestCase):
    """Stick right turns the robot to its right (left side logically faster)."""

    def setUp(self):
        # Plain example settings: convention holds whatever the pivot gain.
        self.settings = example_settings()

    def pivoting(self):
        """Settings with pivot on (TypeError until the feature exists)."""
        return with_pivot(self.settings, 0.5)

    def frame(self, *, throttle=172, steering=992, reverse=172):
        s = self.settings
        c = s.channels
        channels = [992] * 16
        channels[c.throttle - 1] = throttle
        channels[c.steering - 1] = steering
        channels[c.reverse - 1] = reverse
        channels[c.profile - 1] = 172   # Gentle
        channels[c.arm - 1] = 172
        channels[c.stop - 1] = 172
        now = time.monotonic()
        return CRSFSnapshot(tuple(channels), now, 80, now, None)

    def sides(self, *, pivot=False, **kwargs):
        settings = self.pivoting() if pivot else self.settings
        req = drive_request(self.frame(**kwargs), settings)
        targets = wheel_targets(req, settings)
        return targets[1], targets[2], req

    def test_v1_forward_without_pivot(self):
        left, right, _ = self.sides(throttle=1600, steering=1811, reverse=172)
        self.assertGreater(left, right)
        left, right, _ = self.sides(throttle=1600, steering=172, reverse=172)
        self.assertLess(left, right)

    def test_v1_forward(self):
        right_stick = self.sides(pivot=True, throttle=1600, steering=1811, reverse=172)
        left, right, _ = right_stick
        self.assertGreater(left, right)
        left, right, _ = self.sides(pivot=True, throttle=1600, steering=172, reverse=172)
        self.assertLess(left, right)

    def test_v2_reversing(self):
        left, right, req = self.sides(pivot=True, throttle=1600, steering=1811, reverse=1811)
        self.assertTrue(req.reverse)
        self.assertLess(right, 0)
        self.assertGreater(left - right, 0)
        # Mirror: full left stick while reversing.
        left, right, _ = self.sides(pivot=True, throttle=1600, steering=172, reverse=1811)
        self.assertLess(left, right)

    def test_v3_pivot(self):
        left, right, req = self.sides(pivot=True, throttle=172, steering=1811, reverse=172)
        self.assertEqual(req.throttle, 0)
        self.assertGreater(left, 0)
        self.assertLess(right, 0)
        left, right, _ = self.sides(pivot=True, throttle=172, steering=172, reverse=172)
        self.assertLess(left, right)

    def test_v3_pivot_direction_unchanged_by_reverse_switch(self):
        fwd = self.sides(pivot=True, throttle=172, steering=1811, reverse=172)[:2]
        rev = self.sides(pivot=True, throttle=172, steering=1811, reverse=1811)[:2]
        self.assertEqual(fwd, rev)

    def test_v5_reverse_switch_semantics(self):
        for raw, expected in ((172, False), (992, False), (1811, True)):
            with self.subTest(raw=raw):
                req = drive_request(self.frame(reverse=raw), self.settings)
                self.assertIs(req.reverse, expected)

    def test_throttle_and_steering_mapping(self):
        req = drive_request(self.frame(throttle=172, steering=992), self.settings)
        self.assertEqual(req.throttle, 0)
        self.assertEqual(req.steering, 0)
        req = drive_request(self.frame(throttle=172, steering=1811), self.settings)
        self.assertAlmostEqual(req.steering, 1.0)


if __name__ == "__main__":
    unittest.main()
