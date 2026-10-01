"""Raspberry Pi drive program for the DDSM115 skid-steer prototype.

The radio and robot behavior live here. ddsm115.py only speaks to the HAT;
crsf.py only decodes the receiver's serial stream. Nothing moves on import.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from crsf import CRSFReader, CRSFSnapshot
from ddsm115 import DDSMHat, Feedback, HatError, Mode, MotorFault


LOG = logging.getLogger("robot")
CRSF_LOW = 172
CRSF_HIGH = 1811


from robot_config import Channels, Settings, Wheel, load_settings
from robot_radio import Profile, ProfileSelector, calibrated_profile


def normalized(raw: int) -> float:
    """Map the ordinary CRSF 172..1811 channel range to -1..+1."""
    return max(-1.0, min(1.0, 2 * (raw - CRSF_LOW) / (CRSF_HIGH - CRSF_LOW) - 1))


def channel(snapshot: CRSFSnapshot, number: int) -> float:
    if snapshot.channels is None or len(snapshot.channels) < number:
        raise ValueError("No complete 16-channel RC frame")
    return normalized(snapshot.channels[number - 1])


def radio_healthy(snapshot: CRSFSnapshot, now: float, settings: Settings) -> bool:
    """Require both fresh RC frames and a fresh positive receiver link quality.

    A receiver may keep reporting held channel values during RF failsafe. Fresh
    channel frames alone therefore do not authorize movement.
    """
    return (
        snapshot.error is None
        and snapshot.channels is not None
        and len(snapshot.channels) == 16
        and all(isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 2047 for v in snapshot.channels)
        and all(100 <= snapshot.channels[number - 1] <= 1900 for number in (
            settings.channels.throttle, settings.channels.steering, settings.channels.arm,
            settings.channels.reverse, settings.channels.stop))
        and snapshot.channels_at is not None
        and 0 <= now - snapshot.channels_at <= settings.radio_timeout_s
        and snapshot.link_quality is not None
        and 0 < snapshot.link_quality <= 100
        and snapshot.link_at is not None
        and 0 <= now - snapshot.link_at <= settings.link_timeout_s
    )


def _deadband(value: float, width: float) -> float:
    if abs(value) <= width:
        return 0.0
    return math.copysign((abs(value) - width) / (1 - width), value)


@dataclass(frozen=True)
class DriveRequest:
    throttle: float
    steering: float
    reverse: bool
    current_cap_a: float
    profile: Profile = Profile.GENTLE
    profile_valid: bool = True


def drive_request(snapshot: CRSFSnapshot, settings: Settings,
                  selector: ProfileSelector | None = None) -> DriveRequest:
    assigned = settings.channels
    throttle = (channel(snapshot, assigned.throttle) + 1) / 2
    throttle = max(0.0, (throttle - 0.04) / 0.96)
    steering = _deadband(channel(snapshot, assigned.steering), 0.04)
    if settings.motor_backend == "fast":
        assert assigned.profile is not None
        raw = snapshot.channels[assigned.profile - 1]
        requested = calibrated_profile(raw, settings.profiles)
        profile = selector.observe(raw, snapshot.channels_at) if selector else requested or Profile.GENTLE
        valid = selector.valid if selector else requested is not None
        # Candidate profile caps remain visible; the HAT owns final protection.
        cap = getattr(settings.profiles, f"{profile.name.lower()}_a")
    else:
        assert assigned.current_dial is not None
        dial = channel(snapshot, assigned.current_dial)
        fraction = max(0.0, min(1.0, (dial + 0.8) / 1.8))
        cap, profile, valid = fraction * settings.max_current_a, Profile.GENTLE, True
    return DriveRequest(
        throttle=throttle, steering=steering,
        reverse=channel(snapshot, assigned.reverse) > 0.5,
        current_cap_a=cap, profile=profile, profile_valid=valid,
    )


def wheel_targets(request: DriveRequest, settings: Settings) -> dict[int, float]:
    """Logical forward RPM; wheel polarity is applied only at the motor boundary."""
    throttle = request.throttle if request.current_cap_a > 0.05 else 0.0
    linear = throttle * (-1 if request.reverse else 1)
    turn = request.steering * throttle * settings.steering_gain
    left, right = linear + turn, linear - turn
    scale = max(1, abs(left), abs(right))
    return {
        wheel.motor_id: settings.max_rpm * ((left if wheel.side == "left" else right) / scale)
        for wheel in settings.wheels
    }


class ArmingGate:
    """A fresh link, neutral throttle, and a low-then-high arm switch are required."""

    def __init__(self, settings: Settings):
        self.settings = settings
        self.armed = False
        self.saw_arm_low = False
        self.arm_was_high = False

    def invalidate(self) -> None:
        self.armed = False
        self.saw_arm_low = False
        self.arm_was_high = False

    def observe(self, snapshot: CRSFSnapshot, now: float) -> str:
        if not radio_healthy(snapshot, now, self.settings):
            was_armed = self.armed
            self.invalidate()
            return "link_lost" if was_armed else "waiting_for_link"

        channels = self.settings.channels
        arm = channel(snapshot, channels.arm)
        stop = channel(snapshot, channels.stop) > 0.5
        if stop:
            was_armed = self.armed
            self.invalidate()
            return "stop_requested" if was_armed else "stop_held"

        if arm < -0.5:
            self.saw_arm_low = True
            self.arm_was_high = False
            if self.armed:
                self.armed = False
                return "disarmed"
            return "ready_to_arm"

        if self.armed:
            if arm <= 0.5:
                self.invalidate()
                return "disarmed"
            return "armed"

        if arm > 0.5:
            rising_edge = not self.arm_was_high
            self.arm_was_high = True
            if rising_edge and self.saw_arm_low:
                self.saw_arm_low = False
                request = drive_request(snapshot, self.settings)
                if request.throttle == 0 and request.steering == 0:
                    self.armed = True
                    return "newly_armed"
        return "waiting_for_neutral_or_arm_cycle"


@dataclass
class WheelControl:
    last_rpm: int = 0
    last_feedback_at: float = 0.0
    target_rpm: float = 0.0
    integral_a: float = 0.0
    updated_at: float = 0.0
    last_current_a: float = 0.0
    measured_current_a: float = 0.0

    def command(self, requested_rpm: float, current_cap_a: float, now: float, settings: Settings) -> int:
        if self.updated_at:
            dt = max(0.001, min(now - self.updated_at, 0.3))
        else:
            dt = settings.loop_period_s
        self.updated_at = now

        previous = self.target_rpm
        reversing = requested_rpm * previous < 0
        slowing = abs(requested_rpm) < abs(previous)
        rate = settings.deceleration_rpm_s if reversing or slowing else settings.acceleration_rpm_s
        limit = rate * dt
        self.target_rpm = previous + max(-limit, min(limit, requested_rpm - previous))

        # Neutral retains a limited amount of active deceleration even if the
        # driver's current-limit dial is at zero. Disarm/stop uses speed zero.
        neutral = abs(requested_rpm) < 0.001
        cap = max(current_cap_a, settings.neutral_braking_current_a) if neutral else current_cap_a
        if neutral:
            self.integral_a = 0
            # Do not drive a wheel away from rest merely to follow a decaying
            # ramp target left over from its previous motion command.
            if abs(self.last_rpm) <= 2:
                self.target_rpm = 0.0
        error = self.target_rpm - self.last_rpm
        if neutral and abs(self.last_rpm) <= 2:
            current_a = 0.0
        else:
            proportional = settings.kp_a_per_rpm * error
            proposed_integral = self.integral_a + settings.ki_a_per_rpm_s * error * dt
            proposed_integral = max(-cap, min(cap, proposed_integral))
            raw = proportional + proposed_integral
            if abs(raw) <= cap or math.copysign(1, raw) != math.copysign(1, error):
                self.integral_a = proposed_integral
            current_a = max(-cap, min(cap, proportional + self.integral_a))
        self.last_current_a = current_a
        return round(current_a * 32767 / 8)

    def accept(self, feedback: Feedback, now: float) -> None:
        self.last_rpm = feedback.rpm
        self.last_feedback_at = now
        self.measured_current_a = feedback.torque_current_a_estimate


class Robot:
    def __init__(self, settings: Settings, radio: CRSFReader):
        self.settings = settings
        self.radio = radio
        self.gate = ArmingGate(settings)
        self.hat: DDSMHat | None = None
        self.controls = {wheel.motor_id: WheelControl() for wheel in settings.wheels}
        self._last_status = 0.0
        self._running = True
        self._shutdown = threading.Event()

    def request_shutdown(self, *_args: Any) -> None:
        self._shutdown.set()
        self._running = False

    @property
    def ids(self) -> tuple[int, ...]:
        return tuple(wheel.motor_id for wheel in self.settings.wheels)

    def open_hat(self) -> None:
        """Open the HAT and positively request zero speed from every installed ID."""
        hat = DDSMHat(
            self.settings.motor_port,
            timeout=self.settings.motor_timeout_s,
            startup_delay=2.0,
            stop_on_close=False,
        )
        try:
            for mid in self.ids:
                feedback = hat.stop(mid)
                self._check_feedback(feedback)
            # The stock watchdog sends a mode-dependent zero setpoint. Put
            # every configured motor in speed mode before enabling it.
            hat.set_heartbeat(self.settings.hat_heartbeat_ms)
        except BaseException:
            hat.emergency_stop(self.ids)
            hat.close()
            raise
        self.hat = hat

    def _check_feedback(self, feedback: Feedback) -> None:
        if feedback.error:
            raise MotorFault(feedback)
        overspeed_limit = max(30, round(self.settings.max_rpm * 1.5))
        if abs(feedback.rpm) > overspeed_limit:
            raise HatError(
                f"Motor {feedback.motor_id} reported {feedback.rpm} RPM, "
                f"above the {overspeed_limit} RPM test limit"
            )

    def prepare_current_mode(self) -> None:
        assert self.hat is not None
        for wheel in self.settings.wheels:
            self._verify_still_armed(require_neutral=True)
            feedback = self.hat.set_mode(wheel.motor_id, Mode.CURRENT)
            self._check_feedback(feedback)
            if feedback.mode != Mode.CURRENT:
                raise HatError(f"Motor {wheel.motor_id} current mode was not verified")
            control = self.controls[wheel.motor_id] = WheelControl()
            control.accept(feedback, time.monotonic())
        # Mode setup can take several transactions per wheel. Refresh all four
        # after setup so the earliest wheel does not start with stale RPM.
        for wheel in self.settings.wheels:
            feedback = self.hat.get_feedback(wheel.motor_id)
            self._check_feedback(feedback)
            self.controls[wheel.motor_id].accept(feedback, time.monotonic())

    def stop_all(self) -> None:
        """Best-effort stop for the complete configured set, even after a fault."""
        if self.hat is None:
            return
        hat, self.hat = self.hat, None
        try:
            failures = hat.emergency_stop(self.ids)
            if failures:
                LOG.error("Some stop writes failed: %s", ", ".join(failures))
        finally:
            hat.close()

    def _verify_still_armed(self, *, require_neutral: bool = False) -> None:
        if self._shutdown.is_set() or not self._running:
            raise ShutdownRequested("Shutdown requested")
        snapshot = self.radio.snapshot()
        now = time.monotonic()
        if not radio_healthy(snapshot, now, self.settings):
            raise RadioLost("Radio frames or link statistics went stale")
        assigned = self.settings.channels
        if channel(snapshot, assigned.arm) <= 0.5 or channel(snapshot, assigned.stop) > 0.5:
            raise RadioLost("Arm switch released or stop switch requested")
        if require_neutral:
            request = drive_request(snapshot, self.settings)
            if request.throttle != 0 or request.steering != 0:
                raise RadioLost("Throttle or steering changed during arming; neutral and fresh arm cycle required")

    def drive(self, request: DriveRequest) -> None:
        assert self.hat is not None
        targets = wheel_targets(request, self.settings)
        max_feedback_age = (
            len(self.settings.wheels) * self.settings.motor_timeout_s
            + self.settings.loop_period_s
            + 0.1
        )
        for wheel in self.settings.wheels:
            self._verify_still_armed()
            control = self.controls[wheel.motor_id]
            if time.monotonic() - control.last_feedback_at > max_feedback_age:
                raise HatError(f"Motor {wheel.motor_id} feedback became stale")
            motor_target = targets[wheel.motor_id] * wheel.polarity
            counts = control.command(motor_target, request.current_cap_a, time.monotonic(), self.settings)
            feedback = self.hat.set_current_raw(wheel.motor_id, counts)
            self._check_feedback(feedback)
            control.accept(feedback, time.monotonic())

    def log_status(self, state: str, request: DriveRequest | None, snapshot: CRSFSnapshot) -> None:
        now = time.monotonic()
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        if request is None:
            LOG.info("%s; link quality=%s", state, snapshot.link_quality)
        else:
            wheels = " ".join(
                f"{wheel.motor_id}:{self.controls[wheel.motor_id].last_rpm:+d}rpm/"
                f"{self.controls[wheel.motor_id].measured_current_a:+.2f}A"
                for wheel in self.settings.wheels
            )
            LOG.info("%s; throttle=%.2f steer=%+.2f limit=%.2fA LQ=%s %s",
                     state, request.throttle, request.steering, request.current_cap_a,
                     snapshot.link_quality, wheels)

    def run(self) -> None:
        self.open_hat()
        LOG.info("Motor HAT ready. Radio must be healthy, throttle zero, then arm switch low -> high.")
        try:
            while self._running:
                began = time.monotonic()
                snapshot = self.radio.snapshot()
                state = self.gate.observe(snapshot, time.monotonic())

                if state in ("link_lost", "stop_requested", "disarmed"):
                    LOG.warning("%s; stopping all configured motors", state)
                    self.stop_all()
                elif state == "newly_armed":
                    try:
                        if self.hat is None:
                            self.open_hat()
                        self._verify_still_armed(require_neutral=True)
                        self.prepare_current_mode()
                        self._verify_still_armed(require_neutral=True)
                        LOG.info("Armed")
                    except (RadioLost, ShutdownRequested):
                        self.gate.invalidate()
                        self.stop_all()
                        LOG.warning("Radio changed during arming; arm again after switch low")
                elif state == "armed":
                    request = drive_request(snapshot, self.settings)
                    try:
                        self.drive(request)
                    except (RadioLost, ShutdownRequested) as exc:
                        self.gate.invalidate()
                        self.stop_all()
                        LOG.warning("%s; arm again after switch low", exc)
                    else:
                        self.log_status(state, request, snapshot)
                else:
                    self.log_status(state, None, snapshot)

                delay = self.settings.loop_period_s - (time.monotonic() - began)
                if delay > 0:
                    time.sleep(delay)
        finally:
            self.stop_all()


class RadioLost(RuntimeError):
    pass


class ShutdownRequested(RuntimeError):
    pass


def monitor_radio(settings: Settings) -> None:
    with CRSFReader(settings.radio_port, baudrate=settings.radio_baud) as radio:
        LOG.info("Listening to receiver; no motor connection will be opened")
        if settings.motor_backend == "fast":
            LOG.warning("Verify physical SB changes the configured profile channel through three distinct raw ranges. Example: SB CH6; SA arm CH5; SC reverse CH8; SD stop CH7. Confirm these EdgeTX mappings on your radio.")
        while True:
            snapshot = radio.snapshot()
            now = time.monotonic()
            if snapshot.channels is None:
                LOG.info("Waiting for RC channels; LQ=%s error=%s", snapshot.link_quality, snapshot.error)
            else:
                mapped = {
                    name: round(channel(snapshot, number), 2)
                    for name, number in vars(settings.channels).items() if number is not None
                }
                LOG.info("healthy=%s LQ=%s controls=%s raw_CH1_to_CH16=%s", radio_healthy(snapshot, now, settings),
                         snapshot.link_quality, mapped, snapshot.channels)
            time.sleep(0.5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("ports", "check", "monitor-radio", "run"))
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--telemetry", type=Path, default=Path("robot_telemetry.csv"),
                        help="Fast backend CSV log written during run")
    parser.add_argument("--live-status", action=argparse.BooleanOptionalAction, default=True,
                        help="Display local measured profile, protection, holding and stop feedback")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        if args.command == "ports":
            try:
                from serial.tools import list_ports
            except ImportError as exc:
                raise RuntimeError("Install pyserial: python3 -m pip install -r requirements.txt") from exc
            found = list(list_ports.comports())
            for port in found:
                print(f"{port.device}\t{port.description}\t{port.hwid}")
            if not found:
                LOG.info("No serial devices found")
            return 0
        settings = load_settings(args.config)
        if args.command == "check":
            LOG.info("Config valid: backend %s, motor IDs %s, motor port %s, radio port %s",
                     settings.motor_backend,
                     [wheel.motor_id for wheel in settings.wheels],
                     settings.motor_port, settings.radio_port)
            return 0
        if args.command == "monitor-radio":
            monitor_radio(settings)
            return 0

        with CRSFReader(settings.radio_port, baudrate=settings.radio_baud,
                        telemetry_enabled=settings.radio_telemetry_enabled,
                        telemetry_hz=settings.radio_telemetry_hz) as radio:
            if settings.motor_backend == "fast":
                from fast_robot import FastRobot
                robot = FastRobot(settings, radio, telemetry_path=args.telemetry, live_status=args.live_status)
            else:
                robot = Robot(settings, radio)
            signal.signal(signal.SIGINT, robot.request_shutdown)
            signal.signal(signal.SIGTERM, robot.request_shutdown)
            robot.run()
        return 0
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, HatError) as exc:
        LOG.error("Robot stopped: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
