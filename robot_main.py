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
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from crsf import CRSFReader, CRSFSnapshot
from ddsm115 import DDSMHat, Feedback, HatError, Mode, MotorFault


LOG = logging.getLogger("robot")
CRSF_LOW = 172
CRSF_HIGH = 1811


def _number(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be within {minimum}..{maximum}")
    return result


def _integer(value: Any, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer within {minimum}..{maximum}")
    return value


@dataclass(frozen=True)
class Wheel:
    motor_id: int
    side: str
    polarity: int


@dataclass(frozen=True)
class Channels:
    throttle: int
    steering: int
    current_dial: int
    arm: int
    reverse: int
    stop: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Channels":
        names = ("throttle", "steering", "current_dial", "arm", "reverse", "stop")
        values = {name: _integer(data[name], f"channels.{name}", 1, 16) for name in names}
        if len(set(values.values())) != len(values):
            raise ValueError("Every assigned control channel must be distinct")
        return cls(**values)


@dataclass(frozen=True)
class Settings:
    motor_port: str
    radio_port: str
    radio_baud: int
    wheels: tuple[Wheel, ...]
    channels: Channels
    max_rpm: int
    max_current_a: float
    neutral_braking_current_a: float
    acceleration_rpm_s: float
    deceleration_rpm_s: float
    steering_gain: float
    kp_a_per_rpm: float
    ki_a_per_rpm_s: float
    loop_period_s: float
    radio_timeout_s: float
    link_timeout_s: float
    motor_timeout_s: float
    hat_heartbeat_ms: int
    motor_backend: str
    hat_baud: int
    fast_period_ms: int
    fast_watchdog_ms: int
    fast_status_timeout_s: float
    stall_time_ms: int
    temp_limit_c: int
    ff_ma_per_rpm_s: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Settings":
        motor_port = data["motor_port"]
        radio_port = data["radio_port"]
        for name, value in (("motor_port", motor_port), ("radio_port", radio_port)):
            if not isinstance(value, str) or not value.strip() or "REPLACE" in value:
                raise ValueError(f"Set a real {name} in the config file")
        if motor_port == radio_port:
            raise ValueError("The radio and HAT must use different serial ports")

        wheel_data = data["wheels"]
        if not isinstance(wheel_data, list) or not 1 <= len(wheel_data) <= 4:
            raise ValueError("Configure one to four installed wheels")
        wheels: list[Wheel] = []
        for entry in wheel_data:
            mid = _integer(entry["id"], "wheel.id", 1, 4)
            side = entry["side"]
            if side not in ("left", "right"):
                raise ValueError("wheel.side must be left or right")
            polarity = _integer(entry["polarity"], "wheel.polarity", -1, 1)
            if polarity == 0:
                raise ValueError("wheel.polarity must be -1 or 1")
            wheels.append(Wheel(mid, side, polarity))
        if len({wheel.motor_id for wheel in wheels}) != len(wheels):
            raise ValueError("Wheel motor IDs must be unique")
        if {wheel.side for wheel in wheels} != {"left", "right"}:
            raise ValueError("At least one left and one right wheel are required")

        backend = data.get("motor_backend", "legacy")
        if backend not in ("legacy", "fast"):
            raise ValueError("motor_backend must be legacy or fast")
        if backend == "fast" and {wheel.motor_id for wheel in wheels} != {1, 2, 3, 4}:
            raise ValueError("Fast HAT firmware requires exactly motor IDs 1, 2, 3, and 4")
        hat_baud = _integer(data.get("hat_baud", 230400), "hat_baud", 115200, 1000000)
        if backend == "fast" and hat_baud != 230400:
            raise ValueError("Fast HAT firmware v1 uses 230400 baud on the Pi connection")

        # Waveshare quotes 1.25 A rated on its product page (1.5 A on its
        # wiki). Keep both transports within the conservative 1.2 A limit.
        max_current = _number(data["max_current_a"], "max_current_a", 0.1, 1.2)
        brake_current = _number(data["neutral_braking_current_a"], "neutral_braking_current_a", 0, 8.0)
        if brake_current > max_current:
            raise ValueError("neutral_braking_current_a cannot exceed max_current_a")
        loop_period = _number(data["loop_period_s"], "loop_period_s", 0.05, 1)
        radio_timeout = _number(data["radio_timeout_s"], "radio_timeout_s", 0.1, 2)
        link_timeout = _number(data["link_timeout_s"], "link_timeout_s", 0.2, 3)
        motor_timeout = _number(data["motor_timeout_s"], "motor_timeout_s", 0.05, 1)
        heartbeat = _integer(data["hat_heartbeat_ms"], "hat_heartbeat_ms", 100, 2000)
        if heartbeat / 1000 <= motor_timeout:
            raise ValueError("HAT heartbeat must exceed motor transaction timeout")

        return cls(
            motor_port=motor_port,
            radio_port=radio_port,
            radio_baud=_integer(data.get("radio_baud", 420000), "radio_baud", 9600, 1000000),
            wheels=tuple(wheels),
            channels=Channels.from_dict(data["channels"]),
            max_rpm=_integer(data["max_rpm"], "max_rpm", 1, 200 if backend == "fast" else 330),
            max_current_a=max_current,
            neutral_braking_current_a=brake_current,
            acceleration_rpm_s=_number(data["acceleration_rpm_s"], "acceleration_rpm_s", 1, 1000),
            deceleration_rpm_s=_number(data["deceleration_rpm_s"], "deceleration_rpm_s", 1, 2000),
            steering_gain=_number(data["steering_gain"], "steering_gain", 0, 1),
            kp_a_per_rpm=_number(data["kp_a_per_rpm"], "kp_a_per_rpm", 0, 1),
            ki_a_per_rpm_s=_number(data["ki_a_per_rpm_s"], "ki_a_per_rpm_s", 0, 1),
            loop_period_s=loop_period,
            radio_timeout_s=radio_timeout,
            link_timeout_s=link_timeout,
            motor_timeout_s=motor_timeout,
            hat_heartbeat_ms=heartbeat,
            motor_backend=backend,
            hat_baud=hat_baud,
            fast_period_ms=_integer(data.get("fast_period_ms", 15), "fast_period_ms", 10, 100),
            fast_watchdog_ms=_integer(data.get("fast_watchdog_ms", 300), "fast_watchdog_ms", 200, 1000),
            fast_status_timeout_s=_number(data.get("fast_status_timeout_s", 0.2), "fast_status_timeout_s", 0.05, 1),
            stall_time_ms=_integer(data.get("stall_time_ms", 1000), "stall_time_ms", 300, 5000),
            temp_limit_c=_integer(data.get("temp_limit_c", 70), "temp_limit_c", 40, 70),
            ff_ma_per_rpm_s=_integer(data.get("ff_ma_per_rpm_s", 0), "ff_ma_per_rpm_s", 0, 20),
        )


def load_settings(path: Path) -> Settings:
    with path.open(encoding="utf-8") as file:
        return Settings.from_dict(json.load(file))


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


def drive_request(snapshot: CRSFSnapshot, settings: Settings) -> DriveRequest:
    assigned = settings.channels
    throttle = (channel(snapshot, assigned.throttle) + 1) / 2
    throttle = max(0.0, (throttle - 0.04) / 0.96)
    steering = _deadband(channel(snapshot, assigned.steering), 0.04)
    # The Pocket's S1 dial reportedly reaches about -80%, not -100%.
    dial = channel(snapshot, assigned.current_dial)
    fraction = max(0.0, min(1.0, (dial + 0.8) / 1.8))
    return DriveRequest(
        throttle=throttle,
        steering=steering,
        reverse=channel(snapshot, assigned.reverse) > 0.5,
        current_cap_a=fraction * settings.max_current_a,
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
                if drive_request(snapshot, self.settings).throttle == 0:
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

    def request_shutdown(self, *_args: Any) -> None:
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
            feedback = self.hat.set_mode(wheel.motor_id, Mode.CURRENT)
            self._check_feedback(feedback)
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

    def _verify_still_armed(self) -> None:
        if not self._running:
            raise ShutdownRequested("Shutdown requested")
        snapshot = self.radio.snapshot()
        now = time.monotonic()
        if not radio_healthy(snapshot, now, self.settings):
            raise RadioLost("Radio frames or link statistics went stale")
        assigned = self.settings.channels
        if channel(snapshot, assigned.arm) <= 0.5 or channel(snapshot, assigned.stop) > 0.5:
            raise RadioLost("Arm switch released or stop switch requested")

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
                        self._verify_still_armed()
                        self.prepare_current_mode()
                        self._verify_still_armed()
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
        while True:
            snapshot = radio.snapshot()
            now = time.monotonic()
            if snapshot.channels is None:
                LOG.info("Waiting for RC channels; LQ=%s error=%s", snapshot.link_quality, snapshot.error)
            else:
                mapped = {
                    name: round(channel(snapshot, number), 2)
                    for name, number in vars(settings.channels).items()
                }
                LOG.info("healthy=%s LQ=%s channels=%s", radio_healthy(snapshot, now, settings),
                         snapshot.link_quality, mapped)
            time.sleep(0.5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("ports", "check", "monitor-radio", "run"))
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--telemetry", type=Path, default=Path("robot_telemetry.csv"),
                        help="Fast backend CSV log written during run")
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

        with CRSFReader(settings.radio_port, baudrate=settings.radio_baud) as radio:
            if settings.motor_backend == "fast":
                from fast_robot import FastRobot
                robot = FastRobot(settings, radio, telemetry_path=args.telemetry)
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
