"""Validated, immutable robot configuration in explicit SI and wire units."""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _keys(data: dict[str, Any], allowed: set[str], name: str) -> None:
    unexpected = set(data) - allowed
    if unexpected:
        raise ValueError(f"Unknown {name} fields: {', '.join(sorted(unexpected))}")


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


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be true or false")
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
    arm: int
    reverse: int
    stop: int
    current_dial: int | None = None
    profile: int | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any], backend: str = "legacy") -> Channels:
        data = _object(data, "channels")
        _keys(data, set(cls.__dataclass_fields__), "channels")
        required = {"throttle", "steering", "arm", "reverse", "stop"}
        required.add("profile" if backend == "fast" else "current_dial")
        if not required <= set(data):
            raise ValueError(f"Missing channel mapping: {', '.join(sorted(required - set(data)))}")
        values = {name: _integer(value, f"channels.{name}", 1, 16) for name, value in data.items()}
        if len(set(values.values())) != len(values):
            raise ValueError("Every assigned control channel must be distinct")
        return cls(**values)


@dataclass(frozen=True)
class ProfileSettings:
    gentle_a: float = 0.8
    normal_a: float = 1.5
    boost_a: float = 2.5
    independent_ceiling_a: float = 1.2
    boost_capacity_s: float = 20.0
    boost_refill_s: float = 60.0
    cap_ramp_a_s: float = 1.0
    debounce_s: float = 0.12
    gentle_range: tuple[int, int] = (150, 350)
    normal_range: tuple[int, int] = (850, 1150)
    boost_range: tuple[int, int] = (1650, 1850)

    @classmethod
    def from_dict(cls, raw: Any) -> ProfileSettings:
        data = _object(raw, "profiles")
        _keys(data, set(cls.__dataclass_fields__), "profiles")
        result: dict[str, Any] = {}
        for key, default in asdict(cls()).items():
            value = data.get(key, default)
            if key.endswith("_range"):
                if not isinstance(value, (list, tuple)) or len(value) != 2:
                    raise ValueError(f"profiles.{key} must contain two raw CRSF values")
                low, high = (_integer(v, f"profiles.{key}", 0, 2047) for v in value)
                if low >= high:
                    raise ValueError(f"profiles.{key} must be an increasing range")
                result[key] = (low, high)
            else:
                limits = (0.1, 2.7) if key.endswith("_a") else (0.05, 10) if key == "cap_ramp_a_s" else (0, 1) if key == "debounce_s" else (1, 65)
                result[key] = _number(value, f"profiles.{key}", *limits)
        if not result["gentle_a"] <= result["normal_a"] <= result["boost_a"]:
            raise ValueError("Profile current caps must increase Gentle <= Normal <= Boost")
        if result["boost_refill_s"] < result["boost_capacity_s"]:
            raise ValueError("Boost refill time must be at least its capacity")
        ranges = [result[f"{name}_range"] for name in ("gentle", "normal", "boost")]
        if any(a[1] >= b[0] for a, b in zip(ranges, ranges[1:])):
            raise ValueError("Calibrated switch ranges must be separate and increasing")
        return cls(**result)


@dataclass(frozen=True)
class ProtectionSettings:
    temp_warn_c: int = 50
    temp_derate_c: int = 55
    temp_release_c: int = 45
    temp_hysteresis_c: int = 3
    cooldown_dwell_ms: int = 3000
    temp_boost_age_ms: int = 750
    temp_stop_age_ms: int = 1500
    hold_current_a: float = 0.3
    hold_temp_c: int = 55
    hold_kp_ma_per_degree: int = 2
    hold_ki_ma_per_degree_s: int = 1
    hold_damping_ma_per_rpm: int = 20
    hold_enabled: bool = True
    stall_enabled: bool = True
    temp_poll_ms: int = 500
    feedback_timeout_ms: int = 150
    neutral_settle_ms: int = 300
    disarmed_hold: bool = False
    stall_target_rpm: int = 8
    stall_speed_rpm: int = 2
    stall_current_a: float = 0.25
    abnormal_current_a: float = 2.7
    abnormal_current_ms: int = 200
    abnormal_margin_a: float = 0.4
    saturation_warn_ms: int = 500
    stop_verify_ms: int = 1500

    @classmethod
    def from_dict(cls, raw: Any, temp_limit_c: int, ceiling: float) -> ProtectionSettings:
        data = _object(raw, "protection")
        _keys(data, set(cls.__dataclass_fields__), "protection")
        result: dict[str, Any] = {}
        for key, default in asdict(cls()).items():
            value = data.get(key, default)
            if isinstance(default, bool):
                result[key] = _boolean(value, f"protection.{key}")
            elif key.endswith("_a"):
                result[key] = _number(value, f"protection.{key}", 0 if key == "hold_current_a" else 0.1, 2.7)
            else:
                upper = 65000 if key.endswith("_ms") else 100 if key.endswith("_pct") else 2000 if "_ma_per_" in key else 200 if key.endswith("_rpm") else 100
                result[key] = _integer(value, f"protection.{key}", 0, upper)
        if not result["temp_release_c"] < result["temp_warn_c"] < result["temp_derate_c"] < temp_limit_c:
            raise ValueError("Temperature thresholds must be release < warning < derating < stop")
        if not 0 < result["temp_hysteresis_c"] < result["temp_derate_c"] - result["temp_release_c"]:
            raise ValueError("Temperature hysteresis is invalid")
        if not 0 < result["temp_boost_age_ms"] < result["temp_stop_age_ms"]:
            raise ValueError("Temperature age limits must be 0 < Boost age < stop age")
        if result["hold_current_a"] > ceiling or result["hold_temp_c"] >= temp_limit_c:
            raise ValueError("Holding limits must respect the independent current ceiling and thermal stop")
        if result["stall_speed_rpm"] >= result["stall_target_rpm"]:
            raise ValueError("Stall detection requires lower measured speed and positive effort threshold")
        if result["cooldown_dwell_ms"] < 100 or result["temp_stop_age_ms"] < 100:
            raise ValueError("Cooldown dwell and feedback stop age must be at least 100 ms")
        return cls(**result)


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
    profiles: ProfileSettings = ProfileSettings()
    protection: ProtectionSettings = ProtectionSettings()
    radio_telemetry_enabled: bool = False
    radio_telemetry_hz: float = 2.0
    telemetry_queue_rows: int = 32
    live_status_hz: float = 1.0

    @classmethod
    def from_dict(cls, raw: Any) -> Settings:
        data = _object(raw, "configuration")
        _keys(data, set(cls.__dataclass_fields__), "configuration")
        backend = data.get("motor_backend", "legacy")
        if backend not in ("legacy", "fast"):
            raise ValueError("motor_backend must be legacy or fast")
        for name in ("motor_port", "radio_port"):
            value = data[name]
            if not isinstance(value, str) or not value.strip() or "REPLACE" in value:
                raise ValueError(f"Set a real {name} in the config file")
        if data["motor_port"] == data["radio_port"]:
            raise ValueError("The radio and HAT must use different serial ports")
        wheel_data = data["wheels"]
        if not isinstance(wheel_data, list) or not 1 <= len(wheel_data) <= 4:
            raise ValueError("Configure one to four installed wheels")
        wheels = []
        for entry in wheel_data:
            entry = _object(entry, "wheel")
            _keys(entry, {"id", "side", "polarity"}, "wheel")
            mid = _integer(entry["id"], "wheel.id", 1, 4)
            if entry["side"] not in ("left", "right"):
                raise ValueError("wheel.side must be left or right")
            polarity = _integer(entry["polarity"], "wheel.polarity", -1, 1)
            if polarity == 0:
                raise ValueError("wheel.polarity must be -1 or 1")
            wheels.append(Wheel(mid, entry["side"], polarity))
        if len({w.motor_id for w in wheels}) != len(wheels):
            raise ValueError("Wheel motor IDs must be unique")
        if {w.side for w in wheels} != {"left", "right"}:
            raise ValueError("At least one left and one right wheel are required")
        if backend == "fast" and {w.motor_id for w in wheels} != {1, 2, 3, 4}:
            raise ValueError("Fast HAT firmware requires exactly motor IDs 1, 2, 3, and 4")
        hat_baud = _integer(data.get("hat_baud", 230400), "hat_baud", 115200, 1000000)
        if backend == "fast" and hat_baud != 230400:
            raise ValueError("Fast HAT firmware uses 230400 baud on the Pi connection")
        profiles = ProfileSettings.from_dict(data.get("profiles", {}))
        max_current = _number(data["max_current_a"], "max_current_a", 0.1, 1.2 if backend == "legacy" else 2.7)
        brake = _number(data["neutral_braking_current_a"], "neutral_braking_current_a", 0, 2.7)
        if brake > (min(max_current, profiles.independent_ceiling_a) if backend == "fast" else max_current):
            raise ValueError("neutral_braking_current_a cannot exceed max_current_a or the independent ceiling")
        temp_limit = _integer(data.get("temp_limit_c", 65), "temp_limit_c", 40, 70)
        protection = ProtectionSettings.from_dict(data.get("protection", {}), temp_limit, profiles.independent_ceiling_a)
        motor_timeout = _number(data["motor_timeout_s"], "motor_timeout_s", 0.05, 1)
        heartbeat = _integer(data["hat_heartbeat_ms"], "hat_heartbeat_ms", 100, 2000)
        if heartbeat / 1000 <= motor_timeout:
            raise ValueError("HAT heartbeat must exceed motor transaction timeout")
        return cls(
            motor_port=data["motor_port"], radio_port=data["radio_port"], wheels=tuple(wheels),
            channels=Channels.from_dict(data["channels"], backend), motor_backend=backend, hat_baud=hat_baud,
            radio_baud=_integer(data.get("radio_baud", 420000), "radio_baud", 9600, 1000000),
            max_rpm=_integer(data["max_rpm"], "max_rpm", 1, 200 if backend == "fast" else 330),
            max_current_a=max_current, neutral_braking_current_a=brake,
            acceleration_rpm_s=_number(data["acceleration_rpm_s"], "acceleration_rpm_s", 1, 1000),
            deceleration_rpm_s=_number(data["deceleration_rpm_s"], "deceleration_rpm_s", 1, 2000),
            steering_gain=_number(data["steering_gain"], "steering_gain", 0, 1),
            kp_a_per_rpm=_number(data["kp_a_per_rpm"], "kp_a_per_rpm", 0, 1),
            ki_a_per_rpm_s=_number(data["ki_a_per_rpm_s"], "ki_a_per_rpm_s", 0, 1),
            loop_period_s=_number(data["loop_period_s"], "loop_period_s", 0.05, 1),
            radio_timeout_s=_number(data["radio_timeout_s"], "radio_timeout_s", 0.1, 2),
            link_timeout_s=_number(data["link_timeout_s"], "link_timeout_s", 0.2, 3),
            motor_timeout_s=motor_timeout, hat_heartbeat_ms=heartbeat,
            fast_period_ms=_integer(data.get("fast_period_ms", 15), "fast_period_ms", 10, 100),
            fast_watchdog_ms=_integer(data.get("fast_watchdog_ms", 300), "fast_watchdog_ms", 200, 1000),
            fast_status_timeout_s=_number(data.get("fast_status_timeout_s", 0.2), "fast_status_timeout_s", 0.05, 1),
            stall_time_ms=_integer(data.get("stall_time_ms", 1000), "stall_time_ms", 300, 5000),
            temp_limit_c=temp_limit,
            ff_ma_per_rpm_s=_integer(data.get("ff_ma_per_rpm_s", 0), "ff_ma_per_rpm_s", 0, 20),
            profiles=profiles, protection=protection,
            radio_telemetry_enabled=_boolean(data.get("radio_telemetry_enabled", False), "radio_telemetry_enabled"),
            radio_telemetry_hz=_number(data.get("radio_telemetry_hz", 2), "radio_telemetry_hz", 0.1, 10),
            telemetry_queue_rows=_integer(data.get("telemetry_queue_rows", 32), "telemetry_queue_rows", 1, 4096),
            live_status_hz=_number(data.get("live_status_hz", 1), "live_status_hz", 0.1, 10),
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["wheels"] = [{"id": w.motor_id, "side": w.side, "polarity": w.polarity} for w in self.wheels]
        result["channels"] = {key: value for key, value in vars(self.channels).items() if value is not None}
        return result

    @property
    def configuration_identity(self) -> str:
        canonical = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(canonical.encode()).hexdigest()


def load_settings(path: Path) -> Settings:
    with path.open(encoding="utf-8") as file:
        return Settings.from_dict(json.load(file))
