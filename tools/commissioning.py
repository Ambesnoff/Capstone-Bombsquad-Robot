"""Create trial records and refuse qualification without measured acceptance."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0


def qualification_errors(record: dict) -> list[str]:
    errors = []
    for name in ("motor_power_cutoff_tested", "computer_power_preserved"):
        if record.get("hardware", {}).get(name) is not True:
            errors.append(f"hardware.{name} has not passed")
    hardware = record.get("hardware", {})
    if hardware.get("motor_ids") != [1, 2, 3, 4]:
        errors.append("hardware.motor_ids must identify the four commissioned wheels")
    if not number(hardware.get("load_kg")) or not hardware.get("terrain"):
        errors.append("Record measured load and terrain")
    release = record.get("release", {})
    for name in ("firmware_build_id", "configuration_id", "release_sha256"):
        if not release.get(name):
            errors.append(f"release.{name} missing")
    expected_checks = json.loads((ROOT / "commissioning/trial.example.json").read_text())["checks"]
    for name in expected_checks:
        if record.get("checks", {}).get(name) is not True:
            errors.append(f"checks.{name} has not passed")
    targets = record.get("acceptance_targets", {})
    measures = record.get("measurements", {})
    for target, measure, minimum in (
        ("max_stop_time_s", "stop_time_s", False),
        ("max_stop_distance_m", "stop_distance_m", False),
        ("max_hold_drift_m", "hold_drift_m", False),
        ("hold_duration_s", "hold_duration_s", True),
        ("max_temperature_c", "peak_temperature_c", False),
    ):
        limit, actual = targets.get(target), measures.get(measure)
        if not number(limit) or not number(actual):
            errors.append(f"Measured {measure} and numerical target {target} required")
        elif (actual < limit if minimum else actual > limit):
            errors.append(f"{measure}={actual} fails {target}={limit}")
    for name in ("peak_motor_current_a", "worst_temperature_age_ms", "worst_sweep_ms", "worst_command_application_ms"):
        if not number(measures.get(name)):
            errors.append(f"measurements.{name} missing")
    ceiling = record.get("effective_current_ceiling_a")
    if not number(ceiling):
        errors.append("Record the effective current ceiling")
    elif ceiling > 1.2 and record.get("higher_current_acceptance") is not True:
        errors.append("Separate measured electrical/thermal acceptance required above 1.2 A")
    if record.get("unresolved_conditions"):
        errors.append("Unresolved conditions must be resolved or explicitly constrain a separate release")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("new", "check"))
    parser.add_argument("record", type=Path)
    args = parser.parse_args(argv)
    if args.command == "new":
        args.record.parent.mkdir(parents=True, exist_ok=True)
        with args.record.open("x", encoding="utf-8") as out:
            out.write((ROOT / "commissioning/trial.example.json").read_text())
        print(f"Created unqualified trial: {args.record}")
        return 0
    record = json.loads(args.record.read_text())
    errors = qualification_errors(record)
    if errors:
        print("NOT QUALIFIED\n" + "\n".join(f"- {error}" for error in errors))
        return 1
    print("RECORDED TARGETS PASSED; acceptance applies only to this recorded hardware/load/terrain/configuration")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
