"""Assign and check DDSM115 wheel IDs with one motor connected at a time.

This utility uses the Pi's serial connection to the ESP32 side of the
Waveshare HAT. It never sends a motion command. The physical one-motor
requirement cannot be checked in software, so every operation requires an
explicit acknowledgement.
"""

from __future__ import annotations

import argparse
import sys
from typing import Callable, Optional, Sequence, TextIO

from ddsm115 import DDSMHat, Feedback, HatError


WHEEL_POSITIONS = {
    1: "front left",
    2: "front right",
    3: "rear right",
    4: "rear left",
}


def wheel_id(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("wheel ID must be 1, 2, 3, or 4") from exc
    if number not in WHEEL_POSITIONS:
        raise argparse.ArgumentTypeError("wheel ID must be 1, 2, 3, or 4")
    return number


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Discover, assign, or verify one DDSM115 motor ID at a time."
    )
    result.add_argument("--port", default="/dev/serial0", help="Pi HAT serial port (default: /dev/serial0)")
    commands = result.add_subparsers(dest="command", required=True)
    for name in ("discover", "assign", "verify"):
        command = commands.add_parser(name)
        command.add_argument(
            "--one-motor-only", action="store_true",
            help="acknowledge that exactly one motor is connected and no other controller is active",
        )
        if name != "discover":
            command.add_argument("id", type=wheel_id, help="wheel ID, 1 through 4")
    return result


def acknowledge_one_motor(
    confirmed: bool, input_fn: Callable[[str], str], err: TextIO
) -> bool:
    print(
        "WARNING: Disconnect all but ONE motor from the HAT and stop any other "
        "controller before continuing. ID discovery and assignment are unsafe "
        "on a multi-motor bus. Keep the wheel stationary and power cutoff accessible.",
        file=err,
    )
    if confirmed:
        return True
    try:
        answer = input_fn("Type ONE MOTOR to confirm the wiring: ")
    except EOFError:
        answer = ""
    if answer.strip() != "ONE MOTOR":
        print("Cancelled: one-motor acknowledgement was not given.", file=err)
        return False
    return True


def report(feedback: Feedback, out: TextIO) -> None:
    position = WHEEL_POSITIONS.get(feedback.motor_id, "unassigned wheel position")
    temperature = (
        f", temperature={feedback.temperature_c} C"
        if feedback.temperature_c is not None else ""
    )
    print(
        f"ID {feedback.motor_id} ({position}): mode={feedback.mode.name.lower()}, "
        f"speed={feedback.rpm} RPM, fault=0x{feedback.error:02X}{temperature}",
        file=out,
    )


def main(
    argv: Optional[Sequence[str]] = None,
    *,
    hat_factory: Callable[..., DDSMHat] = DDSMHat,
    input_fn: Callable[[str], str] = input,
    out: TextIO = sys.stdout,
    err: TextIO = sys.stderr,
) -> int:
    args = parser().parse_args(argv)
    if not acknowledge_one_motor(args.one_motor_only, input_fn, err):
        return 2

    try:
        with hat_factory(port=args.port) as hat:
            found = hat.get_id()
            report(found, out)
            if found.error:
                raise HatError(f"Motor fault 0x{found.error:02X}; inspect the motor before setup")

            if args.command == "discover":
                return 0

            if args.command == "assign":
                if found.motor_id != args.id:
                    if found.rpm != 0:
                        raise HatError("Motor is moving; wait for 0 RPM before changing its ID")
                    print(f"Assigning ID {args.id} ({WHEEL_POSITIONS[args.id]})...", file=out)
                    changed = hat.set_id(args.id)
                    if changed.motor_id != args.id:
                        raise HatError("ID change could not be verified")
                else:
                    print("Motor already has the requested ID; no ID write sent.", file=out)
            elif found.motor_id != args.id:
                raise HatError(f"Expected ID {args.id}, but the connected motor reports ID {found.motor_id}")

            checked = hat.get_feedback(args.id)
            if checked.motor_id != args.id:
                raise HatError(f"Expected ID {args.id}, but feedback reports ID {checked.motor_id}")
            report(checked, out)
            if checked.error:
                raise HatError(f"Motor fault 0x{checked.error:02X}; inspect the motor")
            print(f"Verified ID {args.id} ({WHEEL_POSITIONS[args.id]}).", file=out)
            return 0
    except (HatError, OSError, RuntimeError) as exc:
        print(f"Setup failed: {exc}", file=err)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
