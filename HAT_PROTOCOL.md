# Fast Pi-to-HAT protocol, version 1

This protocol is used only by `fast_hat.py` and `hat_firmware/robot_hat/robot_hat.ino`.
The XR4 remains attached to the Raspberry Pi. The HAT's motor-side RS-485 bus
remains at the DDSM115's documented 115200 baud. Its Pi-side UART runs at
230400 baud in this firmware; both sides must use the same rate.

All integer fields are little endian. Each frame is:

| Field | Bytes | Value |
| --- | ---: | --- |
| Sync | 2 | `A5 5A` |
| Version | 1 | `01` |
| Type | 1 | See below |
| Sequence | 2 | Incremented by the sender, wraps at 65535 |
| Payload length | 1 | 0 through 64 |
| Payload | 0–64 | Type-specific |
| CRC | 2 | CRC-16/CCITT-FALSE, little endian, over version through payload |

The receiver discards invalid frames and resynchronizes at the next sync pair.
A valid but unknown type receives no action. A sender must use a fresh sequence
number for each new command. Duplicate or older commands cannot refresh the
motor-command watchdog. Both endpoints discard serial input accumulated before
their session begins.

Pi-to-HAT types:

* `01 HELLO`: empty. Begin a mode-safe stop, reset command-sequence freshness
  for a new host session, and return STATUS once the stop sequence has run.
  HELLO is accepted regardless of its sequence number; it never authorizes
  motion. The HAT remains in STOPPING until fresh feedback proves all four
  wheels stationary, then enters DISARMED.
* `02 CONFIG`: 23 bytes, in this order: unsigned 16-bit maximum RPM, maximum
  current mA, neutral braking current mA, acceleration RPM/s, deceleration
  RPM/s, proportional gain mA/RPM, integral gain mA/(RPM*s), acceleration
  feedforward gain mA/(RPM/s), watchdog ms, control period ms, stall duration
  ms; then unsigned 8-bit temperature limit C. This configuration is accepted
  only while disarmed and all four motors report stationary. The firmware also
  enforces its compiled hard limits.
* `03 ARM`: empty. Accepted only after CONFIG, with four fresh fault-free
  stationary motor reports and no latched HAT fault.
* `04 TARGETS`: 10 bytes: signed 16-bit target RPM for IDs 1, 2, 3, 4 followed
  by unsigned 16-bit current cap mA. Accepted only while armed. A newly
  accepted TARGETS frame refreshes the HAT motor-command watchdog.
* `05 STOP`: empty. Latches motion disabled. It first checks each wheel's
  mode without commanding motion, sends a zero setpoint only to wheels
  confirmed in current or speed mode, switches all wheels to speed mode, and
  sends speed zero after verifying that mode. This avoids accidentally
  treating zero as a position target. The STOP command has priority over
  other pending host commands and enters DISARMED only after fresh stopped
  feedback from all four IDs.
* `06 STATUS_REQ`: empty. Requests a status frame.

HAT-to-Pi type `80 STATUS`, 36-byte payload:

* Unsigned 16-bit last accepted command sequence.
* Unsigned 8-bit state: `0=BOOT/STOPPING`, `1=DISARMED`, `2=ARMED`, `3=FAULT`.
* Unsigned 8-bit fault code: `0=none`, `1=command timeout`, `2=motor timeout`,
  `3=bad motor frame`, `4=motor fault`, `5=overspeed`, `6=overtemperature`,
  `7=stall`, `8=abnormal current`, `9=configuration fault`.
* Unsigned 16-bit last full four-wheel sweep time in microseconds (saturated
  at 65535).
* Unsigned 16-bit age in milliseconds of the last accepted TARGETS command
  (saturated at 65535).
* Four seven-byte wheel records in ID order 1, 2, 3, 4: signed 16-bit actual
  RPM, signed 16-bit reported torque current in mA, signed 8-bit temperature
  Celsius (127 means unavailable), unsigned 8-bit motor error, unsigned 8-bit
  feedback age in milliseconds (255 means unavailable/older).

The HAT publishes STATUS after each full control sweep, after the stop
sequence, during stop verification, and in response to STATUS_REQ. The Pi
treats missing or stale STATUS,
unexpected fault or disarmed state, or unacknowledged commands as a latched
drive fault. Neither endpoint treats a valid radio frame as proof that a motor
has stopped; motor reports are required for that conclusion. A HAT motor or
communication fault remains latched until HAT power is cycled after inspection.
