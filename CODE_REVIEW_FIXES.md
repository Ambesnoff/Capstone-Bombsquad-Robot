# Robot architecture review fixes

Implemented October 1, 2026 on `gpt/architecture-v2`, addressing all four
findings against commit `f2fe51981a0b852d063fc62626470c5a60288649` in
[draft PR #1](https://github.com/Ambesnoff/Capstone-Bombsquad-Robot/pull/1).

## Changes

1. **Motor errors retain reset lockout.** Firmware records inspection-required
   severity independently of the first diagnostic cause. Every valid motor
   reply, including stop transactions, can latch this severity. A later healthy
   reply, cooldown, STOP, CONFIG, or HELLO cannot clear it. Communication-only
   faults retain their existing recovery behavior.
2. **Status decoding supports older enum behavior.** Fault values are validated
   with `FaultCode(value)`; decoding no longer depends on integer containment in
   an enum. Unknown fault values still raise `ValueError` and are rejected by
   the reader without terminating it.
3. **Configuration changes invalidate pending ARM.** Accepted CONFIG cancels
   pending arming. ARM execution captures and rechecks its boot/session/config
   identity and sequence before and after motor IO, preventing an outdated
   request from enabling motion or moving the accepted sequence backward.
4. **Shutdown publishes stop outcomes.** The public view publishes local stop
   intent before transport IO, observed progress while verifying, and confirmed
   or unconfirmed outcomes before stopping consumers. Verification and write
   failures expose their inspection fault. The display wakes on phase changes
   and drains the final view; repeated progress does not flood output. Console
   work stays on its consumer thread, and shutdown retains its bounded join.

## Verification

- All **93 unittest tests pass** on Python **3.14.7**.
- The actual firmware runs **45 deterministic native scenarios**: 37 existing
  and extended protection scenarios plus eight ARM interleaving scenarios.
- All **23 transport/protocol tests pass** on the available Python **3.9.6**,
  including decoding and handshake. A regression enforces pre-3.12 enum
  containment behavior. Python 3.10/3.11 are not installed locally, so no direct
  runs on those versions are claimed; the declared minimum remains 3.10.
- New regressions reject original firmware/supervisor/decoder copies: three
  motor-error cases, both ARM/CONFIG cases (including sequence wrap), shutdown
  confirmation/failure, and enum decoding.
- Protocol generation, firmware identity, fast/legacy example configuration,
  and Git whitespace checks pass.
- ESP32 Dev Module compilation succeeds with Arduino-ESP32 **3.3.12**:
  **283444 bytes flash**, **23068 bytes RAM**.
- Firmware build ID: `0xbf82bb7d`. Source SHA256:
  `bf82bb7d88316ec81bf876dae3cf9849ce8f771e8ab5dc602da510937d7eebe5`.

No motor hardware was operated. Physical commissioning and qualification
remain pending as described in `COMMISSIONING.md`.
