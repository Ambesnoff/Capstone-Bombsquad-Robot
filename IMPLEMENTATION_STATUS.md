# Architecture implementation and verification

This is the software implementation of the attached **Robot architecture & implementation plan, revision 1.1**. Development is isolated on `gpt/architecture-v2` in [Ambesnoff/Capstone-Bombsquad-Robot](https://github.com/Ambesnoff/Capstone-Bombsquad-Robot). Files retain their repository paths. Synced references and the Desktop Claude checkout are untouched. `main` contained a partial development upload before this task's Git work; it has not been advanced by this task. The exact original runtime is preserved on `baseline/original-v1` and in `releases/robot-v1-baseline.zip`.

The fast example implements the user's updated requirements: **SB selects Gentle / Normal / Boost**, mapped to CH6 by the example EdgeTX mixes, and the **1.2 A ceiling is removed**. Profile caps are 800 / 1500 / 2500 mA, with a separate 2700 mA absolute ceiling. Current is a permitted envelope, not constant commanded current. Boost starts empty on an ESP32 reset and becomes available through the configured cool, fault-free refill policy. Protection or an exhausted budget can reduce the applied mode; the live view and logs explain that reduction.

## Delivered work

| Plan requirement | Implementation and evidence |
| --- | --- |
| Reproducible original baseline and Git isolation | Original archive verified with all 44 original tests; separate baseline and GPT branches; `.gitignore`, `requirements.txt`, `hat_firmware/toolchain.lock.json`, `tools/release.py` |
| Pi stop race and command ownership | `fast_robot.py`, `robot_main.py`, `fast_hat.py`; shutdown cancellation, sole supervisor writer, rechecks around pending ARM/targets, transport stop generation; regression tests reproduce the original second-snapshot race and stop during pending ARM |
| Fault-independent stop verification | `operating_state.h`, `FastRobot._wait_disarmed`; bounded wheel polling continues while faults are latched; fresh four-wheel stationary feedback required independently of fault severity |
| Mode-safe transitions and old STOP | Mode write, query, confirm before current setpoints; old STOP always inhibits without moving freshness backward; native tests for wrong mode, missing replies, replay, and sequence rollover |
| Progress watchdog | ESP32 task-watchdog user with one-second timeout and panic/reset enabled; feed only after bounded completed work; actual board compilation and native reboot-entry test |
| Protocol v2 | `robot_protocol.json`, checked generated Python/C++ definitions, `fast_hat.py`, `host_protocol.h`, `HAT_PROTOCOL.md`; boot/session/config/build/capability identities, accepted/applied sequence and timing, exact configuration acknowledgment, separate validity/ages/stop progress |
| Strict complete configuration | `robot_config.py`, `generate_protocol.py`; immutable settings, unit/range/type/relationship validation shared with firmware, stable configuration identity, stopped application requirement |
| Three-position modes | `robot_radio.py`, `robot_config.py`, HAT protection controller; configurable disjoint raw switch ranges and debounce, invalid input falls back to Gentle; independent current envelope, cap ramp and anti-windup |
| Boost and thermal enforcement | HAT-owned 20-second capacity / 60-second refill, no reset entitlement or reconnect replenishment, held exhausted Boost cannot self-refill; warning/derate/stop, freshness, hysteresis, release and cooldown policies |
| Neutral, turning, and holding | Separate driving/settling/holding states; independent hold current/temperature bounds; encoder-assisted drift correction and configurable wrap; an individual zero wheel remains driving; optional disarmed hold preserves its effort when arming |
| Recovery without automatic motion | New sessions discard requests; radio/communication/thermal recovery restores readiness and requires neutral low-to-high arming; inspection faults remain inhibited; `deploy/run_service.py` bounds restart attempts and backoff |
| Live feedback and radio return | Background local view shows requested/applied mode, caps, current, RPM, temperatures/ages/trend, wheel errors, stop state, holding limits, Boost and reasons; optional bounded CRSF RPM, temperature, and flight-mode telemetry |
| Nonblocking logs | Unique CSV + complete configuration/session metadata + events, bounded queue, visible dropped rows/events and storage errors; no disk operation on the command path; tests inject blocked and failed storage |
| Improved simulator and shared contract checks | Actual firmware runs with independent continuous wheel physics, deterministic time, failing assertions and injected errors; fixed Python/C++ golden frames and full field-boundary/relationship agreement tests |
| Independent motor-power stop | `INDEPENDENT_STOP.md` documents actual supplied schematic topology, separately fused computer branch, normally open DC contactor, latching stop and manual reset, backfeed checks and loss of powered holding |
| Commissioning, qualification, and cleanup | `COMMISSIONING.md`, `tools/commissioning.py`, explicit unqualified trial template, setup/control/handoff instructions, `CLEANUP.md`; factory motor-ID tools retained |

## Software verification

The complete Python test suite, generated-definition check, firmware-source identity check, actual ESP32 compilation, and fresh-checkout verification are release gates.

- **86 tests pass**, including the actual firmware harness's **34 deterministic scenarios**. The original archived baseline separately passes its 44 tests.
- Protocol generation, firmware source identity, both example configuration checks and Git whitespace checks pass.
- A separate fresh Git clone repeats all 86 tests, both generated/source identity checks, both configuration checks, and actual ESP32 compilation successfully; its checkout stays clean.
- Actual board: `esp32:esp32:esp32`, Arduino-ESP32 **3.3.12**, ESP-IDF **5.5.5**.
- Firmware source SHA256: `7e039d421fba12bb221168ff74cfab72d3c1e0ae8f755a948403818c98966aea`; reported build ID `0x7e039d42`.
- Fresh-checkout firmware compilation: **283080 bytes flash, 23044 bytes RAM** (the primary development checkout compiled to 283144 bytes flash).
- The native simulator is behavioral evidence. Its mechanics, temperature evolution and watchdog reboot modeling are not calibration of the real robot.

Reproduce from the checkout:

```sh
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
python3 generate_protocol.py --check
python3 hat_firmware/generate_build_identity.py --check
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.12 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32 hat_firmware/robot_hat
python3 tools/release.py
```

## Work requiring the assembled robot

Stages 4–6 require physical measurements. They cannot be truthfully completed from source code or simulation. The modes, telemetry, recovery, commissioning records and acceptance checker are implemented so those trials use the completed control behavior.

- Install and verify independent motor-power removal, computer supply/backfeed behavior, power-loss retention, and actual watchdog reset-to-stop timing.
- Verify the actual SB channel and its three received values. Verify receiver telemetry routing and Pocket/EdgeTX discovery/display for each wheel's RPM/temperature and drive-state text.
- Measure the encoder feedback period/scaling and temperature encoding. `encoder_counts_per_rev=65536` is an **unverified commissioning assumption**, not a measured hardware fact; modular arithmetic is tested for 32768 and 65536.
- Measure bus/sweep/command timing, freshness, missed deadlines, wheel direction, loaded drive response, stopping distance/time, hold drift, reversal and turning.
- Record numerical acceptance criteria, load and surface; measure current/temperature peaks, repeated Boost cycles, stalls, connection/power faults and slopes. Normal and Boost require separate measured electrical/thermal acceptance.
- Refine settings from those records, then qualify each hardware/configuration/load/surface combination. Retire legacy driving only after acceptance; keep `motor_setup.py` and `ddsm115.py` for factory motor-ID setup.

No hardware record is marked qualified, no powered trial was performed, and no files were deleted. See `CLEANUP.md` for removable generated files and conditional legacy retirement.
