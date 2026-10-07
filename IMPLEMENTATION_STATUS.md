# Architecture implementation and verification

This is the software implementation of the attached **Robot architecture & implementation plan, revision 1.1**, in [Ambesnoff/Capstone-Bombsquad-Robot](https://github.com/Ambesnoff/Capstone-Bombsquad-Robot). It was developed on `gpt/architecture-v2`, audited and fixed in PR #2 (see `AUDIT.md`), and merged into `main` by PR #1. The exact original runtime is preserved on the `baseline/original-v1` branch.

The fast example implements the user's updated requirements: **SB selects Gentle / Normal / Boost**, mapped to CH6 by the example EdgeTX mixes, and the **1.2 A ceiling is removed**. Profile caps are 800 / 1500 / 2500 mA, with a separate 2700 mA absolute ceiling. Current is a permitted envelope, not constant commanded current. Boost starts empty on an ESP32 reset and becomes available through the configured cool, fault-free refill policy. Protection or an exhausted budget can reduce the applied mode; the live view and logs explain that reduction.

## Shared 250 rpm update

The `codex/shared-250-rpm` change extends Pi/HAT validation to 250 rpm, updates the fast example, and adds requirement tests. Its generated definitions and firmware identity were refreshed and its source was reviewed, but tests, firmware compilation and hardware trials were not run for this change. Earlier verification records below describe earlier builds; they do not certify this update. Upload steps are in [the update guide](docs/UPDATE_250_RPM.md).

## Delivered work

| Plan requirement | Implementation and evidence |
| --- | --- |
| Reproducible original baseline and Git isolation | Original archive verified with all 44 original tests; separate baseline and GPT branches; `.gitignore`, `requirements.txt`, `hat_firmware/toolchain.lock.json`, `tools/release.py` |
| Pi stop race and command ownership | `fast_robot.py`, `robot_main.py`, `fast_hat.py`; shutdown cancellation, sole supervisor writer, rechecks around pending ARM/targets, transport stop generation; regression tests reproduce the original second-snapshot race and stop during pending ARM |
| Fault-independent stop verification | `operating_state.h`, `FastRobot._wait_disarmed`; bounded wheel polling continues while faults are latched; fresh four-wheel stationary feedback required independently of fault severity |
| Mode-safe transitions and old STOP | Mode write, query, confirm before current setpoints; old STOP always inhibits without moving freshness backward; native tests for wrong mode, missing replies, replay, and sequence rollover. A missed or garbled reply is retried (3 attempts); a driving wheel that misses all is skipped and faults only after `feedback_timeout_ms`, with an informational `REPLY_RETRY` reason |
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
| Commissioning, qualification, and cleanup | `COMMISSIONING.md`, `tools/commissioning.py`, explicit unqualified trial template, setup/control/handoff instructions; factory motor-ID tools retained |

## Software verification

The complete Python test suite, generated-definition check, firmware-source identity check, actual ESP32 compilation, and fresh-checkout verification are release gates.

- **93 tests pass**, including the actual firmware harness's **45 deterministic scenarios**. The original archived baseline separately passed its 44 tests. All four subsequent code-review findings are fixed; see `CODE_REVIEW_FIXES.md` for changes and regression evidence.
- Protocol generation, firmware source identity, both example configuration checks and Git whitespace checks pass.
- The original implementation was verified in a separate fresh Git clone with all 86 then-current tests, both generated/source identity checks, both configuration checks, and actual ESP32 compilation. The review fixes pass the updated 93-test suite and the checks listed in `CODE_REVIEW_FIXES.md`.
- Actual board: `esp32:esp32:esp32`, Arduino-ESP32 **3.3.12**, ESP-IDF **5.5.5**.
- Firmware source SHA256: `bf82bb7d88316ec81bf876dae3cf9849ce8f771e8ab5dc602da510937d7eebe5`; reported build ID `0xbf82bb7d`.
- Firmware compilation after review fixes: **283444 bytes flash, 23068 bytes RAM**.
- **Superseded by the audit fixes (PR #2, see `AUDIT.md`):** the firmware sources changed, so the build ID is now `0x4ff06011` and the ESP32 compile above no longer applies. Recompile with Arduino-ESP32 3.3.12 and record the new sizes. `python3 tools/verify.py` (Python 3.14.8) runs the software checks under Clang and GNU GCC.
- **Superseded again by the reply-retry change:** the firmware sources changed, so the build ID is now `0xca638f3b` (3395522363). This change has only been compiled for the native simulator; recompile with Arduino-ESP32 3.3.12 before flashing.
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
- Measure the encoder feedback period/scaling and temperature encoding. `encoder_counts_per_rev=32768` follows Waveshare's documented 0–32767 position range but is still an **unverified commissioning assumption**, not a measured hardware fact; modular arithmetic is tested for 32768 and 65536.
- Measure bus/sweep/command timing, freshness, missed deadlines, wheel direction, loaded drive response, stopping distance/time, hold drift, reversal and turning.
- Record numerical acceptance criteria, load and surface; measure current/temperature peaks, repeated Boost cycles, stalls, connection/power faults and slopes. Normal and Boost require separate measured electrical/thermal acceptance.
- Refine settings from those records, then qualify each hardware/configuration/load/surface combination. Retire legacy driving only after acceptance; keep `motor_setup.py` and `ddsm115.py` for factory motor-ID setup.

No hardware record is marked qualified and no powered trial was performed. Retire the legacy drive only after fast-path hardware acceptance, and keep `motor_setup.py` and `ddsm115.py` for factory motor-ID work.
