# Architecture v2 audit

Audit of PR #1 (`gpt/architecture-v2` at `46027ac`) against the *Robot architecture & implementation plan*, revision 1.1. Fixes are on `claude/v2-audit-fixes` (PR #2). No motor hardware was operated. Physical behavior below comes from the native firmware simulator and a Pi-to-firmware co-simulation: behavioral evidence, not calibration.

## Bottom line

PR #1 keeps the plan's architecture. The Pi is the single motor-command writer. The HAT owns current envelopes, the Boost budget, thermal protection, holding and stop verification. Protocol v2 binds every motion command to boot, session and configuration identity. Arming and stopping verify motor modes before any setpoint, stop polling continues while faulted, and logging is bounded and nonblocking. Every software claim in the PR reproduces.

It was not ready for commissioning. Five defects would have appeared on the robot: holding that spins a wheel at neutral, recovery lockouts that leave a warm robot unusable after ordinary glitches, a permanent lockout from a bumped wheel, neutral creep on slopes, and a supervisor that dies after a 60 ms scheduling stall. PR #2 fixes them, each with a regression test that fails on the old code and passes now. What remains is physical acceptance (plan stage 4), which software cannot close.

## Baselines

- `main` (`3a156ec`) is a partial development upload, not the pre-plan code. The true pre-plan code is `baseline/original-v1` (`7c16be4`); its 44 tests pass.
- PR #1 is 2 commits, 50 files, +5231/−1515 against `main`.

## Findings

High: unintended motion, or the robot cannot recover in normal use. Medium: plan deviation, or an availability or diagnostic defect. Low: robustness, clarity or test gaps.

| # | Severity | Finding | Evidence | Status |
|---|---|---|---|---|
| 1 | High | The default `encoder_counts_per_rev=65536` contradicts Waveshare's documented DDSM115 position range (0–32767 per turn). Each wrap looks like a half-turn error, so holding drives the wheel at its 300 mA cap in the direction it was already drifting. It re-triggers every revolution and raises no fault. | Simulator: 499° in 10 s and 3117° in 60 s at neutral; 3.5° with 32768. The repository's simulator wrapped at 65536, the same assumption, so its tests could not catch this. | Fixed: default 32768, the simulator models the documented range, `encoder_wrap_hold`. Confirm on hardware. |
| 2 | High | Command-expiry, motor-timeout, bad-frame and stale-temperature faults cleared only when every motor was at or below 45 °C. Only overtemperature should wait for cooling. | Simulator: no recovery in 30 s at 47 °C; recovery in 3.1 s at 44 °C. | Fixed in firmware, `warm_command_recovery`. |
| 3 | High | `_establish_session` refused any HAT fault. After a Pi restart while armed, every new process failed in about 0.6 s and `run_service.py` used up its 5 starts in about 36 s. | Co-simulation with the real supervisor and firmware, motors at 50 °C: all 5 starts fail. At 25 °C it costs one extra restart. | Fixed in the Pi: waits, motion inhibited and reason visible, for recoverable faults and for a stop the HAT cannot yet confirm (for example motor power off at the cutoff during startup, which also exhausted the restarts). |
| 4 | Medium-High | Neutral settling never reached holding on moderate slopes. A coasting wheel stays just above the 2 RPM stationary threshold, so the robot creeps downhill indefinitely at neutral. The earlier audit's neutral-creep defect persisted in a new form. | Simulator slope: about 570° in 30 s, never HOLDING. | Fixed: holding engages once every wheel stays at or below 6 RPM, `slope_settle` (34° peak drift). Tune hold gains during commissioning. |
| 5 | Medium | Moving a disarmed wheel for 100 ms at 8 RPM, any time after the 1.5 s stop window, latched an inspection-level `MOTOR_FAULT` with wheel 0. Only an ESP32 reset cleared it. | Simulator. | Fixed: a new verification window; persistent motion still latches and names the wheel, `disarmed_push`. |
| 6 | Medium | FastHat's reader thread exited for good when more than 772 bytes were buffered, which is 60 ms of armed status traffic. A brief Pi stall then needed a process restart, counted against the 5-start budget. | Experiment. | Fixed in the Pi. |
| 7 | Medium | When the first cause was recoverable and a later motor error required inspection (review fix 1), STATUS carried only the recoverable code. The Pi waited forever and told the operator "fault 1". | Code and the `stop_motor_fault` scenario. | Fixed: reason bit 17 plus a Pi latch. |
| 8 | Medium | The native tests did not build with GNU GCC: `-Werror` plus `-Wtype-limits` on the generated validator. The 45-scenario evidence could not be reproduced on the Pi or on Linux CI. | `g++-16`: 14 errors. | Fixed in the generator; both compilers verified. |
| 9 | Medium | The Pi's unconfirmed-stop latch depended on sampling: it latched only if the 15 ms loop happened to read that status, and then lasted until a process restart. | Co-simulation. | Fixed in the Pi: deterministic, inhibits while unconfirmed. |
| 10 | Medium | In the default configuration (disarmed holding off), arming releases the speed-loop hold, contrary to "Arming should preserve established holding effort". | Simulator slope: 10–20° of roll during arming; the hold is preserved with disarmed holding on. | Documented. Policy decision: disarmed holding keeps motor current on while parked. |
| 11 | Medium | Channel remap. In v1, CH6 was reverse; v2 puts the profile on CH6 and reverse on CH8. With an un-updated radio model the old reverse switch selects Boost. | Example configurations. | Human check in `HUMAN_CHECKS.txt`. |
| 12 | Low | A motion frame with the wrong session overwrote `config_result`. The Pi's per-cycle configuration check then failed and forced a new handshake. | `config_result_scope`. | Fixed. |
| 13 | Low | The firmware hand-copied fault codes, reason bits, states and CONFIG offsets with nothing checking them against the generated contract. | Code. | Fixed: compile-time checks and a by-name CONFIG copy. |
| 14 | Low | The stale-status read at the end of each Pi loop iteration was outside the error handling and could end the process. | Code and experiment. | Fixed in the Pi. |
| 15 | Low | One implausible motor temperature, such as 251, made the Pi reject every STATUS frame. | Code. | Fixed in the Pi. |
| 16 | Low | The CSV log wrote enums through `str()`, which changes between Python versions. | Python 3.9 and 3.14 runs. | Fixed in the Pi: explicit integers. |
| 17 | Low | With no C++ compiler the suite reported OK while the native tests were skipped. | Experiment. | Fixed: `ROBOT_REQUIRE_NATIVE` and `tools/verify.py`. |
| 18 | Low | `profiles.independent_ceiling_a` duplicated `max_current_a`. | Configuration. | Fixed: removed. |
| 19 | Low | `STATUS_REQ` consumes Pi sequence numbers without advancing the HAT's freshness boundary. After about 32768 requests (about 8 minutes of unconfirmed-stop polling) the HAT treats the Pi's next commands as stale. STOP still works. | Code. | Open, low impact. |
| 20 | Low | The `COOLDOWN` reason is set whenever any motor is above 45 °C, so the live view shows WARNING during normal warm running. | Code. | Open. |
| 21 | Low | While the supervisor waits for HAT recovery during session setup, the live view and an event are produced but no CSV rows. | Code. | Open. |
| 22 | Info | Design choices to confirm: derating drops straight to the Gentle level at 55 °C; Boost drains while parked with the switch in Boost; Boost refills only at or below 45 °C; a zero inner wheel in a turn gets only the 0.3 A neutral-brake cap; hold gains are soft (2 mA/degree). | Simulator. | Open: commissioning or policy. |

## Test-suite effectiveness

Mutation testing applied 26 single-point changes to firmware and Pi safety logic; the suite caught 19.

- **Real gaps.** Boost was not checked to be disabled during derating, and the firmware could ignore motor-reported temperature without any test failing. Both are now covered by `derate_disables_boost` and `warm_command_recovery`. The shutdown checks in the armed TARGETS path are covered by a new Pi test.
- **Layered defenses.** Each guard against a STOP arriving during ARM is redundant with another; tests catch removing two layers, not one.
- **Equivalent mutants.** The arm-edge flag is already enforced by `saw_arm_low`; the anti-windup condition is dominated by the integral clamp; the duplicate command-expiry check is backed by the transaction-level check.

## Plan compliance

| Plan requirement | Status |
|---|---|
| One Pi command owner; background radio, HAT reader and logging | Met |
| HAT owns profiles, current envelopes, Boost budget, thermal limits, holding and stopping | Met |
| Pi stop race, faulted stop verification, mode-safe arming, old STOP freshness | Met, with regression tests |
| Watchdog that resets on lost control progress | Implemented (task watchdog with panic); reset-to-stop time not measured |
| Protocol v2 identities, capabilities, accepted versus applied, per-wheel validity and ages, stop progress | Met; inspection latch added (#7) |
| Gentle / Normal / Boost: configured channel, calibrated ranges, debounce, invalid input falls back to Gentle | Met; channel remap is a human check (#11) |
| HAT-owned Boost budget with no reset or reconnect entitlement; refill outside Boost when cool and fault-free | Met |
| Thermal warning 50 °C, derating 55 °C, stop 65 °C; temperature ages 500 / 750 / 1500 ms | Met (derating is a step, #22) |
| Separate holding limits; Gentle and Boost expiry do not release holding | Met. Arming releases the hold by default (#10); slope settling (#4) and wrap (#1) fixed |
| Recovery restores readiness, never motion | Met after #2, #3, #5 and #9 |
| Bounded, nonblocking logging | Met; CSV serialization made explicit (#16) |
| Shared, checked protocol definitions; cross-language golden frames | Met; firmware constants now checked (#13) |
| Simulator with continuous physics, failing assertions and injected faults | Met. No end-to-end Pi-plus-firmware test was kept from the earlier harness (recommended below) |
| Reproducible checks from a fresh checkout; ESP32 compile | Checks reproduce under Clang and GCC. The ESP32 compile could not be reproduced on the audit machine (no toolchain), and the changed firmware needs a fresh compile |
| Independent motor-power stop | Design document only, as expected at this stage |

## Verification performed

- Reproduced PR #1's claims: 93 tests on Python 3.14.7 and 3.9.6; 45 firmware scenarios at -O0, -O1, -O2 and under ASan with UBSan; generator, build-identity and configuration checks; the 23 transport tests on 3.9.6; the 44 original tests on `baseline/original-v1`.
- The integer sanitizer flagged 19 firmware sites; all are intentional modular arithmetic.
- PR #2: `tools/verify.py --allow-python-mismatch` passes under Apple clang and GNU GCC 16.2: 125 tests each, with no failures, errors or skips. Full verification requires Python 3.14.8; only 3.14.7 was available.
- Simulated stop latency: every wheel receives zero current 7–15 ms after STOP, and the full stop sequence takes 32–44 ms. The status link runs at 53% of its capacity while armed.

## Needs your decision

- Deleting `releases/robot-v1-baseline.zip` (byte-identical to `baseline/original-v1`) and `CLEANUP.md`. Deletion was blocked for the auditor; done after the merge.
- Whether disarmed holding should be on by default (#10).
- When to retire the legacy backend. The plan says after fast-path acceptance.
- Adding a CI workflow that runs `tools/verify.py` on Python 3.14.8 with GCC. The repository has no CI today.

## Hardware-only acceptance

These cannot be closed in software: the independent cutoff; watchdog reset-to-stop time; the encoder range (one raised-wheel turn); temperature encoding below 0 °C; sweep and command timing; stopping distance and holding drift; current and thermal envelopes for Normal and Boost; ELRS telemetry display.

## Recommendations not implemented

- Port the earlier end-to-end harness (real supervisor, real firmware, chassis physics) into the test suite. The co-simulation used in this audit found defects that the unit tests missed.
- Hand the braking effort over to the hold integrator when SETTLING becomes HOLDING, to reduce drift on slopes.
- Advance or rate-limit `STATUS_REQ` sequence use (#19), and flag `COOLDOWN` only when it blocks something (#20).
