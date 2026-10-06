# Drive state and recovery contract

The Pi requests motion; the HAT owns operating transitions and protective limits. A radio frame, transmitted STOP, or acknowledged sequence does not prove that a wheel has stopped.

| Operating state | Motion authorization and behavior |
| --- | --- |
| BOOT / STOPPING | No drive permission. Discard previous targets, query actual motor modes, use only mode-safe zero commands, and poll all four wheels. |
| DISARMED | Motion inhibited. Configuration can be applied only with confirmed fresh stationary feedback and a complete valid candidate. |
| SETTLING | Armed with whole-chassis neutral. Apply bounded braking; once every wheel stays at or below 6 RPM for the settle dwell, enter holding if configured (a slope keeps a coasting wheel just above the 2 RPM stationary threshold). |
| DRIVING / ARMED | Apply fresh targets through independent wheel speed/current loops. An individual zero target during a turn is a driving target. |
| HOLDING | Use measured position drift and damping within independent holding/current/thermal limits. Report whether hold is active, disarmed, or limited. |
| FAULT | Inhibit drive, retain cause and wheel, continue bounded stop-mode repair/verification. Fault severity and stop confirmation remain independent. |

`STOP REQUESTED`, `IN PROGRESS`, `CONFIRMED`, and `UNCONFIRMED` are separate status values. Confirmation needs fresh feedback from every configured wheel in a known non-position mode, within the stated stationary threshold. Missing feedback or expired verification leaves an explicit unconfirmed stop. Polling continues while faulted so later fresh feedback can establish confirmation without authorizing movement.

## Entry and exit rules

- Boot, HELLO/new session, operator stop, disarm, command expiry, and fault discard previous targets and arming permission. STOP discards the previous profile request and returns it to Gentle without granting Boost allowance or resetting cooldown. Repeated STOP preserves the original verification deadline. A replayed HELLO can only inhibit movement; its old motion packets no longer match the HAT-issued session.
- Arming requires a healthy radio, neutral throttle/steering, deliberate arm low then high, an accepted configuration, matching capabilities/session, and fresh stationary healthy wheel feedback. Each motor must report current mode after a mode write before any setpoint is sent. Powered holding, when enabled while disarmed, retains its anchor/effort through arming.
- Neutral applies to all wheel targets. Enter position-assisted holding only after whole-chassis settling. Leaving hold resets stale drive integration; entering hold captures its anchor. Use configured encoder counts/wrap behavior, and measure actual encoder semantics during commissioning.
- Mode changes alter only the permitted current envelope. Increases are ramped; reductions honor protective ceilings and anti-windup. Gentle selection and Boost expiry do not silently release independent required holding effort. Fault or thermal stop can remove holding and must be visible.
- The HAT owns Boost tokens. After reset the budget is zero. It consumes while Boost is applied; a held exhausted Boost request cannot refill itself. Refill requires leaving Boost, fresh fault-free cool feedback, and cooldown dwell. Pi restart, HELLO, or switch cycling does not grant a fresh budget.
- Warning/derating thresholds use hysteresis. Thermal stop and required-feedback loss inhibit driving immediately according to configured freshness/persistence rules. Holding has separate thermal/current limits but does not override protective stops.
- A missed or garbled motor reply is retried at once (3 attempts). While driving or holding, a wheel that misses all 3 is skipped for that sweep and sent its last acknowledged current again next sweep, with normal control resuming after a valid reply; it faults only after the feedback timeout. Arming, stop and disarmed polling fault after the third failed attempt. The informational `REPLY_RETRY` reason lasts 1 s and gates nothing.

## Recovery

Radio loss, recoverable communication faults, command expiry, and stale temperature return to readiness after a confirmed stop and the cooldown dwell of fresh healthy feedback; overtemperature also waits for every motor to cool to the release temperature. Readiness is not permission to drive: the Pi requires a new neutral arm cycle. Motor errors, abnormal current, validated stall, and other inspection faults remain inhibited until the cause is resolved, the HAT is reset, and the Pi supervisor is restarted. Stop confirmation can change while the cause remains latched.

A starting or reconnecting supervisor waits, motion-inhibited and visible, while the HAT clears a recoverable fault or cannot yet confirm a stop (for example, motor power off at the independent cutoff). Only a silent HAT or an inspection-level fault ends the session attempt and uses the bounded service restarts.

The Pi is the sole motor-command writer. Shutdown handlers record cancellation; pending arm/target paths check it so they cannot restore motion after stopping is requested. HAT and radio readers publish snapshots. Logging, local feedback, and optional CRSF return telemetry consume those snapshots with bounded queues; they never issue competing motor commands.

The ESP32 progress watchdog uses reset/panic behavior and is fed by completed bounded control/stop/poll work. A busy loop or stalled I/O must not be allowed to masquerade as control progress. Physical watchdog hang/reset-to-stop behavior remains a commissioning measurement. Hardware cutoff removes motor power independently and can remove powered holding.
