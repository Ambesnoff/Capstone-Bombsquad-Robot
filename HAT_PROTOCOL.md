# Pi-to-HAT protocol, version 2

The Pi requests wheel speeds and Gentle / Normal / Boost. The ESP32 authorizes
motion, controls current, enforces protection and reports what it accepted and
applied. Version 1 peers are incompatible and must be updated together.

The XR4-to-Pi CRSF link is 420000 baud. Pi-to-HAT UART is 230400 baud, and
the separate DDSM115 RS-485 bus is 115200 baud. Wheel IDs remain 1 front left,
2 front right, 3 rear right, 4 rear left. Side and polarity are Pi configuration.

## Framing and freshness

All integers are little endian. The packed wire definitions are generated from
`robot_protocol.json` into `protocol_defs.py` and
`hat_firmware/robot_hat/protocol_v2.h`. Update the schema, run
`python3 generate_protocol.py`, and verify `python3 generate_protocol.py --check`.
The generator produces field offsets, size assertions and equivalent Python/C++
configuration range and relationship validators (the C++ one omits bounds its
wire type already guarantees, which GNU GCC rejects under `-Werror`).

| Field | Bytes | Value |
|---|---:|---|
| Sync | 2 | `A5 5A` |
| Version | 1 | `02` |
| Type | 1 | Command or STATUS type below |
| Sequence | 2 | Unsigned sender sequence, wraps after 65535 |
| Payload length | 1 | 0–240 |
| Payload | 0–240 | Type-specific |
| CRC | 2 | CRC-16/CCITT-FALSE, little endian, over version through payload |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR; `123456789` gives `0x29B1`. Maximum frame length is 249 bytes.
Noise, wrong versions, excessive lengths and CRC failures are discarded and the
parser resynchronizes. Unknown valid frame types cannot authorize motion.

A command is newer only when `0 < uint16(candidate - boundary) < 32768`.
Duplicate, old and exactly half-range sequences cannot refresh the command
watchdog. STATUS has its own sequence; old or duplicated STATUS cannot refresh
Pi feedback timestamps. Both sides clear serial input on startup. The host
discards an excessive receive backlog instead of presenting old buffered reports
as live measurements, then keeps receiving; motion requires a new ARM.

## Boot and session identities

Each ESP32 boot has a nonzero 64-bit boot identity. `HELLO` contains a fresh host
nonce and immediately inhibits motion. The HAT issues a fresh session token for
every HELLO and echoes the nonce in STATUS. The host accepts a handshake only
when nonce and command sequence match the HELLO it sent. HELLO establishes the
new session's sequence boundary; it does not configure or arm the robot.

CONFIG, ARM and TARGETS contain both boot identity and HAT-issued session token.
A captured command from a previous host session or ESP32 boot is ineligible.
Replaying HELLO can only inhibit motion and rotate the token; it cannot make
captured motion commands eligible. A Pi restart, HELLO or reconnect does not
refill Boost. An ESP32 reset begins with zero Boost budget.

STOP is unconditional and carries no session token. A stale STOP still disables
motion. It advances the freshness boundary only if its sequence is newer, and
never rewinds it. STATUS_REQ is observational and never changes the motion
freshness boundary or watchdog. This serial protocol provides freshness and
corruption detection; it is not an authenticated communications channel.

## Host commands

| Type | Name | Payload bytes | Meaning |
|---|---|---:|---|
| `01` | HELLO | 8 | Host nonce, unsigned 64-bit |
| `02` | CONFIG | 103 | Boot ID (8), session (8), config ID (4), complete config (83) |
| `03` | ARM | 20 | Boot ID (8), session (8), applied config ID (4) |
| `04` | TARGETS | 25 | Boot ID (8), session (8), four signed centi-RPM (8), profile (1) |
| `05` | STOP | 0 | Motion inhibition and mode-safe stop |
| `06` | STATUS_REQ | 0 | Request a current STATUS snapshot |

Profile values are `0=Gentle`, `1=Normal`, `2=Boost`. Targets are motor-ID order
1–4, with 0.01 RPM encoding and a maximum absolute target of configured max RPM
(no more than 200 RPM). Python rounds to the nearest centi-RPM with ties to even.
DDSM115 measured speed remains whole RPM; finer target encoding does not improve
sensor resolution. TARGETS carries a profile request, never an arbitrary current
command. A zero target for an inner wheel remains a driving target; whole-robot
neutral settling requires every target to be zero.

CONFIG identity is IEEE CRC32 of the 83-byte canonical config body, using
`0xEDB88320` reflected polynomial, initial/final XOR `0xFFFFFFFF`; zero CRC is
represented by identity 1. Firmware recomputes the identity before application.
The complete candidate is validated before any field changes. Application
requires motion inhibition, confirmed stopped and fresh fault-free feedback.
An acknowledgment identifies the particular CONFIG request, result and loaded
config identity. ARM binds that applied identity and requires fresh stationary,
fault-free reports. Firmware verifies each motor's current mode before issuing
any current setpoint, and preserves established authorized holding effort.

CONFIG starts with the following 35 unsigned 16-bit fields in order:

| Order | Name | Units | Default |
|---:|---|---|---:|
| 1 | max_rpm | RPM | 40 |
| 2 | max_current_ma | Independent current ceiling, mA | 2700 |
| 3 | neutral_brake_ma | mA | 300 |
| 4 | accel_rpm_s | RPM/s | 120 |
| 5 | decel_rpm_s | RPM/s | 180 |
| 6 | kp_ma_per_rpm | mA/RPM | 20 |
| 7 | ki_ma_per_rpm_s | mA/(RPM·s) | 4 |
| 8 | ff_ma_per_rpm_s | mA/(RPM/s) | 0 |
| 9 | watchdog_ms | TARGETS expiry, ms | 300 |
| 10 | control_period_ms | Requested control period, ms | 15 |
| 11 | stall_time_ms | Stall persistence, ms | 1000 |
| 12 | gentle_current_ma | mA | 800 |
| 13 | normal_current_ma | mA | 1500 |
| 14 | boost_current_ma | mA | 2500 |
| 15 | boost_capacity_ms | Applied Boost allowance, ms | 20000 |
| 16 | boost_refill_ms | Eligible time to refill an empty budget, ms | 60000 |
| 17 | temp_poll_ms | Per-wheel temperature query target, ms | 500 |
| 18 | temp_boost_stale_ms | Disable Boost above this temperature age, ms | 750 |
| 19 | temp_stop_stale_ms | Stop above this required temperature age, ms | 1500 |
| 20 | cooldown_ms | Cool, valid feedback dwell before thermal recovery/refill, ms | 3000 |
| 21 | cap_ramp_ma_s | Current envelope ramp, mA/s | 1000 |
| 22 | hold_current_ma | Separate holding ceiling, mA | 300 |
| 23 | hold_kp_ma_per_degree | Holding position gain, mA/degree | 2 |
| 24 | hold_ki_ma_per_degree_s | Holding integral gain, mA/(degree·s) | 1 |
| 25 | hold_damping_ma_per_rpm | Holding speed damping, mA/RPM | 20 |
| 26 | neutral_settle_ms | Neutral settling dwell, ms | 300 |
| 27 | feedback_timeout_ms | Required motor feedback age, ms | 150 |
| 28 | stall_target_centi_rpm | Stall target threshold, 0.01 RPM | 800 |
| 29 | stall_speed_centi_rpm | Stall measured-speed threshold, 0.01 RPM | 200 |
| 30 | stall_current_ma | Minimum effort for persistent stall detection, mA | 250 |
| 31 | abnormal_current_ma | Absolute reported-current threshold, mA | 2700 |
| 32 | abnormal_current_ms | Abnormal-current persistence, ms | 200 |
| 33 | abnormal_margin_ma | Reported current allowance above effective cap, mA | 400 |
| 34 | saturation_warn_ms | Saturation/speed-error warning persistence, ms | 500 |
| 35 | stop_verify_ms | Initial stop-confirmation deadline, ms | 1500 |

Then nine unsigned byte fields follow: `temp_warn_c=50`, `temp_derate_c=55`,
`temp_limit_c=65` (thermal stop), `temp_release_c=45`, `temp_hysteresis_c=3`,
`hold_enabled=1`, `disarmed_hold_enabled=0`, `stall_enabled=1`, `hold_temp_c=55`.
The final field is unsigned 32-bit `encoder_counts_per_rev=32768`, at body offset
79. Its allowed range is 256–65536. The default follows Waveshare's documented
DDSM115 position feedback (0–32767 per turn). A wrong value makes every wrap look
like a half-turn error, so holding drives the wheel; verify it on the assembled
motors (one raised-wheel turn) before tuning position holding.

Ranges and cross-field constraints are authoritative in `robot_protocol.json`.
Among them: profile caps must be ascending; holding and braking cannot exceed
the independent ceiling; thermal thresholds must ascend from release to warning,
derating and stop; polling must precede Boost stale and required stale ages;
control period must precede feedback timeout and command expiry. Boolean inputs
must be booleans in Python and 0/1 on the wire. Invalid settings cannot partly
apply. The compiled current ceiling remains 2700 mA. Defaults permit the complete
0.8 / 1.5 / 2.5 A candidate envelopes, subject to protection and Boost availability.
These settings are experiment parameters, not hardware qualification results.

## HAT STATUS, type `80`

STATUS payload is 184 bytes: an 84-byte header followed by four 25-byte wheel
records in motor-ID order. Firmware reports after control sweeps, operating
transitions and stop verification, plus STATUS_REQ. Transmission is bounded;
UART congestion cannot block motor control.

| Offset | Bytes | Header field | Meaning |
|---:|---:|---|---|
| 0 | 8 | boot_id | ESP32 boot identity |
| 8 | 8 | host_session | HAT-issued session token |
| 16 | 4 | capabilities | Supported features, bit mask |
| 20 | 4 | build_id | Build/source identity |
| 24 | 4 | config_id | Actually loaded configuration identity |
| 28 | 2 | accepted_seq | Most recently accepted command |
| 30 | 2 | applied_seq | Most recently applied command |
| 32 | 4 | accepted_ms | HAT uptime when accepted, ms |
| 36 | 4 | applied_ms | HAT uptime when applied, ms |
| 40 | 1 | state | Operating state |
| 41 | 1 | fault_code | Latched fault severity/cause |
| 42 | 1 | stop_state | Independent stop progress |
| 43 | 1 | requested_profile | Operator profile request |
| 44 | 1 | applied_profile | Profile HAT permits |
| 45 | 1 | config_result | Most recent CONFIG result |
| 46 | 4 | reason_flags | Warning/protection/reduction reasons |
| 50 | 4 | sweep_us | Most recent complete motor sweep duration, µs |
| 54 | 2 | command_age_ms | Accepted TARGETS age; 65535 unavailable/older |
| 56 | 4 | boost_remaining_ms | Remaining Boost allowance |
| 60 | 4 | boost_capacity_ms | Loaded budget capacity |
| 64 | 4 | boost_refill_remaining_ms | Eligible refill time required to reach full |
| 68 | 4 | cooldown_remaining_ms | Remaining cool/valid dwell |
| 72 | 1 | hold_flags | Active 1, limited 2, disarmed holding 4 |
| 73 | 1 | fault_wheel | 1–4 affected wheel, or 0 system/no wheel |
| 74 | 8 | hello_nonce | Echo of latest HELLO nonce |
| 82 | 2 | config_ack_seq | CONFIG request identified by config_result |

The refill-time field does not promise a running wall-clock countdown: refill
suspends while conditions are ineligible. Accepted/applied times are modulo
32-bit HAT uptime; elapsed time must use wrap-safe subtraction. TARGETS can be
accepted before application in the next motor sweep. ARM and successful CONFIG
acknowledgments must confirm application. STOP acknowledgment records inhibition
and does not imply that the wheels have stopped.

| Wheel offset | Bytes | Field | Meaning |
|---:|---:|---|---|
| 0 | 2 signed | target_centi_rpm | Applied/requested wheel target at 0.01 RPM |
| 2 | 2 signed | rpm | Motor-reported whole RPM |
| 4 | 2 signed | current_ma | Reported motor torque current |
| 6 | 2 | position_raw | Raw encoder feedback |
| 8 | 2 signed | temp_c | Reported temperature, whole °C |
| 10 | 2 | effective_cap_ma | Actual driving current envelope |
| 12 | 2 | hold_cap_ma | Actual separate holding envelope |
| 14 | 2 | age_ms | Speed/current feedback age |
| 16 | 2 | temp_age_ms | Independently measured temperature-feedback age |
| 18 | 1 | error | Motor error bits |
| 19 | 1 | mode | Reported motor mode: current 1, speed 2, position 3 |
| 20 | 1 | validity | RPM 1, current 2, position 4, temperature 8 |
| 21 | 4 | reason_flags | Wheel-specific warning/protection reasons |

A validity bit, not a plausible numeric value, determines whether a measurement
is present. Ages saturate at 65535, which the host exposes as unavailable. Speed
and current can remain valid while temperature is stale. Position validity is
independent and must be fresh before holding. The host requires every wheel's
fresh valid speed/current feedback to establish stationarity. Motor current is
not battery current.

## States and reasons

Operating state values are `0=BOOT_STOPPING`, `1=DISARMED`, `2=ARMED/DRIVING`,
`3=FAULT`, `4=SETTLING`, `5=HOLDING`, `6=STOPPING`. Motion remains inhibited
when faulted or explicitly disarmed holding. Stop state is separate:
`0=NONE`, `1=REQUESTED`, `2=IN_PROGRESS`, `3=CONFIRMED`, `4=UNCONFIRMED`.
A fault can coexist with confirmed or unconfirmed stop. Bounded polling continues
while faulted and while stop is unconfirmed; later fresh stationary reports may
confirm the stop.

CONFIG results are `0=NONE`, `1=APPLIED`, `2=REJECTED_STATE`,
`3=REJECTED_VALUE`, `4=REJECTED_SESSION`. Rejection reports config_ack_seq and
never advances the accepted motion sequence or changes loaded config identity.
config_result and config_ack_seq change only for CONFIG frames; rejected motion
frames and old-sequence CONFIG frames leave them unchanged.

Fault codes are `0=none`, `1=command expiry`, `2=motor timeout`,
`3=invalid motor frame`, `4=motor error`, `5=overspeed`, `6=overtemperature`,
`7=stall`, `8=abnormal current`, `9=configuration fault`,
`10=required temperature stale`, `11=control progress failure`.

Reason bit masks are:

| Bit | Mask | Reason |
|---:|---:|---|
| 0 | 1 | Independent firmware/config ceiling |
| 1 | 2 | Boost budget empty |
| 2 | 4 | Temperature stale / unavailable for Boost |
| 3 | 8 | Thermal warning |
| 4 | 16 | Thermal derating |
| 5 | 32 | Thermal stop |
| 6 | 64 | Required motor feedback stale |
| 7 | 128 | Motor error |
| 8 | 256 | Persistent saturation / stall warning |
| 9 | 512 | Validated persistent stall |
| 10 | 1024 | Abnormal reported current |
| 11 | 2048 | Command expiry |
| 12 | 4096 | Holding limited / unavailable |
| 13 | 8192 | Cooldown incomplete |
| 14 | 16384 | Configuration rejected |
| 15 | 32768 | Session mismatch |
| 16 | 65536 | Persistent speed error |
| 17 | 131072 | Inspection required; fault latched until ESP32 reset |
| 18 | 262144 | Motor reply retried in the last 1000 ms; informational |

Capability bits are profiles 1, sessions 2, holding 4, independent temperature
age 8, stop confirmation 16, config identity 32, reset watchdog 64, position 128.
The v2 host requires all these features before configuration/arming.

## Protection and stop semantics

Boost consumes one millisecond of budget per millisecond while Boost is applied,
even at low load. It refills linearly outside Boost at
`capacity_ms / refill_ms` budget milliseconds per elapsed millisecond, only after
cooldown with fresh, fault-free, sufficiently cool wheel feedback. Profile
switching, CONFIG, HELLO and Pi reconnects cannot manufacture budget. Expiry,
stale temperature or derating selects the permitted lower profile; reason and
per-wheel effective caps expose that fallback. Cap changes ramp and use
controller anti-windup. Holding has its own current and thermal limits and is
not released simply because a profile falls back.

STOP discards targets, pending arm work and stale profile requests, returning
the requested profile to Gentle without changing the Boost budget or cooldown.
Repeated STOP while verification is pending preserves its original deadline
and wheel observations. Firmware queries unknown wheel
modes before sending zero, switches to speed mode and verifies it before speed
zero. A zero setpoint in unknown/position mode is never assumed safe. Fresh
stationary reports from all four IDs confirm stopping; a transmitted command,
radio frame or motor mode write alone cannot prove it. A confirmed stop that is
later disturbed (for example a pushed wheel) starts a new `stop_verify_ms`
window; motion that persists past it latches an inspection fault naming the
wheel. The host separately
invalidates pending ARM writes and late ARM acknowledgments when STOP begins.

A missed or garbled motor reply is retried at once, 3 attempts in all (wrong-mode,
interrupted and motor-error replies are answers and are not retried). While driving
or holding, a wheel that fails all 3 is skipped for that sweep: no health check on
stale data, and the targets are not reported applied. Its next sweep re-sends the
last acknowledged current without running its controller; control resumes after a
valid reply. It faults only after `feedback_timeout_ms` without one (`3` invalid
motor frame if its latest failed attempt was garbled, else `2` motor timeout).
Arming, stop and disarmed polling fault after the third failed attempt. Reason bit
18 marks a wheel's failed attempt for 1000 ms and gates no arming, motion or Boost.

Overtemperature, required feedback loss, motor errors, command expiry, persistent
stall and abnormal current inhibit motion automatically. Recoverable faults
clear after a confirmed stop plus `cooldown_ms` of fresh healthy feedback; only
overtemperature additionally waits for every motor to reach `temp_release_c`.
Each recovery still requires a deliberate rearm; stale targets are discarded. The firmware progress watchdog covers a wedged control
path. Independent motor-power removal remains a hardware safety requirement;
software cannot provide powered holding after power removal.

`tests/test_fast_hat.py` covers host transport and races.
`tests/test_protocol_schema.py` compares checked definitions, fixed golden frames,
native packed layout/CRC and configuration boundaries across Python/C++.
Firmware simulator and commissioning checks are separate. Passing software
checks does not establish stopping distance, temperature margin, slope retention
or motor/encoder scale on the assembled robot.
