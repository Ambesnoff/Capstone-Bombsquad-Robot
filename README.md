# Four-wheel robot: protocol v2 commissioning build

The Raspberry Pi reads the XR4, supervises deliberate arming, mixes wheel targets, and records status. The Waveshare HAT's ESP32 controls the four DDSM115 wheels and independently enforces current profiles, Boost allowance, thermal protection, holding, command expiry, and stopping.

Start with [ROBOT_SETUP_GUIDE.md](ROBOT_SETUP_GUIDE.md). The Python files and firmware keep their existing repository paths on **gpt/architecture-v2**; all ongoing edits use this separate Git checkout. `sources/` remains read-only. [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) maps the architecture plan to delivered files and remaining physical acceptance work.

## Controls

The mode control is the **RadioMaster Pocket SB three-position switch**. A physical switch name is not a channel number: assign these mixes in your EdgeTX Robot model and check actual received values before enabling the motors.

| Channel | Physical source | Meaning |
| --- | --- | --- |
| CH1 | Steering stick | Centered skid steering, no pivot at zero throttle |
| CH3 | Throttle stick | Fully low is neutral; higher requests RPM |
| CH5 | SA latching switch | Low, then a fresh high edge at neutral to arm |
| CH6 | **SB three-position switch** | Low Gentle, center Normal, high Boost |
| CH7 | SD latching switch | High requests stop and deliberate rearm |
| CH8 | SC position switch | High reverse; low/center forward |

Channel numbers and mode-switch calibration remain configuration fields. The old S1 current dial is retained only in the factory-firmware legacy backend. Profile changes set the permitted current envelope; they do not directly change requested RPM.

Motor IDs viewed from above run clockwise: **1 front left, 2 front right, 3 rear right, 4 rear left**. Wheel side and polarity stay configurable. Verify each raised wheel's direction.

## Implemented behavior

The Pi and HAT use a checked binary v2 contract with boot/session/configuration identity, capabilities, accepted/applied command feedback, per-wheel feedback/temperature ages, requested/applied profiles, actual current ceilings, reason flags, Boost budget, and stop progress. A protocol mismatch refuses arming. An old STOP always disables motion without making old targets fresh. A reconnect discards prior motion permission.

Candidate profile caps are **0.8 A Gentle, 1.5 A Normal, 2.5 A Boost**. The supplied fast configuration permits all three in full, with an independent **2.7 A absolute ceiling**; there is no 1.2 A runtime cap. Boost budget and temperature/fault protection can still reduce the applied envelope. Configuration changes require stopped application acknowledgment; physical current/thermal acceptance is recorded separately. Current limits are experimental robot settings, not a manufacturer-certified operating envelope.

The HAT owns a **20-second Boost capacity / 60-second refill** budget. It starts empty after reset, survives Pi reconnects, consumes while Boost applies, and refills only outside Boost with fresh, cool, fault-free readings. It reports fallback/derating reasons. Initial temperature settings are warning 50 C, derating 55 C, stop 65 C, with explicit freshness, hysteresis, and cooldown settings.

Neutral settling and bounded encoder-assisted holding are separate from driving and stopping. A zero inner wheel in a turn does not park the whole robot. Holding uses its own limits and remains subordinate to faults and thermal/power protection. Powered holding is unavailable after motor-power removal.

CSV logging uses bounded background work, unique session files, full configuration metadata, event records, visible dropped rows, and storage-error reporting. Logging loss does not authorize movement or block the control loop. The live view explains requested/applied modes, wheel limits, feedback, protection, holding, stop confirmation, and Boost availability. Optional CRSF return telemetry requires the Pi TX wire to XR4 RX.

## Run and verify

```sh
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
cp config.example.json config.json
./.venv/bin/python robot_main.py check --config config.json
./.venv/bin/python robot_main.py monitor-radio --config config.json
./.venv/bin/python robot_main.py run --config config.json --telemetry logs/robot.csv
```

The monitor never opens the motor port. Run the drive only with raised wheels, restraints, and an independent power cutoff until commissioning acceptance passes.

```sh
python3 -m unittest discover -s tests -v
python3 tools/release.py
python3 tools/commissioning.py new commissioning/runs/trial-001.json
```

Pin Arduino-ESP32 to **3.3.12**, board **ESP32 Dev Module** (`esp32:esp32:esp32`). Use the generated firmware headers beside the existing sketch; copying only the `.ino` is insufficient. See [HAT_PROTOCOL.md](HAT_PROTOCOL.md) for the wire contract.

## Before ground driving

Build and verify [INDEPENDENT_STOP.md](INDEPENDENT_STOP.md), then follow [COMMISSIONING.md](COMMISSIONING.md). The assembled robot still needs measured stopping, holding, timing, thermal/electrical, and load/terrain qualification. The 15 ms sweep is a timing target; expanded telemetry and motor turnaround must be measured.

The original 44-test behavior is preserved in `releases/robot-v1-baseline.zip`. Release tooling exports this Git checkout as a source archive, manifest, and portable Git history. The separate `baseline/original-v1` branch preserves the original code for Claude; `main` is unchanged. [CLEANUP.md](CLEANUP.md) identifies removable generated files. The legacy drive is retained until fast-path physical acceptance; **keep `motor_setup.py` and `ddsm115.py`** for factory motor-ID work.
