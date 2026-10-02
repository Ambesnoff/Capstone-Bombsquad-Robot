# Commission the implemented modes

The software implementation precedes these measurements. A passing simulator and a successful ESP32 compile do not qualify current limits, stopping distance, thermal margin, or slope holding on this robot.

## Create a record before every trial

```sh
python3 tools/commissioning.py new commissioning/runs/trial-001.json
```

Fill in the actual four-wheel hardware, firmware build ID, accepted configuration ID, release hash, load, terrain, and numerical stopping/drift/thermal acceptance targets before testing. Preserve the session CSV, configuration metadata, and event log beside the record. Change one setting between trials. `python3 tools/commissioning.py check commissioning/runs/trial-001.json` exits unsuccessfully until every required measurement and acceptance check passes. Keep separate acceptance records for different loads/terrain/configurations.

## Test progression

1. Verify the independent motor-power stop circuit in `INDEPENDENT_STOP.md`. Assign IDs with factory firmware and `motor_setup.py` before installing the custom sketch. Verify all motor directions with wheels raised.
2. Verify actual received radio channels. SB on the configured profile channel (example CH6) must produce three distinct stable values; SA/CH5 controls arm, SD/CH7 stop, SC/CH8 reverse. Test invalid switch values, held throttle, stale link statistics, link loss, and reconnection without enabling motion.
3. With raised wheels, start in Gentle with low speed; the supplied fast configuration permits full 0.8/1.5/2.5 A profiles. Record each requested/applied mode, per-wheel effective cap, current, actual/target RPM, temperature and separate feedback ages. Exercise neutral settling/holding, individual zero wheels during turns, forward/reverse ramps, mode changes, explicit disarm, stop, and deliberate rearm.
4. Measure complete four-wheel sweep timing, status arrivals, accepted/applied sequence latency, and temperature ages. Expanded v2 status takes more serial bandwidth than v1. If deadlines or temperature freshness cannot be sustained, lower the control update rate or keep Boost unavailable; measure again.
5. Exercise Boost exhaustion, refill outside Boost, repeated switching, Pi restart/reconnection, HAT restart with zero budget, warm-motor refill suspension, stale temperature rejection, persistent stall, motor error, bad motor reply/CRC, and missing feedback. Verify clear reason codes and independent STOP REQUESTED / CONFIRMED / UNCONFIRMED behavior. Do fault injection in a restrained setup; software tests already cover the logic.
6. Progress to restrained level-ground trials only after raised-wheel checks and the physical cutoff pass. Measure stopping time/distance and holding drift. Then use restrained ramp trials to measure encoder wrap, low-speed drift, current/temperature, and loss of holding on protection/power removal.
7. Normal and Boost require a separate electrical/thermal acceptance record before claiming qualification. The 0.8/1.5/2.5 A profiles and 20 s / 60 s allowance are experimental candidates. Do not increase limits just to silence warnings. Measure battery/BMS/wiring/regulator current separately from motor torque current.
8. Qualify repeated Boost cycles, sustained intended load, turning on intended surfaces, power faults, and loaded slopes. Record unresolved conditions and constrain deployment to measured combinations. Retire the legacy drive only after fast-path hardware acceptance; retain factory motor-ID setup tools.

## Initial targets are explicit, not measured facts

The trial template deliberately leaves numerical stopping and drift targets unset. Set targets appropriate to the restraint, geometry, load, and application before testing. Do not backfill a target to match an observed failure. A completed record applies only to the identified hardware, firmware/configuration, terrain/load, and envelope.

## Returning to service

After communication/radio/thermal recovery, readiness can return but motion permission cannot. Require neutral, arm low, then a new arm edge. Inspection faults require identifying and resolving the original cause. If stop remains unconfirmed, use the physical cutoff and inspect the missing feedback instead of issuing drive commands. Service retries use a bounded delay sequence and stop after five process starts; their original failures remain in the journal.
