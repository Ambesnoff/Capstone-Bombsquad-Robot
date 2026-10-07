# Update ramps, braking and pivot

This is a Pi-only update. There is no HAT upload.

## What changed and why

- **Ramps.** The old ramps were tuned when `max_rpm` was 40. At 250 rpm they took about 2.1 s to reach full speed and 1.4 s to stop. The new values (`acceleration_rpm_s` 500, `deceleration_rpm_s` 1000) give about 0.5 s and 0.25 s.
- **Braking.** When a wheel's target is zero, the HAT caps braking current at `neutral_braking_current_a`. At 0.3 A a faster ramp could not stop the robot on the ground, so the example now uses 1.0 A.
- **Pivot.** A new optional Pi-only key, `pivot_gain` (0 to 1, default 0 = off; the example uses 0.5), lets the robot turn in place. The turn mix is:

  `turn = steering * (throttle * steering_gain + (1 - throttle) * pivot_gain)`

  At full throttle this is the old steering. At zero throttle the two sides counter-rotate at up to `pivot_gain × max_rpm`.
- **Steering convention is unchanged.** Stick right turns the robot to its own right (clockwise from above) going forward, in reverse and when pivoting. The left side runs faster than the right.
- **Gains are unchanged.** The drive gains stay at the low gain-test values and will be tuned through config.

## No HAT upload

The HAT firmware and its build ID (`0xc050c91b`) are unchanged. Do not reflash.

## 1. Copy the Pi files first

Copy before creating any config. An older `robot_config.py` rejects `pivot_gain` as an unknown key. In a **Mac Terminal**:

```bash
scp "$HOME/Desktop/Capstone/Robot Code/Capstone-Bombsquad-Robot"/*.py robot@robotpi.local:~/robot/
```

## 2. Check and fix direction first

Do this with the wheels raised, before anything else.

- Switch low plus a little throttle: the tops of the tyres should roll toward the front (motors 1 and 2).
- Stick right: the left side should be faster.

If both are backwards, flip all four wheel polarities. In the **Pi SSH terminal**, this writes `config.speed-250-fixed.json` and refuses to overwrite an existing file:

```bash
cd ~/robot && ./.venv/bin/python - <<'PY'
import json
from pathlib import Path
from robot_config import Settings
data = json.loads(Path("config.speed-250.json").read_text())
for w in data["wheels"]:
    w["polarity"] = -w["polarity"]
Settings.from_dict(data)
with open("config.speed-250-fixed.json", "x") as f:
    f.write(json.dumps(data, indent=2) + "\n")
print("polarities now:", [(w["id"], w["polarity"]) for w in data["wheels"]])
PY
```

## 3. Create the fast config

In the **Pi SSH terminal**. This builds `config.speed-250-fast.json` from `config.speed-250-fixed.json` if it exists, otherwise from `config.speed-250.json`. It sets the four values above, leaves every gain untouched, and refuses to overwrite:

```bash
cd ~/robot && ./.venv/bin/python - <<'PY'
import json
from pathlib import Path
from robot_config import Settings
source = Path("config.speed-250-fixed.json")
if not source.exists():
    source = Path("config.speed-250.json")
data = json.loads(source.read_text())
data["acceleration_rpm_s"] = 500
data["deceleration_rpm_s"] = 1000
data["neutral_braking_current_a"] = 1.0
data["pivot_gain"] = 0.5
Settings.from_dict(data)
with open("config.speed-250-fast.json", "x") as f:
    f.write(json.dumps(data, indent=2) + "\n")
print("source:", source)
print("acceleration_rpm_s:", data["acceleration_rpm_s"])
print("deceleration_rpm_s:", data["deceleration_rpm_s"])
print("neutral_braking_current_a:", data["neutral_braking_current_a"])
print("pivot_gain:", data["pivot_gain"])
PY
```

Check it (no hardware connection):

```bash
cd ~/robot && ./.venv/bin/python robot_main.py check --config config.speed-250-fast.json
```

Run it:

```bash
cd ~/robot && ./.venv/bin/python robot_main.py run --config config.speed-250-fast.json --telemetry logs/speed-250-fast.csv
```

## 4. Test order and cautions

1. **Wheels raised first.** Pivot at zero throttle with stick right: left wheels forward, right wheels backward. Releasing the stick should stop the wheels within about 0.3 s.
2. Then slow ground runs.
3. With the current low gain-test gains (kp 0.005, ki 0), a pivot asks for at most about kp × pivot target current at standstill (0.005 × 125 rpm ≈ 0.6 A). On grippy ground it may not turn until the gains are raised through config.
4. Faster ramps make direction changes abrupt. Bring the throttle to zero before moving the reverse switch.
5. Watch for stall fault 7.
