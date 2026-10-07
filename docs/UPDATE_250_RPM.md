# Upload the shared 250 rpm update

This update needs **both** the Pi program and the HAT's ESP32 firmware. Gentle, Normal and Boost share the same 250 rpm maximum target; their current ceilings remain 0.8, 1.5 and 2.5 A. Controller gains and wheel-direction settings are a separate concern. This update does not complete torque tuning.

The code is on the `codex/shared-250-rpm` pull-request branch. It has not been merged into `main`. The Mac checkout used for this change is:

`/Users/ambesnoff/Desktop/Capstone/Robot Code/Capstone-Bombsquad-Robot`

## What you need

- Your Mac and Pi on the same network, with working SSH to `robot@robotpi.local`.
- The Pi's existing `~/robot` installation and `.venv`; this change adds no Python package dependency.
- The reduced-gain file used in the successful trial, `~/robot/config.gain-test.json`.
- A USB-C **data** cable connected to the HAT's **ESP32-USB** port.
- Arduino IDE 2, **esp32 by Espressif Systems 3.3.12**, and board **ESP32 Dev Module** (`esp32:esp32:esp32`).
- The entire `hat_firmware/robot_hat` folder, including `robot_hat.ino` and every `.h` file.

## 1. Get the PR code on the Mac

The working Mac checkout is already updated. To retrieve the published branch again, use a Mac Terminal:

```bash
cd "$HOME/Desktop/Capstone/Robot Code/Capstone-Bombsquad-Robot" &&
git fetch origin &&
git switch codex/shared-250-rpm &&
git pull --ff-only origin codex/shared-250-rpm
```

Use this branch while the PR is a draft. Pulling `main` alone will not retrieve the new speed-limit code.

## 2. Stop the controller and upload the HAT firmware

Stop the running Pi control program with **Control+C**. Follow the existing HAT upload procedure in [the setup guide](../ROBOT_SETUP_GUIDE.md#8-upload-the-hat-firmware): shut down the Pi cleanly, disconnect robot/motor power, and remove the HAT for USB upload.

1. In Arduino IDE, open `hat_firmware/robot_hat/robot_hat.ino` from the updated Git checkout. Keep all headers alongside it; copying only the `.ino` will use incomplete or stale firmware.
2. Connect the HAT's **ESP32-USB** port to the Mac using the data cable. Select its serial port and **ESP32 Dev Module** with core **3.3.12**. Set the HAT's control switch to **ESP32**.
3. Click **Verify**, then **Upload**. Wait for **Done uploading**.
4. Disconnect USB, reseat the HAT, restore the robot's verified power arrangement, and reconnect to the Pi.

The new HAT build identity is **`0xc050c91b`**. The old firmware rejects a 250 rpm configuration because its maximum was 200 rpm. Wheel IDs are stored in the motors and do not need reassignment for this firmware update.

## 3. Copy the Pi runtime files

In a **Mac Terminal**:

```bash
ROBOT_PROJECT="$HOME/Desktop/Capstone/Robot Code/Capstone-Bombsquad-Robot"
scp "$ROBOT_PROJECT"/*.py "$ROBOT_PROJECT"/robot_protocol.json robot@robotpi.local:~/robot/
```

This copies all root Python modules, including the updated `robot_config.py`, `fast_hat.py`, and `protocol_defs.py`, plus the protocol schema. It leaves your existing Pi configuration files intact. Tests and documentation are in the Git repository; the copy above is the runtime update.

## 4. Create the 250 rpm configuration

In the **Pi SSH terminal**:

```bash
cd ~/robot && ./.venv/bin/python - <<'PY'
import json
from pathlib import Path
from robot_config import Settings

source = Path("config.gain-test.json")
destination = Path("config.speed-250.json")
data = json.loads(source.read_text())
data["max_rpm"] = 250
Settings.from_dict(data)
with destination.open("x") as file:
    file.write(json.dumps(data, indent=2) + "\n")
print(f"Created {destination}: shared maximum 250 rpm.")
print(f"Drive gains: Kp={data['kp_a_per_rpm']}, Ki={data['ki_a_per_rpm_s']}")
PY
```

This changes only the speed limit, retains the reduced gains and other personal settings, and preserves the original file. If `config.speed-250.json` already exists, inspect it before reusing or replacing it. Use the actual successful gain-test file as input, including any later personal configuration edits.

## 5. Check and run when ready

The following configuration check does not open a hardware connection:

```bash
cd ~/robot && ./.venv/bin/python robot_main.py check --config config.speed-250.json
```

For your next controlled trial, keep the wheels raised and begin in Gentle with small throttle. The new maximum is a ceiling, not a request to start at full speed. End the trial if jitter or a fault returns.

```bash
cd ~/robot && ./.venv/bin/python robot_main.py run --config config.speed-250.json --telemetry logs/speed-250.csv
```

The existing overspeed formula follows the configured maximum: at 250 rpm it trips above **312 rpm**. Therefore absence of an overspeed fault alone does not establish that jitter is gone. The 40 rpm gain-test result has not validated this higher-speed configuration. Low-speed torque still needs separate tuning.

Copy all trial logs back from a **Mac Terminal**:

```bash
scp 'robot@robotpi.local:~/robot/logs/*' "$HOME/Desktop/robot-jitter-logs/"
```

## Verification record

Requirement tests were written before the corresponding implementation changes. Source limits, signed 16-bit representation, generated definitions, and changed Python/JSON syntax were inspected. The firmware identity was regenerated. No tests, firmware compilation, flashing, serial connections or hardware commands were run by Codex; no passing test suite or physical acceptance is claimed.
