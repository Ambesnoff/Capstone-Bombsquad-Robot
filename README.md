# Four-wheel DDSM115 robot drive

For the Mac-to-Pi install, HAT upload, Pocket/XR4 setup, and first test, start
with [ROBOT_SETUP_GUIDE.md](ROBOT_SETUP_GUIDE.md).

The Raspberry Pi 4 reads the XR4 receiver directly and mixes throttle and
steering into four wheel-speed requests. In the **fast** configuration, the Pi
sends those requests to new firmware on the Waveshare DDSM Driver HAT (A)'s
ESP32. The ESP32 closes a separate speed/current feedback loop for each wheel,
reports all four motors to the Pi, and stops on a lost command or motor fault.
The HAT stays seated on the Pi's 40-pin header. The radio uses another Pi UART.

The ESP32 sketch compiles with Arduino-ESP32 core 3.3.12, and the software
tests pass. The actual update rate, wheel direction, stopping distance,
current, and fault behavior still need measurement on the
assembled four-wheel robot. Start with every wheel raised and a physical power
cutoff available.

## Controls and wheel layout

| Control | Example channel | Behavior |
| --- | ---: | --- |
| Throttle | CH3 | -100% is stopped; +100% requests the configured maximum RPM |
| Steering | CH1 | Skid steering while moving; no pivot at zero throttle |
| S1 dial | CH10 | Caps driving current from zero to `max_current_a` |
| Arm switch | CH5 | Move low, then high with throttle at zero to arm |
| Reverse switch | CH6 | High selects reverse |
| Stop switch | CH7 | High stops and requires a new low-to-high arm cycle |

Viewed from above, motor IDs run clockwise: **1 front left, 2 front right,
3 rear right, 4 rear left**. Fast firmware requires all four IDs. The example
expects negative motor commands for the left wheels and positive for the right
wheels when driving forward. Verify each raised wheel before ground testing;
change its `polarity` in `config.json` if necessary.

The channel numbers are starting values. Map CH5/6/7 in the Pocket's EdgeTX
Robot model and adjust `config.json` if you choose other channels. Confirm the
actual travel of CH3, CH1, and CH10 in the radio monitor. Choose an ELRS switch
mode that gives CH10 enough steps to act as a proportional dial.

## Set up in this order

1. **Assign the wheel IDs with the HAT's original firmware still installed.**
   First copy the Python files, `requirements.txt`, and example config from
   this project into one directory on the Pi. Install Python 3.10+ and
   `python3-venv`, then run `python3 -m venv .venv`,
   `source .venv/bin/activate`, and
   `python3 -m pip install -r requirements.txt` there. Keep the virtual
   environment active for the `python3` commands below. Power down before
   changing motor connections. Connect exactly one motor at
   a time, keep the wheel raised, and run `python3 motor_setup.py assign 1` for
   the front-left wheel. Repeat with `2` front right, `3` rear right, and `4`
   rear left. The tool asks you to confirm the one-motor connection and verifies
   the result. `python3 motor_setup.py verify 1` checks a previously assigned
   wheel. Put `--port /your/port` before `assign` or `verify` if the Pi HAT
   port is not `/dev/serial0`. Power down again before connecting all four.
   `motor_setup.py` speaks the factory JSON protocol, so it cannot assign IDs
   after the custom firmware is flashed.
2. **Flash the HAT's ESP32.** After assigning all four IDs, shut down the Pi,
   remove the HAT from its header, and disconnect motor power for upload. In
   Arduino IDE, install **esp32 by Espressif Systems** in Boards Manager
   (verified with core **3.3.12**), select **ESP32 Dev Module** for the HAT's
   ESP-WROOM-32 module, open
   `hat_firmware/robot_hat/robot_hat.ino`, and use **Verify** before **Upload**.
   Connect the HAT's **ESP32-USB** port to the computer with a data-capable USB
   cable and select the newly appearing serial port. Upload through that port,
   then disconnect USB, reseat the HAT, and restore power. Follow the
   [Waveshare HAT (A) instructions](https://www.waveshare.com/wiki/DDSM_Driver_HAT_%28A%29)
   for its connector and switch positions; put the motor-control switch at
   **ESP32** for robot operation. The upload replaces Waveshare's factory
   firmware. [Waveshare's factory restore package](https://github.com/waveshareteam/ddsm_example#quick-factory-reset)
   uses the same ESP32-USB port if you need to return to the original JSON
   interface. Its packaged restore tool is a Windows executable.
3. **Prepare the Pi serial ports.** Copy `config.example.json` to
   `config.json` in the Pi directory. The fast example uses `/dev/serial0`
   for Pi-to-HAT UART at
   **230400 baud**. The ESP32-to-motor RS-485 bus remains at the DDSM115's
   documented **115200 baud**. These are two different serial links; changing
   `hat_baud` alone cannot change the motor bus. The ESP32 sketch and Pi config
   must agree on the Pi-to-HAT rate. Disable the Linux login console on the
   header UART while leaving that UART enabled. Check where `/dev/serial0`
   points with `readlink -f /dev/serial0`: if it is the Pi 4's mini UART
   (`ttyS0`), its baud rate needs a fixed core clock (`enable_uart=1`). For
   the more stable PL011 UART on the header, the [Raspberry Pi UART
   instructions](https://www.raspberrypi.com/documentation/computers/configuration.html)
   describe `dtoverlay=disable-bt` and disabling `hciuart` when Bluetooth is
   not needed.
4. **Wire the XR4 to a separate Pi UART.** On a Pi 4, add `dtoverlay=uart5` to
   `/boot/firmware/config.txt` (or `/boot/config.txt` on older Pi OS) and reboot.
   Leave UART5 CTS/RTS disabled because those would use GPIO14/15, the HAT's
   UART0 pins. UART5 TX is GPIO12, physical pin 32; UART5 RX is GPIO13,
   physical pin 33. Reach the passthrough pins with a stackable header or
   breakout. Connect the XR4's **main CRSF TX** to Pi pin 33/GPIO13 RX and
   connect grounds. Supply the XR4 with regulated 5 V. XR4 RX to Pi pin
   32/GPIO12 TX is optional because this program only reads the receiver. The
   Pi UART input is 3.3 V logic: measure the XR4 TX idle voltage and use a
   suitable high-speed level shifter if it exceeds 3.3 V. Ensure the 5 V
   supply can power the receiver. The example expects `/dev/ttyAMA5`; use
   `python3 robot_main.py ports` and adjust `radio_port` if your device differs.
5. **Check controls before opening the motor port.** Run
   `python3 robot_main.py check --config config.json`, then
   `python3 robot_main.py monitor-radio --config config.json`. Move one control
   at a time. Confirm mapped channels, throttle neutral/full travel, dial
   travel, fresh frames, and positive link quality. `monitor-radio` never
   opens the motor port. A receiver with no CRSF link-statistics frames stays
   disarmed; configure the receiver rather than bypassing that check.
6. **Test with all four wheels raised.** Run
   `python3 robot_main.py run --config config.json --telemetry robot_telemetry.csv`.
   The Pi and HAT first require fresh feedback confirming all four wheels are
   stopped. Set CH5 low and throttle to zero, then move CH5 high. Begin with a
   small dial setting and throttle. Check wheel directions, steering, and
   reverse. Test CH7, CH5 disarm, radio power-off, stale receiver frames, and
   reconnection. After a stop, motion must remain disabled until another valid
   arm cycle. Inspect the CSV and measure behavior before lowering the robot.

## Response and feedback

The fast example requests a **15 ms period (about 67 updates/s)** for Pi target
commands and HAT control sweeps. This is a test target, not a measured
four-wheel rate. Each DDSM115 request and reply uses the same 115200-baud
RS-485 bus. Four sequential 10-byte requests and replies need at least
**6.94 ms** of wire time; an occasional temperature query raises that to
**8.68 ms**, before motor turnaround and processing. The separate 230400-baud
Pi link takes about **0.83 ms** to transmit a target and **1.95 ms** to transmit
a status report. The `loop_period_s` field applies only to the factory-firmware
backend; `fast_period_ms` sets the fast Pi and HAT periods. Waveshare's
["up to 500 Hz" single-motor
specification](https://www.waveshare.com/wiki/DDSM115) does not establish a
four-wheel rate. Watch the actual report rate, sweep duration, command timing,
feedback age, and missed Pi deadlines during raised-wheel testing. The HAT's
reported sweep time ends before the status frame is sent, so use Pi arrival
timing as well when judging whether 15 ms is sustainable.

The receiver reader publishes each complete frame without waiting for a
64-byte buffer to fill. CSV writing runs in a bounded background worker so
ordinary disk writes do not hold up the Pi control pass; a full queue or writer
failure stops the drive. The CSV includes the interval between HAT reports and
the actual Pi control-pass interval. Filter for armed rows when evaluating
those intervals, because disarmed status reports run more slowly. Requested
RPM in a row is the latest Pi command; the HAT feedback beside it may describe
an earlier command, as indicated by the acknowledgment sequence.

The dial limits commanded drive current. At neutral, the controller may use up
to `neutral_braking_current_a` while a wheel is still turning to keep its
deceleration smooth; it commands zero current once the wheel is stationary.
The HAT also enforces time-based acceleration and deceleration ramps, current
limits, a command watchdog, motor feedback and fault checks, temperature, stall,
overspeed, and abnormal-current checks. A fault requests a stop on all four
motors and stays latched until the HAT is power cycled after inspection.
The example's 120 RPM/s acceleration and 180 RPM/s deceleration change a
wheel's requested speed by about **1.8 RPM** and **2.7 RPM** per 15 ms sweep.
For a late sweep, the ramp uses elapsed time up to 100 ms; longer gaps are
capped to avoid a sudden speed jump. At the example's 40 RPM maximum, a full
0-to-40 RPM request
takes about **333 ms**; a 40-to-0 RPM request takes about **222 ms**. Actual
wheel speed may lag, depending on load and current limit.
During normal driving, CH7 and disarm request zero current on all wheels,
then verified zero-speed mode. This is a powered software stop, not a
physical emergency stop or proof that the chassis is stationary.

The example starts at 40 RPM and a 1.0 A maximum drive current for raised-wheel
testing. These are tuning values, not validated ground-driving limits. Increase
only from measured wheel response, current, and temperature. The program caps
the configured maximum at 1.2 A; Waveshare's
[product specification](https://www.waveshare.com/product/modules/motors-servos/ddsm115.htm)
lists 1.25 ± 0.05 A rated current. Its quoted 30 ms mechanical time constant
is a motor specification, not the robot's measured response.

## Original firmware option

`config.legacy.example.json` is for Waveshare's original ESP32 JSON firmware.
Copy it to `config.json` only when that factory firmware is installed. It uses
the older Pi-side 10 Hz control loop and the HAT's 115200-baud JSON link. The
fast `config.example.json` requires the custom ESP32 sketch; neither config
works with the other HAT firmware. The XR4 remains connected directly to the
Pi in both modes.

Run software checks with `python3 -m unittest discover -s tests -v`. Stop the
drive program with Ctrl+C or SIGTERM and keep the physical battery cutoff
accessible. The software does not monitor battery state or replace that cutoff.
