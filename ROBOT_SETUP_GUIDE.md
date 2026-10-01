# Robot software, firmware, and controller setup

This guide covers the remaining setup for your **Raspberry Pi 4**, **Waveshare DDSM Driver HAT (A)**, **RadioMaster Pocket with built-in 2.4 GHz ExpressLRS**, and **XR4 receiver**. You are handling the wheel IDs and wiring, so those are prerequisites here. The drive program expects **four unique wheel IDs**: 1 front left, 2 front right, 3 rear right, 4 rear left.

Two different devices receive software:

| Device | What goes on it | How |
| --- | --- | --- |
| Raspberry Pi | Python drive program and config | Copy files over the network, then install one Python dependency |
| HAT's ESP32 | The custom **robot_hat.ino** firmware | Upload with Arduino IDE through the HAT's **ESP32-USB** port |
| Pocket and XR4 | Their existing EdgeTX/ExpressLRS software | Create a model, set channels, choose an ELRS mode, and bind |

The HAT firmware has compiled and the program's software tests have passed. It has **not** been tested on the assembled four-wheel robot yet. Keep all four wheels raised and a physical power cutoff within reach for the first powered checks.

## 1. Find the files on the Mac

In Finder, press **Command+Shift+G**, paste this path, then press Return:

~~~text
/Users/ambesnoff/.codex/.chatgpt-projects/g-p-6abd17e11e9c81919d539f1e77cd6853
~~~

**Command+Shift+.** toggles hidden files. The Pi files are in that folder. The HAT sketch is **hat_firmware/robot_hat/robot_hat.ino**.

You will need a microSD card and reader, the Mac and Pi on the same network, a USB-C **data** cable for the HAT, Arduino IDE 2, the Pocket's supplied antenna and 18650 cells, and both XR4 antennas.

## 2. Put Raspberry Pi OS on the microSD card

Skip this section if your Pi already boots Raspberry Pi OS and you can SSH into it.

1. Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) on the Mac. Use it to write **Raspberry Pi OS Lite (64-bit)** to the microSD card.
2. In Imager's customization screens, set **hostname: robotpi**, **username: robot**, and a password you keep. Enter Wi-Fi details if needed. Enable **SSH with password authentication**. The commands below use these example names; substitute your actual names if you already use others.
3. Put the card in the Pi and boot it. For Pi-only setup, you can power the Pi through USB-C while the HAT is removed. Later, when the HAT is seated, use the power arrangement you have verified for your robot.
4. Open **Terminal on the Mac**:

~~~sh
ssh robot@robotpi.local
~~~

Accept the first connection prompt and enter the Pi password. If the hostname does not resolve, look up the Pi's IP address in your router and use **ssh robot@IP_ADDRESS**. [Raspberry Pi's first-boot and SSH guide](https://www.raspberrypi.com/documentation/computers/getting-started.html).

## 3. Set up the Pi's two serial ports

The HAT communicates with the Pi through the header UART. The XR4's CRSF output uses a second Pi UART. On the **Pi's SSH terminal**, run:

~~~sh
sudo raspi-config
~~~

Select **Interface Options → Serial Port**. Answer **No** to a login shell on serial, then **Yes** to enabling serial hardware. Exit. Now run:

~~~sh
sudo systemctl disable hciuart
sudo nano /boot/firmware/config.txt
~~~

At the end of the file, under an existing **[all]** section or after adding one, add:

~~~text
dtoverlay=disable-bt
dtoverlay=uart5
~~~

Save with **Control+O**, Enter, **Control+X**. Do not add a CTS/RTS option to uart5. The **disable-bt** choice gives the HAT's header link the stable PL011 UART and disables Pi Bluetooth. Reboot, then reconnect:

~~~sh
sudo reboot
~~~

~~~sh
ssh robot@robotpi.local
readlink -f /dev/serial0
ls -l /dev/ttyAMA5
~~~

The first check should show **/dev/ttyAMA0**; the second should find **/dev/ttyAMA5**. If your OS has **/boot/config.txt** instead, edit that file. [Official Raspberry Pi UART instructions](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts).

## 4. Copy the drive program to the Pi

Use a **Mac Terminal** window for this block. It copies only the eight files the Pi needs:

~~~sh
ROBOT_PROJECT="$HOME/.codex/.chatgpt-projects/g-p-6abd17e11e9c81919d539f1e77cd6853"
ssh robot@robotpi.local 'mkdir -p ~/robot'
scp "$ROBOT_PROJECT"/{robot_main.py,crsf.py,ddsm115.py,fast_hat.py,fast_robot.py,motor_setup.py,requirements.txt,config.example.json} robot@robotpi.local:~/robot/
~~~

In the **Pi's SSH terminal**, install the dependency in a Python virtual environment:

~~~sh
sudo apt update
sudo apt install -y python3-venv
sudo usermod -aG dialout "$USER"
~~~

Log out with **exit** and SSH back in so serial-port permission takes effect. Then:

~~~sh
cd ~/robot
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
cp config.example.json config.json
./.venv/bin/python robot_main.py check --config config.json
~~~

The last line should report **fast** mode, wheel IDs **1–4**, motor port **/dev/serial0**, and radio port **/dev/ttyAMA5**. This is a config check only; it does not communicate with hardware. The virtual environment avoids current Raspberry Pi OS restrictions on system-wide pip installs. [Pi OS Python guidance](https://www.raspberrypi.com/documentation/computers/os.html).

## 5. Configure the Pocket controller

Fit the Pocket's top antenna before turning on its RF module. Install its two 18650 cells with the polarity shown by the radio. Create a new EdgeTX model named **Robot**. In **Model Setup**, choose **Internal RF = CRSF** and **External RF = Off**. The Pocket's internal ELRS is **2.4 GHz**; it can use the XR4 in standard 2.4 GHz mode. The XR4's dual-band Gem-X mode needs a compatible external transmitter module and is not required here. [Pocket manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/Pocket_A1.8.pdf?v=1770617495), [ExpressLRS transmitter setup](https://www.expresslrs.org/quick-start/transmitters/tx-prep/).

On the EdgeTX **Mixes** page, set one direct mix for each channel. Start with **100% weight, zero offset, no added curve, and no delay**. Remove any default mix that would conflict. This is the mapping in **config.json**:

| Channel | Pocket control example | Meaning to robot |
| --- | --- | --- |
| CH1 | Right stick horizontal / Aileron | Steering, centered at 0 |
| CH3 | Throttle stick | Fully low is stop (−100%); up requests speed |
| CH5 | SA latching switch | Low = disarmed; low then high = arm |
| CH6 | SB switch | Low or center = forward; high = reverse |
| CH7 | SD latching switch | Low = normal; high = stop |
| CH10 | S1 dial | Lowest = zero/little drive current; highest = 1 A cap |

Use EdgeTX's **Channel Monitor** screen to check that only the intended channel changes with each control. Reverse a mix in EdgeTX if its high/low direction is wrong. In particular, **CH3 must be low at rest**, **CH5 must be low before arming**, and **CH7 high must be stop**. The [EdgeTX Mixes guide](https://manual.edgetx.org/bw-radios/model-select/inputs-mixes-and-outputs/mixes) shows how to choose a channel's source.

For the faster radio setting, first open **SYS → Hardware → Baudrate**, set the Pocket's **internal ELRS serial baud to 921k**, and restart the Pocket. Then, in **SYS → ExpressLRS**, choose **333 Hz Full / 12ch Mixed** if both radio firmware versions offer it. That gives CH1/CH3 the full packet rate and CH6/CH7/CH10 half rate. If unavailable, use **250 Hz / Wide**; 400k serial baud is sufficient for that rate. Wide gives the S1 dial useful proportional resolution; Hybrid limits CH10 to a few steps. Keep CH5 as the arm channel. Change ELRS switch mode while the XR4 is off, then check the channels again. [ExpressLRS baud-rate recommendations](https://www.expresslrs.org/quick-start/transmitters/tx-prep/#serial-baud-rate), [ExpressLRS switch modes](https://www.expresslrs.org/software/switch-config/).

## 6. Attach and place the antennas; bind the XR4

The XR4 uses **both** supplied T antennas. Align each tiny IPEX-1/U.FL plug with a receiver socket and press straight down until it clicks. Do not pull on the thin cable. Secure the coax so vibration cannot pull the plugs loose. Place the active T ends in open air away from metal, the battery, and motor wiring. Separating them and using different orientations, roughly 90° if practical, is a useful diversity layout; RadioMaster specifies no exact spacing or angle. Keep the Pocket's own antenna attached whenever its RF module is on. [XR4 manual](https://cdn.shopify.com/s/files/1/0609/8324/7079/files/XR4.pdf?v=1739432399), [Pocket manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/Pocket_A1.8.pdf?v=1770617495), [U.FL connector handling](https://www.hirose.com/product/download/?distributor=all&lang=en&series=U.FL&type=catalogue).

You do **not** need an independent XR4 power switch. The XR4 has a small **BIND** button on its top face, near the left antenna socket. With the robot safely raised, the arm switch low, and the stop switch high, power the XR4 through the robot's normal power arrangement. Do **not** rapidly power-cycle the entire Pi/robot to bind. Regulated **5 V** is within both of RadioMaster's conflicting published XR4 voltage ranges. [XR4 product page](https://www.radiomasterrc.com/products/xr4-gemini-xrossband-dual-band-expresslrs-receiver), [XR4 manual and button diagram](https://cdn.shopify.com/s/files/1/0609/8324/7079/files/XR4.pdf?v=1739432399).

1. Turn the Pocket **off**. With the XR4 already powered, press and hold its **BIND** button for about **1.5 seconds**; release when its LED shows a quick double-blink. Do not hold the button while first applying power. ExpressLRS 3.4 and newer support this button method; the XR4 manual lists 3.5.1 as preinstalled.
2. Turn on the Pocket. Open **SYS → ExpressLRS → Bind**. A solid XR4 LED means it is bound. [Official ExpressLRS binding guide](https://www.expresslrs.org/quick-start/binding/).
3. If binding fails, check that transmitter and receiver have the same **ExpressLRS major firmware version**, compatible RF region, and Model Match setting. Traditional binding requires **no binding phrase** on the receiver. Check the actual firmware version on your unit before updating anything.

The XR4 retains its normal ExpressLRS firmware. It does not get the robot firmware. Its CRSF serial rate is **420000 baud**, as set in the example Pi config.

## 7. Check the controller in the Pi program

Once you have connected the XR4 to the Pi's UART5 and powered the Pi using your finished hardware arrangement, run:

~~~sh
ssh robot@robotpi.local
cd ~/robot
./.venv/bin/python robot_main.py check --config config.json
./.venv/bin/python robot_main.py monitor-radio --config config.json
~~~

**monitor-radio** reads the receiver without opening the motor port. It should print **healthy=True** and positive **LQ**. Hold each switch long enough to see it in the periodic display. Verify:

- Throttle about **−1** fully low, rising toward **+1** as you raise it.
- Steering about **0** centered, negative left, positive right.
- CH5 arm and CH7 stop each read about **−1** low and **+1** high.
- CH6 reverse changes low/high; S1 changes the current dial value.

Press **Control+C** to exit. If the XR4 LED is solid but there are no channels, check the receiver's CRSF output, UART5 device, and **radio_baud: 420000** in config. If channels appear but **healthy=False**, the program also needs fresh positive link-quality statistics; do not bypass that condition.

## 8. Upload the HAT firmware

Do this **after you have assigned the four motor IDs**, since the ID utility uses the original Waveshare HAT firmware. You are handling that step yourself. Shut down the Pi cleanly, turn robot power off, disconnect motor power, and remove the HAT from the Pi for USB upload.

1. Install [Arduino IDE 2](https://www.arduino.cc/en/software) on the Mac.
2. In **Preferences → Additional Boards Manager URLs**, add **https://espressif.github.io/arduino-esp32/package_esp32_index.json**. In **Boards Manager**, install **esp32 by Espressif Systems**, version **3.3.12**, and select **ESP32 Dev Module**. [Espressif Arduino setup](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
3. In Arduino IDE, open **hat_firmware/robot_hat/robot_hat.ino** from the project folder. Connect the HAT's **ESP32-USB** port, not the **DDSM-USB** motor-bus port, to the Mac with a data cable. Select the new serial port. Set the HAT's control switch to **ESP32**.
4. Click **Verify**. If that succeeds, click **Upload** and wait for **Done uploading**. Disconnect the HAT USB cable, reseat the HAT on the Pi, and restore your verified robot power arrangement. [Arduino upload guide](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started-ide-v2/).

The upload replaces Waveshare's HAT firmware, but the wheel IDs are stored in the motors. If you need the original HAT interface back, Waveshare provides a [factory restore package](https://github.com/waveshareteam/ddsm_example#quick-factory-reset) that flashes through ESP32-USB; its included flashing program is a **Windows executable**.

## 9. Test the four-wheel program

Have all **four** correctly identified motors connected and powered before booting the custom HAT firmware. It checks all four at startup and latches a fault if one is absent. Keep the chassis secured with all wheels raised. Start the Pocket with throttle fully low, S1 low, arm low, and stop high. Then power the robot.

From the Pi:

~~~sh
ssh robot@robotpi.local
cd ~/robot
./.venv/bin/python robot_main.py run --config config.json --telemetry robot_telemetry.csv
~~~

The Pi and HAT first require fresh feedback that all four wheels are stationary. Then:

1. Move CH7 stop **low**. Keep throttle fully low. Move CH5 arm **low**, then **high**. Look for **Armed four-wheel drive**.
2. Raise S1 a small amount and add a little throttle. S1 at minimum can correctly prevent any motion. Check all wheel directions while raised.
3. At low throttle, test steering and CH6 reverse. If a wheel's forward direction is wrong, stop the program and change that wheel's **polarity** in **~/robot/config.json**, then restart and test again.
4. Test CH7 stop and CH5 disarm. Each should require a fresh low-to-high arm cycle at zero throttle. With wheels still raised, test radio loss by turning the Pocket off while moving slowly; it should stop and remain disarmed on reconnection.
5. Press **Control+C** to stop the program. It requests and checks a HAT stop. If a wheel does not stop, use the physical cutoff. After testing, run **sudo shutdown -h now**, wait for the Pi to shut down, then remove robot power.

The example starts at **40 RPM maximum**, **1.0 A drive-current cap**, and a **15 ms requested** Pi/HAT update period. The HAT uses acceleration/deceleration ramps and wheel feedback. These are **starting settings**, not measured four-wheel performance or established ground-driving limits. Review **robot_telemetry.csv**, console rate/deadline messages, wheel temperature, and stop behavior before lowering the robot. The program does not monitor battery charge.

## 10. Make it start when the robot powers on (after testing)

Do this only after the raised-wheel test passes. Stop a manually running copy first. Every powered boot must have all four identified motors connected and powered before the custom HAT checks them. On the Pi:

~~~sh
cd ~/robot
cat > robot-drive.service <<EOF
[Unit]
Description=Four-wheel robot drive
After=local-fs.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PWD
ExecStart=$PWD/.venv/bin/python $PWD/robot_main.py run --config $PWD/config.json --telemetry $PWD/robot_telemetry.csv
Restart=no

[Install]
WantedBy=multi-user.target
EOF
sudo install -m 644 robot-drive.service /etc/systemd/system/robot-drive.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-drive.service
sudo journalctl -u robot-drive.service -f
~~~

**Control+C** exits the log viewer but leaves the service running. To stop/start the program later, use **sudo systemctl stop robot-drive.service** or **sudo systemctl start robot-drive.service**. Radio arming is still required after each boot. Save **robot_telemetry.csv** before restarting the program if you want to keep it; a new run replaces that file. You do not need Wi-Fi for the Pocket to drive once the service is running; Wi-Fi/SSH is for setup and logs.

## Quick troubleshooting

| Problem | First check |
| --- | --- |
| Mac cannot SSH | Pi on same network, Imager SSH enabled, correct username/hostname/IP. |
| Pi serial devices wrong | Serial console disabled, hardware serial enabled, overlay lines, reboot. |
| Serial permission denied | Run **groups** after logging out/in; it should include **dialout**. |
| Pocket and XR4 will not bind | Both antennas, 5 V receiver power, matching ELRS major versions, RF region, binding phrase, Model Match. |
| Bound XR4 but no Pi channels | UART5, CRSF output, **/dev/ttyAMA5**, 420000 baud, receiver power. |
| Channels present but unhealthy | Fresh positive LQ and link-statistics frames. |
| HAT upload fails | ESP32-USB port, data cable, selected serial port, correct board/core; see Waveshare's BOOT-button procedure. |
| HAT fault or cannot arm | All four unique motor IDs, four live stationary motors, HAT power and communication; fix the cause then fully power-cycle the HAT. |
| Unexpected movement or failed stop | Use the physical power cutoff, then inspect logs and hardware before trying again. |

## Official references

[Raspberry Pi setup](https://www.raspberrypi.com/documentation/computers/getting-started.html) · [Pi UARTs](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts) · [Waveshare HAT](https://www.waveshare.com/wiki/DDSM_Driver_HAT_%28A%29) · [Pocket manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/Pocket_A1.8.pdf?v=1770617495) · [XR4 manual](https://cdn.shopify.com/s/files/1/0609/8324/7079/files/XR4.pdf?v=1739432399) · [ExpressLRS binding](https://www.expresslrs.org/quick-start/binding/)
