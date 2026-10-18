# v7.6_UM982_X20P_F9P_TM171_WEB_XIAO

Teensy 4.1 firmware for the **AiO v4.5 boards** (Micro and Standard), for AgOpenWeb and AgOpenGPS.

- GPS: **UM982** (dual antenna) or **u-blox F9P / X20P** (single antenna).
- IMU: **TM171** or **BNO085**.
- Steering: Cytron (hydraulic valve) or Keya motor.
- Optional: a **Seeed XIAO ESP32C3 or ESP32S3** on the GPS2 port, with its own WiFi page.
- A web page on the module: status, messages, settings, the GPS receiver, firmware update over the network.

Both boards wire the Teensy the same way; the differences are only in the sockets (see "AiO v4.5 Micro and Standard" below).

**The standard sentence set needs an AgOpenWeb build that includes PR #300** (the epoch assembler, `develop` from the evening of 9 Oct 2026 onwards). On an older build these sentences show up as "not accepted". The classic output works with any AgOpenWeb and with AgOpenGPS.

## Versions

### 7.6 (18 Oct 2026)

- The web page is now called **Teensy 4.1 Settings**.
- **The receiver says what it is.** The Teensy asks it (`VERSIONA` for the UM982, `UBX-MON-VER` for the u-blox) and shows its model and firmware on Status, in the `[status]` messages and in the GPS tab.
- New **GPS** tab, in place of the "UM982" tab.
  - With the UM982 it has the command console.
  - With a u-blox F9P / X20P it reads and changes the receiver's settings: rate, dynamic model, minimum elevation, high-precision NMEA, NMEA sentences on UART1, constellations, RTCM3 input (see "GPS receiver" below).
- **BNO085 as the IMU.** The firmware finds the IMU at start-up: a TM171, or a BNO085 in RVC mode on Serial5 or on I2C (pins 18 / 19). The IMU card says which one (see "IMU" below).
- The IMU hello to AgIO goes out only when an IMU was found.
- **WAS check off (bench) and the WAS out of range:** the wheel angle goes to the program as 0°. With no WAS the reading is about 0 V, which showed the wheels at full lock to one side.
- The "Version on GitHub" card says **up to date** when the newest version on GitHub is the one installed.
- **Compiles in the Arduino IDE** with Teensyduino 1.57 or newer.
  - Checked by compiling exactly as the IDE does, with the Teensy cores 1.57, 1.58 and 1.62.
  - 7.5 stopped in the IDE with `conflicting declaration of 'void wdtWarnC(const uint32_t*)' with 'C' linkage`; its `.hex` was not affected.
  - Fewer compiler warnings (the code does the same).
- Checked that the AiO v4.5 Micro and Standard wire every Teensy pin it uses the same way.
- The name still says TM171, so that the pages of 7.5 find this version on GitHub.

### 7.5 (18 Oct 2026)

- **WAS check.** A WAS voltage outside 0.2-4.8 V (broken wire, short circuit, plug out, dead sensor or ADS1115) turns autosteer off and blocks it, like a kickout.
  - Only autosteer: the tractor's own steering keeps working.
  - It can be switched off on the web page for a bench without a WAS; the page and the program then warn.
- Autosteer stops **at once** (at the next 25 ms steering loop) when the switch, the button, a kickout or the WAS check turns it off.
- **Why autosteer went off** (switch, button, pressure / current / encoder kickout, WAS, the program silent) is shown on the program's screen, in Messages and on the web page.
  - AgOpenWeb: "AiO Board Msgs". AgOpenGPS: "Hardware Messages".
- **Hardware watchdog.** The Teensy restarts if the firmware stops for 2 s; half a second before, the autosteer outputs go off. It can be switched off on the web page.
- **Why the Teensy last started** (power, restart, watchdog, crash with its address): at boot, on the web page and on the program's screen.
- Web page, steering: the WAS voltage and reading; the pressure / current value with its highest value of the last 10 s and the kickout limit; the steering settings the program sent.
- Web page, "RTCM corrections" card: the radio (XBee) corrections counted like the NTRIP ones (bytes per second, RTCM messages, messages with a bad checksum, message types).
- The radio bytes go to the receiver as they arrive.
- **New version from GitHub.** The web page (and the XIAO's page) checks the `Firmware` folder of github.com/renatorgr/Boards, says when there is a newer version and can install it (see "New versions from GitHub" below).

### 7.4 (17 Oct 2026)

- **Automatic output mode**, now the default (see "Automatic output mode" below).
  - The module tells AgOpenWeb from AgOpenGPS by the hello each program sends once a second.
  - It sends the standard set to AgOpenWeb and `$PAOGI` / `$PANDA` to AgOpenGPS.
  - Settings saved by 7.3 or older become automatic once; AgOpenWeb or AgOpenGPS can still be chosen by hand.
- The web page and the XIAO page show which program is connected.
  - They warn when two programs talk to the module, or when AgOpenGPS is connected with the output set by hand to AgOpenWeb.
- **Web page in Portuguese or English**: the EN / PT button at the top; the browser remembers the choice. The `[status]` messages stay in English.
- **Credits at the top of every file**, a "Credits and licence" section, and the **GNU GPL v3** licence (file `COPYING`).

### 7.3 (16 Oct 2026)

- **Classic mode** (see "Output (UDP 9999), classic" below):
  - with the UM982, always `$PAOGI` (AgOpenGPS heading source "Dual"); without the dual heading, the IMU heading aligned with the antennas and then corrected by the GPS course;
  - with an X20P / F9P, `$PANDA` (heading source "Fix").
- The IMU yaw direction is also learned from the GPS course, and yaw jumps are compensated.
- Standard mode: the `$INHPR` heading turns the right way without the dual heading too, and has no step when the IMU is first aligned to the GPS course (checked with AgOpenWeb's own heading fusion).
- On the web page the output mode is called "Operating mode": **AgOpenWeb** (standard) or **AgOpenGPS** (classic).
- The XIAO program runs on the XIAO ESP32C3 and the XIAO ESP32S3; it refuses an update made for the other one, and the XIAO card shows which one it is.

### 7.2 (15 Oct 2026)

- The module status (the web page's `/api/status`) goes to the XIAO once a second, and every `[status]` line as it is written, so the XIAO's WiFi page shows them on a phone.
- The XIAO sends back its own state (program version, WiFi), shown in the XIAO card of this page.
- Serial2 transmit buffer: 2 KB.

### 7.1 (14 Oct 2026)

- XIAO link at **230400** baud (see "Link speed" below; the XIAO program uses the same).
- The version number is in the folder and file names (`zVersion.h`).

### 7.0 (14 Oct 2026)

- **XIAO ESP32C3 / ESP32S3 bridge** on Serial2 (Teensy pins 7 / 8). Same protocol as the ESP32 bridge of the official Teensy firmware v26.
  - Once the XIAO says `ESP32-hello`, every AgOpenGPS packet that reaches the module on UDP 8888 is copied to it.
  - The AgOpenGPS PGNs it sends go out on UDP 9999 (see "XIAO ESP32C3 / ESP32S3" below).
- A **XIAO card** on the web page (Status), an on/off setting (Settings), and `[xiao]` lines in Messages, with the text lines the XIAO writes.
- The program for the XIAO is in this folder: `v7.6_XIAO_ESP32/` (see its README), and already compiled in `v7.6_XIAO_ESP32_bin/`.

### v6 and v5

- v6: the **web page on the module** — `http://192.168.5.126/` from the AgOpenWeb tablet, a phone or a PC on the same network — with status, the `[status]` messages, settings, the UM982 console, restart and **firmware update over the network**, so the box does not have to be opened (see "Web page" below).
- v6: **classic output** for AgOpenGPS / AgIO: one `$PAOGI` / `$PANDA` per epoch (AgOpenWeb reads it too).
- v5: one firmware for the UM982 (dual antenna) and the X20P / F9P (single antenna), with the TM171, sending the standard NMEA set to AgOpenWeb.

## Compiling and first install

- Arduino IDE (1.8 or 2.x) with **Teensyduino 1.57 or newer** (checked with the cores 1.57, 1.58, 1.59, 1.60 and 1.62).
- Board: Teensy 4.1, USB type Serial, CPU 600 MHz, Optimize "Faster".
- Nothing to install besides Teensyduino: the BNO085 library and the flash routines are in the sketch folder.
- Lines marked "warning" in the IDE's output do not stop the build; only "error" does.
- The GPS and IMU ports are declared with the type `TeensySerial` (main .ino), so the same code builds with Teensyduino 1.58 and older and with 1.59 and newer.
- **Already compiled:** this folder also has `v7.6_UM982_X20P_F9P_TM171_WEB_XIAO.hex`, built with Teensyduino 1.58 (the version tested on the tractor). The Arduino IDE ignores it, and the XIAO folders, when it compiles the sketch.
  - From v6 or v7: web page → Firmware.
  - From v5 or older: by USB, Teensy Loader → File → Open HEX File → press the button on the Teensy.
- The files `FlashTxx.c` / `FlashTxx.h` are FlasherX (Joe Pasquariello, public domain), used for the update over the network. Keep them in the sketch folder.

## AiO v4.5 Micro and Standard

Checked against the AgOpenGPS design files of both boards (`AIO v4.5 Micro` and `AIO v4.5 Std`, EasyEDA, 2024-02-05): every Teensy pin this firmware uses goes to the same function on both.

| Function | Teensy pins | Micro | Standard |
|---|---|---|---|
| GPS receiver | Serial7, 28 / 29 | simpleRTK2B Micro socket, header H11 | Ardusimple socket P2, header P5, UM982 / Bynav socket H20 |
| GPS2 / XIAO | Serial2, 7 / 8 | Micro socket U11 ("Dual-Left"), header H1 | Ardusimple socket P3, header P8 (pin 1 RX2, pin 2 TX2) |
| Correction radio | Serial3 RX, 15 | XBee socket U24, connector P2 | 3-pin header H12 (5 V, GND, signal) |
| IMU on Serial5 (TM171, BNO085 RVC) | RX5, 21 | through jumper SJ1 | through jumper SJ1 |
| BNO085 on I2C | 18 / 19 | BNO085 U15, header H14 | BNO085 U15, header H19 |
| WAS (ADS1115) | Wire1, 16 / 17 | yes | yes |
| Steering outputs PWM1, PWM2, DIR | 2 / 3 / 4 | yes | yes |
| LEDs | 5, 6, 9, 10, 11, 12 | yes | yes |
| Steer / work / remote switches | 32 / 34 / 37 | yes | yes |
| Pressure sensor / current sensor | A10 (24) / A17 (41) | yes | yes |
| Keya CAN | CAN3, 30 / 31 | yes | yes |
| Ethernet | — | yes | yes |

- The IMU headers H14 (Micro) and H19 (Standard) have the same pins in the same order: 1 = 3.3 V, 3 = GND, 4 = SCL, 5 = SDA / RX5.
- For a TM171, or a BNO085 in RVC mode, jumper **SJ1** must join the BNO085's SDA line to RX5, on both boards.
- **XIAO:** the adapter board and the 3D-printed holder fit only the Micro's GPS2 socket. On the Standard, wire the XIAO to header P8 (pin 1 RX2 ← XIAO D6, pin 2 TX2 → XIAO D7) and take 3.3 V and GND from another header.
- **UM982 on the Standard:** its socket H20 has the RX / TX the right way round from board v4.4 on.
- **Ampseal connector (23 pins):** the same on both, except pin 18. On the Standard, pin 18 brings the power for the Cytron (through the current sensor). On the Micro, pin 18 is not used: the Cytron power comes in on its own 2-pin screw terminal (U23).

## Web page

Open `http://<module IP>/` — normally `http://192.168.5.126/`. The third number follows the subnet set from AgOpenWeb.

There is no password: anyone on the module's network can open it.

The page is in Portuguese or English:

- the button at the top right (EN / PT) switches it, and that browser remembers the choice;
- a browser that has not chosen gets Portuguese if its own language is Portuguese, English otherwise;
- the `[status]` messages are always in English;
- the English texts are the `EN` table in the page's script (`zWebPage.h`).

| Tab | What it has |
|---|---|
| **Status** | Warnings at the top: autosteer blocked by the WAS, WAS check off, ADS1115 missing, restarted after a firmware fault. |
| | GNSS: fix, satellites, HDOP, correction age, speed, the receiver's model. |
| | Dual heading: QF, why there is none, the last `$GNHPR` as received. |
| | IMU: which one (TM171 or BNO085) and where, roll, pitch, yaw, alignment, yaw direction. |
| | Steering: WAS angle, requested angle, PWM, autosteer engaged, switch, motor, WAS voltage and reading, pressure or current value (highest of the last 10 s, kickout limit), why autosteer last went off, the last message sent to the program's screen. |
| | The steering settings from the program. |
| | Network (cable, program), RTCM corrections from the program (NTRIP) and from the radio, what the receiver sends, the last sentences sent. |
| | The XIAO: present, last hello, packets each way, last PGN from it, checksum errors, losses. |
| **Messages** | The `[status]` lines (the module keeps the last 60). |
| **Settings** | The settings below, saved in the Teensy's EEPROM and applied at once (no restart). |
| **GPS** | The receiver: model, make, firmware, protocol, when it was identified, "Ask again". |
| | **u-blox F9P / X20P**: its settings, read and written from the page (see "GPS receiver" below). |
| | **UM982** (or not identified): a console. It sends a command to the UM982 port connected to the Teensy and shows the replies for 5 s. Quick buttons: `CONFIG`, `UNILOGLIST`, `GPHPR 0.1`, `SAVECONFIG`. Commands that can cut the link (`FRESET`, `UNLOG`, port baud) ask for confirmation. |
| **Firmware** | Version and build date, why the Teensy last started, the watchdog, the newest version on GitHub with "Install from GitHub", firmware update from a file, restart. |

The page refreshes once a second, one request at a time. The module answers between GNSS epochs and never waits for the browser for more than 3 s (0.5 s for a connection that sends nothing), so steering and GNSS keep running.

### Settings (web page, EEPROM)

| Setting | Default | |
|---|---|---|
| Operating mode (output) | automatic | **Automatic**: the standard set while AgOpenWeb sends the hello; classic for AgOpenGPS and until the first hello. **AgOpenWeb**: always the standard set (AgOpenWeb with PR #300). **AgOpenGPS**: always classic `$PAOGI` / `$PANDA`. |
| Accept a float heading (HPR QF 5) | yes | No = only QF 4 (fixed). |
| Longest wait for the VTG / HPR after the GGA | 60 ms | 20-100 ms. |
| Swap roll and pitch (IMU) | no | IMU turned 90° on the board. |
| Yaw direction (IMU) | automatic | Automatic (learned in turns, from the dual heading or, without one, from the GPS course), +1 or −1. Once you know it, set it: the IMU heading is then usable from start-up. |
| XIAO bridge | on | Off = the Teensy ignores whatever is on Serial2. With no XIAO, on or off makes no difference. |
| Check the WAS sensor | on | Off only on a bench with no WAS: a WAS out of range then does not block autosteer. While off: a warning on Status, at boot, and on the program's screen. Switching it off asks for confirmation. |
| Watchdog | on | Off = no hardware watchdog, from the next start (once on, the chip's watchdog cannot be stopped until a restart). Switching it on works at once. |

- The defaults are the `#define`s at the top of `zHandlers.ino`; a blank or damaged EEPROM block gives the defaults.
- The settings live at EEPROM address 200, away from the AgOpenGPS steer settings (0-63), and survive firmware updates.

### Firmware update over the network

1. Arduino IDE: **Sketch → Export Compiled Binary**. The `.hex` is in the sketch folder (Arduino IDE 2: in `build/teensy.avr.teensy41/`).
2. Web page → Firmware → choose the `.hex` → Install.
   - Only with **autosteer disengaged and the tractor stopped** (otherwise refused).
   - The steering is switched off for the update.
3. The module receives the whole file into a free part of its flash (4-6 MB) and checks it:
   - Intel HEX checksums and end record;
   - starts at the flash base and fits;
   - built for a Teensy 4.1 (`fw_teensy41`);
   - this firmware family (`AOG-UM982-TM171-WEB-OTA-1`), so the next update can also be done from the page.
   - Every sector written is read back. **If anything fails, the running firmware stays** and the page says why.
4. Only then is the new firmware copied over the old one (a few seconds) and the Teensy restarts. The page shows the new version when it is back.

- "Other firmware" installs an image without the marker (for example a firmware without this page); the next update is then by USB again.
- **Do not cut the power during the final copy** (a few seconds after "Installing and restarting"). The Teensy would then have no firmware and must be reprogrammed by USB; the Teensy itself is not damaged, and the 15 s button restore also works.
- The EEPROM (steer settings from AgOpenGPS / AgOpenWeb, web settings) is kept: `FLASH_RESERVE` in `FlashTxx.h` covers the top 256 KB of flash, where the Teensy 4.1 keeps its EEPROM (the stock FlasherX value, 16 KB, would erase it).

### New versions from GitHub

The page looks for the newest version in the `Firmware` folder of **github.com/renatorgr/Boards**: when it opens, every 6 hours while it stays open, and with **Check now** (Firmware tab).

- Each version is one folder named like the firmware, `vX.Y_UM982_X20P_F9P_TM171_WEB_XIAO`, with `vX.Y_UM982_X20P_F9P_TM171_WEB_XIAO.hex` in it. For the XIAO: `vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` and `...S3.bin`. Other folders and files there are ignored.
- A newer version than the one running shows at the top of Status and in the Firmware tab, with a link to its folder on GitHub.
- **Install from GitHub**: the browser downloads that `.hex` and sends it to the Teensy exactly like a chosen file (same checks, same rules). Its size must match what GitHub lists.
- It is **the device the page is open on** (tablet, phone, PC) that talks to GitHub, so that device needs internet. The Teensy never goes to the internet.
- Without internet the page says "no answer from GitHub" and nothing else changes.
- GitHub answers 60 such questions an hour per internet connection without a login.

To publish a new version: upload the whole version folder, as it comes in the zip (`.hex` and XIAO folders included), into `Firmware/` on GitHub. Nothing else needs changing.

## GPS receiver: identification and u-blox settings (`zReceiver.ino`)

Once the receiver has been talking for a second, the Teensy asks it what it is, on Serial7:

- **UM982** (Unicore): the command `VERSIONA`. It answers `#VERSIONA,...;"UM982","R4.10Build11826",...` (model and firmware).
- **ZED-F9P / ZED-X20P** (u-blox): the poll `UBX-MON-VER`. It answers `MOD=ZED-F9P`, `FWVER=HPG 1.32`, `PROTVER=27.31`.

How it asks:

- First the question that fits what the receiver sends (HPR = Unicore), then the other one, up to 6 questions (9 s).
- Asked again when the receiver talks again after 10 s of silence (another receiver, or one switched on later), and with **Ask again** on the page.
- The questions go to the receiver between two RTCM messages of the corrections, never in the middle of one.
- The answers, and any UBX message the receiver is set to send, are taken out before the NMEA parser. The UBX messages show in "the receiver sends" (for example `UBX-NAV-PVT 1.0 Hz`).
- The UM982's answer to the automatic `VERSIONA` is not shown in the console (a `VERSIONA` typed in the console is).

The result:

- `[status] GPS receiver: u-blox ZED-F9P, firmware HPG 1.32 (protocol 27.31)`, on Status (GNSS → Receiver) and in the GPS tab.
- No answer: "not identified". Most likely a u-blox with its UBX output off on UART1: the page then offers to switch it on.
- A receiver that sends HPR but does not answer is taken as a Unicore.
- What is sent to the program does not depend on this: `$PAOGI` / `$PANDA` and the standard set follow the HPR.

### u-blox settings (GPS tab)

- They are read from the receiver the first time the tab is opened, and with **Read from the receiver** (UBX-CFG-VALGET, RAM layer: what it uses now).
- **Write to the receiver** sends only what was changed, in one UBX-CFG-VALSET, to RAM, BBR and flash (kept after a power off).
- The receiver takes all the changes or none.
- If it refuses the flash, they are written again to RAM and BBR only, and the page says so.
- Only with autosteer off. The settings are read again after a write.

| Setting | u-blox key | |
|---|---|---|
| Rate | CFG-RATE-MEAS | 1, 2, 5, 8, 10 or 20 Hz (AgOpenGPS / AgOpenWeb: 10 Hz) |
| Dynamic model | CFG-NAVSPG-DYNMODEL | portable, stationary, pedestrian, automotive, sea, airborne, wrist, bike, lawn mower, e-scooter (the receiver refuses one it does not have) |
| Minimum satellite elevation | CFG-NAVSPG-INFIL_MINELEV | 0-25° |
| High-precision NMEA | CFG-NMEA-HIGHPREC | more decimals in latitude, longitude and altitude |
| NMEA sentences on UART1 | CFG-MSGOUT-NMEA_ID_..._UART1 | GGA and VTG (they cannot be switched off here: the Teensy needs them), RMC, GSA, GSV, GLL, GST, ZDA |
| Constellations | CFG-SIGNAL-..._ENA | GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS. Changing them restarts the receiver's GNSS: RTK is lost for a few seconds |
| RTCM3 corrections in on UART1 | CFG-UART1INPROT-RTCM3X | off = no RTK |
| Shown only | CFG-UART1-BAUDRATE, UART1 UBX in / out, NMEA out | never changed here: the Teensy would lose the receiver |

- The keys are the same on the ZED-F9P and the ZED-X20P (u-blox interface descriptions; checked against the key list of SparkFun's u-blox GNSS library).
- A setting the receiver does not have shows "–".
- The baud rate and a factory reset are left out on purpose: a factory reset puts UART1 at 38400, where the Teensy cannot hear it.
- Messages: `[web] GPS receiver settings: rate=200, gal=0`, then `[status] GPS receiver: settings written to RAM, BBR and flash` or `settings NOT written (the receiver refused them)`.

## IMU: TM171 or BNO085 (`zImu.ino`)

At start-up the firmware looks for the IMU, in this order (as the AgOpenGPS AiO v4 firmware does for the BNO085):

1. **Serial5** at 115200 for 1.5 s: a **TM171** (in the BNO085's place on the board) or a **BNO085 in RVC mode** (frames `AA AA`, 19 bytes, 100 a second). Jumper SJ1 joins that line to RX5.
2. If nothing answers there: a **BNO085 on I2C** (Wire, Teensy pins 18 / 19), address 0x4A, then 0x4B. It is read every 10 ms (it reports every 20 ms).
3. Nothing: `[boot] IMU NOT detected ...`. Serial5 is still listened to: a TM171 or BNO085 that starts later is taken then (`[status] IMU: TM171 found on Serial5 (it started after the Teensy)`).

The `[boot]` line says which one: `IMU: TM171 on Serial5`, `BNO085 on Serial5 (RVC mode)` or `BNO085 on I2C 0x4A (pins 18 / 19)`. The Status IMU card shows it too (Type).

- All of them give the same roll, pitch and yaw, so everything else works the same: alignment to the dual heading or the GPS course, yaw direction learned (or set on the page), yaw jumps compensated, the output to the program.
- Roll / pitch / yaw as in the AiO v4 firmware:
  - RVC: roll = its pitch, pitch = its roll;
  - I2C (game rotation vector): roll about X, pitch about Y;
  - the yaw turns clockwise.
- "Use Y axis" in the program and "Swap roll and pitch" on the page swap roll and pitch. Roll sign and zero: the program's "Roll invert" / "Roll zero".
- RVC or I2C is chosen on the BNO085 module itself (its PS0 / PS1 pins), not in the firmware.
- A BNO085 on I2C that stops reporting for 3 s is started again (at most every 10 s), only with autosteer off: starting it takes about half a second.
- The IMU hello to AgIO (so AgIO shows the IMU as connected) goes out only when an IMU was found.
- The BNO085 was tested on the PC only: check roll and heading on the bench before steering with it.

## Autosteer safety (`zSafety.ino`)

Everything here acts only on autosteer, that is on the module's own outputs (the valve PWM and relay, or the Keya motor), exactly as the steer switch does. The tractor's own steering, the steering wheel and its orbitrol, is never touched and keeps working.

### WAS check

The WAS (wheel angle sensor) is read by the ADS1115 at ±6.144 V full scale, 40 times a second.

- In single-input mode (the AgOpenGPS / AgOpenWeb default) its voltage must stay between **0.2 and 4.8 V**.
- A broken signal or ground wire, a short circuit, a plug that came out, a dead sensor or an ADS1115 that stops answering give a reading at one end. The firmware would then see the wheels fully turned and drive the valve hard the other way.

What the check does:

- 4 readings in a row out of range (100 ms) = fault: autosteer is turned off at once, like a kickout (the program sees the steer switch off), and cannot be engaged.
  - Messages: `[status] WAS: 0.05 V, out of the 0.2-4.8 V range (wire, plug or sensor?) - autosteer blocked`.
  - On the program's screen: `WAS fault (0.05 V): autosteer blocked`.
- A short glitch (up to 3 readings, 75 ms) is ignored.
- The fault is over after 1 s back in range (`[status] WAS: 2.51 V, back in range`).
- Autosteer then has to be engaged again by hand: the program's button, or with a steer switch, off and on again. It never comes back by itself while the switch is still on.
- Differential input (two-wire WAS): only a difference beyond ±4.8 V is a fault; a broken wire there may not be seen.
- A WAS whose signal wire breaks but floats in the middle of the range cannot be told from a real reading by the voltage alone.

**Bench without a WAS:**

- With nothing on the WAS input the reading is close to 0 V, exactly what a broken wire gives, so autosteer cannot be engaged to test the valve or the motor.
- Switch the check off on the web page (Settings → Autosteer safety).
- While it is off: a warning at the top of Status, a `[boot]` line at each start, and `WAS check OFF (bench setting)` on the program's screen when it connects.
- While it is off and the WAS reads out of range, the wheel angle goes to the program as 0° (`[status] ... wheel angle sent as 0 deg`, and "angle sent: 0°" next to the WAS voltage on Status). The motor or valve then follows only the angle the program asks for.
- Switch it back on before using the tractor.

### Autosteer off at once

When the steer switch goes off, the steer button is pressed, a pressure / current / encoder kickout trips or the WAS check blocks, autosteer stops at the next steering loop (25 ms), without waiting for the program's next steering packet.

### Why autosteer went off

| Cause | Program's screen (PGN 221) | Messages | Web page |
|---|---|---|---|
| steer switch / steer button | `Autosteer off: steer switch` (3 s) | `[status] Autosteer OFF: steer switch` | yes |
| pressure / current kickout | `Autosteer off: pressure kickout (86, limit 80)` (6 s, warning colour) | yes | yes |
| steering wheel encoder | `Autosteer off: wheel encoder kickout (5 pulses)` | yes | yes |
| WAS out of range | `WAS fault (0.03 V): autosteer blocked` (8 s) | yes | yes |
| no steering data from the program for 2.5 s | `Autosteer off: no steering data from the program for 2.5 s` | yes | yes |
| the program turned it off, or its speed below 0.1 km/h | — (the program knows) | — | yes |

- **AgOpenWeb**: shown when the display setting "AiO Board Msgs" is on (it is by default).
- **AgOpenGPS**: shown when "Hardware Messages" is ticked in its configuration (off by default).
- The message format is AgOpenGPS's PGN 221: `80 81 7E DD <text length + 2> <seconds> <0 = warning colour> <text> <CRC>`.

### Hardware watchdog

- The Teensy 4.1's watchdog (WDOG1) restarts the Teensy if the firmware stops for **2 s** (a hang, or a crash; the Teensy core would otherwise wait 8 s, with the autosteer outputs as they were).
- Half a second before the restart, an interrupt switches the autosteer outputs off (Cytron PWM and valve relay, or IBT2) and notes which part of the loop was running and at what address.
- Normal operation never waits that long: the web page, the firmware update and the receiver check from AgIO keep it fed. Before the final copy of a firmware update its timeout goes to 128 s.
- It can be switched off on the web page (from the next start).
- With a Keya motor the interrupt does not touch the CAN bus; the restart stops the CAN messages.

### Why the Teensy started

At each start the firmware reads the chip's reset status (and the Teensy core's crash record) and says why:

- in a `[boot]` line;
- on the web page (Firmware → Last start);
- after a watchdog or a crash, on the program's screen when it connects (`Module restarted by its watchdog`).

| Reason | |
|---|---|
| power on | switched on, or the Teensy's button, or a USB upload |
| restart by software | the web page, a firmware update, or the program's subnet change |
| WATCHDOG | the firmware stopped for 2 s; the line gives the part of the loop and the address. Please send it |
| CRASH | the firmware stopped with a fault; the line gives the address (and whether the watchdog restarted it). Please send it |

With the address and the same build, the line of code can be found.

## Inputs

- **GNSS receiver** on Serial7, 460800 baud, 10 Hz. Its model is asked (see "GPS receiver"); what is sent to the program follows what it sends:
  - **UM982 (dual antenna):** GGA + VTG + HPR (GP or GN talker). KSXT is not used (it can stay on; it is ignored).
  - **u-blox F9P / X20P (single antenna):** GGA + VTG. With no HPR, the firmware does not wait for it: each epoch goes out as soon as the GGA and VTG are in.
  - GGA is required (it marks each epoch).
  - A VTG or HPR counts as "sent by the receiver" if one arrived in the last 3 s (`RECEIVER_SEEN_MS`); for the first 3 s after start-up both are waited for.
- **IMU:** TM171 (Serial5, 115200), or BNO085 in RVC mode (Serial5, 115200) or on I2C (pins 18 / 19): roll, pitch and yaw (see "IMU").
- **Correction radio** (Serial3 RX, 115200; XBee LR on the Micro, the 3-pin radio header on the Standard): RTCM corrections, passed on to the receiver as they arrive.
  - They are counted (`zRtcm.h`): bytes per second, whole RTCM 3 messages with a good checksum (CRC-24Q), messages with a bad checksum (a weak radio signal, a wrong baud rate), the message types of the last 10 s.
  - The NTRIP corrections from the program (UDP 2233) are counted the same way.

## Output (UDP 9999), standard set: 4 lines per epoch, one per datagram

| Line | Content | AgOpenWeb uses it for |
|---|---|---|
| `$GNGGA` | the receiver's GGA (a UM982 GGA with `00` satellites / HDOP `9999.0` gets the last good values, for up to 2 s) | position, fix, satellites, HDOP, correction age |
| `$GNVTG` | speed and track from the receiver's VTG; if the VTG is empty, computed from the GGA positions | speed |
| `$GNTHS` | dual-antenna heading (the UM982's HPR value, character for character); mode `A` = valid, `V` = no dual heading | heading with two antennas |
| `$INHPR` | the IMU (talker `IN` = inertial sensor): heading aligned to the dual heading, **roll in the pitch field** (HPR convention), pitch in the roll field; QF 4 = IMU OK, QF 0 = IMU lost | roll, always; heading without two antennas |

Without the dual heading, a re-alignment of the `$INHPR` heading by the GPS course makes no step: AgOpenWeb aligns the IMU heading itself.

Result in AgOpenWeb (checked against AgOpenWeb's own code):

| Situation | Heading | Roll |
|---|---|---|
| Two antennas (HPR QF 4 fixed, or 5 float) | `$GNTHS` (antennas) | IMU |
| Second antenna lost / no heading solution | IMU, with AgOpenWeb's single-antenna fusion (fix-to-fix + IMU, reverse detection) | IMU |
| Single-antenna receiver (F9P / X20P) | IMU, with the same single-antenna fusion | IMU |
| IMU lost | antennas (or fix-to-fix without two antennas) | 0 |

AgOpenWeb's status bar shows the family **`GGA+VTG+HPR+THS`** (the "HPR" is the IMU's `$INHPR`).

### Why the dual heading goes out as `$GNTHS` and not as `$GNHPR`

- AgOpenWeb starts a new epoch when a sentence type repeats.
- A `$GNHPR` (antennas) and an `$INHPR` (IMU) are both "HPR", so they could never be in the same fix: the `$INHPR` would open an epoch with no GGA and be dropped.
- `$GNTHS` carries the same heading as the UM982's HPR and can share the epoch with the `$INHPR`.
- The firmware still **reads** the UM982's `$GNHPR`.

## Output (UDP 9999), classic: one sentence per epoch

For AgOpenGPS / AgIO (AgOpenWeb reads it too). The sentence depends on the receiver:

**UM982 (has sent HPR since start-up): always `$PAOGI`. AgOpenGPS: GPS heading source "Dual", Dual heading offset 0.**

| Situation | Heading | Roll / pitch |
|---|---|---|
| Dual heading (HPR QF 4, or 5 if accepted), confirmed 5 epochs in a row | the UM982 HPR heading, character for character | IMU, degrees |
| Dual heading lost | IMU: aligned to the dual heading while it was valid, then corrected slowly by the GPS course when driving straight (forward and reverse) | IMU |
| Dual heading lost, IMU turning direction not known yet (not learned, not set) or IMU lost | GPS course while moving (reverse told by continuity), held when stopped | IMU (`0.00` if lost) |
| Dual heading back | faded from the previous heading over 1 s if within 5°, else at once | IMU |

Why no `$PANDA` here:

- With the heading source on "Dual", AgOpenGPS ignores the heading of a `$PANDA` (it is an IMU heading; `headingTrueDual` is not updated), and the tractor's heading freezes at the last dual value.
- So the firmware keeps sending `$PAOGI` with the best heading it has.
- A UM982 that stops sending HPR altogether also keeps `$PAOGI`.

**F9P / X20P (never sent HPR): `$PANDA`. AgOpenGPS: GPS heading source "Fix".**

| Situation | Heading | Roll / pitch |
|---|---|---|
| IMU OK, turning direction known (learned from the GPS course, or set on the page) | IMU heading × 10 (aligned to the GPS course); AgOpenGPS fuses it with fix-to-fix (IMU fusion, reverse detection) | IMU, × 10 |
| IMU OK, direction not known yet | `65535` (no IMU): AgOpenGPS uses fix-to-fix only until the direction is learned (about 30° of curves) | IMU, × 10 |
| IMU lost | `65535` | `0` |

- The IMU heading only goes to AgOpenGPS once it already turns the right way and points the right way, so AgOpenGPS's IMU offset starts from the right value (an unaligned IMU appearing mid-run can be read as reversing).
- Common to both: the IMU heading is kept continuous when its yaw jumps (more than 25° in 100 ms, compensated in the offset).
- The yaw direction can be learned from the dual heading (60° of turns) or the GPS course (30°).
- Messages: `[status] TM171: yaw direction learned ...` (or `BNO085: ...`), `heading aligned to the GPS course`, `yaw jumped ... compensated`.

Fields: time, latitude, N/S, longitude, E/W, fix, satellites, HDOP, altitude, correction age (from the GGA), speed in knots (VTG, else from the positions), heading, roll, pitch, yaw rate (0).

- Checked with AgIO's parser (UDP filter, checksum, field formats) and AgOpenGPS's heading code (`UDPComm` PGN 0xD6, "Dual" and "Fix" in `Position.designer.cs`) in a 300 s simulation: every sentence accepted.
- The web page (Status → Sentences sent) shows which heading went out and the AgOpenGPS heading source to use.

## Automatic output mode

AgOpenWeb and AgIO (AgOpenGPS) both send the modules a hello, PGN 200, once a second to UDP 8888. The 9 bytes are the same except the last one:

| Program | Hello | Last byte |
|---|---|---|
| AgIO (AgOpenGPS) | `80 81 7F C8 03 38 00 00 47` | `0x47`, a fixed byte AgIO never computes (`AgIO/Source/Forms/UDP.designer.cs`) |
| AgOpenWeb | `80 81 7F C8 03 38 00 00 82` | the real checksum (`PgnBuilder.BuildHelloPacket`) |

In automatic mode the module follows the program it hears (`zStatus.ino`):

- AgOpenWeb: the standard set. AgOpenGPS: classic (`$PAOGI` with the UM982, `$PANDA` with an F9P / X20P).
- Before the first hello, and for a hello that is neither: classic, because AgOpenWeb reads `$PAOGI` / `$PANDA` too, while AgOpenGPS does not read `$GNTHS` / `$INHPR`.
- After a start-up, or 5 s after the last hello of the program followed, the first hello decides.
- While a program is talking, the other one needs 3 hellos in a row to take over. With both talking at once the module keeps the first one and says so in Messages.
- The switch is per epoch, no restart; it is the same as changing the setting on the page.
- If a future AgIO computes that checksum, it will look like AgOpenWeb: choose AgOpenGPS by hand then.

## Single antenna: u-blox F9P / X20P

- In u-center: UART1 at 460800, GGA + VTG at 10 Hz (100 ms measurement period), high-precision NMEA on, compatibility mode and Limit82 off, RTCM3 input on UART1, all other messages off. NMEA version: any (4.11 recommended). Save to the receiver's memory (BBR / flash).
- Once UART1 is at 460800 (so the Teensy hears it), the rate, NMEA sentences, high precision, constellations and RTCM3 input can also be checked and changed on the module's web page, GPS tab.
- In AgOpenWeb: **"Dual GPS" off**. `$GNTHS` always goes out with `V`; AgOpenWeb uses the IMU heading with its single-antenna fusion (fix-to-fix + IMU, reverse detection), as with `$PANDA`. Set the antenna height and position in the vehicle profile.
- In AgOpenGPS (classic output): GPS heading source **"Fix"**; the firmware sends `$PANDA`.
- IMU yaw direction: learned from the GPS course in the first curves (about 30°, above 2 km/h), then `[status] TM171: yaw direction learned from the GPS course: ±1`. Set that value on the web page so it is known from start-up.
- The IMU heading has an arbitrary zero (about 100° off on this tractor's X20P box). The firmware aligns it to the GPS course when driving straight above 3 km/h, and AgOpenWeb / AgOpenGPS keep correcting it with fix-to-fix.
- The message `[status] Single antenna: no HPR from the receiver ...` confirms the mode.

## UM982 setup

- In UPrecise (or by commands), on the port connected to the Teensy, at 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, then save (`SAVECONFIG`).
- Keep `CONFIG HEADING OFFSET` as it was (the HPR heading already has that offset applied).

## In AgOpenWeb (UM982, dual antenna)

- **"Dual GPS" on** (when off, AgOpenWeb ignores the antenna heading and uses only the IMU + fix-to-fix).
- With `AutoDualFix` on, above `DualSwitchSpeed` AgOpenWeb uses the fix-to-fix heading corrected by the antennas instead of the antenna heading itself; turn it off to steer on the antenna heading.
- Antenna orientation: `CONFIG HEADING OFFSET` in the UM982 **or** `DualHeadingOffset` in AgOpenWeb — only one of the two.
- Roll direction / zero: AgOpenWeb's "Roll invert" and "Roll zero" (they apply to the `$INHPR` roll).
- `MinFixQuality` (default 4 = RTK fixed): below it the fix is marked invalid. On the bench set 1 or 2.
- HDOP and correction age come from the GGA.
- NTRIP (ReNEP): both port 2101 (legacy 1004/1012) and port 2102 (MSM5, GPS + GLONASS + Galileo + BeiDou) give RTK fixed with the UM982 on the tractor (2102 on 11 Oct: RTK fixed 21 s after the first DGPS fix).
- Antenna cables: if the two UM982 antenna cables are swapped, the dual heading is 180° off and the GGA position is the other antenna (with side-by-side antennas, the full baseline away from where `AntennaOffset` expects it).
  - Symptom: driving forward is shown as reverse.
  - Swap the cables back (or change `CONFIG HEADING OFFSET` by 180° and `AntennaOffset` to the other side).
- The line `[status] GPS: the receiver sends (last 5 s): ...` shows what the receiver sends: a UM982 that sends `KSXT` and no `GNHPR` needs `GPHPR 0.1` on its Teensy port (then `SAVECONFIG`).

## Compile-time settings (top of `zHandlers.ino`)

The first four are only the **defaults** of the web page settings.

- `HPR_ACCEPT_FLOAT` 1 = dual heading also with HPR QF 5 (float); 0 = only QF 4 (fixed).
- `EPOCH_WAIT_MS` 60: the firmware sends the epoch as soon as the VTG and the HPR with the GGA's time are in; if one is missing, it sends 60 ms after the GGA. A sentence the receiver does not send at all (HPR on an F9P / X20P) is not waited for (`RECEIVER_SEEN_MS` 3000).
- `TM171_SWAP_ROLL_PITCH`: IMU turned 90° on the board ("Use Y axis" also swaps roll / pitch).
- `TM171_YAW_SIGN`: 0 = yaw direction learned from real turns of the whole rig, against the dual heading (`SIGN_LEARN_DUAL_DEG` 60°) or the GPS course (`SIGN_LEARN_COURSE_DEG` 30°); +1 / -1 forces it.
- `TM171_JUMP_DEG` 25: an IMU yaw step larger than this in one epoch is a glitch and is compensated.
- `DUAL_ENTER_EPOCHS` 5, `DUAL_BLEND_EPOCHS` 10, `DUAL_BLEND_MAX_DEG` 5: classic `$PAOGI`: dual heading used again after 5 valid epochs, faded in over 10 epochs when within 5°.
- `COURSE_*`: IMU heading corrected by the GPS course without dual heading.
  - Course over ≥ 1.5 m, above 2 km/h, IMU turning < 2° over that stretch, 2 % of the error per epoch (max 0.2°).
  - 3 s of straight driving with a course that fits neither forward nor reverse re-aligns it.
  - Never aligned since start-up: aligned to the course above 3 km/h (forward assumed).
- `GGA_HOLD_MS`: how long good satellites / HDOP values are repeated when the UM982 sends `00` / `9999.0`.
- Diagnostics (0 on the tractor):
  - `FUSION_DEBUG` (0): one `[fusion] DUAL ...` or `[fusion] SINGLE ...` line per second: HPR QF and heading, `antRoll` (roll measured by the antennas, to compare with the IMU's), IMU alignment, heading and roll sent, speed in km/h and its source (`vtg` or `positions`).
  - `RAW_NMEA_DEBUG` (0): every GGA, VTG and HPR as the receiver sends it, prefixed `[raw]`.
  - `SEND_TO_USB` (0): the 4 lines sent to AgOpenWeb are also printed on the serial monitor.

## Status messages (`zStatus.ino`)

`[status] ...` lines are written **only when something changes** (a new state must last 1 s).

- They go to the USB serial monitor (`STATUS_TO_USB`) and to the web page (Messages).
- `[boot]` lines at start-up: the firmware and settings, why the Teensy started, the IMU found, the watchdog, the WAS check off.
- `[web]` lines for what is done from the page: settings saved, UM982 commands, u-blox settings, updates, restarts.

`[status]` lines:

- Ethernet: cable connected (module IP and destination) / cable unplugged.
- AgOpenWeb / AgOpenGPS talking (PC IP) / silent for 5 s.
- Program: `Program: AgOpenGPS (AgIO hello from 192.168.5.10). Automatic mode: classic sentences ...` (or AgOpenWeb, standard sentences), when the program changes.
  - A warning with the output set by hand to AgOpenWeb and AgOpenGPS connected.
  - `Hellos from AgOpenWeb AND AgOpenGPS: two programs talk to the module ...`
- RTCM corrections arriving from the program (NTRIP) / stopped 5 s ago.
- Radio (XBee): RTCM corrections arriving / stopped 5 s ago.
- NTRIP or radio: `N RTCM messages with a bad checksum in the last 5 s, M good` / no more; `N bytes in the last 5 s but no whole RTCM message (baud rate? not RTCM 3?)`.
- Autosteer OFF: why, when the module turned it off (see "Autosteer safety").
- WAS: out of range - autosteer blocked / back in range.
- GPS: first position; fix changes with satellites and correction age; no GGA for 5 s; VTG arriving / no VTG (speed computed from the positions).
- GPS receiver: `u-blox ZED-F9P, firmware HPG 1.32 (protocol 27.31)` / `Unicore UM982, firmware R4.10Build11826` / not identified; u-blox settings written / NOT written.
- Dual: dual-antenna heading OK (QF 4 fixed / 5 float, sent as `$GNTHS`, or `$PAOGI` in classic mode) / no dual heading (classic: IMU heading or GPS course in `$PAOGI`, and why).
- Classic with F9P / X20P: `Single antenna: ... sending $PANDA with the TM171 heading - set the AgOpenGPS heading source to Fix`.
- IMU (TM171 / BNO085): data OK / no data; yaw direction learned (from the dual heading or the GPS course); heading aligned to the GPS course; yaw jump compensated; re-aligned to the course; found later on Serial5; I2C BNO085 silent and started again.
- No dual heading: **why**, followed by the last HPR line exactly as the receiver sent it (`[status]   last HPR from the receiver: $GNHPR,...`). Printed when the dual heading is lost, and again when the reason changes:
  - `the UM982 sends HPR with QF 0` — the UM982 itself has no heading from its two antennas (antennas, cables, UM982 heading setup); the firmware can only pass that on;
  - `the HPR arrives too late, N ms after its GGA` — the HPR comes after the firmware has sent the epoch (it waits `EPOCH_WAIT_MS`);
  - `HPRs arrive but their time (...) does not match the GGA time (...)`;
  - `the receiver stopped sending HPR (last one N ms ago)`;
  - `HPR QF n but its heading field is empty or out of range`.
- Single antenna: the receiver sends no HPR (F9P / X20P, or HPR off on the UM982). Followed by `but HPR lines are arriving and none is accepted` (a firmware / format problem: send that line) or by when the last HPR line arrived.
- GPS: the receiver sends (last 5 s): every sentence type that comes in on Serial7 with its rate, for example `GNGGA 10.0 Hz, GNHPR 10.0 Hz, GNVTG 10.0 Hz`. Counted before the parser, so it shows exactly what the receiver sends. Printed when the set of types changes.
- GPS: sentences from the receiver with a bad checksum in the last 5 s (bytes lost or corrupted on the serial line) / no more bad checksums.
- GPS: the Teensy is reading the receiver late (more than one epoch waiting in the serial buffer) / read on time again.

`[xiao]` lines:

- `XIAO said hello on Serial2: ...` / `XIAO silent for 10 s (no hello): ...`
- `first PGN from the XIAO: <pgn> (source <src>, <n> data bytes) -> UDP 9999`, after each hello that follows a silence.
- `frame from the XIAO dropped: bad checksum (...)`, at most one line every 10 s with the running count.
- `> <text>`: a text line written by the XIAO (see below).

LEDs: green = dual heading, red = IMU heading, red blinking = IMU lost.

## Files

| File | |
|---|---|
| `v7.6_UM982_X20P_F9P_TM171_WEB_XIAO.ino` | setup and loop |
| `zVersion.h` | the version number (`FW_VERSION`) and name (`FW_NAME`), shown on the web page; the folder and the main .ino carry the same name |
| `zHandlers.ino` | receiver sentences, epoch assembly, standard set / classic output, UM982 reply capture |
| `zReceiver.ino`, `zReceiver.h` | which receiver (VERSIONA / UBX-MON-VER), UBX frames out of the NMEA stream, the u-blox settings of the GPS tab |
| `zImu.ino`, `zImu.h` | which IMU (TM171, BNO085 RVC, BNO085 I2C) and the BNO085 reading |
| `TM171.ino` | reads the TM171 |
| `BNO08x_AOG.cpp`, `BNO08x_AOG.h` | the BNO080 / BNO085 I2C library (SparkFun, MIT), as in the AgOpenGPS AiO firmware |
| `zSafety.ino`, `zSafety.h` | autosteer safety: WAS check, why autosteer went off, messages on the program's screen, hardware watchdog, why the Teensy started |
| `zRtcm.h` | counts the RTCM 3 messages of the corrections, radio and NTRIP |
| `zStatus.ino`, `zStatusLog.h` | `[status]` messages and the log kept for the web page |
| `zSettings.ino`, `zSettings.h` | web page settings (EEPROM) |
| `zWeb.ino`, `zWebPage.h`, `zWebJson.h` | web server, the page, JSON answers |
| `zOTA.ino`, `FlashTxx.c`, `FlashTxx.h` | firmware update (FlasherX flash routines) |
| `zXiao.ino`, `zXiao.h` | XIAO ESP32C3 / ESP32S3 bridge on Serial2 |
| `Autosteer.ino`, `AutosteerPID.ino`, `KeyaCANBUS.ino`, `zEthernet.ino`, `zUdpNtrip.ino`, `zADS1115.*`, `zNMEAParser.h` | from the AgOpenGPS / Gunics firmware, with small changes (each file says which at the top) |
| `COPYING` | the licence: GNU GPL v3 |
| `v7.6_UM982_X20P_F9P_TM171_WEB_XIAO.hex` | the firmware already compiled (Teensyduino 1.58) |
| `v7.6_XIAO_ESP32/` | the program for the XIAO (its own sketch, compiled for the XIAO, not the Teensy) |
| `v7.6_XIAO_ESP32_bin/` | the XIAO program already compiled, for the XIAO ESP32C3 and ESP32S3 |

## XIAO ESP32C3 / ESP32S3

### Hardware

- XIAO ESP32C3 or XIAO ESP32S3: same pins and size.
- **AiO v4.5 Micro:** in the GPS2 socket ("Dual-Left", U11), through the adapter board (`AiO_Micro45_XIAO_C3`) or the 3D-printed holder.
  - Socket pin 1 = 3V3.
  - Pin 2 = Teensy pin 7 (RX2) ← XIAO D6 (TX; GPIO21 on the C3, GPIO43 on the S3).
  - Pin 3 = Teensy pin 8 (TX2) → XIAO D7 (RX; GPIO20 / GPIO44).
  - Pins 10 / 11 = GND.
- **AiO v4.5 Standard:** the adapter and the holder do not fit. Wire the XIAO to header P8 (pin 1 = RX2 ← XIAO D6, pin 2 = TX2 → XIAO D7), with 3.3 V and GND from another header.
- The GPS2 port then has no second F9P (not needed with the UM982).
- The XIAO is powered from the board's 3.3 V: **a USB cable to it must not carry 5 V (VBUS)** while it is on the board (a panel USB port: data and GND only, see the XIAO program's README).
- Program it off the board, through such a data-only port, or over WiFi with its own page. The Teensy cannot program the XIAO: its BOOT and RESET pins are not wired.
- The Hammond box is aluminium: for WiFi or Bluetooth use an external antenna on the XIAO's U.FL connector.

### Protocol (as the official firmware v26, `docs/ESP32_BRIDGE_INTERFACE.md`), at 230400 baud

| Direction | |
|---|---|
| XIAO → Teensy | `ESP32-hello` (plain ASCII, no line ending) at least every 5 s. The XIAO counts as present for 10 s after each hello |
| AgOpenGPS → XIAO | while the XIAO is present: every packet that reaches the module on UDP 8888 and starts with `80 81`, unchanged (PGN 254, 239, 200, 201, 202, 235, 236, 238, 252, 251 ..., up to 512 bytes) |
| XIAO → AgOpenGPS | AgOpenGPS frames `80 81 src pgn len data... crc`, sent by the Teensy to UDP 9999 (broadcast), from port 8888 like the module's own PGNs |
| XIAO → Teensy log | text lines (printable ASCII ending in a line feed, up to 160 characters, at most 10 per 10 s) → `[xiao] > ...` in Messages |
| Teensy → XIAO | **PGN 141**, source 0x7E: the module status, the same JSON as `/api/status` without what only the web page shows (steering settings, start reason, the receiver's sentence list, the last sentences, the last HPR line, the XIAO's own info), once a second, in pieces of up to 240 bytes (`[set][piece][pieces][text]`); at most about 1.5 kB |
| Teensy → XIAO | **PGN 142**, source 0x7E: each line of the log as it is written (`[number, 4 bytes][Teensy millis, 4 bytes][text]`); after a hello that follows a silence, the last 60 lines again |
| XIAO → Teensy | **PGN 143**: the XIAO's state as a short JSON (program version, own network, other network, IP, signal), kept by the Teensy for the XIAO card; **not** sent to UDP 9999 |

- PGN 141, 142 and 143 travel only on the wire between the Teensy and the XIAO, never on the network, and are not in the AgOpenGPS PGN list.
- They go out only while Serial2's transmit buffer has 256 bytes to spare, so the AgOpenGPS packets for the XIAO always come first.
- In a 60 s PC simulation of AgOpenWeb at 100 Hz with 64 sections plus the status every second, nothing was dropped and every status arrived whole.

Differences from v26, all on the safe side:

- **The checksum is checked**: the low 8 bits of the sum of the bytes from `src` to the last data byte (the AgOpenGPS checksum, "sum of byte 2 to n-2" in the AgOpenGPS PGN list; the XOR shown in the v26 document is not it, and the document's examples match neither).
  - Frames with a wrong checksum are dropped and counted (web page: "Checksum errors").
  - The ESP32-C3 writes its boot messages at 115200 on the same pin; at 230400 they are garbage, and the checksum keeps them from ever reaching AgOpenGPS.
- Frames and text lines from the XIAO are only taken **while it is present** (hello in the last 10 s): the program on the XIAO must say hello first.
- **Never waits**: if Serial2's transmit buffer (2 KB) is full, the packet for the XIAO is dropped and counted ("Lost"); the GNSS and steering loop is never held up. At most 8 frames from the XIAO are sent per loop.
- After a lost byte the Teensy looks for the next frame inside the bytes already received, so one lost byte costs one frame, not several.
- The steering code reads only the first 24 bytes of each UDP 8888 packet; the XIAO gets the whole packet.

### Link speed: 230400

`XIAO_BAUD` in `zXiao.ino` (Teensy) and `LINK_BAUD` in the XIAO program (`xLink.h`) must be the same.

| Speed | Capacity (each way) | Load today: AgOpenWeb at 100 Hz, PGN 254 + 239 (+ 229) | Future WiFi bridge (NMEA out; PGNs and RTCM in) | XIAO UART FIFO (128 bytes) fills in |
|---|---|---|---|---|
| 115200 | 11.5 kB/s | 25-38 % | 35-60 % | 11 ms |
| **230400** | **23 kB/s** | **12-19 %** | **17-30 %** | **5.6 ms** |
| 460800 (v26) | 46 kB/s | 6-10 % | 9-15 % | 2.8 ms |

- AgOpenWeb sends PGN 254 and 239 (14 bytes each) to the modules every 10 ms (its 100 Hz control loop), plus PGN 229 (16 bytes) with more than 16 sections: 2.8-4.4 kB/s. AgOpenGPS / AgIO sends them once per GPS fix (10 Hz). The XIAO's own PGNs are a few bytes.
- 460800 works, but it is ten times what the link carries, and it is the XIAO that pays for it: the ESP32-C3 has one core, shared with the WiFi, and a 128-byte receive FIFO. With the default threshold (120 bytes) a burst leaves it 8 bytes, 0.17 ms at 460800, to empty the FIFO.
- 230400 halves the burst speed and still keeps the load under 20 % today and about 30 % for a WiFi bridge that would carry the NMEA, the PGNs and the RTCM corrections; 115200 would get close to 60 % there.
- Speed error: Teensy +0.16 %, ESP32-C3 under 0.01 % at all these speeds (tolerance about 2 %). The few centimetres of the adapter board or of the wires are fine at any of them.
- With 230400 the ESP32-C3 boot messages (115200, same pin) are garbage for the Teensy, and are dropped (checked in the PC test with the real boot text).
- A XIAO program written for v26 (460800): change its speed to 230400, or `XIAO_BAUD` to 460800.

### The program for the XIAO: `v7.6_XIAO_ESP32`

Its own WiFi network with a page for a phone: the module status and messages (sent by this firmware), what AgOpenGPS / AgOpenWeb sends to the module, WiFi settings, program update over WiFi. See its README.

Bench test without the AiO board: XIAO on USB-C (power and serial monitor), and only three wires to the Teensy: D6 → Teensy pin 7, D7 → Teensy pin 8, GND → GND (no 3V3 wire then).

## Credits and licence

This firmware is built on other people's work:

| Part | From |
|---|---|
| Structure of the main file, steering (`Autosteer.ino`, `AutosteerPID.ino`), Ethernet, NTRIP | the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project) |
| UM982 dual heading from HPR, the first `zHandlers.ino` | chriskinal, AOG_Teensy_UM982 (github.com/chriskinal/AOG_Teensy_UM982), Copyright 2024 chriskinal, GPL v3 |
| TM171, Keya CAN, the version of the AgOpenGPS files used here | Balázs Gunics, fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1) |
| Reading the TM171 (origin) | Paulius a.k.a. babtai |
| BNO085 reading, RVC on Serial5 and I2C, with their roll / pitch / heading conventions | the AgOpenGPS AiO v4 Teensy firmware (`AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025`, its `BNO_RVC.cpp` from the Adafruit BNO08x RVC library) |
| BNO080 / BNO085 I2C library (`BNO08x_AOG.*`) | Nathan Seidle, SparkFun Electronics (SparkFun_BNO080_Arduino_Library, MIT License), in the AgOpenGPS version |
| u-blox messages and configuration keys | the u-blox ZED-F9P / ZED-X20P interface descriptions (key IDs checked against the list of SparkFun's u-blox GNSS library v3); the `VERSIONA` command of the Unicore receivers |
| NMEA parser (`zNMEAParser.h`) | Glinnes Hulden (CC0) |
| ADS1115 (`zADS1115.*`) | ADS1115-Lite by Terry Myers (github.com/terryjmyers/ADS1115-Lite), from the Adafruit library |
| Flash routines for the update (`FlashTxx.*`) | FlasherX by Joe Pasquariello (github.com/joepasquariello/FlasherX), from Niels A. Moseley and Frank Boesing (public domain) |
| Protocol of the XIAO link | the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official) |
| Format of the messages on the program's screen (PGN 221) | AgOpenGPS (`UDPComm`, case 221), read the same way by AgOpenWeb |
| Crash record and reset reason used at start-up | the Teensy core (`CrashReport`, PJRC) |
| Pin comparison of the AiO v4.5 Micro and Standard | the AgOpenGPS board design files (EasyEDA, 2024-02-05) |

The rest was written in 2026 by Claude (Anthropic) for renatorgr: the web page (Portuguese / English), settings, `[status]` messages, update from the page, the XIAO link and the XIAO program, the standard / classic / automatic output, the rewritten `zHandlers.ino`, the autosteer safety and the watchdog (`zSafety.ino`), the RTCM message counter (`zRtcm.h`), the receiver identification and the u-blox settings (`zReceiver.ino`) and the IMU detection (`zImu.ino`). Every file says at the top where it comes from and what was changed.

**Licence:** GNU General Public License v3 or later (file `COPYING`), like chriskinal's AOG_Teensy_UM982 and the AgOpenGPS firmware it comes from. It comes WITHOUT ANY WARRANTY: check it on the bench before trusting it to steer a tractor. The files from other authors keep their own notes too (CC0, public domain, Adafruit, MIT for the BNO080 library).
