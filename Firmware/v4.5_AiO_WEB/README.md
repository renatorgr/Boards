# v4.5_AiO_WEB

Teensy 4.1 firmware for the **AiO v4.5 boards** (Micro and Standard), for AgOpenWeb and AgOpenGPS.

This is the **AiO_WEB** line: everything of `v4.5_UM982_X20P_F9P_TM171_WEB_XIAO`, plus what the AgOpenGPS AiO v4 firmware (27.04.2025) does and that firmware leaves out, so that one firmware covers what the board accepts.

- GPS: **UM982** (dual antenna), **u-blox F9P / X20P** (single antenna), or **dual F9P** (a second F9P on the GPS2 port, see "Dual F9P" below).
- IMU: **TM171** or **BNO085**.
- Steering: hydraulic valve or motor through a **Cytron** or **IBT2** driver, **Danfoss** valve, or **Keya** motor on the CAN bus (see "Steering" below).
- Optional: a **Seeed XIAO ESP32C3 or ESP32S3** on the GPS2 port, with its own WiFi page (the GPS2 port takes the XIAO or the second F9P, one at a time).
- A web page on the module: status, messages, settings, the GPS receiver, firmware update over the network.

Both boards wire the Teensy the same way.

**The standard sentence set needs an AgOpenWeb build that includes PR #300** (the epoch assembler, `develop` from the evening of 9 Oct 2026 onwards). On an older build these sentences show up as "not accepted". The classic output works with any AgOpenWeb and with AgOpenGPS.

## Versions

### 4.5 (19 Oct 2026)

Everything in this README is version 4.5: the firmware v4.5_UM982_X20P_F9P_TM171_WEB_XIAO (same version number) with the AiO firmware's parts it leaves out.

From v4.5_UM982_X20P_F9P_TM171_WEB_XIAO:

- **UM982** (dual antenna) or **u-blox F9P / X20P** (single antenna), told apart by what the receiver sends; the receiver is asked what it is (`VERSIONA`, `UBX-MON-VER`) and its model shows on Status, in the messages and in the GPS tab.
- **IMU: TM171 or BNO085** (RVC mode on Serial5, or I2C), found at start-up; its heading aligned to the dual heading or to the GPS course, its yaw direction learned (or set on the page), its yaw jumps compensated.
- **Output**: the standard set for AgOpenWeb (`$GNGGA`, `$GNVTG`, `$GNTHS`, `$INHPR`), classic `$PAOGI` / `$PANDA` for AgOpenGPS, or automatic by the hello each program sends (the default).
- **Web page on the module**, in Portuguese or English: status, messages, settings, the GPS tab (u-blox and UM982 settings, UM982 console), firmware update over the network or from GitHub, restart.
- **Autosteer safety**: WAS check, autosteer off at once, why it went off on the program's screen, hardware watchdog, why the Teensy started.
- **RTCM corrections** from NTRIP and from the radio counted: messages, bad checksums, types.
- **XIAO ESP32C3 / ESP32S3** on the GPS2 port, at 460800 with the protocol of the official firmware v26, with its own program and WiFi page.
- **Keya motor**: the code of the AiO firmware (27.04.2025): found by its heartbeat at start-up or later; its slip as the kickout; a motor fault turns autosteer off; **without a WAS** its position is the wheel angle, corrected by the GPS heading.
- **PWM at 122 Hz** and **`LOW_HIGH_DEGREES` 1.5**, as the AiO firmware (see "Steering").
- **Compiles in the Arduino IDE** with Teensyduino 1.57 or newer; credits at the top of every file; GNU GPL v3.

Only in AiO_WEB:

- **Dual F9P.** A second F9P on the GPS2 port (Serial2, 460800) sending UBX-NAV-RELPOSNED gives the dual-antenna heading, as in the AiO firmware (see "Dual F9P" below).
  - It goes out exactly like the UM982's: `$GNTHS` for AgOpenWeb, `$PAOGI` for AgOpenGPS.
  - Only with an RTK fixed solution between the antennas, as the AiO firmware.
- **GPS2 port setting** on the web page (Settings): XIAO ESP32, second F9P or off.
- **As the AiO firmware:** autosteer is cut when the wheel angle goes beyond ±50° or the program's speed above 25 km/h; the wheels are not turned below 0.2 km/h; a WAS map (`inputWAS` / `outputWAS`) can straighten a non-linear WAS.
- **Danfoss** valve and **IBT2** driver described (see "Steering").
- Without an IMU, the classic `$PAOGI` carries the roll measured by the two antennas (UM982 HPR, or the RELPOSNED of the second F9P), as the AiO firmware.
- Web page: the dual heading's source (UM982 or second F9P), the distance between the antennas, "Danfoss" as the motor, the new reasons autosteer went off.
- The update over the network works between v4.5_UM982_X20P_F9P_TM171_WEB_XIAO and this firmware, both ways (same firmware family).

## Compiling and first install

- Arduino IDE (1.8 or 2.x) with **Teensyduino 1.57 or newer** (checked with the cores 1.57, 1.58, 1.59, 1.60 and 1.62).
- Board: Teensy 4.1, USB type Serial, CPU 600 MHz, Optimize "Faster".
- Nothing to install besides Teensyduino: the BNO085 library and the flash routines are in the sketch folder, and the CAN library for the Keya (FlexCAN_T4) comes with Teensyduino.
- Lines marked "warning" in the IDE's output do not stop the build; only "error" does.
- The GPS and IMU ports are declared with the type `TeensySerial` (main .ino), so the same code builds with Teensyduino 1.58 and older and with 1.59 and newer.
- **Already compiled:** this folder also has `v4.5_AiO_WEB.hex`, built with Teensyduino 1.58 (the version tested on the tractor with v4.5_UM982_X20P_F9P_TM171_WEB_XIAO). The Arduino IDE ignores it, and the XIAO folders, when it compiles the sketch.
  - From a firmware that already has this web page (v4.5_UM982_X20P_F9P_TM171_WEB_XIAO and its earlier v6 / v7 builds too): web page → Firmware → choose the `.hex`.
  - From an older firmware: by USB, Teensy Loader → File → Open HEX File → press the button on the Teensy.
- The files `FlashTxx.c` / `FlashTxx.h` are FlasherX (Joe Pasquariello, public domain), used for the update over the network. Keep them in the sketch folder.

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
| | Dual heading: its source (UM982 HPR or second F9P RELPOSNED), QF, roll of the antennas, the distance between the antennas and the carrier solution (dual F9P), why there is none, the last `$GNHPR` or RELPOSNED as received. |
| | IMU: which one (TM171 or BNO085) and where, roll, pitch, yaw, alignment, yaw direction. |
| | Steering: WAS angle, requested angle, PWM, autosteer engaged, switch, motor (Cytron, IBT2, Danfoss or Keya), WAS voltage and reading (or "no WAS: angle from the Keya position"), pressure / current value or Keya slip (highest of the last 10 s, kickout limit), why autosteer last went off, the last message sent to the program's screen, the Keya (position, current, slip, faults), the wheel angle from the GPS. |
| | The steering settings from the program. |
| | Network (cable, program), RTCM corrections from the program (NTRIP) and from the radio, what the receiver sends, the last sentences sent. |
| | The XIAO: present, last hello, packets each way, last PGN from it, checksum errors, losses (or that the GPS2 port is set to the second F9P, or off). |
| **Messages** | The `[status]` lines (the module keeps the last 60). |
| **Settings** | The settings below, saved in the Teensy's EEPROM and applied at once (no restart). |
| **GPS** | The receiver: model, make, firmware, protocol, when it was identified, "Ask again". |
| | **u-blox F9P / X20P**: its settings, read and written from the page (see "GPS receiver" below). |
| | **UM982**: its settings, read and written from the page (see "UM982 settings" below). |
| | **UM982** (or not identified): a console. It sends a command to the UM982 port connected to the Teensy and shows the replies for 5 s. Quick buttons: `CONFIG`, `UNILOGLIST`, `GPHPR 0.1`, `SAVECONFIG`. Commands that can cut the link (`FRESET`, `UNLOG`, port baud) ask for confirmation. |
| **Firmware** | Version and build date, why the Teensy last started, the watchdog, the newest version on GitHub with "Install from GitHub", firmware update from a file, restart. |

The page refreshes once a second, one request at a time. The module answers between GNSS epochs and never waits for the browser for more than 3 s (0.5 s for a connection that sends nothing), so steering and GNSS keep running.

### Settings (web page, EEPROM)

| Setting | Default | |
|---|---|---|
| Operating mode (output) | automatic | **Automatic**: the standard set while AgOpenWeb sends the hello; classic for AgOpenGPS and until the first hello. **AgOpenWeb**: always the standard set (AgOpenWeb with PR #300). **AgOpenGPS**: always classic `$PAOGI` / `$PANDA`. |
| Accept a float heading (HPR QF 5) | yes | No = only QF 4 (fixed). UM982 only: the dual F9P always uses only the fixed one, as the AiO firmware. |
| Longest wait for the VTG and heading after the GGA | 60 ms | 20-100 ms (the heading: HPR or RELPOSNED). |
| Swap roll and pitch (IMU) | no | IMU turned 90° on the board. |
| Yaw direction (IMU) | automatic | Automatic (learned in turns, from the dual heading or, without one, from the GPS course), +1 or −1. Once you know it, set it: the IMU heading is then usable from start-up. |
| GPS2 port (Serial2) | XIAO ESP32 | What is on the GPS2 port: **XIAO ESP32** (the bridge below), **second F9P** (dual F9P) or **off** (the Teensy ignores the port), at 460800. One at a time. Applied at once (Serial2 restarts at the new speed). |
| Check the WAS sensor | on | Off only on a bench with no WAS: a WAS out of range then does not block autosteer. While off: a warning on Status, at boot, and on the program's screen. Switching it off asks for confirmation. |
| Watchdog | on | Off = no hardware watchdog, from the next start (once on, the chip's watchdog cannot be stopped until a restart). Switching it on works at once. |

- The defaults are the `#define`s at the top of `zHandlers.ino`; a blank or damaged EEPROM block gives the defaults.
- The settings live at EEPROM address 200, away from the AgOpenGPS steer settings (0-63), and survive firmware updates.

### Firmware update over the network

1. Web page → Firmware → choose the `.hex` → Install.
   - Only with **autosteer disengaged and the tractor stopped** (otherwise refused).
   - The steering is switched off for the update.
2. The module receives the whole file into a free part of its flash (4-6 MB) and checks it:
   - Intel HEX checksums and end record;
   - starts at the flash base and fits;
   - built for a Teensy 4.1 (`fw_teensy41`);
   - this firmware family (`AOG-UM982-TM171-WEB-OTA-1`, the same in v4.5_UM982_X20P_F9P_TM171_WEB_XIAO and in this one), so the next update can also be done from the page.
   - Every sector written is read back. **If anything fails, the running firmware stays** and the page says why.
3. Only then is the new firmware copied over the old one (a few seconds) and the Teensy restarts. The page shows the new version when it is back.

- "Other firmware" installs an image without the marker (for example a firmware without this page); the next update is then by USB again.
- **Do not cut the power during the final copy** (a few seconds after "Installing and restarting"). The Teensy would then have no firmware and must be reprogrammed by USB; the Teensy itself is not damaged, and the 15 s button restore also works.

### New versions from GitHub

The page looks for the newest version in the `Firmware` folder of **github.com/renatorgr/Boards**: when it opens, every 6 hours while it stays open, and with **Check now** (Firmware tab).

- Each version is one folder named like the firmware, `vX.Y_AiO_WEB`, with `vX.Y_AiO_WEB.hex` in it. For the XIAO: `vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` and `...S3.bin`. Other folders and files there are ignored (the `vX.Y_UM982_X20P_F9P_TM171_WEB_XIAO` folders too: each firmware looks only for its own).
- A newer version than the one running shows at the top of Status and in the Firmware tab, with a link to its folder on GitHub.
- **Install from GitHub**: the browser downloads that `.hex` and sends it to the Teensy exactly like a chosen file (same checks, same rules). Its size must match what GitHub lists.
- It is **the device the page is open on** (tablet, phone, PC) that talks to GitHub, so that device needs internet. The Teensy never goes to the internet.
- Without internet the page says "no answer from GitHub" and nothing else changes.
- GitHub answers 60 such questions an hour per internet connection without a login.

To publish a new version: upload the whole version folder, as it comes in the zip (`.hex` and XIAO folders included), into `Firmware/` on GitHub. Nothing else needs changing.

## GPS receiver: identification, u-blox and UM982 settings (`zReceiver.ino`)

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

### UM982 settings (GPS tab)

- They are read the first time the tab is opened, and with **Read from the receiver**: the commands `CONFIG`, `MASK`, `UNILOGLIST` and `MODE`, one after the other.
- **Write to the receiver** sends only what was changed, one command at a time. Each one waits for the UM982's `$command,...,response: OK`.
- Then `SAVECONFIG` (kept after a power off), then everything is read again.
- A command the UM982 refuses (an older firmware may not know `TRACTOR`, for example) is shown with the UM982's reason; the others are still written and saved.
- Only with autosteer off.
- "–": the UM982 did not give that value in its answers (older firmware lists fewer settings). Choosing a value writes it.
- The commands carry no port name, so they act on the port wired to the Teensy. That port is the one at 460800 baud in the `CONFIG` answer (if two are, the one sending GGA).
- **What the receiver answered** (at the bottom of the form) shows every line of the last read, as it came.

| Setting | UM982 command | |
|---|---|---|
| Rate of the GGA, VTG and HPR | `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1` | 1, 2, 5, 10 or 20 Hz, the three together (AgOpenGPS / AgOpenWeb: 10 Hz). If the VTG or the HPR is not at the GGA's rate, the page says so and a write sets them right |
| Working mode | `MODE ROVER UAV` / `AUTOMOTIVE` / `SURVEY` / `SURVEY MOW` | UAV is the UM982's default; the Unicore manual gives survey for precision agriculture |
| Minimum satellite elevation | `MASK 10` | 0-25° |
| Longest age of the RTK corrections | `CONFIG RTK TIMEOUT 60` | 10-600 s (UM982 default 600 s): with no corrections for longer, it leaves RTK |
| SBAS | `CONFIG SBAS DISABLE` / `ENABLE AUTO` / `ENABLE EGNOS` | useful only without RTK |
| Heading mode | `CONFIG HEADING FIXLENGTH` / `VARIABLELENGTH` / `STATIC` / `LOWDYNAMIC` / `TRACTOR` | fixed length is the default |
| Distance between the antennas | `CONFIG HEADING LENGTH 120 3` | centre to centre in cm, and the tolerance in cm; empty = automatic (`CONFIG HEADING LENGTH`). A wrong value: no heading |
| Heading and pitch correction | `CONFIG HEADING OFFSET 1.5 0` | degrees. Use this or AgOpenWeb's `DualHeadingOffset`, not both |
| Messages to the Teensy | `GPRMC 1` / `UNLOG GPRMC` | RMC, GSA, GSV, GST, ZDA, THS, KSXT, on at 1 Hz or off. GGA, VTG and HPR are always on: the Teensy needs them |
| Constellations | `MASK GLO` / `UNMASK GLO` | GPS, GLONASS, Galileo, BeiDou, QZSS. At least one of GPS, GLONASS, Galileo and BeiDou stays on. Changing them, or the working mode, can lose RTK for a few seconds |
| Shown only | `CONFIG COM1` ... `COM3`, NMEA version, `MODE` | the port baud rates are never changed here, and there is no `FRESET`: the Teensy would lose the receiver |

- Messages: one `[web] UM982 <- <command>` line per command, then `[status] GPS receiver: UM982 settings written (4 of 4 commands) and saved (SAVECONFIG)`.
- A refused command: `[status] GPS receiver: UM982 refused CONFIG HEADING TRACTOR: PARSING FAILD ...`.
- While the page reads or writes, the UM982's answers stay out of the console, and the console waits for it to finish.
- The commands are those of the Unicore N4 commands and logs reference. Tested on the PC against a simulated UM982 only: check the result on the tractor (the answers are shown on the page).

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

## Dual F9P: second F9P on the GPS2 port (`zDualF9P.ino`)

Two ZED-F9P, as with the AgOpenGPS AiO v4 firmware:

- **The first F9P** (GPS1, Serial7, 460800): sends GGA + VTG at 10 Hz to the Teensy, like a single F9P. It is the moving base: it sends its corrections (RTCM 4072 and the MSM messages) to the second F9P over their UART2.
- **The second F9P** (GPS2): takes those corrections and sends **UBX-NAV-RELPOSNED** on its UART1 at **460800** to the Teensy (Serial2, pins 7 / 8), every epoch.
- Use the AgOpenGPS dual F9P configuration files for the two receivers (position / moving base, and heading / rover), with UART1 at 460800.
- On the web page: Settings → **GPS2 port: second F9P**. The XIAO cannot be on that port at the same time.

What the firmware does with each RELPOSNED:

- **Heading** = its `relPosHeading` + **90°** (antennas left and right, the AiO firmware's correction; `DUALF9P_HEADING_OFFSET`). Other small offsets: `DualHeadingOffset` in AgOpenWeb or "Dual heading offset" in AgOpenGPS.
- **Used only with an RTK fixed solution between the antennas** (`carrSoln` fixed) and the flags `gnssFixOK`, `diffSoln` and `relPosValid` set. A float solution is never used, as the AiO firmware (the "float heading" setting is for the UM982).
- **Roll** of the antennas = asin(down / distance between the antennas), as the AiO firmware. It goes into `$PAOGI` when there is no IMU.
- **Time**: its GPS time of week (iTOW) minus 18 leap seconds (`DUALF9P_LEAP_S`) must be the GGA's time, like the UM982's HPR.
- From there everything is as with the UM982: `$GNTHS` in the standard set, `$PAOGI` in classic mode, the IMU aligned to it, the same `[status]` messages (with "RELPOSNED" and "the second F9P" in them).

In AgOpenWeb: **"Dual GPS" on**. In AgOpenGPS: GPS heading source **"Dual"** (the firmware sends `$PAOGI`).

On the web page (Status → dual heading): source "second F9P (RELPOSNED)", QF (4 fixed, 5 float, 0 none), roll of the antennas, distance between the antennas with the carrier solution (it should stay close to the real distance, centre to centre), and the last RELPOSNED decoded.

Messages:

- `[status] Second F9P (GPS2 port): RELPOSNED arriving, baseline 118.4 cm` / `no RELPOSNED for 5 s (... bytes, ... bad UBX frames so far) - check its UART1 output: UBX-NAV-RELPOSNED at 460800`.
- `[status] Dual: dual-antenna heading OK (second F9P, RELPOSNED, RTK fixed), sent as $GNTHS`.
- `... - the second F9P has no RTK fixed solution between the antennas (carrSoln 1, float): antennas, cables, the RTCM from the first F9P`.
- `... - RELPOSNED arrives but its time (...) does not match the GGA time (...)`, `the second F9P stopped sending RELPOSNED`.

Tested on the PC with simulated RELPOSNED frames only: check the heading and the roll on the tractor before steering with it.

## Steering: valve, motor driver or Keya (`Autosteer.ino`, `AutosteerPID.ino`, `KeyaCANBUS.ino`)

The output follows the steer settings sent by the program (AgOpenWeb / AgOpenGPS, "Motor driver" and the other settings):

| Driver | |
|---|---|
| **Cytron** | PWM + direction; the relay output switches the hydraulic valve (or the motor's clutch) |
| **IBT2** | PWM on two pins, with the enable pin |
| **Danfoss** valve | PWM around 50 % (25 % full left, 75 % full right), with "Danfoss" ticked in the program |
| **Keya** motor | on the CAN bus (CAN3, Teensy pins 30 / 31, 250 kbit/s), found by itself |

**Keya** (code of the AgOpenGPS AiO v4 firmware, 27.04.2025):

- Found by its heartbeat (CAN id `0x07000001`, every 20 ms): the firmware listens for 2 s at start-up, and a Keya that starts later is taken too (`[status] Keya motor found on the CAN bus (heartbeat): steering by the Keya`).
- Its speed is the PWM × 3.9; it is enabled while steering and disabled when autosteer goes off.
- **Kickout:** with "current sensor" on in the program, the Keya's slip (how much slower it turns than it was told, when the driver holds the wheel) is the kickout value, against the program's limit. Autosteer off: `Keya kickout (86, limit 80)`.
- **Faults:** a fault bit in the heartbeat (over voltage, under voltage, over current, motor stall, hall failure, ...) turns autosteer off: `[status] Keya motor fault: over voltage - autosteer off`, and `Keya motor fault (0x0002)` on the program's screen.
- **Without a WAS:** set the WAS to "differential" in the program (single input off). The wheel angle is then the Keya's position / counts per degree (+ the WAS offset), and the ADS1115 and the WAS check are not used. While driving above 1.2 km/h the firmware corrects it slowly with the wheel angle the GPS heading gives: atan(turn rate × wheelbase / speed), from the dual heading or the aligned IMU heading, reversed when driving backwards. Set your tractor's wheelbase in `WHEELBASE_M` (main .ino, 3.20 m).
- Web page: position, current, slip and faults of the Keya; the wheel angle from the GPS ("only above 1 km/h, with heading" until it is known).

**As the AiO firmware, for every driver:**

- Autosteer is cut when the wheel angle goes beyond **±50°** (the ends of the WAS map: a wrong calibration, a slipped sensor) or the program's speed goes above **25 km/h**. With the WAS voltage out of range, the WAS check decides instead (see "Autosteer safety"), so a glitch or the bench setting is handled as before.
- Below **0.2 km/h** autosteer can stay engaged, but the wheels are not turned (no PWM).
- **WAS map** (`inputWAS` / `outputWAS` at the top of `Autosteer.ino`, 21 points from −50° to +50°): the angle read is moved from the input points to the output points, in between in straight lines. As delivered both are the same: no change. Use it only for a WAS that is not linear.

At the top of `Autosteer.ino`, as the AiO firmware:

- `PWM_Frequency` 1 = **122 Hz** (0 = 490 Hz, 2 = 3921 Hz). On a proportional hydraulic valve 122 Hz acts as dither: the spool does not stick and small corrections get through. A DC motor runs smoother at 490 Hz.
- `LOW_HIGH_DEGREES` **1.5**: below this steering error the highest PWM drops in a straight line from the program's "maximum PWM" to its "low PWM" at 0°. Full PWM until 1.5° from the requested angle: faster corrections; if the steering snakes, lower Kp or the maximum PWM in the program, or go back to 3.0.

## Autosteer safety (`zSafety.ino`)

Everything here acts only on autosteer, that is on the module's own outputs (the valve PWM and relay, the Danfoss valve, or the Keya motor), exactly as the steer switch does. The tractor's own steering, the steering wheel and its orbitrol, is never touched and keeps working.

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

When the steer switch goes off, the steer button is pressed, a pressure / current / encoder / Keya kickout trips, the WAS check blocks, a Keya fault comes, or the wheel angle or the speed is beyond its limit, autosteer stops at the next steering loop (25 ms), without waiting for the program's next steering packet.

### Why autosteer went off

| Cause | Program's screen (PGN 221) | Messages | Web page |
|---|---|---|---|
| steer switch / steer button | `Autosteer off: steer switch` (3 s) | `[status] Autosteer OFF: steer switch` | yes |
| pressure / current kickout | `Autosteer off: pressure kickout (86, limit 80)` (6 s, warning colour) | yes | yes |
| steering wheel encoder | `Autosteer off: wheel encoder kickout (5 pulses)` | yes | yes |
| Keya slip | `Autosteer off: Keya kickout (86, limit 80)` | yes | yes |
| Keya motor fault | `Autosteer off: Keya motor fault (0x0002)` | yes, with the fault's name | yes |
| wheel angle beyond ±50° | `Autosteer off: wheel angle 63 deg, beyond +-50` | yes | yes |
| program's speed above 25 km/h | `Autosteer off: speed 27 km/h, above 25` | yes | yes |
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
  - **Dual F9P:** GGA + VTG from the first F9P, and the RELPOSNED of the second F9P on the GPS2 port, taken like an HPR (see "Dual F9P").
  - GGA is required (it marks each epoch).
  - A VTG or HPR counts as "sent by the receiver" if one arrived in the last 3 s (`RECEIVER_SEEN_MS`); for the first 3 s after start-up both are waited for.
- **IMU:** TM171 (Serial5, 115200), or BNO085 in RVC mode (Serial5, 115200) or on I2C (pins 18 / 19): roll, pitch and yaw (see "IMU").
- **GPS2 port** (Serial2, pins 7 / 8, 460800): the XIAO or the second F9P of a dual F9P, as set on the web page.
- **Keya motor** (CAN3): position, speed, current and faults, 50 times a second.
- **Correction radio** (Serial3 RX, 115200; XBee LR on the Micro, the 3-pin radio header on the Standard): RTCM corrections, passed on to the receiver as they arrive.
  - They are counted (`zRtcm.h`): bytes per second, whole RTCM 3 messages with a good checksum (CRC-24Q), messages with a bad checksum (a weak radio signal, a wrong baud rate), the message types of the last 10 s.
  - The NTRIP corrections from the program (UDP 2233) are counted the same way.

## Output (UDP 9999), standard set: 4 lines per epoch, one per datagram

| Line | Content | AgOpenWeb uses it for |
|---|---|---|
| `$GNGGA` | the receiver's GGA (a UM982 GGA with `00` satellites / HDOP `9999.0` gets the last good values, for up to 2 s) | position, fix, satellites, HDOP, correction age |
| `$GNVTG` | speed and track from the receiver's VTG; if the VTG is empty, computed from the GGA positions | speed |
| `$GNTHS` | dual-antenna heading (the UM982's HPR value, character for character, or the second F9P's RELPOSNED heading + 90°); mode `A` = valid, `V` = no dual heading | heading with two antennas |
| `$INHPR` | the IMU (talker `IN` = inertial sensor): heading aligned to the dual heading, **roll in the pitch field** (HPR convention), pitch in the roll field; QF 4 = IMU OK, QF 0 = IMU lost | roll, always; heading without two antennas |

Without the dual heading, a re-alignment of the `$INHPR` heading by the GPS course makes no step: AgOpenWeb aligns the IMU heading itself.

Result in AgOpenWeb (checked against AgOpenWeb's own code):

| Situation | Heading | Roll |
|---|---|---|
| Two antennas (HPR QF 4 fixed, or 5 float; dual F9P: RTK fixed) | `$GNTHS` (antennas) | IMU |
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

**UM982 or dual F9P (has sent HPR / RELPOSNED since start-up): always `$PAOGI`. AgOpenGPS: GPS heading source "Dual", Dual heading offset 0.**

| Situation | Heading | Roll / pitch |
|---|---|---|
| Dual heading (HPR QF 4, or 5 if accepted; dual F9P: RTK fixed), confirmed 5 epochs in a row | the UM982 HPR heading, character for character (dual F9P: the RELPOSNED heading + 90°) | IMU, degrees; with no IMU, the roll of the antennas |
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

- In UPrecise, by commands, or on the module's web page (GPS tab, "UM982 settings"), on the port connected to the Teensy, at 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, then save (`SAVECONFIG`).
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
- Elsewhere:
  - `WHEELBASE_M` (main .ino, 3.20 m): the tractor's wheelbase, for the wheel angle from the GPS (Keya without a WAS).
  - `DUALF9P_HEADING_OFFSET` (90°), `DUALF9P_LEAP_S` (18 s), `DUALF9P_BAUD` (460800): dual F9P (`zDualF9P.ino`).
  - `inputWAS` / `outputWAS`, `PWM_Frequency`, `LOW_HIGH_DEGREES`: top of `Autosteer.ino` (see "Steering").
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
- GPS receiver: `u-blox ZED-F9P, firmware HPG 1.32 (protocol 27.31)` / `Unicore UM982, firmware R4.10Build11826` / not identified; u-blox settings written / NOT written; UM982 settings written and saved / a command refused / NOT written; no answer to CONFIG.
- Dual: dual-antenna heading OK (QF 4 fixed / 5 float, or second F9P RTK fixed; sent as `$GNTHS`, or `$PAOGI` in classic mode) / no dual heading (classic: IMU heading or GPS course in `$PAOGI`, and why).
- Second F9P (GPS2 port): RELPOSNED arriving (with the distance between the antennas) / none for 5 s.
- Keya motor found on the CAN bus / a motor fault (with its name) - autosteer off / no fault any more.
- Classic with F9P / X20P: `Single antenna: ... sending $PANDA with the TM171 heading - set the AgOpenGPS heading source to Fix`.
- IMU (TM171 / BNO085): data OK / no data; yaw direction learned (from the dual heading or the GPS course); heading aligned to the GPS course; yaw jump compensated; re-aligned to the course; found later on Serial5; I2C BNO085 silent and started again.
- No dual heading: **why**, followed by the last HPR line exactly as the receiver sent it (`[status]   last HPR from the receiver: $GNHPR,...`). Printed when the dual heading is lost, and again when the reason changes:
  - `the UM982 sends HPR with QF 0` — the UM982 itself has no heading from its two antennas (antennas, cables, UM982 heading setup); the firmware can only pass that on;
  - `the HPR arrives too late, N ms after its GGA` — the HPR comes after the firmware has sent the epoch (it waits `EPOCH_WAIT_MS`);
  - `HPRs arrive but their time (...) does not match the GGA time (...)`;
  - `the receiver stopped sending HPR (last one N ms ago)`;
  - `HPR QF n but its heading field is empty or out of range`.
- Single antenna: the receiver sends no HPR (F9P / X20P, or HPR off on the UM982). Followed by `but HPR lines are arriving and none is accepted` (a firmware / format problem: send that line) or by when the last HPR line arrived. With a dual F9P whose second F9P stopped: `Dual: the second F9P stopped sending RELPOSNED ...` instead.
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
| `v4.5_AiO_WEB.ino` | setup and loop |
| `zVersion.h` | the version number (`FW_VERSION`) and name (`FW_NAME`), shown on the web page; the folder and the main .ino carry the same name |
| `zHandlers.ino` | receiver sentences, epoch assembly, standard set / classic output, UM982 reply capture |
| `zReceiver.ino`, `zReceiver.h` | which receiver (VERSIONA / UBX-MON-VER), UBX frames out of the NMEA stream, the u-blox and UM982 settings of the GPS tab |
| `zDualF9P.ino` | the GPS2 port's speed, and the second F9P of a dual F9P (UBX-NAV-RELPOSNED) |
| `KeyaCANBUS.ino` | the Keya motor on the CAN bus, from the AgOpenGPS AiO v4 firmware |
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
| `Autosteer.ino`, `AutosteerPID.ino`, `zEthernet.ino`, `zUdpNtrip.ino`, `zADS1115.*`, `zNMEAParser.h` | from the AgOpenGPS / Gunics firmware, with changes (each file says which at the top) |
| `COPYING` | the licence: GNU GPL v3 |
| `v4.5_AiO_WEB.hex` | the firmware already compiled (Teensyduino 1.58) |
| `v4.5_XIAO_ESP32/` | the program for the XIAO (its own sketch, compiled for the XIAO, not the Teensy) |
| `v4.5_XIAO_ESP32_bin/` | the XIAO program already compiled, for the XIAO ESP32C3 and ESP32S3 |

## XIAO ESP32C3 / ESP32S3

### Hardware

- XIAO ESP32C3 or XIAO ESP32S3: same pins and size.
- **AiO v4.5 Micro:** in the GPS2 socket ("Dual-Left", U11), through the adapter board (`AiO_Micro45_XIAO_C3`) or the 3D-printed holder.
  - Socket pin 1 = 3V3.
  - Pin 2 = Teensy pin 7 (RX2) ← XIAO D6 (TX; GPIO21 on the C3, GPIO43 on the S3).
  - Pin 3 = Teensy pin 8 (TX2) → XIAO D7 (RX; GPIO20 / GPIO44).
  - Pins 10 / 11 = GND.
- **AiO v4.5 Standard:** the adapter and the holder do not fit. Wire the XIAO to header P8 (pin 1 = RX2 ← XIAO D6, pin 2 = TX2 → XIAO D7), with 3.3 V and GND from another header.
- The GPS2 port is then the XIAO's: web page → Settings → GPS2 port: XIAO ESP32 (the default). A dual F9P needs that port, so it is the XIAO or the second F9P.
- The XIAO is powered from the board's 3.3 V: **a USB cable to it must not carry 5 V (VBUS)** while it is on the board (a panel USB port: data and GND only, see the XIAO program's README).
- Program it off the board, through such a data-only port, or over WiFi with its own page. The Teensy cannot program the XIAO: its BOOT and RESET pins are not wired.
- The Hammond box is aluminium: for WiFi or Bluetooth use an external antenna on the XIAO's U.FL connector.

### Protocol (as the official firmware v26, `docs/ESP32_BRIDGE_INTERFACE.md`), at 460800 baud

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
  - The ESP32-C3 writes its boot messages at 115200 on the same pin; at 460800 they are garbage, and the checksum keeps them from ever reaching AgOpenGPS.
- Frames and text lines from the XIAO are only taken **while it is present** (hello in the last 10 s): the program on the XIAO must say hello first.
- **Never waits**: if Serial2's transmit buffer (2 KB) is full, the packet for the XIAO is dropped and counted ("Lost"); the GNSS and steering loop is never held up. At most 8 frames from the XIAO are sent per loop.
- After a lost byte the Teensy looks for the next frame inside the bytes already received, so one lost byte costs one frame, not several.
- The steering code reads only the first 24 bytes of each UDP 8888 packet; the XIAO gets the whole packet.

### Link speed: 460800

`XIAO_BAUD` in `zXiao.h` (Teensy) and `LINK_BAUD` in the XIAO program (`xLink.h`) must be the same: 460800, as the official firmware v26 and the second F9P that can take the same port. A XIAO program of an earlier build (230400) does not talk to this Teensy: install the XIAO program 4.5 too.

| Speed | Capacity (each way) | Load today: AgOpenWeb at 100 Hz, PGN 254 + 239 (+ 229) | Future WiFi bridge (NMEA out; PGNs and RTCM in) | XIAO UART FIFO (128 bytes) fills in |
|---|---|---|---|---|
| 230400 | 23 kB/s | 12-19 % | 17-30 % | 5.6 ms |
| **460800** | **46 kB/s** | **6-10 %** | **9-15 %** | **2.8 ms** |

- AgOpenWeb sends PGN 254 and 239 (14 bytes each) to the modules every 10 ms (its 100 Hz control loop), plus PGN 229 (16 bytes) with more than 16 sections: 2.8-4.4 kB/s. AgOpenGPS / AgIO sends them once per GPS fix (10 Hz). The XIAO's own PGNs are a few bytes.
- The XIAO pays for the speed: the ESP32-C3 has one core, shared with the WiFi, and a 128-byte receive FIFO. Its program empties the FIFO when 64 bytes are in it (`setRxFIFOFull(64)`, not the default 120), so a burst leaves it 64 bytes, 1.4 ms at 460800, and its receive buffer is 4 KB.
- Speed error: Teensy +0.16 %, ESP32-C3 under 0.01 % (tolerance about 2 %). The few centimetres of the adapter board or of the wires are fine at this speed.
- The ESP32-C3 boot messages (115200, same pin) are garbage for the Teensy at 460800, and are dropped (checked in the PC test with the real boot text).
- A XIAO program written for v26 works at this speed as it is (with the AgOpenGPS checksum, see above).

### The program for the XIAO: `v4.5_XIAO_ESP32`

Its own WiFi network with a page for a phone: the module status and messages (sent by this firmware), what AgOpenGPS / AgOpenWeb sends to the module, WiFi settings, program update over WiFi. See its README.

Bench test without the AiO board: XIAO on USB-C (power and serial monitor), and only three wires to the Teensy: D6 → Teensy pin 7, D7 → Teensy pin 8, GND → GND (no 3V3 wire then).

## Credits and licence

This firmware is built on other people's work:

| Part | From |
|---|---|
| Structure of the main file, steering (`Autosteer.ino`, `AutosteerPID.ino`), Ethernet, NTRIP | the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project) |
| Keya CAN (`KeyaCANBUS.ino`), dual F9P RELPOSNED (heading + 90°, roll, the flags), wheel angle from the GPS heading for the Keya without a WAS, the cut beyond ±50° or above 25 km/h, no steering below 0.2 km/h, the WAS map | the AgOpenGPS AiO v4 Teensy firmware `AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025` (AgOpenGPS project, GPL v3) |
| UM982 dual heading from HPR, the first `zHandlers.ino` | chriskinal, AOG_Teensy_UM982 (github.com/chriskinal/AOG_Teensy_UM982), Copyright 2024 chriskinal, GPL v3 |
| TM171, Danfoss and IBT2 output, the version of the AgOpenGPS files used here | Balázs Gunics, fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1) |
| Reading the TM171 (origin) | Paulius a.k.a. babtai |
| BNO085 reading, RVC on Serial5 and I2C, with their roll / pitch / heading conventions | the AgOpenGPS AiO v4 Teensy firmware (`AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025`, its `BNO_RVC.cpp` from the Adafruit BNO08x RVC library) |
| BNO080 / BNO085 I2C library (`BNO08x_AOG.*`) | Nathan Seidle, SparkFun Electronics (SparkFun_BNO080_Arduino_Library, MIT License), in the AgOpenGPS version |
| u-blox messages and configuration keys, UBX-NAV-RELPOSNED | the u-blox ZED-F9P / ZED-X20P interface descriptions (key IDs checked against the list of SparkFun's u-blox GNSS library v3) |
| UM982 commands (`VERSIONA`, `CONFIG`, `MASK`, `UNMASK`, `UNILOGLIST`, `MODE`, `UNLOG`, `SAVECONFIG`, the log commands) | the Unicore N4 high precision commands and logs reference |
| NMEA parser (`zNMEAParser.h`) | Glinnes Hulden (CC0) |
| ADS1115 (`zADS1115.*`) | ADS1115-Lite by Terry Myers (github.com/terryjmyers/ADS1115-Lite), from the Adafruit library |
| Flash routines for the update (`FlashTxx.*`) | FlasherX by Joe Pasquariello (github.com/joepasquariello/FlasherX), from Niels A. Moseley and Frank Boesing (public domain) |
| Protocol of the XIAO link | the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official) |
| Format of the messages on the program's screen (PGN 221) | AgOpenGPS (`UDPComm`, case 221), read the same way by AgOpenWeb |
| Crash record and reset reason used at start-up | the Teensy core (`CrashReport`, PJRC) |
| Pin comparison of the AiO v4.5 Micro and Standard | the AgOpenGPS board design files (EasyEDA, 2024-02-05) |

The rest was written in 2026 by Claude (Anthropic) for renatorgr: the web page (Portuguese / English), settings, `[status]` messages, update from the page, the XIAO link and the XIAO program, the standard / classic / automatic output, the rewritten `zHandlers.ino`, the autosteer safety and the watchdog (`zSafety.ino`), the RTCM message counter (`zRtcm.h`), the receiver identification and the u-blox and UM982 settings (`zReceiver.ino`), the IMU detection (`zImu.ino`), and in this line the GPS2 port setting and the second F9P's reading (`zDualF9P.ino`) and the Keya and steering changes around the AiO firmware's code. Every file says at the top where it comes from and what was changed.

**Licence:** GNU General Public License v3 or later (file `COPYING`), like chriskinal's AOG_Teensy_UM982 and the AgOpenGPS firmware it comes from. It comes WITHOUT ANY WARRANTY: check it on the bench before trusting it to steer a tractor. The files from other authors keep their own notes too (CC0, public domain, Adafruit, MIT for the BNO080 library).
