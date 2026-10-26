# v4.5_AiO_WEB

Teensy 4.1 firmware for the **AiO v4.5 boards** (Micro and Standard; both wire the Teensy the same way), for AgOpenWeb and AgOpenGPS. It is the official AgOpenGPS AiO v4 Teensy firmware (`AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025`) with the additions below.

> **Not tried on a tractor yet:** the dual F9P, the Keya motor, steering with a BNO085. Check them on the bench first.

## What it adds to the official AiO v4 firmware

| | Official AiO v4 | v4.5_AiO_WEB |
|---|---|---|
| **Web page on the module** | none | `http://192.168.5.126/`: status, messages, settings, GPS, IMU, Machine and Firmware tabs; refreshes once a second, never holds the Teensy up. |
| **Firmware update** | over USB | from the page: a `.hex` file, or straight from GitHub. The running firmware stays if anything fails. |
| **GPS receiver** | set up with AOGConfigOMatic on a PC | the Teensy finds the receiver's baud rate, identifies it (F9P, X20P, UM982, a second F9P), checks what it needs and loads the AgOpenGPS settings itself; u-blox and UM982 settings and a UM982 console on the page. |
| **IMU** | BNO085 | TM171 or BNO085 (RVC or I2C), found at start-up. TM171: gyro and new-install calibration, set-up check with one-click fix, and its settings, all from the page. |
| **Output to the program** | `$PAOGI` / `$PANDA` | the same, plus the standard sentence set AgOpenWeb reads (`GGA`, `VTG`, `THS`, `HPR`), chosen automatically from the program's hello. |
| **Autosteer safety** | kickouts | WAS voltage check (broken wire = autosteer blocked), Keya heartbeat lost = autosteer off, outputs off at once on a crash or a watchdog restart, and the reason autosteer went off on the program's screen. |
| **Steering settings** | fixed in the code | on the page: minimum and maximum speed, soft start, current kickout delay. The wheelbase comes from AgOpenWeb. |
| **Diagnostics** | serial prints | `[boot]` and `[status]` messages on the page, a diagnostic report file, why the Teensy last started (with the crash address), the longest loop time. |
| **Network** | NativeEthernet as it comes | the network stack runs from the loop (a browser could crash the Teensy); 8 KB buffers. |
| **XIAO ESP32 on the GPS2 port** | none | its own WiFi network and page: module status, messages, settings, update over WiFi. |
| **LEDs** | link, heading source | the AiO 26 meanings: program talking, fix quality with an RTCM flick, steering state and faults. The in-between state alternates red and green, readable through a panel window. |
| **Machine module** | none | a Machine tab that shows the machine module on UDP 9999: the official Machine_UDP_v5 or the sprayer Nano, its sections, and the sprayer's pressure and flow. |

The board accepts: **GPS** UM982 (dual antenna), u-blox F9P / X20P (single antenna) or a second F9P on the GPS2 port; **IMU** TM171 or BNO085; **steering** hydraulic valve or motor through a Cytron or IBT2 driver, Danfoss valve, or Keya motor on CAN; **optional** a Seeed XIAO ESP32C3 / ESP32S3 on the GPS2 port (the port takes the XIAO or the second F9P, one at a time).

## Changelog

| Date | Change |
|---|---|
| 26 Oct 2026 | Web page: the buttons the module refuses while autosteer is on or the tractor is moving (receiver settings, TM171 calibration and setup, firmware install) are greyed out then, with a line at the top of the tab saying why. Machine tab: the Machine module card fits a phone screen (the Hello replies row wraps; the note about the official module's missing checksum sits under the list). GPS2 port automatic: the Teensy finds a XIAO (its hello) or a second F9P (its UBX or NMEA data, or a search for its baud rate at start-up) by itself, at every start; the setting is now automatic or off. |
| 25 Oct 2026 | Machine tab: the official AgOpenGPS Machine_UDP_v5 is recognised too (its hello reply has no checksum, its PGN 237 carries the program's source byte); the sections come from the module's hello reply, up to 16; the Sprayer card only once PGN 160 arrives. LEDs: the in-between (orange) state is shown as red and green in turn, because orange looks like red through a diffused panel window. |
| 24 Oct 2026 | LEDs with the AiO 26 meanings. TM171 settings: output rate from a list (25, 50, 100 Hz), the Setting layout as a real TM171 sends it, commands addressed to the TM171's own ID (writes sent "to everyone" were ignored), frames in the log. ZED-X20P settings file for the receiver check. Receiver set up in one go at the baud rate it is found at. The module's subnet changed from the XIAO page. The Firmware card shows the build inside a chosen `.hex`. |
| 23 Oct 2026 | IMU tab: TM171 gyro calibration, new-install calibration, set-up check with **Fix TM171 setup**, settings read and written (needs the TX5 wire). Second F9P identified, checked and set up automatically; both receivers set up automatically. |
| 22 Oct 2026 | Update marker renamed `AiO-WEB-OTA-1` (the old name is still accepted). Machine tab: sections as coloured squares. |
| 21 Oct 2026 | Compiled on GitHub for every change. Machine tab (sprayer PGN 160). Wheelbase from AgOpenWeb (PGN 247). |
| 20 Oct 2026 | **Version 4.5.** From the AiO v4 firmware of 27.04.2025: dual F9P, the Keya code, the cuts beyond ±50° and 25 km/h, the WAS map, Danfoss and IBT2. From the AiO 26 firmware: Keya heartbeat timeout, soft start, steering speeds and current kickout delay as settings. The network-stack crash fixed, outputs off on a crash, Teensyduino 1.58 start-up fix. Receiver check with the AgOpenGPS settings. Pages in English with a light mode, simulator mode, diagnostic report, cleaner messages. |

Older changes are in the git history and at the top of each file.

## Compiling and first install

- Arduino IDE 1.8 or 2.x with Teensyduino:
  - **1.58 recommended**: the `.hex` in this folder is built with it.
  - 1.57 also builds (older compiler).
  - 1.59 to 1.61 use the same compiler as 1.58 (1.59 and 1.60 checked: they build), but have not been tried on a tractor.
  - 1.62 brings a new compiler (gcc 15) and has not been checked.
- Board: Teensy 4.1, USB type Serial, CPU 600 MHz, Optimize "Faster".
- Nothing else to install: the BNO085 library and the flash routines (`FlashTxx.c` / `.h`) are in the sketch folder, and FlexCAN_T4 comes with Teensyduino.
- Warnings do not stop the build (the ones in NativeEthernet, FNET and mbedtls come with Teensyduino and are normal); only an "error" does.
- `arm-none-eabi-g++ ... not found`: that Teensyduino's compiler did not install. Remove Teensy in the Boards Manager and install it again.

**Compiled on GitHub too:** every pull request and every change on `main` that touches the firmware is compiled by GitHub (Actions, "Teensy firmware") with Teensyduino 1.58 and the same board settings. A green tick on the pull request means it compiles; a red cross shows the error. The `.hex` (named `v4.5_AiO_WEB.hex`) is under **Artifacts** on the run's page, for 30 days. It is a check, not a test: the bench test is still needed.

**Already compiled:** `v4.5_AiO_WEB.hex` (Teensyduino 1.58). The IDE ignores it and the XIAO folders.

- From a firmware that already has this web page: web page → Firmware → choose the `.hex`.
- From any other firmware: over USB, Teensy Loader → File → Open HEX File → press the button on the Teensy.

## Web page

Open `http://<module IP>/`, normally `http://192.168.5.126/` (the third number follows the subnet set in AgOpenWeb). There is no password: anyone on the module's network can open it.

**Another network** (not 192.168.5.x): the module keeps its subnet in the EEPROM. AgOpenWeb / AgOpenGPS change it when their subnet is set (PGN 201, the module restarts). It can also be changed by hand on the **XIAO's WiFi page** (WiFi tab → Module network), which is on WiFi whatever the module's IP is and shows the module's IP; type the first three numbers and the module restarts at the new IP. The module's own page only shows its IP (in the header): if you can open it you are on the right network already, and a wrong subnet typed there would leave the page unreachable.

Dark or light: the ☀ / ☾ button at the top (remembered by the browser), otherwise the device's setting.

**Without a module:** the [simulator](https://renatorgr.github.io/Boards/) shows this page and the XIAO's with a pretend module behind them (any GPS, IMU, steering and program; tractor driving or parked), rebuilt from the code on every change. See [`Firmware/simulator`](../simulator).

| Tab | |
|---|---|
| **Status** | Warnings at the top (WAS blocking autosteer, WAS check off, simulator mode on, no ADS1115, restarted after a fault, receiver at the wrong baud rate). GNSS, dual heading (not with one antenna), IMU, steering, the steering settings from the program, network and corrections, the XIAO. |
| **Messages** | The last 60 `[status]` lines. |
| **Settings** | The settings below. |
| **IMU** | The IMU found; with a TM171: gyro calibration, new install calibration, set-up check with "Fix TM171 setup", and the TM171 settings (requires the TX5 wire). |
| **GPS** | The receiver (model, firmware), the receiver check, the u-blox or UM982 settings, and a UM982 console (a command and its replies for 5 s; `FRESET`, `UNLOG` and baud rate changes ask first). |
| **Machine** | Only once a machine module (IP .123: the official Machine_UDP_v5, the sprayer Nano, or anything that talks like them) has been heard on UDP 9999. Its IP, last packet, checksum errors, hello replies, and one square per section (up to 16) in the AgOpenWeb section colours: green open, red closed, orange asked on by the program (PGN 254) but still closed on the module, cyan asked off but still open. The module's side comes from its hello reply, or from the sprayer PGN 160. The official module reports its output levels, so with "Invert relay" on in the program its squares read reversed. The Sprayer card (pressure, flow, total litres, main switch) appears once PGN 160 arrives. Only to look at. |
| **Firmware** | Firmware name and build date, why the Teensy last started, the watchdog, new versions on GitHub, update from a file, restart. **Diagnostic report**: one `.json` file with everything the module can tell (status, messages, settings, GPS, UM982 console, build, free RAM, longest loop time, web errors); send it when something goes wrong. |

While autosteer is on, the buttons that change the receiver or the TM171, and the firmware install, are greyed out; the TM171 calibrations and the firmware install also while the tractor is moving (not on a bench: simulator mode on, or no WAS).

The page refreshes once a second. When the Teensy restarts, the page reloads itself on the same tab. The module never waits for a browser for more than 3 s, so steering and GNSS keep running.

### Settings

Saved in the Teensy's EEPROM (address 200, away from the AgOpenGPS steer settings), applied at once, kept through firmware updates. The defaults are the `#define`s at the top of `zHandlers.ino` (the Steering card's: at the top of `Autosteer.ino`).

| Setting | Default | |
|---|---|---|
| Operating mode | automatic | **Automatic**: the standard set while AgOpenWeb sends its hello, classic otherwise. **AgOpenWeb**: always the standard set. **AgOpenGPS**: always classic (`$PAOGI` / `$PANDA`). |
| Simulator (Operating mode card) | off | **For the program's simulator only**: GPS data is not sent to the program (AgOpenWeb greys its simulator out while it gets GPS sentences). Steering still works. Switch it off before going to the field. Asks first. |
| Accept a float heading (HPR QF 5) | yes | No = only QF 4 (fixed). UM982 only; the dual F9P always needs RTK fixed. |
| Maximum wait for the VTG and heading after the GGA | 60 ms | 20-100 ms. |
| Swap roll and pitch (IMU) | no | IMU turned 90° on the board. |
| Yaw direction (IMU) | automatic | Learned in turns, or set to +1 / −1. Once you know it, set it: the IMU heading is then usable from start-up. |
| GPS2 port | automatic | automatic or off; 460800 baud. Automatic: a XIAO ESP32 (its hello) or a second F9P (its UBX or NMEA data) is found at every start, only with autosteer off; Settings shows what was found. A "second F9P" saved by an earlier build loads as automatic. |
| Check the WAS sensor | on | Off only on a bench without a WAS (see "WAS check"). Asks first. |
| Watchdog | on | Off from the next start. |
| UM982 antennas | two | Set on the GPS tab. One: no dual heading card, `ConfigUM982S.txt`; two: `ConfigUM982D.txt`. |
| Minimum speed (Steering) | 0.2 km/h | 0.1-3 km/h. Below it (the program's speed) the wheels are not turned. |
| Maximum speed (Steering) | 25 km/h | 10-40 km/h. Above it autosteer is cut. |
| Soft start (Steering) | off | 0.2-1 s. When autosteer engages, the valve / motor output rises from 0 over this time (never below the program's minimum PWM); on a big step or a change of direction it rises again over half of it, as in the AiO 26 firmware. Try 0.5 s. |
| Current kickout delay (Steering) | 0.5 s | 0 (at once, as before), 0.25, 0.5 or 1 s. Current sensor only, not the Keya: the current must stay above the limit this long. |

### Firmware update over the network

Web page → Firmware → choose the `.hex` → Install.

- Once chosen, the page reads the file in the browser and shows what is inside, next to its name: the version and the build date and time (the `[boot] … built …` line the firmware prints at start-up), "the firmware running now" when it is the same build as the module's, a warning when it is not a Teensy 4.1 firmware, not an AiO_WEB one, or has no update page inside. Nothing is sent until Install.
- Only with **autosteer off and the tractor stopped** (program's speed, and the receiver's with RTK, below 1 km/h). On the bench the speed is not checked: simulator mode on, or a wheel angle a tractor does not give (no WAS, or beyond ±50°).
- The module receives the whole file into free flash and checks it (checksums, size, Teensy 4.1, this firmware family `AiO-WEB-OTA-1`, or `AOG-UM982-TM171-WEB-OTA-1` as the firmware before 22 Oct 2026 called it), reading back every sector. A new firmware carries both names for now, so a module still on an older firmware accepts it from the web page. **If anything fails, the running firmware stays** and the page says why.
- Then the new firmware is copied over the old one (a few seconds) and the Teensy restarts. **Do not cut the power during that copy**: the Teensy would need reprogramming over USB (it is not damaged, and the 15 s button restore also works).
- "Other firmware" installs an image without the family marker; the next update is then over USB.

### New versions from GitHub

The page looks in the `Firmware` folder of **github.com/renatorgr/Boards** when it opens, every 6 hours, and with **Check now**.

- Each version is a folder `vX.Y_AiO_WEB` with `vX.Y_AiO_WEB.hex` (and the XIAO folders). Other folders there are ignored.
- A newer version shows on Status and on the Firmware tab. **Install from GitHub** downloads it in the browser and installs it like a chosen file.
- The device the page is open on needs internet; the Teensy never goes to the internet.
- To publish: upload the whole version folder from the zip into `Firmware/`.

## GPS receiver

### Identification

After the receiver's first second, the Teensy asks it what it is: `VERSIONA` (UM982) or `UBX-MON-VER` (u-blox). It asks again after 10 s of silence and with **Ask again**. The queries go between RTCM messages, never in the middle of one. The result shows on Status, on the GPS tab and in Messages (`GPS receiver: u-blox ZED-F9P, firmware HPG 1.32 (protocol 27.31)`).

What goes to the program does not depend on this: a receiver that sends HPR is treated as dual antenna.

### Receiver check (GPS tab)

- **Data at 460800 baud**: valid, wrong baud rate (bytes but nothing valid; also a warning on Status), or nothing arriving. The GGA, VTG and HPR rates as they arrive (the Teensy wants 10 Hz).
- **At start-up**, with no valid data at 460800 within 5 s (15 s for a silent receiver), the Teensy looks for the receiver's baud rate. Found at another one, **the port stays at that speed**: the receiver is identified and checked there, and the AgOpenGPS settings (which include 460800 baud) are loaded in one go; the port switches to 460800 as the baud rate message goes out. Only a receiver with nothing to load (an unknown one) gets just its baud rate changed and saved (u-blox: `CFG-UART1-BAUDRATE` in all layers; UM982: `CONFIG COM1/2/3 460800` and `SAVECONFIG`). The **Set to 460800 and save** button does that by hand at any time.
- **Set up automatically**: a receiver that lacks what the Teensy needs after its check (a new module, for example) gets the AgOpenGPS settings loaded by the Teensy itself (F9P: `Single.txt` or `DualPosition.txt`, second F9P: `DualRelPos.txt`; X20P: `X20P.txt`; UM982: `ConfigUM982D.txt`, or `ConfigUM982S.txt` with "one antenna" in Settings), then is checked again. Once each per start-up, only with autosteer off and while nothing else has the receiver. Status shows it in the **Setup** row of the GNSS and dual-antenna cards and in a warning at the top ("the Teensy is loading the AgOpenGPS settings…"), and `[status]` records it.
- **What the Teensy needs**: u-blox keys read with `UBX-CFG-VALGET` against the AgOpenGPS files (`Single.txt`, `DualPosition.txt`, `DualRelPos.txt`, `X20P.txt`); UM982: GGA, VTG and HPR at 10 Hz on its Teensy port at 460800. The wrong values first, in orange; the right ones in a collapsed list.
- **Compare with AOGConfigOMatic** (F9P, X20P: every key of the file; UM982: which commands it already has) and **Load the AgOpenGPS settings** (F9P, X20P: each line as one `UBX-CFG-VALSET` to RAM, BBR and flash; UM982: one command at a time, then `SAVECONFIG`).
- **Firmware**: the AgOpenGPS F9P settings are for HPG 1.13. Another version shows in red with links to `UBX113.bin` and AOGConfigOMatic (**Flash firmware**). `X20P.txt` was read from an X20P with HPG 2.11; another version shows in orange.
- **X20P**: AOGConfigOMatic has no file for it, so `X20P.txt` (in the sketch folder, and in `zRxConfig.h`) was read on 24 Oct 2026 from Renato's working ZED-X20P with **Read all settings**: the 25 settings that differ from the factory defaults (UART1 at 460800, 10 Hz, high-precision NMEA, RMC / GLL / GSA / GSV off, I2C off), plus the keys the Teensy needs that stay at their factory values (UBX and RTCM3 in, UBX and NMEA out, GGA and VTG every fix). One antenna only: an X20P with a second F9P has no file yet.
- **Read all settings** (u-blox; the button is commented out in `zWebPage.h` now that `X20P.txt` exists, the Teensy side stays, see the HTML comment there to put it back): every setting of the receiver, read from its RAM layer with `UBX-CFG-VALGET` (the all-items key, 64 items per answer) and compared with its Default layer; the ones that differ from the factory defaults are kept (up to 512; `CFG-SEC-*` left out) and downloaded as a u-center configuration file named after the model (`X20P.txt`): a comment line, then `CFG-VALGET - 06 8B …` lines of up to 64 key + value pairs, the form of the AOGConfigOMatic files that `zRxConfig.h` is made from. The way to make an AgOpenGPS file for a receiver AOGConfigOMatic has none for. Reading does not change the receiver and is allowed with autosteer on.
- One operation at a time, only with autosteer off. AgIO's GPS tool (`!AOGR1`) stops it.

### u-blox settings (GPS tab)

Read from the receiver when the tab opens; **Write to the receiver** sends only what changed, in one `UBX-CFG-VALSET` to RAM, BBR and flash (RAM and BBR only if flash is refused). Only with autosteer off.

| Setting | |
|---|---|
| Rate | 1-20 Hz (AgOpenGPS / AgOpenWeb: 10 Hz) |
| Dynamic model | portable, automotive, ... |
| High-precision NMEA | more decimals in position and altitude |
| UART1 | GGA and VTG (always on), RTCM3 in (off = no RTK) |
| Constellations | GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS (changing them loses RTK for a few seconds) |

The UART1 baud rate and a factory reset are not on the form: the Teensy would lose the receiver. Other NMEA sentences the receiver sends are named under the card.

### UM982 settings (GPS tab)

Read with `CONFIG`, `MASK`, `UNILOGLIST` and `MODE`; **Write to the receiver** sends only what changed, one command at a time, then `SAVECONFIG`. A refused command is shown with the UM982's reason; the others are still written. Only with autosteer off. The commands act on the UM982 port wired to the Teensy.

| Setting | Command |
|---|---|
| GGA, VTG and HPR rate | `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1` (1-20 Hz together) |
| Working mode | `MODE ROVER UAV` / `AUTOMOTIVE` / `SURVEY` / `SURVEY MOW` |
| Maximum age of RTK corrections | `CONFIG RTK TIMEOUT 60` |
| SBAS | `CONFIG SBAS DISABLE` / `ENABLE AUTO` / `ENABLE EGNOS` |
| Heading mode | `CONFIG HEADING FIXLENGTH` / `VARIABLELENGTH` / `STATIC` / `LOWDYNAMIC` / `TRACTOR` |
| Distance between the antennas | `CONFIG HEADING LENGTH 120 3` (cm and tolerance; empty = automatic) |
| Heading and pitch correction | `CONFIG HEADING OFFSET 1.5 0` (use this or AgOpenWeb's `DualHeadingOffset`, not both) |
| KSXT | `KSXT 0.1` (10 Hz) or off |
| Constellations | `MASK` / `UNMASK` GPS, GLONASS, Galileo, BeiDou, QZSS |

The NMEA version shows in the receiver card. Other sentences the UM982 sends are named under the card; switch one off in the console (`UNLOG GPGSV`, then `SAVECONFIG`).

## IMU: TM171 or BNO085

At start-up the firmware looks for:

1. **Serial5** at 115200 for 1.5 s: a **TM171** or a **BNO085 in RVC mode** (jumper SJ1).
2. Then a **BNO085 on I2C** (pins 18 / 19), at 0x4A or 0x4B; up to 3 tries.
3. Nothing: `[boot] IMU NOT detected`. Serial5 is still listened to, so an IMU that starts later is picked up then.

- All give the same roll, pitch and yaw, with the AiO firmware's conventions. "Use Y axis" in the program, or "Swap roll and pitch" on the page, swaps roll and pitch; "Roll invert" / "Roll zero" in the program.
- RVC or I2C is chosen on the BNO085 module (its PS0 / PS1 pins).
- A BNO085 on I2C silent for 3 s is started again (only with autosteer off).
- The yaw direction is learned in turns (or set on the page); yaw jumps over 25° are compensated.

### TM171 gyro calibration from the page

A TM171's yaw drifts while the tractor sits with the engine running. SYD Dynamics' remedy is a gyro
calibration taken with the vehicle parked ("run-time static calibration"; application note AN002 says the
engine can stay running). The **IMU tab** of the web page has a **Calibrate gyro** button for it (the IMU card on the Status page shows the result and links there):

- Park the tractor and switch autosteer off (the page refuses otherwise). The TM171 collects data for 20 s
  while the page counts down, then reports **OK** or **failed** (the tractor moved, or there was too much
  vibration; the previous values are kept). Engaging autosteer or driving off stops the calibration.
- Per SYD's user guide the result is held in the TM171's RAM, so it lasts **until the TM171 is powered
  off**. Run it after starting the tractor whenever the yaw drifts.
- **Wiring**: the board connects TM171 TX to Teensy RX5 (jumper SJ1) but nothing to TM171 RX. The button
  needs a wire from **Teensy pin 20 (TX5) to the TM171 RX pin**. Without it the page reports "no reply from
  the TM171" after 3 s.
- `[status]` lines: started, done, failed, stopped, no reply. Constants in `TM171.ino`: `TM171_CALIB_S`
  (20 s), `TM171_CALIB_LIMIT` (accepted gyro error, 0.5 °/s), `TM171_CALIB_CONF`.

A second button, **New install calibration**, is the "CalibB" of SYD's ImuAssistant: the TM171 collects
about 20 s of data with the module still and stores the result in its **flash memory**. Run it once after
installing the TM171, or after a large temperature change, with the tractor parked and autosteer off. It
cannot be stopped once started, and **power must not be cut while it runs** (SYD warns a power loss during
those 20 s can damage the TM171). The page shows the TM171's reply (saved / failed / no result after 60 s);
**switch the whole module off and on afterwards**, as SYD requires (the TM171 has no power switch of its
own on the AiO board).

The magnetometer is never calibrated: the firmware aligns the IMU yaw to the GPS heading, so it is unused.

### TM171 settings from the page

On the IMU tab, a **TM171 settings** fold-out shows what the TM171 reported when the firmware requested
its settings at start-up (Setting object 21): magnetometer on/off, GyroErrFilter with the accelerometer gain
(the dial next to it in ImuAssistant), Self-Adapt filter with the magnetometer gain, status output, boot
mode and output data rate (a list: 25, 50 or 100 Hz; the TM171's inhibit time, the shortest gap between two
packets, is shown and left as read). 25 Hz is the AgOpenGPS forum set-up and 100 Hz what the AiO v6 firmware
uses; this firmware takes the newest TM171 sample at each GPS fix, so a higher rate only makes that sample
fresher (at most 10 ms old instead of 40 ms). The set-up check accepts the three; a rate read that is not in
the list is shown "as read". **AgOpenGPS defaults** loads the values recommended on the AgOpenGPS forum (magnetometer
off, GyroErrFilter 3.50, Self-Adapt 1.00, RPY + Status, UART 115200, 25 Hz, Auto boot). **Write to TM171**
sends the form, saved to flash unless you untick it, waits for the TM171's acknowledgement and reads the
settings back. The measured output rate is shown next to the rate field.

Deliberately out of reach: the gyro, the accelerometer, the roll-pitch-yaw output and the UART output are
always left on, and the UART baud rate is never changed (the Teensy would lose contact with the TM171; use
ImuAssistant over the TM171's USB-C port for that). SYD's guide does not number the bits of the settings
word, so before writing anything the firmware checks the reserved bits of the word it read and the baud rate
word; if they do not match, the page reports "layout not recognised" with the raw bytes and refuses to write.
The layout was confirmed on a real TM171 on 24 Oct 2026 (the first guess took the rate for an inhibit time and
showed a "silent time" that does not exist; the words the firmware does not know are left as read). All of
this needs the TX5 wire.

Commands are addressed to the TM171's own node ID, taken from the frames it sends (the `[status]` line
"settings read from ID N"). A real TM171 answered requests sent "to everyone" (ID 0) but ignored every
settings write sent that way (24 Oct 2026). To see what happens on the wire, the log gets the write frame
byte by byte ("frame sent"), every acknowledge the TM171 sends, and the first frame of each type the firmware
does not handle.

### TM171 set-up check

If nothing answers at 115200 at start-up, the firmware looks for the TM171 at its other baud rates
(460800, 230400, 921600, 57600, 38400, 9600). Found, Serial5 stays at that rate and the TM171 works as
usual. A baud rate other than 115200, or settings that differ from the AgOpenGPS set-up above, give a
warning at the top of the Status page and in the IMU card (and tab), a `[status]` line, and a message on the
AgOpenWeb / AgOpenGPS screen once a minute ("TM171 not set up for AgOpenGPS: module page, IMU tab, Fix TM171
setup"). **Fix TM171 setup** writes 115200 baud and the AgOpenGPS set-up to the TM171's flash in one go,
switches Serial5 to 115200 and reads the settings back; the tab then asks for a **New install
calibration**. Nothing is written without that click. The settings must have been read first, so the TX5
wire is needed; without it the page can only warn.


## Dual F9P: second F9P on the GPS2 port

- **First F9P** (GPS1, Serial7, 460800): GGA + VTG at 10 Hz, and its corrections to the second F9P over UART2.
- **Second F9P** (GPS2, Serial2, pins 7 / 8, 460800): **UBX-NAV-RELPOSNED** every epoch.
- The receiver check can load `DualPosition.txt` and `DualRelPos.txt` into them.
- Web page: Settings → **GPS2 port: automatic** (the default). The second F9P is found by itself at every start: its valid UBX or NMEA data, or, when only bytes with nothing valid arrive at 460800 and no XIAO says hello, a search for its baud rate at start-up (a new F9P sends at 38400); no u-blox receiver found there, the port goes back to listening. A XIAO's hello gives the port back to the XIAO. The port only changes hands with autosteer off.
- **Status tab**, dual-antenna heading card: the second F9P's model and firmware, its data at 460800 (valid,
  wrong baud rate, nothing arriving) and its **Setup** (all good, or what the Teensy needs and it lacks),
  with a link to the GPS tab. A warning at the top of Status when nothing arrives from it, when it is at the
  wrong baud rate, or when it lacks settings. The GPS tab, **Second F9P check**, has the details and the
  buttons: **Set to 460800 and save**, **Compare with AOGConfigOMatic**, **Load the AgOpenGPS settings**.
- A fresh F9P sends NMEA, not the RELPOSNED. Its sentences count as valid data, so the Teensy identifies
  it and checks it; the cards say "NMEA 10 Hz, no RELPOSNED" meanwhile.
- **Set up automatically**, like the first receiver (see the receiver check): found at another baud rate the
  port stays there and `DualRelPos.txt` is loaded in one go (it sets 460800); lacking what the Teensy needs
  it gets `DualRelPos.txt` loaded, then is checked again.
- A GNSS receiver on the GPS2 port while the port is off (valid NMEA or UBX arriving): Status says to set
  **GPS2 port** to automatic. While autosteer is on, Status says it becomes the second F9P once autosteer is off.

The heading is `relPosHeading` **+ 90°** (antennas left and right, as in the AiO firmware), used only with RTK fixed between the antennas. The roll of the antennas goes into `$PAOGI` when there is no IMU. From there everything is as with the UM982.

In AgOpenWeb: **"Dual GPS" on**. In AgOpenGPS: heading source **"Dual"**.

## Steering

The output follows the program's steer settings ("Motor driver" and the rest):

| Driver | |
|---|---|
| **Cytron** | PWM + direction; the relay output switches the valve (or the motor's clutch) |
| **IBT2** | PWM on two pins, with the enable pin |
| **Danfoss** valve | PWM around 50 % (25 % full left, 75 % full right), "Danfoss" ticked in the program |
| **Keya** motor | CAN bus (CAN3, pins 30 / 31, 250 kbit/s), found by its heartbeat at start-up or later |

**Keya** (the AiO firmware's code):

- Kickout: with "current sensor" on in the program, the Keya's slip is compared with the program's limit.
- A motor fault (over voltage, stall, hall failure, ...) turns autosteer off and is named in the messages.
- No heartbeat for 0.5 s (cable, plug, power) turns autosteer off and blocks it until the heartbeat is back, as in the AiO 26 firmware.
- **Without a WAS** (WAS set to "differential" in the program): the wheel angle comes from the Keya's position, corrected slowly above 1.2 km/h with the angle the GPS heading gives. The wheelbase comes from AgOpenWeb's vehicle settings (PGN 247, sent when the program starts, when the vehicle changes and when the Teensy restarts); until it arrives, and with AgOpenGPS, the firmware uses `WHEELBASE_M` (main .ino, 3.20 m). The Steering settings card shows which one is in use.

**For every driver, as in the AiO firmware:** autosteer is cut beyond **±50°** of wheel angle or above **25 km/h**; below **0.2 km/h** the wheels are not turned (both speeds can be changed on the web page, Settings > Steering); a **WAS map** (`inputWAS` / `outputWAS` in `Autosteer.ino`) can straighten a non-linear WAS (as delivered it changes nothing).

At the top of `Autosteer.ino`:

- `PWM_Frequency` 1 = **122 Hz** (0 = 490 Hz, 2 = 3921 Hz). On a proportional valve 122 Hz acts as dither; a DC motor runs smoother at 490 Hz.
- `LOW_HIGH_DEGREES` **1.5**: within 1.5° of the requested angle the highest PWM drops towards the program's "low PWM". If the steering snakes, lower Kp or the maximum PWM, or go back to 3.0.

## Autosteer safety

Everything here acts only on the module's own outputs (valve, Danfoss, Keya), like the steer switch. The tractor's own steering is never touched.

### WAS check

The WAS voltage (single input) must stay between **0.2 and 4.8 V**. A broken wire, a loose plug or a dead sensor gives a reading at one end, which would drive the valve hard.

- 4 readings in a row out of range (100 ms): autosteer off at once and blocked (`WAS: 0.05 V, out of the 0.2-4.8 V range ... - autosteer blocked`, and on the program's screen). Shorter glitches are ignored.
- Back in range for 1 s: the block ends, and autosteer must be engaged again by hand.
- Differential input: only beyond ±4.8 V. A wire that floats mid-range cannot be told from a real reading.
- **Bench without a WAS**: switch the check off on the page. While off: a warning on Status, a `[boot]` line, a note on the program's screen, and an out-of-range angle goes to the program as 0°. Switch it back on before using the tractor.

### Why autosteer went off

Autosteer stops at the next steering loop (25 ms). The reason goes to Messages, to the web page and to the program's screen (AgOpenWeb: "AiO Board Msgs", on by default; AgOpenGPS: "Hardware Messages", off by default):

| Cause | Program's screen |
|---|---|
| steer switch / button | `Autosteer off: steer switch` |
| pressure / current kickout | `Autosteer off: pressure kickout (40 %, limit 36 %)` |
| wheel encoder | `Autosteer off: wheel encoder kickout (5 pulses)` |
| Keya slip / fault | `Autosteer off: Keya kickout (34 %, limit 31 %)` / `Keya motor fault (0x0002)` |
| Keya not responding (no heartbeat for 0.5 s) | `Keya not responding (CAN cable?): autosteer off` |
| wheel angle / speed beyond the limits | `Autosteer off: wheel angle 63 deg, beyond +-50` / `speed 27 km/h, above 25` (the maximum set on the web page) |
| WAS out of range | `WAS fault (0.03 V): autosteer blocked` |
| no steering data for 2.5 s | `Autosteer off: no steering data from the program for 2.5 s` |

### Watchdog and crash

- The hardware watchdog restarts the Teensy if the firmware stops for **2 s**. Half a second before, the autosteer outputs are switched off.
- On a crash (a processor fault), the outputs go off at once (a Keya gets speed 0), then the Teensy restarts.

### Why the Teensy started

Shown in a `[boot]` line, on the Firmware tab, and after a watchdog or crash restart on the program's screen: power on, restart by software, **WATCHDOG** (the line gives where the loop was) or **CRASH**:

`[boot] Start: CRASH at 0x0001D542, called from 0x0001EC86 (fault 0x00000082, address 0x2007A01C), restarted by the watchdog (v4.5_AiO_WEB, Teensyduino 1.58). Please send this line`

The addresses only mean something with that build. Send the `.elf` too: Arduino IDE → Sketch → Export Compiled Binary, then `build/teensy.avr.teensy41/v4.5_AiO_WEB.ino.elf` in the sketch folder.

## Network stack

NativeEthernet runs the network stack (FNET) from a 1 ms timer interrupt, which could change the stack's memory in the middle of a network call from the loop (the crash at `0x0001D542`). The firmware stops that timer after `Ethernet.begin()` and runs the stack itself from the loop and in every wait for the network (`zNet.h`). NativeEthernet itself is not changed.

## Inputs

- **GNSS receiver**: Serial7, 460800, 10 Hz. UM982: GGA + VTG + HPR. F9P / X20P: GGA + VTG (the HPR is not waited for). Dual F9P: GGA + VTG, and the second F9P's RELPOSNED.
- **IMU**: Serial5 (TM171, BNO085 RVC) or I2C (BNO085).
- **GPS2 port**: Serial2, 460800: the XIAO or the second F9P.
- **Keya**: CAN3.
- **Correction radio**: Serial3 RX, 115200 (XBee on the Micro, radio header on the Standard), passed to the receiver. The RTCM messages from the radio and from NTRIP (UDP 2233) are counted: good, bad checksum, types.

## Output to the program (UDP 9999)

### Standard set (AgOpenWeb)

Four lines per epoch, one per datagram:

| Line | Content |
|---|---|
| `$GNGGA` | the receiver's GGA (a UM982 GGA with `00` satellites / HDOP `9999.0` gets the last good values, up to 2 s) |
| `$GNVTG` | speed and track (computed from the positions if the VTG is empty) |
| `$GNTHS` | the dual-antenna heading; `V` = none |
| `$INHPR` | the IMU: heading, roll (in the pitch field, HPR convention), pitch; QF 4 = OK, 0 = IMU lost |

The dual heading goes out as `$GNTHS`, not `$GNHPR`: AgOpenWeb starts a new epoch when a sentence type repeats, so two HPR lines cannot share one. AgOpenWeb shows the family `GGA+VTG+HPR+THS`. Without two antennas, AgOpenWeb fuses the IMU heading with fix-to-fix itself.

### Classic (AgOpenGPS)

- **UM982 or dual F9P: `$PAOGI`**, heading source **"Dual"**. The antenna heading when valid (5 epochs in a row); without it, the IMU heading (aligned to the antennas, then corrected by the GPS course when driving straight), or the GPS course; faded back over 1 s. No `$PANDA` here: with "Dual", AgOpenGPS ignores its heading.
- **F9P / X20P: `$PANDA`**, heading source **"Fix"**. The IMU heading once its yaw direction is known (AgOpenGPS fuses it with fix-to-fix), `65535` until then or without an IMU.

### Automatic

AgOpenWeb and AgIO both send a hello (PGN 200) once a second; they differ in the last byte (AgIO `0x47`, AgOpenWeb the real checksum). In automatic mode the module sends the standard set to AgOpenWeb and classic to AgOpenGPS (and before the first hello). The other program needs 3 hellos in a row to take over. If a future AgIO computes that checksum, choose AgOpenGPS by hand.

## Setting up

**F9P / X20P (single antenna)**

- UART1 at 460800, GGA + VTG at 10 Hz, high-precision NMEA, RTCM3 in, saved in the receiver. Or from the web page alone: receiver check → **Set to 460800 and save** → **Load the AgOpenGPS settings** (`Single.txt`).
- AgOpenWeb: **"Dual GPS" off**. AgOpenGPS: heading source **"Fix"**.
- The IMU yaw direction is learned in the first turns (about 30°); set it on the page afterwards. The IMU heading is aligned to the GPS course once (2.5 s / 5 m straight above 3 km/h).

**UM982 (dual antenna)**

- On its Teensy port at 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, `SAVECONFIG`. Or receiver check → **Load the AgOpenGPS settings** (two antennas or one).
- AgOpenWeb: **"Dual GPS" on**. With `AutoDualFix` on, above `DualSwitchSpeed` it steers on fix-to-fix corrected by the antennas; turn it off to steer on the antenna heading.
- Antenna orientation: `CONFIG HEADING OFFSET` in the UM982 **or** `DualHeadingOffset` in AgOpenWeb, not both.
- `MinFixQuality` (default 4, RTK fixed): set 1 or 2 on the bench.
- NTRIP (ReNEP) ports 2101 and 2102 both give RTK fixed.
- **Antenna cables swapped**: the heading is 180° off and driving forward shows as reverse. Swap them back.

## Compile-time settings (top of `zHandlers.ino`)

- Defaults of the page settings: `HPR_ACCEPT_FLOAT`, `EPOCH_WAIT_MS` (60), `TM171_SWAP_ROLL_PITCH`, `TM171_YAW_SIGN`.
- `TM171_JUMP_DEG` 25: a larger yaw step in one epoch is compensated.
- `DUAL_ENTER_EPOCHS` 5, `DUAL_BLEND_EPOCHS` 10, `DUAL_BLEND_MAX_DEG` 5: classic `$PAOGI`, dual heading back.
- `COURSE_*`: the IMU heading and the GPS course (first alignment: `COURSE_SEED_*`).
- `GGA_HOLD_MS` 2000: the UM982's `00` / `9999.0` GGA.
- Elsewhere: `WHEELBASE_M` (main .ino), `DUALF9P_*` (`zDualF9P.ino`), `inputWAS` / `outputWAS`, `PWM_Frequency`, `LOW_HIGH_DEGREES` (`Autosteer.ino`).
- Diagnostics (0 on the tractor): `FUSION_DEBUG`, `RAW_NMEA_DEBUG`, `SEND_TO_USB`.

## Messages

On the USB serial monitor and the Messages tab:

- `[boot]`: one line per fact at start-up (firmware and build, why it started, settings, WAS check off, simulator mode, autosteer, Ethernet, IMU, Keya, watchdog, setup time).
- `[status]`: written **only when something changes** (a new state must last 1 s): Ethernet link and the page's address, the program talking and the output mode, corrections, autosteer off and why, WAS, GPS fix, the receiver, its sentences and rates, the receiver check, the dual heading (or why there is none, and what the program gets meanwhile), the IMU, the Keya.
- `[web]`: what is done from the page (settings, commands, updates, restarts).
- `[xiao]`: the XIAO's hello, silence, first PGN, bad frames, its text lines.

## LEDs

The three two-colour LEDs on the board follow the AiO 26 firmware's LED guide as far as red and green allow
(both on = orange; the AiO 26 blue pulse is a short dark flick here). Orange is hard to tell from red through
the diffused light window of an enclosure panel, so the in-between (orange) state is shown as red and green in
turn, changing twice a second (`LED_MID_MS` in `zLeds.ino`; 0 gives steady orange). The IMU state has no
LED: see the IMU card on the page and the message on the program's screen.

| LED | Red | Orange (shown as red and green in turn) | Green | Flick |
|---|---|---|---|---|
| PWR/ETH | no Ethernet link | link, but no program talking (5 s without UDP) | AgOpenWeb or AgOpenGPS talking | |
| GPS | GGA without a fix | fix, not RTK fixed (GPS, DGPS, RTK float) | RTK fixed | corrections arriving (NTRIP or radio): once a second |
| STEER | fault: no ADS1115 (WAS), WAS voltage out of range, Keya heartbeat lost | ready, autosteer off | autosteer on | steer button or switch used |

The GPS LED is off when no GGA has arrived for 10 s. The Teensy's own LED (pin 13) blinks with every GGA.

## Files

| File | |
|---|---|
| `v4.5_AiO_WEB.ino` | setup and loop |
| `zVersion.h` | version number and firmware name |
| `zHandlers.ino` | receiver sentences, epochs, standard / classic output |
| `zReceiver.ino` / `.h` | receiver identification, u-blox and UM982 settings |
| `zRxCheck.ino` / `.h`, `zRxConfig.h` | receiver check; the AgOpenGPS settings of AOGConfigOMatic |
| `zDualF9P.ino` | GPS2 port, second F9P |
| `zImu.ino` / `.h`, `TM171.ino`, `BNO08x_AOG.*` | IMU detection and reading |
| `KeyaCANBUS.ino` | Keya motor |
| `zSafety.ino` / `.h` | WAS check, autosteer off, program's screen messages, watchdog, crash, start reason |
| `zNet.h`, `zEthernet.ino` | Ethernet, network stack run from the loop |
| `zRtcm.h` | RTCM counter |
| `zStatus.ino`, `zStatusLog.h` | messages |
| `zLeds.ino` | the three board LEDs |
| `zSettings.ino` / `.h` | settings (EEPROM) |
| `zWeb.ino`, `zWebPage.h`, `zWebJson.h` | web server and page |
| `zOTA.ino`, `FlashTxx.*` | firmware update |
| `zXiao.ino` / `.h` | XIAO link |
| `Autosteer.ino`, `AutosteerPID.ino`, `zUdpNtrip.ino`, `zADS1115.*`, `zNMEAParser.h` | from the AgOpenGPS / Gunics firmware, changed (each file says how) |
| `v4.5_AiO_WEB.hex` | compiled firmware (Teensyduino 1.58) |
| `v4.5_XIAO_ESP32/`, `v4.5_XIAO_ESP32_bin/` | the XIAO program, and compiled for the C3 and S3 |
| `COPYING` | GNU GPL v3 |

## XIAO ESP32C3 / ESP32S3

**Wiring**

- **AiO v4.5 Micro:** in the GPS2 socket (U11), through the adapter board or the 3D-printed holder: pin 1 = 3V3, pin 2 = Teensy RX2 ← XIAO D6, pin 3 = Teensy TX2 → XIAO D7, pins 10 / 11 = GND.
- **AiO v4.5 Standard:** header P8 (pin 1 = RX2 ← D6, pin 2 = TX2 → D7), 3.3 V and GND from another header.
- Web page: Settings → GPS2 port: automatic (the default): the XIAO is found by its hello.
- The XIAO card on Status says when the XIAO runs another program version than this firmware (update it on
  the XIAO's page, or over USB-C), and, when no hello ever came, that a new XIAO has to be programmed first
  (`v4.5_XIAO_ESP32C3_USB.bin` / `S3_USB.bin`, see `v4.5_XIAO_ESP32_bin/README.md`), with a link to the files.
- Powered from the board's 3.3 V: **a USB cable to the XIAO must not carry 5 V** while it is on the board. Program it off the board, or over WiFi from its own page.
- The box is aluminium: use an external antenna on the XIAO's U.FL connector.
- Bench without the board: XIAO on USB-C, and D6 → Teensy pin 7, D7 → pin 8, GND → GND.

**Link** (the official firmware v26 protocol, 460800 baud; `XIAO_BAUD` in `zXiao.h` and `LINK_BAUD` in the XIAO program must match, so a XIAO program older than 4.5, at 230400, needs updating too):

- XIAO → Teensy: `ESP32-hello` at least every 5 s (present for 10 s after each), AgOpenGPS frames sent on to UDP 9999, and text lines shown as `[xiao] > ...`.
- Teensy → XIAO, while it is present: every UDP 8888 packet starting with `80 81`; PGN 141 (the module status, once a second) and PGN 142 (each log line).
- XIAO → Teensy: PGN 143 (the XIAO's state, for its card); PGN 144 (a command: 1 = the module's subnet, which the Teensy saves and restarts with).
- PGN 141-144 never go on the network.
- Differences from v26, on the safe side: the checksum is checked (bad frames dropped and counted); frames only while the XIAO is present; the Teensy never waits (a packet for the XIAO is dropped if the buffer is full, and counted).
- At 460800, AgOpenWeb's 100 Hz steering packets use 6-10 % of the link.

The XIAO program (`v4.5_XIAO_ESP32`) has its own WiFi network and page: module status and messages, what the program sends to the module, WiFi settings, update over WiFi. See its README, or try the page in the [simulator](https://renatorgr.github.io/Boards/xiao/).

## Credits and licence

This firmware is built on other people's work:

| Part | From |
|---|---|
| Structure of the main file, steering (`Autosteer.ino`, `AutosteerPID.ino`), Ethernet, NTRIP | the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project) |
| The soft start, the Keya heartbeat timeout (0.5 s), the current kickout delay, the steering speeds as settings (ideas, rewritten for this board) | the AgOpenGPS AiO 26 firmware (`AgOpenGPS-Official/Firmware_Teensy_AiO_26`, GPL v3) |
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
| Ethernet library and network stack (used as they are, run from the loop) | NativeEthernet (vjmuzik) and FNET, as they come with Teensyduino |
| Pin comparison of the AiO v4.5 Micro and Standard | the AgOpenGPS board design files (EasyEDA, 2024-02-05) |
| The AgOpenGPS receiver settings (`zRxConfig.h`) and how they are sent: the F9P files `Single.txt`, `DualPosition.txt`, `DualRelPos.txt` (firmware HPG 1.13), the UM982 commands `ConfigUM982D.txt`, `ConfigUM982S.txt` | AOGConfigOMatic by lansalot (github.com/lansalot/AOGConfigOMatic, GPL v3); the F9P configuration by Jaap van den Handel. `X20P.txt` (HPG 2.11) was read from Renato's X20P with this firmware |

The rest was written in 2026 by Claude (Anthropic) for renatorgr: the web page, settings, messages, update from the page, the XIAO link and the XIAO program, the standard / classic / automatic output, the rewritten `zHandlers.ino`, the autosteer safety, watchdog and crash hook (`zSafety.ino`), the network stack run from the loop (`zNet.h`), the RTCM counter (`zRtcm.h`), the receiver identification and settings (`zReceiver.ino`), the receiver check (`zRxCheck.ino`), the IMU detection (`zImu.ino`), the GPS2 port setting, the second F9P (`zDualF9P.ino`) and the Keya and steering changes around the AiO firmware's code. Every file says at the top where it comes from and what was changed.

**Licence:** GNU General Public License v3 or later (file `COPYING`), like chriskinal's AOG_Teensy_UM982 and the AgOpenGPS firmware it comes from. It comes WITHOUT ANY WARRANTY: check it on the bench before trusting it to steer a tractor. The files from other authors keep their own notes too (CC0, public domain, Adafruit, MIT for the BNO080 library).
