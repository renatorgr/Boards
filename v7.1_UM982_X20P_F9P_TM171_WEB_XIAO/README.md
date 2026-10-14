# v7_UM982_X20P_F9P_TM171_WEB_XIAO

Teensy 4.1 (AiO board) firmware for AgOpenWeb and AgOpenGPS — UM982 (dual antenna) **or** u-blox F9P / X20P (single antenna) + TM171 IMU, Cytron or Keya steering, and an optional **Seeed XIAO ESP32C3** on the GPS2 socket.

v7 = v6 plus:

- **XIAO ESP32C3 bridge** on Serial2 (Teensy pins 7 / 8, 230400 baud), for a XIAO plugged into the GPS2 socket ("Dual-Left") of the AiO v4.5 Micro through the adapter board or the 3D-printed holder. Same protocol as the ESP32 bridge of the official Teensy firmware v26 (which runs it at 460800): once the XIAO says `ESP32-hello`, every AgOpenGPS packet that reaches the module on UDP 8888 is copied to it, and the AgOpenGPS PGNs it sends go out on UDP 9999 (see "XIAO ESP32C3" below)
- a **XIAO card** on the web page (Estado), an on/off setting (Definições), and `[xiao]` lines in Mensagens, including the text lines the XIAO writes
- `v7_XIAO_ESP32C3_bridge_test/`: a small test program for the XIAO

Nothing else changes: without a XIAO in the GPS2 socket v7 behaves exactly like v6. The v6 settings in the EEPROM stay valid.

Versions of v7:

- **7.1** (14 Oct 2026): XIAO link at **230400** instead of 460800 (see "Link speed" below; the XIAO program must use the same value); "AIO" taken out of the name.
- 7.0 (14 Oct 2026): first v7, XIAO link at 460800.

v6 = v5 plus:

- **a web page on the module** — `http://192.168.5.126/` (the module's IP) from the AgOpenWeb tablet, a phone or a PC on the same network: status, the `[status]` messages, settings, a console for the UM982, restart and **firmware update over the network**, so the Hammond box does not have to be opened (see "Web page" below)
- **classic output** (setting on the web page): one `$PAOGI` / `$PANDA` per epoch for AgOpenGPS / AgIO (AgOpenWeb reads it too), instead of the standard set

With the default settings v6 sends exactly what v5 sent (the standard set below).

**The standard set needs an AgOpenWeb build that includes PR #300** (the epoch assembler, `develop` from the evening of 9 Oct 2026 onwards). On an older build these sentences show up as "not accepted". The classic output works with any AgOpenWeb and with AgOpenGPS.

## Compiling and first install

- Arduino IDE with **Teensyduino 1.58 or older** (the code uses `HardwareSerial*` with `addMemoryForRead()`, which Teensyduino 1.59 moved to `HardwareSerialIMXRT`). Board: Teensy 4.1, USB type Serial, CPU 600 MHz, Optimize "Faster".
- Checked here with the Teensy 1.58 core, NativeEthernet / FNET, FlexCAN_T4 and the ARM GCC: compiles and links (about 310 KB code).
- **From v6, v7 can be installed from the web page** (Firmware tab): it carries the same update marker. From v5 or older: by USB.
- The extra files `FlashTxx.c` / `FlashTxx.h` are FlasherX (Joe Pasquariello, public domain), used for the update. Keep them in the sketch folder.

## Web page

Open `http://<module IP>/` — normally `http://192.168.5.126/` (the third number follows the subnet set from AgOpenWeb). No password: anyone on the module's network can open it.

| Tab | |
|---|---|
| **Estado** | GNSS (fix, satellites, HDOP, correction age, speed), dual heading (QF, why there is none, the last `$GNHPR` as received), TM171 (roll, pitch, yaw, alignment, yaw direction), steering (WAS angle, requested angle, PWM, autosteer engaged, switch, motor), network (cable, AgOpenWeb, RTCM), what the receiver sends, the last sentences sent, the XIAO (present, last hello, packets each way, last PGN from it, checksum errors, losses) |
| **Mensagens** | the `[status]` lines (the module keeps the last 60) |
| **Definições** | the settings below, saved in the Teensy's EEPROM and applied at once (no restart) |
| **UM982** | sends a command to the UM982 port connected to the Teensy and shows the replies for 5 s (quick buttons: `CONFIG`, `UNILOGLIST`, `GPHPR 0.1`, `SAVECONFIG`). Commands that can cut the link (`FRESET`, `UNLOG`, port baud) ask for confirmation |
| **Firmware** | version and build date, firmware update, restart |

The page refreshes once a second, one request at a time; the module answers between GNSS epochs and never waits for the browser for more than 3 s (0.5 s for a connection that sends nothing), so steering and GNSS keep running.

### Settings (web page, EEPROM)

| Setting | Default | |
|---|---|---|
| Output | standard set | standard set (AgOpenWeb with PR #300) or classic `$PAOGI` / `$PANDA` |
| Classic, without dual heading | `$PANDA` | `$PANDA` with the TM171 heading, or `$PAOGI` with the TM171 heading (only once the TM171 has been aligned to the antennas since start-up; until then `$PANDA`) |
| Accept float heading (HPR QF 5) | yes | no = only QF 4 (fixed) |
| Max wait for VTG / HPR after the GGA | 60 ms | 20-100 ms |
| TM171 swap roll / pitch | no | TM171 rotated 90° on the board |
| TM171 yaw direction | automatic | automatic (learned from the dual heading in turns; +1 with a single antenna), +1 or −1 |
| XIAO bridge (v7) | on | off = the Teensy ignores whatever is on Serial2. With no XIAO in the socket, on or off makes no difference |

The defaults are the `#define`s at the top of `zHandlers.ino`; a blank or damaged EEPROM block gives the defaults. The settings live at EEPROM address 200, away from the AgOpenGPS steer settings (0-63), and survive firmware updates.

### Firmware update over the network

1. Arduino IDE: **Sketch → Export Compiled Binary**. The `.hex` is in the sketch folder (Arduino IDE 2: in `build/teensy.avr.teensy41/`).
2. Web page → Firmware → choose the `.hex` → Install. Only with **autosteer disengaged and the tractor stopped** (otherwise refused); the steering is switched off for the update.
3. The module receives the whole file into a free part of its flash (4-6 MB) and checks it: Intel HEX checksums, end record, starts at the flash base, fits, built for a Teensy 4.1 (`fw_teensy41`), and is this firmware family (`AOG-AIO-UM982-TM171-WEB-OTA-1`, so the next update can also be done from the page; the marker keeps its "AIO" so that v6 and v7 still recognise each other). Every sector written is read back. **If anything fails, the running firmware stays** and the page says why.
4. Only then is the new firmware copied over the old one (a few seconds) and the Teensy restarts; the page shows the new version when it is back.

- "Outro firmware" (other firmware) installs an image without the marker (e.g. a firmware without this page); the next update is then by USB again.
- **Do not cut the power during the final copy** (a few seconds after "A instalar e a reiniciar"): the Teensy would then have no firmware and must be reprogrammed by USB (the Teensy itself is not damaged; the 15 s button restore also works).
- The EEPROM (steer settings from AgOpenGPS / AgOpenWeb, web settings) is kept: `FLASH_RESERVE` in `FlashTxx.h` covers the top 256 KB of flash, where the Teensy 4.1 keeps its EEPROM (the stock FlasherX value, 16 KB, would erase it).

## Inputs

- GNSS receiver on Serial7, 460800 baud, 10 Hz. The receiver type is detected automatically:
  - **UM982 (dual antenna):** GGA + VTG + HPR (GP or GN talker). KSXT is no longer used (it can stay on; it is ignored).
  - **u-blox F9P / X20P (single antenna):** GGA + VTG. With no HPR, the firmware does not wait for it: each epoch goes out as soon as the GGA and VTG are in.
  - GGA is required (it marks each epoch). A VTG or HPR counts as "sent by the receiver" if one arrived in the last 3 s (`RECEIVER_SEEN_MS`); for the first 3 s after start-up both are waited for.
- TM171 (Serial5, 115200): roll, pitch and yaw.

## Output (UDP 9999), standard set: 4 lines per epoch, one per datagram

| Line | Content | AgOpenWeb uses it for |
|---|---|---|
| `$GNGGA` | the receiver's GGA (a UM982 GGA with `00` satellites / HDOP `9999.0` gets the last good values, for up to 2 s) | position, fix, satellites, HDOP, correction age |
| `$GNVTG` | speed and track from the receiver's VTG; if the VTG is empty, computed from the GGA positions | speed |
| `$GNTHS` | dual-antenna heading (the UM982's HPR value, character for character); mode `A` = valid, `V` = no dual heading | heading with two antennas |
| `$INHPR` | TM171 (talker `IN` = inertial sensor): heading aligned to the dual heading, **roll in the pitch field** (HPR convention), pitch in the roll field; QF 4 = TM171 OK, QF 0 = TM171 lost | roll, always; heading without two antennas |

Result in AgOpenWeb (checked against AgOpenWeb's own code):

| Situation | Heading | Roll |
|---|---|---|
| Two antennas (HPR QF 4 fixed, or 5 float) | `$GNTHS` (antennas) | TM171 |
| Second antenna lost / no heading solution | TM171, with AgOpenWeb's single-antenna fusion (fix-to-fix + IMU, reverse detection) | TM171 |
| Single-antenna receiver (F9P / X20P) | TM171, with the same single-antenna fusion | TM171 |
| TM171 lost | antennas (or fix-to-fix without two antennas) | 0 |

AgOpenWeb's status bar shows the family **`GGA+VTG+HPR+THS`** (the "HPR" is the TM171's `$INHPR`).


## Output (UDP 9999), classic: one sentence per epoch

| Situation | Sentence | Heading | Roll / pitch |
|---|---|---|---|
| Dual heading (HPR QF 4, or 5 if accepted) | `$PAOGI` | the UM982 HPR heading, character for character | TM171, degrees (`0.00` if the TM171 is lost) |
| No dual heading (or F9P / X20P) | `$PANDA` | TM171 heading aligned to the last dual heading, × 10 (`65535` = TM171 lost) | TM171, × 10 |
| No dual heading, setting "`$PAOGI` without dual" | `$PAOGI` | the aligned TM171 heading (only once aligned since start-up; until then `$PANDA`) | TM171, degrees |

Fields: time, latitude, N/S, longitude, E/W, fix, satellites, HDOP, altitude, correction age (from the GGA), speed in knots (VTG, else from the positions), heading, roll, pitch, yaw rate (0). Checked with AgOpenWeb's own parser and heading code: every sentence accepted; `$PAOGI` used as dual heading; `$PANDA` as IMU heading with the single-antenna fusion while the second antenna is lost.

In AgOpenGPS: Dual heading in the GPS settings while using two antennas; roll invert / zero apply to the TM171 roll.

### Why the dual heading goes out as `$GNTHS` and not as `$GNHPR`

AgOpenWeb starts a new epoch when a sentence type repeats. A `$GNHPR` (antennas) and an `$INHPR` (TM171) are both "HPR", so they could never be in the same fix: the `$INHPR` would open an epoch with no GGA and be dropped. `$GNTHS` carries the same heading as the UM982's HPR and can share the epoch with the `$INHPR`. The firmware still **reads** the UM982's `$GNHPR`.

## Single antenna: u-blox F9P / X20P

- In u-center: UART1 at 460800, GGA + VTG at 10 Hz (100 ms measurement period), high-precision NMEA on, compatibility mode and Limit82 off, RTCM3 input on UART1, all other messages off. NMEA version: any (4.11 recommended). Save to the receiver's memory (BBR/Flash).
- In AgOpenWeb: **"Dual GPS" off**. `$GNTHS` always goes out with `V`; AgOpenWeb uses the TM171 heading with its single-antenna fusion (fix-to-fix + IMU, reverse detection), as with `$PANDA`. Set the antenna height and position in the vehicle profile.
- TM171 yaw direction: without two antennas the firmware cannot learn it and uses +1 (like the official AiO firmware, which uses the TM171 yaw as is). If the heading turns the wrong way in curves, set the yaw direction to −1 on the web page. The TM171 heading has an arbitrary zero (about 100° off on this tractor's X20P box): AgOpenWeb aligns it to the fix-to-fix heading on the first metres above 1.5 km/h and keeps correcting it.
- The message `[status] Single antenna: no HPR from the receiver ...` confirms the mode.

## UM982 setup

In UPrecise (or by commands), on the port connected to the Teensy, at 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, then save (`SAVECONFIG`). Keep `CONFIG HEADING OFFSET` as it was (the HPR heading already has that offset applied).

## In AgOpenWeb (UM982, dual antenna)

- **"Dual GPS" on** (when off, AgOpenWeb ignores the antenna heading and uses only the TM171 + fix-to-fix).
- With `AutoDualFix` on, above `DualSwitchSpeed` AgOpenWeb uses the fix-to-fix heading corrected by the antennas instead of the antenna heading itself; turn it off to steer on the antenna heading.
- Antenna orientation: `CONFIG HEADING OFFSET` in the UM982 **or** `DualHeadingOffset` in AgOpenWeb — only one of the two.
- Roll direction / zero: AgOpenWeb's "Roll invert" and "Roll zero" (they apply to the `$INHPR` roll).
- `MinFixQuality` (default 4 = RTK fixed): below it the fix is marked invalid. On the bench set 1 or 2.
- HDOP and correction age are shown again (they come from the GGA).
- NTRIP (ReNEP): both port 2101 (legacy 1004/1012) and port 2102 (MSM5, GPS + GLONASS + Galileo + BeiDou) give RTK fixed with the UM982 on the tractor (2102 on 11 Oct: RTK fixed 21 s after the first DGPS fix).
- Antenna cables: if the two UM982 antenna cables are swapped, the dual heading is 180° off and the GGA position is the other antenna (with side-by-side antennas, the full baseline away from where `AntennaOffset` expects it). Symptom: driving forward is shown as reverse. Swap the cables back (or change `CONFIG HEADING OFFSET` by 180° and `AntennaOffset` to the other side).
- The serial monitor line `[status] GPS: the receiver sends (last 5 s): ...` shows which receiver is connected: a UM982 still set up for the old firmware sends `KSXT` and no `GNHPR` (add `GPHPR 0.1` on its Teensy port and `SAVECONFIG`).

## Compile-time settings (top of `zHandlers.ino`)

The first four are only the **defaults** of the web page settings.

- `HPR_ACCEPT_FLOAT` 1 = dual heading also with HPR QF 5 (float); 0 = only QF 4 (fixed).
- `EPOCH_WAIT_MS` 60: the firmware sends the epoch as soon as the VTG and the HPR with the GGA's time are in; if one is missing, it sends 60 ms after the GGA. A sentence the receiver does not send at all (HPR on an F9P / X20P) is not waited for (`RECEIVER_SEEN_MS` 3000).
- `TM171_SWAP_ROLL_PITCH`: TM171 rotated 90° on the board ("Use Y axis" also swaps roll/pitch).
- `TM171_YAW_SIGN`: 0 = yaw direction learned against the dual heading (only from real turns of the whole rig); +1/-1 forces it.
- `GGA_HOLD_MS`: how long good satellites/HDOP values are repeated when the UM982 sends `00` / `9999.0`.
- Diagnostics (0 on the tractor):
  - `FUSION_DEBUG` (0): one `[fusion] DUAL ...` or `[fusion] SINGLE ...` line per second: HPR QF and heading, `antRoll` (roll measured by the antennas, to compare with the TM171's), TM171 alignment, heading and roll sent, speed in km/h and its source (`vtg` or `positions`).
  - `RAW_NMEA_DEBUG` (0): every GGA, VTG and HPR as the receiver sends it, prefixed `[raw]`.
  - `SEND_TO_USB` (0): the 4 lines sent to AgOpenWeb are also printed on the serial monitor.

## Status messages (`zStatus.ino`)

`[status] ...` lines, **only when something changes** (a new state must last 1 s). They go to the USB serial monitor (`STATUS_TO_USB`) and to the web page (Mensagens), with `[boot]` lines at start-up and `[web]` lines for what is done from the page (settings saved, UM982 commands, updates, restarts):

- Ethernet: cable connected (module IP and destination) / cable unplugged
- AgOpenWeb talking (PC IP) / silent for 5 s
- RTCM corrections arriving from AgOpenWeb / stopped 5 s ago
- GPS: first position; fix changes with satellites and correction age; no GGA for 5 s; VTG arriving / no VTG (speed computed from the positions)
- Dual: dual-antenna heading OK (QF 4 fixed / 5 float, sent as `$GNTHS`, or `$PAOGI` in classic mode) / no dual heading
- No dual heading: **why**, followed by the last HPR line exactly as the receiver sent it (`[status]   last HPR from the receiver: $GNHPR,...`). Printed when the dual heading is lost, and again when the reason changes:
  - `the UM982 sends HPR with QF 0` — the UM982 itself has no heading from its two antennas (antennas, cables, UM982 heading setup); the firmware can only pass that on
  - `the HPR arrives too late, N ms after its GGA` — the HPR comes after the firmware has sent the epoch (it waits `EPOCH_WAIT_MS`)
  - `HPRs arrive but their time (...) does not match the GGA time (...)`
  - `the receiver stopped sending HPR (last one N ms ago)`
  - `HPR QF n but its heading field is empty or out of range`
- Single antenna: the receiver sends no HPR (F9P / X20P, or HPR off on the UM982). Followed by `but HPR lines are arriving and none is accepted` (a firmware / format problem: send that line) or by when the last HPR line arrived
- GPS: the receiver sends (last 5 s): every sentence type that comes in on Serial7 with its rate, e.g. `GNGGA 10.0 Hz, GNHPR 10.0 Hz, GNVTG 10.0 Hz`. Counted before the parser, so it shows exactly what the receiver sends. Printed when the set of types changes
- GPS: sentences from the receiver with a bad checksum in the last 5 s (bytes lost or corrupted on the serial line) / no more bad checksums
- GPS: the Teensy is reading the receiver late (more than one epoch waiting in the serial buffer) / read on time again
- TM171: data OK / no data

`[xiao]` lines (v7):

- `XIAO said hello on Serial2: ...` / `XIAO silent for 10 s (no hello): ...`
- `first PGN from the XIAO: <pgn> (source <src>, <n> data bytes) -> UDP 9999`, after each hello that follows a silence
- `frame from the XIAO dropped: bad checksum (...)`, at most one line every 10 s with the running count
- `> <text>`: a text line written by the XIAO (see below)

LEDs: green = dual heading, red = TM171 heading, red blinking = TM171 lost.

## Files

| File | |
|---|---|
| `v7_UM982_X20P_F9P_TM171_WEB_XIAO.ino` | setup and loop |
| `zXiao.ino`, `zXiao.h` | XIAO ESP32C3 bridge on Serial2 (v7) |
| `zHandlers.ino` | receiver sentences, epoch assembly, standard set / classic output, UM982 reply capture |
| `zStatus.ino`, `zStatusLog.h` | `[status]` messages and the log kept for the web page |
| `zSettings.ino`, `zSettings.h` | web page settings (EEPROM) |
| `zWeb.ino`, `zWebPage.h`, `zWebJson.h` | web server, the page, JSON answers |
| `../v7_XIAO_ESP32C3_bridge_test/` | test program for the XIAO (its own sketch, compiled for the XIAO, not the Teensy) |
| `zOTA.ino`, `FlashTxx.c`, `FlashTxx.h` | firmware update (FlasherX flash routines) |
| `TM171.ino`, `Autosteer.ino`, `AutosteerPID.ino`, `KeyaCANBUS.ino`, `zEthernet.ino`, `zUdpNtrip.ino`, `zADS1115.*`, `zNMEAParser.h` | as in v5 |

## XIAO ESP32C3 (v7)

### Hardware

- XIAO ESP32C3 in the GPS2 socket ("Dual-Left", U11) of the AiO v4.5 Micro, through the adapter board (`AiO_Micro45_XIAO_C3`) or the 3D-printed holder. Socket pin 1 = 3V3, pin 2 = Teensy pin 7 (RX2) ← XIAO D6 (TX, GPIO21), pin 3 = Teensy pin 8 (TX2) → XIAO D7 (RX, GPIO20), pins 10 / 11 = GND.
- The GPS2 socket then has no second F9P (not needed with the UM982).
- The XIAO is powered from the board's 3.3 V: **never plug its USB-C while it is on the Micro**. Program it off the board (or over WiFi, if its own program has an update page). The Teensy cannot program the XIAO: its BOOT and RESET pins are not wired.
- The Hammond box is aluminium: for WiFi or Bluetooth use an external antenna on the XIAO's U.FL connector.

### Protocol (as the official firmware v26, `docs/ESP32_BRIDGE_INTERFACE.md`), at 230400 baud

| Direction | |
|---|---|
| XIAO → Teensy | `ESP32-hello` (plain ASCII, no line ending) at least every 5 s. The XIAO counts as present for 10 s after each hello |
| AgOpenGPS → XIAO | while the XIAO is present: every packet that reaches the module on UDP 8888 and starts with `80 81`, unchanged (PGN 254, 239, 200, 201, 202, 235, 236, 238, 252, 251 ..., up to 512 bytes) |
| XIAO → AgOpenGPS | AgOpenGPS frames `80 81 src pgn len data... crc`, sent by the Teensy to UDP 9999 (broadcast), from port 8888 like the module's own PGNs |
| XIAO → Teensy log | text lines (printable ASCII ending in a line feed, up to 160 characters, at most 10 per 10 s) → `[xiao] > ...` in Mensagens |

Differences from v26, all on the safe side:

- **The checksum is checked**: the low 8 bits of the sum of the bytes from `src` to the last data byte (the AgOpenGPS checksum, "sum of byte 2 to n-2" in the AgOpenGPS PGN list; the XOR shown in the v26 document is not it, and the document's examples match neither). Frames with a wrong checksum are dropped and counted (web page: "Erros de checksum"). The ESP32-C3 writes its boot messages at 115200 on the same pin; at 230400 they are garbage, and the checksum keeps them from ever reaching AgOpenGPS.
- Frames and text lines from the XIAO are only taken **while it is present** (hello in the last 10 s): the program on the XIAO must say hello first.
- **Never waits**: if Serial2's transmit buffer (1 KB) is full, the packet for the XIAO is dropped and counted ("Perdidos"); the GNSS and steering loop is never held up. At most 8 frames from the XIAO are sent per loop.
- After a lost byte the Teensy looks for the next frame inside the bytes already received, so one lost byte costs one frame, not several.
- The steering code still reads only the first 24 bytes of each UDP 8888 packet, as before; the XIAO gets the whole packet.

### Link speed: 230400

`XIAO_BAUD` in `zXiao.ino` (Teensy) and `TEENSY_BAUD` in the XIAO program must be the same.

| Speed | Capacity (each way) | Load today: AgOpenWeb at 100 Hz, PGN 254 + 239 (+ 229) | Future WiFi bridge (NMEA out; PGNs and RTCM in) | XIAO UART FIFO (128 bytes) fills in |
|---|---|---|---|---|
| 115200 | 11.5 kB/s | 25-38 % | 35-60 % | 11 ms |
| **230400** | **23 kB/s** | **12-19 %** | **17-30 %** | **5.6 ms** |
| 460800 (v26, v7.0) | 46 kB/s | 6-10 % | 9-15 % | 2.8 ms |

- AgOpenWeb sends PGN 254 and 239 (14 bytes each) to the modules every 10 ms (its 100 Hz control loop), plus PGN 229 (16 bytes) with more than 16 sections: 2.8-4.4 kB/s. AgOpenGPS / AgIO sends them once per GPS fix (10 Hz). The XIAO's own PGNs are a few bytes.
- 460800 works, but it is ten times what the link carries, and it is the XIAO that pays for it: the ESP32-C3 has one core, shared with the WiFi, and a 128-byte receive FIFO. With the default threshold (120 bytes) a burst leaves it 8 bytes, 0.17 ms at 460800, to empty the FIFO. 230400 halves the burst speed and still keeps the load under 20 % today and about 30 % for a WiFi bridge that would carry the NMEA, the PGNs and the RTCM corrections; 115200 would get close to 60 % there.
- Speed error: Teensy +0.16 %, ESP32-C3 under 0.01 % at all these speeds (tolerance about 2 %); the few centimetres of the adapter board or of the holder's wires are fine at any of them.
- With 230400 the ESP32-C3 boot messages (115200, same pin) are still garbage for the Teensy, and are dropped (checked in the PC test with the real boot text).
- A XIAO program written for v26 (460800): change its speed to 230400, or `XIAO_BAUD` to 460800.

### The test program `v7_XIAO_ESP32C3_bridge_test`

Arduino IDE: board package **esp32 by Espressif Systems** (3.x), board **XIAO_ESP32C3**, *USB CDC On Boot: Enabled* (the default). Upload over USB-C, off the Micro. Checked here: compiles and links with esp32 3.0.7 (about 264 KB); together with the Teensy side in a PC simulation.

It talks to the Teensy on UART1 (`Serial1`) at 230400 (`TEENSY_BAUD`), routed to D7 (RX) / D6 (TX), with the UART FIFO emptied at 64 bytes instead of 120 (`setRxFIFOFull(64)`); the USB-C stays free as a serial monitor for the bench. It:

- says `ESP32-hello` every 2 s;
- reads the AgOpenGPS packets from the Teensy, checks their checksum, counts them by PGN and decodes PGN 254 (speed, steer angle asked for, guidance);
- every 5 s sends a test PGN to AgOpenGPS: **PGN 140, source 0x50**, 8 bytes (a counter and `XIAO`). PGN 140 is not in the AgOpenGPS PGN list and AgOpenWeb ignores it (checked in its code); on the Teensy page "PGN do XIAO" goes up and "Último PGN do XIAO" shows 140;
- at start-up and every 60 s writes a line such as `[xiao] > test: 960 PGN in (254: 600, 239: 300, 200: 60), 0 bad, 12 test PGN out, 12.3 km/h, steer -2.50, guidance 1` (Mensagens).

Bench test without the Micro: XIAO on USB-C (power and serial monitor), and only three wires to the Teensy: D6 → Teensy pin 7, D7 → Teensy pin 8, GND → GND (no 3V3 wire then).

What the XIAO should really do (WiFi bridge to a tablet, NTRIP client, extra sensors or sections, ...) is a program of its own; its settings (WiFi name, password, ...) belong on the XIAO's own WiFi page, not on the Teensy page.

