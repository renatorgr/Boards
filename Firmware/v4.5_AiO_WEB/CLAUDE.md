# CLAUDE.md

This file guides Claude Code (claude.ai/code) when working on the Teensy firmware in this folder.
The user-facing description (what each setting does, how to install, what the web page shows) is in
`README.md`. This file is for whoever changes the code.

## Project Overview

**v4.5_AiO_WEB** is the Teensy 4.1 firmware for the **AiO v4.5 boards** (Micro and Standard, same Teensy
wiring), for AgOpenWeb and AgOpenGPS. It is not the official AgOpenGPS firmware: it started from the AiO v4
firmware (27.04.2025), chriskinal's AOG_Teensy_UM982 and Balázs Gunics' V4.1, and took some ideas from the
official AiO 26 firmware (`AgOpenGPS-Official/Firmware_Teensy_AiO_26`, which is for the **AiO v5** board:
other pins, PCA9685 outputs, QNEthernet, so its code cannot be copied here as it is).

It provides:
- GNSS: UM982 (dual antenna, HPR), u-blox F9P / X20P (single), or dual F9P (second F9P on the GPS2 port)
- IMU: TM171 or BNO085 (RVC on Serial5, or I2C), roll always, heading when there is no dual heading
- Output to the program on UDP 9999: AgOpenWeb "standard set" or AgOpenGPS "classic" `$PAOGI` / `$PANDA`
- Autosteer: Cytron / IBT2 / Danfoss through the PWM outputs, or a Keya motor on CAN3
- Autosteer safety: WAS check, angle / speed cut-offs, Keya heartbeat, hardware watchdog, crash report
- Web page on the module (port 80): status, messages, settings, GPS receiver tools, Machine tab, OTA update
- Links to companion modules: XIAO ESP32 on the GPS2 port, the sprayer Nano (machine module, IP .123)

The owner, Renato, is a farmer and beginner programmer. He builds the `.hex` himself in the Arduino IDE and
installs it over USB or from the web page. Explain changes to him simply.

## Build

The build is the **Arduino IDE** (1.8 or 2.x) with **Teensyduino 1.58** (1.57 and 1.59-1.60 also build; 1.62
with gcc 15 has not been checked).

- Open `v4.5_AiO_WEB.ino`. Board: Teensy 4.1, USB type Serial, CPU 600 MHz, Optimize "Faster".
- Sketch → Export compiled binary writes `v4.5_AiO_WEB.ino.hex` into this folder.
- No extra libraries to install: NativeEthernet, FNET, FlexCAN_T4, Wire, EEPROM come with Teensyduino; the
  BNO085 library, ADS1115-Lite, the NMEA parser and FlasherX are copies inside this folder.
- Command line equivalent (Teensy core from `https://www.pjrc.com/teensy/package_teensy_index.json`):
  `arduino-cli compile --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std --export-binaries Firmware/v4.5_AiO_WEB`

**Claude's cloud sessions cannot compile this firmware**: pjrc.com, arduino.cc and the PlatformIO registry
are blocked there. Read the code carefully, say in the pull request that it was not compiled, and Renato
compiles it before merging. Do not commit a `.hex` built from code he has not compiled.

There is no PlatformIO build (`platformio.ini`) and no `src/` folder: the Arduino IDE builds every `.ino`,
`.cpp` and `.c` in this folder, and ignores the XIAO subfolders.

## How the Arduino IDE builds the .ino files (important)

All the `.ino` files are **one C++ file**: the IDE puts `v4.5_AiO_WEB.ino` first, then the other `.ino`
files in alphabetical order, generates prototypes for the functions, and compiles the result. So:

- A global variable defined in one `.ino` is visible in the `.ino` files **after** it, not before. That is
  why most files start with `z` (they come after `Autosteer.ino`, `AutosteerPID.ino`, `KeyaCANBUS.ino`,
  `TM171.ino`).
- When an earlier file needs a name from a later one, declare it in the matching `z*.h` header (e.g.
  `keyaLost` is declared `extern` in `zSafety.h`) and include that header at the top of `v4.5_AiO_WEB.ino`.
- The `.cpp` / `.c` files (`BNO08x_AOG.cpp`, `zADS1115.cpp`, `FlashTxx.c`) are compiled separately and see
  only what they include.
- Do not add a `.cpp` that uses globals from the `.ino` files without `extern` declarations in a header.

## Folder Structure

```
Firmware/
├── v4.5_AiO_WEB/                 # this firmware (folder name = main .ino name = FW_NAME)
│   ├── v4.5_AiO_WEB.ino          # globals, setup(), loop()
│   ├── *.ino / *.h / *.cpp / *.c # the modules below (flat: the Arduino IDE needs them here or in src/)
│   ├── v4.5_AiO_WEB.ino.hex      # compiled firmware (Teensyduino 1.58), uploaded by Renato
│   ├── README.md                 # user documentation
│   ├── CLAUDE.md                 # this file
│   ├── COPYING                   # GPL v3
│   ├── v4.5_XIAO_ESP32/          # the XIAO ESP32C3 / S3 program (separate Arduino sketch)
│   └── v4.5_XIAO_ESP32_bin/      # the XIAO program compiled (.bin, also _USB.bin)
└── *.png                         # screenshots of the web pages
```

## Module Organization

The files are flat, but they group into modules like the AiO 26 `lib/aio_*` folders.

### Main
| File | Purpose |
|------|---------|
| `v4.5_AiO_WEB.ino` | Serial ports, buffers, UDP sockets, Keya state, `setup()` (start-up order) and `loop()` (steps) |
| `zVersion.h` | `FW_VERSION`, `FW_NAME` |

### Steering (AiO 26: `aio_autosteer`)
| File | Purpose |
|------|---------|
| `Autosteer.ino` | Pins, steer settings / config (EEPROM 10 / 40 / 60), WAS (ADS1115) and WAS map, switches, min / max speed, UDP 8888 PGNs (`ReceiveUdp`), 40 Hz `autosteerLoop()`, PGN 253 / 250 |
| `AutosteerPID.ino` | PID and output: Cytron, IBT2, Danfoss, Keya; soft start |
| `KeyaCANBUS.ino` | Keya on CAN3 (250 kbit/s): heartbeat, enable / speed commands, slip |
| `zSafety.ino` / `.h` | WAS check, why autosteer went off, PGN 221 text to the program, watchdog, crash hook, boot reason, `loopStep` names |
| `zADS1115.cpp` / `.h` | ADS1115-Lite (WAS on I2C, Wire1) |

### GNSS (AiO 26: `aio_navigation`)
| File | Purpose |
|------|---------|
| `zHandlers.ino` | GGA / VTG / HPR handlers, epoch assembly, IMU heading fusion, standard / classic output, PGN 247 vehicle geometry |
| `zNMEAParser.h` | NMEA parser (Glinnes Hulden, CC0) |
| `zDualF9P.ino` | Second F9P on Serial2: UBX-NAV-RELPOSNED → dual heading and roll |
| `zReceiver.ino` / `.h` | Receiver identification (VERSIONA / UBX-MON-VER), u-blox and UM982 settings from the web page, UBX frames taken out of the stream |
| `zRxCheck.ino` / `.h` | Receiver check: data at 460800, baud search, settings against the AgOpenGPS files |
| `zRxConfig.h` | AOGConfigOMatic settings (generated by a script, do not edit by hand) |
| `zUdpNtrip.ino`, `zRtcm.h` | NTRIP from the program (UDP 2233) to the receiver; RTCM message counter |

### IMU
| File | Purpose |
|------|---------|
| `zImu.ino` / `.h` | Which IMU is there (TM171, BNO085 RVC, BNO085 I2C), BNO085 reading |
| `TM171.ino` | TM171 on Serial5: reading, and the gyro calibration from the web page (commands on TX5, pin 20: needs a wire to the TM171 RXD) |
| `BNO08x_AOG.cpp` / `.h` | SparkFun BNO080 library (MIT), AgOpenGPS version |

### Network and web (AiO 26: `aio_system`)
| File | Purpose |
|------|---------|
| `zEthernet.ino`, `zNet.h` | Ethernet start, IP, UDP sockets, the FNET stack polled from `loop()` (`netPoll`, `netDelay`) |
| `zWeb.ino` | Web server: one request at a time, `/api/*` endpoints, OTA upload |
| `zWebPage.h` | The whole page (HTML / CSS / JS) as one `PROGMEM` string |
| `zWebJson.h` | Small JSON writer |
| `zOTA.ino`, `FlashTxx.c` / `.h` | Firmware update from the web page (FlasherX) |
| `zStatus.ino`, `zStatusLog.h` | `[status]` messages, which program talks to the module (PGN 200 hello) |
| `zSettings.ino` / `.h` | Web page settings block in the EEPROM (address 200) |

### Companion modules
| File | Purpose |
|------|---------|
| `zXiao.ino` / `.h` | XIAO ESP32 on Serial2 (460800): hello, PGN 141 / 142 / 143, frames to and from UDP |
| `zMachine.ino` | Machine module (sprayer Nano, IP .123) on UDP 9999: PGN 160 for the Machine tab (read only) |

## Architecture Overview

### Communication flow
```
AgOpenWeb / AgOpenGPS (tablet)
   │  UDP 8888 in: PGN 254, 252, 251, 247, 200, 201, 202  ──► ReceiveUdp() (Autosteer.ino)
   │  UDP 2233 in: NTRIP RTCM                              ──► udpNtrip() ──► receiver (Serial7)
   ◄── UDP 9999 out: NMEA per GGA epoch (zHandlers.ino), PGN 253 / 250 / 221 / 126 / 121 / 203
Receiver (Serial7, 460800) ──► parser (GGA, VTG, HPR) + ubxFeed() ──► gnssProcess() ──► UDP 9999
IMU (Serial5 or I2C) ──► imuProcess() ──► roll, heading
GPS2 port (Serial2, 460800) ──► dualF9pProcess() or xiaoProcess()
Keya (CAN3) ◄──► KeyaBus_Receive() / AutosteerPID.ino
Machine module .123 ──► UDP 9999 in ──► machineLoop() ──► web page Machine tab
Browser ──► port 80 ──► webLoop() (zWeb.ino)
```

### setup() order
`bootCapture` → `faultHookInstall` → LEDs → NMEA handlers → `settingsLoad` → serial ports → `xiaoSetup` →
`autosteerSetup` (ADS1115, EEPROM) → `EthernetStart` → `webSetup` → `netPollFromLoop` → `imuSetup` →
`CAN_Setup` → 2 s Keya heartbeat search (still polling IMU, GPS2, network) → `gnssFlushAfterSetup` →
`wdtStart` (2 s hardware watchdog).

### loop() steps
`loop()` sets `loopStep` before each part so a watchdog reset or crash can say where it was
(`stepName()` in `zSafety.ino`). A new part of the loop gets a new number **and** a name there.

| Step | Part |
|------|------|
| 14 | `netPoll()` network stack |
| 1 | `KeyaBus_Receive()` |
| 2 | USB serial (`!AOGR1` / `!AOGED`, bytes to the receiver) |
| 3 | receiver bytes → UBX / NMEA parser |
| 4 | `udpNtrip()` |
| 5 | radio (Serial3) → receiver |
| 6 | `imuProcess()` |
| 7 | `dualF9pProcess()`, `xiaoProcess()` |
| 8 | `gnssProcess()` output per epoch |
| 9 | `statusUpdate()` |
| 10 | `webLoop()` |
| 11 | `autosteerLoop()` (40 Hz inside) |
| 15 | `machineLoop()` |
| 13 | `receiverLoop()`, `rxCheckLoop()` |
| 12 | Ethernet link LEDs |
| 20 | firmware update (zOTA.ino) |

There is no scheduler (AiO 26 has `SimpleScheduler`): every part runs each loop and keeps its own timers.

## Network

| Port | Direction | Use |
|------|-----------|-----|
| 8888 | in | PGNs from the program (`Eth_udpAutoSteer`) |
| 9999 | out | NMEA and PGNs to the program (`Eth_ipDestination`, x.x.x.255) |
| 9999 | in | Machine module PGNs (`Eth_udpModules`, zMachine.ino) |
| 2233 | in | NTRIP RTCM from the program |
| 5120 | out | source port of the GNSS output (`Eth_udpPAOGI`) |
| 80 | TCP | web page |

IP: `<networkAddress>.126` (`.120` when there is no ADS1115: GPS only, no autosteer); the first three numbers
are in EEPROM 60, default 192.168.5 (PGN 201 changes them and restarts).

## PGN Reference

PGN frame: `0x80 0x81 <source> <PGN> <length> <data...> <CRC>`, CRC = sum of bytes from index 2 to the
byte before it (low 8 bits).

### In (UDP 8888, source 0x7F)
| PGN | Name | Where |
|-----|------|-------|
| 254 (0xFE) | Steer data: speed, guidance status, set angle, sections | Autosteer.ino |
| 252 (0xFC) | Steer settings: gains, PWM, WAS offset / counts | Autosteer.ino, EEPROM 10 |
| 251 (0xFB) | Steer config: invert, relay, motor driver, sensors | Autosteer.ino, EEPROM 40 |
| 247 (0xF7) | Vehicle geometry (AgOpenWeb): wheelbase | zHandlers.ino `vehicleGeometry()` |
| 200 | Hello: answers 126 (steer) and 121 (IMU); tells AgOpenWeb from AgOpenGPS | Autosteer.ino, zStatus.ino |
| 201 | Subnet change → EEPROM 60, restart | Autosteer.ino |
| 202 | Scan request → 203 reply on 255.255.255.255:9999 | Autosteer.ino |

### Out (UDP 9999)
| PGN | Name |
|-----|------|
| 253 (0xFD) | Steer status: actual angle, switches, PWM |
| 250 (0xFA) | Sensor: pressure / current / Keya slip |
| 221 | Text on the program's screen (autosteer off reason, restart) |
| 126 / 121 | Hello replies (steer, IMU) |
| 203 | Scan reply |

### XIAO link only (Serial2, never on the network)
141 status JSON (Teensy → XIAO), 142 log line (Teensy → XIAO), 143 XIAO info (XIAO → Teensy).

### Machine module (UDP 9999 in)
160 (0xA0) sprayer data from the Nano at .123, see `zMachine.ino` for the 16-byte layout.

## Web Interface

One page (`/`) with tabs, drawn by JavaScript from JSON endpoints:

| Path | Method | Handler (zWeb.ino) |
|------|--------|-------------------|
| `/` | GET | the page (`WEB_PAGE`, zWebPage.h) |
| `/api/status` | GET | `apiStatus` (also sent to the XIAO as PGN 141) |
| `/api/log` | GET | `apiLog` |
| `/api/settings` | GET / POST | `apiSettingsGet` / `apiSettingsPost` |
| `/api/gps` | GET / POST | receiver check and u-blox settings |
| `/api/um982` | GET / POST | UM982 console |
| `/api/diag` | GET | diagnostic report |
| `/api/reboot` | POST | restart |
| `/api/ota` | POST | firmware upload (zOTA.ino) |

The page never makes the loop wait for a browser for more than 3 s. Large replies are written in pieces.

## Configuration and EEPROM

| Address | Content | Written by |
|---------|---------|-----------|
| 0 | `EEP_Ident` (2400) | first start |
| 10 | `steerSettings` | PGN 252 |
| 40 | `steerConfig` | PGN 251 |
| 60 | `networkAddress` (first three IP numbers) | PGN 201 |
| 200 | `WebSettings cfg` (magic 0x5A17, version 2, checksum) | web page |

The addresses must not move: settings must survive a firmware update.

### Adding a web page setting
1. Add the field to `WebSettings` in `zSettings.h` (use `reserved[]`; if the block grows, bump
   `SETTINGS_VERSION` and take over the older block in `settingsLoad()` like `settingsFromV1()`).
2. Default in `settingsDefaults()` and range check in `settingsSanitize()` (`zSettings.ino`).
3. Add it to the table in `apiSettingsPost()` and to `apiSettingsGet()` (`zWeb.ino`).
4. Add the control to the Settings tab in `zWebPage.h`.
5. Document it in the README settings table.

## Hardware Pin Reference

| Pin / port | Function |
|------------|----------|
| Serial7 (28 RX / 29 TX) | GNSS receiver, 460800 |
| Serial5 | TM171 or BNO085 RVC, 115200. TX5 (pin 20) is not wired on the board: the TM171 calibration needs a wire to the TM171 RXD |
| Wire (18 / 19) | BNO085 on I2C |
| Wire1 | ADS1115 (WAS) |
| Serial3 | correction radio (XBee), 115200 |
| Serial2 (7 RX / 8 TX) | GPS2 port: second F9P or XIAO, 460800 |
| CAN3 (30 / 31) | Keya, 250 kbit/s |
| 2 / 3 / 4 | PWM1 (Cytron PWM, IBT2 LPWM) / PWM2 (IBT2 RPWM) / DIR1 (Cytron DIR, IBT2 enable) |
| 32 / 34 / 37 | steer switch / work switch / remote |
| A17 / A10 | current sensor / pressure sensor |
| 36 | speed pulse output |
| 5, 6, 9-13 | LEDs (power, Ethernet, GPS red / green, autosteer standby / active, GGA) |

## Common Gotchas

1. **Safety first.** Anything that touches steering must fail safe: outputs off on doubt. Keep the WAS check,
   the angle / speed cut-offs, the Keya heartbeat timeout and the crash hook working.
2. **Never block the loop.** The steering runs from `loop()`. Use `netDelay()` instead of `delay()` after
   start-up, feed the watchdog (`wdtFeed()`) in any wait, and keep waits short.
3. **The network stack runs from `loop()`** (`zNet.h`): NativeEthernet's 1 ms timer is stopped on purpose.
   Do not start it again and do not call the network from an interrupt.
4. **.ino order** (see above): declare shared names in a `z*.h` header.
5. **Teensyduino 1.59+** split `HardwareSerial`: use the `TeensySerial` typedef from the main `.ino`.
6. **EEPROM**: addresses fixed, write only when a value changes.
7. **OTA marker**: `OTA_MARKER` in `zOTA.ino` (`AiO-WEB-OTA-1`) must stay the same, or modules in the field
   refuse the new firmware from the web page. The running firmware refuses a .hex without the marker it knows.
   Renamed on 22 Oct 2026 from `AOG-UM982-TM171-WEB-OTA-1`: `OTA_MARKER_OLD` keeps the old name in the image
   (and accepted) so modules on older firmware still take the update. Drop it only in a later release, after
   every module runs a firmware that knows `OTA_MARKER`.
8. **GitHub update check**: the web page (and the XIAO page) list `Firmware/` on GitHub, take the highest
   folder matching `^v(\d+(\.\d+)*)_AiO_WEB$`, and download `<folder>.hex` from it (the XIAO:
   `<folder>/vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` or `S3`). Do not rename these. Note: the Arduino
   IDE exports `v4.5_AiO_WEB.ino.hex`, but the page looks for `v4.5_AiO_WEB.hex`, so publish the next version
   under the `.hex` name the page looks for (or teach the page both names first).
9. **Debug switches** at the top of `zHandlers.ino` (`FUSION_DEBUG`, `RAW_NMEA_DEBUG`, `SEND_TO_USB`) must
   be 0 in a published build.
10. **Web page size**: `zWebPage.h` is one big string in flash; keep the JavaScript plain (no libraries).

## Version and Publishing

- Version in `zVersion.h` (`FW_VERSION`). The XIAO program (`xVersion.h`) must have the same number.
- A new version is a new folder: copy `v4.5_AiO_WEB` to `vX.Y_AiO_WEB`, rename the main `.ino` to the same
  name, set `FW_VERSION`, build, put the `.hex` (named `vX.Y_AiO_WEB.hex`) and the XIAO `.bin` files in it.
- Add a "Version X.Y" section to the README.

## Code Style

- Code, comments, web page and README in **English**.
- Each file starts with a header: what it does, where it comes from (credits), what was changed and by
  whom, the GPL v3 licence line and `SPDX-License-Identifier: GPL-3.0-or-later`. Keep the original notices.
- Comments say why, in plain words; keep the existing density.
- Status messages: `[boot]` once per fact at start-up, `[status]` only on a change, `[web]` for page
  actions, `[xiao]` for the XIAO.

## Third-Party Code (change only with a note in the file header)

| File | Origin | Licence |
|------|--------|---------|
| `BNO08x_AOG.*` | SparkFun BNO080 library, AgOpenGPS version | MIT |
| `zADS1115.*` | ADS1115-Lite, Terry Myers | MIT |
| `zNMEAParser.h` | NMEA parser, Glinnes Hulden | CC0 |
| `FlashTxx.*` | FlasherX, Joe Pasquariello | public domain |
| `zRxConfig.h` | AOGConfigOMatic, lansalot (generated) | as in the file |

## Comparison with the AiO 26 organization

AiO 26 uses PlatformIO (`platformio.ini`, `src/main.cpp`), one class per subsystem in `lib/aio_*/`
(`.h` + `.cpp`), a cooperative scheduler, a PGN router with callbacks, a GitHub Actions build, and design
notes in `docs/` and `knowledge/`. This firmware keeps the Arduino IDE sketch layout so Renato can build it
as he always has. The module tables above map each file to the AiO 26 module it matches. A move to `.cpp`
modules in `src/<module>/` is possible without leaving the Arduino IDE (it compiles a sketch's `src/` folder
too), one module at a time; see the project's plan before starting it.

## License

GPL v3 or later (file `COPYING`).
