# v4.5_XIAO_ESP32

Program for a Seeed **XIAO ESP32C3** or **XIAO ESP32S3** on the GPS2 port of the **AiO v4.5 boards**.

- It goes with the Teensy firmware **v4.5_AiO_WEB**: the two must have the same version number.
- On the Teensy's web page, Settings → **GPS2 port** must be **automatic** (the default): the Teensy finds the XIAO by its hello. The same port takes the second F9P of a dual F9P, so it is one or the other.
- The XIAO creates its own WiFi network with a page for a phone: you can see what the module is doing in the cab or next to the tractor, without the tablet or a network cable.
- It only monitors: it does not change anything in the module or in AgOpenWeb / AgOpenGPS.

## Where it goes

- **AiO v4.5 Micro:** in the GPS2 socket ("Dual-Left"), through the adapter board or the 3D-printed holder.
- **AiO v4.5 Standard:** the adapter and the holder do not fit. Wire it to header P8: pin 1 (RX2) ← XIAO D6, pin 2 (TX2) → XIAO D7, with 3.3 V and GND from another header.
- The two XIAO boards have the same size and the same pins (3V3, GND, D6 = TX, D7 = RX), so the wiring, the adapter board and the holder are the same for both.
- Compile the program for the XIAO you have. It then calls itself `v4.5_XIAO_ESP32C3` or `v4.5_XIAO_ESP32S3`, and the page refuses an update made for the other one.
- On the board the XIAO is powered from the board's 3.3 V: **never plug a USB cable with 5 V into it while it is on the board** (see "USB-C port on the panel").

The Teensy side of the link (pins, frames, PGN 141 / 142 / 143, speed) is described in the README of the Teensy firmware.

## Versions

### 4.5 (19 Oct 2026)

Everything in this README is version 4.5, the same number as the Teensy firmware it goes with (v4.5_AiO_WEB).

- Its own WiFi network with the page: the module status and messages, what the program sends to the module, WiFi settings, update over WiFi.
- Runs on the **XIAO ESP32C3 and the XIAO ESP32S3**; the page refuses an update made for the other one.
- The page in **English**, **dark or light** (the button at the top; without a choice, the phone's setting decides).
- The module card names the program that talks to the module (AgOpenWeb or AgOpenGPS), the GPS receiver's model and the IMU the Teensy found (TM171 or BNO085).
- The newest version on GitHub, with **Install from GitHub** (see "Update from GitHub" below); "latest version installed" when it is the one installed.
- The page shows its cards in **three columns** on a wide screen (one column on a phone), and the tab with the version and the update is called **Firmware**.
- Changed later on 20 Oct 2026: the page only in English, a light mode, and no repeated rows (the XIAO's version only in the header and the Firmware tab, the requested steer angle only in Program → module).
- Fixes on 20 Oct 2026 (evening): the start-up reason and the USB help in English (`status | wifi-reset | restart`; the old Portuguese names still work); the page opens on the farm network under any name the router gives the XIAO (only requests on the XIAO's own network are sent to 192.168.4.1); a warning at the top of Status when the Teensy runs another version; "no heading (QF too low)" instead of "UM982 without a solution" (it is also the second F9P's reason).
- **Link to the Teensy at 460800**, like the official firmware v26 and the second F9P that can take the same port. A Teensy with an earlier build (230400) does not talk to it: the Teensy firmware must be 4.5 too.

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (the last 4 characters come from the XIAO's own serial number), password **`agopengps`** until you change it.
  - A phone that joins it opens the page by itself (like a hotel WiFi page). If not, open **http://192.168.4.1/**.
  - Up to 4 phones at the same time.
- **Another WiFi network too** (optional): the farm WiFi or a phone hotspot. Then the page is also at **http://xiao.local/** (the name can be changed) or at the IP shown on the page.
  - The XIAO has one radio: its own network moves to the other network's channel when it joins it, so a phone on the XIAO network may lose the connection for a few seconds.
  - `xiao.local` works on iPhone, Mac, Windows 10/11 and Linux; on many Android phones use the IP instead.
- **The page**, in English, dark or light:
  - the button at the top (☀ / ☾) switches it, and the phone remembers the choice;
  - without a choice, the phone's setting (dark or light mode) decides;
  - when the XIAO restarts (its uptime goes back, or another build runs), the page loads itself again, on the same tab.

| Tab | What it shows |
|---|---|
| **Status** | **Module (Teensy)**, sent by the Teensy once a second: GNSS (fix, satellites, age of corrections), dual antenna heading, IMU (TM171 or BNO085), the GPS receiver's model, steering, network. |
| | **Program → module** (named AgOpenWeb or AgOpenGPS when the Teensy knows which program sends the hello): speed, requested steer angle, autosteer on / off, sections (16 boxes), hydraulic lift (up / down), tramline, PGN 254 per second. |
| | **XIAO**: uptime, start-up reason, WiFi, the link to the Teensy, memory (its version is in the header and the Firmware tab; the requested steer angle is in Program → module). |
| **Messages** | The module's `[status]` messages (the same as the Messages tab of the Teensy page, last 60), and the XIAO's own events (WiFi, phones joining, updates). |
| **WiFi** | The XIAO's network name and password. |
| | The other network: a scan lists the networks found, one per line, strongest first; tap one to choose it. |
| | The name on the network (`xiao` → `xiao.local`). |
| | Saved in the XIAO; it restarts to apply them. |
| | **Module network**: the module's (Teensy's) Ethernet IP and its subnet. For a module on a network that is not 192.168.5.x: type that network's first three numbers (and the XIAO password); the Teensy saves it and restarts with the new IP. This page is on WiFi, so it works whatever the module's IP is. |
| **Firmware** | Version, the newest version on GitHub with **Install from GitHub**, **program update over WiFi** (a `.bin` file, see below), restart. One password field for all. |

- Changes (WiFi, update, restart) **ask for the password of the XIAO's own network**, to confirm.
  - Typed once, it is filled in both tabs until the page is closed.
  - The page never shows or stores it.
  - Just viewing the page needs nothing more than being on the network.
- It tells the Teensy its version and WiFi state: they show in the **XIAO** card of the Teensy page (normally http://192.168.5.126/; the module's IP is shown on the Status tab). Its events also go to the Teensy log as `[xiao] > ...`. From the WiFi tab it can set the module's subnet (PGN 144 to the Teensy).
- **Watchdog**: if the program ever stops for 15 s, the XIAO restarts by itself.
- The Teensy sends it the program's packets and the module status; the XIAO sends back only its hello, its state and text lines.

## Install the first time (USB-C, off the board)

**Without the Arduino IDE:** the firmware folder has the program already compiled for both XIAOs (`v4.5_XIAO_ESP32_bin`, see its README).

- `v4.5_XIAO_ESP32C3_USB.bin` / `v4.5_XIAO_ESP32S3_USB.bin`: the first install over USB, from the browser (https://espressif.github.io/esptool-js/, address 0x0).
- `v4.5_XIAO_ESP32C3.bin` / `v4.5_XIAO_ESP32S3.bin`: for the update over WiFi.

**With the Arduino IDE:**

1. Arduino IDE 2: File > Preferences > Additional boards manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Tools > Board > Boards Manager: install **esp32 by Espressif Systems** (3.x; tested with 3.0.7).
3. Open `v4.5_XIAO_ESP32.ino` (the 4 files must stay in the folder `v4.5_XIAO_ESP32`).
4. Tools:

| Setting | XIAO ESP32C3 | XIAO ESP32S3 |
|---|---|---|
| Board | **XIAO_ESP32C3** | **XIAO_ESP32S3** |
| USB CDC On Boot | **Enabled** | **Enabled** |
| USB Mode | (not shown) | Hardware CDC and JTAG |
| Partition Scheme | Default 4MB with spiffs | Default with spiffs (3MB APP/1.5MB SPIFFS) |
| PSRAM | (not shown) | not needed (Disabled is fine) |

   - If USB CDC On Boot is not enabled, or the board is another ESP32, compiling stops with a message saying so.
   - The default partition schemes have room for updates over WiFi.
5. XIAO **off the board**, USB-C to the PC, choose its port, Upload.
   - If the port does not show up: hold BOOT, press and release RESET, release BOOT, then Upload.
6. Serial Monitor (any speed): it prints the network name and the commands.

No other libraries are needed: everything is in the esp32 package.

## Update from GitHub

- The page looks for the newest version in the `Firmware` folder of **github.com/renatorgr/Boards**: when it opens, every 6 hours while it stays open, and with **Check now** (Firmware tab).
- A newer one shows at the top of Status and in Firmware.
- **Install from GitHub** (with the password of the XIAO's network):
  - the browser downloads the `.bin` for this XIAO (in the version folder `vX.Y_AiO_WEB`: `vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` or `...S3.bin`, chosen from the chip it reports);
  - it checks that the file is for this chip and that its size matches GitHub;
  - it sends it to the XIAO like a chosen file.
- **The phone (or PC) needs internet.** On the XIAO's own network alone it usually has none: connect the XIAO to the farm network or to the phone's hotspot ("Other network" in WiFi), and open the page over that network (`http://xiao.local/` or the IP shown).
- Without internet the page says "no answer from GitHub" and nothing else changes.

## Update over WiFi (from a file)

1. Arduino IDE: Sketch > **Export Compiled Binary**. It writes a `build` folder inside the sketch folder.
2. Use **`v4.5_XIAO_ESP32.ino.bin`**. Not the files with `merged`, `bootloader` or `partitions` in the name (the page refuses them).
   - A `.bin` compiled for the XIAO ESP32C3 does not run on the ESP32S3, and the other way round. The page reads the start of the file and says so before sending it.
   - With both XIAOs, rename the files after exporting (for example `…_C3.bin` and `…_S3.bin`).
3. Phone or PC on the XIAO's network → page → **Firmware** → choose the file → password → **Install**.
4. It takes less than a minute and the XIAO restarts with the new program. The WiFi settings are kept.

If the update fails halfway (WiFi lost), the old program keeps running: just try again.

## USB serial commands

Serial Monitor at any speed, line ending "Newline" (or any):

| Command | What it does |
|---|---|
| `status` | version, WiFi, phones connected, Teensy link counters |
| `wifi-reset` | WiFi settings back to the defaults (`XIAO-xxxx` / `agopengps`, other network off) and restart: for a **forgotten password** |
| `restart` | restart |

## USB-C port on the panel

To set up or recover the XIAO without opening the box: a panel-mount USB-C socket with a short cable (right-angle USB-C plug) into the XIAO.

- Wires: **D+ → D+, D− → D−, GND → GND**. Same name to same name: they are not crossed.
- **Do not connect VBUS (5 V)**: on the board the XIAO is powered by the board's 3.3 V.
  - Without VBUS the XIAO's USB still works while the board is on.
  - With the board off the XIAO has no power: the panel port then does nothing.
- The panel socket needs the two **5.1 kΩ resistors from CC1 and CC2 to GND** (most panel sockets with a board have them), or a C-to-C cable from the PC will not work. A USB-A to USB-C cable works without them.
- Hold the cable so it does not pull on the XIAO's socket.
- The XIAO's USB data wires reach only its USB-C socket: they are not on its pins or on the pads underneath.

## XIAO ESP32S3: what is different

- Faster (two cores, 8 MB flash, 8 MB PSRAM). This program does not need it, but there is more room to grow (a WiFi bridge for the tablet, for example).
- D6 / D7 are GPIO43 / GPIO44 (on the C3, GPIO21 / GPIO20). Same pads, so nothing changes in the wiring.
- Seeed quotes about 100 mA with WiFi on, and an operating range of −20 to 65 °C. Mount the box where it does not sit in the sun.
- The antenna socket (U.FL) is at the other end from the USB-C, next to D6. Route the D6 wire so it does not press on the antenna plug.
- Underneath it has more gold pads and a large thermal pad.
  - On the 3D holder they sit on plastic.
  - On the adapter board they sit on solder mask, and there is no via or track under them.
  - Solder only the pads at the edge, as with the C3.
- If it has a small black connector underneath (the "Sense" version, for the camera board), it does not sit flat on the adapter or in the holder: the holder then needs a hole for it.

## The other files

| File | What it has |
|---|---|
| `v4.5_XIAO_ESP32.ino` | WiFi, web server, settings, update, USB commands, setup and loop |
| `xLink.h` | the link to the Teensy (460800 baud, frames, status, log). Plain C++ so it can be tested on a PC |
| `xPage.h` | the web page (HTML, CSS and JavaScript in one text) |
| `xVersion.h` | the version number and the board (C3 or S3, from the board chosen in the Arduino IDE). When the number changes, the folder and the `.ino` get the new name too |
| `COPYING` | the licence: GNU GPL v3 |

`LINK_BAUD` here (`xLink.h`) must be the same as `XIAO_BAUD` in the Teensy firmware (`zXiao.h`).

## What was tested

- It compiles with esp32 3.0.7 with no warnings:
  - XIAO_ESP32C3: 1,049,140 bytes (80 % of the space for a program with WiFi updates);
  - XIAO_ESP32S3: 1,002,413 bytes (29 %).
- On a PC: `xLink.h` talking to the Teensy's `zXiao.ino` through a simulated 460800 line.
  - 60 s of AgOpenWeb at 100 Hz with 64 sections, plus the status every second: nothing dropped, every status whole, the log lines in order, a Teensy restart detected.
  - `xLink.h` was also fed random bytes with memory checks on.
- The page was checked in a browser at phone and PC size with test data.
- **Not yet tested on a real XIAO.** The first time, watch the Serial Monitor and the Messages tab.

## Credits and licence

Written in 2026 by Claude (Anthropic) for renatorgr, to go with the Teensy firmware v4.5_AiO_WEB.

- The link to the Teensy uses the same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official).
- Built with the Arduino ESP32 core (Espressif) and its libraries.

**Licence:** GNU General Public License v3 or later (file `COPYING`), WITHOUT ANY WARRANTY.
