# v7.5_XIAO_ESP32

Program for the Seeed **XIAO ESP32C3** or **XIAO ESP32S3** on the GPS2 socket of the AiO v4.5 Micro (adapter board or 3D-printed holder). It goes with the Teensy firmware **v7.5_UM982_X20P_F9P_TM171_WEB_XIAO**: the two must have the same version number.

7.5: to go with the Teensy firmware 7.5 (autosteer safety, watchdog). New on the page: the newest version on GitHub, with **Install from GitHub** (see "Update from GitHub" below). The status the Teensy sends it once a second is now shorter (it leaves out what only the Teensy's own page shows), so the link carries less.

The two boards have the same size and the same pins (3V3, GND, D6 = TX, D7 = RX), so the wiring, the adapter board and the 3D-printed holder are the same. Compile the program for the board you have: it then calls itself `v7.5_XIAO_ESP32C3` or `v7.5_XIAO_ESP32S3`, and the page refuses an update made for the other board.

The XIAO makes its own WiFi network with a page for a phone: you can see what the module is doing in the cab or next to the tractor, without the tablet or a network cable.

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (the last 4 characters come from the XIAO's own serial number), password **`agopengps`** until you change it.
  - A phone that joins it opens the page by itself (like a hotel WiFi page). If not, open **http://192.168.4.1/**.
  - Up to 4 phones at the same time.
- **Another WiFi network too** (optional): the farm WiFi or a phone hotspot. Then the page is also at **http://xiao.local/** (the name can be changed) or at the IP shown on the page.
  - The XIAO has one radio: its own network moves to the other network's channel when it joins it, so a phone on the XIAO network may drop for a few seconds then.
  - `xiao.local` works on iPhone, Mac, Windows 10/11 and Linux; on many Android phones use the IP instead.
- **The page**, in Portuguese or English (v7.4: the EN / PT button at the top; the phone remembers the choice; without one, the phone's language decides; the messages stay in English), 4 tabs:

| Tab | What it shows |
|---|---|
| Status | **Module (Teensy)**: GNSS (fix, satellites, age of corrections), dual antenna heading, TM171, steering, network, sent by the Teensy once a second. **Program → module** (named AgOpenWeb or AgOpenGPS when the Teensy knows which program sends the hello, v7.4): speed, steer angle asked for, autosteer on/off, sections (16 boxes), hydraulic lift (up/down), tramline, PGN 254 per second. **XIAO**: program, WiFi, memory, time on |
| Messages | the module's `[status]` messages (the same as the Messages tab of the Teensy page, last 60), and the XIAO's own events (WiFi, phones joining, updates) |
| WiFi | the XIAO's network name and password, the other network (a scan lists the networks found, one per line, strongest first: tap one to choose it), the name on the network (`xiao` → `xiao.local`). Saved in the XIAO; it restarts to apply them |
| Program | version, the newest version on GitHub with **Install from GitHub** (7.5), **program update over WiFi** (a `.bin` file, see below), restart. One password field for all |

- Changes (WiFi, update, restart) **ask for the password of the XIAO's own network**, to confirm. Typed once, it is filled in both tabs until the page is closed; the page never shows or stores it. Looking needs nothing more than being on the network.
- It tells the Teensy its version and WiFi state: they show in the **XIAO** card of the Teensy page (http://192.168.5.126/). Its events also go to the Teensy log as `[xiao] > ...`.
- **Watchdog**: if the program ever stops for 15 s, the XIAO restarts by itself.
- It does **not** change anything in the module or in AgOpenWeb: it only looks. The Teensy sends it the AgOpenWeb packets and the status; the XIAO sends back only its hello, its state and text lines.

## Install the first time (USB-C, off the board)

Without the Arduino IDE: the firmware folder has the program already compiled for both boards (`v7.5_XIAO_ESP32_bin`, see its README): `v7.5_XIAO_ESP32C3_USB.bin` / `v7.5_XIAO_ESP32S3_USB.bin` go in from the browser (https://espressif.github.io/esptool-js/, address 0x0), and `v7.5_XIAO_ESP32C3.bin` / `v7.5_XIAO_ESP32S3.bin` are for the update over WiFi.

With the Arduino IDE:

1. Arduino IDE 2: File > Preferences > Additional boards manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Tools > Board > Boards Manager: install **esp32 by Espressif Systems** (3.x; tested with 3.0.7).
3. Open `v7.5_XIAO_ESP32.ino` (the 4 files must stay in the folder `v7.5_XIAO_ESP32`).
4. Tools:

| Setting | XIAO ESP32C3 | XIAO ESP32S3 |
|---|---|---|
| Board | **XIAO_ESP32C3** | **XIAO_ESP32S3** |
| USB CDC On Boot | **Enabled** | **Enabled** |
| USB Mode | (not shown) | Hardware CDC and JTAG |
| Partition Scheme | Default 4MB with spiffs | Default with spiffs (3MB APP/1.5MB SPIFFS) |
| PSRAM | (not shown) | not needed (Disabled is fine) |

   If USB CDC On Boot is not enabled, or the board is another ESP32, compiling stops with a message saying so. The default partition schemes have room for updates over WiFi.
5. XIAO **off the Micro**, USB-C to the PC, choose its port, Upload.
   - If the port does not show: hold BOOT, press and release RESET, release BOOT, then Upload.
6. Serial Monitor (any speed): it prints the network name and the commands.

No other libraries are needed: everything is in the esp32 package.

## Update from GitHub (7.5)

The page looks for the newest version in the `Firmware` folder of **github.com/renatorgr/Boards** when it opens, every 6 hours while it stays open, and with **Check now** (Program tab). A newer one shows at the top of Status and in Program.

**Install from GitHub** (with the password of the XIAO's network): the browser downloads the `.bin` for this XIAO (`vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` or `...S3.bin`, chosen from the chip it reports), checks that it is for this chip and that its size matches GitHub, and sends it to the XIAO like a chosen file.

**The phone (or PC) needs internet.** On the XIAO's own network alone it usually has none: connect the XIAO to the farm network or to the phone's hotspot ("Other network" in WiFi) and open the page over that network (`http://xiao.local/` or the IP shown). Without internet the page says "no answer from GitHub" and nothing else changes.

## Update over WiFi (later versions)

1. Arduino IDE: Sketch > **Export Compiled Binary**. It writes a `build` folder inside the sketch folder.
2. Use **`v7.5_XIAO_ESP32.ino.bin`**. Not the files with `merged`, `bootloader` or `partitions` in the name (the page refuses them).
   - A `.bin` compiled for the XIAO ESP32C3 does not run on the ESP32S3, and the other way round. The page reads the start of the file and says so before sending it. With both boards, rename the files after exporting (for example `…_C3.bin` and `…_S3.bin`).
3. Phone or PC on the XIAO's network → page → **Program** → choose the file → password → **Install**.
4. It takes less than a minute and the XIAO restarts with the new program. The WiFi settings stay.

If the update fails half way (WiFi lost), the old program keeps running: just try again.

## USB serial commands

Serial Monitor at any speed, line ending "Newline" (or any):

| Command | What it does |
|---|---|
| `estado` | version, WiFi, phones connected, Teensy link counters |
| `wifi-reset` | WiFi settings back to the defaults (`XIAO-xxxx` / `agopengps`, other network off) and restart: for a **forgotten password** |
| `reiniciar` | restart |

## USB-C port on the panel

To set up or recover the XIAO without opening the box: a panel-mount USB-C socket with a short cable (right-angle USB-C plug) into the XIAO.

- Wires: **D+ → D+, D− → D−, GND → GND**. Same name to same name: they are not crossed.
- **Do not connect VBUS (5 V)**: on the Micro the XIAO is powered by the board's 3.3 V.
  - Without VBUS the XIAO's USB still works while the Micro is on.
  - With the Micro off the XIAO has no power: the panel port then does nothing.
- The panel socket needs the two **5.1 kΩ resistors from CC1 and CC2 to GND** (most panel sockets with a board have them) or a C-to-C cable from the PC will not work. A USB-A to USB-C cable works without them.
- Hold the cable so it does not pull on the XIAO's socket.
- The XIAO's USB data wires reach only its USB-C socket: they are not on its pins or on the pads underneath.

## XIAO ESP32S3: what is different

- Faster (two cores, 8 MB flash, 8 MB PSRAM); this program does not need it, but has more room to grow (a WiFi bridge for the tablet, for example).
- D6 / D7 are GPIO43 / GPIO44 (on the C3, GPIO21 / GPIO20). Same pads, so nothing changes in the wiring.
- Seeed gives about 100 mA with WiFi on, and an operating range of −20 to 65 °C. Mount the box where it does not sit in the sun.
- The antenna socket (U.FL) is at the other end from the USB-C, next to D6. Route the D6 wire so it does not press on the antenna plug.
- Underneath it has more gold pads and a large thermal pad. On the 3D holder they sit on plastic. On the adapter board they sit on solder mask, and there is no via or track under them. Solder only the pads at the edge, as with the C3.
- If the board has a small black connector underneath (the "Sense" version, for the camera board), it does not sit flat on the adapter or in the holder: tell me and the holder gets a hole for it.

## The other files

| File | What it has |
|---|---|
| `v7.5_XIAO_ESP32.ino` | WiFi, web server, settings, update, USB commands, setup and loop |
| `xLink.h` | the link to the Teensy (230400 baud, frames, status, log). Plain C++ so it can be tested on a PC |
| `xPage.h` | the web page (HTML, CSS and JavaScript in one text) |
| `xVersion.h` | the version number and the board (C3 or S3, from the board chosen in the Arduino IDE). When the number changes, the folder and the `.ino` get the new name too |
| `COPYING` | the licence: GNU GPL v3 |

The link to the Teensy (pins, frames, PGN 141/142/143, speed) is described in the README of the Teensy firmware. `LINK_BAUD` here must be the same as `XIAO_BAUD` there.

## What was tested

- It compiles with esp32 3.0.7 with no warnings: for XIAO_ESP32C3 1,048,932 bytes (80 % of the space for a program with WiFi updates), for XIAO_ESP32S3 1,002,221 bytes (29 %).
- On a PC: `xLink.h` talking to the Teensy's `zXiao.ino` through a simulated 230400 line, 60 s of AgOpenWeb at 100 Hz with 64 sections plus the status every second: nothing dropped, every status whole, the log lines in order, a Teensy restart detected. `xLink.h` was also fed random bytes with memory checks on.
- The page was checked in a browser at phone and PC size with test data.
- **Not yet tested on a real XIAO.** The first time, watch the Serial Monitor and the Messages tab.

## Credits and licence

Written in 2026 by Claude (Anthropic) for renatorgr, to go with the Teensy firmware v7.5_UM982_X20P_F9P_TM171_WEB_XIAO. The link to the Teensy uses the same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official). Built with the Arduino ESP32 core (Espressif) and its libraries.

**Licence:** GNU General Public License v3 or later (file `COPYING`), WITHOUT ANY WARRANTY.
