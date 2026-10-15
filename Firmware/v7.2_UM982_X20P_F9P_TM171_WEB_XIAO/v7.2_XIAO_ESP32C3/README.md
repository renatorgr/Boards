# v7.2_XIAO_ESP32C3

Program for the Seeed XIAO ESP32C3 on the GPS2 socket of the AiO v4.5 Micro (adapter board or 3D-printed holder). It goes with the Teensy firmware **v7.2_UM982_X20P_F9P_TM171_WEB_XIAO**: the two must have the same version number.

The XIAO makes its own WiFi network with a page for a phone: you can see what the module is doing in the cab or next to the tractor, without the tablet or a network cable.

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (the last 4 characters come from the XIAO's own serial number), password **`agopengps`** until you change it.
  - A phone that joins it opens the page by itself (like a hotel WiFi page). If not, open **http://192.168.4.1/**.
  - Up to 4 phones at the same time.
- **Another WiFi network too** (optional): the farm WiFi or a phone hotspot. Then the page is also at **http://xiao.local/** (the name can be changed) or at the IP shown on the page.
  - The XIAO has one radio: its own network moves to the other network's channel when it joins it, so a phone on the XIAO network may drop for a few seconds then.
  - `xiao.local` works on iPhone, Mac, Windows 10/11 and Linux; on many Android phones use the IP instead.
- **The page** (in Portuguese), 4 tabs:

| Tab | What it shows |
|---|---|
| Estado | **Módulo (Teensy)**: GNSS (fix, satellites, age of corrections), dual antenna heading, TM171, steering, network, sent by the Teensy once a second. **AgOpenGPS → módulo**: speed, steer angle asked for, autosteer on/off, sections (16 boxes), hydraulic lift (up/down), tramline, PGN 254 per second. **XIAO**: program, WiFi, memory, time on |
| Mensagens | the module's `[status]` messages (the same as the Mensagens tab of the Teensy page, last 60), and the XIAO's own events (WiFi, phones joining, updates) |
| WiFi | the XIAO's network name and password, the other network (with a scan), the name on the network (`xiao` → `xiao.local`). Saved in the XIAO; it restarts to apply them |
| Programa | version, **program update over WiFi** (a `.bin` file from the Arduino IDE), restart |

- Changes (WiFi, update, restart) **ask for the password of the XIAO's own network**. Looking needs nothing more than being on the network.
- It tells the Teensy its version and WiFi state: they show in the **XIAO** card of the Teensy page (http://192.168.5.126/). Its events also go to the Teensy log as `[xiao] > ...`.
- **Watchdog**: if the program ever stops for 15 s, the XIAO restarts by itself.
- It does **not** change anything in the module or in AgOpenGPS: it only looks. The Teensy sends it the AgOpenGPS packets and the status; the XIAO sends back only its hello, its state and text lines.

## Install the first time (USB-C, off the board)

1. Arduino IDE 2: File > Preferences > Additional boards manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Tools > Board > Boards Manager: install **esp32 by Espressif Systems** (3.x; tested with 3.0.7).
3. Open `v7.2_XIAO_ESP32C3.ino` (the 4 files must stay in the folder `v7.2_XIAO_ESP32C3`).
4. Tools:
   - Board: **XIAO_ESP32C3**
   - **USB CDC On Boot: Enabled** (if you forget, compiling stops with a message saying so)
   - Partition Scheme: **Default 4MB with spiffs** (it has room for updates over WiFi)
5. XIAO **off the Micro**, USB-C to the PC, choose its port, Upload.
   - If the port does not show: hold BOOT, press and release RESET, release BOOT, then Upload.
6. Serial Monitor (any speed): it prints the network name and the commands.

No other libraries are needed: everything is in the esp32 package.

## Update over WiFi (later versions)

1. Arduino IDE: Sketch > **Export Compiled Binary**. It writes a `build` folder inside the sketch folder.
2. Use **`v7.2_XIAO_ESP32C3.ino.bin`**. Not the files with `merged`, `bootloader` or `partitions` in the name (the page refuses them).
3. Phone or PC on the XIAO's network → page → **Programa** → choose the file → password → Enviar.
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

## The other files

| File | What it has |
|---|---|
| `v7.2_XIAO_ESP32C3.ino` | WiFi, web server, settings, update, USB commands, setup and loop |
| `xLink.h` | the link to the Teensy (230400 baud, frames, status, log). Plain C++ so it can be tested on a PC |
| `xPage.h` | the web page (HTML, CSS and JavaScript in one text) |
| `xVersion.h` | the version number. When it changes, the folder and the `.ino` get the new name too |

The link to the Teensy (pins, frames, PGN 141/142/143, speed) is described in the README of the Teensy firmware. `LINK_BAUD` here must be the same as `XIAO_BAUD` there.

## What was tested

- It compiles with esp32 3.0.7 for XIAO_ESP32C3 with no warnings: 1,031,254 bytes (78 % of the space for a program with WiFi updates).
- On a PC: `xLink.h` talking to the Teensy's `zXiao.ino` through a simulated 230400 line, 60 s of AgOpenWeb at 100 Hz with 64 sections plus the status every second: nothing dropped, every status whole, the log lines in order, a Teensy restart detected. `xLink.h` was also fed random bytes with memory checks on.
- The page was checked in a browser at phone and PC size with test data.
- **Not yet tested on a real XIAO.** The first time, watch the Serial Monitor and the Mensagens tab.
