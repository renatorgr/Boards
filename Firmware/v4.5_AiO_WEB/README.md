# v4.5_XIAO_ESP32

Program for a Seeed **XIAO ESP32C3** or **XIAO ESP32S3** on the GPS2 port of the **AiO v4.5 boards**.

- It goes with the Teensy firmware **v4.5_AiO_WEB**, with the same version number.
- On the Teensy's web page, Settings → **GPS2 port** must be **XIAO ESP32** (the default). The second F9P of a dual F9P uses the same port, so it is one or the other.
- The XIAO creates its own WiFi network with a page for a phone, to see what the module is doing without the tablet or a network cable.
- It only monitors: it changes nothing in the module or in AgOpenWeb / AgOpenGPS.

## Where it goes

- **AiO v4.5 Micro:** in the GPS2 socket ("Dual-Left"), through the adapter board or the 3D-printed holder.
- **AiO v4.5 Standard:** wire it to header P8: pin 1 (RX2) ← XIAO D6, pin 2 (TX2) → XIAO D7, with 3.3 V and GND from another header.
- Both XIAOs have the same size and pins, so the wiring, the adapter and the holder are the same.
- On the board the XIAO is powered from the board's 3.3 V: **never plug a USB cable with 5 V into it while it is on the board** (see "USB-C port on the panel").

## Version 4.5 (19 Oct 2026)

- Its own WiFi network and page: module status and messages, what the program sends to the module, WiFi settings, update over WiFi or from GitHub.
- Runs on the XIAO ESP32C3 and ESP32S3; the page refuses an update made for the other one.
- Link to the Teensy at **460800 baud**, like the official firmware v26. A Teensy firmware older than 4.5 (230400) does not talk to it.
- 20 Oct 2026, same version number: the page in English only, with a light mode; no repeated rows; it reloads by itself when the XIAO restarts.

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (from the XIAO's serial number), password **`agopengps`** until you change it. A phone that joins it opens the page by itself; otherwise open **http://192.168.4.1/**. Up to 4 phones at once.
- **Another WiFi network** (optional): the farm WiFi or a phone hotspot. The page is then also at **http://xiao.local/** (on many Android phones, use the IP shown on the page). The XIAO has one radio, so a phone on its own network may lose the connection for a few seconds when it joins the other one.
- **The page**: dark or light (☀ / ☾ button, remembered by the phone; otherwise the phone's setting).

| Tab | |
|---|---|
| **Status** | **Module (Teensy)**: GNSS, dual heading, IMU, receiver, steering, network. **Program → module** (AgOpenWeb or AgOpenGPS): speed, requested steer angle, autosteer, sections, hydraulic lift, tramline. **XIAO**: uptime, start-up reason, WiFi, link to the Teensy, memory. |
| **Messages** | The module's `[status]` messages (the last 60) and the XIAO's own events. |
| **WiFi** | The XIAO's network name and password; the other network (a scan lists those found, tap one); the name `xiao` → `xiao.local`. The XIAO restarts to apply them. |
| **Firmware** | Version, the newest version on GitHub with **Install from GitHub**, update from a file, restart. |

- Changes (WiFi, update, restart) ask for the **password of the XIAO's network**. The page never shows or stores it. Just viewing the page needs nothing more than being on the network.
- The XIAO's version and WiFi state show in the **XIAO** card of the Teensy page; its events also go to the Teensy's messages as `[xiao] > ...`.
- **Watchdog**: if the program stops for 15 s, the XIAO restarts.

## Install the first time (USB-C, off the board)

**Without the Arduino IDE:** use the compiled files in `v4.5_XIAO_ESP32_bin` (see its README): `..._USB.bin` for the first install from the browser (https://espressif.github.io/esptool-js/, address 0x0).

**With the Arduino IDE 2:**

1. File > Preferences > Additional boards manager URLs: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Boards Manager: install **esp32 by Espressif Systems** (3.x; tested with 3.0.7). No other libraries are needed.
3. Open `v4.5_XIAO_ESP32.ino` (the 4 files stay in the folder `v4.5_XIAO_ESP32`).
4. Tools:

| Setting | XIAO ESP32C3 | XIAO ESP32S3 |
|---|---|---|
| Board | **XIAO_ESP32C3** | **XIAO_ESP32S3** |
| USB CDC On Boot | **Enabled** | **Enabled** |
| USB Mode | (not shown) | Hardware CDC and JTAG |
| Partition Scheme | Default 4MB with spiffs | Default with spiffs (3MB APP/1.5MB SPIFFS) |

5. XIAO off the board, USB-C to the PC, Upload. If the port does not show up: hold BOOT, press and release RESET, release BOOT, then Upload.
6. Serial Monitor (any speed): it prints the network name and the commands.

## Updates

**From GitHub:** the page looks in the `Firmware` folder of **github.com/renatorgr/Boards** (when it opens, every 6 hours, and with **Check now**). **Install from GitHub** downloads the `.bin` for this chip (`vX.Y_AiO_WEB/vX.Y_XIAO_ESP32_bin/`), checks it and sends it to the XIAO. The phone needs internet: connect the XIAO to the farm network or a hotspot ("Other network") and open the page over that network.

**From a file:**

1. Arduino IDE: Sketch > **Export Compiled Binary**.
2. Use **`v4.5_XIAO_ESP32.ino.bin`** (not the `merged`, `bootloader` or `partitions` files: the page refuses them). A file for the C3 does not run on the S3, and the page says so before sending it.
3. Phone on the XIAO's network → **Firmware** → choose the file → password → **Install**. The WiFi settings are kept.

If an update fails halfway, the old program keeps running: just try again.

## USB serial commands

Serial Monitor at any speed, line ending "Newline":

| Command | |
|---|---|
| `status` | version, WiFi, phones connected, Teensy link counters |
| `wifi-reset` | WiFi settings back to the defaults (`XIAO-xxxx` / `agopengps`, other network off) and restart: for a **forgotten password** |
| `restart` | restart |

## USB-C port on the panel

To reach the XIAO without opening the box: a panel-mount USB-C socket with a short cable into the XIAO.

- **D+ → D+, D− → D−, GND → GND** (not crossed). **Do not connect VBUS (5 V)**: on the board the XIAO runs from the board's 3.3 V, so with the board off the panel port does nothing.
- The panel socket needs **5.1 kΩ resistors from CC1 and CC2 to GND** (most have them), or a C-to-C cable will not work; a USB-A to USB-C cable works without them.
- The XIAO's USB data lines reach only its USB-C socket, not its pins or pads.

## XIAO ESP32S3

- Faster and with more memory than this program needs. D6 / D7 are GPIO43 / GPIO44 (GPIO21 / GPIO20 on the C3): same pads, same wiring.
- About 100 mA with WiFi on, −20 to 65 °C: keep the box out of the sun.
- The antenna socket (U.FL) is next to D6: route the D6 wire clear of the antenna plug.
- Its extra pads underneath sit on plastic (holder) or solder mask (adapter, no vias or tracks under them); solder only the edge pads. The "Sense" version has a connector underneath and does not sit flat.

## Files

| File | |
|---|---|
| `v4.5_XIAO_ESP32.ino` | WiFi, web server, settings, updates, USB commands |
| `xLink.h` | the link to the Teensy (frames, status, log) |
| `xPage.h` | the web page |
| `xVersion.h` | version number and board (C3 or S3) |
| `COPYING` | GNU GPL v3 |

`LINK_BAUD` (`xLink.h`) must match `XIAO_BAUD` in the Teensy firmware (`zXiao.h`).

## What was tested

- Compiles with esp32 3.0.7 without warnings (C3: 80 % of the program space; S3: 29 %).
- On a PC: `xLink.h` talking to the Teensy's `zXiao.ino` over a simulated 460800 link (60 s of AgOpenWeb at 100 Hz with 64 sections: nothing dropped), and fed random bytes with memory checks on.
- The page was checked in a browser at phone and PC size with test data.
- **Not yet tested on a real XIAO**: the first time, watch the Serial Monitor and the Messages tab.

## Credits and licence

Written in 2026 by Claude (Anthropic) for renatorgr, to go with the Teensy firmware v4.5_AiO_WEB.

- The link to the Teensy uses the same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official).
- Built with the Arduino ESP32 core (Espressif) and its libraries.

**Licence:** GNU General Public License v3 or later (file `COPYING`), WITHOUT ANY WARRANTY.
