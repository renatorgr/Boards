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

## Changelog

Version 4.5, the same number as the Teensy firmware it goes with (v4.5_AiO_WEB). One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history.

| Date | What changed |
|---|---|
| 13 Nov 2026 | Loading the page no longer stops the XIAO on weak WiFi ("XIAO silent for 10 s"); the browser keeps the page. Enter in a password field does what its button does ([What it does](#what-it-does)). |
| 11 Nov 2026 | The page checks that it is the page of the program running: after an update an old page a browser kept (old buttons, new data) fetches the new one by itself ([What it does](#what-it-does)). |
| 10 Nov 2026 | Tabs and the light/dark button 44 px high with 8 px corners; the WiFi tab's password to confirm in a card of its own, no longer a field across the page ([What it does](#what-it-does)). Messages give the size of an updated program, not 0 bytes. |
| 9 Nov 2026 | The page's questions (save and restart, install, the module's firmware, subnet) in its own box: the browser's pop-ups show nothing inside an AgOpenWeb web card, so those buttons did nothing there ([What it does](#what-it-does)). |
| 7 Nov 2026 | Add-on card (`_aowcard._tcp`) under the XIAO's own WiFi name ([What it does](#what-it-does)). Program → module shows up to 64 sections (PGN 229); the WiFi settings save in one row; a shorter note in the Module firmware card; short file headers. |
| 4 Nov 2026 | The XIAO's temperature in its card, here and on the Teensy page; the Teensy's clock and temperature in the [Module card](#what-it-does). |
| 3 Nov 2026 | An [update over Wi-Fi](#update-over-wifi-from-a-file) on a weak link no longer stops after a 5 s pause (it waits up to 20 s), and an upload that is cut off is written in Messages. |
| 2 Nov 2026 | [Module (Teensy) firmware](#the-modules-firmware-from-this-page) card on the Firmware tab: install the Teensy's firmware from this page, or go back to its previous one (needs a Teensy firmware of 2 Nov 2026 or later). Red / green messages under the cards. |
| 1 Nov 2026 | Header and tabs stay at the top while the page scrolls. Report card last on the Firmware tab. Plainer **On GitHub** row. |
| 31 Oct 2026 | GPS fix green whenever there is a position. **Download the report** on the Firmware tab. [Install from GitHub](#update-from-github) also sees a new build of the same version. |
| 24 Oct 2026 | The module's subnet changed from this page (WiFi tab). The module's page and this page as links at their real addresses. |
| 20 Oct 2026 | English only, light mode, no repeated rows. The page opens on the farm network under any name. A warning when the Teensy runs another version. |
| 19 Oct 2026 | **Version 4.5**: its own WiFi network and [page](#what-it-does) (module status and messages, what the program sends, WiFi settings, update over WiFi and from GitHub), on the XIAO ESP32C3 and ESP32S3. Link to the Teensy at 460800. |

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (the last 4 characters come from the XIAO's own serial number), password **`agopengps`** until you change it.
  - A phone that joins it opens the page by itself (like a hotel WiFi page). If not, open **http://192.168.4.1/**.
  - Up to 4 phones at the same time.
- **Another WiFi network too** (optional): the farm WiFi or a phone hotspot. Then the page is also at **http://xiao.local/** (the name can be changed) or at the IP shown on the page.
  - The XIAO has one radio: its own network moves to the other network's channel when it joins it, so a phone on the XIAO network may lose the connection for a few seconds.
  - `xiao.local` works on iPhone, Mac, Windows 10/11 and Linux; on many Android phones use the IP instead.
  - The XIAO announces its page by mDNS, so AgOpenWeb's web cards (⚙ › **Search the network**) list it in a second when the AgOpenWeb computer is on that network, under the name of its own WiFi network (`XIAO-xxxx`). It is also announced as an AgOpenWeb add-on card (`_aowcard._tcp`, TXT `v=1`), the official format, so the official AgOpenWeb will list it too when its add-on card list ships.
- **The page**, in English, dark or light:
  - the button at the top (☀ / ☾) switches it, and the phone remembers the choice;
  - without a choice, the phone's setting (dark or light mode) decides;
  - when the XIAO restarts (its uptime goes back, or another build runs), the page loads itself again, on the same tab.
  - it goes out without holding up the XIAO: on a weak WiFi link the 53 kB page can take a few seconds, but the link to the Teensy (hello, data) keeps going meanwhile. Before 13 Nov 2026 the XIAO waited for the browser, and the Teensy log showed "XIAO silent for 10 s";
  - the browser keeps the page and asks the XIAO each time whether it changed: the same build answers "not modified" (a few hundred bytes), so only the first visit after an update sends the whole page;
  - a spare connection a browser opens and leaves silent is dropped after 1 s when another one is waiting (it used to hold up the next request for 5 s).

| Tab | What it shows |
|---|---|
| **Status** | **Module (Teensy)**, sent by the Teensy once a second: GNSS (fix, satellites, age of corrections), dual antenna heading, IMU (TM171 or BNO085), the GPS receiver's model, steering, network, the Teensy's clock and temperature. The fix is green whenever the receiver has a position (GPS, DGPS, RTK float, RTK fixed: the name says which) and red only without one. |
| | **Program → module** (named AgOpenWeb or AgOpenGPS when the Teensy knows which program sends the hello): speed, requested steer angle, autosteer on / off, sections (one box each: 16, or up to 64 with PGN 229), hydraulic lift (up / down), tramline, PGN 254 per second. |
| | **XIAO**: uptime, start-up reason, WiFi, the link to the Teensy, memory, the chip's temperature (its version is in the header and the Firmware tab; the requested steer angle is in Program → module). |
| **Messages** | The module's `[status]` messages (the same as the Messages tab of the Teensy page, last 60), and the XIAO's own events (WiFi, phones joining, updates). |
| **WiFi** | The XIAO's network name and password. |
| | The other network: a scan lists the networks found, one per line, strongest first; tap one to choose it. |
| | The name on the network (`xiao` → `xiao.local`). |
| | Saved in the XIAO; it restarts to apply them. |
| | **Module network**: the module's (Teensy's) Ethernet IP and its subnet. For a module on a network that is not 192.168.5.x: type that network's first three numbers (and the XIAO password); the Teensy saves it and restarts with the new IP. This page is on WiFi, so it works whatever the module's IP is. |
| **Firmware** | Version, the newest version on GitHub with **Install from GitHub**, **program update over WiFi** (a `.bin` file, see below), restart. One password field for all. **Module (Teensy) firmware**: the module's firmware installed from this page over the link, and its previous firmware put back (see below). **Download the report**: the XIAO's state and settings (no passwords), the module status and both message lists in one `.json` file, to send when something goes wrong. |

- Changes (WiFi, update, restart) **ask for the password of the XIAO's own network**, to confirm.
  - The question before each one (save and restart, install, the module's firmware, the subnet) opens in a box of the page itself, not a browser pop-up: inside an AgOpenWeb web card the browser's pop-ups show nothing, so those buttons did nothing there.
  - **Enter** in a password field does what its button does, and the question still comes first. On the Firmware tab: the action that asked for the password (*Type the XIAO network password*), else **Install** with a `.bin` chosen, else **Install from GitHub**; never **Restart**. An empty field does nothing, and a held Enter does not also answer the question it opened.
  - The tabs and the light/dark button at the top are 44 px high, with the same 8 px corners as that box's buttons: a size a finger hits on the cab's screen.
  - On the WiFi tab the password and **Save and restart the XIAO** have a card of their own, *Confirm and save*, as wide as the other two.
  - The page carries the date its own program was compiled and compares it with the one the XIAO reports. When they differ (an old page the browser kept after an update), it fetches the page again at an address the browser has never seen (`/?b=<build date>`); if that page is still old, a notice says to reload it by hand.
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
- A newer one shows at the top of Status and in Firmware. **A new build of the same version counts as new**: the page compares when the `.bin` for this XIAO's chip (C3 or S3) on GitHub was compiled with this XIAO's **Built**. That date is read from the file each time it reaches `main` and published with the simulator (`https://renatorgr.github.io/Boards/firmware.json`); without it, the page compares the version number only.
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

If the update fails halfway (WiFi lost), the old program keeps running: just try again. Messages then shows "Program update STOPPED after … kB: the browser's connection dropped". With a weak signal, connect the phone or PC to the XIAO's own network and install from http://192.168.4.1/: download the `.bin` first (Install from GitHub needs internet, and the XIAO's own network has none).

## The module's firmware from this page

For when the module's (Teensy's) own page does not open: its network cable or network, a fault in the page, the box closed, while this XIAO still answers on WiFi. Firmware tab → **Module (Teensy) firmware**.

- The card shows the firmware the module runs and when it was built, the `.hex` on GitHub (newer, the same, or older than the module's), and the **previous firmware** the Teensy keeps (the one that ran before its last update; "none kept yet" until the module's first update with a firmware of 2 Nov 2026 or later).
- **Install from GitHub** downloads the Teensy `.hex` of the newest version folder in the browser (the phone needs internet, as for the XIAO's own update), **Choose .hex** takes a file from the phone or PC; the page reads it first and says what is inside (version, build date, whether it is a Teensy 4.1 AiO_WEB firmware). **Install on the module** sends it to the XIAO with the network password, and the XIAO passes it on to the Teensy over the serial link in pieces, each one confirmed by the Teensy before the next: about a minute for the 2 MB file. The progress bar is the file leaving the phone; then the Teensy checks the whole file as it does one from its own page, installs it and restarts.
- **Go back to the previous firmware** asks the Teensy to put its kept copy back: no file and no internet needed. The copy is used up by it.
- Only with **autosteer off and the tractor stopped**: the Teensy refuses otherwise, and keeps autosteer off while it receives. If anything fails the module's current firmware keeps running and the card says why; a Teensy that does not answer (an older firmware, or the link down) is reported after a few seconds.
- The card then waits for the module to come back (its time on starts again, or another build runs) and says which firmware it runs. The module's own messages are on the Messages tab.
- **Other firmware** installs a `.hex` without the update page inside (another firmware for the Teensy 4.1); the next update is then by USB.
- Not yet tried on a board (2 Nov 2026): try it first on the bench with the box open and the USB cable at hand.

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
| `xLink.h` | the link to the Teensy (460800 baud, frames, status, log, the firmware pieces and the Teensy's answers, PGN 145). Plain C++ so it can be tested on a PC |
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
