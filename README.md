# Boards

Hardware and firmware for the AgOpenGPS **AiO v4.5** autosteer boards (Micro and Standard), plus a section controller for a field sprayer. Everything here works with [AgOpenWeb](https://github.com/AgOpenGPS-Official/AgOpenWeb) and [AgOpenGPS](https://github.com/AgOpenGPS-Official/AgOpenGPS).

> [!WARNING]
> This firmware steers a tractor. Test every new version on the bench before you use it in the field. It comes with no warranty.

## What's inside

| Folder | What it is |
|---|---|
| [`Firmware/v4.5_AiO_WEB`](Firmware/v4.5_AiO_WEB) | Teensy 4.1 firmware for the AiO v4.5 Micro and Standard. Handles the GPS (UM982, F9P, X20P or dual F9P), the IMU (TM171 or BNO085) and autosteer (Cytron, IBT2, Danfoss or Keya). Has its own web page for status, settings and firmware updates over the network. |
| [`Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32`](Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32) | Program for a Seeed XIAO ESP32C3 or ESP32S3 on the GPS2 port. The XIAO runs its own Wi-Fi network, so you can check the module from your phone without the tablet. |
| [`Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin`](Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin) | The XIAO program, compiled and ready to install. |
| [`Firmware/simulator`](Firmware/simulator) | The web pages of the Teensy, the XIAO and the ESP32 sprayer module with pretend boards behind them, to try online without a board: **[renatorgr.github.io/Boards](https://renatorgr.github.io/Boards/)**. Rebuilt from the code on every change. |
| [`AiO_Micro45_XIAO_C3`](AiO_Micro45_XIAO_C3) | Adapter board (KiCad) that plugs a XIAO into the GPS2 socket of the AiO v4.5 Micro. Gerber, BOM and CPL files are ready to order from JLCPCB. |
| [`AiO_Std45_XIAO_C3`](AiO_Std45_XIAO_C3) | The same adapter for the AiO v4.5 Standard. |
| [`LED_Bezel`](LED_Bezel) | A small 3D-printed comb (OpenSCAD and STL) that slides over the three front LEDs of the AiO v4.5 so each one lights only its own window on the panel. |
| [`Sprayer/Sprayer_ESP32`](Sprayer/Sprayer_ESP32) | Program for an ESP32 8-relay board that runs a 5-section sprayer over Wi-Fi. Opens and closes the section valves on command from AgOpenWeb or AgOpenGPS (the official machine-module messages), reports boom pressure and flow, and has its own web page for setup and updates. |
| [`Sprayer/Sprayer_Nano`](Sprayer/Sprayer_Nano) | The earlier Arduino Nano + Ethernet version of the same sprayer controller. |

Each folder has its own README with the details.

## Status

- **Teensy firmware v4.5:** runs on the bench with a UM982, F9P or X20P and a TM171 or BNO085. The dual F9P, the Keya motor and steering with a BNO085 have not been tried on a tractor yet.
- **XIAO program v4.5:** must match the Teensy firmware version.
- **Adapter boards:** designed and checked in KiCad, but not built or tested yet.
- **Sprayer ESP32 v0.5:** runs on the bench (page, Wi-Fi), not tested on the sprayer yet. The Nano version (v0.3) is kept as it was.

## Getting started

### Teensy firmware

1. **First install:** connect the Teensy over USB, open the `.hex` file from [`Firmware/v4.5_AiO_WEB`](Firmware/v4.5_AiO_WEB) in Teensy Loader, then press the button on the Teensy.
2. **Web page:** open `http://192.168.5.126/` in a browser on the same network. The third number follows the subnet set in AgOpenWeb; it can also be changed from the XIAO's WiFi page (WiFi tab → Module network). To see the page before you have a module: the [simulator](https://renatorgr.github.io/Boards/).
3. **Updates:** on the web page, go to **Firmware** and choose a new `.hex` file, or click **Install from GitHub**. The page checks this repository for new versions.

### XIAO

1. **First install:** over USB-C, using the `…_USB.bin` file for your XIAO and the [esptool-js](https://espressif.github.io/esptool-js/) web page. Step-by-step instructions are in the [`v4.5_XIAO_ESP32_bin` README](Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin).
2. Join the `XIAO-xxxx` Wi-Fi network (password `agopengps`) and open `http://192.168.4.1/`.
3. **Updates:** over Wi-Fi from the XIAO page.

### Sprayer

Wiring, the first upload, the Wi-Fi setup and a bench test are in the [`Sprayer_ESP32` README](Sprayer/Sprayer_ESP32). Its web page can be tried without the board in the [simulator](https://renatorgr.github.io/Boards/sprayer/). The pressure, flow and alarms show in the [AgOpenWeb fork with the sprayer readout](https://github.com/renatorgr/AgOpenWeb). The older Nano version is in [`Sprayer_Nano`](Sprayer/Sprayer_Nano).

## Building from source

- Arduino IDE 1.8 or 2.x with **Teensyduino 1.58**.
- Board settings: Teensy 4.1, USB Type "Serial", CPU Speed 600 MHz, Optimize "Faster".
- No extra libraries needed: everything the sketch uses is in its folder or comes with Teensyduino.
- GitHub Actions compiles the Teensy firmware on every pull request. A green check means it builds. The `.hex` file is under **Artifacts** on the run's page.

## Credits and licence

This work builds on the AgOpenGPS project and its community. The full list of credits is in the [Teensy firmware README](Firmware/v4.5_AiO_WEB/README.md#credits-and-licence).

- **Teensy and XIAO firmware:** GNU General Public License v3 or later. See [`COPYING`](Firmware/v4.5_AiO_WEB/COPYING).
- **Adapter boards:** same licence as the AgOpenGPS boards they fit.
