# Web page simulator

Try the module's web pages without a module: **https://renatorgr.github.io/Boards/**

These are the real pages of the firmware, the Teensy page (`zWebPage.h`), the XIAO Wi-Fi page (`xPage.h`), the seed drill module's page (`Seeder/Seeder_ESP32/dPage.h`) and the sprayer module's page (`Sprayer/Sprayer_ESP32/sPage.h`), with pretend modules behind them instead of the boards. A tractor drives up and down a field with autosteer on; the GPS gets its RTK fix, the IMU aligns, the machine module opens the 19 sections of the no-till seed drill (or, with the sprayer chosen, its 5 sections, measuring the boom's pressure and flow). Nothing is real, but what the pages show and how they behave is what you see on the tractor.

A strip at the top of each page chooses the set-up:

| Choice | Options |
|---|---|
| GPS | UM982 with two antennas, X20P, F9P, dual F9P |
| IMU | TM171, BNO085 on I2C, BNO085 in RVC mode, none |
| Steering | Cytron, IBT2, Danfoss, Keya motor |
| Program | AgOpenWeb, AgOpenGPS |
| XIAO | on the GPS2 port or not (a dual F9P takes that port, so no XIAO then) |
| Machine module | our module program for the implement: Seeder_ESP32 on the seed drill (hello reply and PGN 237 with every open section) or Sprayer_ESP32 on the sprayer (hello reply, PGN 237 and PGN 160 with the readings); the Machine tab then links to its page. Or two of our modules (.123 with sections 1-16, .124 with the rest: the Machine tab shows them together and lists both IPs), the machine part of the AgOpenGPS AiO v26 firmware (PGN 237 with sections 1-16, its hello reply with none), the official AgOpenGPS Machine_UDP_v5 (hello reply only, sections 1-16), or none |
| Implement | Teensy page only: the no-till seed drill with 19 sections, about 3.4 m (a KinCony KC868-A32 on the network cable, 32 relays, relay 1 = section 1 and so on, no pressure or flow sensor, so no Sprayer card on the Teensy page; the Machine tab shows all 19 sections, from the program's PGN 229 and the module's PGN 237), or the 8 m sprayer with 5 sections on the ESP32_Relay_X8 board (8 relays, pressure, flow and litres). The seed drill page is always the seed drill, the sprayer page the sprayer |
| Tractor | driving with autosteer on, or parked |
| XIAO chip | ESP32-C3 or ESP32-S3 (XIAO page only) |
| Wi-Fi | module pages only: on the tractor network, the tractor network lost (every section closed, the program's request outlined), or not set yet. On the seed drill's KC868-A32 this is the network cable: plugged in, unplugged, or never plugged in |

With the tractor **parked** you can try what the module refuses while autosteer is on: the firmware update (choose any `.hex` from this repository, or any file to see it refused), the TM171 calibrations, writing the receiver settings, the restart. The pretend module goes away for a few seconds and the page reloads itself, as on the tractor. On the XIAO page and the module pages the password is the default, `agopengps`; the seed drill page's network settings, relay setting, restart and program update, and the sprayer page's calibration, Wi-Fi settings, total reset, restart and program update (any `.bin`, or another file to see it refused) all work.

The choices are in the page address (`teensy/?gps=f9p&imu=bno&motor=keya&prog=aog`), so a link opens a given set-up. The state of the pretend module lives in the browser tab and is forgotten when the tab closes.

## How it is built

`build.py` cuts the pages out of the newest `Firmware/vX.Y_AiO_WEB` folder, `Seeder/Seeder_ESP32` and `Sprayer/Sprayer_ESP32` exactly as the firmware serves them and puts the simulator scripts in their `<head>` (a module page also gets its implement, `window.SIM_IMPL`). The scripts answer the page's `/api/...` requests (and the firmware upload) instead of the module, so the page's own JavaScript is not changed: what you see is the page as it is in the firmware. The GitHub Action `.github/workflows/simulator.yml` runs on every change under `Firmware/`, `Seeder/Seeder_ESP32/` or `Sprayer/Sprayer_ESP32/`, opens every tab in a browser as a check (`smoke-test.js`), and publishes the result on GitHub Pages when the change reaches `main`. A pull request only builds and checks: the simulator shows the merged firmware.

| File | What it does |
|---|---|
| `build.py` | builds the site: `python3 Firmware/simulator/build.py [folder]` (default `_site`) |
| `index.html` | the start page |
| `sim-common.js` | the pretend module: the tractor, the GNSS, the IMU, the steering, the receiver settings, the log, the firmware update, and the strip with the choices |
| `sim-teensy.js` | the Teensy page's requests (zWeb.ino `webDispatch()`) answered by the pretend module |
| `sim-xiao.js` | the XIAO page's requests (v4.5_XIAO_ESP32.ino) answered by a pretend XIAO, with the module behind it |
| `sim-seeder.js` | the seed drill module page's requests (Seeder_ESP32 zNet.ino) answered by a pretend KC868-A32, sowing what the tractor sows |
| `sim-sprayer.js` | the sprayer module page's requests (Sprayer_ESP32 zNet.ino) answered by a pretend ESP32, spraying what the tractor sprays |
| `smoke-test.js` | the check: `node Firmware/simulator/smoke-test.js _site [screenshots]` (needs the `playwright` package) |

To run it on your own computer: `python3 Firmware/simulator/build.py` and open `_site/index.html` in a browser (a `file://` page works). The check needs Node and `npm install playwright && npx playwright install chromium`.

## When the firmware changes

The pages are always current: they come from the firmware's own files. The pretend module is not: when a change in the firmware adds or changes a field in the JSON of `/api/status`, `/api/gps`, `/api/settings`, `/api/log`, `/api/diag` or `/api/um982`, in the XIAO's `/api/state`, `/api/teensy` or `/api/log`, or in the seed drill or sprayer module's `/api/status` or `/api/settings`, the same field has to be added in `sim-common.js` (or `sim-xiao.js`, `sim-seeder.js`, `sim-sprayer.js`), or the page shows a dash there. The check in the Action catches a page that breaks, not a value that is missing.

## Licence

GNU General Public License v3 or later, as the firmware (`Firmware/v4.5_AiO_WEB/COPYING`). Written 2026 by Claude (Anthropic) for renatorgr.
