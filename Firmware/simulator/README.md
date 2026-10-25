# Web page simulator

Try the module's web pages without a module: **https://renatorgr.github.io/Boards/**

These are the real pages of the firmware, the Teensy page (`zWebPage.h`), the XIAO Wi-Fi page (`xPage.h`) and the ESP32 sprayer module's page (`Sprayer/Sprayer_ESP32/sPage.h`), with pretend modules behind them instead of the boards. A tractor drives up and down a field with autosteer on; the GPS gets its RTK fix, the IMU aligns, the sprayer module opens its sections and measures the boom. Nothing is real, but what the pages show and how they behave is what you see on the tractor.

A strip at the top of each page chooses the set-up:

| Choice | Options |
|---|---|
| GPS | UM982 with two antennas, X20P, F9P, dual F9P |
| IMU | TM171, BNO085 on I2C, BNO085 in RVC mode, none |
| Steering | Cytron, IBT2, Danfoss, Keya motor |
| Program | AgOpenWeb, AgOpenGPS |
| XIAO | on the GPS2 port or not (a dual F9P takes that port, so no XIAO then) |
| Machine module | the sprayer module (ESP32 or Nano: hello reply, PGN 237 and PGN 160 with the readings; the Machine tab then links to the sprayer module's page), the official AgOpenGPS Machine_UDP_v5 (hello reply only), or none |
| Tractor | driving with autosteer on, or parked |
| XIAO chip | ESP32-C3 or ESP32-S3 (XIAO page only) |
| Wi-Fi | sprayer page only: on the tractor network, the tractor network lost (every section closed, the program's request outlined), or not set yet |

With the tractor **parked** you can try what the module refuses while autosteer is on: the firmware update (choose any `.hex` from this repository, or any file to see it refused), the TM171 calibrations, writing the receiver settings, the restart. The pretend module goes away for a few seconds and the page reloads itself, as on the tractor. On the XIAO page and the sprayer page the password is the default, `agopengps`; the sprayer page's calibration, Wi-Fi settings, total reset, restart and program update (any `.bin`, or another file to see it refused) all work.

The choices are in the page address (`teensy/?gps=f9p&imu=bno&motor=keya&prog=aog`), so a link opens a given set-up. The state of the pretend module lives in the browser tab and is forgotten when the tab closes.

## How it is built

`build.py` cuts the pages out of the newest `Firmware/vX.Y_AiO_WEB` folder and out of `Sprayer/Sprayer_ESP32` exactly as the firmware serves them and puts the simulator scripts in their `<head>`. The scripts answer the page's `/api/...` requests (and the firmware upload) instead of the module, so the page's own JavaScript is not changed: what you see is the page as it is in the firmware. The GitHub Action `.github/workflows/simulator.yml` runs on every change under `Firmware/` or `Sprayer/Sprayer_ESP32/`, opens every tab in a browser as a check (`smoke-test.js`), and publishes the result on GitHub Pages when the change reaches `main`. A pull request only builds and checks: the simulator shows the merged firmware.

The pages of the site count their visits with [GoatCounter](https://www.goatcounter.com/) (the tag is `COUNTER` in `build.py`): no cookies, nothing that says who the visitor is, only the page (without the choices) and the time. The counter is in the simulator only; the firmware's own pages on the module have none.

| File | What it does |
|---|---|
| `build.py` | builds the site: `python3 Firmware/simulator/build.py [folder]` (default `_site`) |
| `index.html` | the start page |
| `sim-common.js` | the pretend module: the tractor, the GNSS, the IMU, the steering, the receiver settings, the log, the firmware update, and the strip with the choices |
| `sim-teensy.js` | the Teensy page's requests (zWeb.ino `webDispatch()`) answered by the pretend module |
| `sim-xiao.js` | the XIAO page's requests (v4.5_XIAO_ESP32.ino) answered by a pretend XIAO, with the module behind it |
| `sim-sprayer.js` | the sprayer module page's requests (Sprayer_ESP32 zNet.ino) answered by a pretend ESP32, spraying what the tractor sprays |
| `smoke-test.js` | the check: `node Firmware/simulator/smoke-test.js _site [screenshots]` (needs the `playwright` package) |

To run it on your own computer: `python3 Firmware/simulator/build.py` and open `_site/index.html` in a browser (a `file://` page works). The check needs Node and `npm install playwright && npx playwright install chromium`.

## When the firmware changes

The pages are always current: they come from the firmware's own files. The pretend module is not: when a change in the firmware adds or changes a field in the JSON of `/api/status`, `/api/gps`, `/api/settings`, `/api/log`, `/api/diag` or `/api/um982`, in the XIAO's `/api/state`, `/api/teensy` or `/api/log`, or in the sprayer module's `/api/status` or `/api/settings`, the same field has to be added in `sim-common.js` (or `sim-xiao.js`, `sim-sprayer.js`), or the page shows a dash there. The check in the Action catches a page that breaks, not a value that is missing.

## Licence

GNU General Public License v3 or later, as the firmware (`Firmware/v4.5_AiO_WEB/COPYING`). Written 2026 by Claude (Anthropic) for renatorgr.
