# Web page simulator

Try the module's web pages without a module: **https://renatorgr.github.io/Boards/**

These are the real pages of the firmware, the Teensy page (`zWebPage.h`), the XIAO Wi-Fi page (`xPage.h`), the seed drill module's page (`Seeder/Seeder_ESP32/dPage.h`) and the sprayer module's page (`Sprayer/Sprayer_ESP32/sPage.h`), with pretend modules behind them instead of the boards. A tractor drives up and down a field with autosteer on; the GPS gets its RTK fix, the IMU aligns, the machine module opens the 19 sections of the no-till seed drill (or, with the sprayer chosen, its 8 sections, measuring the boom's pressure and flow and working out the L/ha). Nothing is real, but what the pages show and how they behave is what you see on the tractor.

A strip at the top of each page chooses the set-up:

| Choice | Options |
|---|---|
| GPS | UM982 with two antennas, X20P, F9P, dual F9P |
| IMU | TM171, BNO085 on I2C, BNO085 in RVC mode, none |
| Steering | Cytron, IBT2, Danfoss, Keya motor |
| Program | AgOpenWeb, AgOpenGPS |
| XIAO | on the GPS2 port or not (a dual F9P takes that port, so no XIAO then) |
| Machine module | our module program for the implement: Seeder_ESP32 on the seed drill (hello reply and PGN 237 with every open section) or Sprayer_ESP32 on the sprayer (hello reply, PGN 237 and PGN 160 with the readings); the Machine tab then links to its page. Or two of our modules (.123 with sections 1-16, .124 with the rest: the Machine tab shows them together and lists both IPs), the machine part of the AgOpenGPS AiO v26 firmware (PGN 237 with sections 1-16, its hello reply with none), the official AgOpenGPS Machine_UDP_v5 (hello reply only, sections 1-16), or none |
| Implement | Teensy page only: the no-till seed drill with 19 sections, about 3.4 m (the Relay24_ESP32 board on the network cable, 24 relays, relay 1 = section 1 and so on, no pressure or flow sensor, so no Sprayer card on the Teensy page; the Machine tab shows all 19 sections, from the program's PGN 229 and the module's PGN 237), or the 8 m sprayer with 8 sections of 1 m on the ESP32_Relay_X8 board (8 relays, pressure, flow and litres). The seed drill page is always the seed drill, the sprayer page the sprayer |
| Tractor | driving with autosteer on, or parked |
| XIAO chip | ESP32-C3 or ESP32-S3 (XIAO page only) |
| Wi-Fi | module pages only: on the tractor network, the tractor network lost (every section closed, the program's request outlined), or not set yet. On the seed drill's Relay24_ESP32 this is the network cable: plugged in, unplugged, or never plugged in |

With the tractor **parked** you can try what the module refuses while autosteer is on: the firmware update (choose any `.hex` from this repository, or any file to see it refused) and, after it, **Go back to the previous firmware** the module keeps, the TM171 calibrations, writing the receiver settings, the restart. On the XIAO page the **Module (Teensy) firmware** card sends a `.hex` to the pretend module over the link and puts its previous firmware back, as on the tractor. The pretend module goes away for a few seconds and the page reloads itself, as on the tractor. On the XIAO page and the module pages the password is the default, `agopengps`; the seed drill page's network settings, relay setting, restart and program update, and the sprayer page's calibration, Wi-Fi settings, total reset, restart and program update (any `.bin`, or another file to see it refused) all work.

The choices are in the page address (`teensy/?gps=f9p&imu=bno&motor=keya&prog=aog`), so a link opens a given set-up. The state of the pretend module lives in the browser tab and is forgotten when the tab closes.

## Changelog

One row per day, in short; the details are in the sections of this file (the links) and in the git history.

| Date | What changed |
|---|---|
| 8 Nov 2026 | **AgOpenWeb settings from the dimensions**: a calculator page that turns the tractor and implement measures into the U-turn, headland and tramline settings, with the reasons and a drawing ([the page](#agopenweb-settings-from-the-dimensions)). |
| 7 Nov 2026 | The seed drill page's four tabs and Firmware tab checked, with a made-up newer `.bin` "on GitHub"; `firmware.json` lists the seed drill `.bin` once it is on `main`; the modules' names on the network ([How it is built](#how-it-is-built)). |
| 4 Nov 2026 | The Teensy's clock and temperature, the receivers' and the TM171's temperatures and the XIAO's in the pretend modules' answers. |
| 3 Nov 2026 | The pretend Teensy answers the new fields: the distance to the line and the program's steering limits, the RELPOSNED flags, the loop times in the report, the steering loop rate, the frozen-IMU flag, the installation note. |
| 2 Nov 2026 | The sprayer page's tabs, Firmware tab and report checked end to end; the previous firmware the Teensy keeps, put back from its page and from the XIAO page; "Search for networks"; the check waits for the module's answers. |
| 1 Nov 2026 | The sprayer boom is 8 sections of 1 m; the "On GitHub" wording. |
| 31 Oct 2026 | The update check reads the site's own `firmware.json`: a newer build of the same version shows as new (Teensy, XIAO C3 and S3). |
| 28 Oct 2026 | The seed drill's board is the Relay24_ESP32 (24 relays). |
| 27 Oct 2026 | The seed drill module (Seeder_ESP32, 19 sections) with its own page; the AiO v26's machine part and two of our modules on the Machine tab; the sprayer's readings only with its sensors. |

## How it is built

`build.py` cuts the pages out of the newest `Firmware/vX.Y_AiO_WEB` folder, `Seeder/Seeder_ESP32` and `Sprayer/Sprayer_ESP32` exactly as the firmware serves them and puts the simulator scripts in their `<head>` (a module page also gets its implement, `window.SIM_IMPL`). The scripts answer the page's `/api/...` requests (and the firmware upload) instead of the module, so the page's own JavaScript is not changed: what you see is the page as it is in the firmware. The GitHub Action `.github/workflows/simulator.yml` runs on every change under `Firmware/`, `Seeder/Seeder_ESP32/` or `Sprayer/Sprayer_ESP32/`, opens every tab in a browser as a check (`smoke-test.js`), and publishes the result on GitHub Pages when the change reaches `main`. A pull request only builds and checks: the simulator shows the merged firmware.

`build.py` also writes `firmware.json` at the top of the site: the Teensy `.hex` and the two XIAO `.bin` files of the newest version folder, each with the date and time it was compiled, read from the file itself (the `__DATE__ __TIME__` text every program carries). The Teensy page and the XIAO page fetch it from `https://renatorgr.github.io/Boards/firmware.json` for **Install from GitHub**, so a new build of the same version shows as new once its file is on `main` and the site is published again. The pretend Teensy and XIAO run those same builds. It also has the sprayer module's `Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino.bin` (`"sprayer"`): its version, board and build date, read from the text its `/api/status` answer starts with, for the sprayer page's **Firmware** tab; the pretend sprayer module runs that build. The same for the seed drill module's `Seeder/Seeder_ESP32/Seeder_ESP32.ino.bin` (`"seeder"`), once that file is on `main`.

| File | What it does |
|---|---|
| `build.py` | builds the site: `python3 Firmware/simulator/build.py [folder]` (default `_site`) |
| `index.html` | the start page |
| `sim-common.js` | the pretend module: the tractor, the GNSS, the IMU, the steering, the receiver settings, the log, the firmware update, and the strip with the choices |
| `sim-teensy.js` | the Teensy page's requests (zWeb.ino `webDispatch()`) answered by the pretend module |
| `sim-xiao.js` | the XIAO page's requests (v4.5_XIAO_ESP32.ino) answered by a pretend XIAO, with the module behind it |
| `sim-seeder.js` | the seed drill module page's requests (Seeder_ESP32 zNet.ino) answered by a pretend Relay24_ESP32, sowing what the tractor sows |
| `sim-sprayer.js` | the sprayer module page's requests (Sprayer_ESP32 zNet.ino) answered by a pretend ESP32, spraying what the tractor sprays |
| `smoke-test.js` | the check: `node Firmware/simulator/smoke-test.js _site [screenshots]` (needs the `playwright` package) |

To run it on your own computer: `python3 Firmware/simulator/build.py` and open `_site/index.html` in a browser (a `file://` page works). The check needs Node and `npm install playwright && npx playwright install chromium`.

## AgOpenWeb settings from the dimensions

**https://renatorgr.github.io/Boards/aow-settings/** is not a module page: a calculator. Choose the machine (a tractor, an articulated one that bends at its joint, a harvester steered from the back, each with its own turning geometry), type its wheelbase, the angle on the stop and the overhang, and the implement as AgOpenWeb's Tool page has it (front fixed, rear fixed, trailing or TBT, the sections and their width, the overlap, the tool offset, the hitch, trailing hitch and pivot distances, the implement length), and it suggests what to put in AgOpenWeb's Vehicle & Tool page, each value with its reason, and draws the turn from above: the U-turn radius, the turn style and the rows to skip, the distance from boundary, the headland width, the extension, the Go around radius and the tramline passes. Two buttons fill in the 8 m sprayer and the 3 m seed drill.

The rules are AgOpenWeb's own (its `SagittaTurnGeometry`, `YouTurnCreationService` and `VehicleToolLimits`): the turn is drawn by the tractor's pivot, which must stay inside the field edge pulled in by the distance from boundary; a pass closer than twice the radius needs a loop (Sagitta or Keyhole), skipped rows or a K-turn (AgOpenWeb drives a 126° arc and a straight tail, then the tractor is backed up by hand: the turn is over for AgOpenWeb as soon as it reverses, and guidance is on the next pass), a pass at twice the radius or more gets a plain semicircle. The radius is 1.2 × the tightest the machine can do (wheelbase ÷ tan of the steering angle; an articulated machine (wheelbase ÷ 2) ÷ tan of half the articulation angle), never under 4 m; the distance from boundary keeps what sweeps the widest in the turn, the tractor's nose or the implement's outer end, a safety margin inside the edge (1 m by default, a field of the page: more where the edge is irregular); the headland is the turn's depth plus that distance plus the implement behind the axle, in whole passes. What is not geometry (the section timing, the steering gains, the antenna position) is not suggested.

| File | What it does |
|---|---|
| `aow-settings/settings.js` | the arithmetic, with no page: runs in the browser and in Node, and is the model for a C# version on AgOpenWeb's Vehicle & Tool page |
| `aow-settings/index.html` | the form, the table of suggestions and the drawing |
| `aow-settings/test.js` | the check of the arithmetic with known machines: `node Firmware/simulator/aow-settings/test.js`, a step of the Action |

## When the firmware changes

The pages are always current: they come from the firmware's own files. The pretend module is not: when a change in the firmware adds or changes a field in the JSON of `/api/status`, `/api/gps`, `/api/settings`, `/api/log`, `/api/diag` or `/api/um982`, in the XIAO's `/api/state`, `/api/teensy` or `/api/log`, or in the seed drill or sprayer module's `/api/status` or `/api/settings`, the same field has to be added in `sim-common.js` (or `sim-xiao.js`, `sim-seeder.js`, `sim-sprayer.js`), or the page shows a dash there. The check in the Action catches a page that breaks, not a value that is missing.

## Licence

GNU General Public License v3 or later, as the firmware (`Firmware/v4.5_AiO_WEB/COPYING`). Written 2026 by Claude (Anthropic) for renatorgr.
