# Web page simulator

Try the module's web pages without a module: **https://renatorgr.github.io/Boards/**

These are the real pages of the firmware. Pretend modules answer them instead of the boards:

- the Teensy page (`zWebPage.h`)
- the XIAO Wi-Fi page (`xPage.h`)
- the seed drill module's page (`Seeder/Seeder_ESP32/dPage.h`)
- the sprayer module's page (`Sprayer/Sprayer_ESP32/sPage.h`)

What you see:

- A tractor drives up and down a field with autosteer on.
- The GPS gets its RTK fix and the IMU aligns.
- The machine module opens the 19 sections of the no-till seed drill.
- With the sprayer chosen, it opens its 8 sections, measures the boom's pressure and flow, and works out the L/ha.

Nothing is real, but the pages show and behave as they do on the tractor.

### Choosing the set-up

A strip at the top of each page chooses the set-up:

| Choice | Options |
|---|---|
| GPS | UM982 with two antennas, X20P, F9P, dual F9P |
| IMU | TM171, BNO085 on I2C, BNO085 in RVC mode, none |
| Steering | Cytron, IBT2, Danfoss, Keya motor |
| Program | AgOpenWeb, AgOpenGPS |
| XIAO | on the GPS2 port, or not (a dual F9P takes that port, so no XIAO then) |
| Machine module | see the list below |
| Implement | Teensy page only: the seed drill or the sprayer, see below |
| Tractor | driving with autosteer on, or parked |
| XIAO chip | ESP32-C3 or ESP32-S3 (XIAO page only) |
| Wi-Fi | module pages only, see below |

**Machine module**:

- Seeder_ESP32 on the seed drill: our module program. It sends the hello reply and PGN 237 with every open section.
- Sprayer_ESP32 on the sprayer: the hello reply, PGN 237 and PGN 160 with the readings.
- With either of these two, the Machine tab links to the module's page.
- Two of our modules: .123 with sections 1-16, .124 with the rest. The Machine tab shows them together and lists both IPs.
- The machine part of the AgOpenGPS AiO v26 firmware: PGN 237 with sections 1-16, its hello reply with none.
- The official AgOpenGPS Machine_UDP_v5: hello reply only, sections 1-16.
- None.

**Implement** (Teensy page only):

- The no-till seed drill: 19 sections, about 3.4 m.
  - It runs on the Relay24_ESP32 board, on the network cable: 24 relays, relay 1 = section 1 and so on.
  - It has no pressure or flow sensor, so the Teensy page shows no Sprayer card.
  - The Machine tab shows all 19 sections, from the program's PGN 229 and the module's PGN 237.
- The 8 m sprayer: 8 sections of 1 m on the ESP32_Relay_X8 board (8 relays, pressure, flow and litres).

The seed drill page is always the seed drill, and the sprayer page the sprayer.

**Wi-Fi** (module pages only):

- on the tractor network
- the tractor network lost: every section closed, the program's request outlined
- not set yet

On the seed drill's Relay24_ESP32 this choice is the network cable: plugged in, unplugged, or never plugged in.

### With the tractor parked

The module refuses some things while autosteer is on. With the tractor **parked** you can try them:

- the firmware update: choose any `.hex` from this repository, or any other file to see it refused
- after the update, **Go back to the previous firmware** the module keeps
- the TM171 calibrations
- writing the receiver settings
- the restart

On the XIAO page, the **Module (Teensy) firmware** card sends a `.hex` to the pretend module over the link. It also puts the previous firmware back, as on the tractor.

The pretend module goes away for a few seconds and the page reloads itself, as on the tractor.

### The module pages

On the XIAO page and the module pages the password is the default, `agopengps`.

Everything on the module pages works:

- the seed drill page: network settings, relay setting, restart and program update
- the sprayer page: calibration, Wi-Fi settings, total reset, restart and program update (any `.bin`, or another file to see it refused)

### Links

- The choices are in the page address, for example `teensy/?gps=f9p&imu=bno&motor=keya&prog=aog`. So a link opens a given set-up.
- The pretend module's state lives in the browser tab. It is forgotten when the tab closes.

## Changelog

One row per day, in short; the details are in the sections of this file (the links) and in the git history.

| Date | What changed |
|---|---|
| 24 Nov 2026 | The pretend XIAO sends `memLow` (never short of memory), for the XIAO page's lowest point of free memory. The pretend XIAO, sprayer and seed drill answer `/api/update-check`; the check tries an install with a wrong password on each. |
| 22 Nov 2026 | The XIAO page's WiFi bridge: its switch (refused with the XIAO's starting password), its rows on both pages; the pretend program is on the network cable, so a bridge switched on stands by. Module pages: the own network's address. |
| 21 Nov 2026 | **Set up AgOpenWeb from your measures** (version 1.13): settings in AgOpenWeb's order; Smoothing row; a hard field edge gives the margin as distance from boundary ([the page](#set-up-agopenweb-from-your-measures)). |
| 15 Nov 2026 | **Set up AgOpenWeb from your measures** (version 1.11): Renato's centre-pivot mower with his measures, onto the next pass (the tractor on the cut strip), swung across to the other side as the pass ends: a 4 m turn ([the page](#set-up-agopenweb-from-your-measures)). |
| 13 Nov 2026 | **Set up AgOpenWeb from your measures** (version 1.8): measured noses (Maxxum 1 m, Farmall 0.85 m, Puma 1.1 m), the boom 2.1 m ahead, Renato's one-axle manure spreader, his centre-pivot mower ([the page](#set-up-agopenweb-from-your-measures)). |
| 12 Nov 2026 | **Set up AgOpenWeb from your measures** (version 1.5): buttons for Renato's Farmall 115C, Maxxum 135 and Puma 180; a drawbar implement takes the tractor's pin ([the page](#set-up-agopenweb-from-your-measures)). [Action](#how-it-is-built) on `ubuntu-26.04`. |
| 11 Nov 2026 | `build.py` leaves the pages' own build date (`PAGE_BUILT`) empty, so the pages' new "is this the running firmware's page" check stays off here. The pretend Keya reports its current in amps, the kickout sensor. |
| 10 Nov 2026 | Teensy page: the pretend UM982 reports its temperature (59 °C, from `HWSTATUSA`) and the module's name on the network reads `AiO-3F2A`, as the firmware now does. |
| 9 Nov 2026 | **Set up AgOpenWeb from your measures** 1.3: front corner, drawbar at the tyre, turns by headland, reasons ([the page](#set-up-agopenweb-from-your-measures)). Seed drill: 19 sections offline; short start page. [Action](#how-it-is-built) on Node 24. Check: pages in a web card's frame ([How it is built](#how-it-is-built)). |
| 8 Nov 2026 | **Set up AgOpenWeb from your measures**: a calculator that turns the tractor and implement measures into the U-turn, headland and tramline settings, with the reasons, a drawing, examples and drawbars kept off the rear tyres ([the page](#set-up-agopenweb-from-your-measures)). |
| 7 Nov 2026 | The seed drill page's four tabs and Firmware tab checked, with a made-up newer `.bin` "on GitHub"; `firmware.json` lists the seed drill `.bin` once it is on `main`; the modules' names on the network ([How it is built](#how-it-is-built)). |
| 4 Nov 2026 | The Teensy's clock and temperature, the receivers' and the TM171's temperatures and the XIAO's in the pretend modules' answers. |
| 3 Nov 2026 | The pretend Teensy answers the new fields: the distance to the line and the program's steering limits, the RELPOSNED flags, the loop times in the report, the steering loop rate, the frozen-IMU flag, the installation note. |
| 2 Nov 2026 | The sprayer page's tabs, Firmware tab and report checked end to end; the previous firmware the Teensy keeps, put back from its page and from the XIAO page; "Search for networks"; the check waits for the module's answers. |
| 1 Nov 2026 | The sprayer boom is 8 sections of 1 m; the "On GitHub" wording. |
| 31 Oct 2026 | The update check reads the site's own `firmware.json`: a newer build of the same version shows as new (Teensy, XIAO C3 and S3). |
| 28 Oct 2026 | The seed drill's board is the Relay24_ESP32 (24 relays). |
| 27 Oct 2026 | The seed drill module (Seeder_ESP32, 19 sections) with its own page; the AiO v26's machine part and two of our modules on the Machine tab; the sprayer's readings only with its sensors. |

## How it is built

`build.py` makes the site:

1. It cuts the pages out of the newest `Firmware/vX.Y_AiO_WEB` folder, `Seeder/Seeder_ESP32` and `Sprayer/Sprayer_ESP32`, exactly as the firmware serves them.
2. It puts the simulator scripts in each page's `<head>`. A module page also gets its implement (`window.SIM_IMPL`).
3. The scripts answer the page's `/api/...` requests (and the firmware upload) instead of the module.

The page's own JavaScript is not changed: what you see is the page as it is in the firmware.

### The GitHub Action

The Action `.github/workflows/simulator.yml`:

- runs on every change under `Firmware/`, `Seeder/Seeder_ESP32/` or `Sprayer/Sprayer_ESP32/`
- opens every tab in a browser as a check (`smoke-test.js`)
- publishes the result on GitHub Pages when the change reaches `main`

A pull request only builds and checks. The simulator shows the merged firmware.

Its versions:

- Since 9 Nov 2026 it uses the versions of GitHub's own actions that run on Node 24: `checkout@v7`, `setup-node@v7`, `upload-artifact@v7`, `upload-pages-artifact@v5`, `deploy-pages@v5`. The runners now run every action on Node 24 and warn about one built for Node 20.
- Since 12 Nov 2026 every job of the repository's Actions names its runner, `ubuntu-26.04`, instead of `ubuntu-latest`. GitHub moved `ubuntu-latest` to Ubuntu 26.04 that autumn, with a notice on every run.

### firmware.json

`build.py` also writes `firmware.json` at the top of the site. It lists:

- The Teensy `.hex` and the two XIAO `.bin` files of the newest version folder, each with the date and time it was compiled. The date is read from the file itself (the `__DATE__ __TIME__` text every program carries).
- The sprayer module's `Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino.bin` (`"sprayer"`): its version, board and build date, read from the text its `/api/status` answer starts with. It is for the sprayer page's **Firmware** tab.
- The seed drill module's `Seeder/Seeder_ESP32/Seeder_ESP32.ino.bin` (`"seeder"`), the same way, once that file is on `main`.

The Teensy page and the XIAO page fetch it from `https://renatorgr.github.io/Boards/firmware.json` for **Install from GitHub**. So a new build of the same version shows as new once its file is on `main` and the site is published again.

The pretend Teensy, XIAO and sprayer module run those same builds.

### The files

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

`smoke-test.js` also opens the sprayer and Teensy pages inside a frame with the sandbox of an AgOpenWeb web card (`/Boards/_card`, served by the check only). There the browser's own pop-ups show nothing, so:

- the pages' own question box must open, fit 480 × 320 and answer
- a browser pop-up on any page fails the check

### Running it on your own computer

1. Run `python3 Firmware/simulator/build.py`.
2. Open `_site/index.html` in a browser (a `file://` page works).

The check needs Node and `npm install playwright && npx playwright install chromium`.

## Set up AgOpenWeb from your measures

**https://renatorgr.github.io/Boards/aow-settings/** is not a module page: it is a calculator.

### What you type

1. The machine. Each kind has its own turning geometry:
   - a tractor
   - an articulated machine, which bends at its joint
   - a harvester, steered from the back
2. Its wheelbase, the steering angle on the stop and the overhang.
3. The implement, as AgOpenWeb's Tool page has it:
   - front fixed, rear fixed, trailing or TBT
   - the sections and their width, the overlap, the tool offset
   - the hitch, trailing hitch and pivot distances
   - the implement length

### What you get

- What to put in AgOpenWeb's Vehicle & Tool page, each value with its reason.
- A drawing of the turn from above.

The values come in AgOpenWeb's order, so you can type them in from top to bottom:

1. the hitch to working point of a front implement
2. the U-Turn tab, top to bottom: turn style and the rows to skip, extension, radius, distance from boundary, Going around obstacles
3. the tramline passes
4. the headland width

### Renato's machines

Three tractor buttons, by power:

| Tractor | Wheelbase | Steering angle | Other measures |
|---|---|---|---|
| Case IH Farmall 115C | 2.3 m | about 45° | 540/65 R34 tyres; the drawbar pin 0.74 m behind the rear axle |
| Maxxum 135 | 2.7 m | 42° | |
| Puma 180 CVX | 2.88 m | 33° (from its manual's turning circle) | 2 m track; 650/65 R42 tyres; the pin 0.9 m back; the front quick hitch 1.1 m ahead of the front axle |

An implement on the drawbar moves to the chosen tractor's pin.

Implement buttons:

- **8 m sprayer** on the front linkage, the boom 2.1 m ahead of the front axle. The page opens with it on the Maxxum.
- **3 m no-till drill**, semi-mounted: its pivot behind the lower links, its wheels behind the discs.
- **Manure spreader**: one axle 4.2 m behind the drawbar pin. Its body ends 1.5 m past the axle.
- **3.2 m centre-pivot mower conditioner** ([below](#the-mower-conditioner)).
- **Quick disc harrow** on the 3-point linkage and **semi-mounted roller** (120° of swing in all). Their measures are guessed until Renato takes them.

Two more buttons have typical measures:

- a trailed disc harrow
- a 4-furrow reversible plough on the 3-point linkage, drawn with a K-turn, since a plough skips no rows

### The mower conditioner

- It is on the lower links, its swivel 1.55 m behind the Farmall's rear axle (0.5 m behind the balls).
- It mows 25° to one side, just outside the rear tyre, and the other side every pass.
- Its button also presses the Farmall it works with.
- The tractor drives on the strip it has just mown. So the turn is drawn as a Keyhole onto the next pass, no skipped rows, and the tool offset stays 0.
- As the pass ends it is lifted and put straight away on the other side, outside the turn. So the page follows it through the turn instead of round a steady circle.
- Its tongue stays off the tyres even at 4 m. Held in line, it would touch one.

### Long implements and drawbars

A long implement's rear corner (its length) is followed through the turn too. A K-turn backs it towards the edge first.

A trailed implement's drawbar must stay off the rear tyres:

- From the hitch pin, the drawbar's width, the rear track and the tyres' width and height, the page finds the angle where the drawbar touches a tyre.
- It keeps the turn 5° under that angle.
- The tyre counts at the drawbar's height (a field of the page, 0.5 m by default). There the tyre is shorter than at its axle, so the drawbar touches it later than the whole wheel would say.
- A drawbar pin inside the tyres' back edge touches them long before 90°. Then the drawbar sets the radius.
- A semi-mounted implement's pivot is further back and does not touch them. But its own stop may: a field takes the most the drawbar may turn to each side, from its stop or from what the farmer knows.

### Measuring a rigid implement

- Measure it with a tape from the axle nearest to it (a front sprayer from the front axle).
- AgOpenWeb measures it from the pivot. So for a front implement on a tractor, the page adds the wheelbase and shows the hitch length to type in.

### The rules

The rules are AgOpenWeb's own (its `SagittaTurnGeometry`, `YouTurnCreationService` and `VehicleToolLimits`).

**The turn**

- The turn is drawn by the tractor's pivot. The pivot must stay inside the field edge, pulled in by the distance from boundary.
- A pass at twice the radius or more gets a plain semicircle.
- A pass closer than twice the radius needs a loop (Sagitta or Keyhole), skipped rows or a K-turn.

**The K-turn**

- AgOpenWeb drives a 126° arc and a straight tail.
- As soon as the tractor reverses, the turn is over for AgOpenWeb. Its guidance is then on the next pass, backing up too.
- So the tractor backs up until it is on that pass and straight.
- A trailed implement that would jackknife backing up gets no K-turn.

**The radius**

- It is 1.2 × the tightest the machine can do, never under 4 m.
- The tightest is the wheelbase ÷ tan of the steering angle.
- For an articulated machine it is (wheelbase ÷ 2) ÷ tan of half the articulation angle.

**The distance from boundary**

- It keeps whatever sweeps the widest in the turn a safety margin inside the edge. That is either:
  - the outer corner of the tractor's nose (half the machine's width, from its rear track and tyres, out from the centre line), or
  - the implement's outer end.
- The safety margin is a field of the page, 1 m by default. Give more where the edge is irregular.
- A hard field edge is a fence, a wall or a ditch, marked hard in AgOpenWeb. There AgOpenWeb itself moves the turn in until the implement is the distance from boundary away. So the page suggests the margin alone, plus how much further the tractor's nose reaches, which AgOpenWeb does not check.

**The headland**

- Each turn is followed step by step every 10 cm, with the implement and the machine's far end. A trailed implement follows its hitch without side slip.
- The headland is, in whole passes:
  - the turn's depth,
  - plus the distance from boundary,
  - plus how far back into the field anything not yet straight on a pass reaches: the implement finishing its pass, a front or trailed implement until it is straight on the next pass, a K-turn's tail.

**The drawing** shows the pivot's path, both ends of the implement and the outer corner of the machine's far end.

**Not suggested**: what is not geometry (the section timing, the steering gains, the antenna position).

### Its files

| File | What it does |
|---|---|
| `aow-settings/settings.js` | the arithmetic, with no page: runs in the browser and in Node, and is the model for a C# version on AgOpenWeb's Vehicle & Tool page |
| `aow-settings/index.html` | the form, the table of suggestions, the list of turns, the drawing and its legend |
| `aow-settings/test.js` | the check of the arithmetic with known machines: `node Firmware/simulator/aow-settings/test.js`, a step of the Action |

The version under the page's title is `VERSION` at the top of `settings.js`:

- Raise it with every change, together with the `settings.js?v=` address in the page.
- Then a browser fetches the new file with the new page.
- `test.js` checks the two agree.

## When the firmware changes

The pages are always current: they come from the firmware's own files.

The pretend module is not. A firmware change may add or change a field in the JSON of:

- `/api/status`, `/api/gps`, `/api/settings`, `/api/log`, `/api/diag` or `/api/um982`
- the XIAO's `/api/state`, `/api/teensy` or `/api/log`
- the seed drill or sprayer module's `/api/status` or `/api/settings`

Then add the same field in `sim-common.js` (or `sim-xiao.js`, `sim-seeder.js`, `sim-sprayer.js`). Otherwise the page shows a dash there.

The check in the Action catches a page that breaks, not a value that is missing.

## Licence

GNU General Public License v3 or later, as the firmware (`Firmware/v4.5_AiO_WEB/COPYING`). Written 2026 by Claude (Anthropic) for renatorgr.
