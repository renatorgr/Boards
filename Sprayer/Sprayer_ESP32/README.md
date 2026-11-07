# Sprayer_ESP32

Section control for an 8 m sprayer boom with 8 sections of 1 m, driven by **AgOpenWeb** or
**AgOpenGPS** over Wi-Fi, on an **ESP32 8-relay board**. It also measures boom
pressure and flow, and has its own web page.

Version 0.10.1 (successor of [Sprayer_Nano](../Sprayer_Nano) v0.3): runs on the bench, not yet
tested on the sprayer.

## Changelog

One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history and at the top of each file.

| Date | What changed |
|---|---|
| 7 Nov 2026 | **0.10.2**: the page is announced as an AgOpenWeb add-on card (`_aowcard._tcp`) under its Wi-Fi name; `_aow-ui` and `/aow/status` are gone ([Notes](#notes)). **0.10.3**: the alarms also as PGN 221, shown on the program's screen ([Messages](#messages)). |
| 2 Nov 2026 | **0.10**: rate (L/ha), target pressure and alarms worked out on the module and sent to the program (PGN 161); the page gets Spraying, [Settings](#setup), Module and Firmware tabs, with Install from GitHub. **0.10.1**: Wi-Fi settings as on the XIAO page. |
| 27 Oct 2026 | **0.7.1** sensor dividers from a common resistor kit. **0.7.2** the page says which sensors there are. **0.8** up to 64 sections, relays from any section. **0.9** sprayer boards only (the seed drill got its own folder). **0.9.1** PGN 237 in the AiO v26 layout. |
| 26 Oct 2026 | **0.6**: all 8 relays are sections by default. **0.7**: also a plain ESP32 with [relay modules](#a-separate-esp32-with-relay-modules), up to 16 relays. The relay number above its function on the page. |
| 25 Oct 2026 | **0.4**, first version on the ESP32 8-relay board (after Sprayer_Nano 0.3): sections from the program, pressure and flow, readings to the program (PGN 160), [its own page](#how-it-works). **0.5**: program subnet set on the page. The page in the simulator. |

## How it works

```
Tablet (AgOpenWeb / AgOpenGPS)
   │  Wi-Fi (the tractor's router), UDP
   ▼
ESP32_Relay_X8 board ──► 8 relays ──► 8 solenoid valves (NC), one per section
   ▲
   ├── pressure sensor (GPIO 34) + flow meter (GPIO 35)
   └── own Wi-Fi network "Sprayer-xxxx" + web page (phone)
```

- The ESP32 is the standard AgOpenGPS **machine module**, IP `<subnet>.123` on the
  tractor's Wi-Fi network (192.168.5.123 until the program, or the page, says otherwise).
- It speaks the same messages as the official `Machine_UDP_v5` module: sections
  (PGN 239 / 229), machine config (238: relay polarity, lift times), pin functions
  (236: what each relay does), subnet (201), hello (200 → 123) and scan (202 → 203).
  Everything the program sends is kept in flash, so it survives a power cut.
- It works out the **application rate (L/ha)** from the flow, the speed (PGN 239) and the
  widths of the open sections (PGN 235, which AgOpenWeb sends; kept in flash), the
  pressure that gives the **target rate** at this speed, and the **alarms**: low or high
  pressure, flow outside the tolerance, flow with every section closed. The target rate,
  nozzles, pressure limits and flow tolerance are set on the module's page (*Settings*);
  the section widths stay in AgOpenWeb.
- Once a second it sends all of it to the program: PGN 160 (pressure, flow, litres,
  master switch, sections open) as before, and PGN 161 with the same readings plus the
  rate, target, pressure for the target, pressure limits and alarms as **ISOBUS process
  data** (DDIs, see *Messages*). The
  [AgOpenWeb fork with the sprayer readout](https://github.com/renatorgr/AgOpenWeb) shows
  them above the section buttons and sounds the alarms. An alarm also goes to the program
  as PGN 221, the text message it shows on its screen, so the official AgOpenWeb (which
  reads no PGN 161) shows the alarm too: when it comes on or goes off, and every 10 s
  while it lasts. With no sensor ticked it sends none of them: see *Sensors fitted* below.
- The valves are normally closed: no power = valve closed. No message from the program
  for **2 seconds** (Wi-Fi gone, tablet frozen): every section closes.
- The master switch cuts the 12 V to the relay COM terminals in the wiring, so it
  works even if the ESP32 or the tablet freezes.
- Its **own Wi-Fi network** is always on: join `Sprayer-xxxx` (password `agopengps`)
  with a phone and the page opens by itself, or go to `http://192.168.4.1/`. On the
  tractor network the same page is at `http://192.168.5.123/` or `http://sprayer.local/`
  (the module announces it by mDNS, so AgOpenWeb's web cards, ⚙ › **Search the network**, list
  it by name in a second). It has four tabs:
  - **Spraying**: the rate in L/ha against the target (green within 10 %, orange outside),
    the pressure and the pressure that gives the target (or "slow down / speed up to …
    km/h" when the target needs a pressure outside the limits), the sections and the alarms.
  - **Settings**: target rate, nozzles, pressure limits, flow tolerance; the section widths
    received from AgOpenWeb; the sensors fitted and their calibration. Saving needs the
    own network's password, so nothing changes by a touch while driving.
  - **Module**: the link to AgOpenWeb, the relays, the Wi-Fi settings (laid out as on the XIAO
    page, with *Search for networks*) and the events.
  - **Firmware**, as on the Teensy and XIAO pages: the program installed (version, build
    date, board), the **version on GitHub** (the `Sprayer_ESP32.ino.bin` in this folder;
    checked when the page opens, every 6 hours and with *Check now*) with **Install from
    GitHub**, install a chosen `.bin` with a progress bar, Restart, and **Download the
    report**: the module's state, settings and events in one `.json` file to send when
    something goes wrong (no passwords in it). The phone or tablet asks GitHub, so it needs
    internet: open the page over the tractor network, not over the module's own Wi-Fi.
    A `.bin` for another board, another module's program or the `.merged.bin` is refused.
  Without the board, the page can be tried in the
  [simulator](https://renatorgr.github.io/Boards/sprayer/), rebuilt from this code on every change.

## Hardware

| Part | Notes |
|---|---|
| ESP32_Relay_X8 board (303E32DC810) | ESP32-WROOM-32E, 8 × SRD-05VDC relays (10 A), LM2596 on board, **5–30 V DC in** |
| 8 × solenoid valve FPD-270A, one per section | 12 V, 0.3 A, NC, 0.2–8 bar, G1/2 |
| Pressure sensor 0–10 bar | 0.5–4.5 V output, powered at 5 V |
| YF-S201 flow meter | 5 V, about 450 pulses per litre, 1–30 L/min |
| USB-serial adapter (CH340 or FTDI), 3.3 V | Only for the first upload; later updates go over Wi-Fi |
| 3 × 10 kΩ + 1 × 22 kΩ resistors (¼ W) | Dividers for the two sensor signals |

The relays switch **on when the pin is HIGH** (checked with `ESP32_Relay_Test`):
relay 1 = GPIO 32, 2 = 33, 3 = 25, 4 = 26, 5 = 27, 6 = 14, 7 = 12, 8 = 13.

## Pins to solder

Only two pin headers are needed (2.54 mm male pins, soldered from the back):

| Header | Pins | Used for |
|---|---|---|
| 6-pin row `IO0 GND GND RX TX 5V` | all 6 | first upload with the USB-serial adapter, 5 V for the sensors |
| 2 × 10 row from `GND 3V3` to `5V GND` | G34, G35, 3V3, GND, EN | sensors (G34 pressure, G35 flow), reset |

The other 2 × 10 row (G23, G22, G21 …) stays empty.

## Wiring

The ESP32 pins take **3.3 V at most**. Both sensors run on 5 V, so each signal goes
through a divider, **10 kΩ in series** and then a resistor to GND:

- pressure: **10 kΩ to GND** (0.5). 0.5–4.5 V from the sensor becomes 0.25–2.25 V, where
  the ESP32 measures best. The page's calibration has this 0.5 as the divider.
- flow: **22 kΩ to GND** (0.69). A 5 V pulse becomes about 3.4 V, well above the 2.5 V
  the ESP32 needs to see a pulse, even if the meter has its own pull-up resistor.

Both values come in the usual ¼ W resistor kits. With other resistors, put the pressure
divider (to-GND ÷ total) in the page's calibration.

| Connection | Wire |
|---|---|
| Board `DC+` / `GND` (3-pin green terminal) | 12 V from the battery through a **1 A fuse**. Separate feed, not through the master switch |
| Pressure sensor red / black | 5V pin / GND |
| Pressure sensor green (signal) | 10 kΩ → **G34**, and 10 kΩ from G34 to GND |
| YF-S201 red / black | 5V pin / GND |
| YF-S201 yellow (signal) | 10 kΩ → **G35**, and 22 kΩ from G35 to GND |
| Valves | 12 V from the battery, 5 A fuse → master switch → relay COM. Relay NO → valve +. Valve − → GND. 1N4007 diode across each valve (stripe to +) |

- Use a separate shielded cable for the sensors, with its own GND wire back to the board.
- All grounds (battery, board, valves) joined together.
- On the tractor, solder the sensor wires or use a latching connector: Dupont jumpers
  come loose with the vibration.

## First upload (USB-serial adapter)

1. Arduino IDE → *File → Preferences → Additional boards manager URLs*:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
   *Tools → Board → Boards Manager*: install **esp32 by Espressif Systems**.
2. Board **ESP32 Dev Module**, default partition scheme. No libraries to install.
3. Power the board with 12 V. Adapter jumper on **3.3 V**. Three wires to the 6-pin
   header: adapter TX → board **RX**, adapter RX → board **TX**, GND → **GND**.
   Leave the adapter's 5V/VCC unconnected.
4. *Tools → Port*: the adapter's COM port.
5. Hold **IO0**, press and release **EN**, release IO0. Click *Upload*. When it says
   "Hard resetting", press **EN** once.
6. Serial Monitor at **115200**: it prints `Sprayer_ESP32 v0.10.1` and `ready`.

"Failed to connect": TX and RX are probably swapped, or step 5 was skipped.

## Setup

1. On a phone, join the Wi-Fi network **Sprayer-xxxx**, password **agopengps**. The
   page opens by itself (or open `http://192.168.4.1/`).
2. **Module** tab, card *Tractor Wi-Fi network*: tap *Search for networks* and then the
   router the tablet uses (that ticks *Join the tractor network*), and type its password.
   In the *Save* card type the own network's password (**agopengps**), *Save and restart
   the module*. The header then shows `on <network> as 192.168.5.123`.
3. **Program subnet**: the module and the tablet must be on the same subnet, which is
   the first three numbers of the tablet's IP on that network (Wi-Fi settings on the
   tablet; a Starlink router gives 192.168.1.x, for example). If it is not 192.168.5,
   type it in the *Program network* card, then *Save and restart the module*: the header then shows
   `as 192.168.1.123`. In AgOpenWeb set the module subnet to the same three numbers.
   AgOpenWeb can also send its subnet to the module (PGN 201) once it sees it.
4. Open AgOpenWeb. The machine module shows up at `<subnet>.123`, and the page's dot
   turns green (*program talking*).
5. In AgOpenWeb's machine settings, leave **"relay active high" unticked** for this
   board. The page shows *Output polarity: normal*. If it says *inverted*, untick it.
6. **Sections.** On the module, relays 1–8 are sections 1–8 by default, so the board is
   ready for an 8-section boom. But AgOpenWeb sends its own pin config to the module at
   start and on profile load, and its default is pins 1–6 = sections 1–6, pins 7–8 =
   none. So in AgOpenWeb set *Tool → sections* to the number of sections on the boom
   (8, each 1 m wide), and set *Machine → Pin config* pins 7 and 8 to
   *Section 7* and *Section 8*: without that, relays 7 and 8 never open. A relay that is not a section can be the hydraulic lift
   or the tramline instead. The page's *Relays* card shows what each relay does.
7. **Sensors fitted.** In *Settings → Sensors*, tick *Pressure sensor fitted* and *Flow meter
   fitted* for the sensors on the machine (both on the sprayer, ticked by default), then
   *Save*. The page only shows the readings and calibration fields of the sensors ticked;
   an unticked sensor reads 0 (its pin floats without it). With **neither ticked** the
   rate and pressure go away and the module stops sending PGN 160 and 161, so
   AgOpenWeb's sprayer strip and the AiO module page's *Sprayer (PGN 160)* card do not
   show; the sections still reach them in the hello reply.
8. **Section widths.** *Settings → Sections from AgOpenWeb* should say *8 sections of
   1.00 m*. AgOpenWeb sends them (PGN 235) when it starts and when a tool profile is saved
   or loaded; the module keeps them. If it says *not received yet*, save the tool profile
   in AgOpenWeb with the module on the network. Without them there is no L/ha.
9. **Spraying settings.** In *Settings → Spraying*: the target rate (L/ha), the nozzle
   spacing (0.5 m: 2 nozzles per 1 m section), one nozzle's flow at a pressure from the
   nozzle chart, the minimum and maximum pressure and the flow tolerance, then *Save*.
   These used to be in AgOpenWeb (*Machine → Sprayer*); with this module AgOpenWeb hides
   them and shows what the module sends.

Later program updates, on the page's **Firmware** tab:
- **Install from GitHub** when it says "newer than the installed one": the page downloads
  `Sprayer_ESP32.ino.bin` from this folder on GitHub and sends it to the module.
- Or *Sketch → Export compiled binary* in the Arduino IDE, then *Choose .bin*
  (`Sprayer_ESP32.ino.bin` in the sketch folder, not the `.merged.bin`) and *Install*.

The page knows which version is on GitHub from the `.bin` itself (its version, board and
build date, published in `firmware.json` with the simulator). So after a code change,
export the binary again and put the new `.bin` files on GitHub with the code, or the page
keeps offering the older program.

## Bench test (before the tractor, no water)

1. Only the board, valves disconnected. With the program closed the relay LEDs must be
   **off**.
2. Join the tractor network (Setup above). In AgOpenWeb turn sections on from the screen:
   each relay should click and its LED light up, and the page shows them green.
3. Switch the tablet's Wi-Fi off: all relays switch off within 2 seconds, the page says
   *program silent, sections closed*.
4. Blow through the flow meter: the flow on the page goes above 0.
5. Press on the pressure sensor's connector pins with nothing attached: the sensor
   voltage shown should sit near 0.5 V (0 bar).

## Calibrating the flow meter

With the pump running and one nozzle open, fill a 10 L bucket and note the litres the
page counted. New pulses per litre = old value × (litres counted ÷ 10). Enter it under
*Settings → Sensors* on the page (it needs the own network's password).

## Forgotten Wi-Fi password

Hold the **IO0** button for 5 seconds while the board is running: the Wi-Fi settings go
back to the defaults (`Sprayer-xxxx`, `agopengps`, no tractor network) and the board
restarts. The program's config and the calibration stay. The same from the Serial
Monitor: `wifi-reset`.

## Messages

| PGN | Direction | What |
|---|---|---|
| 239 | program → module | sections 1–16 (bytes 11–12), hydraulic lift (7), tramline (8), geo stop (9), speed (6, km/h × 10) |
| 229 | program → module | sections 1–64 (bytes 5–12, 8 per byte, section 1 = byte 5 bit 0), sent with every 239 when the tool has more than 16 sections. The relays here follow sections 1–16 only (the program's pin config) |
| 238 | program → module | machine config: raise / lower time, lift enable, byte 8 bit 0 = outputs inverted. Saved |
| 235 | program → module | section dimensions: widths of sections 1–16 in cm (uint16, bytes 5–36), number of sections (37). For the L/ha. Saved |
| 236 | program → module | 24 output functions: 1–16 section, 17 lift lower, 18 lift raise, 19 tramline right, 20 tramline left, 21 geo stop. Saved |
| 201 | program → module | subnet; the module restarts as `<subnet>.123` (or the last number set on the page). Saved |
| 200 → 123 | hello / reply | reply carries the sections 1–16 really open (bytes 5–6) |
| 202 → 203 | scan / reply | the module's IP |
| 237 | module → program | every 200 ms, in the layout of the AgOpenGPS AiO v26 firmware: sections 1–16 open (5–6), lift (7: 0 off, 1 down, 2 up), tramline (8), geo stop (9), status (10: bit 0 = the program's PGN 239 still arrives) |
| 160 | module → program | once a second, only with a sensor ticked: pressure (bar × 100), flow (L/min × 100), total (L × 10, uint32), master switch, sections open 1–8 (byte 14) and 9–16 (byte 15). See `Docs/PGN.md` of the AgOpenWeb fork, which reads byte 14 |
| 161 | module → program | right after each 160: ISOBUS process data, 8 bytes per value (table below) |
| 221 | module → program | when an alarm comes on or goes off, and every 10 s while one is on: the text message the program shows on its screen, as the AiO Teensy sends it (byte 5 seconds to show, byte 6 colour: 0 warning, 1 normal; text from byte 7, 60 bytes at most). The official AgOpenWeb shows it; the fork has the alarms from 161 as well |

All messages: header `0x80 0x81`, source `0x7B` from the module, checksum = sum of the
bytes from the source byte to the one before the checksum. The module only accepts
messages with a right checksum from source `0x7F` (the program).

**PGN 161, ISOBUS process data.** As the Process Data Message of ISO 11783-10, which the
AgOpenGPS ISOBUS group proposes for modules ("ISOBUS Data Everywhere"): each value is
8 bytes, little-endian. Byte 0 = command 3 (*value*) in bits 0–3 and the element number's
bits 0–3 in bits 4–7, byte 1 = the element number's bits 4–11, bytes 2–3 = the DDI,
bytes 4–7 = the value (int32). Element 0 = the sprayer, element 1 = its boom. A value
that is not known (sensor not fitted, tractor stopped, no target) is left out of that
message.

| DDI | What | Unit (value = ) | Element | Sent |
|---|---|---|---|---|
| 194 | actual pressure | 0.1 Pa (bar × 1 000 000) | 1 | pressure sensor fitted |
| 37 | actual flow | mm³/s (L/min × 16 667) | 1 | flow meter fitted |
| 351 | total volume | ml | 0 | flow meter fitted |
| 141 | actual work state (master switch) | 1 on, 0 off | 1 | always |
| 161 | actual condensed work state, sections 1–16 | 2 bits per section: 01 open, 00 closed, 11 not there | 1 | always |
| 2 | actual rate | 0.01 mm³/m² (L/ha × 10 000) | 1 | moving, widths known, flow meter fitted |
| 1 | target rate | 0.01 mm³/m² (L/ha × 10 000) | 1 | target set |
| 193 | pressure that gives the target now | 0.1 Pa | 1 | moving with a target |
| 195 / 196 | minimum / maximum pressure | 0.1 Pa | 1 | when set |
| 57344 | sprayer alarms (proprietary DDI): 1 low pressure, 2 high pressure, 4 flow too low, 8 flow too high, 16 flow with the sections off | bits | 1 | always |

## A separate ESP32 with relay modules

The same program runs on a plain ESP32 dev module (30 or 38 pins) with one or two relay
modules, for up to **16 relays**. At the top of `Sprayer_ESP32.ino` set
`#define BOARD BOARD_DEVKIT`. Then:

| | ESP32_Relay_X8 board | ESP32 dev module + relay modules |
|---|---|---|
| Relay pins | 32, 33, 25, 26, 27, 14, 12, 13 | 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 (relay 1 first; shorten the list to the relays you have) |
| Relay closes when the pin is | HIGH (`RELAY_ON_HIGH 1`) | LOW on the usual optocoupled modules (`RELAY_ON_HIGH 0`); set 1 for a module with high-trigger jumpers |
| LED | GPIO 23 | GPIO 2 (the blue LED) |
| Sensors | G34 pressure (10k / 10k), G35 flow (10k / 22k) | the same pins and dividers |
| Power | 5–30 V DC on the board | 5 V to the dev module's `5V`/`VIN` pin from an LM2596; the relay module's `JD-VCC` from the same 5 V (jumper off), its `VCC` from the ESP32's 3V3, GND to GND |

Wiring the relay module: `IN1`…`IN16` to the pins above in order, one common GND. The 12 V
for the valves goes through the master switch to the relays' COM, as on the X8 board.

**Which relays are sections** is decided by the program, not by the module: in its pin
config (AgOpenGPS: *Machine → Pin config*, default pins 1–16 = sections 1–16; AgOpenWeb:
*Machine → Pin config*, default pins 1–6 = sections 1–6) every pin marked *Section n*
makes relay n open with section n. The page's *Relays* card shows the result ("16 relays,
8 of them sections"). A relay that is not a section can be the hydraulic lift or the
tramline. The AgOpenWeb sprayer strip shows the first 8 sections; the module reports
all 16 (PGN 160 byte 15, PGN 161 DDI 161).

If a relay module closes on HIGH instead of LOW, either set `RELAY_ON_HIGH 1` or tick
"relay active high" in the program: the page's *Output polarity* line says which pin
level the module is using.

## Notes

- `USE_MASTER_SWITCH` (top of `Sprayer_ESP32.ino`): set to 1 to read the master switch
  on **G39** (12 V through 47 kΩ, then 10 kΩ to GND). Off by default: the switch cuts the
  valves' 12 V in the wiring.
- `LED_PIN` is 23 on the X8 board: if its LED does not blink, that is not the right pin; set
  it to the right one or to −1.
- The FPD-270A valves and the YF-S201 are plastic and made for water. Rinse with clean
  water after each job. Fit a filter before the flow meter.
- The ISOBUS group of AgOpenGPS is working on a standard way for modules to send such
  readings (ISOBUS DDIs). PGN 161 follows that proposal; the PGN number itself is this
  project's choice until the group picks one. The module keeps sending PGN 160 for the AiO
  module page and older AgOpenWeb builds.
- The page is announced over mDNS as `_http._tcp` (the AgOpenWeb fork's **Search the network**)
  and as `_aowcard._tcp` with TXT `v=1` and `path=/`, the official AgOpenWeb add-on card
  announcement (`Docs/ADD_ON_CARDS.md`), both under the module's own Wi-Fi name
  (`Sprayer-xxxx`, "Name on the network" on the Firmware tab), unique to each module. The
  earlier `_aow-ui._tcp` and `GET /aow/status` (0.10.0 and 0.10.1) are gone.
