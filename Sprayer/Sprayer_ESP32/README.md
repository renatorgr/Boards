# Sprayer_ESP32

Section control for a 5-section, 8 m sprayer boom, driven by **AgOpenWeb** or
**AgOpenGPS** over Wi-Fi, on an **ESP32 8-relay board**. It also measures boom
pressure and flow, and has its own web page.

The 19-section no-till seed drill on a KinCony KC868-A32 has its own program since
v0.9: [Seeder/Seeder_ESP32](../../Seeder/Seeder_ESP32).

Version 0.9 (successor of [Sprayer_Nano](../Sprayer_Nano) v0.3): runs on the bench, not yet
tested on the sprayer.

## How it works

```
Tablet (AgOpenWeb / AgOpenGPS)
   │  Wi-Fi (the tractor's router), UDP
   ▼
ESP32_Relay_X8 board ──► 8 relays ──► up to 8 solenoid valves (NC), 5 on this boom
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
- Once a second it also sends the sprayer readings (PGN 160: pressure, flow, litres,
  master switch, sections open), which the
  [AgOpenWeb fork with the sprayer readout](https://github.com/renatorgr/AgOpenWeb) shows
  above the section buttons. With no sensor ticked it sends none: see *Sensors fitted*
  below.
- The valves are normally closed: no power = valve closed. No message from the program
  for **2 seconds** (Wi-Fi gone, tablet frozen): every section closes.
- The master switch cuts the 12 V to the relay COM terminals in the wiring, so it
  works even if the ESP32 or the tablet freezes.
- Its **own Wi-Fi network** is always on: join `Sprayer-xxxx` (password `agopengps`)
  with a phone and the page opens by itself, or go to `http://192.168.4.1/`. On the
  tractor network the same page is at `http://192.168.5.123/` or `http://sprayer.local/`.
  Without the board, the page can be tried in the
  [simulator](https://renatorgr.github.io/Boards/sprayer/), rebuilt from this code on every change.

## Hardware

| Part | Notes |
|---|---|
| ESP32_Relay_X8 board (303E32DC810) | ESP32-WROOM-32E, 8 × SRD-05VDC relays (10 A), LM2596 on board, **5–30 V DC in** |
| 5 × solenoid valve FPD-270A (up to 8, one per relay) | 12 V, 0.3 A, NC, 0.2–8 bar, G1/2 |
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
6. Serial Monitor at **115200**: it prints `Sprayer_ESP32 v0.9` and `ready`.

"Failed to connect": TX and RX are probably swapped, or step 5 was skipped.

## Setup

1. On a phone, join the Wi-Fi network **Sprayer-xxxx**, password **agopengps**. The
   page opens by itself (or open `http://192.168.4.1/`).
2. **Wi-Fi settings**: tick *Join the tractor network*, enter the name and password of
   the router the tablet uses, *Save and restart*. The header then shows
   `on <network> as 192.168.5.123`.
3. **Program subnet**: the module and the tablet must be on the same subnet, which is
   the first three numbers of the tablet's IP on that network (Wi-Fi settings on the
   tablet; a Starlink router gives 192.168.1.x, for example). If it is not 192.168.5,
   type it in *Program subnet* on the page, *Save and restart*: the header then shows
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
   (5 now), and for more than 6 sections set *Machine → Pin config* pins 7 and 8 to
   *Section 7* and *Section 8*. A relay that is not a section can be the hydraulic lift
   or the tramline instead. The page's *Relays* card shows what each relay does.
7. **Sensors fitted.** In *Calibration*, tick *Pressure sensor fitted* and *Flow meter
   fitted* for the sensors on the machine (both on the sprayer, ticked by default), then
   *Save*. The page only shows the readings and calibration fields of the sensors ticked;
   an unticked sensor reads 0 (its pin floats without it). With **neither ticked** the
   *Sprayer* card goes away and the module stops sending PGN 160, so
   AgOpenWeb's sprayer strip and the AiO module page's *Sprayer (PGN 160)* card do not
   show; the sections still reach them in the hello reply.

Later program updates: *Sketch → Export compiled binary* in the Arduino IDE, then on the
page *Install a new program* and choose the `.bin`.

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
*Calibration* on the page (it needs the own network's password).

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
| 236 | program → module | 24 output functions: 1–16 section, 17 lift lower, 18 lift raise, 19 tramline right, 20 tramline left, 21 geo stop. Saved |
| 201 | program → module | subnet; the module restarts as `<subnet>.123` (or the last number set on the page). Saved |
| 200 → 123 | hello / reply | reply carries the sections 1–16 really open (bytes 5–6) |
| 202 → 203 | scan / reply | the module's IP |
| 237 | module → program | every 200 ms: sections 1–16 open (5–6), relays 1–16 on (7–8), master switch (9) |
| 160 | module → program | once a second, only with a sensor ticked: pressure (bar × 100), flow (L/min × 100), total (L × 10, uint32), master switch, sections open 1–8 (byte 14) and 9–16 (byte 15). See `Docs/PGN.md` of the AgOpenWeb fork, which reads byte 14 |

All messages: header `0x80 0x81`, source `0x7B` from the module, checksum = sum of the
bytes from the source byte to the one before the checksum. The module only accepts
messages with a right checksum from source `0x7F` (the program).

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
all 16 (PGN 160 byte 15).

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
  readings (ISOBUS DDIs) and to show their own page in AgOpenWeb. This program keeps
  PGN 160 until that is decided; the page already answers `GET /aow/status` and announces
  itself as `_aow-ui._tcp` over mDNS as the proposal describes.
