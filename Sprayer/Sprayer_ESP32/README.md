# Sprayer_ESP32

Section control for a 5-section, 8 m sprayer boom, driven by **AgOpenWeb** or
**AgOpenGPS** over Wi-Fi, on an **ESP32 8-relay board**. It also measures boom
pressure and flow, and has its own web page.

Version 0.5 (successor of [Sprayer_Nano](../Sprayer_Nano) v0.3): runs on the bench, not yet
tested on the sprayer.

## How it works

```
Tablet (AgOpenWeb / AgOpenGPS)
   │  Wi-Fi (the tractor's router), UDP
   ▼
ESP32_Relay_X8 board ──► 8 relays ──► 5 solenoid valves (NC)
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
  above the section buttons.
- The valves are normally closed: no power = valve closed. No message from the program
  for **2 seconds** (Wi-Fi gone, tablet frozen): every section closes.
- The master switch cuts the 12 V to the relay COM terminals in the wiring, so it
  works even if the ESP32 or the tablet freezes.
- Its **own Wi-Fi network** is always on: join `Sprayer-xxxx` (password `agopengps`)
  with a phone and the page opens by itself, or go to `http://192.168.4.1/`. On the
  tractor network the same page is at `http://192.168.5.123/` or `http://sprayer.local/`.

## Hardware

| Part | Notes |
|---|---|
| ESP32_Relay_X8 board (303E32DC810) | ESP32-WROOM-32E, 8 × SRD-05VDC relays (10 A), LM2596 on board, **5–30 V DC in** |
| 5 × solenoid valve FPD-270A | 12 V, 0.3 A, NC, 0.2–8 bar, G1/2 |
| Pressure sensor 0–10 bar | 0.5–4.5 V output, powered at 5 V |
| YF-S201 flow meter | 5 V, about 450 pulses per litre, 1–30 L/min |
| USB-serial adapter (CH340 or FTDI), 3.3 V | Only for the first upload; later updates go over Wi-Fi |
| 2 × 10 kΩ + 2 × 15 kΩ resistors | Dividers for the two sensor signals |

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
through a divider: **10 kΩ in series, then 15 kΩ to GND** (0.6). 4.5 V from the
pressure sensor becomes 2.7 V, right inside the range the ESP32 measures well.

| Connection | Wire |
|---|---|
| Board `DC+` / `GND` (3-pin green terminal) | 12 V from the battery through a **1 A fuse**. Separate feed, not through the master switch |
| Pressure sensor red / black | 5V pin / GND |
| Pressure sensor green (signal) | 10 kΩ → **G34**, and 15 kΩ from G34 to GND |
| YF-S201 red / black | 5V pin / GND |
| YF-S201 yellow (signal) | 10 kΩ → **G35**, and 15 kΩ from G35 to GND |
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
6. Serial Monitor at **115200**: it prints `Sprayer_ESP32 v0.5` and `ready`.

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
6. In AgOpenWeb's pin config, relays 1–5 are sections 1–5 by default. Relays 6–8 are
   free (hydraulic lift, tramline).

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
| 229 | program → module | 64 sections; the first 16 are used |
| 238 | program → module | machine config: raise / lower time, lift enable, byte 8 bit 0 = outputs inverted. Saved |
| 236 | program → module | 24 output functions: 1–16 section, 17 lift lower, 18 lift raise, 19 tramline right, 20 tramline left, 21 geo stop. Saved |
| 201 | program → module | subnet; the module restarts as `<subnet>.123`. Saved |
| 200 → 123 | hello / reply | reply carries the sections really open (bytes 5–6) |
| 202 → 203 | scan / reply | the module's IP |
| 237 | module → program | every 200 ms: sections open (5–6), relays on (7), master switch (8) |
| 160 | module → program | once a second: pressure (bar × 100), flow (L/min × 100), total (L × 10, uint32), master switch, sections open. See `Docs/PGN.md` of the AgOpenWeb fork |

All messages: header `0x80 0x81`, source `0x7B` from the module, checksum = sum of the
bytes from the source byte to the one before the checksum. The module only accepts
messages with a right checksum from source `0x7F` (the program).

## Notes

- `USE_MASTER_SWITCH` (top of `Sprayer_ESP32.ino`): set to 1 to read the master switch
  on **G39** (12 V through 47 kΩ, then 10 kΩ to GND). Off by default: the switch cuts the
  valves' 12 V in the wiring.
- `LED_PIN` is 23: if the board's LED does not blink, that is not the right pin; set it
  to the right one or to −1.
- The FPD-270A valves and the YF-S201 are plastic and made for water. Rinse with clean
  water after each job. Fit a filter before the flow meter.
- The ISOBUS group of AgOpenGPS is working on a standard way for modules to send such
  readings (ISOBUS DDIs) and to show their own page in AgOpenWeb. This program keeps
  PGN 160 until that is decided; the page already answers `GET /aow/status` and announces
  itself as `_aow-ui._tcp` over mDNS as the proposal describes.
