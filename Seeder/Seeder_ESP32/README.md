# Seeder_ESP32

Section control for the **19-section no-till seed drill**, driven by **AgOpenWeb** or
**AgOpenGPS** over the tractor's network. Each section is one linear actuator, switched
by a 12 V DPDT relay. Two boards:

- the **[Relay24_ESP32](../../Relay24_ESP32)** board of this repository (the default): an
  ESP32 DevKitC on a board with 24 DPDT relays and a network socket, the actuators wired
  straight to it;
- a **KinCony KC868-A32** (ESP32, 32 relays, network socket), each relay switching an
  external DPDT relay.

Version 0.2, made from [Sprayer_ESP32](../../Sprayer/Sprayer_ESP32) v0.8 when the seed drill
got its own folder. Written from the Relay24_ESP32 design and KinCony's pin list: it has
not run on either board yet.

## How it works

```
Tablet (AgOpenWeb) ── Wi-Fi ── RUT901 router ── LAN1: AiO v4.5 (192.168.5.126)
                                   └─────────── LAN2: network cable to the seed drill
Relay24_ESP32 (192.168.5.123) ──► DPDT relays 1–19 ──► 19 actuators
   └── own Wi-Fi network "Seeder-xxxx" + web page (phone)
```

- The board is the standard AgOpenGPS **machine module**, IP `<subnet>.123`
  (192.168.5.123 until the program, or the page, says otherwise), on the **network
  cable**. No Wi-Fi to set up; the page can switch it to the tractor's Wi-Fi instead.
- It speaks the same messages as the official `Machine_UDP_v5` module: sections
  (PGN 239 for 1–16, PGN 229 for 1–64), machine config (238), pin functions (236),
  subnet (201), hello (200 → 123) and scan (202 → 203). Everything the program sends is
  kept in flash, so it survives a power cut.
- **Relay 1 = section 1, relay 2 = section 2** and so on, up to relay 24 (32 on the
  KC868-A32). The page's *Relay 1 follows section* setting can start the row at another
  section; 0 there gives the relays back to the program's pin config, which only knows
  sections 1–16.
- No message from the program for **2 seconds** (cable pulled, tablet frozen): every
  relay drops and every row closes.
- No pressure or flow sensor: no PGN 160 (that is the sprayer's).
- Its **own Wi-Fi network** is always on: join `Seeder-xxxx` (password `agopengps`) with
  a phone and the page opens by itself, or go to `http://192.168.4.1/`. On the tractor
  network the same page is at `http://192.168.5.123/` or `http://seeder.local/`. Without
  the board, the page can be tried in the
  [simulator](https://renatorgr.github.io/Boards/seeder/), rebuilt from this code on every
  change.

## Choosing the board

`BOARD` near the top of `Seeder_ESP32.ino`:

| `BOARD` | Board |
|---|---|
| `BOARD_RELAY24` (default) | Relay24_ESP32: 24 relays through 74HC595 shift registers, RTL8201F network chip |
| `BOARD_KC868_A32` | KinCony KC868-A32: 32 relays through PCF8574 chips, LAN8720 network chip |

The page shows the board next to the version (*Program* card).

## Hardware: Relay24_ESP32

| Part | Notes |
|---|---|
| [Relay24_ESP32](../../Relay24_ESP32) board | 24 DPDT relays (one per section, they reverse the actuator), RJ45 network socket, 12 V in |
| ESP32 DevKitC V4 (38 pins) | plugged into the board's two sockets |
| 19 × 12 V linear actuator | with end switches inside, or two external end switches each (below) |
| Network cable | from the RUT901's `LAN2` to the board's RJ45 |

How the program drives it (pins in `Seeder_ESP32.ino`, more in the board's README):

- **Relays:** three 74HC595 chips in a chain (data GPIO 13, clock GPIO 16, latch GPIO 32):
  the program shifts 24 bits in, relay 24 first, then latches them. A high output closes
  the relay (`RELAY_ON_HIGH 1`). The outputs stay off (pull-up on GPIO 4) until the
  program has cleared the chips at start-up, so no relay clicks while the ESP32 boots.
- **Network:** RTL8201F, address 0, MDC GPIO 23, MDIO GPIO 18, reset GPIO 33. The chip
  sends the 50 MHz network clock into **GPIO 0**, so once the cable is in use the
  DevKitC's **BOOT** button is not read by the program, and it should not be pressed
  (the network stops while it is held). The chip is held in reset before every restart.

## Hardware: KC868-A32

| Part | Notes |
|---|---|
| KinCony KC868-A32 | ESP32, 32 relays (10 A, COM / NO / NC) through four PCF8574 chips (I2C 0x24, 0x25, 0x21, 0x22 on GPIO 15 / 13), LAN8720 network chip, 32 inputs, 4 analog inputs, 12 V in |
| 19 × 12 V DPDT relay (8 pins, with socket) | one per section, to turn the actuator's polarity round |
| 19 × 12 V linear actuator | with end switches inside, or two external end switches each (below) |
| Network cable | from the RUT901's `LAN2` to the KC868-A32 |

The relays close when their PCF8574 pin is **LOW** (`RELAY_ON_HIGH 0`).

## First upload

1. Arduino IDE → *File → Preferences → Additional boards manager URLs*:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
   *Tools → Board → Boards Manager*: install **esp32 by Espressif Systems**.
2. Board **ESP32 Dev Module**, default partition scheme. No libraries to install.
3. Open `Seeder_ESP32.ino` (this folder: the Arduino IDE needs the folder and the `.ino`
   to have the same name). Check the `BOARD` line.
4. Relay24_ESP32: plug the DevKitC's USB socket into the computer (the board can be
   powered or not). KC868-A32: power it with 12 V and plug its USB socket in, as in
   KinCony's guide. *Tools → Port*: its COM port. Click *Upload*.
5. Serial Monitor at **115200**: it prints `Seeder_ESP32 v0.2` and `ready`.

Later program updates: *Sketch → Export compiled binary* in the Arduino IDE, then on the
page *Install a new program* and choose the `.bin`.

## Setup

1. Plug the network cable into the RUT901's `LAN2` and the board. The page's *Network* card shows
   *Network cable: connected*.
2. **Program subnet**: the module and the tablet must be on the same subnet (the first
   three numbers of the tablet's IP). If it is not 192.168.5, set it in *Network settings*
   on the page, or let AgOpenWeb send it (PGN 201).
3. Open AgOpenWeb: the page's dot turns green (*program talking*). Leave
   **"relay active high" unticked** in its machine settings: the page shows
   *Output polarity: normal*.
4. **AgOpenWeb**: *Tool Configuration → Tool → Sections*: 19 individual sections with
   their widths. With more than 16 sections AgOpenWeb sends PGN 229 by itself, and the
   page shows *Sections 17-64 (PGN 229): now*. The actuators take a few seconds for their
   stroke, so set *Tool → Timing → Look ahead on / off* to that time.

## Wiring: Relay24_ESP32

| Connection | Wire |
|---|---|
| `J25` (12 V IN) | 12 V from the tractor through a **2 A fuse**: the board, the ESP32 and the relay coils |
| `J26` / `J27` (12 V ACT A, + and −) | the actuator supply for sections 1–12, fused for the actuators (both ends when several move at once) |
| `J28` / `J29` (12 V ACT B, + and −) | the same for sections 13–24 |
| `J1` … `J24`, pin 1 / pin 2 | actuator *n* red / black (`J1` = section 1) |
| RJ45 | network cable to the RUT901's `LAN2` |

Relay off = black at +12 V, red at GND = the actuator pulls in = section **closed**.
Relay on = red at +12 V, black at GND = the actuator pushes out = section **open**. So a
module that loses power, or hears nothing from the program for 2 s, closes every row.
An actuator that moves the wrong way: swap its two wires.

## Wiring: KC868-A32, one section

Repeat for sections 1–19 on relays 1–19.

| Connection | Wire |
|---|---|
| KC868-A32 `12V` / `GND` | 12 V from the tractor through a **2 A fuse** |
| KC868 relay *n* `COM` | +12 V (fused) |
| KC868 relay *n* `NO` | coil + of DPDT relay *n*; coil − to GND; a 1N4007 across the coil (stripe to +) |
| DPDT relay *n* `COM1` / `COM2` | actuator *n* red / black |
| DPDT relay *n* `NO1` and `NC2` | +12 V (actuator supply) |
| DPDT relay *n* `NC1` and `NO2` | GND |
| KC868 network socket | network cable to the RUT901's `LAN2` |

KC868 relay off = DPDT relay off = black at +12 V, red at GND = the actuator pulls in =
section **closed**. Relay on = red at +12 V, black at GND = the actuator pushes out =
section **open**, as on the Relay24_ESP32.

The actuators need their own fused 12 V feed: 19 actuators moving at once draw about
12 A (check the label: some draw 2–3 A each while moving).

## Stopping the actuator at each end

The DPDT relay keeps 12 V on the actuator all the time, in one direction or the other.
Something must cut it at each end of the stroke:

- **Actuators with end switches inside** (most 12 V linear actuators): they stop by
  themselves at each end. Nothing else to wire. The stroke has to suit the seed drill's
  mechanism, or be matched to it with the lever: fix the actuator to the lever at the
  point where its whole stroke gives the movement needed (further from the pivot for a
  longer stroke).
- **External end switches**, for an actuator with a longer stroke than needed (say 100 mm
  instead of 60 mm), or one without switches inside: two roller-lever microswitches per
  actuator, fixed on the frame and pressed by the moving part at each end. Use their
  **NC** contacts (closed until pressed), rated for direct current above the actuator's
  current (look for a 30 V DC rating, 5 A or more).

Both switches go **in series in the red wire**, between the relay (the Relay24_ESP32's
terminal pin 1, or the DPDT relay's `COM1`) and the actuator, each with a **1N5408 diode
across it** (3 A; a 6A10 for actuators over 3 A).
The diodes face opposite ways:

```
relay red ──┬── switch OUT (NC) ──┬──┬── switch IN (NC) ──┬── actuator red
            │                     │  │                    │
            └──|<|── diode OUT ───┘  └───|>|── diode IN ──┘
```

| Switch | Opens when | Its diode | Stripe (cathode) towards |
|---|---|---|---|
| OUT | the actuator is fully out (section open) | lets the current through only to pull back in | the relay |
| IN | the actuator is fully in (section closed) | lets the current through only to push out | the actuator |

How it works: pushing out, the current flows in the red wire from the relay to the
actuator. At the far end switch OUT opens, and its diode blocks that direction: the
actuator stops. When the relay drops, the current turns round, flows through the diode
of switch OUT, and the actuator pulls back; a few millimetres later switch OUT closes
again. Pulling in, it is the same with switch IN and its diode. The program, the board
and the relays do not change.

Check each actuator on the bench before the field: section on from AgOpenWeb, it must
push out and stop at switch OUT; section off, it must pull in and stop at switch IN. If it
does not move away from an end, that diode is the wrong way round.

## Bench test (before the tractor)

1. The board alone, no actuators connected. With the program closed every relay LED must
   be **off** (also while the ESP32 starts).
2. With AgOpenWeb running, turn sections on from the screen: relays 1–19 click, their
   LEDs light, and the page's *Sections* card shows 19 green cells.
3. Pull the network cable: every relay drops within 2 seconds, the page says *program
   silent, sections closed* and *Network cable: no link*.
4. Then one actuator (on the KC868-A32 with its DPDT relay): check the direction
   (on = out = open) and the stops at each end.

## What the AiO module page shows

The AiO v4.5 page's *Machine* tab shows every section the program asks for (PGN 254 for
1–16, PGN 229 above) and every section the machine modules report open (their PGN 237,
below, as any module in the AiO v26 layout sends it; several modules together), so here the
19 of the seed drill. The XIAO page reads PGN 254 only, so it shows sections 1–16.

## Forgotten Wi-Fi password

From the Serial Monitor (USB cable, 115200): type `wifi-reset`. The Wi-Fi settings go
back to the defaults (`Seeder-xxxx`, `agopengps`, network cable) and the board restarts.
The program's config stays.

The same with the **IO0 / BOOT** button held for 5 seconds while the board runs: on the
KC868-A32 always; on the Relay24_ESP32 only while it uses the tractor's Wi-Fi, because
with the network cable in use GPIO 0 carries the network clock.

## Messages

| PGN | Direction | What |
|---|---|---|
| 239 | program → module | sections 1–16 (bytes 11–12), hydraulic lift (7), tramline (8), geo stop (9), speed (6, km/h × 10) |
| 229 | program → module | sections 1–64 (bytes 5–12, 8 per byte, section 1 = byte 5 bit 0), sent with every 239 when the tool has more than 16 sections |
| 238 | program → module | machine config: raise / lower time, lift enable, byte 8 bit 0 = outputs inverted. Saved |
| 236 | program → module | 24 output functions: 1–16 section, 17 lift lower, 18 lift raise, 19 tramline right, 20 tramline left, 21 geo stop. Saved; used only with *Relay 1 follows section* at 0 |
| 201 | program → module | subnet; the module restarts as `<subnet>.123` (or the last number set on the page). Saved |
| 200 → 123 | hello / reply | reply carries the sections 1–16 really open (bytes 5–6) |
| 202 → 203 | scan / reply | the module's IP |
| 237 | module → network | every 200 ms, 14 data bytes: the layout of the AgOpenGPS AiO v26 firmware, sections 1–16 open (5–6), lift (7: 0 off, 1 down, 2 up), tramline (8), geo stop (9), status (10: bit 0 = the program's PGN 239 still arrives), 0, 0, then sections 17–64 open (13–18). The official Machine_UDP_v5 sends 8 bytes of fixed content here; the AiO Teensy reads the sections from it |

All messages: header `0x80 0x81`, source `0x7B` from the module, checksum = sum of the
bytes from the source byte to the one before the checksum. The module only accepts
messages with a right checksum from source `0x7F` (the program).

## Notes

- The page also answers `GET /aow/status` and announces itself as `_aow-ui._tcp` over
  mDNS, as the AgOpenGPS ISOBUS group's proposal for implements with their own page
  describes.
- Licence: GNU General Public License v3 or later (`Firmware/v4.5_AiO_WEB/COPYING`).
  Written 2026 by Claude (Anthropic) for renatorgr.
