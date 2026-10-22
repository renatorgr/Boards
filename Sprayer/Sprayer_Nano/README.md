# Sprayer_Nano

Section control for a 5-section, 8 m sprayer boom, driven by **AgOpenWeb** or
**AgOpenGPS** (6.8.6) over Ethernet. It also measures boom pressure and flow.

Version 0.2: compiles (about 10.7 kB of 30 kB), not yet tested on the sprayer.

## How it works

```
Tablet (AgOpenWeb / AgOpenGPS)
   │  Ethernet (router), UDP
   ▼
Arduino Nano + ENC28J60 ──► relay board ──► 5 solenoid valves (NC)
   ▲
   └── pressure sensor (A0) + flow meter (D2)
```

- The program tells the Nano which sections to open (PGN 239, byte 11).
  AgOpenWeb uses the same messages as AgOpenGPS, so one firmware works with both.
- The Nano is the standard machine module: **IP 192.168.5.123**.
- The valves are normally closed: no power = valve closed. If no messages
  arrive for 2 seconds (loose cable, frozen tablet), every section closes.
- The master switch cuts the 12 V to the relay COM terminals in the wiring,
  so it works even if the Nano or the tablet freezes.
- To open or close a section by hand, use the section buttons on the
  AgOpenWeb screen. The program then knows what is open and maps it correctly.

## Hardware

| Part | Notes |
|---|---|
| Arduino Nano + ENC28J60 Ethernet shield | CS on D10 |
| Blue 8-relay board, SRD-05VDC (5 V relays) | Switches on LOW. Optocoupler inputs |
| 5 × solenoid valve FPD-270A | 12 V, 0.3 A, NC, 0.2–8 bar, G1/2 |
| Pressure sensor 0–10 bar | 0.5–4.5 V output, powered at 5 V |
| YF-S201 flow meter | 5 V, about 450 pulses per litre, 1–30 L/min |
| LM2596 step-down converter | **Set to 5.0 V with a multimeter before connecting anything** |

## Pins

| Pin | Connection |
|---|---|
| D2 | Flow meter signal (yellow) |
| D3–D7 | Relay board IN1–IN5 (sections 1–5) |
| A0 | Pressure sensor signal (green), 10 k in series + 100 nF to GND |
| A6 | Master switch sense (optional, off by default) |
| D10–D13 | Ethernet shield |

## Power and wiring

- **Relay coils (JD-VCC):** 5 V from the LM2596. **Never 12 V**: the relays are
  5 V and will overheat and burn. Remove the VCC–JD-VCC jumper.
- **Relay board VCC** (next to IN1–IN8): 5 V from the Nano (optocouplers only).
- **Nano:** USB is fine for testing. On the tractor, feed the Nano's 5V pin from
  the LM2596 as well, so a loose USB cable can't drop the sections.
- **Valves:** 12 V from the battery, 5 A fuse → master switch → relay COM.
  Relay NO → valve +. Valve − → GND. 1N4007 diode across each valve (stripe to +).
- **Sensors:** red = 5 V, black = GND, signal to A0 / D2. Never power the YF-S201
  from 12 V: its signal would rise to 12 V and damage the Nano.
- Use a separate shielded cable for the sensors, with its own GND wire back to the
  Nano. Don't share the valve GND with the sensors.
- All grounds (battery, LM2596, Nano, relay board) joined together.

## Settings (top of `Sprayer_Nano.ino`)

| Setting | Default | Change it when |
|---|---|---|
| `RELAY_ON` | `LOW` | The relays switch on with HIGH (e.g. DEWIN board with jumpers on "High") |
| `PULSES_PER_LITRE` | `450.0` | After calibrating the flow meter (see below) |
| `PRESSURE_MAX_BAR` | `10.0` | A sensor with another range is fitted |
| `USE_MASTER_SWITCH` | `0` | The master switch is wired to A6 through a 10 k / 4.7 k divider |
| `NO_DATA_TIMEOUT_MS` | `2000` | You want sections to close faster or slower when the link drops |

## Uploading

1. Arduino IDE → Library Manager → install **EtherCard** (by Jean-Claude Wippler).
2. Open `Sprayer_Nano/Sprayer_Nano.ino`.
3. Board: Arduino Nano. Processor: ATmega328P (try "Old Bootloader" if the upload fails).
4. Upload, then open the Serial Monitor at **38400**. You should see `Network ready`.

## Bench test (before the tractor, no water)

1. Only the Nano, shield and relay board, valves disconnected.
2. With the program closed, the relay LEDs must be **off**. If they are all on,
   the board switches on HIGH: set `RELAY_ON` to `HIGH`.
3. The Serial Monitor prints one line per second:
   `Master ON | Sections 0 | 0.0 bar | 0.0 L/min | 0.0 L`
4. Connect the Ethernet cable to the router and open AgOpenWeb (or AgOpenGPS + AgIO).
   The machine module should show up at 192.168.5.123.
5. In the simulator, turn sections on from the screen: each relay should click
   and its LED light up.
6. Unplug the Ethernet cable: all relays switch off within 2 seconds.
7. Blow through the flow meter: the flow should go above 0.

## Calibrating the flow meter

With the pump running and one nozzle open, fill a 10 L bucket and note how many
litres the Serial Monitor counted. Then:

`PULSES_PER_LITRE = 450 × (litres counted ÷ 10)`

## Sprayer readings message (PGN 0xA0)

Sent once a second to UDP port 9999 (broadcast). AgOpenGPS and AgOpenWeb ignore
it for now. It is meant for a future pressure/flow display.

| Byte | Content |
|---|---|
| 0–1 | Header `0x80 0x81` |
| 2 | Source `0x7B` (123, machine module) |
| 3 | PGN `0xA0` |
| 4 | Length `10` |
| 5–6 | Pressure, hundredths of a bar (uint16, little-endian) |
| 7–8 | Flow, hundredths of a L/min (uint16) |
| 9–12 | Total, tenths of a litre (uint32) |
| 13 | Master switch (1 = on) |
| 14 | Sections on (bit 0 = section 1) |
| 15 | Checksum (sum of bytes 2–14) |

## Notes

- The FPD-270A valves and the YF-S201 are plastic and made for water. Rinse with
  clean water after each job. Fit a filter before the flow meter.
- The YF-S201 is only about ±10 % accurate. The boom uses about 15 L/min
  (Albuz AVI110 red nozzles), well inside its 1–30 L/min range.
