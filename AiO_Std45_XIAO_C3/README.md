# XIAO ESP32C3 adapter for the AiO v4.5 Standard (GPS2 socket → Serial2)

A small board (40.0 × 52.6 mm overall, 2 layers, cut away where the Bynav / UM982 module goes) that plugs into the **second GPS socket (GPS2, "F9P RelPosNED")** of the AiO v4.5 Standard, in place of a second simpleRTK2B, and connects a **Seeed XIAO ESP32C3** (or **ESP32S3**) to the **Teensy's Serial2** (pins 7 and 8). The XIAO plugs into two female headers on the adapter.

## Connections

| Socket pin | On the AiO v4.5 Standard | On the adapter |
|---|---|---|
| P8 pin 1 | Teensy pin 7 (RX2) | R1 1 kΩ → D6 (TX) |
| P8 pin 2 | Teensy pin 8 (TX2) | R2 1 kΩ → D7 (RX) |
| P8 pin 3 | pin 1 of H6 (GPS1 socket, dual F9P only) | not connected |
| P7 pin 4 | 5 V (5V1, K7805M 1 A regulator) | D1 (Schottky) → XIAO 5V pin + C1 22 µF |
| P7 pins 2, 3 | GND | GND |
| P7 pin 7 | 3.3 V of the Teensy (IOREF) | **not connected** |
| P7 pins 1, 5, 6, 8 | — | not connected |
| H14 pins 1–6 | — (A0–A5 of the socket, no connections) | not connected: they hold the adapter |

- XIAO ESP32C3: D6 = GPIO21 (TX), D7 = GPIO20 (RX).
- XIAO ESP32S3: D6 = GPIO43 (TX), D7 = GPIO44 (RX).

The positions of the 17 pins were copied from the official AiO v4.5 Standard PCB file (EasyEDA, 2024-02-05) and the nets were checked against its schematic (2024-03-27): 0.000 mm difference.

## Why 5 V and not 3.3 V

- On the Standard, the 3.3 V of the GPS2 socket (P7 pin 7, IOREF) is the Teensy's own 3.3 V output.
- PJRC recommends at most 250 mA from it for external use, and the board already uses it (BNO085, CAN, LEDs, ...). The ESP32's WiFi transmit peaks can go over that.
- The Micro is different: its GPS2 socket has its own 1 A 3.3 V regulator, so the Micro adapter uses 3.3 V.
- This adapter takes the 5 V of the socket (P7 pin 4) through a Schottky diode (D1) into the XIAO's 5V pin, and the XIAO makes its own 3.3 V. Seeed asks for this diode when the 5V pin is fed from outside.
- With D1, the XIAO's USB-C can be plugged in with the adapter on the board: D1 keeps the USB 5 V out of the AiO's 5 V.
- R1 and R2 are 1 kΩ (the Micro adapter too, from its v1.2). If the XIAO is on USB with the AiO switched off, its TX pin feeds the unpowered Teensy through R1; 1 kΩ keeps that current to about 3 mA. At 460800 baud, 1 kΩ makes no difference.

## Shape

- It plugs into P7 (power), P8 pins 1–3 (Serial2) and H14 (6 pins, only to hold it).
- The XIAO sits on the H14 / P8 side of the socket (towards the ADS1115), with its USB-C on the left edge.
- A strip along P7 carries the 5 V. Everything to the right of the XIAO and below that strip is cut away: from 4.0 mm to the right of P7 pin 1 and from 4.6 mm below the P7 row. The Bynav / UM982 module (and the U16 MP1584) can sit under that area.
- A leg goes down to P8 and stays at least 1.5 mm away from the UM982 board drawn on the official PCB (part GPS_UM982EB, socket H20), which covers P8 pins 5–8 and H7.
- P8 and P7 alone would let the adapter go in turned 180°. With H14 it cannot: turned around, its H14 pins land between the holes of H7.
- The outline stays inside the drawing of the GPS2 module on the Standard's silkscreen.

## Files

- `AiO_Std45_XIAO_C3.kicad_pro / .kicad_sch / .kicad_pcb`: KiCad 7 project (schematic + PCB), with its own libraries (`AiO_XIAO_Adapter.pretty`, `AiO_XIAO_Adapter.kicad_sym`).
- `fabrication_JLCPCB/Gerber_AiO_Std45_XIAO_C3.zip`: Gerbers + drill file, ready for JLCPCB.
- `fabrication_JLCPCB/BOM_…csv` and `CPL_…csv`: for JLCPCB's assembly (R1, R2, C1, D1 and the XIAO sockets J4, J5 on the top side, the pin headers J1, J2 and J3 on the bottom side).
- `fabrication_JLCPCB/BOM_…_top_only.csv` and `CPL_…_top_only.csv`: the same without J1, J2 and J3, for an assembly on the top side only.
- `documentation/`: the schematic as a PDF (`adapter_schematic.pdf`), views of the board (`adapter_PCB.png`) and its position on the Standard (`GPS2_socket_view.png`).

## Opening it in EasyEDA

EasyEDA Pro: **File → Import → KiCad**, then choose the project folder (or the .zip). It imports the schematic and the PCB.
EasyEDA is not needed to order the board: the Gerber zip goes straight to JLCPCB.

## Ordering (JLCPCB)

- Gerber: `Gerber_AiO_Std45_XIAO_C3.zip`. 2 layers, 1.6 mm, any colour. The default settings are fine (minimum track 0.3 mm, minimum hole 0.3 mm).
- Assembly: BOM + CPL. JLCPCB solders everything except the XIAO:
  - R1, R2, C1 and D1: SMD parts on the top side;
  - the pin headers J1, J2 and J3: through-hole parts on the **bottom** side, with the long pins pointing down (away from the board). The CPL already gives them on the bottom layer, at the centre of each header;
  - the two female headers J4 and J5 for the XIAO: through-hole parts on the **top** side.
- Through-hole assembly costs a little more than SMD only.
- There are parts on both sides, so choose **Standard PCBA** with assembly on **both sides**. With Economic PCBA or top side only, JLCPCB leaves out J1, J2 and J3 (they are on the bottom) and stops with "Coordinate file missing data for: J1, J2, J3".
- Cheaper alternative: Economic PCBA, top side only, with `BOM_…_top_only.csv` and `CPL_…_top_only.csv` (the same files without J1, J2 and J3), and solder J1, J2 and J3 by hand.
- In JLCPCB's placement preview, check:
  - J1, J2 and J3 on the bottom side, over the rows of holes marked "P8", "P7" and "H14";
  - J4 and J5 on the top side, over the two rows of 7 holes of the XIAO;
  - D1: its band (cathode) on the C1 side, as on the silkscreen.

| Ref. | Part | LCSC |
|---|---|---|
| R1, R2 | 1 kΩ 1 % 0805 | C17513 |
| C1 | 22 µF 0805 X5R 25 V | C45783 |
| D1 | B5819W Schottky 40 V 1 A, SOD-123 | C8598 |
| J1 | 1x03 male header, 2.54 mm, XFCN PZ254V-11-03P | C2937625 |
| J2 | 1x08 male header, 2.54 mm, XFCN PZ254V-11-08P | C492407 |
| J3 | 1x06 male header, 2.54 mm, XFCN PZ254V-11-06P | C492405 |
| J4, J5 | 1x07 female header, 2.54 mm, 8.5 mm high, HCTL PM254-1-07-Z-8.5 | C2897370 |
| U1 | Seeed XIAO ESP32C3 or ESP32S3, plugged into J4 / J5 | (not assembled) |

The headers have 6.1 mm mating pins and a 2.5 mm insulator, like the ones on the AiO board itself. Without assembly, J1, J2 and J3 can be cut from one ordinary 1x40 strip of 2.54 mm male headers.

## Assembly

1. Only if the board did not come assembled: R1, R2, C1 and D1 (D1: the band towards C1), then the headers.
2. **The male headers go underneath**, with the long pins pointing down. When soldering them by hand, to get them well aligned: push them into the Standard's P8, P7 and H14, lay the adapter on top and solder them from above.
   - J1 goes into **P8 pins 1 to 3**: the three holes at the end of P8 closest to the ADS1115 module.
   - J2 goes into P7 (all 8 pins) and J3 into H14 (all 6 pins).
3. **XIAO:** solder the two 1x7 male headers that come with it **underneath the XIAO**, long pins pointing down. To keep them straight, push them into J4 and J5 first, lay the XIAO on top and solder from above.
4. **Plug the XIAO into J4 and J5 with its USB-C at the "USB-C" mark** (left edge, ADS1115 side). The two rows have 7 pins each, so the XIAO also goes in turned 180°: that way the 5 V goes into D6 (a 3.3 V pin) and the XIAO is damaged. Check the USB-C mark before powering the AiO.
5. Orientation of the adapter: the square pads are pin 1, and the silkscreen says "P8", "P7" and "H14" next to the header that goes into each one. The H14 header stops it going in turned around.

## Cautions

- With the adapter in place, the GPS2 socket cannot hold a second F9P (dual F9P).
- The Bynav / UM982 module (H20) and the XIAO can be fitted at the same time. They use different serial ports (the module the GPS1 port, the XIAO Serial2), and both take their 5 V from the 1 A K7805M (5V1).
- Check that the antenna cables of the Bynav / UM982 module do not press on the adapter.
- If there is a standoff in the GPS2 mounting hole next to the ADS1115 (left there for a simpleRTK2B), take it out: it would push on the adapter's leg.
- The XIAO's USB-C can be used with the adapter on the board (see "Why 5 V"): the plug goes in from the left, over the ADS1115 side. The XIAO can also be unplugged to program it, or programmed over WiFi (OTA) after the first time.
- On the sockets the XIAO sits about 11 mm higher than soldered flat (8.5 mm socket + 2.5 mm of the male header). Check that the box lid closes.
- Tractor vibration: hold the XIAO in its sockets, for example with a cable tie around the XIAO and the adapter, or a dot of neutral-cure silicone at the corners (not on the U.FL connector or the buttons).
- In a metal box, use an external WiFi antenna on the XIAO's U.FL connector, taken out through the box.

## Firmware

- Teensy: firmware `v4.5_AiO_WEB`, Serial2 (pins 7 RX2 and 8 TX2) at **460800**. On its web page, Settings → "GPS2 port" must be on automatic (the default): the Teensy finds the XIAO by its hello.
- XIAO: the program `v4.5_XIAO_ESP32` that comes in the firmware zip (already compiled in `v4.5_XIAO_ESP32_bin`, for the C3 and the S3). It uses UART1 on D7 (RX) and D6 (TX) at 460800 and empties the receive FIFO at 64 bytes (`setRxFIFOFull(64)`).
- When it starts, the ESP32 writes a few boot lines at 115200 on D6: the Teensy receives garbage for half a second, and the firmware ignores it.
- The official firmware v26 (Firmware_Teensy_AiO-New-Dawn) uses this same link (Serial2) for its ESP32, also at 460800. See `docs/ESP32_BRIDGE_INTERFACE.md` in that repository.

## Checks done

- The schematic and the PCB have the same connections (netlist exported from the schematic = PCB connections).
- KiCad 7 DRC: 0 errors, 0 warnings, 0 unconnected items.
- The 17 header holes in the drill file match P8 pins 1–3, P7 and H14 of the official AiO v4.5 Standard PCB (AgOpenGPS-Official/Boards): 0.000 mm difference.
- The nets of P8, P7 and H14 checked against the official AiO v4.5 Standard schematic.
- The outline is at least 1.5 mm away from the UM982 board (GPS_UM982EB) drawn on the official PCB, and nothing of the adapter is right of the cut (4.0 mm right of P7 pin 1, 4.6 mm below the P7 row).
- The XIAO sockets J4 and J5 are on the XIAO pin positions of Seeed's official KiCad library (XIAO-ESP32-C3-DIP-SMD): 2 × 7 pins, 2.54 mm, rows 15.24 mm apart.
- J1, J2 and J3 are on the bottom side and J4, J5 on the top side, in the PCB and in the CPL. The holes are the same as in 1.1.
- Not yet made or tested on a real board.

## Versions

- **1.2** (19 Oct 2026): pin headers J1, J2 and J3 with LCSC codes in the BOM and the CPL (bottom side), so JLCPCB solders them too. The XIAO plugs into two 1x7 female headers (J4, J5, top side, also soldered by JLCPCB) instead of being soldered to the board.
- **1.1** (19 Oct 2026): XIAO moved to the H14 / P8 side, USB-C on the left edge; board cut away to the right of the XIAO so the Bynav / UM982 module can be fitted under that area.
- **1.0** (19 Oct 2026): first version (not made).

## Credits

- Design: renatorgr, with Claude (Anthropic), 2026.
- Pin positions and nets: official AiO v4.5 Standard board files (EasyEDA PCB 2024-02-05, schematic 2024-03-27), AgOpenGPS-Official/Boards.
- XIAO pin positions: Seeed Studio OPL KiCad library (XIAO-ESP32-C3-DIP-SMD). 0805 and SOD-123 footprints: KiCad standard library.
