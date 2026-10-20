# XIAO ESP32C3 adapter for the AiO v4.5 Micro (GPS2 socket → Serial2)

A small board (27.3 × 24.4 mm, 2 layers) that plugs into the **second GPS socket ("Dual-Left", U11)** of the AiO v4.5 Micro, in place of a simpleRTK2B Micro, and connects a **Seeed XIAO ESP32C3** (or **ESP32S3**) to the **Teensy's Serial2** (pins 7 and 8). The XIAO plugs into two female headers on the adapter.

## Connections

| Socket pin | On the AiO v4.5 Micro | On the adapter |
|---|---|---|
| 1 | 3.3 V (1 A K7803M regulator) | XIAO 3V3 + C1 22 µF |
| 2 | Teensy pin 7 (RX2) | R1 100 Ω → D6 (TX, GPIO21) |
| 3 | Teensy pin 8 (TX2) | R2 100 Ω → D7 (RX, GPIO20) |
| 10, 11 | GND | GND |
| 6, 7, 8 | F9P micro-USB | not connected |
| 16 | pin 12 of the GPS1 socket (dual F9P only) | not connected |
| 18 | LED RTK_STAT_1 | not connected |
| others | — | not connected |

The plug is two 1x10 male headers: J1 = socket pins 1–10, J2 = socket pins 11–20 (J2 pin 1 = socket pin 11). The XIAO plugs into two 1x7 female headers: J3 = XIAO pins 1–7 (D0–D6), J4 = XIAO pins 8–14 (D7–5V).

The positions of the 20 pins were copied from the official AiO v4.5 Micro PCB file (EasyEDA, 2024-02-05) and checked against the schematic: 0.00 mm difference. The outline stays inside the drawing of the original module on the Micro's silkscreen, with the same cut corner (on the side of LED1 and the RJ45 jack).

## Files

- `AiO_Micro45_XIAO_C3.kicad_pro / .kicad_sch / .kicad_pcb`: KiCad 7 project (schematic + PCB), with its own libraries (`AiO_XIAO_Adapter.pretty`, `AiO_XIAO_Adapter.kicad_sym`).
- `fabrication_JLCPCB/Gerber_AiO_Micro45_XIAO_C3.zip`: Gerbers + drill file, ready for JLCPCB.
- `fabrication_JLCPCB/BOM_…csv` and `CPL_…csv`: for JLCPCB's assembly (R1, R2, C1 and the XIAO sockets J3, J4 on the top side, the two pin headers J1 and J2 on the bottom side).
- `fabrication_JLCPCB/BOM_…_top_only.csv` and `CPL_…_top_only.csv`: the same without J1 and J2, for an assembly on the top side only.
- `documentation/`: the schematic as a PDF (`adapter_schematic.pdf`), views of the board (`adapter_PCB.png`) and its position on the Micro (`GPS2_socket_view.png`).

## Opening it in EasyEDA

EasyEDA Pro: **File → Import → KiCad**, then choose the project folder (or the .zip). It imports the schematic and the PCB.
EasyEDA is not needed to order the board: the Gerber zip goes straight to JLCPCB.

## Ordering (JLCPCB)

- Gerber: `Gerber_AiO_Micro45_XIAO_C3.zip`. 2 layers, 1.6 mm, any colour. The default settings are fine (minimum track 0.3 mm, minimum hole 0.3 mm).
- Assembly: BOM + CPL. JLCPCB solders everything except the XIAO:
  - R1, R2 and C1: SMD parts on the top side;
  - the pin headers J1 and J2: through-hole parts on the **bottom** side, with the long pins pointing down (away from the board). The CPL already gives them on the bottom layer, at the centre of each header;
  - the two female headers J3 and J4 for the XIAO: through-hole parts on the **top** side.
- Through-hole assembly costs a little more than SMD only.
- There are parts on both sides, so choose **Standard PCBA** with assembly on **both sides**. With Economic PCBA or top side only, JLCPCB leaves out J1 and J2 (they are on the bottom) and stops with "Coordinate file missing data for: J1, J2".
- Cheaper alternative: Economic PCBA, top side only, with `BOM_…_top_only.csv` and `CPL_…_top_only.csv` (the same files without J1 and J2), and solder J1 and J2 by hand.
- In JLCPCB's placement preview, check that J1 and J2 are on the bottom side, over the two rows of 10 holes, and J3 and J4 on the top side, over the two rows of 7 holes.

| Ref. | Part | LCSC |
|---|---|---|
| R1, R2 | 100 Ω 1 % 0805 | C17408 |
| C1 | 22 µF 0805 X5R | C45783 |
| J1, J2 | 1x10 male header, **2.00 mm pitch**, HCTL PZ200-1-10-Z | C2905956 |
| J3, J4 | 1x07 female header, 2.54 mm, 8.5 mm high, HCTL PM254-1-07-Z-8.5 | C2897370 |
| U1 | Seeed XIAO ESP32C3 or ESP32S3, plugged into J3 / J4 | (not assembled) |

The 2.00 mm headers have 4 mm mating pins and a 2 mm insulator.

## Assembly

1. Only if the board did not come assembled: R1, R2 and C1, then the headers.
2. **The 2.00 mm male headers go underneath**, with the pins pointing down. When soldering them by hand, to get them well aligned: push them into the Micro's GPS2 socket, lay the adapter on top and solder them from above.
3. **XIAO:** solder the two 1x7 male headers that come with it **underneath the XIAO**, long pins pointing down. To keep them straight, push them into J3 and J4 first, lay the XIAO on top and solder from above.
4. **Plug the XIAO into J3 and J4 with its USB-C at the "USB-C" mark.** The two rows have 7 pins each, so the XIAO also goes in turned 180°: that way the 3.3 V goes into D4 instead of 3V3 and the XIAO can be damaged. Check the USB-C mark before powering the AiO.
5. Under J3 (the D0–D6 row), on the Micro, there is resistor R31: check that the solder joints of J3 underneath the adapter do not touch it.
6. Orientation on the Micro: the cut corner of the adapter matches the cut corner drawn on the silkscreen of the GPS2 socket, and pin 1 (square pad, dot on the silkscreen) sits next to pin 1 of the socket.

## Cautions

- The XIAO is powered through its 3V3 pin (the board's 3.3 V). **Do not plug in the XIAO's USB-C while it is on the adapter on the Micro.** Unplug the XIAO to program it by USB, or program it over WiFi (OTA) after the first time.
- On the sockets the XIAO sits about 11 mm higher than soldered flat (8.5 mm socket + 2.5 mm of the male header). Check that the box lid closes.
- Tractor vibration: hold the XIAO in its sockets, for example with a cable tie around the XIAO and the adapter, or a dot of neutral-cure silicone at the corners (not on the U.FL connector or the buttons).
- Hammond 1455 boxes are aluminium: use an external WiFi antenna on the XIAO's U.FL connector, taken out through the lid.
- With the adapter in place the GPS2 socket cannot hold a second F9P (dual F9P).

## Firmware

- Teensy: firmware `v4.5_AiO_WEB`, Serial2 (pins 7 RX2 and 8 TX2) at **460800**. On its web page, Settings → "GPS2 port" must be on XIAO ESP32 (the default).
- XIAO: the program `v4.5_XIAO_ESP32` that comes in the firmware zip (already compiled in `v4.5_XIAO_ESP32_bin`). It uses UART1 on D7 (RX, GPIO20) and D6 (TX, GPIO21) at 460800 and empties the receive FIFO at 64 bytes (`setRxFIFOFull(64)`).
- The XIAO ESP32S3 fits too: same size and pins (D6 = GPIO43 TX, D7 = GPIO44 RX).
- When it starts, the ESP32-C3 writes a few boot lines at 115200 on GPIO21: the Teensy receives garbage for half a second, and the firmware ignores it.
- The official firmware v26 (Firmware_Teensy_AiO-New-Dawn) uses this same link (Serial2) for its ESP32, also at 460800. See `docs/ESP32_BRIDGE_INTERFACE.md` in that repository.

## Checks done

- The schematic and the PCB have the same connections (netlist exported from the schematic = PCB connections).
- KiCad 7 DRC: 0 errors, 0 unconnected items.
- Pins and connections checked against the official AiO v4.5 Micro schematic and PCB (AgOpenGPS-Official/Boards).
- The XIAO sockets J3 and J4 are on the XIAO pin positions of Seeed's official KiCad library (XIAO-ESP32-C3-DIP-SMD): 2 × 7 pins, 2.54 mm, rows 15.24 mm apart.
- J1 and J2 are on the bottom side and J3, J4 on the top side, in the PCB and in the CPL; the drill file has the same 44 holes as version 1.0.
- Not yet made or tested on a real board.

## Versions

- **1.1** (19 Oct 2026): the plug is now two 1x10 2.00 mm headers (J1, J2) with LCSC codes in the BOM and the CPL (bottom side), so JLCPCB solders them too. The XIAO plugs into two 1x7 female headers (J3, J4, top side, also soldered by JLCPCB) instead of being soldered to the board. Same holes and tracks as 1.0.
- **1.0** (14 Oct 2026): first version.

## Credits

- Design: renatorgr, with Claude (Anthropic), 2026.
- Pin positions: official AiO v4.5 Micro board files (EasyEDA PCB 2024-02-05 and schematic), AgOpenGPS-Official/Boards.
- XIAO pin positions: Seeed Studio OPL KiCad library (XIAO-ESP32-C3-DIP-SMD). 0805 footprints: KiCad standard library.
