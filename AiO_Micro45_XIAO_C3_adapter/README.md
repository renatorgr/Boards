# XIAO ESP32C3 adapter for the AiO v4.5 Micro (GPS2 socket → Serial2)

A small board (27.3 × 24.4 mm, 2 layers) that plugs into the **second GPS socket ("Dual-Left", U11)** of the AiO v4.5 Micro, in place of a simpleRTK2B Micro, and connects a **Seeed XIAO ESP32C3** to the **Teensy's Serial2** (pins 7 and 8).

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

The positions of the 20 pins were copied from the official AiO v4.5 Micro PCB file (EasyEDA, 2024-02-05) and checked against the schematic: 0.00 mm difference. The outline stays inside the drawing of the original module on the Micro's silkscreen, with the same cut corner (on the side of LED1 and the RJ45 jack).

## Files

- `AiO_Micro45_XIAO_C3.kicad_pro / .kicad_sch / .kicad_pcb`: KiCad 7 project (schematic + PCB), with its own libraries (`AiO_XIAO_Adapter.pretty`, `AiO_XIAO_Adapter.kicad_sym`).
- `fabrication_JLCPCB/Gerber_AiO_Micro45_XIAO_C3.zip`: Gerbers + drill file, ready for JLCPCB.
- `fabrication_JLCPCB/BOM_…csv` and `CPL_…csv`: for JLCPCB's SMT assembly (R1, R2 and C1 only).
- `documentation/`: the schematic as a PDF (`adapter_schematic.pdf`), views of the board (`adapter_PCB.png`) and its position on the Micro (`GPS2_socket_view.png`).

## Opening it in EasyEDA

EasyEDA Pro: **File → Import → KiCad**, then choose the project folder (or the .zip). It imports the schematic and the PCB.
EasyEDA is not needed to order the board: the Gerber zip goes straight to JLCPCB.

## Ordering (JLCPCB)

- Gerber: `Gerber_AiO_Micro45_XIAO_C3.zip`. 2 layers, 1.6 mm, any colour. The default settings are fine (minimum track 0.3 mm, minimum hole 0.3 mm).
- SMT assembly (optional): BOM + CPL. They are only three 0805 parts and they are easy to solder by hand.

| Ref. | Part | LCSC |
|---|---|---|
| R1, R2 | 100 Ω 1 % 0805 | C17408 |
| C1 | 22 µF 0805 X5R | C45783 |
| J1 | 2 × 1x10 male header, **2.00 mm pitch** | (solder by hand) |
| U1 | Seeed XIAO ESP32C3 | (solder by hand) |

## Assembly

1. R1, R2 and C1 (if they did not come assembled).
2. **The 2.00 mm male headers go underneath**, with the pins pointing down. To get them well aligned: push them into the Micro's GPS2 socket, lay the adapter on top and solder them from above.
3. **XIAO on top**, with the USB-C on the side of the "USB-C" mark:
   - preferably **soldered flat** (by the half-holes on its edge): it sits lower, stands vibration better and leaves no pin ends underneath;
   - if you use the pins that come with the XIAO, **cut the ends flush underneath**. Under the adapter, on the Micro, there is resistor R31, right below the D0–D6 row.
4. Orientation on the Micro: the cut corner of the adapter matches the cut corner drawn on the silkscreen of the GPS2 socket, and pin 1 (square pad, dot on the silkscreen) sits next to pin 1 of the socket.

## Cautions

- The XIAO is powered through its 3V3 pin (the board's 3.3 V). **Do not plug in the XIAO's USB-C with the adapter mounted on the Micro.** Program it off the board, or over WiFi (OTA) after the first time.
- Hammond 1455 boxes are aluminium: use an external WiFi antenna on the XIAO's U.FL connector, taken out through the lid.
- With the adapter in place the GPS2 socket cannot hold a second F9P (dual F9P).

## Firmware

- Teensy: firmware `v4.5_UM982_X20P_F9P_TM171_WEB_XIAO` or `v4.5_AiO_WEB`, Serial2 (pins 7 RX2 and 8 TX2) at **460800**. With `v4.5_AiO_WEB`, the web page setting "GPS2 port" must be on XIAO ESP32 (the default).
- XIAO: the program `v4.5_XIAO_ESP32` that comes in the firmware zip (already compiled in `v4.5_XIAO_ESP32_bin`). It uses UART1 on D7 (RX, GPIO20) and D6 (TX, GPIO21) at 460800 and empties the receive FIFO at 64 bytes (`setRxFIFOFull(64)`).
- The XIAO ESP32S3 fits too: same size and pins (D6 = GPIO43 TX, D7 = GPIO44 RX). The adapter has no copper under its extra pads underneath.
- When it starts, the ESP32-C3 writes a few boot lines at 115200 on GPIO21: the Teensy receives garbage for half a second, and the firmware ignores it.
- The official firmware v26 (Firmware_Teensy_AiO-New-Dawn) uses this same link (Serial2) for its ESP32, also at 460800. See `docs/ESP32_BRIDGE_INTERFACE.md` in that repository.

## Checks done

- The schematic and the PCB have the same connections (netlist exported from the schematic = PCB connections).
- KiCad 7 DRC: 0 errors, 0 unconnected items.
- Pins and connections checked against the official AiO v4.5 Micro schematic and PCB (AgOpenGPS-Official/Boards).
- XIAO footprint as in Seeed's official KiCad library (XIAO-ESP32-C3-DIP-SMD).
- Not yet made or tested on a real board.
