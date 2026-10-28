#!/usr/bin/env python3
"""
Relay24_ESP32: the parts, nets and positions of the 24-section relay board.

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

This file is the single source of the design: make_sch.py writes the KiCad 7 schematic
from it, make_pcb.py writes the PCB from it (so the two always agree), make_fab.py the
JLCPCB files. Everything is in millimetres; board coordinates have X to the right and
Y down, the origin at the top-left corner of the board.

What the board does (see ../README.md):
- an ESP32 DevKitC V4 (38 pins) plugged in two 1x19 sockets runs Seeder_ESP32 or
  Sprayer_ESP32;
- three 74HC595 shift registers + three ULN2803A drive 24 DPDT relays HF115F/012-2ZS4;
  each relay reverses one 12 V linear actuator: relay off = pull in (section closed);
- a LAN8720A + RJ45 give the network cable (RMII, 50 MHz clock from GPIO17, like the
  KinCony KC868-A32), the ESP32 Wi-Fi stays available;
- 12 V in -> LM2596S-5.0 -> 5 V (DevKitC) -> AMS1117-3.3 (logic, PHY);
- the actuator power comes in on its own terminals, one pair of rails per relay row.
"""

# ----------------------------------------------------------------------------- board
BOARD_W = 210.0
BOARD_H = 120.0
PROJECT = "Relay24_ESP32"
TITLE = "Relay24_ESP32 - 24-section relay board for an ESP32 DevKitC (AgOpenGPS machine module)"
REV = "1.0"
DATE = "2026-10-28"

CELL_PITCH = 14.0          # one relay + one 2-pin terminal per 14 mm
CELL0_X = 20.25            # centre of the first cell (section 1 / section 24)
COIL_Y_TOP = 38.5          # coil pads of the top row of relays (contacts towards the top edge)
COIL_Y_BOT = BOARD_H - COIL_Y_TOP
TERM_Y_TOP = 5.5           # terminal pins of the top row
TERM_Y_BOT = BOARD_H - TERM_Y_TOP
ESP_Y = 60.0               # centre line of the DevKitC
ESP_ROW_DY = 12.7          # half the distance between its two pin rows (25.4 mm)
ESP_PIN1_X = 52.06         # pin 1 (3V3 / GND end, antenna end); pin 19 is at the USB end
RJ45_POS = (192.7, 63.0)   # footprint origin, rotated 90: the opening at the right edge

FP = {
    "relay": ("Relay_THT", "Relay_DPDT_Omron_G2RL-2"),
    "term2": ("TerminalBlock", "TerminalBlock_bornier-2_P5.08mm"),
    "sock19": ("Connector_PinSocket_2.54mm", "PinSocket_1x19_P2.54mm_Vertical"),
    "hdr4": ("Connector_PinHeader_2.54mm", "PinHeader_1x04_P2.54mm_Vertical"),
    "soic16": ("Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm"),
    "soic18w": ("Package_SO", "SOIC-18W_7.5x11.6mm_P1.27mm"),
    "qfn24": ("Package_DFN_QFN", "VQFN-24-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias"),
    "to263": ("Package_TO_SOT_SMD", "TO-263-5_TabPin3"),
    "sot223": ("Package_TO_SOT_SMD", "SOT-223-3_TabPin2"),
    "rj45": ("Connector_RJ", "RJ45_Hanrun_HR911105A_Horizontal"),
    "r0603": ("Resistor_SMD", "R_0603_1608Metric"),
    "c0603": ("Capacitor_SMD", "C_0603_1608Metric"),
    "c0805": ("Capacitor_SMD", "C_0805_2012Metric"),
    "led0603": ("LED_SMD", "LED_0603_1608Metric"),
    "sma": ("Diode_SMD", "D_SMA"),
    "smb": ("Diode_SMD", "D_SMB"),
    "elec8": ("Capacitor_SMD", "CP_Elec_8x10"),
    "l12": ("Inductor_SMD", "L_12x12mm_H8mm"),
    "hole": ("MountingHole", "MountingHole_3.2mm_M3"),
}

# ----------------------------------------------------------------------------- parts
# One entry per component. "sym" = (library, symbol) in KiCad's standard libraries,
# "fp" = key in FP, "pins" = {pin number: net name}, "pcb" = (x, y, rotation, side),
# "sch" = (sheet, x, y, rotation). Nets named "NC_*" are left unconnected.
PARTS = {}


def add(ref, sym, fp, value, pins, pcb, sch, lcsc="", desc="", jlc_note=""):
    PARTS[ref] = dict(ref=ref, sym=sym, fp=FP[fp], value=value, pins=pins, pcb=pcb, sch=sch,
                      lcsc=lcsc, desc=desc, note=jlc_note)


def cell_x(j):
    """Centre of cell j (0..11, left to right)."""
    return CELL0_X + CELL_PITCH * j


def section_cell(n):
    """Section n (1..24) -> (cell index, row). Top row 1..12 left to right, bottom row
    13..24 right to left, so 13 is under 12 and the wiring stays short."""
    return (n - 1, "top") if n <= 12 else (24 - n, "bot")


def relay_rail(n):
    return "12V_ACT_A" if n <= 12 else "12V_ACT_B"


# --- relays, terminals, LEDs -----------------------------------------------------------
# Relay pin numbers follow the DIN numbering of the footprint: A1/A2 coil, 11/21 common,
# 12/22 normally closed, 14/24 normally open.
for n in range(1, 25):
    j, row = section_cell(n)
    xc = cell_x(j)
    rail = relay_rail(n)
    bank = (n - 1) // 8                      # 0..2 -> driver pair
    sheet = f"relays_{bank * 8 + 1}_{bank * 8 + 8}"
    i = (n - 1) % 8                          # position inside the sheet
    sx, sy = 40 + (i % 4) * 95, 60 + (i // 4) * 120
    if row == "top":
        relay_pos = (xc + 3.75, COIL_Y_TOP, 180)    # pole 1 at xc+3.75, pole 2 at xc-3.75
        term_pos = (xc + 2.54, TERM_Y_TOP, 180)     # pad 1 at xc+2.54 (red), pad 2 at xc-2.54
        r_pos = (xc + 1.6, COIL_Y_TOP + 4.4, 0)        # LED and resistor just below the relay body
        led_pos = (xc - 1.6, COIL_Y_TOP + 4.4, 0)
    else:
        relay_pos = (xc - 3.75, COIL_Y_BOT, 0)      # pole 1 at xc-3.75, pole 2 at xc+3.75
        term_pos = (xc - 2.54, TERM_Y_BOT, 0)       # pad 1 at xc-2.54 (red), pad 2 at xc+2.54
        r_pos = (xc - 1.6, COIL_Y_BOT - 4.4, 180)
        led_pos = (xc + 1.6, COIL_Y_BOT - 4.4, 180)
    add(f"K{n}", ("Relay", "G2RL-2"), "relay", "HF115F/012-2ZS4",
        {"A1": "+12V", "A2": f"RLY{n}", "11": f"SEC{n}_R", "12": "GND", "14": rail,
         "21": f"SEC{n}_B", "22": rail, "24": "GND"},
        relay_pos + ("F",), (sheet, sx, sy, 0),
        lcsc="C163534", desc="Relay DPDT 8 A, 12 V coil 360 ohm, 29x12.7 mm (RT2 / G2RL-2 layout)")
    add(f"J{n}", ("Connector", "Screw_Terminal_01x02"), "term2", f"Section {n}",
        {"1": f"SEC{n}_R", "2": f"SEC{n}_B"},
        term_pos + ("F",), (sheet, sx + 35, sy - 2.54, 0),
        lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P (pin 1 red, pin 2 black)",
        jlc_note="wire openings towards the board edge")
    add(f"D{n}", ("Device", "LED"), "led0603", "green",
        {"1": f"RLY{n}", "2": f"LEDA{n}"},
        led_pos + ("F",), (sheet, sx - 25, sy - 10.16, 180),
        lcsc="C965805", desc="LED 0603 green (yellow-green, XINGLIGHT XL-1608SYGC-06), on when the section is open")
    add(f"R{n}", ("Device", "R"), "r0603", "2.2k",
        {"1": f"LEDA{n}", "2": "+12V"},
        r_pos + ("F",), (sheet, sx - 25, sy - 20.32, 90),
        lcsc="C4190", desc="Resistor 0603 2.2 kohm (section LED)")

# --- power terminals for the actuators -------------------------------------------------
add("J26", ("Connector", "Screw_Terminal_01x02"), "term2", "12V ACT A (left)",
    {"1": "12V_ACT_A", "2": "GND"}, (3.5 + 5.08, TERM_Y_TOP, 180, "F"), ("root", 40, 300, 0),
    lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P: + and - of the actuator supply, sections 1-12")
add("J27", ("Connector", "Screw_Terminal_01x02"), "term2", "12V ACT A (right)",
    {"1": "12V_ACT_A", "2": "GND"}, (BOARD_W - 3.5, TERM_Y_TOP, 180, "F"), ("root", 40, 315, 0),
    lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P: same rail as J26 (feed from either end or both)")
add("J28", ("Connector", "Screw_Terminal_01x02"), "term2", "12V ACT B (left)",
    {"1": "12V_ACT_B", "2": "GND"}, (3.5, TERM_Y_BOT, 0, "F"), ("root", 40, 330, 0),
    lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P: + and - of the actuator supply, sections 13-24")
add("J29", ("Connector", "Screw_Terminal_01x02"), "term2", "12V ACT B (right)",
    {"1": "12V_ACT_B", "2": "GND"}, (BOARD_W - 3.5 - 5.08, TERM_Y_BOT, 0, "F"), ("root", 40, 345, 0),
    lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P: same rail as J28")

# --- 12 V in, protection, 5 V buck, 3.3 V LDO --------------------------------------------
add("J25", ("Connector", "Screw_Terminal_01x02"), "term2", "12V IN",
    {"1": "+12V_IN", "2": "GND"}, (BOARD_W - 4.5, 81.0, 90, "F"), ("root", 30, 40, 0),
    lcsc="C8465", desc="Screw terminal 2 pins 5.08 mm KF301-5.0-2P: 12 V for the board and the relay coils (2 A fuse)")
add("D25", ("Device", "D_Schottky"), "sma", "SS54",
    {"1": "+12V", "2": "+12V_IN"}, (205.5, 88.5, 0, "F"), ("root", 50, 37.46, 0),
    lcsc="C22452", desc="Schottky 5 A 40 V SMA: reverse polarity protection")
add("D26", ("Device", "D_TVS"), "smb", "SMBJ18A",
    {"1": "+12V", "2": "GND"}, (204.0, 104.5, 0, "F"), ("root", 65, 50, 90),
    lcsc="C151256", desc="TVS 18 V standoff SMB (DO-214AA, Littelfuse SMBJ18A): spikes from the tractor")
add("C25", ("Device", "C_Polarized"), "elec8", "100uF 50V",
    {"1": "+12V", "2": "GND"}, (204.0, 96.0, 0, "F"), ("root", 75, 50, 0),
    lcsc="C97812", desc="Electrolytic SMD 8x10.5 mm 100 uF 50 V (CS1H101M-CRF10): buck input")
add("U7", ("Regulator_Switching", "LM2596S-5"), "to263", "LM2596S-5.0",
    {"1": "+12V", "2": "SW", "3": "GND", "4": "+5VB", "5": "GND"},
    (192.0, 102.0, 270, "F"), ("root", 95, 45, 0),
    lcsc="C347421", desc="Buck regulator 5 V 3 A TO-263-5 (UMW LM2596S-5.0)")
add("D27", ("Device", "D_Schottky"), "sma", "SS54",
    {"1": "SW", "2": "GND"}, (198.5, 88.5, 0, "F"), ("root", 112, 55, 90),
    lcsc="C22452", desc="Schottky 5 A 40 V SMA: buck catch diode")
add("L1", ("Device", "L"), "l12", "33uH 3A",
    {"1": "SW", "2": "+5VB"}, (189.5, 74.8, 180, "F"), ("root", 120, 42.46, 90),
    lcsc="C2929502", desc="Power inductor 33 uH 3 A, 12.3x12.3 mm SMD (SHOU HAN CYH127-33UH)")
add("C26", ("Device", "C_Polarized"), "elec8", "220uF 25V",
    {"1": "+5VB", "2": "GND"}, (187.5, 87.0, 180, "F"), ("root", 130, 50, 0),
    lcsc="C165536", desc="Electrolytic SMD 8x10 mm 220 uF 25 V (Samyoung MVK 25V220): buck output")
add("D28", ("Device", "D_Schottky"), "sma", "SS34",
    {"1": "+5V", "2": "+5VB"}, (152.0, 60.5, 0, "F"), ("root", 140, 42.46, 0),
    lcsc="C8678", desc="Schottky 3 A SMA: lets the DevKitC USB power the logic without feeding the buck")
add("C27", ("Device", "C"), "c0805", "10uF",
    {"1": "+5V", "2": "GND"}, (145.0, 60.5, 90, "F"), ("root", 150, 50, 0),
    lcsc="C15850", desc="Capacitor 0805 10 uF 25 V")
add("U8", ("Regulator_Linear", "AMS1117-3.3"), "sot223", "AMS1117-3.3",
    {"1": "GND", "2": "+3V3", "3": "+5V"}, (136.0, 66.5, 0, "F"), ("root", 165, 45, 0),
    lcsc="C6186", desc="LDO 3.3 V 1 A SOT-223: PHY, shift registers, pull-ups")
add("C28", ("Device", "C"), "c0805", "10uF",
    {"1": "+5V", "2": "GND"}, (136.0, 72.5, 0, "F"), ("root", 158, 52.5, 0),
    lcsc="C15850", desc="Capacitor 0805 10 uF 25 V")
add("C29", ("Device", "C"), "c0805", "10uF",
    {"1": "+3V3", "2": "GND"}, (145.0, 66.5, 90, "F"), ("root", 175, 52.5, 0),
    lcsc="C15850", desc="Capacitor 0805 10 uF 25 V")
add("D29", ("Device", "LED"), "led0603", "red",
    {"1": "PWR_K", "2": "+3V3"}, (145.0, 72.5, 0, "F"), ("root", 185, 52.5, 90),
    lcsc="C2286", desc="LED 0603 red: 3.3 V present")
add("R29", ("Device", "R"), "r0603", "1k",
    {"1": "PWR_K", "2": "GND"}, (149.0, 72.5, 0, "F"), ("root", 185, 62.5, 90),
    lcsc="C21190", desc="Resistor 0603 1 kohm")

# --- the ESP32 DevKitC V4 sockets -------------------------------------------------------
# J30 = the 3V3 ... 5V row (J2 of the DevKitC), J31 = the GND ... CLK row (J3). Pin 1 at
# the antenna end (right), pin 19 at the USB end (the left board edge).
ESP_J2 = ["NC_3V3", "NC_EN", "NC_VP", "NC_VN", "A34", "A35", "SR_RCLK", "ETH_RST", "RMII_RXD0",
          "RMII_RXD1", "RMII_CRS_DV", "I2C_SCL", "NC_IO12", "GND", "SR_SER", "NC_D2", "NC_D3",
          "NC_CMD", "+5V"]
ESP_J3 = ["GND", "RMII_MDC", "RMII_TXD1", "NC_TX0", "NC_RX0", "RMII_TXEN", "GND", "RMII_TXD0",
          "RMII_MDIO", "NC_IO5", "ETH_CLK", "SR_SRCLK", "SR_OE", "NC_IO0", "NC_IO2", "I2C_SDA",
          "NC_D1", "NC_D0", "NC_CLK"]
ESP_J2_NAMES = ["3V3", "EN", "VP/36", "VN/39", "34", "35", "32", "33", "25", "26", "27", "14", "12",
                "GND", "13", "D2/9", "D3/10", "CMD/11", "5V"]
ESP_J3_NAMES = ["GND", "23", "22", "TX0/1", "RX0/3", "21", "GND", "19", "18", "5", "17", "16", "4",
                "0", "2", "15", "D1/8", "D0/7", "CLK/6"]
add("J30", ("Connector_Generic", "Conn_01x19"), "sock19", "DevKitC J2 (3V3..5V)",
    {str(i + 1): ESP_J2[i] for i in range(19)}, (ESP_PIN1_X, ESP_Y - ESP_ROW_DY, 270, "F"),
    ("root", 225, 60, 0), lcsc="C319202", desc="Female header 1x19 2.54 mm (socket for the ESP32 DevKitC V4)")
add("J31", ("Connector_Generic", "Conn_01x19"), "sock19", "DevKitC J3 (GND..CLK)",
    {str(i + 1): ESP_J3[i] for i in range(19)}, (ESP_PIN1_X, ESP_Y + ESP_ROW_DY, 270, "F"),
    ("root", 285, 60, 0), lcsc="C319202", desc="Female header 1x19 2.54 mm (socket for the ESP32 DevKitC V4)")

# --- shift registers and drivers ----------------------------------------------------------
SR_IN = ["SR_SER", "SR_Q1", "SR_Q2"]
SR_OUT = ["SR_Q1", "SR_Q2", "NC_Q3"]
DRV_POS = [((92.0, 52.0), (106.5, 52.0)), ((148.0, 52.0), (162.5, 52.0)), ((92.0, 69.0), (106.5, 69.0))]
for b in range(3):
    base = b * 8
    q = {"15": f"DRV{base + 1}", "1": f"DRV{base + 2}", "2": f"DRV{base + 3}", "3": f"DRV{base + 4}",
         "4": f"DRV{base + 5}", "5": f"DRV{base + 6}", "6": f"DRV{base + 7}", "7": f"DRV{base + 8}",
         "8": "GND", "16": "+3V3", "10": "+3V3", "13": "SR_OE", "12": "SR_RCLK", "11": "SR_SRCLK",
         "14": SR_IN[b], "9": SR_OUT[b]}
    (sx, sy), (ux, uy) = DRV_POS[b]
    add(f"U{b + 1}", ("74xx", "74HC595"), "soic16", "74HC595", q, (sx, sy, 180, "F"),
        ("root", 60 + b * 90, 220, 0), lcsc="C5947", desc="Shift register 8 bits SOIC-16 (Nexperia 74HC595D)")
    add(f"C{b + 1}", ("Device", "C"), "c0603", "100nF", {"1": "+3V3", "2": "GND"},
        ((sx - 1.2, sy + 6.8, 270, "F") if b == 1 else (sx - 5.6, sy + 3.5, 90, "F")),
        ("root", 45 + b * 90, 240, 0), lcsc="C14663", desc="Capacitor 0603 100 nF")
    # ULN input 1 = QA (relay 1 of the bank); inputs 8..2 = QB..QH, so that the seven lines
    # between the two chips run straight across (the 74HC595 pins 1-7 face them in that
    # order). Output k is on pin 19-k, so pin 18 drives relay 1 and pins 11..17 relays 2..8.
    uln = {"1": f"DRV{base + 1}", "18": f"RLY{base + 1}"}
    uln.update({str(k): f"DRV{base + 10 - k}" for k in range(2, 9)})
    uln.update({str(p): f"RLY{base + p - 9}" for p in range(11, 18)})
    uln.update({"9": "GND", "10": "+12V"})
    add(f"U{b + 4}", ("Transistor_Array", "ULN2803A"), "soic18w", "ULN2803A", uln, (ux, uy, 0, "F"),
        ("root", 95 + b * 90, 220, 0), lcsc="C22437476", desc="Darlington array 8 x 500 mA SOP-18 300 mil, ULN2803A pinout (Shanghai Belling ULN2803)")

# --- Ethernet: LAN8720A + RJ45 --------------------------------------------------------------
PHY = (66.0, 60.0)
add("U9", ("Interface_Ethernet", "LAN8720A"), "qfn24", "LAN8720A",
    {"1": "+3V3", "2": "LED_SPD", "3": "LED_LNK", "4": "NC_XTAL2", "5": "ETH_CLKIN", "6": "VDDCR",
     "7": "RMII_RXD1", "8": "RMII_RXD0", "9": "+3V3", "10": "PHYAD0", "11": "RMII_CRS_DV", "12": "RMII_MDIO",
     "13": "RMII_MDC", "14": "NC_INT", "15": "ETH_RST", "16": "RMII_TXEN", "17": "RMII_TXD0", "18": "RMII_TXD1",
     "19": "+3V3", "20": "TXN", "21": "TXP", "22": "RXN", "23": "RXP", "24": "RBIAS", "25": "GND"},
    PHY + (0, "F"), ("root", 400, 75, 0), lcsc="C45223", desc="Ethernet PHY RMII QFN-24 (Microchip LAN8720A-CP-TR)")
add("R30", ("Device", "R"), "r0603", "33", {"1": "ETH_CLK", "2": "ETH_CLKIN"},
    (60.4, 60.9, 0, "F"), ("root", 365, 57.5, 90), lcsc="C23140", desc="Resistor 0603 33 ohm (50 MHz clock from GPIO17)")
add("R31", ("Device", "R"), "r0603", "12.1k 1%", {"1": "RBIAS", "2": "GND"},
    (63.5, 54.9, 180, "F"), ("root", 435, 72.46, 90), lcsc="C123417", desc="Resistor 0603 12.1 kohm 1 % (RBIAS, Yageo RC0603FR-0712K1L)")
add("R32", ("Device", "R"), "r0603", "10k", {"1": "PHYAD0", "2": "GND"},
    (66.6, 65.9, 270, "F"), ("root", 365, 87.5, 90), lcsc="C25804", desc="Resistor 0603 10 kohm (PHY address 0)")
add("R33", ("Device", "R"), "r0603", "10k", {"1": "ETH_RST", "2": "GND"},
    (71.5, 61.3, 0, "F"), ("root", 365, 100, 90), lcsc="C25804", desc="Resistor 0603 10 kohm (PHY held in reset until GPIO33 goes high)")
add("R34", ("Device", "R"), "r0603", "1.5k", {"1": "RMII_MDIO", "2": "+3V3"},
    (70.0, 69.0, 270, "F"), ("root", 365, 112.5, 90), lcsc="C22843", desc="Resistor 0603 1.5 kohm (MDIO pull-up)")
for i, (net, pos) in enumerate([("RMII_RXD0", (63.5, 69.0)), ("RMII_RXD1", (62.0, 69.0)), ("RMII_CRS_DV", (68.0, 69.0))]):
    add(f"R{35 + i}", ("Device", "R"), "r0603", "10k", {"1": net, "2": "+3V3"},
        pos + (270, "F"), ("root", 345 + i * 10, 125, 90), lcsc="C25804", desc="Resistor 0603 10 kohm (MODE strap: all capable, auto-negotiation)")
for i, (net, pos) in enumerate([("TXP", (84.0, 61.0)), ("TXN", (84.0, 62.5)), ("RXP", (84.0, 58.0)), ("RXN", (84.0, 59.5))]):
    add(f"R{38 + i}", ("Device", "R"), "r0603", "49.9 1%", {"1": net, "2": "+3V3"},
        pos + (0, "F"), ("root", 460 + i * 10, 60, 90), lcsc="C23185", desc="Resistor 0603 49.9 ohm 1 % (line termination to 3.3 V)")
add("R42", ("Device", "R"), "r0603", "330", {"1": "LED_LNK_K", "2": "GND"},
    (58.0, 59.4, 180, "F"), ("root", 445, 100, 0), lcsc="C23138", desc="Resistor 0603 330 ohm")
add("R43", ("Device", "R"), "r0603", "330", {"1": "+3V3", "2": "LED_SPD_A"},
    (58.0, 57.7, 0, "F"), ("root", 445, 115, 0), lcsc="C23138", desc="Resistor 0603 330 ohm")
add("D30", ("Device", "LED"), "led0603", "yellow", {"1": "LED_LNK_K", "2": "LED_LNK"},
    (61.0, 59.4, 0, "F"), ("root", 460, 100, 0), lcsc="C965802", desc="LED 0603 yellow (XINGLIGHT XL-1608UYC-06): link / activity")
add("D31", ("Device", "LED"), "led0603", "green", {"1": "LED_SPD", "2": "LED_SPD_A"},
    (61.0, 57.9, 180, "F"), ("root", 460, 115, 0), lcsc="C965805", desc="LED 0603 green (yellow-green, Vf 2.2 V, XINGLIGHT XL-1608SYGC-06): 100 Mbit")
for i, (net, pos, rot) in enumerate([("+3V3", (61.5, 56.4), 180), ("+3V3", (65.0, 69.0), 270), ("+3V3", (72.0, 57.0), 0),
                                     ("VDDCR", (63.0, 64.3), 270)]):
    add(f"C{30 + i}", ("Device", "C"), "c0603", "100nF", {"1": net, "2": "GND"},
        pos + (rot, "F"), ("root", 400 + i * 10, 125, 0), lcsc="C14663", desc="Capacitor 0603 100 nF")
add("C34", ("Device", "C"), "c0603", "1uF", {"1": "VDDCR", "2": "GND"},
    (61.4, 64.3, 270, "F"), ("root", 440, 125, 0), lcsc="C15849", desc="Capacitor 0603 1 uF (PHY 1.2 V regulator)")
add("C35", ("Device", "C"), "c0805", "10uF", {"1": "+3V3", "2": "GND"},
    (56.0, 64.5, 90, "F"), ("root", 450, 125, 0), lcsc="C15850", desc="Capacitor 0805 10 uF 25 V")
add("J32", ("Connector", "RJ45_LED_Shielded"), "rj45", "HR911105A",
    {"1": "TXN", "2": "TXP", "3": "RXN", "4": "+3V3", "5": "+3V3", "6": "RXP", "7": "+3V3", "8": "+3V3",
     "9": "NC_LED1", "10": "NC_LED2", "11": "NC_LED3", "12": "NC_LED4", "SH": "GND"},
    RJ45_POS + (90, "F"), ("root", 520, 75, 0), lcsc="C12074",
    desc="RJ45 with magnetics and LEDs (HanRun HR911105A); centre taps to 3.3 V, its LED pins unused; "
         "PHY TX on pins 1-2 and RX on 3-6 would need crossing tracks, so the pairs are swapped (Auto-MDIX) "
         "and the polarity follows the layout (harmless for 10/100BASE-T)",
    jlc_note="opening at the board edge")
add("C36", ("Device", "C"), "c0603", "100nF", {"1": "+3V3", "2": "GND"},
    (186.0, 55.5, 90, "F"), ("root", 540, 110, 0), lcsc="C14663", desc="Capacitor 0603 100 nF (centre taps)")
add("C37", ("Device", "C"), "c0805", "10uF", {"1": "+3V3", "2": "GND"},
    (186.0, 60.5, 90, "F"), ("root", 550, 110, 0), lcsc="C15850", desc="Capacitor 0805 10 uF 25 V (centre taps)")

# --- small headers, pull-ups -------------------------------------------------------------
add("R44", ("Device", "R"), "r0603", "4.7k", {"1": "SR_OE", "2": "+3V3"},
    (50.8, 50.8, 90, "F"), ("root", 290, 130, 90), lcsc="C23162", desc="Resistor 0603 4.7 kohm (outputs off until the program starts)")
add("R45", ("Device", "R"), "r0603", "4.7k", {"1": "I2C_SDA", "2": "+3V3"},
    (60.78, 50.8, 270, "F"), ("root", 320, 160, 90), lcsc="C23162", desc="Resistor 0603 4.7 kohm (I2C pull-up)")
add("R46", ("Device", "R"), "r0603", "4.7k", {"1": "I2C_SCL", "2": "+3V3"},
    (63.32, 50.8, 270, "F"), ("root", 330, 160, 90), lcsc="C23162", desc="Resistor 0603 4.7 kohm (I2C pull-up)")
add("J33", ("Connector_Generic", "Conn_01x04"), "hdr4", "I2C",
    {"1": "+3V3", "2": "GND", "3": "I2C_SDA", "4": "I2C_SCL"}, (55.7, 47.5, 90, "F"), ("root", 355, 165, 0),
    lcsc="C124378", desc="Pin header 1x4 2.54 mm: 3V3, GND, SDA (GPIO15), SCL (GPIO14) for future sensors")
add("J34", ("Connector_Generic", "Conn_01x04"), "hdr4", "Analog",
    {"1": "+3V3", "2": "GND", "3": "A34", "4": "A35"}, (67.0, 47.5, 90, "F"), ("root", 355, 185, 0),
    lcsc="C124378", desc="Pin header 1x4 2.54 mm: 3V3, GND, GPIO34, GPIO35 (analog inputs, 0-3.3 V)")

# --- mounting holes ---------------------------------------------------------------------
for i, pos in enumerate([(7.5, 24.0), (BOARD_W - 7.5, 24.0), (7.5, BOARD_H - 24.0), (186.0, 113.0),
                         (81.0, 49.5), (125.0, 52.0)]):
    add(f"H{i + 1}", ("Mechanical", "MountingHole"), "hole", "M3", {}, pos + (0, "F"),
        ("root", 500 + (i % 3) * 15, 300 + (i // 3) * 15, 0), desc="Mounting hole 3.2 mm (M3)")

# ----------------------------------------------------------------------------- nets
NETS = {}
for p in PARTS.values():
    for pin, net in p["pins"].items():
        NETS.setdefault(net, []).append((p["ref"], pin))

POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V", "+12V": "power:+12V"}

SHEETS = {
    "root": ("A2", TITLE),
    "relays_1_8": ("A3", "Relays 1-8 (sections 1-8, driver U4)"),
    "relays_9_16": ("A3", "Relays 9-16 (sections 9-16, driver U5)"),
    "relays_17_24": ("A3", "Relays 17-24 (sections 17-24, driver U6)"),
}

if __name__ == "__main__":
    print(len(PARTS), "parts,", len(NETS), "nets")
    for n, pins in sorted(NETS.items()):
        if len(pins) == 1 and not n.startswith("NC_"):
            print("single-pin net:", n, pins)
