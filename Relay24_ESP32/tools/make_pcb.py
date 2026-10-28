#!/usr/bin/env python3
"""
Builds the KiCad 7 PCB of Relay24_ESP32 from design.py with the pcbnew Python module.

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

Usage: python3 make_pcb.py <output folder>   (run with the python3 that has pcbnew)

Steps: place every footprint, assign the nets, draw the outline, route the regular parts
by formula (the 24 relay cells, the coil buses from the ULN2803 to the relays, the 12 V
bus), pour the copper (ground on both sides, the two actuator rails), route the rest of
the logic with a small grid router (router.py), fill the zones, run the DRC.
"""
import os
import sys

import pcbnew
from pcbnew import FromMM, VECTOR2I, VECTOR2I_MM, F_Cu, B_Cu

import design as D
import router

FPDIR = "/usr/share/kicad/footprints/"
board = None
NETS = {}
SIG_W = 0.25       # signal track width
PWR_W = 0.6        # 3.3 V / 5 V tracks
VIA_D, VIA_DRILL = 0.6, 0.3


def mm(x, y):
    return VECTOR2I_MM(float(x), float(y))


def net(name):
    return NETS[name]


def track(x1, y1, x2, y2, w, layer, netname):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(mm(x1, y1))
    t.SetEnd(mm(x2, y2))
    t.SetWidth(FromMM(w))
    t.SetLayer(layer)
    t.SetNet(net(netname))
    board.Add(t)
    return t


def polyline(pts, w, layer, netname):
    for a, b in zip(pts, pts[1:]):
        if a != b:
            track(a[0], a[1], b[0], b[1], w, layer, netname)


def via(x, y, netname, d=VIA_D, drill=VIA_DRILL):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(mm(x, y))
    v.SetDrill(FromMM(drill))
    v.SetWidth(FromMM(d))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(F_Cu, B_Cu)
    v.SetNet(net(netname))
    board.Add(v)
    return v


def zone(netname, layer, pts, priority=0, clearance=0.3, min_w=0.25, name=""):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNetCode(net(netname).GetNetCode())
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(FromMM(clearance))
    z.SetMinThickness(FromMM(min_w))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(FromMM(0.4))
    z.SetThermalReliefSpokeWidth(FromMM(0.8))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName(name)
    z.Outline().NewOutline()
    for x, y in pts:
        z.Outline().Append(FromMM(x), FromMM(y))
    board.Add(z)
    return z


def rect(x1, y1, x2, y2):
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def text(s, x, y, layer=pcbnew.F_SilkS, size=1.0, rot=0, thick=0.15, h="c"):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(mm(x, y))
    t.SetLayer(layer)
    t.SetTextSize(VECTOR2I(FromMM(size), FromMM(size)))
    t.SetTextThickness(FromMM(thick))
    t.SetTextAngleDegrees(rot)
    if h == "l":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    elif h == "r":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    board.Add(t)
    return t


def edge(x1, y1, x2, y2):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(mm(x1, y1))
    s.SetEnd(mm(x2, y2))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(FromMM(0.1))
    board.Add(s)


def pad_pos(ref, num):
    fp = board.FindFootprintByReference(ref)
    p = fp.FindPadByNumber(str(num))
    pos = p.GetPosition()
    return pos.x / 1e6, pos.y / 1e6


# ------------------------------------------------------------------------------- build
def place_parts():
    for name in sorted(D.NETS):
        if name.startswith("NC_"):
            continue
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        NETS[name] = ni
    for p in D.PARTS.values():
        lib, fpname = p["fp"]
        fp = pcbnew.FootprintLoad(FPDIR + lib + ".pretty", fpname)
        if fp is None:
            raise SystemExit(f"footprint {lib}:{fpname} not found")
        fp.SetReference(p["ref"])
        fp.SetValue(p["value"])
        x, y, rot, side = p["pcb"]
        fp.SetPosition(mm(x, y))
        fp.SetOrientationDegrees(rot)
        if side == "B":
            fp.Flip(mm(x, y), False)
        for key in ("LCSC", "Description"):
            fp.SetProperty(key, p["lcsc"] if key == "LCSC" else p["desc"])
        ref = fp.Reference()
        ref.SetTextSize(VECTOR2I(FromMM(0.8), FromMM(0.8)))
        ref.SetTextThickness(FromMM(0.12))
        if p["fp"][0] in ("Resistor_SMD", "Capacitor_SMD", "LED_SMD", "Connector_PinHeader_2.54mm",
                          "Connector_PinSocket_2.54mm"):
            ref.SetVisible(False)              # too dense for silkscreen; the CPL has them
        fp.Value().SetVisible(False)
        board.Add(fp)
        for pad in fp.Pads():
            n = p["pins"].get(pad.GetNumber())
            if n and not n.startswith("NC_"):
                pad.SetNet(net(n))
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetDrillSize().x < FromMM(0.3):
                pad.SetDrillSize(VECTOR2I(FromMM(0.3), FromMM(0.3)))      # thermal vias of the QFN
                pad.SetSize(VECTOR2I(FromMM(0.6), FromMM(0.6)))


def outline():
    W, H = D.BOARD_W, D.BOARD_H
    edge(0, 0, W, 0)
    edge(W, 0, W, H)
    edge(W, H, 0, H)
    edge(0, H, 0, 0)


def relay_cells():
    """The fixed wiring of each relay + terminal + LED cell (see design.py for the geometry)."""
    for n in range(1, 25):
        K, J = f"K{n}", f"J{n}"
        top = n <= 12
        s = -1 if top else 1                     # +y direction towards the board edge
        a1, a2 = pad_pos(K, "A1"), pad_pos(K, "A2")
        com1, nc1 = pad_pos(K, "11"), pad_pos(K, "12")
        com2, nc2 = pad_pos(K, "21"), pad_pos(K, "22")
        t1, t2 = pad_pos(J, "1"), pad_pos(J, "2")
        yedge = 10.0 if top else D.BOARD_H - 10.0            # inside the actuator rail bands
        d = 1 if (com2[0] > com1[0]) else -1                 # +x direction from pole 1 to pole 2
        # commons to the terminal, bottom layer, between the two NO pads
        xa = com1[0] + d * 2.2
        polyline([com1, (xa, com1[1] + s * 2.2), (xa, t1[1] - s * 2.5), t1], 1.2, B_Cu, f"SEC{n}_R")
        xb = com2[0] - d * 2.2
        polyline([com2, (xb, com2[1] + s * 2.2), (xb, t2[1] - s * 2.5), t2], 1.2, B_Cu, f"SEC{n}_B")
        # NC2 (pole 2) to the actuator rail: round the NC row on the bottom layer, up the
        # gap between the relays, via into the rail band (the NO1 pad sits in the band)
        yr = nc1[1] - s * 2.5
        xg = com1[0] - d * 3.25
        rail = D.relay_rail(n)
        polyline([nc2, (nc2[0], yr), (xg, yr), (xg, yedge)], 1.2, B_Cu, rail)
        via(xg, yedge, rail, 1.0, 0.5)
        # NC1 and NO2 are ground: the ground pours reach those through-hole pads
        # coil + (A1) to the 12 V bus (top layer)
        ybus = bus_y(top)
        polyline([a1, (a1[0], ybus)], 0.8, F_Cu, "+12V")
        # section LED: 2.2 k from the 12 V bus, LED cathode to the coil drive (A2)
        r1, r2 = pad_pos(f"R{n}", "1"), pad_pos(f"R{n}", "2")
        l1, l2 = pad_pos(f"D{n}", "1"), pad_pos(f"D{n}", "2")
        polyline([r2, (r2[0], ybus)], 0.4, F_Cu, "+12V")
        polyline([r1, l2], 0.3, F_Cu, f"LEDA{n}")
        polyline([l1, (l1[0], l1[1] + s * 1.3), (a2[0], l1[1] + s * 2.65), a2], 0.4, F_Cu, f"RLY{n}")


def bus_y(top):
    """y of the 12 V coil bus (top layer, 1.2 mm) of a relay row."""
    return D.COIL_Y_TOP + 6.25 if top else D.COIL_Y_BOT - 6.25


def coil_buses():
    """ULN2803 outputs to the relay coils (A2) on the bottom layer: a via next to each
    output, a vertical to the line's level, a horizontal to the coil. Levels 0.55 mm
    apart, via columns 0.7 mm apart (0.9 mm where a top-layer vertical passes a via).
    The levels nest so that no two lines cross."""
    def line(uln, n, xv, level, yvia=None):
        """yvia None: via next to the pad, bottom layer straight up/down to the level.
        yvia given: top layer down to the via row at yvia (keeps the bottom layer free
        for the signals that cross the board), then bottom layer to the level."""
        pin = [p for p, net in D.PARTS[uln]["pins"].items() if net == f"RLY{n}"][0]
        pad = pad_pos(uln, pin)
        a2 = pad_pos(f"K{n}", "A2")
        if yvia is None:
            polyline([pad, (xv, pad[1])], 0.3, F_Cu, f"RLY{n}")
            via(xv, pad[1], f"RLY{n}")
            polyline([(xv, pad[1]), (xv, level), (a2[0], level), a2], 0.3, B_Cu, f"RLY{n}")
        else:
            polyline([pad, (xv, pad[1]), (xv, yvia)], 0.3, F_Cu, f"RLY{n}")
            via(xv, yvia, f"RLY{n}")
            polyline([(xv, yvia), (xv, level), (a2[0], level), a2], 0.3, B_Cu, f"RLY{n}")
    top_lvl = D.COIL_Y_TOP + 1.5           # nearest line to the coil pads (pads end 1 mm below the centre)
    bot_lvl = D.COIL_Y_BOT - 1.5
    yvia = bus_y(False) - 1.25             # via row just above the bottom 12 V bus
    for n in range(1, 9):                   # U4: relays 1..8, all to the left except relay 8
        line("U4", n, 113.0 + 0.7 * (n - 1), top_lvl + 0.55 * (8 - n))
    for n in range(9, 13):                  # U5: relays 9..12 (top row, to the left)
        line("U5", n, 172.0 + 0.7 * (n - 9), top_lvl + 0.55 * (12 - n))
    # U5: relays 13..16 (bottom row). Top layer down to a via row above the bottom 12 V bus
    # (the bottom layer there carries the Ethernet lines), the lower the pin the nearer the
    # column; relay 13 sits right below, the others turn back up to levels above the via
    # row and run west (highest level = relay furthest west).
    for n in range(13, 17):
        lvl = bot_lvl - 1.5 if n == 13 else yvia - 0.7 - 0.55 * (n - 14)
        line("U5", n, 175.3 + 0.9 * (n - 13), lvl, yvia)
    for n in range(17, 25):                 # U6: relays 17..24 (bottom row, to the left)
        line("U6", n, 119.3 - 0.7 * (n - 17), bot_lvl - 0.55 * (n - 17))


def power_buses():
    W = D.BOARD_W
    yt, yb = bus_y(True), bus_y(False)
    # 12 V for the coils: along both relay rows, joined on the right, fed from the buck area
    polyline([(pad_pos("R1", "2")[0], yt), (181.5, yt), (181.5, 93.0)], 1.2, F_Cu, "+12V")
    polyline([(pad_pos("K24", "A1")[0], yb), (181.5, yb)], 1.2, F_Cu, "+12V")
    via(181.5, 93.0, "+12V", 1.0, 0.5)
    polyline([(181.5, 93.0), (199.5, 93.0)], 1.2, B_Cu, "+12V")
    via(199.5, 93.0, "+12V", 1.0, 0.5)
    d25k = pad_pos("D25", "1")
    c25p = pad_pos("C25", "1")
    d26p = pad_pos("D26", "1")
    u7in = pad_pos("U7", "1")
    polyline([(199.5, 93.0), (d25k[0], 93.0), d25k], 1.2, F_Cu, "+12V")
    polyline([(c25p[0], 93.0), c25p, (c25p[0], c25p[1] + 1.5), (d26p[0], c25p[1] + 4.0), d26p], 1.2, F_Cu, "+12V")
    polyline([(c25p[0], 95.5), (u7in[0], 95.5)], 1.2, F_Cu, "+12V")
    # ULN2803 commons (pin 10) to the 12 V bus. U4's riser stands east of the crossing
    # lanes (gap_lanes), U5's goes straight up, U6's straight down.
    p = pad_pos("U4", "10")
    polyline([p, (120.5, p[1]), (120.5, 55.6)], 0.6, F_Cu, "+12V")
    via(120.5, 55.6, "+12V")
    polyline([(120.5, 55.6), (120.5, yt)], 0.6, B_Cu, "+12V")
    via(120.5, yt, "+12V")
    p = pad_pos("U5", "10")
    polyline([p, (p[0], 58.5)], 0.6, F_Cu, "+12V")
    via(p[0], 58.5, "+12V")
    polyline([(p[0], 58.5), (p[0], yt)], 0.6, B_Cu, "+12V")
    via(p[0], yt, "+12V")
    p = pad_pos("U6", "10")
    polyline([p, (p[0], yb)], 0.6, F_Cu, "+12V")
    # 5 V for the DevKitC (VIN, the far left pin of the top row): one top-layer line just
    # below the top 12 V bus, down east of the ULN2803/lane area to the buck diode
    j19, d28, c27 = pad_pos("J30", "19"), pad_pos("D28", "1"), pad_pos("C27", "1")
    polyline([j19, (j19[0], 45.9), (122.5, 45.9), (122.5, 62.5), (d28[0], 62.5), d28], PWR_W, F_Cu, "+5V")
    polyline([c27, (c27[0], 62.5)], PWR_W, F_Cu, "+5V")
    # the ground pins of the I2C / analog headers sit between the +5V line (top layer)
    # and the crowded band under the module: a short bottom-layer stub straight up
    # into the ground pour, which is continuous there
    for ref in ("J33", "J34"):
        p = pad_pos(ref, "2")
        polyline([p, (p[0], 44.9)], SIG_W, B_Cu, "GND")
    # C31 (PHY 3.3 V decoupling) sits in the strap-resistor row, where tracks on both
    # layers cut the pour to pieces: its ground pad gets a straight top-layer stub down
    # to a via in the clear band just above the bottom relay row
    p = pad_pos("C31", "2")
    polyline([p, (p[0], 74.1)], SIG_W, F_Cu, "GND")
    via(p[0], 74.1, "GND")
    # 12 V in: terminal to the reverse-protection diode
    j1 = pad_pos("J25", "1")
    d25a = pad_pos("D25", "2")
    polyline([j1, (j1[0], j1[1] + 3.5), (d25a[0], j1[1] + 5.5), d25a], 1.5, F_Cu, "+12V_IN")
    # buck: switch node and 5 V
    u7out, u7fb = pad_pos("U7", "2"), pad_pos("U7", "4")
    l1a, l1b = pad_pos("L1", "1"), pad_pos("L1", "2")
    d27k = pad_pos("D27", "1")
    c26p = pad_pos("C26", "1")
    polyline([(u7out[0], u7out[1] + 1.5), (u7out[0], l1a[1] + 3.5), (l1a[0], l1a[1] + 2.0), l1a], 1.5, F_Cu, "SW")
    polyline([(u7out[0], d27k[1]), d27k], 1.5, F_Cu, "SW")
    polyline([(u7fb[0], u7fb[1] + 1.5), (u7fb[0], c26p[1] + 1.5), (c26p[0], c26p[1])], 1.5, F_Cu, "+5VB")
    polyline([c26p, (c26p[0], c26p[1] - 4.5), (l1b[0] + 1.0, l1b[1] + 3.0), l1b], 1.5, F_Cu, "+5VB")
    # ground vias under the buck regulator tab
    for (x, y) in ((189.5, 100.0), (194.5, 100.0), (189.5, 104.0), (194.5, 104.0)):
        via(x, y, "GND", 0.8, 0.4)
    # 74HC595 ground pins (top right corner): a via 2 mm to the east
    for ref in ("U1", "U2", "U3"):
        p = pad_pos(ref, "8")
        polyline([p, (p[0] + 2.0, p[1])], SIG_W, F_Cu, "GND")
        via(p[0] + 2.0, p[1], "GND")
    # Ethernet pairs. Top layer: the PHY pins fan out above the chip (the rightmost pin
    # turns first, so nothing crosses), down to a column of vias at x 82 with the 49.9 ohm
    # terminations next to them. Bottom layer: four lines at 0.45 mm pitch across the board
    # (the inner lines change level first), fan-out into the RJ45 pads.
    xv = 82.0
    for net, pin, yf, xd, yv, xj, lvl, xt, yt2, rjpin in (
            ("RXP", "23", 54.0, 82.0, 58.0, 84.5, 59.3, 188.5, 56.66, "6"),
            ("RXN", "22", 54.5, 81.3, 59.5, 83.5, 59.75, 189.0, 60.46, "3"),
            ("TXP", "21", 55.0, 80.6, 61.0, 84.0, 60.2, 188.1, 61.74, "2"),
            ("TXN", "20", 55.5, 79.9, 62.5, 85.0, 60.65, 187.3, 63.0, "1")):
        p = pad_pos("U9", pin)
        polyline([p, (p[0], yf), (xd, yf), (xd, yv), (xv, yv)], SIG_W, F_Cu, net)
        via(xv, yv, net)
        rt = [r for r in ("R38", "R39", "R40", "R41") if D.PARTS[r]["pins"]["1"] == net][0]
        polyline([(xv, yv), pad_pos(rt, "1")], SIG_W, F_Cu, net)
        rj = pad_pos("J32", rjpin)
        polyline([(xv, yv), (xj, yv), (xj, lvl), (xt, lvl), (xt, yt2), rj], SIG_W, B_Cu, net)
    # actuator rail terminals into the rail bands
    for ref, y2 in (("J26", 9.4), ("J27", 9.4), ("J28", D.BOARD_H - 9.4), ("J29", D.BOARD_H - 9.4)):
        p = pad_pos(ref, "1")
        polyline([p, (p[0], y2)], 2.0, F_Cu, D.PARTS[ref]["pins"]["1"])
    phy_fanout()
    sr_fanout()
    gap_lanes()


def sr_fanout():
    """74HC595 left columns. The first output (QA, pin 15) runs under the chip on the bottom
    layer to input 1 of its ULN2803A, and SRCLR (pin 10) loops on the bottom layer to VCC
    (pin 16) and the decoupling capacitor, so that the exits of pins 9 and 11-14 stay free
    for the router. U1 and U3 have the capacitor on the left and the loop west of it; U2
    (whose left side receives the lanes from the ESP32) has it below the chip and the loop
    runs under the chip. The capacitor ground vias are placed here too, where the pour is
    continuous."""
    for sr, uln, cap, drv, side in (("U1", "U4", "C1", "DRV1", "L"), ("U2", "U5", "C2", "DRV9", "R"),
                                   ("U3", "U6", "C3", "DRV17", "L")):
        p10, p15, p16 = pad_pos(sr, "10"), pad_pos(sr, "15"), pad_pos(sr, "16")
        u1, c1, c2 = pad_pos(uln, "1"), pad_pos(cap, "1"), pad_pos(cap, "2")
        xa, xc = p15[0] - 1.125, u1[0] - 1.85
        polyline([p15, (xa, p15[1])], SIG_W, F_Cu, drv)
        via(xa, p15[1], drv)
        polyline([(xa, p15[1]), (xc, p15[1]), (xc, u1[1])], SIG_W, B_Cu, drv)
        via(xc, u1[1], drv)
        polyline([(xc, u1[1]), u1], SIG_W, F_Cu, drv)
        polyline([p10, (xa, p10[1])], SIG_W, F_Cu, "+3V3")
        via(xa, p10[1], "+3V3")
        if side == "L":
            xb = c1[0] - 2.6
            polyline([(xa, p10[1]), (xb, p10[1]), (xb, c1[1])], SIG_W, B_Cu, "+3V3")
            via(xb, c1[1], "+3V3")
            polyline([(xb, c1[1]), c1], SIG_W, F_Cu, "+3V3")
            polyline([p16, (c1[0], p16[1])], SIG_W, F_Cu, "+3V3")
            polyline([c2, (c2[0] - 3.6, c2[1])], SIG_W, F_Cu, "GND")
            via(c2[0] - 3.6, c2[1], "GND")
        else:
            xr = (p15[0] + pad_pos(sr, "1")[0]) / 2        # under the middle of the chip
            polyline([(xa, p10[1]), (xr, p10[1]), (xr, 54.2)], SIG_W, B_Cu, "+3V3")
            via(xr, 54.2, "+3V3")
            polyline([(xr, 54.2), (xr, 57.2), (c1[0], 57.2), c1], SIG_W, F_Cu, "+3V3")
            polyline([p16, (p16[0], c1[1]), c1], SIG_W, F_Cu, "+3V3")
            polyline([c2, (c2[0], c2[1] + 1.82)], SIG_W, F_Cu, "GND")   # below the Ethernet lines
            via(c2[0], c2[1] + 1.82, "GND")


def gap_lanes():
    """The only way between the left half (ESP32, U1, U3) and the right half (U2, power) is
    the 7 mm gap between the coil lines of banks A and C, at x 113-119; the four Ethernet
    lines already cross it at y 59.3-60.65. One bottom-layer lane per other crossing net:
    the shift-register signals above the Ethernet lines, in the order that keeps the fan-out
    into U2 planar (pin 9 at the top ... pin 14 at the bottom), 3.3 V below them."""
    for net, y, w in (("SR_Q2", 56.5, SIG_W), ("SR_SRCLK", 57.0, SIG_W), ("SR_RCLK", 57.5, SIG_W),
                      ("SR_OE", 58.0, SIG_W), ("SR_Q1", 58.5, SIG_W), ("+3V3", 61.6, PWR_W)):
        track(104.0, y, 122.0, y, w, B_Cu, net)


def phy_fanout():
    """LAN8720A pin escapes. The pins are 0.5 mm apart, so each one gets a fixed track to
    its resistor, capacitor, LED or to a via (the grid router takes over from there).
    West column: 1 +3V3, 2 LED_SPD, 3 LED_LNK, 5 CLKIN, 6 VDDCR. South row: 7 RXD1, 8 RXD0,
    9 +3V3, 10 PHYAD0, 11 CRS_DV, 12 MDIO (pull-ups in a row below). East column: 13 MDC,
    15 RST, 16 TXEN, 17 TXD0, 18 TXD1. North row: 19 +3V3, 24 RBIAS (the pairs are above)."""
    P = lambda n: pad_pos("U9", str(n))
    w = SIG_W
    def run(pts, net):
        polyline(pts, w, F_Cu, net)
    c30, d31, d30, r30 = pad_pos("C30", "1"), pad_pos("D31", "1"), pad_pos("D30", "2"), pad_pos("R30", "2")
    c33, c34 = pad_pos("C33", "1"), pad_pos("C34", "1")
    run([P(1), (63.35, P(1)[1]), (63.35, c30[1]), c30], "+3V3")
    run([P(2), (62.85, P(2)[1]), (62.85, d31[1]), d31], "LED_SPD")
    run([P(3), (d30[0], P(3)[1]), d30], "LED_LNK")
    run([P(5), (r30[0], P(5)[1]), r30], "ETH_CLKIN")
    run([P(6), (63.3, P(6)[1]), (63.3, c33[1]), c33, c34], "VDDCR")
    r36, r35, c31, r32, r37, r34 = (pad_pos(r, "1") for r in ("R36", "R35", "C31", "R32", "R37", "R34"))
    run([P(7), (P(7)[0], 66.3), (r36[0], 66.3), r36], "RMII_RXD1")
    run([P(8), (P(8)[0], 66.8), (r35[0], 66.8), r35], "RMII_RXD0")
    run([P(9), (P(9)[0], 67.3), (c31[0], 67.3), c31], "+3V3")
    run([P(10), (P(10)[0], r32[1]), r32], "PHYAD0")
    run([P(11), (P(11)[0], 63.5), (r37[0], 63.5), r37], "RMII_CRS_DV")
    run([P(12), (P(12)[0], 62.95), (r34[0], 62.95), r34], "RMII_MDIO")
    r33 = pad_pos("R33", "1")
    run([P(13), (68.9, P(13)[1]), (68.9, 62.4), (72.5, 62.4)], "RMII_MDC")
    via(72.5, 62.4, "RMII_MDC")
    run([P(15), (69.4, P(15)[1]), (69.4, r33[1]), r33], "ETH_RST")
    run([P(16), (70.0, P(16)[1])], "RMII_TXEN")
    via(70.0, P(16)[1], "RMII_TXEN")
    run([P(17), (69.4, P(17)[1]), (69.4, 58.4), (71.1, 58.4)], "RMII_TXD0")
    via(71.1, 58.4, "RMII_TXD0")
    run([P(18), (68.9, P(18)[1]), (68.9, 57.5), (69.9, 57.5)], "RMII_TXD1")
    via(69.9, 57.5, "RMII_TXD1")
    c32, r31 = pad_pos("C32", "1"), pad_pos("R31", "1")
    run([P(19), (P(19)[0], 56.5), (c32[0], 56.5), c32], "+3V3")
    run([P(24), (P(24)[0], r31[1]), r31], "RBIAS")


def zones():
    W, H = D.BOARD_W, D.BOARD_H
    zone("GND", F_Cu, rect(0.5, 0.5, W - 0.5, H - 0.5), 0, name="GND top")
    zone("GND", B_Cu, rect(0.5, 0.5, W - 0.5, H - 0.5), 0, name="GND bottom")
    # actuator rails along the two long edges (top layer band with the NO1 pads, a second
    # band on the bottom layer next to the terminals), thermal reliefs on the pads
    zone("12V_ACT_A", F_Cu, rect(2.0, 7.2, W - 2.0, 16.4), 2, clearance=0.4, name="12V ACT A")
    zone("12V_ACT_B", F_Cu, rect(2.0, H - 16.4, W - 2.0, H - 7.2), 2, clearance=0.4, name="12V ACT B")
    zone("12V_ACT_A", B_Cu, rect(2.0, 7.5, W - 2.0, 12.5), 2, clearance=0.4, name="12V ACT A bottom")
    zone("12V_ACT_B", B_Cu, rect(2.0, H - 12.5, W - 2.0, H - 7.5), 2, clearance=0.4, name="12V ACT B bottom")


def silkscreen():
    W, H = D.BOARD_W, D.BOARD_H
    for n in range(1, 25):
        j, row = D.section_cell(n)
        xc = D.cell_x(j)
        y = 10.6 if row == "top" else H - 10.6
        text(str(n), xc, y, size=1.2, thick=0.2)
    text("ACT A  -  +", 5.8, 10.6, size=0.8, thick=0.12)
    text("+  -  ACT A", W - 5.8, 10.6, size=0.8, thick=0.12)
    text("-  +  ACT B", 6.0, H - 10.6, size=0.8, thick=0.12)
    text("+  -  ACT B", W - 6.0, H - 10.6, size=0.8, thick=0.12)
    text("12V IN", 199.5, 78.5, size=0.8, thick=0.12, rot=90)
    text("+", 203.0, 84.8, size=0.8, thick=0.15)
    text("-", 203.0, 72.5, size=0.8, thick=0.15)
    text("ESP32 DevKitC V4 - USB to the left, antenna to the right", 29.2, 60.0, size=1.0, thick=0.15)
    # DevKitC pin names next to each socket pin (under the module, readable with it removed)
    for ref, names, y0, rot in (("J30", D.ESP_J2_NAMES, 48.7, 270), ("J31", D.ESP_J3_NAMES, 71.3, 90)):
        for i, name in enumerate(names):
            x = pad_pos(ref, str(i + 1))[0]
            text(name, x, y0, size=0.8, thick=0.12, rot=rot, h="l")
    text("I2C", 59.5, 45.6, size=0.8, thick=0.12)
    text("ADC", 70.8, 45.6, size=0.8, thick=0.12)
    for ref, names in (("J33", ("3V3", "GND", "SDA", "SCL")), ("J34", ("3V3", "GND", "34", "35"))):
        for i, name in enumerate(names):
            text(name, pad_pos(ref, str(i + 1))[0], 48.95, size=0.8, thick=0.12)
    text("LNK", 55.0, 59.4, size=0.8, thick=0.12)
    text("SPD", 55.0, 57.7, size=0.8, thick=0.12)
    text("PWR", 141.6, 72.5, size=0.8, thick=0.12)
    text("Relay24_ESP32 v1.0  2026  renatorgr  GPL v3", 29.2, 63.0, size=0.8, thick=0.12)
    text("Relay24_ESP32 v1.0", W / 2, 60.0, layer=pcbnew.B_SilkS, size=2.0, thick=0.3)


def design_rules():
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = FromMM(0.2)
    ds.m_ViasMinSize = FromMM(0.5)
    ds.m_MinThroughDrill = FromMM(0.3)
    ds.m_MinClearance = FromMM(0.15)
    ds.m_ViasMinAnnularWidth = FromMM(0.13)
    ds.m_CopperEdgeClearance = FromMM(0.4)
    ds.m_HoleClearance = FromMM(0.25)
    ds.m_HoleToHoleMin = FromMM(0.5)
    ds.m_SilkClearance = FromMM(0.0)
    ds.SetCopperLayerCount(2)
    try:
        nc = ds.m_NetSettings.m_DefaultNetClass
        nc.SetClearance(FromMM(0.2))
        nc.SetTrackWidth(FromMM(SIG_W))
        nc.SetViaDiameter(FromMM(VIA_D))
        nc.SetViaDrill(FromMM(VIA_DRILL))
    except Exception as e:
        print("netclass:", e)
    tb = board.GetTitleBlock()
    tb.SetTitle(D.TITLE)
    tb.SetDate(D.DATE)
    tb.SetRevision(D.REV)
    tb.SetCompany("renatorgr")


# nets wired by the formulas above; the router does the rest
FORMULA_NETS = {"GND", "+12V", "+12V_IN", "SW", "12V_ACT_A", "12V_ACT_B"}
FORMULA_NETS |= {f"{p}{n}" for n in range(1, 25) for p in ("SEC", "RLY", "LEDA")}
FORMULA_NETS |= {f"SEC{n}_R" for n in range(1, 25)} | {f"SEC{n}_B" for n in range(1, 25)}
POWER_NETS = {"+3V3", "+5V", "+5VB"}
# routing regions: the strip between the two 12 V buses, the buck corner
REGIONS = [(1.0, 46.0, 209.0, 74.0), (181.0, 68.0, 209.5, 110.0)]


def router_jobs():
    """[(net, [(x, y, tht)])] in routing order, and the top-side ground pads."""
    order = []
    def rank(net):
        if net in ("TXP", "TXN", "RXP", "RXN", "RBIAS", "VDDCR", "ETH_CLKIN", "PHYAD0"):
            return 0                                   # the fine-pitch PHY pins first
        if net.startswith("DRV"):
            return 1
        if net in ("LED_LNK", "LED_SPD", "LED_LNK_K", "LED_SPD_A", "PWR_K"):
            return 1
        if net == "+3V3":
            return 1.5                                 # wide: before the signals weave around it
        if net.startswith("RMII") or net.startswith("ETH") or net.startswith("SR_") or net.startswith("I2C") \
                or net in ("A34", "A35"):
            return 2
        return 4
    for net, pins in D.NETS.items():
        if net.startswith("NC_") or net in FORMULA_NETS:
            continue
        pads = []
        for ref, pin in pins:
            if net == "+5VB" and ref != "D28":
                continue
            if net == "+3V3" and ref == "U9":
                continue                               # wired to its capacitor in phy_fanout()
            fp = board.FindFootprintByReference(ref)
            pad = fp.FindPadByNumber(pin)
            x, y = pad_pos(ref, pin)
            tht = pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
            pads.append((x, y, tht))
        order.append((rank(net), net, pads))
    order.sort(key=lambda t: (t[0], t[1]))
    gnd = []
    for fp in board.GetFootprints():
        if fp.GetReference() == "U9":
            continue
        for pad in fp.Pads():
            if fp.GetReference() in ("U1", "U2", "U3") and pad.GetNumber() == "8":
                continue                               # via placed in power_buses()
            if fp.GetReference() in ("C1", "C2", "C3", "C31"):
                continue                               # via placed in sr_fanout() / power_buses()
            if pad.GetNetname() == "GND" and pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
                p = pad.GetPosition()
                gnd.append((p.x / 1e6, p.y / 1e6))
    # last: a track from every top-side ground pad placed above to the nearest other
    # ground copper, so that no pad depends on the pour alone (the pads with a fixed
    # via and the header ground pins, which have a stub into the pour, are left out)
    order.append((5, "GND", [(x, y, False) for (x, y) in gnd]))
    order.sort(key=lambda t: (t[0], t[1]))
    return [(net, pads) for _, net, pads in order], gnd


def project_settings(folder):
    """DRC severities in the .kicad_pro: cosmetic checks off (footprints edited in place,
    overlapping silkscreen, thermal spokes, TXP/TXN taken for a differential pair)."""
    import json
    pro = os.path.join(folder, D.PROJECT + ".kicad_pro")
    data = {}
    if os.path.exists(pro):
        try:
            data = json.load(open(pro))
        except ValueError:
            data = {}
    ds = data.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rule_severities", {}).update({
        "lib_footprint_issues": "ignore", "lib_footprint_mismatch": "ignore", "silk_overlap": "ignore",
        "starved_thermal": "ignore", "diff_pair_gap_out_of_range": "ignore"})
    data.setdefault("meta", {"filename": D.PROJECT + ".kicad_pro", "version": 1})
    with open(pro, "w") as f:
        json.dump(data, f, indent=2)


def gnd_islands(brd):
    """Ground items that are not connected to the main ground: [((px, py), (vx, vy))] one per
    isolated group, the position of a top-side pad in it (or None) and of a via in it (or
    None). Uses the filled zone polygons."""
    import math
    gnd = brd.FindNet("GND").GetNetCode()
    items = []                                     # (kind, layers, x, y, r)
    tracks = []                                    # (index, x2, y2)
    L = {F_Cu: 0, B_Cu: 1}
    for fp in brd.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() != gnd:
                continue
            ls = {0, 1} if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else {0 if p.IsOnLayer(F_Cu) else 1}
            pos, sz = p.GetPosition(), p.GetSize()
            items.append(("pad", ls, pos.x / 1e6, pos.y / 1e6, min(sz.x, sz.y) / 2e6))
    for t in brd.GetTracks():
        if t.GetNetCode() != gnd:
            continue
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            items.append(("via", {0, 1}, pos.x / 1e6, pos.y / 1e6, t.GetWidth() / 2e6))
        else:
            a, e = t.GetStart(), t.GetEnd()
            items.append(("track", {L.get(t.GetLayer(), 0)}, a.x / 1e6, a.y / 1e6, t.GetWidth() / 2e6))
            tracks.append((len(items) - 1, e.x / 1e6, e.y / 1e6))
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        parent[find(i)] = find(j)

    for (i, ex, ey) in tracks:
        _, ls, sx, sy, r = items[i]
        for j, (kind, ls2, x, y, r2) in enumerate(items):
            if j != i and (ls & ls2) and any(math.hypot(px - x, py - y) <= r + r2 + 0.01 for (px, py) in ((sx, sy), (ex, ey))):
                union(i, j)
    pads = [(j, it) for j, it in enumerate(items) if it[0] in ("pad", "via")]
    for a, (i, (_, ls, sx, sy, r)) in enumerate(pads):
        for (j, (_, ls2, x, y, r2)) in pads[a + 1:]:
            if (ls & ls2) and math.hypot(sx - x, sy - y) <= r + r2 + 0.01:
                union(i, j)
    main_group = None
    for z in brd.Zones():
        if z.GetNetCode() != gnd:
            continue
        for layer, l in L.items():
            if not z.IsOnLayer(layer):
                continue
            polys = z.GetFilledPolysList(layer)
            for k in range(polys.OutlineCount()):
                members = []
                for j, (kind, ls2, x, y, r2) in enumerate(items):
                    if l not in ls2:
                        continue
                    pts = [(x, y), (x + r2, y), (x - r2, y), (x, y + r2), (x, y - r2)]
                    if kind == "track":
                        pts = pts[:1] + [(tracks[[t[0] for t in tracks].index(j)][1], tracks[[t[0] for t in tracks].index(j)][2])]
                    if any(polys.Contains(mm(px, py), k) for (px, py) in pts):
                        members.append(j)
                for j in members[1:]:
                    union(members[0], j)
                if members and (main_group is None or polys.Outline(k).Area() > main_group[0]):
                    main_group = (polys.Outline(k).Area(), find(members[0]))
    groups = {}
    for j, it in enumerate(items):
        groups.setdefault(find(j), []).append(it)
    out = []
    for root, members in groups.items():
        if main_group and root == main_group[1]:
            continue
        if all(kind == "pad" and 0 in ls and 1 in ls for kind, ls, *_ in members):
            continue                           # through-hole pads: the pour reaches them by spokes
        vias = [m for m in members if m[0] == "via"]
        pads = [m for m in members if m[0] == "pad" and 0 in m[1] and 1 not in m[1]]
        out.append(((pads[0][2], pads[0][3]) if pads else None, (vias[0][2], vias[0][3]) if vias else None))
    return out


def main_pour(brd):
    """Test whether a point lies in the largest filled polygon of the bottom ground pour."""
    gnd = brd.FindNet("GND").GetNetCode()
    best = None
    for z in brd.Zones():
        if z.GetNetCode() == gnd and z.IsOnLayer(B_Cu):
            polys = z.GetFilledPolysList(B_Cu)
            for k in range(polys.OutlineCount()):
                a = abs(polys.Outline(k).Area())
                if best is None or a > best[0]:
                    best = (a, polys, k)
    return (lambda x, y: bool(best[1].Contains(mm(x, y), best[2]))) if best else (lambda x, y: True)


def main(folder):
    global board
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, D.PROJECT + ".kicad_pcb")
    project_settings(folder)                 # before KiCad opens the project
    board = pcbnew.CreateEmptyBoard()
    board.SetFileName(path)
    design_rules()
    place_parts()
    outline()
    relay_cells()
    coil_buses()
    power_buses()
    zones()
    silkscreen()
    todo, gnd = router_jobs()
    r = router.Router(board, NETS, track, via, SIG_W, PWR_W, D.BOARD_W, D.BOARD_H)
    failed = r.run(REGIONS, todo, POWER_NETS, gnd)
    board.Save(path)
    print("saved", path, "router failures:", len(failed), flush=True)
    # the zone filler needs a board that belongs to a project: reload the saved file
    board = pcbnew.LoadBoard(path)
    for name in list(NETS):
        NETS[name] = board.FindNet(name)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    # ground vias that ended up in a pocket of the bottom pour: a track to the nearest ground
    for _ in range(3):
        lost = gnd_islands(board)
        if not lost:
            break
        print("isolated ground items:", [(p, v) for p, v in lost], flush=True)
        pour = main_pour(board)
        # first choice: drop the via and put a new one where the pour is continuous (the
        # stub to the old via is removed later as dangling)
        for (pad, v) in lost:
            if pad and v:
                for t in list(board.GetTracks()):
                    if isinstance(t, pcbnew.PCB_VIA) and abs(t.GetPosition().x / 1e6 - v[0]) < 0.01 \
                            and abs(t.GetPosition().y / 1e6 - v[1]) < 0.01:
                        board.Remove(t)
        r = router.Router(board, NETS, track, via, SIG_W, PWR_W, D.BOARD_W, D.BOARD_H)
        r.load_board(REGIONS)
        again = []
        for (pad, v) in lost:
            if pad and v:
                n = len(r.failed)
                r.ground_via(pad[0], pad[1], ok=pour)
                if len(r.failed) > n:
                    r.failed = r.failed[:n]
                    again.append((pad[0], pad[1], False))
            else:
                again.append((v or pad) + (v is not None,))
        if again:
            # second choice: a track to the nearest ground, moving signal tracks if needed
            r2 = router.Router(board, NETS, track, via, SIG_W, PWR_W, D.BOARD_W, D.BOARD_H)
            failed += r2.run(REGIONS, [("GND", again)], POWER_NETS, [], rippable=dict(todo))
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
    print("DRC:", remove_dangling(board, folder, path))


def remove_dangling(brd, folder, path):
    """What the DRC calls dangling (a via connected on one layer only, a track with a free
    end) is either a pre-placed via the router did not need, or the free end of a lane or
    bus that was joined in its middle. Vias and tracks that touch nothing are deleted; a
    track joined in its middle is cut back to the last junction. Repeats until clean and
    returns the DRC counts."""
    import math
    counts = None
    for _ in range(15):
        brd.Save(path)
        counts, dangling = drc(folder)
        if not dangling:
            break
        for (x, y, kind, netname, layer, length) in dangling:
            for t in list(brd.GetTracks()):
                isvia = isinstance(t, pcbnew.PCB_VIA)
                if isvia != (kind == "Via") or t.GetNetname() != netname:
                    continue
                p = t.GetPosition()
                if abs(p.x / 1e6 - x) > 0.01 or abs(p.y / 1e6 - y) > 0.01:
                    continue
                if not isvia and (brd.GetLayerName(t.GetLayer()) != layer or abs(t.GetLength() / 1e6 - length) > 0.01):
                    continue
                if isvia:
                    bridge_via(brd, t)
                    print("  dangling via removed:", netname, (x, y), flush=True)
                    brd.Remove(t)
                else:
                    trim_dangling(brd, t)
                break
        filler = pcbnew.ZONE_FILLER(brd)
        filler.Fill(brd.Zones())
    return counts


def bridge_via(brd, v):
    """A pre-placed via that the router only touched on one layer still joins the track
    ends that sit on its ring. Before the via goes, put a short track between them."""
    import math
    code, c = v.GetNetCode(), v.GetPosition()
    r = v.GetWidth() / 2e6 + 0.01
    ends = {}
    for t in brd.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) or t.GetNetCode() != code:
            continue
        for P in (t.GetStart(), t.GetEnd()):
            if math.hypot((P.x - c.x) / 1e6, (P.y - c.y) / 1e6) <= r:
                ends.setdefault(t.GetLayer(), []).append((P, t.GetWidth()))
    for layer, pts in ends.items():
        if len(pts) < 2:
            continue
        for (P, w) in pts:
            if P.x == c.x and P.y == c.y:
                continue
            s = pcbnew.PCB_TRACK(brd)
            s.SetStart(P)
            s.SetEnd(c)
            s.SetWidth(w)
            s.SetLayer(layer)
            s.SetNetCode(code)
            brd.Add(s)
            print("  via bridged:", v.GetNetname(), brd.GetLayerName(layer), (P.x / 1e6, P.y / 1e6), flush=True)


def trim_dangling(brd, t):
    """Cut a track with a free end back to its last junction, or delete it."""
    import math
    code, layer = t.GetNetCode(), t.GetLayer()
    w = t.GetWidth() / 2e6
    A = (t.GetStart().x / 1e6, t.GetStart().y / 1e6)
    B = (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)

    def seg_dist(P, S, E):
        dx, dy = E[0] - S[0], E[1] - S[1]
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((P[0] - S[0]) * dx + (P[1] - S[1]) * dy) / L2))
        return math.hypot(P[0] - S[0] - u * dx, P[1] - S[1] - u * dy)

    others, vias = [], []
    me = t.m_Uuid.AsString()
    for o in brd.GetTracks():
        if o.m_Uuid.AsString() == me or o.GetNetCode() != code:
            continue
        if isinstance(o, pcbnew.PCB_VIA):
            vias.append(((o.GetPosition().x / 1e6, o.GetPosition().y / 1e6), o.GetWidth() / 2e6))
        elif o.GetLayer() == layer:
            others.append(((o.GetStart().x / 1e6, o.GetStart().y / 1e6), (o.GetEnd().x / 1e6, o.GetEnd().y / 1e6),
                           o.GetWidth() / 2e6))
    pads = [p for fp in brd.GetFootprints() for p in fp.Pads() if p.GetNetCode() == code and p.IsOnLayer(layer)]
    zones = [z for z in brd.Zones() if z.GetNetCode() == code and z.IsOnLayer(layer)]

    def touched(P):
        if any(math.hypot(P[0] - S[0], P[1] - S[1]) < 0.02 or math.hypot(P[0] - E[0], P[1] - E[1]) < 0.02
               or seg_dist(P, S, E) < r + 0.01 for (S, E, r) in others):
            return True
        if any(math.hypot(P[0] - V[0], P[1] - V[1]) <= r + 0.01 for (V, r) in vias):
            return True
        if any(p.HitTest(mm(P[0], P[1])) for p in pads):
            return True
        return any(z.GetFilledPolysList(layer).Contains(mm(P[0], P[1])) for z in zones)

    ca, cb = touched(A), touched(B)
    if ca and cb:
        return
    if not ca and not cb:
        print("  dangling track removed:", t.GetNetname(), A, B, flush=True)
        brd.Remove(t)
        return
    fixed, free = (A, B) if ca else (B, A)
    joints = []
    for (S, E, r) in others:
        for P in (S, E):
            if seg_dist(P, A, B) < 0.02 and math.hypot(P[0] - A[0], P[1] - A[1]) > 0.02 \
                    and math.hypot(P[0] - B[0], P[1] - B[1]) > 0.02:
                joints.append(P)
    for (V, r) in vias:
        if seg_dist(V, A, B) < r and math.hypot(V[0] - free[0], V[1] - free[1]) > r:
            joints.append(V)
    if not joints:
        print("  dangling track removed:", t.GetNetname(), A, B, flush=True)
        brd.Remove(t)
        return
    J = max(joints, key=lambda P: math.hypot(P[0] - fixed[0], P[1] - fixed[1]))
    # keep the joint on the track's axis
    dx, dy = free[0] - fixed[0], free[1] - fixed[1]
    L2 = dx * dx + dy * dy
    u = ((J[0] - fixed[0]) * dx + (J[1] - fixed[1]) * dy) / L2
    J = (fixed[0] + u * dx, fixed[1] + u * dy)
    print("  dangling end trimmed:", t.GetNetname(), free, "->", (round(J[0], 3), round(J[1], 3)), flush=True)
    if ca:
        t.SetEnd(mm(J[0], J[1]))
    else:
        t.SetStart(mm(J[0], J[1]))


def drc(folder):
    """Run the DRC on the saved board in a new KiCad process (board.Save() rewrites the
    .kicad_pro from default settings, so the severities are written first and loaded by that
    process). Returns the counts found and the positions of the dangling vias/track ends."""
    import re
    import subprocess
    project_settings(folder)
    rep = os.path.join(folder, "drc.rpt")
    code = ("import pcbnew,sys\nb=pcbnew.LoadBoard(sys.argv[1])\n"
            "pcbnew.WriteDRCReport(b,sys.argv[2],pcbnew.EDA_UNITS_MILLIMETRES,True)\n")
    subprocess.run([sys.executable, "-c", code, os.path.join(folder, D.PROJECT + ".kicad_pcb"), rep], check=True)
    txt = open(rep).read()
    counts = re.findall(r"\*\* Found (\d+) (DRC violations|unconnected pads|Footprint errors)", txt)
    dangling, current = [], None
    for line in txt.splitlines():
        m = re.match(r"\[(\w+)\]:", line)
        if m:
            current = m.group(1)
        m = re.match(r"\s+@\(([-\d.]+) mm, ([-\d.]+) mm\): (Via|Track) \[(\S+)\] on (\S+?)(?:,| -) ?(?:length ([\d.]+) mm)?", line)
        if m and current in ("via_dangling", "track_dangling"):
            dangling.append((float(m.group(1)), float(m.group(2)), m.group(3), m.group(4), m.group(5),
                             float(m.group(6)) if m.group(6) else 0.0))
    return counts, dangling


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "..")
