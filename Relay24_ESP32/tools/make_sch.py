#!/usr/bin/env python3
"""
Writes the KiCad 7 schematic of Relay24_ESP32 from design.py.

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

Usage: python3 make_sch.py <output folder>

Every part is a symbol from KiCad's standard libraries (copied into the file, so the
project opens without extra libraries), except the RTL8201F, which KiCad does not have:
custom_symbol() draws it as a box with its pins. Every pin gets a short wire and a global label
with its net name, or a power symbol for GND / +3V3 / +5V / +12V, or a no-connect
cross. The root sheet holds the power, the ESP32 sockets, the Ethernet and the drivers;
three sub-sheets hold the 24 relays, one per driver.
"""
import copy
import math
import os
import sys
import uuid

import sexp
from sexp import q
import design as D

SYMDIR = "/usr/share/kicad/symbols/"
_libs = {}


def lib(name):
    if name not in _libs:
        with open(SYMDIR + name + ".kicad_sym") as f:
            top = sexp.parse(f.read())[0]
        _libs[name] = {str(s[1]): s for s in sexp.find_all(top, "symbol")}
    return _libs[name]


def flat_symbol(libname, name):
    """The symbol with its parent's drawing merged in (KiCad 'extends'), renamed to
    'lib:name' as the schematic's lib_symbols wants it."""
    node = copy.deepcopy(lib(libname)[name])
    ext = sexp.find(node, "extends")
    if ext:
        parent = flat_symbol(libname, str(ext[1]))
        props = {str(p[1]): p for p in sexp.find_all(node, "property")}
        merged = [x for x in parent if not (isinstance(x, list) and x and x[0] == "property")]
        pprops = {str(p[1]): p for p in sexp.find_all(parent, "property")}
        pprops.update(props)
        out = merged[:2] + list(pprops.values()) + [x for x in merged[2:] if not (isinstance(x, list) and x[0] == "symbol")]
        pname = str(ext[1])
        for unit in sexp.find_all(parent, "symbol"):
            u = copy.deepcopy(unit)
            u[1] = q(str(u[1]).replace(pname + "_", name + "_", 1))
            out.append(u)
        node = out
    node[1] = q(libname + ":" + name)
    for unit in sexp.find_all(node, "symbol"):
        s = str(unit[1])
        if ":" not in s:
            pass
    return node


def custom_symbol(name):
    """Symbols that KiCad's libraries do not have, drawn here as a box with named pins,
    in the same form as flat_symbol() returns."""
    if name != "RTL8201F":
        raise SystemExit("no custom symbol " + name)
    L, T = 17.78, 30.48                      # pin ends: x = +-L on the sides, y = +-T above / below
    font = ["effects", ["font", ["size", "1.27", "1.27"]]]
    hide = ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]

    def pin(num, pname, kind, x, y, ang):
        return ["pin", kind, "line", ["at", fmt(x), fmt(y), fmt(ang)], ["length", "2.54"],
                ["name", q(pname), copy.deepcopy(font)], ["number", q(num), copy.deepcopy(font)]]
    pins = []
    left = [("16", "TXD0", "input"), ("17", "TXD1", "input"), ("18", "TXD2", "input"), ("19", "TXD3", "input"),
            ("20", "TXEN", "input"), ("15", "TXC/REF_CLK", "bidirectional"), None,
            ("9", "RXD0", "output"), ("10", "RXD1", "output"), ("11", "RXD2/INTB", "bidirectional"),
            ("12", "RXD3/CLK_CTL", "bidirectional"), ("13", "RXC", "output"), ("8", "RXDV", "bidirectional"),
            ("26", "CRS_DV", "output"), ("28", "RXER", "output"), ("27", "COL", "output"), None,
            ("22", "MDC", "input"), ("23", "MDIO", "bidirectional"), ("21", "PHYRSTB", "input")]
    for i, e in enumerate(left):
        if e:
            pins.append(pin(e[0], e[1], e[2], -L, 22.86 - 2.54 * i, 0))
    right = [("3", "MDI+[0]", "bidirectional"), ("4", "MDI-[0]", "bidirectional"), ("5", "MDI+[1]", "bidirectional"),
             ("6", "MDI-[1]", "bidirectional"), None, ("24", "LED0/PHYAD0", "bidirectional"),
             ("25", "LED1/PHYAD1", "bidirectional"), None, ("1", "RSET", "passive")]
    for i, e in enumerate(right):
        if e:
            pins.append(pin(e[0], e[1], e[2], L, 22.86 - 2.54 * i, 180))
    for i, (num, pname, kind) in enumerate([("7", "AVDD33", "power_in"), ("30", "AVDD33", "power_in"),
                                            ("14", "DVDD33", "power_in"), ("2", "AVDD10OUT", "power_out"),
                                            ("29", "DVDD10OUT", "power_out")]):
        pins.append(pin(num, pname, kind, -10.16 + 5.08 * i, T, 270))
    for i, (num, pname, kind) in enumerate([("31", "CKXTAL1", "passive"), ("32", "CKXTAL2", "passive"),
                                            ("33", "EP/GND", "power_in")]):
        pins.append(pin(num, pname, kind, -5.08 + 5.08 * i, -T, 90))
    return ["symbol", q("Relay24:" + name), ["pin_names", ["offset", "1.016"]], ["in_bom", "yes"], ["on_board", "yes"],
            ["property", q("Reference"), q("U"), ["at", "-15.24", "29.21", "0"],
             ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "left"]]],
            ["property", q("Value"), q(name), ["at", "2.54", "29.21", "0"],
             ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "left"]]],
            ["property", q("Footprint"), q(""), ["at", "0", "0", "0"], copy.deepcopy(hide)],
            ["property", q("Datasheet"), q(""), ["at", "0", "0", "0"], copy.deepcopy(hide)],
            ["symbol", q(name + "_0_1"),
             ["rectangle", ["start", "-15.24", "27.94"], ["end", "15.24", "-27.94"],
              ["stroke", ["width", "0.254"], ["type", "default"]], ["fill", ["type", "background"]]]],
            ["symbol", q(name + "_1_1")] + pins]


def symbol_pins(node):
    """[(number, name, x, y, angle)] in symbol coordinates (y up)."""
    pins = []
    for unit in sexp.find_all(node, "symbol"):
        for p in sexp.find_all(unit, "pin"):
            at = sexp.find(p, "at")
            pins.append((str(sexp.find(p, "number")[1]), str(sexp.find(p, "name")[1]),
                         float(at[1]), float(at[2]), float(at[3])))
    return pins


def rot(vx, vy, deg):
    """Rotate a schematic-space vector (y down) counter-clockwise on screen."""
    a = math.radians(deg)
    return (vx * math.cos(a) + vy * math.sin(a), -vx * math.sin(a) + vy * math.cos(a))


def fmt(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def uid():
    return str(uuid.uuid4())


class Sheet:
    def __init__(self, name, paper, title):
        self.name, self.paper, self.title = name, paper, title
        self.uuid = uid()
        self.items = []
        self.libsyms = {}
        self.pwr_n = 0

    def use(self, libname, name):
        key = libname + ":" + name
        if key not in self.libsyms:
            self.libsyms[key] = custom_symbol(name) if libname == "Relay24" else flat_symbol(libname, name)
        return self.libsyms[key]

    def wire(self, x1, y1, x2, y2):
        self.items.append(["wire", ["pts", ["xy", fmt(x1), fmt(y1)], ["xy", fmt(x2), fmt(y2)]],
                           ["stroke", ["width", "0"], ["type", "default"]], ["uuid", q(uid())]])

    def label(self, name, x, y, angle):
        just = "right" if angle == 180 else "left"
        self.items.append(["global_label", q(name), ["shape", "input"], ["at", fmt(x), fmt(y), fmt(angle)],
                           ["fields_autoplaced"],
                           ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", just]], ["uuid", q(uid())],
                           ["property", q("Intersheetrefs"), q("${INTERSHEET_REFS}"), ["at", fmt(x), fmt(y), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]]])

    def no_connect(self, x, y):
        self.items.append(["no_connect", ["at", fmt(x), fmt(y)], ["uuid", q(uid())]])

    def text(self, s, x, y, size=1.5):
        self.items.append(["text", q(s), ["at", fmt(x), fmt(y), "0"],
                           ["effects", ["font", ["size", fmt(size), fmt(size)]], ["justify", "left", "bottom"]],
                           ["uuid", q(uid())]])

    def power(self, net, x, y, path, dx=0, dy=0):
        """A power symbol whose pin sits at (x, y), its drawing continuing in the
        direction (dx, dy) of the wire that leads to it."""
        libname, name = D.POWER_NETS[net].split(":")
        self.use(libname, name)
        self.pwr_n += 1
        ref = f"#PWR{self.pwr_n:03d}"
        down = net == "GND"                       # GND is drawn below its pin, the others above
        ang = {(0, 1): 0 if down else 180, (0, -1): 180 if down else 0,
               (1, 0): 90 if down else 270, (-1, 0): 270 if down else 90}[(dx, dy)]
        self.items.append(["symbol", ["lib_id", q(libname + ":" + name)], ["at", fmt(x), fmt(y), fmt(ang)],
                           ["unit", "1"], ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"],
                           ["uuid", q(uid())],
                           ["property", q("Reference"), q(ref), ["at", fmt(x), fmt(y), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                           ["property", q("Value"), q(net), ["at", fmt(x + dx * 5.5), fmt(y + dy * 5.5), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]]]],
                           ["property", q("Footprint"), q(""), ["at", fmt(x), fmt(y), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                           ["property", q("Datasheet"), q(""), ["at", fmt(x), fmt(y), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                           ["pin", q("1"), ["uuid", q(uid())]],
                           ["instances", ["project", q(D.PROJECT), ["path", q(path), ["reference", q(ref)], ["unit", "1"]]]]])

    def part(self, p, path):
        libname, name = p["sym"]
        node = self.use(libname, name)
        _, sx, sy, srot = p["sch"]
        pins = symbol_pins(node)
        fpname = p["fp"][0] + ":" + p["fp"][1]
        props = [["property", q("Reference"), q(p["ref"]), ["at", fmt(sx - 2), fmt(sy - 2), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "right"]]],
                 ["property", q("Value"), q(p["value"]), ["at", fmt(sx + 2), fmt(sy + 2), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "left"]]],
                 ["property", q("Footprint"), q(fpname), ["at", fmt(sx), fmt(sy), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                 ["property", q("Datasheet"), q(""), ["at", fmt(sx), fmt(sy), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                 ["property", q("LCSC"), q(p["lcsc"]), ["at", fmt(sx), fmt(sy), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]],
                 ["property", q("Description"), q(p["desc"]), ["at", fmt(sx), fmt(sy), "0"],
                  ["effects", ["font", ["size", "1.27", "1.27"]], "hide"]]]
        sym = ["symbol", ["lib_id", q(libname + ":" + name)], ["at", fmt(sx), fmt(sy), fmt(srot)], ["unit", "1"],
               ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"], ["uuid", q(uid())]] + props
        for num, _, _, _, _ in pins:
            sym.append(["pin", q(num), ["uuid", q(uid())]])
        sym.append(["instances", ["project", q(D.PROJECT), ["path", q(path), ["reference", q(p["ref"])], ["unit", "1"]]]])
        self.items.append(sym)
        for num, pname, px, py, pa in pins:
            net = p["pins"].get(num)
            vx, vy = rot(px, -py, srot)
            x, y = sx + vx, sy + vy
            dx, dy = rot(-math.cos(math.radians(pa)), math.sin(math.radians(pa)), srot)
            dx, dy = round(dx), round(dy)
            if net is None or net.startswith("NC_"):
                self.no_connect(x, y)
                continue
            L = 2.54
            x2, y2 = x + dx * L, y + dy * L
            self.wire(x, y, x2, y2)
            if net in D.POWER_NETS:
                self.power(net, x2, y2, path, dx, dy)
            else:
                ang = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[(dx, dy)]
                self.label(net, x2, y2, ang)

    def sheet_symbol(self, sub, x, y, w, h, page, path):
        self.items.append(["sheet", ["at", fmt(x), fmt(y)], ["size", fmt(w), fmt(h)], ["fields_autoplaced"],
                           ["stroke", ["width", "0.1524"], ["type", "solid"]], ["fill", ["color", "0", "0", "0", "0.0000"]],
                           ["uuid", q(sub.uuid)],
                           ["property", q("Sheetname"), q(sub.title), ["at", fmt(x), fmt(y - 0.7), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "left", "bottom"]]],
                           ["property", q("Sheetfile"), q(sub.name + ".kicad_sch"), ["at", fmt(x), fmt(y + h + 0.6), "0"],
                            ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", "left", "top"]]],
                           ["instances", ["project", q(D.PROJECT), ["path", q(path), ["page", q(str(page))]]]]])

    def write(self, folder, root_uuid=None, page=1):
        top = ["kicad_sch", ["version", "20230121"], ["generator", "eeschema"], ["uuid", q(self.uuid)],
               ["paper", q(self.paper)],
               ["title_block", ["title", q(self.title)], ["date", q(D.DATE)], ["rev", q(D.REV)], ["company", q("renatorgr")],
                ["comment", "1", q("Written by Claude (Anthropic) for renatorgr, GPL v3 or later")]],
               ["lib_symbols"] + list(self.libsyms.values())]
        top += self.items
        if root_uuid is None:
            top.append(["sheet_instances", ["path", q("/"), ["page", q("1")]]])
        with open(os.path.join(folder, self.name + ".kicad_sch"), "w") as f:
            f.write(sexp.dump(top) + "\n")


def main(folder):
    os.makedirs(folder, exist_ok=True)
    sheets = {name: Sheet(name if name != "root" else D.PROJECT, paper, title) for name, (paper, title) in D.SHEETS.items()}
    root = sheets["root"]
    root_path = "/" + root.uuid
    subs = [n for n in D.SHEETS if n != "root"]
    paths = {"root": root_path}
    for i, n in enumerate(subs):
        paths[n] = root_path + "/" + sheets[n].uuid
        root.sheet_symbol(sheets[n], 440 + 0 * i, 215 + i * 30, 120, 20, i + 2, root_path)
    for p in D.PARTS.values():
        sheets[p["sch"][0]].part(p, paths[p["sch"][0]])
    # notes
    root.text("12 V in: reverse protection D25, TVS D26, LM2596 buck to 5 V, AMS1117 to 3.3 V. D28 lets the DevKitC USB power the logic for bench tests.", 20, 100, 2)
    root.text("ESP32 DevKitC V4 (38 pins) in two 1x19 sockets. RMII pins are fixed by the ESP32: TXD0 19, TXD1 22, TXEN 21, RXD0 25, RXD1 26, CRS_DV 27, MDC 23, MDIO 18, 50 MHz reference clock in on GPIO0 (from the PHY).", 200, 130, 2)
    root.text("Shift registers: SER 13, SRCLK 16, RCLK 32, OE 4 (pulled up: all relays off until the program starts). PHY reset 33. I2C 15/14. Analog 34/35.", 200, 136, 2)
    root.text("RTL8201F on a 25 MHz crystal (Y1, C34/C35); the 50 MHz RMII clock comes OUT of TXC -> R30 -> GPIO0 (ETH_CLOCK_GPIO0_IN). R33 keeps the PHY in reset until GPIO33 is high, so GPIO0 is free at boot.", 330, 190, 2)
    root.text("Straps: RXDV up (R35) = RMII; LED0/LED1 down (R32/R36) = PHY address 0, LEDs active high: D30 = 10 Mbit link/activity, D31 = 100 Mbit (default LED mode). RSET 2.49 k (R31). 49.9 ohm terminations to 3.3 V (R38-R41).", 330, 196, 2)
    root.text("RJ45 HR911105A: PHY pair 0 (transmit) on pins 1-2, pair 1 (receive) on 3-6, centre taps (4, 5, 7, 8) to 3.3 V with C36/C37, its own LEDs unused.", 330, 202, 2)
    root.text("Drivers: 74HC595 (3.3 V) -> ULN2803A (coils at 12 V, flyback diodes inside, COM to +12V). One ULN per sheet of 8 relays.", 20, 275, 2)
    for n in subs:
        sheets[n].text("Each section: relay K (DPDT, coil A1 +12V / A2 driven low by the ULN), terminal J (1 = red wire, 2 = black wire). Relay off: red at GND, black at +12V ACT (pull in, closed). Relay on: red +12V ACT, black GND (push out, open).", 20, 280, 2)
        sheets[n].text("LED D with R 2.2k across the coil shows the section open. 12V_ACT_A = sections 1-12 (J26/J27), 12V_ACT_B = sections 13-24 (J28/J29).", 20, 286, 2)
    root.write(folder)
    for n in subs:
        sheets[n].write(folder, root.uuid)
    print("wrote", folder, [s.name for s in sheets.values()])


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "..")
