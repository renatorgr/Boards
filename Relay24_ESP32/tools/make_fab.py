#!/usr/bin/env python3
"""
JLCPCB fabrication files for Relay24_ESP32: Gerber + drill zip, BOM and CPL (pick and
place) CSVs, plus PNG views of the board.

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

Usage: python3 make_fab.py <project folder>     (needs kicad-cli 7 and rsvg-convert)

Two BOM/CPL pairs are written: *_all.csv with every part that has a footprint, and
*_smd.csv with the surface-mount parts only (for an order where the relays, terminals,
sockets and the RJ45 are soldered by hand).
"""
import csv
import os
import subprocess
import sys
import zipfile

import pcbnew

import design as D

COPPER = ["F.Cu", "B.Cu"]
GERBER_LAYERS = COPPER + ["F.Paste", "B.Paste", "F.SilkS", "B.SilkS", "F.Mask", "B.Mask", "Edge.Cuts"]
THT_REFS = {p["ref"] for p in D.PARTS.values()
            if p["fp"][0] in ("Relay_THT", "TerminalBlock", "Connector_PinSocket_2.54mm",
                              "Connector_PinHeader_2.54mm", "Connector_RJ")}

# JLCPCB places a part at the centre of its body. KiCad anchors surface-mount footprints
# there too, but through-hole footprints on pin 1, so the CPL "Mid X/Y" of a through-hole
# part is the centre of its courtyard, not the footprint position. JLCPCB's 2.54 mm pin
# header and socket models also sit 90 degrees from KiCad's (checked on the JLCPCB
# assembly preview, where the DevKitC sockets came out across the board), hence the
# rotation offsets below, by footprint library.
ROT_OFFSET = {"Connector_PinSocket_2.54mm": 90, "Connector_PinHeader_2.54mm": 90}


def mid(fp, tht):
    """CPL position of a footprint in mm: the anchor for SMD, the courtyard centre for THT."""
    if not tht:
        p = fp.GetPosition()
        return p.x / 1e6, p.y / 1e6
    box = None
    for item in fp.GraphicalItems():
        if item.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            if box is None:
                box = item.GetBoundingBox()
            else:
                box.Merge(item.GetBoundingBox())
    c = box.GetCenter()
    return c.x / 1e6, c.y / 1e6


def run(cmd):
    print(" ", " ".join(cmd))
    subprocess.run(cmd, check=True)


def gerbers(folder, fab):
    pcb = os.path.join(folder, D.PROJECT + ".kicad_pcb")
    tmp = os.path.join(fab, "gerber_tmp")
    os.makedirs(tmp, exist_ok=True)
    for f in os.listdir(tmp):
        os.remove(os.path.join(tmp, f))
    run(["kicad-cli", "pcb", "export", "gerbers", "-o", tmp + "/", "--layers", ",".join(GERBER_LAYERS),
         "--no-x2", "--no-netlist", "--subtract-soldermask", pcb])
    run(["kicad-cli", "pcb", "export", "drill", "-o", tmp + "/", "--format", "excellon", "--excellon-units", "mm",
         "--excellon-zeros-format", "decimal", "--drill-origin", "absolute", "--generate-map",
         "--map-format", "gerberx2", pcb])
    zpath = os.path.join(fab, f"Gerber_{D.PROJECT}.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(tmp)):
            z.write(os.path.join(tmp, f), f)
    for f in os.listdir(tmp):
        os.remove(os.path.join(tmp, f))
    os.rmdir(tmp)
    print("wrote", zpath)


def bom_cpl(folder, fab):
    board = pcbnew.LoadBoard(os.path.join(folder, D.PROJECT + ".kicad_pcb"))
    rows = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref.startswith("H"):
            continue                                  # mounting holes
        p = D.PARTS[ref]
        tht = ref in THT_REFS
        x, y = mid(fp, tht)
        rot = (fp.GetOrientationDegrees() + ROT_OFFSET.get(p["fp"][0], 0)) % 360
        rows.append(dict(ref=ref, value=p["value"], fpname=p["fp"][1], lcsc=p["lcsc"],
                         x=x, y=y, rot=rot,
                         layer="Top" if fp.GetLayer() == pcbnew.F_Cu else "Bottom",
                         smd=not tht))

    def refkey(r):
        head = r.rstrip("0123456789")
        return (head, int(r[len(head):]) if r[len(head):] else 0)

    for variant, sel in (("all", rows), ("smd", [r for r in rows if r["smd"]])):
        groups = {}
        for r in sel:
            groups.setdefault((r["value"], r["fpname"], r["lcsc"]), []).append(r["ref"])
        bom = os.path.join(fab, f"BOM_{D.PROJECT}_{variant}.csv")
        with open(bom, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
            for (value, fpname, lcsc), refs in sorted(groups.items(), key=lambda kv: refkey(sorted(kv[1], key=refkey)[0])):
                w.writerow([value, ",".join(sorted(refs, key=refkey)), fpname, lcsc])
        cpl = os.path.join(fab, f"CPL_{D.PROJECT}_{variant}.csv")
        with open(cpl, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
            for r in sorted(sel, key=lambda r: refkey(r["ref"])):
                w.writerow([r["ref"], f"{r['x']:.3f}mm", f"{-r['y']:.3f}mm", r["layer"], f"{r['rot']:.0f}"])
        print("wrote", bom, "and", cpl, f"({len(sel)} parts, {len(groups)} lines)")


def renders(folder):
    pcb = os.path.join(folder, D.PROJECT + ".kicad_pcb")
    views = {
        "pcb_top.png": (["F.Cu", "F.SilkS", "F.Mask", "Edge.Cuts"], []),
        "pcb_bottom.png": (["B.Cu", "B.SilkS", "B.Mask", "Edge.Cuts"], ["--mirror"]),
        "pcb_copper_top.png": (["F.Cu", "Edge.Cuts"], []),
        "pcb_copper_bottom.png": (["B.Cu", "Edge.Cuts"], ["--mirror"]),
    }
    for name, (layers, extra) in views.items():
        svg = os.path.join(folder, name.replace(".png", ".svg"))
        run(["kicad-cli", "pcb", "export", "svg", "-o", svg, "--layers", ",".join(layers), "--page-size-mode", "2",
             "--exclude-drawing-sheet"] + extra + [pcb])
        run(["rsvg-convert", "-w", "2400", "-o", os.path.join(folder, name), svg])
        os.remove(svg)
    sch = os.path.join(folder, D.PROJECT + ".kicad_sch")
    run(["kicad-cli", "sch", "export", "pdf", "-o", os.path.join(folder, D.PROJECT + "_schematic.pdf"), sch])


def main(folder):
    fab = os.path.join(folder, "fabrication_JLCPCB")
    os.makedirs(fab, exist_ok=True)
    gerbers(folder, fab)
    bom_cpl(folder, fab)
    renders(folder)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "..")
