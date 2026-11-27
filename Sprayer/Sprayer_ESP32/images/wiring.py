#!/usr/bin/env python3
# wiring.py - draws wiring.png, the wiring of the sprayer module, for the Sprayer_ESP32 README.
# Part of the Boards repository (github.com/renatorgr/Boards). History: README changelog and git log.
# Usage: python3 wiring.py   (needs rsvg-convert, apt package librsvg2-bin, and Pillow; writes wiring.png)
# Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING).
# SPDX-License-Identifier: GPL-3.0-or-later

import os
import subprocess
import tempfile

from PIL import Image

W, H = 1600, 1330
RED, BLK, BLU, GRN, YEL = "#dc2626", "#111827", "#2563eb", "#16a34a", "#ca8a04"
ORG, GREY, BOARD = "#c2410c", "#6b7280", "#1e40af"
out = []


def line(pts, c=BLK, w=4, dash=None):
    d = " ".join(f"{x},{y}" for x, y in pts)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<polyline points="{d}" fill="none" stroke="{c}" stroke-width="{w}" '
               f'stroke-linejoin="round" stroke-linecap="round"{extra}/>')


def text(x, y, s, size=20, c=BLK, bold=False, anchor="start"):
    s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    out.append(f'<text x="{x}" y="{y}" font-family="DejaVu Sans, Arial, sans-serif" font-size="{size}" '
               f'fill="{c}" text-anchor="{anchor}"{" font-weight=\"bold\"" if bold else ""}>{s}</text>')


def rect(x, y, w, h, fill="white", stroke=BLK, sw=3, r=6, dash=None):
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" '
               f'stroke-width="{sw}"{extra}/>')


def dot(x, y, c=BLK, r=7):
    out.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{c}"/>')


def pin(x, y, c="#fbbf24"):
    out.append(f'<circle cx="{x}" cy="{y}" r="9" fill="{c}" stroke="{BLK}" stroke-width="2.5"/>')


def resistor_v(x, y1, y2, label, c=BLK, side=1):
    """Vertical resistor from (x, y1) to (x, y2), the label beside it."""
    m = (y1 + y2) / 2
    line([(x, y1), (x, m - 28)], c)
    line([(x, m + 28), (x, y2)], c)
    rect(x - 10, m - 28, 20, 56, fill="#f5e6c8", stroke=BLK, sw=2.5, r=3)
    text(x + side * 18, m + 7, label, 19, BLK, True, "start" if side > 0 else "end")


def fuse(x1, x2, y, label, c=RED):
    m = (x1 + x2) / 2
    line([(x1, y), (m - 30, y)], c)
    line([(m + 30, y), (x2, y)], c)
    rect(m - 30, y - 12, 60, 24, fill="white", stroke=BLK, sw=2.5, r=4)
    line([(m - 30, y), (m + 30, y)], BLK, 2)
    text(m, y - 20, label, 19, BLK, True, "middle")


def diode_h(x1, x2, y, c=ORG):
    """Horizontal diode, cathode (band) on the left at x1."""
    m = (x1 + x2) / 2
    line([(x1, y), (m - 14, y)], c, 3)
    line([(m + 14, y), (x2, y)], c, 3)
    out.append(f'<polygon points="{m - 14},{y} {m + 14},{y - 12} {m + 14},{y + 12}" fill="{c}"/>')
    line([(m - 14, y - 12), (m - 14, y + 12)], c, 4)


# ---- page
out.append(f'<rect x="0" y="0" width="{W}" height="{H}" fill="white"/>')
text(40, 56, "Sprayer module wiring", 38, BLK, True)
text(40, 94, "ESP32_Relay_X8 board, 8 section valves, pressure sensor and flow meter. A drawing of the "
     "connections, not of the box.", 21, "#374151")

# ---- battery and the two feeds
BX1, BY1, BX2, BY2 = 470, 175, 1010, 870
BY1_TOP = BY1
rect(40, 600, 130, 100, fill="#e5e7eb")
text(105, 642, "12 V", 24, BLK, True, "middle")
text(105, 672, "battery", 20, BLK, False, "middle")
text(78, 592, "+", 28, RED, True, "middle")
text(132, 728, "−", 30, BLK, True, "middle")
line([(78, 600), (78, 150)], RED)
dot(78, 245, RED)
# feed A: 1 A fuse -> board 7-30V, always on
fuse(78, 470, 245, "1 A fuse")
text(300, 282, "board: own feed,", 18, GREY, False, "middle")
text(300, 304, "not through the switch", 18, GREY, False, "middle")
# feed B: 5 A fuse -> master switch -> COM bus
fuse(78, 290, 150, "5 A fuse")
line([(290, 150), (330, 150)], RED)
dot(330, 150, BLK, 6)
line([(330, 150), (384, 126)], BLK, 4)
dot(390, 150, BLK, 6)
text(360, 190, "master switch", 18, BLK, True, "middle")
line([(390, 150), (890, 150), (890, BY1_TOP)], RED)
text(640, 138, "+12 V for the valves", 18, RED, True, "middle")

# ---- the board
rect(BX1, BY1, BX2 - BX1, BY2 - BY1, fill="#dbeafe", stroke=BOARD, sw=4, r=10)
text(500, 212, "ESP32_Relay_X8 board", 24, BOARD, True)
# P1 power terminal
rect(470, 222, 22, 140, fill="#16a34a", stroke=BLK, sw=2, r=3)
for y, s in ((245, "7-30V"), (295, "GND"), (345, "5V (not used)")):
    pin(481, y, "#d1d5db")
    text(505, y + 7, s, 19, BLK)
# ESP32
rect(520, 420, 230, 150, fill="#9ca3af", stroke=BLK, sw=2.5)
text(635, 485, "ESP32", 24, "white", True, "middle")
text(635, 520, "drives relays 1-8", 18, "white", False, "middle")
text(635, 545, "GPIO 32 33 25 26 27 14 12 13", 15, "white", False, "middle")
line([(750, 495), (810, 495)], GREY, 3, "6 6")
# valves on NO, polarity normal: a module with no power closes every valve
rect(500, 582, 290, 222, fill="#fef2f2", stroke=RED, sw=3, r=8)
text(516, 612, "Valves on NO", 20, RED, True)
for k, s in enumerate(("Leave 'relay active high'",
                       "unticked in AgOpenWeb.",
                       "Relay off, meter on beep:",
                       "COM + NC beeps,",
                       "COM + NO is silent.",
                       "A module with no power",
                       "then closes every valve.")):
    text(516, 642 + 23 * k + (6 if k >= 2 else 0) + (6 if k >= 5 else 0), s, 17, BLK, k >= 5)

# relays: COM bus at x=890, COM at 910, NC at (990, y-22), NO at (990, y)
ROWS = [320 + 70 * i for i in range(8)]
line([(890, BY1), (890, ROWS[-1])], RED)          # the COM bridge, drawn over the board
for i, y in enumerate(ROWS):
    rect(810, y - 20, 50, 40, fill="#3b82f6", stroke=BLK, sw=2, r=4)
    text(835, y + 8, str(i + 1), 21, "white", True, "middle")
    dot(890, y, RED, 6)
    line([(890, y), (910, y)], RED)
    pin(910, y, "#d1d5db")
    pin(990, y - 24, "#e5e7eb")
    pin(990, y, "#d1d5db")
    line([(916, y - 5), (982, y - 22)], BLK, 3)      # relay off: COM rests on NC
text(910, 290, "COM", 15, BLK, True, "middle")
text(990, 268, "NC", 15, GREY, True, "middle")
text(1032, 310, "NO", 15, BLK, True, "middle")
text(868, 262, "COM bridged", 16, RED, True, "end")
text(868, 282, "1 to 8", 16, RED, True, "end")

# ---- valves with their diodes
for i, y in enumerate(ROWS):
    line([(999, y), (1110, y)], BLU)
    dot(1110, y, BLU, 6)
    line([(1110, y), (1150, y)], BLU)
    rect(1150, y - 16, 140, 32, fill="#e0e7ff", stroke=BLK, sw=2.5, r=4)
    text(1220, y + 7, f"valve {i + 1}", 18, BLK, True, "middle")
    line([(1290, y), (1560, y)], BLK)
    dot(1330, y, BLK, 6)
    line([(1110, y), (1110, y + 30)], ORG, 3)
    diode_h(1110, 1330, y + 30)
    line([(1330, y + 30), (1330, y)], ORG, 3)
    dot(1560, y, BLK, 6)
line([(1560, ROWS[0]), (1560, 1270)], BLK)
text(1080, 312, "+", 22, BLU, True, "middle")
text(1360, 312, "−", 24, BLK, True, "middle")

# diode note
rect(1040, 150, 545, 120, fill="#fff7ed", stroke=ORG, sw=3, r=8)
text(1060, 182, "1N4007 diode across each valve", 21, ORG, True)
text(1060, 210, "Band (stripe) on the + side: the wire from NO.", 18, BLK)
text(1060, 234, "At the valve, or in the box from each NO to GND.", 18, BLK)
text(1060, 258, "Reversed, it shorts the 12 V when the relay closes.", 18, BLK)

# ---- sensors and dividers
# header pins on the board's bottom edge
for x, s in ((500, "5V"), (600, "G34"), (720, "GND"), (840, "G35"), (960, "GND")):
    pin(x, BY2, "#fbbf24")
    text(x, BY2 - 18, s, 17, BLK, True, "middle")
text(500, 830, "pin headers:", 16, GREY, False, "start")
# pressure: G34 node at (600, 950), 10k series to the sensor, 10k to GND
rect(565, 925, 430, 140, fill="none", stroke=GREY, sw=2, r=8, dash="8 6")
line([(600, BY2 + 9), (600, 950)], GRN)
dot(600, 950, GRN, 6)
resistor_v(600, 950, 1110, "10 kΩ", GRN, -1)
line([(600, 950), (660, 950)], GRN)
resistor_v(660, 950, 1045, "10 kΩ", BLK, 1)
line([(660, 1045), (720, 1045)], BLK)
dot(720, 1045, BLK, 6)
line([(720, BY2 + 9), (720, 1110)], BLK)
# flow: G35 node at (840, 950), 10k series, 22k to GND
line([(840, BY2 + 9), (840, 950)], YEL)
dot(840, 950, YEL, 6)
resistor_v(840, 950, 1110, "10 kΩ", YEL, -1)
line([(840, 950), (900, 950)], YEL)
resistor_v(900, 950, 1045, "22 kΩ", BLK, 1)
line([(900, 1045), (960, 1045)], BLK)
dot(960, 1045, BLK, 6)
line([(960, BY2 + 9), (960, 1110)], BLK)
# sensors
rect(540, 1110, 220, 80, fill="#f3f4f6", stroke=BLK, sw=2.5)
text(650, 1143, "pressure sensor", 18, BLK, True, "middle")
text(650, 1168, "0.5-4.5 V, 0-10 bar", 16, GREY, False, "middle")
rect(780, 1110, 220, 80, fill="#f3f4f6", stroke=BLK, sw=2.5)
text(890, 1143, "flow meter", 18, BLK, True, "middle")
text(890, 1168, "YF-S201", 16, GREY, False, "middle")
# 5 V to both sensors, round the bottom
line([(500, BY2 + 9), (500, 1225), (820, 1225)], RED)
line([(580, 1225), (580, 1190)], RED)
dot(580, 1225, RED, 6)
line([(820, 1225), (820, 1190)], RED)
text(510, 1252, "5 V from the board to both sensors", 17, RED, True)
text(1010, 940, "dividers on a", 17, GREY, False)
text(1010, 962, "small perfboard", 17, GREY, False)

# ---- common GND
line([(470, 295), (440, 295), (440, 1270)], BLK)
line([(132, 700), (132, 1270), (1560, 1270)], BLK)
dot(440, 1270, BLK, 6)
dot(1560, 1270, BLK, 6)
text(160, 1300, "GND: battery −, board GND and every valve's − joined", 18, BLK, True)

# ---- checks
rect(1040, 1000, 495, 240, fill="#f9fafb", stroke="#d1d5db", sw=2, r=8)
text(1060, 1034, "Check before the 12 V", 21, BLK, True)
for k, s in enumerate(("Sections off: every valve shut. Board's",
                       "1 A fuse out, master on: still all shut.",
                       "Sensors on their own shielded cable,",
                       "with its GND to the board's GND pin.",
                       "G34 and G35 take 3.3 V at most: test",
                       "the dividers before the sensors go on.")):
    text(1060 if k % 2 == 0 else 1078, 1068 + 28 * k + (8 if k >= 2 else 0) + (8 if k >= 4 else 0),
         ("• " if k % 2 == 0 else "") + s, 17, BLK)

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
       + "\n".join(out) + "</svg>\n")
here = os.path.dirname(os.path.abspath(__file__))
png = os.path.join(here, "wiring.png")
with tempfile.TemporaryDirectory(dir=here) as tmp:     # wiring.png is only replaced once the new one is whole
    src, raw, small = (os.path.join(tmp, n) for n in ("wiring.svg", "raw.png", "small.png"))
    with open(src, "w", encoding="utf-8") as f:
        f.write(svg)
    subprocess.run(["rsvg-convert", "-o", raw, src], check=True)
    Image.open(raw).convert("RGB").quantize(colors=128).save(small, optimize=True)   # flat colours: a small file
    os.replace(small, png)
print("wrote", png)
