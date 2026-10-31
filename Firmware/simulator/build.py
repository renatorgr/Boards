#!/usr/bin/env python3
# build.py - builds the web page simulator (a static site for GitHub Pages) from the firmware's own pages.
# Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
# Written 2026 by Claude (Anthropic) for renatorgr.
# 2026-10-27 (Claude): the seed drill module's page (Seeder/Seeder_ESP32/dPage.h); each module page fixes its implement.
# 2026-10-31 (Claude): firmware.json, the date and time the Teensy .hex and the XIAO .bin files on main were compiled
#                      (the update check of both pages compares it with the running program's).
# Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# python3 Firmware/simulator/build.py [output folder, default _site]
#
# Takes the newest Firmware/vX.Y_AiO_WEB folder (as the module's update check does), cuts the page out of
# zWebPage.h, the XIAO page out of vX.Y_XIAO_ESP32/xPage.h, the sprayer module's page out of
# Sprayer/Sprayer_ESP32/sPage.h and the seed drill module's page out of Seeder/Seeder_ESP32/dPage.h exactly as
# the firmware serves them, and puts the simulator scripts in their <head>. Also passes them what the firmware
# knows and the simulator shows: the versions, the AgOpenGPS receiver settings (zRxConfig.h), the size of the
# XIAO programs; a module page also gets its implement (window.SIM_IMPL). Writes:
#   index.html          the start page (index.html here, with the versions and the commit filled in)
#   teensy/index.html   the Teensy page
#   xiao/index.html     the XIAO page
#   sprayer/index.html  the sprayer module's page
#   seeder/index.html   the seed drill module's page
#   sim-*.js            the simulator
#   firmware.json       what "Install from GitHub" offers: the Teensy .hex and the XIAO .bin files of that folder, each
#                       with the date and time it was compiled, read from the file itself (the __DATE__ __TIME__ text
#                       every program carries). The Teensy page and the XIAO page fetch it from
#                       https://renatorgr.github.io/Boards/firmware.json, so a new build of the same version (4.5)
#                       shows as new as soon as its file is on main.

import datetime
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = os.path.dirname(HERE)
ROOT = os.path.dirname(FIRMWARE)
SPRAYER = os.path.join(ROOT, "Sprayer", "Sprayer_ESP32")
SEEDER = os.path.join(ROOT, "Seeder", "Seeder_ESP32")
REPO = "renatorgr/Boards"


def fail(msg):
    sys.exit("build.py: " + msg)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def newest_folder():
    found = []
    for d in os.listdir(FIRMWARE):
        m = re.fullmatch(r"v(\d+(?:\.\d+)*)_AiO_WEB", d)
        if m and os.path.isdir(os.path.join(FIRMWARE, d)):
            found.append((tuple(int(x) for x in m.group(1).split(".")), d))
    if not found:
        fail("no Firmware/vX.Y_AiO_WEB folder")
    return max(found)[1]


def raw_string(path, name):
    """The C++ raw string literal of `name` (const char name[] PROGMEM = R"TAG(...)TAG";)."""
    m = re.search(r"\b" + name + r"\[\]\s*PROGMEM\s*=\s*R\"(\w*)\((.*?)\)\1\"", read(path), re.S)
    if not m:
        fail("no raw string %s in %s" % (name, path))
    return m.group(2)


def define(path, name):
    m = re.search(r"#define\s+" + name + r"\s+\"([^\"]*)\"", read(path))
    if not m:
        fail("no #define %s in %s" % (name, path))
    return m.group(1)


def rx_config(path):
    """The AgOpenGPS receiver settings: the u-blox sets and the two UM982 command lists (zRxConfig.h)."""
    text = read(path)
    m = re.search(r"RXCFG_UBX\[RXCFG_UBX_N\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if not m:
        fail("no RXCFG_UBX in " + path)
    sets = [{"name": a, "file": b, "fw": c, "count": int(d), "keys": int(e)}
            for a, b, c, d, e in re.findall(r"\{\"([^\"]*)\",\s*\"([^\"]*)\",\s*\"([^\"]*)\",\s*\w+,\s*(\d+),\s*(\d+)\}", m.group(1))]
    lists = {}
    for name in ("RXCFG_UM982D", "RXCFG_UM982S"):
        m = re.search(name + r"\[\]\s*=\s*\{(.*?)\};", text, re.S)
        if not m:
            fail("no %s in %s" % (name, path))
        lists[name] = re.findall(r"\"([^\"]*)\"", m.group(1))
    if len(sets) < 4 or not lists["RXCFG_UM982D"] or not lists["RXCFG_UM982S"]:
        fail("zRxConfig.h: the receiver settings were not all found")
    return sets, lists["RXCFG_UM982D"], lists["RXCFG_UM982S"]


MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
BUILT_RX = re.compile(rb"(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec) ([ 0-3]\d) (\d{4}) (\d\d:\d\d:\d\d)")


def built_key(built):
    """ "Oct 31 2026 19:28:53" (as __DATE__ __TIME__) -> "20261031192853", which sorts by time; "" if not a date."""
    m = BUILT_RX.fullmatch(built.encode())
    return "%s%02d%02d%s" % (m.group(3).decode(), MONTHS.index(m.group(1).decode()) + 1, int(m.group(2)),
                             m.group(4).decode().replace(":", "")) if m else ""


def built_in(data):
    """The newest __DATE__ __TIME__ text in a program image, or "". The sketch is compiled every time, so its
    date is the newest; a library compiled earlier and kept by the IDE could only carry an older one."""
    best = ""
    for m in BUILT_RX.finditer(data):
        b = m.group(0).decode()
        if built_key(b) > built_key(best):
            best = b
    return best


def hex_bytes(path):
    """The data of an Intel HEX file (the Teensy .hex), in address order. Only read to find the build date."""
    mem, base = {}, 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            b = bytes.fromhex(line[1:])
            n, addr, kind, data = b[0], (b[1] << 8) | b[2], b[3], b[4:4 + b[0]]
            if kind == 0:
                mem[base + addr] = data
            elif kind == 4:
                base = ((data[0] << 8) | data[1]) << 16
            elif kind == 2:
                base = ((data[0] << 8) | data[1]) << 4
    return b"".join(mem[a] for a in sorted(mem))


def published(fw, folder, version, bins):
    """firmware.json: the files "Install from GitHub" takes from the folder, with the date each was compiled.
    The Teensy .hex: vX.Y_AiO_WEB.hex or the Arduino IDE's name vX.Y_AiO_WEB.ino.hex (the newer if both are there)."""
    teensy = None
    for name in (folder + ".hex", folder + ".ino.hex"):
        p = os.path.join(fw, name)
        if not os.path.isfile(p):
            continue
        try:
            built = built_in(hex_bytes(p))
        except (ValueError, IndexError, KeyError, UnicodeDecodeError):
            built = ""   # not a readable .hex: offered without a date (the page then compares the version only)
        if teensy is None or built_key(built) > built_key(teensy["built"]):
            teensy = {"file": name, "built": built, "size": os.path.getsize(p)}
    xiao = {}
    for chip in ("C3", "S3"):
        name = "v%s_XIAO_ESP32%s.bin" % (version, chip)
        p = os.path.join(bins, name)
        if os.path.isfile(p):
            with open(p, "rb") as f:
                built = built_in(f.read())
            xiao[chip] = {"file": os.path.basename(bins) + "/" + name, "built": built, "size": os.path.getsize(p)}
    return {"version": version, "folder": folder, "teensy": teensy, "xiao": xiao}


def commit():
    sha = os.environ.get("GITHUB_SHA", "")
    if not sha:
        try:
            sha = subprocess.run(["git", "-C", HERE, "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            sha = ""
    return sha


def inject(html, scripts, what):
    """The simulator scripts right after <head>, before the page's own script; "Simulator" in the title."""
    if html.count("<head>") != 1 or html.count("<title>") != 1:
        fail(what + ": expected one <head> and one <title>")
    return html.replace("<head>", "<head>\n" + scripts, 1).replace("<title>", "<title>Simulator · ", 1)


def main():
    out = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "_site")
    folder = newest_folder()
    fw = os.path.join(FIRMWARE, folder)
    xiao = os.path.join(fw, folder.replace("_AiO_WEB", "_XIAO_ESP32"))
    bins = os.path.join(fw, folder.replace("_AiO_WEB", "_XIAO_ESP32_bin"))
    version = define(os.path.join(fw, "zVersion.h"), "FW_VERSION")
    spr_ino = os.path.join(SPRAYER, "Sprayer_ESP32.ino")
    sprayer = {"name": define(spr_ino, "FW_NAME"), "version": define(spr_ino, "FW_VERSION"), "folder": os.path.relpath(SPRAYER, ROOT)}
    sd_ino = os.path.join(SEEDER, "Seeder_ESP32.ino")
    seeder = {"name": define(sd_ino, "FW_NAME"), "version": define(sd_ino, "FW_VERSION"), "folder": os.path.relpath(SEEDER, ROOT)}
    ubx_sets, um982d, um982s = rx_config(os.path.join(fw, "zRxConfig.h"))
    now = datetime.datetime.now(datetime.timezone.utc)
    built = "%s %2d %d %s" % (now.strftime("%b"), now.day, now.year, now.strftime("%H:%M:%S"))   # as __DATE__ __TIME__
    sha = commit()
    pub = published(fw, folder, version, bins)
    pub["commit"] = sha
    xiao_bin = {chip.lower(): x["size"] for chip, x in pub["xiao"].items()}
    xiao_built = {chip.lower(): x["built"] for chip, x in pub["xiao"].items() if x["built"]}
    if pub["teensy"] and pub["teensy"]["built"]:
        built = pub["teensy"]["built"]   # the pretend Teensy runs the build that is on main, as an updated module does
    build = {"version": version, "folder": folder, "built": built, "commit": sha[:7], "ubxSets": ubx_sets,
             "um982d": um982d, "um982s": um982s, "xiaoBin": xiao_bin, "xiaoBuilt": xiao_built, "sprayer": sprayer, "seeder": seeder}
    head = "<script>window.SIM_BUILD=%s;</script>\n" % json.dumps(build, ensure_ascii=False).replace("</", "<\\/")
    pages = {   # folder: the page, its raw string, its script, the implement it is for (a module page)
        "teensy": (os.path.join(fw, "zWebPage.h"), "WEB_PAGE", "sim-teensy.js", ""),
        "xiao": (os.path.join(xiao, "xPage.h"), "XIAO_PAGE", "sim-xiao.js", ""),
        "sprayer": (os.path.join(SPRAYER, "sPage.h"), "SPRAYER_PAGE", "sim-sprayer.js", "sprayer"),
        "seeder": (os.path.join(SEEDER, "dPage.h"), "SEEDER_PAGE", "sim-seeder.js", "seeder"),
    }
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    for name, (path, var, script, impl) in pages.items():
        page_head = head + ("<script>window.SIM_IMPL=%s;</script>\n" % json.dumps(impl) if impl else "")
        html = inject(raw_string(path, var), page_head + '<script src="../sim-common.js"></script>\n<script src="../%s"></script>\n' % script,
                      os.path.relpath(path, ROOT))
        os.makedirs(os.path.join(out, name))
        with open(os.path.join(out, name, "index.html"), "w", encoding="utf-8") as f:
            f.write(html)
    for js in ("sim-common.js", "sim-teensy.js", "sim-xiao.js", "sim-sprayer.js", "sim-seeder.js"):
        shutil.copy(os.path.join(HERE, js), out)
    start = read(os.path.join(HERE, "index.html"))
    for k, v in {"VERSION": version, "FOLDER": folder, "SPRAYER_VERSION": sprayer["version"], "SEEDER_VERSION": seeder["version"],
                 "BUILT": now.strftime("%d %b %Y, %H:%M UTC"),
                 "COMMIT": sha[:7] or "local",
                 "COMMIT_URL": "https://github.com/%s/commit/%s" % (REPO, sha) if sha else "https://github.com/" + REPO}.items():
        start = start.replace("{{" + k + "}}", v)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(start)
    with open(os.path.join(out, "firmware.json"), "w", encoding="utf-8") as f:
        json.dump(pub, f, indent=1)
    open(os.path.join(out, ".nojekyll"), "w").close()
    print("Simulator of %s, %s v%s and %s v%s (commit %s) in %s" % (folder, sprayer["name"], sprayer["version"], seeder["name"], seeder["version"],
                                                                   sha[:7] or "local", out))


if __name__ == "__main__":
    main()
