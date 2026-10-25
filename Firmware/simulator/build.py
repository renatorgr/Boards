#!/usr/bin/env python3
# build.py - builds the web page simulator (a static site for GitHub Pages) from the firmware's own pages.
# Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
# Written 2026 by Claude (Anthropic) for renatorgr.
# Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# python3 Firmware/simulator/build.py [output folder, default _site]
#
# Takes the newest Firmware/vX.Y_AiO_WEB folder (as the module's update check does), cuts the page out of
# zWebPage.h, the XIAO page out of vX.Y_XIAO_ESP32/xPage.h and the sprayer module's page out of
# Sprayer/Sprayer_ESP32/sPage.h exactly as the firmware serves them, and puts the simulator scripts in their
# <head>. Also passes them what the firmware knows and the simulator shows: the versions, the AgOpenGPS
# receiver settings (zRxConfig.h), the size of the XIAO programs. Writes:
#   index.html          the start page (index.html here, with the versions and the commit filled in)
#   teensy/index.html   the Teensy page
#   xiao/index.html     the XIAO page
#   sprayer/index.html  the sprayer module's page
#   sim-*.js            the simulator
# Every page of the site carries the visit counter below (GoatCounter: no cookies, nothing personal kept, the
# path without the choices). The counter is only here, in the simulator: the firmware's own pages have none.

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


def commit():
    sha = os.environ.get("GITHUB_SHA", "")
    if not sha:
        try:
            sha = subprocess.run(["git", "-C", HERE, "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            sha = ""
    return sha


COUNTER = ('<script>window.goatcounter={path:function(){return location.pathname}};</script>\n'
           '<script data-goatcounter="https://renatorgr.goatcounter.com/count" async src="https://gc.zgo.at/count.js"></script>\n')


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
    ubx_sets, um982d, um982s = rx_config(os.path.join(fw, "zRxConfig.h"))
    now = datetime.datetime.now(datetime.timezone.utc)
    built = "%s %2d %d %s" % (now.strftime("%b"), now.day, now.year, now.strftime("%H:%M:%S"))   # as __DATE__ __TIME__
    sha = commit()
    xiao_bin = {}
    for chip in ("c3", "s3"):
        p = os.path.join(bins, "v%s_XIAO_ESP32%s.bin" % (version, chip.upper()))
        if os.path.exists(p):
            xiao_bin[chip] = os.path.getsize(p)
    build = {"version": version, "folder": folder, "built": built, "commit": sha[:7], "ubxSets": ubx_sets,
             "um982d": um982d, "um982s": um982s, "xiaoBin": xiao_bin, "sprayer": sprayer}
    head = "<script>window.SIM_BUILD=%s;</script>\n" % json.dumps(build, ensure_ascii=False).replace("</", "<\\/")
    pages = {
        "teensy": (os.path.join(fw, "zWebPage.h"), "WEB_PAGE", "sim-teensy.js"),
        "xiao": (os.path.join(xiao, "xPage.h"), "XIAO_PAGE", "sim-xiao.js"),
        "sprayer": (os.path.join(SPRAYER, "sPage.h"), "SPRAYER_PAGE", "sim-sprayer.js"),
    }
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    for name, (path, var, script) in pages.items():
        html = inject(raw_string(path, var), head + '<script src="../sim-common.js"></script>\n<script src="../%s"></script>\n' % script + COUNTER,
                      os.path.relpath(path, ROOT))
        os.makedirs(os.path.join(out, name))
        with open(os.path.join(out, name, "index.html"), "w", encoding="utf-8") as f:
            f.write(html)
    for js in ("sim-common.js", "sim-teensy.js", "sim-xiao.js", "sim-sprayer.js"):
        shutil.copy(os.path.join(HERE, js), out)
    start = read(os.path.join(HERE, "index.html"))
    for k, v in {"VERSION": version, "FOLDER": folder, "SPRAYER_VERSION": sprayer["version"], "BUILT": now.strftime("%d %b %Y, %H:%M UTC"),
                 "COUNTER": COUNTER.rstrip("\n"),
                 "COMMIT": sha[:7] or "local",
                 "COMMIT_URL": "https://github.com/%s/commit/%s" % (REPO, sha) if sha else "https://github.com/" + REPO}.items():
        start = start.replace("{{" + k + "}}", v)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(start)
    open(os.path.join(out, ".nojekyll"), "w").close()
    print("Simulator of %s and %s v%s (commit %s) in %s" % (folder, sprayer["name"], sprayer["version"], sha[:7] or "local", out))


if __name__ == "__main__":
    main()
