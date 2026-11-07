#!/usr/bin/env python3
"""Shared code check: the blocks the sprayer and the seed drill modules have in common must stay the same.

Sprayer/Sprayer_ESP32 and Seeder/Seeder_ESP32 are two programs with one origin: the Wi-Fi, the web helpers, the
settings for the network, the PGN handling and the relay driving are the same code in both. Those parts are
marked in the sources, with the same name in both programs:

    // ==== shared "web-helpers": ...
    ... the code ...
    // ==== end shared "web-helpers"

This script reads every .ino of both programs, pairs the blocks by name and fails when a block differs, is
missing from one program, is nested, or is not closed. Run it from the repository root: no arguments, no
packages, exit code 1 when something is wrong. The "Shared code" GitHub Action runs it on every pull request
that touches either program.

Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING).
SPDX-License-Identifier: GPL-3.0-or-later
"""
import difflib
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PROGRAMS = {"Sprayer_ESP32": "Sprayer/Sprayer_ESP32", "Seeder_ESP32": "Seeder/Seeder_ESP32"}
START = re.compile(r'^\s*// ==== shared "([^"]+)"')
END = re.compile(r'^\s*// ==== end shared "([^"]+)"\s*$')


def read_blocks(folder, errors):
    """{name: (file, first line number, lines)} of one program; problems go to errors"""
    found = {}
    for path in sorted(glob.glob(os.path.join(ROOT, folder, "*.ino"))):
        rel = os.path.relpath(path, ROOT)
        with open(path, encoding="utf-8") as f:
            lines = f.read().split("\n")
        open_name, open_at, body = None, 0, []
        for n, line in enumerate(lines, 1):
            m = START.match(line)
            if m:
                if open_name:
                    errors.append(f'{rel}:{n}: shared "{m.group(1)}" starts inside "{open_name}" (opened at line {open_at})')
                    continue
                open_name, open_at, body = m.group(1), n, []
                continue
            m = END.match(line)
            if m:
                if not open_name:
                    errors.append(f'{rel}:{n}: end of shared "{m.group(1)}" without a start')
                elif m.group(1) != open_name:
                    errors.append(f'{rel}:{n}: end of shared "{m.group(1)}" closes "{open_name}" (opened at line {open_at})')
                else:
                    if open_name in found:
                        errors.append(f'{rel}:{open_at}: shared "{open_name}" is already in {found[open_name][0]}:{found[open_name][1]}')
                    else:
                        found[open_name] = (rel, open_at + 1, body)
                    open_name = None
                continue
            if open_name:
                body.append(line.rstrip())
        if open_name:
            errors.append(f'{rel}:{open_at}: shared "{open_name}" is never closed')
    return found


def main():
    errors = []
    blocks = {name: read_blocks(folder, errors) for name, folder in PROGRAMS.items()}
    (a, blocks_a), (b, blocks_b) = blocks.items()
    for name in sorted(set(blocks_a) ^ set(blocks_b)):
        where = blocks_a.get(name) or blocks_b.get(name)
        other = b if name in blocks_a else a
        errors.append(f'{where[0]}:{where[1]}: shared "{name}" has no block of that name in {other}')
    same_lines = 0
    for name in sorted(set(blocks_a) & set(blocks_b)):
        fa, la, ba = blocks_a[name]
        fb, lb, bb = blocks_b[name]
        if ba == bb:
            same_lines += len(ba)
            continue
        diff = difflib.unified_diff(ba, bb, fromfile=f"{fa}:{la}", tofile=f"{fb}:{lb}", lineterm="", n=1)
        errors.append(f'shared "{name}" differs between {a} and {b}: change both the same way, or move the line out of the block\n' + "\n".join(diff))
    for e in errors:
        print(e)
    if errors:
        print(f"{len(errors)} problem(s) in the shared blocks")
        return 1
    n = len(set(blocks_a) & set(blocks_b))
    print(f"OK: {n} shared blocks, {same_lines} lines the same in {a} and {b}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
