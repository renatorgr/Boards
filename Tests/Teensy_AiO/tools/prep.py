#!/usr/bin/env python3
# prep.py - turns the Teensy sketch into one C++ file the PC can build, as the Arduino IDE does, plus the few
# changes a PC needs.
# Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
# Written 2026 by Claude (Anthropic) for renatorgr.
# The history of changes is in the README changelog and in the git log.
# Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# python3 prep.py <sketch folder> <out folder>
#
# 1. The .ino files become one file: the main .ino first, then the others by name, with a prototype of every
#    function before the first function (what the Arduino IDE generates), so a call to a function written
#    further down builds as it does in the IDE.
# 2. The sketch's .h, .cpp and .c files are copied next to it (FlashTxx.c is left out: the pretend Teensy has
#    its own flash, fake_core.cpp).
# 3. Three things only an ARM chip can run are changed in the copies, never in the sketch itself:
#    - inline assembly (asm volatile(...)) and naked functions: the two interrupt entries of zSafety.ino. The
#      pretend Teensy calls their C halves (wdtWarnC, faultHookC) directly.
#    - fixed RAM addresses (0x2027FFxx, the core's crash record): they point into a pretend RAM2 that keeps its
#      content across a restart, as the real one does.
#    - FLASH_BASE_ADDR (FlashTxx.h): the pretend 8 MB flash. (The running firmware's size, OTA_IMAGE_LEN in zOTA.ino,
#      is given on the compiler's command line: the size of the .hex the harness put in that flash.) The addresses
#      in an uploaded .hex are the real ones (0x60000000 on): otaPutByte (zOTA.ino) moves them into the pretend
#      flash first; an address outside the real 8 MB becomes 0, which its range check refuses as before.
import os
import re
import shutil
import sys

KEYWORDS = {"if", "while", "for", "switch", "return", "sizeof", "catch", "do", "else", "new", "delete"}


def blank_comments_and_strings(text):
    """Same length and same line breaks, with comments, strings and character literals replaced by spaces."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                if text[j] == "\\":
                    j += 1
                if j < n and text[j] == "\n":
                    break
                j += 1
            j = min(j + 1, n)
            out.append(c + re.sub(r"[^\n]", " ", text[i + 1:j - 1]) + (c if j - 1 > i else ""))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def blank_preprocessor(text):
    """Preprocessor lines (with their continuations) as spaces, line breaks kept."""
    lines = text.split("\n")
    cont = False
    for k, l in enumerate(lines):
        if cont or l.lstrip().startswith("#"):
            cont = l.rstrip().endswith("\\")
            lines[k] = " " * len(l)
    return "\n".join(lines)


def strip_defaults(args):
    """'int a = 3, char b = 'x'' -> 'int a, char b' (parentheses and braces inside a default kept apart)."""
    out, depth, skip = [], 0, False
    for ch in args:
        if ch in "([{<":
            depth += 1
        elif ch in ")]}>":
            depth -= 1
        if depth == 0 and ch == "=":
            skip = True
            continue
        if depth == 0 and ch == ",":
            skip = False
        if not skip:
            out.append(ch)
    return "".join(out).strip()


SIG = re.compile(r"^(?P<pre>[\w\s\*&:<>,]*?[\w\*&>])\s*\b(?P<name>[A-Za-z_]\w*)\s*\((?P<args>[^;{}]*)\)\s*(?P<const>const)?\s*$", re.S)


def prototypes(code):
    """Prototypes of the functions defined at the top level of code (comments, strings and # lines blanked)."""
    protos, first_at = [], None
    depth, start = 0, 0
    for i, ch in enumerate(code):
        if ch == "{":
            if depth == 0:
                head = code[start:i].strip()
                m = SIG.match(head)
                if m and m.group("name") not in KEYWORDS and not re.search(r"\b(extern|template|operator|struct|class|enum|union|namespace|typedef)\b", head) \
                        and "=" not in m.group("pre"):
                    pre = " ".join(m.group("pre").split())
                    if pre and pre.split()[-1] not in KEYWORDS:
                        args = " ".join(strip_defaults(m.group("args")).split())
                        protos.append("%s %s(%s)%s;" % (pre, m.group("name"), args, " const" if m.group("const") else ""))
                        if first_at is None:
                            first_at = start + (len(code[start:i]) - len(code[start:i].lstrip()))
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                start = i + 1
        elif ch == ";" and depth == 0:
            start = i + 1
        elif ch == "\n" and depth == 0 and code[start:i].strip() == "":
            start = i + 1
    return protos, first_at


def strip_asm(text):
    """asm volatile( ... ); -> ; and __attribute__((naked)) -> nothing."""
    text = re.sub(r"__attribute__\s*\(\(\s*naked\s*\)\)", "", text)
    out, i = [], 0
    pat = re.compile(r"\b(?:__asm__|asm)\s*(?:volatile|__volatile__)?\s*\(")
    while True:
        m = pat.search(text, i)
        if not m:
            out.append(text[i:])
            break
        out.append(text[i:m.start()])
        j, depth, instr = m.end(), 1, None
        while j < len(text) and depth:
            c = text[j]
            if instr:
                if c == "\\":
                    j += 1
                elif c == instr:
                    instr = None
            elif c in "\"'":
                instr = c
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            j += 1
        removed = text[m.start():j]
        out.append("/* asm taken out for the PC */" + "\n" * removed.count("\n"))
        i = j
    return "".join(out)


def ram2_addresses(text):
    return re.sub(r"\b0x2027FF([0-9A-Fa-f]{2})\b", lambda m: "fake_teensy_ram2(0x2027FF%su)" % m.group(1), text)


def hex_addresses(text):
    """An uploaded .hex has the real flash addresses: moved into the pretend flash where zOTA.ino takes a byte."""
    head = "static bool otaPutByte(uint32_t a, uint8_t v)\n{\n"
    if head not in text:
        sys.exit("prep.py: otaPutByte (zOTA.ino) not found: the harness cannot run the firmware update")
    return text.replace(head, head + "    a = (a >= 0x60000000u && a < 0x60800000u) ? a - 0x60000000u + FLASH_BASE_ADDR : 0;   // (harness)\n", 1)


def main():
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    name = os.path.basename(os.path.normpath(src))
    main_ino = os.path.join(src, name + ".ino")
    inos = [main_ino] + sorted(os.path.join(src, f) for f in os.listdir(src) if f.endswith(".ino") and f != name + ".ino")

    parts = []
    for f in inos:
        with open(f, encoding="utf-8") as fh:
            parts.append((f, fh.read()))
    merged = ""
    for f, t in parts:
        merged += '#line 1 "%s"\n' % f + t + ("\n" if not t.endswith("\n") else "")
    code = blank_preprocessor(blank_comments_and_strings(merged))
    protos, at = prototypes(code)
    line_no = merged.count("\n", 0, at) + 1
    # the #line that tells the compiler which file and line the first function is on
    file_at, line_in_file = None, 0
    for m in re.finditer(r'#line 1 "([^"]+)"\n', merged):
        if m.start() < at:
            file_at, line_in_file = m.group(1), merged.count("\n", m.end(), at) + 1
    body = merged[:at] + "\n// prototypes, as the Arduino IDE adds them\n" + "\n".join(protos) + \
        '\n#line %d "%s"\n' % (line_in_file, file_at) + merged[at:]
    body = hex_addresses(ram2_addresses(strip_asm(body)))
    with open(os.path.join(out, name + ".cpp"), "w", encoding="utf-8") as fh:
        fh.write('#include <Arduino.h>\n' + body)

    for f in os.listdir(src):
        p = os.path.join(src, f)
        if not os.path.isfile(p) or f == "FlashTxx.c" or not f.endswith((".h", ".cpp", ".c")):
            continue
        with open(p, encoding="utf-8", errors="replace") as fh:
            t = fh.read()
        if f == "FlashTxx.h":
            t, n = re.subn(r"#define\s+FLASH_BASE_ADDR\s*\(0x60000000\)", "#define FLASH_BASE_ADDR\t(fake_teensy_flash())", t)
            if n == 0:
                sys.exit("prep.py: FLASH_BASE_ADDR (FlashTxx.h) not found: the harness cannot give the firmware its flash")
        t = ram2_addresses(strip_asm(t))
        with open(os.path.join(out, f), "w", encoding="utf-8") as fh:
            fh.write(t)
    print("%s: %d .ino files, %d prototypes, first function at line %d" % (name, len(inos), len(protos), line_no))


if __name__ == "__main__":
    main()
