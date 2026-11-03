#!/usr/bin/env python3
# check_changelog.py - keeps the README changelogs short: one row per day, a summary, the details in the sections.
# Part of the Boards repository (github.com/renatorgr/Boards); run by .github/workflows/docs.yml on every pull request.
# Written 2026-11-03 by Claude (Anthropic) for renatorgr, after the 3 Nov 2026 row grew to 2000 characters.
# Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# What it checks, in every README.md that has a "## Changelog" table (or in the files given on the command line):
#   - each row is "| 3 Nov 2026 | text |": a real date, newest first, one row per day;
#   - the text reads as at most MAX_CHARS characters (links and bold marks not counted): a summary, not the story;
#   - every link to a section of the same file, "(#heading)", points to a heading that exists.
# A row that needs more words is the sign that the words belong in a section lower in the file, with a link
# to it from the row. Exit code 1 and a plain message name the file and the row when something is wrong.
#
# Usage: python3 .github/scripts/check_changelog.py [README.md ...]

import os
import re
import sys
from datetime import datetime

MAX_CHARS = 240                      # visible characters of the "What changed" cell (about two lines on GitHub)
SKIP_DIRS = {".git", "node_modules", "build", "_site", "ext", "skbuild"}

ROW_RE = re.compile(r"^\|\s*(?P<date>\d{1,2} [A-Za-z]{3} \d{4})\s*\|\s*(?P<text>.*?)\s*\|\s*$")
LINK_RE = re.compile(r"\[([^\]]*)\]\(([^)]*)\)")
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")


def anchor(heading):
    """The anchor GitHub gives a heading: lower case, punctuation dropped, spaces to hyphens."""
    text = re.sub(r"`([^`]*)`", r"\1", heading)                 # `code` keeps its letters
    text = LINK_RE.sub(r"\1", text)                              # [text](url) keeps the text
    text = text.lower().strip()
    text = re.sub(r"[^\w\- ]", "", text)                         # letters, digits, _, - and spaces stay
    return text.replace(" ", "-")


def anchors(lines):
    seen = {}
    out = set()
    in_code = False
    for line in lines:
        if line.strip().startswith("```"):
            in_code = not in_code
            continue
        if in_code:
            continue
        m = HEADING_RE.match(line)
        if not m:
            continue
        a = anchor(m.group(2))
        n = seen.get(a, 0)
        seen[a] = n + 1
        out.add(a if n == 0 else f"{a}-{n}")
    return out


def visible(text):
    """The text as the reader sees it: link targets, bold and code marks removed."""
    text = LINK_RE.sub(r"\1", text)
    return re.sub(r"\*\*|`", "", text)


def changelog_rows(lines):
    """(line number, date text, cell text) for each row of the table under '## Changelog'; [] when there is none."""
    rows = []
    i = 0
    while i < len(lines) and not re.match(r"^##\s+Changelog\s*$", lines[i]):
        i += 1
    if i == len(lines):
        return rows
    i += 1
    while i < len(lines) and not lines[i].startswith("|"):
        if lines[i].startswith("#"):                             # the next heading: no table
            return rows
        i += 1
    while i < len(lines) and lines[i].startswith("|"):
        line = lines[i]
        if not re.match(r"^\|\s*-+", line) and not re.match(r"^\|\s*Date\s*\|", line, re.I):
            m = ROW_RE.match(line)
            rows.append((i + 1, m.group("date") if m else None, m.group("text") if m else line))
        i += 1
    return rows


def check(path):
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")
    rows = changelog_rows(lines)
    if not rows:
        return None
    problems = []
    known = anchors(lines)
    last_date = None
    longest = 0
    for lineno, date, text in rows:
        where = f"{path}:{lineno}"
        if date is None:
            problems.append(f"{where}: a changelog row must look like '| 3 Nov 2026 | what changed |'")
            continue
        try:
            d = datetime.strptime(date, "%d %b %Y")
        except ValueError:
            problems.append(f"{where}: '{date}' is not a date like '3 Nov 2026'")
            continue
        if last_date is not None and d >= last_date:
            problems.append(f"{where}: {date} is not older than the row above it (newest first, one row per day)")
        last_date = d
        shown = visible(text)
        longest = max(longest, len(shown))
        if len(shown) > MAX_CHARS:
            problems.append(f"{where}: the {date} row reads as {len(shown)} characters, the limit is {MAX_CHARS}: "
                            f"keep the row a summary and put the details in a section below, with a link to it")
        for label, target in LINK_RE.findall(text):
            if target.startswith("#") and target[1:] not in known:
                problems.append(f"{where}: the link '{label}' points to {target}, and no heading of this file gives that anchor")
    return rows, problems, longest


def readmes(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        if "README.md" in filenames:
            yield os.path.join(dirpath, "README.md")


def main(argv):
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    paths = argv or sorted(readmes(root))
    failed = False
    checked = 0
    for path in paths:
        result = check(path)
        if result is None:
            continue
        rows, problems, longest = result
        checked += 1
        shown = os.path.relpath(path, root) if not argv else path
        if problems:
            failed = True
            print("\n".join(problems))
        else:
            print(f"OK {shown}: {len(rows)} changelog rows, the longest reads as {longest} characters (limit {MAX_CHARS})")
    if checked == 0:
        print("no README.md with a '## Changelog' table found")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
