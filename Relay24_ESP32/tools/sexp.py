#!/usr/bin/env python3
"""
Tiny S-expression reader / writer for KiCad files (symbols, schematics).

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

A parsed file is nested Python lists; atoms are str, quoted strings are Quoted (a str
subclass) so they are written back with quotes.
"""


class Quoted(str):
    pass


def parse(text):
    """Parse one or more S-expressions; returns a list of top-level nodes."""
    i, n = 0, len(text)
    stack = [[]]
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c == "(":
            stack.append([])
            i += 1
        elif c == ")":
            node = stack.pop()
            stack[-1].append(node)
            i += 1
        elif c == '"':
            j = i + 1
            out = []
            while text[j] != '"':
                if text[j] == "\\":
                    out.append(text[j:j + 2])
                    j += 2
                else:
                    out.append(text[j])
                    j += 1
            stack[-1].append(Quoted("".join(out)))
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in "()":
                j += 1
            stack[-1].append(text[i:j])
            i = j
    return stack[0]


def q(s):
    return Quoted(str(s))


def dump(node, indent=0):
    """Write a node back as text, one list per line beyond the first level."""
    if isinstance(node, Quoted):
        return '"' + str(node) + '"'
    if isinstance(node, str):
        return node
    simple = all(not isinstance(x, list) for x in node)
    if simple:
        return "(" + " ".join(dump(x) for x in node) + ")"
    pad = "  " * (indent + 1)
    parts = []
    head = []
    for x in node:
        if isinstance(x, list):
            break
        head.append(dump(x))
    rest = node[len(head):]
    inline = [dump(x) for x in rest if not isinstance(x, list) or all(not isinstance(y, list) for y in x) and len(dump(x)) < 60]
    # keep short child lists on the same line as the head when they all are short
    if rest and all(not isinstance(x, list) or len(dump(x)) < 60 and all(not isinstance(y, list) for y in x) for x in rest) \
            and sum(len(s) for s in inline) < 100:
        return "(" + " ".join(head + inline) + ")"
    out = "(" + " ".join(head)
    for x in rest:
        out += "\n" + pad + dump(x, indent + 1)
    out += "\n" + "  " * indent + ")"
    return out


def find(node, key):
    """First child list whose head is key."""
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def find_all(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]
