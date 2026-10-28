#!/usr/bin/env python3
"""
A small grid router for the logic part of Relay24_ESP32 (used by make_pcb.py).

Written 2026 by Claude (Anthropic) for renatorgr. SPDX-License-Identifier: GPL-3.0-or-later

Two copper layers on a 0.25 mm grid. Everything already on the board (pads, tracks, vias)
becomes an obstacle with a clearance halo; the cells that belong to the net being routed
stay passable and are the targets. Each remaining pad of a net is joined to the copper
the net already has with A* (vertical runs preferred on the top layer, where the parts are, and
horizontal runs on the bottom layer; a via costs as much as 12 mm of track; the heuristic is the exact
taxicab distance to the nearest target cell). When a pad is boxed in by tracks routed earlier,
those nets are ripped up, the pad is routed, and the ripped nets are routed again afterwards
(each net at most three times). Ground pads of surface-mount parts get a short track and a
via into the bottom ground pour.
"""
import heapq
from collections import deque

import numpy as np
import pcbnew
from pcbnew import F_Cu, B_Cu

G = 0.25            # grid pitch, mm
CLR = 0.2           # clearance between copper of different nets
VIA_R = 0.3         # via radius (0.6 mm via)
VIA_COST = 48       # in grid steps (12 mm)
TURN_COST = 2
OFF_PREF = 2        # extra cost of a step against the layer's preferred direction
EDGE = -2           # owner value of cells nothing may use
DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
DIRS8 = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0)]
VIA_CHECK = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)] + [(-2, 0), (2, 0), (0, -2), (0, 2)]


class Router:
    def __init__(self, board, nets, track, via, sig_w, pwr_w, W, H):
        self.board, self.nets = board, nets
        self.track, self.via = track, via
        self.sig_w, self.pwr_w = sig_w, pwr_w
        self.W, self.H = W, H
        self.nx = int(round(W / G)) + 1
        self.ny = int(round(H / G)) + 1
        shape = (2, self.ny, self.nx)
        # owner: 0 free, >0 net code of the copper there, EDGE forbidden
        self.owner = np.zeros(shape, dtype=np.int32)
        # halos: 0 none, >0 the only net whose clearance covers the cell, -1 several nets.
        # halo_s is for signal tracks, halo_p for the wider power tracks.
        self.halo_s = np.zeros(shape, dtype=np.int32)
        self.halo_p = np.zeros(shape, dtype=np.int32)
        self.failed = []
        self.items = {}        # net name -> board items laid by the router
        self.segs = {}         # net name -> [(layers, x1, y1, x2, y2, w, code)] for re-marking
        self.rips = {}         # net name -> how many times it was ripped up
        self.requeue = []      # ripped-up nets waiting to be routed again
        self.static = None     # the obstacle arrays as loaded from the board
        self.code2name = {}
        self.pads_of = {}

    # ------------------------------------------------------------------ marking
    def cell(self, x, y):
        return int(round(x / G)), int(round(y / G))

    def _span(self, x1, y1, x2, y2):
        ix1, iy1 = max(0, int(np.ceil(x1 / G - 1e-9))), max(0, int(np.ceil(y1 / G - 1e-9)))
        ix2, iy2 = min(self.nx - 1, int(np.floor(x2 / G + 1e-9))), min(self.ny - 1, int(np.floor(y2 / G + 1e-9)))
        return ix1, iy1, ix2, iy2

    def _halo_set(self, h, mask, code):
        h[mask & (h != 0) & (h != code)] = -1
        h[mask & (h == 0)] = code

    def mark_rect(self, layers, x1, y1, x2, y2, code):
        """Copper rectangle (mm) of a net (or EDGE) with its halos."""
        for l in layers:
            ix1, iy1, ix2, iy2 = self._span(x1 - 0.05, y1 - 0.05, x2 + 0.05, y2 + 0.05)
            if ix2 >= ix1 and iy2 >= iy1:
                o = self.owner[l, iy1:iy2 + 1, ix1:ix2 + 1]
                if code == EDGE:
                    o[o == 0] = EDGE
                else:
                    o[(o == 0) | (o == EDGE)] = code
            for h, w in ((self.halo_s, self.sig_w), (self.halo_p, self.pwr_w)):
                r = CLR + w / 2
                ix1, iy1, ix2, iy2 = self._span(x1 - r, y1 - r, x2 + r, y2 + r)
                if ix2 >= ix1 and iy2 >= iy1:
                    sub = h[l, iy1:iy2 + 1, ix1:ix2 + 1]
                    if code == EDGE:
                        sub[:] = -1
                    else:
                        self._halo_set(sub, np.ones(sub.shape, dtype=bool), code)

    def mark_halo(self, layers, x1, y1, x2, y2, code):
        """Only the halos of a rectangle (no copper)."""
        for l in layers:
            for h, w in ((self.halo_s, self.sig_w), (self.halo_p, self.pwr_w)):
                r = CLR + w / 2
                ix1, iy1, ix2, iy2 = self._span(x1 - r, y1 - r, x2 + r, y2 + r)
                if ix2 >= ix1 and iy2 >= iy1:
                    sub = h[l, iy1:iy2 + 1, ix1:ix2 + 1]
                    self._halo_set(sub, np.ones(sub.shape, dtype=bool), code)

    def mark_segment(self, layers, x1, y1, x2, y2, w, code):
        """A track (or a via when x1,y1 == x2,y2): body within w/2, halos beyond."""
        r_body = w / 2 + 0.02
        r_max = w / 2 + CLR + self.pwr_w / 2
        ix1, iy1, ix2, iy2 = self._span(min(x1, x2) - r_max, min(y1, y2) - r_max,
                                        max(x1, x2) + r_max, max(y1, y2) + r_max)
        if ix2 < ix1 or iy2 < iy1:
            return
        X, Y = np.meshgrid(np.arange(ix1, ix2 + 1) * G, np.arange(iy1, iy2 + 1) * G)
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = np.zeros_like(X) if L2 == 0 else np.clip(((X - x1) * dx + (Y - y1) * dy) / L2, 0, 1)
        dist = np.hypot(X - (x1 + t * dx), Y - (y1 + t * dy))
        for l in layers:
            o = self.owner[l, iy1:iy2 + 1, ix1:ix2 + 1]
            o[(dist <= r_body) & ((o == 0) | (o == EDGE))] = code
            for h, hw in ((self.halo_s, self.sig_w), (self.halo_p, self.pwr_w)):
                self._halo_set(h[l, iy1:iy2 + 1, ix1:ix2 + 1], dist <= w / 2 + CLR + hw / 2, code)

    def layers_of(self, item):
        ls = []
        if item.IsOnLayer(F_Cu):
            ls.append(0)
        if item.IsOnLayer(B_Cu):
            ls.append(1)
        return ls

    def load_board(self, regions):
        """Obstacles from the board. regions: [(x1, y1, x2, y2)] where routing is allowed."""
        allowed = np.zeros((self.ny, self.nx), dtype=bool)
        for (x1, y1, x2, y2) in regions:
            ix1, iy1, ix2, iy2 = self._span(x1, y1, x2, y2)
            allowed[iy1:iy2 + 1, ix1:ix2 + 1] = True
        m = 0.8
        ix1, iy1, ix2, iy2 = self._span(m, m, self.W - m, self.H - m)
        inside = np.zeros_like(allowed)
        inside[iy1:iy2 + 1, ix1:ix2 + 1] = True
        self.owner[:, ~(allowed & inside)] = EDGE
        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                bb = pad.GetBoundingBox()
                x1, y1, x2, y2 = bb.GetLeft() / 1e6, bb.GetTop() / 1e6, bb.GetRight() / 1e6, bb.GetBottom() / 1e6
                code = pad.GetNetCode()
                ls = self.layers_of(pad)
                if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                    ls = [0, 1]
                self.mark_rect(ls, x1, y1, x2, y2, code if code > 0 else EDGE)
                # keep other nets out of the exits of long, narrow pads (SOIC, QFN): a halo
                # 1 mm beyond both ends of the pad, as wide as the pad
                w, h = x2 - x1, y2 - y1
                if code > 0 and pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and max(w, h) > 2.2 * min(w, h):
                    ext = 1.0
                    if h > w:
                        self.mark_halo(ls, x1, y1 - ext, x2, y2 + ext, code)
                    else:
                        self.mark_halo(ls, x1 - ext, y1, x2 + ext, y2, code)
        for t in self.board.GetTracks():
            code = t.GetNetCode()
            if isinstance(t, pcbnew.PCB_VIA):
                p = t.GetPosition()
                self.mark_segment([0, 1], p.x / 1e6, p.y / 1e6, p.x / 1e6, p.y / 1e6, t.GetWidth() / 1e6, code)
            else:
                a, b = t.GetStart(), t.GetEnd()
                self.mark_segment(self.layers_of(t), a.x / 1e6, a.y / 1e6, b.x / 1e6, b.y / 1e6, t.GetWidth() / 1e6, code)
        self.static = (self.owner.copy(), self.halo_s.copy(), self.halo_p.copy())
        self.code2name = {ni.GetNetCode(): name for name, ni in self.nets.items()}
        # pad boxes per net, to finish a path at the pad centre (a grid cell in the corner
        # of the box can lie outside the copper of a round or rounded pad)
        self.padboxes = {}
        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                code = pad.GetNetCode()
                if code <= 0:
                    continue
                bb = pad.GetBoundingBox()
                c = pad.GetPosition()
                ls = [0, 1] if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else self.layers_of(pad)
                self.padboxes.setdefault(code, []).append(
                    (bb.GetLeft() / 1e6, bb.GetTop() / 1e6, bb.GetRight() / 1e6, bb.GetBottom() / 1e6,
                     c.x / 1e6, c.y / 1e6, ls))

    # ------------------------------------------------------------------ search helpers
    def passable(self, code, l, iy, ix, halo):
        o = self.owner[l, iy, ix]
        if o == code:
            return True
        if o != 0:
            return False
        h = halo[l, iy, ix]
        return h == 0 or h == code

    def via_ok(self, code, iy, ix):
        for dy, dx in VIA_CHECK:
            yy, xx = iy + dy, ix + dx
            if yy < 0 or xx < 0 or yy >= self.ny or xx >= self.nx:
                return False
            for l in range(2):
                o = self.owner[l, yy, xx]
                if o != 0 and o != code:
                    return False
                h = self.halo_s[l, yy, xx]
                if h != 0 and h != code:
                    return False
        return True

    def taxicab(self, mask):
        """Exact taxicab distance (in cells) to the nearest True cell of mask [ny, nx]."""
        INF = 10 ** 7
        d = np.where(mask, 0, INF).astype(np.int32)
        for i in range(1, self.nx):
            np.minimum(d[:, i], d[:, i - 1] + 1, out=d[:, i])
        for i in range(self.nx - 2, -1, -1):
            np.minimum(d[:, i], d[:, i + 1] + 1, out=d[:, i])
        for j in range(1, self.ny):
            np.minimum(d[j, :], d[j - 1, :] + 1, out=d[j, :])
        for j in range(self.ny - 2, -1, -1):
            np.minimum(d[j, :], d[j + 1, :] + 1, out=d[j, :])
        return d

    def flood(self, l, iy, ix, code, cap=60000):
        """Cells connected to (l, iy, ix) through copper of this net (vias join the layers).
        Diagonal neighbours count: a thin track laid at 45 degrees marks a diagonal chain of
        cells, which must not look like separate copper of the net (the router would then
        add stubs to join it)."""
        if self.owner[l, iy, ix] != code:
            best = None
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    yy, xx = iy + dy, ix + dx
                    if 0 <= yy < self.ny and 0 <= xx < self.nx and self.owner[l, yy, xx] == code:
                        d = abs(dx) + abs(dy)
                        if best is None or d < best[0]:
                            best = (d, yy, xx)
            if best is None:
                return []
            iy, ix = best[1], best[2]
        seen = {(l, iy, ix)}
        stack = [(l, iy, ix)]
        while stack and len(seen) < cap:
            ll, y, x = stack.pop()
            nb = [(ll, y + dy, x + dx) for dy, dx in DIRS8] + [(1 - ll, y, x)]
            for c in nb:
                cl, yy, xx = c
                if 0 <= yy < self.ny and 0 <= xx < self.nx and c not in seen and self.owner[cl, yy, xx] == code:
                    seen.add(c)
                    stack.append(c)
        return list(seen)

    def astar(self, code, starts, target, halo, limit=400000, weight=1.3):
        """starts: [(l, iy, ix)]; target: bool array [2, ny, nx]. Returns [(l, iy, ix)] or None.
        weight > 1 trades the shortest path for a faster search."""
        dist = self.taxicab(target[0] | target[1])
        openq = []
        best = {}
        parent = {}
        for s in starts:
            key = s + (4,)
            best[key] = 0.0
            parent[key] = None
            heapq.heappush(openq, (weight * float(dist[s[1], s[2]]), 0.0, key))
        closed = set()
        n = 0
        while openq:
            f, g, key = heapq.heappop(openq)
            if key in closed:
                continue
            closed.add(key)
            n += 1
            if n > limit:
                return None
            l, iy, ix, d = key
            if target[l, iy, ix]:
                path = []
                k = key
                while k is not None:
                    path.append(k[:3])
                    k = parent[k]
                path.reverse()
                return path
            for di, (ndx, ndy) in enumerate(DIRS):
                nx_, ny_ = ix + ndx, iy + ndy
                if nx_ < 0 or ny_ < 0 or nx_ >= self.nx or ny_ >= self.ny:
                    continue
                if not self.passable(code, l, ny_, nx_, halo):
                    continue
                step = 1.0
                if (l == 0 and ndx != 0) or (l == 1 and ndy != 0):
                    step += OFF_PREF
                if d != 4 and d != di:
                    step += TURN_COST
                ng = g + step
                nkey = (l, ny_, nx_, di)
                if nkey in closed:
                    continue
                if ng < best.get(nkey, 1e18):
                    best[nkey] = ng
                    parent[nkey] = key
                    heapq.heappush(openq, (ng + weight * dist[ny_, nx_], ng, nkey))
            ol = 1 - l
            if self.via_ok(code, iy, ix):
                ng = g + VIA_COST
                nkey = (ol, iy, ix, 4)
                if nkey not in closed and ng < best.get(nkey, 1e18):
                    best[nkey] = ng
                    parent[nkey] = key
                    heapq.heappush(openq, (ng + weight * dist[iy, ix], ng, nkey))
        return None

    # ------------------------------------------------------------------ building
    def lay_path(self, path, netname, code, width, pad_from=None):
        """Turn a cell path into tracks and vias on the board (and into obstacles)."""
        pts = [(l, ix * G, iy * G) for (l, iy, ix) in path]
        layers = [F_Cu, B_Cu]
        for (l, x, y) in (pts[0], pts[-1]):
            self.snap(l, x, y, netname, code, width)
        i = 0
        while i < len(pts) - 1:
            l, x, y = pts[i]
            j = i + 1
            if pts[j][0] != l:
                self._add(netname, self.via(x, y, netname), [0, 1], x, y, x, y, 2 * VIA_R, code)
                i = j
                continue
            dx, dy = pts[j][1] - x, pts[j][2] - y
            while j + 1 < len(pts) and pts[j + 1][0] == l and \
                    abs((pts[j + 1][1] - pts[j][1]) - dx) < 1e-9 and abs((pts[j + 1][2] - pts[j][2]) - dy) < 1e-9:
                j += 1
            x2, y2 = pts[j][1], pts[j][2]
            self._add(netname, self.track(x, y, x2, y2, width, layers[l], netname), [l], x, y, x2, y2, width, code)
            i = j

    def snap(self, l, x, y, netname, code, width):
        """A stub from a path end that lies in a pad of the net to that pad's centre."""
        for (x1, y1, x2, y2, cx, cy, ls) in self.padboxes.get(code, ()):
            if l in ls and x1 - 0.1 <= x <= x2 + 0.1 and y1 - 0.1 <= y <= y2 + 0.1:
                if abs(cx - x) > 1e-6 or abs(cy - y) > 1e-6:
                    self._add(netname, self.track(x, y, cx, cy, width, [F_Cu, B_Cu][l], netname),
                              [l], x, y, cx, cy, width, code)
                return

    def _add(self, netname, item, layers, x1, y1, x2, y2, w, code):
        self.items.setdefault(netname, []).append(item)
        self.segs.setdefault(netname, []).append((layers, x1, y1, x2, y2, w, code))
        self.mark_segment(layers, x1, y1, x2, y2, w, code)

    def rip(self, netname):
        """Remove everything the router laid for a net and rebuild the obstacle arrays."""
        for it in self.items.pop(netname, []):
            self.board.Remove(it)
        self.segs.pop(netname, None)
        self.rips[netname] = self.rips.get(netname, 0) + 1
        self.owner[:], self.halo_s[:], self.halo_p[:] = self.static
        for segs in self.segs.values():
            for (layers, x1, y1, x2, y2, w, code) in segs:
                self.mark_segment(layers, x1, y1, x2, y2, w, code)

    def reach(self, code, seeds, halo, cap=300000):
        """Cells a track of this net could reach from the seeds (vias included)."""
        seen = set(seeds)
        q = deque(seeds)
        while q and len(seen) < cap:
            l, iy, ix = q.popleft()
            for dy, dx in DIRS:
                c = (l, iy + dy, ix + dx)
                if 0 <= c[1] < self.ny and 0 <= c[2] < self.nx and c not in seen and self.passable(code, l, c[1], c[2], halo):
                    seen.add(c)
                    q.append(c)
            if self.via_ok(code, iy, ix):
                c = (1 - l, iy, ix)
                if c not in seen:
                    seen.add(c)
                    q.append(c)
        return seen

    def blockers(self, code, seen, halo):
        """Routed nets whose copper or clearance bounds a reachable set: {name: cells}."""
        count = {}
        for (l, iy, ix) in seen:
            for (ll, yy, xx) in [(l, iy + dy, ix + dx) for dy, dx in DIRS] + [(1 - l, iy, ix)]:
                if not (0 <= yy < self.ny and 0 <= xx < self.nx):
                    continue
                o = self.owner[ll, yy, xx]
                c = o if (o > 0 and o != code) else 0
                if c == 0 and o == 0:
                    h = halo[ll, yy, xx]
                    c = h if (h > 0 and h != code) else 0
                if c:
                    name = self.code2name.get(int(c))
                    if name in self.segs:
                        count[name] = count.get(name, 0) + 1
        return count

    def route_net(self, netname, pads, width):
        """pads: [(x, y, tht)] to connect to the copper the net already has."""
        code = self.nets[netname].GetNetCode()
        halo = self.halo_p if width > self.sig_w else self.halo_s
        for (x, y, tht) in pads:
            ix, iy = self.cell(x, y)
            layers = [0, 1] if tht else [0]
            start = []
            for l in layers:
                start += self.flood(l, iy, ix, code)
            if not start:
                self.failed.append((netname, x, y, "no start"))
                continue
            start = list(set(start))
            target = (self.owner == code)
            for (l, yy, xx) in start:
                target[l, yy, xx] = False
            if not target.any():
                continue                      # first copper of the net (or already joined)
            path = self.astar(code, start, target, halo)
            tries = 0
            while path is None and tries < 4:
                tries += 1
                S = self.reach(code, start, halo)
                tgt = [tuple(int(v) for v in c) for c in np.argwhere(target)]
                if any(c in S for c in tgt):
                    print(f"    {netname}: target reachable, searching harder", flush=True)
                    path = self.astar(code, start, target, halo, limit=1500000, weight=2.0)
                    break
                T = self.reach(code, tgt, halo)
                side = S if len(S) <= len(T) else T
                cnt = self.blockers(code, side, halo)
                # ground and the short 74HC595 -> ULN2803 lines are never moved, and the
                # ground net itself moves nothing (it is routed last, as a tidy-up)
                rippable = [] if netname == "GND" else sorted(
                    ((n, nm) for nm, n in cnt.items()
                     if nm != netname and nm in self.pads_of and self.rips.get(nm, 0) < 3
                     and not nm.startswith("DRV") and nm != "GND"),
                    reverse=True)[:3]
                if not rippable:
                    print(f"    {netname}: boxed in by {sorted(cnt)} (nothing left to rip up)", flush=True)
                    break
                print(f"    {netname}: ripping up {[nm for _, nm in rippable]}", flush=True)
                for _, nm in rippable:
                    self.rip(nm)
                    if nm not in self.requeue:
                        self.requeue.append(nm)
                target = (self.owner == code)
                for (l, yy, xx) in start:
                    target[l, yy, xx] = False
                path = self.astar(code, start, target, halo)
            if path is None:
                self.failed.append((netname, x, y, "no path"))
                self.diagnose(code, start, target, halo)
                continue
            self.lay_path(path, netname, code, width, pad_from=(x, y))

    def diagnose(self, code, start, target, halo):
        """Print how far a failed search could get (reachable area, nearest target)."""
        seen = self.reach(code, start, halo)
        arr = np.array(list(seen))
        hit = sum(1 for c in np.argwhere(target) if (c[0], c[1], c[2]) in seen)
        print(f"    reachable {len(seen)} cells, x {arr[:, 2].min() * G:.1f}-{arr[:, 2].max() * G:.1f} "
              f"y {arr[:, 1].min() * G:.1f}-{arr[:, 1].max() * G:.1f}, target cells reachable: {hit}", flush=True)

    def ground_via(self, x, y, netname="GND", ok=None):
        """A via into the bottom ground pour next to a top-side ground pad. ok(x, y): an
        extra condition on the via spot (e.g. that the pour there is the main one)."""
        code = self.nets[netname].GetNetCode()
        ix, iy = self.cell(x, y)
        start = [c for c in self.flood(0, iy, ix, code, cap=400) if c[0] == 0]
        if not start:
            self.failed.append((netname, x, y, "no pad"))
            return
        startset = set((c[1], c[2]) for c in start)
        q = deque()
        prev = {}
        for (yy, xx) in startset:
            q.append((yy, xx))
            prev[(yy, xx)] = None
        found = None
        n = 0
        while q and n < 20000:
            yy, xx = q.popleft()
            n += 1
            if (yy, xx) not in startset and self.via_ok(code, yy, xx) and (ok is None or ok(xx * G, yy * G)):
                found = (yy, xx)
                break
            for dy, dx in DIRS:
                y2, x2 = yy + dy, xx + dx
                if 0 <= y2 < self.ny and 0 <= x2 < self.nx and (y2, x2) not in prev \
                        and self.passable(code, 0, y2, x2, self.halo_s):
                    prev[(y2, x2)] = (yy, xx)
                    q.append((y2, x2))
        in_pad = False
        if found is None and ok is None:               # last resort: a via in the pad itself
            for c in sorted(startset, key=lambda c: abs(c[0] * G - y) + abs(c[1] * G - x)):
                if self.via_ok(code, c[0], c[1]):
                    found, in_pad = c, True
                    break
        if found is None:
            self.failed.append((netname, x, y, f"no via spot ({n} cells searched)"))
            return
        path = []
        k = found
        while k is not None and not in_pad:
            path.append((0, k[0], k[1]))
            k = prev[k]
        path.reverse()
        if in_pad:
            path = [(0, found[0], found[1])]
        path.append((1, found[0], found[1]))          # the via at the end
        self.lay_path(path, netname, code, self.sig_w, pad_from=(x, y))

    # ------------------------------------------------------------------ driver
    def run(self, regions, todo, power_nets, gnd_pads, rippable=None):
        """todo: [(net, [(x, y, tht)])] in routing order; gnd_pads: [(x, y)] top-side pads;
        rippable: {net: pads} of nets already on the board that may be moved out of the way."""
        import time
        self.power_nets = set(power_nets)
        self.load_board(regions)
        for (x, y) in gnd_pads:                 # first, while there is room next to the pads
            self.ground_via(x, y)
        pads_of = dict(todo)
        if rippable:
            pads_of.update(rippable)
        self.pads_of = pads_of                  # only these nets can be ripped up (not the ground vias)
        # the ground net goes last, once every signal net (ripped up ones included) is
        # done: its many short stubs must not box in a net that is routed again later
        queue = deque((n, p) for n, p in todo if n != "GND")
        last = [(n, p) for n, p in todo if n == "GND"]
        while queue or last:
            if not queue:
                queue.extend(last)
                last = []
            net, pads = queue.popleft()
            t = time.time()
            w = self.pwr_w if net in power_nets else self.sig_w
            self.route_net(net, pads, w)
            print(f"  routed {net:12s} {len(pads)} pads {time.time() - t:5.1f} s", flush=True)
            while self.requeue:
                nm = self.requeue.pop(0)
                self.failed = [f for f in self.failed if f[0] != nm]
                queue.append((nm, pads_of[nm]))
        for f in self.failed:
            print("ROUTER FAILED:", f, "(ground: the pour decides, see the DRC)" if f[0] == "GND" else "")
        return self.failed
