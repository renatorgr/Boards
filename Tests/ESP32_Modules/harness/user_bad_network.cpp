/* user_bad_network.cpp - user 5: everything set up, but the network misbehaves (router off, garbage, floods, a
   module left on for 49 days).
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
using namespace h;
using chk::fmt;
using chk::must;
using chk::note;
using chk::User;

// A message straight into the module's network socket (from anyone on the network)
static void inject(Module& m, const std::vector<uint8_t>& bytes, IPAddress from = IPAddress(192, 168, 5, 66))
{
    fake::Packet p;
    p.ms = fake::clockMs;
    p.src = from;
    p.srcPort = 9999;
    p.dst = IPAddress(192, 168, 5, 255);
    p.dstPort = 8888;
    p.data = bytes;
    m.dev.rx.push_back(p);
}

struct Rng
{
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    uint32_t below(uint32_t n) { return next() % n; }
};

SCENARIO(User::BadNetwork, "network-drops", "the tractor network goes down while working (router off / switch off), then comes back", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.startup();
    const std::vector<int> rs = inARow(m.bi.relays, 1, 8);
    driveAndCompare(w, m, first(8), rs, 2000);
    if (m.bi.sprayer) fake::net.wifiOn = false;
    else m.dev.cableIn = false;
    const int off = msUntilAllOff(w, m, 3000);
    must("drop-all-off", off >= 0 && off <= 2100, "every section closes within 2 s of the network going down",
         off < 0 ? "still on after 3 s" : fmt("after %d ms", off));
    w.run(5000);
    if (m.bi.sprayer) fake::net.wifiOn = true;
    else m.dev.cableIn = true;
    w.run(3000);
    std::string bad;
    const int n = driveAndCompare(w, m, first(8), rs, 3000, &bad);
    must("back-works", n == 0, "when the network is back the relays follow again by themselves", bad);
}

SCENARIO(User::BadNetwork, "garbage-packets", "the network carries garbage: random bytes, bad checksums, other senders, cut messages", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.startup();
    w.run(500);
    w.aow.on = false;                                       // only the garbage from now on
    w.run(2500);
    Json before = m.status();
    Rng rng(0xC0FFEE ^ (uint32_t)board);
    const std::vector<uint8_t> pgns = { 239, 229, 238, 236, 235, 201, 202, 200, 237, 160, 1, 255 };
    uint32_t seen = 0;
    // 1. nothing valid: random bytes, wrong checksums, the right message from the wrong sender
    for (int i = 0; i < 6000; i++)
    {
        std::vector<uint8_t> b;
        switch (rng.below(4))
        {
        case 0:
        {
            const uint32_t n = rng.below(400);
            for (uint32_t k = 0; k < n; k++) b.push_back((uint8_t)rng.next());
            break;
        }
        case 1:                                             // a 239 with every section on, from a module (0x7B) or AgIO's 0x7E
            b = pgn(239, { 0, 80, 0, 0, 0, 0, 0xFF, 0xFF }, rng.below(2) ? 0x7B : 0x7E);
            break;
        default:                                            // the program's header, anything after it, a wrong checksum
        {
            const uint8_t number = pgns[rng.below(pgns.size())];
            const uint32_t n = rng.below(64);
            std::vector<uint8_t> d;
            for (uint32_t k = 0; k < n; k++) d.push_back((uint8_t)rng.next());
            b = pgn(number, d);
            b.back() ^= (uint8_t)(1 + rng.below(255));
            if (rng.below(3) == 0) b[4] = (uint8_t)rng.next();          // a length that does not fit
            // the 8-bit checksum matches by chance once in 256: make sure it does not (that would be a real message)
            const size_t dl = b[4];
            if (b.size() >= dl + 6)
            {
                uint8_t sum = 0;
                for (size_t k = 2; k < dl + 5; k++) sum += b[k];
                if (sum == b[dl + 5]) b[dl + 5] ^= 0x55;
            }
            break;
        }
        }
        inject(m, b);
        w.run(1);
        seen |= m.relays();
    }
    Json after = m.status();
    must("garbage-no-relay", seen == 0, "no relay closes on garbage", "relays seen on: " + hex(seen));
    must("garbage-no-restart", m.boots == (m.bi.sprayer ? 2u : 1u), "the module does not restart", fmt("%u starts", m.boots));
    must("garbage-config-kept", after["aog"]["invert"].num() == before["aog"]["invert"].num() &&
                                after["out"]["fn"].size() == before["out"]["fn"].size(),
         "its settings stay as they were");
    bool fnSame = true;
    for (size_t i = 0; i < after["out"]["fn"].size(); i++) if (after["out"]["fn"][i].num() != before["out"]["fn"][i].num()) fnSame = false;
    must("garbage-pins-kept", fnSame, "what each relay does stays as it was");
    note("bad checksums counted", fmt("%g", after["aog"]["crcErr"].num()));

    // 2. right checksum, wrong length: every known message too short, and a length longer than the message
    seen = 0;
    const std::vector<std::pair<uint8_t, int>> need = { { 239, 8 }, { 229, 8 }, { 238, 8 }, { 236, 24 }, { 235, 33 }, { 201, 5 }, { 202, 3 }, { 200, 3 } };
    for (auto [number, len] : need)
        for (int n = 0; n < len + 3; n++)
        {
            // a whole one is a real message (sections, settings, a subnet): only the cut ones here
            if (n >= len && number != 200 && number != 202) continue;
            std::vector<uint8_t> d(n);
            for (auto& x : d) x = (uint8_t)rng.next();
            if (number == 201 && n >= 2) d[0] = d[1] = 201;
            if (number == 202 && n >= 2) d[0] = d[1] = 202;
            std::vector<uint8_t> b = pgn(number, d);
            inject(m, b);
            std::vector<uint8_t> longer = b;
            longer[4] = (uint8_t)(n + 10);                  // says more than it carries
            inject(m, longer);
            w.run(2);
            seen |= m.relays();
        }
    w.run(2000);
    must("short-no-relay", seen == 0, "no relay closes on cut messages", "relays seen on: " + hex(seen));
    must("short-no-restart", m.boots == (m.bi.sprayer ? 2u : 1u), "no restart from a cut subnet message", fmt("%u starts", m.boots));
    statusAlwaysJson(m);
}

SCENARIO(User::BadNetwork, "flood", "a flood of PGN 239 (5000 a second) with random sections", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.startup();
    w.aow.on = false;
    w.run(2500);
    Rng rng(42);
    for (int t = 0; t < 10000; t++)
    {
        for (int k = 0; k < 5; k++)
            inject(m, pgn(239, { 0, 80, 0, 0, 0, 0, (uint8_t)rng.next(), (uint8_t)rng.next() }), IPAddress(192, 168, 5, 10));
        w.run(1);
    }
    const uint64_t sections = 0xA5;
    inject(m, pgn(239, { 0, 80, 0, 0, 0, 0, (uint8_t)sections, 0 }), IPAddress(192, 168, 5, 10));
    w.run(60);
    must("flood-last-wins", m.relays() == expected(inARow(m.bi.relays, 1, 16), sections),
         "after the flood the relays show the last message", fmt("relays %s, want %s", hex(m.relays()).c_str(),
         hex(expected(inARow(m.bi.relays, 1, 16), sections)).c_str()));
    const int off = msUntilAllOff(w, m, 3000);
    must("flood-then-silent", off >= 0 && off <= 2100, "and close 2 s after it stops", off < 0 ? "still on after 3 s" : fmt("after %d ms", off));
}

SCENARIO(User::BadNetwork, "uptime-49-days", "a module left on for 49.7 days (its clock wraps round) while working", allBoards())
{
    World w;
    Module& m = w.add(board);
    m.dev.millisOffset = 0xFFFFFFFFu - 25000u;              // the clock wraps 25 s after each start
    if (!m.bi.sprayer) m.dev.cableIn = true;
    m.powerOn();
    w.run(500);
    if (m.bi.sprayer) joinTractorWifi(w, m);
    w.aow.startup();
    const std::vector<int> rs = inARow(m.bi.relays, 1, 8);
    std::string bad;
    int n = 0;
    for (int i = 0; i < 12; i++) n += driveAndCompare(w, m, i % 2 ? 0x0F : 0xF3, rs, 4000, n ? nullptr : &bad);
    must("wrap-follow", n == 0, "the relays follow the sections across the clock's wrap", bad);
    w.aow.on = false;
    const int off = msUntilAllOff(w, m, 3000);
    must("wrap-silent-off", off >= 0 && off <= 2100, "after the wrap a silent program still closes everything within 2 s",
         off < 0 ? "still on after 3 s" : fmt("after %d ms", off));
    const uint32_t boots = m.boots;
    m.http(HTTP_POST, "/api/restart", { { "pw", "agopengps" } });
    w.run(2000);
    must("wrap-restart", m.boots == boots + 1, "a restart asked from the page still happens after the wrap", fmt("%u -> %u starts", boots, m.boots));
}

SCENARIO(User::BadNetwork, "stale-229", "the program stops sending PGN 229 (sections 17+) but keeps sending 239", seederBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.toolSections = 19;
    w.aow.startup();
    driveAndCompare(w, m, first(19), inARow(m.bi.relays, 1, 19), 2000);
    w.aow.send229 = false;
    int ms = -1;
    for (int t = 0; t < 3000; t++) { w.run(1); if ((m.relays() & ~(uint32_t)0xFFFF) == 0) { ms = t; break; } }
    must("stale-229-off", ms >= 0 && ms <= 2100, "sections 17-19 close within 2 s of the last PGN 229",
         ms < 0 ? "still on after 3 s" : fmt("after %d ms", ms));
    must("stale-229-rest", (m.relays() & 0xFFFF) == 0xFFFF, "sections 1-16 keep working from PGN 239", hex(m.relays()));
}

SCENARIO(User::BadNetwork, "program-restarts", "AgOpenWeb is closed and opened again with another tool (12 sections)", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 3000);
    w.aow.on = false;
    w.run(5000);
    must("closed-while-away", m.relays() == 0, "everything closed while the program is away", hex(m.relays()));
    w.aow.on = true;
    w.aow.toolSections = 12;
    for (int i = 0; i < 12; i++) w.aow.widthCm[i] = 200;
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });
    std::string bad;
    std::vector<int> rs = inARow(m.bi.relays, 1, 12);
    const int n = driveAndCompare(w, m, first(12), rs, 3000, &bad);
    must("new-tool-follows", n == 0, "the relays follow the new tool's pin config", bad);
    Json s = m.status();
    must("new-widths", s["tool"]["n"].num() == 12 && s["tool"]["cm"][0].num() == 200, "the new tool's widths are taken (12 sections of 2 m)",
         fmt("%g sections, first %g cm", s["tool"]["n"].num(), s["tool"]["cm"][0].num()));
}
