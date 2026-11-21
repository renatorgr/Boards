/* user_careful.cpp - user 1: everything set up right. The module must just work.
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

// The sprayer: 8 sections of 3 m, 6 nozzles each (0.5 m), 1.6 L/min at 3 bar, pump at 3 bar; AgOpenWeb's pins 1-8 =
// sections 1-8; on the page the nozzle chart, 240 L/ha (what 3 bar gives at 8 km/h), 1-6 bar, flow +-20 %.
static Module& carefulSprayer(World& w, Board board)
{
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.toolSections = 8;
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    Http r = m.http(HTTP_POST, "/api/spray", { { "pw", "agopengps" }, { "rate", "240" }, { "nzSp", "0.5" }, { "nzFl", "1.6" },
                                               { "nzBar", "3" }, { "pLow", "1" }, { "pHigh", "6" }, { "fTol", "20" } });
    must("spray-settings-saved", r.code == 200, "the page saves the spraying settings", fmt("answer %d %s", r.code, r.body.c_str()));
    w.run(1000);
    return m;
}

SCENARIO(User::Careful, "careful-sprayer", "sets up the sprayer module right and sprays a field", sprayerBoards())
{
    World w;
    Module& m = carefulSprayer(w, board);
    const std::vector<int> rs = inARow(m.bi.relays, 1, 8);
    std::string bad;

    must("joined", m.dev.staUp() && m.dev.tractorIp() == IPAddress(192, 168, 5, 123), "joins the tractor's Wi-Fi as 192.168.5.123",
         m.dev.tractorIp().toString().str());

    int n = driveAndCompare(w, m, first(8), rs, 20000, &bad);
    must("relays-follow-all", n == 0, "all 8 relays open with all 8 sections for 20 s", bad);

    // the rate, the pressure the target needs and no alarm while everything is as set
    Json s = m.status();
    const double lha = s["rate"]["lha"].num(), need = s["rate"]["needBar"].num();
    must("rate-right", fabs(lha - 240) < 12, "the L/ha is the real one (240 L/ha = 76.8 L/min on 24 m at 8 km/h)", fmt("module says %.1f L/ha", lha));
    must("need-bar-right", fabs(need - 3.0) < 0.15, "the pressure for the target is 3.0 bar", fmt("module says %.2f bar", need));
    must("no-alarm", s["rate"]["alarms"].num() == 0, "no alarm while everything sprays as set", fmt("alarms %g", s["rate"]["alarms"].num()));
    note("rate", fmt("%.1f L/ha, %.2f L/min measured, needs %.2f bar", lha, s["spr"]["lpm"].num(), need));

    // sections on and off as on a field
    const uint64_t patterns[] = { 0x0F, 0xF0, 0xAA, 0x55, 0x01, 0x80, 0x00, 0xFF, 0x3C };
    int wrong = 0;
    for (uint64_t p : patterns) wrong += driveAndCompare(w, m, p, rs, 3000, wrong ? nullptr : &bad);
    must("relays-follow-patterns", wrong == 0, "the relays follow the sections as they change", bad);

    // what the tablet hears: 237 five times a second, 160 + 161 once a second, a hello reply per hello
    const uint64_t from = w.now();
    w.run(10000);
    const size_t n237 = w.aow.heardPgn(237, from).size(), n160 = w.aow.heardPgn(160, from).size();
    const size_t n161 = w.aow.heardPgn(161, from).size(), n123 = w.aow.heardPgn(123, from).size();
    must("sends-237", n237 >= 48 && n237 <= 52, "AgOpenWeb hears PGN 237 five times a second", fmt("%zu in 10 s", n237));
    must("sends-160-161", n160 >= 9 && n161 >= 9, "AgOpenWeb hears the readings (160) and process data (161) every second",
         fmt("%zu x 160, %zu x 161 in 10 s", n160, n161));
    must("hello-reply", n123 >= 9, "the module answers AgOpenWeb's hello", fmt("%zu replies in 10 s", n123));
    must("no-alarm-message", screenMessages(w, 0).empty(), "no alarm text on AgOpenWeb's screen", screenMessages(w, 0));

    // the tablet stops: everything closes within 2 s
    w.aow.on = false;
    const int off = msUntilAllOff(w, m, 3000);
    must("silent-all-off", off >= 0 && off <= 2100, "every section closes within 2 s when AgOpenWeb goes silent", off < 0 ? "still on after 3 s" : fmt("after %d ms", off));
    w.run(5000);
    must("silent-stays-off", m.relays() == 0, "and stays closed", hex(m.relays()));

    // a power cycle keeps everything; back to work without setting anything again
    m.powerOff();
    w.run(1000);
    m.powerOn();
    w.aow.on = true;
    w.run(3000);
    n = driveAndCompare(w, m, first(8), rs, 3000, &bad);
    must("after-power-cycle", n == 0, "after a power cycle the relays follow at once (settings kept)", bad);
    s = m.status();
    must("target-kept", s["rate"]["target"].num() == 240 && s["tool"]["n"].num() == 8, "the target and the tool's widths survive the power cycle",
         fmt("target %g, tool sections %g", s["rate"]["target"].num(), s["tool"]["n"].num()));
    statusAlwaysJson(m);
}

SCENARIO(User::Careful, "careful-seeder", "sets up the seed drill module (19 sections, cable) and sows a field", seederBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.toolSections = 19;
    w.aow.startup();
    Http r = m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "1" }, { "secs", "19" } });
    must("relays-saved", r.code == 200, "the page saves 'relay 1 = section 1, 19 sections'", fmt("answer %d %s", r.code, r.body.c_str()));
    must("cable-up", m.dev.onTractorNet() && m.dev.tractorIp() == IPAddress(192, 168, 5, 123), "on the tractor network by its cable as 192.168.5.123",
         m.dev.tractorIp().toString().str());

    const std::vector<int> rs = inARow(m.bi.relays, 1, 19);
    std::string bad;
    const uint64_t patterns[] = { first(19), first(16), first(19) & ~first(16), 0x55555, 0x2AAAA, 0x40001, 0, first(19) };
    int wrong = 0;
    for (uint64_t p : patterns) wrong += driveAndCompare(w, m, p, rs, 3000, wrong ? nullptr : &bad);
    must("relays-follow", wrong == 0, "relays 1-19 follow sections 1-19, relays 20+ never close", bad);

    // 237 with the open sections above 16 (bytes 13-15)
    const uint64_t from = w.now();
    w.aow.sections = first(19);
    w.run(2000);
    auto h237 = w.aow.heardPgn(237, from);
    bool all19 = !h237.empty();
    for (auto& hd : h237)
    {
        if (hd.data.size() < 16) { all19 = false; break; }
        const uint32_t s = hd.data[5] | hd.data[6] << 8 | hd.data[13] << 16;
        if (hd.ms > from + 300 && s != first(19)) all19 = false;
    }
    must("237-all-sections", all19, "PGN 237 tells AgOpenWeb sections 1-19 are open", fmt("%zu messages", h237.size()));

    w.aow.on = false;
    const int off = msUntilAllOff(w, m, 3000);
    must("silent-all-off", off >= 0 && off <= 2100, "every section closes within 2 s when AgOpenWeb goes silent",
         off < 0 ? "still on after 3 s" : fmt("after %d ms", off));

    w.aow.on = true;
    w.run(1000);
    driveAndCompare(w, m, first(19), rs, 1000);
    m.dev.cableIn = false;                      // the plug comes out
    const int unplug = msUntilAllOff(w, m, 3000);
    must("unplugged-all-off", unplug >= 0 && unplug <= 2100, "every section closes within 2 s when the cable comes out",
         unplug < 0 ? "still on after 3 s" : fmt("after %d ms", unplug));
    m.dev.cableIn = true;
    w.run(1000);

    m.powerOff();
    w.run(1000);
    m.powerOn();
    w.run(1500);
    const int n = driveAndCompare(w, m, first(19), rs, 3000, &bad);
    must("after-power-cycle", n == 0, "after a power cycle the relays follow at once (settings kept)", bad);
    Json s = m.status();
    must("secs-kept", s["out"]["first"].num() == 1 && s["out"]["secs"].num() == 19, "first section and section count survive the power cycle",
         fmt("first %g, secs %g", s["out"]["first"].num(), s["out"]["secs"].num()));
    const std::string said = pageLog(m) + screenMessages(w, 0);
    must("no-false-no-relay", !contains(said, "no relay"), "no 'no relay' warning on a drill set up right", said);
    statusAlwaysJson(m);
}
