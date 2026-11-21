/* user_careful.cpp - the careful user: everything fitted and set up right, AgOpenWeb steers.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"

using namespace bench;
using namespace chk;

SCENARIO(User::Careful, "careful-day", "Starts, AgOpenWeb engages autosteer and follows a line for 40 s, then disengages", allKits())
{
    World w(kit);
    w.startUp();
    const Json st = w.status();
    must("boots", w.aio.boots == 1, "Starts once, no restart", fmt("%u starts", w.aio.boots));
    must("ip", st["ip"].str() == "192.168.5.126", "IP 192.168.5.126 (steer module)", st["ip"].str());
    must("hello", !w.aow.heardPgn(126).empty(), "Answers AgOpenWeb's hello (PGN 126)");
    must("nmea", w.aow.nmeaGga >= 30, "GGA to AgOpenWeb every epoch", fmt("%u GGA in 8 s", w.aow.nmeaGga));
    must("nmea-sum", w.aow.nmeaBadSum == 0, "Every NMEA line's checksum is right", fmt("%u bad of %u", w.aow.nmeaBadSum, w.aow.nmeaLines));
    must("got-settings", st["steerCfg"]["got252"].num() >= 0 && st["steerCfg"]["counts"].num() == w.aow.ss.counts,
         "Takes AgOpenWeb's steer settings", fmt("counts %g", st["steerCfg"]["counts"].num()));
    note("set-up", fmt("setup + 8 s: %zu log bytes, IMU %s, receiver %s", w.aio.log.size(), st["tm"]["type"].str().c_str(), st["gnss"]["model"].str().c_str()));

    // drive along, 1 m left of the line, then engage
    w.tractor.kmh = 8;
    w.tractor.x = -1.0f;
    w.run(2000);
    engage(w, true);
    w.aio.resetLoopMax();
    w.run(40000);
    const float xte = fabsf(w.tractor.x);
    must("follows-line", xte < 0.10f, "Follows the line (within 10 cm after 40 s)", fmt("%.2f m off the line, wheels %.1f deg", xte, w.tractor.wheelDeg));
    must("wheels-follow", w.maxAbsErrDeg < 3.0f, "Wheels within 3 deg of the asked angle while steering", fmt("worst %.1f deg", w.maxAbsErrDeg));
    note("loop", fmt("longest loop() while steering: %.1f ms", w.aio.loopMaxUs / 1000.0));
    must("loop-time", w.aio.loopMaxUs < 50000, "No loop() pass longer than 50 ms while steering (two steering periods)",
         fmt("%.1f ms", w.aio.loopMaxUs / 1000.0));
    const auto s253 = w.aow.heardPgn(253, w.now() - 10000);
    must("pgn253", s253.size() >= 90, "PGN 253 (wheel angle, switches) back for every steering packet", fmt("%zu in 10 s", s253.size()));
    const uint32_t gga0 = w.aow.nmeaGga;
    w.run(10000);
    must("gga-10hz", w.aow.nmeaGga - gga0 >= 97, "The receiver's 10 Hz reach AgOpenWeb (GGA 10 times a second)", fmt("%u GGA in 10 s", w.aow.nmeaGga - gga0));

    engage(w, false);
    w.expectOff("AgOpenWeb disengaged", 200);   // its next steering packet (100 ms) and a steering loop
    w.run(2000);
    must("off", !w.engaged(), "Outputs off after AgOpenWeb disengages");
    w.expectOffEnd();
    w.status();
    reportRules(w);
}

SCENARIO(User::Careful, "careful-silent-program", "AgOpenWeb stops talking while steering (tablet frozen): steering off within 2.5 s", allKits())
{
    World w(kit);
    w.startUp();
    w.tractor.kmh = 8;
    w.run(1000);
    engage(w, true);
    w.run(5000);
    must("steering", w.engaged(), "Steering before the program goes silent");
    w.aow.on = false;                              // frozen: nothing more is sent
    const uint64_t t0 = w.now();
    w.runUntil([&] { return !w.engaged(); }, 5000);
    const uint64_t off = w.now() - t0;
    note("silent-off", fmt("outputs off %llu ms after the last packet", (unsigned long long)off));
    must("off-in-time", off <= 2600, "Outputs off within 2.6 s of the last steering packet", fmt("%llu ms", (unsigned long long)off));
    w.run(3000);
    must("stays-off", !w.engaged(), "Stays off while the program is silent");
    reportRules(w);
}

SCENARIO(User::Careful, "careful-restart-page", "Restarts the module from its web page with autosteer off, AgOpenWeb running", allKits())
{
    World w(kit);
    w.startUp();
    w.tractor.kmh = 0;
    const size_t mark = w.aio.logMark();
    const Http h = w.http("POST", "/api/reboot", "");
    w.run(9000);
    must("restarted", w.aio.boots == 2, "The module restarts once", fmt("%u starts", w.aio.boots));
    must("reason", w.aio.logHas("restart by software", mark), "Says why it started (restart by software)", w.aio.logLine("[boot] Start", mark));
    must("answer", h.code == 200, "The page gets its answer before the restart", fmt("HTTP %d", h.code));
    must("back", !w.aow.heardPgn(126, w.now() - 3000).empty(), "Back on the network, answering AgOpenWeb");
    reportRules(w);
}

SCENARIO(User::Careful, "careful-pages", "Opens every page of the module's web site while AgOpenWeb steers", { Kit::PumaCytron, Kit::Um982Keya })
{
    World w(kit);
    w.startUp();
    w.tractor.kmh = 8;
    engage(w, true);
    w.run(3000);
    w.aio.resetLoopMax();
    const char* pages[] = { "/", "/api/status", "/api/log", "/api/settings", "/api/diag", "/api/gps", "/api/um982", "/api/note" };
    for (const char* p : pages)
    {
        const Http h = w.http("GET", p);
        must(std::string("page") + p, h.code == 200 || (h.code == 304 && !strcmp(p, "/")), std::string("GET ") + p + " answers",
             fmt("HTTP %d after %u ms, %zu bytes", h.code, h.ms, h.body.size()));
        if (h.body.size() && h.body[0] == '{') must(std::string("json") + p, h.jsonOk, std::string("GET ") + p + " is valid JSON", h.body.substr(0, 200));
    }
    note("loop-pages", fmt("longest loop() while the pages were read: %.1f ms", w.aio.loopMaxUs / 1000.0));
    must("loop-pages", w.aio.loopMaxUs < 50000, "The pages do not hold the steering loop (no pass over 50 ms)", fmt("%.1f ms", w.aio.loopMaxUs / 1000.0));
    must("steering-on", w.engaged(), "Still steering after the pages");
    reportRules(w);
}
