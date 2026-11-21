/* user_unconfigured.cpp - user 2: the module is fitted and switched on, but never set up (or only half).
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
using chk::tell;
using chk::User;

SCENARIO(User::Unconfigured, "power-on-quiet", "switches a new module on: no relay may close by itself", allBoards())
{
    World w;
    Module& m = w.add(board);
    if (!m.bi.sprayer) m.dev.cableIn = true;
    w.aow.on = false;
    // sample the relays at every pin the program writes during start-up, then every ms
    uint32_t seen = 0;
    auto decode = m.dev.onWrite;
    m.dev.onWrite = [&](uint8_t pin, uint8_t level) { decode(pin, level); seen |= m.relays(); };
    m.powerOn();
    for (int t = 0; t < 5000; t++) { w.run(1); seen |= m.relays(); }
    must("no-blip", seen == 0, "no relay closes at power-on while nothing asks for a section", "relays seen on: " + hex(seen));
    // and the same after a restart from the page
    m.http(HTTP_POST, "/api/restart", { { "pw", "agopengps" } });
    for (int t = 0; t < 3000; t++) { w.run(1); seen |= m.relays(); }
    must("no-blip-restart", seen == 0, "no relay closes across a restart", "relays seen on: " + hex(seen));
    must("restarted", m.boots == 2, "the restart from the page happens", fmt("%u starts", m.boots));
}

SCENARIO(User::Unconfigured, "sprayer-never-set-up", "fits a new sprayer module and never opens its page", sprayerBoards())
{
    World w;
    Module& m = w.add(board);
    w.addRig(m);
    m.powerOn();
    w.aow.startup();
    int extra = 0;
    for (int i = 0; i < 300; i++) { w.aow.sections = first(8); w.run(100); if (m.relays()) extra++; }
    must("never-sprays", extra == 0, "no relay closes: the module is not on the tractor network, so nothing asks it to", fmt("%d samples with relays on", extra));
    note("AgOpenWeb hears", fmt("%zu messages from the module", w.aow.heard.size()));
    Json s = m.status();
    const std::string log = pageLog(m);
    tell("says-not-on-network", !s["wifi"]["staOn"].b && contains(log, "tractor network not set"),
         "the page says the tractor network is not set (Module tab: 'not set', log: 'tractor network not set')", log);
    statusAlwaysJson(m);
}

SCENARIO(User::Unconfigured, "seeder-never-set-up", "fits a new seed drill module on the cable and never opens its page", seederBoards())
{
    World w;
    Module& m = w.add(board);
    m.dev.cableIn = true;
    m.powerOn();
    w.aow.toolSections = 19;
    w.aow.startup();
    w.run(1000);
    std::string bad;
    const int n = driveAndCompare(w, m, first(19), inARow(m.bi.relays, 1), 5000, &bad);
    must("works-out-of-the-box", n == 0, "relay n follows section n with nothing set (the defaults: relay 1 = section 1, cable)", bad);
    statusAlwaysJson(m);
}

SCENARIO(User::Unconfigured, "aow-default-pins", "8-section boom, but AgOpenWeb's pin config left as it comes (pins 7-8 = none)", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.toolSections = 8;
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 0, 0 });       // AgOpenWeb's default pin config
    w.run(500);
    const uint64_t from = w.now();
    std::string bad;
    const int n = driveAndCompare(w, m, first(8), { 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 10000, &bad);
    must("follows-pin-config", n == 0, "relays 1-6 follow sections 1-6, relays 7-8 stay open-circuit (as the pin config says)", bad);
    Json s = m.status();
    note("what AgOpenWeb is told", fmt("asked %s, open %s (PGN 237)", s["aog"]["req"].str().c_str(), s["out"]["on"].str().c_str()));
    const std::string said = pageLog(m) + screenMessages(w, from);
    tell("sections-without-relay", contains(said, "no relay"),
         "the module says sections 7-8 are asked for but no relay follows them (they would never spray)",
         "the page only outlines sections 7-8 in orange; nothing on AgOpenWeb's screen");
    statusAlwaysJson(m);
}

SCENARIO(User::Unconfigured, "sensors-ticked-none-fitted", "leaves 'pressure sensor + flow meter' ticked (the default) with no sensors wired", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    SprayerRig& rig = w.addRig(m);
    rig.pressureFitted = false;
    rig.flowFitted = false;
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    w.run(500);
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 20000);
    Json s = m.status();
    const std::string screen = screenMessages(w, from);
    note("module reads", fmt("%.2f bar (%.2f V), %.2f L/min, alarms %g", s["spr"]["bar"].num(), s["spr"]["volts"].num(), s["spr"]["lpm"].num(),
                             s["rate"]["alarms"].num()));
    note("AgOpenWeb's screen shows", screen.empty() ? "(nothing)" : screen);
    tell("sensor-missing-said", contains(pageLog(m) + screen, "sensor"),
         "the module says the pressure sensor looks unplugged (it reads 0 V; a 0.5-4.5 V sensor never does)",
         "only 'low pressure 0.0 bar' reaches the screen, which reads like a pump problem");
    statusAlwaysJson(m);
}

SCENARIO(User::Unconfigured, "no-section-widths", "uses a program that never sends the section widths (PGN 235)", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.sendMachineConfig(2, 4, 0, false);
    w.aow.sendPinConfig({ 1, 2, 3, 4, 5, 6, 7, 8 });
    m.http(HTTP_POST, "/api/spray", { { "pw", "agopengps" }, { "rate", "200" }, { "nzSp", "0.5" }, { "nzFl", "1.6" },
                                      { "nzBar", "3" }, { "pLow", "1" }, { "pHigh", "6" }, { "fTol", "20" } });
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 15000);
    Json s = m.status();
    must("no-false-alarm", s["rate"]["alarms"].num() == 0 && screenMessages(w, from).empty(), "no alarm when the widths are missing",
         fmt("alarms %g", s["rate"]["alarms"].num()));
    must("no-made-up-rate", s["rate"]["lha"].num() == -1, "no L/ha is shown without the widths (not a made-up one)", fmt("lha %g", s["rate"]["lha"].num()));
    tell("widths-missing-said", s["tool"]["n"].num() == 0, "the page says 'no section widths from AgOpenWeb yet' (status tool.n = 0)",
         fmt("tool.n %g", s["tool"]["n"].num()));
    statusAlwaysJson(m);
}
