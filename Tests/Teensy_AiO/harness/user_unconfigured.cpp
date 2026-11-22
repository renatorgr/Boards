/* user_unconfigured.cpp - a module never set up: a new board, nothing sent by the program, parts missing.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"

using namespace bench;
using namespace chk;

SCENARIO(User::Unconfigured, "new-board-no-program", "A new board (empty EEPROM) with everything wired, the tablet off for a minute", allKits())
{
    World w(kit);
    w.aow.on = false;
    w.aio.powerOn();
    w.tractor.kmh = 6;
    w.run(60000);
    must("boots", w.aio.boots == 1, "Starts once and keeps running", fmt("%u starts", w.aio.boots));
    must("no-output", w.engagedMs == 0, "No steering output without the program", fmt("%llu ms of outputs", (unsigned long long)w.engagedMs));
    const Json st = w.status();
    must("page", st.has("fw"), "The web page answers", w.lastBadStatus);
    tell("says-no-program", w.aio.logHas("rogram") && (w.aio.logHas("no program") || w.aio.logHas("No program") || w.aio.logHas("not heard") ||
                                                       w.aio.logHas("waiting for the program")),
         "Says it is waiting for AgOpenWeb", logSince(w, 0, 300));
    reportRules(w);
}

SCENARIO(User::Unconfigured, "no-steer-settings", "AgOpenWeb engages but never sent its steer settings (the module's defaults: counts 30, no offset)", allKits())
{
    World w(kit);
    w.aio.powerOn();
    w.run(8000);                               // hello and steering packets, no PGN 252 / 251
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.tractor.x = -0.5f;
    steerFor(w, 8, 30000);
    const Json st = w.status();
    note("defaults", fmt("counts %g, Kp %g, max PWM %g; %.2f m off the line after 30 s", st["steerCfg"]["counts"].num(), st["steerCfg"]["kp"].num(),
                         st["steerCfg"]["hiPwm"].num(), fabsf(w.tractor.x)));
    tell("says-no-settings", st["steerCfg"]["got252"].num() < 0 || told(w, mark, smark, "252") || told(w, 0, 0, "steer settings"),
         "Says AgOpenWeb has not sent its steer settings (PGN 252): the page's Steering card says 'not yet'", logSince(w, mark, 300));
    (void)smark;
    w.ruleRightWay = false;                    // the module does not know the sensor: its idea of the angle is off
    reportRules(w);
}

SCENARIO(User::Unconfigured, "no-ads1115", "A board without the ADS1115 (WAS converter): GPS only, AgOpenWeb asks to steer", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.adsFitted = false;
    w.aio.dev.i2c[1].erase(0x48);
    w.startUp();
    steerFor(w, 8, 10000);
    const Json st = w.status();
    must("ip-120", st["ip"].str() == "192.168.5.120", "IP .120 (GPS only)", st["ip"].str());
    must("no-output", w.engagedMs == 0, "Never steers without the WAS converter", fmt("%llu ms of outputs", (unsigned long long)w.engagedMs));
    must("gps-still", w.aow.nmeaGga > 50, "GPS still goes to AgOpenWeb", fmt("%u GGA", w.aow.nmeaGga));
    tell("says-no-ads", w.aio.logHas("ADS1115 (WAS) not found"), "Says the ADS1115 is missing", w.aio.logLine("ADS1115"));
    must("no-steer-hello", w.aow.heardPgn(126).empty(), "Does not answer AgOpenWeb's hello as a steer module (PGN 126): AgOpenWeb shows no steer module",
         fmt("%zu steer hellos", w.aow.heardPgn(126).size()));
    reportRules(w);
}

SCENARIO(User::Unconfigured, "nothing-plugged", "Only the board and the network cable: no receiver, no IMU, no WAS, no Keya", { Kit::PumaCytron })
{
    World w(kit);
    w.rx.on = false;
    w.imu.on = false;
    w.was.unplugged = true;
    w.was.floatV = 4.95f;                      // an open AiO WAS input with its pull-up reads about 5 V
    w.startUp();
    steerFor(w, 0, 20000);
    const Json st = w.status();
    must("boots", w.aio.boots == 1, "Starts once and keeps running", fmt("%u starts", w.aio.boots));
    must("no-output", w.engagedMs == 0, "Never steers", fmt("%llu ms", (unsigned long long)w.engagedMs));
    must("page", st.has("fw"), "The web page answers");
    tell("says-no-gps", w.aio.logHas("GPS: nothing at"), "Says no GPS data comes in", w.aio.logLine("GPS:"));
    tell("says-no-imu", w.aio.logHas("IMU: NO DATA") || w.aio.logHas("No IMU") || w.aio.logHas("no IMU"), "Says no IMU was found", w.aio.logLine("IMU"));
    tell("says-was", told(w, 0, 0, "WAS"), "Says the WAS reads out of range", w.aio.logLine("WAS"));
    reportRules(w);
}

SCENARIO(User::Unconfigured, "cable-late", "The network cable goes in 20 s after power-on", { Kit::PumaCytron })
{
    World w(kit);
    w.cableIn = false;
    w.aio.dev.cableIn = false;
    w.aio.powerOn();
    w.run(20000);
    w.cableIn = true;
    w.aio.dev.cableIn = true;
    w.run(5000);
    w.aow.sendSettings();
    w.run(3000);
    must("hello", !w.aow.heardPgn(126, w.now() - 3000).empty(), "Answers AgOpenWeb once the cable is in");
    const Json st = w.status();
    must("page", st.has("fw"), "The web page answers once the cable is in", w.lastBadStatus);
    must("nmea", w.aow.nmeaGga > 20, "GPS reaches AgOpenWeb", fmt("%u GGA", w.aow.nmeaGga));
    steerFor(w, 8, 5000);
    must("steers", w.engagedMs > 3000, "Steers when asked", fmt("%llu ms", (unsigned long long)w.engagedMs));
    reportRules(w);
}

SCENARIO(User::Unconfigured, "eeprom-damaged", "Stored steer settings damaged: the mark 2400 is right, the bytes after it read 0xFF", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    auto& e = w.aio.dev.eeprom;
    e[0] = 0x60; e[1] = 0x09;                  // EEP_Ident 2400 (Autosteer.ino)
    for (int i = 10; i < 60; i++) e[i] = 0xFF; // what an erased area reads: the float counts per degree NaN, every flag on
    e[60] = 192; e[61] = 168; e[62] = 5;
    w.aio.powerOn();
    w.run(5000);
    const Json st = w.status();
    must("json", w.badStatus == 0, "The web page's status stays valid JSON with odd stored settings", w.lastBadStatus);
    w.tractor.kmh = 6;
    engage(w, true);
    w.run(5000);
    note("odd-settings", fmt("%llu ms of outputs before AgOpenWeb's settings came; wheels at %.1f deg", (unsigned long long)w.engagedMs, w.tractor.wheelDeg));
    must("no-output-nan", w.engagedMs == 0 || fabsf(w.tractor.wheelDeg) < 10, "No steering on stored settings that are not numbers",
         fmt("%llu ms of outputs, wheels at %.1f deg", (unsigned long long)w.engagedMs, w.tractor.wheelDeg));
    w.aow.sendSettings();
    w.run(8000);
    must("recovers", w.status()["steerCfg"]["counts"].num() == w.aow.ss.counts, "AgOpenWeb's settings replace them");
    w.ruleRightWay = false;
    reportRules(w);
}
