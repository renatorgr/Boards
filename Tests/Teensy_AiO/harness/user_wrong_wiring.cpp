/* user_wrong_wiring.cpp - parts plugged into the wrong place, at the wrong speed, or not at all.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"

using namespace bench;
using namespace chk;

SCENARIO(User::WrongWiring, "gps-on-gps2-port", "The receiver plugged into the GPS2 port (for the XIAO / second F9P) instead of the GPS port", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.rx.port = 2;
    w.startUp(20000);
    must("no-crash", w.aio.boots == 1, "Keeps running", fmt("%u starts", w.aio.boots));
    tell("says-port", w.aio.logHas("GPS2 port: a GNSS receiver") || w.aio.logHas("wrong port") || w.aio.logHas("GPS2 port: NMEA"),
         "Says the receiver is on the GPS2 port, not the GPS port", w.aio.logLine("GPS2"));
    reportRules(w);
}

SCENARIO(User::WrongWiring, "gps-wrong-baud", "The receiver set to 115200 baud (the module listens at 460800)", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.rx.baud = 115200;
    w.startUp(30000);
    note("baud", fmt("receiver now at %u baud (%u changes by the module), %u GGA reached AgOpenWeb", w.rx.baud, w.rx.baudChanges, w.aow.nmeaGga));
    tell("says-baud", w.aio.logHas("baud"), "Says the receiver's baud rate is wrong (or finds it)", w.aio.logLine("baud"));
    must("fixes-or-keeps-quiet", w.aow.nmeaBadSum == 0, "Sends AgOpenWeb no broken NMEA from the wrong speed", fmt("%u bad", w.aow.nmeaBadSum));
    must("gps-after", w.aow.nmeaGga > 20 || w.rx.baud != 460800, "GPS reaches AgOpenWeb once the receiver is at 460800", fmt("%u GGA", w.aow.nmeaGga));
    reportRules(w);
}

SCENARIO(User::WrongWiring, "imu-other-baud", "The TM171 left at 460800 baud (the module listens at 115200 first)", { Kit::PumaCytron })
{
    World w(kit);
    w.imu.baud = 460800;
    w.startUp(10000);
    const Json st = w.status();
    note("imu", fmt("IMU found: '%s' on '%s'", st["tm"]["type"].str().c_str(), st["tm"]["port"].str().c_str()));
    tell("says-imu-baud", w.aio.logHas("baud") && w.aio.logHas("TM171"), "Says the TM171 is at another baud rate (Fix TM171 setup)", w.aio.logLine("TM171"));
    must("imu-found", st["tm"]["type"].str().find("TM171") != std::string::npos, "Finds the TM171 at its other baud rate", st["tm"]["type"].str());
    reportRules(w);
}

SCENARIO(User::WrongWiring, "was-not-connected", "The WAS plug left out (the input floats at 3.9 V), AgOpenWeb asks to steer", allKits())
{
    World w(kit);
    w.was.unplugged = true;
    w.startUp();
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    steerFor(w, 8, 8000);
    note("was-out", fmt("%llu ms of outputs; screen: %s", (unsigned long long)w.engagedMs, lastScreen(w).c_str()));
    tell("says-was", told(w, mark, smark, "WAS") || told(w, 0, 0, "WAS"), "Says the WAS reads wrong", lastScreen(w));
    must("no-blind-steer", w.engagedMs < 2000, "Does not keep steering with no WAS (a still reading inside the range)",
         fmt("%llu ms of outputs", (unsigned long long)w.engagedMs));
    w.ruleRightWay = false;
    reportRules(w);
}

SCENARIO(User::WrongWiring, "was-signal-to-ground", "The WAS signal wire touches ground (0 V)", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.was.shortToGround = true;
    w.startUp();
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    steerFor(w, 8, 5000);
    must("no-output", w.engagedMs == 0, "Never steers with the WAS at 0 V", fmt("%llu ms", (unsigned long long)w.engagedMs));
    tell("says-was", told(w, 0, 0, "WAS fault") || told(w, mark, smark, "WAS"), "Says the WAS is out of range, on AgOpenWeb's screen", lastScreen(w));
    reportRules(w);
}

SCENARIO(User::WrongWiring, "was-3wire-differential", "A 3-wire WAS with AgOpenWeb set to 'Differential' (nothing on WAS_L)", { Kit::PumaCytron })
{
    World w(kit);
    w.was.noLowWire = true;
    w.aow.sc.singleWas = false;
    w.startUp();
    steerFor(w, 8, 5000);
    must("no-output", w.engagedMs == 0, "Does not steer on a meaningless reading", fmt("%llu ms", (unsigned long long)w.engagedMs));
    tell("says-single", told(w, 0, 0, "Single"), "Says to choose Single (or wire pin 3)", w.aio.logLine("WAS"));
    reportRules(w);
}

SCENARIO(User::WrongWiring, "keya-can-unplugged", "The Keya's CAN plug left out at start-up", { Kit::Um982Keya })
{
    World w(kit);
    w.keya.canWired = false;
    w.startUp();
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.aow.guide = false;
    w.aow.setDeg = 8;                          // a curve: the wheels have to turn
    steerFor(w, 8, 6000);
    note("keya-unplugged", fmt("Keya commands %u; screen: %s", w.keya.speedCmds, lastScreen(w).c_str()));
    tell("says-boot", w.aio.logHas("No Keya heartbeat"), "Says at start-up that no Keya was found", w.aio.logLine("Keya"));
    tell("says-when-asked", told(w, mark, smark, "Keya") || told(w, mark, smark, "did not follow"), "Says why nothing moves when AgOpenWeb asks to steer", lastScreen(w));
    w.ruleRightWay = false;
    reportRules(w);
}

SCENARIO(User::WrongWiring, "keya-powered-late", "The Keya gets its power 10 s after the module (its fuse put in later)", { Kit::Um982Keya })
{
    World w(kit);
    w.keya.powered = false;
    w.startUp();
    w.keya.powered = true;
    w.run(2000);
    w.tractor.x = -0.5f;
    steerFor(w, 8, 20000);
    must("found-later", w.aio.logHas("Keya motor found"), "Finds the Keya when it starts later", w.aio.logLine("Keya"));
    must("steers", w.keya.speedCmds > 100 && fabsf(w.tractor.x) < 0.2f, "Steers with it", fmt("%u commands, %.2f m off the line", w.keya.speedCmds, fabsf(w.tractor.x)));
    reportRules(w);
}

SCENARIO(User::WrongWiring, "tablet-other-subnet", "The tablet on 192.168.1.x, the module on 192.168.5: AgOpenWeb scans and sets the subnet", { Kit::PumaCytron })
{
    World w(kit);
    w.aow.ip = IPAddress(192, 168, 1, 10);
    w.startUp();
    must("unheard", w.aow.heardPgn(126).empty(), "(bench) AgOpenWeb does not hear the module on another subnet");
    w.aow.scan();
    w.run(1000);
    const auto r = w.aow.heardPgn(203);
    must("scan-reply", !r.empty(), "Answers AgOpenWeb's scan (PGN 203 to everyone)");
    w.aow.sendSubnet(192, 168, 1);
    w.run(8000);
    w.aow.sendSettings();
    w.run(3000);
    must("moved", w.aio.dev.ip == IPAddress(192, 168, 1, 126), "Moves to the tablet's subnet", w.aio.dev.ip.str());
    must("heard", !w.aow.heardPgn(126, w.now() - 3000).empty(), "AgOpenWeb hears it after the change");
    reportRules(w);
}
