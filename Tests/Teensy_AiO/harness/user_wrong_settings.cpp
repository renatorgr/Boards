/* user_wrong_settings.cpp - wrong settings in AgOpenWeb or on the module's web page.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"

using namespace bench;
using namespace chk;

// The wheels pushed the wrong way, or not at all (a setting that does not match the wiring): how far they go and
// how soon the module gives up. The wheels-did-not-follow check (zSafety.ino steerStuckCheck) is the net here.
// moves: the wheels move (the other way, or past the asked angle); else nothing moves (a driver set wrong)
static void wrongWay(World& w, const char* what, bool moves)
{
    w.ruleRightWay = false;                    // on purpose: the module cannot know which way is right
    w.tractor.x = 0;
    w.aow.guide = false;
    w.aow.setDeg = 10;                         // AgOpenWeb asks for 10 deg right
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    const float from = w.tractor.wheelDeg;
    float far = 0;                             // the wrong way, or past the asked angle (a WAS read backwards)
    w.watchers.push_back([&] { far = std::max({ far, -(w.tractor.wheelDeg - from), w.tractor.wheelDeg - w.aow.setDeg }); });
    w.tractor.kmh = 6;
    engage(w, true);
    const uint64_t t0 = w.now();
    w.run(300);
    w.runUntil([&] { return !w.engaged(); }, 4000);
    const uint64_t offMs = w.now() - t0;
    w.run(2000);
    w.watchers.pop_back();
    note("wrong-way", fmt("%s: the wheels went %.1f deg the wrong way or past the asked 10 deg, outputs off after %llu ms", what, far,
                          (unsigned long long)offMs));
    must("gives-up", offMs < 2000, "Stops pushing when the wheels do not go where asked (within 2 s)",
         fmt("%llu ms, %.1f deg the wrong way", (unsigned long long)offMs, far));
    must("not-far", far < 25, "The wheels go less than 25 deg the wrong way (or past the asked angle) before it stops", fmt("%.1f deg", far));
    tell("says-why", told(w, mark, smark, "did not follow"), "Says the wheels did not follow", lastScreen(w));
    if (moves)
        tell("says-which-setting", told(w, mark, smark, "Invert") || told(w, mark, smark, "other way") || told(w, mark, smark, "wrong way"),
             "Says the wheels went the OTHER way, and to check Invert WAS or Motor direction", logSince(w, mark, 400));
    else
        tell("says-which-setting", told(w, mark, smark, "driver") || told(w, mark, smark, "Danfoss"),
             "Says the wheels did not move at all, and to check the motor driver setting (Cytron / IBT2 / Danfoss)", logSince(w, mark, 400));
}

SCENARIO(User::WrongSettings, "was-not-inverted", "The WAS turns the other way but 'Invert WAS' is not ticked", allKits())
{
    World w(kit);
    w.was.mountedBackwards = true;
    w.startUp();
    wrongWay(w, "WAS backwards", true);
    reportRules(w);
}

SCENARIO(User::WrongSettings, "motor-direction", "The valve coils / motor wires are swapped and 'Motor direction' is not ticked", allKits())
{
    World w(kit);
    w.tractor.wiredBackwards = true;
    w.startUp();
    wrongWay(w, "motor wired backwards", true);
    reportRules(w);
}

SCENARIO(User::WrongSettings, "ibt2-set-as-cytron", "An IBT2 motor driver, AgOpenWeb set to Cytron", { Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    w.aow.sc.cytron = true;
    w.aow.send251();
    w.run(1000);
    w.ruleSteerOnlyWhenAsked = false;          // the Cytron's relay pin is the IBT2's right PWM: "on" means something else here
    const size_t mark = w.aio.logMark();
    float far = 0;
    w.watchers.push_back([&] { far = std::max(far, fabsf(w.tractor.wheelDeg)); });
    w.tractor.kmh = 0;
    w.run(3000);
    note("idle", fmt("autosteer off: the wheels moved %.1f deg by themselves", far));
    must("idle-still", far < 1, "With autosteer off, the wheels stay where they are", fmt("moved %.1f deg", far));
    wrongWay(w, "IBT2 driven as a Cytron", false);
    (void)mark;
    reportRules(w);
}

SCENARIO(User::WrongSettings, "danfoss-not-set", "A Danfoss valve, AgOpenWeb set to a plain Cytron valve", { Kit::F9pDanfoss })
{
    World w(kit);
    w.startUp();
    w.aow.sc.danfoss = false;
    w.aow.send251();
    w.run(1000);
    wrongWay(w, "Danfoss driven as a plain valve", false);
    reportRules(w);
}

SCENARIO(User::WrongSettings, "counts-zero", "Counts per degree set to 0 in AgOpenWeb's WAS settings", allKits())
{
    World w(kit);
    w.startUp();
    w.aow.ss.counts = 0;
    w.aow.send252();
    w.run(1000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.ruleRightWay = false;
    float far = 0;
    w.watchers.push_back([&] { far = std::max(far, fabsf(w.tractor.wheelDeg - w.setAtModule)); });
    steerFor(w, 8, 8000);
    w.watchers.pop_back();
    const Json st = w.status();
    must("json", w.badStatus == 0, "The web page's status stays valid JSON", w.lastBadStatus);
    const auto s253 = w.aow.heardPgn(253, w.now() - 2000);
    int16_t angle = 0;
    if (!s253.empty() && s253.back().data.size() > 6) angle = (int16_t)(s253.back().data[5] | s253.back().data[6] << 8);
    note("counts-zero", fmt("%llu ms of outputs, wheels up to %.1f deg from the asked angle, PGN 253 angle %d (x0.01 deg)",
                            (unsigned long long)w.engagedMs, far, angle));
    must("no-steer", w.engagedMs < 1000 || far < 5, "Does not steer blind with counts 0 (a division by zero)",
         fmt("%llu ms of outputs, wheels %.1f deg off", (unsigned long long)w.engagedMs, far));
    note("counts-kept", fmt("counts per degree now %g (the angle is the WAS reading divided by it)", st["steerCfg"]["counts"].num()));
    tell("says-counts", told(w, mark, smark, "counts") || told(w, mark, smark, "Counts"),
         "Says counts per degree 0 is not usable (not an angle of millions of degrees)", logSince(w, mark, 300));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "min-above-max-pwm", "Minimum PWM (Min To Move) set above the Max Limit", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.aow.ss.minPwm = 200;
    w.aow.ss.highPwm = 100;
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();   // before the start: said when the settings come
    w.startUp();
    w.tractor.x = -0.5f;
    int top = 0;                               // the highest PWM the driver got
    w.watchers.push_back([&] {
        const auto& d = w.aio.dev;
        for (int pin : { 2, 3 })
            if (d.pwm[pin] > top && !(pin == 3 && kit != Kit::F9pIbt2)) top = d.pwm[pin];
    });
    steerFor(w, 8, 20000);
    w.watchers.pop_back();
    const Json st = w.status();
    note("min-above-max", fmt("low PWM became %g; PWM up to %d; %.2f m off the line after 20 s, worst %.1f deg from the asked angle",
                              st["steerCfg"]["lowPwm"].num(), top, fabsf(w.tractor.x), w.maxAbsErrDeg));
    must("max-limit", top <= 100, "The PWM never goes above the Max Limit (100)", fmt("PWM up to %d", top));
    tell("says-pwm", told(w, mark, smark, "PWM") || told(w, mark, smark, "Min"), "Says Min To Move is above the Max Limit", logSince(w, mark, 300));
    w.ruleRightWay = false;                    // a bang-bang valve overshoots: not the module's direction choice
    reportRules(w);
}

SCENARIO(User::WrongSettings, "current-sensor-not-fitted", "'Current sensor' ticked in AgOpenWeb, no sensor fitted", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.aow.sc.current = true;
    w.aow.sc.pulseCountMax = 80;
    w.startUp();
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    steerFor(w, 8, 6000);
    note("current", fmt("%llu ms of steering out of 6 s", (unsigned long long)w.engagedMs));
    tell("says-kickout", told(w, mark, smark, "current"), "Says why autosteer does not come on (the current kickout, all the time)", lastScreen(w));
    tell("says-not-fitted", told(w, mark, smark, "no current sensor") || told(w, mark, smark, "not fitted") || told(w, mark, smark, "nothing on"),
         "Says the current sensor reads as if nothing is fitted", logSince(w, mark, 300));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "switch-mode-no-switch", "AgOpenWeb set to 'Switch', no steer switch wired", { Kit::PumaCytron, Kit::F9pIbt2, Kit::F9pDanfoss })
{
    World w(kit);
    w.aow.sc.steerSwitch = true;
    w.aow.sc.steerButton = false;
    w.startUp();
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    steerFor(w, 8, 6000);
    must("no-output", w.engagedMs == 0, "Does not steer with the switch open", fmt("%llu ms", (unsigned long long)w.engagedMs));
    // AgOpenWeb learns it from PGN 253: byte 11 bit 1 = the steer switch off (its button follows it)
    const auto s253 = w.aow.heardPgn(253, w.now() - 1000);
    const bool offBit = !s253.empty() && s253.back().data.size() > 11 && (s253.back().data[11] & 2);
    must("reports-switch", offBit, "Tells AgOpenWeb the steer switch is off (PGN 253)", s253.empty() ? "no PGN 253" : fmt("switch byte 0x%02X", s253.back().data[11]));
    (void)mark; (void)smark;
    reportRules(w);
}

SCENARIO(User::WrongSettings, "subnet-while-steering", "Presses 'send subnet' in AgOpenWeb while autosteer is steering", { Kit::PumaCytron, Kit::Um982Keya })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 4000);
    must("steering", w.engaged(), "Steering before the subnet change");
    w.aow.sendSubnet(192, 168, 5);             // the subnet it is on already
    w.run(500);
    must("same-subnet-no-restart", w.aio.boots == 1, "The subnet it is on already: nothing to do, no restart",
         fmt("%u starts; the module restarted in the field", w.aio.boots));
    if (w.aio.boots > 1) { w.run(6000); steerFor(w, 8, 3000); }
    const unsigned before = w.aio.boots;
    w.aow.sendSubnet(192, 168, 1);
    w.run(500);
    note("restart", fmt("%u starts", w.aio.boots));
    must("refused-while-steering", w.aio.boots == before, "A subnet change is not done while steering (as the XIAO page's own change is not)",
         fmt("%u starts; the module restarted in the field", w.aio.boots));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "subnet-invalid", "AgOpenWeb sends the subnet 0.0.0 (an empty field) with autosteer off", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    w.aow.sendSubnet(0, 0, 0);
    w.run(8000);
    const uint8_t* e = w.aio.dev.eeprom.data();
    note("subnet", fmt("EEPROM subnet now %u.%u.%u, IP %s, %u starts", e[60], e[61], e[62], w.aio.dev.ip.str().c_str(), w.aio.boots));
    must("refuses-0", !(e[60] == 0 && e[61] == 0 && e[62] == 0), "Refuses the subnet 0.0.0 (the XIAO page's change refuses it)",
         fmt("module now at %s: unreachable", w.aio.dev.ip.str().c_str()));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "page-settings-range", "Types values out of range on the module's Settings page", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    const char* bad[] = { "steerMaxKmh=0", "steerMaxKmh=200", "steerMinKmhX10=0", "epochWaitMs=5", "outputMode=7", "softStartX10ms=-1",
                          "steerMaxKmh=abc", "steerMaxKmh=", "tm171YawSign=5" };
    for (const char* b : bad)
    {
        const Http h = w.http("POST", "/api/settings", b);
        must(std::string("refused:") + b, h.code == 400, std::string("Refuses ") + b, fmt("HTTP %d %s", h.code, h.body.substr(0, 80).c_str()));
    }
    const Json s = w.http("GET", "/api/settings").json;
    must("unchanged", s["steerMaxKmh"].num() == 25 || s["settings"]["steerMaxKmh"].num() == 25 || w.http("GET", "/api/settings").body.find("\"steerMaxKmh\":25") != std::string::npos,
         "The settings stay as they were", w.http("GET", "/api/settings").body.substr(0, 200));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "output-mode-for-agopengps", "Uses AgOpenGPS (AgIO) with the module's output set to AgOpenWeb (standard)", { Kit::PumaCytron })
{
    World w(kit);
    w.aow.agio = true;
    w.aow.on = false;                          // the module was set up for AgOpenWeb: output standard
    w.aio.powerOn();
    w.run(1000);
    const Http h = w.http("POST", "/api/settings", "outputMode=0");
    must("saved", h.code == 200, "(bench) the page takes outputMode=0", fmt("HTTP %d %s", h.code, h.body.substr(0, 80).c_str()));
    w.aow.on = true;                           // then AgIO starts
    w.run(6000);
    tell("says-mode", w.aio.logHas("does not read"), "Says AgOpenGPS cannot read the AgOpenWeb sentences", w.aio.logLine("Program:"));
    // the other way: AgOpenGPS already talking (automatic mode), then the mode set to AgOpenWeb on the page
    w.http("POST", "/api/settings", "outputMode=2");
    w.run(3000);
    const size_t mark = w.aio.logMark();
    w.http("POST", "/api/settings", "outputMode=0");
    w.run(6000);
    tell("says-mode-later", w.aio.logHas("does not read", mark), "Says it too when the mode is changed while AgOpenGPS talks", logSince(w, mark, 300));
    reportRules(w);
}

SCENARIO(User::WrongSettings, "wifi-bridge-subnet", "On the XIAO's WiFi (no network cable to the tablet), presses 'send subnet' and scans in the program", { Kit::PumaCytron })
{
    World w(kit);
    wifiTablet(w);                                  // the module's cable goes to a switch with no program on it
    w.startUp();
    const uint32_t before = w.aio.boots;
    const size_t mark = w.aio.logMark();
    w.tab.sendSubnet(192, 168, 4);
    w.tab.scan();
    w.run(6000);
    const Json st = w.status();
    must("no-restart", w.aio.boots == before, "A subnet sent over the XIAO's WiFi does not restart the module", fmt("%u starts", w.aio.boots));
    must("subnet-kept", st["ip"].str() == "192.168.5.126", "The module keeps its subnet (it is for the network cable)", st["ip"].str());
    must("still-answered", !w.tab.heardPgn(126, w.now() - 2500).empty(), "The program on the WiFi is still answered");
    must("no-scan-reply", w.tab.heardPgn(203).empty(), "No scan reply over the WiFi (it would name the cable's subnet)");
    tell("says-why", w.aio.logHas("PGN 201) not done", mark), "Says the subnet from the XIAO's WiFi was not done, and where to change it", logSince(w, mark, 300));
    reportRules(w);
}
