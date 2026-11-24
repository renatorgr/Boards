/* user_field_failures.cpp - things that break or go wrong in the field while autosteer works.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
#include <fstream>
#include <sstream>

using namespace bench;
using namespace chk;

SCENARIO(User::FieldFailures, "was-unplugged-steering", "The WAS plug comes out while steering (the input floats to 3.9 V, inside the range)", allKits())
{
    World w(kit);
    w.startUp();
    w.tractor.x = -0.3f;
    steerFor(w, 8, 5000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.was.unplugged = true;
    w.expectOff("the WAS plug came out", 300);
    const uint64_t off = msUntilOff(w, 5000);
    w.run(3000);
    note("was-unplugged", fmt("outputs off %llu ms after the plug came out", (unsigned long long)off));
    must("off-fast", off <= 300, "Steering off within 0.3 s of the WAS plug coming out", fmt("%llu ms", (unsigned long long)off));
    tell("says-was", told(w, mark, smark, "WAS"), "Says it on AgOpenWeb's screen", lastScreen(w));
    w.expectOffEnd();
    w.ruleRightWay = false;
    reportRules(w);
}

SCENARIO(User::FieldFailures, "was-wire-cut", "The WAS signal wire breaks while steering (0 V)", { Kit::PumaCytron, Kit::F9pIbt2, Kit::F9pDanfoss })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 5000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.was.shortToGround = true;
    w.expectOff("the WAS wire broke", 200);
    const uint64_t off = msUntilOff(w, 5000);
    w.run(2000);
    must("off-fast", off <= 200, "Steering off within 0.2 s", fmt("%llu ms", (unsigned long long)off));
    tell("says-was", told(w, mark, smark, "WAS fault"), "Says 'WAS fault' on AgOpenWeb's screen", lastScreen(w));
    // the wire is mended: autosteer comes back only when the driver engages again
    w.was.shortToGround = false;
    w.expectOffEnd();
    w.run(3000);
    w.ruleRightWay = false;
    reportRules(w);
}

SCENARIO(User::FieldFailures, "keya-lost", "The Keya's CAN cable comes off while steering", { Kit::Um982Keya })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 5000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.keya.canWired = false;
    w.ruleRightWay = false;                    // the Keya runs on its last command until its own timeout: nobody can stop it sooner
    w.expectOff("the Keya stopped answering", 700);
    w.run(3000);
    tell("says-keya", told(w, mark, smark, "Keya not responding"), "Says the Keya does not answer, on AgOpenWeb's screen", lastScreen(w));
    must("blocked", w.status()["steer"]["keyaBlock"].b || w.aio.logHas("autosteer off and blocked", mark), "Autosteer blocked until the Keya answers");
    w.keya.canWired = true;
    w.run(2000);
    w.expectOffEnd();
    reportRules(w);
}

SCENARIO(User::FieldFailures, "keya-lost-was-back", "The WAS wire breaks and is mended while the Keya's CAN cable is off", { Kit::Um982Keya })
{
    World w(kit);
    w.aow.sc.steerSwitch = true;               // a steer switch, left on all through: both blocks stay
    w.sw.steerClosed = true;
    w.startUp();
    steerFor(w, 8, 5000);
    w.ruleRightWay = false;                    // the Keya runs on its last command until its own timeout
    w.was.shortToGround = true;                // the WAS check blocks
    w.expectOff("the WAS wire broke", 700);
    w.run(1500);
    w.keya.canWired = false;                   // and the Keya stops answering
    w.run(3000);
    w.was.shortToGround = false;               // the WAS is mended; the Keya is still lost
    w.run(2000);
    w.aow.engage = false;                      // AgOpenWeb's button off and on; the steer switch stays on
    w.run(500);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.aow.engage = true;                       // AgOpenWeb asks to steer again
    w.run(2000);
    const std::string last = lastScreen(w);
    note("screen", last);
    must("says-keya", w.aow.screenHas("Keya not responding", smark) && last.find("Keya not responding") != std::string::npos,
         "AgOpenWeb's screen says the Keya still does not answer, not only that the WAS is back", last);
    tell("logs-both", w.aio.logHas("Keya motor does not answer", mark) && w.aio.logHas("WAS is back in range", mark),
         "The module's log keeps both reasons", w.aio.logLine("[status]"));
    w.expectOffEnd();
    reportRules(w);
}

SCENARIO(User::FieldFailures, "keya-fault", "The Keya reports an over-current fault while steering", { Kit::Um982Keya })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 5000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.keya.faults = 1 << 6;                    // byte 7 bit 6: over current
    w.run(1000);
    must("off", w.keya.speedCmd == 0 || !w.keya.enabled, "The Keya is told to stop", fmt("enabled %d, speed %d", w.keya.enabled, w.keya.speedCmd));
    tell("says-fault", told(w, mark, smark, "Keya motor fault") || told(w, mark, smark, "over current"), "Says which fault", lastScreen(w));
    w.ruleSteerOnlyWhenAsked = false;          // AgOpenWeb still asks: the fault is the module's reason to stop
    reportRules(w);
}

SCENARIO(User::FieldFailures, "firmware-hangs", "The firmware stops (a hang) while steering: the watchdog", allKits())
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 4000);
    w.expectWatchdog = true;
    w.ruleRightWay = false;                    // a hung loop leaves the outputs as they were until the watchdog warning
    w.aio.hung = true;
    const size_t smark = w.aow.screen.size();
    const uint64_t off = msUntilOff(w, 6000);
    note("hang", fmt("outputs off %llu ms after the hang", (unsigned long long)off));
    if (kit != Kit::Um982Keya)
        must("off-by-warning", off <= 1600, "Outputs off by the watchdog warning (1.5 s), before the restart", fmt("%llu ms", (unsigned long long)off));
    else
        must("keya-stops", off <= 2100, "The Keya stops within the watchdog time (it is not told: its own timeout, 0.5 s assumed)",
             fmt("%llu ms", (unsigned long long)off));
    w.ruleSteerOnlyWhenAsked = false;          // AgOpenWeb keeps asking: the hang is the reason it stops
    w.run(8000);
    must("restarted", w.aio.boots == 2 && w.aio.dev.wdtResets == 1, "Restarted by the watchdog", fmt("%u starts, %u watchdog resets", w.aio.boots, w.aio.dev.wdtResets));
    must("says-watchdog", w.aio.logHas("Start: WATCHDOG"), "Its log says WATCHDOG and where it stopped", w.aio.logLine("Start:", 0));
    tell("screen-watchdog", w.aow.screenHas("watchdog", smark), "AgOpenWeb's screen says the module restarted by its watchdog", lastScreen(w));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "firmware-crash", "The firmware crashes (a fault) while steering", allKits())
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 4000);
    w.expectWatchdog = true;
    const size_t smark = w.aow.screen.size();
    w.aio.crash();                             // the crash hook, then the core's fault handler until the restart
    const bool restarted = w.aio.boots == 2;
    w.ruleSteerOnlyWhenAsked = false;
    w.run(8000);
    must("restarted", restarted, "Restarts after the crash", fmt("%u starts", w.aio.boots));
    must("says-crash", w.aio.logHas("Start: CRASH"), "Its log says CRASH with the address", w.aio.logLine("Start:", 0));
    tell("screen-crash", w.aow.screenHas("crash", smark), "AgOpenWeb's screen says the module restarted after a crash", lastScreen(w));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "crash-outputs-off", "The firmware crashes while steering: outputs off at once (crash hook)", allKits())
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 4000);
    must("steering", w.engaged(), "Steering before the crash");
    w.expectWatchdog = true;
    w.ruleSteerOnlyWhenAsked = false;
    const float wheelsAtCrash = w.tractor.wheelDeg;
    float moved = 0;
    w.watchers.push_back([&] { moved = std::max(moved, fabsf(w.tractor.wheelDeg - wheelsAtCrash)); });
    w.aio.crash();                             // the crash hook, then nothing until the restart
    w.watchers.pop_back();
    note("crash", fmt("the wheels moved %.2f deg between the crash and the restart", moved));
    must("still", moved < 1.0f, "The wheels stop at the crash (moved less than 1 deg until the restart)", fmt("%.2f deg", moved));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "power-dip", "The module loses power for 0.2 s while steering (starter motor, loose plug)", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 4000);
    const size_t smark = w.aow.screen.size();
    w.aio.powerOff();
    w.run(200);
    const uint64_t t0 = w.now();
    w.aio.powerOn();
    uint64_t backMs = 0;
    w.runUntil([&] { return w.engaged(); }, 15000);
    backMs = w.now() - t0;
    note("power-dip", fmt("steering again %llu ms after the power came back, without a new engage (AgOpenWeb still asked)", (unsigned long long)backMs));
    tell("screen-restart", w.aow.screen.size() > smark, "AgOpenWeb's screen says the module restarted", lastScreen(w));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "too-fast", "Drives above 25 km/h (road) with autosteer on", { Kit::PumaCytron, Kit::Um982Keya })
{
    World w(kit);
    w.startUp();
    steerFor(w, 20, 4000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.tractor.kmh = 30;
    w.expectOff("above the maximum speed", 250);
    w.run(2000);
    tell("says-fast", told(w, mark, smark, "above"), "Says why (speed above the maximum)", lastScreen(w));
    w.expectOffEnd();
    w.tractor.kmh = 20;
    w.run(2000);
    reportRules(w);
}

SCENARIO(User::FieldFailures, "packet-loss", "A bad network: one steering packet in three lost", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    w.aow.dropEvery = 3;
    w.tractor.x = -0.5f;
    steerFor(w, 8, 30000);
    must("follows", fabsf(w.tractor.x) < 0.15f, "Still follows the line", fmt("%.2f m off", fabsf(w.tractor.x)));
    must("no-dropout", w.engagedMs >= 29000, "No dropouts of the steering", fmt("%llu ms of 30 s", (unsigned long long)w.engagedMs));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "short-settings-packet", "A broken (short) steer settings packet arrives while steering", allKits())
{
    World w(kit);
    w.startUp();
    w.aow.guide = false;
    w.aow.setDeg = 3;
    steerFor(w, 8, 3000);
    const Json before = w.status();
    // 80 81 7F FC with only 3 of its 8 data bytes (and its length byte still says 8)
    w.aow.send({ 0x80, 0x81, 0x7F, 0xFC, 8, 60, 150, 30 });
    w.run(2000);
    const Json after = w.status();
    must("ignored", after["steerCfg"]["counts"].num() == before["steerCfg"]["counts"].num() &&
                    after["steerCfg"]["offset"].num() == before["steerCfg"]["offset"].num() &&
                    after["steerCfg"]["minPwm"].num() == before["steerCfg"]["minPwm"].num(),
         "A steer settings packet shorter than its length byte says is ignored, not saved",
         fmt("counts %g -> %g, WAS offset %g -> %g, min PWM %g -> %g (the missing bytes come from the packet before)",
             before["steerCfg"]["counts"].num(), after["steerCfg"]["counts"].num(), before["steerCfg"]["offset"].num(), after["steerCfg"]["offset"].num(),
             before["steerCfg"]["minPwm"].num(), after["steerCfg"]["minPwm"].num()));
    // a settings packet with a wrong checksum: taken (as by every AgOpenGPS firmware). A check would have to be
    // sure that every program sends it right first (the official machine module does not, zMachine.ino)
    w.aow.sendSettings();
    w.run(500);
    std::vector<uint8_t> p = pgn(252, { 60, 150, 30, 25, 7, 0, 0, 100 });
    p.back() ^= 0x55;
    w.aow.send(p);
    w.run(1000);
    note("bad-crc", fmt("a steer settings packet with a wrong checksum: counts now %g (%s)", w.status()["steerCfg"]["counts"].num(),
                        w.status()["steerCfg"]["counts"].num() == 7 ? "taken" : "ignored"));
    w.ruleRightWay = false;
    reportRules(w);
}

SCENARIO(User::FieldFailures, "udp-garbage", "Random packets to the steering port (another device on the network) while steering", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    w.tractor.x = -0.3f;
    steerFor(w, 8, 2000);
    uint32_t s = 777;
    w.watchers.push_back([&] {
        if (w.now() % 5) return;
        std::vector<uint8_t> p(1 + (s >> 4) % 60);
        for (auto& b : p) { s = s * 1103515245u + 12345u; b = (uint8_t)(s >> 16); }
        if ((s >> 8) % 3 == 0) { p.resize(std::max<size_t>(p.size(), 6)); p[0] = 0x80; p[1] = 0x81; p[2] = 0x7F; }   // looks like a PGN
        if (p.size() > 3 && (p[3] == 0xFC || p[3] == 0xFB || p[3] == 201 || p[3] == 0xFE)) p[3] = 0x10;           // (those have their own scenarios)
        w.aow.send(p);
    });
    w.run(20000);
    w.watchers.pop_back();
    must("no-restart", w.aio.boots == 1, "Keeps running", fmt("%u starts", w.aio.boots));
    must("follows", fabsf(w.tractor.x) < 0.15f, "Keeps steering on the line", fmt("%.2f m off", fabsf(w.tractor.x)));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "gps-lost", "The receiver stops sending while driving (cable, power)", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 3000);
    const size_t mark = w.aio.logMark(), smark = w.aow.screen.size();
    w.rx.on = false;
    w.run(8000);
    tell("says-gps-lost", told(w, mark, smark, "GPS") || told(w, mark, smark, "GGA"), "Says the GPS data stopped", logSince(w, mark, 300));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "rtk-lost", "RTK fix drops to plain GPS while driving", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 3000);
    const size_t mark = w.aio.logMark();
    w.rx.fix = '1';
    w.run(8000);
    tell("says-rtk-lost", w.aio.logHas("fix 1", mark) || w.aio.logHas("GPS fix", mark) || w.aio.logHas("RTK", mark), "Says the RTK fix was lost", logSince(w, mark, 300));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "imu-frozen", "The IMU keeps sending the same roll and heading while driving", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 3000);
    const size_t smark = w.aow.screen.size();
    w.imu.frozen = true;
    w.run(6000);
    tell("says-frozen", w.aow.screenHas("IMU frozen", smark), "Says 'IMU frozen' on AgOpenWeb's screen", lastScreen(w));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "web-hammer", "A browser tab reloads the status 20 times a second while steering", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    w.tractor.x = -0.3f;
    steerFor(w, 8, 1000);
    w.aio.resetLoopMax();
    const uint64_t end = w.now() + 15000;
    int n = 0, ok = 0;
    while (w.now() < end)
    {
        const Http h = w.http("GET", "/api/status", "", {}, 2000);
        n++;
        if (h.code == 200 && h.jsonOk) ok++;
        w.run(50);
    }
    note("hammer", fmt("%d of %d status answers OK; longest loop %.1f ms", ok, n, w.aio.loopMaxUs / 1000.0));
    must("answers", ok >= n * 9 / 10, "Answers the page (9 in 10 at least)", fmt("%d of %d", ok, n));
    must("loop", w.aio.loopMaxUs < 50000, "The page does not hold the steering loop", fmt("%.1f ms", w.aio.loopMaxUs / 1000.0));
    must("follows", fabsf(w.tractor.x) < 0.15f, "Keeps steering on the line", fmt("%.2f m off", fabsf(w.tractor.x)));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "bad-requests", "Odd requests to the web page (too long, no header end, a huge Content-Length) while steering", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 1000);
    w.aio.resetLoopMax();
    const std::string longPath(5000, 'a');
    const Http a = w.httpRaw("GET /" + longPath + " HTTP/1.1\r\n\r\n", 5000);
    const Http b = w.httpRaw("GET /api/status HTTP/1.1\r\nHost: x\r\n", 5000);       // never ends its headers
    const Http c = w.httpRaw("POST /api/settings HTTP/1.1\r\nContent-Length: 99999999\r\n\r\nabc", 5000);
    const Http d = w.httpRaw("POST /api/note HTTP/1.1\r\nContent-Length: 4\r\n\r\n\x01\x02\x03\x04", 5000);
    static const char zeros[] = "\0\0\0\0GET / HTTP/1.1\r\n\r\n";
    const Http e = w.httpRaw(std::string(zeros, sizeof(zeros) - 1), 5000);
    note("bad-requests", fmt("long path %d, no header end %d, huge length %d, binary note %d, zeros %d", a.code, b.code, c.code, d.code, e.code));
    const Json st = w.status();
    must("page-after", st.has("fw"), "The page still works afterwards", w.lastBadStatus);
    must("loop", w.aio.loopMaxUs < 50000, "The steering loop is not held", fmt("%.1f ms", w.aio.loopMaxUs / 1000.0));
    must("no-restart", w.aio.boots == 1, "Keeps running", fmt("%u starts", w.aio.boots));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "ntrip-flood", "AgOpenWeb sends 60 KB/s of NTRIP corrections (more than the receiver's port takes)", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 1000);
    w.aio.resetLoopMax();
    std::vector<uint8_t> rtcm(500, 0xD3);
    w.watchers.push_back([&] { if (w.now() % 8 == 0) w.aow.send(rtcm, 2233); });
    w.run(10000);
    w.watchers.pop_back();
    note("ntrip", fmt("longest loop %.1f ms; %u bytes reached the receiver", w.aio.loopMaxUs / 1000.0, w.rx.bytesFromModule));
    must("loop", w.aio.loopMaxUs < 50000, "The corrections do not hold the steering loop", fmt("%.1f ms", w.aio.loopMaxUs / 1000.0));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "millis-wrap", "Runs through the 49.7-day millis() wrap while steering", { Kit::PumaCytron, Kit::Um982Keya })
{
    World w(kit);
    w.aio.dev.millisOffset = 0xFFFFFFFFu - 25000u;    // the wrap 25 s after power-on
    w.startUp();
    w.tractor.x = -0.3f;
    steerFor(w, 8, 30000);
    must("follows", fabsf(w.tractor.x) < 0.15f, "Keeps steering through the wrap", fmt("%.2f m off", fabsf(w.tractor.x)));
    must("no-dropout", w.engagedMs > 25000, "No dropout at the wrap", fmt("%llu ms of 30 s", (unsigned long long)w.engagedMs));
    engage(w, false);
    w.run(500);
    engage(w, true);
    w.run(3000);
    must("re-engage", w.engaged(), "Engages again after the wrap");
    w.aow.on = false;
    const uint64_t off = msUntilOff(w, 5000);
    must("watchdog-after-wrap", off <= 2600, "Stops within 2.6 s when AgOpenWeb goes silent after the wrap", fmt("%llu ms", (unsigned long long)off));
    reportRules(w);
}

// ---------------------------------------------------------------- firmware updates from the web page
static std::vector<uint8_t> repoHex()
{
    std::vector<uint8_t> v;
    const char* p = getenv("TEENSY_HEX");
    if (!p) return v;
    std::ifstream f(p, std::ios::binary);
    v.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return v;
}

SCENARIO(User::FieldFailures, "update-while-steering", "Starts a firmware update from the web page while autosteer steers", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    steerFor(w, 8, 2000);
    std::vector<uint8_t> file = repoHex();
    if (file.empty()) file.assign(200000, ':');
    Http h;
    w.ruleRightWay = false;                    // (the steering rule below already catches outputs left on)
    w.aio.resetLoopMax();
    const float wheels0 = w.tractor.wheelDeg;
    float moved = 0;
    uint32_t pushMs = 0;
    w.watchers.push_back([&] {
        moved = std::max(moved, fabsf(w.tractor.wheelDeg - wheels0));
        if (w.outputActive()) pushMs++;
    });
    w.upload(file, h);                         // 200 kB/s: a tablet's Wi-Fi
    w.watchers.pop_back();
    w.run(1000);
    note("update-refused", fmt("%zu kB sent in %u ms; HTTP %d; longest loop() %.0f ms; outputs on %u ms of it, the wheels moved up to %.1f deg",
                               file.size() / 1024, h.ms, h.code, w.aio.loopMaxUs / 1000.0, pushMs, moved));
    must("refused", h.code == 409, "Refused while steering", fmt("HTTP %d %s", h.code, h.body.substr(0, 100).c_str()));
    must("no-restart", w.aio.boots == 1, "No restart", fmt("%u starts", w.aio.boots));
    must("loop-not-held", w.aio.loopMaxUs < 50000, "The steering loop goes on while the refused file comes in",
         fmt("loop() held %.1f s with the outputs as they were (the file is read to its end before the answer)", w.aio.loopMaxUs / 1e6));
    note("steering", fmt("autosteer %s after the refusal", w.engaged() ? "still on" : "off"));
    reportRules(w);
}

SCENARIO(User::FieldFailures, "update-bad-files", "Uploads wrong files as a firmware update (a photo, a half file, another board's .hex)", { Kit::PumaCytron })
{
    World w(kit);
    w.startUp();
    std::vector<uint8_t> photo(300000);
    uint32_t s = 5;
    for (auto& b : photo) { s = s * 1103515245u + 12345u; b = (uint8_t)(s >> 16); }
    Http h;
    w.upload(photo, h);
    must("photo", h.code == 400 && w.aio.boots == 1, "A photo is refused, the firmware kept", fmt("HTTP %d %s", h.code, h.body.substr(0, 100).c_str()));
    std::vector<uint8_t> hex = repoHex();
    if (hex.size() > 100000)
    {
        std::vector<uint8_t> half(hex.begin(), hex.begin() + hex.size() / 2);
        w.upload(half, h);
        must("half", h.code == 400 && w.aio.boots == 1, "Half a .hex is refused, the firmware kept", fmt("HTTP %d %s", h.code, h.body.substr(0, 100).c_str()));
        // the same .hex with "fw_teensy41" changed (another board's build)
        std::string t(hex.begin(), hex.end());
        std::vector<uint8_t> other = hex;
        // a Teensy 4.0 build says fw_teensy40: the id is in the data bytes, as hex text "66775F7465656E73793431"
        const std::string id41 = "66775F7465656E73793431", id40 = "66775F7465656E73793430";
        size_t at = t.find(id41);
        if (at != std::string::npos)
        {
            for (size_t i = 0; i < id40.size(); i++) other[at + i] = (uint8_t)id40[i];
            w.upload(other, h);
            must("other-board", h.code == 400 && w.aio.boots == 1, "A .hex for another Teensy is refused (its line checksum is wrong too)",
                 fmt("HTTP %d %s", h.code, h.body.substr(0, 120).c_str()));
        }
        else note("other-board", "the id fw_teensy41 is not in one piece in the .hex lines: not tried");
    }
    const Json st = w.status();
    must("page-after", st.has("fw"), "The page still works", w.lastBadStatus);
    reportRules(w);
}

SCENARIO(User::FieldFailures, "update-and-back", "Updates with the repository's .hex, then goes back to the previous firmware", { Kit::PumaCytron })
{
    World w(kit);
    std::vector<uint8_t> hex = repoHex();
    if (hex.empty()) { note("update", "no .hex in the repository: not tried"); return; }
    w.startUp();
    Http h;
    w.upload(hex, h, "", 400);
    w.run(10000);
    note("update", fmt("HTTP %d %s, %u starts after %u ms", h.code, h.body.substr(0, 80).c_str(), w.aio.boots, h.ms));
    must("installed", w.aio.boots == 2, "The update installs and restarts", fmt("%u starts, HTTP %d %s", w.aio.boots, h.code, h.body.substr(0, 120).c_str()));
    must("prev-kept", w.aio.logHas("Previous firmware kept in the flash: v"), "Keeps the previous firmware (offered on the Firmware tab)", w.aio.logLine("Previous firmware"));
    const Http r = w.http("POST", "/api/revert", "");
    w.run(10000);
    must("reverted", w.aio.boots == 3, "Goes back to the previous firmware and restarts", fmt("HTTP %d %s, %u starts", r.code, r.body.substr(0, 100).c_str(), w.aio.boots));
    must("eeprom-kept", w.status()["steerCfg"]["counts"].num() == w.aow.ss.counts, "The steer settings survive both", fmt("counts %g", w.status()["steerCfg"]["counts"].num()));
    reportRules(w);
}

// ---------------------------------------------------------------- the WiFi bridge (a tablet on the XIAO's own WiFi)
SCENARIO(User::FieldFailures, "wifi-bridge-xiao-stops", "Steering through the XIAO's WiFi bridge, the XIAO stops (its plug, a restart), then comes back", { Kit::PumaCytron, Kit::F9pIbt2, Kit::Um982Keya })
{
    World w(kit);
    wifiTablet(w);
    w.startUp();
    w.tractor.kmh = 8;
    w.run(1000);
    engageWifi(w, true);
    w.run(8000);
    must("steering", w.engaged(), "Steers for the program on the XIAO's WiFi");
    const size_t mark = w.aio.logMark();
    const uint32_t boots = w.aio.boots;
    w.xiao.fitted = false;                          // nothing more from the XIAO
    const uint64_t off = msUntilOff(w, 5000);
    note("xiao-off", fmt("outputs off %llu ms after the XIAO's last byte", (unsigned long long)off));
    must("off-in-time", off <= 2600, "Outputs off within 2.6 s of the last steering packet", fmt("%llu ms", (unsigned long long)off));
    w.run(6000);
    must("stays-off", !w.engaged(), "Stays off while the XIAO is silent");
    tell("says-free", w.aio.logHas("the module is free again", mark), "Says the program on the XIAO's WiFi is gone", logSince(w, mark, 400));
    const uint32_t net0 = w.netTo9999;
    w.run(3000);
    must("gps-on-cable", w.netTo9999 - net0 >= 25, "GPS goes out on the network cable again", fmt("%u datagrams in 3 s", w.netTo9999 - net0));

    w.xiao.fitted = true;                           // the XIAO is back
    w.runUntil([&] { return w.xiao.state == 2; }, 8000);
    w.run(8000);
    must("back", w.xiao.state == 2 && !w.tab.heardPgn(253, w.now() - 2000).empty(), "With the XIAO back the program is answered again",
         fmt("bridge state %u, %zu PGN 253 in 2 s", w.xiao.state, w.tab.heardPgn(253, w.now() - 2000).size()));
    must("no-restart", w.aio.boots == boots, "No restart of the module", fmt("%u starts", w.aio.boots));
    engageWifi(w, false);
    w.run(1000);
    reportRules(w);
}

SCENARIO(User::FieldFailures, "wifi-bridge-tablet-leaves", "Steering through the XIAO's WiFi bridge, the tablet loses the XIAO's WiFi", { Kit::PumaCytron, Kit::F9pDanfoss })
{
    World w(kit);
    wifiTablet(w);
    w.tab.period254 = 10;                           // AgOpenWeb's 100 steering packets a second
    w.startUp();
    Json st = w.status();
    must("page-says", st["xiao"]["bridge"].num() == 2 && st["net"]["aogWifi"].num() == 1 && st["net"]["aogIp"].str() == "192.168.4.2",
         "The module's page says the program is on the XIAO's WiFi", fmt("bridge %g, program IP %s", st["xiao"]["bridge"].num(), st["net"]["aogIp"].str().c_str()));
    const uint32_t net0 = w.netTo9999;
    w.tractor.kmh = 8;
    w.run(1000);
    engageWifi(w, true);
    w.run(8000);
    must("steering", w.engaged(), "Steers for the program on the XIAO's WiFi");
    must("one-way", w.netTo9999 == net0, "Nothing for the program goes out on the network cable while it is on the WiFi (it would get everything twice)",
         fmt("%u datagrams to UDP 9999 on the cable in 9 s", w.netTo9999 - net0));
    st = w.status();
    must("none-lost", st["xiao"]["brDrop"].num() == 0 && w.xiao.dropped == 0, "At 100 steering packets a second nothing is dropped at the link, either way",
         fmt("to the program: %g dropped of %g; to the module: %u dropped of %u", st["xiao"]["brDrop"].num(), st["xiao"]["brOut"].num(), w.xiao.dropped, w.xiao.toModule));
    const size_t n253 = w.tab.heardPgn(253, w.now() - 5000).size();
    must("pgn253-100hz", n253 >= 480, "PGN 253 back for every steering packet (100 a second)", fmt("%zu in 5 s", n253));
    const uint32_t s141 = w.xiao.pgns[141];
    w.run(3000);
    must("page-data-goes-on", w.xiao.pgns[141] - s141 >= 20, "The module's status for the XIAO's page still gets through beside the program's packets",
         fmt("%u status frames in 3 s", w.xiao.pgns[141] - s141));

    const size_t mark = w.aio.logMark();
    w.tab.on = false;                               // out of the XIAO's range
    const uint64_t off = msUntilOff(w, 5000);
    must("off-in-time", off <= 2600, "Outputs off within 2.6 s of the last steering packet", fmt("%llu ms", (unsigned long long)off));
    w.run(4000);
    st = w.status();
    must("free", st["xiao"]["bridge"].num() == 0, "After 3 s of silence the module is free again", fmt("bridge %g", st["xiao"]["bridge"].num()));
    tell("says-free", w.aio.logHas("the module is free again", mark), "Says the program on the XIAO's WiFi went silent", logSince(w, mark, 400));
    const uint32_t net1 = w.netTo9999;
    w.run(3000);
    must("gps-on-cable", w.netTo9999 - net1 >= 25, "GPS goes out on the network cable again", fmt("%u datagrams in 3 s", w.netTo9999 - net1));
    reportRules(w);
}

// The module's page through the XIAO when the phone or the link fails, a tablet steering through the bridge all along
SCENARIO(User::FieldFailures, "page-through-xiao-phone-leaves", "The module's page through the XIAO: the phone walks away mid-page, a XIAO that stops taking, a piece lost on the link", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.cableIn = false;
    wifiTablet(w);
    w.startUp();
    w.tractor.kmh = 8;
    w.tractor.x = -0.5f;
    w.run(2000);
    engageWifi(w, true);
    w.run(8000);

    // a slow phone (2 kB/s) that leaves after 1.5 s: the XIAO says so, the next request is answered at once
    w.xiao.web.takePerMs = 2;
    const Http left = w.httpXiaoRaw("GET / HTTP/1.1\r\nHost: 192.168.5.1:8080\r\n\r\n", 1500);
    w.xiao.web.takePerMs = 4000;
    w.run(300);
    const Http next = w.httpXiao("GET", "/api/status", "", {}, 3000);
    must("after-leave", next.code == 200 && next.jsonOk, "After a phone left mid-page, the next request is answered", fmt("HTTP %d after %u ms", next.code, next.ms));
    note("left", fmt("the phone had %zu bytes of the page when it left", left.body.size()));

    // a XIAO that stops taking the answer and says nothing (its own trouble): the module gives up, the next one works
    w.xiao.web.takePerMs = 0;
    w.httpXiaoRaw("GET / HTTP/1.1\r\nHost: 192.168.5.1:8080\r\n\r\n", 2000, false);
    w.xiao.web.takePerMs = 4000;
    w.run(8000);
    const Http diag = w.httpXiao("GET", "/api/diag", "", {}, 3000);
    must("given-up", diag.jsonOk && diag.json["web"]["viaXiaoBroken"].num() >= 1, "An answer the XIAO stopped taking is given up (counted in the report)", diag.body.substr(0, 300));

    // a piece of a request lost on the link: no half request is served, the next one is
    const std::string text(400, 'n');
    w.xiao.web.dropPieceAt = 240;
    const Http lost = w.httpXiao("POST", "/api/note", text, {}, 2500);
    w.xiao.web.dropPieceAt = -1;
    must("lost-not-served", lost.code == 0, "A request with a piece lost on the link is not served", fmt("HTTP %d", lost.code));
    const Http note2 = w.httpXiao("GET", "/api/note", "", {}, 3000);
    must("lost-not-saved", note2.jsonOk && note2.json["note"].s != text, "... and its half is not saved", note2.body.substr(0, 120));
    must("still-steering", w.engaged() && fabsf(w.tractor.x) < 0.15f, "Steers on through all of it", fmt("%s, %.2f m off the line", w.engaged() ? "steering" : "not steering", w.tractor.x));
    reportRules(w);
}
