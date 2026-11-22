/* user_bad_link.cpp - a weak WiFi, a noisy serial link, browsers that stall or vanish, too much at once.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
#include <cmath>
using namespace h;

SCENARIO(User::BadLink, "link-phone-vanishes-mid-page", "A phone walks out of reach in the middle of loading the page, while the tablet steers through the bridge", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        Link gone;
        gone.ipLast = 21;
        gone.deafAt = 15000;                // it takes 15 kB of the page, then nothing more, without a word
        gone.patienceMs = 40000;
        const int a = w.ask(GET("/"), gone);
        w.run(500);
        const Reply other = w.get("/api/state");
        must("other-served", other.jsonOk, "Another phone's page is answered in the end", other.why);
        tell("other-wait", other.tookMs() < 3000, "... and does not wait for the XIAO to give the first phone up",
             fmt("it waited %.1f s (the XIAO serves one connection at a time, and gives a silent one up after 20 s)", other.tookMs() / 1000.0));
        w.run(3000);
        must("given-up", w.reply(a).failed || !w.reply(a).done, "(the first phone never got its page)", "");
        endSteer(w);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-spare-connection", "A browser opens a spare connection and leaves it silent", oneKit())
{
    World w(kit);
    w.start();
    Link spare;
    spare.neverAsks = true;
    spare.patienceMs = 30000;
    w.ask(GET("/api/state"), spare);
    w.run(200);
    const Reply r = w.get("/api/state");
    must("served", r.jsonOk && r.tookMs() < 2500, "Another request is answered about a second later: the silent connection gives way", fmt("%u ms", r.tookMs()));
    Link late;
    late.idleFirstMs = 3000;                // connects, and asks 3 s later (with nobody else waiting)
    const Reply l = w.http(GET("/api/state"), late);
    must("late", l.jsonOk, "A connection that asks only 3 s after it opened is answered too", l.why);
    rules(w);
}

SCENARIO(User::BadLink, "link-request-in-two-pieces", "A phone on a weak link saves the settings: the second part of its request arrives 3 s after the first, while the tablet steers", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        w.run(3000);
        const std::string req = POST("/api/settings", settings("trator-2026", w.knownSsid, "", true));
        Link weak;
        weak.ipLast = 21;
        weak.stallAt = req.find("\r\n\r\n") + 4;        // the head arrives, the form 3 s later (a lost packet sent again)
        weak.stallMs = 3000;
        const Reply r = w.http(req, weak);
        const uint32_t gap = w.rules.worstSteerGapMs;
        endSteer(w);
        if (gap >= 1000)
        {
            note("Not checked further", fmt("the XIAO waited for the rest of the request and did nothing else; the page got code %d (while the tablet steers, Save should be refused with 409: "
                                            "after 2 s without steering data the XIAO no longer knows that it steers)", r.code));
            return;
        }
        must("refused", r.code == 409 && has(r.error(), "autosteer is on"), "Saving is refused while the tablet steers (it would restart the XIAO)", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
        w.run(3000);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-page-wait-other-network-away", "The farm's WiFi is set but out of reach in the field; a phone's requests arrive in two pieces, 1.2 s apart, while the tablet steers", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.dev.router.on = false;                // the farm's WiFi does not reach the field
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        w.run(3000);
        must("paused", w.said("not searching for 'Farm' while a program runs") && !w.dev.staSearching(), "While the program has the module the XIAO does not search for the farm's WiFi", "");
        const uint32_t scans = w.dev.scans, begins = w.dev.staBegins;
        const std::string req = GET("/api/state");
        int answered = 0, asked = 0;
        // The XIAO looks at its WiFi twice a second, and first of all right after a long wait. So the requests are
        // timed to arrive, one after the other, at every millisecond around such a look: the wait for the second
        // piece then starts in the very pass of loop() that looks.
        for (int early = 12; early >= -3; early--)
        {
            Link weak;
            weak.ipLast = 21;
            weak.stallAt = 20;                          // the first 20 bytes arrive, the rest 1.2 s later (a lost packet sent again)
            weak.stallMs = 1200;                        // (the Teensy says once a second who has the module)
            asked++;
            if (w.http(req, weak).code == 200) answered++;
            w.run(1000 - early);
        }
        must("answered", answered == asked, "Each request is answered when its second piece arrives", fmt("%d of %d", answered, asked));
        must("no-search", w.dev.scans == scans && w.dev.staBegins == begins && !w.said("searching for 'Farm' again"),
             "After such a wait the XIAO still does not search for the farm's WiFi (a search takes the radio away from the tablet)",
             fmt("%u search(es) started%s", w.dev.staBegins - begins, w.said("searching for 'Farm' again") ? "; Messages: 'searching for 'Farm' again'" : ""));
        endSteer(w);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-request-never-finished", "A phone's request stops half way (it lost the WiFi), with no tablet steering", oneKit())
{
    World w(kit);
    w.start();
    const std::string req = POST("/api/restart", { { "pw", "agopengps" } });
    Link gone;
    gone.vanishAt = req.find("\r\n\r\n") + 4;           // the head arrives, the rest never
    gone.patienceMs = 20000;
    const int a = w.ask(req, gone);
    w.run(300);
    const Reply next = w.get("/api/state");
    must("next-served", next.jsonOk && next.tookMs() < 8000, "The next request is answered once the XIAO gives the first one up (5 s)", fmt("%u ms %s", next.tookMs(), next.why.c_str()));
    w.run(3000);
    must("no-restart", w.xiao.restarts.empty() && !w.reply(a).done, "The half request does nothing", "");
    Link half;
    half.vanishAt = 9;                                  // "POST /api" and nothing more
    half.patienceMs = 20000;
    w.ask(req, half);
    w.run(300);
    const Reply next2 = w.get("/api/state");
    must("next-served-2", next2.jsonOk && next2.tookMs() < 8000, "The same with only half a first line", fmt("%u ms", next2.tookMs()));
    std::string how;
    const bool alone = linkLeftAlone(w, how);
    must("link-kept", !alone, "While it waits for the rest of such a request, the XIAO keeps the link to the Teensy going", how);
    if (alone) { note("Not checked further", fmt("the XIAO waited %u ms in one go for the rest of the request; the rules at the end would say the same again", w.xiao.longestPassMs)); return; }
    rules(w);
}

SCENARIO(User::BadLink, "link-update-stalls", "The XIAO's own update over a weak link: the upload stops for 18 s in the middle, then goes on", bothKits())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    Link weak;
    weak.upPerMs = 100;
    weak.stallAt = 300000;
    weak.stallMs = 18000;
    weak.patienceMs = 120000;
    const int boots = w.xiao.boots;
    const Reply r = w.upload("/update?pw=agopengps", "update.bin", bin, weak);
    const bool wdt = !w.xiao.restarts.empty() && w.xiao.restarts[0] == ESP_RST_TASK_WDT;
    must("no-watchdog", !wdt, "The XIAO waits for the phone (up to 20 s) without its own watchdog restarting it",
         "restarted by its watchdog after 15 s in the wait: the page then shows 'program stopped (watchdog)' and the update is lost");
    if (wdt) { note("Not checked further", "what follows (no answer to the page, the Teensy calling the XIAO silent, its data lost) comes from that restart"); return; }
    must("done", r.code == 200 || (r.code == 400 && w.said("Program update STOPPED")), "The update goes through, or stops with a message that says why", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
    if (r.code == 200) { w.noteUserRestart(); backAfterRestart(w, boots); }
    w.run(4000);
    must("runs", w.state()["ver"].str() == "4.5", "Afterwards the XIAO runs and its page answers", "");
    rules(w);
}

SCENARIO(User::BadLink, "link-update-phone-vanishes", "The phone walks out of reach in the middle of the XIAO's own update (nothing closes the connection)", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    Link gone;
    gone.upPerMs = 200;
    gone.vanishAt = 400000;
    gone.patienceMs = 40000;
    w.ask(UPLOAD("/update?pw=agopengps", "update.bin", bin), gone);
    w.run(30000);
    const bool wdt = !w.xiao.restarts.empty() && w.xiao.restarts[0] == ESP_RST_TASK_WDT;
    must("no-watchdog", !wdt, "The XIAO gives the upload up without its own watchdog restarting it",
         "restarted by its watchdog 15 s after the phone went (it waits 20 s for the phone, the watchdog allows 15)");
    if (wdt) { note("Not checked further", "what follows (nothing in Messages, the Teensy calling the XIAO silent, its data lost) comes from that restart"); return; }
    must("said", w.said("Program update STOPPED"), "Messages say that the update stopped because the connection dropped, old program kept", "");
    must("kept", w.dev.updInstalled == 0 && w.state()["ver"].str() == "4.5" && !w.dev.updDone, "The old program keeps running", "");
    const int boots = w.xiao.boots;
    const Reply again = w.upload("/update?pw=agopengps", "update.bin", bin);
    w.noteUserRestart();
    must("again", again.code == 200 && backAfterRestart(w, boots), "A second try on a good link goes through", fmt("code %d '%s'", again.code, again.error().c_str()));
    rules(w);
}

// (In the two that follow the program is inside one loop() pass for minutes, waiting for the phone: what the scenario
// looks at in the middle of that wait is written down by w.later().)
SCENARIO(User::BadLink, "link-upload-drips-while-steering", "While the tablet steers through the bridge, a phone's upload gets through a few bytes at a time, for two minutes", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (!beginSteer(w)) { rules(w); return; }
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    Link drip;
    drip.ipLast = 21;
    drip.dripAt = 20000;                    // the first 20 kB go well, then 10 bytes every 2 s: no piece of the file is ever whole
    drip.dripBytes = 10;
    drip.dripEveryMs = 2000;
    drip.patienceMs = 600000;
    w.ask(UPLOAD("/update?pw=trator-2026", "update.bin", bin), drip);
    // after two minutes the tablet stops steering (the upload still drips)
    uint32_t gap = 0, offs = 0;
    size_t restartsThen = 0;
    w.later(120000, [&]() {
        gap = w.rules.worstSteerGapMs; offs = w.teensy.steerOffs; restartsThen = w.xiao.restarts.size();
        w.steerWatch_ = false;
        w.tab.engaged = false;
    });
    const int boots = w.xiao.boots;
    w.run(121000);                          // (comes back when that loop() pass ends)
    must("no-restart", restartsThen == 0, "The XIAO does not restart while the tablet steers: its page is stuck on that phone, the bridge is not", fmt("%u restart(s)", (unsigned)restartsThen));
    must("steer-gap", gap < 1000 && offs == 0, "Steering data kept reaching the Teensy for those two minutes",
         fmt("longest time without it %u ms; the Teensy switched autosteer off %u time(s)", gap, offs));
    note("Tablet to Teensy through the bridge, the page stuck on a phone", fmt("%u of %u packets passed, longest delay %u ms, longest gap in the steering data %u ms",
                                                                              w.tab.nPassed, w.tab.nSent, w.tab.worstDelayMs, gap));
    if (restartsThen || gap >= 1000) { note("Not checked further", "the rules at the end would say the same again"); return; }
    // the steering is over and the page is still stuck on that phone: now the watchdog may clear it
    w.watchdogExpected = 1;
    const bool back = backAfterRestart(w, boots, 30000);
    must("unstuck", back && w.xiao.restarts.size() == 1 && w.xiao.restarts[0] == ESP_RST_TASK_WDT,
         "When the steering is over, the XIAO's watchdog restarts it (within 30 s), so its page is not stuck for ever",
         w.xiao.restarts.empty() ? "no restart" : reasonText(w.xiao.restarts.back()));
    w.run(4000);
    const JsonDoc st = w.state();
    must("page-back", st["ver"].str() == "4.5" && has(st["reset"].str(), "watchdog"), "... its page answers again and says why it restarted", st["reset"].str());
    rules(w);
}

SCENARIO(User::BadLink, "link-upload-drips", "A phone's upload gets through a few bytes at a time and never ends (no tablet steering)", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    Link drip;
    drip.dripAt = 20000;
    drip.dripBytes = 10;
    drip.dripEveryMs = 2000;
    drip.patienceMs = 600000;
    w.ask(UPLOAD("/update?pw=agopengps", "update.bin", bin), drip);
    size_t restartsThen = 0;
    const uint64_t t0 = fake::clockMs;
    w.later(50000, [&]() { restartsThen = w.xiao.restarts.size(); });
    const int boots = w.xiao.boots;
    w.watchdogExpected = 1;
    const bool back = backAfterRestart(w, boots, 100000);
    const uint32_t backS = (uint32_t)((fake::clockMs - t0) / 1000);
    std::string how;
    must("link-kept", restartsThen == 0 && !linkLeftAlone(w, how), "For the first 50 s the XIAO waits for the phone with the link to the Teensy kept, and does not restart",
         restartsThen ? std::string("it restarted: ") + reasonText(w.xiao.restarts[0]) : how);
    must("unstuck", back && w.xiao.restarts.size() == 1 && w.xiao.restarts[0] == ESP_RST_TASK_WDT && backS < 90,
         "After over a minute stuck on that phone, the XIAO's watchdog restarts it (the page served nobody else meanwhile)",
         w.xiao.restarts.empty() ? "no restart in 100 s" : fmt("%s, running again %u s after the upload began", reasonText(w.xiao.restarts.back()), backS));
    w.run(4000);
    const JsonDoc st = w.state();
    must("page-back", st["ver"].str() == "4.5" && has(st["reset"].str(), "watchdog") && w.dev.updInstalled == 0, "... its page answers again, says why it restarted, and the old program runs", st["reset"].str());
    rules(w);
}

SCENARIO(User::BadLink, "link-update-tab-closed", "The browser tab is closed in the middle of an update (the XIAO's own, then the Teensy's)", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    Link closed;
    closed.upPerMs = 500;
    closed.closeAt = 300000;
    w.ask(UPLOAD("/update?pw=agopengps", "update.bin", bin), closed);
    w.run(4000);
    must("said", w.said("Program update STOPPED after 29"), "The XIAO's own: Messages say it stopped and how far it got (about 290 kB)", "");
    must("kept", w.xiao.restarts.empty() && !w.dev.updDone, "... the old program is kept, with no restart", "");
    const std::vector<uint8_t> hex = makeHex(300000);
    Link closed2;
    closed2.upPerMs = 500;
    closed2.closeAt = 100000;
    w.ask(UPLOAD(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex), closed2);
    w.run(12000);
    must("teensy-said", w.said("Module firmware update STOPPED"), "The Teensy's: Messages say it stopped", "");
    must("teensy-told", w.teensy.fwRefused == 1 && has(w.teensy.fwWhy, "cancelled from the XIAO page") && w.teensy.fwDone == 0, "... and the Teensy is told to drop what it got (its firmware is kept)", w.teensy.fwWhy);
    w.teensy.fwExpect = hex;
    const Reply again = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    must("again", again.code == 200 && w.teensy.fwDone == 1, "A second try goes through", fmt("code %d '%s'", again.code, again.error().c_str()));
    w.runUntil([&]() { return w.teensy.on && w.teensy.detected; }, 15000);
    rules(w);
}

SCENARIO(User::BadLink, "link-serial-noise", "The serial link to the Teensy is noisy (one byte in 2000 arrives wrong) while a tablet steers and a phone watches", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    w.tab.rtcmEveryMs = 1000;
    if (beginSteer(w))
    {
        w.noise = 0.0005;
        int ok = 0;
        for (int s = 0; s < 40; s++)
        {
            w.run(1000);
            ok += w.get("/api/state").jsonOk + w.get("/api/teensy").jsonOk + w.get("/api/log?after=0&xafter=0").jsonOk;
        }
        const JsonDoc st = w.state();
        must("page", ok == 120, "Every answer to the phone is valid (a damaged status never reaches the page)", fmt("%d of 120", ok));
        must("counted", st["link"]["crcErr"].num() + st["link"]["cut"].num() > 5 && st["link"]["talking"].num() == 1, "The page counts the damaged frames, and still shows the module", fmt("%g bad checksums", st["link"]["crcErr"].num()));
        must("status-fresh", w.get("/api/teensy").json["ago"].num() <= 6, "The module's status on the page stays fresh (some sets are lost, the next one comes)", "");
        w.noise = 0;
        w.run(2000);
        endSteer(w);
        note("With that noise", fmt("%u of %u of the tablet's packets reached the Teensy; the Teensy dropped %u damaged frames, the XIAO %g; status sets broken %g",
                                    w.tab.nPassed, w.tab.nSent, w.teensy.crcErr, st["link"]["crcErr"].num(), st["link"]["broken"].num()));
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-teensy-update-lost-answer", "The Teensy's firmware over the link: one answer is lost; another time the Teensy stops answering", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> hex = makeHex(120000);
    w.teensy.fwExpect = hex;
    w.teensy.fwLoseAnswer = 37;
    Reply r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    must("retried", r.code == 200 && w.teensy.fwDone == 1, "One lost answer: the block is sent again and the file arrives whole", fmt("code %d '%s' / %s", r.code, r.error().c_str(), w.teensy.fwWhy.c_str()));
    w.runUntil([&]() { return w.teensy.on && w.teensy.detected; }, 15000);
    w.run(3000);
    w.teensy.fwStopAt = 100;
    r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    must("stopped", r.code == 400 && has(r.error(), "stopped answering during the transfer"), "The Teensy stops answering: the page is told so", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
    must("stopped-fast", r.tookMs() < 12000, "... within a few seconds (two tries of 3 s), not at the end of the upload", fmt("%.1f s", r.tookMs() / 1000.0));
    must("stopped-said", w.said("Module firmware update REFUSED"), "... and Messages say so", "");
    w.teensy.fwStopAt = 0;
    w.run(3000);
    r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    tell("retry-soon", r.code == 200 || has(r.error(), "wait") || has(r.error(), "still"), "A new try 3 s later goes through, or is told to wait",
         "it is told '" + r.error() + "': the Teensy holds on to the first transfer for 15 s and says nothing meanwhile");
    if (r.code != 200)
    {
        w.run(16000);                       // the Teensy gives a transfer left half way up after 15 s
        r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    }
    must("then-good", r.code == 200 && w.teensy.fwDone == 2, "A new try 20 s later goes through", fmt("code %d '%s'", r.code, r.error().c_str()));
    w.runUntil([&]() { return w.teensy.on && w.teensy.detected; }, 15000);
    rules(w);
}

SCENARIO(User::BadLink, "link-teensy-refuses-update", "A .hex for the Teensy is sent while its autosteer is engaged, or the tractor moves (program on the cable)", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> hex = makeHex(400000);
    w.teensy.engaged = true;
    Link slow;
    slow.upPerMs = 100;                     // 100 kB a second: the file takes 4 s
    Reply r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex, slow);
    must("refused", r.code == 400 && has(r.error(), "autosteer is engaged"), "The Teensy refuses, and the page shows its reason", fmt("code %d '%s'", r.code, r.error().c_str()));
    std::string how;
    const bool alone = linkLeftAlone(w, how);
    must("link-kept", !alone, "While the rest of the refused file comes in (4 s), the XIAO keeps the link to the Teensy going", how);
    if (alone) { note("Not checked further", "the rules at the end would say the same again"); return; }
    w.teensy.engaged = false;
    w.teensy.moving = true;
    r = w.post("/api/teensy-revert", { { "pw", "agopengps" } });
    must("revert-refused", r.code == 409 && has(r.error(), "moving") && r.tookMs() < 1500, "The previous firmware back while moving: refused with the reason, at once", fmt("code %d '%s' in %u ms", r.code, r.error().c_str(), r.tookMs()));
    must("kept", w.teensy.fwDone == 0 && w.teensy.reverts == 0 && w.teensy.restarts == 0, "The Teensy keeps its firmware and does not restart", "");
    rules(w);
}

SCENARIO(User::BadLink, "link-hex-while-steering", "While the tablet steers through the bridge, someone sends a .hex for the Teensy from the XIAO page", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        w.run(3000);
        const std::vector<uint8_t> hex = makeHex(600000);
        Link phone;
        phone.ipLast = 21;
        phone.upPerMs = 150;                // 150 kB a second: the file takes 4 s (a real .hex is 2 MB)
        const Reply r = w.upload(fmt("/api/teensy-update?pw=trator-2026&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex, phone);
        must("refused", r.code == 400 && has(r.error(), "autosteer is on"), "It is refused: autosteer is on", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
        must("teensy-untouched", w.teensy.fwRefused == 0 && w.teensy.fwDone == 0, "Nothing of it reaches the Teensy", "");
        w.run(3000);
        const uint32_t gap = w.rules.worstSteerGapMs;
        endSteer(w);
        if (gap >= 1000) { note("Not checked further", "while the refused file came in (4 s) the XIAO did nothing else: no bridge, no hello, the Teensy's data unread"); return; }
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-wrong-password-big-file", "With no tablet steering: a big file is sent to the Teensy update with a wrong password", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> hex = makeHex(1500000);
    Link phone;
    phone.upPerMs = 100;                    // 15 s
    phone.patienceMs = 90000;
    const Reply r = w.upload(fmt("/api/teensy-update?pw=guess&force=1&size=%u", (unsigned)hex.size()), "fw.hex", hex, phone);
    must("refused", r.code == 403, "It is refused", fmt("code %d %s", r.code, r.why.c_str()));
    must("teensy-untouched", w.teensy.fwRefused == 0 && w.teensy.fwDone == 0, "Nothing of it reaches the Teensy", "");
    std::string how;
    const bool alone = linkLeftAlone(w, how);
    must("link-kept", !alone, "While the refused file comes in (15 s), the XIAO keeps the link to the Teensy going", how);
    if (alone) { note("Not checked further", "the rules at the end would say the same again"); return; }
    rules(w);
}

SCENARIO(User::BadLink, "link-own-update-while-steering", "While the tablet steers through the bridge: the XIAO's own update, Save, Restart, the Teensy's previous firmware", bothKits())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        w.run(3000);
        Link phone;
        phone.ipLast = 21;
        phone.upPerMs = 300;
        const Reply u = w.upload("/update?pw=trator-2026", "update.bin", repoFile(info(kit).bin), phone);
        must("update-refused", u.code == 400 && has(u.error(), "autosteer is on") && !w.dev.updDone, "The XIAO's own update is refused", fmt("code %d '%s' %s", u.code, u.error().c_str(), u.why.c_str()));
        const Reply s = w.post("/api/settings", settings("trator-2026", "Other name", "", true), phone);
        must("save-refused", s.code == 409 && has(s.error(), "Save: not now"), "Save is refused, with the reason", fmt("code %d '%s'", s.code, s.error().c_str()));
        const Reply rs = w.post("/api/restart", { { "pw", "trator-2026" } }, phone);
        must("restart-refused", rs.code == 409 && has(rs.error(), "Restart: not now"), "Restart is refused, with the reason", fmt("code %d '%s'", rs.code, rs.error().c_str()));
        const Reply rv = w.post("/api/teensy-revert", { { "pw", "trator-2026" } }, phone);
        must("revert-refused", rv.code == 409 && w.teensy.reverts == 0 && rv.tookMs() < 1500, "The Teensy's previous firmware back is refused (by the Teensy), at once", fmt("code %d '%s' in %u ms", rv.code, rv.error().c_str(), rv.tookMs()));
        const Reply sn = w.post("/api/subnet", { { "subnet", "192.168.5" }, { "pw", "trator-2026" } }, phone);
        w.run(1000);
        must("subnet-refused", w.teensy.subnets.empty() && w.teensy.restarts == 0, "A new subnet for the module is not taken by the Teensy", fmt("the page got code %d", sn.code));
        w.run(2000);
        must("nothing", w.dev.flashWrites == 0 && w.xiao.restarts.empty(), "Nothing was saved and nothing restarted", "");
        endSteer(w);
        w.run(2500);                        // autosteer off for 2 s: now it may
        const int boots = w.xiao.boots;
        const Reply rs2 = w.post("/api/restart", { { "pw", "trator-2026" } }, phone);
        w.noteUserRestart();
        must("restart-after", rs2.code == 200 && backAfterRestart(w, boots, 4000), "With autosteer off for 2 s, Restart is accepted", fmt("code %d '%s'", rs2.code, rs2.error().c_str()));
        w.run(3000);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-flood", "Something floods the bridge's port with packets (5000 a second on top of the tablet's own)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    w.tab.on = true;
    w.run(4000);
    w.tab.extraPerMs = 5;
    int ok = 0;
    uint32_t slowest = 0;
    for (int s = 0; s < 10; s++)
    {
        w.run(1000);
        const Reply r = w.get("/api/state");
        ok += r.jsonOk;
        if (r.tookMs() > slowest) slowest = r.tookMs();
    }
    w.tab.extraPerMs = 0;
    must("page", ok == 10 && slowest < 2000, "The page keeps answering during the flood", fmt("%d of 10, slowest %u ms", ok, slowest));
    w.run(3000);
    const JsonDoc st = w.state();
    must("counted", st["bridge"]["lost"].num() > 1000, "What the link to the Teensy could not carry is counted as lost on the page (the XIAO never waits for the link)", fmt("lost %g", st["bridge"]["lost"].num()));
    const uint32_t passed = w.tab.nPassed;
    w.run(3000);
    must("after", w.tab.nPassed - passed > 300 && w.teensy.brState == 2, "When the flood stops the tablet's packets pass as before", fmt("%u in 3 s", w.tab.nPassed - passed));
    note("During the flood", fmt("%u of %u packets reached the Teensy, longest loop() pass %u ms", w.tab.nPassed, w.tab.nSent, w.xiao.longestPassMs));
    rules(w);
}

SCENARIO(User::BadLink, "link-wifi-takes-nothing", "The WiFi takes no packet for 3 s (its queue is full) while the tablet steers", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.run(3000);
        const uint32_t got = w.tab.got;
        w.dev.udpJam = true;
        w.run(3000);
        w.dev.udpJam = false;
        must("jam", w.tab.got == got && w.dev.udpSendFails > 300, "(nothing reaches the tablet meanwhile)", fmt("%u sends failed", w.dev.udpSendFails));
        w.run(3000);
        const JsonDoc st = w.state();
        must("counted", st["bridge"]["lost"].num() > 300, "The page counts what the WiFi did not take", fmt("lost %g", st["bridge"]["lost"].num()));
        must("flows-again", w.tab.got > got + 300, "Afterwards the Teensy's answers reach the tablet again", fmt("%u since", w.tab.got - got));
        endSteer(w);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-many-phones", "Five phones keep the page open (three requests a second each) while the tablet steers", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 5;
    w.start();
    if (beginSteer(w))
    {
        std::vector<int> ids;
        Link spare;
        spare.neverAsks = true;
        spare.patienceMs = 25000;
        for (int t = 0; t < 200; t++)       // 20 s, in steps of 100 ms: each phone's three requests are spread over its second
        {
            for (int p = 0; p < 5; p++)
            {
                Link l;
                l.ipLast = (uint8_t)(20 + p);
                l.downPerMs = p == 4 ? 20 : 2000;       // one of them on a weak link
                l.patienceMs = 15000;
                const int at = (t + 2 * p) % 10;
                if (at == 0) ids.push_back(w.ask(GET("/api/state"), l));
                if (at == 3) ids.push_back(w.ask(GET("/api/teensy"), l));
                if (at == 6) ids.push_back(w.ask(GET("/api/log?after=0&xafter=0"), l));
            }
            if (t % 50 == 0) w.ask(GET("/api/state"), spare);       // and spare connections left silent
            w.run(100);
        }
        w.run(16000);
        int ok = 0;
        for (int id : ids) ok += w.reply(id).jsonOk;
        tell("answered", ok >= (int)ids.size() * 9 / 10, "At least nine in ten of their requests are answered", fmt("%d of %u (one connection is served at a time)", ok, (unsigned)ids.size()));
        must("some", ok > (int)ids.size() / 3, "(the page does keep answering)", fmt("%d of %u", ok, (unsigned)ids.size()));
        endSteer(w);
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-odd-requests", "Things that are no good requests arrive at the page's port", oneKit())
{
    World w(kit);
    w.start();
    const Reply longUrl = w.http("GET /" + std::string(9000, 'a') + " HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n");
    must("long-url", longUrl.code == 414 || longUrl.failed, "A 9000-character address is turned away", fmt("code %d %s", longUrl.code, longUrl.why.c_str()));
    const Reply longLine = w.http("GET / HTTP/1.1\r\nHost: 192.168.4.1\r\nCookie: " + std::string(5000, 'c') + "\r\n\r\n");
    must("long-header", longLine.code == 431 || longLine.failed, "A header line of 5000 characters is turned away", fmt("code %d %s", longLine.code, longLine.why.c_str()));
    std::string many = "GET /api/state HTTP/1.1\r\nHost: 192.168.4.1\r\n";
    for (int i = 0; i < 300; i++) many += fmt("X-Header-%d: %s\r\n", i, std::string(60, 'v').c_str());
    const Reply big = w.http(many + "\r\n");
    must("many-headers", big.done && (big.jsonOk || big.code >= 400), "A request with 300 header lines gets a well formed answer", fmt("code %d %s", big.code, big.why.c_str()));
    Link quick;
    quick.patienceMs = 8000;
    const Reply brew = w.http("BREW /coffee HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n", quick);
    must("method", brew.failed || brew.code >= 400, "An unknown method gets no page", fmt("code %d", brew.code));
    std::string bin;
    for (int i = 0; i < 700; i++) bin += (char)(i * 37 + 11);
    const Reply junk = w.http(bin, quick);
    must("junk", junk.failed || junk.code >= 400, "Bytes that are no request get no page", fmt("code %d", junk.code));
    const Reply noLen = w.http("POST /api/restart HTTP/1.1\r\nHost: 192.168.4.1\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n", quick);
    must("no-length", noLen.code == 403 || noLen.failed || noLen.code >= 400, "A POST with no form and no password restarts nothing", fmt("code %d", noLen.code));
    const Reply up = w.http("POST /update?pw=agopengps HTTP/1.1\r\nHost: 192.168.4.1\r\nContent-Type: multipart/form-data; boundary=xx\r\nContent-Length: 30\r\n\r\n--xx--\r\n", quick);
    must("empty-upload", up.failed || up.code == 400, "An upload with no file in it is refused", fmt("code %d '%s'", up.code, up.error().c_str()));
    const Reply q = w.get("/api/log?after=99999999999999999999&xafter=-5");
    must("odd-numbers", q.jsonOk, "Numbers that make no sense in an address get a valid answer", q.body.substr(0, 80));
    w.run(6000);
    must("alive", w.xiao.restarts.empty() && w.state()["ver"].str() == "4.5", "After all of that the XIAO runs as before, with no restart", "");
    rules(w);
}

SCENARIO(User::BadLink, "link-sensor-and-clock", "The chip's temperature sensor gives no number; the XIAO's clock passes its 49-day wrap while a tablet steers", bothKits())
{
    World w(kit);
    w.bridgeSaved();
    w.dev.tempC = NAN;
    w.dev.millisOffset = 0xFFFFFFFFu - 20000;       // 20 s before its millisecond counter starts again at 0
    w.start();
    const Reply st0 = w.get("/api/state");
    must("temp", st0.jsonOk && st0.json.has("temp") && st0.json["temp"].type == JsonDoc::Null, "With no temperature the page gets 'null' (valid JSON), not 'nan'", st0.body.substr(0, 200));
    bool ok = false;
    const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
    must("temp-info", ok && inf.has("t") && inf["t"].type == JsonDoc::Null, "... and so does the Teensy's page", w.teensy.info);
    if (beginSteer(w))
    {
        int good = 0;
        for (int s = 0; s < 40; s++) { w.run(1000); good += w.get("/api/state").jsonOk; }
        must("page", good == 40, "The page answers all the way through the wrap", fmt("%d of 40", good));
        must("no-false-silence", !w.said("is silent") && w.teensy.brState == 2, "The bridge never thinks the program went silent", "");
        endSteer(w);
        const JsonDoc st = w.state();
        must("fresh", st["up"].num() < 30 && st["link"]["talking"].num() == 1 && st["link"]["lastAgo"].num() <= 1 && st["aog"]["helloAgo"].num() <= 2,
             "After the wrap the page's ages are right (seconds, not 49 days)", fmt("up %g, last frame %g s ago, hello %g s ago", st["up"].num(), st["link"]["lastAgo"].num(), st["aog"]["helloAgo"].num()));
    }
    rules(w);
}

SCENARIO(User::BadLink, "link-restart-at-clock-wrap", "Restart is pressed half a second before the XIAO's clock wraps", oneKit())
{
    World w(kit);
    w.dev.millisOffset = 0xFFFFFFFFu - 3600;
    w.start();
    const int boots = w.xiao.boots;
    const Reply r = w.post("/api/restart", { { "pw", "agopengps" } });
    w.noteUserRestart();
    must("restarts", r.code == 200 && backAfterRestart(w, boots, 4000), "It restarts a second later all the same", fmt("code %d", r.code));
    w.dev.millisOffset = 0;
    w.run(4000);
    rules(w);
}
