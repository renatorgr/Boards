/* user_careful.cpp - a user who does everything right: every scenario here must pass.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
using namespace h;

SCENARIO(User::Careful, "careful-first-start", "A new XIAO is switched on with the Teensy, and a phone opens its page", bothKits())
{
    World w(kit);
    w.start();
    must("ap", w.dev.apOn && w.dev.apSsid == w.knownSsid && w.dev.apPass == "agopengps", "Its own network is up: XIAO-xxxx, password agopengps",
         w.dev.apSsid + " / " + w.dev.apPass);
    must("hello", w.teensy.detected && w.teensy.hellos >= 1, "The Teensy heard its hello", fmt("%u hellos", w.teensy.hellos));
    bool ok = false;
    const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
    must("info", ok && inf["v"].str() == "4.5" && inf["ap"].str() == w.knownSsid && inf["apIp"].str() == "192.168.4.1" && inf["br"].num() == 0,
         "The Teensy got its info (version, network, bridge off) as valid JSON", w.teensy.info);
    must("info-chip", inf["c"].str() == (kit == Kit::S3 ? "ESP32-S3" : "ESP32-C3"), "... with the right chip name", inf["c"].str());
    bool card = false, cardV = false, cardPath = false, http = false;
    for (auto& s : w.dev.mdnsServices) { card |= s == "_aowcard._tcp:80"; cardV |= s == "_aowcard._tcp v=1"; cardPath |= s == "_aowcard._tcp path=/"; http |= s == "_http._tcp:80"; }
    must("mdns", w.dev.mdnsOn && card && cardV && cardPath && http && w.dev.mdnsInstance == w.knownSsid,
         "It announces its page for AgOpenWeb (_aowcard._tcp with v=1 and path=/, and _http._tcp)", fmt("%u records, name '%s'", (unsigned)w.dev.mdnsServices.size(), w.dev.mdnsInstance.c_str()));
    must("no-bridge", w.dev.udpOpen() == 0, "The WiFi bridge is off from new: no UDP port is open", fmt("%u sockets", (unsigned)w.dev.udp.size()));

    Reply page = w.get("/");
    must("page", page.done && page.code == 200 && has(page.hdr("content-type"), "text/html") && page.body.size() > 50000 && has(page.body, "</html>"),
         "The page arrives whole", fmt("code %d, %u bytes, %s", page.code, (unsigned)page.body.size(), page.why.c_str()));
    must("page-etag", !page.hdr("etag").empty() && page.hdr("cache-control") == "no-cache", "... with its ETag, to be asked for again", page.hdr("etag"));
    Reply again = w.http(GET("/", "192.168.4.1", "If-None-Match: " + page.hdr("etag") + "\r\n"));
    must("page-304", again.done && again.code == 304 && again.body.empty(), "A second visit gets '304 Not Modified' (the page does not travel again)",
         fmt("code %d, %u bytes", again.code, (unsigned)again.body.size()));
    const JsonDoc st = w.state();
    must("state", st["ver"].str() == "4.5" && st["reset"].str() == "power on" && st["resetBad"].num() == 0 && st["link"]["talking"].num() == 1 && st["wifi"]["apCl"].num() == 1,
         "The Status tab's data: version, 'power on', the Teensy talking, one device on the network",
         fmt("ver %s, reset %s, talking %g, clients %g", st["ver"].str().c_str(), st["reset"].str().c_str(), st["link"]["talking"].num(), st["wifi"]["apCl"].num()));
    must("state-chip", st["chipId"].num() == info(kit).chip && st["fw"].str() == (kit == Kit::S3 ? "v4.5_XIAO_ESP32S3" : "v4.5_XIAO_ESP32C3"),
         "It names itself after its chip", st["fw"].str());
    must("said", w.said("started (power on)") && w.said("WiFi: own network"), "Its Messages say it started and which network it made", "");
    rules(w);
}

SCENARIO(User::Careful, "careful-status-and-messages", "The phone keeps the Status and Messages tabs open for a minute", oneKit())
{
    World w(kit);
    w.start();
    for (int i = 0; i < 25; i++) w.teensy.log(fmt("[status] line %d of the module's log, with 'quotes', a \\ and some text to make it a realistic length for the page", i));
    uint32_t after = 0, xafter = 0, lines = 0, events = 0, lastN = 0, behind = 0, asked = 0;
    bool inOrder = true, sFresh = true;
    for (int s = 0; s < 60; s++)
    {
        w.run(1000);
        if (s == 20) for (int i = 0; i < 70; i++) w.teensy.log(fmt("[status] burst line %d", i));
        const Reply t = w.get("/api/teensy");
        asked++;
        if (!t.jsonOk || t.json["ago"].num() < 0 || t.json["ago"].num() > 2) sFresh = false;
        const uint32_t n = (uint32_t)t.json["s"]["n"].num();
        if (n < lastN) inOrder = false;
        if (w.teensy.statusN > n + 2) behind++;
        lastN = n;
        for (int more = 0; more < 8; more++)
        {
            const Reply l = w.get(fmt("/api/log?after=%u&xafter=%u", after, xafter));
            asked++;
            if (!l.jsonOk) break;
            for (size_t i = 0; i < l.json["t"].size(); i++)
            {
                const uint32_t q = (uint32_t)l.json["t"][i]["s"].num();
                if (q <= after) inOrder = false;
                after = q;
                lines++;
            }
            for (size_t i = 0; i < l.json["x"].size(); i++) { xafter = (uint32_t)l.json["x"][i]["s"].num(); events++; }
            if (l.json["more"].num() == 0) break;
        }
        w.state();
        asked++;
    }
    must("status-fresh", sFresh && lastN > 55 && behind == 0, "The module's status on the page is never more than 2 s old", fmt("last number %u of %u, behind %u times", lastN, w.teensy.statusN, behind));
    must("log-lines", inOrder && after + 1 == w.teensy.nextSeq, "The module's log lines come in order, up to the last one", fmt("%u lines read, last %u, the Teensy's last %u", lines, after, w.teensy.nextSeq - 1));
    must("log-burst", lines >= 25 + 60, "A burst of 70 lines: the page gets the 60 the XIAO keeps", fmt("%u lines read", lines));
    must("events", events >= 2, "The XIAO's own events are in the same answer", fmt("%u", events));
    const JsonDoc st = w.state();
    must("link-clean", st["link"]["crcErr"].num() == 0 && st["link"]["broken"].num() == 0 && st["link"]["cut"].num() == 0, "The link counters show no error",
         fmt("bad checksums %g, broken status %g, cut frames %g", st["link"]["crcErr"].num(), st["link"]["broken"].num(), st["link"]["cut"].num()));
    note("Answers", fmt("%u requests, slowest %u ms", asked, w.rules.worstAnswerMs));
    rules(w);
}

SCENARIO(User::Careful, "careful-settings-save", "The user gives the XIAO's network a name and a password of their own", oneKit())
{
    World w(kit);
    w.start();
    const int boots = w.xiao.boots;
    const Reply r = w.post("/api/settings", settings("agopengps", "Trator Puma", "puma-2026", false, "", "", "Puma"));
    w.noteUserRestart();
    must("saved", r.done && r.code == 200 && r.json["ok"].num() == 1, "The page gets 'ok'", fmt("code %d %s %s", r.code, r.error().c_str(), r.why.c_str()));
    must("flash", w.flashStr("apSsid") == "Trator Puma" && w.flashStr("apPass") == "puma-2026" && w.flashStr("host") == "puma",
         "The new name, password and network name (in small letters) are in its flash", w.flashStr("apSsid") + " / " + w.flashStr("host"));
    const bool back = backAfterRestart(w, boots);
    must("restarted", back && w.dev.apSsid == "Trator Puma" && w.dev.apPass == "puma-2026", "It restarts by itself with the new network", w.dev.apSsid);
    w.knownSsid = "Trator Puma";
    w.knownPass = "puma-2026";
    w.run(3000);
    const JsonDoc st = w.state();
    must("state", st["reset"].str() == "restart requested" && st["resetBad"].num() == 0 && st["wifi"]["ap"].str() == "Trator Puma" && st["wifi"]["host"].str() == "puma",
         "After it: 'restart requested', the new names on the page", st["reset"].str() + ", " + st["wifi"]["ap"].str());
    must("mdns", w.dev.mdnsHost == "puma" && w.dev.mdnsInstance == "Trator Puma", "It answers to puma.local, and AgOpenWeb's list shows 'Trator Puma'", w.dev.mdnsHost);
    must("old-pass", w.post("/api/restart", { { "pw", "agopengps" } }).code == 403, "The old password no longer changes anything", "");
    must("teensy-log", w.teensy.logged("[xiao] > Settings saved: own network 'Trator Puma'"), "The Teensy's log has the XIAO's line about it", "");
    rules(w);
}

SCENARIO(User::Careful, "careful-other-network", "The XIAO also joins the farm's WiFi; phones reach its page on both networks", oneKit())
{
    World w(kit);
    w.start();
    const int boots = w.xiao.boots;
    const Reply r = w.post("/api/settings", settings("agopengps", w.knownSsid, "", false, "Farm", "farm12345"));
    w.noteUserRestart();
    must("saved", r.code == 200, "The other network is saved", r.error());
    backAfterRestart(w, boots);
    const bool joined = w.runUntil([&]() { return w.said("WiFi: joined 'Farm'"); }, 8000);
    must("joined", joined && w.dev.staJoined, "It joins the farm's WiFi and says so in Messages", "");
    w.run(2500);
    JsonDoc st = w.state();
    must("state", st["wifi"]["st"].str() == "ok" && st["wifi"]["ip"].str() == "192.168.1.50" && st["wifi"]["staOn"].num() == 1,
         "The page shows the address it got there", st["wifi"]["st"].str() + " " + st["wifi"]["ip"].str());
    Link other;
    other.fromAp = false;
    const Reply page = w.get("/", other);
    must("page-other", page.done && page.code == 200 && page.body.size() > 50000, "A phone on the farm's WiFi gets the page at that address", fmt("code %d %s", page.code, page.why.c_str()));
    const Reply named = w.http(GET("/", "xiao.home"), other);
    must("page-router-name", named.code == 200, "... also under a name the router gave it (no redirect to 192.168.4.1, which is not on that network)", fmt("code %d", named.code));
    must("404-other", w.http(GET("/nothing-here", "192.168.1.50"), other).code == 404, "An address that does not exist: 404 there", "");
    const Reply probe = w.http(GET("/generate_204", "connectivitycheck.gstatic.com"));
    must("captive", probe.code == 302 && probe.hdr("location") == "http://192.168.4.1/", "A phone that joins the XIAO's own network and checks for internet is sent to the page",
         fmt("code %d, to '%s'", probe.code, probe.hdr("location").c_str()));
    must("local-name", w.http(GET("/", "xiao.local")).code == 200 && w.http(GET("/", "XIAO.local")).code == 200, "http://xiao.local/ gets the page, in any letter case", "");
    w.runUntil([&]() { return has(w.teensy.info, "\"st\":\"ok\""); }, 6000);        // (its info goes to the Teensy every 5 s)
    bool ok = false;
    const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
    must("info", ok && inf["sta"].str() == "Farm" && inf["st"].str() == "ok" && inf["ip"].str() == "192.168.1.50", "The Teensy's page gets the other network and the address too", w.teensy.info);

    w.dev.router.on = false;                // the router is switched off
    must("lost", w.runUntil([&]() { return w.said("WiFi: lost the network 'Farm'"); }, 3000), "The router goes off: Messages say the network was lost", "");
    w.run(1000);
    st = w.state();
    must("state-lost", st["wifi"]["st"].str() == "con" && !st["wifi"].has("ip"), "... and the page shows it is searching", st["wifi"]["st"].str());
    w.dev.router.on = true;
    w.dev.usbOut.clear();
    must("back", w.runUntil([&]() { return w.said("WiFi: joined 'Farm'"); }, 10000), "The router comes back: it joins again by itself", "");
    rules(w);
}

SCENARIO(User::Careful, "careful-update", "The XIAO's own program is updated from the page with the .bin of the repository", bothKits())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> bin = repoFile(info(kit).bin);
    const int boots = w.xiao.boots;
    Link l;
    l.upPerMs = 150;                        // 150 kB a second
    const Reply r = w.upload("/update?pw=agopengps", std::string(info(kit).lib) + ".bin", bin, l);
    w.noteUserRestart();
    must("accepted", r.done && r.code == 200 && r.json["ok"].num() == 1, "The page gets 'ok' for the right file", fmt("code %d %s %s", r.code, r.error().c_str(), r.why.c_str()));
    must("said", w.said("Program update: receiving") && w.said(fmt("Program update: %u bytes checked, restarting", (unsigned)bin.size())),
         "Messages say what was received and that every byte was checked", "");
    const bool back = backAfterRestart(w, boots);
    must("installed", back && w.dev.updInstalled == 1 && w.dev.updInstalledNotApp == 0, "It restarts into the new program", fmt("installed %u", w.dev.updInstalled));
    w.run(3000);
    must("after", w.state()["reset"].str() == "restart requested" && w.teensy.detected, "After it the page answers and the Teensy hears the hello again", "");
    note("Upload", fmt("%u kB in %.1f s", (unsigned)(bin.size() / 1024), r.tookMs() / 1000.0));
    rules(w);
}

SCENARIO(User::Careful, "careful-update-other-chip", "By mistake the .bin of the other XIAO (C3 / S3) is chosen", bothKits())
{
    World w(kit);
    w.start();
    const Kit otherKit = kit == Kit::C3 ? Kit::S3 : Kit::C3;
    const std::vector<uint8_t> bin = repoFile(info(otherKit).bin);
    const Reply r = w.upload("/update?pw=agopengps", std::string(info(otherKit).lib) + ".bin", bin);
    const std::string want = otherKit == Kit::S3 ? "other chip ESP32-S3" : "other chip ESP32-C3";
    must("refused", r.done && r.code == 400 && r.error() == want, "It is refused, and the page is told which chip the file is for", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
    must("said", w.said("Program update REFUSED: " + want), "Messages say so", "");
    w.run(3000);
    must("kept", w.xiao.restarts.empty() && w.dev.updInstalled == 0 && w.state()["ver"].str() == "4.5", "The program that was running keeps running", "");
    const int boots = w.xiao.boots;
    const Reply good = w.upload("/update?pw=agopengps", "right.bin", makeImage(info(kit).chip, 40000));
    w.noteUserRestart();
    must("then-right", good.code == 200 && backAfterRestart(w, boots) && w.dev.updInstalled == 1, "The right file is accepted straight after", good.error());
    rules(w);
}

SCENARIO(User::Careful, "careful-teensy-update", "The Teensy's firmware (.hex) is sent from the XIAO page, over the link", oneKit())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> hex = makeHex(160000);
    w.teensy.fwExpect = hex;
    Link l;
    l.upPerMs = 500;
    const Reply r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "v4.5_AiO_WEB.ino.hex", hex, l);
    must("accepted", r.done && r.code == 200 && r.json["ok"].num() == 1 && r.json["size"].num() == hex.size(), "The page gets 'ok' and the size the Teensy checked",
         fmt("code %d %s %s", r.code, r.error().c_str(), r.why.c_str()));
    must("whole", w.teensy.fwDone == 1 && w.teensy.fwRefused == 0, "The Teensy got the file whole, byte for byte, and installs it", w.teensy.fwWhy);
    must("said", w.said("Module firmware: sending v4.5_AiO_WEB.ino.hex") && w.said("bytes checked by the Teensy"), "Messages say what was sent and that the Teensy checked it", "");
    must("teensy-back", w.runUntil([&]() { return w.teensy.on && w.teensy.detected && w.teensy.statusSent > 0; }, 15000), "The Teensy restarts and talks to the XIAO again", "");
    w.run(2500);
    const JsonDoc st = w.state();
    must("page-after", st["link"]["talking"].num() == 1, "The page shows the module talking again", "");
    const Reply lg = w.get("/api/log?after=0&xafter=0");
    must("log-epoch", lg.json["epoch"].num() == 2, "The page is told the module's log started again (it restarted)", fmt("epoch %g", lg.json["epoch"].num()));
    note("Transfer", fmt("%u kB in %.1f s (a real .hex is about 2 MB)", (unsigned)(hex.size() / 1024), r.tookMs() / 1000.0));
    rules(w);
}

SCENARIO(User::Careful, "careful-teensy-revert-subnet", "From the XIAO page: the Teensy's previous firmware back, then another subnet for the module", oneKit())
{
    World w(kit);
    w.start();
    Reply r = w.post("/api/teensy-revert", { { "pw", "agopengps" } });
    must("revert", r.code == 200 && r.json["ok"].num() == 1 && w.teensy.reverts == 1, "The Teensy is asked for its previous firmware and says yes", fmt("code %d %s", r.code, r.error().c_str()));
    must("revert-said", w.said("the Teensy goes back to its previous firmware and restarts"), "Messages say so", "");
    must("revert-back", w.runUntil([&]() { return w.teensy.on && w.teensy.detected; }, 15000), "The Teensy restarts and talks again", "");
    w.run(3000);
    r = w.post("/api/subnet", { { "subnet", "192.168.5" }, { "pw", "agopengps" } });
    must("subnet", r.code == 200 && w.runUntil([&]() { return w.teensy.subnets.size() == 1; }, 500) && w.teensy.subnets[0] == std::array<uint8_t, 3>{ 192, 168, 5 },
         "The subnet 192.168.5 reaches the Teensy", fmt("code %d %s", r.code, r.error().c_str()));
    must("subnet-said", w.said("Module subnet 192.168.5 sent to the Teensy from the page"), "Messages say so", "");
    must("subnet-back", w.runUntil([&]() { return w.teensy.on && w.teensy.detected; }, 15000), "The Teensy restarts and talks again", "");
    rules(w);
}

SCENARIO(User::Careful, "careful-bridge", "A tablet with no network cable: the WiFi bridge is switched on, AgOpenWeb steers through the XIAO for a minute", bothKits())
{
    World w(kit);
    w.start();
    const int boots = w.xiao.boots;
    const Reply r = w.post("/api/settings", settings("agopengps", w.knownSsid, "trator-2026", true));
    w.noteUserRestart();
    must("saved", r.code == 200, "The bridge is switched on together with a password of one's own", r.error());
    backAfterRestart(w, boots);
    w.knownPass = "trator-2026";
    w.run(3000);
    must("on", w.said("WiFi bridge on") && w.dev.udpOpen() == 2, "After the restart the bridge is on (UDP 8888 and 2233 open)", "");
    w.tab.rtcmEveryMs = 200;                // corrections: 1.3 kB a second
    if (beginSteer(w))
    {
        JsonDoc st;
        for (int s = 0; s < 60; s++) { w.run(1000); st = w.state(); }
        must("state", st["bridge"]["on"].num() == 1 && st["bridge"]["teensy"].num() == 1 && st["bridge"]["state"].num() == 2 && st["bridge"]["prog"].str() == "192.168.4.2" && st["bridge"]["steer"].num() == 1,
             "The page shows the bridge: on, the Teensy knows it, the program at 192.168.4.2 has the module and steers",
             fmt("state %g, prog %s, steer %g", st["bridge"]["state"].num(), st["bridge"]["prog"].str().c_str(), st["bridge"]["steer"].num()));
        must("state-count", st["bridge"]["in"].num() > 6000 && st["bridge"]["out"].num() > 9000 && st["bridge"]["lost"].num() == 0 && st["bridge"]["other"].num() == 0,
             "... with its counters, none lost", fmt("in %g, out %g, lost %g", st["bridge"]["in"].num(), st["bridge"]["out"].num(), st["bridge"]["lost"].num()));
        must("aog-card", st["aog"]["rate254"].num() >= 98 && st["aog"]["status"].num() == 1 && st["aog"]["kmh"].num() == 7.2, "The 'Program -> module' card shows what the tablet sends (100 a second, 7.2 km/h, autosteer on)",
             fmt("%g a second, %g km/h", st["aog"]["rate254"].num(), st["aog"]["kmh"].num()));
        endSteer(w);
        must("all-passed", w.tab.nNotPassed == 0 && w.tab.nNotTaken == 0 && w.tab.worstDelayMs <= 20, "Every packet of the tablet reached the Teensy, at most 20 ms later",
             fmt("%u not passed, %u not taken by the XIAO, longest delay %u ms", w.tab.nNotPassed, w.tab.nNotTaken, w.tab.worstDelayMs));
        must("answers", w.tab.got + 8 >= w.teensy.netOut && w.tab.got253 > 5900 && w.tab.gotNmea > 2300 && w.tab.gotHello >= 100, "Everything the Teensy answered reached the tablet (steering answers, positions, hellos)",
             fmt("%u of %u; %u steering answers, %u sentences", w.tab.got, w.teensy.netOut, w.tab.got253, w.tab.gotNmea));
        must("rtcm", w.teensy.rtcm.size() >= 295 * 256 && w.teensy.rtcm.size() <= w.tab.rtcmSent.size() && std::equal(w.teensy.rtcm.begin(), w.teensy.rtcm.end(), w.tab.rtcmSent.begin()),
             "The corrections (NTRIP) reached the receiver whole and in order", fmt("%u of %u bytes", (unsigned)w.teensy.rtcm.size(), (unsigned)w.tab.rtcmSent.size()));
        must("teensy-log", w.teensy.logged("WiFi bridge: the program at 192.168.4.2 is on the XIAO's WiFi") && w.said("WiFi bridge: a program at 192.168.4.2 is calling the module"),
             "The Teensy's log and the XIAO's Messages both say who has the module", "");
        bool ok = false;
        const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
        must("info", ok && inf["br"].num() == 1, "The Teensy's page is told the bridge is on", w.teensy.info);
        w.tab.on = false;                   // the program is closed
        must("silent", w.runUntil([&]() { return w.said("is silent"); }, 5000) && w.runUntil([&]() { return w.teensy.brState == 0; }, 5000),
             "The program is closed: 3 s later the XIAO says it is silent and the module is free again", "");
    }
    rules(w);
}

SCENARIO(User::Careful, "careful-bridge-page-while-steering", "While the tablet steers through the bridge, two phones look at the XIAO's page (one on a slow link)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    if (beginSteer(w))
    {
        Link slow;
        slow.ipLast = 21;
        slow.downPerMs = 3;                 // 3 kB a second: the page takes over 20 s
        const int slowPage = w.ask(GET("/"), slow);
        const Reply page = w.get("/");
        must("page", page.code == 200 && page.body.size() > 50000, "The phone on the good link gets the page", fmt("code %d %s", page.code, page.why.c_str()));
        int okAnswers = 0;
        for (int s = 0; s < 30; s++)
        {
            w.run(1000);
            okAnswers += w.get("/api/state").jsonOk + w.get("/api/teensy").jsonOk + w.get("/api/log?after=0&xafter=0").jsonOk;
        }
        const Reply sp = w.wait(slowPage);
        must("slow-page", sp.done && sp.code == 200 && sp.body.size() == page.body.size(), "The phone on the slow link gets it whole too", fmt("%u bytes in %.1f s %s", (unsigned)sp.body.size(), sp.tookMs() / 1000.0, sp.why.c_str()));
        tell("served-meanwhile", okAnswers >= 80, "While the slow phone is being served, the other phone's page keeps updating",
             fmt("%d of 90 answers arrived (the XIAO serves one connection at a time)", okAnswers));
        endSteer(w);
    }
    rules(w);
}

SCENARIO(User::Careful, "careful-usb-and-restart", "USB serial monitor: status, an unknown word, restart; then the page's Restart button", oneKit())
{
    World w(kit);
    w.start();
    w.dev.usbOut.clear();
    w.usb("status");
    w.run(200);
    must("status", w.said("v4.5_XIAO_ESP32C3, up ") && w.said("Own WiFi '" + w.knownSsid + "' at 192.168.4.1, 1 client(s)") && w.said("Teensy: talking") && w.said("WiFi bridge: off"),
         "'status' prints the program, the network, the Teensy and the bridge", w.dev.usbOut.substr(0, 300));
    w.dev.usbOut.clear();
    w.usb("what?\r");
    w.run(200);
    must("help", w.said("Commands: status | wifi-reset | restart"), "An unknown word prints the commands", w.dev.usbOut);
    int boots = w.xiao.boots;
    w.usb("restart");
    w.noteUserRestart();
    must("usb-restart", backAfterRestart(w, boots, 3000), "'restart' restarts it", "");
    w.run(3000);
    boots = w.xiao.boots;
    const Reply r = w.post("/api/restart", { { "pw", "agopengps" } });
    w.noteUserRestart();
    must("button", r.done && r.code == 200 && r.json["ok"].num() == 1, "The Restart button: the page gets its 'ok' before the XIAO goes", fmt("code %d %s", r.code, r.why.c_str()));
    must("button-restart", backAfterRestart(w, boots, 4000) && w.said("Restart requested from the page"), "... and it restarts a second later", "");
    w.run(3000);
    must("reason", w.state()["reset"].str() == "restart requested", "The page then shows 'restart requested', not an error", "");
    rules(w);
}
