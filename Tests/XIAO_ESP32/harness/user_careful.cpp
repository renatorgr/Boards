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
    must("link-task", w.dev.tasks.size() == 1 && w.dev.wdtUserOn && !w.said("link task did NOT start"),
         "Its link task runs, and its watchdog is on (the link and the bridge go on while the page waits for a browser)",
         fmt("%u task(s), the program's watchdog entry %s", (unsigned)w.dev.tasks.size(), w.dev.wdtUserOn ? "on" : "off"));
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
    must("on", w.said("WiFi bridge on") && w.dev.udpOpen() == 3, "After the restart the bridge is on (UDP 8888, 2233 and 9999 open)", "");
    w.tab.rtcmEveryMs = 200;                // corrections: 1.3 kB a second
    if (beginSteer(w))
    {
        JsonDoc st;
        for (int s = 0; s < 60; s++) { w.run(1000); st = w.state(); }
        must("state", st["bridge"]["on"].num() == 1 && st["bridge"]["teensy"].num() == 1 && st["bridge"]["state"].num() == 2 && st["bridge"]["prog"].str() == "192.168.5.2" && st["bridge"]["steer"].num() == 1,
             "The page shows the bridge: on, the Teensy knows it, the program at 192.168.5.2 has the module and steers",
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
        must("teensy-log", w.teensy.logged("WiFi bridge: the program at 192.168.5.2 is on the XIAO's WiFi") && w.said("WiFi bridge: a program at 192.168.5.2 is calling the module"),
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

SCENARIO(User::Careful, "careful-bridge-network-numbers", "The WiFi bridge is switched on: the XIAO's network takes the module's numbers (192.168.5.x), a sprayer module joins it as it is; switched off, back to 192.168.4.x", oneKit())
{
    World w(kit);
    w.start();
    w.run(1500);                            // (the Teensy's status, with its address, has come)
    JsonDoc st = w.state();
    must("before", st["wifi"]["apIp"].str() == "192.168.4.1" && w.get("/api/settings").json["modNet"].str() == "192.168.5",
         "With the bridge off the XIAO's network is at 192.168.4.1, and the page knows the module's numbers (192.168.5, from the Teensy's status)",
         st["wifi"]["apIp"].str());
    int boots = w.xiao.boots;
    Reply r = w.post("/api/settings", settings("agopengps", w.knownSsid, "trator-2026", true));
    w.noteUserRestart();
    must("saved", r.code == 200, "The bridge is switched on together with a password of one's own", r.error());
    backAfterRestart(w, boots);
    w.knownPass = "trator-2026";
    w.run(3000);
    st = w.state();
    must("moved", w.dev.apIp == IPAddress(192, 168, 5, 1) && st["wifi"]["apIp"].str() == "192.168.5.1" && w.said("own network '" + w.knownSsid + "' at 192.168.5.1"),
         "After the restart the XIAO's network is at 192.168.5.1, the module's numbers, and Messages say so", st["wifi"]["apIp"].str());
    const Reply probe = w.http(GET("/generate_204", "connectivitycheck.gstatic.com"));
    must("captive", probe.code == 302 && probe.hdr("location") == "http://192.168.5.1/", "A phone that checks for internet there is sent to http://192.168.5.1/",
         fmt("code %d, to '%s'", probe.code, probe.hdr("location").c_str()));
    must("page", w.http(GET("/", "192.168.5.1")).code == 200, "The page answers at its new address", "");
    w.runUntil([&]() { return has(w.teensy.info, "192.168.5.1"); }, 6000);        // (its info goes to the Teensy every 5 s)
    bool ok = false;
    const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
    must("info", ok && inf["apIp"].str() == "192.168.5.1", "The Teensy's page is told the new address", w.teensy.info);
    Machine& sprayer = w.mod[0];            // 192.168.5.123: the subnet it always had
    sprayer.on = true;
    if (beginSteer(w))
    {
        w.run(10000);
        endSteer(w);
        must("module", sprayer.got239 >= 90 && w.tab.gotMachine >= 45 && sprayer.throughXiao(),
             "A sprayer module at 192.168.5.123, its subnet unchanged, gets the program's section data through the XIAO, and the tablet its packets",
             fmt("%u section packets to it, %u of its packets to the tablet", sprayer.got239, w.tab.gotMachine));
    }
    w.tab.on = false;
    sprayer.on = false;
    w.run(4000);
    boots = w.xiao.boots;
    r = w.post("/api/settings", settings("trator-2026", w.knownSsid, "", false));
    w.noteUserRestart();
    must("off", r.code == 200 && backAfterRestart(w, boots), "The bridge is switched off again", r.error());
    w.run(3000);
    must("back", w.dev.apIp == IPAddress(192, 168, 4, 1) && w.state()["wifi"]["apIp"].str() == "192.168.4.1", "... and the XIAO's network is at 192.168.4.1 again",
         w.dev.apIp.toString().c_str());
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
        const int slowPage = w.ask(GET("/", w.hostFor(slow)), slow);
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

SCENARIO(User::Careful, "careful-bridge-modules", "No router on the tractor: the sprayer and the seed drill modules join the XIAO's WiFi too, and AgOpenWeb works with them through the XIAO", bothKits())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    Machine& sprayer = w.mod[0];            // 192.168.5.123
    Machine& drill = w.mod[1];              // 192.168.5.124
    sprayer.on = drill.on = true;
    w.run(2000);
    JsonDoc st = w.state();
    must("known", st["bridge"]["mods"].a.size() == 2 && w.said("a module at 192.168.5.123 is on this network") && w.said("a module at 192.168.5.124 is on this network"),
         "With no program running yet the XIAO already knows both modules, from their own packets, and Messages say so", fmt("%u listed", (unsigned)st["bridge"]["mods"].a.size()));
    must("room", w.dev.apMaxStations >= 6, "The XIAO's network lets in a tablet, two modules and phones (the core's own limit is 4 devices)", fmt("%d devices", w.dev.apMaxStations));
    must("quiet", sprayer.got == 0 && drill.got == 0 && w.tab.got == 0, "... and sends them nothing while no program runs", fmt("%u and %u datagrams", sprayer.got, drill.got));
    must("machine-tab", w.teensy.machFrom.count("192.168.5.123") && w.teensy.machFrom.count("192.168.5.124") && w.teensy.machIn >= 10 && w.teensy.machBad == 0,
         "The module's page (Machine tab) hears both modules through the XIAO too, with no program running", fmt("%u packets from %u modules", w.teensy.machIn, (unsigned)w.teensy.machFrom.size()));
    if (beginSteer(w))
    {
        w.run(1000);
        const uint32_t a0 = sprayer.got239, b0 = drill.got239, m0 = w.tab.gotMachine, d0 = w.tab.gotMachineDirect, n0 = w.tab.nSent;
        w.tab.lastMachineMs = fake::clockMs;
        w.tab.machineWatch = true;
        w.run(60000);
        w.tab.machineWatch = false;
        must("to-modules", sprayer.got239 - a0 >= 598 && drill.got239 - b0 >= 598 && sprayer.gotHello >= 60 && drill.gotHello >= 60 && sprayer.worstDelayMs <= 20 && drill.worstDelayMs <= 20,
             "Each module got the program's machine data (10 a second) and its hellos, at most 20 ms after the tablet sent them",
             fmt("sprayer %u of 600, %u hellos, longest delay %u ms; seed drill %u of 600, %u hellos, %u ms", sprayer.got239 - a0, sprayer.gotHello, sprayer.worstDelayMs, drill.got239 - b0, drill.gotHello, drill.worstDelayMs));
        must("no-steering", sprayer.gotSteer == 0 && drill.gotSteer == 0, "The steering data (a hundred packets a second) is not sent to them: nothing in it is for a machine module",
             fmt("%u and %u", sprayer.gotSteer, drill.gotSteer));
        must("from-modules", w.tab.gotMachine - m0 >= 2 * (300 + 59) && w.tab.worstMachineGapMs <= 250,
             "The tablet got both modules' packets through the XIAO (PGN 237 five times a second each, and the hello answers)",
             fmt("%u packets in a minute, longest time without one %u ms", w.tab.gotMachine - m0, w.tab.worstMachineGapMs));
        must("to-it-only", w.tab.gotMachineDirect == d0, "Once the program's packets come through the XIAO, the modules answer the XIAO only (nothing to everyone, which the WiFi would hold back)",
             fmt("%u heard straight from a module", w.tab.gotMachineDirect - d0));
        st = w.state();
        const JsonDoc& ml = st["bridge"]["mods"];
        must("page", ml.a.size() == 2 && ml.a[0]["ip"].str() == "192.168.5.123" && ml.a[1]["ip"].str() == "192.168.5.124" && ml.a[0]["to"].num() > 600 && ml.a[0]["from"].num() > 350 && st["bridge"]["lost"].num() == 0,
             "The page lists both modules with what went to them and came from them, nothing lost", fmt("%u listed, lost %g", (unsigned)ml.a.size(), st["bridge"]["lost"].num()));
        note("Through the XIAO to the modules", fmt("the tablet sent %u packets in the minute; %u went on to each module (machine data, hellos), longest delay %u ms", w.tab.nSent - n0, sprayer.got239 - a0 + 60, std::max(sprayer.worstDelayMs, drill.worstDelayMs)));

        drill.on = false;                   // the seed drill module is switched off
        const bool silent = w.runUntil([&]() { return w.said("the module at 192.168.5.124 is silent"); }, 5000);
        must("gone", silent && w.state()["bridge"]["mods"].a.size() == 1, "The seed drill module is switched off: 3 s later the XIAO says it is silent and sends it nothing more", "");
        const uint32_t b1 = drill.got239;
        drill.on = true;                    // ... and on again
        must("back", w.runUntil([&]() { return drill.got239 >= b1 + 5; }, 2000), "Switched on again, it gets the program's packets within a second", fmt("%u in 2 s", drill.got239 - b1));
        endSteer(w);
        must("steering-untouched", w.tab.nNotPassed == 0 && w.tab.nNotTaken == 0 && w.tab.worstDelayMs <= 20, "All the while every packet of the tablet reached the Teensy, at most 20 ms later",
             fmt("%u not passed, %u not taken, longest delay %u ms", w.tab.nNotPassed, w.tab.nNotTaken, w.tab.worstDelayMs));
        w.tab.on = false;                   // the program is closed
        w.run(5000);
        const uint32_t a2 = sprayer.got;
        w.run(3000);
        must("program-closed", sprayer.got == a2 && w.state()["bridge"]["mods"].a.size() == 2, "The program is closed: the modules get nothing more, and stay listed", fmt("%u more", sprayer.got - a2));
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

SCENARIO(User::Careful, "careful-sign-in-page", "A phone joins the XIAO's WiFi: the XIAO's page opens by itself (sign-in page); 'Done on this device' stops the sign-in notice on that phone only, over restarts", oneKit())
{
    World w(kit);
    w.start();
    w.run(1500);
    const std::string ap = w.dev.apIp.toString().c_str();
    Link phone;
    phone.ipLast = 30;
    World::DnsReply d = w.dns("connectivitycheck.gstatic.com", 30);
    must("dns", d.rcode == 0 && d.addr == ap, "A phone asking for any name gets the XIAO's address (so its 'is there internet?' lands on the page)", fmt("code %d, %s", d.rcode, d.addr.c_str()));
    Reply r = w.http(GET("/generate_204", "connectivitycheck.gstatic.com"), phone);
    must("redirect", r.code == 302 && has(r.headers["location"], ap), "... and is sent to the XIAO's page: the phone opens it by itself", fmt("code %d -> %s", r.code, r.headers["location"].c_str()));
    r = w.get("/api/state", phone);
    must("banner", r.json["captive"]["ap"].num() != 0 && r.json["captive"]["done"].num() == 0, "The page shows the phone the 'Done on this device' button", "");
    r = w.post("/api/captive", {}, phone);
    must("done", r.code == 200 && r.json["ok"].num() != 0 && w.said("Sign-in page: done"), "'Done on this device': answered ok, and Messages say so", fmt("code %d %s", r.code, r.error().c_str()));
    d = w.dns("connectivitycheck.gstatic.com", 30);
    must("no-such-name", d.rcode == 3 && d.addr.empty(), "From then on that phone gets 'no such name': it says only 'no internet', no sign-in notice", fmt("code %d", d.rcode));
    r = w.http(GET("/generate_204", "connectivitycheck.gstatic.com"), phone);
    must("no-redirect", r.code != 302, "... and is no longer sent to the page", fmt("code %d", r.code));
    must("page", w.get("/", phone).code == 200, "The XIAO's page still opens at its address", "");
    Link other;
    other.ipLast = 31;
    d = w.dns("www.msftconnecttest.com", 31);
    must("others", d.rcode == 0 && d.addr == ap, "Another device still gets the sign-in page", fmt("code %d, %s", d.rcode, d.addr.c_str()));
    must("flash", has(w.flashStr("capOk"), "02000000001e"), "The phone is kept in the flash (by its MAC)", w.flashStr("capOk"));
    const int boots = w.xiao.boots;
    w.noteUserRestart();
    w.usb("restart");
    must("restarted", backAfterRestart(w, boots), "The XIAO restarts", "");
    w.run(2000);
    d = w.dns("connectivitycheck.gstatic.com", 30);
    must("kept", d.rcode == 3, "After a restart that phone still gets no sign-in notice", fmt("code %d", d.rcode));
    r = w.post("/api/captive", { { "forget", "1" }, { "pw", "wrong-one" } }, other);
    must("forget-pw", r.code == 403, "'Show it again on all' needs the XIAO's password", fmt("code %d", r.code));
    r = w.post("/api/captive", { { "forget", "1" }, { "pw", "agopengps" } }, other);
    d = w.dns("connectivitycheck.gstatic.com", 30);
    must("forget", r.code == 200 && d.rcode == 0 && d.addr == ap, "With the password: every device gets the sign-in page again", fmt("code %d, then %d %s", r.code, d.rcode, d.addr.c_str()));
    rules(w);
}
