/* user_wrong_connections.cpp - things connected wrong: two tablets, a program on the wrong network, a program on
   the cable too, a broken wire, something that is no Teensy on the port.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
using namespace h;

SCENARIO(User::WrongConnections, "conn-two-tablets", "Two tablets on the XIAO's WiFi both run AgOpenWeb", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.tab2.on = w.tab2.engaged = true;          // the second one starts, autosteer on too
        w.run(20000);
        must("first-only", w.tab2.nPassed == 0 && w.tab2.got == 0, "Only the first tablet's program reaches the Teensy and gets its answers",
             fmt("second tablet: %u packets passed, %u answers", w.tab2.nPassed, w.tab2.got));
        const JsonDoc st = w.state();
        must("counted", st["bridge"]["other"].num() > 1000 && st["bridge"]["prog"].str() == "192.168.5.2", "The page counts the second program's packets as 'from another program'",
             fmt("other %g", st["bridge"]["other"].num()));
        tell("told", w.said("192.168.5.3") || w.teensy.logged("192.168.5.3"), "Messages say that a second program (at 192.168.5.3) is calling and is not answered",
             "nothing: only a counter in the page's data moves ('other'), and the page does not show it");
        endSteer(w);
        must("first-untouched", w.tab.nNotPassed == 0 && w.tab.worstDelayMs <= 20, "The first tablet's steering was not disturbed", fmt("%u not passed, longest delay %u ms", w.tab.nNotPassed, w.tab.worstDelayMs));
        w.tab.on = false;                           // the first tablet's program is closed
        must("second-takes-over", w.runUntil([&]() { return w.tab2.nPassed > 0 && w.teensy.steering() && w.tab2.got253 > 0; }, 6000),
             "When the first program is closed, the second one gets the module 3 s later", fmt("second tablet: %u passed, %u answers", w.tab2.nPassed, w.tab2.got));
        w.tab2.on = false;
        w.run(4000);
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-program-on-other-network", "The tablet is on the farm's WiFi (the XIAO joined it too), not on the XIAO's own", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.start();
    must("joined", w.runUntil([&]() { return w.dev.staJoined; }, 5000), "The XIAO is on the farm's WiFi", "");
    w.far.on = w.far.engaged = true;
    w.far.rtcmEveryMs = 1000;
    w.run(15000);
    must("not-passed", w.far.nPassed == 0 && w.teensy.netIn == 0 && w.teensy.rtcm.empty() && !w.teensy.steering() && w.far.got == 0,
         "Nothing from the other network reaches the Teensy: the bridge is for the XIAO's own network only", fmt("%u passed, %u bytes of corrections", w.far.nPassed, (unsigned)w.teensy.rtcm.size()));
    must("heard", w.far.nSent > 1000 && w.far.nNotTaken < w.far.nSent, "(the XIAO did hear those packets)", fmt("%u sent, %u not taken", w.far.nSent, w.far.nNotTaken));
    const JsonDoc st = w.state();
    tell("told", w.said("other network") && w.said("192.168.1.77"), "Messages say that a program on the other network is calling and why it is not answered",
         fmt("nothing in Messages; on the page the bridge shows no program (counter 'other': %g)", st["bridge"]["other"].num()));
    // ... and then the tablet is moved to the XIAO's own network
    w.far.on = false;
    if (beginSteer(w)) { w.run(5000); endSteer(w); }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-forged-address", "Something on the farm's WiFi sends steering packets that claim to come from the XIAO's own network", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.start();
    must("joined", w.runUntil([&]() { return w.dev.staJoined; }, 5000), "The XIAO is on the farm's WiFi", "");
    w.far.forged = true;                    // "from 192.168.5.77", sent to the XIAO's address on the farm's network
    w.far.on = w.far.engaged = true;
    w.run(8000);
    const bool passed = w.teensy.netIn > 0 || w.teensy.steering();
    must("not-passed", !passed, "They are not passed to the Teensy (nobody on the other network may steer the tractor)",
         fmt("%u packets reached the Teensy, and it %s", w.teensy.netIn, w.teensy.steering() ? "switched autosteer on for them" : "did not steer"));
    if (passed) { note("Not checked further", "the Teensy's answers then go to that address on the XIAO's own network, where nobody is"); return; }
    w.far.on = false;
    w.run(4000);
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-forged-module", "Something on the farm's WiFi sends a section module's packets that claim to come from the XIAO's own network", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.start();
    must("joined", w.runUntil([&]() { return w.dev.staJoined; }, 5000), "The XIAO is on the farm's WiFi", "");
    w.far.forged = true;                    // "from 192.168.5.77", sent to the XIAO's address on the farm's network
    for (int k = 0; k < 10; k++)
    {
        w.far.send(9999, packet(237, { (uint8_t)k, 0, 0, 0, 0, 1, 0, 0 }, 0x7B), false);
        w.run(200);
    }
    must("not-taken", w.state()["bridge"]["mods"].a.empty() && !w.said("a module at 192.168.5.77"), "It is not taken for a module on the XIAO's network",
         "the XIAO lists a module at 192.168.5.77: the program's packets would go to that address on its own network (where nobody is), and such packets would go on to the program");
    w.run(4000);
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-modules-bridge-off", "The modules are set to join the XIAO's WiFi, but the WiFi bridge was never switched on", oneKit())
{
    World w(kit);
    w.start();                              // a XIAO as it comes: the bridge is off
    w.mod[0].on = w.mod[1].on = true;
    w.tab.on = true;
    w.run(10000);
    must("nothing", w.mod[0].got == 0 && w.mod[1].got == 0 && w.tab.gotMachine == 0 && w.dev.udpOpen() == 0, "Nothing is passed on in either direction (the bridge is off: no UDP port is open)",
         fmt("%u datagrams to the modules, %u to the tablet", w.mod[0].got + w.mod[1].got, w.tab.gotMachine));
    const JsonDoc st = w.state();
    tell("told", st["bridge"]["on"].num() == 0, "The page says where to look: 'WiFi bridge: off (WiFi tab)'", "");
    note("With the bridge off", fmt("the tablet still hears what the modules send to everyone (%u packets in 10 s), as far as the WiFi lets a broadcast through; "
                                    "the modules hear the program the same way. The XIAO cannot tell that modules are waiting", w.tab.gotMachineDirect));
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-five-modules", "Five section modules join the XIAO's WiFi (it has places for four)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    for (Machine& m : w.mod) m.on = true;   // 192.168.5.123 to .127
    if (beginSteer(w))
    {
        w.run(20000);
        int served = 0;
        for (Machine& m : w.mod) served += m.got239 >= 190;
        must("four", served == 4, "Four of them get the program's packets, the fifth none", fmt("%d served", served));
        tell("told", w.said("more than 4 modules on this network"), "Messages say that one module too many is on the network, and which", "nothing");
        endSteer(w);
        must("steering-untouched", w.tab.nNotPassed == 0 && w.tab.worstDelayMs <= 20, "The steering through the bridge is not disturbed", fmt("%u not passed, longest delay %u ms", w.tab.nNotPassed, w.tab.worstDelayMs));
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-module-subnet-and-scan", "With a module on the XIAO's WiFi, the program's network window scans for modules and sends a new subnet", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    Machine& sprayer = w.mod[0];
    sprayer.on = true;
    if (beginSteer(w))
    {
        w.run(2000);
        endSteer(w);                        // (the subnet is for a tractor that stands still)
        w.run(1000);
        const uint32_t got = sprayer.got239;
        w.tab.send(8888, packet(202, { 202, 202, 5 }), true);
        w.run(500);
        w.tab.send(8888, packet(201, { 201, 201, 192, 168, 5 }), true);
        w.run(3000);
        must("not-passed", sprayer.gotSubnetOrScan == 0, "Neither reaches the module: a new subnet would take it off the XIAO's network, where nothing could call it back",
             fmt("%u reached it", sprayer.gotSubnetOrScan));
        must("still-served", sprayer.got239 >= got + 30, "The module keeps getting the program's machine data", fmt("%u in 3.5 s", sprayer.got239 - got));
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-cable-and-wifi", "A program on the network cable and one on the XIAO's WiFi at the same time", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.teensy.cable = true;                  // AgOpenWeb on a tablet with the cable
    w.run(2000);
    w.tab.on = w.tab.engaged = true;        // ... and another on the XIAO's WiFi
    w.run(10000);
    must("cable-has-it", w.teensy.brState == 1 && !w.teensy.steering() && w.tab.got == 0, "The cable keeps the module: the WiFi tablet is not answered and does not steer",
         fmt("state %u, %u answers to the WiFi tablet", w.teensy.brState, w.tab.got));
    const JsonDoc st = w.state();
    must("page", st["bridge"]["state"].num() == 1 && st["bridge"]["steer"].num() == 0, "The XIAO's page shows that a program on the cable has the module", fmt("state %g", st["bridge"]["state"].num()));
    must("log", w.teensy.logged("one is talking on the network cable"), "The Teensy's log says who is calling and why it is not answered", "");
    must("hello-only", w.teensy.netIgnored >= 8 && w.teensy.netIgnored <= 14, "Only the WiFi program's hello (one a second) is passed on meanwhile", fmt("%u packets", w.teensy.netIgnored));
    w.teensy.cable = false;                 // the cable's program is closed
    must("wifi-gets-it", w.runUntil([&]() { return w.teensy.steering() && w.tab.got253 > 0; }, 6000), "The cable's program is closed: the WiFi tablet gets the module about 3 s later",
         fmt("state %u", w.teensy.brState));
    w.steerLastMs_ = fake::clockMs;
    w.steerWatch_ = true;
    w.run(5000);
    endSteer(w);
    w.tab.engaged = true;                   // it steers again ...
    w.run(3000);
    w.teensy.cable = true;                  // ... and the cable's program starts
    must("cable-takes", w.runUntil([&]() { return w.teensy.brState == 1; }, 500), "A program starts on the cable while the WiFi tablet steers: the cable takes the module at once", "");
    w.run(5000);
    const Reply r = w.post("/api/restart", { { "pw", "trator-2026" } });
    must("restart-allowed", r.code == 200, "... and the XIAO may be restarted from the page again (the WiFi tablet no longer steers)", fmt("code %d '%s'", r.code, r.error().c_str()));
    w.noteUserRestart();
    w.run(4000);
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-not-a-teensy", "The XIAO sits on a port where something else talks (a GNSS receiver, noise)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.teensy.on = false;
    w.wireJunk = "$GNGGA,123519.00,3855.1234567,N,00907.1234567,W,4,24,0.6,112.34,M,52.1,M,1.0,0123*5B\r\n$GNVTG,84.4,T,,M,7.2,N,13.3,K,D*1F\r\n";
    w.start();
    w.tab.on = w.tab.engaged = true;
    w.run(10000);
    JsonDoc st = w.state();
    must("no-data", st["link"]["talking"].num() == 0 && st["link"]["frames"].num() == 0 && st["link"]["rxKB"].num() > 300, "The page shows bytes arriving but no data from a module",
         fmt("%g kB, %g frames", st["link"]["rxKB"].num(), st["link"]["frames"].num()));
    must("nothing-out", w.dev.udpOut.empty() && w.tab.got == 0, "Nothing of it goes to the tablet's program", "");
    // now bytes that look like the start of frames, at random
    std::string junk;
    uint32_t r = 99;
    for (int i = 0; i < 6000; i++)
    {
        r = r * 1664525u + 1013904223u;
        const uint8_t pick = (uint8_t)(r >> 24);
        if (pick < 40) junk += std::string("\x80\x81\x7E", 3) + (char)(140 + pick % 10) + (char)(r >> 12);
        else if (pick < 60) junk += std::string("\x80\x81\x7F\xFE\x08", 5);
        else junk += (char)(r >> 16);
    }
    w.tab.on = false;                       // (the tablet's program is closed: one frame in 256 of such noise passes the link's one-byte checksum)
    w.wireJunk = junk;
    w.run(20000);
    st = w.state();
    must("survives", st["ver"].str() == "4.5" && w.get("/api/teensy").jsonOk && w.get("/api/log?after=0&xafter=0").jsonOk, "With bytes that look like broken frames the page still answers (valid JSON)", "");
    note("Link counters with noise on the port", fmt("%g frames taken as good, %g bad checksums, %g cut", st["link"]["frames"].num(), st["link"]["crcErr"].num(), st["link"]["cut"].num()));
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-broken-wire", "One wire of the link breaks while a tablet steers through the bridge, then the other", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.run(5000);
        w.steerWatch_ = false;
        w.cutToXiao = true;                 // the XIAO no longer hears the Teensy
        const uint32_t passedAt = w.tab.nPassed;
        w.run(4000);
        const uint32_t passedLater = w.tab.nPassed;
        w.run(6000);
        must("stops-passing", w.tab.nPassed == passedLater && passedLater - passedAt < 400, "Within 3 s the XIAO stops passing the program's packets (it no longer knows who has the module)",
             fmt("%u more packets in the first 4 s, %u after", passedLater - passedAt, w.tab.nPassed - passedLater));
        must("steer-off", w.teensy.steerOffs == 1 && !w.teensy.steering(), "... so the Teensy's own watchdog switches autosteer off", fmt("%u time(s)", w.teensy.steerOffs));
        const JsonDoc st = w.state();
        must("page", st["link"]["talking"].num() == 0 && st["bridge"]["teensy"].num() == 0 && st["bridge"]["steer"].num() == 0, "The page shows no data from the module", "");
        w.tab.engaged = false;
        w.cutToXiao = false;
        w.run(4000);
        must("back", w.state()["link"]["talking"].num() == 1, "The wire is mended: the page shows the module again", "");
        w.run(2000);
        w.cutToTeensy = true;               // now the Teensy no longer hears the XIAO
        w.run(15000);
        must("teensy-says-gone", w.teensy.silentEvents == 1 && !w.teensy.detected, "The other wire breaks: the Teensy says the XIAO is silent", fmt("%u time(s)", w.teensy.silentEvents));
        must("nothing-in", w.tab.got253 > 0 && w.runUntil([&]() { return fake::clockMs - w.tab.lastGotMs > 3000; }, 6000), "... and the tablet's program gets no more answers", "");
        w.teensy.silentEvents = 0;          // (asked for by this scenario, not a late hello)
        w.cutToTeensy = false;
        must("back-2", w.runUntil([&]() { return w.teensy.detected && w.teensy.brState == 2; }, 8000), "Mended: the Teensy hears the XIAO and the program has the module again", "");
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-teensy-restarts", "The Teensy restarts (a dip in its power) while a tablet steers through the bridge", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.run(5000);
        w.steerWatch_ = false;
        const uint32_t before = w.tab.nPassed;
        w.teensy.restart(2500);
        w.run(2400);
        must("off", !w.teensy.on, "(the Teensy is off)", "");
        must("back", w.runUntil([&]() { return w.teensy.on && w.teensy.brState == 2 && w.tab.nPassed > before + 50; }, 8000), "When it runs again the program gets the module back by itself",
             fmt("state %u, %u packets since", w.teensy.brState, w.tab.nPassed - before));
        w.run(3000);
        const JsonDoc st = w.state();
        must("page", st["link"]["talking"].num() == 1 && st["bridge"]["state"].num() == 2, "The page shows the module and the bridge again", "");
        const Reply lg = w.get("/api/log?after=0&xafter=0");
        must("log", lg.jsonOk && lg.json["epoch"].num() >= 2, "The page is told the module's log started again", fmt("epoch %g", lg.json["epoch"].num()));
        w.tab.engaged = false;
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-xiao-loses-power", "The XIAO loses its power for a moment (a loose socket) while a tablet steers through the bridge", bothKits())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.run(5000);
        w.steerWatch_ = false;
        w.xiao.powerOff();
        w.run(1500);
        w.xiao.powerOn();
        const uint64_t t0 = fake::clockMs;
        must("back", w.runUntil([&]() { return w.teensy.steering() && w.tab.lastGotMs > t0 + 100; }, 12000), "Power back: the tablet (it kept sending) gets the module again with nobody touching anything",
             fmt("Teensy bridge state %u", w.teensy.brState));
        note("From power back to steering data at the Teensy", fmt("%.1f s (2 s of it: the tablet joining the WiFi again)", (fake::clockMs - t0) / 1000.0));
        must("reason", w.state()["reset"].str() == "power on", "The page shows 'power on' as the reason of the last start", "");
        w.tab.engaged = false;
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-odd-packets", "Things that are no good AgOpenGPS packets arrive at the bridge's ports", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    if (beginSteer(w))
    {
        w.run(2000);
        const uint32_t in0 = w.teensy.netIn;
        std::vector<uint8_t> big = packet(100, std::vector<uint8_t>(246, 7));       // 252 bytes: one more than a frame carries
        std::vector<uint8_t> most = packet(100, std::vector<uint8_t>(245, 9));      // 251 bytes: just fits
        for (int k = 0; k < 3; k++)
        {
            w.tab.send(8888, big, true);
            w.tab.send(8888, std::vector<uint8_t>(1472, 0x80), true);
            w.tab.send(8888, { 'h', 'e', 'l', 'l', 'o' }, true);
            w.tab.send(8888, { 0x80 }, true);
            w.tab.send(8888, {}, true);
            w.tab.send(8888, { 0x80, 0x81, 0x7F, 0xC8, 0x00 }, true);
            w.run(30);
        }
        w.tab.send(8888, most, true);
        w.run(500);
        const JsonDoc st = w.state();
        must("big-dropped", st["bridge"]["lost"].num() >= 3, "Packets longer than the link carries are counted as lost on the page", fmt("lost %g", st["bridge"]["lost"].num()));
        must("fits", w.teensy.netIn > in0, "(the tablet's good packets keep passing, the longest one that fits too)", "");
        // corrections: from the program that has the module, and from somebody else
        w.tab2.send(2233, std::vector<uint8_t>(300, 0xD3), false);
        w.run(100);
        must("rtcm-other", w.teensy.rtcm.empty(), "Corrections from another device than the program that has the module are not passed to the receiver", fmt("%u bytes", (unsigned)w.teensy.rtcm.size()));
        w.tab.rtcmEveryMs = 25;             // as fast as AgOpenWeb sends them when it has a lot: 256 bytes every 25 ms (10 kB a second)
        w.run(10000);
        w.tab.rtcmEveryMs = 0;
        w.run(300);
        must("rtcm", w.teensy.rtcm.size() >= 399 * 256 && w.teensy.rtcm == w.tab.rtcmSent, "Corrections at the program's fastest pace (10 kB a second) reach the receiver whole and in order",
             fmt("%u of %u bytes", (unsigned)w.teensy.rtcm.size(), (unsigned)w.tab.rtcmSent.size()));
        endSteer(w);
        const size_t before = w.teensy.rtcm.size();
        w.tab.send(2233, std::vector<uint8_t>(1400, 0xD3), false);
        w.run(300);
        note("One corrections packet of 1400 bytes (no program sends one: AgOpenWeb and AgIO send 256 at a time)",
             fmt("%u bytes of it reach the receiver: the link's transmit buffer takes about three frames at once", (unsigned)(w.teensy.rtcm.size() - before)));
    }
    rules(w);
}

SCENARIO(User::WrongConnections, "conn-search-while-steering", "The other network is out of reach in the field; someone presses 'Search networks' while the tablet steers", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.dev.router.on = false;                // the farm's WiFi does not reach the field
    w.start();
    must("searching", w.dev.staSearching() && w.state()["wifi"]["st"].str() == "con", "With no program running the XIAO keeps searching for the farm's WiFi", "");
    const Reply s0 = w.get("/api/scan");
    w.run(3000);
    const Reply s1 = w.get("/api/scan");
    must("scan-free", s0.json["busy"].num() == 1 && s1.jsonOk && s1.json["list"].a.size() == 1 && s1.json["list"].a[0]["s"].str() == "Neighbour \"5G\"" && s1.json["list"].a[0]["open"].num() == 1,
         "'Search networks' works then (the hidden network left out)", s1.body);
    if (beginSteer(w))
    {
        w.run(1500);
        bool searched = false;
        const uint32_t scans = w.dev.scans;
        Reply sc;
        for (int k = 0; k < 200; k++)
        {
            w.run(100);
            searched |= w.dev.staSearching() || w.dev.scanning();
            if (k % 40 == 10) sc = w.get("/api/scan");
        }
        must("no-search", !searched && w.dev.scans == scans, "While the program has the module the XIAO does not search (a search takes the radio away from the tablet)", fmt("%u scan(s) started", w.dev.scans - scans));
        must("scan-refused", sc.jsonOk && sc.json["busy"].num() == 0 && has(sc.json["note"].str(), "Not searching now") && sc.json["list"].a.empty(), "'Search networks' answers why not", sc.body);
        must("said", w.said("not searching for 'Farm' while a program runs") && w.state()["wifi"]["st"].str() == "wait", "Messages and the page say so", "");
        endSteer(w);
        w.tab.on = false;
        const uint64_t t0 = fake::clockMs;
        must("again", w.runUntil([&]() { return w.dev.staSearching(); }, 16000) && w.said("searching for 'Farm' again"), "The program is closed: the search starts again by itself", "");
        note("Search for the other network back", fmt("%.1f s after the program's last packet", (fake::clockMs - t0) / 1000.0));
        w.dev.router.on = true;             // back at the farm
        must("joins", w.runUntil([&]() { return w.dev.staJoined; }, 8000), "Back in reach of the farm's WiFi: it joins", "");
    }
    rules(w);
}
