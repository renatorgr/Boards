/* user_wrong_settings.cpp - user 3: sets things up, but some values are wrong (a typo, a box ticked by mistake).
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

static std::map<std::string, std::string> sprayArgs(const std::string& rate = "240", const std::string& nzFl = "1.6")
{
    return { { "pw", "agopengps" }, { "rate", rate }, { "nzSp", "0.5" }, { "nzFl", nzFl }, { "nzBar", "3" },
             { "pLow", "1" }, { "pHigh", "6" }, { "fTol", "20" } };
}

SCENARIO(User::WrongSettings, "relay-active-high-ticked", "ticks 'relay active high' in AgOpenWeb's machine settings by mistake", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    if (m.bi.sprayer) w.addRig(m);
    w.aow.toolSections = 8;
    w.aow.startup(true, { 1, 2, 3, 4, 5, 6, 7, 8 });         // invert = 1
    const uint64_t from = w.now();
    w.aow.sections = 0;                                     // headland: every section off
    w.run(12000);
    note("relays with every section off", hex(m.relays()));
    tell("inverted-in-log", contains(pageLog(m), "INVERTED"), "the module's log says the outputs are inverted ('relay active high' ticked)");
    if (m.bi.sprayer)
    {
        const std::string screen = screenMessages(w, from);
        tell("inverted-flow-alarm", contains(screen, "flow with the sections off"),
             "the flow meter sees the boom spraying with every section off and AgOpenWeb's screen says so", screen.empty() ? "(nothing on the screen)" : screen);
    }
    else
        tell("inverted-seeder-said", contains(screenMessages(w, from), "invert"),
             "AgOpenWeb's screen warns that the seed drill's outputs are inverted (every row sows on the headland)",
             "only the module's log says it; the seed drill has no sensor to notice");
    statusAlwaysJson(m);
}

SCENARIO(User::WrongSettings, "page-typos", "types impossible values on the module's page", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    auto refused = [&](const std::string& uri, std::map<std::string, std::string> a, const std::string& what) {
        Http r = m.http(HTTP_POST, uri, a);
        std::string key = "refuses-";
        for (char c : what) key += isalnum((unsigned char)c) ? (char)tolower((unsigned char)c) : '-';
        must(key, r.code == 400 || r.code == 403, "the page refuses " + what, fmt("answer %d %s", r.code, r.body.c_str()));
    };
    // the network settings
    std::map<std::string, std::string> net = { { "pw", "agopengps" }, { "apSsid", "Module" }, { "apPass", "" }, { "staOn", "1" },
                                               { "staSsid", "Tractor" }, { "staPass", "tractor123" }, { "host", m.bi.sprayer ? "sprayer" : "seeder" },
                                               { "subnet", "192.168.5" }, { "ipLast", "123" } };
    auto with = [](std::map<std::string, std::string> a, const std::string& k, const std::string& v) { a[k] = v; return a; };
    refused("/api/wifi", with(net, "pw", "wrong"), "a wrong password");
    refused("/api/wifi", with(net, "apPass", "short7!"), "an own-network password of 7 characters");
    refused("/api/wifi", with(net, "subnet", "192.168.5.1"), "a subnet with four numbers");
    refused("/api/wifi", with(net, "subnet", "300.1.1"), "a subnet number over 255");
    refused("/api/wifi", with(net, "subnet", "192.168"), "a subnet with two numbers");
    refused("/api/wifi", with(net, "ipLast", "1"), "module IP .1 (the router)");
    refused("/api/wifi", with(net, "ipLast", "255"), "module IP .255 (broadcast)");
    refused("/api/wifi", with(net, "staSsid", ""), "an empty tractor network name");
    refused("/api/wifi", with(net, "apSsid", std::string(33, 'x')), "an own-network name of 33 characters");

    Http r = m.http(HTTP_POST, "/api/wifi", with(net, "host", "my sprayer"));
    tell("host-name-checked", r.code == 400, "the page refuses a host name with a space (http://my sprayer.local/ cannot work)",
         fmt("answer %d", r.code));
    if (r.code == 200) w.run(3000);

    if (m.bi.sprayer)
    {
        refused("/api/spray", with(sprayArgs(), "rate", "-5"), "a negative target");
        refused("/api/spray", with(sprayArgs(), "rate", "abc"), "a target that is not a number");
        refused("/api/spray", with(sprayArgs(), "rate", "99999"), "a target of 99999 L/ha");
        refused("/api/spray", with(sprayArgs(), "rate", ""), "an empty target");
        refused("/api/spray", with(sprayArgs(), "nzBar", "0"), "a chart pressure of 0 bar");
        refused("/api/spray", with(with(sprayArgs(), "pLow", "5"), "pHigh", "3"), "a minimum pressure above the maximum");
        refused("/api/spray", with(sprayArgs(), "fTol", "0"), "a flow tolerance of 0 %");
        refused("/api/spray", with(sprayArgs(), "nzSp", "6"), "nozzles 6 m apart");
        std::map<std::string, std::string> cal = { { "pw", "agopengps" }, { "ppl", "450" }, { "pmax", "10" }, { "pdiv", "0.5" }, { "sensors", "3" } };
        refused("/api/calib", with(cal, "ppl", "0"), "0 pulses per litre");
        refused("/api/calib", with(cal, "ppl", "nan"), "pulses per litre 'nan'");
        refused("/api/calib", with(cal, "pdiv", "2"), "a divider of 2");
        refused("/api/calib", with(cal, "sensors", "4"), "sensors 4");
        refused("/api/calib", with(cal, "sensors", "abc"), "sensors 'abc' (not a number)");
    }
    else
    {
        std::map<std::string, std::string> rel = { { "pw", "agopengps" }, { "first", "1" }, { "secs", "19" } };
        refused("/api/relays", with(rel, "first", "65"), "first section 65");
        refused("/api/relays", with(rel, "secs", "-1"), "-1 sections");
        refused("/api/relays", with(rel, "first", "1.5"), "first section 1.5");
    }
    statusAlwaysJson(m);
}

SCENARIO(User::WrongSettings, "flash-garbage", "a module whose saved settings are garbage (flash from an odd old build)", allBoards())
{
    World w;
    Module& m = w.add(board);
    if (!m.bi.sprayer) m.dev.cableIn = true;
    // what a broken flash could hold: values out of every range, wrong lengths
    const std::string ns = m.bi.sprayer ? "sprayer" : "seeder";
    auto put = [&](const char* k, char type, std::vector<uint8_t> b) { m.dev.prefs[ns][k] = fake::PrefValue { type, b }; };
    auto putF = [&](const char* k, float f) { std::vector<uint8_t> b(4); memcpy(b.data(), &f, 4); put(k, 'b', b); };
    put("apPass", 's', { 'a', 'b', 'c' });
    put("ipLast", 'u', { 0 });
    put("invert", 'u', { 7 });
    put("pins", 'b', std::vector<uint8_t>(24, 99));
    put("subnet", 'b', { 10, 0 });
    put("first", 'u', { 200 });
    put("secs", 'u', { 250 });
    putF("rate", NAN);
    putF("ppl", -1);
    putF("pdiv", INFINITY);
    putF("nzSp", 1e30f);
    put("fTol", 'u', { 0 });
    put("sensors", 'u', { 0xFF });
    put("secN", 'u', { 200 });
    m.powerOn();
    w.aow.toolSections = 8;
    w.run(1000);
    Json s = m.status();
    must("boots", m.on() && s.has("fw"), "the module starts and its page answers");
    must("own-network-up", m.dev.apOn && m.dev.apPass == "agopengps", "its own network is up with the default password (a 3-letter one is not allowed)",
         fmt("ap %s, password '%s'", m.dev.apOn ? "on" : "off", m.dev.apPass.c_str()));
    must("ip-sane", s["wifi"]["ipLast"].num() == 123, "the module IP goes back to .123", fmt("ipLast %g", s["wifi"]["ipLast"].num()));
    if (m.bi.sprayer)
    {
        Json st = m.http(HTTP_GET, "/api/settings").json;
        must("calib-sane", st["ppl"].num() == 450 && st["pdiv"].num() == 0.5 && st["nzSp"].num() <= 5 && st["rate"].num() == 0,
             "the calibration and spraying values go back to the defaults", fmt("ppl %g pdiv %g nzSp %g rate %g", st["ppl"].num(), st["pdiv"].num(),
             st["nzSp"].num(), st["rate"].num()));
    }
    else
        must("first-sane", s["out"]["first"].num() <= 64 && s["out"]["secs"].num() <= 64, "first section and sections back in 0-64",
             fmt("first %g secs %g", s["out"]["first"].num(), s["out"]["secs"].num()));
    // 'invert' holds 7: read as "inverted", so every relay closes with nothing asked (only 0 or 1 is ever saved)
    must("invert-read-as-bit", s["aog"]["invert"].num() <= 1 && m.relays() == 0,
         "the inverted-outputs setting is read back as 0 or 1 (no relay closes with nothing asked)",
         fmt("invert %g, relays on %s", s["aog"]["invert"].num(), hex(m.relays()).c_str()));
    statusAlwaysJson(m);
}

SCENARIO(User::WrongSettings, "long-lift-times", "sets the hydraulic lift's lower time to 60 s in AgOpenWeb", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    if (!m.bi.sprayer) m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "0" }, { "secs", "0" } });   // the program's pin config
    // relay 8 = hydraulic lower (pin function 17), relay 7 = raise (18)
    w.aow.sendPinConfig({ 1, 2, 3, 4, 5, 6, 18, 17 });
    auto lowerFor = [&](uint8_t seconds) {
        w.aow.sendMachineConfig(seconds, seconds, 1, false);
        w.aow.hydLift = 2;                                  // raised first
        w.run(1000);
        w.aow.hydLift = 1;                                  // then lower
        int onMs = 0;
        for (int t = 0; t < (seconds + 10) * 1000; t++) { w.run(1); if ((m.relays() >> 7) & 1) onMs++; }
        w.aow.hydLift = 0;
        w.run(500);
        return onMs;
    };
    const int four = lowerFor(4);
    must("lower-4s", abs(four - 4000) <= 250, "with lower time 4 s, the lower relay is on for 4 s", fmt("on %d ms", four));
    const int sixty = lowerFor(60);
    must("lower-60s", abs(sixty - 60000) <= 250, "with lower time 60 s, the lower relay is on for 60 s", fmt("on %d ms", sixty));
}

SCENARIO(User::WrongSettings, "wrong-wifi-password", "types the tractor Wi-Fi's password (or name) wrong on the page", sprayerBoards())
{
    World w;
    Module& m = w.add(board);
    m.powerOn();
    w.run(500);
    joinTractorWifi(w, m, "tractor12");                     // one character short
    w.aow.startup();
    w.aow.sections = first(8);
    w.run(10000);
    must("stays-off", m.relays() == 0, "no relay closes (nothing reaches the module)", hex(m.relays()));
    Json s = m.status();
    std::string said = pageLog(m) + s["wifi"]["why"].str();
    tell("says-wrong-password", contains(said, "password"), "the page says the tractor network refused the password",
         "the page only says '" + s["wifi"]["sta"].str() + " (not joined)'");
    // the name wrong this time
    m.http(HTTP_POST, "/api/wifi", { { "pw", "agopengps" }, { "apSsid", m.dev.apSsid }, { "apPass", "" }, { "staOn", "1" },
                                     { "staSsid", "Tractr" }, { "staPass", "tractor123" }, { "host", "sprayer" }, { "subnet", "192.168.5" },
                                     { "ipLast", "123" } });
    w.run(10000);
    said = pageLog(m);
    tell("says-not-found", contains(said, "not found") || contains(said, "no network"), "the page says no network of that name is in range",
         "the page only says 'Tractr (not joined)'");
}

SCENARIO(User::WrongSettings, "wrong-subnet-from-program", "changes the subnet in AgOpenWeb to 192.168.1 while the router stays on 192.168.5", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.startup();
    w.aow.sendSubnet(192, 168, 1);
    w.run(4000);
    Json s = m.status();
    must("took-subnet", s["wifi"]["subnet"].str() == "192.168.1", "the module takes the program's subnet and restarts", s["wifi"]["subnet"].str());
    w.aow.sections = first(8);
    w.run(3000);
    must("off-when-unreachable", m.relays() == 0, "on the wrong subnet nothing reaches it: every relay stays open", hex(m.relays()));
    // AgOpenWeb looks for modules (PGN 202 to everyone): the module must answer with where it is
    const uint64_t from = w.now();
    w.aow.sendScan();
    w.run(500);
    auto r = w.aow.heardPgn(203, from);
    must("answers-scan", !r.empty() && r[0].data.size() >= 9 && r[0].data[5] == 192 && r[0].data[6] == 168 && r[0].data[7] == 1,
         "it still answers AgOpenWeb's scan, saying it is at 192.168.1.x", fmt("%zu answers", r.size()));
    // and the program can put it right
    w.aow.sendSubnet(192, 168, 5);
    w.run(4000);
    std::string bad;
    const int n = driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 3000, &bad);
    must("back-on-subnet", n == 0, "PGN 201 with 192.168.5 brings it back and it works again", bad);
}

SCENARIO(User::WrongSettings, "nozzle-flow-typo", "types the nozzle flow 0.5 L/min instead of 1.6 (as on 20 Nov)", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    m.http(HTTP_POST, "/api/spray", sprayArgs("240", "0.5"));
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 20000);
    Json s = m.status();
    note("module says", fmt("%.1f L/ha, needs %.1f bar, slow down to %.1f km/h, expected %.1f L/min (real %.1f)", s["rate"]["lha"].num(),
                            s["rate"]["needBar"].num(), s["rate"]["hintKmh"].num(), s["rate"]["expected"].num(), s["spr"]["lpm"].num()));
    const std::string screen = screenMessages(w, from);
    tell("typo-gives-flow-alarm", contains(screen, "flow too high"),
         "the flow meter shows more flow than the settings expect and AgOpenWeb's screen says 'flow too high'", screen.empty() ? "(nothing)" : screen);
    must("rate-still-real", fabs(s["rate"]["lha"].num() - 240) < 12, "the L/ha stays the real one (it comes from the flow meter)",
         fmt("%.1f L/ha", s["rate"]["lha"].num()));
}

SCENARIO(User::WrongSettings, "flow-meter-calibration-typo", "types 45 pulses per litre instead of 450", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    m.http(HTTP_POST, "/api/spray", sprayArgs());
    m.http(HTTP_POST, "/api/calib", { { "pw", "agopengps" }, { "ppl", "45" }, { "pmax", "10" }, { "pdiv", "0.5" }, { "sensors", "3" } });
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 20000);
    Json s = m.status();
    note("module says", fmt("%.1f L/min, %.0f L/ha", s["spr"]["lpm"].num(), s["rate"]["lha"].num()));
    const std::string screen = screenMessages(w, from);
    tell("calibration-typo-alarm", contains(screen, "flow too high"), "AgOpenWeb's screen says 'flow too high'", screen.empty() ? "(nothing)" : screen);
}

SCENARIO(User::WrongSettings, "first-section-past-drill", "sets 'relay 1 = section 30' on a 19-section seed drill", seederBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.toolSections = 19;
    w.aow.startup();
    Http r = m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "30" }, { "secs", "19" } });
    w.aow.sections = first(19);
    w.run(3000);
    note("relays with all 19 sections asked", hex(m.relays()));
    const std::string log = pageLog(m);
    tell("first-past-drill-said", r.code == 400 || contains(log, "no relay"),
         "the page refuses it, or says no relay follows any section", fmt("answer %d; log: %s", r.code, log.c_str()));
}
