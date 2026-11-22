/* user_wrong_settings.cpp - settings typed wrong, wrong files chosen, things saved that make no sense.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
using namespace h;

SCENARIO(User::WrongSettings, "settings-bad-values", "The WiFi tab is saved with values that cannot work, one after the other", oneKit())
{
    World w(kit);
    w.start();
    struct Bad { const char* key; Form form; const char* says; const char* what; };
    const std::string name = w.knownSsid;
    const std::vector<Bad> bad = {
        { "name-empty",     settings("agopengps", ""), "network name", "no network name" },
        { "name-long",      settings("agopengps", std::string(33, 'A')), "network name", "a network name of 33 characters" },
        { "name-tab",       settings("agopengps", "Trator\tPuma"), "network name", "a network name with a tab in it" },
        { "pass-short",     settings("agopengps", name, "1234567"), "password: 8 to 63", "a password of 7 characters" },
        { "pass-long",      settings("agopengps", name, std::string(64, 'p')), "password: 8 to 63", "a password of 64 characters" },
        { "other-no-name",  { { "pw", "agopengps" }, { "apSsid", name }, { "apPass", "" }, { "staOn", "1" }, { "staSsid", "" }, { "staPass", "" }, { "host", "xiao" }, { "bridge", "0" } },
          "other network: no name", "'join another network' ticked with no name" },
        { "other-name-long", settings("agopengps", name, "", false, std::string(33, 'B'), "farm12345"), "other network name", "another network's name of 33 characters" },
        { "other-pass-short", settings("agopengps", name, "", false, "Farm", "farm1"), "other network password", "another network's password of 5 characters" },
        { "host-space",     settings("agopengps", name, "", false, "", "", "my xiao"), "name on the network", "a name on the network with a space" },
        { "host-empty",     settings("agopengps", name, "", false, "", "", ""), "name on the network", "no name on the network" },
        { "host-long",      settings("agopengps", name, "", false, "", "", std::string(25, 'x')), "name on the network", "a name on the network of 25 characters" },
        { "host-underscore", settings("agopengps", name, "", false, "", "", "trator_1"), "name on the network", "a name on the network with '_'" },
        { "host-dot",       settings("agopengps", name, "", false, "", "", "xiao.local"), "name on the network", "a name on the network with a dot" },
        { "nothing",        { { "pw", "agopengps" } }, "network name", "a form with the password only" },
    };
    for (const Bad& b : bad)
    {
        const Reply r = w.post("/api/settings", b.form);
        must(b.key, r.code == 400 && has(r.error(), b.says), std::string("Saving ") + b.what + " is refused, and the page is told which field", fmt("code %d '%s'", r.code, r.error().c_str()));
    }
    w.run(3000);
    must("nothing-saved", w.dev.flashWrites == 0 && w.xiao.restarts.empty() && !w.said("Settings saved"), "After all of that nothing was saved and the XIAO did not restart",
         fmt("%u flash writes, %u restarts", w.dev.flashWrites, (unsigned)w.xiao.restarts.size()));
    const Reply accent = w.post("/api/settings", settings("agopengps", "Trator Jo\xC3\xA3o"));
    must("accent", accent.code == 400, "A network name with an accent (Joao with a tilde) is refused", fmt("code %d", accent.code));
    tell("accent-told", has(accent.error(), "accent") || has(accent.error(), "letters"), "... and the page says that accents are the reason",
         "it answers '" + accent.error() + "', and the name has 11 characters");
    const int boots = w.xiao.boots;
    const Reply ok = w.post("/api/settings", settings("agopengps", name, "", false, "", "", "Puma-1"));
    w.noteUserRestart();
    must("then-good", ok.code == 200 && backAfterRestart(w, boots) && w.flashStr("host") == "puma-1", "Good values are saved straight after (the name on the network in small letters)", ok.error());
    w.run(3000);
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-special-characters", "Network names with quotes, <, &, %, + and backslashes", oneKit())
{
    World w(kit);
    w.start();
    const std::string ap = "Trator \"Puma\" <b>&'\\ 50%+1=?", sta = "Monte & \"Vale\" \\ <5G> +1%";
    w.dev.router.ssid = sta;
    int boots = w.xiao.boots;
    const Reply r = w.post("/api/settings", settings("agopengps", ap, "", false, sta, "farm12345"));
    w.noteUserRestart();
    must("saved", r.code == 200 && backAfterRestart(w, boots), "They are saved", fmt("code %d '%s'", r.code, r.error().c_str()));
    w.knownSsid = ap;
    w.run(5000);
    must("exact", w.dev.apSsid == ap && w.dev.staSsid == sta && w.dev.staJoined, "The XIAO's network has exactly that name, and it joins the other one", w.dev.apSsid + " / " + w.dev.staSsid);
    const JsonDoc st = w.state();
    const Reply se = w.get("/api/settings");
    must("page", st["wifi"]["ap"].str() == ap && st["wifi"]["sta"].str() == sta && se.jsonOk && se.json["apSsid"].str() == ap && se.json["staSsid"].str() == sta,
         "The page gets both names back whole (valid JSON)", se.body);
    w.runUntil([&]() { return has(w.teensy.info, "\"st\":\"ok\""); }, 6000);
    bool ok = false;
    const JsonDoc inf = JsonDoc::parse(w.teensy.info, ok);
    must("info", ok && inf["ap"].str() == ap && inf["sta"].str() == sta, "The Teensy's page gets them whole too", w.teensy.info);
    must("messages", w.get("/api/log?after=0&xafter=0").jsonOk && w.said("WiFi: joined '" + sta + "'"), "Messages show them (valid JSON)", "");
    const Reply sc = w.get("/api/scan");
    w.run(3000);
    const Reply list = w.get("/api/scan");
    bool found = false, neighbour = false;
    for (const JsonDoc& n : list.json["list"].a) { found |= n["s"].str() == sta; neighbour |= n["s"].str() == "Neighbour \"5G\""; }
    must("scan", sc.jsonOk && list.jsonOk && found && neighbour, "The list of networks shows such names whole", list.body);

    // names made of quotes only: every one takes two characters in what the XIAO tells the Teensy
    const std::string q32(32, '"');
    w.dev.router.ssid = q32;
    boots = w.xiao.boots;
    const Reply r2 = w.post("/api/settings", settings("agopengps", q32, "", false, q32, "farm12345"));
    w.noteUserRestart();
    must("quotes-saved", r2.code == 200 && backAfterRestart(w, boots), "Two names of 32 quotes are saved too", r2.error());
    w.knownSsid = q32;
    w.runUntil([&]() { return w.dev.staJoined; }, 8000);
    w.run(1000);
    const uint32_t infos = w.teensy.infoRx;
    w.run(11000);
    must("quotes-page", w.state()["wifi"]["ap"].str() == q32, "... and the XIAO's own page shows them", "");
    tell("quotes-info", w.teensy.infoRx > infos, "... and the Teensy's page still gets the XIAO's state every 5 s",
         "nothing is sent: the text is longer than the 250 characters a frame carries (only with names made almost all of quotes or backslashes)");
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-other-network-wrong", "The other network's password is typed wrong, then its name; then both right", oneKit())
{
    World w(kit);
    w.start();
    int boots = w.xiao.boots;
    Reply r = w.post("/api/settings", settings("agopengps", w.knownSsid, "", false, "Farm", "farm12346"));
    w.noteUserRestart();
    must("saved", r.code == 200 && backAfterRestart(w, boots), "A wrong password for the other network is saved (the XIAO cannot know)", r.error());
    w.run(20000);
    JsonDoc st = w.state();
    must("own-works", st["wifi"]["st"].str() == "con" && !st["wifi"].has("ip") && w.dev.apOn, "The XIAO's own network and page keep working while it cannot join", st["wifi"]["st"].str());
    tell("told", w.said("password") || w.said("could not join") || w.said("NOT joined"), "Messages say that the other network refused it (wrong password?)",
         "the page shows 'connecting' for ever, with nothing in Messages");
    boots = w.xiao.boots;
    r = w.post("/api/settings", settings("agopengps", w.knownSsid, "", false, "Farmm", "farm12345"));
    w.noteUserRestart();
    must("saved-2", r.code == 200 && backAfterRestart(w, boots), "A name that does not exist is saved too", r.error());
    w.run(10000);
    must("own-works-2", w.state()["wifi"]["st"].str() == "con", "... and the page keeps working", "");
    boots = w.xiao.boots;
    r = w.post("/api/settings", settings("agopengps", w.knownSsid, "", false, "Farm", "farm12345"));
    w.noteUserRestart();
    must("right", r.code == 200 && backAfterRestart(w, boots) && w.runUntil([&]() { return w.dev.staJoined; }, 8000), "With both right it joins", r.error());
    w.run(2000);
    st = w.state();
    must("state", st["wifi"]["st"].str() == "ok" && st["wifi"]["ip"].str() == "192.168.1.50", "The page shows its address there", "");
    w.dev.router.on = false;                // the router is switched off
    must("lost", w.runUntil([&]() { return w.said("WiFi: lost the network 'Farm'"); }, 3000), "The router goes off: Messages say the network was lost", "");
    w.run(5000);
    w.dev.router.on = true;
    must("back", w.runUntil([&]() { return w.dev.staJoined; }, 8000), "The router comes back: it joins again by itself", "");
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-what-is-kept", "Saving again: an empty password field keeps the password, a page of before the bridge keeps the bridge", oneKit())
{
    World w(kit);
    w.bridgeSaved("trator-2026");
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.start();
    w.runUntil([&]() { return w.dev.staJoined; }, 5000);
    int boots = w.xiao.boots;
    // a page of before 22 Nov 2026 (kept in a browser): it sends no 'bridge' field; both password fields are empty
    Reply r = w.post("/api/settings", { { "pw", "trator-2026" }, { "apSsid", w.knownSsid }, { "apPass", "" }, { "staOn", "1" }, { "staSsid", "Farm" }, { "staPass", "" }, { "host", "puma" } });
    w.noteUserRestart();
    must("saved", r.code == 200 && backAfterRestart(w, boots), "Saved", r.error());
    w.run(5000);
    must("kept", w.flashStr("apPass") == "trator-2026" && w.flashStr("staPass") == "farm12345" && w.flashBool("bridge") && w.dev.udpOpen() == 2 && w.dev.staJoined && w.flashStr("host") == "puma",
         "Both passwords and the WiFi bridge are kept; only the name changed", fmt("bridge %d, joined %d", (int)w.flashBool("bridge"), (int)w.dev.staJoined));
    r = w.post("/api/settings", settings("trator-2026", w.knownSsid, "agopengps", true, "Farm", ""));
    must("default-pass", r.code == 400 && has(r.error(), "password of your own"), "Back to the password every XIAO starts with, the bridge still ticked: refused", fmt("code %d", r.code));
    boots = w.xiao.boots;
    w.dev.router.ssid = "Open net";
    w.dev.router.pass = "";
    r = w.post("/api/settings", settings("trator-2026", w.knownSsid, "", true, "Open net", ""));
    w.noteUserRestart();
    must("open", r.code == 200 && backAfterRestart(w, boots) && w.runUntil([&]() { return w.dev.staJoined; }, 8000) && w.flashStr("staPass").empty(),
         "Another network with no password: the old network's password is not used for it", fmt("saved password '%s'", w.flashStr("staPass").c_str()));
    boots = w.xiao.boots;
    r = w.post("/api/settings", { { "pw", "trator-2026" }, { "apSsid", w.knownSsid }, { "apPass", "" }, { "staOn", "0" }, { "staSsid", "Open net" }, { "staPass", "" }, { "host", "puma" }, { "bridge", "1" } });
    w.noteUserRestart();
    must("off", r.code == 200 && backAfterRestart(w, boots), "'Join another network' unticked", r.error());
    w.run(4000);
    must("off-state", !w.dev.staBegun && w.state()["wifi"]["st"].str() == "off" && w.flashStr("staSsid") == "Open net", "It no longer joins it, and the name stays in the form for next time", "");
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-flash-nonsense", "The settings in the XIAO's flash make no sense (another program used the chip before, or a bad write)", oneKit())
{
    World w(kit);
    w.flash("apSsid", std::string(40, 'Z'));
    w.flash("apPass", "short");
    w.flash("host", std::string(30, 'h'));
    w.flash("staOn", true);
    w.flash("staSsid", "");
    w.flash("bridge", true);
    w.start();
    must("defaults", w.dev.apOn && w.dev.apSsid == w.knownSsid && w.dev.apPass == "agopengps" && w.dev.mdnsHost == "xiao", "It starts with its own name, the first password and the name 'xiao'",
         w.dev.apSsid + " / " + w.dev.apPass + " / " + w.dev.mdnsHost);
    must("bridge-off", w.dev.udpOpen() == 0 && w.state()["bridge"]["on"].num() == 0, "The WiFi bridge stays off (it never runs with the password every XIAO starts with)", "");
    must("no-sta", !w.dev.staBegun && w.state()["wifi"]["st"].str() == "off", "It does not search for a network with no name", "");
    w.tab.on = w.tab.engaged = true;
    w.run(5000);
    must("nothing-passed", w.tab.nPassed == 0 && !w.teensy.steering(), "A tablet's program is not passed to the Teensy", "");
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-forgotten-password", "The password of the XIAO's network was forgotten: 'wifi-reset' on the USB cable", oneKit())
{
    World w(kit);
    w.bridgeSaved("something-forgotten");
    w.flash("apSsid", "Trator");
    w.flash("host", "puma");
    w.knownSsid = "Trator";
    w.knownPass = "what-was-it";              // the user's devices no longer join
    w.start();
    w.run(3000);
    const int a = w.ask(GET("/api/state"));
    w.run(3000);
    must("locked-out", !w.reply(a).done, "With the wrong password nobody reaches the page", "");
    const int boots = w.xiao.boots;
    w.usb("wifi-reset");
    w.noteUserRestart();
    must("reset", backAfterRestart(w, boots, 3000) && w.said("WiFi settings back to the defaults (USB command)"), "'wifi-reset' restarts the XIAO", "");
    w.knownSsid = "XIAO-B2A1";
    w.knownPass = "agopengps";
    w.run(4000);
    const JsonDoc st = w.state();
    must("defaults", w.dev.apSsid == "XIAO-B2A1" && w.dev.apPass == "agopengps" && st["wifi"]["host"].str() == "xiao" && st["bridge"]["on"].num() == 0 && w.dev.udpOpen() == 0,
         "Its network is XIAO-xxxx / agopengps again, and the WiFi bridge is off", w.dev.apSsid);
    w.dev.usbOut.clear();
    w.usb("estado");
    w.run(200);
    must("old-names", w.said("Own WiFi 'XIAO-B2A1'"), "The old command 'estado' still prints the state", w.dev.usbOut.substr(0, 200));
    rules(w);
}

SCENARIO(User::WrongSettings, "settings-subnet-values", "A subnet for the module typed wrong; a good one while autosteer is engaged", oneKit())
{
    World w(kit);
    w.start();
    for (const char* v : { "", "192.168", "192.168.5.126", "abc", "0.1.2", "127.0.0", "224.1.1", "192.168.256", "192.168.5 ", "-1.2.3", "192.168.5x", "192,168,5", "1e2.3.4", "999.1.1", "192.168.-5" })
    {
        const Reply r = w.post("/api/subnet", { { "subnet", v }, { "pw", "agopengps" } });
        must(std::string("bad-") + enc(v), r.code == 400 && has(r.error(), "three numbers"), std::string("'") + v + "' is refused, with an example of a good one", fmt("code %d '%s'", r.code, r.error().c_str()));
    }
    w.run(1000);
    must("none-sent", w.teensy.subnets.empty() && w.teensy.subnetRefused.empty() && w.teensy.restarts == 0, "None of them reached the Teensy", "");
    w.teensy.engaged = true;
    const Reply r = w.post("/api/subnet", { { "subnet", "192.168.5" }, { "pw", "agopengps" } });
    w.run(1500);
    must("engaged", w.teensy.subnets.empty() && w.teensy.subnetRefused.size() == 1, "With autosteer engaged the Teensy refuses a good one", "");
    const Reply lg = w.get("/api/log?after=0&xafter=0");
    bool told = false;
    for (const JsonDoc& l : lg.json["t"].a) told |= has(l["x"].str(), "refused: not while autosteer is engaged");
    tell("engaged-told", r.code != 200 || told, "... and the user is told: the page's answer or the module's lines in Messages", fmt("the page got code %d", r.code));
    tell("engaged-answer", r.code != 200, "... by the answer itself (the page says 'sent' and the module never changes)", "the page gets 'ok': the refusal is only a line in Messages");
    rules(w);
}

SCENARIO(User::WrongSettings, "update-wrong-files", "The wrong file is chosen for the XIAO's own update", bothKits())
{
    World w(kit);
    w.start();
    const std::vector<uint8_t> good = repoFile(info(kit).bin);
    auto refused = [&](const char* key, const char* name, const std::vector<uint8_t>& file, const char* what) {
        const Reply r = w.upload("/update?pw=agopengps", name, file);
        must(key, r.done && r.code == 400 && !w.dev.updDone, std::string(what) + " is refused", fmt("code %d '%s' %s", r.code, r.error().c_str(), r.why.c_str()));
        note(std::string("The page's message for ") + what, r.error());
        return r;
    };
    refused("hex", "v4.5_AiO_WEB.ino.hex", makeHex(200000), "The Teensy's .hex");      // (the page's own script refuses these before sending them)
    refused("text", "notes.txt", std::vector<uint8_t>(3000, 'a'), "A small text file");
    std::vector<uint8_t> cut(good.begin(), good.begin() + good.size() * 6 / 10);
    refused("cut", "half.bin", cut, "A .bin that was cut short (a download that stopped)");
    std::vector<uint8_t> flipped = good;
    flipped[flipped.size() / 2] ^= 0x10;
    flipped[flipped.size() / 3] ^= 0x01;
    refused("damaged", "damaged.bin", flipped, "A .bin with two wrong bits in it");
    std::vector<uint8_t> noHead = good;
    noHead[0] = 0x00;
    refused("no-magic", "odd.bin", noHead, "A .bin whose first byte is wrong");
    w.run(3000);
    must("kept", w.xiao.restarts.empty() && w.dev.updInstalled == 0 && w.state()["ver"].str() == "4.5", "After all of them the program that was running keeps running, with no restart", "");

    // the file for the USB cable ("..._USB.bin": the bootloader first, then the program) chosen for the page
    std::string usbName = info(kit).bin;
    usbName.insert(usbName.size() - 4, "_USB");
    const std::vector<uint8_t> usb = repoFile(usbName);
    const Reply u = w.upload("/update?pw=agopengps", "firmware.bin", usb);
    tell("usb-file", u.code == 400, "The file for the USB cable (bootloader first) is refused when sent under another name",
         "the XIAO accepts it and restarts (it checks the first byte and the chip only; the page's script refuses such a file by its name, '_USB' or 'merged', so only a renamed one gets here). "
         "What the chip then does with a bootloader where its program should be is not tested here");
    if (u.code == 200) { w.noteUserRestart(); w.run(3000); }
    rules(w);
}

SCENARIO(User::WrongSettings, "update-header-byte-later", "A good .bin that happens to have the byte E9 where the second piece of the upload starts", bothKits())
{
    World w(kit);
    w.start();
    const uint16_t other = info(kit == Kit::C3 ? Kit::S3 : Kit::C3).chip;
    for (size_t at : { (size_t)1436, (size_t)2872 })
    {
        std::vector<uint8_t> bin = makeImage(info(kit).chip, 60000, 7);
        bin[at] = 0xE9;                     // as the first byte of a program file ...
        bin[at + 12] = (uint8_t)other;      // ... and where its chip number would be, another number
        bin[at + 13] = (uint8_t)(other >> 8);
        sealImage(bin);
        const int boots = w.xiao.boots;
        const Reply r = w.upload("/update?pw=agopengps", "good.bin", bin);
        const bool ok = r.code == 200;
        must(fmt("accepted-%u", (unsigned)at), ok, fmt("With it %u bytes in, the file is accepted (it is a good program for this XIAO)", (unsigned)at), fmt("code %d '%s'", r.code, r.error().c_str()));
        if (ok) { w.noteUserRestart(); backAfterRestart(w, boots); w.run(3000); }
    }
    rules(w);
}
