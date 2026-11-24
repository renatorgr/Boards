/* user_unconfigured.cpp - a XIAO (or the Teensy beside it) that was never set up: things used as they come.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
using namespace h;

SCENARIO(User::Unconfigured, "new-bridge-needs-password", "The WiFi bridge is switched on while the network still has the password every XIAO starts with", oneKit())
{
    World w(kit);
    w.start();
    const Reply r = w.post("/api/settings", settings("agopengps", w.knownSsid, "", true));
    must("refused", r.code == 400 && has(r.error(), "password of your own"), "It is refused, and the page is told to set a password of one's own first", fmt("code %d '%s'", r.code, r.error().c_str()));
    w.run(3000);
    must("not-saved", w.dev.flashWrites == 0 && w.xiao.restarts.empty(), "Nothing is saved and it does not restart", fmt("%u writes", w.dev.flashWrites));
    w.tab.on = w.tab.engaged = true;        // the tablet starts AgOpenWeb all the same
    w.run(8000);
    must("nothing-passed", w.tab.nPassed == 0 && w.teensy.netIn == 0 && w.tab.got == 0 && !w.teensy.steering(), "Nothing of the tablet's program reaches the Teensy",
         fmt("%u passed, %u answers", w.tab.nPassed, w.tab.got));
    const JsonDoc st = w.state();
    tell("told", st["bridge"]["on"].num() == 0 && (w.said("program") || st["bridge"]["other"].num() > 0 || !st["bridge"]["prog"].str().empty()),
         "The XIAO says that a program on its network is calling the module while the bridge is off", "nothing in Messages (with the bridge off its port is closed: the XIAO cannot hear the program); the Status tab does show 'WiFi bridge: off'");
    rules(w);
}

SCENARIO(User::Unconfigured, "new-no-teensy", "The XIAO is powered alone (USB on the desk, or the Teensy is off) and its page is used", oneKit())
{
    World w(kit);
    w.teensy.on = false;
    w.start();
    w.run(5000);
    const JsonDoc st = w.state();
    must("state", st["link"]["talking"].num() == 0 && st["link"]["lastAgo"].num() == -1 && st["link"]["frames"].num() == 0, "The page shows no data from the module",
         fmt("talking %g, last %g", st["link"]["talking"].num(), st["link"]["lastAgo"].num()));
    const Reply t = w.get("/api/teensy");
    must("teensy", t.jsonOk && t.json["ago"].num() == -1 && !t.json.has("s"), "The module's status is empty, not something made up", t.body);
    Reply r = w.post("/api/subnet", { { "subnet", "192.168.5" }, { "pw", "agopengps" } });
    must("subnet", r.code == 409 && has(r.error(), "no data from the module") && r.tookMs() < 1000, "A subnet for the module: refused at once, with the reason", fmt("code %d '%s' in %u ms", r.code, r.error().c_str(), r.tookMs()));
    r = w.post("/api/teensy-revert", { { "pw", "agopengps" } });
    must("revert", r.code == 409 && has(r.error(), "no data from the module") && r.tookMs() < 1000, "The previous firmware back: refused at once, with the reason", fmt("code %d in %u ms", r.code, r.tookMs()));
    const std::vector<uint8_t> hex = makeHex(60000);
    r = w.upload(fmt("/api/teensy-update?pw=agopengps&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    must("update", r.code == 400 && has(r.error(), "no data from the module") && r.tookMs() < 3000, "A .hex for the module: refused, with the reason", fmt("code %d '%s' in %u ms", r.code, r.error().c_str(), r.tookMs()));
    r = w.post("/api/update-check", { { "for", "teensy" }, { "pw", "agopengps" } });
    must("update-check", r.code == 409 && has(r.error(), "no data from the module") && r.tookMs() < 1000,
         "The page's check before a .hex is sent (or downloaded): refused at once, with the reason", fmt("code %d '%s' in %u ms", r.code, r.error().c_str(), r.tookMs()));
    r = w.post("/api/update-check", { { "pw", "agopengps" } });
    must("own-update-check", r.code == 200 && r.jsonOk, "The check before its own update says yes", fmt("code %d '%s'", r.code, r.error().c_str()));
    must("own-update", w.upload("/update?pw=agopengps", "x.bin", makeImage(info(kit).chip, 30000)).code == 200, "Its own update still works without the Teensy", "");
    w.noteUserRestart();
    w.run(4000);
    rules(w);
}

SCENARIO(User::Unconfigured, "new-gps2-port-not-set", "The Teensy's GPS2 port was never set to 'XIAO': it ignores the XIAO", oneKit())
{
    World w(kit);
    w.teensy.gps2On = false;
    w.start();
    w.run(6000);
    const JsonDoc st = w.state();
    must("no-data", st["link"]["talking"].num() == 0 && st["link"]["frames"].num() == 0, "The page shows no data from the module", "");
    tell("told", st["link"]["talking"].num() == 0 && st["link"]["hellos"].num() >= 3, "The page shows the XIAO keeps calling and gets no answer (so the user looks at the Teensy's GPS2 setting)",
         fmt("%g hellos sent, 0 frames back", st["link"]["hellos"].num()));
    const Reply r = w.post("/api/subnet", { { "subnet", "192.168.5" }, { "pw", "agopengps" } });
    must("subnet", r.code == 409, "A subnet for the module is refused (nobody would take it)", fmt("code %d", r.code));
    rules(w);
}

SCENARIO(User::Unconfigured, "new-older-teensy", "The Teensy still has a firmware from before the WiFi bridge (and before updates over the link)", oneKit())
{
    World w(kit);
    w.teensy.knowsBridge = false;
    w.teensy.knowsFw = false;
    w.bridgeSaved();
    w.start();
    w.tab.on = w.tab.engaged = true;
    w.run(10000);
    must("nothing-passed", w.teensy.old146 == 0 && w.tab.nPassed == 0, "Nothing of the tablet's program is passed to a Teensy that does not know the bridge (it would send those frames out on its network)",
         fmt("%u bridge frames reached it", w.teensy.old146));
    const JsonDoc st = w.state();
    tell("told-page", st["bridge"]["on"].num() == 1 && st["bridge"]["teensy"].num() == 0 && st["bridge"]["state"].num() == -1, "The page shows that the Teensy does not know the bridge",
         fmt("teensy %g, state %g", st["bridge"]["teensy"].num(), st["bridge"]["state"].num()));
    w.dev.usbOut.clear();
    w.usb("status");
    w.run(200);
    tell("told-usb", w.said("does NOT know it (older firmware?)"), "... and so does 'status' on the USB monitor", "");
    const std::vector<uint8_t> hex = makeHex(40000);
    const Reply r = w.upload(fmt("/api/teensy-update?pw=trator-2026&force=0&size=%u", (unsigned)hex.size()), "fw.hex", hex);
    must("update", r.code == 400 && has(r.error(), "firmware of 2 Nov 2026 or later") && r.tookMs() < 8000, "A .hex for it: after 5 s without an answer the page is told which firmware it needs",
         fmt("code %d '%s' in %u ms", r.code, r.error().c_str(), r.tookMs()));
    const Reply v = w.post("/api/teensy-revert", { { "pw", "trator-2026" } });
    must("revert", v.code == 409 && v.tookMs() < 12000, "The previous firmware back: the same, after 10 s", fmt("code %d in %u ms", v.code, v.tookMs()));
    rules(w);
}

SCENARIO(User::Unconfigured, "new-wrong-password", "Someone on the network tries every change with a wrong password, or with none", oneKit())
{
    World w(kit);
    w.flash("apPass", "segredo-2026");
    w.knownPass = "segredo-2026";
    w.start();
    const std::vector<uint8_t> hex = makeHex(30000), bin = makeImage(info(kit).chip, 30000);
    for (const std::string pw : { "agopengps", "", "segredo-2025", "SEGREDO-2026", "segredo-2026 " })
    {
        const std::string q = pw.empty() ? "" : "pw=" + enc(pw);
        const int c1 = w.post("/api/settings", settings(pw, "Hacked", "hacked-123", false)).code;
        const int c2 = w.post("/api/restart", { { "pw", pw } }).code;
        const int c3 = w.post("/api/subnet", { { "subnet", "10.0.0" }, { "pw", pw } }).code;
        const int c4 = w.post("/api/teensy-revert", { { "pw", pw } }).code;
        const Reply u = w.upload("/update?" + q, "x.bin", bin);
        const Reply t = w.upload(fmt("/api/teensy-update?%s&force=1&size=%u", q.c_str(), (unsigned)hex.size()), "x.hex", hex);
        const Reply k1 = w.post("/api/update-check", { { "pw", pw } }), k2 = w.post("/api/update-check", { { "for", "teensy" }, { "pw", pw } });
        must("refused-" + (pw.empty() ? std::string("none") : enc(pw)), c1 == 403 && c2 == 403 && c3 == 403 && c4 == 403 && u.code == 403 && t.code == 403 && u.error() == "wrong password",
             "Password '" + pw + "': every change answers 403 'wrong password'", fmt("%d %d %d %d %d %d", c1, c2, c3, c4, u.code, t.code));
        must("check-refused-" + (pw.empty() ? std::string("none") : enc(pw)), k1.code == 403 && k2.code == 403 && k1.error() == "wrong password" && k2.error() == "wrong password",
             "Password '" + pw + "': the page's check before an update (the XIAO's, the module's) answers 403 'wrong password' at once", fmt("%d %d", k1.code, k2.code));
    }
    // the page's check asked again and again with a wrong password (a script, a stuck key): Messages say it once in 10 s at most,
    // so the older messages stay; a check for something unknown is refused
    auto refusedLines = [&]() { size_t n = 0; for (size_t p = 0; (p = w.dev.usbOut.find("REFUSED: wrong password", p)) != std::string::npos; p++) n++; return n; };
    w.run(11000);                           // the last line about a wrong password is older than 10 s: the next one is said
    const size_t linesBefore = refusedLines();
    int burst403 = 0;
    for (int k = 0; k < 40; k++) burst403 += w.post("/api/update-check", { { "pw", "guess-" + std::to_string(k) } }).code == 403;
    const size_t burstLines = refusedLines() - linesBefore;
    must("check-burst-calm", burst403 == 40 && burstLines == 1, "40 checks with a wrong password: each refused, Messages say it once",
         fmt("%d refused, %u lines", burst403, (unsigned)burstLines));
    const Reply odd = w.post("/api/update-check", { { "for", "other" }, { "pw", "segredo-2026" } });
    must("check-unknown-for", odd.code == 400, "A check for something that is neither the XIAO nor the module is refused", fmt("code %d '%s'", odd.code, odd.error().c_str()));
    w.run(3000);
    must("nothing-changed", w.dev.flashWrites == 0 && w.xiao.restarts.empty() && w.teensy.subnets.empty() && w.teensy.reverts == 0 && w.teensy.fwDone == 0 && w.dev.updInstalled == 0 && !w.dev.updDone,
         "Nothing changed: no setting, no restart, nothing sent to the Teensy, no program written", fmt("%u flash writes, %u restarts", w.dev.flashWrites, (unsigned)w.xiao.restarts.size()));
    const Reply s = w.get("/api/settings");
    must("no-secret", s.jsonOk && !has(s.body, "segredo") && !has(w.get("/api/state").body, "segredo") && !has(w.get("/").body, "segredo"), "No answer gives the password away", "");
    must("right-one", w.post("/api/restart", { { "pw", "segredo-2026" } }).code == 200, "The right password still works", "");
    w.noteUserRestart();
    w.run(3000);
    rules(w);
}

SCENARIO(User::Unconfigured, "new-phones-join", "Phones and a laptop join the XIAO's network: their 'is there internet?' checks, and odd first requests", oneKit())
{
    World w(kit);
    w.start();
    const char* probes[][2] = { { "/generate_204", "connectivitycheck.gstatic.com" }, { "/hotspot-detect.html", "captive.apple.com" },
                                { "/connecttest.txt", "www.msftconnecttest.com" }, { "/ncsi.txt", "www.msftncsi.com" }, { "/", "example.com" } };
    for (auto& p : probes)
    {
        const Reply r = w.http(GET(p[0], p[1]));
        must(std::string("probe-") + p[1], r.code == 302 && r.hdr("location") == "http://192.168.4.1/" && r.tookMs() < 500, std::string("The check to ") + p[1] + " is sent to the page",
             fmt("code %d, to '%s', %u ms", r.code, r.hdr("location").c_str(), r.tookMs()));
    }
    must("favicon", w.get("/favicon.ico").code == 404, "The browser's own /favicon.ico gets 404, no redirect loop", "");
    const Reply old = w.http("GET / HTTP/1.0\r\n\r\n");
    must("no-host", old.code == 200 && old.body.size() > 50000, "A request with no Host line gets the page", fmt("code %d", old.code));
    const Reply head = w.http("HEAD / HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n");
    must("head", head.done || head.failed, "A HEAD request does not hang the XIAO", "");
    must("after", w.state()["ver"].str() == "4.5", "The page works after all of that", "");
    rules(w);
}
