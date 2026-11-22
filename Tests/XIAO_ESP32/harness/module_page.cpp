/* module_page.cpp - scenarios: the module's (Teensy's) own page through the XIAO, http://<the XIAO>:8080/.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The five kinds of user, on the one thing: the module's page carried over the link (PGN 150 / 151) for a phone or
// tablet on the XIAO's own WiFi. The pretend Teensy serves a page of 136 kB and a few /api answers (teensy.cpp).
#include "helpers.h"

namespace
{
using namespace h;

Link page(uint8_t ipLast = 30)
{
    Link l;
    l.port = 8080;
    l.ipLast = ipLast;
    return l;
}
std::string host8080(World& w) { return std::string(w.dev.apIp.toString().c_str()) + ":8080"; }
Reply pageGet(World& w, const std::string& path, const Link& l = page()) { return w.http(GET(path, host8080(w)), l); }
Reply pagePost(World& w, const std::string& path, const Form& f, const Link& l = page()) { return w.http(POST(path, f, host8080(w)), l); }
}

SCENARIO(User::Careful, "careful-module-page", "With the WiFi bridge on and the tablet steering, opens the module's page at http://192.168.5.1:8080/, saves a setting, restarts the module", bothKits())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.run(1500);
    JsonDoc st = w.state();
    must("ready", st["tpage"]["ready"].num() != 0, "The XIAO page says the module's page can be opened through the XIAO", st["tpage"]["why"].str());
    const bool steering = beginSteer(w);
    Reply r = pageGet(w, "/");
    must("page", r.done && r.code == 200 && r.body == w.teensy.webPage, "The module's whole page arrives, byte for byte as the Teensy sent it",
         fmt("code %d, %u of %u bytes, %s", r.code, (unsigned)r.body.size(), (unsigned)w.teensy.webPage.size(), r.why.c_str()));
    note("The module's page (136 kB) through the XIAO", fmt("%u ms, while the tablet steered through the bridge", r.tookMs()));
    // the page asks for several things at once: they are answered one after the other
    const int a = w.ask(GET("/api/status", host8080(w)), page(31));
    const int b = w.ask(GET("/api/status", host8080(w)), page(32));
    const int c = w.ask(GET("/api/nothing", host8080(w)), page(33));
    const Reply ra = w.wait(a), rb = w.wait(b), rc = w.wait(c);
    must("several", ra.code == 200 && rb.code == 200 && ra.json["ip"].str() == "192.168.5.126" && rc.code == 404,
         "Three requests at once are all answered, each with its own answer", fmt("%d %d %d", ra.code, rb.code, rc.code));
    r = pagePost(w, "/api/settings", { { "minKmh", "0.5" }, { "note", "bench" } });
    must("save", r.code == 200 && !w.teensy.webBodies.empty() && has(w.teensy.webBodies.back(), "minKmh=0.5"),
         "A setting saved on the module's page reaches the Teensy, and its answer comes back", fmt("code %d", r.code));
    must("window", w.teensy.webOverWindow == 0 && w.teensy.webMostAhead <= 6144, "The Teensy never had more of an answer on the way than the XIAO holds",
         fmt("at most %u bytes on the way", w.teensy.webMostAhead));
    if (steering) endSteer(w);
    w.tab.on = false;
    const uint32_t restarts = w.teensy.restarts;
    r = pagePost(w, "/api/reboot", {});
    must("reboot", r.code == 200 && w.runUntil([&]() { return w.teensy.restarts > restarts; }, 3000), "Restart on the module's page: answered, then the module restarts", fmt("code %d", r.code));
    w.run(5000);
    st = w.state();
    must("count", st["tpage"]["req"].num() >= 6 && st["tpage"]["broken"].num() == 0, "The XIAO page counts the requests passed on, none broken off",
         fmt("%ld passed, %ld broken", (long)st["tpage"]["req"].num(), (long)st["tpage"]["broken"].num()));
    rules(w);
}

SCENARIO(User::Unconfigured, "module-page-default-password", "The XIAO's WiFi still has its password from new: opens the module's page through the XIAO", oneKit())
{
    World w(kit);
    w.start();
    w.run(1500);
    const Reply r = pageGet(w, "/");
    tell("told", r.code == 403 && has(r.body, "password"), "The browser is told to give the XIAO's WiFi a password of one's own first", fmt("code %d: %s", r.code, r.body.substr(0, 160).c_str()));
    must("nothing", w.teensy.webOpens == 0, "Nothing went to the Teensy", fmt("%u requests opened", w.teensy.webOpens));
    const JsonDoc st = w.state();
    tell("page", !st["tpage"]["ready"].num() != 0 && has(st["tpage"]["why"].str(), "password"), "The XIAO page says why the module's page cannot be opened", st["tpage"]["why"].str());
    rules(w);
}

SCENARIO(User::WrongConnections, "module-page-old-teensy", "The Teensy runs an older firmware (no page through the XIAO, or no WiFi bridge at all): opens the module's page through the XIAO", oneKit())
{
    for (int older = 0; older < 2; older++)
    {
        World w(kit);
        w.bridgeSaved();
        w.teensy.knowsPage = false;
        if (older) w.teensy.knowsBridge = false;
        w.start();
        w.run(4000);
        const Reply r = pageGet(w, "/");
        const char* which = older ? "no-bridge" : "bridge-only";
        tell(fmt("told-%s", which), r.code == 503 && has(r.body, "older"), "The browser is told the module's firmware is older, and what to do",
             fmt("code %d: %s", r.code, r.body.substr(0, 160).c_str()));
        must(fmt("nothing-%s", which), w.teensy.old150 == 0 && w.teensy.webOpens == 0, "Nothing of the page went to the older Teensy (it would send it on to the network)",
             fmt("%u frames", w.teensy.old150));
        rules(w);
    }
}

SCENARIO(User::WrongConnections, "module-page-other-network", "Opens the module's page through the XIAO from the farm's WiFi (the XIAO joined it too)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.flash("staOn", true);
    w.flash("staSsid", "Farm");
    w.flash("staPass", "farm12345");
    w.start();
    w.runUntil([&]() { return w.dev.staJoined; }, 8000);
    w.run(2000);
    Link l = page(40);
    l.fromAp = false;
    const Reply r = w.http(GET("/", std::string(w.dev.staIp().toString().c_str()) + ":8080"), l);
    must("refused", r.code == 403 && has(r.body, "own WiFi"), "Refused, with the reason: it opens only from the XIAO's own WiFi", fmt("code %d: %s", r.code, r.body.substr(0, 160).c_str()));
    must("nothing", w.teensy.webOpens == 0, "Nothing went to the Teensy", fmt("%u requests opened", w.teensy.webOpens));
    rules(w);
}

SCENARIO(User::WrongSettings, "module-page-firmware", "On the module's page through the XIAO, tries a firmware update and going back to the previous firmware", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.run(1500);
    const std::vector<uint8_t> hex = makeHex(30000);
    Link l = page();
    const Reply up = w.http(UPLOAD("/api/ota", "v4.5_AiO_WEB.ino.hex", hex, host8080(w)), l);
    tell("ota", up.code == 403 && has(up.error(), "Firmware tab"), "A firmware update there is refused, with where to do it", fmt("code %d: %s", up.code, up.error().c_str()));
    const Reply rv = pagePost(w, "/api/revert", {});
    tell("revert", rv.code == 403 && has(rv.error(), "Firmware tab"), "Going back there is refused the same way", fmt("code %d: %s", rv.code, rv.error().c_str()));
    bool asked = false;
    for (auto& a : w.teensy.webAsked) asked |= has(a, "/api/ota") || has(a, "/api/revert");
    must("nothing", !asked && w.teensy.reverts == 0 && w.teensy.fwDone == 0, "Neither reached the Teensy", fmt("%u requests reached it", (unsigned)w.teensy.webAsked.size()));
    must("after", pageGet(w, "/api/status").code == 200, "The module's page works on after that", "");
    rules(w);
}

SCENARIO(User::BadLink, "module-page-phone-leaves", "A phone loading the module's page through the XIAO leaves the WiFi half way; another phone opens it next", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.phones = 2;
    w.start();
    w.run(1500);
    const bool steering = beginSteer(w);
    Link gone = page(30);
    gone.deafAt = 20000;                    // takes 20 kB of the page, then nothing more (gone, without a word)
    gone.patienceMs = 40000;
    const int a = w.ask(GET("/", host8080(w)), gone);
    w.run(1000);
    const Reply rb = w.http(GET("/api/status", host8080(w)), page(31));
    must("next", rb.code == 200 && rb.tookMs() <= 20000, "The other phone gets its answer once the first one is given up (within 20 s)",
         fmt("code %d after %u ms", rb.code, rb.tookMs()));
    w.wait(a);
    w.run(6000);
    const JsonDoc st = w.state();
    must("counted", st["tpage"]["broken"].num() >= 1, "The XIAO page counts the request broken off", fmt("%ld", (long)st["tpage"]["broken"].num()));
    must("teensy", !w.teensy.webAsked.empty() && w.teensy.webGone + w.teensy.webBroken >= 1 && w.teensy.webEnds >= 1, "The Teensy let go of the first request and served the next",
         fmt("gone %u, given up %u, ended %u", w.teensy.webGone, w.teensy.webBroken, w.teensy.webEnds));
    if (steering) endSteer(w);
    rules(w);
}

SCENARIO(User::BadLink, "module-page-noisy-link", "The serial link is noisy (one byte in 5000 arrives wrong) while a phone loads the module's page through the XIAO", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.run(1500);
    w.noise = 0.0002;
    int whole = 0, cut = 0, wrong = 0;
    for (int i = 0; i < 4; i++)
    {
        Link l = page((uint8_t)(30 + i));
        l.cutOk = true;
        const Reply r = pageGet(w, "/", l);
        if (r.done && r.code == 200) { whole++; if (r.body != w.teensy.webPage) wrong++; }
        else cut++;
    }
    w.noise = 0;
    must("right", wrong == 0, "No wrong byte of the module's page ever reached the browser", fmt("%d whole, %d cut short (asked again by a browser), %d wrong", whole, cut, wrong));
    note("The module's page on a noisy link", fmt("%d of 4 whole, %d cut short", whole, cut));
    w.run(2000);
    must("after", pageGet(w, "/").body == w.teensy.webPage, "With the noise gone the page loads whole again", "");
    rules(w);
}
