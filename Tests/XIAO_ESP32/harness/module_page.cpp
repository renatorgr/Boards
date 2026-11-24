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

#include <regex>

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

SCENARIO(User::WrongConnections, "machine-tab-old-teensy", "A sprayer module on the XIAO's WiFi, the Teensy's firmware older than the one that shows such a module in its Machine tab", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.teensy.knowsMach = false;             // PGN 149 version 2: the page through the XIAO, not PGN 152
    w.start();
    w.mod[0].on = true;
    w.run(4000);
    must("nothing", w.teensy.machIn == 0 && w.teensy.old152 == 0, "Nothing of the module went to the older Teensy (it would send it on to the network cable)",
         fmt("%u taken, %u sent on", w.teensy.machIn, w.teensy.old152));
    must("listed", w.state()["bridge"]["mods"].a.size() == 1, "The XIAO page still lists the module", "");
    rules(w);
}

SCENARIO(User::BadLink, "module-page-low-memory", "The XIAO is short of memory (a busy WiFi) while a phone loads the module's page through it", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.run(1500);
    fake::freeHeap = 20000;                 // as on the bench, 22 Nov 2026: the C3 fell to 2 kB free while the page loaded
    const int a = w.ask(GET("/", host8080(w)), page(30));
    w.run(4000);
    JsonDoc st = w.state();
    must("waits", st["tpage"]["lowMem"].num() >= 1 && w.teensy.webOpens == 0, "With little memory the XIAO does not start the request (nothing to the Teensy)",
         fmt("waited %g times, %u requests opened", st["tpage"]["lowMem"].num(), w.teensy.webOpens));
    tell("told", w.said("short of memory"), "Messages say the module's page waits for memory", "");
    fake::freeHeap = 182000;                // the memory comes back
    const Reply r = w.wait(a);
    must("then", r.done && r.code == 200 && r.body == w.teensy.webPage, "With the memory back the page loads whole", fmt("code %d, %u bytes, %s", r.code, (unsigned)r.body.size(), r.why.c_str()));
    rules(w);
}

namespace
{
int timesSaid(World& w, const std::string& part)
{
    int n = 0;
    for (size_t at = w.dev.usbOut.find(part); at != std::string::npos; at = w.dev.usbOut.find(part, at + 1)) n++;
    return n;
}

// The note in Messages (USB): the line that starts with head and the "... " lines right after it, each without its line end
std::vector<std::string> noteLines(World& w, const std::string& head)
{
    std::vector<std::string> lines;
    const std::string& out = w.dev.usbOut;
    size_t at = out.find(head);
    if (at == std::string::npos || (at > 0 && out[at - 1] != '\n')) return lines;
    while (at < out.size())
    {
        size_t end = out.find('\n', at);
        if (end == std::string::npos) end = out.size();
        std::string line = out.substr(at, end - at);
        while (!line.empty() && line.back() == '\r') line.pop_back();
        if (!lines.empty() && line.rfind("... ", 0) != 0) break;
        lines.push_back(line);
        at = end + 1;
    }
    return lines;
}
}

SCENARIO(User::BadLink, "memory-low-noted", "The XIAO runs short of memory while a phone loads the module's page: when, and what was going on, are noted (bench, 24 Nov 2026: 5.5 kB at the lowest, nothing said)", oneKit())
{
    World w(kit);
    w.bridgeSaved();
    w.start();
    w.run(1500);
    JsonDoc st = w.state();
    must("none", st.has("memLow") && st["memLow"]["free"].num() == 0 && st["memLow"]["ago"].num() == -1 && !w.said("Short of memory"),
         "With plenty of memory nothing is noted", fmt("free %g, ago %g", st["memLow"]["free"].num(), st["memLow"]["ago"].num()));
    const int a = w.ask(GET("/", host8080(w)), page(30));
    w.run(300);                             // the page is on its way (136 kB) ...
    fake::freeHeap = 6000;                  // ... and the WiFi takes the memory
    w.run(2000);
    st = w.state();
    const std::string what = st["memLow"]["what"].str();
    must("noted", st["memLow"]["free"].num() == 6000 && st["memLow"]["ago"].num() >= 0 && st["memLow"]["ago"].num() <= 3 && st["memLow"]["freeThen"].num() == 6000,
         "The XIAO page shows the lowest point and when", fmt("free %g, %g s ago, %g free then", st["memLow"]["free"].num(), st["memLow"]["ago"].num(), st["memLow"]["freeThen"].num()));
    must("what", what.find("module page (port 8080) GET /,") != std::string::npos && what.find("device(s) on our WiFi") != std::string::npos,
         "... with what was going on: the module's page loading, the devices on its WiFi", what);
    tell("told", w.said("Short of memory: 6000 bytes free at the lowest") && w.said("... module page (port 8080) GET /,"),
         "Messages say the XIAO was short of memory, and what it was doing", "");
    // The note whole, in order: the lowest point alone, then each piece on a line of its own (Sourcery on #185, #186, #189)
    const std::vector<std::string> note = noteLines(w, "Short of memory: 6000");
    std::string joined;
    bool fits = !note.empty();
    for (const std::string& line : note) { joined += (joined.empty() ? "" : "\n") + line; fits = fits && line.size() <= 120; }
    static const std::regex whole(
        "Short of memory: 6000 bytes free at the lowest\\. Going on then:\n"
        "\\.\\.\\. module page \\(port 8080\\) GET /, [0-9]+ bytes held, [0-9]+ browser connection\\(s\\)\n"
        "(\\.\\.\\. our page answer(ed|ing) (/[a-z/-]*|\\(an unknown address\\))\n)?"   // a whole address
        "\\.\\.\\. bridge a second: [0-9]+ to the Teensy, [0-9]+ to the program, [0-9]+ to modules, [0-9]+ lost\n"
        "\\.\\.\\. [0-9]+ device\\(s\\) on our WiFi");   // (no other network set here: no join)
    must("whole", fits && std::regex_match(joined, whole),
         "... the lowest point on a line alone, then each piece whole on a line of its own, in order, none longer than a line of Messages",
         joined);
    fake::freeHeap = 4000;                  // a little lower: the page shows it, Messages do not say it again
    w.run(1000);
    st = w.state();
    must("lower", st["memLow"]["free"].num() == 4000, "The page follows the lowest point", fmt("free %g", st["memLow"]["free"].num()));
    must("quiet", timesSaid(w, "Short of memory:") == 1, "A little lower is not said again in Messages", fmt("said %d time(s)", timesSaid(w, "Short of memory:")));
    fake::freeHeap = 1500;                  // much lower: said again
    w.run(1000);
    must("again", timesSaid(w, "Short of memory:") == 2 && w.said("Short of memory: 1500 bytes"), "Much lower is said again", fmt("said %d time(s)", timesSaid(w, "Short of memory:")));
    fake::freeHeap = 182000;                // the memory comes back: the lowest point stays, nothing more is said
    const Reply r = w.wait(a);
    w.run(2000);
    st = w.state();
    must("kept", st["memLow"]["free"].num() == 1500 && st["heapMin"].num() == 1500 && timesSaid(w, "Short of memory:") == 2,
         "With the memory back the lowest point stays on the page, and nothing more is said", fmt("free %g, heapMin %g", st["memLow"]["free"].num(), st["heapMin"].num()));
    must("then", r.done && r.code == 200 && r.body == w.teensy.webPage, "With the memory back the page loads whole", fmt("code %d, %u bytes, %s", r.code, (unsigned)r.body.size(), r.why.c_str()));
    rules(w);
}
