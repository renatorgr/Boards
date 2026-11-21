/* user_wrong_slots.cpp - user 4: modules, relays or sensors in the wrong place (the wrong section, the wrong IP, the
   wrong input, the other module's program).
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "helpers.h"
#include <fstream>
#include <iterator>
#include <set>
using namespace h;
using chk::fmt;
using chk::must;
using chk::note;
using chk::tell;
using chk::User;

static const char* NO_RELAY_WORDS = "no relay";

SCENARIO(User::WrongSlots, "two-relays-one-section", "AgOpenWeb's pin config: relays 1 and 2 both on section 1 (section 2 has none)", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.startup(false, { 1, 1, 3, 4, 5, 6, 7, 8 });
    const uint64_t from = w.now();
    std::string bad;
    std::vector<int> rs = { 1, 1, 3, 4, 5, 6, 7, 8 };
    rs.resize(m.bi.relays, 0);
    int n = driveAndCompare(w, m, first(8), rs, 5000, &bad);
    n += driveAndCompare(w, m, 0x02, rs, 3000, n ? nullptr : &bad);
    must("follows-pin-config", n == 0, "the relays do what the pin config says (relay 2 follows section 1, not section 2)", bad);
    const std::string said = pageLog(m) + screenMessages(w, from);
    must("double-said", contains(said, "section 2:") && contains(said, NO_RELAY_WORDS), "the module says section 2 has no relay", said);
}

SCENARIO(User::WrongSlots, "relay-on-missing-section", "pin config: relay 8 on section 12 of an 8-section boom", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 12 });
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), {}, 5000);
    must("section-12-never", ((m.relays() >> 7) & 1) == 0, "relay 8 stays open (section 12 is never asked for)", hex(m.relays()));
    const std::string said = pageLog(m) + screenMessages(w, from);
    must("missing-section-said", contains(said, "section 8:") && contains(said, NO_RELAY_WORDS), "the module says section 8 has no relay", said);
}

SCENARIO(User::WrongSlots, "more-sections-than-relays", "12-section boom on the 8-relay board", { Board::SprayerX8 })
{
    World w;
    Module& m = readyModule(w, board);
    w.addRig(m);
    w.aow.toolSections = 12;
    for (int i = 8; i < 12; i++) w.aow.widthCm[i] = 300;
    w.aow.startup();                                        // pins 1-12 = sections 1-12
    const uint64_t from = w.now();
    std::string bad;
    const int n = driveAndCompare(w, m, first(12), inARow(8, 1, 8), 5000, &bad);
    must("first-8-follow", n == 0, "relays 1-8 follow sections 1-8", bad);
    Json s = m.status();
    note("PGN 237 tells AgOpenWeb", "open " + s["out"]["on"].str() + " of asked " + s["aog"]["req"].str());
    const std::string said = pageLog(m) + screenMessages(w, from);
    must("too-few-relays-said", contains(said, "sections 9-12") && contains(said, NO_RELAY_WORDS),
         "the module says sections 9-12 have no relay (8 relays for 12 sections)", said);
}

// Two seed drill modules for one wide drill: A = sections 1..R, B = sections R+1..2R (R = the board's relays)
static void twoSeeders(Board board, bool rightFirst, bool rightIp)
{
    World w;
    const int R = info(board).relays, total = std::min(64, 2 * R);
    Module& a = readyModule(w, board, "module A", 123);
    Module& b = readyModule(w, board, "module B", rightIp ? 124 : 123);
    a.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "1" }, { "secs", std::to_string(total) } });
    b.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", rightFirst ? std::to_string(R + 1) : "1" }, { "secs", std::to_string(total) } });
    w.aow.toolSections = total;
    w.aow.startup();
    const uint64_t from = w.now();
    w.aow.sections = first(total);
    w.run(3000);
    const uint64_t pattern = 0x5555555555555555ull & first(total);
    w.aow.sections = pattern;
    w.run(500);
    const std::vector<int> ra = inARow(R, 1, total), rb = inARow(R, R + 1, total);
    const bool okA = a.relays() == expected(ra, pattern), okB = b.relays() == expected(rb, pattern);
    if (rightFirst && rightIp)
    {
        must("both-follow", okA && okB, chk::fmt("module A follows sections 1-%d and module B %d-%d", R, R + 1, total),
             fmt("A %s, B %s", hex(a.relays()).c_str(), hex(b.relays()).c_str()));
        const std::string said = pageLog(a) + pageLog(b) + screenMessages(w, from);
        must("no-false-no-relay", !contains(said, NO_RELAY_WORDS), "no 'no relay' warning: each module drives its own half", said);
        return;
    }
    // who answers on 192.168.5.123
    std::set<std::string> senders;
    for (auto& hd : w.aow.heardPgn(237, from)) senders.insert(hd.src.toString().str());
    std::string who;
    for (auto& s : senders) who += s + " ";
    note("AgOpenWeb hears PGN 237 from", who);
    const std::string said = pageLog(a) + pageLog(b) + screenMessages(w, from);
    if (!rightFirst)
        tell("overlap-said", contains(said, NO_RELAY_WORDS) || contains(said, "other module"),
             chk::fmt("module B left at 'relay 1 = section 1': the modules say sections 1-%d are doubled and %d-%d have no relay", R, R + 1, total),
             fmt("B drives %s (= A's sections), nobody drives %d-%d; nothing is said", hex(b.relays()).c_str(), R + 1, total));
    if (!rightIp)
        tell("same-ip-said", contains(said, "same IP") || contains(said, "123 is taken") || contains(said, "another module"),
             "module B left at IP .123 like A: a module says the IP is taken", "both answer as 192.168.5.123; AgOpenWeb cannot tell them apart");
}

SCENARIO(User::WrongSlots, "two-seeders-right", "two seed drill modules for one wide drill, set up right", seederBoards()) { twoSeeders(board, true, true); }
SCENARIO(User::WrongSlots, "two-seeders-same-first", "second seed drill module left at 'relay 1 = section 1'", seederBoards()) { twoSeeders(board, false, true); }
SCENARIO(User::WrongSlots, "two-seeders-same-ip", "second seed drill module left at IP .123", seederBoards()) { twoSeeders(board, true, false); }

SCENARIO(User::WrongSlots, "seeder-fewer-sections", "seed drill page set to 16 sections, the drill (and AgOpenWeb) has 19", seederBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.toolSections = 19;
    w.aow.startup();
    m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "1" }, { "secs", "16" } });
    const uint64_t from = w.now();
    std::string bad;
    const int n = driveAndCompare(w, m, first(19), inARow(m.bi.relays, 1, 16), 5000, &bad);
    must("follows-16", n == 0, "relays 1-16 follow sections 1-16, 17-19 stay off (as set)", bad);
    const std::string said = pageLog(m) + screenMessages(w, from);
    note("AgOpenWeb's screen shows", screenMessages(w, from));
    must("fewer-sections-said", contains(said, "sections 17-19") && contains(said, NO_RELAY_WORDS),
         "the module says sections 17-19 have no relay (the page is set to 16 sections)", said);
}

SCENARIO(User::WrongSlots, "seeder-pin-config-19", "seed drill set to 'the program's pin config' with 19 sections (it only knows 1-16)", seederBoards())
{
    World w;
    Module& m = readyModule(w, board);
    w.aow.toolSections = 19;
    w.aow.startup();
    m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "0" }, { "secs", "19" } });
    const uint64_t from = w.now();
    std::string bad;
    std::vector<int> rs = inARow(16, 1, 16);
    rs.resize(m.bi.relays, 0);
    const int n = driveAndCompare(w, m, first(19), rs, 5000, &bad);
    must("follows-pins", n == 0, "relays 1-16 follow the pin config, sections 17-19 have no relay", bad);
    const std::string said = pageLog(m) + screenMessages(w, from);
    must("pin-config-limit-said", contains(said, "sections 17-19") && contains(said, NO_RELAY_WORDS),
         "the module says sections 17-19 have no relay (the program's pin config only knows 1-16)", said);
}

SCENARIO(User::WrongSlots, "sensors-swapped", "plugs the flow meter into the pressure input and the pressure sensor into the flow input", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);
    SprayerRig& rig = w.addRig(m);
    rig.swapped = true;
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    m.http(HTTP_POST, "/api/spray", { { "pw", "agopengps" }, { "rate", "240" }, { "nzSp", "0.5" }, { "nzFl", "1.6" }, { "nzBar", "3" },
                                      { "pLow", "1" }, { "pHigh", "6" }, { "fTol", "20" } });
    const uint64_t from = w.now();
    driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 20000);
    Json s = m.status();
    note("module reads", fmt("%.2f bar, %.2f L/min (real: 3.0 bar, 76.8 L/min)", s["spr"]["bar"].num(), s["spr"]["lpm"].num()));
    const std::string screen = screenMessages(w, from);
    tell("swap-gives-alarm", !screen.empty(), "an alarm reaches AgOpenWeb's screen", screen.empty() ? "(nothing)" : screen);
}

static std::vector<uint8_t> readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

SCENARIO(User::WrongSlots, "other-modules-program", "sends the other module's program (.bin) to a module's update", allBoards())
{
    World w;
    Module& m = readyModule(w, board);
    const char* root = getenv("BOARDS_REPO");
    const std::string repo = root && root[0] ? root : "../..";
    const std::string own = m.bi.sprayer ? "/Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino" : "/Seeder/Seeder_ESP32/Seeder_ESP32.ino";
    const std::string other = m.bi.sprayer ? "/Seeder/Seeder_ESP32/Seeder_ESP32.ino" : "/Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino";
    const std::vector<uint8_t> merged = readFile(repo + own + ".merged.bin");
    const std::vector<uint8_t> wrong = readFile(repo + other + ".bin");
    const std::vector<uint8_t> right = readFile(repo + own + ".bin");
    if (merged.empty() || wrong.empty() || right.empty()) { must("bins-found", false, "the .bin files are in the repository", repo); return; }

    Http r = m.http(HTTP_POST, "/api/update", { { "pw", "agopengps" } }, true, &merged, "x.merged.bin");
    must("merged-refused", r.code == 500 && !m.dev.updDone, "a .merged.bin (USB-only file) is refused and the old program stays", fmt("answer %d %s", r.code, r.body.c_str()));
    w.run(1000);
    r = m.http(HTTP_POST, "/api/update", { { "pw", "agopengps" } }, true, &wrong, "other.bin");
    const bool took = m.dev.updDone;
    w.run(1000);
    tell("other-program-refused", !took, "the module itself refuses the other module's program (the page already does)",
         fmt("answer %d: the module would restart into the %s program", r.code, m.bi.sprayer ? "seed drill" : "sprayer"));
    Module& m2 = w.add(board, "module 2");
    if (!m2.bi.sprayer) m2.dev.cableIn = true;
    m2.powerOn();
    w.run(500);
    r = m2.http(HTTP_POST, "/api/update", { { "pw", "agopengps" } }, true, &right, "own.bin");
    must("own-program-taken", r.code == 200 && m2.dev.updDone, "its own program is taken (and it restarts)", fmt("answer %d %s", r.code, r.body.c_str()));
    r = m2.http(HTTP_POST, "/api/update", { { "pw", "wrong" } }, true, &right, "own.bin");
    must("update-needs-password", r.code == 403, "an update with the wrong password is refused", fmt("answer %d", r.code));
}
