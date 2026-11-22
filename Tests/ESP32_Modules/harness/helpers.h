/* helpers.h - what several scenarios do: set a module up, drive a field, watch the relays.
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once
#include "checks.h"

namespace h
{
using namespace bench;

// What the relays must do for the asked sections, from the user's set-up (worked out here, not by the firmware):
// relay i follows section relaySection[i] (0 = nothing)
inline uint32_t expected(const std::vector<int>& relaySection, uint64_t sections)
{
    uint32_t r = 0;
    for (size_t i = 0; i < relaySection.size(); i++)
        if (relaySection[i] >= 1 && relaySection[i] <= 64 && ((sections >> (relaySection[i] - 1)) & 1)) r |= 1u << i;
    return r;
}

// Relay i -> section i + firstSection, up to the board's relays (and the drill's sections, when set)
inline std::vector<int> inARow(int relays, int firstSection, int lastSection = 64)
{
    std::vector<int> v;
    for (int i = 0; i < relays; i++) v.push_back(firstSection + i <= lastSection ? firstSection + i : 0);
    return v;
}

// A set-up "ready to work": on the tractor network (the sprayer by Wi-Fi, the seed drill by its cable) and
// AgOpenWeb has sent its start-up messages
inline Module& readyModule(World& w, Board b, const std::string& label = "module", int ipLast = 123)
{
    Module& m = w.add(b, label);
    if (!info(b).sprayer) m.dev.cableIn = true;
    m.powerOn();
    w.run(500);
    if (info(b).sprayer || ipLast != 123)
    {
        if (info(b).sprayer) joinTractorWifi(w, m, "tractor123", "192.168.5", ipLast);
        else
        {
            // the seed drill keeps its cable: only the module IP's last number changes
            m.http(HTTP_POST, "/api/wifi", { { "pw", "agopengps" }, { "apSsid", m.dev.apSsid }, { "apPass", "" }, { "staOn", "0" },
                                             { "staSsid", "" }, { "staPass", "" }, { "host", "seeder" }, { "subnet", "192.168.5" },
                                             { "ipLast", std::to_string(ipLast) } });
            w.run(3000);
        }
    }
    return m;
}

// No router on the tractor: the tablet and the module are on the WiFi of the AiO board's XIAO, its WiFi bridge on.
// The user does what the note on the module's page says: tractor Wi-Fi = the XIAO's network, the subnet kept (192.168.5,
// which the XIAO's network has with its bridge on; 'third' another for an AiO module on 192.168.<third>).
// Then AgOpenWeb's start-up messages (and, on the seed drill, 'relay 1 = section 1').
inline Module& onXiaoWifi(World& w, Board b, const std::string& subnet = "192.168.5", uint8_t third = 5)
{
    w.useXiao(third);
    Module& m = w.add(b);
    m.powerOn();
    w.run(500);
    joinTractorWifi(w, m, fake::net.pass, subnet);
    w.aow.toolSections = info(b).sprayer ? 8 : 19;
    if (info(b).sprayer) w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    else
    {
        w.aow.startup();
        m.http(HTTP_POST, "/api/relays", { { "pw", "agopengps" }, { "first", "1" }, { "secs", "19" } });
    }
    w.run(1000);
    return m;
}

// Drive with these sections for ms and check, every 10 ms after the first 150 ms, that the relays are what the
// set-up says. Returns the number of samples that were wrong (and the first wrong one in firstBad).
inline int driveAndCompare(World& w, Module& m, uint64_t sections, const std::vector<int>& relaySection, uint32_t ms,
                           std::string* firstBad = nullptr)
{
    w.aow.sections = sections;
    int bad = 0;
    for (uint32_t t = 0; t < ms; t += 10)
    {
        w.run(10);
        if (t < 150) continue;
        const uint32_t want = expected(relaySection, sections);
        const uint32_t got = m.relays();
        if (got != want)
        {
            if (!bad && firstBad) *firstBad = chk::fmt("sections %s: relays %s, should be %s", hex(sections).c_str(), hex(got).c_str(), hex(want).c_str());
            bad++;
        }
    }
    return bad;
}

// How long after "now" the relays all dropped (ms), or -1 if they did not within maxMs
inline int msUntilAllOff(World& w, Module& m, uint32_t maxMs)
{
    for (uint32_t t = 0; t <= maxMs; t++)
    {
        if (m.relays() == 0) return (int)t;
        w.run(1);
    }
    return -1;
}

// The event log on the module's page (its /api/status "log")
inline std::string pageLog(Module& m)
{
    Json s = m.status();
    std::string o;
    for (auto& l : s["log"].a) o += l.str() + "\n";
    return o;
}

inline bool contains(const std::string& hay, const std::string& needle) { return hay.find(needle) != std::string::npos; }

// The text of the PGN 221 messages (AgOpenWeb shows them on its screen) the tablet got since sinceMs
inline std::string screenMessages(World& w, uint64_t sinceMs)
{
    std::string o;
    for (auto& hd : w.aow.heardPgn(221, sinceMs))
        if (hd.data.size() > 8) o += std::string(hd.data.begin() + 7, hd.data.end() - 1) + "\n";
    return o;
}

// The status JSON stayed valid all along (a NaN or a broken string would break the page)
inline void statusAlwaysJson(Module& m)
{
    chk::must("status-json", m.badStatus == 0, "the page's status answer is valid JSON every time",
              m.badStatus ? chk::fmt("%d broken answers, last: %.300s", m.badStatus, m.lastBadStatus.c_str()) : "");
}
}
