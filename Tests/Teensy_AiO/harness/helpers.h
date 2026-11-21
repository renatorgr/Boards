/* helpers.h - small steps the scenarios share.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once
#include "checks.h"
#include <cmath>
#include <cstring>

namespace bench
{
// The driver engages (or disengages) autosteer the way the kit is set up: AgOpenWeb's button, and the steer
// switch closed with a switch (a button kit uses the screen's button, which the firmware takes as the button)
inline void engage(World& w, bool on)
{
    w.aow.engage = on;
    if (w.ki.steerMode == 1) w.sw.steerClosed = on;
}

// Started, driving at kmh, AgOpenWeb steering for ms
inline void steerFor(World& w, float kmh, uint32_t ms)
{
    w.tractor.kmh = kmh;
    engage(w, true);
    w.run(ms);
}

// How long until the outputs are off (ms), at most maxMs
inline uint64_t msUntilOff(World& w, uint32_t maxMs)
{
    const uint64_t t0 = w.now();
    w.runUntil([&] { return !w.engaged(); }, maxMs);
    return w.now() - t0;
}

// A text on AgOpenWeb's screen or in the module's log (its web page's Messages) since the marks
inline bool told(World& w, size_t logMark, size_t screenMark, const std::string& text)
{
    return w.aio.logHas(text, logMark) || w.aow.screenHas(text, screenMark);
}

inline std::string lastScreen(World& w) { return w.aow.screen.empty() ? std::string("(nothing)") : w.aow.screen.back(); }

// the module's log since a mark, cut to n characters (for a detail)
inline std::string logSince(World& w, size_t mark, size_t n = 600)
{
    std::string s = w.aio.log.substr(std::min(mark, w.aio.log.size()));
    if (s.size() > n) s = "..." + s.substr(s.size() - n);
    for (auto& c : s) if (c == '\n' || c == '\r') c = ' ';
    return s;
}
}
