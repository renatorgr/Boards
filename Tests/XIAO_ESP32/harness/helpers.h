/* helpers.h - what the scenarios share: the checks every one ends with, steering through the WiFi bridge.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
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
using namespace chk;

inline std::string firstOf(const std::vector<std::string>& v)
{
    std::string o;
    for (size_t i = 0; i < v.size() && i < 3; i++) o += (i ? "; " : "") + v[i];
    if (v.size() > 3) o += fmt(" (and %u more)", (unsigned)(v.size() - 3));
    return o;
}

inline const char* reasonText(esp_reset_reason_t r)
{
    return r == ESP_RST_SW ? "asked by the program" : r == ESP_RST_TASK_WDT ? "WATCHDOG (the program stopped for 15 s)" : r == ESP_RST_POWERON ? "power" : "other";
}

// The rules that hold whatever the user does. Every scenario ends with them.
//   helloMaxMs: the Teensy calls the XIAO gone after 10 s without its hello (the XIAO sends one every 2 s)
inline void rules(World& w, uint32_t helloMaxMs = 9000)
{
    must("rule-pass", w.rules.passed.empty(), "Nothing reached the Teensy through the WiFi bridge that should not", firstOf(w.rules.passed));
    must("rule-dest", w.rules.dest.empty(), "The Teensy's data for the program went to that program only, as the Teensy sent it", firstOf(w.rules.dest));
    must("rule-answers", w.rules.answers.empty(), "Every answer to a browser was well formed (HTTP, its length, valid JSON)", firstOf(w.rules.answers));
    bool asked = (int)w.xiao.restarts.size() == w.restartsExpected;
    std::string why;
    for (auto r : w.xiao.restarts) { if (r != ESP_RST_SW) asked = false; why += std::string(why.empty() ? "" : ", ") + reasonText(r); }
    must("rule-restart", asked, "The XIAO restarted only when the user asked for it",
         fmt("%u restart(s), %d asked for%s%s", (unsigned)w.xiao.restarts.size(), w.restartsExpected, why.empty() ? "" : ": ", why.c_str()));
    must("rule-hello", w.rules.worstHelloGapMs <= helloMaxMs && w.teensy.silentEvents == 0, "The XIAO's hello to the Teensy was never late (the Teensy never called it gone)",
         fmt("longest time without a hello %u ms; the Teensy logged 'XIAO silent' %u time(s)", w.rules.worstHelloGapMs, w.teensy.silentEvents));
    must("rule-rx", w.dev.uartRxLost == 0, "No data from the Teensy was lost in the XIAO's receive buffer", fmt("%u bytes lost", w.dev.uartRxLost));
    note("Longest loop() pass / time without the watchdog fed / time without a hello",
         fmt("%u ms / %u ms / %u ms", w.xiao.longestPassMs, w.dev.wdtLongestMs, w.rules.worstHelloGapMs));
}

// Was the link to the Teensy left alone for a while (no hello in time, its data not read)? With the words for it.
inline bool linkLeftAlone(World& w, std::string& how)
{
    how = fmt("longest time without a hello %u ms (the Teensy calls the XIAO silent after 10 s: it did %u time(s)), %u bytes of the Teensy's data lost unread",
              w.rules.worstHelloGapMs, w.teensy.silentEvents, w.dev.uartRxLost);
    return w.rules.worstHelloGapMs > 9000 || w.teensy.silentEvents > 0 || w.dev.uartRxLost > 0;
}

typedef std::vector<std::pair<std::string, std::string>> Form;

// The WiFi tab's form, as the page sends it
inline Form settings(const std::string& pw, const std::string& apSsid, const std::string& apPass = "", bool bridge = false,
                     const std::string& staSsid = "", const std::string& staPass = "", const std::string& host = "xiao")
{
    return { { "pw", pw }, { "apSsid", apSsid }, { "apPass", apPass }, { "staOn", staSsid.empty() ? "0" : "1" }, { "staSsid", staSsid },
             { "staPass", staPass }, { "host", host }, { "bridge", bridge ? "1" : "0" } };
}

// The XIAO restarts and runs again within ms (boots: its count before what should restart it)
inline bool backAfterRestart(World& w, int boots, uint32_t ms = 6000)
{
    return w.runUntil([&]() { return w.xiao.boots > boots && w.xiao.on; }, ms);
}

// a .hex file for the Teensy, in small: text lines as the Arduino IDE writes them
inline std::vector<uint8_t> makeHex(size_t size)
{
    std::string t;
    for (uint32_t a = 0; t.size() < size; a += 16) t += fmt(":10%04X00%08X%08X%08X%08X%02X\r\n", a & 0xFFFF, a * 2654435761u, a ^ 0x5A5A5A5Au, a + 77, ~a, a & 0xFF);
    return std::vector<uint8_t>(t.begin(), t.end());
}

// The tablet on the XIAO's WiFi starts its program and switches autosteer on; from here the gaps in its steering
// data at the Teensy are measured (endSteer).
inline bool beginSteer(World& w)
{
    w.tab.on = true;
    w.tab.engaged = true;
    const bool ok = w.runUntil([&]() { return w.teensy.steering(); }, 8000);
    must("steer-start", ok, "A tablet on the XIAO's WiFi gets the module and steers through the bridge",
         fmt("Teensy bridge state %u, %u packets passed of %u sent", w.teensy.brState, w.tab.nPassed, w.tab.nSent));
    w.steerLastMs_ = fake::clockMs;
    w.steerWatch_ = ok;
    return ok;
}

// ... and stops. A second without steering data is a bug (the wheel holds its last angle that long); after 2.5 s
// the Teensy switches autosteer off.
inline void endSteer(World& w, uint32_t gapMaxMs = 1000)
{
    const bool watched = w.steerWatch_;
    w.steerWatch_ = false;
    w.tab.engaged = false;
    if (!watched) return;
    must("steer-gap", w.rules.worstSteerGapMs < gapMaxMs && w.teensy.steerOffs == 0, "Steering data kept reaching the Teensy while the tablet steered",
         fmt("longest time without it %u ms; the Teensy switched autosteer off %u time(s)", w.rules.worstSteerGapMs, w.teensy.steerOffs));
    note("Tablet to Teensy through the bridge", fmt("%u of %u packets passed, longest delay %u ms, longest gap in the steering data %u ms",
                                                   w.tab.nPassed, w.tab.nSent, w.tab.worstDelayMs, w.rules.worstSteerGapMs));
}
}
