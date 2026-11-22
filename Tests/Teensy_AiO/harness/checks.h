/* checks.h - the scenarios' checks and the list of scenarios.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Two kinds of check, as in Tests/ESP32_Modules:
//   must(key, ok, what, detail)  the module must do this (safe and right). A failure is a BUG and fails the run,
//                                unless known-bugs.txt lists it (then it is reported as a known bug).
//   tell(key, ok, what, detail)  the user made a mistake or something broke: does the module tell them (its web
//                                page's messages, a text on AgOpenWeb's screen)? A failure is "not told", never fails the run.
//   note(what, detail)           a number or a fact for the report.
// A check's id is "<scenario>/<key>@<kit>"; known-bugs.txt can list "<scenario>/<key>" (every kit) or the full id.
#pragma once
#include "bench.h"
#include <string>
#include <vector>
#include <functional>

namespace chk
{
void must(const std::string& key, bool ok, const std::string& what, const std::string& detail = "");
void tell(const std::string& key, bool ok, const std::string& what, const std::string& detail = "");
void note(const std::string& what, const std::string& detail = "");
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));

enum class User { Careful, Unconfigured, WrongSettings, WrongWiring, FieldFailures };

struct Scenario
{
    User user;
    std::string id;                 // short, stable: used in known-bugs.txt
    std::string title;              // what the user does (or what happens), in one line
    std::vector<bench::Kit> kits;
    std::function<void(bench::Kit)> run;
};
std::vector<Scenario>& registry();

struct Add
{
    Add(User u, const char* id, const char* title, std::vector<bench::Kit> kits, std::function<void(bench::Kit)> fn)
    {
        registry().push_back({ u, id, title, kits, fn });
    }
};
}

#define SCENARIO_CAT2(a, b) a##b
#define SCENARIO_CAT(a, b) SCENARIO_CAT2(a, b)
// SCENARIO(user, id, title, kits...): kits is a list (allKits()) or { Kit::A, Kit::B }
#define SCENARIO(user, id, title, ...) \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Kit kit); \
    static chk::Add SCENARIO_CAT(add_, __LINE__)(user, id, title, __VA_ARGS__, SCENARIO_CAT(scenario_, __LINE__)); \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Kit kit)
