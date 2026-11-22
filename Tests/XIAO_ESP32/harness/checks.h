/* checks.h - the scenarios' checks and the list of scenarios.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Two kinds of check:
//   must(key, ok, what, detail)  the XIAO must do this (safe and right). A failure is a BUG and fails the run,
//                                unless known-bugs.txt lists it (then it is reported as a known bug).
//   tell(key, ok, what, detail)  the user made a mistake, or something around the XIAO is wrong: does the XIAO tell
//                                them (its page, its Messages)? A failure is reported as "not told", never fails the run.
//   note(what, detail)           a number or a fact for the report.
// A check's id is "<scenario>/<key>@<kit>"; known-bugs.txt can list "<scenario>/<key>" (both XIAOs) or the full id.
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
using bench::fmt;

enum class User { Careful, Unconfigured, WrongSettings, WrongConnections, BadLink };

struct Scenario
{
    User user;
    std::string id;                 // short, stable: used in known-bugs.txt
    std::string title;              // what the user does, in one line
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
#define SCENARIO(user, id, title, kits) \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Kit kit); \
    static chk::Add SCENARIO_CAT(add_, __LINE__)(user, id, title, kits, SCENARIO_CAT(scenario_, __LINE__)); \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Kit kit)
