/* checks.h - the scenarios' checks and the list of scenarios.
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Two kinds of check:
//   must(key, ok, what, detail)  the module must do this (safe and right). A failure is a BUG and fails the run,
//                                unless known-bugs.txt lists it (then it is reported as a known bug).
//   tell(key, ok, what, detail)  the user made a mistake: does the module tell them (its page, its log, a message
//                                on AgOpenWeb's screen)? A failure is reported as "not told", never fails the run.
//   note(what, detail)           a number or a fact for the report.
// A check's id is "<scenario>/<key>@<board>"; known-bugs.txt can list "<scenario>/<key>" (every board) or the full id.
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

enum class User { Careful, Unconfigured, WrongSettings, WrongSlots, BadNetwork };

struct Scenario
{
    User user;
    std::string id;                 // short, stable: used in known-bugs.txt
    std::string title;              // what the user does, in one line
    std::vector<bench::Board> boards;
    std::function<void(bench::Board)> run;
};
std::vector<Scenario>& registry();

struct Add
{
    Add(User u, const char* id, const char* title, std::vector<bench::Board> boards, std::function<void(bench::Board)> fn)
    {
        registry().push_back({ u, id, title, boards, fn });
    }
};
}

#define SCENARIO_CAT2(a, b) a##b
#define SCENARIO_CAT(a, b) SCENARIO_CAT2(a, b)
#define SCENARIO(user, id, title, boards) \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Board board); \
    static chk::Add SCENARIO_CAT(add_, __LINE__)(user, id, title, boards, SCENARIO_CAT(scenario_, __LINE__)); \
    static void SCENARIO_CAT(scenario_, __LINE__)(bench::Board board)
