/* main.cpp - runs the scenarios (each in its own process, several at once) and writes the report.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// ./harness [--only <text>] [--kit <key>] [--report <file.md>] [--known <known-bugs.txt>] [--jobs <n>] [--list]
// Each scenario runs on each of its kits in a child process, so a crash (or a memory error the address
// sanitizer catches in the firmware) is one finding and the rest still runs; --jobs of them at once (default:
// the number of processors). Exit code 1 when a "must" check failed that known-bugs.txt does not list, or a
// scenario crashed.
#include "checks.h"
#include <array>
#include <map>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

namespace chk
{
static int resultFd = -1;

std::vector<Scenario>& registry()
{
    static std::vector<Scenario> r;
    return r;
}

static std::string esc(const std::string& s)
{
    std::string o;
    for (char c : s)
    {
        if (c == '\t') o += "\\t";
        else if (c == '\n') o += "\\n";
        else if (c == '\\') o += "\\\\";
        else o += c;
    }
    return o;
}

static void emit(const char* kind, const std::string& key, bool ok, const std::string& what, const std::string& detail)
{
    const std::string line = std::string(kind) + "\t" + (ok ? "1" : "0") + "\t" + esc(key) + "\t" + esc(what) + "\t" + esc(detail) + "\n";
    if (resultFd >= 0) { ssize_t n = write(resultFd, line.data(), line.size()); (void)n; }
}

void must(const std::string& key, bool ok, const std::string& what, const std::string& detail) { emit("must", key, ok, what, detail); }
void tell(const std::string& key, bool ok, const std::string& what, const std::string& detail) { emit("tell", key, ok, what, detail); }
void note(const std::string& what, const std::string& detail) { emit("note", "", true, what, detail); }

std::string fmt(const char* f, ...)
{
    char b[2048];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof(b), f, ap);
    va_end(ap);
    return b;
}
}

using namespace chk;

static const char* userName(User u)
{
    switch (u)
    {
    case User::Careful:       return "Careful user";
    case User::Unconfigured:  return "Never set up";
    case User::WrongSettings: return "Wrong settings";
    case User::WrongWiring:   return "Wrong wiring";
    case User::FieldFailures: return "Failures in the field";
    }
    return "?";
}

struct Line
{
    std::string kind, key, what, detail;
    bool ok;
    std::string id;                 // scenario/key@kit
};
struct RunResult
{
    const Scenario* sc;
    bench::Kit kit;
    std::vector<Line> lines;
    bool crashed = false;
    std::string crash;
};

static std::string unesc(const std::string& s)
{
    std::string o;
    for (size_t i = 0; i < s.size(); i++)
    {
        if (s[i] == '\\' && i + 1 < s.size())
        {
            const char c = s[++i];
            o += c == 't' ? '\t' : c == 'n' ? '\n' : c;
        }
        else o += s[i];
    }
    return o;
}

static std::vector<std::string> split(const std::string& s, char d)
{
    std::vector<std::string> r;
    std::string cur;
    for (char c : s) { if (c == d) { r.push_back(cur); cur.clear(); } else cur += c; }
    r.push_back(cur);
    return r;
}

// What a crashed run wrote: from a sanitizer's first line (ERROR: / runtime error:) n lines, so its stack is in
// the report; else the last n lines
static std::string tailLines(const std::string& s, int n)
{
    std::vector<std::string> l = split(s, '\n');
    size_t from = l.size() > (size_t)n ? l.size() - n : 0;
    for (size_t i = 0; i < l.size(); i++)
        if (l[i].find("ERROR: ") != std::string::npos || l[i].find("runtime error:") != std::string::npos) { from = i; break; }
    std::string o;
    for (size_t i = from; i < l.size() && i < from + n; i++) o += l[i] + "\n";
    return o;
}

struct Child
{
    pid_t pid = 0;
    const Scenario* sc = nullptr;
    bench::Kit kit;
    std::string outPath, errPath;
};

static Child startOne(const Scenario& sc, bench::Kit k)
{
    Child ch;
    ch.sc = &sc;
    ch.kit = k;
    char outPath[] = "/tmp/teensy-harness-out-XXXXXX";
    char errPath[] = "/tmp/teensy-harness-err-XXXXXX";
    const int outFd = mkstemp(outPath);
    const int errFd = mkstemp(errPath);
    ch.outPath = outPath;
    ch.errPath = errPath;
    fflush(stdout);
    fflush(stderr);
    const pid_t pid = fork();
    if (pid == 0)
    {
        dup2(errFd, 2);
        dup2(errFd, 1);
        resultFd = outFd;
        alarm(600);
        try { sc.run(k); }
        catch (std::exception& e) { fprintf(stderr, "harness error: %s\n", e.what()); _exit(3); }
        fflush(stdout);
        fflush(stderr);
        _exit(0);
    }
    close(outFd);
    close(errFd);
    ch.pid = pid;
    return ch;
}

static std::string slurp(const std::string& path)
{
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static RunResult finishOne(const Child& ch, int st)
{
    RunResult rr;
    rr.sc = ch.sc;
    rr.kit = ch.kit;
    const std::string data = slurp(ch.outPath), err = slurp(ch.errPath);
    unlink(ch.outPath.c_str());
    unlink(ch.errPath.c_str());
    for (const std::string& l : split(data, '\n'))
    {
        if (l.empty()) continue;
        std::vector<std::string> f = split(l, '\t');
        if (f.size() < 5) continue;
        Line x;
        x.kind = f[0];
        x.ok = f[1] == "1";
        x.key = unesc(f[2]);
        x.what = unesc(f[3]);
        x.detail = unesc(f[4]);
        x.id = ch.sc->id + "/" + x.key + "@" + bench::info(ch.kit).key;
        rr.lines.push_back(x);
    }
    if (!(WIFEXITED(st) && WEXITSTATUS(st) == 0))
    {
        rr.crashed = true;
        if (WIFSIGNALED(st)) rr.crash = fmt("killed by signal %d", WTERMSIG(st));
        else rr.crash = fmt("exit code %d", WEXITSTATUS(st));
        rr.crash += "\n" + tailLines(err, 30);
    }
    return rr;
}

int main(int argc, char** argv)
{
    std::string only, kitKey, reportPath, knownPath = "known-bugs.txt";
    bool list = false;
    long jobs = sysconf(_SC_NPROCESSORS_ONLN);
    for (int i = 1; i < argc; i++)
    {
        const std::string a = argv[i];
        if (a == "--only" && i + 1 < argc) only = argv[++i];
        else if (a == "--kit" && i + 1 < argc) kitKey = argv[++i];
        else if (a == "--jobs" && i + 1 < argc) jobs = atol(argv[++i]);
        else if (a == "--report" && i + 1 < argc) reportPath = argv[++i];
        else if (a == "--known" && i + 1 < argc) knownPath = argv[++i];
        else if (a == "--list") list = true;
        else { fprintf(stderr, "usage: %s [--only text] [--kit key] [--report file.md] [--known file] [--jobs n] [--list]\n", argv[0]); return 2; }
    }

    // known bugs: "<scenario>/<key>[@kit]  reason"
    std::map<std::string, std::string> known;
    {
        std::ifstream f(knownPath);
        std::string l;
        while (std::getline(f, l))
        {
            if (l.empty() || l[0] == '#') continue;
            const size_t sp = l.find_first_of(" \t");
            const std::string id = l.substr(0, sp);
            std::string why = sp == std::string::npos ? "" : l.substr(sp);
            while (!why.empty() && (why[0] == ' ' || why[0] == '\t')) why.erase(0, 1);
            known[id] = why;
        }
    }
    auto knownFor = [&](const Line& x, const std::string& scId) -> const std::string* {
        auto it = known.find(x.id);
        if (it != known.end()) return &it->second;
        it = known.find(scId + "/" + x.key);
        return it != known.end() ? &it->second : nullptr;
    };

    if (jobs < 1) jobs = 1;
    std::vector<std::pair<const Scenario*, bench::Kit>> todo;
    for (const Scenario& sc : registry())
    {
        if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
        for (bench::Kit k : sc.kits)
        {
            if (!kitKey.empty() && kitKey != bench::info(k).key) continue;
            if (list) { printf("%-30s %-14s %s\n", sc.id.c_str(), bench::info(k).key, sc.title.c_str()); continue; }
            todo.push_back({ &sc, k });
        }
    }
    std::vector<RunResult> results(todo.size());
    std::map<pid_t, std::pair<size_t, Child>> running;
    size_t next = 0, done = 0;
    while (done < todo.size())
    {
        while (next < todo.size() && (long)running.size() < jobs)
        {
            Child ch = startOne(*todo[next].first, todo[next].second);
            running[ch.pid] = { next, ch };
            next++;
        }
        int st = 0;
        const pid_t pid = waitpid(-1, &st, 0);
        auto it = running.find(pid);
        if (it == running.end()) continue;
        const size_t idx = it->second.first;
        results[idx] = finishOne(it->second.second, st);
        running.erase(it);
        done++;
        RunResult& rr = results[idx];
        const Scenario& sc = *rr.sc;
        int bad = 0, gaps = 0;
        for (auto& x : rr.lines) { if (!x.ok && x.kind == "must" && !knownFor(x, sc.id)) bad++; if (!x.ok && x.kind == "tell") gaps++; }
        printf("%-5s %-30s %-14s %s%s\n", rr.crashed ? "CRASH" : bad ? "BUG" : gaps ? "told?" : "ok", sc.id.c_str(),
               bench::info(rr.kit).key, sc.title.c_str(), gaps ? fmt("  (%d not told)", gaps).c_str() : "");
        if (rr.crashed) printf("%s\n", rr.crash.c_str());
        for (auto& x : rr.lines)
            if (!x.ok && x.kind == "must") printf("      %s %s: %s\n", knownFor(x, sc.id) ? "known bug" : "BUG", x.what.c_str(), x.detail.c_str());
        fflush(stdout);
    }
    if (list) return 0;

    // ---------------------------------------------------------------- the report
    int bugs = 0, knownBugs = 0, notTold = 0, passed = 0, crashes = 0, fixedKnown = 0;
    std::ostringstream bugsMd, knownMd, toldMd, fixedMd, passMd, notesMd;
    std::map<User, std::array<int, 4>> perUser;     // passed, bugs, known, not told
    for (auto& rr : results)
    {
        const std::string head = rr.sc->title + " (`" + rr.sc->id + "`, " + bench::info(rr.kit).key + ")";
        auto& pu = perUser[rr.sc->user];
        if (rr.crashed)
        {
            crashes++;
            pu[1]++;
            bugsMd << "- **Crashed:** " << head << "\n\n```\n" << rr.crash << "```\n";
        }
        for (auto& x : rr.lines)
        {
            const std::string* k = x.kind == "must" ? knownFor(x, rr.sc->id) : nullptr;
            const std::string item = "- **" + x.what + "** in " + head + (x.detail.empty() ? "" : ": " + x.detail) + " `" + x.id + "`\n";
            if (x.kind == "note") { notesMd << "- " << head << ": " << x.what << (x.detail.empty() ? "" : " " + x.detail) << "\n"; continue; }
            if (x.ok)
            {
                passed++;
                pu[0]++;
                passMd << "- " << x.what << " (" << rr.sc->id << ", " << bench::info(rr.kit).key << ")\n";
                if (k) { fixedKnown++; fixedMd << "- `" << x.id << "` passes now: take it out of known-bugs.txt\n"; }
                continue;
            }
            if (x.kind == "tell") { notTold++; pu[3]++; toldMd << item; }
            else if (k) { knownBugs++; pu[2]++; knownMd << item << "  - known: " << *k << "\n"; }
            else { bugs++; pu[1]++; bugsMd << item; }
        }
    }

    std::ostringstream md;
    char when[64];
    const time_t t = time(nullptr);
    strftime(when, sizeof(when), "%Y-%m-%d %H:%M UTC", gmtime(&t));
    const char* sha = getenv("HARNESS_COMMIT");
    md << "# Teensy AiO harness report\n\n";
    md << "Run " << when << (sha && sha[0] ? std::string(", commit ") + sha : std::string()) << ": " << results.size()
       << " scenario runs, " << passed << " checks passed, " << bugs + crashes << " bugs, " << knownBugs << " known bugs, "
       << notTold << " mistakes or failures the module does not tell the user about.\n\n";
    md << "| User | Passed | Bugs | Known bugs | Not told |\n|---|---|---|---|---|\n";
    for (auto& [u, c] : perUser) md << "| " << userName(u) << " | " << c[0] << " | " << c[1] << " | " << c[2] << " | " << c[3] << " |\n";
    md << "\n## Bugs: the module did something wrong or unsafe\n\n" << (bugsMd.str().empty() ? "None.\n" : bugsMd.str());
    md << "\n## Known bugs (known-bugs.txt)\n\n" << (knownMd.str().empty() ? "None.\n" : knownMd.str());
    if (!fixedMd.str().empty()) md << "\n## Known bugs that pass now\n\n" << fixedMd.str();
    md << "\n## Not told: a mistake or a failure the module keeps quiet about\n\n" << (toldMd.str().empty() ? "None.\n" : toldMd.str());
    md << "\n## Numbers\n\n" << (notesMd.str().empty() ? "None.\n" : notesMd.str());
    md << "\n<details><summary>Checks passed (" << passed << ")</summary>\n\n" << passMd.str() << "\n</details>\n";

    if (!reportPath.empty())
    {
        std::ofstream f(reportPath);
        f << md.str();
    }
    printf("\n%d checks passed, %d bugs, %d crashes, %d known bugs, %d not told%s\n", passed, bugs, crashes, knownBugs, notTold,
           reportPath.empty() ? "" : (", report: " + reportPath).c_str());
    if (fixedKnown) printf("%d known bugs pass now: take them out of known-bugs.txt\n", fixedKnown);
    return (bugs || crashes) ? 1 : 0;
}
