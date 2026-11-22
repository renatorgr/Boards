/* json.h - a small JSON reader for the module's /api/status answers.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Json::parse() gives ok = false on anything that is not strict JSON (a NaN or "inf" a firmware printed,
// a missing comma), so a broken answer counts as a finding. j["a"]["b"].num() reads a value; a missing one
// reads as 0 / "" (check has() first when it matters).
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <cctype>

class Json
{
public:
    enum Type { Null, Bool, Number, String, Array, Object };
    Type type = Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Json> a;
    std::map<std::string, Json> o;

    static Json parse(const std::string& text, bool& ok)
    {
        size_t i = 0;
        Json j;
        ok = value(text, i, j);
        skip(text, i);
        if (i != text.size()) ok = false;
        return j;
    }

    const Json& operator[](const std::string& k) const
    {
        static const Json none;
        auto it = o.find(k);
        return it == o.end() ? none : it->second;
    }
    const Json& operator[](size_t k) const
    {
        static const Json none;
        return k < a.size() ? a[k] : none;
    }
    bool has(const std::string& k) const { return o.count(k) > 0; }
    double num() const { return type == Number ? n : type == Bool ? (b ? 1 : 0) : 0; }
    const std::string& str() const { return s; }
    size_t size() const { return type == Array ? a.size() : o.size(); }

private:
    static void skip(const std::string& t, size_t& i) { while (i < t.size() && isspace((unsigned char)t[i])) i++; }
    static bool lit(const std::string& t, size_t& i, const char* w)
    {
        const size_t n = strlen(w);
        if (t.compare(i, n, w) != 0) return false;
        i += n;
        return true;
    }
    static bool str(const std::string& t, size_t& i, std::string& out)
    {
        if (i >= t.size() || t[i] != '"') return false;
        i++;
        out.clear();
        while (i < t.size() && t[i] != '"')
        {
            char c = t[i++];
            if ((unsigned char)c < 0x20) return false;
            if (c == '\\')
            {
                if (i >= t.size()) return false;
                c = t[i++];
                switch (c)
                {
                case '"': case '\\': case '/': out += c; break;
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'u': if (i + 4 > t.size()) return false; out += '?'; i += 4; break;
                default: return false;
                }
            }
            else out += c;
        }
        if (i >= t.size()) return false;
        i++;
        return true;
    }
    static bool value(const std::string& t, size_t& i, Json& j)
    {
        skip(t, i);
        if (i >= t.size()) return false;
        const char c = t[i];
        if (c == '{')
        {
            j.type = Object;
            i++;
            skip(t, i);
            if (i < t.size() && t[i] == '}') { i++; return true; }
            for (;;)
            {
                skip(t, i);
                std::string k;
                if (!str(t, i, k)) return false;
                skip(t, i);
                if (i >= t.size() || t[i] != ':') return false;
                i++;
                Json v;
                if (!value(t, i, v)) return false;
                j.o[k] = v;
                skip(t, i);
                if (i < t.size() && t[i] == ',') { i++; continue; }
                if (i < t.size() && t[i] == '}') { i++; return true; }
                return false;
            }
        }
        if (c == '[')
        {
            j.type = Array;
            i++;
            skip(t, i);
            if (i < t.size() && t[i] == ']') { i++; return true; }
            for (;;)
            {
                Json v;
                if (!value(t, i, v)) return false;
                j.a.push_back(v);
                skip(t, i);
                if (i < t.size() && t[i] == ',') { i++; continue; }
                if (i < t.size() && t[i] == ']') { i++; return true; }
                return false;
            }
        }
        if (c == '"') { j.type = String; return str(t, i, j.s); }
        if (lit(t, i, "true")) { j.type = Bool; j.b = true; return true; }
        if (lit(t, i, "false")) { j.type = Bool; j.b = false; return true; }
        if (lit(t, i, "null")) { j.type = Null; return true; }
        // a number: -?digits(.digits)?([eE][+-]?digits)?
        const size_t start = i;
        if (t[i] == '-') i++;
        if (i >= t.size() || !isdigit((unsigned char)t[i])) return false;
        while (i < t.size() && isdigit((unsigned char)t[i])) i++;
        if (i < t.size() && t[i] == '.')
        {
            i++;
            if (i >= t.size() || !isdigit((unsigned char)t[i])) return false;
            while (i < t.size() && isdigit((unsigned char)t[i])) i++;
        }
        if (i < t.size() && (t[i] == 'e' || t[i] == 'E'))
        {
            i++;
            if (i < t.size() && (t[i] == '+' || t[i] == '-')) i++;
            if (i >= t.size() || !isdigit((unsigned char)t[i])) return false;
            while (i < t.size() && isdigit((unsigned char)t[i])) i++;
        }
        j.type = Number;
        j.n = strtod(t.substr(start, i - start).c_str(), nullptr);
        return true;
    }
};
