/* zStatusLog.h - the [status] log kept for the web page.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zStatusLog.h
// The [status] log (zStatus.ino): every line goes to the USB serial monitor and to a ring of
// LOG_LINES lines in RAM that the web page shows (zWeb.ino). Included at the top of the main
// .ino so every file can write to it.
#ifndef ZSTATUSLOG_H
#define ZSTATUSLOG_H

#include <stdarg.h>

#define STATUS_TO_USB       1       // 0 = [status] lines only in the web page log, not on the USB serial monitor
#define LOG_LINES           60      // lines kept for the web page
#define LOG_LINE_LEN        200
// ---- The log: a line-oriented printer. print()/println() like Serial; each completed line is
// stored with a sequence number and its time (millis), and echoed to USB (STATUS_TO_USB).
struct LogLine { uint32_t seq; uint32_t ms; char text[LOG_LINE_LEN]; };

class StatusLogClass
{
public:
    LogLine  lines[LOG_LINES];
    uint32_t nextSeq = 1;                 // sequence number of the next line

    void print(const char* s)        { while (*s) put(*s++); }
    void print(char c)               { put(c); }
    void print(int v)                { char b[16]; snprintf(b, sizeof(b), "%d", v); print(b); }
    void print(unsigned int v)       { char b[16]; snprintf(b, sizeof(b), "%u", v); print(b); }
    void print(long v)               { char b[24]; snprintf(b, sizeof(b), "%ld", v); print(b); }
    void print(unsigned long v)      { char b[24]; snprintf(b, sizeof(b), "%lu", v); print(b); }
    void print(double v, int d = 2)  { char b[32]; snprintf(b, sizeof(b), "%.*f", d, v); print(b); }
    void print(const IPAddress& ip)  { char b[20]; snprintf(b, sizeof(b), "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]); print(b); }
    void println()                   { put('\n'); }
    template <typename T> void println(T v)    { print(v); put('\n'); }
    void println(double v, int d)    { print(v, d); put('\n'); }

    // printf-style complete line
    void line(const char* fmt, ...)
    {
        char b[LOG_LINE_LEN];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        print(b);
        put('\n');
    }

private:
    char   cur[LOG_LINE_LEN];
    size_t len = 0;

    void put(char c)
    {
        if (c == '\r') return;
        if (c == '\n')
        {
            cur[len] = '\0';
#if STATUS_TO_USB
            Serial.println(cur);
#endif
            LogLine& l = lines[nextSeq % LOG_LINES];
            l.seq = nextSeq++;
            l.ms  = millis();
            memcpy(l.text, cur, len + 1);
            len = 0;
            return;
        }
        if (len < LOG_LINE_LEN - 1) cur[len++] = c;
    }
};
extern StatusLogClass StatusLog;   // defined in zStatus.ino

#endif
