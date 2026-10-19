/* xLink.h - the XIAO side of the link to the Teensy.
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware
   AiO_WEB (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn"
   (AgOpenGPS-Official, GPL v3).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// xLink.h
// The XIAO's side of the link to the Teensy (firmware v4.5_AiO_WEB, zXiao.ino):
// UART1 on D7 (RX: GPIO20 on the C3, GPIO44 on the S3) / D6 (TX: GPIO21 / GPIO43), 460800 baud, 8N1.
//
//   XIAO -> Teensy   "ESP32-hello" every LINK_HELLO_MS (no line ending): the Teensy then sends us its
//                    AgOpenGPS packets, its status and its messages
//                    text lines ending in "\n": shown in the Teensy log as "[xiao] > ..."
//                    PGN 143 (source 0x50): our state as a short JSON (program, WiFi), for the Teensy page
//   Teensy -> XIAO   AgOpenGPS packets from UDP 8888 (source 0x7F): PGN 254, 239, 229, 200, ...
//                    PGN 141 (source 0x7E): the module status JSON, once a second, in pieces
//                    PGN 142 (source 0x7E): one line of the module's [status] log
// Every frame: [0x80][0x81][source][pgn][length][data...][checksum = low 8 bits of the sum of the
// bytes from source to the last data byte]. Frames with a wrong checksum are dropped.
//
// Pure C++ (no Arduino-only calls) so it can also be tested on a PC: S is any type with
// available(), read(), write(const uint8_t*, size_t).
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define LINK_BAUD               460800   // = XIAO_BAUD in the Teensy firmware (zXiao.h)
#define LINK_HELLO_MS           2000
#define LINK_INFO_MS            5000
#define LINK_SILENT_MS          3000     // nothing from the Teensy for this long: "sem dados"
#define LINK_GAP_MS             50       // a frame not finished this long after its last byte is dropped
#define LINK_LOG_LINES          60       // module log lines kept (= LOG_LINES in the Teensy)
#define LINK_LOG_LEN            200
#define LINK_STATUS_MAX         3072
#define LINK_SRC_XIAO           0x50
#define LINK_SRC_TEENSY         0x7E
#define LINK_SRC_AOG            0x7F
#define LINK_PGN_STATUS         141
#define LINK_PGN_LOG            142
#define LINK_PGN_INFO           143

struct LinkLogLine
{
    uint32_t seq;                        // the Teensy's sequence number (0 = empty slot)
    uint32_t ms;                         // the Teensy's millis() when the line was written
    char     text[LINK_LOG_LEN + 1];
};

template <class S>
class TeensyLink
{
public:
    explicit TeensyLink(S& s) : port(s) { memset(pgnCount, 0, sizeof(pgnCount)); memset(logRing, 0, sizeof(logRing)); }

    // ---- counters (web page)
    uint32_t rxBytes = 0, txBytes = 0, goodFrames = 0, crcErr = 0, cut = 0, hellos = 0;
    uint32_t pgnCount[256];              // frames from AgOpenGPS (source 0x7F) by PGN
    uint32_t lastFrameMs = 0;            // last good frame from the Teensy (0 = never)

    // ---- what AgOpenGPS / AgOpenWeb sends to the module (decoded here)
    struct Steer   { bool have = false; uint32_t ms = 0; float kmh = 0, angle = 0; uint8_t status = 0; uint16_t sections = 0; } p254;
    struct Machine { bool have = false; uint32_t ms = 0; uint8_t uturn = 0, hydLift = 0, tram = 0, geoStop = 0; uint16_t sections = 0; } p239;
    uint32_t last200Ms = 0;              // last hello from AgIO / AgOpenWeb (PGN 200)
    float    rate254 = 0;                // PGN 254 per second (last full second)

    // ---- the module status (JSON from the Teensy) and its log
    char     status[LINK_STATUS_MAX];
    uint16_t statusLen = 0;
    uint32_t statusMs = 0;               // when the last complete status arrived (0 = never)
    uint32_t statusSets = 0, statusBroken = 0;
    LinkLogLine logRing[LINK_LOG_LINES];
    uint32_t logLast = 0;                // highest sequence number stored
    uint32_t logEpoch = 1;               // +1 when the Teensy restarted (its numbers start again)

    bool teensyTalking(uint32_t now) const { return lastFrameMs != 0 && now - lastFrameMs < LINK_SILENT_MS; }

    // Called from loop(): read and parse, send the hello when it is due
    void poll(uint32_t now)
    {
        if (now - helloMs >= LINK_HELLO_MS || helloMs == 0) { helloMs = now | 1; sendRaw((const uint8_t*)"ESP32-hello", 11); hellos++; }

        if (fLen > 0 && now - fLastMs > LINK_GAP_MS && port.available() == 0) { cut++; shift(1); scan(now); fLastMs = now; }

        int budget = 4096;
        while (budget-- > 0 && port.available() > 0)
        {
            const int c = port.read();
            if (c < 0) break;
            rxBytes++;
            if (fLen == 0 && c != 0x80) continue;      // the Teensy only sends frames
            f[fLen++] = (uint8_t)c;                    // never overflows: scan() keeps fLen < 261
            fLastMs = now;
            scan(now);
        }

        if (now - rateMs >= 1000) { rate254 = (float)(n254 * 1000.0 / (now - rateMs)); n254 = 0; rateMs = now; }
    }

    // A text line for the Teensy log ("[xiao] > ..."); printable ASCII only, cut at 150 characters
    void sendText(const char* line)
    {
        uint8_t b[152];
        size_t k = 0;
        for (const char* p = line; *p && k < 150; p++) if ((uint8_t)*p >= 0x20 && (uint8_t)*p <= 0x7E) b[k++] = (uint8_t)*p;
        b[k++] = '\n';
        sendRaw(b, k);
    }

    // Our state for the Teensy page (PGN 143), a JSON text of at most 250 bytes
    void sendInfo(const char* json)
    {
        size_t n = strlen(json);
        if (n > 250) return;
        sendFrame(LINK_SRC_XIAO, LINK_PGN_INFO, (const uint8_t*)json, (uint8_t)n);
    }

    void sendFrame(uint8_t src, uint8_t pgn, const uint8_t* data, uint8_t len)
    {
        uint8_t b[6 + 255];
        b[0] = 0x80; b[1] = 0x81; b[2] = src; b[3] = pgn; b[4] = len;
        memcpy(b + 5, data, len);
        uint8_t sum = 0;
        for (int i = 2; i < 5 + len; i++) sum += b[i];
        b[5 + len] = sum;
        sendRaw(b, 6 + len);
    }

private:
    S&       port;
    uint8_t  f[5 + 255 + 1 + 1];
    uint16_t fLen = 0;
    uint32_t fLastMs = 0, helloMs = 0, rateMs = 0, n254 = 0;
    char     asmBuf[LINK_STATUS_MAX];
    uint16_t asmLen = 0;
    int      asmSet = -1;
    uint8_t  asmNext = 0;

    void sendRaw(const uint8_t* b, size_t n) { port.write(b, n); txBytes += n; }

    void shift(uint16_t n)
    {
        while (n < fLen && f[n] != 0x80) n++;
        if (n >= fLen) { fLen = 0; return; }
        memmove(f, f + n, fLen - n);
        fLen -= n;
    }

    void scan(uint32_t now)
    {
        while (fLen > 0)
        {
            if (f[0] != 0x80 || (fLen >= 2 && f[1] != 0x81)) { shift(1); continue; }
            if (fLen < 5) return;
            const uint16_t need = 6 + f[4];
            if (fLen < need) return;
            uint8_t sum = 0;
            for (uint16_t i = 2; i < need - 1; i++) sum += f[i];
            if (sum == f[need - 1]) { frame(f[2], f[3], f + 5, f[4], now); shift(need); }
            else                    { crcErr++; shift(1); }
        }
    }

    void frame(uint8_t src, uint8_t pgn, const uint8_t* d, uint8_t n, uint32_t now)
    {
        goodFrames++;
        lastFrameMs = now | 1;
        if (src == LINK_SRC_TEENSY && pgn == LINK_PGN_STATUS && n >= 3) { statusPiece(d, n, now); return; }
        if (src == LINK_SRC_TEENSY && pgn == LINK_PGN_LOG && n >= 8)     { logLine(d, n); return; }
        pgnCount[pgn]++;
        if (pgn == 254 && n >= 8)
        {
            p254.have = true; p254.ms = now | 1;
            p254.kmh = (uint16_t)(d[0] | d[1] << 8) * 0.1f;
            p254.status = d[2];
            p254.angle = (int16_t)(d[3] | d[4] << 8) * 0.01f;
            p254.sections = (uint16_t)(d[6] | d[7] << 8);
            n254++;
        }
        else if (pgn == 239 && n >= 8)
        {
            p239.have = true; p239.ms = now | 1;
            p239.uturn = d[0]; p239.hydLift = d[2]; p239.tram = d[3]; p239.geoStop = d[4];
            p239.sections = (uint16_t)(d[6] | d[7] << 8);
        }
        else if (pgn == 200) last200Ms = now | 1;
    }

    // PGN 141: [set][piece][pieces][JSON text]; a set is kept only when every piece arrived in order
    void statusPiece(const uint8_t* d, uint8_t n, uint32_t now)
    {
        const uint8_t set = d[0], part = d[1], parts = d[2];
        const uint8_t k = n - 3;
        if (part == 0) { asmSet = set; asmLen = 0; asmNext = 0; }
        if (asmSet != set || part != asmNext || parts == 0 || asmLen + k > sizeof(asmBuf)) { if (asmSet >= 0) statusBroken++; asmSet = -1; return; }
        memcpy(asmBuf + asmLen, d + 3, k);
        asmLen += k;
        asmNext++;
        if (asmNext == parts)
        {
            asmSet = -1;
            if (asmLen >= 2 && asmBuf[0] == '{' && asmBuf[asmLen - 1] == '}')
            {
                memcpy(status, asmBuf, asmLen);
                statusLen = asmLen;
                statusMs = now | 1;
                statusSets++;
            }
            else statusBroken++;
        }
    }

    // PGN 142: [seq LE][ms LE][text]. The last LINK_LOG_LINES lines, by sequence number.
    void logLine(const uint8_t* d, uint8_t n)
    {
        uint32_t seq, ms;
        memcpy(&seq, d, 4);                       // ESP32-C3/S3 and Teensy are all little endian
        memcpy(&ms, d + 4, 4);
        if (seq == 0) return;
        LinkLogLine* l = &logRing[seq % LINK_LOG_LINES];
        // The Teensy restarted (its numbers start again at 1): the numbers went back a lot, or a
        // number we hold comes again with another time
        if ((logLast > LINK_LOG_LINES && seq + LINK_LOG_LINES < logLast) || (l->seq == seq && l->ms != ms))
        {
            memset(logRing, 0, sizeof(logRing));
            logLast = 0;
            logEpoch++;
        }
        else
        {
            if (l->seq == seq) return;                  // already have it (sent again after a hello)
            if (seq + LINK_LOG_LINES <= logLast) return; // too old for the ring
        }
        l->seq = seq;
        l->ms = ms;
        size_t k = 0;
        for (uint8_t i = 8; i < n && k < LINK_LOG_LEN; i++) if (d[i] >= 0x20 && d[i] <= 0x7E) l->text[k++] = (char)d[i];
        l->text[k] = '\0';
        if (seq > logLast) logLast = seq;
    }
};
