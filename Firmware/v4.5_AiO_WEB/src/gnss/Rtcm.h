/* Rtcm.h - counts the RTCM 3 messages in a stream of correction bytes.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Rtcm.h
// The corrections pass to the receiver untouched (radio on Serial3, NTRIP on UDP 2233). On the
// way, this counts them for the web page and the [status] messages: bytes, bytes per second,
// whole RTCM 3 messages with a good checksum (CRC-24Q), messages with a bad checksum (bytes lost
// or changed on the way: a weak radio signal, a wrong baud rate) and the message types seen in
// the last 10 s (for example 1005 1074 1084 1094 1124 1230).
// RTCM 3 message: D3, 6 bits 0 + 10 bits length, the message (its first 12 bits = its type),
// 3 bytes of CRC-24Q over everything before them.
#ifndef ZRTCM_H
#define ZRTCM_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RTCM_TYPES      12        // message types remembered
#define RTCM_TYPE_MS    10000     // a type is listed if it was seen in the last 10 s
#define RTCM_GAP_MS     2000      // after a gap this long the next bytes may start in mid-message

struct RtcmCounter
{
    uint32_t bytes = 0, good = 0, bad = 0;      // since the start
    uint32_t lastMs = 0, lastGoodMs = 0;         // last byte, last good message
    uint32_t bps = 0;                            // bytes per second in the last window
    uint32_t winBytes = 0, winGood = 0, winBad = 0;   // the counts when the window started
    uint32_t newGood = 0, newBad = 0, newBytes = 0;   // counts in the last finished window
    uint16_t types[RTCM_TYPES] = {0};
    uint32_t typeMs[RTCM_TYPES] = {0};

    // the message being read
    uint8_t  st = 0;
    bool     syncing = true;                     // looking for the first whole message: a bad one is not counted
    uint16_t len = 0, pos = 0, type = 0;
    uint32_t crc = 0, rx = 0;

    static uint32_t crc24(uint32_t c, uint8_t b)
    {
        c ^= (uint32_t)b << 16;
        for (int i = 0; i < 8; i++) { c <<= 1; if (c & 0x1000000UL) c ^= 0x1864CFBUL; }
        return c & 0xFFFFFFUL;
    }

    void feed(const uint8_t* d, uint32_t n, uint32_t now)
    {
        if (n == 0) return;
        if (lastMs == 0 || now - lastMs > RTCM_GAP_MS) { st = 0; syncing = true; }
        bytes += n;
        lastMs = now;
        for (uint32_t i = 0; i < n; i++) next(d[i], now);
    }

    void next(uint8_t b, uint32_t now)
    {
        switch (st)
        {
            case 0:                                   // preamble
                if (b == 0xD3) { crc = crc24(0, b); st = 1; }
                break;
            case 1:                                   // 6 reserved bits (0) + length bits 9-8
                if (b & 0xFC) { st = 0; if (b == 0xD3) { crc = crc24(0, b); st = 1; } break; }
                len = (uint16_t)(b & 0x03) << 8;
                crc = crc24(crc, b);
                st  = 2;
                break;
            case 2:                                   // length bits 7-0
                len |= b;
                crc  = crc24(crc, b);
                pos  = 0; type = 0; rx = 0;
                st   = len ? 3 : 4;
                break;
            case 3:                                   // the message
                crc = crc24(crc, b);
                if (pos == 0)      type = (uint16_t)b << 4;
                else if (pos == 1) type |= (uint16_t)(b >> 4);
                if (++pos >= len) { pos = 0; st = 4; }
                break;
            default:                                  // CRC-24Q, 3 bytes
                rx = (rx << 8) | b;
                if (++pos >= 3)
                {
                    if (rx == crc) { good++; lastGoodMs = now; syncing = false; noteType(type, now); }
                    else if (!syncing) bad++;
                    st = 0;
                }
                break;
        }
    }

    void noteType(uint16_t t, uint32_t now)
    {
        int slot = -1, oldest = 0;
        for (int k = 0; k < RTCM_TYPES; k++)
        {
            if (types[k] == t && typeMs[k]) { slot = k; break; }
            if (typeMs[k] < typeMs[oldest]) oldest = k;
        }
        if (slot < 0) slot = oldest;
        types[slot]  = t;
        typeMs[slot] = now ? now : 1;
    }

    // The types seen in the last RTCM_TYPE_MS, in order: "1005 1074 1084"
    void typesText(char* out, size_t n, uint32_t now) const
    {
        uint16_t list[RTCM_TYPES];
        int k = 0;
        for (int i = 0; i < RTCM_TYPES; i++)
            if (typeMs[i] && now - typeMs[i] < RTCM_TYPE_MS) list[k++] = types[i];
        for (int a = 0; a < k; a++)
            for (int b = a + 1; b < k; b++)
                if (list[b] < list[a]) { uint16_t t = list[a]; list[a] = list[b]; list[b] = t; }
        out[0] = '\0';
        size_t used = 0;
        for (int i = 0; i < k && used + 6 < n; i++)
            used += snprintf(out + used, n - used, i ? " %u" : "%u", (unsigned)list[i]);
    }

    // Called every window (5 s): rate and the counts of the window that ended
    void window(uint32_t seconds)
    {
        newBytes = bytes - winBytes;
        newGood  = good - winGood;
        newBad   = bad - winBad;
        bps      = seconds ? newBytes / seconds : 0;
        winBytes = bytes; winGood = good; winBad = bad;
    }

    bool arriving(uint32_t now, uint32_t silenceMs) const { return lastMs != 0 && now - lastMs < silenceMs; }
};

#endif
