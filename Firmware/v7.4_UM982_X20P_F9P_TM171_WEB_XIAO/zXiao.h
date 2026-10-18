/* zXiao.h - state of the XIAO ESP32 link.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 Micro board
   (renatorgr, github.com/renatorgr/Boards).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zXiao.h
// Counters and state of the XIAO ESP32C3 bridge (zXiao.ino), read by the web page (zWeb.ino).
// In a header, included at the top of the main .ino, because the Arduino IDE joins the .ino files
// in alphabetical order (zWeb.ino comes before zXiao.ino).
#ifndef ZXIAO_H
#define ZXIAO_H

#include <stdint.h>

struct XiaoStats
{
    uint32_t hellos;          // "ESP32-hello" received
    uint32_t toXiao;          // AgOpenGPS packets copied to the XIAO
    uint32_t fromXiao;        // good frames from the XIAO sent to UDP 9999
    uint32_t crcErr;          // frames from the XIAO with a wrong checksum (dropped)
    uint32_t cut;             // frames from the XIAO that stopped half way (dropped)
    uint32_t dropTx;          // packets for the XIAO dropped: Serial2 transmit buffer full
    uint32_t dropBig;         // packets for the XIAO dropped: larger than the UDP read buffer
    uint32_t beforeHello;     // good frames from the XIAO not sent: no hello in the last 10 s
    uint32_t rxBytes, txBytes;
    uint32_t textLines, textDropped;
    int16_t  lastPgn;         // last good PGN from the XIAO (-1: none yet)
    uint32_t statusSent;      // v7.2: module status JSONs sent to the XIAO (PGN 141)
    uint32_t logSent;         // v7.2: [status] lines sent to the XIAO (PGN 142)
    uint32_t infoRx;          // v7.2: XIAO info frames received (PGN 143)
};

extern XiaoStats xiao;               // defined in zXiao.ino
extern bool      xiaoDetected;       // "ESP32-hello" in the last XIAO_HELLO_TIMEOUT_MS
extern uint32_t  xiaoLastHelloMs;
extern char      xiaoInfo[];         // v7.2: the XIAO's info JSON (program version, WiFi), "" = none yet
extern uint32_t  xiaoInfoMs;         // when it arrived (millis, 0 = never)

#endif
