/* zXiao.h - state of the XIAO ESP32 link.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
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

#define XIAO_BAUD               460800    // Serial2 with the XIAO, as v26 and the second F9P (the XIAO program: LINK_BAUD in xLink.h)

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
    uint32_t statusSent;      // module status JSONs sent to the XIAO (PGN 141)
    uint32_t logSent;         // [status] lines sent to the XIAO (PGN 142)
    uint32_t infoRx;          // XIAO info frames received (PGN 143)
    // the WiFi bridge (22 Nov 2026): a program on the XIAO's own WiFi
    uint32_t netIn;           // its packets taken (PGN 146)
    uint32_t netIgnored;      // its packets not taken: a program is talking on the network cable
    uint32_t netOut;          // datagrams for it handed to the XIAO (PGN 148)
    uint32_t netOutDrop;      // ... dropped: Serial2's transmit buffer was full, or over 255 bytes
    uint32_t rtcmIn;          // bytes of its corrections written to the receiver (PGN 147)
};

extern XiaoStats xiao;               // defined in zXiao.ino
extern bool      xiaoDetected;       // "ESP32-hello" in the last XIAO_HELLO_TIMEOUT_MS
extern uint32_t  xiaoLastHelloMs;
extern char      xiaoInfo[];         // the XIAO's info JSON (program version, WiFi), "" = none yet
extern uint32_t  xiaoInfoMs;         // when it arrived (millis, 0 = never)

// The WiFi bridge (zXiao.ino): a tablet with no network cable runs the program on the XIAO's own WiFi
#define BRIDGE_FREE   0              // no program on the cable, none on the XIAO's WiFi
#define BRIDGE_CABLE  1              // a program is talking on the network cable: it has the module
#define BRIDGE_WIFI   2              // the program is on the XIAO's WiFi: what goes to it goes through the XIAO
bool    xiaoBridgeWifi();            // BRIDGE_WIFI
uint8_t xiaoBridgeState();
void    xiaoBridgeNoteCable();       // a program packet came in on the cable (UDP 8888)
void    xiaoNetOut(const uint8_t* d, uint16_t n);   // one datagram for the program, through the XIAO

#endif
