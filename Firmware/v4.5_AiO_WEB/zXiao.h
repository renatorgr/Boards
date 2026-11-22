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
    // the page through the XIAO (Nov 2026): a browser on the XIAO's WiFi
    uint32_t webReq;          // its requests (PGN 150 "open")
    uint32_t webBroken;       // ... broken off: a piece lost on the link, or an answer the XIAO stopped taking
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

// The module's page through the XIAO (zXiao.ino): a browser on the XIAO's own WiFi opens http://<the XIAO>:8080/,
// the XIAO passes its request on over the link (PGN 150) and the answer goes back the same way (PGN 151). webLoop()
// (zWeb.ino) serves it as one more connection, through xiaoWeb: one request at a time, as on the cable.
#define XWEB_RX_MAX   4608           // the request: WEB_HEAD_MAX (zWeb.ino) and a margin; more is dropped (413 there)
#define XWEB_TX_MAX   8192           // the answer, waiting for its turn on the link (as a socket's buffer, 8 kB)
class XiaoWebClient : public Client
{
public:
    int     connect(IPAddress, uint16_t) override { return 0; }
    int     connect(const char*, uint16_t) override { return 0; }
    size_t  write(uint8_t b) override { return write(&b, 1); }
    size_t  write(const uint8_t* buf, size_t size) override;
    int     availableForWrite() override;
    int     available() override;
    int     read() override;
    int     read(uint8_t* buf, size_t size) override;
    int     peek() override;
    void    flush() override {}
    void    stop() override;         // the answer is complete: the rest goes out, then "the end" (or dropped: the browser went)
    uint8_t connected() override;
    operator bool() override { return open; }

    bool     open = false;           // a session: from the XIAO's "open" until the end of the answer went out
    bool     waiting = false;        // opened, not taken by webLoop() yet
    bool     gone = false;           // the browser went (the XIAO said so), or the session broke
    bool     ending = false;         // stop(): the answer is complete
    uint8_t  sid = 0;                // the session's number (the XIAO's)
    uint16_t window = 0;             // answer bytes the XIAO holds for the browser (it says so at "open")
    uint16_t rxLen = 0, rxPos = 0;   // the request
    uint32_t rxGot = 0;              // request bytes the XIAO sent (each piece says where it starts)
    uint16_t txHead = 0, txLen = 0;  // the answer bytes not sent yet (a ring)
    uint32_t txSent = 0, txTaken = 0;    // answer bytes sent to the XIAO / passed on by it to the browser
    uint32_t moveMs = 0;             // the last time the answer moved (a stuck one is given up)
};
extern XiaoWebClient xiaoWeb;
void xiaoWebFlush();                 // before a restart: the last answer out to the XIAO (zXiao.ino)
void webXiaoDropped();               // the session webLoop() has on xiaoWeb is over (zWeb.ino)

#endif
