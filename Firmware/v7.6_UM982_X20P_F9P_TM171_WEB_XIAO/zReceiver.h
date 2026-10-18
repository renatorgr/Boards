/* zReceiver.h - shared names for the GPS receiver identification and the u-blox settings (zReceiver.ino).
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr (v7.6).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zReceiver.h
// Included at the top of the main .ino, so every .ino file (and the automatic function
// prototypes) can use these names.
#ifndef ZRECEIVER_H
#define ZRECEIVER_H

#include <stdint.h>
#include "zWebJson.h"
#include "zRtcm.h"

// The receiver on Serial7, as it answered (zReceiver.ino)
enum RxKind : uint8_t
{
    RX_UNKNOWN = 0,
    RX_UM982,       // Unicore (UM982, UM980, ...): answered VERSIONA, or sends HPR
    RX_UBLOX        // u-blox (ZED-F9P, ZED-X20P, ...): answered UBX-MON-VER
};

// Identification state
enum RxIdState : uint8_t
{
    RXID_WAIT = 0,  // waiting for the receiver to talk
    RXID_ASK,       // asking (VERSIONA / UBX-MON-VER)
    RXID_DONE,      // answered
    RXID_NONE       // no answer to either question
};

extern uint8_t  rxKind;
extern uint8_t  rxIdState;
extern char     rxModel[24];          // "UM982", "ZED-F9P", "ZED-X20P" ("" = not known)
extern char     rxFw[40];             // its firmware version
extern char     rxProt[16];           // u-blox protocol version ("" for others)
extern uint32_t rxIdQuietUntil;       // millis: the console ignores the answer to the automatic VERSIONA until then

// RTCM counters of the corrections forwarded to the receiver (zStatus.ino): a question to the
// receiver is sent between two RTCM messages, never inside one
extern RtcmCounter ntripRtcm, radioRtcm;

bool        ubxFeed(uint8_t b);                 // every byte from the receiver: true = part of a UBX frame (not NMEA)
void        rxIdLine(const char* line);         // every text line from the receiver ("#VERSIONA...")
void        receiverLoop();                     // from loop()
void        receiverIdentifyAgain();
const char* rxKindName();                       // "u-blox", "Unicore", ""
const char* rxShortName(const char* fallback);  // the model, or fallback if not known
void        receiverJson(Json& j);              // GET /api/gps
// POST /api/gps (zWeb.ino reads the form): each returns nullptr = OK, else the error
uint8_t     ubxItemCount();
const char* ubxItemName(uint8_t i);
const char* ubxStartRead();
void        ubxWriteBegin();
const char* ubxWriteAdd(uint8_t i, long v);
const char* ubxWriteGo();
const char* ubxOutputOn();

#endif
