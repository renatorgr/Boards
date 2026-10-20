/* zReceiver.h - shared names for the GPS receiver identification and the u-blox and UM982 settings (zReceiver.ino).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
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
#include "zRxCheck.h"

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
bool        rxSingleAntenna();                  // F9P / X20P alone, UM980 / UM981, UM982 set to one antenna: no dual heading
void        receiverJson(Json& j);              // GET /api/gps
// POST /api/gps (zWeb.ino reads the form): each returns nullptr = OK, else the error
uint8_t     ubxItemCount();
const char* ubxItemName(uint8_t i);
const char* ubxStartRead();
void        ubxWriteBegin();
const char* ubxWriteAdd(uint8_t i, long v);
const char* ubxWriteGo();
const char* ubxOutputOn();
// the UM982 settings (same form: each returns nullptr = OK, else the error)
void        umLine(const char* line);           // every text line from the receiver
bool        umIsReply(const char* line);        // $command / $CONFIG / #UNILOGLIST / #MODE: answers, not sentence types
bool        umQuiet();                          // reading or writing the UM982 now: its answers stay out of the console
uint8_t     umItemCount();
const char* umItemName(uint8_t i);
const char* umStartRead();
void        umWriteBegin();
const char* umWriteAdd(uint8_t i, double v);
const char* umWriteGo();
// the AgOpenGPS commands of the receiver check (zRxCheck.ino): sent one at a time, SAVECONFIG, read again
const char* umWriteRaw(const char* const* cmds, uint8_t n, const char* what);

#endif
