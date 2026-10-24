/* zRxCheck.h - shared names for the receiver check (zRxCheck.ino).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zRxCheck.h
// Included by zReceiver.h, so the main .ino and every other file know these names (the Arduino IDE
// adds its function prototypes at the top of the sketch, before any type defined in an .ino file).
#ifndef ZRXCHECK_H
#define ZRXCHECK_H

#include <stdint.h>
#include "zWebJson.h"

#ifndef FLASHMEM
#define FLASHMEM                     // (Teensy: code run from flash; nothing elsewhere)
#endif

#define RXCK_GPS    0                // Serial7: the GPS receiver
#define RXCK_GPS2   1                // Serial2: the second F9P of a dual F9P (GPS2 port set to it)
#define RX_TX_MAX   704              // largest message to a receiver (an AgOpenGPS UBX-CFG-VALSET: 664 bytes)

extern uint32_t rxGpsBaud;           // Serial7's baud rate now (zReceiver.ino; 460800, or the receiver's own while it is being set up)
extern uint32_t rxCkBytes[2];        // bytes from each port (counted by their readers)
extern uint32_t rxCkGood[2];         // whole sentences / UBX frames with a valid checksum from each port

bool rxCkOwns(uint8_t port);         // a baud rate search or change has the port: nobody else reads or writes it
bool rxCkBusy(uint8_t port);         // a check, search or load is running on the port (the GPS tab's reads and writes wait)
bool rxCkUbx(uint8_t port, uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len);   // a UBX frame from the port: true = taken
void rxCkNoteType(const char* name); // a sentence type from the GPS receiver (for the GGA / VTG / HPR rates)
bool nmeaLineOk(const char* l);      // a whole line with a valid checksum ("$...*hh" or Unicore "#...*xxxxxxxx")
void rxCkUmBegin();                  // zReceiver.ino starts reading the UM982
void rxCkUmConfig(const char* line); // one "$CONFIG,..." line of that read
const char* rxCkHealthName(uint8_t port);   // "ok", "junk", "none", "wait"
const char* rxCkStart(const char* what, uint8_t port, uint8_t um);   // the page's buttons: nullptr = started, else the error
void rxCkJson(Json& j);              // GET /api/gps: "ck"
void rxCkStatusJson(Json& j, uint8_t port);   // GET /api/status: a short summary of a port for the Status tab ("ck2")
bool rxCkDumpReady();                // "Read all settings" done: GET /api/rxdump has a file
size_t rxCkDumpLine(uint16_t i, char* out, size_t cap);   // line i of that file (u-center form), 0 = no more lines
void rxCheckLoop();                  // from loop()

// from zDualF9P.ino
extern uint32_t dualF9pFrames;
extern uint32_t dualF9pNmea;         // NMEA sentences with a valid checksum from the second F9P (not set up yet: it sends NMEA, no RELPOSNED)
bool dualF9pOn();                    // the GPS2 port is set to the second F9P
void dualF9pRestart();               // Serial2 back at 460800 after a baud rate search / change, the frame reader reset

#endif
