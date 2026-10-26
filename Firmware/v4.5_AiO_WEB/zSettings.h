/* zSettings.h - the settings block saved in the EEPROM.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   26 Oct 2026: the GPS2 port is automatic or off; "second F9P" saved by an earlier build loads as automatic.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zSettings.h
// The runtime settings block (see zSettings.ino). Included at the top of the main .ino so every
// .ino file can read "cfg".
#ifndef ZSETTINGS_H
#define ZSETTINGS_H

#include <stdint.h>

#define SETTINGS_EEPROM_ADDR  200
#define SETTINGS_MAGIC        0x5A17
#define SETTINGS_VERSION      2         // 2: steering speeds, soft start, current kickout delay (a version 1 block is
                                        //    taken over with their defaults, settingsLoad)

// The GPS2 port (cfg.xiaoOff)
#define GPS2_AUTO 0     // automatic: the XIAO's or the second F9P's, found while running (zDualF9P.ino); "XIAO" before 26 Oct 2026
#define GPS2_OFF  1
#define GPS2_F9P  2     // "second F9P", chosen by hand before 26 Oct 2026: loaded as automatic (settingsLoad)

enum OutputMode : uint8_t
{
    OUT_STANDARD = 0,   // $GNGGA $GNVTG $GNTHS $INHPR per epoch (AgOpenWeb with PR #300)
    OUT_CLASSIC  = 1,   // one sentence per epoch (AgOpenGPS / AgIO, AgOpenWeb): UM982 $PAOGI, F9P / X20P $PANDA
    OUT_AUTO     = 2    // default: standard while AgOpenWeb sends the hello, classic otherwise (zStatus.ino)
};

// the program that talks to the module, told apart by its hello (PGN 200), see zStatus.ino
enum HostProgram : uint8_t
{
    HOST_NONE      = 0, // no hello yet
    HOST_AGOPENWEB = 1,
    HOST_AGOPENGPS = 2, // AgIO
    HOST_OTHER     = 3  // a hello that is neither
};
bool    outClassicNow();  // zStatus.ino: classic sentences now? (the setting, or in automatic mode the program)
uint8_t hostProgram();    // zStatus.ino: HostProgram of the last hellos

struct WebSettings
{
    uint16_t magic;
    uint8_t  version;
    uint8_t  outputMode;          // OutputMode
    uint8_t  classicNoDual;       // not used any more (classic: UM982 always $PAOGI, F9P / X20P $PANDA); kept for the block layout
    uint8_t  hprAcceptFloat;      // 1 = dual heading also with HPR QF 5 (float)
    uint8_t  epochWaitMs;         // max wait after a GGA for its VTG / HPR (20..100 ms)
    uint8_t  tm171SwapRollPitch;  // 1 = TM171 rotated 90 deg on the board
    int8_t   tm171YawSign;        // 0 = learn from the dual heading, +1 / -1 = forced
    uint8_t  xiaoOff;             // the GPS2 port (Serial2): GPS2_AUTO (0, default) = the XIAO's or the second F9P's,
                                  //     whichever is found (zDualF9P.ino, zXiao.ino); GPS2_OFF (1) = ignored.
                                  //     GPS2_F9P (2), saved by builds before 26 Oct 2026, is loaded as automatic
    uint8_t  autoSet;             // 1 = the automatic output mode was offered once (an older block: 0, see
                                  //       settingsLoad); was reserved[0] = 0
    uint8_t  wasCheckOff;         // 1 = WAS voltage check off (bench without a WAS). 0 = on (default; was
                                  //       reserved = 0, so settings saved by older versions get the check on), zSafety.ino
    uint8_t  wdtOff;              // 1 = hardware watchdog off (from the next start). 0 = on (default), zSafety.ino
    uint8_t  umSingle;            // 1 = UM982 with one antenna (no dual-antenna heading on the page; the receiver
                                  //       check loads ConfigUM982S.txt). 0 = two antennas (default; was reserved = 0)
    uint8_t  simMode;             // 1 = simulator mode, for the program's simulator only: GPS data is not sent to the
                                  //       program (AgOpenWeb stops its simulator while it gets GPS sentences, with or
                                  //       without a fix), zHandlers.ino. 0 = off (default; was reserved = 0)
    // AiO_WEB version 2 (from the AiO 26 firmware's ideas), Autosteer.ino / AutosteerPID.ino:
    uint8_t  steerMinKmhX10;      // below this speed (x 0.1 km/h, the program's speed) the wheels are not turned (default 2 = 0.2)
    uint8_t  steerMaxKmh;         // above this speed (km/h) autosteer is cut (default 25)
    uint8_t  softStartX10ms;      // soft start of the valve / motor (x 10 ms; 0 = off, the default): the output rises over
                                  //       this time when autosteer engages, and over half of it on a hard step or a reversal
    uint8_t  currentKickX10ms;    // current sensor kickout only after the current is high this long (x 10 ms; 0 = at once;
                                  //       default 50 = 0.5 s), so a short peak does not switch autosteer off
    uint8_t  reserved[1];         // room for later settings
    uint8_t  checksum;
};

extern WebSettings cfg;          // defined in zSettings.ino
extern bool        cfgFromEeprom; // true: cfg came from the EEPROM (else defaults)
extern bool        settingsAutoSetNow; // this start switched older settings to the automatic mode
void settingsSave();             // zSettings.ino

#endif
