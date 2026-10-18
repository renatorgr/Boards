/* zSettings.h - the settings block saved in the EEPROM.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 Micro board
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
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
#define SETTINGS_VERSION      1

enum OutputMode : uint8_t
{
    OUT_STANDARD = 0,   // $GNGGA $GNVTG $GNTHS $INHPR per epoch (AgOpenWeb with PR #300)
    OUT_CLASSIC  = 1,   // one sentence per epoch (AgOpenGPS / AgIO, AgOpenWeb): UM982 $PAOGI, F9P / X20P $PANDA
    OUT_AUTO     = 2    // v7.4 (default): standard while AgOpenWeb sends the hello, classic otherwise (zStatus.ino)
};

// v7.4: the program that talks to the module, told apart by its hello (PGN 200), see zStatus.ino
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
    uint8_t  classicNoDual;       // v7.3: not used any more (classic: UM982 always $PAOGI, F9P / X20P $PANDA); kept for the block layout
    uint8_t  hprAcceptFloat;      // 1 = dual heading also with HPR QF 5 (float)
    uint8_t  epochWaitMs;         // max wait after a GGA for its VTG / HPR (20..100 ms)
    uint8_t  tm171SwapRollPitch;  // 1 = TM171 rotated 90 deg on the board
    int8_t   tm171YawSign;        // 0 = learn against the dual heading, +1 / -1 = forced
    uint8_t  xiaoOff;             // v7: 1 = ignore the XIAO ESP32C3 on Serial2 (zXiao.ino). 0 = bridge on (default;
                                  //     was a reserved byte = 0 in v6, so v6 settings stay valid)
    uint8_t  autoSet;             // v7.4: 1 = the automatic output mode was offered once (an older block: 0, see
                                  //       settingsLoad); was reserved[0] = 0
    uint8_t  wasCheckOff;         // v7.5: 1 = WAS voltage check off (bench without a WAS). 0 = on (default; was
                                  //       reserved = 0, so settings saved by older versions get the check on), zSafety.ino
    uint8_t  wdtOff;              // v7.5: 1 = hardware watchdog off (from the next start). 0 = on (default), zSafety.ino
    uint8_t  reserved[3];         // room for later settings (keep the block size)
    uint8_t  checksum;
};

extern WebSettings cfg;          // defined in zSettings.ino
extern bool        cfgFromEeprom; // true: cfg came from the EEPROM (else defaults)
extern bool        settingsAutoSetNow; // v7.4: this start switched older settings to the automatic mode
void settingsSave();             // zSettings.ino

#endif
