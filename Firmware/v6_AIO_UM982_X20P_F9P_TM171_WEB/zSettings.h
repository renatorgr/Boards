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
    OUT_CLASSIC  = 1    // one $PAOGI (dual) or $PANDA (single) per epoch (AgOpenGPS / AgIO, AgOpenWeb)
};

struct WebSettings
{
    uint16_t magic;
    uint8_t  version;
    uint8_t  outputMode;          // OutputMode
    uint8_t  classicNoDual;       // classic, no dual heading: 0 = $PANDA with the TM171 heading, 1 = $PAOGI with the TM171 heading
    uint8_t  hprAcceptFloat;      // 1 = dual heading also with HPR QF 5 (float)
    uint8_t  epochWaitMs;         // max wait after a GGA for its VTG / HPR (20..100 ms)
    uint8_t  tm171SwapRollPitch;  // 1 = TM171 rotated 90 deg on the board
    int8_t   tm171YawSign;        // 0 = learn against the dual heading, +1 / -1 = forced
    uint8_t  reserved[7];         // room for later settings (keep the block size)
    uint8_t  checksum;
};

extern WebSettings cfg;          // defined in zSettings.ino
extern bool        cfgFromEeprom; // true: cfg came from the EEPROM (else defaults)

#endif
