/* zSafety.h - shared names for the autosteer safety code (zSafety.ino).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zSafety.h
// Included at the top of the main .ino, so every .ino file (and the automatic function
// prototypes) can use these names.
#ifndef ZSAFETY_H
#define ZSAFETY_H

#include <stdint.h>

// Why autosteer went off (zSafety.ino). The first ones come from the module itself: they go to
// the program's screen (PGN 221) and to the [status] messages. The last ones come from the
// program, which knows them already: they are only shown on the web page.
enum SteerOff : uint8_t
{
    SO_NONE = 0,
    SO_SWITCH,      // steer switch turned off
    SO_BUTTON,      // steer button pressed
    SO_PRESSURE,    // pressure sensor kickout
    SO_CURRENT,     // current sensor kickout (motor current, Keya or analog)
    SO_ENCODER,     // steering wheel encoder kickout
    SO_WAS,         // WAS voltage out of range
    SO_SILENT,      // no steering data (PGN 254) from the program for 2.5 s
    SO_PROGRAM,     // the program turned it off (its button, end of the line, ...)
    SO_SPEED,       // the program sends a speed below 0.1 km/h
    SO_OTHER,
    SO_KEYA,        // AiO_WEB: Keya motor slip kickout (the driver holds the wheel)
    SO_KEYA_FAULT,  // AiO_WEB: the Keya reports a fault
    SO_ANGLE,       // AiO_WEB: wheel angle beyond the WAS map (+-50 deg)
    SO_FAST,        // AiO_WEB: the program's speed above the maximum (web page, default 25 km/h)
    SO_KEYA_LOST    // AiO_WEB: no Keya heartbeat for 0.5 s (KeyaCANBUS.ino)
};
#define SO_FROM_MODULE(c) (((c) >= SO_SWITCH && (c) <= SO_SILENT) || (c) >= SO_KEYA)

// zSafety.ino, used by the other files (declared here because some are one-line functions and
// the main .ino comes first)
extern volatile uint8_t loopStep;     // which part of loop() runs (for the watchdog report)
void  wdtFeed();
void  loopTick();                     // top of loop(): how long the loops take (the report, web page Firmware tab)
extern uint32_t loopCount, loopMaxUs, loopMax5Us;
void  wdtStart();
// C linkage declared here, before the prototypes the Arduino IDE adds. The IDE writes its own
// prototype of wdtWarnC without extern "C" (it only sees extern "C" { } blocks), and a C++ declaration
// before the extern "C" definition stopped an earlier build in the IDE ("conflicting declaration of
// 'void wdtWarnC(const uint32_t*)' with 'C' linkage"). A C declaration first, then the IDE's, is allowed.
extern "C" void wdtWarnC(const uint32_t* frame);
extern "C" void faultHookC(const uint32_t* frame);    // the same for the crash hook
void  faultHookInstall();
void  wdtLongTimeout();
void  bootCapture();
void  bootReport();
bool  wasCheck(int16_t raw);
bool  wasBenchZero();
bool  wasVoltsOk();
bool  keyaCheck();                    // KeyaCANBUS.ino
extern bool keyaBlock;                 // KeyaCANBUS.ino: autosteer blocked by a lost Keya heartbeat
extern bool keyaLost;                  // KeyaCANBUS.ino: no Keya heartbeat now (zWeb.ino)
void  steerKick(uint8_t cause, int value);
void  steerNote254();
void  steerNote252();
void  steerNote251();
void  steerOnNote(bool on);
float pressureFiltered(float sample);
void  sensorViewNote(float v);
float sensorPeakNow();
void  safetyNoteHello();
void  hwMessage(const char* text, uint8_t seconds, bool warning);
const char* offName(uint8_t c);
const char* bootName(uint8_t w);
const char* stepName(uint8_t s);

// Why the Teensy started this time (zSafety.ino, from the i.MX RT reset status)
enum BootWhy : uint8_t
{
    BOOT_UNKNOWN = 0,
    BOOT_POWER,     // power on, or a reset from the Teensy's own chip (button, USB upload)
    BOOT_UPLOAD,    // JTAG reset (programming)
    BOOT_RESTART,   // software restart: web page, firmware update, the program's subnet change
    BOOT_WATCHDOG,  // the hardware watchdog: the firmware got stuck
    BOOT_CRASH,     // the firmware crashed (fault), the core restarted it
    BOOT_CRASH_WDT, // the firmware crashed and the watchdog restarted it
    BOOT_TEMP,      // the chip's temperature monitor
    BOOT_OTHER
};

#endif
