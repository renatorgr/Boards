/* zImu.h - shared names for the IMU detection and the BNO085 (zImu.ino).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zImu.h
// Included at the top of the main .ino, so every .ino file (and the automatic function
// prototypes) can use these names.
#ifndef ZIMU_H
#define ZIMU_H

#include <stdint.h>

// The IMU found at start-up (or later on Serial5). All of them fill the same values
// (TM171.ino: RollV, PitchV, YawV, TM171lastData, TM171DataSeen), so the rest of the firmware
// works the same with any of them.
enum ImuType : uint8_t
{
    IMU_NONE = 0,
    IMU_TM171,      // TM171 on Serial5 (115200)
    IMU_BNO_RVC,    // BNO085 in RVC mode on Serial5 (115200, 100 Hz)
    IMU_BNO_I2C     // BNO085 on I2C (Wire, pins 18 / 19, address 0x4A or 0x4B)
};

extern uint8_t imuType;
extern uint8_t imuBnoAddr;

void        imuSetup();
void        imuProcess();
const char* imuName();      // "TM171", "BNO085", "IMU" (none found)
const char* imuWhere();     // "Serial5", "Serial5 (RVC)", "I2C 0x4A", ""
uint32_t    imuErrors();    // bad packets (TM171 CRC, BNO085 RVC checksum) or BNO085 restarts (I2C)

#endif
