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

// AiO_WEB: TM171 gyro calibration from the web page (TM171.ino)
enum TmCalib : uint8_t
{
    TMC_IDLE = 0,
    TMC_STARTING,   // start command sent, waiting for the TM171's "started" report
    TMC_RUNNING,    // the TM171 collects data (TM171_CALIB_S seconds)
    TMC_FINISHING,  // finish command sent, waiting for the result
    TMC_OK,         // succeeded (kept until the TM171 loses power)
    TMC_FAILED,     // the TM171 rejected the data (it moved, too much vibration)
    TMC_NO_ANSWER,  // nothing came back (the TX5 wire?)
    TMC_ABORTED     // stopped: web page, autosteer on, the tractor moving
};
enum TmCalibKind : uint8_t { TMC_KIND_GYRO = 0, TMC_KIND_INSTALL = 1 };   // RAM, or saved in the TM171's flash
extern uint8_t  tmCalibState, tmCalibKind;
extern uint32_t tmCalibDoneMs, tmCalibCount;
bool        tm171CalibStart(uint8_t kind, const char** why);
void        tm171CalibStop(const char* why);
void        tm171CalibLoop();
void        tm171CalibReport(uint8_t type, uint8_t ctrl);
int         tm171CalibLeft();
const char* tmCalibName(uint8_t s);

// AiO_WEB: the TM171's settings from the web page (TM171.ino)
struct TmSetForm
{
    bool     mag, gyroErr, selfAdapt, status, save;
    uint8_t  boot;            // 0 auto, 1 static, 2 dynamic
    uint16_t accGain, magGain; // x 0.01
    uint16_t inhibit;         // ms between packets
    uint32_t silent;          // s after power-on
};
extern uint32_t TM171rpyRate;
void tm171SettingRead();
bool tm171SettingWrite(const TmSetForm& f, const char** why);
void tm171SettingLoop();
void tm171SettingJson(Json& j);

void        imuSetup();
void        imuProcess();
const char* imuName();      // "TM171", "BNO085", "IMU" (none found)
const char* imuWhere();     // "Serial5", "Serial5 (RVC)", "I2C 0x4A", ""
uint32_t    imuErrors();    // bad packets (TM171 CRC, BNO085 RVC checksum) or BNO085 restarts (I2C)

#endif
