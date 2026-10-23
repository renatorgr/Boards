/* zImu.ino - which IMU is there (TM171 or BNO085), and the BNO085.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr. The BNO085 is read as in the AgOpenGPS
   AiO v4 Teensy firmware (AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025, AgOpenGPS project):
   first in RVC mode on Serial5 (BNO_RVC.cpp, frame format of the Adafruit BNO08x RVC library), then
   on I2C with the BNO08x_AOG library (SparkFun BNO080 library), with the same roll / pitch / heading
   conventions.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zImu.ino
// At start-up (imuSetup), in this order:
//   1. Serial5 at 115200 for IMU_DETECT_MS: a TM171 (packets AA 55 ..., CRC16) or a BNO085 in RVC
//      mode (frames AA AA ..., 19 bytes, 100 a second). The TM171 sits where the BNO085 module goes
//      on the AiO v4.5 boards, on the same Serial5 (jumper SJ1 joins the BNO085's SDA / TX to RX5).
//   2. If neither: a BNO085 on I2C (Wire, pins 18 / 19), address 0x4A then 0x4B.
// Whatever it finds fills the same values (roll, pitch, yaw), so the heading alignment, the yaw
// direction learning and the output (zHandlers.ino) work the same with any of them.
// With none found, the firmware keeps listening on Serial5: a TM171 / BNO085 that starts later
// is picked up then.
// BNO085 conventions (as in the AiO firmware): RVC: roll = its pitch, pitch = its roll; I2C (game
// rotation vector): roll about X, pitch about Y; the yaw turns clockwise, like a heading. "Use Y
// axis" in the program and "Swap roll and pitch" on the web page swap roll and pitch as with the
// TM171. The yaw direction is learned (or set on the web page) as with the TM171.

#include "zImu.h"
#include "BNO08x_AOG.h"

#define IMU_DETECT_MS      1500    // listening on Serial5 at start-up
#define RVC_FRAMES_NEEDED  3       // BNO085 RVC frames with a good checksum before it counts as found
#define BNO_READ_MS        10      // I2C: polled this often (it reports every BNO_REPORT_MS)
#define BNO_REPORT_MS      20
#define BNO_RESTART_MS     3000    // I2C BNO085 silent this long, autosteer off: started again
#define BNO_RESTART_EVERY  10000   // ...at most this often

uint8_t imuType    = IMU_NONE;
uint8_t imuBnoAddr = 0;

static uint32_t rvcGood = 0, rvcBad = 0;
static uint8_t  rvcBuf[19];
static uint8_t  rvcPos = 0;
static BNO080   bno08x;
static uint32_t bnoReadMs = 0, bnoRestartMs = 0, bnoRestarts = 0;

const char* imuName()
{
    return imuType == IMU_TM171 ? "TM171" : (imuType == IMU_BNO_RVC || imuType == IMU_BNO_I2C) ? "BNO085" : "IMU";
}

const char* imuWhere()
{
    static char w[12];
    switch (imuType)
    {
        case IMU_TM171:   return "Serial5";
        case IMU_BNO_RVC: return "Serial5 (RVC)";
        case IMU_BNO_I2C: snprintf(w, sizeof(w), "I2C 0x%02X", imuBnoAddr); return w;
        default:          return "";
    }
}

uint32_t imuErrors()
{
    return imuType == IMU_BNO_RVC ? rvcBad : imuType == IMU_BNO_I2C ? bnoRestarts : TM171crcErrors;
}

// ---------------------------------------------------------------- BNO085, RVC mode (Serial5)
// AA AA, index, yaw, pitch, roll (int16, 0.01 deg), 3 accelerations, 3 reserved bytes, checksum
// (sum of the 16 bytes after AA AA). use = put the values in (only once it is the IMU found).
static bool rvcByte(uint8_t b, bool use)
{
    if (rvcPos == 0) { if (b == 0xAA) rvcBuf[rvcPos++] = b; return false; }
    if (rvcPos == 1) { if (b == 0xAA) rvcBuf[rvcPos++] = b; else rvcPos = 0; return false; }
    rvcBuf[rvcPos++] = b;
    if (rvcPos < sizeof(rvcBuf)) return false;
    rvcPos = 0;
    uint8_t sum = 0;
    for (uint8_t k = 2; k < 18; k++) sum += rvcBuf[k];
    if (sum != rvcBuf[18]) { rvcBad++; return false; }
    rvcGood++;
    if (use)
    {
        const int16_t yaw   = (int16_t)(rvcBuf[3] | (rvcBuf[4] << 8));
        const int16_t pitch = (int16_t)(rvcBuf[5] | (rvcBuf[6] << 8));
        const int16_t roll  = (int16_t)(rvcBuf[7] | (rvcBuf[8] << 8));
        YawV.fValue   = yaw * 0.01f;
        RollV.fValue  = pitch * 0.01f;        // the AiO firmware: roll = the RVC pitch (X axis mounting)
        PitchV.fValue = roll * 0.01f;
        TM171lastData = 0;
        TM171DataSeen = true;
    }
    return true;
}

// ---------------------------------------------------------------- BNO085 on I2C
static bool bnoStart(uint8_t addr)
{
    // begin() resets the BNO085 and asks for its product ID once. A BNO085 that kept running while the
    // Teensy restarted (a software restart or an update, no power cut) can answer that first query
    // late or with another packet: up to 3 tries, 200 ms apart.
    bool ok = false;
    for (uint8_t t = 0; t < 3 && !ok; t++)
    {
        if (t) { wdtFeed(); delay(200); wdtFeed(); }
        ok = bno08x.begin(addr, Wire);
    }
    if (!ok) return false;
    Wire.setClock(400000);
    wdtFeed();
    delay(300);
    wdtFeed();
    bno08x.enableGameRotationVector(BNO_REPORT_MS);
    return true;
}

static bool bnoProbe()
{
    Wire.begin();
    static const uint8_t ADDR[] = {0x4A, 0x4B};
    for (uint8_t a : ADDR)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;          // nothing answers there
        if (bnoStart(a)) { imuType = IMU_BNO_I2C; imuBnoAddr = a; return true; }
        StatusLog.line("[boot] IMU: something answers on I2C 0x%02X (pins 18 / 19), but not as a BNO085", a);
    }
    return false;
}

// Game rotation vector (quaternion) -> roll, pitch, yaw in degrees, as in the AiO firmware
static void bnoRead()
{
    if (!bno08x.dataAvailable()) return;
    float qx, qy, qz, qw, acr;
    uint8_t ac;
    bno08x.getQuat(qx, qy, qz, qw, acr, ac);
    const float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 0.5f) return;                                 // not a rotation
    qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    const float ysqr = qy * qy;
    const float t3 = 2.0f * (qw * qz + qx * qy);
    const float t4 = 1.0f - 2.0f * (ysqr + qz * qz);
    float t2 = 2.0f * (qw * qy - qz * qx);
    if (t2 > 1.0f)  t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;
    const float t0 = 2.0f * (qw * qx + qy * qz);
    const float t1 = 1.0f - 2.0f * (qx * qx + ysqr);
    YawV.fValue   = -atan2f(t3, t4) * (float)RAD_TO_DEG;      // clockwise, like a heading
    RollV.fValue  = atan2f(t0, t1) * (float)RAD_TO_DEG;       // about X
    PitchV.fValue = asinf(t2) * (float)RAD_TO_DEG;            // about Y
    TM171lastData = 0;
    TM171DataSeen = true;
}

// A BNO085 that stops reporting (it reset itself, a loose wire) is started again, but only with
// autosteer off: starting it stops the loop for half a second (up to about a second and a half when it
// needs the three tries of bnoStart).
static void bnoWatch()
{
    const uint32_t now = millis();
    if (TM171lastData < BNO_RESTART_MS || watchdogTimer < WATCHDOG_THRESHOLD) return;
    if (bnoRestartMs && now - bnoRestartMs < BNO_RESTART_EVERY) return;
    bnoRestartMs = now | 1;
    bnoRestarts++;
    const bool ok = bnoStart(imuBnoAddr);
    StatusLog.line("[status] IMU: BNO085 (I2C 0x%02X) silent for %u s, started again: %s", imuBnoAddr, (unsigned)(BNO_RESTART_MS / 1000),
                   ok ? "it answers" : "NO ANSWER (wire? power?)");
}

// ---------------------------------------------------------------- detection
// Serial5 bytes to both parsers until one of them is sure
static void imuListen()
{
    while (SerialImu->available())
    {
        const uint8_t b = SerialImu->read();
        TM171byte(b);
        rvcByte(b, false);
        if (TM171goodPackets > 0)          { imuType = IMU_TM171;   break; }
        if (rvcGood >= RVC_FRAMES_NEEDED)  { imuType = IMU_BNO_RVC; break; }
    }
}

void imuSetup()
{
    SerialImu = &Serial5;
    TM171setup();                                           // 115200, larger buffers (the BNO085 RVC uses the same)
    while (SerialImu->available()) SerialImu->read();
    TM171DataSeen = false;
    TM171lastData = 0;
    const uint32_t t0 = millis();
    while (imuType == IMU_NONE && millis() - t0 < IMU_DETECT_MS)
    {
        imuListen();
        delay(1);
    }
    if (imuType == IMU_NONE) bnoProbe();
    switch (imuType)
    {
        case IMU_TM171:   StatusLog.println("[boot] IMU: TM171 on Serial5"); break;
        case IMU_BNO_RVC: StatusLog.println("[boot] IMU: BNO085 on Serial5 (RVC mode)"); break;
        case IMU_BNO_I2C: StatusLog.line("[boot] IMU: BNO085 on I2C 0x%02X (pins 18 / 19)", imuBnoAddr); bnoReadMs = millis(); break;
        default:          StatusLog.println("[boot] IMU NOT detected: no TM171 or BNO085 on Serial5, no BNO085 on I2C 0x4A / 0x4B - no roll, no IMU heading (still listening on Serial5)");
    }
}

// From loop()
void imuProcess()
{
    switch (imuType)
    {
        case IMU_TM171:
            TM171process();
            tm171CalibLoop();                               // the gyro calibration from the web page (TM171.ino)
            tm171SettingLoop();                             // the settings from the web page (TM171.ino)
            break;
        case IMU_BNO_RVC:
            while (SerialImu->available()) rvcByte(SerialImu->read(), true);
            break;
        case IMU_BNO_I2C:
            if (millis() - bnoReadMs >= BNO_READ_MS) { bnoReadMs = millis(); bnoRead(); }
            bnoWatch();
            break;
        default:                                            // none yet: one may start later on Serial5
            imuListen();
            if (imuType != IMU_NONE)
                StatusLog.line("[status] IMU: %s found on %s (it started after the Teensy)", imuName(), imuWhere());
            break;
    }
}
