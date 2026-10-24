/* zDualF9P.ino - the second F9P of a dual F9P (moving base), on the GPS2 port (Serial2).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   The RELPOSNED decoding (heading + 90 deg, baseline, roll from the down component, the flags that
   must be set) is that of the AgOpenGPS AiO v4 Teensy firmware (zRelPos.ino of
   AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025, AgOpenGPS project, GPL v3); the UBX-NAV-RELPOSNED
   message is described in the u-blox ZED-F9P interface description.
   Written 2026 by Claude (Anthropic) for renatorgr (v4.5_AiO_WEB). The receiver check (zRxCheck.ino)
   gets the other UBX messages of the second F9P (its answers) and its counts.
   2026-10-23 (Claude for renatorgr): NMEA sentences from the second F9P count as valid data too, so a
   fresh F9P (NMEA out of the box, no RELPOSNED yet) is recognised, checked and can take the AgOpenGPS
   settings; a GNSS receiver on the GPS2 port while the port is set to the XIAO or off is noticed
   (valid NMEA or UBX on it) and the Status page says to change the setting.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zDualF9P.ino
// The GPS2 port (Teensy Serial2, pins 7 / 8) has one use at a time, chosen on the web page
// (Settings, cfg.xiaoOff): the XIAO ESP32 (zXiao.ino), nothing, or the second F9P of a dual F9P
// (here), both at 460800 baud.
//
// Dual F9P, as in the AgOpenGPS AiO v4 firmware: the main F9P (Serial7) sends GGA + VTG; it is the
// moving base and sends its RTCM corrections straight to the second F9P (F9P to F9P on the board).
// The second F9P sends UBX-NAV-RELPOSNED on its UART1 to the Teensy, 460800 baud, every epoch.
// Each RELPOSNED becomes this epoch's dual-antenna heading exactly as a UM982 HPR would
// (hprTake() in zHandlers.ino), so everything after it is the same for both receivers:
//   heading = relPosHeading + DUALF9P_HEADING_OFFSET (90 deg: antennas left and right)
//   roll    = asin(relPosD / relPosLength), sign as in the AiO firmware (used when there is no IMU)
//   QF      = 4 with an RTK fixed carrier solution, 5 float, 0 otherwise; gnssFixOK, diffSoln and
//             relPosValid must be set (else QF 0). A float heading is never used with the F9P
//             (as in the AiO firmware), whatever the "float heading" setting says.
//   time    = iTOW (GPS time of week) - DUALF9P_LEAP_S, as hhmmss.ss, matched to the GGA time like
//             the HPR's.

#define DUALF9P_BAUD            460800
#define DUALF9P_HEADING_OFFSET  90.0      // degrees added to relPosHeading (AiO firmware: 900 x 0.1 deg)
#define DUALF9P_LEAP_S          18        // GPS - UTC, seconds (u-blox default; the GGA time is UTC)
#define DUALF9P_MAX_LEN         1200      // longest UBX frame kept (RELPOSNED: 64 bytes; the receiver check's answers: up to 772)


// What the second F9P sent last (web page, [status] lines)
uint32_t dualF9pFrames   = 0;             // RELPOSNED frames with a good checksum
uint32_t dualF9pBad      = 0;             // UBX frames with a bad checksum
uint32_t dualF9pMs       = 0;             // last RELPOSNED (millis, 0 = never)
float    dualF9pHeading  = 0;             // degrees, offset included
float    dualF9pBaseline = 0;             // cm
float    dualF9pRoll     = 0;             // degrees
uint8_t  dualF9pCarr     = 0;             // carrSoln: 0 none, 1 float, 2 fixed
uint16_t dualF9pFlags    = 0;
uint32_t dualF9pBytes    = 0;             // bytes from the second F9P
uint32_t dualF9pNmea     = 0;             // NMEA sentences with a valid checksum (a fresh F9P sends NMEA, not RELPOSNED)
uint32_t dualF9pNmeaMs   = 0;             // last one (millis, 0 = never)
static uint8_t  dualBaud = 0;             // 0 not started, 1 XIAO speed, 2 F9P speed

static uint8_t  dSt = 0, dCls = 0, dId = 0, dCkA = 0, dCkB = 0;
static uint16_t dLen = 0, dPos = 0;
static uint8_t  dBuf[DUALF9P_MAX_LEN];
static char     dLine[100];               // an NMEA sentence being collected (outside the UBX frames)
static uint8_t  dLineLen = 0;

// The GPS2 port set to the XIAO or off, but a GNSS receiver sends on it (zXiao.ino hands over every byte):
// whole NMEA sentences or UBX frames with a valid checksum. The Status page then says to set the port to
// the second F9P. The XIAO's own frames (0x80 0x81 ...) and text lines never pass this test.
#define GPS2_SNIFF_N      3               // valid sentences / frames before the receiver counts as seen
#define GPS2_SNIFF_MS     15000           // ... and for how long after the last one
uint32_t gps2SniffGood   = 0;
uint32_t gps2SniffMs     = 0;
static uint8_t  sSt = 0, sCkA = 0, sCkB = 0;
static uint16_t sLen = 0, sPos = 0;
static uint32_t dLastMs = 0;

bool dualF9pOn() { return cfg.xiaoOff == GPS2_F9P; }

// Short: it goes into the [boot] and [web] settings lines, which are at most LOG_LINE_LEN - 1 characters
const char* gps2Name()
{
    return cfg.xiaoOff == GPS2_F9P ? "F9P" : cfg.xiaoOff == GPS2_OFF ? "off" : "XIAO";
}

// Serial2 at the speed of what is on the GPS2 port (at start-up, and when the setting changes; 460800
// for both, it restarts the port anyway)
void gps2Begin()
{
    const uint8_t want = dualF9pOn() ? 2 : 1;
    if (want == dualBaud) return;
    Serial2.begin(want == 2 ? DUALF9P_BAUD : XIAO_BAUD);         // (its buffers: xiaoSetup, zXiao.ino)
    dualBaud = want;
    dSt = 0;                       // (what is on the port: the [boot] and [web] settings lines)
    dLineLen = 0;
    sSt = 0;
    gps2SniffGood = 0;
    gps2SniffMs = 0;
}

// One byte of an NMEA sentence: true when a whole sentence with a valid checksum has just ended.
// Shared by the second F9P's reader (between its UBX frames) and the GPS2 sniffer: never both at once.
static bool dualNmeaByte(uint8_t b)
{
    if (b == '$') { dLineLen = 0; dLine[dLineLen++] = '$'; return false; }
    if (!dLineLen) return false;
    if (b == '\n')
    {
        dLine[dLineLen] = '\0';
        dLineLen = 0;
        return nmeaLineOk(dLine);                        // (zRxCheck.ino)
    }
    if (b == '\r') return false;
    if (b < 0x20 || b > 0x7E || dLineLen >= sizeof(dLine) - 1) { dLineLen = 0; return false; }
    dLine[dLineLen++] = (char)b;
    return false;
}

static void gps2SniffHit()
{
    gps2SniffGood++;
    gps2SniffMs = millis() | 1;
    if (gps2SniffGood == GPS2_SNIFF_N)
        StatusLog.line("[status] GPS2 port: a GNSS receiver is sending valid data on it, but the port is set to %s - "
                       "for a dual F9P set Settings, GPS2 port, to the second F9P", gps2Name());
}

// Called by zXiao.ino for every byte from Serial2 while the port is not set to the second F9P
void gps2SniffByte(uint8_t b)
{
    if (dualNmeaByte(b)) gps2SniffHit();
    switch (sSt)                                         // a UBX frame, checked without keeping it
    {
        case 0: sSt = (b == 0xB5) ? 1 : 0; break;
        case 1: sSt = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;
        case 2: sCkA = sCkB = 0; sCkA += b; sCkB += sCkA; sSt = 3; break;
        case 3: sCkA += b; sCkB += sCkA; sSt = 4; break;
        case 4: sLen = b; sCkA += b; sCkB += sCkA; sSt = 5; break;
        case 5: sLen |= (uint16_t)b << 8; sCkA += b; sCkB += sCkA; sPos = 0; sSt = (sLen > DUALF9P_MAX_LEN) ? 0 : (sLen ? 6 : 7); break;
        case 6: sCkA += b; sCkB += sCkA; if (++sPos >= sLen) sSt = 7; break;
        case 7: sSt = (b == sCkA) ? 8 : 0; break;
        case 8: if (b == sCkB) gps2SniffHit(); sSt = 0; break;
        default: sSt = 0; break;
    }
}

// A GNSS receiver seen on the GPS2 port while the port is set to the XIAO or off (web page, Status)
bool gps2ReceiverSeen()
{
    return !dualF9pOn() && gps2SniffGood >= GPS2_SNIFF_N && gps2SniffMs && millis() - gps2SniffMs < GPS2_SNIFF_MS;
}

// After the receiver check has searched for or changed the second F9P's baud rate (zRxCheck.ino): Serial2 at
// 460800 again, the frame reader from the start
void dualF9pRestart()
{
    Serial2.begin(DUALF9P_BAUD);
    dualBaud = 2;
    dSt = 0;
    dLineLen = 0;
}

static int32_t le32(const uint8_t* p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }

// UBX-NAV-RELPOSNED (version 1, 64 bytes): iTOW @4, relPosD @16 (cm), relPosLength @20 (cm),
// relPosHeading @24 (1e-5 deg), relPosHPD @34 and relPosHPLength @35 (0.1 mm), flags @60
static void relPosNed(const uint8_t* p, uint16_t len)
{
    if (len < 64 || p[0] != 1) return;
    const uint32_t iTow = (uint32_t)le32(p + 4);
    // heading in 1e-4 deg, 0 .. 359.9999 (whole numbers: no "360.0000" from rounding)
    int64_t h4 = llround((le32(p + 24) + DUALF9P_HEADING_OFFSET * 1e5) / 10.0);
    h4 %= 3600000;
    if (h4 < 0) h4 += 3600000;
    const double heading = h4 * 1e-4;
    const double baseline = le32(p + 20) + (int8_t)p[35] * 0.01;     // cm
    const double down     = le32(p + 16) + (int8_t)p[34] * 0.01;     // cm
    const uint32_t flags  = (uint32_t)le32(p + 60);
    const bool fixOk      = flags & 1, diffSoln = flags & 2, relPosValid = flags & 4;
    const uint8_t carr    = (flags >> 3) & 3;

    dualF9pFrames++;
    dualF9pMs       = millis() | 1;
    dualF9pFlags    = (uint16_t)flags;
    dualF9pCarr     = carr;
    dualF9pBaseline = (float)baseline;
    dualF9pHeading  = (float)heading;
    if (baseline > 0.5 && fabs(down) <= baseline) dualF9pRoll = (float)(asin(down / baseline) * -RAD_TO_DEG);

    int qf = 0;
    if (fixOk && diffSoln && relPosValid) qf = carr == 2 ? 4 : carr == 1 ? 5 : 0;

    // UTC time of the epoch in the form of the GGA time, hhmmss.ss x 100 (utcCs() in zHandlers.ino)
    const uint32_t ms = (iTow % 86400000UL + 86400000UL - DUALF9P_LEAP_S * 1000UL) % 86400000UL;
    const uint32_t c  = ms / 10UL;                                     // centiseconds of the UTC day
    const int32_t  cs = (int32_t)((c / 360000UL) * 1000000UL + ((c / 6000UL) % 60UL) * 10000UL + ((c / 100UL) % 60UL) * 100UL + c % 100UL);

    char h[16];
    snprintf(h, sizeof(h), "%ld.%04ld", (long)(h4 / 10000), (long)(h4 % 10000));
    char raw[100];
    snprintf(raw, sizeof(raw), "RELPOSNED iTOW %lu, heading %.2f, baseline %.1f cm, down %.1f cm, carrSoln %u, flags 0x%03lX",
             (unsigned long)iTow, heading, baseline, down, carr, (unsigned long)(flags & 0x3FF));
    hprRawSet(raw);
    hprTake('F', h, cs, qf, dualF9pRoll);
}

static inline void dCk(uint8_t b) { dCkA += b; dCkB += dCkA; }

// Called from loop(): the bytes of the second F9P (only with the GPS2 port set to it)
void dualF9pProcess()
{
    gps2Begin();
    if (!dualF9pOn() || rxCkOwns(RXCK_GPS2)) return;     // (the receiver check is searching for its baud rate)
    const uint32_t now = millis();
    if (dSt && now - dLastMs > 100) dSt = 0;               // a frame cut off: forget it
    int budget = 512;
    while (budget-- > 0 && Serial2.available() > 0)
    {
        const uint8_t b = (uint8_t)Serial2.read();
        dualF9pBytes++;
        rxCkBytes[RXCK_GPS2]++;
        dLastMs = now;
        // NMEA between the UBX frames: a fresh F9P, not set up as the second F9P yet, sends GGA and the rest
        // instead of the RELPOSNED. Valid sentences count as valid data, so the receiver check can identify
        // it, check it and load the AgOpenGPS settings (zRxCheck.ino refuses all that on "nothing valid").
        if (dSt == 0 && dualNmeaByte(b)) { rxCkGood[RXCK_GPS2]++; dualF9pNmea++; dualF9pNmeaMs = now | 1; }
        switch (dSt)
        {
            case 0: if (b == 0xB5) { dSt = 1; dLineLen = 0; } break;
            case 1: dSt = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;
            case 2: dCls = b; dCkA = dCkB = 0; dCk(b); dSt = 3; break;
            case 3: dId = b; dCk(b); dSt = 4; break;
            case 4: dLen = b; dCk(b); dSt = 5; break;
            case 5:
                dLen |= (uint16_t)b << 8;
                dCk(b);
                dPos = 0;
                dSt = (dLen > DUALF9P_MAX_LEN) ? 0 : (dLen ? 6 : 7);
                break;
            case 6:
                dBuf[dPos++] = b;
                dCk(b);
                if (dPos >= dLen) dSt = 7;
                break;
            case 7: dSt = (b == dCkA) ? 8 : 9; break;
            case 8:
                dSt = 0;
                if (b == dCkB)
                {
                    rxCkGood[RXCK_GPS2]++;
                    if (dCls == 0x01 && dId == 0x3C) relPosNed(dBuf, dLen);
                    else rxCkUbx(RXCK_GPS2, dCls, dId, dBuf, dLen);    // answers to the receiver check (zRxCheck.ino)
                }
                else dualF9pBad++;
                break;
            default: dSt = 0; dualF9pBad++; break;
        }
    }
}

// Web page: Status, dual card (zWeb.ino)
void dualF9pJson(Json& j)
{
    const uint32_t now = millis();
    j.obj("f9p2");
    j.bo("on", dualF9pOn());
    j.i("ago", dualF9pMs ? (long)((now - dualF9pMs) / 1000) : -1);
    j.u("frames", dualF9pFrames);
    j.u("bad", dualF9pBad);
    j.u("bytes", dualF9pBytes);
    j.u("nmea", dualF9pNmea);                              // NMEA sentences: the F9P is not set up as the second F9P
    j.i("nmeaAgo", dualF9pNmeaMs ? (long)((now - dualF9pNmeaMs) / 1000) : -1);
    j.str("port", gps2Name());                             // "F9P", "XIAO", "off" (the setting)
    j.bo("seen", gps2ReceiverSeen());                      // a receiver on the port while it is set to the XIAO or off
    j.num("heading", dualF9pHeading, 2);
    j.num("baseline", dualF9pBaseline, 1);
    j.num("roll", dualF9pRoll, 2);
    j.i("carr", dualF9pCarr);
    j.i("flags", dualF9pFlags);
    j.close();
}
