// zHandlers.ino
// UM982 + TM171 for AgOpenWeb - DEDICATED build (v4).
//
// Data sources:
//   GGA  (Serial7) - position, fix quality, satellites, HDOP, altitude, correction age
//   KSXT (Serial7) - UM982 dual-antenna fix: heading + heading quality, speed (VTG and HPR are NOT used)
//   TM171 (Serial5) - roll, pitch, yaw (always running)
//
// One sentence per GGA epoch, chosen once per epoch. Both are sentences AgOpenWeb's
// NmeaParserServiceFast decodes as a whole fix (needs AgOpenWeb with $KSXT support, PR #288):
//   DUAL   KSXT heading quality >= KSXT_MIN_HDG_QUALITY (2 = RTK float, 3 = RTK fixed), fresh
//            -> the $KSXT line is forwarded to AgOpenWeb exactly as the UM982 sent it.
//               AgOpenWeb reads position, speed, heading and roll (antenna baseline) from it.
//               The TM171 is not in this sentence; its yaw is only compared with the KSXT
//               heading so that it is already aligned when the second antenna is lost.
//   SINGLE KSXT missing / heading quality too low / stale
//            -> $PANDA built from the GGA:
//               field 11 = speed (knots): KSXT speed of this epoch, else from GGA positions
//               field 12 = TM171 yaw + offset learned in dual, int (deg x 10); 65535 if no TM171
//               field 13 = TM171 roll, int (deg x 10)
//               field 14 = TM171 pitch, field 15 = 0 (AgOpenWeb ignores the yaw rate)
//               AgOpenWeb then does its own single-antenna fusion (fix-to-fix heading + IMU
//               with a learned offset, reverse detection) - the same code AgOpenGPS uses.

/************************* Tunables *************************/
#define KSXT_MIN_HDG_QUALITY  2      // KSXT heading quality accepted as dual: 2 = RTK float, 3 = RTK fixed
#define KSXT_FRESH_MS         400    // KSXT older than this is considered lost
#define KSXT_WAIT_MS          60     // max wait after a GGA for its KSXT (if the UM982 prints KSXT after GGA)
#define TM171_FRESH_MS        300    // TM171 older than this is considered lost (it sends at 25 Hz)

// TM171 mounting / sign conventions. AgOpenGPS: positive roll = leaning right.
#define TM171_SWAP_ROLL_PITCH 0      // 1 = TM171 rotated 90 deg on the board (AOG "Use Y axis" also swaps at runtime)
#define TM171_INVERT_ROLL     0      // 1 = flip roll sign
#define TM171_ROLL_OFFSET_DEG 0.0f   // added to roll (AgOpenWeb RollZero also works)
#define TM171_YAW_SIGN        0      // 0 = auto-detect against the KSXT heading (default), +1 / -1 = force

// TM171 yaw alignment, learned while the KSXT heading is valid
#define OFFSET_K_DUAL         0.05f  // per epoch
#define OFFSET_SNAP_DEG       10.0f  // a TM171-vs-KSXT difference larger than this is snapped at once

#define GGA_HOLD_MS           2000   // UM982 sometimes sends a GGA with a valid position but "00" satellites / HDOP 9999.0: repeat the last good values for up to this long
#define GGA_SPEED_WINDOW      10     // epochs (1 s at 10 Hz) over which the GGA speed fallback measures distance
#define GGA_SPEED_MIN_KNOTS   0.4f   // below this the GGA-derived speed is reported as 0 (position noise at standstill)

#define FUSION_DEBUG          0      // 1 = status line once per second on USB serial (bench). 0 on the tractor.
#define RAW_NMEA_DEBUG        0      // 1 = print every GGA / KSXT line as the UM982 sends it, prefixed "[raw]". 0 on the tractor.
#define SEND_TO_USB           0      // 1 = also print every sentence sent to AgOpenWeb ($KSXT / $PANDA) on USB. 0 on the tractor.
/************************************************************/

const char* asciiHex = "0123456789ABCDEF";

// Every field copy goes through argCopy(), which truncates, so a longer-than-expected
// field can never write past its buffer. ($PANDA worst case ~110 bytes.)
char nmea[200];

// GGA fields
char fixTime[12];
char latitude[16];
char latNS[4];
char longitude[16];
char lonEW[4];
char fixQuality[4];
char numSats[6];
char HDOP[10];
char altitude[14];
char ageDGPS[10];

// $PANDA fields
char speedKnots[10] = {};
char imuHeading[12];
char imuRoll[12];
char imuPitch[12];

// Epoch sync
bool          ggaPending = false;    // a GGA arrived and its sentence has not been sent yet
uint32_t      ggaMs      = 0;

// GGA "00 satellites / HDOP 9999.0" bridging
char          goodSats[6]  = {};
char          goodHdop[10] = {};
uint32_t      goodGgaMs    = 0;
uint32_t      ggaHeldCount = 0;

// Speed from GGA positions (when the KSXT has none)
double        posLatDeg[GGA_SPEED_WINDOW + 1], posLonDeg[GGA_SPEED_WINDOW + 1];
uint32_t      posMs[GGA_SPEED_WINDOW + 1];
uint8_t       posCount = 0;          // valid entries, newest at index 0

// KSXT
bool          ksxtReady   = false;   // a KSXT (valid or not) arrived since the last send
bool          ksxtValid   = false;   // last KSXT carried a usable dual heading
uint32_t      ksxtLastMs  = 0;
float         dualHeading = 0;       // deg 0-360
char          ksxtLine[200] = {};    // last valid KSXT line, verbatim ("$KSXT,...*hh"), forwarded in dual
char          ksxtSpeedKn[10] = {};  // KSXT speed (km/h) converted to knots, "" if absent
uint32_t      ksxtSpeedMs = 0;
uint32_t      ksxtRxCount = 0;       // diagnostics
int           ksxtLastQuality = -1;
char          ksxtLastHdg[16] = {};

// TM171 yaw alignment
float         yawOffset      = 0;    // aligned heading = yawSign * TM171 yaw + yawOffset
bool          offsetSeeded   = false;
float         yawSign        = (TM171_YAW_SIGN == 0) ? 1.0f : (float)TM171_YAW_SIGN;
bool          yawSignLocked  = (TM171_YAW_SIGN != 0);
bool          signHaveLast   = false;
float         signLastDual   = 0, signLastTm = 0;
float         signCorr       = 0, signMag = 0;

// Current epoch (diagnostics)
bool          sendKsxt       = false; // true: forward $KSXT, false: build $PANDA
uint8_t       headingSource  = 0;     // 1 KSXT, 2 TM171+offset, 3 TM171 unaligned, 4 none (65535)
char          speedSource    = '-';   // 'k' KSXT, 'g' GGA positions, '-' none
float         lastOut = 0, lastRoll = 0;

void errorHandler()
{
    // NMEA parse error - silent
}

// ---------------------------------------------------------------- helpers
// Safe replacement for parser.getArg(n, char*), which does an unbounded strcpy.
static void argCopy(uint8_t n, char* dst, size_t dstSize)
{
    char tmp[200];                      // larger than the parser's whole sentence buffer
    tmp[0] = '\0';
    if (!parser.getArg(n, tmp)) tmp[0] = '\0';
    strncpy(dst, tmp, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

static float wrap360(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a <    0.0f) a += 360.0f;
    return a;
}

static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void updateOffset(float measured, float k)
{
    if (!offsetSeeded) { yawOffset = wrap180(measured); offsetSeeded = true; }
    else               { yawOffset = wrap180(yawOffset + k * wrap180(measured - yawOffset)); }
}

// Does the TM171 yaw turn the same way as the KSXT heading? Learned only from real
// turns of the whole rig (>= 8 deg, TM171 following), then locked.
static void learnYawSign(float dual, float tmRaw)
{
    if (yawSignLocked) return;
    if (!signHaveLast) { signLastDual = dual; signLastTm = tmRaw; signHaveLast = true; return; }

    float dH = wrap180(dual - signLastDual);
    if (fabsf(dH) < 8.0f) return;
    float dT = wrap180(tmRaw - signLastTm);
    signLastDual = dual;
    signLastTm   = tmRaw;

    if (fabsf(dH) > 90.0f) return;                       // jump, not a turn
    if (fabsf(dT) < 0.3f * fabsf(dH)) return;            // TM171 did not follow: no evidence
    if (fabsf(dT) > 3.0f * fabsf(dH)) return;            // implausible ratio

    signCorr += (dH * dT > 0) ? fabsf(dH) : -fabsf(dH);
    signMag  += fabsf(dH);
    if (signMag > 60.0f)
    {
        if (fabsf(signCorr) > 0.7f * signMag)
        {
            if (signCorr < 0) { yawSign = -1.0f; offsetSeeded = false; }
            yawSignLocked = true;
        }
        else if (signMag > 360.0f) { signCorr = 0; signMag = 0; }
    }
}

// ---------------------------------------------------------------- line capture
// Every byte from the UM982 goes through here before the parser. The parser calls the
// handlers on the '\n' that ends a sentence, after this function has already seen the
// '\r', so the finished line is kept in lastLine: KSXT_Handler copies it for forwarding,
// RAW_NMEA_DEBUG prints it.
static char    rawLine[200];
static uint8_t rawLen = 0;
static char    lastLine[200];
static uint8_t lastLen = 0;

void rawNmeaTee(char c)
{
    if (c == '$') rawLen = 0;
    if (c == '\n' || c == '\r')
    {
        if (rawLen > 0)
        {
            memcpy(lastLine, rawLine, rawLen);
            lastLine[rawLen] = '\0';
            lastLen = rawLen;
#if RAW_NMEA_DEBUG
            if (lastLen > 6 && lastLine[0] == '$' && (!strncmp(lastLine + 3, "GGA", 3) || !strncmp(lastLine + 1, "KSXT", 4)))
            {
                Serial.print("[raw] ");
                Serial.println(lastLine);
            }
#endif
        }
        rawLen = 0;
        return;
    }
    if (rawLen < sizeof(rawLine) - 1) rawLine[rawLen++] = c;
    else rawLen = 0;                                      // garbage / too long: drop it
}

// Called at the end of setup(): drop the epochs that piled up in the serial buffer
// while setup() was blocking, so the first sentences sent are not seconds old.
void gnssFlushAfterSetup()
{
    while (SerialGPS->available()) SerialGPS->read();
    ggaPending   = false;
    ksxtReady    = false;
    ksxtValid    = false;
    ksxtSpeedKn[0] = '\0';
    goodSats[0]  = '\0';
    goodHdop[0]  = '\0';
    posCount     = 0;
    rawLen       = 0;
    lastLen      = 0;
}

// ---------------------------------------------------------------- GGA speed fallback
static double nmeaToDeg(const char* f, char hemi)
{
    double v = atof(f);                                    // DDMM.MMMMM
    double d = floor(v / 100.0);
    double deg = d + (v - d * 100.0) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -deg : deg;
}

static void pushPosition()
{
    for (int i = GGA_SPEED_WINDOW; i > 0; i--) { posLatDeg[i] = posLatDeg[i - 1]; posLonDeg[i] = posLonDeg[i - 1]; posMs[i] = posMs[i - 1]; }
    posLatDeg[0] = nmeaToDeg(latitude, latNS[0]);
    posLonDeg[0] = nmeaToDeg(longitude, lonEW[0]);
    posMs[0]     = millis();
    if (posCount <= GGA_SPEED_WINDOW) posCount++;
}

// Knots from the distance travelled over the window, or -1 if not available.
static float speedFromPositions()
{
    if (posCount <= GGA_SPEED_WINDOW) return -1.0f;
    const uint32_t dtMs = posMs[0] - posMs[GGA_SPEED_WINDOW];
    if (dtMs < 500 || dtMs > 3000) return -1.0f;           // gap or burst: do not trust it
    const double lat0 = posLatDeg[0] * (M_PI / 180.0);
    const double dN   = (posLatDeg[0] - posLatDeg[GGA_SPEED_WINDOW]) * 111194.9;
    const double dE   = (posLonDeg[0] - posLonDeg[GGA_SPEED_WINDOW]) * 111194.9 * cos(lat0);
    float kn = (float)(sqrt(dN * dN + dE * dE) / (dtMs / 1000.0) * 1.943844);
    if (kn < GGA_SPEED_MIN_KNOTS) kn = 0.0f;
    return kn;
}

// ---------------------------------------------------------------- handlers
void GGA_Handler()
{
    argCopy(0,  fixTime,    sizeof(fixTime));
    argCopy(1,  latitude,   sizeof(latitude));
    argCopy(2,  latNS,      sizeof(latNS));
    argCopy(3,  longitude,  sizeof(longitude));
    argCopy(4,  lonEW,      sizeof(lonEW));
    argCopy(5,  fixQuality, sizeof(fixQuality));
    argCopy(6,  numSats,    sizeof(numSats));
    argCopy(7,  HDOP,       sizeof(HDOP));
    argCopy(8,  altitude,   sizeof(altitude));
    argCopy(12, ageDGPS,    sizeof(ageDGPS));

    const bool hasFix = fixQuality[0] != '0' && fixQuality[0] != '\0' && latitude[0] != '\0' && longitude[0] != '\0';

    // The UM982 intermittently reports "00" satellites / HDOP "9999.0" with a valid fix and position.
    if (hasFix)
    {
        if (atoi(numSats) > 0 && atof(HDOP) < 99.0f)
        {
            strncpy(goodSats, numSats, sizeof(goodSats) - 1);  goodSats[sizeof(goodSats) - 1] = 0;
            strncpy(goodHdop, HDOP,    sizeof(goodHdop) - 1);  goodHdop[sizeof(goodHdop) - 1] = 0;
            goodGgaMs = millis();
        }
        else if (goodSats[0] != '\0' && (millis() - goodGgaMs) <= GGA_HOLD_MS)
        {
            strncpy(numSats, goodSats, sizeof(numSats) - 1);   numSats[sizeof(numSats) - 1] = 0;
            strncpy(HDOP,    goodHdop, sizeof(HDOP) - 1);      HDOP[sizeof(HDOP) - 1] = 0;
            ggaHeldCount++;
        }
        pushPosition();
    }
    else posCount = 0;

    digitalWrite(GGAReceivedLED, blink ? HIGH : LOW);
    blink         = !blink;
    GGA_Available = true;
    gpsReadyTime  = systick_millis_count;

    // Sent by gnssProcess() once this epoch's KSXT is in (or KSXT_WAIT_MS has passed).
    ggaPending = true;
    ggaMs      = millis();
}

// $KSXT (Unicore). getArg indices are 0-based after "KSXT" (Unicore manual field number - 2),
// the same fields AgOpenWeb's DecodeKsxt reads:
//   0 time  1 lon  2 lat  3 height  4 heading  5 pitch  6 track  7 speed (km/h)  8 roll
//   9 position quality  10 heading quality (0 none, 1 single, 2 RTK float, 3 RTK fixed)
// Only heading, heading quality and speed are read here; the line itself is forwarded.
void KSXT_Handler()
{
    char hdgStr[16] = {}, spdStr[16] = {}, qStr[6] = {};
    argCopy(4,  hdgStr, sizeof(hdgStr));
    argCopy(7,  spdStr, sizeof(spdStr));
    argCopy(10, qStr,   sizeof(qStr));
    const int quality = (qStr[0] != '\0') ? atoi(qStr) : 0;

    ksxtRxCount++;
    ksxtLastQuality = quality;
    strcpy(ksxtLastHdg, hdgStr);

    // The handler runs on the line's '\n', so lastLine holds "$KSXT,...*hh" (checksum already verified)
    const bool lineOk = (lastLen > 10 && !strncmp(lastLine, "$KSXT,", 6) && lastLine[lastLen - 3] == '*');

    ksxtValid = false;
    if (lineOk && hdgStr[0] != '\0' && quality >= KSXT_MIN_HDG_QUALITY)
    {
        const float h = atof(hdgStr);
        if (h >= 0.0f && h <= 360.0f)
        {
            dualHeading = (h >= 360.0f) ? 0.0f : h;
            strcpy(ksxtLine, lastLine);
            ksxtValid  = true;
            ksxtLastMs = millis();
        }
    }

    if (spdStr[0] != '\0')
    {
        const float kmh = atof(spdStr);
        if (kmh >= 0.0f && kmh < 200.0f)
        {
            dtostrf(kmh / 1.852f, 1, 3, ksxtSpeedKn);
            ksxtSpeedMs = millis();
        }
    }
    else ksxtSpeedKn[0] = '\0';

    ksxtReady = true;
}

// ---------------------------------------------------------------- per epoch
static void sendLine(const char* line)
{
#if SEND_TO_USB
    SerialAOG.write(line);
#endif
    if (Ethernet_running)
    {
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(line, strlen(line));
        Eth_udpPAOGI.endPacket();
    }
}

// Called from loop(). One sentence per GGA, as soon as the epoch's KSXT is in or
// KSXT_WAIT_MS has passed (the UM982 may print KSXT after GGA, or not at all).
void gnssProcess()
{
    if (!ggaPending) return;

    const uint32_t now = millis();
    if (!ksxtReady && (now - ggaMs) < KSXT_WAIT_MS) return;

    const bool  tmOk  = TM171DataSeen && (TM171lastData < TM171_FRESH_MS);
    const float tmRaw = YawV.fValue;

    sendKsxt = ksxtReady && ksxtValid && ((now - ksxtLastMs) < KSXT_FRESH_MS);

    if (sendKsxt)
    {
        // Keep the TM171 yaw aligned to the dual heading for the next $PANDA
        if (tmOk)
        {
            learnYawSign(dualHeading, tmRaw);
            const float measured = dualHeading - yawSign * tmRaw;
            const bool  snap = offsetSeeded && fabsf(wrap180(measured - yawOffset)) > OFFSET_SNAP_DEG;
            updateOffset(measured, snap ? 1.0f : OFFSET_K_DUAL);
        }
        headingSource = 1;
        lastOut = dualHeading;

        strcpy(nmea, ksxtLine);
        strcat(nmea, "\r\n");
        sendLine(nmea);
    }
    else
    {
        // Speed: KSXT of this epoch, else from the GGA positions, else empty
        speedSource = '-';
        speedKnots[0] = '\0';
        if (ksxtSpeedKn[0] != '\0' && (now - ksxtSpeedMs) < KSXT_FRESH_MS)
        {
            strcpy(speedKnots, ksxtSpeedKn);
            speedSource = 'k';
        }
        else
        {
            const float kn = speedFromPositions();
            if (kn >= 0.0f) { dtostrf(kn, 1, 2, speedKnots); speedSource = 'g'; }
        }

        // Roll / pitch from the TM171
        float r = 0, p = 0;
        if (tmOk)
        {
            const bool swapAxes = ((TM171_SWAP_ROLL_PITCH != 0) != (steerConfig.IsUseY_Axis != 0));
            if (swapAxes) { r = PitchV.fValue; p = RollV.fValue; }
            else          { r = RollV.fValue;  p = PitchV.fValue; }
#if TM171_INVERT_ROLL
            r = -r;
#endif
            r += TM171_ROLL_OFFSET_DEG;

            const float tmYaw = yawSign * tmRaw;
            const float out = wrap360(offsetSeeded ? tmYaw + yawOffset : tmYaw);
            headingSource = offsetSeeded ? 2 : 3;
            lastOut = out;

            // AgOpenWeb reads $PANDA heading and roll as (int)(deg * 10)
            int h10 = (int)lroundf(out * 10.0f);
            if (h10 >= 3600) h10 -= 3600;
            if (h10 < 0)     h10 += 3600;
            snprintf(imuHeading, sizeof(imuHeading), "%d", h10);
            snprintf(imuRoll,    sizeof(imuRoll),    "%d", (int)lroundf(r * 10.0f));
        }
        else
        {
            headingSource = 4;
            lastOut = 0;
            strcpy(imuHeading, "65535");                       // AgOpenWeb: no IMU
            strcpy(imuRoll,    "0");
        }
        dtostrf(p, 1, 2, imuPitch);
        lastRoll = r;

        BuildPanda();
        sendLine(nmea);
    }

    // LEDs: green = dual (KSXT), red = TM171 heading, red blinking = TM171 lost
    digitalWrite(GPSGREEN_LED, sendKsxt ? HIGH : LOW);
    if (!tmOk) digitalWrite(GPSRED_LED, blink ? HIGH : LOW);
    else       digitalWrite(GPSRED_LED, sendKsxt ? LOW : HIGH);

#if FUSION_DEBUG
    static uint32_t dbgMs = 0;
    if (millis() - dbgMs > 1000)
    {
        dbgMs = millis();
        if (sendKsxt)
            Serial.printf("[fusion] KSXT ksxtRx=%lu q=%d hdg=%.2f tm=%.2f sign=%+.0f%s off=%.2f%s tmCrcErr=%lu\r\n",
                          (unsigned long)ksxtRxCount, ksxtLastQuality, dualHeading, tmRaw,
                          yawSign, yawSignLocked ? "(locked)" : "(learning)",
                          yawOffset, offsetSeeded ? "" : "(unseeded)", (unsigned long)TM171crcErrors);
        else
            Serial.printf("[fusion] PANDA src=%u ksxtRx=%lu q=%d ksxtHdg='%s' tm=%.2f sign=%+.0f%s off=%.2f%s out=%.2f roll=%.2f spd=%.1fkm/h(%skn)%s tmCrcErr=%lu ggaHeld=%lu\r\n",
                          headingSource, (unsigned long)ksxtRxCount, ksxtLastQuality, ksxtLastHdg, tmRaw,
                          yawSign, yawSignLocked ? "(locked)" : "(learning)",
                          yawOffset, offsetSeeded ? "" : "(unseeded)", lastOut, lastRoll,
                          atof(speedKnots) * 1.852, speedKnots,
                          speedSource == 'k' ? "(ksxt)" : speedSource == 'g' ? "(gga)" : "(none)",
                          (unsigned long)TM171crcErrors, (unsigned long)ggaHeldCount);
        if (ksxtRxCount == 0 && fixQuality[0] != '0' && fixQuality[0] != '\0' && millis() > 20000)
            Serial.println("[fusion] NO KSXT received from the UM982 - check it outputs KSXT at 10 Hz on the Teensy port");
    }
#endif

    ggaPending = false;
    ksxtReady  = false;
}

// ---------------------------------------------------------------- $PANDA
void BuildPanda()
{
    strcpy(nmea, "$PANDA,");
    strcat(nmea, fixTime);    strcat(nmea, ",");
    strcat(nmea, latitude);   strcat(nmea, ",");
    strcat(nmea, latNS);      strcat(nmea, ",");
    strcat(nmea, longitude);  strcat(nmea, ",");
    strcat(nmea, lonEW);      strcat(nmea, ",");
    strcat(nmea, fixQuality); strcat(nmea, ",");
    strcat(nmea, numSats);    strcat(nmea, ",");
    strcat(nmea, HDOP);       strcat(nmea, ",");
    strcat(nmea, altitude);   strcat(nmea, ",");
    strcat(nmea, ageDGPS);    strcat(nmea, ",");
    strcat(nmea, speedKnots); strcat(nmea, ",");
    strcat(nmea, imuHeading); strcat(nmea, ",");
    strcat(nmea, imuRoll);    strcat(nmea, ",");
    strcat(nmea, imuPitch);   strcat(nmea, ",");
    strcat(nmea, "0");                                     // yaw rate (ignored by AgOpenWeb)
    strcat(nmea, "*");
    CalculateChecksum();
    strcat(nmea, "\r\n");
}

void CalculateChecksum()
{
    int16_t sum = 0;
    for (int16_t inx = 1; inx < (int16_t)sizeof(nmea); inx++)
    {
        char tmp = nmea[inx];
        if (tmp == '*') break;
        sum ^= tmp;
    }
    char hex[3] = {asciiHex[(sum >> 4) & 0x0F], asciiHex[sum & 0x0F], 0};
    strcat(nmea, hex);
}
