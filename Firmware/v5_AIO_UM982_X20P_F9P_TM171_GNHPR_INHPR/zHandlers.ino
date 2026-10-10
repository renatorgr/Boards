// zHandlers.ino
// GNSS receiver + TM171 for AgOpenWeb - standard NMEA sentence set (v4, GNHPR_INHPR build).
//
// Receivers (Serial7, 10 Hz), detected automatically:
//   UM982 dual antenna:          GGA, VTG, HPR  -> dual heading + TM171
//   u-blox F9P / X20P (single):  GGA, VTG       -> TM171 heading (no HPR: the epoch is not held for it)
// The TM171 (Serial5, 25 Hz) gives roll, pitch, yaw.
//
// Out to AgOpenWeb (UDP 9999), per epoch, one line per datagram, in this order:
//   $GNGGA  the receiver's GGA (a UM982 "00 satellites / HDOP 9999.0" GGA is bridged)
//   $GNVTG  speed and track: the receiver's VTG, else measured from the GGA positions
//   $GNTHS  the UM982's dual-antenna heading (from its HPR): mode A = valid,
//           mode V = no dual heading (second antenna lost, no heading solution,
//           or a single-antenna receiver)
//   $INHPR  the TM171 ("IN" = inertial talker): heading aligned to the dual heading,
//           vehicle roll in the PITCH field (AgOpenGPS convention, as the UM982 HPR),
//           pitch in the roll field, QF 4 = TM171 OK, QF 0 = TM171 lost (no IMU)
//
// AgOpenWeb (a build with the "standard set" epoch assembler, PR #300) joins the four
// lines of an epoch into one fix:
//   heading: $GNTHS when it is valid (dual antenna); otherwise the TM171 heading from
//            $INHPR, which AgOpenWeb fuses with the fix-to-fix heading like $PANDA's IMU
//   roll:    always the TM171 ($GNTHS has no roll)
//   speed:   $GNVTG; position, fix, satellites, HDOP, correction age: $GNGGA
//
// Why the dual heading goes out as $GNTHS and not as $GNHPR: AgOpenWeb starts a new epoch
// when a sentence type repeats, so a $GNHPR and an $INHPR can never be in the same fix.
// The heading in $GNTHS is the UM982's HPR heading, character for character.

/************************* Tunables *************************/
#define EPOCH_WAIT_MS         60     // max wait after a GGA for the VTG and HPR of the same epoch
#define RECEIVER_SEEN_MS      3000   // VTG / HPR are waited for only if the receiver sent one this recently
                                     // (a single-antenna F9P / X20P never sends HPR: its epochs go out at once)
#define HPR_ACCEPT_FLOAT      1      // 1 = dual heading also when HPR QF = 5 (float), 0 = only QF 4 (fixed)
#define TM171_FRESH_MS        300    // TM171 older than this is considered lost (it sends at 25 Hz)

// TM171 mounting. Roll sign / zero: use AgOpenWeb's "Roll invert" and "Roll zero".
#define TM171_SWAP_ROLL_PITCH 0      // 1 = TM171 rotated 90 deg on the board (AOG "Use Y axis" also swaps at runtime)
#define TM171_YAW_SIGN        0      // 0 = auto-detect against the dual heading (default; +1 until learned, so with a
                                     //     single-antenna receiver it stays +1, as the official AiO firmware), +1 / -1 = force

// TM171 yaw alignment, learned while the dual heading is valid
#define OFFSET_K_DUAL         0.05f  // per epoch
#define OFFSET_SNAP_DEG       10.0f  // a TM171-vs-dual difference larger than this is snapped at once

#define GGA_HOLD_MS           2000   // UM982 sometimes sends a GGA with a valid position but "00" satellites / HDOP 9999.0: repeat the last good values for up to this long
#define GGA_SPEED_WINDOW      10     // epochs (1 s at 10 Hz) over which the speed fallback measures distance
#define GGA_SPEED_MIN_KNOTS   0.4f   // below this the position-derived speed is 0 (position noise at standstill)

#define FUSION_DEBUG          0      // 1 = [fusion] line once per second on USB serial (bench). 0 on the tractor.
#define RAW_NMEA_DEBUG        0      // 1 = print every GGA / VTG / HPR line as the receiver sends it, prefixed "[raw]"
#define SEND_TO_USB           0      // 1 = also print every line sent to AgOpenWeb on USB. 0 on the tractor.
/************************************************************/

const char* asciiHex = "0123456789ABCDEF";

// GGA fields (also read by zStatus.ino)
char fixTime[12];
char latitude[20];
char latNS[4];
char longitude[20];
char lonEW[4];
char fixQuality[4];
char numSats[6];
char HDOP[10];
char altitude[14];
char geoidSep[12];
char ageDGPS[10];
char stationId[8];

bool          ggaPending = false;    // a GGA arrived and its epoch has not been sent yet
uint32_t      ggaMs      = 0;
uint32_t      ggaFirstMs = 0;        // first GGA since start-up (VTG / HPR are expected for RECEIVER_SEEN_MS after it)
int32_t       ggaUtcCs   = -1;       // GGA time, centiseconds since midnight (-1 = none)

// GGA "00 satellites / HDOP 9999.0" bridging
char          goodSats[6]  = {};
char          goodHdop[10] = {};
uint32_t      goodGgaMs    = 0;
uint32_t      ggaHeldCount = 0;

// Speed / track from GGA positions (when the VTG has none)
double        posLatDeg[GGA_SPEED_WINDOW + 1], posLonDeg[GGA_SPEED_WINDOW + 1];
uint32_t      posMs[GGA_SPEED_WINDOW + 1];
uint8_t       posCount = 0;          // valid entries, newest at index 0

// VTG from the receiver
char          vtgTrack[12] = {};
char          vtgKmh[12]   = {};     // "" when the receiver has no speed (mode N)
uint32_t      vtgMs        = 0;
uint32_t      vtgRxCount   = 0;

// HPR from the UM982 (a single-antenna receiver sends none)
char          hprHeading[16] = {};
int32_t       hprUtcCs     = -1;
int           hprQf        = -1;
float         hprPitch     = 0;      // antenna-baseline pitch = vehicle roll (shown by FUSION_DEBUG only)
uint32_t      hprMs        = 0;
uint32_t      hprRxCount   = 0;
bool          hprPresent   = false;  // the receiver sends HPR (dual-antenna receiver)

// TM171 yaw alignment
float         yawOffset      = 0;    // aligned heading = yawSign * TM171 yaw + yawOffset
bool          offsetSeeded   = false;
float         yawSign        = (TM171_YAW_SIGN == 0) ? 1.0f : (float)TM171_YAW_SIGN;
bool          yawSignLocked  = (TM171_YAW_SIGN != 0);
bool          signHaveLast   = false;
float         signLastDual   = 0, signLastTm = 0;
float         signCorr       = 0, signMag = 0;

// Last epoch sent (diagnostics, also read by zStatus.ino)
bool          epochDual    = false;  // $GNTHS valid
bool          epochTmOk    = false;  // $INHPR valid
int           epochHprQf   = -1;     // QF of this epoch's HPR, -1 = no HPR for this epoch
float         dualHeading  = 0;
char          speedSource  = '-';    // 'v' receiver VTG, 'p' GGA positions, '-' none
float         lastSpeedKmh = 0, lastOut = 0, lastRoll = 0;

void errorHandler()
{
    // NMEA parse error (bad checksum, too long): the sentence is ignored
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

// "hhmmss.ss" -> centiseconds since midnight; -1 if empty or negative (UM982 prints
// "-00001.00" in HPR before it has a time)
static int32_t utcCs(const char* t)
{
    if (strlen(t) < 6 || t[0] == '-') return -1;
    return (int32_t)lround(atof(t) * 100.0);
}

static void updateOffset(float measured, float k)
{
    if (!offsetSeeded) { yawOffset = wrap180(measured); offsetSeeded = true; }
    else               { yawOffset = wrap180(yawOffset + k * wrap180(measured - yawOffset)); }
}

// Does the TM171 yaw turn the same way as the dual heading? Learned only from real
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

// ---------------------------------------------------------------- raw debug
// Every byte from the receiver goes through here before the parser (prints only with RAW_NMEA_DEBUG).
void rawNmeaTee(char c)
{
#if RAW_NMEA_DEBUG
    static char    rawLine[200];
    static uint8_t rawLen = 0;
    if (c == '$') rawLen = 0;
    if (c == '\n' || c == '\r')
    {
        if (rawLen > 6 && rawLine[0] == '$')
        {
            rawLine[rawLen] = '\0';
            if (!strncmp(rawLine + 3, "GGA", 3) || !strncmp(rawLine + 3, "VTG", 3) || !strncmp(rawLine + 3, "HPR", 3))
            {
                Serial.print("[raw] ");
                Serial.println(rawLine);
            }
        }
        rawLen = 0;
        return;
    }
    if (rawLen < sizeof(rawLine) - 1) rawLine[rawLen++] = c;
    else rawLen = 0;                                      // garbage / too long: drop it
#else
    (void)c;
#endif
}

// Called at the end of setup(): drop the epochs that piled up in the serial buffer
// while setup() was blocking, so the first sentences sent are not seconds old.
void gnssFlushAfterSetup()
{
    while (SerialGPS->available()) SerialGPS->read();
    ggaPending    = false;
    goodSats[0]   = '\0';
    goodHdop[0]   = '\0';
    posCount      = 0;
    vtgKmh[0]     = '\0';
    vtgTrack[0]   = '\0';
    vtgMs         = 0;
    hprUtcCs      = -1;
    hprHeading[0] = '\0';
    hprMs         = 0;
    hprPresent    = false;
    ggaFirstMs    = 0;
}

// ---------------------------------------------------------------- speed fallback
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

// Knots and track (deg; -1 when standing) from the distance travelled over the window.
// False when there are not enough positions.
static bool motionFromPositions(float& knots, float& trackDeg)
{
    if (posCount <= GGA_SPEED_WINDOW) return false;
    const uint32_t dtMs = posMs[0] - posMs[GGA_SPEED_WINDOW];
    if (dtMs < 500 || dtMs > 3000) return false;           // gap or burst: do not trust it
    const double lat0 = posLatDeg[0] * (M_PI / 180.0);
    const double dN   = (posLatDeg[0] - posLatDeg[GGA_SPEED_WINDOW]) * 111194.9;
    const double dE   = (posLonDeg[0] - posLonDeg[GGA_SPEED_WINDOW]) * 111194.9 * cos(lat0);
    knots    = (float)(sqrt(dN * dN + dE * dE) / (dtMs / 1000.0) * 1.943844);
    trackDeg = -1.0f;
    if (knots < GGA_SPEED_MIN_KNOTS) knots = 0.0f;
    else trackDeg = wrap360((float)(atan2(dE, dN) * 180.0 / M_PI));
    return true;
}

// ---------------------------------------------------------------- handlers
// GGA: 0 time, 1 lat, 2 N/S, 3 lon, 4 E/W, 5 fix, 6 sats, 7 HDOP, 8 alt, 9 M,
//      10 geoid separation, 11 M, 12 correction age, 13 station
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
    argCopy(10, geoidSep,   sizeof(geoidSep));
    argCopy(12, ageDGPS,    sizeof(ageDGPS));
    argCopy(13, stationId,  sizeof(stationId));

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
    blink        = !blink;
    gpsReadyTime = systick_millis_count;

    // Sent by gnssProcess() once this epoch's VTG and HPR are in (or EPOCH_WAIT_MS has passed)
    ggaPending = true;
    ggaMs      = millis();
    ggaUtcCs   = utcCs(fixTime);
    if (ggaFirstMs == 0) ggaFirstMs = ggaMs | 1;
}

// VTG: 0 track true, 1 T, 2 track magnetic, 3 M, 4 knots, 5 N, 6 km/h, 7 K, 8 mode
void VTG_Handler()
{
    char knots[12] = {}, mode[4] = {};
    argCopy(0, vtgTrack, sizeof(vtgTrack));
    argCopy(4, knots,    sizeof(knots));
    argCopy(6, vtgKmh,   sizeof(vtgKmh));
    argCopy(8, mode,     sizeof(mode));

    if (vtgKmh[0] == '\0' && knots[0] != '\0') dtostrf(atof(knots) * 1.852, 1, 3, vtgKmh);
    if (mode[0] == 'N') { vtgKmh[0] = '\0'; vtgTrack[0] = '\0'; }   // "not valid": no speed
    if (vtgKmh[0] != '\0' && (atof(vtgKmh) < 0.0f || atof(vtgKmh) > 200.0f)) vtgKmh[0] = '\0';

    vtgMs = millis();
    vtgRxCount++;
}

// HPR (Unicore): 0 time, 1 heading, 2 pitch, 3 roll, 4 QF (0 none, 4 fixed, 5 float), 5 sats, 6 age, 7 station
void HPR_Handler()
{
    char t[12] = {}, pitch[12] = {}, qf[4] = {};
    argCopy(0, t,          sizeof(t));
    argCopy(1, hprHeading, sizeof(hprHeading));
    argCopy(2, pitch,      sizeof(pitch));
    argCopy(4, qf,         sizeof(qf));

    hprUtcCs = utcCs(t);
    hprQf    = (qf[0] != '\0') ? atoi(qf) : 0;
    hprPitch = atof(pitch);
    hprMs    = millis();
    hprRxCount++;
}

// ---------------------------------------------------------------- output
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

// body = everything between '$' and '*'. Adds '$', "*hh" and CR LF; one datagram per line.
static void sendSentence(const char* body)
{
    static char line[200];
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
    snprintf(line, sizeof(line), "$%s*%c%c\r\n", body, asciiHex[cs >> 4], asciiHex[cs & 0x0F]);
    sendLine(line);
}

// Called from loop(). Sends the epoch once its VTG and HPR are in, or EPOCH_WAIT_MS after
// the GGA (the UM982 prints GGA, VTG, HPR; a missing one must not stop the epoch). A sentence
// the receiver does not send at all (HPR on an F9P / X20P) is not waited for.
void gnssProcess()
{
    if (!ggaPending) return;

    const uint32_t now = millis();
    // Does the receiver send VTG / HPR at all? (one seen in the last RECEIVER_SEEN_MS; both are
    // assumed for the first RECEIVER_SEEN_MS after start-up, so the UM982's first epochs wait too)
    const bool startUp    = (now - ggaFirstMs) < RECEIVER_SEEN_MS;
    const bool vtgPresent = startUp || (vtgMs != 0 && (now - vtgMs) < RECEIVER_SEEN_MS);
    hprPresent            = startUp || (hprMs != 0 && (now - hprMs) < RECEIVER_SEEN_MS);
    // VTG of this epoch: arrived after the GGA, or just before it (VTG has no time field)
    const bool vtgIn = vtgMs != 0 && (int32_t)(vtgMs - ggaMs) > -30;
    // HPR of this epoch: same time as the GGA
    const bool hprIn = hprUtcCs >= 0 && hprUtcCs == ggaUtcCs;
    const bool complete = (vtgIn || !vtgPresent) && (hprIn || !hprPresent);
    if (!complete && (now - ggaMs) < EPOCH_WAIT_MS) return;

    char body[180];

    // ---- $GNGGA
    snprintf(body, sizeof(body), "GNGGA,%s,%s,%s,%s,%s,%s,%s,%s,%s,M,%s,M,%s,%s",
             fixTime, latitude, latNS, longitude, lonEW, fixQuality, numSats, HDOP,
             altitude, geoidSep, ageDGPS, stationId);
    sendSentence(body);

    // ---- $GNVTG: receiver speed, else from the GGA positions
    char track[12] = {}, kmh[12] = {};
    speedSource = '-';
    if (vtgIn && vtgKmh[0] != '\0')
    {
        strcpy(kmh, vtgKmh);
        strcpy(track, vtgTrack);
        speedSource = 'v';
    }
    else
    {
        float kn, trk;
        if (motionFromPositions(kn, trk))
        {
            dtostrf(kn * 1.852f, 1, 3, kmh);
            if (trk >= 0.0f) dtostrf(trk, 1, 2, track);
            speedSource = 'p';
        }
    }
    if (kmh[0] != '\0')
    {
        lastSpeedKmh = atof(kmh);
        char kn[12];
        dtostrf(lastSpeedKmh / 1.852f, 1, 3, kn);
        snprintf(body, sizeof(body), "GNVTG,%s,T,,M,%s,N,%s,K,A", track, kn, kmh);
    }
    else
    {
        lastSpeedKmh = 0;
        strcpy(body, "GNVTG,,T,,M,,N,,K,N");
    }
    sendSentence(body);

    // ---- $GNTHS: dual-antenna heading
    epochHprQf = hprIn ? hprQf : -1;
    epochDual  = false;
    if (hprIn && hprHeading[0] != '\0' && (hprQf == 4 || (HPR_ACCEPT_FLOAT && hprQf == 5)))
    {
        const float h = atof(hprHeading);
        if (h >= 0.0f && h <= 360.0f)
        {
            dualHeading = (h >= 360.0f) ? 0.0f : h;
            epochDual   = true;
        }
    }
    if (epochDual) snprintf(body, sizeof(body), "GNTHS,%s,A", hprHeading);
    else           strcpy(body, "GNTHS,,V");
    sendSentence(body);

    // ---- $INHPR: TM171
    const float tmRaw = YawV.fValue;
    epochTmOk = TM171DataSeen && (TM171lastData < TM171_FRESH_MS);
    if (epochTmOk)
    {
        // Keep the TM171 yaw aligned to the dual heading for when the second antenna is lost
        if (epochDual)
        {
            learnYawSign(dualHeading, tmRaw);
            const float measured = dualHeading - yawSign * tmRaw;
            const bool  snap = offsetSeeded && fabsf(wrap180(measured - yawOffset)) > OFFSET_SNAP_DEG;
            updateOffset(measured, snap ? 1.0f : OFFSET_K_DUAL);
        }

        float r, p;
        const bool swapAxes = ((TM171_SWAP_ROLL_PITCH != 0) != (steerConfig.IsUseY_Axis != 0));
        if (swapAxes) { r = PitchV.fValue; p = RollV.fValue; }
        else          { r = RollV.fValue;  p = PitchV.fValue; }

        float out = wrap360(yawSign * tmRaw + (offsetSeeded ? yawOffset : 0.0f));
        if (out >= 359.995f) out = 0.0f;                   // never print 360.00
        lastOut  = out;
        lastRoll = r;

        // heading, PITCH field = vehicle roll (as the UM982 HPR), roll field = pitch, QF 4
        snprintf(body, sizeof(body), "INHPR,%s,%.2f,%.2f,%.2f,4,,,", fixTime, out, r, p);
    }
    else
    {
        lastOut = lastRoll = 0;
        snprintf(body, sizeof(body), "INHPR,%s,,,,0,,,", fixTime);
    }
    sendSentence(body);

    // LEDs: green = dual heading, red = TM171 heading (single antenna), red blinking = TM171 lost
    digitalWrite(GPSGREEN_LED, epochDual ? HIGH : LOW);
    if (!epochTmOk) digitalWrite(GPSRED_LED, blink ? HIGH : LOW);
    else            digitalWrite(GPSRED_LED, epochDual ? LOW : HIGH);

#if FUSION_DEBUG
    static uint32_t dbgMs = 0;
    if (millis() - dbgMs > 1000)
    {
        dbgMs = millis();
        Serial.printf("[fusion] %s hpr%s q=%d hdg='%s' antRoll=%.2f | TM171 %s yaw=%.2f sign=%+.0f%s off=%.2f%s -> hdg=%.2f roll=%.2f | spd=%.1fkm/h(%s) | ggaHeld=%lu tmCrcErr=%lu\r\n",
                      epochDual ? "DUAL" : "SINGLE", hprPresent ? "" : "(none: single-antenna receiver)", epochHprQf, hprIn ? hprHeading : "", hprPitch,
                      epochTmOk ? "ok" : "LOST", tmRaw, yawSign, yawSignLocked ? "(locked)" : "(learning)",
                      yawOffset, offsetSeeded ? "" : "(unseeded)", lastOut, lastRoll,
                      lastSpeedKmh, speedSource == 'v' ? "vtg" : speedSource == 'p' ? "positions" : "none",
                      (unsigned long)ggaHeldCount, (unsigned long)TM171crcErrors);
        const bool fix = fixQuality[0] != '0' && fixQuality[0] != '\0';
        if (fix && millis() > 20000)
        {
            if (vtgRxCount == 0) Serial.println("[fusion] NO VTG received from the receiver - add VTG at 10 Hz to its output");
        }
    }
#endif

    ggaPending = false;
}
