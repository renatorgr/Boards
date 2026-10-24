/* zHandlers.ino - receiver sentences, epochs, TM171 heading, output to AgOpenWeb / AgOpenGPS.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Started from the zHandlers.ino of chriskinal's AOG_Teensy_UM982 (github.com/chriskinal/AOG_Teensy_UM982,
   Copyright 2024 chriskinal, GPL v3),
   itself from the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project).
   Rewritten almost entirely 2026 by Claude (Anthropic) for renatorgr. "TM171" here means the IMU
   found (TM171 or BNO085, zImu.ino); the receiver's #VERSIONA answer and UBX message names go to
   zReceiver.ino. AiO_WEB 4.5: the dual heading also from a second F9P (hprTake(), zDualF9P.ino);
   with no IMU, the $PAOGI roll from the two antennas; the wheel angle from the heading's turn rate
   (for the Keya without a WAS), as in the AgOpenGPS AiO v4 firmware; the IMU aligned to the GPS course
   only once the rig has driven straight for 2.5 s / 5 m, and kept on the course afterwards only for the
   $PAOGI of a dual-antenna receiver (AgOpenWeb and AgOpenGPS with $PANDA align it themselves); the valid sentences and the
   GGA / VTG / HPR rates counted for the receiver check (zRxCheck.ino); the simulator mode setting
   (the GPS not sent to the program).
   Original notice (chriskinal), kept as it was:
     Copywrite 2024 chriskinal
     This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
     Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
     option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY
     WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
     GNU General Public License for more details. You should have received a copy of the GNU General Public
     License along with this program. If not, see <https://www.gnu.org/licenses/>.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zHandlers.ino
// GNSS receiver + TM171 for AgOpenWeb / AgOpenGPS (standard NMEA sentence set, or $PAOGI/$PANDA).
//
// Output mode (web page, zSettings.ino): "standard" (below, AgOpenWeb with PR #300) or "classic"
// (AgOpenGPS / AgIO, also read by AgOpenWeb), one sentence per epoch, chosen by the receiver:
//   UM982 (dual antenna, sends HPR): ALWAYS $PAOGI - AgOpenGPS heading source "Dual".
//     dual heading valid: heading = UM982 HPR, roll = TM171
//     dual heading lost:  heading = TM171, aligned to the dual heading while it was valid and then
//                         corrected by the GPS course when driving straight; roll = TM171
//     (AgOpenGPS in "Dual" ignores the heading of a $PANDA and would freeze, so no $PANDA here)
//   F9P / X20P (single antenna, no HPR): $PANDA with the TM171 heading and roll (x 10) -
//     AgOpenGPS heading source "Fix": AgOpenGPS does its own fix-to-fix + IMU fusion.
//
// Receivers (Serial7, 10 Hz), detected automatically:
//   UM982 dual antenna:          GGA, VTG, HPR  -> dual heading + TM171
//   dual F9P (AiO_WEB):          GGA, VTG + the second F9P's RELPOSNED (Serial2) -> dual heading + TM171,
//                                exactly like an HPR (hprTake(); heading + 90 deg, RTK fixed only)
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
//           vehicle roll in the PITCH field (AgOpenGPS convention, as in the UM982 HPR),
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
// The first four are DEFAULTS: the web page (zSettings.ino) can change them without reprogramming.
#define EPOCH_WAIT_MS         60     // max wait after a GGA for the VTG and HPR of the same epoch
#define RECEIVER_SEEN_MS      3000   // VTG / HPR are waited for only if the receiver sent one this recently
                                     // (a single-antenna F9P / X20P never sends HPR: its epochs go out at once)
#define HPR_ACCEPT_FLOAT      1      // 1 = dual heading also when HPR QF = 5 (float), 0 = only QF 4 (fixed)
#define TM171_FRESH_MS        300    // TM171 data older than this is considered lost (it sends at 25 Hz)

// TM171 mounting. Roll sign / zero: use AgOpenWeb's "Roll invert" and "Roll zero".
#define TM171_SWAP_ROLL_PITCH 0      // 1 = TM171 rotated 90 deg on the board (AOG "Use Y axis" also swaps at runtime)
#define TM171_YAW_SIGN        0      // 0 = auto-detect (default; +1 until learned) from turns, against the dual heading or,
                                     //     without one, against the GPS course; +1 / -1 = force

// TM171 yaw alignment, learned while the dual heading is valid
#define OFFSET_K_DUAL         0.05f  // per epoch
#define OFFSET_SNAP_DEG       10.0f  // a TM171-vs-dual difference larger than this is snapped at once
#define TM171_JUMP_DEG        25.0f  // TM171 yaw change between two epochs (100 ms) larger than this is a glitch, not a
                                     // turn: compensated in the offset so the heading stays continuous

// Dual heading in the classic output: leave at once, come back after DUAL_ENTER_EPOCHS good epochs
#define DUAL_ENTER_EPOCHS     5      // consecutive valid dual epochs before the dual heading is used again (0.5 s)
#define DUAL_BLEND_EPOCHS     10     // back to dual: the $PAOGI heading moves from TM171 to dual over this many epochs
#define DUAL_BLEND_MAX_DEG    5.0f   // ...only if they differ by less than this; more = the TM171 heading was off: switch at once

// TM171 heading corrected by the GPS course while there is no dual heading
#define COURSE_MIN_DIST_M     1.5f   // course measured between positions at least this far apart
#define COURSE_MIN_KMH        2.0f   // no correction below this speed
#define COURSE_MAX_TURN_DEG   2.0f   // TM171 yaw change allowed over the same stretch (= driving straight)
#define COURSE_K              0.02f  // per epoch (10 Hz: ~5 s time constant)
#define COURSE_MAX_STEP_DEG   0.2f   // max correction per epoch
#define COURSE_FWD_DEG        45.0f  // |course - heading| below this = driving forward
#define COURSE_REV_DEG        135.0f // |course - heading| above this = reversing (the course is 180 deg off the heading)
#define COURSE_RESEED_EPOCHS  30     // driving straight for 3 s with a course that fits neither: re-align to the course (forward assumed)
#define COURSE_SEED_KMH       3.0f   // TM171 not aligned since start-up: aligned to the course above this speed (forward assumed)...
#define COURSE_SEED_MS        2500   // ...without a break for this long...
#define COURSE_SEED_DIST_M    5.0f   // ...over at least this distance...
#define COURSE_SEED_SPREAD_DEG 10.0f // ...the course staying within this of where the run started. A receiver standing still
                                     // without RTK jumps 1-2 m now and then (one "course" above 3 km/h), but does not travel
                                     // 5 m in one direction for 2.5 s; a tractor does, with or without RTK
#define SIGN_LEARN_DUAL_DEG   60.0f  // turning needed to lock the TM171 yaw direction against the dual heading
#define SIGN_LEARN_COURSE_DEG 30.0f  // ...against the GPS course (no dual heading: learn it sooner)
#define POS_HISTORY           30     // GGA positions kept (3 s at 10 Hz) for the speed fallback and the course

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

// GGA positions, newest at index 0: speed / track fallback (when the VTG has none) and GPS course
double        posLatDeg[POS_HISTORY], posLonDeg[POS_HISTORY];
uint32_t      posMs[POS_HISTORY];
float         posTmYaw[POS_HISTORY];  // raw TM171 yaw at that epoch (for the "driving straight" check)
bool          posTmOk[POS_HISTORY];
uint8_t       posCount = 0;          // valid entries

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
char          dualSrc      = 0;      // AiO_WEB: where the dual heading comes from: 'U' UM982 HPR, 'F' second F9P (RELPOSNED, zDualF9P.ino)
float         epochDualRoll = 0;     // AiO_WEB: roll from the two antennas in this epoch (used when there is no IMU)

// TM171 yaw alignment
float         yawOffset      = 0;    // aligned heading = yawSign * TM171 yaw + yawOffset
bool          offsetSeeded   = false;
float         yawSign        = (TM171_YAW_SIGN == 0) ? 1.0f : (float)TM171_YAW_SIGN;
bool          yawSignLocked  = (TM171_YAW_SIGN != 0);
bool          signHaveLast   = false;
float         signLastDual   = 0, signLastTm = 0;
float         signCorr       = 0, signMag = 0;
char          signSrc        = '-';  // reference for the sign learning: 'D' dual heading, 'C' GPS course
char          offsetFrom     = '-';  // what aligned the TM171 heading: 'D' dual heading, 'C' GPS course
bool          tmPrevOk       = false;
float         tmPrevRaw      = 0;
uint32_t      tmPrevMs       = 0;
uint32_t      tmJumpCount    = 0;    // TM171 yaw glitches compensated
uint16_t      courseBadRun   = 0;
uint32_t      courseFixCount = 0, courseReseedCount = 0;
char          courseState    = '-';  // no dual: 'F' corrected driving forward, 'R' reversing, 'x' ignored, '-' no course
char          courseLastFix  = 'F';  // last forward / reverse decision
float         lastCourse     = -1;

// Dual heading with hysteresis (classic $PAOGI) and the fade back to it
uint8_t       dualGoodRun    = 0;
bool          dualActive     = false;
uint8_t       blendLeft      = 0;
float         blendErr       = 0;

// Last epoch sent (diagnostics, also read by zStatus.ino)
bool          epochDual    = false;  // $GNTHS valid
bool          epochTmOk    = false;  // $INHPR valid
int           epochHprQf   = -1;     // QF of this epoch's HPR, -1 = no HPR for this epoch
float         dualHeading  = 0;
char          speedSource  = '-';    // 'v' receiver VTG, 'p' GGA positions, '-' none
float         lastSpeedKmh = 0, lastOut = 0, lastRoll = 0;
float         stdShift     = 0;      // standard $INHPR heading = lastOut + stdShift (0 while the dual heading is in use;
                                     // without it, the course alignment taken back out: AgOpenWeb does its own)
float         classicHdg   = 0;      // heading of the last classic $PAOGI (UM982)
bool          haveClassicHdg = false;
bool          classicHdgTrusted = false; // classicHdg came from the dual heading, an aligned TM171 or the GPS course
uint8_t       classicSrc   = 0;      // 1 dual, 2 TM171 aligned, 3 GPS course (TM171 lost), 4 held

// The TM171 heading is used without the dual heading only when its turning direction is known (learned
// from turns, or set on the Settings page): with the wrong direction it would turn the wrong way.
static bool tmHeadingUsable() { return epochTmOk && yawSignLocked && offsetSeeded; }

// The receiver is a dual-antenna one (has sent HPR since start-up). Sticky: a UM982 that stops sending
// HPR keeps the $PAOGI output (AgOpenGPS stays on "Dual"); only a receiver that never sent HPR gets $PANDA.
static bool receiverIsDual() { return hprRxCount > 0; }

// Why an epoch went out without dual heading ($GNTHS V). zStatus.ino prints the reason.
enum NoDualReason : uint8_t {
    NODUAL_NONE = 0,       // dual heading OK
    NODUAL_NO_HPR,         // the receiver sends no HPR at all (F9P / X20P, or HPR off)
    NODUAL_NOT_IN_TIME,    // HPR is sent, but none with this GGA's time arrived within EPOCH_WAIT_MS
    NODUAL_QF,             // this epoch's HPR has QF 0 (or another value that is not 4 / 5)
    NODUAL_HEADING,        // QF 4 / 5 but the heading field is empty or out of range
    NODUAL_COUNT
};
uint8_t       epochNoDual  = NODUAL_NONE;
uint32_t      noDualCount[NODUAL_COUNT] = {};  // epochs per reason (zStatus.ino reads and clears it every 5 s)
char          hprRawLast[100] = {};  // the last HPR line exactly as the receiver sent it
uint32_t      hprRawMs     = 0;      // when it arrived (millis, 0 = never), valid or not
uint32_t      hprLateCount = 0;      // HPRs that arrived after their epoch had already been sent
uint32_t      hprLateMs    = 0;      // how long after its GGA the last late HPR arrived
uint32_t      hprLateAtMs  = 0;      // when the last late HPR arrived (millis, 0 = never)
uint32_t      nmeaCrcErrors = 0;     // receiver sentences dropped for a bad checksum (bytes lost / corrupted)
uint16_t      gnssRxPeak   = 0;      // largest backlog seen in the receiver serial buffer (zStatus.ino clears it)

void errorHandler()
{
    // NMEA parse error: the sentence is ignored. Bad checksums are counted (sign of lost bytes).
    if (parser.error() == NMEA::CRC_ERROR) nmeaCrcErrors++;
}

// Called from loop() before reading the receiver: how many bytes are waiting. A backlog of more
// than one epoch means the Teensy reads the receiver too slowly (zStatus.ino reports it).
void gnssNoteRxBacklog()
{
    const int n = SerialGPS->available();
    if (n > gnssRxPeak) gnssRxPeak = (uint16_t)n;
}

// ---------------------------------------------------------------- helpers
// Safe replacement for parser.getArg(n, char*), which does an unbounded strcpy.
static void argCopy(uint8_t n, char* dst, size_t dstSize)
{
    char tmp[200];                      // larger than the parser's whole sentence buffer
    tmp[0] = '\0';
    if (!parser.getArg(n, tmp)) tmp[0] = '\0';
    const size_t k = strnlen(tmp, dstSize - 1);       // memcpy, the same copy without a compiler warning
    memcpy(dst, tmp, k);
    dst[k] = '\0';
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

// "hhmmss.ss" -> the same number x 100 (only compared for equality); -1 if empty or negative
// (UM982 prints "-00001.00" in HPR before it has a time)
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

// Does the TM171 yaw turn the same way as the vehicle? Learned only from real turns of the whole
// rig (>= 8 deg, TM171 following), then locked. Reference: the dual heading, or without one the GPS
// course (its change has the same sign forward and in reverse). src: 'D' dual, 'C' course.
static void learnYawSign(float dual, float tmRaw, char src)
{
    if (yawSignLocked) return;
    if (src != signSrc) { signSrc = src; signHaveLast = false; }    // never compare across references
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
    if (signMag > (src == 'C' ? SIGN_LEARN_COURSE_DEG : SIGN_LEARN_DUAL_DEG))
    {
        if (fabsf(signCorr) > 0.7f * signMag)
        {
            if (signCorr < 0) { yawSign = -1.0f; offsetSeeded = false; }
            yawSignLocked = true;
        }
        else if (signMag > 360.0f) { signCorr = 0; signMag = 0; }
    }
}

// ---------------------------------------------------------------- raw lines
// Sentence types the receiver sends ("GNGGA", "GNHPR", "KSXT", "HEADINGA", ...), counted before
// the parser, whatever the parser does with them. zStatus.ino lists them every 5 s when the set changes.
#define RX_TYPES_MAX 10
struct RxType { char name[16]; uint16_t count; };    // 16 (UBX message names, zReceiver.ino)
RxType        rxTypes[RX_TYPES_MAX];
uint8_t       rxTypeCount  = 0;
uint16_t      rxTypeOthers = 0;      // lines of further types that did not fit in the table

static void noteRxTypeName(const char* name)
{
    rxCkNoteType(name);                                    // the GGA / VTG / HPR rates (zRxCheck.ino)
    for (uint8_t k = 0; k < rxTypeCount; k++)
        if (!strcmp(rxTypes[k].name, name)) { if (rxTypes[k].count < 65535) rxTypes[k].count++; return; }
    if (rxTypeCount < RX_TYPES_MAX)
    {
        strncpy(rxTypes[rxTypeCount].name, name, sizeof(rxTypes[0].name) - 1);
        rxTypes[rxTypeCount].name[sizeof(rxTypes[0].name) - 1] = '\0';
        rxTypes[rxTypeCount].count = 1;
        rxTypeCount++;
    }
    else if (rxTypeOthers < 65535) rxTypeOthers++;
}

static void noteRxType(const char* line)
{
    char name[12];
    uint8_t i = 0;
    for (const char* p = line + 1; *p && *p != ',' && *p != '*'; p++)
    {
        if (!isalnum((unsigned char)*p) || i >= sizeof(name) - 1) return;   // not a sentence name
        name[i++] = *p;
    }
    name[i] = '\0';
    if (i < 2) return;
    noteRxTypeName(name);
}

// ---- UM982 console (web page): replies to the commands sent from the page (command answers,
// CONFIG / UNILOGLIST lists, ...) - every printable line that is not the receiver's normal
// output, for RX_REPLY_WINDOW_MS after a command. "$command,..." answers are always kept.
#define RX_REPLY_LINES      40
#define RX_REPLY_LEN        160
#define RX_REPLY_WINDOW_MS  5000
struct RxReply { uint32_t seq; char text[RX_REPLY_LEN]; };
RxReply       rxReplies[RX_REPLY_LINES];
uint32_t      rxReplyNextSeq = 1;
uint32_t      rxReplyUntilMs = 0;

void rxReplyStartCapture()
{
    rxReplyUntilMs = millis() + RX_REPLY_WINDOW_MS;
}

static bool isNormalOutput(const char* l)
{
    if (l[0] != '$') return false;
    if (!strncmp(l, "$KSXT", 5)) return true;
    static const char* const types[] = {"GGA", "VTG", "HPR", "GSV", "GSA", "RMC", "GLL", "ZDA", "GST", "GNS", "HDT", "THS"};
    if (strlen(l) < 6) return false;
    for (const char* t : types) if (!strncmp(l + 3, t, 3) && (l[6] == ',' || l[6] == '*')) return true;
    return false;
}

static void rxReplyNote(const char* l)
{
    const bool answer = !strncmp(l, "$command", 8);
    // the answer to the Teensy's own VERSIONA (zReceiver.ino) is not shown in the console
    if (answer && (int32_t)(rxIdQuietUntil - millis()) > 0 && (int32_t)(rxReplyUntilMs - millis()) <= 0 && strstr(l, "VERSIONA")) return;
    // nor are the answers to the page's UM982 settings (zReceiver.ino), unless a command was typed in the console
    if (answer && umQuiet() && (int32_t)(rxReplyUntilMs - millis()) <= 0) return;
    if (!answer && ((int32_t)(rxReplyUntilMs - millis()) <= 0 || isNormalOutput(l))) return;
    for (const char* p = l; *p; p++) if ((uint8_t)*p < 0x20 || (uint8_t)*p > 0x7E) return;   // binary data
    RxReply& r = rxReplies[rxReplyNextSeq % RX_REPLY_LINES];
    r.seq = rxReplyNextSeq++;
    const size_t k = strnlen(l, sizeof(r.text) - 1);
    memcpy(r.text, l, k);
    r.text[k] = '\0';
}

// Every byte from the receiver goes through here before the parser. Counts the sentence types,
// keeps the last HPR line as received (shown by the [status] Dual messages); prints GGA / VTG /
// HPR only with RAW_NMEA_DEBUG.
void rawNmeaTee(char c)
{
    static char    rawLine[200];
    static uint8_t rawLen = 0;
    if (c == '$' || c == '#') rawLen = 0;              // '#' = Unicore ASCII log (#HEADINGA, ...)
    if (c == '\n' || c == '\r')
    {
        if (rawLen > 0)
        {
            rawLine[rawLen] = '\0';
            rxReplyNote(rawLine);
            umLine(rawLine);                                // the UM982 settings (zReceiver.ino)
        }
        if (rawLen > 2 && (rawLine[0] == '$' || rawLine[0] == '#'))
        {
            rawLine[rawLen] = '\0';
            if (nmeaLineOk(rawLine)) rxCkGood[RXCK_GPS]++;     // valid data at 460800 (zRxCheck.ino)
            if (!strncmp(rawLine, "#VERSIONA", 9)) rxIdLine(rawLine);   // the Unicore receiver's answer (zReceiver.ino)
            else if (!umIsReply(rawLine)) noteRxType(rawLine);           // answers to commands are not sentence types
        }
        if (rawLen > 6 && rawLine[0] == '$')
        {
            rawLine[rawLen] = '\0';
            const bool isHpr = !strncmp(rawLine + 3, "HPR", 3);
            if (isHpr)
            {
                strncpy(hprRawLast, rawLine, sizeof(hprRawLast) - 1);
                hprRawLast[sizeof(hprRawLast) - 1] = '\0';
                hprRawMs = millis() | 1;
            }
#if RAW_NMEA_DEBUG
            if (isHpr || !strncmp(rawLine + 3, "GGA", 3) || !strncmp(rawLine + 3, "VTG", 3))
            {
                Serial.print("[raw] ");
                Serial.println(rawLine);
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
    gnssRxPeak    = 0;
    hprRawMs      = 0;
    hprRawLast[0] = '\0';
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
    for (int i = POS_HISTORY - 1; i > 0; i--)
    {
        posLatDeg[i] = posLatDeg[i - 1]; posLonDeg[i] = posLonDeg[i - 1]; posMs[i] = posMs[i - 1];
        posTmYaw[i]  = posTmYaw[i - 1];  posTmOk[i]   = posTmOk[i - 1];
    }
    posLatDeg[0] = nmeaToDeg(latitude, latNS[0]);
    posLonDeg[0] = nmeaToDeg(longitude, lonEW[0]);
    posMs[0]     = millis();
    posTmOk[0]   = TM171DataSeen && (TM171lastData < TM171_FRESH_MS);
    posTmYaw[0]  = YawV.fValue;
    if (posCount < POS_HISTORY) posCount++;
}

// GPS course (deg) over the shortest recent stretch of at least COURSE_MIN_DIST_M. False if there
// is none (stopped, too slow, gap). kmh = speed over it, tmTurn = TM171 yaw change over it (999 = unknown).
static bool gpsCourse(float& course, float& kmh, float& tmTurn)
{
    const double lat0 = posLatDeg[0] * (M_PI / 180.0);
    for (uint8_t i = 1; i < posCount; i++)
    {
        const uint32_t dtMs = posMs[0] - posMs[i];
        if (dtMs > 3000) return false;
        const double dN = (posLatDeg[0] - posLatDeg[i]) * 111194.9;
        const double dE = (posLonDeg[0] - posLonDeg[i]) * 111194.9 * cos(lat0);
        const double dist = sqrt(dN * dN + dE * dE);
        if (dist >= COURSE_MIN_DIST_M)
        {
            if (dtMs < 50) return false;
            course = wrap360((float)(atan2(dE, dN) * 180.0 / M_PI));
            kmh    = (float)(dist / (dtMs / 1000.0) * 3.6);
            tmTurn = (posTmOk[0] && posTmOk[i]) ? fabsf(wrap180(posTmYaw[0] - posTmYaw[i])) : 999.0f;
            return true;
        }
    }
    return false;
}

// Knots and track (deg; -1 when standing still) from the distance travelled over the window.
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

    hprTake('U', hprHeading, utcCs(t), (qf[0] != '\0') ? atoi(qf) : 0, atof(pitch));
}

// AiO_WEB: this epoch's dual-antenna heading, from the UM982 (HPR_Handler) or from the second F9P
// (UBX-NAV-RELPOSNED, zDualF9P.ino); everything after this is the same for both
void hprTake(char src, const char* heading, int32_t utc, int qf, float pitch)
{
    dualSrc = src;
    if (heading != hprHeading)
    {
        strncpy(hprHeading, heading, sizeof(hprHeading) - 1);
        hprHeading[sizeof(hprHeading) - 1] = '\0';
    }
    hprUtcCs = utc;
    hprQf    = qf;
    hprPitch = pitch;
    hprMs    = millis();
    hprRxCount++;

    // Diagnostics: the HPR of an epoch that has already gone out (after EPOCH_WAIT_MS) came too late
    if (hprUtcCs >= 0 && hprUtcCs == ggaUtcCs && !ggaPending)
    {
        hprLateCount++;
        hprLateMs   = hprMs - ggaMs;
        hprLateAtMs = hprMs | 1;
    }
}

// AiO_WEB: the second F9P's RELPOSNED, as text, for the [status] lines that show "the last HPR"
void hprRawSet(const char* text)
{
    strncpy(hprRawLast, text, sizeof(hprRawLast) - 1);
    hprRawLast[sizeof(hprRawLast) - 1] = '\0';
    hprRawMs = millis() | 1;
}

// A dual heading with this QF is used: 4 (fixed), or 5 (float) when accepted on the page - never
// from the second F9P (as in the AiO firmware: RTK fixed only)
static bool hprQfOk(int qf)
{
    return qf == 4 || (qf == 5 && cfg.hprAcceptFloat && dualSrc != 'F');
}

// AiO_WEB: wheel angle from the turn rate of the heading and the speed (as in the AiO firmware, for the
// Keya without a WAS, Autosteer.ino): rate = change of the dual heading (or the aligned IMU heading)
// between epochs; reverse when the GPS track is more than 120 deg from it
static void wheelAngleFromHeading(uint32_t now, const char* track)
{
    static float    hPrev  = 0;
    static uint32_t msPrev = 0;
    static bool     have   = false;
    float h;
    if (epochDual)               h = dualHeading;
    else if (tmHeadingUsable())  h = lastOut;
    else { have = false; wheelAngleGPSOk = false; return; }
    if (have && now - msPrev > 20 && now - msPrev < 1000)
    {
        headingRateDps = wrap180(h - hPrev) * 1000.0f / (float)(now - msPrev);
        if (track[0] != '\0' && lastSpeedKmh > 0.5f) workingDir = fabsf(wrap180(atof(track) - h)) > 120.0f ? -1 : 1;
        if (lastSpeedKmh > 1.0f)
        {
            const float ms = lastSpeedKmh / 3.6f;
            const float a  = atanf(headingRateDps * DEG_TO_RAD * wheelbaseM / ms) * RAD_TO_DEG * workingDir;
            wheelAngleGPSOk = a > -50.0f && a < 50.0f;
            if (wheelAngleGPSOk) wheelAngleGPS = a;
        }
        else wheelAngleGPSOk = false;
    }
    hPrev  = h;
    msPrev = now;
    have   = true;
}

// PGN 247 vehicle geometry from AgOpenWeb (its Docs/PGN.md "Vehicle geometry", issue #387).
// Only the wheelbase is used (bytes 7-8, cm). Later layouts only add fields at the end, so a
// longer message is fine. Not saved: AgOpenWeb sends it again when the Teensy restarts.
void vehicleGeometry(const uint8_t* p, int n)
{
    if (n < 10 || p[4] < 4) return;                 // too short to hold the wheelbase
    const int last = 5 + p[4];                       // the CRC byte
    if (n <= last) return;
    uint8_t crc = 0;
    for (int i = 2; i < last; i++) crc += p[i];
    if (crc != p[last]) return;
    const uint16_t cm = p[7] | (p[8] << 8);
    if (cm < 50 || cm > 1500) return;                // 0.5 to 15 m, else keep the old value
    wheelbaseM = cm / 100.0f;
    pgn247Ms   = millis();
}

// ---------------------------------------------------------------- output
// The last sentences sent, for the web page (zWeb.ino): standard set = GGA, VTG, THS, INHPR in
// slots 0-3; classic = the one $PAOGI / $PANDA in slot 0.
#define SENT_SLOTS 4
char          lastSent[SENT_SLOTS][128] = {};
uint32_t      epochsSent   = 0;
uint8_t       lastSentKind = 0;      // 0 none yet, 1 standard set, 2 $PAOGI, 3 $PANDA
uint32_t      epochsHeld   = 0;      // simulator mode (cfg.simMode): epochs not sent
bool          simHeld      = false;  // simulator mode: the last epoch was held back

static void sendLine(const char* line, uint8_t slot)
{
#if SEND_TO_USB
    SerialAOG.write(line);
#endif
    if (slot < SENT_SLOTS)
    {
        size_t n = strcspn(line, "\r\n");                  // keep it without CR LF
        if (n > sizeof(lastSent[slot]) - 1) n = sizeof(lastSent[slot]) - 1;
        memcpy(lastSent[slot], line, n);
        lastSent[slot][n] = '\0';
    }
    if (Ethernet_running)
    {
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(line, strlen(line));
        Eth_udpPAOGI.endPacket();
    }
}

// body = everything between '$' and '*'. Adds '$', "*hh" and CR LF; one datagram per line.
static void sendSentence(const char* body, uint8_t slot)
{
    static char line[200];
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
    snprintf(line, sizeof(line), "$%s*%c%c\r\n", body, asciiHex[cs >> 4], asciiHex[cs & 0x0F]);
    sendLine(line, slot);
}

// Standard set (AgOpenWeb): $GNGGA, $GNVTG, $GNTHS, $INHPR - see the top of this file
static void sendStandard(const char* track, const char* kmh, float r, float p)
{
    char body[180];

    // ---- $GNGGA
    snprintf(body, sizeof(body), "GNGGA,%s,%s,%s,%s,%s,%s,%s,%s,%s,M,%s,M,%s,%s",
             fixTime, latitude, latNS, longitude, lonEW, fixQuality, numSats, HDOP,
             altitude, geoidSep, ageDGPS, stationId);
    sendSentence(body, 0);

    // ---- $GNVTG: receiver speed, else from the GGA positions
    if (kmh[0] != '\0')
    {
        char kn[12];
        dtostrf(atof(kmh) / 1.852f, 1, 3, kn);
        snprintf(body, sizeof(body), "GNVTG,%s,T,,M,%s,N,%s,K,A", track, kn, kmh);
    }
    else strcpy(body, "GNVTG,,T,,M,,N,,K,N");
    sendSentence(body, 1);

    // ---- $GNTHS: dual-antenna heading
    if (epochDual) snprintf(body, sizeof(body), "GNTHS,%s,A", hprHeading);
    else           strcpy(body, "GNTHS,,V");
    sendSentence(body, 2);

    // ---- $INHPR: TM171 heading, PITCH field = vehicle roll (as in the UM982 HPR), roll field = pitch, QF 4
    // Heading: the TM171 heading as aligned here, but without the steps of a re-alignment by the GPS course
    // (stdShift): AgOpenWeb fuses the IMU heading with its own offset, so a step would look like a turn.
    float hStd = wrap360(lastOut + stdShift);
    if (hStd >= 359.995f) hStd = 0.0f;                     // never print 360.00
    if (epochTmOk) snprintf(body, sizeof(body), "INHPR,%s,%.2f,%.2f,%.2f,4,,,", fixTime, hStd, r, p);
    else           snprintf(body, sizeof(body), "INHPR,%s,,,,0,,,", fixTime);
    sendSentence(body, 3);

    lastSentKind = 1;
}

// Classic (AgOpenGPS / AgIO, also read by AgOpenWeb): one sentence per epoch - see the top of this file.
//   UM982:      always $PAOGI. Heading: the dual heading (verbatim from the HPR; faded in from the
//               TM171 heading for DUAL_BLEND_EPOCHS when it comes back), else classicHdg (TM171 /
//               GPS course / held, from gnssProcess). Roll and pitch: TM171, degrees (0.00 if lost).
//   F9P / X20P: $PANDA. Heading: the TM171 heading x 10 (65535 = TM171 lost, AgOpenGPS then uses
//               fix-to-fix only). Roll and pitch: TM171, x 10.
static void sendClassic(const char* kmh, float r, float p)
{
    char body[200], knots[16], heading[20], roll[16], pitch[16];
    dtostrf((kmh[0] != '\0' ? atof(kmh) : 0.0f) / 1.852f, 1, 3, knots);   // speed in knots

    if (receiverIsDual())
    {
        if (classicSrc == 1 && blendLeft == 0)
        {
            strncpy(heading, hprHeading, sizeof(heading) - 1);
            heading[sizeof(heading) - 1] = '\0';
        }
        else
        {
            float h = classicHdg;
            if (h >= 359.995f) h = 0.0f;                      // never print 360.00
            dtostrf(h, 1, 2, heading);
        }
        if (epochTmOk)      { dtostrf(r, 1, 2, roll); dtostrf(p, 1, 2, pitch); }
        else if (epochDual) { dtostrf(epochDualRoll, 1, 2, roll); strcpy(pitch, "0.00"); }   // AiO_WEB: no IMU: roll from the antennas
        else                { strcpy(roll, "0.00");   strcpy(pitch, "0.00"); }
        snprintf(body, sizeof(body), "PAOGI,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,0",
                 fixTime, latitude, latNS, longitude, lonEW, fixQuality, numSats, HDOP,
                 altitude, ageDGPS, knots, heading, roll, pitch);
        lastSentKind = 2;
    }
    else
    {
        // The TM171 heading goes to AgOpenGPS (as its IMU) once its direction is known and it is aligned
        // to the GPS course (or the direction is set on the page): AgOpenGPS then gets an IMU heading that
        // turns the right way and already points the right way. Until then 65535 = no IMU, and AgOpenGPS
        // uses fix-to-fix only. Roll and pitch go out in any case.
        const bool imuHeadingOk = epochTmOk && yawSignLocked && (offsetSeeded || cfg.tm171YawSign != 0);
        if (epochTmOk)
        {
            long h10 = lroundf(lastOut * 10.0f);
            if (h10 >= 3600) h10 -= 3600;
            if (h10 < 0)     h10 += 3600;
            if (imuHeadingOk) snprintf(heading, sizeof(heading), "%ld", h10);
            else              strcpy(heading, "65535");      // direction not known yet: no IMU heading
            snprintf(roll,    sizeof(roll),    "%ld", lroundf(r * 10.0f));
            snprintf(pitch,   sizeof(pitch),   "%ld", lroundf(p * 10.0f));
        }
        else
        {
            strcpy(heading, "65535");                        // no IMU
            strcpy(roll, "0");
            strcpy(pitch, "0");
        }
        snprintf(body, sizeof(body), "PANDA,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,0",
                 fixTime, latitude, latNS, longitude, lonEW, fixQuality, numSats, HDOP,
                 altitude, ageDGPS, knots, heading, roll, pitch);
        lastSentKind = 3;
    }
    sendSentence(body, 0);
    for (uint8_t i = 1; i < SENT_SLOTS; i++) lastSent[i][0] = '\0';
}

// No dual heading, TM171 OK: correct the TM171 heading with the GPS course while driving straight.
// Also learns the TM171 yaw direction from the course, and aligns the TM171 to the course when it
// has not been aligned since start-up (forward assumed).
static void courseCorrect(float tmRaw)
{
    float course, kmh, turn;
    courseState = '-';
    if (!gpsCourse(course, kmh, turn)) { courseBadRun = 0; return; }
    lastCourse = course;
    if (kmh >= COURSE_MIN_KMH)
    {
        const float signBefore = yawSign;
        learnYawSign(course, tmRaw, 'C');
        if (yawSign != signBefore)
        {
            // Direction was wrong: re-align at once to the course (keeping forward / reverse)
            yawOffset    = wrap180((courseLastFix == 'R' ? course + 180.0f : course) - yawSign * tmRaw);
            offsetSeeded = true;
            offsetFrom   = 'C';
        }
    }
    if (kmh < COURSE_MIN_KMH || turn > COURSE_MAX_TURN_DEG) { courseState = 'x'; return; }

    const float tmYaw = yawSign * tmRaw;                   // after a possible sign change
    if (!offsetSeeded)
    {
        // the first alignment: COURSE_SEED_MS straight above COURSE_SEED_KMH over COURSE_SEED_DIST_M (see the #defines)
        static uint32_t seedMs = 0, seedLastMs = 0;
        static double   seedLat = 0, seedLon = 0;
        static float    seedCourse = 0;
        const uint32_t now = millis();
        const bool go = kmh >= COURSE_SEED_KMH;
        if (!go || seedLastMs == 0 || now - seedLastMs > 400 || fabsf(wrap180(course - seedCourse)) > COURSE_SEED_SPREAD_DEG)
        {
            seedLastMs = 0;                                // the run is broken: a new one from here
            if (go) { seedMs = seedLastMs = now; seedLat = posLatDeg[0]; seedLon = posLonDeg[0]; seedCourse = course; }
            return;
        }
        seedLastMs = now;
        const double dN = (posLatDeg[0] - seedLat) * 111194.9;
        const double dE = (posLonDeg[0] - seedLon) * 111194.9 * cos(posLatDeg[0] * (M_PI / 180.0));
        if (now - seedMs >= COURSE_SEED_MS && sqrt(dN * dN + dE * dE) >= COURSE_SEED_DIST_M)
        {
            yawOffset    = wrap180(course - tmYaw);
            offsetSeeded = true;
            offsetFrom   = 'C';
            courseState  = 'F';
            seedLastMs   = 0;
        }
        return;
    }

    const float d = wrap180(course - wrap360(tmYaw + yawOffset));
    float err;
    if (fabsf(d) <= COURSE_FWD_DEG)      { err = d;                   courseState = courseLastFix = 'F'; }
    else if (fabsf(d) >= COURSE_REV_DEG) { err = wrap180(d + 180.0f); courseState = courseLastFix = 'R'; }
    else if (!receiverIsDual())          { courseState = 'x'; return; }
    else
    {
        // Driving straight, but the course fits neither forward nor reverse: the TM171 yaw moved
        courseState = 'x';
        if (++courseBadRun >= COURSE_RESEED_EPOCHS)
        {
            yawOffset    = wrap180(course - tmYaw);        // forward assumed
            offsetFrom   = 'C';
            courseBadRun = 0;
            courseReseedCount++;
        }
        return;
    }
    courseBadRun = 0;
    // After the first alignment the GPS course keeps correcting the IMU heading only for a dual-antenna
    // receiver: its $PAOGI fallback, which AgOpenGPS (heading source Dual) takes as it is. AgOpenWeb
    // ($INHPR, $PANDA) and AgOpenGPS with $PANDA align the IMU heading to the GPS themselves
    // (imuGPS_Offset, AgOpenWeb's GpsHeadingFusionService): a second correction here would only fight theirs.
    if (!receiverIsDual()) return;
    float step = COURSE_K * err;
    if (step >  COURSE_MAX_STEP_DEG) step =  COURSE_MAX_STEP_DEG;
    if (step < -COURSE_MAX_STEP_DEG) step = -COURSE_MAX_STEP_DEG;
    yawOffset = wrap180(yawOffset + step);
    courseFixCount++;
}

// Called from loop(). Sends the epoch once its VTG and HPR are in, or cfg.epochWaitMs after
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
    if (!complete && (now - ggaMs) < cfg.epochWaitMs) return;

    // ---- speed and track: receiver VTG, else from the GGA positions
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
    lastSpeedKmh = (kmh[0] != '\0') ? atof(kmh) : 0.0f;

    // ---- dual-antenna heading: the UM982 HPR of this epoch, QF 4 (fixed) or 5 (float)
    epochHprQf = hprIn ? hprQf : -1;
    epochDual  = false;
    if (hprIn && hprHeading[0] != '\0' && hprQfOk(hprQf))
    {
        const float h = atof(hprHeading);
        if (h >= 0.0f && h <= 360.0f)
        {
            dualHeading   = (h >= 360.0f) ? 0.0f : h;
            epochDual     = true;
            epochDualRoll = hprPitch;            // HPR pitch field / RELPOSNED: the vehicle roll
        }
    }

    // Why there is no dual heading (for the [status] lines)
    if (epochDual)                                                   epochNoDual = NODUAL_NONE;
    else if (!hprPresent)                                            epochNoDual = NODUAL_NO_HPR;
    else if (!hprIn)                                                 epochNoDual = NODUAL_NOT_IN_TIME;
    else if (!hprQfOk(hprQf))                                        epochNoDual = NODUAL_QF;
    else                                                             epochNoDual = NODUAL_HEADING;
    noDualCount[epochNoDual]++;

    // ---- dual heading with hysteresis: lost at once, back after DUAL_ENTER_EPOCHS valid epochs.
    // Used for the TM171 alignment and the classic $PAOGI ($GNTHS keeps the raw epochDual).
    if (epochDual) { if (dualGoodRun < 255) dualGoodRun++; }
    else           dualGoodRun = 0;
    const bool wasActive = dualActive;
    if (!epochDual) dualActive = false;
    else if (!dualActive && dualGoodRun >= DUAL_ENTER_EPOCHS) dualActive = true;

    // ---- TM171: roll and pitch; heading aligned to the dual heading, else corrected by the GPS course
    const float tmRaw = YawV.fValue;
    float r = 0, p = 0;
    epochTmOk = TM171DataSeen && (TM171lastData < TM171_FRESH_MS);
    if (epochTmOk)
    {
        // Yaw glitch (a step no tractor turns in 100 ms): shift the offset by the same amount
        if (tmPrevOk && (now - tmPrevMs) < 300)
        {
            const float step = wrap180(tmRaw - tmPrevRaw);
            if (fabsf(step) > TM171_JUMP_DEG)
            {
                if (offsetSeeded) yawOffset = wrap180(yawOffset - yawSign * step);
                tmJumpCount++;
            }
        }
        tmPrevOk  = true;
        tmPrevRaw = tmRaw;
        tmPrevMs  = now;
        // TM171 heading before this epoch's alignment (to keep the standard $INHPR free of steps)
        const float outBefore  = wrap360(yawSign * tmRaw + (offsetSeeded ? yawOffset : 0.0f));
        const float signBefore = yawSign;

        if (dualActive)
        {
            // Keep the TM171 yaw aligned to the dual heading for when the second antenna is lost
            learnYawSign(dualHeading, tmRaw, 'D');
            const float measured = dualHeading - yawSign * tmRaw;
            const bool  snap = !offsetSeeded || fabsf(wrap180(measured - yawOffset)) > OFFSET_SNAP_DEG;
            updateOffset(measured, snap ? 1.0f : OFFSET_K_DUAL);
            offsetFrom   = 'D';
            courseState  = '-';
            courseBadRun = 0;
        }
        else courseCorrect(tmRaw);

        const bool swapAxes = ((cfg.tm171SwapRollPitch != 0) != (steerConfig.IsUseY_Axis != 0));
        if (swapAxes) { r = PitchV.fValue; p = RollV.fValue; }
        else          { r = RollV.fValue;  p = PitchV.fValue; }

        float out = wrap360(yawSign * tmRaw + (offsetSeeded ? yawOffset : 0.0f));
        // Standard $INHPR: aligned to the dual heading while it is in use (AgOpenWeb then uses $GNTHS);
        // without it, every re-alignment by the GPS course goes into stdShift: AgOpenWeb aligns the IMU
        // heading to the GPS itself (GpsHeadingFusionService), so the firmware does not do it a second time.
        // A new yaw direction is the exception: the heading had been turning the wrong way, and the
        // re-aligned one (to the course) is closer to the truth than a continuation of it.
        if (dualActive || yawSign != signBefore) stdShift = 0.0f;
        else stdShift = wrap180(stdShift - wrap180(out - outBefore));
        if (out >= 359.995f) out = 0.0f;                   // never print 360.00
        lastOut  = out;
        lastRoll = r;
    }
    else
    {
        tmPrevOk = false;
        lastOut = lastRoll = 0;
    }

    // ---- heading of the classic $PAOGI (UM982)
    if (dualActive)
    {
        if (!wasActive)
        {
            // Back to dual: fade from the heading AgOpenGPS had, if it is close (else switch at once)
            blendErr  = wrap180(classicHdg - dualHeading);
            blendLeft = (classicHdgTrusted && fabsf(blendErr) < DUAL_BLEND_MAX_DEG) ? DUAL_BLEND_EPOCHS : 0;
        }
        classicSrc = 1;
        classicHdg = dualHeading;
        if (blendLeft > 0)
        {
            classicHdg = wrap360(dualHeading + blendErr * ((float)blendLeft / (DUAL_BLEND_EPOCHS + 1)));
            blendLeft--;
        }
        classicHdgTrusted = true;
    }
    else
    {
        blendLeft = 0;
        float course, kmh, turn;
        if (tmHeadingUsable())
        {
            classicSrc = 2;                                // TM171, direction known and aligned
            classicHdg = lastOut;
            classicHdgTrusted = true;
        }
        else if (gpsCourse(course, kmh, turn) && kmh >= COURSE_MIN_KMH)
        {
            // GPS course while moving: TM171 lost, or its direction / alignment not known yet.
            // Reverse detected by continuity with a trusted heading; with none, forward is assumed.
            lastCourse = course;
            if (classicHdgTrusted && fabsf(wrap180(course - classicHdg)) > 90.0f) course = wrap360(course + 180.0f);
            classicSrc = 3;
            classicHdg = course;
            classicHdgTrusted = true;
        }
        else
        {
            classicSrc = 4;                                // stopped: hold (the raw TM171 heading if there was never one)
            if (!haveClassicHdg) classicHdg = lastOut;
        }
    }
    haveClassicHdg = true;
    wheelAngleFromHeading(now, track);


    // ---- out to AgOpenWeb / AgOpenGPS
    // Simulator mode (setting, for the program's simulator only): the GPS does not go out. AgOpenWeb
    // switches its simulator off (its button greyed out) while it gets GPS sentences, with or without a fix.
    const bool hold = cfg.simMode != 0;
    simHeld = hold;                                    // (its message: the [boot] line, or the [web] line when switched on)
    if (hold) epochsHeld++;
    else
    {
        if (outClassicNow()) sendClassic(kmh, r, p);   // the setting, or in automatic mode the program (zStatus.ino)
        else                 sendStandard(track, kmh, r, p);
        epochsSent++;
    }

    // LEDs: green = dual heading, red = TM171 heading (single antenna), red blinking = TM171 lost
    digitalWrite(GPSGREEN_LED, epochDual ? HIGH : LOW);
    if (!epochTmOk) digitalWrite(GPSRED_LED, blink ? HIGH : LOW);
    else            digitalWrite(GPSRED_LED, epochDual ? LOW : HIGH);

#if FUSION_DEBUG
    static uint32_t dbgMs = 0;
    if (millis() - dbgMs > 1000)
    {
        dbgMs = millis();
        Serial.printf("[fusion] %s%s hpr%s q=%d hdg='%s' antRoll=%.2f | TM171 %s yaw=%.2f sign=%+.0f%s off=%.2f(%c%s) -> hdg=%.2f roll=%.2f | course=%.1f(%c) fix=%lu reseed=%lu jumps=%lu | classic src=%u hdg=%.2f | spd=%.1fkm/h(%s) | ggaHeld=%lu tmCrcErr=%lu\r\n",
                      epochDual ? "DUAL" : "SINGLE", dualActive ? "" : (epochDual ? "(confirming)" : ""), hprPresent ? "" : "(none: single-antenna receiver)", epochHprQf, hprIn ? hprHeading : "", hprPitch,
                      epochTmOk ? "ok" : "LOST", tmRaw, yawSign, yawSignLocked ? "(locked)" : "(learning)",
                      yawOffset, offsetFrom, offsetSeeded ? "" : ",unseeded", lastOut, lastRoll,
                      lastCourse, courseState, (unsigned long)courseFixCount, (unsigned long)courseReseedCount, (unsigned long)tmJumpCount,
                      classicSrc, classicHdg,
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
