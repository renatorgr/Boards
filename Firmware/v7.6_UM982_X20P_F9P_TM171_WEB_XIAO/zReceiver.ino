/* zReceiver.ino - which GPS receiver is on Serial7, and the u-blox settings from the web page.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr (v7.6).
   The u-blox messages and configuration keys are the ones of the u-blox ZED-F9P / ZED-X20P
   interface descriptions (UBX-MON-VER, UBX-CFG-VALGET, UBX-CFG-VALSET, UBX-ACK); the Unicore
   command is VERSIONA (Unicore N4 commands and logs reference).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zReceiver.ino
//   - Which receiver: the Teensy asks it. "VERSIONA" for a Unicore receiver (UM982: it answers
//     #VERSIONA with its model and firmware), UBX-MON-VER for a u-blox one (ZED-F9P / ZED-X20P: it
//     answers MOD=..., FWVER=..., PROTVER=...). First the question that fits what it sends (HPR =
//     Unicore), then the other one, up to RXID_QUESTIONS questions. Asked once the receiver has
//     talked for a second; asked again when it talks again after RXID_SILENT_MS of silence (another
//     receiver, or one switched on later) and from the web page.
//   - UBX frames (B5 62 ...) are taken out of the bytes from the receiver before the NMEA parser,
//     so the u-blox answers (and any UBX output it is set to send) never reach the NMEA parser.
//   - u-blox settings (web page, GPS tab): read with UBX-CFG-VALGET one key at a time from the RAM
//     layer (what the receiver uses now), and written with one UBX-CFG-VALSET with only the keys
//     changed on the page, to RAM + BBR + flash (kept after a power off; the receiver takes all of
//     them or none). If the receiver refuses the flash, again to RAM + BBR. Never written from the
//     page: the UART1 baud rate and the UBX / NMEA output of UART1 (the Teensy would lose the receiver).
//   - Every question goes to the receiver between two RTCM messages of the corrections (zRtcm.h),
//     never in the middle of one.
// The receiver type does not change what is sent to the program: $PAOGI / $PANDA still follow
// the HPR (zHandlers.ino).

#include "zReceiver.h"

#define RXID_TALK_MS      1000     // the receiver talks this long before the first question
#define RXID_ANSWER_MS    1500     // time for an answer
#define RXID_QUESTIONS    6        // questions per round, VERSIONA and UBX-MON-VER in turn
#define RXID_SILENT_MS    10000    // silent this long, then talking again: asked again
#define RX_TALKS_MS       2000     // a byte in the last 2 s: the receiver talks
#define UBX_MAX           1200     // largest UBX frame kept (a longer one is skipped)
#define UBX_GET_MS        400      // time for the answer to one VALGET
#define UBX_SET_MS        2500     // time for the answer to a VALSET (the flash takes a moment)

uint8_t  rxKind      = RX_UNKNOWN;
uint8_t  rxIdState   = RXID_WAIT;
char     rxModel[24] = "";
char     rxFw[40]    = "";
char     rxProt[16]  = "";
uint32_t rxIdQuietUntil = 0;
static char     rxHw[12]      = "";
static uint32_t rxIdMs        = 0;       // when it was identified (millis, 0 = not yet)
static uint8_t  rxIdAsked     = 0;       // questions asked in this round
static bool     rxIdLastUm    = false;   // the last question was VERSIONA (else UBX-MON-VER)
static uint32_t rxIdAskMs     = 0;
static uint32_t rxByteMs      = 0;       // last byte from the receiver
static uint32_t rxTalkSinceMs = 0;       // talking without a gap since
static bool     rxSilentSeen  = true;    // silent for RXID_SILENT_MS since the last round

// ---------------------------------------------------------------- to the receiver
// One message waits here until the corrections are between two RTCM messages (or 300 ms).
static uint8_t  rxTxBuf[320];
static uint16_t rxTxLen    = 0;
static uint32_t rxTxSince  = 0;
static uint32_t rxTxSentMs = 0;

static bool rtcmBetween(const RtcmCounter& r, uint32_t now)
{
    return r.st == 0 || now - r.lastMs > 200;
}

static bool rxTxQueue(const uint8_t* d, uint16_t n)
{
    if (rxTxLen || n > sizeof(rxTxBuf)) return false;
    memcpy(rxTxBuf, d, n);
    rxTxLen   = n;
    rxTxSince = millis();
    return true;
}

static void rxTxFlush(uint32_t now)
{
    if (!rxTxLen) return;
    if (!(rtcmBetween(ntripRtcm, now) && rtcmBetween(radioRtcm, now)) && now - rxTxSince < 300) return;
    SerialGPS->write(rxTxBuf, rxTxLen);
    rxTxLen    = 0;
    rxTxSentMs = now;
}

static bool ubxSend(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len)
{
    uint8_t f[sizeof(rxTxBuf)];
    if (len + 8u > sizeof(f)) return false;
    f[0] = 0xB5; f[1] = 0x62; f[2] = cls; f[3] = id; f[4] = (uint8_t)len; f[5] = (uint8_t)(len >> 8);
    if (len) memcpy(f + 6, p, len);
    uint8_t a = 0, b = 0;
    for (uint16_t k = 2; k < 6 + len; k++) { a += f[k]; b += a; }
    f[6 + len] = a;
    f[7 + len] = b;
    return rxTxQueue(f, len + 8);
}

// ---------------------------------------------------------------- identification
// The receiver sends HPR now (a Unicore receiver: UM982)
static bool rxSendsHpr()
{
    return hprMs != 0 && millis() - hprMs < RECEIVER_SEEN_MS;
}

const char* rxKindName()
{
    return rxKind == RX_UBLOX ? "u-blox" : rxKind == RX_UM982 ? "Unicore" : "";
}

const char* rxShortName(const char* fallback)
{
    return rxModel[0] ? rxModel : fallback;
}

// printable ASCII only, at most n - 1 characters, trailing spaces cut
static void rxCopyText(char* out, size_t n, const char* s, size_t max)
{
    size_t k = 0;
    for (size_t i = 0; i < max && s[i] && k + 1 < n; i++)
        if ((uint8_t)s[i] >= 0x20 && (uint8_t)s[i] <= 0x7E) out[k++] = s[i];
    while (k && out[k - 1] == ' ') k--;
    out[k] = '\0';
}

static void rxIdentified(uint8_t kind)
{
    rxKind    = kind;
    rxIdState = RXID_DONE;
    rxIdMs    = millis() | 1;
    rxSilentSeen = false;                                  // (also when the answer came without a question)
    StatusLog.line("[status] GPS receiver: %s %s, firmware %s%s%s%s", rxKindName(), rxModel[0] ? rxModel : "(model not given)",
                   rxFw[0] ? rxFw : "?", rxProt[0] ? " (protocol " : "", rxProt, rxProt[0] ? ")" : "");
}

// UBX-MON-VER: swVersion[30], hwVersion[10], then 30-character extensions (MOD=, FWVER=, PROTVER=)
static void ubxMonVer(const uint8_t* p, uint16_t len)
{
    char sw[32], ext[32];
    rxCopyText(sw, sizeof(sw), (const char*)p, 30);
    rxCopyText(rxHw, sizeof(rxHw), (const char*)p + 30, 10);
    rxModel[0] = rxFw[0] = rxProt[0] = '\0';
    for (uint16_t off = 40; off + 30 <= len; off += 30)
    {
        rxCopyText(ext, sizeof(ext), (const char*)p + off, 30);
        if      (!strncmp(ext, "MOD=", 4))     rxCopyText(rxModel, sizeof(rxModel), ext + 4, 30);
        else if (!strncmp(ext, "FWVER=", 6))   rxCopyText(rxFw, sizeof(rxFw), ext + 6, 30);
        else if (!strncmp(ext, "PROTVER=", 8)) rxCopyText(rxProt, sizeof(rxProt), ext + 8, 30);
    }
    if (!rxModel[0])
    {
        if (!strcmp(rxHw, "00190000")) strcpy(rxModel, "u-blox F9");
        else                           snprintf(rxModel, sizeof(rxModel), "u-blox (hw %s)", rxHw);
    }
    if (!rxFw[0])    rxCopyText(rxFw, sizeof(rxFw), sw, sizeof(sw));
    rxIdentified(RX_UBLOX);
}

// Every text line from the receiver (zHandlers.ino, rawNmeaTee). The Unicore answer:
// #VERSIONA,<header>;"UM982","R4.10Build11826","HRPT00-S10C-P",...*crc   (quotes may be missing)
void rxIdLine(const char* l)
{
    if (strncmp(l, "#VERSIONA", 9)) return;
    const char* s = strchr(l, ';');
    if (!s) return;
    char f[2][40];
    uint8_t k = 0, n = 0;
    bool inq = false;
    for (const char* p = s + 1; *p && *p != '*' && k < 2; p++)
    {
        if (*p == '"') { inq = !inq; continue; }
        if (*p == ',' && !inq) { f[k][n] = '\0'; k++; n = 0; continue; }
        if (n < sizeof(f[0]) - 1 && (uint8_t)*p >= 0x20 && (uint8_t)*p <= 0x7E) f[k][n++] = *p;
    }
    if (k < 2) { f[k][n] = '\0'; k++; }
    if (k < 2) f[1][0] = '\0';
    rxCopyText(rxModel, sizeof(rxModel), f[0], sizeof(f[0]));
    rxCopyText(rxFw, sizeof(rxFw), f[1], sizeof(f[1]));
    rxProt[0] = rxHw[0] = '\0';
    rxIdentified(RX_UM982);
}

static void rxAsk(bool um)
{
    static const uint8_t VERSIONA[] = "VERSIONA\r\n";
    if (um)
    {
        if (!rxTxQueue(VERSIONA, sizeof(VERSIONA) - 1)) return;
        rxIdQuietUntil = millis() + 4000;                 // its "$command,VERSIONA,response: OK" stays out of the console
    }
    else if (!ubxSend(0x0A, 0x04, nullptr, 0)) return;    // UBX-MON-VER poll
    rxIdLastUm = um;
    rxIdAskMs  = millis();
    rxIdAsked++;
}

static void ubxForget();

void receiverIdentifyAgain()
{
    ubxForget();                                           // the settings read belonged to the receiver before
    rxIdState = RXID_WAIT;
    rxKind    = RX_UNKNOWN;
    rxModel[0] = rxFw[0] = rxProt[0] = rxHw[0] = '\0';
    rxIdMs    = 0;
    rxIdAsked = 0;
    rxTalkSinceMs = rxByteMs ? millis() - RXID_TALK_MS : 0;    // ask at once if it talks
}

// ---------------------------------------------------------------- UBX frames from the receiver
static uint8_t  ubxSt = 0, ubxCls = 0, ubxId = 0, ubxCkA = 0, ubxCkB = 0;
static uint16_t ubxLen = 0, ubxPos = 0;
static uint8_t  ubxBuf[UBX_MAX];
static uint32_t ubxFrames = 0, ubxBad = 0;

static void ubxFrame(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len);

static inline void ubxCk(uint8_t b) { ubxCkA += b; ubxCkB += ubxCkA; }

// Every byte from the receiver, before the NMEA parser. True: the byte belongs to a UBX frame.
bool ubxFeed(uint8_t b)
{
    const uint32_t now = millis();
    if (ubxSt && now - rxByteMs > 100) ubxSt = 0;         // a frame cut off (receiver unplugged): forget it
    if (!rxByteMs || now - rxByteMs > RX_TALKS_MS) rxTalkSinceMs = now;
    rxByteMs = now;
    switch (ubxSt)
    {
        case 0:
            if (b != 0xB5) return false;                   // NMEA / Unicore text (never 0xB5)
            ubxSt = 1;
            return true;
        case 1:
            if (b == 0x62) { ubxSt = 2; return true; }
            ubxSt = 0;                                     // a lone 0xB5 (dropped): this byte is ordinary
            return ubxFeed(b);
        case 2: ubxCls = b; ubxCkA = 0; ubxCkB = 0; ubxCk(b); ubxSt = 3; return true;
        case 3: ubxId = b; ubxCk(b); ubxSt = 4; return true;
        case 4: ubxLen = b; ubxCk(b); ubxSt = 5; return true;
        case 5:
            ubxLen |= (uint16_t)b << 8;
            ubxCk(b);
            ubxPos = 0;
            if (ubxLen > UBX_MAX) { ubxSt = 0; ubxBad++; return true; }   // not a frame this firmware reads
            ubxSt = ubxLen ? 6 : 7;
            return true;
        case 6:
            ubxBuf[ubxPos++] = b;
            ubxCk(b);
            if (ubxPos >= ubxLen) ubxSt = 7;
            return true;
        case 7:
            ubxSt = (b == ubxCkA) ? 8 : 9;
            return true;
        case 8:
            ubxSt = 0;
            if (b == ubxCkB) { ubxFrames++; ubxFrame(ubxCls, ubxId, ubxBuf, ubxLen); }
            else ubxBad++;
            return true;
        default:                                           // second checksum byte of a bad frame
            ubxSt = 0;
            ubxBad++;
            return true;
    }
}

// ---------------------------------------------------------------- u-blox settings
// Key IDs from the u-blox interface descriptions (same keys on the ZED-F9P and the ZED-X20P).
// The value size is in the key (bits 28-30): 1 = one bit, 2 = one byte, 3 = two, 4 = four.
struct UbxItem { const char* name; uint32_t key; long lo, hi; bool ro, sgn; };
static const UbxItem UBX_ITEMS[] = {
    {"rate",    0x30210001UL, 25, 10000, false, false},   // CFG-RATE-MEAS, ms between measurements
    {"navRate", 0x30210002UL, 1, 127,    true,  false},   // CFG-RATE-NAV, measurements per solution
    {"dyn",     0x20110021UL, 0, 12,     false, false},   // CFG-NAVSPG-DYNMODEL
    {"minElev", 0x201100A4UL, -90, 90,   false, true},    // CFG-NAVSPG-INFIL_MINELEV, degrees
    {"hp",      0x10930006UL, 0, 1,      false, false},   // CFG-NMEA-HIGHPREC
    {"gga",     0x209100BBUL, 0, 255,    false, false},   // CFG-MSGOUT-NMEA_ID_..._UART1: every n solutions, 0 = off
    {"vtg",     0x209100B1UL, 0, 255,    false, false},
    {"rmc",     0x209100ACUL, 0, 255,    false, false},
    {"gsa",     0x209100C0UL, 0, 255,    false, false},
    {"gsv",     0x209100C5UL, 0, 255,    false, false},
    {"gll",     0x209100CAUL, 0, 255,    false, false},
    {"gst",     0x209100D4UL, 0, 255,    false, false},
    {"zda",     0x209100D9UL, 0, 255,    false, false},
    {"gps",     0x1031001FUL, 0, 1,      false, false},   // CFG-SIGNAL-..._ENA
    {"glo",     0x10310025UL, 0, 1,      false, false},
    {"gal",     0x10310021UL, 0, 1,      false, false},
    {"bds",     0x10310022UL, 0, 1,      false, false},
    {"qzss",    0x10310024UL, 0, 1,      false, false},
    {"sbas",    0x10310020UL, 0, 1,      false, false},
    {"inRtcm",  0x10730004UL, 0, 1,      false, false},   // CFG-UART1INPROT-RTCM3X
    {"inUbx",   0x10730001UL, 0, 1,      true,  false},   // CFG-UART1INPROT-UBX
    {"outUbx",  0x10740001UL, 0, 1,      true,  false},   // CFG-UART1OUTPROT-UBX
    {"outNmea", 0x10740002UL, 0, 1,      true,  false},   // CFG-UART1OUTPROT-NMEA
    {"baud",    0x40520001UL, 0, 0,      true,  false},   // CFG-UART1-BAUDRATE
};
#define UBX_N ((uint8_t)(sizeof(UBX_ITEMS) / sizeof(UBX_ITEMS[0])))
#define KEY_UART1OUTPROT_UBX 0x10740001UL

enum UbxOp : uint8_t { UBXOP_NONE = 0, UBXOP_READ, UBXOP_WRITE, UBXOP_UBXON };
enum UbxRes : uint8_t { UBXRES_NONE = 0, UBXRES_READ, UBXRES_READ_NONE, UBXRES_WRITE, UBXRES_WRITE_RAM,
                        UBXRES_WRITE_NAK, UBXRES_WRITE_NONE, UBXRES_UBXON };

static int32_t  ubxVal[UBX_N];
static uint8_t  ubxHave[UBX_N];          // 0 not read, 1 read, 2 the receiver does not know the key
static int32_t  ubxNew[UBX_N];
static bool     ubxNewOn[UBX_N];
static uint8_t  ubxOp = UBXOP_NONE, ubxIdx = 0, ubxTimeouts = 0;
static uint8_t  ubxWait = 0;             // 1 waiting for a VALGET answer, 2 for the ACK of a VALSET
static uint8_t  ubxGot = 0;              // 1 answer / ACK, 2 NAK
static uint8_t  ubxRes = UBXRES_NONE;
static uint8_t  ubxLayers = 0;
static uint32_t ubxSeq = 0, ubxReadMs = 0;

static void ubxForget()
{
    for (uint8_t i = 0; i < UBX_N; i++) ubxHave[i] = 0;
    ubxOp = UBXOP_NONE;
    ubxWait = 0;
    ubxRes = UBXRES_NONE;
    ubxReadMs = 0;
    ubxSeq++;
}

uint8_t     ubxItemCount()          { return UBX_N; }
const char* ubxItemName(uint8_t i)  { return i < UBX_N ? UBX_ITEMS[i].name : ""; }

static uint8_t ubxKeySize(uint32_t key)
{
    switch ((key >> 28) & 7)
    {
        case 1: case 2: return 1;
        case 3:         return 2;
        case 4:         return 4;
        default:        return 8;
    }
}

static void ubxAck(bool ack, uint8_t cls, uint8_t id)
{
    if (cls != 0x06) return;
    if (ubxWait == 1 && id == 0x8B && !ack) ubxGot = 2;                 // VALGET: the value comes in its own answer
    if (ubxWait == 2 && id == 0x8A)         ubxGot = ack ? 1 : 2;       // VALSET
}

// UBX-CFG-VALGET answer: version, layer, position (2), then key (4) + value pairs
static void ubxValget(const uint8_t* p, uint16_t len)
{
    uint16_t o = 4;
    while (o + 4 <= len)
    {
        const uint32_t key = (uint32_t)p[o] | ((uint32_t)p[o + 1] << 8) | ((uint32_t)p[o + 2] << 16) | ((uint32_t)p[o + 3] << 24);
        const uint8_t  sz  = ubxKeySize(key);
        o += 4;
        if (o + sz > len) break;
        uint32_t v = 0;
        for (uint8_t k = 0; k < sz && k < 4; k++) v |= (uint32_t)p[o + k] << (8 * k);
        o += sz;
        for (uint8_t i = 0; i < UBX_N; i++)
        {
            if (UBX_ITEMS[i].key != key) continue;
            ubxVal[i]  = UBX_ITEMS[i].sgn && sz == 1 ? (int32_t)(int8_t)v : (int32_t)v;
            ubxHave[i] = 1;
            if (ubxWait == 1 && i == ubxIdx) ubxGot = 1;
        }
    }
}

// A short name for the UBX messages the receiver sends by itself ("O recetor envia" on the page)
static void ubxNoteType(uint8_t cls, uint8_t id)
{
    static const struct { uint8_t c, i; const char* n; } N[] = {
        {0x01, 0x07, "UBX-NAV-PVT"}, {0x01, 0x3C, "UBX-NAV-RELPOS"}, {0x01, 0x14, "UBX-NAV-HPPOS"}, {0x01, 0x02, "UBX-NAV-POSLLH"},
        {0x01, 0x35, "UBX-NAV-SAT"}, {0x01, 0x03, "UBX-NAV-STATUS"}, {0x01, 0x04, "UBX-NAV-DOP"}, {0x01, 0x12, "UBX-NAV-VELNED"},
        {0x01, 0x43, "UBX-NAV-SIG"}, {0x02, 0x15, "UBX-RXM-RAWX"}, {0x02, 0x13, "UBX-RXM-SFRBX"}, {0x02, 0x32, "UBX-RXM-RTCM"},
        {0x0A, 0x09, "UBX-MON-HW"}, {0x0A, 0x38, "UBX-MON-RF"}, {0x0D, 0x01, "UBX-TIM-TP"}};
    for (const auto& e : N) if (e.c == cls && e.i == id) { noteRxTypeName(e.n); return; }
    char n[16];
    snprintf(n, sizeof(n), "UBX-%02X-%02X", cls, id);
    noteRxTypeName(n);
}

static void ubxFrame(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len)
{
    if (cls == 0x0A && id == 0x04)                 // MON-VER (length 0 = our own question, repeated by a
    {                                              // receiver that is not u-blox)
        if (len >= 40) ubxMonVer(p, len);
        return;
    }
    if (cls == 0x05) { if (len >= 2) ubxAck(id == 0x01, p[0], p[1]); return; }
    if (cls == 0x06) { if (id == 0x8B && len >= 4) ubxValget(p, len); return; }
    ubxNoteType(cls, id);
}

static void ubxGetNext()
{
    const uint32_t key = UBX_ITEMS[ubxIdx].key;
    const uint8_t q[8] = {0, 0, 0, 0, (uint8_t)key, (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24)};   // version 0, RAM layer
    if (ubxSend(0x06, 0x8B, q, sizeof(q))) { ubxWait = 1; ubxGot = 0; }
}

static void ubxSetSend()
{
    uint8_t q[4 + UBX_N * 8];
    uint16_t n = 0;
    q[n++] = 0; q[n++] = ubxLayers; q[n++] = 0; q[n++] = 0;     // version 0, layers, reserved
    for (uint8_t i = 0; i < UBX_N; i++)
    {
        if (!ubxNewOn[i]) continue;
        const uint32_t key = UBX_ITEMS[i].key;
        for (uint8_t k = 0; k < 4; k++) q[n++] = (uint8_t)(key >> (8 * k));
        const uint8_t sz = ubxKeySize(key);
        for (uint8_t k = 0; k < sz; k++) q[n++] = (uint8_t)((uint32_t)ubxNew[i] >> (8 * k));
    }
    if (ubxSend(0x06, 0x8A, q, n)) { ubxWait = 2; ubxGot = 0; }
}

static void ubxReadStart()
{
    ubxOp = UBXOP_READ;
    ubxIdx = 0;
    ubxTimeouts = 0;
    ubxWait = 0;
}

static void ubxOpEnd()
{
    ubxOp = UBXOP_NONE;
    ubxWait = 0;
    ubxSeq++;
}

static void ubxOpLoop(uint32_t now)
{
    if (ubxOp == UBXOP_NONE || rxTxLen) return;           // nothing to do, or the question is not out yet
    if (ubxOp == UBXOP_READ)
    {
        if (ubxWait == 0)
        {
            if (ubxIdx >= UBX_N)
            {
                if (ubxRes != UBXRES_WRITE && ubxRes != UBXRES_WRITE_RAM) ubxRes = UBXRES_READ;
                ubxReadMs = now | 1;
                ubxOpEnd();
                return;
            }
            ubxGetNext();
            return;
        }
        if (ubxGot == 0 && now - rxTxSentMs < UBX_GET_MS) return;
        if (ubxGot == 2) ubxHave[ubxIdx] = 2;
        if (ubxGot == 0)
        {
            ubxHave[ubxIdx] = 0;
            if (++ubxTimeouts >= 3)
            {
                ubxRes = UBXRES_READ_NONE;
                StatusLog.line("[status] GPS receiver: no answer to UBX-CFG-VALGET (UBX output off on UART1?)");
                ubxOpEnd();
                return;
            }
        }
        else ubxTimeouts = 0;
        ubxIdx++;
        ubxWait = 0;
        return;
    }
    if (ubxOp == UBXOP_WRITE)
    {
        if (ubxWait == 0) { ubxSetSend(); return; }
        if (ubxGot == 0 && now - rxTxSentMs < UBX_SET_MS) return;
        if (ubxGot == 1)
        {
            ubxRes = ubxLayers == 7 ? UBXRES_WRITE : UBXRES_WRITE_RAM;
            StatusLog.line("[status] GPS receiver: settings written to %s", ubxLayers == 7 ? "RAM, BBR and flash" : "RAM and BBR (the flash refused them)");
            ubxSeq++;
            ubxReadStart();                                // read them back
            return;
        }
        if (ubxGot == 2 && ubxLayers == 7) { ubxLayers = 3; ubxWait = 0; return; }   // again without the flash
        ubxRes = ubxGot == 2 ? UBXRES_WRITE_NAK : UBXRES_WRITE_NONE;
        StatusLog.line("[status] GPS receiver: settings NOT written (%s)", ubxGot == 2 ? "the receiver refused them" : "no answer");
        ubxOpEnd();
        return;
    }
    if (ubxOp == UBXOP_UBXON)
    {
        if (ubxWait == 0) { ubxSetSend(); return; }
        if (ubxGot == 0 && now - rxTxSentMs < UBX_SET_MS) return;
        ubxRes = UBXRES_UBXON;
        StatusLog.line("[status] GPS receiver: UBX output on UART1 %s", ubxGot == 1 ? "switched on" : ubxGot == 2 ? "refused" : "asked (no answer)");
        ubxOpEnd();
        receiverIdentifyAgain();
    }
}

const char* ubxStartRead()
{
    if (rxKind != RX_UBLOX)     return "not a u-blox receiver";
    if (ubxOp != UBXOP_NONE)    return "busy";
    ubxReadStart();
    return nullptr;
}

void ubxWriteBegin()
{
    for (uint8_t i = 0; i < UBX_N; i++) ubxNewOn[i] = false;
}

const char* ubxWriteAdd(uint8_t i, long v)
{
    if (i >= UBX_N || UBX_ITEMS[i].ro)                 return "setting cannot be written";
    if (v < UBX_ITEMS[i].lo || v > UBX_ITEMS[i].hi)    return "invalid value";
    if (UBX_ITEMS[i].key == 0x20110021UL && v == 1)    return "invalid value";   // dynamic model 1 does not exist
    ubxNew[i]   = (int32_t)v;
    ubxNewOn[i] = true;
    return nullptr;
}

const char* ubxWriteGo()
{
    if (rxKind != RX_UBLOX)     return "not a u-blox receiver";
    if (ubxOp != UBXOP_NONE)    return "busy";
    uint8_t n = 0;
    char list[200] = "";
    for (uint8_t i = 0; i < UBX_N; i++)
    {
        if (!ubxNewOn[i]) continue;
        n++;
        const size_t u = strlen(list);
        snprintf(list + u, sizeof(list) - u, "%s%s=%ld", u ? ", " : "", UBX_ITEMS[i].name, (long)ubxNew[i]);
    }
    if (!n) return "nothing to change";
    StatusLog.line("[web] GPS receiver settings: %s", list);
    ubxOp = UBXOP_WRITE;
    ubxLayers = 7;                                         // RAM + BBR + flash
    ubxWait = 0;
    return nullptr;
}

// The receiver did not answer: switch its UBX output on UART1 on (u-blox only; it can only help)
const char* ubxOutputOn()
{
    if (rxKind == RX_UM982)     return "not a u-blox receiver";
    if (ubxOp != UBXOP_NONE)    return "busy";
    ubxWriteBegin();
    for (uint8_t i = 0; i < UBX_N; i++)
        if (UBX_ITEMS[i].key == KEY_UART1OUTPROT_UBX) { ubxNew[i] = 1; ubxNewOn[i] = true; }
    StatusLog.println("[web] GPS receiver: switching the UBX output of UART1 on");
    ubxOp = UBXOP_UBXON;
    ubxLayers = 7;
    ubxWait = 0;
    return nullptr;
}

// ---------------------------------------------------------------- loop
void receiverLoop()
{
    const uint32_t now = millis();
    if (passThroughGPS) return;                           // AgIO's GPS tool has the receiver
    rxTxFlush(now);
    const bool talks = rxByteMs && now - rxByteMs < RX_TALKS_MS;
    if (rxByteMs && now - rxByteMs > RXID_SILENT_MS) rxSilentSeen = true;

    switch (rxIdState)
    {
        case RXID_WAIT:
            if (talks && now - rxTalkSinceMs >= RXID_TALK_MS && !rxTxLen)
            {
                rxSilentSeen = false;
                rxIdAsked = 0;
                rxIdState = RXID_ASK;
                rxAsk(rxSendsHpr());                       // sends HPR: Unicore first
            }
            break;
        case RXID_ASK:
            if (rxTxLen || now - rxIdAskMs < RXID_ANSWER_MS) break;
            if (rxIdAsked < RXID_QUESTIONS) { rxAsk(!rxIdLastUm); break; }
            rxIdState = RXID_NONE;
            if (rxSendsHpr())
            {
                rxKind = RX_UM982;                         // it sends HPR: Unicore, model not known
                StatusLog.println("[status] GPS receiver: no answer to VERSIONA, but it sends HPR: Unicore (UM982) assumed");
            }
            else
                StatusLog.println("[status] GPS receiver: not identified (no answer to VERSIONA or UBX-MON-VER) - a u-blox with its UBX output off on UART1?");
            break;
        default:                                           // identified, or no answer
            if (rxSilentSeen && talks)                     // silent, now talking again: maybe another receiver
            {
                receiverIdentifyAgain();
                rxTalkSinceMs = now;
            }
            break;
    }
    ubxOpLoop(now);
}

// ---------------------------------------------------------------- web page (GET /api/gps)
static const char* ubxResName(uint8_t r)
{
    switch (r)
    {
        case UBXRES_READ:        return "read";
        case UBXRES_READ_NONE:   return "read-none";
        case UBXRES_WRITE:       return "write";
        case UBXRES_WRITE_RAM:   return "write-ram";
        case UBXRES_WRITE_NAK:   return "write-nak";
        case UBXRES_WRITE_NONE:  return "write-none";
        case UBXRES_UBXON:       return "ubx-on";
        default:                 return "";
    }
}

void receiverJson(Json& j)
{
    const uint32_t now = millis();
    j.raw("{");
    j.bo("ok", true);
    j.str("kind", rxKind == RX_UBLOX ? "ublox" : rxKind == RX_UM982 ? "um982" : "");
    j.str("model", rxModel);
    j.str("fw", rxFw);
    j.str("prot", rxProt);
    j.str("hw", rxHw);
    j.str("id", rxIdState == RXID_WAIT ? "wait" : rxIdState == RXID_ASK ? "ask" : rxIdState == RXID_DONE ? "done" : "none");
    j.i("idAgo", rxIdMs ? (long)((now - rxIdMs) / 1000) : -1);
    j.bo("talks", rxByteMs && now - rxByteMs < RX_TALKS_MS);
    j.bo("hpr", rxSendsHpr());
    j.obj("ubx");
    j.str("op", ubxOp == UBXOP_READ ? "read" : ubxOp == UBXOP_WRITE ? "write" : ubxOp == UBXOP_UBXON ? "ubx-on" : "");
    j.u("seq", ubxSeq);
    j.str("res", ubxResName(ubxRes));
    j.i("readAgo", ubxReadMs ? (long)((now - ubxReadMs) / 1000) : -1);
    j.u("frames", ubxFrames);
    j.u("bad", ubxBad);
    j.obj("v");
    for (uint8_t i = 0; i < UBX_N; i++) if (ubxHave[i] == 1) j.i(UBX_ITEMS[i].name, ubxVal[i]);
    j.close();
    j.arr("na");
    for (uint8_t i = 0; i < UBX_N; i++) if (ubxHave[i] == 2) j.item(UBX_ITEMS[i].name);
    j.closeArr();
    j.close();
    j.raw("}");
}
