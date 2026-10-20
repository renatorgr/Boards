/* zReceiver.ino - which GPS receiver is on Serial7, and the u-blox and UM982 settings from the web page.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The u-blox messages and configuration keys are the ones of the u-blox ZED-F9P / ZED-X20P
   interface descriptions (UBX-MON-VER, UBX-CFG-VALGET, UBX-CFG-VALSET, UBX-ACK); the Unicore
   commands (VERSIONA, CONFIG, MASK, UNMASK, UNILOGLIST, MODE, UNLOG, SAVECONFIG and the log
   commands) are those of the Unicore N4 high precision commands and logs reference.
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
//     them or none). If the receiver refuses the flash, again to RAM + BBR. Not on this form: the UART1
//     baud rate and the UBX / NMEA protocols of UART1 (the Teensy would lose the receiver; the receiver
//     check of zRxCheck.ino reads them and sets the baud rate).
//   - UM982 settings (web page, GPS tab): read with CONFIG, MASK, UNILOGLIST and MODE, and written
//     one command at a time (each waits for its "$command,...,response: OK"), only what was changed
//     on the page, then SAVECONFIG (kept after a power off), then read again. Commands without a
//     port name act on the port they arrive on: the one wired to the Teensy. Never written from the
//     page: the port baud rates (CONFIG COMx) and FRESET (the Teensy would lose the receiver).
//   - Every question goes to the receiver between two RTCM messages of the corrections (zRtcm.h),
//     never in the middle of one. The receiver check (zRxCheck.ino) sends through the same queue,
//     takes the UBX answers meant for it (rxCkUbx), loads the UM982 AgOpenGPS commands through the
//     form's write (umWriteRaw), and has Serial7 to itself while it searches or changes the baud rate.
//     The page's reads and writes wait while it runs.
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
static uint8_t  rxTxBuf[RX_TX_MAX];
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
    if (!rxTxLen || rxCkOwns(RXCK_GPS)) return;           // (the receiver check searches its baud rate)
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

// One antenna, so no dual-antenna heading (the web page leaves its card out): a u-blox F9P / X20P
// without a second F9P on the GPS2 port, a Unicore receiver other than the UM982 (UM980, UM981), or a
// UM982 set to one antenna (Settings block, GPS tab). Not identified: not known, taken as two.
bool rxSingleAntenna()
{
    if (dualF9pOn())        return false;
    if (rxKind == RX_UBLOX) return true;
    if (rxKind == RX_UM982) return cfg.umSingle || (rxModel[0] && strcmp(rxModel, "UM982") != 0);
    return false;
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

static void umForget();

void receiverIdentifyAgain()
{
    ubxForget();                                           // the settings read belonged to the receiver before
    umForget();
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
            if (b == ubxCkB) { ubxFrames++; rxCkGood[RXCK_GPS]++; ubxFrame(ubxCls, ubxId, ubxBuf, ubxLen); }
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
    // the UART1 baud rate and its UBX / NMEA protocols: the receiver check's (zRxCheck.ino), not read here
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
    if (rxCkUbx(RXCK_GPS, cls, id, p, len)) return;       // an answer to the receiver check (zRxCheck.ino)
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
    if (ubxOp != UBXOP_NONE || rxCkBusy(RXCK_GPS)) return "busy";     // (the receiver check, zRxCheck.ino)
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
    if (ubxOp != UBXOP_NONE || rxCkBusy(RXCK_GPS)) return "busy";
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
    if (ubxOp != UBXOP_NONE || rxCkBusy(RXCK_GPS)) return "busy";
    ubxWriteBegin();
    for (uint8_t i = 0; i < UBX_N; i++)
        if (UBX_ITEMS[i].key == KEY_UART1OUTPROT_UBX) { ubxNew[i] = 1; ubxNewOn[i] = true; }
    StatusLog.println("[web] GPS receiver: switching the UBX output of UART1 on");
    ubxOp = UBXOP_UBXON;
    ubxLayers = 7;
    ubxWait = 0;
    return nullptr;
}

// ---------------------------------------------------------------- UM982 settings
// Read: the commands CONFIG ($CONFIG lines), MASK ($CONFIG,MASK lines), UNILOGLIST (#UNILOGLIST and
// "<" lines) and MODE (#MODE line), one after the other. Write: one command at a time, each
// waiting for its "$command,<command>,response: OK", then SAVECONFIG, then read again.
// Commands with no port name act on the port they arrive on: the one wired to the Teensy.
#define UM_ANSWER_MS    1500     // time for the first answer to a command
#define UM_LIST_MS      300      // a list is over this long after its last line
#define UM_LIST_MAX_MS  3000     // longest list
#define UM_SAVE_MS      3000     // time for the answer to SAVECONFIG
#define UM_LOGS         32       // UNILOGLIST entries kept
#define UM_CMDS         32       // commands in one write
#define UM_CMD_LEN      48
#define UM_FAILS        4        // refused commands kept (with the receiver's reason)
#define UM_TEENSY_BAUD  460800   // Serial7

enum UmItem : uint8_t { UM_RATE, UM_MODE, UM_ELEV, UM_RTKTO, UM_SBAS, UM_HDMODE, UM_HDLEN, UM_HDTOL, UM_HDOFF, UM_PTOFF,
                        UM_GGA, UM_VTG, UM_HPR, UM_RMC, UM_GSA, UM_GSV, UM_GST, UM_ZDA, UM_THS, UM_KSXT,
                        UM_GPS, UM_BDS, UM_GLO, UM_GAL, UM_QZSS, UM_N };
static const char* const UM_NAMES[UM_N] = {"rate", "mode", "elev", "rtkTo", "sbas", "hdMode", "hdLen", "hdTol", "hdOff", "ptOff",
                                           "gga", "vtg", "hpr", "rmc", "gsa", "gsv", "gst", "zda", "ths", "ksxt",
                                           "gps", "bds", "glo", "gal", "qzss"};
// the log command of each message item (UM_GGA ... UM_KSXT): NMEA ones are asked with "GP"
static const char* const UM_MSG_CMD[UM_KSXT - UM_GGA + 1] = {"GPGGA", "GPVTG", "GPHPR", "GPRMC", "GPGSA", "GPGSV", "GPGST", "GPZDA", "GPTHS", "KSXT"};
static const char* const UM_SYS[UM_QZSS - UM_GPS + 1]     = {"GPS", "BDS", "GLO", "GAL", "QZSS"};
static const char* const UM_HDMODES[5] = {"FIXLENGTH", "VARIABLELENGTH", "STATIC", "LOWDYNAMIC", "TRACTOR"};
static const char* const UM_MODES[4]   = {"MODE ROVER UAV", "MODE ROVER AUTOMOTIVE", "MODE ROVER SURVEY", "MODE ROVER SURVEY MOW"};
static const long UM_PERIODS[5] = {50, 100, 200, 500, 1000};      // ms: 20, 10, 5, 2, 1 Hz

enum UmOp  : uint8_t { UMOP_NONE = 0, UMOP_READ, UMOP_WRITE };
enum UmSt  : uint8_t { UMS_CONFIG = 0, UMS_MASK, UMS_LOGS, UMS_MODE, UMS_READ_END, UMS_CMD, UMS_SAVE };
enum UmRes : uint8_t { UMRES_NONE = 0, UMRES_READ, UMRES_READ_NONE, UMRES_WRITE, UMRES_WRITE_PART, UMRES_WRITE_NOSAVE, UMRES_WRITE_NONE };

struct UmLog { char m[14]; int8_t port; long ms; };    // port 0..2 = COM1..COM3, -1 other; ms 0 = on change / once

static double   umVal[UM_N];
static bool     umHave[UM_N];
static double   umNew[UM_N];
static bool     umNewOn[UM_N];
static long     umBaud[3];               // COM1..COM3 (0 = not reported)
static int8_t   umPort = -1;             // the port wired to the Teensy (0..2), -1 not known
static char     umNmea[12] = "";
static char     umModeText[40] = "";
static UmLog    umLogs[UM_LOGS];
static uint8_t  umLogN = 0;
static char     umCmd[UM_CMDS][UM_CMD_LEN];
static uint8_t  umCmdN = 0, umCmdI = 0;
static char     umFail[UM_FAILS][UM_CMD_LEN + 56];      // "<command>: <the receiver's reason>"
static uint8_t  umFailN = 0, umOkN = 0, umNoAnsN = 0;
static uint8_t  umOp = UMOP_NONE, umSt = UMS_CONFIG, umRes = UMRES_NONE;
static uint8_t  umWriteRes = UMRES_NONE; // the last write's result (kept when the read after it fails)
static bool     umAsked = false;         // the command of this step was queued
static uint8_t  umGot = 0;               // 1 "response: OK", 2 refused
static char     umGotWhy[48] = "";
static char     umWaitWord[16] = "";     // the first word of the command waited for
static bool     umAnyLine = false, umMode = false, umCfgAns = false, umMaskAns = false, umLogsAns = false;
static uint32_t umLastLineMs = 0, umEndMs = 0, umReadMs = 0, umSeq = 0;

// case-insensitive "s contains w"
static bool umHas(const char* s, const char* w)
{
    const size_t n = strlen(w);
    for (; *s; s++)
    {
        size_t k = 0;
        while (k < n && s[k] && toupper((unsigned char)s[k]) == toupper((unsigned char)w[k])) k++;
        if (k == n) return true;
    }
    return false;
}

// a number, the whole token
static bool umNum(const char* t, double& v)
{
    if (!t || !*t) return false;
    char* e;
    v = strtod(t, &e);
    return e != t && *e == '\0';
}

// up to max space-separated tokens of s (upper case) into buf
static uint8_t umTokens(const char* s, char* buf, size_t bufLen, char** tok, uint8_t max)
{
    size_t k = 0;
    for (; s[k] && s[k] != '*' && k + 1 < bufLen; k++) buf[k] = (char)toupper((unsigned char)s[k]);
    buf[k] = '\0';
    uint8_t n = 0;
    for (char* p = strtok(buf, " \t"); p && n < max; p = strtok(nullptr, " \t")) tok[n++] = p;
    return n;
}

static void umSet(uint8_t i, double v) { umVal[i] = v; umHave[i] = true; }

// "$CONFIG,<item>,<command as it would be typed>*cs"
static void umConfigLine(const char* l)
{
    const char* a = strchr(l + 8, ',');
    if (!a) return;
    char buf[96], *t[8];
    const uint8_t n = umTokens(a + 1, buf, sizeof(buf), t, 8);
    double v;
    if (n >= 3 && !strcmp(t[0], "CONFIG"))
    {
        if (!strncmp(t[1], "COM", 3) && t[1][3] >= '1' && t[1][3] <= '3' && !t[1][4] && umNum(t[2], v)) umBaud[t[1][3] - '1'] = (long)v;
        else if (!strcmp(t[1], "HEADING"))
        {
            for (uint8_t k = 0; k < 5; k++) if (!strcmp(t[2], UM_HDMODES[k])) umSet(UM_HDMODE, k);
            if (!strcmp(t[2], "OFFSET") && n >= 4 && umNum(t[3], v))
            {
                umSet(UM_HDOFF, v);
                umSet(UM_PTOFF, n >= 5 && umNum(t[4], v) ? v : 0);
            }
            if (!strcmp(t[2], "LENGTH"))
            {
                if (n >= 4 && umNum(t[3], v)) { umSet(UM_HDLEN, v); if (n >= 5 && umNum(t[4], v)) umSet(UM_HDTOL, v); }
                else umSet(UM_HDLEN, 0);                        // no length: automatic
            }
        }
        else if (!strcmp(t[1], "RTK") && n >= 4 && !strcmp(t[2], "TIMEOUT") && umNum(t[3], v)) umSet(UM_RTKTO, v);
        else if (!strcmp(t[1], "SBAS"))
        {
            if (!strcmp(t[2], "DISABLE")) umSet(UM_SBAS, 0);
            else if (!strcmp(t[2], "ENABLE")) umSet(UM_SBAS, n < 4 || !strcmp(t[3], "AUTO") ? 1 : !strcmp(t[3], "EGNOS") ? 2 : 3);
        }
        else if (!strcmp(t[1], "NMEAVERSION") || !strcmp(t[1], "NMEA0183")) rxCopyText(umNmea, sizeof(umNmea), t[2], 16);
    }
    else if (n == 2 && !strcmp(t[0], "MASK"))
    {
        umMaskAns = true;
        if (umNum(t[1], v)) { if (!umHave[UM_ELEV]) umSet(UM_ELEV, v); }     // the first one: all systems
        else for (uint8_t k = 0; k <= UM_QZSS - UM_GPS; k++) if (!strcmp(t[1], UM_SYS[k])) umSet(UM_GPS + k, 0);
    }
}

// "<  GPGGA COM1 1" (and "<  3", the count): message, port, trigger (optional), period in seconds
static void umLogLine(const char* s)
{
    char buf[96], *t[8];
    const uint8_t n = umTokens(s, buf, sizeof(buf), t, 8);
    if (n < 2 || umLogN >= UM_LOGS) return;
    UmLog e;
    e.m[0] = '\0'; e.port = -1; e.ms = 0;
    bool havePort = false, haveMs = false;
    double v;
    for (uint8_t k = 0; k < n; k++)
    {
        if (!strncmp(t[k], "COM", 3) && t[k][3] >= '1' && t[k][3] <= '3' && !t[k][4]) { e.port = t[k][3] - '1'; havePort = true; }
        else if (!strncmp(t[k], "COM", 3) || !strncmp(t[k], "USB", 3) || !strncmp(t[k], "ICOM", 4)) havePort = true;
        else if (umNum(t[k], v)) { if (!haveMs && e.m[0]) { e.ms = (long)(v * 1000.0 + 0.5); haveMs = true; } }   // the period: the first number after the name
        else if (!e.m[0] && strncmp(t[k], "ON", 2)) rxCopyText(e.m, sizeof(e.m), t[k], sizeof(e.m));   // not ONTIME / ONCHANGED / ONCE
    }
    if (e.m[0] && havePort) umLogs[umLogN++] = e;
}

// "#MODE,<header>;MODE ROVER SURVEY,*1B"
static void umModeLine(const char* l)
{
    const char* s = strchr(l, ';');
    if (!s) return;
    char m[40];
    size_t k = 0;
    for (s++; *s && *s != ',' && *s != '*' && k + 1 < sizeof(m); s++) m[k++] = (char)toupper((unsigned char)*s);
    while (k && m[k - 1] == ' ') k--;
    m[k] = '\0';
    rxCopyText(umModeText, sizeof(umModeText), m, sizeof(m));
    umMode = true;
    if (!strcmp(m, "MODE ROVER") || !strcmp(m, "MODE ROVER UAV DEFAULT")) { umSet(UM_MODE, 0); return; }   // the UM982's default: UAV
    if (!strcmp(m, "MODE ROVER AUTOMOTIVE DEFAULT")) { umSet(UM_MODE, 1); return; }
    if (!strcmp(m, "MODE ROVER SURVEY DEFAULT"))     { umSet(UM_MODE, 2); return; }
    for (uint8_t i = 0; i < 4; i++) if (!strcmp(m, UM_MODES[i])) { umSet(UM_MODE, i); return; }
}

// the Unicore answers ($command, $CONFIG, #UNILOGLIST, #MODE) are not sentence types of their own
bool umIsReply(const char* l)
{
    return !strncmp(l, "$command,", 9) || !strncmp(l, "$CONFIG,", 8) || !strncmp(l, "#UNILOGLIST", 11) || !strncmp(l, "#MODE,", 6);
}

// while the page reads or writes the UM982, its answers stay out of the console (unless a command was typed there)
bool umQuiet()
{
    return umOp != UMOP_NONE || (umEndMs && millis() - umEndMs < 1000);
}

// every text line from the receiver (zHandlers.ino, rawNmeaTee)
void umLine(const char* l)
{
    if (umOp == UMOP_NONE) return;
    const uint32_t now = millis();
    if (!strncmp(l, "$command,", 9))
    {
        if (!umWaitWord[0] || umGot || !umHas(l + 9, umWaitWord)) return;   // not the answer to ours
        const char* r = strstr(l, "response:");
        if (r) { r += 9; while (*r == ' ') r++; }
        umGot = r && !strncmp(r, "OK", 2) ? 1 : 2;
        size_t k = 0;
        if (r) for (; r[k] && r[k] != '*' && k + 1 < sizeof(umGotWhy); k++) umGotWhy[k] = r[k];
        umGotWhy[k] = '\0';
        if (umGot == 1 && umSt == UMS_CONFIG) umCfgAns = true;
        if (umGot == 1 && umSt == UMS_MASK)   umMaskAns = true;
        if (umGot == 1 && umSt == UMS_LOGS)   umLogsAns = true;
        return;
    }
    if (umOp != UMOP_READ) return;
    if (!strncmp(l, "$CONFIG,", 8))
    {
        umConfigLine(l);
        rxCkUmConfig(l);                                   // the AgOpenGPS commands it has (zRxCheck.ino)
        if (umSt == UMS_CONFIG) umCfgAns = true;
        umAnyLine = true;
        umLastLineMs = now;
        return;
    }
    if (umSt == UMS_LOGS && (!strncmp(l, "#UNILOGLIST", 11) || l[0] == '<'))
    {
        if (l[0] == '<') umLogLine(l + 1);
        umLogsAns = true;
        umAnyLine = true;
        umLastLineMs = now;
        return;
    }
    if (umSt == UMS_MODE && !strncmp(l, "#MODE,", 6)) { umModeLine(l); umLastLineMs = now; }
}

static bool umSend(const char* cmd)
{
    char b[UM_CMD_LEN + 4];
    const int n = snprintf(b, sizeof(b), "%s\r\n", cmd);
    if (n <= 0 || n >= (int)sizeof(b) || !rxTxQueue((const uint8_t*)b, (uint16_t)n)) return false;
    size_t k = 0;
    for (; cmd[k] && cmd[k] != ' ' && k + 1 < sizeof(umWaitWord); k++) umWaitWord[k] = cmd[k];
    umWaitWord[k] = '\0';
    umGot = 0;
    umGotWhy[0] = '\0';
    umAnyLine = false;
    umAsked = true;
    return true;
}

static void umReadBegin()
{
    for (uint8_t i = 0; i < UM_N; i++) umHave[i] = false;
    for (uint8_t k = 0; k < 3; k++) umBaud[k] = 0;
    umNmea[0] = umModeText[0] = '\0';
    umLogN = 0;
    umPort = -1;
    umMode = umCfgAns = umMaskAns = umLogsAns = false;
    rxCkUmBegin();
    umOp = UMOP_READ;
    umSt = UMS_CONFIG;
    umAsked = false;
}

// what was read, put together: the port wired to the Teensy, the messages on it, the constellations
static void umReadDone()
{
    uint8_t fast = 0;
    int8_t  p = -1;
    for (uint8_t k = 0; k < 3; k++) if (umBaud[k] == UM_TEENSY_BAUD) { fast++; p = k; }
    umPort = fast == 1 ? p : -1;
    if (umPort < 0)                                        // more than one (or none) at 460800: the one sending GGA
    {
        int8_t gga = -1, ggaFast = -1;
        for (uint8_t e = 0; e < umLogN; e++)
            if (!strcmp(umLogs[e].m, "GPGGA") || !strcmp(umLogs[e].m, "GNGGA"))
            {
                const int8_t q = umLogs[e].port;
                if (q < 0) continue;
                if (gga < 0) gga = q;
                if (ggaFast < 0 && umBaud[q] == UM_TEENSY_BAUD) ggaFast = q;
            }
        umPort = ggaFast >= 0 ? ggaFast : gga;
    }
    if (umLogsAns && umPort >= 0)
        for (uint8_t i = UM_GGA; i <= UM_KSXT; i++)
        {
            const char* c = UM_MSG_CMD[i - UM_GGA];
            double ms = 0;
            for (uint8_t e = 0; e < umLogN; e++)
            {
                if (umLogs[e].port != umPort) continue;
                const char* m = umLogs[e].m;
                const bool same = i == UM_KSXT ? (!strcmp(m, "KSXT") || !strcmp(m, "KSXTA"))
                                               : (strlen(m) == 5 && m[0] == 'G' && !strcmp(m + 2, c + 2));
                if (same) ms = umLogs[e].ms > 0 ? umLogs[e].ms : 1000;      // on change: shown as on
            }
            umSet(i, ms);
        }
    if (umHave[UM_GGA]) umSet(UM_RATE, umVal[UM_GGA]);
    if (umMaskAns)
        for (uint8_t i = UM_GPS; i <= UM_QZSS; i++) if (!umHave[i]) umSet(i, 1);    // not masked: tracked
}

static void umForget()
{
    for (uint8_t i = 0; i < UM_N; i++) umHave[i] = false;
    umOp = UMOP_NONE;
    umRes = UMRES_NONE;
    umReadMs = 0;
    umLogN = umFailN = 0;
    umPort = -1;
    umWaitWord[0] = '\0';
    umSeq++;
}

static void umEnd(uint8_t res)
{
    umOp = UMOP_NONE;
    umRes = res;
    umWaitWord[0] = '\0';
    umEndMs = millis() | 1;
    umSeq++;
}

static void umOpLoop(uint32_t now)
{
    if (umOp == UMOP_NONE || rxTxLen) return;             // nothing to do, or the command is not out yet
    if (umOp == UMOP_READ)
    {
        static const char* const Q[4] = {"CONFIG", "MASK", "UNILOGLIST", "MODE"};
        if (umSt < UMS_READ_END)
        {
            if (!umAsked) { umSend(Q[umSt]); return; }
            const uint32_t since = now - rxTxSentMs;
            bool over;
            if (umSt == UMS_MODE) over = umMode || umGot == 2 || since > UM_ANSWER_MS;
            else over = umGot == 2 || (umAnyLine && now - umLastLineMs > UM_LIST_MS) || (!umAnyLine && since > UM_ANSWER_MS) || since > UM_LIST_MAX_MS;
            if (!over) return;
            if (umSt == UMS_CONFIG && !umCfgAns && !umGot)
            {
                StatusLog.println("[status] GPS receiver: no answer to CONFIG (UM982 settings not read)");
                umEnd(UMRES_READ_NONE);
                return;
            }
            umSt++;
            umAsked = false;
            return;
        }
        umReadDone();
        umReadMs = now | 1;
        umEnd(umRes == UMRES_WRITE || umRes == UMRES_WRITE_PART || umRes == UMRES_WRITE_NOSAVE ? umRes : UMRES_READ);
        return;
    }
    // write: the commands one at a time, then SAVECONFIG, then read back
    if (umSt == UMS_CMD)
    {
        if (umAsked)
        {
            if (!umGot && now - rxTxSentMs < UM_ANSWER_MS) return;
            if (umGot == 1) umOkN++;
            else
            {
                if (!umGot) umNoAnsN++;
                if (umFailN < UM_FAILS)
                    snprintf(umFail[umFailN++], sizeof(umFail[0]), "%s: %s", umCmd[umCmdI], umGot ? (umGotWhy[0] ? umGotWhy : "refused") : "no answer");
                StatusLog.line("[status] GPS receiver: UM982 %s %s: %s", umGot ? "refused" : "did not answer", umCmd[umCmdI], umGot ? umGotWhy : "-");
            }
            umCmdI++;
            umAsked = false;
        }
        if (umCmdI < umCmdN) { umSend(umCmd[umCmdI]); return; }
        if (!umOkN)
        {
            StatusLog.line("[status] GPS receiver: UM982 settings NOT written (%s)", umNoAnsN == umCmdN ? "no answer" : "every command refused");
            umWriteRes = UMRES_WRITE_NONE;
            umEnd(UMRES_WRITE_NONE);
            return;
        }
        umSt = UMS_SAVE;
        umSend("SAVECONFIG");
        return;
    }
    if (umSt == UMS_SAVE)
    {
        if (!umGot && now - rxTxSentMs < UM_SAVE_MS) return;
        const uint8_t res = umGot != 1 ? UMRES_WRITE_NOSAVE : umFailN ? UMRES_WRITE_PART : UMRES_WRITE;
        if (umGot == 1) StatusLog.line("[status] GPS receiver: UM982 settings written (%u of %u commands) and saved (SAVECONFIG)", umOkN, umCmdN);
        else            StatusLog.line("[status] GPS receiver: UM982 settings written (%u of %u commands) but SAVECONFIG %s: lost at the next power off",
                                       umOkN, umCmdN, umGot ? "refused" : "not answered");
        umRes = res;
        umWriteRes = res;
        umSeq++;
        umReadBegin();                                     // read them back (keeps umRes)
    }
}

const char* umStartRead()
{
    if (rxKind != RX_UM982)                   return "not a UM982 receiver";
    if (umOp != UMOP_NONE || rxIdState == RXID_ASK || ubxOp != UBXOP_NONE) return "busy";
    umRes = UMRES_NONE;
    umReadBegin();
    return nullptr;
}

uint8_t     umItemCount()          { return UM_N; }
const char* umItemName(uint8_t i)  { return i < UM_N ? UM_NAMES[i] : ""; }

void umWriteBegin()
{
    for (uint8_t i = 0; i < UM_N; i++) umNewOn[i] = false;
}

static bool umIsPeriod(double v)
{
    for (long p : UM_PERIODS) if (v == p) return true;
    return false;
}

const char* umWriteAdd(uint8_t i, double v)
{
    if (i >= UM_N || i == UM_GGA || i == UM_VTG || i == UM_HPR) return "setting cannot be written";
    bool ok;
    switch (i)
    {
        case UM_RATE:   ok = umIsPeriod(v); break;
        case UM_MODE:   ok = v == (int)v && v >= 0 && v <= 3; break;
        case UM_ELEV:   ok = v == (int)v && v >= 0 && v <= 60; break;
        case UM_RTKTO:  ok = v == (int)v && v >= 1 && v <= 1800; break;
        case UM_SBAS:   ok = v == (int)v && v >= 0 && v <= 2; break;
        case UM_HDMODE: ok = v == (int)v && v >= 0 && v <= 4; break;
        case UM_HDLEN:  ok = v == (int)v && (v == 0 || (v >= 10 && v <= 20000)); break;
        case UM_HDTOL:  ok = v == (int)v && v >= 1 && v <= 1000; break;
        case UM_HDOFF:  ok = v >= -180 && v <= 180; break;
        case UM_PTOFF:  ok = v >= -90 && v <= 90; break;
        default:
            if (i >= UM_GPS) ok = v == 0 || v == 1;
            else             ok = v == 0 || umIsPeriod(v);         // messages: 0 = off
            break;
    }
    if (!ok) return "invalid value";
    umNew[i]   = v;
    umNewOn[i] = true;
    return nullptr;
}

// a number as it is typed: no trailing zeros ("90", "-1.5", "0.05")
static void umFmt(char* out, size_t n, double v)
{
    snprintf(out, n, "%.2f", v);
    char* d = strchr(out, '.');
    if (!d) return;
    char* e = out + strlen(out) - 1;
    while (e > d && *e == '0') *e-- = '\0';
    if (e == d) *e = '\0';
}

static void umAdd(const char* fmt, ...)
{
    if (umCmdN >= UM_CMDS) return;
    va_list a;
    va_start(a, fmt);
    vsnprintf(umCmd[umCmdN++], UM_CMD_LEN, fmt, a);
    va_end(a);
}

const char* umWriteGo()
{
    if (rxKind != RX_UM982)                   return "not a UM982 receiver";
    if (umOp != UMOP_NONE || rxIdState == RXID_ASK || ubxOp != UBXOP_NONE || rxCkBusy(RXCK_GPS)) return "busy";
    auto cur = [](uint8_t i, double dflt) { return umNewOn[i] ? umNew[i] : umHave[i] ? umVal[i] : dflt; };
    // never every constellation masked (GPS, GLONASS, Galileo, BeiDou)
    if (!cur(UM_GPS, 1) && !cur(UM_GLO, 1) && !cur(UM_GAL, 1) && !cur(UM_BDS, 1)) return "at least one of GPS, GLONASS, Galileo and BeiDou must stay on";
    char a[16], b[16];
    umCmdN = 0;
    for (uint8_t i = UM_GPS; i <= UM_QZSS; i++)
        if (umNewOn[i]) umAdd("%s %s", umNew[i] ? "UNMASK" : "MASK", UM_SYS[i - UM_GPS]);
    if (umNewOn[UM_ELEV])   umAdd("MASK %d", (int)umNew[UM_ELEV]);
    if (umNewOn[UM_SBAS])   umAdd("CONFIG SBAS %s", umNew[UM_SBAS] == 0 ? "DISABLE" : umNew[UM_SBAS] == 1 ? "ENABLE AUTO" : "ENABLE EGNOS");
    if (umNewOn[UM_RTKTO])  umAdd("CONFIG RTK TIMEOUT %d", (int)umNew[UM_RTKTO]);
    if (umNewOn[UM_HDMODE]) umAdd("CONFIG HEADING %s", UM_HDMODES[(int)umNew[UM_HDMODE]]);
    if (umNewOn[UM_HDLEN] || umNewOn[UM_HDTOL])
    {
        const double len = cur(UM_HDLEN, 0);
        if (len <= 0) umAdd("CONFIG HEADING LENGTH");
        else          umAdd("CONFIG HEADING LENGTH %d %d", (int)len, (int)cur(UM_HDTOL, 3));
    }
    if (umNewOn[UM_HDOFF] || umNewOn[UM_PTOFF])
    {
        umFmt(a, sizeof(a), cur(UM_HDOFF, 0));
        umFmt(b, sizeof(b), cur(UM_PTOFF, 0));
        umAdd("CONFIG HEADING OFFSET %s %s", a, b);
    }
    if (umNewOn[UM_MODE])   umAdd("%s", UM_MODES[(int)umNew[UM_MODE]]);
    for (uint8_t i = UM_RMC; i <= UM_KSXT; i++)
    {
        if (!umNewOn[i]) continue;
        if (umNew[i] > 0) { umFmt(a, sizeof(a), umNew[i] / 1000.0); umAdd("%s %s", UM_MSG_CMD[i - UM_GGA], a); }
        else umAdd("UNLOG %s", UM_MSG_CMD[i - UM_GGA]);
    }
    if (umNewOn[UM_RATE])
    {
        umFmt(a, sizeof(a), umNew[UM_RATE] / 1000.0);
        for (uint8_t i = UM_GGA; i <= UM_HPR; i++) umAdd("%s %s", UM_MSG_CMD[i - UM_GGA], a);
    }
    if (!umCmdN) return "nothing to change";
    for (uint8_t k = 0; k < umCmdN; k++) StatusLog.line("[web] UM982 <- %s", umCmd[k]);
    umCmdI = umOkN = umFailN = umNoAnsN = 0;
    umRes = umWriteRes = UMRES_NONE;
    umOp = UMOP_WRITE;
    umSt = UMS_CMD;
    umAsked = false;
    return nullptr;
}

// The AgOpenGPS commands (receiver check, zRxCheck.ino: ConfigUM982D.txt / ConfigUM982S.txt of
// AOGConfigOMatic, the log commands without their port): written like the page's, one at a time,
// then SAVECONFIG, then read again. Only the refused ones get a [status] line of their own.
const char* umWriteRaw(const char* const* cmds, uint8_t n, const char* what)
{
    if (rxKind != RX_UM982)                   return "not a UM982 receiver";
    if (umOp != UMOP_NONE || rxIdState == RXID_ASK || ubxOp != UBXOP_NONE) return "busy";
    umCmdN = 0;
    for (uint8_t k = 0; k < n; k++) umAdd("%s", cmds[k]);
    if (!umCmdN) return "nothing to change";
    StatusLog.line("[web] UM982: loading the AgOpenGPS settings (AOGConfigOMatic %s: %u commands, then SAVECONFIG)", what, umCmdN);
    umCmdI = umOkN = umFailN = umNoAnsN = 0;
    umRes = umWriteRes = UMRES_NONE;
    umOp = UMOP_WRITE;
    umSt = UMS_CMD;
    umAsked = false;
    return nullptr;
}

// ---------------------------------------------------------------- loop
void receiverLoop()
{
    const uint32_t now = millis();
    if (passThroughGPS)                                   // AgIO's GPS tool has the receiver
    {
        if (umOp != UMOP_NONE) umEnd(UMRES_NONE);
        return;
    }
    rxTxFlush(now);
    const bool talks = rxByteMs && now - rxByteMs < RX_TALKS_MS;
    if (rxByteMs && now - rxByteMs > RXID_SILENT_MS) rxSilentSeen = true;

    switch (rxIdState)
    {
        case RXID_WAIT:
            if (talks && now - rxTalkSinceMs >= RXID_TALK_MS && !rxTxLen && !rxCkBusy(RXCK_GPS))
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
    umOpLoop(now);
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

static const char* umResName(uint8_t r)
{
    switch (r)
    {
        case UMRES_READ:         return "read";
        case UMRES_READ_NONE:    return "read-none";
        case UMRES_WRITE:        return "write";
        case UMRES_WRITE_PART:   return "write-part";
        case UMRES_WRITE_NOSAVE: return "write-nosave";
        case UMRES_WRITE_NONE:   return "write-none";
        default:                 return "";
    }
}

static void umJson(Json& j)
{
    const uint32_t now = millis();
    char t[8];
    j.obj("um");
    j.str("op", umOp == UMOP_READ ? "read" : umOp == UMOP_WRITE ? "write" : "");
    j.u("seq", umSeq);
    j.str("res", umResName(umRes));
    j.i("readAgo", umReadMs ? (long)((now - umReadMs) / 1000) : -1);
    j.i("port", umPort + 1);                               // 1..3 = COM1..COM3, 0 = not known
    j.obj("baud");
    for (uint8_t k = 0; k < 3; k++) if (umBaud[k]) { snprintf(t, sizeof(t), "COM%u", k + 1); j.i(t, umBaud[k]); }
    j.close();
    j.str("nmea", umNmea);
    j.obj("v");
    for (uint8_t i = 0; i < UM_N; i++)
        if (umHave[i]) { if (i == UM_HDOFF || i == UM_PTOFF) j.num(UM_NAMES[i], umVal[i], 2); else j.i(UM_NAMES[i], (long)umVal[i]); }
    j.close();
    j.u("cmds", umCmdN);
    j.u("okN", umOkN);
    j.arr("fail");
    for (uint8_t k = 0; k < umFailN; k++) j.item(umFail[k]);
    j.closeArr();
    j.close();
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
    umJson(j);
    rxCkJson(j);                                           // the receiver check (zRxCheck.ino)
    j.raw("}");
}
