/* zRxCheck.ino - receiver check (web page, GPS tab): is the data from the receiver valid at 460800
   baud, what baud rate the receiver is really set to, does it have the settings the Teensy needs, and the
   AgOpenGPS settings of AOGConfigOMatic, loaded with one button.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The AgOpenGPS settings (zRxConfig.h) are those of AOGConfigOMatic by lansalot
   (github.com/lansalot/AOGConfigOMatic, GNU GPL v3; the F9P configuration by Jaap van den Handel),
   sent the way that program sends them: each line of its u-center file as one UBX-CFG-VALSET, the
   UM982 commands one at a time, then SAVECONFIG. Those u-blox files are made for firmware HPG 1.13.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zRxCheck.ino
// Two ports: the GPS receiver (Serial7) and, with the GPS2 port set to it, the second F9P of a dual F9P
// (Serial2, the "Dual - Left - RelPos" one).
//   - Data at 460800, judged every 5 s: bytes, and whole sentences / UBX frames with a valid checksum.
//     Bytes with nothing valid in them for 10 s: the receiver is at another baud rate (or its settings
//     or the wiring are wrong): a [status] line, a warning on Status, and the card says so. A port
//     with nothing on it and one with a silent receiver look the same.
//   - At start-up (once: the receivers are not unplugged with the board on): no valid data at 460800
//     in the first 5 s (or nothing at all in 15 s), so the Teensy looks for the receiver's baud rate. It
//     listens at 460800, 115200, 38400, 9600, 230400, 921600, 57600 and 19200 (1.3 s each), sending
//     UBX-MON-VER (and VERSIONA on Serial7) at each, until valid sentences or frames come. Then the
//     page shows "set to 460800 and save" (button): u-blox UBX-CFG-VALSET CFG-UART1-BAUDRATE to all
//     its layers (RAM, BBR and flash); Unicore CONFIG COM1 / COM2 / COM3 460800 and, back at 460800,
//     SAVECONFIG. Only with autosteer off: the GPS data stops in the meantime.
//   - What the Teensy needs (u-blox, read with UBX-CFG-VALGET from the RAM layer): UBX and RTCM3 in and
//     UBX and NMEA out on UART1, GGA and VTG on UART1, the rate, the UART1 speed; for the first F9P of a
//     dual F9P also the RTCM3 out on UART2 to the second one; for the second F9P RELPOSNED on UART1 and
//     RTCM3 in on UART2. The expected values are those of the AgOpenGPS settings. Checked automatically once
//     the receiver is identified (at start-up); "compare with AOGConfigOMatic" (button) checks it again
//     and compares all the AgOpenGPS settings (about 1200 keys, F9P only). UM982: the GPS tab's UM982 read (CONFIG, MASK, UNILOGLIST, MODE),
//     then the card shows GGA / VTG / HPR on the Teensy's port and compares the AgOpenGPS commands. On
//     every receiver: the GGA / VTG / HPR (RELPOSNED) rates that really arrive.
//   - Load the AgOpenGPS settings (button, autosteer off): u-blox Single.txt, or DualPosition.txt with
//     the GPS2 port set to the second F9P, and DualRelPos.txt for the second F9P; each message is a
//     UBX-CFG-VALSET to RAM, BBR and flash (RAM and BBR if the flash refuses), then everything is
//     checked again. UM982: ConfigUM982D.txt (two antennas) or ConfigUM982S.txt (one), one command at a
//     time, then SAVECONFIG (zReceiver.ino), then read again.
// One operation at a time; on Serial7 it waits for the GPS tab's own reads and writes, and they wait
// for it. Every message to the receiver on Serial7 goes between two RTCM messages of the corrections
// (zReceiver.ino's queue), except while a baud rate search or change has the port.

#include "zRxCheck.h"
#include "zRxConfig.h"

// The operations, the web page's answer and the comparisons run from flash (FLASHMEM), not from the
// Teensy's fast RAM (ITCM): they are rare and not time-critical, and the RAM stays free for the rest.

#define CK_WINDOW_MS       5000      // the data at 460800 is judged over this window
#define CK_JUNK_BYTES      64        // bytes in a window with nothing valid in them: junk
#define CK_JUNK_WINDOWS    2         // windows in a row before the [status] line
#define CK_LISTEN_MS       1300      // baud rate search: listening at each speed
#define CK_GOOD_FOUND      2         // valid sentences / frames that say "this is the speed"
#define CK_GET_MS          600       // time for the answer to a UBX-CFG-VALGET
#define CK_SET_MS          2500      // time for the answer to a UBX-CFG-VALSET (the flash takes a moment)
#define CK_MONVER_MS       1500      // time for the answer to UBX-MON-VER
#define CK_BAUD_OK_MS      4000      // after a baud rate change: valid data at 460800 within this time
#define CK_SAVE_MS         2000      // Unicore: time for the answer to SAVECONFIG
#define CK_BOOT_MS         5000      // after start-up: the data from the receivers judged after this long,
#define CK_BOOT_WAIT_MS    15000     //   and a receiver that sends nothing at all searched for after this long
#define CK_TEENSY_BAUD     460800
#define CK_HPG             113       // HPG 1.13: the firmware the AgOpenGPS u-blox settings are made for
#define CK_LINE_MAX        100

uint32_t rxCkBytes[2] = {0, 0};
uint32_t rxCkGood[2]  = {0, 0};

static const uint32_t CK_BAUDS[] = {460800, 115200, 38400, 9600, 230400, 921600, 57600, 19200};
#define CK_BAUD_N ((uint8_t)(sizeof(CK_BAUDS) / sizeof(CK_BAUDS[0])))

// ---------------------------------------------------------------- what the Teensy needs (u-blox)
// Roles: R_S Single (alone on Serial7), R_D Dual - Right - Position (Serial7, second F9P on the GPS2
// port), R_R Dual - Left - RelPos (the second F9P). The expected value is the one in the AgOpenGPS settings.
#define R_S 1
#define R_D 2
#define R_R 4
struct CkEss { const char* id; uint32_t key; uint8_t roles; };
static const CkEss CK_ESS[] = {
    {"ubxIn",    0x10730001UL, R_S | R_D | R_R},   // CFG-UART1INPROT-UBX: the Teensy's queries
    {"rtcmIn",   0x10730004UL, R_S | R_D},         // CFG-UART1INPROT-RTCM3X: the corrections from NTRIP / radio
    {"ubxOut",   0x10740001UL, R_S | R_D | R_R},   // CFG-UART1OUTPROT-UBX: its answers (and the RELPOSNED)
    {"nmeaOut",  0x10740002UL, R_S | R_D},         // CFG-UART1OUTPROT-NMEA
    {"gga",      0x209100BBUL, R_S | R_D},         // CFG-MSGOUT-NMEA_ID_GGA_UART1
    {"vtg",      0x209100B1UL, R_S | R_D},         // CFG-MSGOUT-NMEA_ID_VTG_UART1
    {"rate",     0x30210001UL, R_S | R_D | R_R},   // CFG-RATE-MEAS (ms)
    {"navRate",  0x30210002UL, R_S | R_D | R_R},   // CFG-RATE-NAV
    {"baud1",    0x40520001UL, R_S | R_D | R_R},   // CFG-UART1-BAUDRATE
    {"rtcmOut2", 0x10760004UL, R_D},               // CFG-UART2OUTPROT-RTCM3X: corrections to the second F9P
    {"baud2",    0x40530001UL, R_D | R_R},         // CFG-UART2-BAUDRATE (the same on both F9P)
    {"r1005",    0x209102BFUL, R_D},               // CFG-MSGOUT-RTCM_3X_TYPE..._UART2
    {"r1074",    0x20910360UL, R_D},
    {"r1084",    0x20910365UL, R_D},
    {"r1094",    0x2091036AUL, R_D},
    {"r1230",    0x20910305UL, R_D},
    {"r4072",    0x20910300UL, R_D},
    {"relpos",   0x2091008EUL, R_R},               // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1
    {"rtcmIn2",  0x10750004UL, R_R},               // CFG-UART2INPROT-RTCM3X: corrections from the first F9P
};
#define CK_ESS_N ((uint8_t)(sizeof(CK_ESS) / sizeof(CK_ESS[0])))

enum CkHealth : uint8_t { CKH_WAIT = 0, CKH_NONE, CKH_OK, CKH_JUNK };
enum CkOp     : uint8_t { CKOP_NONE = 0, CKOP_SCAN, CKOP_BAUD, CKOP_CHECK, CKOP_LOAD };
enum CkRes    : uint8_t { CKR_NONE = 0, CKR_FOUND, CKR_NOTFOUND, CKR_BAUD_OK, CKR_BAUD_FAIL, CKR_CHECKED, CKR_NOANSWER,
                          CKR_LOADED, CKR_LOAD_PART, CKR_LOAD_FAIL, CKR_STOPPED };
enum CkKind   : uint8_t { CKK_UNKNOWN = 0, CKK_UBLOX, CKK_UNICORE, CKK_NMEA };
enum CkBoot   : uint8_t { CKB_WAIT = 0, CKB_OK, CKB_SCAN, CKB_OFF };   // the start-up check of a port
#define CK_ROLE_UM   255                   // the UM982 (its settings are read by zReceiver.ino)
#define CK_LOAD_UMD  10                    // loadRole: ConfigUM982D.txt
#define CK_LOAD_UMS  11                    // loadRole: ConfigUM982S.txt

struct CkPort
{
    uint32_t b0, g0, wb, wg;              // the window: counters at its start; the last window's bytes and valid ones
    uint8_t  health, junkN;
    bool     junkSaid;
    uint16_t hz[3];                       // Hz x 10 in the last window: port 0 GGA, VTG, HPR; port 1 RELPOSNED
    uint32_t relpos0;
    char     model[24], fw[40];           // port 1: from its UBX-MON-VER (port 0: zReceiver.ino)
    int64_t  ess[CK_ESS_N];
    uint8_t  essHave[CK_ESS_N];           // 0 not read, 1 read, 2 the receiver does not know the key
    uint8_t  essRole;                     // RXCFG_ROLE_..., CK_ROLE_UM
    uint32_t essMs;                       // when it was checked (0 = not yet)
    uint16_t cmpSame, cmpDiff, cmpUnk;
    uint8_t  cmpRole;
    uint32_t cmpMs;
    uint32_t found;                       // the baud rate search: the speed found (0 none)
    uint8_t  foundKind, scanRes, baudRes, chkRes, loadRes;
    uint16_t loadOk, loadRam, loadNak, loadNone, loadTotal;
    uint8_t  loadRole;
    uint32_t loadMs;
    uint32_t autoFor;                     // port 0: the identification the automatic check was made for
    bool     autoDone;                    // port 1: checked automatically since its data arrived
    uint8_t  boot;                        // CkBoot: valid data at 460800 after start-up, or its baud rate searched for
};
static CkPort ck[2];

static uint8_t  ckOp = CKOP_NONE, ckPort = 0, ckStep = 0, ckRole = 0, ckTries = 0, ckMsg = 0, ckKeyI = 0;
static uint8_t  ckWait = 0;               // 1 VALGET answer, 2 ACK of a VALSET, 3 MON-VER
static uint8_t  ckGot = 0;                // 1 answer / ACK, 2 NAK
static uint8_t  ckLayers = 7;
static uint8_t  ckBaudI = 0, ckProbes = 0, ckNoneRow = 0;
static bool     ckFull = false, ckChain = false, ckSaved = false, ckRefreshForm = false;
static uint32_t ckT0 = 0, ckSentMs = 0;
static uint16_t ckTypeCnt[3] = {0, 0, 0};   // GGA, VTG, HPR in the current window (port 0)
static uint32_t ckWinMs = 0;
static uint32_t ckBootAt = 0, ckBootB0[2] = {0, 0}, ckBootG0[2] = {0, 0};
static uint8_t  ckBootScan = 0;             // ports (bit 0 Serial7, bit 1 Serial2) the start-up check wants searched for

// UM982: the AgOpenGPS CONFIG commands found in the receiver's CONFIG list (bit per command)
static uint32_t ckUmSeen[2] = {0, 0};      // [0] ConfigUM982D.txt, [1] ConfigUM982S.txt
static bool     ckUmListed = false;

// the baud rate search / change: what came at the speed tried
static uint8_t  cvSt = 0, cvA = 0, cvB = 0;
static uint16_t cvLen = 0, cvPos = 0, cvGood = 0;
static char     cvLine[CK_LINE_MAX];
static uint8_t  cvLn = 0;
static bool     cvUbx = false, cvUni = false, cvSaveOk = false;

// ---------------------------------------------------------------- helpers
// a time stamp that is never 0 (0 = "not yet"). Not "now | 1": that is 1 ms in the future when now is
// even, and "now - stamp" in the same millisecond then wraps to 49 days (the start-up check ran at once).
static uint32_t ckStamp(uint32_t now) { return now ? now : 1; }

static uint8_t ckHex(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 255;
}

// "$...*hh": XOR of the characters between '$' and '*' (the Unicore "$command" / "$CONFIG" answers
// count the '$' too); "#...*xxxxxxxx": a Unicore ASCII log with its 8-digit CRC
bool nmeaLineOk(const char* l)
{
    if (l[0] != '$' && l[0] != '#') return false;
    const char* s = strrchr(l, '*');
    if (!s || s == l) return false;
    if (l[0] == '#')
    {
        uint8_t k = 0;
        for (const char* p = s + 1; *p && *p != '\r' && *p != '\n'; p++, k++) if (ckHex(*p) == 255) return false;
        return k == 8;
    }
    if (!s[1] || !s[2]) return false;
    const uint8_t h = ckHex(s[1]), lo = ckHex(s[2]);
    if (h == 255 || lo == 255) return false;
    uint8_t x = 0;
    for (const char* p = l + 1; p < s; p++) x ^= (uint8_t)*p;
    const uint8_t want = (uint8_t)(h << 4 | lo);
    return x == want || (uint8_t)(x ^ '$') == want;
}

// the sentence types the GPS receiver sends (zHandlers.ino, before the parser)
void rxCkNoteType(const char* name)
{
    const size_t n = strlen(name);
    if (n < 5) return;
    const char* t = name + n - 3;
    const int k = !strcmp(t, "GGA") ? 0 : !strcmp(t, "VTG") ? 1 : !strcmp(t, "HPR") ? 2 : -1;
    if (k >= 0 && ckTypeCnt[k] < 65535) ckTypeCnt[k]++;
}

static int64_t ckLe(const uint8_t* p, uint8_t n)
{
    uint64_t v = 0;
    for (uint8_t k = 0; k < n && k < 8; k++) v |= (uint64_t)p[k] << (8 * k);
    return (int64_t)v;
}

static uint8_t ckKeySize(uint32_t key)
{
    switch ((key >> 28) & 7) { case 1: case 2: return 1; case 3: return 2; case 4: return 4; default: return 8; }
}

// the AgOpenGPS value of a key (false: not in the settings of that role)
FLASHMEM static bool ckRefValue(uint8_t role, uint32_t key, int64_t& v)
{
    if (role > RXCFG_ROLE_RELPOS) return false;
    const RxCfgSet& s = RXCFG_UBX[role];
    for (uint8_t m = 0; m < s.count; m++)
    {
        const uint8_t* d = s.msgs[m].d;
        const uint16_t n = s.msgs[m].n;
        for (uint16_t o = 0; o + 4 <= n; )
        {
            const uint32_t k = (uint32_t)ckLe(d + o, 4);
            const uint8_t sz = ckKeySize(k);
            if (k == key) { v = ckLe(d + o + 4, sz); return true; }
            o += 4 + sz;
        }
    }
    return false;
}

static uint8_t ckRoleBit(uint8_t role) { return role == RXCFG_ROLE_SINGLE ? R_S : role == RXCFG_ROLE_DUALPOS ? R_D : R_R; }

// the role of the u-blox receiver on a port
static uint8_t ckUbxRole(uint8_t port)
{
    if (port == RXCK_GPS2) return RXCFG_ROLE_RELPOS;
    return dualF9pOn() ? RXCFG_ROLE_DUALPOS : RXCFG_ROLE_SINGLE;
}

static bool ckPortActive(uint8_t port) { return port == RXCK_GPS || dualF9pOn(); }

static const char* ckModel(uint8_t port) { return port == RXCK_GPS ? rxModel : ck[RXCK_GPS2].model; }
static const char* ckFw(uint8_t port)    { return port == RXCK_GPS ? rxFw : ck[RXCK_GPS2].fw; }

// at most n - 1 characters of s (a buffer of smax bytes), always null-terminated
static void ckCopy(char* out, size_t n, const char* s, size_t smax)
{
    const size_t k = strnlen(s, n - 1 < smax ? n - 1 : smax);
    memcpy(out, s, k);
    out[k] = '\0';
}

// a ZED-F9P (the second F9P before its UBX-MON-VER: assumed)
static bool ckIsF9p(uint8_t port)
{
    const char* m = ckModel(port);
    return strstr(m, "F9P") != nullptr || (port == RXCK_GPS2 && !m[0]);
}

// "HPG 1.13" -> 113, 0 = not an HPG firmware
static int ckHpg(const char* fw)
{
    const char* p = strstr(fw, "HPG ");
    int a = 0, b = 0;
    if (!p || sscanf(p + 4, "%d.%d", &a, &b) != 2) return 0;
    return a * 100 + b;
}

// the F9P firmware against HPG 1.13: "same", "older", "newer", "" (not known / not an F9P)
static const char* ckFwState(uint8_t port)
{
    if (!ckIsF9p(port)) return "";
    const int v = ckHpg(ckFw(port));
    return !v ? "" : v == CK_HPG ? "same" : v < CK_HPG ? "older" : "newer";
}

static void ckBegin(uint8_t port, uint32_t baud)
{
    if (port == RXCK_GPS) SerialGPS->begin(baud);
    else Serial2.begin(baud);
}

static void ckWrite(uint8_t port, const uint8_t* d, uint16_t n)
{
    if (port == RXCK_GPS) SerialGPS->write(d, n);
    else Serial2.write(d, n);
}

// a UBX frame
static uint16_t ckFrame(uint8_t* f, uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len)
{
    f[0] = 0xB5; f[1] = 0x62; f[2] = cls; f[3] = id; f[4] = (uint8_t)len; f[5] = (uint8_t)(len >> 8);
    if (len) memcpy(f + 6, p, len);
    uint8_t a = 0, b = 0;
    for (uint16_t k = 2; k < 6 + len; k++) { a += f[k]; b += a; }
    f[6 + len] = a;
    f[7 + len] = b;
    return len + 8;
}

// a UBX message to the receiver of the port: Serial7 through zReceiver.ino's queue (between two RTCM
// messages), Serial2 at once
static bool ckSendUbx(uint8_t port, uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len)
{
    if (port == RXCK_GPS) return ubxSend(cls, id, p, len);
    static uint8_t f[RX_TX_MAX];
    if (len + 8u > sizeof(f)) return false;
    Serial2.write(f, ckFrame(f, cls, id, p, len));
    ckSentMs = millis();
    return true;
}

// the answer did not come within ms after the message went out (still in the queue: not yet)
static bool ckTimedOut(uint32_t now, uint32_t ms)
{
    if (ckPort == RXCK_GPS) return !rxTxLen && now - rxTxSentMs >= ms;
    return now - ckSentMs >= ms;
}

// ---------------------------------------------------------------- what came at the speed tried
static void cvReset()
{
    cvSt = 0; cvLn = 0; cvGood = 0;
    cvUbx = cvUni = cvSaveOk = false;
}

static void cvLineEnd()
{
    if (cvLn < 2) { cvLn = 0; return; }
    cvLine[cvLn] = '\0';
    if (nmeaLineOk(cvLine))
    {
        cvGood++;
        if (cvLine[0] == '#' || !strncmp(cvLine, "$command", 8) || !strncmp(cvLine, "$CONFIG", 7) ||
            (cvLn > 7 && !strncmp(cvLine + 3, "HPR,", 4))) cvUni = true;
        if (strstr(cvLine, "SAVECONFIG") && strstr(cvLine, "OK")) cvSaveOk = true;
    }
    cvLn = 0;
}

static void cvFeed(uint8_t c)
{
    // UBX frames (B5 62 class id length payload ck_a ck_b)
    switch (cvSt)
    {
        case 0: if (c == 0xB5) { cvSt = 1; return; } break;
        case 1: if (c == 0x62) { cvSt = 2; return; } cvSt = 0; break;
        case 2: cvA = c; cvB = cvA; cvSt = 3; return;
        case 3: cvA += c; cvB += cvA; cvSt = 4; return;
        case 4: cvLen = c; cvA += c; cvB += cvA; cvSt = 5; return;
        case 5: cvLen |= (uint16_t)c << 8; cvA += c; cvB += cvA; cvPos = 0; cvSt = cvLen > 2000 ? 0 : (cvLen ? 6 : 7); return;
        case 6: cvA += c; cvB += cvA; if (++cvPos >= cvLen) cvSt = 7; return;
        case 7: cvSt = (c == cvA) ? 8 : 0; return;
        case 8: cvSt = 0; if (c == cvB) { cvGood++; cvUbx = true; } return;
    }
    // text lines
    if (c == '$' || c == '#') { cvLn = 0; cvLine[cvLn++] = (char)c; return; }
    if (c == '\r' || c == '\n') { cvLineEnd(); return; }
    if (cvLn == 0) return;
    if (c < 0x20 || c > 0x7E || cvLn >= CK_LINE_MAX - 1) { cvLn = 0; return; }
    cvLine[cvLn++] = (char)c;
}

static void cvRead(uint8_t port)
{
    int budget = 1024;
    if (port == RXCK_GPS) { while (budget-- > 0 && SerialGPS->available() > 0) cvFeed((uint8_t)SerialGPS->read()); }
    else                  { while (budget-- > 0 && Serial2.available() > 0)    cvFeed((uint8_t)Serial2.read()); }
}

static void cvProbe(uint8_t port)
{
    uint8_t f[8];
    ckWrite(port, f, ckFrame(f, 0x0A, 0x04, nullptr, 0));          // UBX-MON-VER poll
    if (port == RXCK_GPS)
    {
        static const char VERSIONA[] = "\r\nVERSIONA\r\n";
        ckWrite(port, (const uint8_t*)VERSIONA, sizeof(VERSIONA) - 1);
    }
}

// ---------------------------------------------------------------- the ports, every 5 s
const char* rxCkHealthName(uint8_t port)
{
    if (port > 1) return "";
    switch (ck[port].health) { case CKH_NONE: return "none"; case CKH_OK: return "ok"; case CKH_JUNK: return "junk"; default: return "wait"; }
}

static const char* ckWho(uint8_t port) { return port == RXCK_GPS ? "GPS" : "Second F9P"; }

FLASHMEM static void ckWindow(uint8_t port)
{
    CkPort& k = ck[port];
    const uint32_t b = rxCkBytes[port], g = rxCkGood[port];
    k.wb = b - k.b0; k.wg = g - k.g0;
    k.b0 = b;        k.g0 = g;
    const uint8_t secs = CK_WINDOW_MS / 1000;
    if (port == RXCK_GPS)
        for (uint8_t t = 0; t < 3; t++) { k.hz[t] = (uint16_t)(ckTypeCnt[t] * 10u / secs); ckTypeCnt[t] = 0; }
    else
    {
        k.hz[0] = (uint16_t)((dualF9pFrames - k.relpos0) * 10u / secs);
        k.relpos0 = dualF9pFrames;
    }
    if (!ckPortActive(port)) { k.health = CKH_WAIT; k.junkN = 0; k.junkSaid = false; k.autoDone = false; return; }
    if ((port == RXCK_GPS && passThroughGPS) || rxCkOwns(port)) return;     // not read by the Teensy now
    const uint8_t was = k.health;
    if (k.wg > 0)                     k.health = CKH_OK;
    else if (k.wb >= CK_JUNK_BYTES)   k.health = CKH_JUNK;
    else                              k.health = CKH_NONE;
    if (k.health == CKH_JUNK)
    {
        if (k.junkN < 255) k.junkN++;
        if (k.junkN >= CK_JUNK_WINDOWS && !k.junkSaid)
        {
            StatusLog.line("[status] %s: %lu bytes in 5 s at 460800 baud but nothing valid: the %s's baud rate (or its settings, or the wiring) is wrong - web page: GPS, Receiver check",
                           ckWho(port), (unsigned long)k.wb, port == RXCK_GPS ? "receiver" : "F9P");
            k.junkSaid = true;
        }
        return;
    }
    k.junkN = 0;
    if (k.health == CKH_OK && k.junkSaid)
    {
        StatusLog.line("[status] %s: valid data at 460800 baud again", ckWho(port));
        k.junkSaid = false;
    }
    if (k.health == CKH_NONE && was == CKH_OK && port == RXCK_GPS2) k.autoDone = false;   // unplugged: checked again when it is back
}

// ---------------------------------------------------------------- operations
bool rxCkOwns(uint8_t port)
{
    return ckPort == port && (ckOp == CKOP_SCAN || ckOp == CKOP_BAUD);
}

bool rxCkBusy(uint8_t port)
{
    return ckOp != CKOP_NONE && ckPort == port;
}

static void ckEnd()
{
    ckOp = CKOP_NONE;
    ckWait = 0;
    ckGot = 0;
}

// the GPS tab's own work on Serial7 (zReceiver.ino) is idle
static bool ckGpsIdle()
{
    return ubxOp == UBXOP_NONE && umOp == UMOP_NONE && rxIdState != RXID_ASK && !rxTxLen;
}

FLASHMEM static void ckStartCheck(uint8_t port, bool full)
{
    CkPort& k = ck[port];
    ckOp = CKOP_CHECK;
    ckPort = port;
    ckRole = (port == RXCK_GPS && rxKind == RX_UM982) ? CK_ROLE_UM : ckUbxRole(port);
    ckFull = full && ckIsF9p(port);
    ckTries = 0;
    ckWait = 0;
    ckGot = 0;
    k.chkRes = CKR_NONE;
    ckStep = 0;
}

static void ckStartScan(uint8_t port)
{
    CkPort& k = ck[port];
    ckOp = CKOP_SCAN; ckPort = port; ckStep = 0; ckBaudI = 0;
    k.found = 0; k.foundKind = CKK_UNKNOWN; k.scanRes = CKR_NONE; k.baudRes = CKR_NONE;
}

// Start-up: valid data at 460800 from the receivers? If not, their baud rate is searched for (once: the
// receivers are not unplugged with the board on). A port not in use then is not checked.
static void ckBoot(uint32_t now)
{
    if (!ckBootAt)
    {
        ckBootAt = ckStamp(now);
        for (uint8_t p = 0; p < 2; p++) { ckBootB0[p] = rxCkBytes[p]; ckBootG0[p] = rxCkGood[p]; }
        return;
    }
    const uint32_t t = now - ckBootAt;
    if (t < CK_BOOT_MS) return;
    for (uint8_t p = 0; p < 2; p++)
    {
        CkPort& k = ck[p];
        if (k.boot != CKB_WAIT) continue;
        if (!ckPortActive(p)) { k.boot = CKB_OFF; continue; }
        if (p == RXCK_GPS && passThroughGPS) continue;
        const uint32_t b = rxCkBytes[p] - ckBootB0[p], g = rxCkGood[p] - ckBootG0[p];
        if (g) { k.boot = CKB_OK; continue; }
        if (b < CK_JUNK_BYTES && t < CK_BOOT_WAIT_MS) continue;          // nothing yet: a receiver slower to start
        k.boot = CKB_SCAN;
        ckBootScan |= (uint8_t)(1 << p);
        StatusLog.line("[status] %s: %s at 460800 baud after start-up: looking for the receiver's baud rate", ckWho(p),
                       b ? "no valid data" : "nothing");
    }
}

// The web page's buttons (zWeb.ino, POST /api/gps act=ckcheck / ckscan / ckbaud / ckload).
// um (load on a UM982): 0 two antennas (ConfigUM982D.txt), 1 one antenna (ConfigUM982S.txt)
FLASHMEM const char* rxCkStart(const char* what, uint8_t port, uint8_t um)
{
    if (port > 1)                                     return "invalid port";
    if (port == RXCK_GPS2 && !dualF9pOn())            return "the GPS2 port is not set to a second F9P";
    if (ckOp != CKOP_NONE || ckChain)                 return "busy";
    if (port == RXCK_GPS && passThroughGPS)           return "the program's GPS tool has the receiver";
    if (port == RXCK_GPS && (ubxOp != UBXOP_NONE || umOp != UMOP_NONE)) return "busy";     // the GPS tab reads or writes
    const bool owns = !strcmp(what, "scan") || !strcmp(what, "baud");   // (they take the port: the identification waits)
    if (port == RXCK_GPS && !owns && !ckGpsIdle())    return "busy";
    CkPort& k = ck[port];
    if (!owns && k.health == CKH_JUNK)                return "no valid data from the receiver at 460800: find its baud rate first";
    if (!strcmp(what, "check"))
    {
        if (port == RXCK_GPS && rxKind != RX_UBLOX && rxKind != RX_UM982) return "the receiver is not identified";
        ckStartCheck(port, true);
        StatusLog.line("[web] %s: receiver check", ckWho(port));
        return nullptr;
    }
    if (!strcmp(what, "scan"))
    {
        ckStartScan(port);
        StatusLog.line("[web] %s: looking for the receiver's baud rate (no GPS data in the meantime)", ckWho(port));
        return nullptr;
    }
    if (!strcmp(what, "baud"))
    {
        if (k.scanRes != CKR_FOUND || !k.found)       return "no baud rate found yet";
        if (k.found == CK_TEENSY_BAUD)                return "the receiver is at 460800 already";
        ckOp = CKOP_BAUD; ckPort = port; ckStep = 0; ckSaved = false;
        k.baudRes = CKR_NONE;
        StatusLog.line("[web] %s: setting the receiver from %lu to 460800 baud", ckWho(port), (unsigned long)k.found);
        return nullptr;
    }
    if (!strcmp(what, "load"))
    {
        if (port == RXCK_GPS && rxKind == RX_UM982)
        {
            ckOp = CKOP_LOAD; ckPort = port; ckStep = 0;
            k.loadOk = k.loadRam = k.loadNak = k.loadNone = 0;
            k.loadRes = CKR_NONE;
            k.loadRole = um ? CK_LOAD_UMS : CK_LOAD_UMD;
            k.loadTotal = um ? RXCFG_UM982S_N : RXCFG_UM982D_N;
            return nullptr;
        }
        if (port == RXCK_GPS && rxKind != RX_UBLOX)   return "the receiver is not identified";
        if (!ckIsF9p(port))                           return "AOGConfigOMatic has AgOpenGPS settings for the ZED-F9P only";
        ckOp = CKOP_LOAD; ckPort = port; ckStep = 0;
        ckRole = ckUbxRole(port);
        k.loadOk = k.loadRam = k.loadNak = k.loadNone = 0;
        k.loadRes = CKR_NONE;
        k.loadRole = ckRole;
        k.loadTotal = RXCFG_UBX[ckRole].count;
        StatusLog.line("[web] %s: loading the AgOpenGPS settings (AOGConfigOMatic %s: %u messages, %u keys)", ckWho(port),
                       RXCFG_UBX[ckRole].file, RXCFG_UBX[ckRole].count, RXCFG_UBX[ckRole].keys);
        return nullptr;
    }
    return "unknown action";
}

// UBX frames from the receivers (zReceiver.ino: Serial7, zDualF9P.ino: Serial2). True: taken here.
bool rxCkUbx(uint8_t port, uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len)
{
    if (ckOp == CKOP_NONE || port != ckPort || !ckWait) return false;
    CkPort& k = ck[port];
    if (cls == 0x05 && len >= 2 && p[0] == 0x06)
    {
        if (ckWait == 2 && p[1] == 0x8A) { ckGot = id == 0x01 ? 1 : 2; return true; }
        if (ckWait == 1 && p[1] == 0x8B) { if (id == 0x00) ckGot = 2; return true; }   // NAK: a key the receiver does not know
        return false;
    }
    if (cls == 0x06 && id == 0x8B && ckWait == 1 && len >= 4)
    {
        // version, layer, position (2), then key + value pairs
        uint16_t answered = 0;
        const RxCfgMsg* m = (ckStep == 7 && ckRole <= RXCFG_ROLE_RELPOS) ? &RXCFG_UBX[ckRole].msgs[ckMsg] : nullptr;
        for (uint16_t o = 4; o + 4 <= len; )
        {
            const uint32_t key = (uint32_t)ckLe(p + o, 4);
            const uint8_t sz = ckKeySize(key);
            if (o + 4 + sz > len) break;
            const int64_t v = ckLe(p + o + 4, sz);
            o += 4 + sz;
            if (m)
            {
                for (uint16_t r = 0; r + 4 <= m->n; )
                {
                    const uint32_t rk = (uint32_t)ckLe(m->d + r, 4);
                    const uint8_t rs = ckKeySize(rk);
                    if (rk == key) { answered++; if (ckLe(m->d + r + 4, rs) == v) k.cmpSame++; else k.cmpDiff++; break; }
                    r += 4 + rs;
                }
            }
            else
                for (uint8_t e = 0; e < CK_ESS_N; e++)
                    if (CK_ESS[e].key == key) { k.ess[e] = v; k.essHave[e] = 1; }
        }
        if (m && answered < m->keys) k.cmpUnk += m->keys - answered;
        ckGot = 1;
        return true;
    }
    if (cls == 0x0A && id == 0x04 && ckWait == 3 && len >= 40)
    {
        // UBX-MON-VER: swVersion[30], hwVersion[10], then 30-character extensions (MOD=, FWVER=)
        k.model[0] = k.fw[0] = '\0';
        for (uint16_t off = 40; off + 30 <= len; off += 30)
        {
            char ext[32];
            uint8_t n = 0;
            for (uint8_t q = 0; q < 30 && p[off + q] && (size_t)n + 1 < sizeof(ext); q++) if (p[off + q] >= 0x20 && p[off + q] <= 0x7E) ext[n++] = (char)p[off + q];
            ext[n] = '\0';
            if (!strncmp(ext, "MOD=", 4))        ckCopy(k.model, sizeof(k.model), ext + 4, sizeof(ext) - 4);
            else if (!strncmp(ext, "FWVER=", 6)) ckCopy(k.fw, sizeof(k.fw), ext + 6, sizeof(ext) - 6);
        }
        ckGot = 1;
        return true;
    }
    return false;
}

static const char* ckResName(uint8_t r)
{
    switch (r)
    {
        case CKR_FOUND:     return "found";
        case CKR_NOTFOUND:  return "notfound";
        case CKR_BAUD_OK:   return "baud-ok";
        case CKR_BAUD_FAIL: return "baud-fail";
        case CKR_CHECKED:   return "checked";
        case CKR_NOANSWER:  return "noanswer";
        case CKR_LOADED:    return "loaded";
        case CKR_LOAD_PART: return "load-part";
        case CKR_LOAD_FAIL: return "load-fail";
        case CKR_STOPPED:   return "stopped";
        default:            return "";
    }
}

// what the Teensy needs and the receiver does not have (u-blox), as "gga, vtg"
FLASHMEM static void ckMissing(uint8_t port, char* out, size_t n)
{
    const CkPort& k = ck[port];
    out[0] = '\0';
    if (k.essRole > RXCFG_ROLE_RELPOS) return;
    const uint8_t bit = ckRoleBit(k.essRole);
    for (uint8_t e = 0; e < CK_ESS_N; e++)
    {
        if (!(CK_ESS[e].roles & bit)) continue;
        int64_t want;
        if (!ckRefValue(k.essRole, CK_ESS[e].key, want)) continue;
        if (k.essHave[e] == 1 && k.ess[e] == want) continue;
        const size_t u = strlen(out);
        snprintf(out + u, n - u, "%s%s", u ? ", " : "", CK_ESS[e].id);
    }
}

// after a u-blox check: the [status] lines (what is missing, the firmware, the AgOpenGPS settings)
FLASHMEM static void ckCheckSay(uint8_t port)
{
    CkPort& k = ck[port];
    char miss[120];
    ckMissing(port, miss, sizeof(miss));
    const char* st = ckFwState(port);
    char fwNote[104] = "";
    if (!strcmp(st, "older") || !strcmp(st, "newer"))
        snprintf(fwNote, sizeof(fwNote), "; firmware %s, the AgOpenGPS settings are made for HPG 1.13", ckFw(port));
    if (miss[0]) StatusLog.line("[status] %s receiver check: missing what the Teensy needs: %s%s", ckWho(port), miss, fwNote);
    else         StatusLog.line("[status] %s receiver check: it has what the Teensy needs%s", ckWho(port), fwNote);
    if (ckFull && k.cmpMs && k.cmpRole == k.essRole)
        StatusLog.line("[status] %s receiver check: AgOpenGPS settings (%s): %u the same, %u different, %u not compared", ckWho(port),
                       RXCFG_UBX[k.cmpRole].file, k.cmpSame, k.cmpDiff, k.cmpUnk);
}

// back to 460800, and the reader of the port starts again
static void ckPortBack(uint8_t port)
{
    ckBegin(port, CK_TEENSY_BAUD);
    if (port == RXCK_GPS2) dualF9pRestart();
}

FLASHMEM static void ckOpScan(uint32_t now)
{
    CkPort& k = ck[ckPort];
    if (ckStep == 0)
    {
        ckBegin(ckPort, CK_BAUDS[ckBaudI]);
        cvReset();
        ckProbes = 0;
        ckT0 = now;
        ckStep = 1;
        return;
    }
    cvRead(ckPort);
    const uint32_t t = now - ckT0;
    if ((ckProbes == 0 && t >= 100) || (ckProbes == 1 && t >= 700)) { cvProbe(ckPort); ckProbes++; }
    // valid data: this is the speed; what it is, once the answer to the first query has had time to arrive
    if (cvGood >= CK_GOOD_FOUND && (cvUbx || cvUni || t >= 600 || t >= CK_LISTEN_MS))
    {
        k.found = CK_BAUDS[ckBaudI];
        k.foundKind = cvUbx ? CKK_UBLOX : cvUni ? CKK_UNICORE : CKK_NMEA;
        k.scanRes = CKR_FOUND;
    }
    else if (t < CK_LISTEN_MS || (cvGood >= CK_GOOD_FOUND && t < 600)) return;
    else if (++ckBaudI < CK_BAUD_N) { ckStep = 0; return; }
    else k.scanRes = CKR_NOTFOUND;
    ckPortBack(ckPort);
    if (k.scanRes == CKR_FOUND)
        StatusLog.line("[status] %s: the receiver answers at %lu baud (%s)%s", ckWho(ckPort), (unsigned long)k.found,
                       k.foundKind == CKK_UBLOX ? "u-blox" : k.foundKind == CKK_UNICORE ? "Unicore" : "NMEA",
                       k.found == CK_TEENSY_BAUD ? "" : " - the Teensy needs 460800: web page, GPS, Receiver check, set it to 460800");
    else
        StatusLog.line("[status] %s: no valid data at any baud rate (9600 to 921600): nothing connected, a silent receiver, or the wiring", ckWho(ckPort));
    ckEnd();
}

FLASHMEM static void ckOpBaud(uint32_t now)
{
    CkPort& k = ck[ckPort];
    const uint32_t t = now - ckT0;
    switch (ckStep)
    {
        case 0:
            ckBegin(ckPort, k.found);
            ckT0 = now;
            ckStep = 1;
            return;
        case 1:
        {
            if (t < 150) return;
            if (k.foundKind != CKK_UNICORE)
            {
                // UBX-CFG-VALSET version 0, RAM + BBR + flash: CFG-UART1-BAUDRATE = 460800
                const uint32_t key = 0x40520001UL, v = CK_TEENSY_BAUD;
                const uint8_t q[12] = {0, 7, 0, 0, (uint8_t)key, (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                                       (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
                uint8_t f[20];
                ckWrite(ckPort, f, ckFrame(f, 0x06, 0x8A, q, sizeof(q)));
            }
            if (ckPort == RXCK_GPS && k.foundKind != CKK_UBLOX)
            {
                static const char C[] = "\r\nCONFIG COM1 460800\r\nCONFIG COM2 460800\r\nCONFIG COM3 460800\r\n";
                ckWrite(ckPort, (const uint8_t*)C, sizeof(C) - 1);
            }
            ckT0 = now;
            ckStep = 2;
            return;
        }
        case 2:
            if (t < 400) return;                           // the message out, and the receiver at its new speed
            ckBegin(ckPort, CK_TEENSY_BAUD);
            cvReset();
            ckT0 = now;
            ckStep = 3;
            return;
        case 3:
            cvRead(ckPort);
            if (cvGood >= 3)
            {
                if (ckPort == RXCK_GPS && k.foundKind != CKK_UBLOX)
                {
                    static const char S[] = "SAVECONFIG\r\n";
                    ckWrite(ckPort, (const uint8_t*)S, sizeof(S) - 1);
                    ckT0 = now;
                    ckStep = 4;
                    return;
                }
                k.baudRes = CKR_BAUD_OK;
                break;
            }
            if (t < CK_BAUD_OK_MS) return;
            k.baudRes = CKR_BAUD_FAIL;
            break;
        default:                                           // waiting for "$command,SAVECONFIG,response: OK"
            cvRead(ckPort);
            if (!cvSaveOk && t < CK_SAVE_MS) return;
            ckSaved = cvSaveOk;
            k.baudRes = CKR_BAUD_OK;
            break;
    }
    ckPortBack(ckPort);
    if (k.baudRes == CKR_BAUD_OK)
    {
        StatusLog.line("[status] %s: the receiver is at 460800 baud now (was %lu)%s", ckWho(ckPort), (unsigned long)k.found,
                       ckPort == RXCK_GPS && k.foundKind != CKK_UBLOX ? (ckSaved ? ", SAVECONFIG done" : ", SAVECONFIG not confirmed") : "");
        k.scanRes = CKR_NONE;
        k.found = 0;
    }
    else
        StatusLog.line("[status] %s: no valid data at 460800 after the change: the receiver is still at %lu baud?", ckWho(ckPort), (unsigned long)k.found);
    ckEnd();
    if (ckPort == RXCK_GPS) receiverIdentifyAgain();
    else ck[RXCK_GPS2].autoDone = false;
}

// a UBX-CFG-VALGET of n keys (the RAM layer)
FLASHMEM static bool ckValget(const uint32_t* keys, uint8_t n)
{
    uint8_t q[4 + 64 * 4];
    if (n > 64) n = 64;
    q[0] = 0; q[1] = 0; q[2] = 0; q[3] = 0;                // version 0, RAM layer, position 0
    for (uint8_t i = 0; i < n; i++) for (uint8_t b = 0; b < 4; b++) q[4 + i * 4 + b] = (uint8_t)(keys[i] >> (8 * b));
    if (!ckSendUbx(ckPort, 0x06, 0x8B, q, (uint16_t)(4 + n * 4))) return false;
    ckWait = 1;
    ckGot = 0;
    return true;
}

FLASHMEM static void ckOpCheck(uint32_t now)
{
    CkPort& k = ck[ckPort];
    if (ckRole == CK_ROLE_UM)                              // UM982: the GPS tab's read, then the card shows it
    {
        if (ckStep == 0)
        {
            if (umStartRead()) { k.chkRes = CKR_NOANSWER; ckEnd(); return; }
            ckStep = 1;
            return;
        }
        if (umOp != UMOP_NONE) return;
        k.chkRes = umRes == UMRES_READ_NONE || umRes == UMRES_NONE ? CKR_NOANSWER : CKR_CHECKED;
        k.essRole = CK_ROLE_UM;
        k.essMs = ckStamp(now);
        ckEnd();
        return;
    }
    switch (ckStep)
    {
        case 0:                                            // the second F9P: what it is (MON-VER)
            if (ckPort == RXCK_GPS) { ckStep = 2; return; }
            if (!ckSendUbx(ckPort, 0x0A, 0x04, nullptr, 0)) return;
            ckWait = 3; ckGot = 0; ckStep = 1;
            return;
        case 1:
            if (!ckGot && !ckTimedOut(now, CK_MONVER_MS)) return;
            if (!ckGot)
            {
                k.chkRes = CKR_NOANSWER;
                StatusLog.line("[status] %s: no answer to UBX-MON-VER (UBX output off on its UART1, or the Teensy's Serial2 TX not wired to it?)", ckWho(ckPort));
                ckEnd();
                return;
            }
            ckFull = ckFull && ckIsF9p(ckPort);
            ckWait = 0;
            ckStep = 2;
            return;
        case 2:                                            // the keys the Teensy needs, in one VALGET
        {
            const uint8_t bit = ckRoleBit(ckRole);
            uint32_t keys[CK_ESS_N];
            uint8_t n = 0;
            for (uint8_t e = 0; e < CK_ESS_N; e++) { k.essHave[e] = 0; if (CK_ESS[e].roles & bit) keys[n++] = CK_ESS[e].key; }
            if (!ckValget(keys, n)) return;
            ckStep = 3;
            return;
        }
        case 3:
            if (!ckGot && !ckTimedOut(now, CK_GET_MS)) return;
            if (ckGot == 2) { ckKeyI = 0; ckWait = 0; ckStep = 5; return; }   // a key it does not know: one at a time
            if (!ckGot)
            {
                ckWait = 0;
                if (++ckTries < 3) { ckStep = 2; return; }
                k.chkRes = CKR_NOANSWER;
                StatusLog.line("[status] %s: no answer to UBX-CFG-VALGET (UBX output off on UART1?)", ckWho(ckPort));
                ckEnd();
                return;
            }
            ckWait = 0;
            ckStep = 4;
            return;
        case 5:                                            // one key at a time
        {
            const uint8_t bit = ckRoleBit(ckRole);
            if (ckWait)
            {
                if (!ckGot && !ckTimedOut(now, CK_GET_MS)) return;
                if (ckGot == 2) k.essHave[ckKeyI] = 2;
                ckWait = 0;
                ckKeyI++;
            }
            while (ckKeyI < CK_ESS_N && !(CK_ESS[ckKeyI].roles & bit)) ckKeyI++;
            if (ckKeyI >= CK_ESS_N) { ckStep = 4; return; }
            const uint32_t key = CK_ESS[ckKeyI].key;
            ckValget(&key, 1);
            return;
        }
        case 4:                                            // what the Teensy needs: read
            k.essRole = ckRole;
            k.essMs = ckStamp(now);
            if (ckPort == RXCK_GPS2) k.autoDone = true;      // (no further automatic check for this one)
            if (!ckFull) { k.chkRes = CKR_CHECKED; ckCheckSay(ckPort); ckEnd(); return; }
            k.cmpSame = k.cmpDiff = k.cmpUnk = 0;
            k.cmpRole = ckRole;
            k.cmpMs = 0;
            ckMsg = 0;
            ckTries = 0;
            ckStep = 6;
            return;
        case 6:                                            // every AgOpenGPS key, one VALGET per message of the file
        {
            const RxCfgSet& s = RXCFG_UBX[ckRole];
            if (ckMsg >= s.count) { k.cmpMs = ckStamp(now); k.chkRes = CKR_CHECKED; ckCheckSay(ckPort); ckEnd(); return; }
            const RxCfgMsg& m = s.msgs[ckMsg];
            uint32_t keys[64];
            uint8_t n = 0;
            for (uint16_t o = 0; o + 4 <= m.n && n < 64; )
            {
                const uint32_t key = (uint32_t)ckLe(m.d + o, 4);
                keys[n++] = key;
                o += 4 + ckKeySize(key);
            }
            if (!ckValget(keys, n)) return;
            ckStep = 7;
            return;
        }
        default:                                           // 7: its answer
            if (!ckGot && !ckTimedOut(now, CK_GET_MS)) return;
            ckWait = 0;
            if (!ckGot && ++ckTries < 2) { ckStep = 6; return; }
            if (ckGot != 1) k.cmpUnk += RXCFG_UBX[ckRole].msgs[ckMsg].keys;   // NAK (a key it does not know) or no answer
            ckTries = 0;
            ckMsg++;
            ckStep = 6;
            return;
    }
}

FLASHMEM static void ckOpLoad(uint32_t now)
{
    CkPort& k = ck[ckPort];
    if (k.loadRole >= CK_LOAD_UMD)                         // UM982: the commands, SAVECONFIG and the read (zReceiver.ino)
    {
        const bool one = k.loadRole == CK_LOAD_UMS;
        if (ckStep == 0)
        {
            if (umWriteRaw(one ? RXCFG_UM982S : RXCFG_UM982D, one ? RXCFG_UM982S_N : RXCFG_UM982D_N,
                           one ? "ConfigUM982S.txt" : "ConfigUM982D.txt"))
            {
                k.loadRes = CKR_LOAD_FAIL; k.loadMs = ckStamp(now); ckEnd(); return;
            }
            ckStep = 1;
            return;
        }
        if (umOp != UMOP_NONE) return;
        const uint8_t r = umWriteRes;
        k.loadOk = umOkN;
        k.loadNone = umNoAnsN;
        k.loadNak = (uint16_t)(k.loadTotal > umOkN + umNoAnsN ? k.loadTotal - umOkN - umNoAnsN : 0);
        k.loadRes = r == UMRES_WRITE ? CKR_LOADED : (r == UMRES_WRITE_PART || r == UMRES_WRITE_NOSAVE) ? CKR_LOAD_PART : CKR_LOAD_FAIL;
        if (r == UMRES_NONE) { k.loadRes = CKR_STOPPED; k.loadNak = 0; k.loadNone = 0; }
        k.loadMs = ckStamp(now);
        k.essRole = CK_ROLE_UM;                            // read again after the write (zReceiver.ino)
        k.essMs = ckStamp(now);
        ckEnd();
        return;
    }
    const RxCfgSet& s = RXCFG_UBX[ckRole];
    if (ckStep == 0) { ckMsg = 0; ckLayers = 7; ckNoneRow = 0; ckWait = 0; ckStep = 1; }
    if (ckWait == 0)
    {
        if (ckMsg >= s.count)
        {
            const uint16_t bad = k.loadNak + k.loadNone;
            k.loadRes = !bad ? CKR_LOADED : (k.loadOk + k.loadRam) ? CKR_LOAD_PART : CKR_LOAD_FAIL;
            k.loadMs = ckStamp(now);
            StatusLog.line("[status] %s: AgOpenGPS settings (%s) %s: %u of %u messages to RAM, BBR and flash, %u to RAM and BBR only, %u refused, %u not answered",
                           ckWho(ckPort), s.file, k.loadRes == CKR_LOADED ? "loaded" : k.loadRes == CKR_LOAD_PART ? "partly loaded" : "NOT loaded",
                           k.loadOk, s.count, k.loadRam, k.loadNak, k.loadNone);
            ckEnd();
            if (k.loadOk + k.loadRam)
            {
                ckChain = true;                            // check everything again (and refresh the GPS tab's form)
                if (ckPort == RXCK_GPS) ckRefreshForm = true;
            }
            return;
        }
        // UBX-CFG-VALSET version 0 (no transaction), layers, the key + value pairs of the message
        const RxCfgMsg& m = s.msgs[ckMsg];
        static uint8_t q[RX_TX_MAX];
        if (m.n + 4u + 8u > sizeof(q)) { k.loadNak++; ckMsg++; return; }
        q[0] = 0; q[1] = ckLayers; q[2] = 0; q[3] = 0;
        memcpy(q + 4, m.d, m.n);
        if (!ckSendUbx(ckPort, 0x06, 0x8A, q, (uint16_t)(m.n + 4))) return;
        ckWait = 2;
        ckGot = 0;
        return;
    }
    if (!ckGot && !ckTimedOut(now, CK_SET_MS)) return;
    ckWait = 0;
    if (ckGot == 2 && ckLayers == 7) { ckLayers = 3; return; }        // again without the flash
    if (ckGot == 1) { if (ckLayers == 7) k.loadOk++; else k.loadRam++; ckNoneRow = 0; }
    else if (ckGot == 2) { k.loadNak++; ckNoneRow = 0; }
    else
    {
        k.loadNone++;
        if (++ckNoneRow >= 3)                              // three without an answer: it is not listening
        {
            k.loadNone += (uint16_t)(s.count - ckMsg - 1);
            ckMsg = s.count;
            return;
        }
    }
    ckMsg++;
    ckLayers = 7;
}

// another receiver on the port: what was checked and loaded belonged to the one before
static void ckForget(uint8_t port)
{
    CkPort& k = ck[port];
    k.chkRes = k.loadRes = CKR_NONE;
    k.essMs = k.cmpMs = k.loadMs = 0;
    k.loadOk = k.loadRam = k.loadNak = k.loadNone = k.loadTotal = 0;
}

// stop what is running (AgIO's GPS tool took the receiver, or the GPS2 port changed)
FLASHMEM static void ckStop(const char* why)
{
    CkPort& k = ck[ckPort];
    if (rxCkOwns(ckPort)) ckPortBack(ckPort);
    if (ckOp == CKOP_SCAN) k.scanRes = CKR_STOPPED;
    if (ckOp == CKOP_BAUD) k.baudRes = CKR_STOPPED;
    if (ckOp == CKOP_CHECK) k.chkRes = CKR_STOPPED;
    if (ckOp == CKOP_LOAD) { k.loadRes = CKR_STOPPED; k.loadMs = ckStamp(millis()); }
    StatusLog.line("[status] %s: receiver check stopped (%s)", ckWho(ckPort), why);
    ckEnd();
    ckChain = ckRefreshForm = false;
}

// ---------------------------------------------------------------- UM982: the AgOpenGPS commands in its CONFIG list
// the same command: the same words (upper / lower case alike), numbers by their value ("90.0" = "90.00")
FLASHMEM static bool ckSameCmd(const char* a, const char* b)
{
    for (;;)
    {
        while (*a == ' ') a++;
        while (*b == ' ') b++;
        const bool ea = !*a || *a == '*', eb = !*b || *b == '*';
        if (ea || eb) return ea && eb;
        const char* a2 = a; while (*a2 && *a2 != ' ' && *a2 != '*') a2++;
        const char* b2 = b; while (*b2 && *b2 != ' ' && *b2 != '*') b2++;
        char* ae; char* be;
        const double x = strtod(a, &ae), y = strtod(b, &be);
        if (ae == a2 && be == b2 && ae != a && be != b) { if (fabs(x - y) > 1e-6) return false; }
        else
        {
            if (a2 - a != b2 - b) return false;
            for (const char *p = a, *q = b; p < a2; p++, q++) if (toupper((unsigned char)*p) != toupper((unsigned char)*q)) return false;
        }
        a = a2; b = b2;
    }
}

void rxCkUmBegin()
{
    ckUmSeen[0] = ckUmSeen[1] = 0;
    ckUmListed = false;
}

// "$CONFIG,<item>,<command as it would be typed>*cs" (zReceiver.ino, while it reads the UM982)
FLASHMEM void rxCkUmConfig(const char* l)
{
    const char* a = strchr(l + 8, ',');
    if (!a) return;
    ckUmListed = true;
    for (uint8_t i = 0; i < RXCFG_UM982D_N; i++) if (ckSameCmd(RXCFG_UM982D[i], a + 1)) ckUmSeen[0] |= 1UL << i;
    for (uint8_t i = 0; i < RXCFG_UM982S_N; i++) if (ckSameCmd(RXCFG_UM982S[i], a + 1)) ckUmSeen[1] |= 1UL << i;
}

// which AgOpenGPS commands the UM982 has (bit per command): CONFIG ones from its CONFIG list, MODE from
// its MODE, the logs ("GPGGA 0.1") from its UNILOGLIST on the Teensy's port
FLASHMEM static uint32_t ckUmHas(uint8_t list)
{
    const char* const* c = list ? RXCFG_UM982S : RXCFG_UM982D;
    const uint8_t n = list ? RXCFG_UM982S_N : RXCFG_UM982D_N;
    uint32_t m = ckUmSeen[list];
    for (uint8_t i = 0; i < n; i++)
    {
        if (!strncmp(c[i], "MODE ", 5)) { if (ckSameCmd(c[i], umModeText)) m |= 1UL << i; continue; }
        if (!strncmp(c[i], "CONFIG ", 7)) continue;
        const char* sp = strchr(c[i], ' ');
        if (!sp) continue;
        for (uint8_t t = UM_GGA; t <= UM_KSXT; t++)
        {
            const char* w = UM_MSG_CMD[t - UM_GGA];
            if (strlen(w) != (size_t)(sp - c[i]) || strncmp(w, c[i], sp - c[i])) continue;
            if (umHave[t] && fabs(umVal[t] - atof(sp + 1) * 1000.0) < 0.5) m |= 1UL << i;
        }
    }
    return m;
}

// ---------------------------------------------------------------- loop
void rxCheckLoop()
{
    const uint32_t now = millis();
    if (now - ckWinMs >= CK_WINDOW_MS) { ckWinMs = now; ckWindow(RXCK_GPS); ckWindow(RXCK_GPS2); }
    ckBoot(now);

    // another receiver identified on Serial7 (zReceiver.ino): the results of the one before are forgotten
    static char ident[32] = "";
    if (rxIdState == RXID_DONE)
    {
        char id[32];
        snprintf(id, sizeof(id), "%u %.24s", (unsigned)rxKind, rxModel);
        if (strcmp(id, ident)) { if (ident[0] && !rxCkBusy(RXCK_GPS)) ckForget(RXCK_GPS); ckCopy(ident, sizeof(ident), id, sizeof(id)); }
    }
    if (ckOp != CKOP_NONE && ckPort == RXCK_GPS && passThroughGPS) ckStop("the program's GPS tool took the receiver");
    if (ckOp != CKOP_NONE && ckPort == RXCK_GPS2 && !dualF9pOn()) ckStop("the GPS2 port is no longer set to the second F9P");
    if (ckOp == CKOP_NONE)
    {
        // the start-up check found no valid data: the baud rate is searched for (the GPS tab's work waits)
        if (ckBootScan)
        {
            const uint8_t p = (ckBootScan & 1) ? RXCK_GPS : RXCK_GPS2;
            if (!ckPortActive(p)) ckBootScan &= (uint8_t)~(1 << p);
            else if (p == RXCK_GPS2 || (!passThroughGPS && ubxOp == UBXOP_NONE && umOp == UMOP_NONE))
            {
                ckBootScan &= (uint8_t)~(1 << p);
                ckStartScan(p);
            }
        }
        // after a load: everything checked again, then the GPS tab's form read again
        else if (ckChain)
        {
            if (ckPort != RXCK_GPS || ckGpsIdle()) { ckChain = false; ckStartCheck(ckPort, true); }
        }
        else if (ckRefreshForm)
        {
            if (ckGpsIdle()) { ckRefreshForm = false; if (rxKind == RX_UBLOX) ubxStartRead(); }
        }
        // automatically: the receiver once it is identified, the second F9P once its data is valid
        else if ((rxKind == RX_UBLOX || rxKind == RX_UM982) && rxIdState == RXID_DONE && ck[RXCK_GPS].autoFor != rxIdMs &&
                 ck[RXCK_GPS].health == CKH_OK && ckGpsIdle() && !passThroughGPS)
        {
            ck[RXCK_GPS].autoFor = rxIdMs;
            ckStartCheck(RXCK_GPS, false);
        }
        else if (dualF9pOn() && !ck[RXCK_GPS2].autoDone && ck[RXCK_GPS2].health == CKH_OK)
        {
            ck[RXCK_GPS2].autoDone = true;
            ckStartCheck(RXCK_GPS2, false);
        }
    }
    switch (ckOp)
    {
        case CKOP_SCAN:  ckOpScan(now);  break;
        case CKOP_BAUD:  ckOpBaud(now);  break;
        case CKOP_CHECK: ckOpCheck(now); break;
        case CKOP_LOAD:  ckOpLoad(now);  break;
        default: break;
    }
}

// ---------------------------------------------------------------- web page (GET /api/gps, "ck")
FLASHMEM static void ckPortJson(Json& j, uint8_t port)
{
    const uint32_t now = millis();
    CkPort& k = ck[port];
    const bool um = port == RXCK_GPS && rxKind == RX_UM982;
    const uint8_t role = um ? CK_ROLE_UM : ckUbxRole(port);
    j.open();
    j.bo("on", ckPortActive(port));
    j.str("health", rxCkHealthName(port));
    j.u("bytes", k.wb);
    j.u("good", k.wg);
    j.arr("hz");                                           // x 10: port 0 GGA, VTG, HPR; port 1 RELPOSNED
    for (uint8_t t = 0; t < (port == RXCK_GPS ? 3 : 1); t++) { char v[8]; snprintf(v, sizeof(v), "%u", k.hz[t]); j.sep(); j.raw(v); }
    j.closeArr();
    j.str("model", ckModel(port));
    j.str("fw", ckFw(port));
    j.str("fwSt", ckFwState(port));
    j.bo("f9p", ckIsF9p(port));
    j.str("role", um ? "" : RXCFG_UBX[role].name);
    j.str("file", um ? "" : RXCFG_UBX[role].file);
    j.u("keys", um ? 0 : RXCFG_UBX[role].keys);
    // u-blox, what the Teensy needs: {id, st 0 not read / 1 read / 2 unknown key, v read, want}
    const bool essNow = k.essMs && k.essRole == role;
    j.i("essAgo", essNow ? (long)((now - k.essMs) / 1000) : -1);
    j.arr("ess");
    if (!um && essNow)
    {
        const uint8_t bit = ckRoleBit(role);
        for (uint8_t e = 0; e < CK_ESS_N; e++)
        {
            if (!(CK_ESS[e].roles & bit)) continue;
            int64_t want = 0;
            if (!ckRefValue(role, CK_ESS[e].key, want)) continue;
            j.open();
            j.str("id", CK_ESS[e].id);
            j.i("st", k.essHave[e]);
            if (k.essHave[e] == 1) j.i("v", (long)k.ess[e]);
            j.i("want", (long)want);
            j.close();
        }
    }
    j.closeArr();
    const bool cmpNow = !um && k.cmpMs && k.cmpRole == role;
    j.i("cmpAgo", cmpNow ? (long)((now - k.cmpMs) / 1000) : -1);
    j.u("same", cmpNow ? k.cmpSame : 0);
    j.u("diff", cmpNow ? k.cmpDiff : 0);
    j.u("unk", cmpNow ? k.cmpUnk : 0);
    if (port == RXCK_GPS)                                  // UM982: the AgOpenGPS commands it has (bit per command)
    {
        const bool listed = um && ckUmListed && umReadMs;
        j.bo("umListed", listed);
        j.u("umD", listed ? ckUmHas(0) : 0);
        j.u("umS", listed ? ckUmHas(1) : 0);
    }
    j.str("boot", k.boot == CKB_OK ? "ok" : k.boot == CKB_SCAN ? "scan" : k.boot == CKB_OFF ? "off" : "wait");
    j.str("scan", ckResName(k.scanRes));
    j.u("found", k.found);
    j.str("foundKind", k.foundKind == CKK_UBLOX ? "ublox" : k.foundKind == CKK_UNICORE ? "unicore" : k.foundKind == CKK_NMEA ? "nmea" : "");
    j.str("baud", ckResName(k.baudRes));
    j.str("chk", ckResName(k.chkRes));
    j.str("load", ckResName(k.loadRes));
    j.str("loadFile", k.loadRole <= RXCFG_ROLE_RELPOS ? RXCFG_UBX[k.loadRole].file : k.loadRole == CK_LOAD_UMS ? "ConfigUM982S.txt" : "ConfigUM982D.txt");
    j.u("loadOk", k.loadOk);
    j.u("loadRam", k.loadRam);
    j.u("loadNak", k.loadNak);
    j.u("loadNone", k.loadNone);
    j.u("loadTotal", k.loadTotal);
    j.i("loadAgo", k.loadMs ? (long)((now - k.loadMs) / 1000) : -1);
    j.close();
}

FLASHMEM void rxCkJson(Json& j)
{
    j.obj("ck");
    j.str("op", ckOp == CKOP_SCAN ? "scan" : ckOp == CKOP_BAUD ? "baud" : ckOp == CKOP_CHECK ? "check" : ckOp == CKOP_LOAD ? "load" : ckChain ? "check" : "");
    j.i("port", ckPort);
    j.u("baudTry", ckOp == CKOP_SCAN && ckBaudI < CK_BAUD_N ? CK_BAUDS[ckBaudI] : 0);
    const bool msgs = (ckOp == CKOP_LOAD && ck[ckPort].loadRole <= RXCFG_ROLE_RELPOS) || (ckOp == CKOP_CHECK && ckFull && ckStep >= 6);
    j.u("msg", msgs ? ckMsg : 0);
    j.u("msgs", msgs ? RXCFG_UBX[ckRole].count : 0);
    j.str("fwNeed", "HPG 1.13");
    j.bo("umSingle", cfg.umSingle != 0);                   // UM982 with one antenna (setting; the AgOpenGPS file)
    j.arr("um982d");
    for (uint8_t i = 0; i < RXCFG_UM982D_N; i++) j.item(RXCFG_UM982D[i]);
    j.closeArr();
    j.arr("um982s");
    for (uint8_t i = 0; i < RXCFG_UM982S_N; i++) j.item(RXCFG_UM982S[i]);
    j.closeArr();
    j.arr("ports");
    ckPortJson(j, RXCK_GPS);
    ckPortJson(j, RXCK_GPS2);
    j.closeArr();
    j.close();
}
