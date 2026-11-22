/* zXiao.ino - link to the XIAO ESP32 on the GPS2 socket (Serial2).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Same link protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn"
   (AgOpenGPS-Official, GPL v3), at the same speed, 460800 baud.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zXiao.ino
// Seeed XIAO ESP32C3 / ESP32S3 on Teensy Serial2 (pin 7 = RX2, pin 8 = TX2), 460800 baud: on the AiO v4.5
// Micro in the GPS2 socket ("Dual-Left") through the adapter board or the 3D-printed holder; on the AiO v4.5
// Standard wired to its GPS2 header (P8: pin 1 RX2, pin 2 TX2) with 3.3 V and GND.
//
// Same protocol as the "ESP32 bridge" of the official Teensy firmware v26 (Firmware_Teensy_AiO-New-Dawn,
// docs/ESP32_BRIDGE_INTERFACE.md), at the same speed, 460800 (XIAO_BAUD, zXiao.h). A XIAO program
// written for v26 works here (with the checksum below):
//   XIAO -> Teensy    "ESP32-hello" (plain ASCII, no line ending) at least every 5 s. The XIAO counts
//                     as present for XIAO_HELLO_TIMEOUT_MS after each hello.
//   AgOpenGPS -> XIAO every AgOpenGPS packet that reaches the module on UDP 8888 (80 81 src pgn len
//                     data crc) is copied, unchanged, to the XIAO - only while it is present.
//   XIAO -> AgOpenGPS AgOpenGPS frames from the XIAO, [0x80][0x81][src][pgn][len][data...][crc], are
//                     sent to UDP 9999 (broadcast), like the module's own PGNs.
// Differences from v26:
//   - A frame from the XIAO must carry the AgOpenGPS checksum: the low 8 bits of the sum of the bytes
//     from src to the last data byte (the XOR in the v26 document is not what AgOpenGPS uses). Frames
//     with a wrong checksum are dropped and counted. This also keeps the ESP32-C3 boot messages
//     (115200 baud on the same pin, garbage at 460800) from ever reaching AgOpenGPS as a PGN.
//   - Text lines from the XIAO (printable ASCII ending in a line feed) go to the web page log as
//     "[xiao] > ...". Handy when the XIAO is mounted and its USB-C must not be used.
//   - Never waits: when Serial2's transmit buffer is full the packet for the XIAO is dropped
//     (counted), the steering and GNSS loop is never held up.
//   - The XIAO's frames and text lines are only taken while it is present (hello in the last 10 s):
//     whatever else might sit in the GPS2 socket is ignored. So the XIAO must say hello first.
//   - Can be switched off on the web page (Definições): the Teensy then ignores the XIAO.
// Additions, only between the Teensy and the XIAO (never on the network), same frame format
// and checksum, PGN numbers that AgOpenGPS does not use:
//   Teensy -> XIAO  PGN 141 (source 0x7E): the module's status, the same JSON as the web page's
//                   /api/status, once a second, in pieces of up to 240 bytes:
//                   data = [set number][piece 0..n-1][n][JSON text]
//   Teensy -> XIAO  PGN 142 (source 0x7E): each [status] / [boot] / [web] / [xiao] line of the log:
//                   data = [sequence number, 4 bytes LE][Teensy millis, 4 bytes LE][text]
//                   After a hello that follows a silence the last LOG_LINES lines are sent again.
//   XIAO -> Teensy  PGN 143: the XIAO's own state as a short JSON text (program version, WiFi),
//                   kept by the Teensy for its web page (XIAO card) and NOT sent to UDP 9999.
//   XIAO -> Teensy  PGN 144: a command for the Teensy, NOT sent to UDP 9999: data = [1][a][b][c] sets
//                   the module's subnet to a.b.c (as AgIO's PGN 201; netSetSubnet, zEthernet.ino) and
//                   restarts the Teensy (2026-10-24: from the XIAO page, for a module on another network).
//   XIAO -> Teensy  PGN 145 (2026-11-02): a firmware update from the XIAO page, for a module whose own page cannot
//                   be reached (NOT sent to UDP 9999): data = [1][flags][file size, 4 bytes LE][file name] begins it
//                   (flags bit 0 = "other firmware", no family marker needed); [2][block number, 2 bytes LE][up to
//                   252 bytes of the .hex file] one block, in order from 1; [3] the end (every check, install,
//                   restart); [4] cancel; [5] go back to the previous firmware kept in the flash (zOTA.ino).
//   Teensy -> XIAO  PGN 145: the answer to each of those, before the XIAO sends the next:
//                   data = [command][block number, 2 bytes LE][0 = ok, 1 = refused][text: why, or the size].
//   These go out only when Serial2's transmit buffer has room to spare (XIAO_TX_RESERVE), so the
//   AgOpenGPS packets for the XIAO always come first (the PGN 145 answers wait for room instead).
// The WiFi bridge (22 Nov 2026), for a tablet with no network cable: the program runs on the XIAO's own WiFi
// and the XIAO carries its datagrams to and from the Teensy on this link (see "the WiFi bridge" below):
//   XIAO -> Teensy  PGN 146: a datagram the program sent to UDP 8888 on the XIAO's WiFi:
//                   data = [the program's IP, 4 bytes][the datagram, as sent]
//   XIAO -> Teensy  PGN 147: bytes of the corrections the program sent to UDP 2233: data = [the bytes]
//   Teensy -> XIAO  PGN 148: a datagram for the program's UDP 9999 (a GPS sentence, a PGN): data = [the datagram].
//                   The XIAO sends it to that program only, never to everyone.
//   Teensy -> XIAO  PGN 149: the bridge's state, once a second and when it changes:
//                   data = [version][0 free / 1 a program on the cable has the module / 2 the program on the WiFi has it]
//                   A XIAO that does not hear it sends no PGN 146 (an older Teensy would send them on to UDP 9999).
//                   Version 2 (Nov 2026): this firmware also serves its page through the XIAO (PGN 150 / 151).
//                   Version 3 (Nov 2026): it also takes PGN 152.
//   XIAO -> Teensy  PGN 152: a machine module (sprayer, seed drill) on the XIAO's WiFi sent this to UDP 9999:
//                   data = [the module's IP, 4 bytes][the datagram]; only for the Machine tab (zMachine.ino), never sent on.
// The module's page through the XIAO (Nov 2026; "the page through the XIAO" below), for a phone on the XIAO's WiFi:
//   XIAO -> Teensy  PGN 150: [1 open][session][window, 2 bytes LE] a browser's new request (window: answer bytes the
//                   XIAO holds for it); [2][session][offset, 2 bytes LE][bytes] the request; [3][session] the browser
//                   went; [4][session][count, 2 bytes LE] answer bytes the XIAO has passed on to the browser so far
//   Teensy -> XIAO  PGN 151: [2][session][offset, 2 bytes LE][bytes] the answer; [3][session] its end. Never more than
//                   the window ahead of the count; offsets and counts are the low 16 bits of the bytes so far.
//
// The XIAO is powered from the board's 3.3 V: never plug its USB-C (with 5 V) while it is on the board.
// The Teensy cannot program the XIAO (its BOOT / RESET pins are not wired): program it over USB-C
// off the board, or over WiFi if its own program has an update page.

#define SerialXIAO              Serial2
// Link speed (XIAO_BAUD, zXiao.h). AiO_WEB 4.5: 460800 = 46 kB/s each way, as v26 and as the second
// F9P on the same port. The most the link carries today is AgOpenWeb's PGN 254 + 239 (+ 229 with
// more than 16 sections) at 100 Hz: 2.8-4.4 kB/s, under 10 % of it. The XIAO, whose single core
// also runs the WiFi, empties its 128-byte UART FIFO at 64 bytes (setRxFIFOFull in its program):
// 1.4 ms of room at this speed. The XIAO program must use the same value (LINK_BAUD).
#define XIAO_HELLO_TIMEOUT_MS   10000   // as v26: no hello for this long = XIAO gone
#define XIAO_GAP_MS             50      // a frame not finished this long after its last byte is dropped
#define XIAO_FRAMES_PER_LOOP    8       // at most this many frames to UDP per loop (the loop stays fast)
#define XIAO_TEXT_MAX           160     // longer text lines are cut
#define XIAO_TEXT_PER_10S       10      // at most this many text lines per 10 s in the log
#define XIAO_STATUS_MS          1000    // module status to the XIAO this often
#define XIAO_STATUS_MAX         3072    // largest status JSON (it is about 1.3-2.5 kB)
#define XIAO_LOG_PER_LOOP       4       // at most this many log lines to the XIAO per loop
#define XIAO_TX_RESERVE         256     // room always left in the transmit buffer for AgOpenGPS packets
#define XIAO_SRC_TEENSY         0x7E    // source byte of the Teensy's own frames (steer module)
#define XIAO_PGN_STATUS         141     // Teensy -> XIAO: status JSON piece
#define XIAO_PGN_LOG            142     // Teensy -> XIAO: one log line
#define XIAO_PGN_INFO           143     // XIAO -> Teensy: XIAO info JSON (not sent to UDP)
#define XIAO_PGN_CMD            144     // XIAO -> Teensy: a command (not sent to UDP): 1 = subnet a.b.c
#define XIAO_PGN_FW             145     // both ways: a firmware update for the Teensy from the XIAO page (not sent to UDP)
#define XIAO_PGN_NET_IN         146     // XIAO -> Teensy: a datagram from the program on the XIAO's WiFi (not sent to UDP)
#define XIAO_PGN_RTCM           147     // XIAO -> Teensy: corrections from the program on the XIAO's WiFi (not sent to UDP)
#define XIAO_PGN_NET_OUT        148     // Teensy -> XIAO: a datagram for the program on the XIAO's WiFi
#define XIAO_PGN_BRIDGE         149     // Teensy -> XIAO: the bridge's state
#define XIAO_PGN_WEB_IN         150     // XIAO -> Teensy: the page through the XIAO, a browser's request (not sent to UDP)
#define XIAO_PGN_WEB_OUT        151     // Teensy -> XIAO: the answer
#define XIAO_PGN_MACH_IN        152     // XIAO -> Teensy: a machine module's datagram on the XIAO's WiFi (not sent to UDP)
#define BRIDGE_VERSION          3       // 2: the page through the XIAO too (PGN 150 / 151); 3: PGN 152 too
#define XWEB_PIECE              240     // answer bytes per PGN 151
#define XWEB_STUCK_MS           5000    // an answer that cannot move for this long (the XIAO takes nothing): given up
#define BRIDGE_CABLE_MS         3000    // a program packet on the cable in this time: the cable has the module
#define BRIDGE_GONE_MS          3000    // nothing from the program on the WiFi for this long: the module is free again
#define BRIDGE_STATE_MS         1000    // the state to the XIAO this often
#define XIAO_TX_IDLE_WIFI       128     // while the program is on the WiFi: the Teensy's own frames wait until the
                                        // transmit buffer holds less than this (a status piece is 5 ms of the link)
#define XIAO_FW_TIMEOUT_MS      15000   // nothing from the XIAO for this long during an update: given up
#define XIAO_INFO_MAX           250

static const char XIAO_HELLO[] = "ESP32-hello";

#include "zXiao.h"   // XiaoStats (included first by the main .ino)

uint8_t XIAOrxbuffer[1024];
uint8_t XIAOtxbuffer[2048];            // 2 kB (status JSON pieces + log lines + AgOpenGPS packets)

XiaoStats xiao = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

char     xiaoInfo[XIAO_INFO_MAX + 1] = "";
uint32_t xiaoInfoMs = 0;

static char     xsBuf[XIAO_STATUS_MAX];   // status JSON being sent in pieces
static uint16_t xsLen = 0, xsPos = 0;     // xsLen 0 = nothing being sent
static uint8_t  xsSet = 0, xsPart = 0, xsParts = 0;
static uint32_t xsLastMs = 0;
static uint32_t xlNext = 1;               // next log sequence number to send to the XIAO

bool     xiaoDetected    = false;
uint32_t xiaoLastHelloMs = 0;

static uint8_t  xf[5 + 255 + 1 + 1];   // frame being received (max 261 bytes)
static uint16_t xfLen       = 0;
static uint32_t xfLastMs    = 0;       // time of the last byte added to xf
static uint32_t xRxMs       = 0;       // time of the last byte from the XIAO
static uint8_t  helloPos    = 0;
static char     xtext[XIAO_TEXT_MAX + 1];
static uint8_t  xtextLen    = 0;
static bool     xtextBad    = false;   // the line has non-printable bytes (boot garbage): not logged
static uint32_t xtextWinMs  = 0;
static uint8_t  xtextWinN   = 0;
static uint32_t xcrcLogMs   = 0;
static uint32_t xcrcLogged  = 0;
static bool     xFirstFrame = true;    // log the first good frame after each hello / detection

static inline bool xiaoOn() { return gps2XiaoOn(); }

// ---------------------------------------------------------------- the WiFi bridge (22 Nov 2026)
// A tablet with no network cable (no USB-Ethernet adapter) joins the XIAO's own WiFi and runs AgOpenWeb / AgIO
// there. The XIAO listens on UDP 8888 and 2233 on its WiFi and hands each datagram to the Teensy (PGN 146, 147);
// the Teensy takes it as if it had come in on the network (programPacket, Autosteer.ino) and what it sends to
// the program (GPS sentences, PGN 253, 250, 126, 121, 221) goes to the XIAO (PGN 148), which sends it to that
// program only. Never to everyone: on a router's WiFi the module's broadcasts were held and came 1.5 s late,
// half of them lost (measured on a tractor, 22 Nov 2026).
// ONE way at a time, decided here. The cable wins: a program packet on the network (UDP 8888) in the last
// BRIDGE_CABLE_MS keeps the module the cable's, and the packets from the WiFi are not taken. With no program on
// the cable, the first packet from the WiFi makes the module the WiFi's: from then on NOTHING for the program
// goes out on the network (a tablet on both would get everything twice, from two addresses), until a program
// talks on the cable or the WiFi is silent for BRIDGE_GONE_MS. A link that dies is covered by what covers a
// pulled cable: no PGN 254, the steering watchdog turns the outputs off (Autosteer.ino).
// Switched on and off on the XIAO's page (off from new); the Teensy has no setting for it.
static uint8_t  brState     = BRIDGE_FREE;
static uint8_t  brStateSent = 255;       // the state the XIAO was last told (255 = not yet)
static uint32_t brStateMs   = 0;         // when (PGN 149)
static uint32_t brCableMs   = 0;         // the last program packet on the cable (msStamp; 0 = none since the start)
static uint32_t brWifiMs    = 0;         // the last program packet taken from the WiFi
static uint32_t brIgnoreMs  = 0;         // the last "not taken" line in the log (0 = none yet)
static uint8_t  brIp[4]     = {0, 0, 0, 0};   // the program on the WiFi

bool    xiaoBridgeWifi()  { return brState == BRIDGE_WIFI; }
uint8_t xiaoBridgeState() { return brState; }

// Called from ReceiveUdp() (Autosteer.ino) for every packet on UDP 8888 of the network
void xiaoBridgeNoteCable()
{
    brCableMs = msStamp(millis());
    if (brState == BRIDGE_WIFI)
        StatusLog.line("[xiao] WiFi bridge: a program is talking on the network cable - the cable has the module, the program on the XIAO's WiFi (%u.%u.%u.%u) is no longer answered",
                       brIp[0], brIp[1], brIp[2], brIp[3]);
    brState = BRIDGE_CABLE;
}

// one frame to the XIAO, as it is (the caller has checked the room)
static void xiaoWriteFrame(uint8_t pgn, const uint8_t* data, uint8_t len)
{
    uint8_t h[5] = {0x80, 0x81, XIAO_SRC_TEENSY, pgn, len};
    uint8_t sum = XIAO_SRC_TEENSY + pgn + len;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    SerialXIAO.write(h, 5);
    SerialXIAO.write(data, len);
    SerialXIAO.write(&sum, 1);
    xiao.txBytes += len + 6;
}

// One datagram for the program on the XIAO's WiFi (what would go to UDP 9999 on the network). Never waits:
// with the transmit buffer full it is dropped and counted, like a packet for the XIAO.
void xiaoNetOut(const uint8_t* d, uint16_t n)
{
    if (n == 0 || n > 255 || SerialXIAO.availableForWrite() < (int)n + 6) { xiao.netOutDrop++; return; }
    xiaoWriteFrame(XIAO_PGN_NET_OUT, d, (uint8_t)n);
    xiao.netOut++;
}

// AiO_WEB: the GPS2 port (Serial2) is the XIAO's or the second F9P's, found by itself, or off (cfg.xiaoOff,
// zDualF9P.ino); gps2Begin() starts it (460800 for both)
void xiaoSetup()
{
    SerialXIAO.addMemoryForRead(XIAOrxbuffer, sizeof(XIAOrxbuffer));
    SerialXIAO.addMemoryForWrite(XIAOtxbuffer, sizeof(XIAOtxbuffer));
    gps2Begin();
}

static void xiaoHello()
{
    xiao.hellos++;
    xiaoLastHelloMs = millis();
    if (!xiaoDetected)
    {
        xiaoDetected = true;
        xFirstFrame  = true;
        // send it the recent log again and the status at once (it may have restarted)
        xlNext   = StatusLog.nextSeq > LOG_LINES ? StatusLog.nextSeq - LOG_LINES : 1;
        xsLen    = 0;
        xsLastMs = millis() - XIAO_STATUS_MS;
        if (xiaoOn())
            StatusLog.line("[xiao] XIAO said hello on Serial2: AgOpenWeb packets (UDP 8888) now go to it, its PGNs go to UDP 9999");
        else
            StatusLog.line("[xiao] XIAO said hello on Serial2, but the GPS2 port is off in the settings: ignored");
    }
}

static void xiaoTextFlush()
{
    if (xtextLen > 0 && !xtextBad && xiaoOn() && xiaoDetected)
    {
        xtext[xtextLen] = '\0';
        const uint32_t now = millis();
        if (now - xtextWinMs > 10000) { xtextWinMs = now; xtextWinN = 0; }
        if (xtextWinN < XIAO_TEXT_PER_10S)
        {
            xtextWinN++;
            xiao.textLines++;
            StatusLog.line("[xiao] > %s", xtext);
        }
        else xiao.textDropped++;
    }
    xtextLen = 0;
    xtextBad = false;
}

static void xiaoText(uint8_t b)
{
    if (b == '\n') { xiaoTextFlush(); return; }
    if (b == '\r') return;
    if (b < 0x20 || b > 0x7E) { xtextBad = true; return; }
    if (xtextLen < XIAO_TEXT_MAX) xtext[xtextLen++] = (char)b;
}

static void xiaoBadCrc(uint8_t pgn, uint8_t got, uint8_t want)
{
    xiao.crcErr++;
    const uint32_t now = millis();
    if (xcrcLogged == 0 || now - xcrcLogMs >= 10000)
    {
        StatusLog.line("[xiao] frame from the XIAO dropped: bad checksum (PGN %u, got %02X, expected %02X; %lu so far)",
                       pgn, got, want, (unsigned long)xiao.crcErr);
        xcrcLogMs = now;
        xcrcLogged++;
    }
}

static void xiaoSendFrame(const uint8_t* d, uint16_t n)
{
    xiao.fromXiao++;
    xiao.lastPgn = d[3];
    if (xFirstFrame)
    {
        xFirstFrame = false;
        StatusLog.line("[xiao] first PGN from the XIAO: %u (source %u, %u data bytes) -> UDP %u", d[3], d[2], d[4], portDestination);
    }
    if (!Ethernet_running) return;
    Eth_udpAutoSteer.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpAutoSteer.write(d, n);
    Eth_udpAutoSteer.endPacket();
}

// the XIAO's info JSON (printable text only), shown in the XIAO card of the web page
static void xiaoTakeInfo(const uint8_t* d, uint8_t n)
{
    uint8_t k = 0;
    for (uint8_t i = 0; i < n && k < XIAO_INFO_MAX; i++)
        if (d[i] >= 0x20 && d[i] <= 0x7E) xiaoInfo[k++] = (char)d[i];
    xiaoInfo[k] = '\0';
    xiaoInfoMs = msStamp(millis());
    xiao.infoRx++;
}

// a command from the XIAO page (PGN 144): 1 = the module's subnet (the XIAO page is on WiFi, so it is
// reached whatever network the module is on). Saved; the Teensy restarts to take the new IP.
static void xiaoTakeCmd(const uint8_t* d, uint8_t n)
{
    if (n >= 4 && d[0] == 1)
    {
        const char* err = netSetSubnet(d[1], d[2], d[3], "the XIAO page");
        if (err) { StatusLog.line("[xiao] subnet %u.%u.%u refused: %s", d[1], d[2], d[3], err); return; }
        SCB_AIRCR = 0x05FA0004;                      // Teensy restart (as PGN 201 does)
        for (;;) {}
    }
    StatusLog.line("[xiao] unknown command from the XIAO: PGN 144, command %u, %u bytes", n ? d[0] : 0, n);
}

// ---------------------------------------------------------------- the Teensy's firmware from the XIAO page
// PGN 145 (2 Nov 2026): the XIAO page sends the .hex file over the link in blocks of up to 252 bytes, each one
// answered before the XIAO sends the next (so nothing arrives while a flash sector is being written, when the
// UART is not read), and the Teensy checks and installs it with the same code as a file from its own page
// (zOTA.ino: otaBegin, otaFeed, otaFinish, otaInstall). For a module whose own page cannot be reached (its
// Ethernet, its network, a bug in the page): the XIAO is on WiFi. While the file comes in the loop stops in
// xiaoFwRun() with the steering off, as for an update from the page; a 2 MB .hex takes about a minute.
// Command 5 puts the previous firmware kept in the flash back (zOTA.ino).
struct XiaoFw
{
    bool        pending;       // a begin was accepted: xiaoProcess() runs xiaoFwRun()
    bool        active;        // the blocks are coming (xiaoFwRun)
    bool        force;         // "other firmware": no family marker needed
    uint16_t    expect;        // the next block number
    uint32_t    size, got, lastMs;
    const char* err;
    char        name[32];
};
static XiaoFw xfw;

// The answer to the XIAO: [command][block number LE][0 ok / 1 refused][text]. Waits for room in the transmit
// buffer (up to 100 ms, then write() waits itself): during an update the loop is stopped anyway, and a refusal
// is a few bytes once.
static void xiaoFwAnswer(uint8_t cmd, uint16_t seq, uint8_t code, const char* text)
{
    uint8_t d[4 + 160];
    d[0] = cmd; d[1] = (uint8_t)seq; d[2] = (uint8_t)(seq >> 8); d[3] = code;
    size_t t = text ? strlen(text) : 0;
    if (t > sizeof(d) - 4) t = sizeof(d) - 4;
    memcpy(d + 4, text, t);
    const uint8_t len = (uint8_t)(4 + t);
    const uint32_t t0 = millis();
    while (SerialXIAO.availableForWrite() < (int)len + 6 && millis() - t0 < 100) { wdtFeed(); delayMicroseconds(200); }
    uint8_t h[5] = {0x80, 0x81, XIAO_SRC_TEENSY, XIAO_PGN_FW, len};
    uint8_t sum = XIAO_SRC_TEENSY + XIAO_PGN_FW + len;
    for (uint8_t i = 0; i < len; i++) sum += d[i];
    SerialXIAO.write(h, 5);
    SerialXIAO.write(d, len);
    SerialXIAO.write(&sum, 1);
    xiao.txBytes += len + 6;
}

static const char* xiaoFwRefusal()                 // the checks an update from the page makes too (zWeb.ino)
{
    if (autosteerEngaged()) return "autosteer is engaged - disengage it first";
    if (tractorMoving())    return "the tractor is moving - stop first (on a bench, switch simulator mode on)";
    return nullptr;
}

// A PGN 145 frame from the XIAO (from xfScan)
static void xiaoTakeFw(const uint8_t* d, uint8_t n)
{
    if (n < 1) return;
    const uint8_t  cmd = d[0];
    const uint16_t seq = n >= 3 ? (uint16_t)(d[1] | (d[2] << 8)) : 0;
    if (cmd == 1)                                                   // begin: checked here, run by xiaoFwRun()
    {
        if (xfw.active || xfw.pending) { xiaoFwAnswer(1, 0, 1, "an update is already running"); return; }
        uint32_t size = 0;
        if (n >= 6) memcpy(&size, d + 2, 4);
        const char* why = n < 6 ? "bad begin frame" : xiaoFwRefusal();
        if (!why && (size < 1000 || size > 8000000)) why = "no file, or file size not plausible";
        if (!why) why = otaBegin();
        if (why)
        {
            StatusLog.line("[xiao] firmware update from the XIAO page REFUSED: %s (old firmware kept)", why);
            xiaoFwAnswer(1, 0, 1, why);
            return;
        }
        memset(&xfw, 0, sizeof(xfw));
        xfw.force  = (d[1] & 1) != 0;
        xfw.size   = size;
        xfw.expect = 1;
        uint8_t k = 0;
        for (uint8_t i = 6; i < n && k < sizeof(xfw.name) - 1; i++) if (d[i] >= 0x20 && d[i] <= 0x7E) xfw.name[k++] = (char)d[i];
        xfw.pending = true;                                         // xiaoProcess() takes it from here
        return;
    }
    if (cmd == 5)                                                   // the previous firmware back (zOTA.ino)
    {
        const char* why = xiaoFwRefusal();
        if (!why) why = otaRevertCheck();
        if (why)
        {
            StatusLog.line("[xiao] going back to the previous firmware REFUSED: %s", why);
            xiaoFwAnswer(5, 0, 1, why);
            return;
        }
        const char *pf, *pb;
        uint32_t ps = 0;
        otaPrevGet(&pf, &pb, &ps);
        steerSafeOff();
        StatusLog.line("[xiao] going back to the previous firmware from the XIAO page: %s, built %s (%lu bytes), installing and restarting, steering off",
                       pf, pb, (unsigned long)ps);
        xiaoFwAnswer(5, 0, 0, "");
        SerialXIAO.flush();
        delay(50);
        otaRevertInstall();                                         // does not return
    }
    if (!xfw.active)
    {
        if (cmd == 2 || cmd == 3) xiaoFwAnswer(cmd, seq, 1, "no update running");
        return;
    }
    xfw.lastMs = millis();
    if (cmd == 2)                                                   // a block of the file
    {
        if (seq + 1 == xfw.expect) { xiaoFwAnswer(2, seq, 0, ""); return; }   // the last block again: its answer was lost
        if (seq != xfw.expect) xfw.err = "blocks out of order (link error) - try again";
        else                   xfw.err = otaFeed(d + 3, n - 3);
        if (!xfw.err) { xfw.got += n - 3; xfw.expect++; xiaoFwAnswer(2, seq, 0, ""); return; }
    }
    else if (cmd == 3)                                              // the end: every check, then install
    {
        uint32_t size = 0;
        xfw.err = otaFinish(xfw.force, &size);
        if (!xfw.err)
        {
            StatusLog.line("[xiao] firmware update: %lu bytes checked, installing and restarting", (unsigned long)size);
            char t[16];
            snprintf(t, sizeof(t), "%lu", (unsigned long)size);
            xiaoFwAnswer(3, 0, 0, t);
            SerialXIAO.flush();
            delay(50);
            otaInstall(size);                                       // does not return
        }
    }
    else if (cmd == 4) xfw.err = "cancelled from the XIAO page";
    else return;
    xiaoFwAnswer(cmd, seq, 1, xfw.err);                             // refused: the old firmware keeps running
    xfw.active = false;
}

// The update takes over the loop (from xiaoProcess(), after an accepted begin): steering off, watchdog fed, the
// network stack going on. Every byte from the XIAO goes through the frame reader, so the blocks reach
// xiaoTakeFw(). Ends with the install (a restart) or an error; XIAO_FW_TIMEOUT_MS without a block = given up.
static void xiaoFwRun()
{
    xfw.pending = false;
    xfw.active  = true;
    xfw.lastMs  = millis();
    steerSafeOff();
    StatusLog.line("[xiao] firmware update from the XIAO page: receiving %s (%lu bytes) over the XIAO link, steering off",
                   xfw.name[0] ? xfw.name : "a .hex file", (unsigned long)xfw.size);
    xiaoFwAnswer(1, 0, 0, "");
    loopStep = 20;                                                  // for the watchdog report (zSafety.ino)
    while (xfw.active)
    {
        wdtFeed();
        netPoll();
        while (xfw.active && SerialXIAO.available() > 0) xiaoByte((uint8_t)SerialXIAO.read());
        if (xfw.active && millis() - xfw.lastMs > XIAO_FW_TIMEOUT_MS)
        {
            xfw.err = "upload stopped (timeout): nothing from the XIAO for 15 s";
            xiaoFwAnswer(2, xfw.expect, 1, xfw.err);
            xfw.active = false;
        }
        if (xfw.active) delayMicroseconds(50);
    }
    StatusLog.line("[xiao] firmware update REFUSED: %s (old firmware kept)", xfw.err ? xfw.err : "stopped");
    xfLen = 0;
    xtextLen = 0;
    xtextBad = false;
    loopStep = 7;
}

// A datagram the program sent to UDP 8888 on the XIAO's WiFi (PGN 146): [its IP, 4 bytes][the datagram].
// Taken only with no program on the cable ("the WiFi bridge", at the top), and not during a firmware update.
static void xiaoTakeNet(uint8_t* d, uint8_t n)
{
    if (n < 4 + 5 || xfw.active) return;                   // an IP and at least 80 81 src pgn len
    const uint32_t now = millis();
    if (brCableMs != 0 && now - brCableMs < BRIDGE_CABLE_MS)
    {
        xiao.netIgnored++;
        if (brIgnoreMs == 0 || now - brIgnoreMs >= 30000)  // one line per 30 s at most
        {
            brIgnoreMs = msStamp(now);
            StatusLog.line("[xiao] WiFi bridge: a program on the XIAO's WiFi (%u.%u.%u.%u) is calling, but one is talking on the network cable: the cable has the module, the WiFi one is not answered",
                           d[0], d[1], d[2], d[3]);
        }
        return;
    }
    if (brState != BRIDGE_WIFI)
    {
        brState = BRIDGE_WIFI;
        StatusLog.line("[xiao] WiFi bridge: the program at %u.%u.%u.%u is on the XIAO's WiFi - GPS and steering go to it through the XIAO, nothing for the program goes out on the network",
                       d[0], d[1], d[2], d[3]);
    }
    memcpy(brIp, d, 4);
    brWifiMs = msStamp(now);
    xiao.netIn++;
    const IPAddress from(d[0], d[1], d[2], d[3]);
    statusNoteAogPacket(from);                             // [status] messages, the LEDs (zStatus.ino)
    programPacket(d + 4, (int)n - 4, from, true);          // as a packet on UDP 8888 (Autosteer.ino)
}

// Corrections the program sent to UDP 2233 on the XIAO's WiFi (PGN 147), to the receiver as udpNtrip() does
// (zUdpNtrip.ino). Only from the program that has the module.
static void xiaoTakeRtcm(const uint8_t* d, uint8_t n)
{
    if (n == 0 || brState != BRIDGE_WIFI || xfw.active) return;
    if (!rxCkOwns(RXCK_GPS)) SerialGPS->write(d, n);       // (not while the receiver check searches its baud rate)
    statusNoteRtcm(d, n);
    xiao.rtcmIn += n;
}

// The bridge's clock, from xiaoProcess(): the module is free again when the WiFi (or the cable) has been
// silent, or the XIAO is gone; the XIAO is told the state once a second and at each change.
static void xiaoBridgeTick()
{
    const uint32_t now = millis();
    const bool here = xiaoDetected && xiaoOn();
    if (brState == BRIDGE_WIFI && (!here || now - brWifiMs >= BRIDGE_GONE_MS))
    {
        brState = BRIDGE_FREE;
        StatusLog.line("[xiao] WiFi bridge: %s - the module is free again, GPS goes out on the network",
                       here ? "nothing from the program on the XIAO's WiFi for 3 s" : "the XIAO is gone");
    }
    else if (brState == BRIDGE_CABLE && now - brCableMs >= BRIDGE_CABLE_MS) brState = BRIDGE_FREE;

    if (!here) { brStateSent = 255; return; }
    if ((brState != brStateSent || now - brStateMs >= BRIDGE_STATE_MS) && SerialXIAO.availableForWrite() >= 3 + 6)
    {
        const uint8_t d[3] = { BRIDGE_VERSION, brState, 0 };
        xiaoWriteFrame(XIAO_PGN_BRIDGE, d, 3);
        brStateSent = brState;
        brStateMs = now;
    }
}

// One frame of the Teensy's own to the XIAO, only with room to spare in the transmit buffer. While the program
// is on the XIAO's WiFi its datagrams share the link: the Teensy's own frames then go one at a time into a
// buffer that is nearly empty, so a GPS sentence never waits behind more than one of them.
static bool xiaoSendOwn(uint8_t pgn, const uint8_t* data, uint8_t len)
{
    const int room = SerialXIAO.availableForWrite();
    if (brState == BRIDGE_WIFI ? room < (int)sizeof(XIAOtxbuffer) - XIAO_TX_IDLE_WIFI
                               : room < (int)len + 6 + XIAO_TX_RESERVE) return false;
    xiaoWriteFrame(pgn, data, len);
    return true;
}

// the module status (once a second, in pieces) and the new log lines, to the XIAO
static void xiaoTxTick(uint32_t now)
{
    if (!xiaoDetected || !xiaoOn()) { xsLen = 0; return; }

    // log lines (the backlog after a hello goes out a few per loop)
    for (uint8_t n = 0; n < XIAO_LOG_PER_LOOP && xlNext < StatusLog.nextSeq; n++)
    {
        const LogLine& l = StatusLog.lines[xlNext % LOG_LINES];
        if (l.seq != xlNext) { xlNext++; continue; }          // already overwritten in the ring
        uint8_t d[8 + 200];
        size_t t = strlen(l.text);
        if (t > sizeof(d) - 8) t = sizeof(d) - 8;
        memcpy(d, &l.seq, 4);                                // Teensy 4.1 is little endian
        memcpy(d + 4, &l.ms, 4);
        memcpy(d + 8, l.text, t);
        if (!xiaoSendOwn(XIAO_PGN_LOG, d, (uint8_t)(8 + t))) return;   // no room: next loop
        xlNext++;
        xiao.logSent++;
    }

    // status: build it once a second, then send it piece by piece as the buffer allows
    if (xsLen == 0 && now - xsLastMs >= XIAO_STATUS_MS)
    {
        xsLastMs = now;
        Json j(xsBuf, sizeof(xsBuf));
        statusJson(j, now, false);          // without the parts only the web page shows
        if (j.n + 2 < j.cap)                                 // not cut short
        {
            xsLen = (uint16_t)j.n;
            xsPos = 0;
            xsSet++;
            xsPart = 0;
            xsParts = (uint8_t)((xsLen + 239) / 240);
        }
    }
    while (xsLen > 0 && xsPart < xsParts)
    {
        const uint16_t k = (xsLen - xsPos) < 240 ? (xsLen - xsPos) : 240;
        uint8_t d[3 + 240];
        d[0] = xsSet; d[1] = xsPart; d[2] = xsParts;
        memcpy(d + 3, xsBuf + xsPos, k);
        if (!xiaoSendOwn(XIAO_PGN_STATUS, d, (uint8_t)(3 + k))) return;
        xsPos += k;
        xsPart++;
    }
    if (xsLen > 0 && xsPart >= xsParts) { xsLen = 0; xiao.statusSent++; }
}

// ---------------------------------------------------------------- the page through the XIAO
// A phone on the XIAO's own WiFi cannot reach the module's network. The XIAO listens on port 8080 of its own network,
// passes what a browser sends there on to us (PGN 150) and our answer back to it (PGN 151). webLoop() (zWeb.ino) takes
// it as one more connection, through xiaoWeb, so every page and every answer is the one the cable gets. The answer goes
// out as the link has room for the Teensy's own frames (xiaoSendOwn): while a tablet steers through the bridge its
// datagrams come first, and a piece of the page makes one of them wait no more than about 5 ms (246 bytes at 460800).
// The XIAO says at "open" how much of the answer it can hold, and then how much it has passed on to the browser:
// never more than that is on the way, so nothing is lost when the phone's WiFi is slow.
XiaoWebClient xiaoWeb;
DMAMEM static uint8_t xwRx[XWEB_RX_MAX];      // in RAM2: RAM1 is the code's and the stack's (firmware CLAUDE.md, gotcha 13)
DMAMEM static uint8_t xwTx[XWEB_TX_MAX];

FLASHMEM size_t XiaoWebClient::write(const uint8_t* buf, size_t size)
{
    if (!open || gone || ending) return 0;
    size_t n = 0;
    while (n < size && txLen < XWEB_TX_MAX)
    {
        const uint16_t at = (uint16_t)((txHead + txLen) % XWEB_TX_MAX);
        size_t k = XWEB_TX_MAX - at;                            // up to the ring's end
        if (k > (size_t)(XWEB_TX_MAX - txLen)) k = XWEB_TX_MAX - txLen;
        if (k > size - n) k = size - n;
        memcpy(xwTx + at, buf + n, k);
        txLen += (uint16_t)k;
        n += k;
    }
    return n;
}
FLASHMEM int XiaoWebClient::availableForWrite() { return open && !gone && !ending ? XWEB_TX_MAX - txLen : 0; }
FLASHMEM int XiaoWebClient::available()         { return open ? rxLen - rxPos : 0; }
FLASHMEM int XiaoWebClient::read()              { return available() > 0 ? xwRx[rxPos++] : -1; }
FLASHMEM int XiaoWebClient::peek()              { return available() > 0 ? xwRx[rxPos] : -1; }
FLASHMEM int XiaoWebClient::read(uint8_t* buf, size_t size)
{
    int n = available();
    if (n <= 0) return -1;
    if ((size_t)n > size) n = (int)size;
    memcpy(buf, xwRx + rxPos, n);
    rxPos += (uint16_t)n;
    return n;
}
// as a socket: still "connected" while what the browser sent is not all read
FLASHMEM uint8_t XiaoWebClient::connected()     { return open && (!gone || rxPos < rxLen) ? 1 : 0; }
FLASHMEM void XiaoWebClient::stop()
{
    rxLen = rxPos = 0;
    waiting = false;
    if (!open) return;
    if (gone) { open = false; txLen = 0; return; }             // the browser went: nobody to send the rest to
    ending = true;                                              // the rest goes out, then the end (xiaoWebPump)
    moveMs = msStamp(millis());
}

// PGN 150 from the XIAO
FLASHMEM static void xiaoTakeWeb(const uint8_t* d, uint8_t n)
{
    if (n < 2) return;
    XiaoWebClient& w = xiaoWeb;
    const uint8_t cmd = d[0], sid = d[1];
    if (cmd == 1 && n >= 4)                                     // a browser's new request
    {
        if (w.open) webXiaoDropped();                           // the XIAO is done with the one before (it said so)
        w.open = true;
        w.waiting = true;
        w.gone = w.ending = false;
        w.sid = sid;
        w.window = (uint16_t)(d[2] | d[3] << 8);
        if (w.window < XWEB_PIECE) w.window = XWEB_PIECE;
        w.rxLen = w.rxPos = 0;
        w.rxGot = 0;
        w.txHead = w.txLen = 0;
        w.txSent = w.txTaken = 0;
        w.moveMs = msStamp(millis());
        xiao.webReq++;
        return;
    }
    if (!w.open || sid != w.sid) return;                        // an older request's: done with
    if (cmd == 2 && n >= 4)                                     // the request's bytes
    {
        const uint16_t off = (uint16_t)(d[2] | d[3] << 8);
        if (off != (uint16_t)w.rxGot) { if (!w.gone) xiao.webBroken++; w.gone = true; return; }   // a piece lost on the link
        const uint16_t k = n - 4;
        w.rxGot += k;
        const uint16_t room = XWEB_RX_MAX - w.rxLen;
        const uint16_t take = k < room ? k : room;              // more than a request can be is dropped (413 in webLoop)
        memcpy(xwRx + w.rxLen, d + 4, take);
        w.rxLen += take;
    }
    else if (cmd == 3) w.gone = true;                           // the browser went
    else if (cmd == 4 && n >= 4)                                // answer bytes the XIAO has passed on to the browser
    {
        const uint16_t v = (uint16_t)(d[2] | d[3] << 8);
        const uint32_t taken = w.txTaken + (uint16_t)(v - (uint16_t)w.txTaken);
        if (taken <= w.txSent && taken != w.txTaken) { w.txTaken = taken; w.moveMs = msStamp(millis()); }
    }
}

// The answer's next pieces to the XIAO, as the link and the XIAO's window allow, then its end. Called from xiaoProcess().
FLASHMEM static void xiaoWebPump()
{
    XiaoWebClient& w = xiaoWeb;
    if (!w.open) return;
    const uint32_t now = millis();
    if (!xiaoDetected || !xiaoOn()) w.gone = true;              // the XIAO went: nobody to send it to
    if (w.gone) { if (w.ending) { w.open = false; w.txLen = 0; } return; }
    for (uint8_t i = 0; i < 4 && w.txLen > 0; i++)
    {
        const uint32_t ahead = w.txSent - w.txTaken;
        if (ahead >= w.window) break;                           // the XIAO holds all it can: wait for its count
        uint16_t k = w.txLen;
        if (k > XWEB_PIECE) k = XWEB_PIECE;
        if (k > XWEB_TX_MAX - w.txHead) k = XWEB_TX_MAX - w.txHead;   // up to the ring's end
        if (k > w.window - ahead) k = (uint16_t)(w.window - ahead);
        uint8_t d[4 + XWEB_PIECE];
        d[0] = 2;
        d[1] = w.sid;
        d[2] = (uint8_t)w.txSent;
        d[3] = (uint8_t)(w.txSent >> 8);
        memcpy(d + 4, xwTx + w.txHead, k);
        if (!xiaoSendOwn(XIAO_PGN_WEB_OUT, d, (uint8_t)(4 + k))) return;   // no room on the link now: next pass
        w.txHead = (uint16_t)((w.txHead + k) % XWEB_TX_MAX);
        w.txLen -= k;
        w.txSent += k;
        w.moveMs = msStamp(now);
    }
    if (w.ending && w.txLen == 0)
    {
        const uint8_t d[2] = { 3, w.sid };
        if (xiaoSendOwn(XIAO_PGN_WEB_OUT, d, 2)) { w.open = false; return; }
    }
    if (w.ending && now - w.moveMs > XWEB_STUCK_MS)             // the XIAO takes nothing more (its browser stopped): given up
    {
        w.open = false;
        w.txLen = 0;
        xiao.webBroken++;
    }
}

// Before a restart asked through the XIAO (/api/reboot): its short answer and the end go out first
FLASHMEM void xiaoWebFlush()
{
    const uint32_t t0 = millis();
    while (xiaoWeb.open && millis() - t0 < 500)
    {
        xiaoWebPump();
        wdtFeed();
        delay(1);
    }
    SerialXIAO.flush();
}

// Drop the first n bytes of xf and anything up to the next 0x80 (a possible frame start)
static void xfShift(uint16_t n)
{
    while (n < xfLen && xf[n] != 0x80) n++;
    if (n >= xfLen) { xfLen = 0; return; }
    memmove(xf, xf + n, xfLen - n);
    xfLen -= n;
}

// Take the complete frames out of xf. Returns how many good frames were sent (to UDP, or taken as program packets).
// After a bad checksum the search starts again one byte later: when a byte was lost, the next
// frame may already be inside the bytes received.
static uint8_t xfScan()
{
    uint8_t sent = 0;
    while (xfLen > 0)
    {
        if (xf[0] != 0x80 || (xfLen >= 2 && xf[1] != 0x81)) { xfShift(1); continue; }
        if (xfLen < 5) break;
        const uint16_t need = 6 + xf[4];
        if (xfLen < need) break;
        uint8_t sum = 0;
        for (uint16_t i = 2; i < need - 1; i++) sum += xf[i];
        if (sum == xf[need - 1])
        {
            if (!xiaoDetected)                 xiao.beforeHello++;   // no hello yet (or not for 10 s): not sent
            else if (xf[3] == XIAO_PGN_INFO)   xiaoTakeInfo(xf + 5, xf[4]);   // kept here, not sent to UDP
            else if (xf[3] == XIAO_PGN_CMD)    xiaoTakeCmd(xf + 5, xf[4]);    // a command: the subnet (not sent to UDP)
            else if (xf[3] == XIAO_PGN_FW)     xiaoTakeFw(xf + 5, xf[4]);     // a firmware update for the Teensy (not sent to UDP)
            else if (xf[3] == XIAO_PGN_NET_IN) { xiaoTakeNet(xf + 5, xf[4]); sent++; }   // the WiFi bridge: a program packet (counted: one of this loop's frames)
            else if (xf[3] == XIAO_PGN_RTCM)   xiaoTakeRtcm(xf + 5, xf[4]);   // the WiFi bridge: corrections for the receiver
            else if (xf[3] == XIAO_PGN_WEB_IN) xiaoTakeWeb(xf + 5, xf[4]);    // the page through the XIAO: a browser's request
            else if (xf[3] == XIAO_PGN_MACH_IN) machineFromXiao(xf + 5, xf[4]);   // a machine module on the XIAO's WiFi: Machine tab only
            else                               { xiaoSendFrame(xf, need); sent++; }
            xfShift(need);
        }
        else
        {
            xiaoBadCrc(xf[3], xf[need - 1], sum);
            xfShift(1);
        }
    }
    return sent;
}

// One byte from the XIAO. Returns how many frames it completed.
static uint8_t xiaoByte(uint8_t b)
{
    xiao.rxBytes++;
    gps2SniffByte(b);                          // AiO_WEB: a GNSS receiver on the port instead of the XIAO? (zDualF9P.ino)

    // "ESP32-hello" wherever it is (between frames, inside a text line)
    if (b == (uint8_t)XIAO_HELLO[helloPos])
    {
        if (++helloPos == sizeof(XIAO_HELLO) - 1)
        {
            helloPos = 0;
            xiaoHello();
            xtextLen = 0;                      // the hello is not a text line
            xtextBad = false;
            return 0;
        }
    }
    else helloPos = (b == (uint8_t)XIAO_HELLO[0]) ? 1 : 0;

    if (!xiaoOn()) return 0;                   // bridge off: only the hello is looked at

    if (xfLen == 0 && b != 0x80) { xiaoText(b); return 0; }
    if (xfLen == 0) { xtextLen = 0; xtextBad = false; }   // binary from here: a half text line is dropped
    xf[xfLen++] = b;                           // never overflows: xfScan keeps xfLen < 261
    xfLastMs = millis();
    return xfScan();
}

// Called from loop()
void xiaoProcess()
{
    const uint32_t now = millis();
    if (dualF9pOn())                           // AiO_WEB: the GPS2 port is the second F9P's (zDualF9P.ino)
    {
        if (xiaoDetected) xiaoDetected = false;
        xfLen = 0;
        xtextLen = 0;
        xiaoBridgeTick();                      // (no XIAO: a program that was on its WiFi no longer has the module)
        return;
    }

    // XIAO gone?
    if (xiaoDetected && now - xiaoLastHelloMs > XIAO_HELLO_TIMEOUT_MS)
    {
        xiaoDetected = false;
        StatusLog.line("[xiao] XIAO silent for %u s (no hello): stopped sending it the AgOpenWeb packets", XIAO_HELLO_TIMEOUT_MS / 1000);
    }

    // A frame that stopped half way (bytes lost, or the XIAO restarted while sending): drop its
    // first byte and look again in the bytes received after it (a whole frame may be in there)
    uint8_t frames = 0;
    if (xfLen > 0 && now - xfLastMs > XIAO_GAP_MS && SerialXIAO.available() == 0)
    {
        xiao.cut++;
        xfShift(1);
        frames += xfScan();
        xfLastMs = now;
    }

    // A text line left half way for 0.5 s (boot garbage, a lost line feed) is dropped
    if ((xtextLen > 0 || xtextBad) && now - xRxMs > 500 && SerialXIAO.available() == 0)
    {
        xtextLen = 0;
        xtextBad = false;
    }

    int budget = 2048;                         // bytes per call, so a flood cannot stall the loop
    while (budget-- > 0 && frames < XIAO_FRAMES_PER_LOOP && SerialXIAO.available() > 0)
    {
        frames += xiaoByte((uint8_t)SerialXIAO.read());
        xRxMs = now;
    }

    if (xfw.pending) xiaoFwRun();              // a firmware update from the XIAO page: takes over the loop until done

    xiaoBridgeTick();                          // the WiFi bridge: free again after a silence, its state to the XIAO
    xiaoTxTick(now);                           // status and log lines to the XIAO
    xiaoWebPump();                             // the page through the XIAO: the answer, after those
}

// Called from ReceiveUdp() (Autosteer.ino) with every packet that arrives on UDP 8888.
// packetLen: the size of the UDP packet; n: the bytes read into d (less when it did not fit).
void xiaoForward(const uint8_t* d, uint16_t n, uint16_t packetLen)
{
    if (!xiaoDetected || !xiaoOn()) return;
    if (n < 6 || d[0] != 0x80 || d[1] != 0x81) return;        // AgOpenGPS packets only
    if (packetLen > n) { xiao.dropBig++; return; }             // never send half a packet
    if (SerialXIAO.availableForWrite() < (int)n) { xiao.dropTx++; return; }
    SerialXIAO.write(d, n);
    xiao.toXiao++;
    xiao.txBytes += n;
}

// Settings changed on the web page
void xiaoSettingsChanged()
{
    xfLen = 0;
    xtextLen = 0;
    xtextBad = false;
    xFirstFrame = true;
}
