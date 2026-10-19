/* zXiao.ino - link to the XIAO ESP32 on the GPS2 socket (Serial2).
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Same link protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn"
   (AgOpenGPS-Official, GPL v3), at the same speed, 460800 baud.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zXiao.ino
// Seeed XIAO ESP32C3 / ESP32S3 on Teensy Serial2 (pin 7 = RX2, pin 8 = TX2), 460800 baud: on the AiO v4.5
// Micro in the GPS2 socket ("Dual-Left") through the adapter board or the 3D-printed holder; on the AiO v4.5
// Standard wired to its GPS2 header (P8: pin 1 RX2, pin 2 TX2) with 3.3 V and GND.
//
// Same protocol as the "ESP32 bridge" of the official Teensy firmware v26 (Firmware_Teensy_AiO-New-Dawn,
// docs/ESP32_BRIDGE_INTERFACE.md), at the same speed, 460800 (XIAO_BAUD, below). A XIAO program
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
//   These go out only when Serial2's transmit buffer has room to spare (XIAO_TX_RESERVE), so the
//   AgOpenGPS packets for the XIAO always come first.
//
// The XIAO is powered from the board's 3.3 V: never plug its USB-C (with 5 V) while it is on the board.
// The Teensy cannot program the XIAO (its BOOT / RESET pins are not wired): program it over USB-C
// off the board, or over WiFi if its own program has an update page.

#define SerialXIAO              Serial2
// Link speed. 460800 = 46 kB/s each way, as v26. The most the link carries today is AgOpenWeb's
// PGN 254 + 239 (+ 229 with more than 16 sections) at 100 Hz: 2.8-4.4 kB/s, under 10 % of it. The
// XIAO, whose single core also runs the WiFi, empties its 128-byte UART FIFO at 64 bytes
// (setRxFIFOFull in its program): 1.4 ms of room at this speed. The XIAO program must use the same
// value (LINK_BAUD in xLink.h).
#define XIAO_BAUD               460800
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
#define XIAO_INFO_MAX           250

static const char XIAO_HELLO[] = "ESP32-hello";

#include "zXiao.h"   // XiaoStats (included first by the main .ino)

uint8_t XIAOrxbuffer[1024];
uint8_t XIAOtxbuffer[2048];            // 2 kB (status JSON pieces + log lines + AgOpenGPS packets)

XiaoStats xiao = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0};

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

static inline bool xiaoOn() { return cfg.xiaoOff == 0; }

void xiaoSetup()
{
    SerialXIAO.begin(XIAO_BAUD);
    SerialXIAO.addMemoryForRead(XIAOrxbuffer, sizeof(XIAOrxbuffer));
    SerialXIAO.addMemoryForWrite(XIAOtxbuffer, sizeof(XIAOtxbuffer));
    Serial.println("XIAO ESP32 bridge on Serial2 (pins 7/8, 460800)");
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
            StatusLog.line("[xiao] XIAO said hello on Serial2, but the XIAO bridge is off in the settings: ignored");
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
    xiaoInfoMs = millis() | 1;
    xiao.infoRx++;
}

// one frame of the Teensy's own to the XIAO, only with room to spare in the transmit buffer
static bool xiaoSendOwn(uint8_t pgn, const uint8_t* data, uint8_t len)
{
    if (SerialXIAO.availableForWrite() < (int)len + 6 + XIAO_TX_RESERVE) return false;
    uint8_t h[5] = {0x80, 0x81, XIAO_SRC_TEENSY, pgn, len};
    uint8_t sum = XIAO_SRC_TEENSY + pgn + len;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    SerialXIAO.write(h, 5);
    SerialXIAO.write(data, len);
    SerialXIAO.write(&sum, 1);
    xiao.txBytes += len + 6;
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

// Drop the first n bytes of xf and anything up to the next 0x80 (a possible frame start)
static void xfShift(uint16_t n)
{
    while (n < xfLen && xf[n] != 0x80) n++;
    if (n >= xfLen) { xfLen = 0; return; }
    memmove(xf, xf + n, xfLen - n);
    xfLen -= n;
}

// Take the complete frames out of xf. Returns how many good frames were sent.
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

    xiaoTxTick(now);                           // status and log lines to the XIAO
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
