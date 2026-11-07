/* zWeb.ino - the module's web server.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   24 Oct 2026: the STEER LED is driven by zLeds.ino, no longer from the stop command here.
   26 Oct 2026: /api/status says whether the tractor is moving (steer.moving), so the page greys out the
   buttons this file would refuse. The GPS2 port setting is automatic or off (zDualF9P.ino); /api/status
   says what automatic found.
   2 Nov 2026: the server listens and accepts its connections itself (webListenStart, webAccept), no longer
   through NativeEthernet's EthernetServer, whose accept hands FNET an address length it never sets (see
   webListen below). With the mDNS responder (zEthernet.ino, the same day) that stack garbage became small
   and the page died seconds after boot ("the connection has timed out") while UDP went on.
   2 Nov 2026, later: /api/status says which previous firmware is kept in the flash (prev), and POST /api/revert
   puts it back (zOTA.ino): "Go back to the previous firmware" on the Firmware tab, no file or internet needed.
   3 Nov 2026: the page and the JSON answers go out in pieces from webLoop() (webTxStep), so a browser never
   holds the loop up: the page (~120 KB) used to stop the steering for 100 ms and more while the 8 KB socket
   buffer drained. The page carries an ETag (its build date): a browser that has it gets "304 Not Modified".
   /api/status: the program's maximum wheel angle and minimum steering speed (steerCfg), the distance to the
   guidance line (steer.line), for the Steering cards.
   3 Nov 2026: the diagnostic report says the loops per second and the time of each part of loop() (diag.loop.hz / steps).
   3 Nov 2026: steerLoop100 in /api/settings (the steering loop at 40 or 100 Hz); steer.loopMs / steer.loopHz (the loop
   time set and the steering loops per second measured) for the Steering card and the report.
   4 Nov 2026: cpuMhz / cpuTemp / cpuTempMax / cpuHot (the Teensy's clock and temperature, zSafety.ino) in /api/status and
   the report (diag.cpu); gnss.temp, dual.f9p2.temp and tm.temp, the temperatures the receivers and the TM171 report.
   4 Nov 2026: the JSON answers and the GET handlers run from flash (FLASHMEM). The code that runs from RAM1 is counted
   in 32 KB blocks: the temperatures above took it 1 KB over 256 KB and the stack lost 32 KB (46 KB -> 10 KB free in
   the report). The page's answers are not in a hurry; the steering, the GNSS and the IMU stay in RAM1.
   3 Nov 2026: tm.frozen / tm.frozenN in /api/status (roll and heading unchanged while driving, zImu.ino imuFreezeCheck).
   3 Nov 2026: GET / POST /api/note, the installation note (zSettings.ino); the request buffers grew for its text.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zWeb.ino
// The module's web page: http://<module IP>/  (normally http://192.168.5.126/) from the
// AgOpenWeb tablet, a phone or a PC on the same network. No need to open the box:
//   - Status: GNSS, dual heading, IMU (TM171 / BNO085), steering, network, what the receiver sends, the
//     sentences sent to AgOpenWeb / AgOpenGPS, the XIAO ESP32C3 bridge (zXiao.ino)
//   - Messages: the [status] lines (zStatus.ino)
//   - Settings (zSettings.ino), saved in the EEPROM
//   - GPS (was "UM982"): the receiver's model (zReceiver.ino); UM982: console, commands to the
//     receiver and its replies; u-blox F9P / X20P: its settings, read and written with UBX
//     the receiver check (zRxCheck.ino): POST /api/gps act=ckcheck / ckscan / ckbaud / ckload / ckdump,
//     GET /api/rxdump: the settings read by "Read all settings" as a u-center configuration file
//   - Machine: the machine module (sprayer Nano, PGN 160 on UDP 9999), only once it has been heard (zMachine.ino)
//   - Firmware update (zOTA.ino) and restart
//   - the report (GET /api/diag + the other answers, put together by the page): Firmware tab
//   the autosteer safety (zSafety.ino), the steering settings from the program, the radio
//   corrections, why the Teensy started; the WAS check, the watchdog and the simulator mode in the settings
//
// One request at a time, answered with "Connection: close". A request is read as its bytes arrive, and the
// page and the JSON answers go out in pieces, one per pass of loop(), as much as the socket buffer takes
// (webTxStep): the GNSS / steering loop never waits for the browser. The small answers are written at once
// (they fit the empty socket buffer); a browser that stops taking data is given up after WEB_TIMEOUT_MS.
// Only the firmware update takes over the loop, with the steering switched off.
//
// There is no password: anyone on the module's network can open the page.

#include "zWebPage.h"            // the page itself (HTML, CSS, JavaScript), kept in flash
#include "zWebJson.h"            // small JSON writer for the answers

#include "zVersion.h"            // FW_NAME, FW_VERSION (also included by the main .ino)

#define WEB_PORT         80
#define WEB_HEAD_MAX     3072    // request line + headers + small body (settings, UM982 command, the installation note)
#define WEB_BODY_MAX     1300    // the installation note: NOTE_MAX bytes (zSettings.h) and a margin
#define WEB_TIMEOUT_MS   3000
#define WEB_FIRST_BYTE_MS 500    // a connection with no request yet (browser "preconnect") is closed after this
#define OTA_MAX_KMH      1.0f    // no firmware update above this speed

// The listening socket, FNET's own, accepted by webAccept(). NativeEthernet's EthernetServer is not used:
// its poll() (NativeEthernetServer.cpp) calls fnet_socket_accept() with an address length it never sets
// ("fnet_size_t fromlen;"). FNET checks that length before anything else and refuses the accept when the
// stack garbage there is below 16 (fnet_socket.c, _fnet_socket_addr_check_len); the connection then stays
// in the listen queue, which holds one, and every later SYN is dropped: the page times out while UDP goes
// on, until a restart. What is on the stack there depends on what fnet_poll() ran just before; with the
// mDNS responder (2 Nov 2026) it was the small numbers of its name parsing, so the page died a few seconds
// after boot, with the first mDNS packet from the network.
#define WEB_BACKLOG      4       // connections waiting to be accepted (a browser opens several at once)
static fnet_socket_t webListen = FNET_NULL;
static bool webRunning = false;

static struct
{
    EthernetClient client;
    bool     active;
    uint32_t startMs;
    char     head[WEB_HEAD_MAX + 1];
    uint16_t len;
    int16_t  bodyAt;              // index of the body in head[] (-1: headers not complete yet)
    char     method[8];
    char     path[48];
    char     query[96];
    uint32_t contentLength;
    const char* tx;               // the rest of the answer's body still to send, in pieces from webLoop() (webTxStep)
    size_t   txLeft;              // bytes of it left (0: nothing pending)
    uint32_t txMs;                // when the last piece went out (a browser that takes nothing for WEB_TIMEOUT_MS is given up)
} rq;

static char webBuf[16384];       // JSON answers
#define WEB_TX_PIECE     4096    // at most this much of a body per pass of loop() (the socket buffer holds 8 KB)

// The page's ETag: its build date. The page is served with "Cache-Control: no-cache", so the browser keeps
// it but asks every time ("If-None-Match"); the same build answers "304 Not Modified" (about 100 bytes), a
// new build sends the page. So the ~120 KB page only travels after a firmware update.
static char webEtag[40];
static void webEtagSetup()
{
    const char* b = __DATE__ " " __TIME__;           // e.g. Nov  3 2026 14:20:11 -> "Nov-3-2026-14:20:11" (no spaces in an ETag)
    size_t k = 0;
    webEtag[k++] = '"';
    for (const char* p = b; *p && k < sizeof(webEtag) - 2; p++)
    {
        if (*p == ' ') { if (webEtag[k - 1] != '-') webEtag[k++] = '-'; }
        else webEtag[k++] = *p;
    }
    webEtag[k++] = '"';
    webEtag[k] = '\0';
}

// True when the request has a header "name" whose value contains "value" (the headers are in rq.head, as
// one string up to the empty line, while a request is being answered)
static bool webHeaderHas(const char* name, const char* value)
{
    const size_t nl = strlen(name), wn = strlen(value);
    for (const char* l = strstr(rq.head, "\r\n"); l; l = strstr(l + 2, "\r\n"))
    {
        if (strncasecmp(l + 2, name, nl) || l[2 + nl] != ':') continue;
        const char* v = l + 3 + nl;
        const char* e = strstr(v, "\r\n");
        const size_t vn = e ? (size_t)(e - v) : strlen(v);
        for (size_t i = 0; i + wn <= vn; i++) if (!memcmp(v + i, value, wn)) return true;
        return false;
    }
    return false;
}

// For the report (GET /api/diag, web page Firmware tab): requests answered, error answers (with the last
// one), and requests closed with no answer (the browser too slow or gone)
static uint32_t webReqN = 0, webErrN = 0, webDropN = 0;
static char     webErrLast[96] = "";

// ---------------------------------------------------------------- sending
// webWrite writes all of d now, waiting for the browser between pieces (at most WEB_TIMEOUT_MS): for the
// headers and the small answers, which fit the empty socket buffer and go out without a wait. A body that
// stays in memory (the page in flash, webBuf) is not written here: webTxStart keeps it and webLoop() writes
// a piece per pass (webTxStep), so the loop never waits for the browser.
static bool webWrite(EthernetClient& c, const char* d, size_t n)
{
    uint32_t t0 = millis();
    while (n > 0)
    {
        const int room = c.availableForWrite();
        if (room <= 0)
        {
            if (!c.connected() || millis() - t0 > WEB_TIMEOUT_MS) return false;
            wdtFeed();                                   // a slow browser must not trip the watchdog
            netPoll();                                   // the network stack (zNet.h): retransmissions
            delayMicroseconds(200);
            continue;
        }
        size_t k = (size_t)room < n ? (size_t)room : n;
        if (k > 1024) k = 1024;
        const size_t w = c.write((const uint8_t*)d, k);
        if (w == 0) return false;
        d += w;
        n -= w;
        t0 = millis();
    }
    return true;
}

// One piece of the pending body: as much as the socket buffer takes now, never waiting for it. The
// browser gone, or taking nothing for WEB_TIMEOUT_MS: the rest is dropped (counted in webDropN).
static void webTxStep()
{
    EthernetClient& c = rq.client;
    if (!c.connected()) { webDropN++; rq.txLeft = 0; return; }
    const int room = c.availableForWrite();
    if (room <= 0)
    {
        if (millis() - rq.txMs > WEB_TIMEOUT_MS) { webDropN++; rq.txLeft = 0; }
        return;
    }
    size_t k = (size_t)room < rq.txLeft ? (size_t)room : rq.txLeft;
    if (k > WEB_TX_PIECE) k = WEB_TX_PIECE;
    const size_t w = c.write((const uint8_t*)rq.tx, k);  // never more than the buffer takes: socketSend() would spin
    if (w == 0) { webDropN++; rq.txLeft = 0; return; }
    rq.tx     += w;
    rq.txLeft -= w;
    rq.txMs    = millis();
}

static void webTxStart(const char* body, size_t len)
{
    rq.tx     = body;
    rq.txLeft = len;
    rq.txMs   = millis();
    webTxStep();                                         // the first piece now; webLoop() sends the rest
}

static void webSend(EthernetClient& c, int code, const char* type, const char* body, size_t len)
{
    const char* text = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" :
                       code == 409 ? "Conflict" : code == 413 ? "Payload Too Large" : "Error";
    char h[200];
    const int hn = snprintf(h, sizeof(h),
                            "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                            "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                            code, text, type, (unsigned)len);
    if (!webWrite(c, h, hn)) return;
    if (body == webBuf) webTxStart(body, len);           // stays in memory until sent: in pieces from webLoop()
    else webWrite(c, body, len);                         // a short literal: fits the socket buffer, no wait
}

// GET /: the page, from flash. A browser that has this build's page sends its ETag back and gets "304 Not
// Modified"; otherwise the page goes out in pieces from webLoop().
static void webSendPage(EthernetClient& c)
{
    char h[260];
    if (webHeaderHas("If-None-Match", webEtag))
    {
        const int hn = snprintf(h, sizeof(h), "HTTP/1.1 304 Not Modified\r\nETag: %s\r\nCache-Control: no-cache\r\n"
                                              "Connection: close\r\n\r\n", webEtag);
        webWrite(c, h, hn);
        return;
    }
    const size_t len = strlen(WEB_PAGE);
    const int hn = snprintf(h, sizeof(h), "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %u\r\n"
                                          "ETag: %s\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n", (unsigned)len, webEtag);
    if (!webWrite(c, h, hn)) return;
    webTxStart(WEB_PAGE, len);
}

static void webSendJson(EthernetClient& c, int code, const char* json)
{
    webSend(c, code, "application/json", json, strlen(json));
}

static void webSendError(EthernetClient& c, int code, const char* msg)
{
    char j[300];
    char e[200];
    size_t k = 0;
    for (const char* p = msg; *p && k < sizeof(e) - 2; p++) { if (*p == '"' || *p == '\\') e[k++] = '\\'; e[k++] = *p; }
    e[k] = '\0';
    snprintf(j, sizeof(j), "{\"ok\":false,\"error\":\"%s\"}", e);
    webErrN++;
    snprintf(webErrLast, sizeof(webErrLast), "%d %s %s: %s", code, rq.method, rq.path, msg);
    webSendJson(c, code, j);
}

// ---------------------------------------------------------------- request helpers
// Value of "name" in a query string / urlencoded body, decoded. False if absent.
static bool formValue(const char* src, const char* name, char* out, size_t outSize)
{
    const size_t nl = strlen(name);
    const char* p = src;
    while (p && *p)
    {
        if (!strncmp(p, name, nl) && p[nl] == '=')
        {
            p += nl + 1;
            size_t k = 0;
            while (*p && *p != '&' && k < outSize - 1)
            {
                if (*p == '+') { out[k++] = ' '; p++; }
                else if (*p == '%' && hexNib(p[1]) >= 0 && hexNib(p[2]) >= 0) { out[k++] = (char)((hexNib(p[1]) << 4) | hexNib(p[2])); p += 3; }
                else out[k++] = *p++;
            }
            out[k] = '\0';
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

// Strict integer: the whole text must be a number
static bool parseIntStrict(const char* s, long& v)
{
    if (!*s) return false;
    char* end;
    v = strtol(s, &end, 10);
    return *end == '\0';
}

// Strict decimal number (the UM982 settings): the whole text must be a number
static bool parseNumStrict(const char* s, double& v)
{
    if (!*s) return false;
    char* end;
    v = strtod(s, &end);
    return *end == '\0' && !isnan(v) && !isinf(v);
}

static uint32_t queryAfter()
{
    char v[16];
    long n;
    return (formValue(rq.query, "after", v, sizeof(v)) && parseIntStrict(v, n) && n > 0) ? (uint32_t)n : 0;
}

// ---------------------------------------------------------------- steering off (firmware update)
// Same as Autosteer.ino when AgOpenWeb stops steering: valve / H-bridge off, PWM 0, Keya disabled.
static void steerSafeOff()
{
    watchdogTimer = WATCHDOG_FORCE_VALUE;
    if (steerConfig.CytronDriver) digitalWrite(PWM2_RPWM, steerConfig.IsRelayActiveHigh ? 1 : 0);
    else                          digitalWrite(DIR1_RL_ENABLE, 0);
    pwmDrive = 0;
    if (isKeya) disableKeyaSteer();
    motorDrive();
}

static bool autosteerEngaged()
{
    return watchdogTimer < WATCHDOG_THRESHOLD;
}
static bool tractorMoving();   // below, with the firmware update

// ---------------------------------------------------------------- answers
static const char* fixNameOf(int q)
{
    switch (q)
    {
        case 0: return "no fix";   case 1: return "GPS";      case 2: return "DGPS";
        case 4: return "RTK fixed"; case 5: return "RTK float"; default: return "other";
    }
}

// The module's state as one JSON object: the web page (/api/status) and, once a second, the XIAO
// (zXiao.ino, so its WiFi page can show it). full = also what only the web page shows (the
// steering settings, the start reason, the long texts), so the XIAO's copy stays well below its 3 kB.
FLASHMEM void statusJson(Json& j, uint32_t now, bool full)
{
    j.raw("{");
    j.str("fw", FW_NAME " " FW_VERSION);
    j.str("built", __DATE__ " " __TIME__);
    j.str("target", otaFlashId);
    j.str("marker", otaMarker);
    j.u("cpuMhz", F_CPU_ACTUAL / 1000000);                 // AiO_WEB 4 Nov 2026: the clock (the IDE's CPU Speed) and the
    j.num("cpuTemp", cpuTemp, 1);                          // chip's temperature, now and the highest since start-up (zSafety.ino)
    j.num("cpuTempMax", cpuTempMax, 1);
    j.bo("cpuHot", cpuHot);
    j.u("up", now / 1000);
    {
        char ip[20];
        IPAddress a = Ethernet.localIP();
        snprintf(ip, sizeof(ip), "%u.%u.%u.%u", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
        j.str("ip", ip);
    }
    {   // the previous firmware kept in the flash (zOTA.ino): "Go back to it" on this page and on the XIAO page
        const char *pf, *pb;
        uint32_t ps;
        if (otaPrevGet(&pf, &pb, &ps)) { j.obj("prev"); j.str("fw", pf); j.str("built", pb); j.u("size", ps); j.close(); }
        else                           { j.key("prev"); j.raw("null"); }
    }

    j.obj("net");
    j.bo("link", Ethernet.linkStatus() == LinkON);
    j.str("mdns", mdnsName);                               // AiO_WEB 7 Nov 2026: the name on the network (zEthernet.ino mdnsStart)
    j.bo("aog", aogLastMs != 0 && now - aogLastMs < STATUS_SILENCE_MS);
    {
        char ip[20];
        snprintf(ip, sizeof(ip), "%u.%u.%u.%u", aogIp[0], aogIp[1], aogIp[2], aogIp[3]);
        j.str("aogIp", ip);
    }
    // the program, from its hello (zStatus.ino)
    j.str("host", hostNow == HOST_AGOPENWEB ? "AgOpenWeb" : hostNow == HOST_AGOPENGPS ? "AgOpenGPS" : hostNow == HOST_OTHER ? "other" : "");
    j.bo("hostBoth", hostBoth());
    {
        uint32_t last = 0;
        for (uint8_t k = 1; k < 4; k++) if (hostLastMs[k] > last) last = hostLastMs[k];
        j.i("helloAgo", hostHellos ? (long)((now - last) / 1000) : -1);
    }
    j.bo("rtcm", ntripRtcm.arriving(now, STATUS_SILENCE_MS));
    j.num("rtcmKB", ntripRtcm.bytes / 1024.0, 1);
    // the RTCM messages, from the program (NTRIP) and from the radio (XBee)
    {
        char t[80];
        j.u("rtcmBps", ntripRtcm.arriving(now, STATUS_SILENCE_MS) ? ntripRtcm.bps : 0);
        j.u("rtcmOk", ntripRtcm.good);
        j.u("rtcmBad", ntripRtcm.bad);
        ntripRtcm.typesText(t, sizeof(t), now);
        j.str("rtcmTypes", t);
        j.bo("radio", radioRtcm.arriving(now, STATUS_SILENCE_MS));
        j.num("radioKB", radioRtcm.bytes / 1024.0, 1);
        j.u("radioBps", radioRtcm.arriving(now, STATUS_SILENCE_MS) ? radioRtcm.bps : 0);
        j.u("radioOk", radioRtcm.good);
        j.u("radioBad", radioRtcm.bad);
        radioRtcm.typesText(t, sizeof(t), now);
        j.str("radioTypes", t);
    }
    j.close();

    j.obj("gnss");
    const bool ggaOn = ggaMs != 0 && now - ggaMs < STATUS_SILENCE_MS;
    j.bo("gga", ggaOn);
    j.i("fix", ggaOn ? atoi(fixQuality) : -1);
    j.str("fixName", ggaOn ? fixNameOf(atoi(fixQuality)) : "no data");
    j.str("sats", numSats);
    j.str("hdop", HDOP);
    j.str("age", ageDGPS);
    j.num("lat", latitude[0] ? nmeaToDeg(latitude, latNS[0]) : NAN, 7);
    j.num("lon", longitude[0] ? nmeaToDeg(longitude, lonEW[0]) : NAN, 7);
    j.str("alt", altitude);
    j.num("kmh", lastSpeedKmh, 2);
    j.str("speedSrc", speedSource == 'v' ? "VTG" : speedSource == 'p' ? "positions" : "-");
    if (full) j.str("rx", rxTypesLast);                     // the long texts only for the web page
    j.str("model", rxModel);                                // the receiver, as it answered (zReceiver.ino)
    rxTempJson(j);                                          // AiO_WEB 4 Nov 2026: its temperature, "temp" / "tempSt" (zReceiver.ino)
    if (full) { j.str("rxKind", rxKindName()); j.str("rxFw", rxFw); }
    if (full) { j.str("ckH", rxCkHealthName(RXCK_GPS)); j.str("ck2H", rxCkHealthName(RXCK_GPS2)); }   // the data at 460800 (zRxCheck.ino)
    if (full) j.bo("single", rxSingleAntenna());          // one antenna: the page leaves the dual heading card out
    if (full) rxCkStatusJson(j, RXCK_GPS);                // the receiver check's summary (zRxCheck.ino): "ck"

    j.u("crcErr", nmeaCrcErrors);
    j.close();

    j.obj("dual");
    j.bo("hpr", hprMs != 0 && now - hprMs < RECEIVER_SEEN_MS);
    j.bo("ok", epochDual);
    j.i("qf", epochHprQf);
    j.i("lastQf", hprQf);
    j.str("heading", epochDual ? hprHeading : "");
    j.i("why", epochNoDual);
    j.num("antRoll", hprPitch, 2);
    if (full) j.str("raw", hprRawLast);
    j.u("late", hprLateCount);
    j.str("src", dualSrc == 'F' ? "f9p" : dualSrc == 'U' ? "um982" : "");   // AiO_WEB: UM982 HPR or second F9P (zDualF9P.ino)
    if (full) { dualF9pJson(j); rxCkStatusJson(j, RXCK_GPS2); }   // the second F9P: its data, and the receiver check's summary
    j.close();

    j.obj("tm");
    j.str("type", imuType == IMU_NONE ? "" : imuName());   // TM171 or BNO085 (zImu.ino)
    j.str("port", imuWhere());
    j.bo("ok", epochTmOk);
    j.bo("seen", TM171DataSeen);
    j.num("roll", RollV.fValue, 2);
    j.num("pitch", PitchV.fValue, 2);
    j.num("yaw", YawV.fValue, 2);
    j.num("out", lastOut, 2);
    j.i("sign", (long)yawSign);
    j.bo("locked", yawSignLocked);
    j.bo("aligned", offsetSeeded);
    j.num("offset", yawOffset, 2);
    j.u("crcErr", imuErrors());
    j.bo("frozen", imuFrozen);                      // AiO_WEB: roll and heading unchanged while driving (zImu.ino imuFreezeCheck)
    j.u("frozenN", imuFrozenCount);
    if (imuType == IMU_TM171 && tm171TempMs && now - tm171TempMs < 10000) j.num("temp", TemperatureV.fValue, 1);   // AiO_WEB 4 Nov 2026: from its Status Output (TM171.ino)
    // AiO_WEB: the TM171 gyro calibration (TM171.ino)
    j.str("calib", tmCalibName(tmCalibState));
    j.str("calibKind", tmCalibKind == TMC_KIND_INSTALL ? "install" : "gyro");
    j.i("calibLeft", tm171CalibLeft());
    j.i("calibAgo", tmCalibDoneMs ? (long)((now - tmCalibDoneMs) / 1000) : -1);
    j.u("calibN", tmCalibCount);
    if (imuType == IMU_TM171) tm171SettingJson(j);    // "set": the TM171's settings as read (TM171.ino)
    j.close();

    j.obj("steer");
    j.num("was", steerAngleActual, 2);
    j.num("set", steerAngleSetPoint, 2);
    j.i("pwm", pwmDisplay);
    j.bo("switchOn", steerSwitch == 0);
    j.bo("active", autosteerEngaged());
    j.bo("moving", tractorMoving());   // the page greys out what is refused then (firmware update, TM171 calibration)
    j.str("motor", isKeya ? "Keya" : steerConfig.IsDanfoss ? "Danfoss" : steerConfig.CytronDriver ? "Cytron" : "IBT2");
    j.num("aogKmh", gpsSpeed, 1);
    j.i("guidance", guidanceStatus);
    j.i("line", lineDist);             // distance to the guidance line, the lightbar byte (Autosteer.ino)
    j.u("loopMs", loopTimeMs);        // AiO_WEB: the steering loop set, 25 ms (40 Hz) or 10 ms (100 Hz), and
    j.u("loopHz", steerLoopHz);       //   the loops per second measured (zSafety.ino)
    // zSafety.ino: the WAS reading and its check, the kickout sensor, why it last went off
    j.bo("adc", Autosteer_running);
    j.i("wasRaw", wasRaw);
    j.num("wasV", wasVolts, 2);
    j.bo("wasOut", wasOut);
    j.bo("wasBlock", wasBlock);
    j.bo("wasZero", wasBenchZero());   // angle sent as 0 deg (bench)
    j.bo("wasCheck", cfg.wasCheckOff == 0);
    j.str("sensor", steerConfig.PressureSensor ? "pressure" : steerConfig.CurrentSensor ? "current" : "");
    j.num("sensorV", sensorView, 0);
    j.num("sensorMax", sensorPeakNow(), 0);
    j.i("kick", steerConfig.PulseCountMax);
    j.i("enc", steerConfig.ShaftEncoder ? (long)pulseCount : -1);
    j.str("off", offName(offCause));
    j.i("offAgo", offMs ? (long)((now - offMs) / 1000) : -1);
    j.i("offVal", offValue);
    j.bo("wasless", isKeya && !steerConfig.SingleInputWAS);   // AiO_WEB: wheel angle from the Keya position
    j.bo("keyaLost", keyaLost);        // AiO_WEB: no Keya heartbeat (KeyaCANBUS.ino)
    j.bo("keyaBlock", keyaBlock);
    if (full) keyaJson(j);
    j.close();

    if (full)
    {
        // the steering settings the program sent (PGN 252) and its steering configuration
        // (PGN 251), as the Teensy uses them (kept in its EEPROM between starts)
        j.obj("steerCfg");
        j.i("got252", pgn252Ms ? (long)((now - pgn252Ms) / 1000) : -1);
        j.i("got251", pgn251Ms ? (long)((now - pgn251Ms) / 1000) : -1);
        j.i("got247", pgn247Ms ? (long)((now - pgn247Ms) / 1000) : -1);
        j.num("wheelbase", wheelbaseM, 2);
        j.i("aowMaxDeg", aowMaxSteerDeg);     // the program's limits, shown only (main .ino)
        j.i("aowMinKmhX10", aowMinKmhX10);
        j.i("kp", steerSettings.Kp);
        j.i("hiPwm", steerSettings.highPWM);
        j.i("minPwm", steerSettings.minPWM);
        j.i("lowPwm", steerSettings.lowPWM);
        j.num("counts", steerSettings.steerSensorCounts, 1);
        j.i("offset", steerSettings.wasOffset);
        j.num("ackerman", steerSettings.AckermanFix * 100.0f, 0);
        j.bo("invertWas", steerConfig.InvertWAS);
        j.bo("singleWas", steerConfig.SingleInputWAS);
        j.bo("relayHigh", steerConfig.IsRelayActiveHigh);
        j.bo("motorDir", steerConfig.MotorDriveDirection);
        j.bo("cytron", steerConfig.CytronDriver);
        j.bo("danfoss", steerConfig.IsDanfoss);
        j.bo("sw", steerConfig.SteerSwitch);
        j.bo("btn", steerConfig.SteerButton);
        j.bo("encoder", steerConfig.ShaftEncoder);
        j.bo("pressure", steerConfig.PressureSensor);
        j.bo("current", steerConfig.CurrentSensor);
        j.i("kickMax", steerConfig.PulseCountMax);
        j.bo("yAxis", steerConfig.IsUseY_Axis);
        j.close();

        // why the Teensy started, and its watchdog
        j.obj("boot");
        j.str("why", bootName(bootWhy));
        {
            char t[16];
            snprintf(t, sizeof(t), "0x%02lX", (unsigned long)bootSrsr);
            j.str("srsr", t);
            snprintf(t, sizeof(t), "0x%08lX", (unsigned long)(bootStep ? bootStuckPc : bootFaultPc));
            j.str("addr", (bootWhy == BOOT_CRASH || bootWhy == BOOT_CRASH_WDT || bootStep) ? t : "");
        }
        j.str("step", bootStep ? stepName(bootStep) : "");
        j.bo("wdt", wdtRunning);
        j.bo("wdtOff", cfg.wdtOff != 0);
        j.u("hwMsgs", hwMsgCount);
        j.str("hwMsg", hwMsgLast);
        j.close();
    }

    j.obj("out");
    j.str("mode", outClassicNow() ? "classic" : "standard");     // what is sent now
    j.bo("auto", cfg.outputMode == OUT_AUTO);
    j.i("kind", lastSentKind);
    // AgOpenGPS heading source to use with the classic output: "Dual" for a UM982, "Fix" for F9P / X20P
    j.str("aogSrc", !outClassicNow() ? "" : receiverIsDual() ? "Dual" : (hprPresent ? "" : "Fix"));
    j.i("classicSrc", outClassicNow() && receiverIsDual() ? classicSrc : 0);
    j.u("epochs", epochsSent);
    j.bo("sim", cfg.simMode != 0);           // simulator mode (zHandlers.ino): the GPS is not sent to the program
    j.bo("simHeld", cfg.simMode && simHeld);  // ...and epochs are being held back
    j.u("held", epochsHeld);
    if (full)
    {
        j.arr("lines");
        for (int k = 0; k < SENT_SLOTS; k++) if (lastSent[k][0]) j.item(lastSent[k]);
        j.closeArr();
    }
    j.close();

    j.obj("xiao");
    j.bo("on", gps2XiaoOn());
    j.i("port", cfg.xiaoOff == GPS2_OFF ? 1 : dualF9pOn() ? 2 : 0);   // AiO_WEB: GPS2 port 0 automatic (the XIAO's once it says hello), 1 off, 2 the second F9P's
    j.bo("det", xiaoDetected);
    j.i("helloAgo", xiao.hellos ? (long)((now - xiaoLastHelloMs) / 1000) : -1);
    j.u("hellos", xiao.hellos);
    j.u("to", xiao.toXiao);
    j.u("from", xiao.fromXiao);
    j.i("lastPgn", xiao.lastPgn);
    j.u("crcErr", xiao.crcErr);
    j.u("cut", xiao.cut);
    j.u("drop", xiao.dropTx + xiao.dropBig + xiao.beforeHello);
    j.num("rxKB", xiao.rxBytes / 1024.0, 1);
    j.num("txKB", xiao.txBytes / 1024.0, 1);
    j.u("text", xiao.textLines);
    if (full) j.str("info", xiaoInfo);                        // the XIAO's own JSON (program, WiFi), as text
    j.i("infoAgo", xiaoInfoMs ? (long)((now - xiaoInfoMs) / 1000) : -1);
    j.close();

    if (full) machineJson(j, now);                            // the machine module (Machine tab, zMachine.ino)

    j.raw("}");
}

static void apiStatus(EthernetClient& c)
{
    Json j(webBuf, sizeof(webBuf));
    statusJson(j, millis(), true);
    webSendJson(c, 200, webBuf);
}

FLASHMEM static void apiLog(EthernetClient& c)
{
    const uint32_t after = queryAfter();
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.u("next", StatusLog.nextSeq);
    j.u("now", millis());
    j.arr("lines");
    const uint32_t first = StatusLog.nextSeq > LOG_LINES ? StatusLog.nextSeq - LOG_LINES : 1;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s < StatusLog.nextSeq; s++)
    {
        const LogLine& l = StatusLog.lines[s % LOG_LINES];
        if (l.seq != s) continue;
        if (j.n + LOG_LINE_LEN + 64 > j.cap) break;          // the page asks again for the rest
        j.open(); j.u("s", l.seq); j.u("t", l.ms); j.str("x", l.text); j.close();
    }
    j.closeArr();
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

FLASHMEM static void settingsJson(Json& j)
{
    j.obj("settings");
    j.i("outputMode", cfg.outputMode);
    j.i("classicNoDual", cfg.classicNoDual);
    j.i("hprAcceptFloat", cfg.hprAcceptFloat);
    j.i("epochWaitMs", cfg.epochWaitMs);
    j.i("tm171SwapRollPitch", cfg.tm171SwapRollPitch);
    j.i("tm171YawSign", cfg.tm171YawSign);
    j.i("xiaoOff", cfg.xiaoOff);
    j.i("wasCheckOff", cfg.wasCheckOff);
    j.i("wdtOff", cfg.wdtOff);
    j.i("umSingle", cfg.umSingle);
    j.i("simMode", cfg.simMode);
    j.i("steerMinKmhX10", cfg.steerMinKmhX10);
    j.i("steerMaxKmh", cfg.steerMaxKmh);
    j.i("softStartX10ms", cfg.softStartX10ms);
    j.i("currentKickX10ms", cfg.currentKickX10ms);
    j.i("steerLoop100", cfg.steerLoop100);
    j.close();
    j.bo("saved", cfgFromEeprom);
    j.bo("wdt", wdtRunning);
}

FLASHMEM static void apiSettingsGet(EthernetClient& c)
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    settingsJson(j);
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

static void apiSettingsPost(EthernetClient& c, const char* body)
{
    WebSettings s = cfg;
    char v[24];
    if (formValue(body, "defaults", v, sizeof(v)) && !strcmp(v, "1")) settingsDefaults(s);
    else
    {
        struct Field { const char* name; long lo, hi; void* dst; bool isSigned; } fields[] = {
            {"outputMode",         0, 2,   &s.outputMode,         false},
            {"classicNoDual",      0, 1,   &s.classicNoDual,      false},
            {"hprAcceptFloat",     0, 1,   &s.hprAcceptFloat,     false},
            {"epochWaitMs",        20, 100, &s.epochWaitMs,       false},
            {"tm171SwapRollPitch", 0, 1,   &s.tm171SwapRollPitch, false},
            {"tm171YawSign",       -1, 1,  &s.tm171YawSign,       true},
            {"xiaoOff",            0, 1,   &s.xiaoOff,            false},   // GPS2 port: automatic / off
            {"wasCheckOff",        0, 1,   &s.wasCheckOff,        false},
            {"wdtOff",             0, 1,   &s.wdtOff,             false},
            {"umSingle",           0, 1,   &s.umSingle,           false},   // UM982 antennas (GPS tab, receiver check)
            {"simMode",            0, 1,   &s.simMode,            false},   // simulator mode (zHandlers.ino)
            {"steerMinKmhX10",     1, 30,  &s.steerMinKmhX10,     false},   // steering (Autosteer.ino, AutosteerPID.ino)
            {"steerMaxKmh",        5, 40,  &s.steerMaxKmh,        false},
            {"softStartX10ms",     0, 100, &s.softStartX10ms,     false},
            {"currentKickX10ms",   0, 200, &s.currentKickX10ms,   false},
            {"steerLoop100",       0, 1,   &s.steerLoop100,       false},   // steering loop 40 / 100 Hz (steerLoopApply)
        };
        for (const Field& f : fields)
        {
            if (!formValue(body, f.name, v, sizeof(v))) continue;
            long n;
            if (!parseIntStrict(v, n) || n < f.lo || n > f.hi)
            {
                char e[80];
                snprintf(e, sizeof(e), "invalid value for %s", f.name);
                webSendError(c, 400, e);
                return;
            }
            if (f.isSigned) *(int8_t*)f.dst = (int8_t)n;
            else            *(uint8_t*)f.dst = (uint8_t)n;
        }
    }
    const bool yawChanged  = s.tm171YawSign != cfg.tm171YawSign;
    const bool xiaoChanged = s.xiaoOff != cfg.xiaoOff;
    const bool wasChanged  = s.wasCheckOff != cfg.wasCheckOff;
    const bool wdtChanged  = s.wdtOff != cfg.wdtOff;
    const bool umChanged   = s.umSingle != cfg.umSingle;
    const bool simChanged  = s.simMode != cfg.simMode;
    const bool loopChanged = s.steerLoop100 != cfg.steerLoop100;
    const bool steerChanged = s.steerMinKmhX10 != cfg.steerMinKmhX10 || s.steerMaxKmh != cfg.steerMaxKmh ||
                              s.softStartX10ms != cfg.softStartX10ms || s.currentKickX10ms != cfg.currentKickX10ms || loopChanged;
    cfg = s;
    settingsSave();
    settingsApply(yawChanged);
    if (loopChanged) steerLoopApply();   // Autosteer.ino: the loop time, the ADS1115 rate, the kickout filters
    if (xiaoChanged) { gps2Begin(); xiaoSettingsChanged(); }   // AiO_WEB: Serial2 at the new speed (zDualF9P.ino)
    // zSafety.ino
    if (wasChanged)
        StatusLog.line(cfg.wasCheckOff ? "[web] WAS check OFF (bench setting): a broken WAS wire will NOT stop autosteer"
                                       : "[web] WAS check on");
    if (wdtChanged)
    {
        if (!cfg.wdtOff) wdtStart();
        else if (wdtRunning) StatusLog.line("[web] Watchdog switched off: it stops at the next start of the Teensy");
    }
    StatusLog.line("[web] settings saved: output %s%s, float heading %s, epoch wait %u ms, IMU roll/pitch swap %s, yaw %s, GPS2 port %s, WAS check %s, watchdog %s",
                   cfg.outputMode == OUT_AUTO ? "automatic" : cfg.outputMode == OUT_CLASSIC ? "classic" : "standard",
                   cfg.outputMode == OUT_AUTO ? (outClassicNow() ? " (now classic)" : " (now standard, AgOpenWeb)") :
                   cfg.outputMode == OUT_CLASSIC ? " ($PAOGI with UM982, $PANDA with F9P / X20P)" : "",
                   cfg.hprAcceptFloat ? "yes" : "no", cfg.epochWaitMs, cfg.tm171SwapRollPitch ? "yes" : "no",
                   cfg.tm171YawSign == 0 ? "auto" : cfg.tm171YawSign > 0 ? "+1" : "-1", gps2Name(),
                   cfg.wasCheckOff ? "OFF" : "on", cfg.wdtOff ? "off" : "on");
    if (umChanged)
        StatusLog.line(cfg.umSingle ? "[web] UM982 with one antenna: no dual-antenna heading on the page, AgOpenGPS settings ConfigUM982S.txt"
                                    : "[web] UM982 with two antennas: AgOpenGPS settings ConfigUM982D.txt");
    if (simChanged)
        StatusLog.line(cfg.simMode ? "[web] Simulator mode ON (for the program's simulator only): the GPS is not sent to the program"
                                   : "[web] Simulator mode off: the GPS goes to the program again");
    if (steerChanged)
        StatusLog.line("[web] Steering: minimum %u.%u km/h, maximum %u km/h, soft start %u ms (0 = off), kickout delay %u ms, loop %u Hz",
                       cfg.steerMinKmhX10 / 10, cfg.steerMinKmhX10 % 10, cfg.steerMaxKmh,
                       (unsigned)cfg.softStartX10ms * 10, (unsigned)cfg.currentKickX10ms * 10, cfg.steerLoop100 ? 100u : 40u);
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    settingsJson(j);
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

static void apiUm982Post(EthernetClient& c, const char* body)
{
    char cmd[128];
    if (!formValue(body, "cmd", cmd, sizeof(cmd))) { webSendError(c, 400, "no command"); return; }
    // trim, printable ASCII only
    char* p = cmd;
    while (*p == ' ') p++;
    size_t n = strlen(p);
    while (n && p[n - 1] == ' ') p[--n] = '\0';
    if (n == 0 || n > 120) { webSendError(c, 400, "empty or too long command"); return; }
    if (umQuiet()) { webSendError(c, 409, "busy: the page is reading or writing the UM982 settings"); return; }
    if (rxCkBusy(RXCK_GPS)) { webSendError(c, 409, "busy: the receiver check is running"); return; }
    for (size_t k = 0; k < n; k++) if ((uint8_t)p[k] < 0x20 || (uint8_t)p[k] > 0x7E) { webSendError(c, 400, "invalid character"); return; }
    rxReplyStartCapture();
    SerialGPS->print(p);
    SerialGPS->print("\r\n");
    StatusLog.line("[web] UM982 <- %s", p);
    webSendJson(c, 200, "{\"ok\":true}");
}

FLASHMEM static void apiUm982Get(EthernetClient& c)
{
    const uint32_t after = queryAfter();
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.u("next", rxReplyNextSeq);
    j.arr("lines");
    const uint32_t first = rxReplyNextSeq > RX_REPLY_LINES ? rxReplyNextSeq - RX_REPLY_LINES : 1;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s < rxReplyNextSeq; s++)
    {
        const RxReply& r = rxReplies[s % RX_REPLY_LINES];
        if (r.seq == s) j.item(r.text);
    }
    j.closeArr();
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

// the receiver (zReceiver.ino): what it is, and the u-blox and UM982 settings
static void apiGpsGet(EthernetClient& c)
{
    Json j(webBuf, sizeof(webBuf));
    receiverJson(j);
    webSendJson(c, 200, webBuf);
}

// GET /api/rxdump: the settings "Read all settings" found different from the factory defaults, as a u-center
// configuration file (the form of the AOGConfigOMatic files; zRxCheck.ino). Line by line: it can be longer than webBuf.
FLASHMEM static void apiRxDump(EthernetClient& c)
{
    if (!rxCkDumpReady()) { webSend(c, 404, "text/plain", "No settings read yet", 20); return; }
    size_t total = 0;
    for (uint16_t i = 0;; i++) { const size_t n = rxCkDumpLine(i, webBuf, sizeof(webBuf)); if (!n) break; total += n; }
    char h[160];
    const int hn = snprintf(h, sizeof(h), "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\nContent-Length: %u\r\n"
                            "Cache-Control: no-store\r\nConnection: close\r\n\r\n", (unsigned)total);
    if (!webWrite(c, h, hn)) return;
    for (uint16_t i = 0;; i++) { const size_t n = rxCkDumpLine(i, webBuf, sizeof(webBuf)); if (!n || !webWrite(c, webBuf, n)) break; }
}

static void apiGpsPost(EthernetClient& c, const char* body)
{
    char act[16], v[16];
    if (!formValue(body, "act", act, sizeof(act))) { webSendError(c, 400, "no action"); return; }
    const char* err = nullptr;
    // the receiver check (zRxCheck.ino): ckcheck / ckscan / ckbaud / ckload, port=0 (GPS) or 1 (second F9P),
    // um=d / s (UM982: two antennas / one)
    if (!strncmp(act, "ck", 2))
    {
        const uint8_t port = formValue(body, "port", v, sizeof(v)) && !strcmp(v, "1") ? RXCK_GPS2 : RXCK_GPS;
        const uint8_t um = formValue(body, "um", v, sizeof(v)) && !strcmp(v, "s") ? 1 : 0;
        // the search, the speed change and the load stop the GPS data for a while
        if (strcmp(act, "ckcheck") && strcmp(act, "ckdump") && autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        err = rxCkStart(act + 2, port, um);
        if (err) { webSendError(c, !strcmp(err, "busy") ? 409 : 400, err); return; }
        webSendJson(c, 200, "{\"ok\":true}");
        return;
    }
    if (rxCkBusy(RXCK_GPS)) { webSendError(c, 409, "busy"); return; }     // the receiver check has the receiver
    if (!strcmp(act, "identify"))
    {
        receiverIdentifyAgain();
        StatusLog.println("[web] GPS receiver: asked again what it is");
    }
    else if (!strcmp(act, "read")) err = ubxStartRead();
    else if (!strcmp(act, "ubxon")) err = ubxOutputOn();
    else if (!strcmp(act, "umread")) err = umStartRead();
    else if (!strcmp(act, "umwrite"))
    {
        if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        umWriteBegin();
        for (uint8_t i = 0; i < umItemCount() && !err; i++)
        {
            if (!formValue(body, umItemName(i), v, sizeof(v))) continue;
            double x;
            if (!parseNumStrict(v, x)) err = "invalid value";
            else err = umWriteAdd(i, x);
        }
        if (!err) err = umWriteGo();
    }
    else if (!strcmp(act, "write"))
    {
        // changing the receiver while steering could stop its output for a moment
        if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        ubxWriteBegin();
        for (uint8_t i = 0; i < ubxItemCount() && !err; i++)
        {
            if (!formValue(body, ubxItemName(i), v, sizeof(v))) continue;
            long n;
            if (!parseIntStrict(v, n)) err = "invalid value";
            else err = ubxWriteAdd(i, n);
        }
        if (!err) err = ubxWriteGo();
    }
    else err = "unknown action";
    if (err) { webSendError(c, !strcmp(err, "busy") ? 409 : 400, err); return; }
    webSendJson(c, 200, "{\"ok\":true}");
}

static void restartTeensy()
{
    netDelay(300);                                         // let the answer leave
    SCB_AIRCR = 0x05FA0004;
    for (;;) {}
}

// A refused update is answered only once the browser has sent the whole file (read and dropped here):
// an answer while it is still sending cuts the connection, and the page could only say "the
// connection failed" instead of why.
static void otaDrain(EthernetClient& c, uint32_t got)
{
    static uint8_t buf[512];
    uint32_t last = millis();
    while (got < rq.contentLength && c.connected() && millis() - last < 5000)
    {
        wdtFeed();
        netPoll();
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0) { got += r; last = millis(); }
        }
        else netDelay(1);
    }
}

static void otaRefuse(EthernetClient& c, int code, const char* why, uint32_t got)
{
    StatusLog.line("[web] firmware update REFUSED: %s (old firmware kept)", why);
    if (rq.contentLength <= 8000000) otaDrain(c, got);
    webSendError(c, code, why);
}

// The firmware update: takes over the loop until the whole file is in (or fails).
// pre/preLen = the first body bytes, already read with the headers.
// Is the tractor moving? (the firmware update and the TM171 calibration refuse to start then)
// Moving: the program's speed, or the receiver's with RTK. Without RTK a receiver standing still
// gives a few km/h of noise (a UM982 with a GPS fix on the bench: 2.7 km/h).
// Not checked on a bench: the simulator mode on, or a wheel angle that a tractor does not give (the
// WAS voltage out of range - no WAS - or an angle beyond the WAS map, +-50 deg). A Keya used as the
// WAS has no WAS voltage: only its angle counts. The program's simulator gives a speed there.
static bool tractorMoving()
{
    const bool keyaWas = isKeya && !steerConfig.SingleInputWAS;
    const bool angleOdd = (!keyaWas && wasOut) ||
                          ((keyaWas || wasVoltsOk()) && (steerAngleActual < inputWAS[0] || steerAngleActual > inputWAS[WAS_MAP_N - 1]));
    const bool bench = cfg.simMode || angleOdd;
    const bool rtk = fixQuality[0] == '4' || fixQuality[0] == '5';
    return !bench && ((rtk && lastSpeedKmh > OTA_MAX_KMH) || gpsSpeed > OTA_MAX_KMH);
}

// POST /api/imu act=calib (TM171 gyro calibration, RAM) / act=install (new install calibration, flash) /
// act=calibstop (TM171.ino)
static void apiImuPost(EthernetClient& c, const char* body)
{
    char act[16];
    if (!formValue(body, "act", act, sizeof(act))) { webSendError(c, 400, "no act"); return; }
    if (!strcmp(act, "calib") || !strcmp(act, "install"))
    {
        if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        if (tractorMoving())    { webSendError(c, 409, "the tractor is moving - stop first (on a bench, switch simulator mode on)"); return; }
        const char* why = "";
        if (!tm171CalibStart(act[0] == 'i' ? TMC_KIND_INSTALL : TMC_KIND_GYRO, &why)) { webSendError(c, 409, why); return; }
    }
    else if (!strcmp(act, "calibstop")) tm171CalibStop("web page");
    else if (!strcmp(act, "setread"))
    {
        if (imuType != IMU_TM171) { webSendError(c, 409, "no TM171 found"); return; }
        tm171SettingRead();
    }
    else if (!strcmp(act, "setwrite"))
    {
        if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        char v[12];
        TmSetForm f;
        auto flag = [&](const char* n) { return formValue(body, n, v, sizeof(v)) && !strcmp(v, "1"); };
        auto num  = [&](const char* n, long dflt) { return formValue(body, n, v, sizeof(v)) ? atol(v) : dflt; };
        f.mag = flag("mag"); f.gyroErr = flag("gyroErr"); f.selfAdapt = flag("selfAdapt"); f.status = flag("status"); f.save = flag("save");
        const long boot = num("boot", 0), acc = num("accGain", 210), mg = num("magGain", 100), odr = num("odr", 25);
        if (boot < 0 || boot > 2 || acc < 0 || acc > 65535 || mg < 0 || mg > 65535 || odr < 0 || odr > 65535)
        { webSendError(c, 400, "value out of range"); return; }
        f.boot = (uint8_t)boot; f.accGain = (uint16_t)acc; f.magGain = (uint16_t)mg; f.odr = (uint16_t)odr;
        f.baud100 = 0;                                   // the baud rate is not changed from the form
        const char* why = "";
        if (!tm171SettingWrite(f, &why)) { webSendError(c, 409, why); return; }
    }
    else if (!strcmp(act, "fixsetup"))               // the AgOpenGPS set-up + 115200, saved to flash
    {
        if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
        const char* why = "";
        if (!tm171FixSetup(&why)) { webSendError(c, 409, why); return; }
    }
    else { webSendError(c, 400, "unknown act"); return; }
    webSendJson(c, 200, "{\"ok\":true}");
}

static void apiOta(EthernetClient& c, const uint8_t* pre, size_t preLen)
{
    char v[8];
    const bool force = formValue(rq.query, "force", v, sizeof(v)) && !strcmp(v, "1");
    if (autosteerEngaged())                                    { otaRefuse(c, 409, "autosteer is engaged - disengage it first", preLen); return; }
    if (tractorMoving())
    {
        otaRefuse(c, 409, "the tractor is moving - stop first (on a bench, switch simulator mode on)", preLen);
        return;
    }
    if (rq.contentLength < 1000 || rq.contentLength > 8000000) { otaRefuse(c, 400, "no file, or file size not plausible", preLen); return; }

    steerSafeOff();
    StatusLog.line("[web] firmware update: receiving %lu bytes, steering off", (unsigned long)rq.contentLength);
    const char* err = otaBegin();
    if (!err) err = otaFeed(pre, preLen);
    uint32_t got = preLen, last = millis();
    static uint8_t buf[2048];
    loopStep = 20;                                         // for the watchdog report (zSafety.ino)
    while (!err && got < rq.contentLength)
    {
        wdtFeed();
        netPoll();                                         // the network stack (zNet.h): ACKs, window
        const int n = c.available();
        if (n > 0)
        {
            const int r = c.read(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
            if (r > 0)
            {
                err = otaFeed(buf, r);
                got += r;
                last = millis();
            }
        }
        else if (!c.connected())              err = "connection lost during the upload";
        else if (millis() - last > 15000)     err = "upload stopped (timeout)";
        else                                  netDelay(1);
    }
    uint32_t size = 0;
    if (!err) err = otaFinish(force, &size);
    if (err) { otaRefuse(c, 400, err, got); return; }
    StatusLog.line("[web] firmware update: %lu bytes checked, installing and restarting", (unsigned long)size);
    char j[160];
    snprintf(j, sizeof(j), "{\"ok\":true,\"size\":%lu}", (unsigned long)size);
    webSendJson(c, 200, j);
    netDelay(400);                                         // let the answer leave
    c.stop();
    netDelay(100);
    otaInstall(size);                                      // does not return
}

// POST /api/revert: the previous firmware kept in the flash (zOTA.ino) back over the running one. The same
// conditions as an update, and no file: the Teensy has it already.
static void apiRevert(EthernetClient& c)
{
    if (autosteerEngaged()) { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
    if (tractorMoving())    { webSendError(c, 409, "the tractor is moving - stop first (on a bench, switch simulator mode on)"); return; }
    const char* err = otaRevertCheck();
    if (err) { StatusLog.line("[web] going back to the previous firmware REFUSED: %s", err); webSendError(c, 409, err); return; }
    const char *pf, *pb;
    uint32_t ps = 0;
    otaPrevGet(&pf, &pb, &ps);
    steerSafeOff();
    StatusLog.line("[web] going back to the previous firmware: %s, built %s (%lu bytes), installing and restarting, steering off",
                   pf, pb, (unsigned long)ps);
    webSendJson(c, 200, "{\"ok\":true}");
    netDelay(400);                                         // let the answer leave
    c.stop();
    netDelay(100);
    otaRevertInstall();                                    // does not return
}

// ---------------------------------------------------------------- request handling
static void webFinish()
{
    rq.client.stop();
    rq.active = false;
    rq.txLeft = 0;
}

// The request is answered: close it, unless its body is still going out in pieces (webLoop() closes it then)
static void webDone()
{
    if (!rq.txLeft) webFinish();
}

// GET /api/diag: the firmware's own state, for the report (web page, Firmware tab)
FLASHMEM static void apiDiag(EthernetClient& c)
{
    const uint32_t now = millis();
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.str("fw", FW_NAME " " FW_VERSION);
    j.str("built", __DATE__ " " __TIME__);
#ifdef TEENSYDUINO
    j.i("teensyduino", TEENSYDUINO);
#endif
    j.u("upMs", now);
    j.obj("ram");
#if defined(__IMXRT1062__)
    extern unsigned long _heap_end, _ebss;
    extern char* __brkval;
    char here;
    j.i("heapFree", (long)((char*)&_heap_end - __brkval));     // RAM2 (malloc / DMAMEM)
    j.i("stackFree", (long)(&here - (char*)&_ebss));           // RAM1, between the variables and the stack
#endif
    j.close();
    j.obj("cpu");                                              // AiO_WEB 4 Nov 2026: the clock and the chip's temperature (zSafety.ino)
    j.u("mhz", F_CPU_ACTUAL / 1000000);
    j.num("temp", cpuTemp, 1);
    j.num("tempMax", cpuTempMax, 1);
    j.close();
    j.obj("loop");
    j.u("n", loopCount);
    j.u("maxUs", loopMaxUs);
    j.u("max5sUs", loopMax5Us);
    j.u("hz", loopHz);                                         // loops per second, last whole 5 s window
    j.arr("steps");                                            // where the time goes (zSafety.ino loopAt)
    for (uint8_t k = 0; k < LOOP_STEPS; k++)
    {
        if (loopStepAvgUs[k] == 0 && loopStepMaxUs[k] == 0) continue;
        j.open(); j.u("s", k); j.str("name", k ? stepName(k) : "other"); j.u("avgUs", loopStepAvgUs[k]); j.u("maxUs", loopStepMaxUs[k]); j.close();
    }
    j.closeArr();
    j.close();
    j.obj("web");
    j.u("req", webReqN);
    j.u("err", webErrN);
    j.u("drop", webDropN);
    j.str("lastErr", webErrLast);
    j.close();
    j.u("logLines", StatusLog.nextSeq - 1);
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

// GET /api/note: the installation note (zSettings.ino, EEPROM). POST /api/note: the body is the new text itself (plain
// text, not a form), up to NOTE_MAX bytes; control characters other than tabs and line breaks are refused.
FLASHMEM static void apiNoteGet(EthernetClient& c)
{
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.bo("ok", true);
    j.str("note", cfgNote);
    j.u("max", NOTE_MAX);
    j.raw("}");
    webSendJson(c, 200, webBuf);
}

static void apiNotePost(EthernetClient& c, const char* body)
{
    const size_t n = rq.contentLength;
    if (n > NOTE_MAX) { webSendError(c, 413, "the note is too long (the limit counts bytes: an accented letter is 2)"); return; }
    if (!noteSave(body, n)) { webSendError(c, 400, "the note has characters that cannot be kept"); return; }
    StatusLog.line("[web] installation note saved (%u bytes)", (unsigned)n);
    apiNoteGet(c);
}

static void webDispatch()
{
    webReqN++;
    EthernetClient& c = rq.client;
    const bool get = !strcmp(rq.method, "GET"), post = !strcmp(rq.method, "POST");
    const char* body = rq.bodyAt >= 0 ? rq.head + rq.bodyAt : "";
    if      (get  && (!strcmp(rq.path, "/") || !strcmp(rq.path, "/index.html")))
        webSendPage(c);
    else if (get  && !strcmp(rq.path, "/api/status"))   apiStatus(c);
    else if (get  && !strcmp(rq.path, "/api/log"))      apiLog(c);
    else if (get  && !strcmp(rq.path, "/api/settings")) apiSettingsGet(c);
    else if (get  && !strcmp(rq.path, "/api/diag"))     apiDiag(c);
    else if (post && !strcmp(rq.path, "/api/settings")) apiSettingsPost(c, body);
    else if (get  && !strcmp(rq.path, "/api/um982"))    apiUm982Get(c);
    else if (post && !strcmp(rq.path, "/api/um982"))    apiUm982Post(c, body);
    else if (get  && !strcmp(rq.path, "/api/gps"))      apiGpsGet(c);
    else if (post && !strcmp(rq.path, "/api/gps"))      apiGpsPost(c, body);
    else if (get  && !strcmp(rq.path, "/api/rxdump"))   apiRxDump(c);
    else if (post && !strcmp(rq.path, "/api/imu"))      apiImuPost(c, body);     // AiO_WEB: TM171 calibration
    else if (get  && !strcmp(rq.path, "/api/note"))     apiNoteGet(c);           // the installation note (zSettings.ino)
    else if (post && !strcmp(rq.path, "/api/note"))     apiNotePost(c, body);
    else if (post && !strcmp(rq.path, "/api/revert"))   apiRevert(c);            // the previous firmware back (zOTA.ino)
    else if (post && !strcmp(rq.path, "/api/reboot"))
    {
        StatusLog.line("[web] restart requested");
        webSendJson(c, 200, "{\"ok\":true}");
        c.stop();
        restartTeensy();
    }
    else webSend(c, 404, "text/plain", "Not found", 9);
    webDone();
}

// Request line and headers are complete in rq.head[0 .. headEnd)
static bool webParseHead(uint16_t headEnd)
{
    rq.head[headEnd] = '\0';
    char target[160] = {};
    if (sscanf(rq.head, "%7s %159s", rq.method, target) != 2) return false;
    char* q = strchr(target, '?');
    // memcpy with the length, the same copy without a compiler warning
    if (q) { *q = '\0'; const size_t k = strnlen(q + 1, sizeof(rq.query) - 1); memcpy(rq.query, q + 1, k); rq.query[k] = '\0'; }
    { const size_t k = strnlen(target, sizeof(rq.path) - 1); memcpy(rq.path, target, k); rq.path[k] = '\0'; }
    rq.contentLength = 0;
    for (char* l = strstr(rq.head, "\r\n"); l; l = strstr(l + 2, "\r\n"))
        if (!strncasecmp(l + 2, "Content-Length:", 15)) rq.contentLength = strtoul(l + 17, nullptr, 10);
    return true;
}

// ---------------------------------------------------------------- listening and accepting (see webListen)
static bool webListenStart()
{
    webListen = fnet_socket(AF_INET, SOCK_STREAM, 0);
    if (webListen == FNET_NULL) return false;
    struct fnet_sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = FNET_HTONS(WEB_PORT);
    a.sin_addr.s_addr = INADDR_ANY;
    // the options as NativeEthernet sets them (Nativesocket.cpp, socketBegin); the accepted connections inherit them
    const fnet_uint32_t bufSize = NET_SOCKET_SIZE;
    const fnet_int32_t  noDelay = 1;
    struct fnet_linger linger;
    linger.l_onoff = FNET_TRUE;
    linger.l_linger = 4;                                // seconds the close waits for the browser's ACK
    if (fnet_socket_bind(webListen, (struct fnet_sockaddr*)&a, sizeof(a)) == FNET_ERR ||
        fnet_socket_setopt(webListen, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize)) == FNET_ERR ||
        fnet_socket_setopt(webListen, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize)) == FNET_ERR ||
        fnet_socket_setopt(webListen, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay)) == FNET_ERR ||
        fnet_socket_setopt(webListen, SOL_SOCKET, SO_LINGER, &linger, sizeof(linger)) == FNET_ERR ||
        fnet_socket_listen(webListen, WEB_BACKLOG) == FNET_ERR)
    {
        fnet_socket_close(webListen);
        webListen = FNET_NULL;
        return false;
    }
    return true;
}

// A connection waiting on the listening socket, as an EthernetClient; an empty one when none is waiting
static EthernetClient webAccept()
{
    if (webListen == FNET_NULL) return EthernetClient();
    struct fnet_sockaddr from;
    fnet_size_t fromLen = sizeof(from);                 // the length NativeEthernet's poll() leaves unset
    fnet_socket_t s = fnet_socket_accept(webListen, &from, &fromLen);
    if (s == FNET_NULL) return EthernetClient();        // nothing waiting
    // a free place in NativeEthernet's socket table: EthernetClient reads, writes and closes it from there
    // (MAX_SOCK_NUM places, as the firmware never calls Ethernet.setSocketNum; the UDP sockets hold four)
    for (uint8_t i = 0; i < MAX_SOCK_NUM; i++)
        if (Ethernet.socket_ptr[i] == nullptr) { Ethernet.socket_ptr[i] = s; return EthernetClient(i); }
    fnet_socket_close(s);                               // no place: the browser tries again
    return EthernetClient();
}

void webSetup()
{
    if (!Ethernet_running) return;
    webEtagSetup();
    webRunning = webListenStart();
    if (!webRunning) StatusLog.println("[boot] web page: port 80 could not be opened (FAILED)");
    // (its address: the "[status] Ethernet: link up ..." line, zStatus.ino)
}

// Called from loop()
void webLoop()
{
    if (!webRunning) return;
    if (rq.active && rq.txLeft)                         // an answer still going out: one piece per pass
    {
        webTxStep();
        if (!rq.txLeft) webFinish();
        return;
    }
    if (!rq.active)
    {
        EthernetClient c = webAccept();
        if (!c) return;
        rq.client = c;
        rq.active = true;
        rq.startMs = millis();
        rq.len = 0;
        rq.bodyAt = -1;
        rq.method[0] = rq.path[0] = rq.query[0] = '\0';
        rq.contentLength = 0;
        rq.txLeft = 0;
    }
    EthernetClient& c = rq.client;
    if (millis() - rq.startMs > WEB_TIMEOUT_MS ||
        (rq.len == 0 && millis() - rq.startMs > WEB_FIRST_BYTE_MS)) { if (rq.len) webDropN++; webFinish(); return; }

    const int n = c.available();
    if (n <= 0)
    {
        if (!c.connected()) { if (rq.len) webDropN++; webFinish(); }
        return;
    }
    const int room = WEB_HEAD_MAX - rq.len;
    if (room <= 0) { webSend(c, 413, "text/plain", "Too large", 9); webFinish(); return; }
    const int r = c.read((uint8_t*)rq.head + rq.len, n < room ? n : room);
    if (r <= 0) return;
    rq.len += r;
    rq.head[rq.len] = '\0';

    if (rq.bodyAt < 0)
    {
        char* end = strstr(rq.head, "\r\n\r\n");
        if (!end) return;                                   // headers not complete yet
        const uint16_t headEnd = (uint16_t)(end - rq.head);
        rq.bodyAt = headEnd + 4;
        const uint16_t have = rq.len - rq.bodyAt;
        char saved[WEB_HEAD_MAX];                           // webParseHead puts a '\0' at headEnd
        memcpy(saved, rq.head + rq.bodyAt, have);
        if (!webParseHead(headEnd)) { webSend(c, 400, "text/plain", "Bad request", 11); webFinish(); return; }
        memcpy(rq.head + rq.bodyAt, saved, have);
        rq.head[rq.len] = '\0';
        if (!strcmp(rq.method, "POST") && !strcmp(rq.path, "/api/ota"))
        {
            apiOta(c, (const uint8_t*)rq.head + rq.bodyAt, have);
            webDone();
            return;
        }
        if (rq.contentLength > WEB_BODY_MAX) { webSend(c, 413, "text/plain", "Too large", 9); webFinish(); return; }
    }
    if ((uint32_t)(rq.len - rq.bodyAt) >= rq.contentLength)
    {
        rq.head[rq.bodyAt + rq.contentLength] = '\0';
        webDispatch();
    }
}
