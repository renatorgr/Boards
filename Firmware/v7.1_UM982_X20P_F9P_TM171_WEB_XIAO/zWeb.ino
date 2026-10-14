// zWeb.ino
// The module's web page: http://<module IP>/  (normally http://192.168.5.126/) from the
// AgOpenWeb tablet, a phone or a PC on the same network. No need to open the box:
//   - Status: GNSS, dual heading, TM171, steering, network, what the receiver sends, the
//     sentences sent to AgOpenWeb / AgOpenGPS, the XIAO ESP32C3 bridge (zXiao.ino)
//   - Messages: the [status] lines (zStatus.ino)
//   - Settings (zSettings.ino), saved in the EEPROM
//   - UM982 console: commands to the receiver and its replies
//   - Firmware update (zOTA.ino) and restart
//
// One request at a time, answered with "Connection: close". Reading a request and sending the
// answer never wait for the browser longer than WEB_TIMEOUT_MS, so the GNSS / steering loop keeps
// running. Only the firmware update takes over the loop, with the steering switched off.
//
// There is no password: anyone on the module's network can open the page.

#include "zWebPage.h"            // the page itself (HTML, CSS, JavaScript), kept in flash
#include "zWebJson.h"            // small JSON writer for the answers

#define FW_NAME          "v7_UM982_X20P_F9P_TM171_WEB_XIAO"
#define FW_VERSION       "7.1"

#define WEB_PORT         80
#define WEB_HEAD_MAX     2048    // request line + headers + small body (settings, UM982 command)
#define WEB_BODY_MAX     512
#define WEB_TIMEOUT_MS   3000
#define WEB_FIRST_BYTE_MS 500    // a connection with no request yet (browser "preconnect") is closed after this
#define OTA_MAX_KMH      1.0f    // no firmware update above this speed

EthernetServer webServer(WEB_PORT);
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
} rq;

static char webBuf[16384];       // JSON answers

// ---------------------------------------------------------------- sending
static bool webWrite(EthernetClient& c, const char* d, size_t n)
{
    uint32_t t0 = millis();
    while (n > 0)
    {
        const int room = c.availableForWrite();
        if (room <= 0)
        {
            if (!c.connected() || millis() - t0 > WEB_TIMEOUT_MS) return false;
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

static void webSend(EthernetClient& c, int code, const char* type, const char* body, size_t len)
{
    const char* text = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" :
                       code == 409 ? "Conflict" : code == 413 ? "Payload Too Large" : "Error";
    char h[200];
    const int hn = snprintf(h, sizeof(h),
                            "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                            "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                            code, text, type, (unsigned)len);
    if (webWrite(c, h, hn)) webWrite(c, body, len);
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
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
}

static bool autosteerEngaged()
{
    return watchdogTimer < WATCHDOG_THRESHOLD;
}

// ---------------------------------------------------------------- answers
static const char* fixNameOf(int q)
{
    switch (q)
    {
        case 0: return "no fix";   case 1: return "GPS";      case 2: return "DGPS";
        case 4: return "RTK fixed"; case 5: return "RTK float"; default: return "other";
    }
}

static void apiStatus(EthernetClient& c)
{
    const uint32_t now = millis();
    Json j(webBuf, sizeof(webBuf));
    j.raw("{");
    j.str("fw", FW_NAME " " FW_VERSION);
    j.str("built", __DATE__ " " __TIME__);
    j.str("target", otaFlashId);
    j.str("marker", otaMarker);
    j.u("up", now / 1000);
    {
        char ip[20];
        IPAddress a = Ethernet.localIP();
        snprintf(ip, sizeof(ip), "%u.%u.%u.%u", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
        j.str("ip", ip);
    }

    j.obj("net");
    j.bo("link", Ethernet.linkStatus() == LinkON);
    j.bo("aog", aogLastMs != 0 && now - aogLastMs < STATUS_SILENCE_MS);
    {
        char ip[20];
        snprintf(ip, sizeof(ip), "%u.%u.%u.%u", aogIp[0], aogIp[1], aogIp[2], aogIp[3]);
        j.str("aogIp", ip);
    }
    j.bo("rtcm", rtcmLastMs != 0 && now - rtcmLastMs < STATUS_SILENCE_MS);
    j.num("rtcmKB", rtcmBytes / 1024.0, 1);
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
    j.str("rx", rxTypesLast);
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
    j.str("raw", hprRawLast);
    j.u("late", hprLateCount);
    j.close();

    j.obj("tm");
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
    j.u("crcErr", TM171crcErrors);
    j.close();

    j.obj("steer");
    j.num("was", steerAngleActual, 2);
    j.num("set", steerAngleSetPoint, 2);
    j.i("pwm", pwmDisplay);
    j.bo("switchOn", steerSwitch == 0);
    j.bo("active", autosteerEngaged());
    j.str("motor", isKeya ? "Keya" : steerConfig.CytronDriver ? "Cytron" : "IBT2");
    j.num("aogKmh", gpsSpeed, 1);
    j.i("guidance", guidanceStatus);
    j.close();

    j.obj("out");
    j.str("mode", cfg.outputMode == OUT_CLASSIC ? "classic" : "standard");
    j.i("kind", lastSentKind);
    j.u("epochs", epochsSent);
    j.arr("lines");
    for (int k = 0; k < SENT_SLOTS; k++) if (lastSent[k][0]) j.item(lastSent[k]);
    j.closeArr();
    j.close();

    j.obj("xiao");
    j.bo("on", cfg.xiaoOff == 0);
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
    j.close();

    j.raw("}");
    webSendJson(c, 200, webBuf);
}

static void apiLog(EthernetClient& c)
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

static void settingsJson(Json& j)
{
    j.obj("settings");
    j.i("outputMode", cfg.outputMode);
    j.i("classicNoDual", cfg.classicNoDual);
    j.i("hprAcceptFloat", cfg.hprAcceptFloat);
    j.i("epochWaitMs", cfg.epochWaitMs);
    j.i("tm171SwapRollPitch", cfg.tm171SwapRollPitch);
    j.i("tm171YawSign", cfg.tm171YawSign);
    j.i("xiaoOff", cfg.xiaoOff);
    j.close();
    j.bo("saved", cfgFromEeprom);
}

static void apiSettingsGet(EthernetClient& c)
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
            {"outputMode",         0, 1,   &s.outputMode,         false},
            {"classicNoDual",      0, 1,   &s.classicNoDual,      false},
            {"hprAcceptFloat",     0, 1,   &s.hprAcceptFloat,     false},
            {"epochWaitMs",        20, 100, &s.epochWaitMs,       false},
            {"tm171SwapRollPitch", 0, 1,   &s.tm171SwapRollPitch, false},
            {"tm171YawSign",       -1, 1,  &s.tm171YawSign,       true},
            {"xiaoOff",            0, 1,   &s.xiaoOff,            false},
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
    cfg = s;
    settingsSave();
    settingsApply(yawChanged);
    if (xiaoChanged) xiaoSettingsChanged();
    StatusLog.line("[web] settings saved: output %s%s, float heading %s, epoch wait %u ms, TM171 swap %s, yaw %s, XIAO bridge %s",
                   cfg.outputMode == OUT_CLASSIC ? "classic" : "standard",
                   cfg.outputMode == OUT_CLASSIC ? (cfg.classicNoDual ? " ($PAOGI without dual)" : " ($PANDA without dual)") : "",
                   cfg.hprAcceptFloat ? "yes" : "no", cfg.epochWaitMs, cfg.tm171SwapRollPitch ? "yes" : "no",
                   cfg.tm171YawSign == 0 ? "auto" : cfg.tm171YawSign > 0 ? "+1" : "-1", cfg.xiaoOff ? "off" : "on");
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
    for (size_t k = 0; k < n; k++) if ((uint8_t)p[k] < 0x20 || (uint8_t)p[k] > 0x7E) { webSendError(c, 400, "invalid character"); return; }
    rxReplyStartCapture();
    SerialGPS->print(p);
    SerialGPS->print("\r\n");
    StatusLog.line("[web] UM982 <- %s", p);
    webSendJson(c, 200, "{\"ok\":true}");
}

static void apiUm982Get(EthernetClient& c)
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

static void restartTeensy()
{
    delay(300);
    SCB_AIRCR = 0x05FA0004;
    for (;;) {}
}

// The firmware update: takes over the loop until the whole file is in (or fails).
// pre/preLen = the first body bytes, already read with the headers.
static void apiOta(EthernetClient& c, const uint8_t* pre, size_t preLen)
{
    char v[8];
    const bool force = formValue(rq.query, "force", v, sizeof(v)) && !strcmp(v, "1");
    if (autosteerEngaged())                                  { webSendError(c, 409, "autosteer is engaged - disengage it first"); return; }
    if (lastSpeedKmh > OTA_MAX_KMH || gpsSpeed > OTA_MAX_KMH) { webSendError(c, 409, "the tractor is moving - stop first"); return; }
    if (rq.contentLength < 1000 || rq.contentLength > 8000000) { webSendError(c, 400, "no file, or file size not plausible"); return; }

    steerSafeOff();
    StatusLog.line("[web] firmware update: receiving %lu bytes, steering off", (unsigned long)rq.contentLength);
    const char* err = otaBegin();
    if (!err) err = otaFeed(pre, preLen);
    uint32_t got = preLen, last = millis();
    static uint8_t buf[2048];
    while (!err && got < rq.contentLength)
    {
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
        else                                  delay(1);
    }
    uint32_t size = 0;
    if (!err) err = otaFinish(force, &size);
    if (err)
    {
        StatusLog.line("[web] firmware update REFUSED: %s (old firmware kept)", err);
        webSendError(c, 400, err);
        return;
    }
    StatusLog.line("[web] firmware update: %lu bytes checked, installing and restarting", (unsigned long)size);
    char j[160];
    snprintf(j, sizeof(j), "{\"ok\":true,\"size\":%lu}", (unsigned long)size);
    webSendJson(c, 200, j);
    delay(400);                                            // let the answer leave (the network runs on a timer)
    c.stop();
    delay(100);
    otaInstall(size);                                      // does not return
}

// ---------------------------------------------------------------- request handling
static void webFinish()
{
    rq.client.stop();
    rq.active = false;
}

static void webDispatch()
{
    EthernetClient& c = rq.client;
    const bool get = !strcmp(rq.method, "GET"), post = !strcmp(rq.method, "POST");
    const char* body = rq.bodyAt >= 0 ? rq.head + rq.bodyAt : "";
    if      (get  && (!strcmp(rq.path, "/") || !strcmp(rq.path, "/index.html")))
        webSend(c, 200, "text/html; charset=utf-8", WEB_PAGE, strlen(WEB_PAGE));
    else if (get  && !strcmp(rq.path, "/api/status"))   apiStatus(c);
    else if (get  && !strcmp(rq.path, "/api/log"))      apiLog(c);
    else if (get  && !strcmp(rq.path, "/api/settings")) apiSettingsGet(c);
    else if (post && !strcmp(rq.path, "/api/settings")) apiSettingsPost(c, body);
    else if (get  && !strcmp(rq.path, "/api/um982"))    apiUm982Get(c);
    else if (post && !strcmp(rq.path, "/api/um982"))    apiUm982Post(c, body);
    else if (post && !strcmp(rq.path, "/api/reboot"))
    {
        StatusLog.line("[web] restart requested");
        webSendJson(c, 200, "{\"ok\":true}");
        c.stop();
        restartTeensy();
    }
    else webSend(c, 404, "text/plain", "Not found", 9);
    webFinish();
}

// Request line and headers are complete in rq.head[0 .. headEnd)
static bool webParseHead(uint16_t headEnd)
{
    rq.head[headEnd] = '\0';
    char target[160] = {};
    if (sscanf(rq.head, "%7s %159s", rq.method, target) != 2) return false;
    char* q = strchr(target, '?');
    if (q) { *q = '\0'; strncpy(rq.query, q + 1, sizeof(rq.query) - 1); }
    strncpy(rq.path, target, sizeof(rq.path) - 1);
    rq.contentLength = 0;
    for (char* l = strstr(rq.head, "\r\n"); l; l = strstr(l + 2, "\r\n"))
        if (!strncasecmp(l + 2, "Content-Length:", 15)) rq.contentLength = strtoul(l + 17, nullptr, 10);
    return true;
}

void webSetup()
{
    if (!Ethernet_running) return;
    webServer.begin();
    webRunning = true;
    IPAddress a = Ethernet.localIP();
    StatusLog.line("[web] page at http://%u.%u.%u.%u/  (%s %s, built %s)", (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3],
                   FW_NAME, FW_VERSION, __DATE__);
}

// Called from loop()
void webLoop()
{
    if (!webRunning) return;
    if (!rq.active)
    {
        EthernetClient c = webServer.accept();
        if (!c) return;
        rq.client = c;
        rq.active = true;
        rq.startMs = millis();
        rq.len = 0;
        rq.bodyAt = -1;
        rq.method[0] = rq.path[0] = rq.query[0] = '\0';
        rq.contentLength = 0;
    }
    EthernetClient& c = rq.client;
    if (millis() - rq.startMs > WEB_TIMEOUT_MS ||
        (rq.len == 0 && millis() - rq.startMs > WEB_FIRST_BYTE_MS)) { webFinish(); return; }

    const int n = c.available();
    if (n <= 0)
    {
        if (!c.connected()) webFinish();
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
            webFinish();
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
