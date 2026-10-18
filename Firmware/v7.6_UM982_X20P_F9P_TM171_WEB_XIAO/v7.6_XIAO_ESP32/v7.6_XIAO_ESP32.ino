/* v7.6_XIAO_ESP32.ino - WiFi, web server, settings, update, setup and loop.
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware
   UM982_X20P_F9P_TM171_WEB_XIAO (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Link to the Teensy: same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO
   New Dawn"
   (AgOpenGPS-Official, GPL v3).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// v7.6_XIAO_ESP32.ino  (version number: xVersion.h)
// Program for the Seeed XIAO ESP32C3 or XIAO ESP32S3 on the GPS2 socket of the AiO v4.5 Micro
// (adapter board or 3D-printed holder), with the Teensy running v7.6_UM982_X20P_F9P_TM171_WEB_XIAO.
// Both boards have the same pins and size; compile it for the one you have (it then calls itself
// v7.6_XIAO_ESP32C3 or v7.6_XIAO_ESP32S3, and refuses an update made for the other one).
//
// What it does:
//   - its own WiFi network (access point), always on: "XIAO-xxxx", password "agopengps" until changed.
//     A phone that joins it gets the page by itself (captive page), or opens http://192.168.4.1/
//   - optionally joins another WiFi network too (farm, phone hotspot): then also http://<name>.local/
//     or its IP on that network
//   - web page (Portuguese or English, button at the top), 4 tabs:
//       Estado     the module (Teensy): GNSS, dual heading, TM171, steering, network - sent by the
//                  Teensy once a second; what AgOpenGPS / AgOpenWeb sends to the module (speed, angle
//                  asked for, autosteer, sections, hydraulic lift, tramline); the XIAO itself
//       Mensagens  the module's [status] messages, and the XIAO's own events
//       WiFi       network settings (saved in the XIAO, applied after a restart), network scan
//       Programa   version, program update over WiFi (.bin from the Arduino IDE), restart
//     Changes (WiFi, update, restart) ask for the password of the XIAO's own WiFi network.
//   - tells the Teensy its version and WiFi state (shown in the XIAO card of the Teensy page) and
//     writes its events to the Teensy log ("[xiao] > ...")
//   - USB-C serial monitor (any baud), for the bench or the panel USB port: "estado", "wifi-reset"
//     (WiFi settings back to the defaults, e.g. a forgotten password), "reiniciar"
//
// Arduino IDE: board package "esp32" by Espressif Systems (3.x), board "XIAO_ESP32C3" or "XIAO_ESP32S3",
// Tools > USB CDC On Boot: Enabled, Partition Scheme: the default one (it has room for updates over
// WiFi). On the S3: USB Mode: Hardware CDC and JTAG; PSRAM not needed. No other libraries needed.
// The XIAO is powered by the board's 3.3 V: a USB-C cable to it must not carry 5 V (VBUS) while it
// sits on the Micro (see README).

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Arduino IDE: Tools > USB CDC On Boot > Enabled (D6/D7 are used for the Teensy)"
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_app_format.h"            // header of a program file (.bin): which chip it is for
#include "xVersion.h"
#include "xLink.h"
#include "xPage.h"

#define PIN_FROM_TEENSY  D7          // C3 GPIO20 / S3 GPIO44 <- Teensy TX2 (pin 8), GPS2 socket pin 3
#define PIN_TO_TEENSY    D6          // C3 GPIO21 / S3 GPIO43 -> Teensy RX2 (pin 7), GPS2 socket pin 2
#define DEFAULT_AP_PASS  "agopengps"
#define DEFAULT_HOST     "xiao"
#define EVENT_LINES      30
#define EVENT_LEN        120

HardwareSerial&           TeensyPort = Serial1;
TeensyLink<HardwareSerial> teensy(TeensyPort);
WebServer                 server(80);
DNSServer                 dns;
Preferences               prefs;

// ---------------------------------------------------------------- settings (saved in the XIAO's flash)
struct Settings
{
    String apSsid, apPass;       // the XIAO's own network
    bool   staOn = false;        // also join another network
    String staSsid, staPass;
    String host;                 // name on the other network: http://<host>.local/
} cfg;

static String defaultApSsid()
{
    const uint64_t mac = ESP.getEfuseMac();
    char b[16];
    snprintf(b, sizeof(b), "XIAO-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
    return String(b);
}

static void settingsLoad()
{
    prefs.begin("xiao", true);
    cfg.apSsid  = prefs.getString("apSsid", defaultApSsid());
    cfg.apPass  = prefs.getString("apPass", DEFAULT_AP_PASS);
    cfg.staOn   = prefs.getBool("staOn", false);
    cfg.staSsid = prefs.getString("staSsid", "");
    cfg.staPass = prefs.getString("staPass", "");
    cfg.host    = prefs.getString("host", DEFAULT_HOST);
    prefs.end();
    if (cfg.apSsid.length() < 1 || cfg.apSsid.length() > 32) cfg.apSsid = defaultApSsid();
    if (cfg.apPass.length() < 8 || cfg.apPass.length() > 63) cfg.apPass = DEFAULT_AP_PASS;
    if (cfg.host.length() < 1 || cfg.host.length() > 24)    cfg.host = DEFAULT_HOST;
}

static void settingsSave()
{
    prefs.begin("xiao", false);
    prefs.putString("apSsid", cfg.apSsid);
    prefs.putString("apPass", cfg.apPass);
    prefs.putBool("staOn", cfg.staOn);
    prefs.putString("staSsid", cfg.staSsid);
    prefs.putString("staPass", cfg.staPass);
    prefs.putString("host", cfg.host);
    prefs.end();
}

static void settingsReset()
{
    prefs.begin("xiao", false);
    prefs.clear();
    prefs.end();
}

// ---------------------------------------------------------------- the XIAO's own events
struct Event { uint32_t seq, ms; char text[EVENT_LEN + 1]; };
static Event    events[EVENT_LINES];
static uint32_t eventNext = 1;

// An event: kept for the page, printed on the USB monitor and sent to the Teensy log
static void logEvent(const char* fmt, ...)
{
    char b[EVENT_LEN + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    Event& e = events[eventNext % EVENT_LINES];
    e.seq = eventNext++;
    e.ms = millis();
    strncpy(e.text, b, EVENT_LEN);
    e.text[EVENT_LEN] = '\0';
    if (Serial) Serial.println(b);
    teensy.sendText(b);
}

// ---------------------------------------------------------------- small JSON writer (fixed buffer)
struct Json
{
    char* b; size_t cap, n; bool first;
    Json(char* buf, size_t size) : b(buf), cap(size), n(0), first(true) { b[0] = '\0'; }
    void raw(const char* s)              { while (*s && n + 1 < cap) b[n++] = *s++; b[n] = '\0'; }
    void raw(const char* s, size_t k)    { while (k-- && n + 1 < cap) b[n++] = *s++; b[n] = '\0'; }
    void ch(char c)                      { if (n + 1 < cap) { b[n++] = c; b[n] = '\0'; } }
    void sep()                           { if (!first) ch(','); first = false; }
    void key(const char* k)              { sep(); ch('"'); raw(k); raw("\":"); }
    void esc(const char* s)
    {
        ch('"');
        for (; *s; s++)
        {
            const uint8_t c = (uint8_t)*s;
            if (c == '"' || c == '\\') { ch('\\'); ch((char)c); }
            else if (c < 0x20)         { char t[8]; snprintf(t, sizeof(t), "\\u%04x", c); raw(t); }
            else                        ch((char)c);
        }
        ch('"');
    }
    void str(const char* k, const char* v)   { key(k); esc(v); }
    void num(const char* k, double v, int d) { key(k); char t[32]; snprintf(t, sizeof(t), "%.*f", d, v); raw(t); }
    void i(const char* k, long v)            { key(k); char t[16]; snprintf(t, sizeof(t), "%ld", v); raw(t); }
    void u(const char* k, unsigned long v)   { key(k); char t[16]; snprintf(t, sizeof(t), "%lu", v); raw(t); }
    void bo(const char* k, bool v)           { key(k); raw(v ? "true" : "false"); }
    void obj(const char* k)                  { key(k); ch('{'); first = true; }
    void arr(const char* k)                  { key(k); ch('['); first = true; }
    void open()                              { sep(); ch('{'); first = true; }
    void close()                             { ch('}'); first = false; }
    void closeArr()                          { ch(']'); first = false; }
};
static char jbuf[8192];
static uint32_t sketchSize = 0, sketchFree = 0;   // read once at start-up (getSketchSize() reads the whole program)

// ---------------------------------------------------------------- WiFi
static bool     staWasUp = false;
static int      apClientsWas = -1;
static uint32_t rebootAtMs = 0;
static String   staIpStr = "";

static const char* resetReasonText()
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:  return "corrente ligada";               // the module was switched on
        case ESP_RST_SW:       return "reinício pedido";               // page, update, new WiFi settings, USB command
        case ESP_RST_PANIC:    return "erro do programa";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:      return "programa parado (watchdog)";
        case ESP_RST_BROWNOUT: return "falha de tensão";
        case ESP_RST_DEEPSLEEP:return "deep sleep";
        case ESP_RST_EXT:      return "botão RESET";
        default:               return "outro";
    }
}

static const char* staState()
{
    if (!cfg.staOn || cfg.staSsid.length() == 0) return "off";
    return WiFi.status() == WL_CONNECTED ? "ok" : "con";
}

static void wifiStart()
{
    WiFi.persistent(false);
    WiFi.setHostname(cfg.host.c_str());
    const bool sta = cfg.staOn && cfg.staSsid.length() > 0;
    WiFi.mode(sta ? WIFI_AP_STA : WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str());
    if (sta)
    {
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
    }
    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", WiFi.softAPIP());            // every name -> this page (captive page)
    if (MDNS.begin(cfg.host.c_str())) MDNS.addService("http", "tcp", 80);
    logEvent("WiFi: own network '%s' at 192.168.4.1%s", cfg.apSsid.c_str(), sta ? ", joining the other network" : "");
}

// State changes of the other network and of the clients on ours: events
static void wifiWatch()
{
    const bool up = WiFi.status() == WL_CONNECTED;
    if (up != staWasUp)
    {
        staWasUp = up;
        if (up)
        {
            staIpStr = WiFi.localIP().toString();
            logEvent("WiFi: joined '%s', IP %s, %d dBm, channel %d", cfg.staSsid.c_str(), staIpStr.c_str(), (int)WiFi.RSSI(), (int)WiFi.channel());
        }
        else if (cfg.staOn) logEvent("WiFi: lost the network '%s'", cfg.staSsid.c_str());
    }
    const int cl = WiFi.softAPgetStationNum();
    if (cl != apClientsWas)
    {
        if (apClientsWas >= 0) logEvent("WiFi: %d device(s) on the XIAO network", cl);
        apClientsWas = cl;
    }
}

// Our state for the Teensy page (PGN 143)
static void sendInfo()
{
    char b[251];
    Json j(b, sizeof(b));
    j.raw("{");
    j.str("v", XIAO_FW_VERSION);
    j.str("c", XIAO_CHIP_NICE);
    j.str("ap", cfg.apSsid.c_str());
    j.str("apIp", "192.168.4.1");
    j.i("cl", WiFi.softAPgetStationNum());
    if (cfg.staOn && cfg.staSsid.length())
    {
        j.str("sta", cfg.staSsid.c_str());
        j.str("st", staState());
        if (WiFi.status() == WL_CONNECTED) { j.str("ip", WiFi.localIP().toString().c_str()); j.i("rssi", WiFi.RSSI()); }
    }
    j.u("up", millis() / 1000);
    j.raw("}");
    if (j.n + 1 < sizeof(b)) teensy.sendInfo(b);
}

// ---------------------------------------------------------------- web: helpers
static void sendJson(Json& j, int code = 200)
{
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(code, "application/json", j.b, j.n);
}

static void sendError(int code, const char* msg)
{
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    j.bo("ok", false);
    j.str("error", msg);
    j.raw("}");
    sendJson(j, code);
}

static bool passwordOk()
{
    if (server.arg("pw") == cfg.apPass) return true;
    sendError(403, "wrong password");
    return false;
}

// Requests for any other host name (phones checking for internet) go to our page
static bool captiveRedirect()
{
    const String h = server.hostHeader();
    if (h.length() == 0 || h == "192.168.4.1" || h == staIpStr || h == cfg.host + ".local") return false;
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
    return true;
}

// ---------------------------------------------------------------- web: answers
static void apiState()
{
    const uint32_t now = millis();
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    j.str("fw", XIAO_FW_NAME);
    j.str("ver", XIAO_FW_VERSION);
    j.str("chip", XIAO_CHIP_NICE);
    j.i("chipId", CONFIG_IDF_FIRMWARE_CHIP_ID);
    j.str("built", __DATE__ " " __TIME__);
    j.u("up", now / 1000);
    j.str("reset", resetReasonText());
    const esp_reset_reason_t rr = esp_reset_reason();
    j.bo("resetBad", rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT);
    j.u("heap", ESP.getFreeHeap());
    j.u("heapMin", ESP.getMinFreeHeap());
    j.u("sketch", sketchSize);
    j.u("sketchFree", sketchFree);

    j.obj("wifi");
    j.str("ap", cfg.apSsid.c_str());
    j.i("apCl", WiFi.softAPgetStationNum());
    j.i("ch", WiFi.channel());
    j.str("host", cfg.host.c_str());
    j.bo("staOn", cfg.staOn && cfg.staSsid.length() > 0);
    j.str("sta", cfg.staSsid.c_str());
    j.str("st", staState());
    if (WiFi.status() == WL_CONNECTED) { j.str("ip", WiFi.localIP().toString().c_str()); j.i("rssi", WiFi.RSSI()); }
    j.close();

    j.obj("link");
    j.bo("talking", teensy.teensyTalking(now));
    j.i("lastAgo", teensy.lastFrameMs ? (long)((now - teensy.lastFrameMs) / 1000) : -1);
    j.u("frames", teensy.goodFrames);
    j.u("crcErr", teensy.crcErr);
    j.u("cut", teensy.cut);
    j.num("rxKB", teensy.rxBytes / 1024.0, 1);
    j.num("txKB", teensy.txBytes / 1024.0, 1);
    j.u("hellos", teensy.hellos);
    j.u("sets", teensy.statusSets);
    j.u("broken", teensy.statusBroken);
    j.close();

    j.obj("aog");
    j.i("helloAgo", teensy.last200Ms ? (long)((now - teensy.last200Ms) / 1000) : -1);
    j.num("rate254", teensy.rate254, 0);
    j.i("ago254", teensy.p254.have ? (long)((now - teensy.p254.ms) / 1000) : -1);
    j.num("kmh", teensy.p254.kmh, 1);
    j.num("angle", teensy.p254.angle, 2);
    j.i("status", teensy.p254.status);
    j.u("sec", teensy.p254.sections);
    j.i("ago239", teensy.p239.have ? (long)((now - teensy.p239.ms) / 1000) : -1);
    j.i("uturn", teensy.p239.uturn);
    j.i("hyd", teensy.p239.hydLift);
    j.i("tram", teensy.p239.tram);
    j.i("geo", teensy.p239.geoStop);
    j.arr("pgns");
    for (int p = 0; p < 256; p++)
        if (teensy.pgnCount[p]) { char t[32]; snprintf(t, sizeof(t), "[%d,%lu]", p, (unsigned long)teensy.pgnCount[p]); j.sep(); j.raw(t); }
    j.closeArr();
    j.close();

    j.raw("}");
    sendJson(j);
}

// The module status as the Teensy sent it, with its age: {"ago":s,"s":{...}}
static void apiTeensy()
{
    const uint32_t now = millis();
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    j.i("ago", teensy.statusMs ? (long)((now - teensy.statusMs) / 1000) : -1);
    if (teensy.statusLen) { j.key("s"); j.raw(teensy.status, teensy.statusLen); }
    j.raw("}");
    sendJson(j);
}

// Module log lines after "after" (Teensy numbers) and XIAO events after "xafter"; at most 20 + 10
// lines per answer ("more": the page asks again for the rest)
static void apiLog()
{
    const uint32_t after = strtoul(server.arg("after").c_str(), nullptr, 10);
    const uint32_t xafter = strtoul(server.arg("xafter").c_str(), nullptr, 10);
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    j.u("epoch", teensy.logEpoch);
    j.u("last", teensy.logLast);
    j.u("now", millis());
    j.arr("t");
    int sent = 0;
    const uint32_t first = teensy.logLast > LINK_LOG_LINES ? teensy.logLast - LINK_LOG_LINES + 1 : 1;
    bool more = false;
    for (uint32_t s = (after + 1 > first ? after + 1 : first); s <= teensy.logLast; s++)
    {
        const LinkLogLine& l = teensy.logRing[s % LINK_LOG_LINES];
        if (l.seq != s) continue;
        if (sent == 20) { more = true; break; }
        j.open(); j.u("s", l.seq); j.u("t", l.ms); j.str("x", l.text); j.close();
        sent++;
    }
    j.closeArr();
    j.u("xlast", eventNext - 1);
    j.arr("x");
    const uint32_t xfirst = eventNext > EVENT_LINES ? eventNext - EVENT_LINES : 1;
    int xsent = 0;
    for (uint32_t s = (xafter + 1 > xfirst ? xafter + 1 : xfirst); s < eventNext; s++)
    {
        const Event& e = events[s % EVENT_LINES];
        if (e.seq != s) continue;
        if (xsent == 10) { more = true; break; }
        j.open(); j.u("s", e.seq); j.u("t", e.ms); j.str("x", e.text); j.close();
        xsent++;
    }
    j.closeArr();
    j.bo("more", more);
    j.raw("}");
    sendJson(j);
}

static void apiSettingsGet()
{
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    j.str("apSsid", cfg.apSsid.c_str());
    j.bo("staOn", cfg.staOn);
    j.str("staSsid", cfg.staSsid.c_str());
    j.bo("staPassSet", cfg.staPass.length() > 0);
    j.str("host", cfg.host.c_str());
    j.raw("}");
    sendJson(j);
}

static bool printable(const String& s, size_t lo, size_t hi)
{
    if (s.length() < lo || s.length() > hi) return false;
    for (size_t i = 0; i < s.length(); i++) if ((uint8_t)s[i] < 0x20 || (uint8_t)s[i] > 0x7E) return false;
    return true;
}

static void apiSettingsPost()
{
    if (!passwordOk()) return;
    Settings s = cfg;
    s.apSsid = server.arg("apSsid");
    if (!printable(s.apSsid, 1, 32)) { sendError(400, "network name: 1 to 32 characters"); return; }
    const String apPass = server.arg("apPass");
    if (apPass.length() && !printable(apPass, 8, 63)) { sendError(400, "password: 8 to 63 characters"); return; }
    if (apPass.length()) s.apPass = apPass;
    s.staOn = server.arg("staOn") == "1";
    const String staSsid = server.arg("staSsid");
    if (staSsid.length() && !printable(staSsid, 1, 32)) { sendError(400, "other network name: 1 to 32 characters"); return; }
    const String staPass = server.arg("staPass");
    if (staPass.length() && !printable(staPass, 8, 63)) { sendError(400, "other network password: 8 to 63 characters"); return; }
    if (staSsid != cfg.staSsid) s.staPass = staPass;          // another network: its password (empty = open)
    else if (staPass.length()) s.staPass = staPass;           // same network: empty keeps the old one
    s.staSsid = staSsid;
    if (s.staOn && s.staSsid.length() == 0) { sendError(400, "other network: no name"); return; }
    String host = server.arg("host");
    host.toLowerCase();
    bool hostOk = host.length() >= 1 && host.length() <= 24;
    for (size_t i = 0; i < host.length(); i++) if (!isalnum((unsigned char)host[i]) && host[i] != '-') hostOk = false;
    if (!hostOk) { sendError(400, "name on the network: letters, digits and '-', up to 24"); return; }
    s.host = host;
    cfg = s;
    settingsSave();
    logEvent("Settings saved: own network '%s', other network %s, name %s.local - restarting", cfg.apSsid.c_str(),
          cfg.staOn ? cfg.staSsid.c_str() : "off", cfg.host.c_str());
    server.send(200, "application/json", "{\"ok\":true}");
    rebootAtMs = millis() + 1500;
}

static int16_t scanState = -2;            // -2 not started, -1 running

static void apiScan()
{
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    const int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) { j.bo("busy", true); }
    else if (n >= 0 && scanState == -1)
    {
        j.bo("busy", false);
        j.arr("list");
        for (int i = 0; i < n; i++)
        {
            const String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;
            bool dup = false;
            for (int k = 0; k < i; k++) if (WiFi.SSID(k) == ssid) dup = true;   // strongest first (sorted)
            if (dup) continue;
            j.open(); j.str("s", ssid.c_str()); j.i("r", WiFi.RSSI(i)); j.i("c", WiFi.channel(i)); j.bo("open", WiFi.encryptionType(i) == WIFI_AUTH_OPEN); j.close();
        }
        j.closeArr();
        WiFi.scanDelete();
        scanState = -2;
    }
    else
    {
        WiFi.scanNetworks(true);          // in the background; the page asks again
        scanState = -1;
        j.bo("busy", true);
    }
    j.raw("}");
    sendJson(j);
}

// Program update: the .bin from "Sketch > Export Compiled Binary"
static bool    updOk = false;
static String  updErr;

static void updateUpload()
{
    HTTPUpload& up = server.upload();
    feedLoopWDT();
    teensy.poll(millis());                  // keep saying hello to the Teensy while the file comes in
    if (up.status == UPLOAD_FILE_START)
    {
        updOk = false;
        updErr = "";
        if (server.arg("pw") != cfg.apPass) { updErr = "wrong password"; return; }
        logEvent("Program update: receiving %s", up.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) updErr = Update.errorString();
    }
    else if (up.status == UPLOAD_FILE_WRITE)
    {
        // The first piece holds the program header: refuse a program made for the other XIAO (C3 / S3)
        if (updErr.length() == 0 && Update.progress() == 0 && up.currentSize >= sizeof(esp_image_header_t))
        {
            const esp_image_header_t* h = (const esp_image_header_t*)up.buf;
            if (h->magic == ESP_IMAGE_HEADER_MAGIC && h->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID)
            {
                updErr = h->chip_id == ESP_CHIP_ID_ESP32C3 ? "other chip ESP32-C3" : h->chip_id == ESP_CHIP_ID_ESP32S3 ? "other chip ESP32-S3" : "other chip";
            }
        }
        if (updErr.length() == 0 && Update.write(up.buf, up.currentSize) != up.currentSize) updErr = Update.errorString();
    }
    else if (up.status == UPLOAD_FILE_END)
    {
        if (updErr.length() == 0)
        {
            if (Update.end(true)) updOk = true;
            else updErr = Update.errorString();
        }
        else Update.abort();
    }
    else if (up.status == UPLOAD_FILE_ABORTED)
    {
        Update.abort();
        updErr = "upload aborted";
    }
}

static void updateDone()
{
    if (updOk)
    {
        logEvent("Program update: %u bytes checked, restarting", (unsigned)Update.progress());
        server.send(200, "application/json", "{\"ok\":true}");
        rebootAtMs = millis() + 1500;
    }
    else
    {
        if (updErr.length() == 0) updErr = "no file";
        logEvent("Program update REFUSED: %s", updErr.c_str());
        sendError(updErr == "wrong password" ? 403 : 400, updErr.c_str());
    }
}

static void apiRestart()
{
    if (!passwordOk()) return;
    logEvent("Restart requested from the page");
    server.send(200, "application/json", "{\"ok\":true}");
    rebootAtMs = millis() + 1000;
}

static void webSetup()
{
    server.on("/", HTTP_GET, []() {
        if (captiveRedirect()) return;
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html; charset=utf-8", XIAO_PAGE);
    });
    server.on("/api/state", HTTP_GET, apiState);
    server.on("/api/teensy", HTTP_GET, apiTeensy);
    server.on("/api/log", HTTP_GET, apiLog);
    server.on("/api/settings", HTTP_GET, apiSettingsGet);
    server.on("/api/settings", HTTP_POST, apiSettingsPost);
    server.on("/api/scan", HTTP_GET, apiScan);
    server.on("/api/restart", HTTP_POST, apiRestart);
    server.on("/update", HTTP_POST, updateDone, updateUpload);
    server.onNotFound([]() {
        if (captiveRedirect()) return;
        server.send(404, "text/plain", "Not found");
    });
    server.begin();
}

// ---------------------------------------------------------------- USB-C serial monitor
static char    cmd[40];
static uint8_t cmdLen = 0;

static void printState()
{
    Serial.printf("%s, up %lu s, last reset: %s\n", XIAO_FW_NAME, (unsigned long)(millis() / 1000), resetReasonText());
    Serial.printf("Own WiFi '%s' at 192.168.4.1, %d client(s)\n", cfg.apSsid.c_str(), WiFi.softAPgetStationNum());
    if (cfg.staOn) Serial.printf("Other network '%s': %s %s\n", cfg.staSsid.c_str(), staState(), WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "");
    Serial.printf("Teensy: %s, %lu frames, %lu bad checksums, status %ld s ago\n", teensy.teensyTalking(millis()) ? "talking" : "SILENT",
                  (unsigned long)teensy.goodFrames, (unsigned long)teensy.crcErr, teensy.statusMs ? (long)((millis() - teensy.statusMs) / 1000) : -1L);
    Serial.println("Commands: estado | wifi-reset | reiniciar");
}

static void usbCommands()
{
    while (Serial.available() > 0)
    {
        const char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (cmdLen < sizeof(cmd) - 1) cmd[cmdLen++] = c; continue; }
        cmd[cmdLen] = '\0';
        cmdLen = 0;
        if (!strcmp(cmd, "estado") || !strcmp(cmd, "status")) printState();
        else if (!strcmp(cmd, "wifi-reset"))
        {
            settingsReset();
            logEvent("WiFi settings back to the defaults (USB command) - restarting");
            delay(300);
            ESP.restart();
        }
        else if (!strcmp(cmd, "reiniciar") || !strcmp(cmd, "restart")) { delay(100); ESP.restart(); }
        else if (cmd[0]) Serial.println("Commands: estado | wifi-reset | reiniciar");
    }
}

// ---------------------------------------------------------------- setup / loop
void setup()
{
    Serial.begin(115200);                                     // USB-C (bench / panel port)
    TeensyPort.setRxBufferSize(4096);                         // a busy web page must not lose the Teensy's data
    TeensyPort.begin(LINK_BAUD, SERIAL_8N1, PIN_FROM_TEENSY, PIN_TO_TEENSY);
    TeensyPort.setRxFIFOFull(64);                             // see README: room for the WiFi's busy moments
    teensy.poll(millis());                                      // first hello at once
    settingsLoad();
    sketchSize = ESP.getSketchSize();
    sketchFree = ESP.getFreeSketchSpace();
    logEvent("%s started (%s)", XIAO_FW_NAME, resetReasonText());
    wifiStart();
    webSetup();
    esp_task_wdt_config_t w = {15000, 0, true};               // restart if the program hangs for 15 s
    esp_task_wdt_reconfigure(&w);
    enableLoopWDT();
}

void loop()
{
    static uint32_t infoMs = 0, watchMs = 0;
    const uint32_t now = millis();

    teensy.poll(now);
    dns.processNextRequest();
    server.handleClient();
    usbCommands();

    if (now - watchMs >= 500) { watchMs = now; wifiWatch(); }
    if (now - infoMs >= LINK_INFO_MS || infoMs == 0) { infoMs = now | 1; sendInfo(); }
    if (rebootAtMs && (int32_t)(now - rebootAtMs) >= 0) { delay(50); ESP.restart(); }
    delay(1);                                                 // let the WiFi have the core
}
