/* v4.5_XIAO_ESP32.ino - WiFi, web server, settings, update, setup and loop.
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware AiO_WEB.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Link to the Teensy: same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO
   New Dawn"
   (AgOpenGPS-Official, GPL v3).
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// v4.5_XIAO_ESP32.ino  (version number: xVersion.h)
// Program for the Seeed XIAO ESP32C3 or XIAO ESP32S3 on the GPS2 port of the AiO v4.5 boards (Micro:
// GPS2 socket, adapter board or 3D-printed holder; Standard: wired to header P8), with the Teensy running v4.5_AiO_WEB.
// Both boards have the same pins and size; compile it for the one you have (it then calls itself
// v4.5_XIAO_ESP32C3 or v4.5_XIAO_ESP32S3, and refuses an update made for the other one).
//
// What it does:
//   - its own WiFi network (access point), always on: "XIAO-xxxx", password "agopengps" until changed.
//     A phone that joins it gets the page by itself (captive page), or opens http://192.168.4.1/ (with the WiFi
//     bridge on, the module's numbers: normally http://192.168.5.1/); "Done on this device" ends the sign-in notice
//     on that device ("the sign-in page" below)
//   - optionally joins another WiFi network too (farm, phone hotspot): then also http://<name>.local/
//     or its IP on that network
//   - web page (English; dark, or light when the phone asks for it, button at the top), 4 tabs:
//       Status     the module (Teensy): GNSS, dual heading, TM171, steering, network - sent by the
//                  Teensy once a second; what AgOpenGPS / AgOpenWeb sends to the module (speed, angle
//                  asked for, autosteer, sections, hydraulic lift, tramline); the XIAO itself
//       Messages   the module's [status] messages, and the XIAO's own events
//       WiFi       network settings (saved in the XIAO, applied after a restart), network scan;
//                  the module's (Teensy's) Ethernet subnet, for a module on another network (2026-10-24)
//       Firmware   version, program update over WiFi (.bin from the Arduino IDE), restart; the module's (Teensy's)
//                  firmware too (2 Nov 2026): a .hex sent to the Teensy over the link, or its previous firmware
//                  put back, for a module whose own page cannot be reached
//     Changes (WiFi, update, restart) ask for the password of the XIAO's own WiFi network.
//   - tells the Teensy its version and WiFi state (shown in the XIAO card of the Teensy page) and
//     writes its events to the Teensy log ("[xiao] > ...")
//   - the WiFi bridge (22 Nov 2026; off from new, WiFi tab): a tablet with no network cable joins the XIAO's own
//     network and runs AgOpenWeb / AgOpenGPS there; the XIAO carries its datagrams to and from the Teensy
//   - the module's own page through the XIAO (Nov 2026): http://<its address on its own network>:8080/ (normally
//     http://192.168.5.1:8080/ with the bridge on) is the Teensy's page, carried over the link; with a password of one's own
//   - USB-C serial monitor (any baud), for the bench or the panel USB port: "status", "wifi-reset"
//     (WiFi settings back to the defaults, e.g. a forgotten password), "restart"
//
// Arduino IDE: board package "esp32" by Espressif Systems (3.x), board "XIAO_ESP32C3" or "XIAO_ESP32S3",
// Tools > USB CDC On Boot: Enabled, Partition Scheme: the default one (it has room for updates over
// WiFi). On the S3: USB Mode: Hardware CDC and JTAG; PSRAM not needed. No other libraries needed.
// The XIAO is powered by the board's 3.3 V: a USB-C cable to it must not carry 5 V (VBUS) while it
// sits on the board (see README).

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Arduino IDE: Tools > USB CDC On Boot > Enabled (D6/D7 are used for the Teensy)"
#endif

#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"                  // the devices on our own network (esp_wifi_ap_get_sta_list) ...
#include "esp_netif.h"                 // ... and the address our DHCP gave each one: the sign-in page
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_app_format.h"            // header of a program file (.bin): which chip it is for
#include "lwip/sockets.h"              // send() with MSG_DONTWAIT: answers that never hold up loop()
#include <errno.h>
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
static void linkPoll(uint32_t now);   // the link to the Teensy and the WiFi bridge (below): what loop() does all the time

// ---------------------------------------------------------------- the link task: it stands in for loop() (22 Nov 2026)
// The core's web server reads a whole request before it returns. When a phone on a weak WiFi sends half of one, or
// an upload pauses, handleClient() waits for the rest (up to 5 s for each piece of a form, UPLOAD_WAIT_MS for each
// piece of a file) and loop() waits with it: nobody said hello to the Teensy, read its data or carried the WiFi
// bridge, and a tablet steering through the bridge lost the module (found by Tests/XIAO_ESP32).
// So a second task runs linkPoll() while the loop is in there (linkTask, after the WiFi bridge below). One lock
// says whose turn it is:
//   - the loop holds it from setup() on and lets go of it only around the core's handleClient();
//   - every handler the core calls takes it back for as long as it runs (LinkWebServer::on, LinkTurn), so the
//     program's own code still runs one thing at a time, as before;
//   - the link task tries it every LINK_TASK_MS and never waits for it: it gets it only while the core is busy
//     with a browser, does one linkPoll() and gives it back. It must never wait for anything.
static SemaphoreHandle_t linkLock = nullptr;
static void stillHere();              // the loop's sign of life, for the watchdog (with the link task, below)
static const char* webNow = nullptr;  // the address our page is answering now (nullptr: none), for memWatch
static void memWatch(const char* where);   // a new lowest point of free memory: when, and what was going on (below)

// "The link is in my hands" for the loop and the page's handlers. The lock counts: who holds it may take it again.
struct LinkTurn
{
    LinkTurn()  { if (linkLock) xSemaphoreTakeRecursive(linkLock, portMAX_DELAY); }
    ~LinkTurn() { if (linkLock) xSemaphoreGiveRecursive(linkLock); }
};

// ---------------------------------------------------------------- web server that never holds up the Teensy link
// 13 Nov 2026. The core's WebServer writes an answer in one go and returns only when the browser has taken all of
// it. The page is 53 kB: on a weak WiFi link (Starlink, -72 dBm) that took about 10 s, and all that time loop()
// was away from the Teensy - no hello ("XIAO silent for 10 s" in the Teensy log), its data piling up unread.
// This one gives the connection what it takes now and, while the browser catches up, keeps the link to the
// Teensy going (the hello, its data), as the uploads already do. The Teensy page sends its page in pieces too.
// 22 Nov 2026. Reading a request is the core's too, and it waits there as well: the link task (above) takes over
// for that time. handleClient() hands the link over, and every handler takes it back (on, onNotFound).
#define SEND_WAIT_MS   20000         // a browser that takes nothing for this long is given up (as UPLOAD_WAIT_MS)
#define SPARE_IDLE_MS  1000          // a connection that asked nothing for this long gives way to the next one
class LinkWebServer : public WebServer
{
public:
    explicit LinkWebServer(int port) : WebServer(port) {}

    // Browsers open a spare connection in advance and may leave it silent. The core waits up to 5 s for it to
    // ask something (HTTP_MAX_DATA_WAIT) and serves no one else meanwhile: when another connection is waiting,
    // the silent one is dropped after SPARE_IDLE_MS (by the core's own clean-up). The browser opens a new one.
    void handleClient() override
    {
        if (_currentStatus == HC_WAIT_READ && millis() - _statusChange > SPARE_IDLE_MS && !_currentClient.available() && _server.hasClient())
            _statusChange = millis() - HTTP_MAX_DATA_WAIT - 1;
        if (linkLock) xSemaphoreGiveRecursive(linkLock);          // the core may wait for a browser in here: the link task's turn
        WebServer::handleClient();
        if (linkLock) xSemaphoreTakeRecursive(linkLock, portMAX_DELAY);
    }

    // These hide the core's own: every handler runs with the link back in the loop's hands, and counts as its sign of life
    void on(const char* uri, HTTPMethod method, THandlerFunction fn)
    {
        WebServer::on(uri, method, [fn, uri]() { LinkTurn mine; stillHere(); webNow = uri; fn(); memWatch(uri); webNow = nullptr; });
    }
    void on(const char* uri, HTTPMethod method, THandlerFunction fn, THandlerFunction upload)
    {
        WebServer::on(uri, method, [fn, uri]() { LinkTurn mine; stillHere(); webNow = uri; fn(); memWatch(uri); webNow = nullptr; },
                      [upload, uri]() { LinkTurn mine; stillHere(); webNow = uri; upload(); memWatch(uri); webNow = nullptr; });
    }
    void onNotFound(THandlerFunction fn)
    {
        WebServer::onNotFound([fn]() { LinkTurn mine; stillHere(); webNow = "(an unknown address)"; fn(); memWatch(webNow); webNow = nullptr; });
    }

protected:
    size_t _currentClientWrite(const char* b, size_t l) override { return writeKeepingLink(b, l); }
    size_t _currentClientWrite_P(PGM_P b, size_t l) override     { return writeKeepingLink(b, l); }

private:
    size_t writeKeepingLink(const char* b, size_t l)
    {
        LinkTurn mine;                                                      // a handler's answer: it has the link already
        const int fd = _currentClient.fd();
        if (fd < 0) return 0;
        size_t done = 0;
        uint32_t takenMs = millis();
        while (done < l)
        {
            const int r = ::send(fd, b + done, l - done, MSG_DONTWAIT);    // what the connection's buffer takes now (~5.7 kB)
            if (r > 0) { done += r; takenMs = millis(); continue; }
            if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) break;    // the browser has gone
            if (millis() - takenMs >= SEND_WAIT_MS) break;                  // it took nothing for 20 s: given up
            linkPoll(millis());                                             // meanwhile: the hello, the Teensy's data, the bridge
            stillHere();
            delay(1);                                                       // and the WiFi gets the core
        }
        return done;
    }
};

LinkWebServer             server(80);
Preferences               prefs;

// ---------------------------------------------------------------- settings (saved in the XIAO's flash)
struct Settings
{
    String apSsid, apPass;       // the XIAO's own network
    bool   staOn = false;        // also join another network
    String staSsid, staPass;
    String host;                 // name on the other network: http://<host>.local/
    bool   bridge = false;       // the WiFi bridge: a program on our own network reaches the module through us
    uint8_t modNet[3] = { 192, 168, 5 };   // the module's (Teensy's) first three numbers, as it last said them (modNetWatch):
                                           // with the bridge on, our own network has them too
} cfg;

static String defaultApSsid()
{
    const uint64_t mac = ESP.getEfuseMac();
    char b[16];
    snprintf(b, sizeof(b), "XIAO-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
    return String(b);
}

// The first three numbers of a network a module can have (as the page's "Module network" takes them)
static bool netNumbersOk(const uint8_t* n)
{
    return n[0] >= 1 && n[0] <= 223 && n[0] != 127;
}

// Our own network: 192.168.4.x, as every ESP32 has it; with the WiFi bridge on, the module's numbers (192.168.5.x
// by default): the sprayer or seed drill joins it with the subnet it has, and AgOpenGPS keeps its own (set in wifiStart)
static uint8_t apNet[3] = { 192, 168, 4 };
static char    apIpText[16] = "192.168.4.1";
static IPAddress apIp() { return IPAddress(apNet[0], apNet[1], apNet[2], 1); }
static bool onApNet(const uint8_t* p) { return p[0] == apNet[0] && p[1] == apNet[1] && p[2] == apNet[2]; }
static bool    staSame = false;        // the other network has our own network's numbers (bridge on): left until a restart (wifiWatch)

static void settingsLoad()
{
    prefs.begin("xiao", true);
    cfg.apSsid  = prefs.getString("apSsid", defaultApSsid());
    cfg.apPass  = prefs.getString("apPass", DEFAULT_AP_PASS);
    cfg.staOn   = prefs.getBool("staOn", false);
    cfg.staSsid = prefs.getString("staSsid", "");
    cfg.staPass = prefs.getString("staPass", "");
    cfg.host    = prefs.getString("host", DEFAULT_HOST);
    cfg.bridge  = prefs.getBool("bridge", false);
    unsigned a, b, c;
    const String mn = prefs.getString("modNet", "");
    if (sscanf(mn.c_str(), "%u.%u.%u", &a, &b, &c) == 3 && a <= 255 && b <= 255 && c <= 255)
    {
        const uint8_t n[3] = { (uint8_t)a, (uint8_t)b, (uint8_t)c };
        if (netNumbersOk(n)) memcpy(cfg.modNet, n, 3);
    }
    prefs.end();
    if (cfg.apSsid.length() < 1 || cfg.apSsid.length() > 32) cfg.apSsid = defaultApSsid();
    if (cfg.apPass.length() < 8 || cfg.apPass.length() > 63) cfg.apPass = DEFAULT_AP_PASS;
    if (cfg.host.length() < 1 || cfg.host.length() > 24)    cfg.host = DEFAULT_HOST;
    if (cfg.apPass == DEFAULT_AP_PASS) cfg.bridge = false;      // never with the password every XIAO starts with
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
    prefs.putBool("bridge", cfg.bridge);
    char mn[16];
    snprintf(mn, sizeof(mn), "%u.%u.%u", cfg.modNet[0], cfg.modNet[1], cfg.modNet[2]);
    prefs.putString("modNet", mn);
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
    void num(const char* k, double v, int d) { key(k); if (isnan(v) || isinf(v)) { raw("null"); return; } char t[32]; snprintf(t, sizeof(t), "%.*f", d, v); raw(t); }
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

// ---------------------------------------------------------------- the WiFi bridge (22 Nov 2026)
// For a tablet with no network cable (no USB-Ethernet adapter): it joins the XIAO's own network and runs
// AgOpenWeb / AgOpenGPS there. The program's datagrams for the module (UDP 8888, and its corrections on UDP 2233)
// are taken here and handed to the Teensy over the link (PGN 146, 147); what the Teensy has for the program comes
// back (PGN 148) and goes to that program's UDP 9999 - to it only, never to everyone: a broadcast waits for the
// WiFi's beacon (on a router's WiFi the module's broadcasts came 1.5 s late, half of them lost; a tractor,
// 22 Nov 2026). The rules:
//   - off from new; switched on in the WiFi tab, and only with a password of one's own on the XIAO's network,
//     because whoever is on this network can then steer the tractor.
//   - only from the XIAO's own network, never from the other network the XIAO may have joined. With the bridge on
//     our own network has the module's numbers (192.168.5.x by default, apNet): the program keeps its subnet, and a
//     sprayer or seed drill module joins it as it is. An other network with those numbers too is left (wifiWatch).
//   - one program at a time: the first one heard, until it has been silent for BRIDGE_PROG_GONE_MS.
//   - the Teensy decides who has the module (xLink.h, bridgeState): a program on its network cable comes first,
//     and then only our program's hello is passed on (so the Teensy's log can say that it is calling).
//   - nothing is passed to a Teensy that does not say it knows the bridge (PGN 149): an older firmware would
//     send these frames on to UDP 9999.
//   - while the program has the module the XIAO does not search for the other network, and the page does not
//     search for networks: a search takes the radio off our own channel, and the steering data with it.
//   - with autosteer on over the bridge, the page does not restart the XIAO (settings, update, restart).
// The other modules (22 Nov 2026): a section-control module (sprayer, seed drill) on a tractor with no router joins
// the XIAO's network too. Every packet of the program also goes to each of them, and what they send goes to the
// program - to their own addresses, as above, never to everyone. A module is known from its own packets (it sends
// one every 200 ms, program or not, to UDP 9999) and forgotten BRIDGE_MOD_GONE_MS after the last. Not passed on to
// them: the subnet (PGN 201), which would take a module off this network, the scan (PGN 202), and what is only for
// the steering (PGN 254 a hundred times a second, 252, 251). Of a module's own packets BRIDGE_MOD_PER_S a second
// go on at most: one gone wrong must not fill the WiFi that carries the steering.
#define BRIDGE_PORT_IN       8888    // the program -> the module
#define BRIDGE_PORT_RTCM     2233    // the program's corrections (NTRIP) -> the receiver
#define BRIDGE_PORT_OUT      9999    // the module -> the program
#define BRIDGE_PROG_GONE_MS  3000
#define BRIDGE_MODULES       4       // other modules on our network, at most
#define BRIDGE_MOD_GONE_MS   3000
#define BRIDGE_MOD_PER_S     60      // (ours send about 8 a second)
#define BRIDGE_MOD_LINK_SPARE 128    // a module's packet goes to the Teensy (PGN 152) only with this much more room on the link
#define AP_STATIONS          8       // devices on our network at most (the core's own number is 4): tablet, modules, phones
#define BRIDGE_PER_LOOP      16      // datagrams taken per loop pass at most
#define BRIDGE_TX_SPARE      64      // room asked for in the link's transmit buffer beside the frame itself: the
                                     // serial driver keeps a few bytes of its own with every write, and a write
                                     // with too little room waits
#define BRIDGE_STA_BACK_MS   10000   // the search for the other network starts again this long after the program left

struct BridgeModule
{
    uint32_t ip = 0;                 // its address (first number in the low byte), 0 = this place is free
    uint32_t ms = 0;                 // its last datagram
    uint32_t toIt = 0, fromIt = 0;   // datagrams of the program sent to it / of its own passed to the program
    uint32_t secMs = 0;              // the second being counted ...
    uint16_t secN = 0;               // ... and how many of its datagrams came in it
    bool     overTold = false;       // Messages said that it sends too many
};

struct Bridge
{
    int      sIn = -1, sRtcm = -1;   // the sockets (-1: the bridge is off)
    int      sMod = -1;              // ... and the one the other modules send to
    BridgeModule mod[BRIDGE_MODULES];
    uint32_t modDrop = 0;            // to or from a module: the WiFi did not take it
    uint32_t modFull = 0;            // from one module more than there are places
    uint32_t modOver = 0;            // from a module that sends more than BRIDGE_MOD_PER_S a second
    uint32_t modTeensy = 0;          // a module's packets passed to the Teensy for its Machine tab (PGN 152)
    uint32_t modLink = 0;            // ... not passed: no room on the link
    uint32_t progIp = 0;             // the program (as in the datagram: first number in the low byte), 0 = none
    uint32_t progMs = 0;             // its last datagram
    uint32_t steerMs = 0;            // its last PGN 254 with autosteer on (0 = never)
    uint32_t busyMs = 0;             // the last time the program had the module (0 = never)
    uint32_t in = 0, out = 0;        // datagrams passed to the Teensy / sent to the program
    uint32_t rtcm = 0;               // bytes of corrections passed on
    uint32_t dropLink = 0;           // not passed on: no room on the link to the Teensy
    uint32_t dropBig = 0;            // not passed on: longer than a frame carries
    uint32_t dropOut = 0;            // not sent to the program: the WiFi did not take it
    uint32_t other = 0;              // from another program while one has the bridge
    bool     staPaused = false;      // the search for the other network is stopped (bridgeStaWatch)
} bridge;

// Datagrams the bridge did not pass on, all reasons together (the page's "lost")
static uint32_t bridgeLost() { return bridge.dropLink + bridge.dropBig + bridge.dropOut + bridge.modDrop + bridge.modOver; }

static void ipText(char* b, size_t cap, uint32_t ip)
{
    const uint8_t* p = (const uint8_t*)&ip;
    snprintf(b, cap, "%u.%u.%u.%u", p[0], p[1], p[2], p[3]);
}

// the program has the module (the Teensy says so) and is still sending
static bool bridgeBusy(uint32_t now)
{
    return bridge.progIp != 0 && teensy.bridgeState == LINK_BRIDGE_WIFI && teensy.bridgeKnown(now);
}

// ... and asked for autosteer in the last 2 s
static bool bridgeSteering(uint32_t now)
{
    return bridgeBusy(now) && bridge.steerMs != 0 && now - bridge.steerMs < 2000;
}

static int bridgeSocket(uint16_t port)
{
    const int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return -1;
    const int one = 1;
    lwip_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    lwip_setsockopt(s, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));   // the program sends to everyone on the network
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) { lwip_close(s); return -1; }
    return s;
}

// A datagram for the program from the Teensy (PGN 148, called by teensy.poll): to that program only
static void bridgeNetOut(const uint8_t* d, uint8_t n)
{
    if (bridge.sIn < 0 || bridge.progIp == 0) return;
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(BRIDGE_PORT_OUT);
    to.sin_addr.s_addr = bridge.progIp;
    if (lwip_sendto(bridge.sIn, d, n, MSG_DONTWAIT, (struct sockaddr*)&to, sizeof(to)) == (int)n) bridge.out++;
    else bridge.dropOut++;
}

// Is this datagram's sender the program we carry? The first one heard on our own network, until it goes silent.
static bool bridgeTakes(uint32_t ip, uint32_t now)
{
    if (!onApNet((const uint8_t*)&ip)) return false;                  // our own network only (apNet, wifiStart)
    if (!teensy.bridgeKnown(now)) return false;                       // the Teensy does not know the bridge, or is silent
    if (bridge.progIp == 0)
    {
        bridge.progIp = ip;
        bridge.steerMs = 0;
        char t[20];
        ipText(t, sizeof(t), ip);
        logEvent("WiFi bridge: a program at %s is calling the module", t);
    }
    else if (ip != bridge.progIp) { bridge.other++; return false; }
    bridge.progMs = msStamp(now);
    return true;
}

// ---- the other modules on our network
static int bridgeModules()
{
    int n = 0;
    for (const BridgeModule& m : bridge.mod) n += m.ip != 0;
    return n;
}

// The module at this address: the one known, or a new one while there is a place
static BridgeModule* bridgeModule(uint32_t ip, uint32_t now)
{
    BridgeModule* spare = nullptr;
    for (BridgeModule& m : bridge.mod)
    {
        if (m.ip == ip) { m.ms = now; return &m; }
        if (m.ip == 0 && !spare) spare = &m;
    }
    char t[20];
    ipText(t, sizeof(t), ip);
    if (!spare)
    {
        if (bridge.modFull++ == 0) logEvent("WiFi bridge: more than %d modules on this network - the one at %s is NOT served", BRIDGE_MODULES, t);
        return nullptr;
    }
    *spare = BridgeModule();
    spare->ip = ip;
    spare->ms = now;
    logEvent("WiFi bridge: a module at %s is on this network (the program's packets go to it too)", t);
    return spare;
}

// One datagram of the program to every module known
static void bridgeToModules(const uint8_t* d, int n)
{
    const uint8_t pgn = d[3];
    if (pgn == 201 || pgn == 202) return;                             // the subnet and the scan are not for them here
    if (pgn == 254 || pgn == 252 || pgn == 251) return;               // the steering's own: nothing in them for a machine module
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(BRIDGE_PORT_IN);
    for (BridgeModule& m : bridge.mod)
    {
        if (m.ip == 0) continue;
        to.sin_addr.s_addr = m.ip;
        if (lwip_sendto(bridge.sIn, d, n, MSG_DONTWAIT, (struct sockaddr*)&to, sizeof(to)) == n) m.toIt++;
        else bridge.modDrop++;
    }
}

// A module's own packet to the Teensy too (PGN 152), for its Machine tab: the module's page shows the sprayer or seed
// drill on our WiFi as it does one on its network cable, with a program or without. Only what that tab reads (the hello
// reply, PGN 237, the sprayer's 160 and 161), only to a Teensy that knows PGN 152, and never when the link has no room:
// the steering's frames come first.
static void bridgeModToTeensy(uint32_t ip, const uint8_t* d, int n, uint32_t now)
{
    const uint8_t pgn = d[3];
    if (pgn != 123 && pgn != 237 && pgn != 160 && pgn != 161) return;
    if (n > LINK_NET_MAX || !teensy.machKnown(now)) return;
    if (TeensyPort.availableForWrite() < n + 4 + 6 + BRIDGE_TX_SPARE + BRIDGE_MOD_LINK_SPARE) { bridge.modLink++; return; }
    static uint8_t f[4 + LINK_NET_MAX];
    memcpy(f, &ip, 4);
    memcpy(f + 4, d, n);
    teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_MACH_IN, f, (uint8_t)(4 + n));
    bridge.modTeensy++;
}

// What the modules sent: the sender is noted, and with a program there the datagram goes on to it
static void bridgeFromModules(uint32_t now)
{
    for (BridgeModule& m : bridge.mod)
    {
        if (m.ip == 0 || now - m.ms < BRIDGE_MOD_GONE_MS) continue;
        char t[20];
        ipText(t, sizeof(t), m.ip);
        logEvent("WiFi bridge: the module at %s is silent (%lu packets to it, %lu from it)", t, (unsigned long)m.toIt, (unsigned long)m.fromIt);
        m.ip = 0;
    }
    static uint8_t b[1472];
    struct sockaddr_in from, to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(BRIDGE_PORT_OUT);
    for (int k = 0; k < BRIDGE_PER_LOOP; k++)
    {
        socklen_t fl = sizeof(from);
        const int n = lwip_recvfrom(bridge.sMod, b, sizeof(b), MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
        if (n <= 0) break;
        const uint32_t ip = from.sin_addr.s_addr;
        const uint8_t* p = (const uint8_t*)&ip;
        if (n < 6 || b[0] != 0x80 || b[1] != 0x81 || b[2] == 0x7F) continue;   // AgOpenGPS packets only, and none of a program's
        if (!onApNet(p) || p[3] < 2 || p[3] > 254) continue;                    // from our own network only, and not from us
        if (ip == bridge.progIp) continue;                            // the program is no module
        BridgeModule* m = bridgeModule(ip, now);
        if (!m) continue;
        if (now - m->secMs >= 1000) { m->secMs = now; m->secN = 0; }
        if (m->secN >= BRIDGE_MOD_PER_S)
        {
            bridge.modOver++;
            if (!m->overTold)
            {
                m->overTold = true;
                char t[20];
                ipText(t, sizeof(t), ip);
                logEvent("WiFi bridge: the module at %s sends more than %d packets a second - the rest is NOT passed on", t, BRIDGE_MOD_PER_S);
            }
            continue;
        }
        m->secN++;
        bridgeModToTeensy(ip, b, n, now);
        if (bridge.progIp == 0) continue;
        to.sin_addr.s_addr = bridge.progIp;
        if (lwip_sendto(bridge.sMod, b, n, MSG_DONTWAIT, (struct sockaddr*)&to, sizeof(to)) == n) m->fromIt++;
        else bridge.modDrop++;
    }
}

static void bridgePoll(uint32_t now)
{
    if (bridge.sIn < 0) return;
    if (bridge.progIp != 0 && now - bridge.progMs >= BRIDGE_PROG_GONE_MS)
    {
        char t[20];
        ipText(t, sizeof(t), bridge.progIp);
        logEvent("WiFi bridge: the program at %s is silent (%lu packets to the module, %lu to it)", t, (unsigned long)bridge.in, (unsigned long)bridge.out);
        bridge.progIp = 0;
    }
    if (bridgeBusy(now)) bridge.busyMs = msStamp(now);
    bridgeFromModules(now);

    static uint8_t b[4 + 1472];                                       // [the sender's IP][one datagram]
    struct sockaddr_in from;
    for (int k = 0; k < BRIDGE_PER_LOOP; k++)                         // the program's packets for the module
    {
        socklen_t fl = sizeof(from);
        const int n = lwip_recvfrom(bridge.sIn, b + 4, sizeof(b) - 4, MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
        if (n <= 0) break;
        const uint32_t ip = from.sin_addr.s_addr;
        if (n < 6 || b[4] != 0x80 || b[5] != 0x81) continue;          // AgOpenGPS packets only
        if (!bridgeTakes(ip, now)) continue;
        bridgeToModules(b + 4, n);                                    // the other modules on our network get it too
        if (n > LINK_NET_MAX) { bridge.dropBig++; continue; }
        if (b[4 + 3] == 254 && n >= 8 && (b[4 + 7] & 1)) bridge.steerMs = msStamp(now);
        if (teensy.bridgeState == LINK_BRIDGE_CABLE && b[4 + 3] != 200) continue;   // the cable has the module: only our hello
        if (TeensyPort.availableForWrite() < n + 4 + 6 + BRIDGE_TX_SPARE) { bridge.dropLink++; continue; }   // never waits for the link
        memcpy(b, &ip, 4);
        teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_NET_IN, b, (uint8_t)(4 + n));
        bridge.in++;
        if (n >= 6 + b[4 + 4]) teensy.aogPacket(b[4 + 3], b + 4 + 5, b[4 + 4], now);   // for the page's "Program -> module" card
    }
    for (int k = 0; k < 4; k++)                                       // its corrections, in frames of up to 255 bytes
    {
        socklen_t fl = sizeof(from);
        const int n = lwip_recvfrom(bridge.sRtcm, b, sizeof(b), MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
        if (n <= 0) break;
        if (from.sin_addr.s_addr != bridge.progIp || !bridgeBusy(now)) continue;   // only from the program that has the module
        for (int at = 0; at < n; at += 255)
        {
            const int part = n - at < 255 ? n - at : 255;
            if (TeensyPort.availableForWrite() < part + 6 + BRIDGE_TX_SPARE) { bridge.dropLink++; break; }
            teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_RTCM, b + at, (uint8_t)part);
            bridge.rtcm += part;
        }
    }
}

// ---------------------------------------------------------------- the module's (Teensy's) page through the XIAO (Nov 2026)
// The module's own page (http://<its IP>/) is on its network cable only. With the WiFi bridge the tablet has no cable,
// and nobody on our WiFi could open the module's settings at all. So we listen on port 8080 of our own network, pass what
// a browser sends there on to the Teensy over the link (PGN 150) and its answer back to the browser (PGN 151):
// http://<our address>:8080/ is the module's own page, every tab of it, the same as on the cable. The rules:
//   - our own network only, and only with a password of one's own on it (as the bridge): whoever opens this page can
//     change how the tractor steers.
//   - one request at a time, as the module serves them: a page asks for several things at once, they wait their turn.
//   - a firmware update for the module, or going back to its previous one, is not passed on: our Firmware tab does both,
//     with its checks (and the module refuses them from here too).
//   - nothing goes to a Teensy that does not say it knows this (PGN 149, version 2): an older firmware would send the
//     frames on to UDP 9999. We answer the browser ourselves then, in words, as for every refusal.
//   - the answer comes as the link has room and waits here until the browser takes it: TPAGE_WINDOW bytes at most, which
//     the Teensy never goes past (we tell it what the browser took). A slow phone never holds up the link, and a tablet
//     steering through the bridge comes first on it both ways (TPAGE_LINK_SPARE here, the Teensy's xiaoSendOwn there).
//   - nothing here waits: it runs in linkPoll(), from the loop and from the link task.
#define TPAGE_PORT        8080
#define TPAGE_CONNS       6          // browsers' connections held at once (a page opens a few)
#define TPAGE_WINDOW      6144       // answer bytes held here for the browser
#define TPAGE_REQ_MAX     4608       // the most of one request passed on (the Teensy takes no more: XWEB_RX_MAX)
#define TPAGE_PIECE       240        // request bytes per PGN 150; the first line is read before anything goes (one piece)
#define TPAGE_ACK_BYTES   1024       // the Teensy hears what the browser took every this many bytes
#define TPAGE_LINK_SPARE  128        // room on the link left to the bridge's datagrams (beside BRIDGE_TX_SPARE)
#define TPAGE_HEAP_LOW    32768      // free memory under this: the page waits (no new browser, no answer bytes sent on)
#define TPAGE_ASK_MS      10000      // a connection that asked nothing for this long is closed (a spare one)
#define TPAGE_STILL_MS    15000      // nothing moved for this long (the Teensy sent nothing, the browser took nothing): given up

// "since" is at least ms ago. Signed: a time stamp taken in this pass can be a millisecond later than the pass's own
// time (firmware CLAUDE.md, gotcha 15), and the unsigned difference would read four billion.
static inline bool tpageAfter(uint32_t now, uint32_t since, uint32_t ms) { return (int32_t)(now - since) >= (int32_t)ms; }

struct TPageConn { NetworkClient c; uint32_t ms = 0; bool used = false; };
struct TPage
{
    TPageConn conn[TPAGE_CONNS];
    int      cur = -1;               // the connection being served (-1: none)
    uint8_t  sid = 0;                // its session number (the Teensy's answers carry it)
    bool     open = false;           // the Teensy has it ("open" sent)
    bool     ended = false;          // the whole answer is here: the Teensy's "end", or an answer of our own
    bool     lost = false;           // a piece of the answer went missing on the link
    char     line[TPAGE_PIECE];      // the request's first line (or its first TPAGE_PIECE bytes)
    uint16_t lineLen = 0;
    bool     lineDone = false, lineSent = false;
    uint32_t reqSent = 0;            // request bytes passed to the Teensy
    uint8_t  ring[TPAGE_WINDOW];     // the answer, waiting for the browser
    uint16_t head = 0, len = 0;
    uint32_t got = 0, passed = 0, acked = 0;
    uint32_t startMs = 0, moveMs = 0;
    uint32_t drainMs = 0;            // the last time the rest of a refused request was read and dropped (0: none)
    uint32_t req = 0, refused = 0, broken = 0;   // for the page: requests passed on, answered here, broken off
    uint32_t lowMem = 0;             // ... and the times it waited for memory
    uint32_t lowMs = 0;              // the last of them began (0: not waiting now)
    uint32_t lowToldMs = 0;          // Messages said so (once a minute at most)
} tpage;
static NetworkServer tpageServer(TPAGE_PORT);
static bool          tpageOn = false;

static bool tpageRoom(int n) { return TeensyPort.availableForWrite() >= n + 6 + BRIDGE_TX_SPARE + TPAGE_LINK_SPARE; }

// Why the module's page cannot be passed on now (nullptr: it can), in a few words for our page and the browser
static const char* tpageNot(uint32_t now)
{
    if (cfg.apPass == DEFAULT_AP_PASS) return "this WiFi still has the password it came with: give it one of your own first (WiFi tab), because whoever opens the module's page can change how the tractor steers";
    if (!teensy.teensyTalking(now))    return "the module (Teensy) does not answer on the link: is it on, with its GPS2 port set to the XIAO?";
    if (!teensy.pageKnown(now))        return "the module's firmware is older than this XIAO program: install its newer firmware (Firmware tab, Module (Teensy) firmware), or open its page on the network cable";
    return nullptr;
}

// An answer of our own instead of the module's: a short page, or for the page's own requests (/api/...) the error as
// the module gives it, {"ok":false,"error":"..."}
static void tpageOwn(int code, const char* why)
{
    const bool api = !strncmp(tpage.line, "GET /api/", 9) || !strncmp(tpage.line, "POST /api/", 10);
    char body[900];
    int bn = api ? snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", why)
                 : snprintf(body, sizeof(body), "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                            "<title>Module page</title></head><body style=\"font-family:sans-serif;max-width:40em;margin:2em auto;padding:0 1em\">"
                            "<h2>The module's page through the XIAO</h2><p>Not now: %s.</p><p><a href=\"http://%s/\">The XIAO page</a></p></body></html>", why, apIpText);
    if (bn < 0) bn = 0;
    if (bn >= (int)sizeof(body)) bn = sizeof(body) - 1;
    const int hn = snprintf((char*)tpage.ring, 200, "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
                            code, code == 403 ? "Forbidden" : code == 503 ? "Service Unavailable" : "Gateway Timeout",
                            api ? "application/json" : "text/html; charset=utf-8", bn);
    memcpy(tpage.ring + hn, body, bn);
    tpage.head = 0;
    tpage.len = (uint16_t)(hn + bn);
    tpage.open = false;
    tpage.ended = true;
    tpage.refused++;
}

// The session is over: the browser's connection closed (what it sent and we did not read is read first: closing with
// it unread would reset the connection and the browser could lose the end of the answer), the Teensy told when the
// browser went before the end
static void tpageFinish(bool tellTeensy, uint32_t now)
{
    if (tpage.cur < 0) return;
    if (tellTeensy && tpage.open && !tpage.ended && teensy.pageKnown(now) && tpageRoom(2))
    {
        const uint8_t d[2] = { 3, tpage.sid };
        teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_WEB_IN, d, 2);
    }
    NetworkClient& c = tpage.conn[tpage.cur].c;
    uint8_t junk[64];
    for (int k = 0; k < 16 && c.available() > 0; k++) if (c.read(junk, sizeof(junk)) <= 0) break;
    c.stop();
    tpage.conn[tpage.cur].used = false;
    tpage.cur = -1;
    tpage.open = tpage.ended = tpage.lost = false;
    tpage.len = 0;
}

// PGN 151 from the Teensy (called by teensy.poll): the answer's pieces into the ring, and its end
static void tpageFromTeensy(const uint8_t* d, uint8_t n, uint32_t now)
{
    if (tpage.cur < 0 || !tpage.open || tpage.ended || d[1] != tpage.sid) return;   // an older request's: done with
    if (d[0] == 3) { tpage.ended = true; tpage.moveMs = msStamp(now); return; }
    if (d[0] != 2 || n < 4) return;
    const uint16_t off = (uint16_t)(d[2] | d[3] << 8), k = n - 4;
    if (off != (uint16_t)tpage.got || tpage.len + k > TPAGE_WINDOW) { tpage.lost = true; return; }
    for (uint16_t i = 0; i < k; i++) tpage.ring[(tpage.head + tpage.len + i) % TPAGE_WINDOW] = d[4 + i];
    tpage.len += k;
    tpage.got += k;
    tpage.moveMs = msStamp(now);
}

// One piece of the request to the Teensy
static void tpagePiece(const uint8_t* b, int n)
{
    uint8_t d[4 + TPAGE_PIECE];
    d[0] = 2;
    d[1] = tpage.sid;
    d[2] = (uint8_t)tpage.reqSent;
    d[3] = (uint8_t)(tpage.reqSent >> 8);
    memcpy(d + 4, b, n);
    teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_WEB_IN, d, (uint8_t)(4 + n));
    tpage.reqSent += n;
}

// The first line is in: passed on to the Teensy, or answered here
static void tpageBegin(uint32_t now)
{
    NetworkClient& c = tpage.conn[tpage.cur].c;
    const char* why = nullptr;
    int code = 403;
    if (!(c.localIP() == WiFi.softAPIP())) why = "it opens only from the XIAO's own WiFi, at its address on that network, port 8080";
    else if (!strncmp(tpage.line, "POST /api/ota", 13) || !strncmp(tpage.line, "POST /api/revert", 16))
        why = "the module's firmware is installed, or put back, on the XIAO page (Firmware tab, Module (Teensy) firmware), not through here";
    else if ((why = tpageNot(now)) != nullptr) code = cfg.apPass == DEFAULT_AP_PASS ? 403 : 503;
    if (why) { tpageOwn(code, why); return; }
    if (!tpageRoom(4)) return;                                       // no room on the link now: the next pass
    tpage.sid++;
    const uint8_t d[4] = { 1, tpage.sid, (uint8_t)TPAGE_WINDOW, (uint8_t)(TPAGE_WINDOW >> 8) };
    teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_WEB_IN, d, 4);
    tpage.open = true;
    tpage.reqSent = tpage.got = tpage.passed = tpage.acked = 0;
    tpage.moveMs = msStamp(now);
    tpage.req++;
}

static void tpagePoll(uint32_t now)
{
    if (!tpageOn) return;
    // ---- short of memory: wait. Each answer byte sent on to the browser takes memory until the browser has it, and
    // a new browser takes some too; on a busy WiFi (the steering's packets, broadcasts held for the next beacon) the
    // XIAO's free memory fell to 2 kB on a C3 while this page loaded (bench, 22 Nov 2026). Out of memory the XIAO
    // restarts, and the bridge with it. The Teensy waits too: it never sends more than the window ahead.
    const uint32_t heap = ESP.getFreeHeap();
    if (heap < TPAGE_HEAP_LOW)
    {
        if (tpage.lowMs == 0)
        {
            tpage.lowMs = msStamp(now);
            tpage.lowMem++;
            if (tpage.lowToldMs == 0 || tpageAfter(now, tpage.lowToldMs, 60000))
            {
                tpage.lowToldMs = msStamp(now);
                logEvent("Module page (port 8080): waits, the XIAO is short of memory (%lu bytes free)", (unsigned long)heap);
            }
        }
        return;                                                      // (a browser kept waiting 15 s is given up as before)
    }
    tpage.lowMs = 0;
    // ---- new connections: a free place, or the oldest one that asked nothing gives way to it
    while (tpageServer.hasClient())
    {
        int at = -1, idle = -1;
        for (int i = 0; i < TPAGE_CONNS; i++)
        {
            if (!tpage.conn[i].used) { at = i; break; }
            if (i != tpage.cur && tpage.conn[i].c.available() <= 0 && (idle < 0 || (int32_t)(tpage.conn[i].ms - tpage.conn[idle].ms) < 0)) idle = i;
        }
        if (at < 0 && idle < 0) break;                               // each one has a request waiting: the new one waits too
        if (at < 0) { tpage.conn[idle].c.stop(); tpage.conn[idle].used = false; at = idle; }
        tpage.conn[at].c = tpageServer.accept();
        if (!tpage.conn[at].c) break;
        tpage.conn[at].used = true;
        tpage.conn[at].ms = msStamp(now);
    }
    // ---- the ones waiting for their turn: gone, or silent for too long
    for (int i = 0; i < TPAGE_CONNS; i++)
    {
        TPageConn& w = tpage.conn[i];
        if (!w.used || i == tpage.cur) continue;
        if (!w.c.connected() || (w.c.available() <= 0 && tpageAfter(now, w.ms, TPAGE_ASK_MS))) { w.c.stop(); w.used = false; }
    }
    // ---- the next one with a request, the oldest first
    if (tpage.cur < 0)
    {
        for (int i = 0; i < TPAGE_CONNS; i++)
            if (tpage.conn[i].used && tpage.conn[i].c.available() > 0 && (tpage.cur < 0 || (int32_t)(tpage.conn[i].ms - tpage.conn[tpage.cur].ms) < 0)) tpage.cur = i;
        if (tpage.cur < 0) return;
        tpage.lineLen = 0;
        tpage.lineDone = tpage.lineSent = false;
        tpage.open = tpage.ended = tpage.lost = false;
        tpage.head = tpage.len = 0;
        tpage.drainMs = 0;
        tpage.startMs = msStamp(now);
    }
    NetworkClient& c = tpage.conn[tpage.cur].c;
    // ---- its first line, then the Teensy's (or our own answer)
    if (!tpage.lineDone)
    {
        while (tpage.lineLen < TPAGE_PIECE && c.available() > 0)
        {
            const int b = c.read();
            if (b < 0) break;
            tpage.line[tpage.lineLen++] = (char)b;
            if (b == '\n') break;
        }
        tpage.lineDone = tpage.lineLen == TPAGE_PIECE || (tpage.lineLen > 0 && tpage.line[tpage.lineLen - 1] == '\n');
        if (!tpage.lineDone)
        {
            if (!c.connected() || tpageAfter(now, tpage.startMs, TPAGE_ASK_MS)) tpageFinish(false, now);
            return;
        }
    }
    if (!tpage.open && !tpage.ended) { tpageBegin(now); if (!tpage.open && !tpage.ended) return; }
    // ---- the rest of the request, as the link has room (more than the Teensy takes is read and dropped)
    if (tpage.open && !tpage.ended && !tpage.lost)
    {
        if (!teensy.pageKnown(now) || !teensy.teensyTalking(now))  // the Teensy went (a restart, the wires): nobody answers
        {
            if (tpage.got == 0) { tpageOwn(503, "the module (Teensy) stopped answering on the link"); }
            else { tpage.broken++; tpageFinish(false, now); return; }
        }
        else
        {
            if (!tpage.lineSent && tpageRoom(4 + tpage.lineLen)) { tpagePiece((const uint8_t*)tpage.line, tpage.lineLen); tpage.lineSent = true; }
            for (int k = 0; k < 4 && tpage.lineSent && c.available() > 0; k++)
            {
                uint8_t b[TPAGE_PIECE];
                if (tpage.reqSent >= TPAGE_REQ_MAX) { c.read(b, sizeof(b)); continue; }
                int n = c.available();
                if (n > TPAGE_PIECE) n = TPAGE_PIECE;
                if (n > (int)(TPAGE_REQ_MAX - tpage.reqSent)) n = TPAGE_REQ_MAX - tpage.reqSent;
                const int room = TeensyPort.availableForWrite() - (4 + 6 + BRIDGE_TX_SPARE + TPAGE_LINK_SPARE);
                if (room < 32 && room < n) break;                    // (one write takes at most half the buffer: ~500 bytes)
                if (n > room) n = room;
                n = c.read(b, n);
                if (n <= 0) break;
                tpagePiece(b, n);
                tpage.moveMs = msStamp(now);
            }
        }
    }
    else if (tpage.ended)                                            // answered here: the rest of the request (an upload) is read and dropped,
    {                                                                // or closing would reset the connection and lose our answer
        uint8_t junk[256];
        for (int k = 0; k < 16 && c.available() > 0; k++) if (c.read(junk, sizeof(junk)) > 0) tpage.drainMs = msStamp(now);
    }
    if (tpage.lost) { tpage.broken++; tpageFinish(true, now); return; }   // the browser gets a cut answer and asks again
    // ---- the answer, as the browser takes it
    const int fd = c.fd();
    while (tpage.len > 0)
    {
        uint16_t k = tpage.len;
        if (k > TPAGE_WINDOW - tpage.head) k = TPAGE_WINDOW - tpage.head;
        const int r = fd < 0 ? -1 : ::send(fd, tpage.ring + tpage.head, k, MSG_DONTWAIT);
        if (r > 0)
        {
            tpage.head = (uint16_t)((tpage.head + r) % TPAGE_WINDOW);
            tpage.len -= r;
            tpage.passed += r;
            tpage.moveMs = msStamp(now);
            continue;
        }
        if (fd < 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) { if (!tpage.ended) tpage.broken++; tpageFinish(true, now); return; }   // the browser went
        break;
    }
    if (tpage.open && !tpage.ended && tpage.passed - tpage.acked >= TPAGE_ACK_BYTES && tpageRoom(4))
    {
        const uint8_t d[4] = { 4, tpage.sid, (uint8_t)tpage.passed, (uint8_t)(tpage.passed >> 8) };
        teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_WEB_IN, d, 4);
        tpage.acked = tpage.passed;
    }
    if (tpage.ended && tpage.len == 0 && (tpage.drainMs == 0 || (c.available() <= 0 && tpageAfter(now, tpage.drainMs, 300))))
    { tpageFinish(false, now); return; }                             // all of it with the browser: closed, as the answer says
    if (!c.connected() && tpage.len == 0) { if (tpage.open) tpage.broken++; tpageFinish(true, now); return; }   // the browser went
    if (tpageAfter(now, tpage.moveMs, TPAGE_STILL_MS))                        // nothing moved for 15 s: given up
    {
        if (tpage.open && tpage.passed == 0 && tpage.len == 0)
        {
            if (tpageRoom(2)) { const uint8_t d[2] = { 3, tpage.sid }; teensy.sendFrame(LINK_SRC_XIAO, LINK_PGN_WEB_IN, d, 2); }
            tpageOwn(504, "the module (Teensy) did not answer in 15 s");
            tpage.moveMs = msStamp(now);
        }
        else { tpage.broken++; tpageFinish(true, now); }
    }
}

static void tpageStart()
{
    teensy.onWeb = tpageFromTeensy;
    tpageServer.begin();
    tpageOn = true;
}

// ---------------------------------------------------------------- short of memory: when, and what was going on
// 24 Nov 2026, bench: the C3's lowest free memory since its start (heapMin) was 5.5 kB, three hours after a restart,
// and nothing said when, nor what the XIAO was doing then (the module's page had waited for memory once). Out of
// memory the XIAO restarts, and the WiFi bridge with it. So a new lowest point under MEM_TELL is noted with what was
// going on: on our page (/api/state "memLow": the lowest of them) and in Messages (again only MEM_STEP lower than the
// last one they gave, so a slow slide does not fill them). The allocator keeps the lowest point itself, whichever task
// took the memory (the WiFi's, the network's), so it is noticed at the next look: the end of every link pass (a few
// milliseconds) and of every request to our page.
#define MEM_TELL   32768      // a lowest point under this is noted (= TPAGE_HEAP_LOW)
#define MEM_STEP   4096       // Messages again only this much lower than the last one they gave
#define MEM_WHAT   320      // room for every piece of what was going on at its longest (about 318)

static const char* staState();        // (below, with the WiFi)

struct MemLow
{
    uint32_t seen = 0;                       // the lowest point seen so far (0: not looked yet)
    uint32_t told = 0;                       // the last one Messages gave (0: none)
    uint32_t lowest = 0;                     // the lowest point under MEM_TELL (0: none yet) ...
    uint32_t ms = 0, freeThen = 0, block = 0;   // ... when it was noticed, the free memory then, its largest piece then
    char     what[MEM_WHAT] = "";            // ... and what was going on
    uint32_t secMs = 0;                      // the bridge's datagrams a second, over the last second or so (for "what")
    uint32_t in0 = 0, out0 = 0, to0 = 0, lost0 = 0;
    uint32_t inS = 0, outS = 0, toS = 0, lostS = 0;
} memLow;

static uint32_t bridgeToModules()
{
    uint32_t n = 0;
    for (const BridgeModule& m : bridge.mod) n += m.toIt;
    return n;
}

// "a; b; c": one more piece of what was going on
static void memAdd(const char* fmt, ...)
{
    const size_t at = strlen(memLow.what);
    if (at + 3 >= sizeof(memLow.what)) return;
    char* b = memLow.what + at;
    size_t cap = sizeof(memLow.what) - at;
    if (at) { *b++ = ';'; *b++ = ' '; cap -= 2; }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, cap, fmt, ap);
    va_end(ap);
}

// where: the address of our page just answered (nullptr: a link pass)
static void memWatch(const char* where)
{
    const uint32_t now = millis();
    const int32_t span = (int32_t)(now - memLow.secMs);
    if (memLow.secMs == 0 || span >= 1000)
    {
        const uint32_t in = bridge.in, out = bridge.out, to = bridgeToModules(), lost = bridgeLost();
        if (memLow.secMs)
        {
            memLow.inS = (in - memLow.in0) * 1000u / (uint32_t)span;
            memLow.outS = (out - memLow.out0) * 1000u / (uint32_t)span;
            memLow.toS = to >= memLow.to0 ? (to - memLow.to0) * 1000u / (uint32_t)span : 0;   // a module's place freed: not counted
            memLow.lostS = (lost - memLow.lost0) * 1000u / (uint32_t)span;
        }
        memLow.secMs = msStamp(now);
        memLow.in0 = in; memLow.out0 = out; memLow.to0 = to; memLow.lost0 = lost;
    }
    const uint32_t lo = ESP.getMinFreeHeap();
    if (memLow.seen && lo >= memLow.seen) return;                    // no new lowest point
    memLow.seen = lo;
    if (lo >= MEM_TELL) return;
    memLow.lowest = lo;
    memLow.ms = msStamp(now);
    memLow.freeThen = ESP.getFreeHeap();
    memLow.block = ESP.getMaxAllocHeap();
    memLow.what[0] = '\0';
    if (tpage.cur >= 0)                                              // the module's page: its request's first words
    {
        int k = 0, spaces = 0;
        while (k < (int)tpage.lineLen && k < 40 && tpage.line[k] != '\r' && tpage.line[k] != '\n' && !(tpage.line[k] == ' ' && ++spaces == 2)) k++;
        int conns = 0;
        for (const TPageConn& c : tpage.conn) if (c.used) conns++;
        memAdd("module page (port 8080) %.*s, %u bytes held, %d browser connection(s)", k, tpage.line, (unsigned)tpage.len, conns);
    }
    if (where) memAdd("our page answered %s", where);
    else if (webNow) memAdd("our page answering %s", webNow);
    if (bridge.sIn >= 0)
        memAdd("bridge a second: %lu to the Teensy, %lu to the program, %lu to modules, %lu lost",
               (unsigned long)memLow.inS, (unsigned long)memLow.outS, (unsigned long)memLow.toS, (unsigned long)memLow.lostS);
    memAdd("%d device(s) on our WiFi", WiFi.softAPgetStationNum());
    if (!strcmp(staState(), "con")) memAdd("joining the other network");
    if (memLow.told && memLow.told - lo < MEM_STEP) return;          // Messages: only a step lower than the last time
    memLow.told = lo;
    // A line of Messages holds EVENT_LEN characters: the lowest point on one line, then each piece on its own
    logEvent("Short of memory: %lu bytes free at the lowest. Going on then:", (unsigned long)lo);
    for (const char* p = memLow.what; *p; )
    {
        const char* end = strstr(p, "; ");
        const int n = end ? (int)(end - p) : (int)strlen(p);
        logEvent("... %.*s", n, p);
        p += n + (end ? 2 : 0);
    }
}

// What loop() does all the time, and every wait for a browser too: the link to the Teensy, the bridge, the module's page
static void linkPoll(uint32_t now)
{
    teensy.poll(now);
    bridgePoll(now);
    tpagePoll(now);
    memWatch(nullptr);
}

// ---- the link task and the watchdog (the why is at the top, with the lock)
// The watchdog (15 s) is an entry of the program's own, not the loop task's: the core's waits do not feed the loop
// task's, so an upload that paused for 15 s restarted the XIAO. The loop answers it at every pass and in its own
// waits (stillHere); while the core's web server waits for a browser the link task answers for it, for
// CORE_WAIT_MAX_MS after the loop's last sign of life - a web server that never comes back still ends in a restart
// (the page serves nobody else meanwhile). But not while a tablet steers through the bridge: the restart would take
// the WiFi away from it, and the bridge works without the page; it comes when the steering is over.
#define LINK_TASK_MS      2
#define LINK_TASK_STACK   8192         // as the loop task's: it runs the same code
#define CORE_WAIT_MAX_MS  60000

static esp_task_wdt_user_handle_t wdtProgram = nullptr;    // null: the loop task's own watchdog is used (setup)
static uint32_t                   loopSeenMs = 0;          // the loop's last sign of life (read and written with the lock held)

// The loop's sign of life: at every pass, in every wait of its own, at every handler the core calls
static void stillHere()
{
    loopSeenMs = millis();
    if (wdtProgram) esp_task_wdt_reset_user(wdtProgram);
    else feedLoopWDT();
}

static void linkTask(void*)
{
    for (;;)
    {
        if (xSemaphoreTakeRecursive(linkLock, 0) == pdTRUE)          // free: the loop is inside the core's web server
        {
            const uint32_t now = millis();
            linkPoll(now);
            if (wdtProgram && (now - loopSeenMs < CORE_WAIT_MAX_MS || bridgeSteering(now))) esp_task_wdt_reset_user(wdtProgram);   // waiting for a browser is not hanging
            xSemaphoreGiveRecursive(linkLock);
        }
        vTaskDelay(pdMS_TO_TICKS(LINK_TASK_MS));
    }
}

static void bridgeStart()
{
    teensy.onNetOut = bridgeNetOut;
    if (!cfg.bridge) return;
    bridge.sIn = bridgeSocket(BRIDGE_PORT_IN);
    bridge.sRtcm = bridgeSocket(BRIDGE_PORT_RTCM);
    bridge.sMod = bridgeSocket(BRIDGE_PORT_OUT);
    if (bridge.sIn < 0 || bridge.sRtcm < 0 || bridge.sMod < 0)
    {
        if (bridge.sIn >= 0) lwip_close(bridge.sIn);
        if (bridge.sRtcm >= 0) lwip_close(bridge.sRtcm);
        if (bridge.sMod >= 0) lwip_close(bridge.sMod);
        bridge.sIn = bridge.sRtcm = bridge.sMod = -1;
        logEvent("WiFi bridge: could NOT open UDP %u / %u / %u - the bridge is off", BRIDGE_PORT_IN, BRIDGE_PORT_RTCM, BRIDGE_PORT_OUT);
        return;
    }
    logEvent("WiFi bridge on: a program on the XIAO's network (UDP %u) reaches the module through the XIAO", BRIDGE_PORT_IN);
}

// While the program has the module the XIAO does not go looking for the other network (and starts again
// BRIDGE_STA_BACK_MS after the program left). Called twice a second from loop().
static void bridgeStaWatch(uint32_t now)
{
    if (!cfg.staOn || cfg.staSsid.length() == 0 || staSame) return;
    const bool busy = bridgeBusy(now);
    if (busy && !bridge.staPaused && WiFi.status() != WL_CONNECTED)
    {
        bridge.staPaused = true;
        WiFi.setAutoReconnect(false);
        WiFi.disconnect();                                            // stops the search; the settings stay
        logEvent("WiFi: not searching for '%s' while a program runs on the XIAO's network", cfg.staSsid.c_str());
    }
    else if (!busy && bridge.staPaused && now - bridge.busyMs >= BRIDGE_STA_BACK_MS)
    {
        bridge.staPaused = false;
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
        logEvent("WiFi: searching for '%s' again", cfg.staSsid.c_str());
    }
}

// ---------------------------------------------------------------- the sign-in page (captive page, 23 Nov 2026)
// A phone or computer that joins our network asks "is there internet?" (Android: connectivitycheck.gstatic.com,
// Windows: www.msftconnecttest.com, Apple: captive.apple.com). We answer every name with our own address, so the
// question lands on our page and the device opens it by itself: handy the first time. But the device then keeps
// saying "sign in to this network", every time it joins. So the page has a button, "Done on this device": from then
// on that device (by its MAC address, kept over restarts) gets "no such name" for every name, and says only "no
// internet", which is true. It never gets "there is internet": a tablet would then send all its traffic here, its
// NTRIP over mobile data too. Our own DNS server (UDP 53) answers per device; the core's DNSServer cannot.
#define CAPTIVE_DEVICES   16                 // devices that pressed "Done" (the oldest is forgotten first)
#define CAPTIVE_TTL_S     5                  // how long a device may keep our answer (short: "Done" works at once)
#define CAPTIVE_MAC_MS    5000               // the access point's list of devices is read again after this long

static struct
{
    uint8_t  mac[CAPTIVE_DEVICES][6];
    uint8_t  n = 0, next = 0;                // how many, and the place the next one takes
    int      sock = -1;                      // our DNS server's socket
    uint32_t asked = 0, refused = 0;         // names asked: answered with our address / "no such name" (a device that is done)
    struct { uint32_t ip; uint8_t mac[6]; bool known; } cache[ESP_WIFI_MAX_CONN_NUM];   // address -> MAC, from the list
    uint8_t  cacheN = 0;
    uint32_t cacheMs = 0;
} captive;

static void captiveLoad()
{
    prefs.begin("xiao", true);
    const String t = prefs.getString("capOk", "");             // "aabbccddeeff,..." oldest first
    prefs.end();
    captive.n = captive.next = 0;
    for (int at = 0; at + 12 <= (int)t.length() && captive.n < CAPTIVE_DEVICES; at += 13)
    {
        uint8_t m[6];
        bool ok = true;
        for (int b = 0; b < 6 && ok; b++)
        {
            unsigned v;
            char h[3] = { t[at + 2 * b], t[at + 2 * b + 1], 0 };
            ok = sscanf(h, "%2x", &v) == 1;
            m[b] = (uint8_t)v;
        }
        if (ok) memcpy(captive.mac[captive.n++], m, 6);
    }
    captive.next = captive.n % CAPTIVE_DEVICES;
}

static void captiveSave()
{
    char t[CAPTIVE_DEVICES * 13 + 1] = "";
    for (uint8_t i = 0; i < captive.n; i++)                    // oldest first: from the place the next one takes
    {
        const uint8_t* m = captive.mac[(captive.n < CAPTIVE_DEVICES ? i : (captive.next + i) % CAPTIVE_DEVICES)];
        snprintf(t + strlen(t), sizeof(t) - strlen(t), "%s%02x%02x%02x%02x%02x%02x", i ? "," : "", m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    prefs.begin("xiao", false);
    prefs.putString("capOk", t);
    prefs.end();
}

// The MAC of the device on our network at this address (ip as in a sockaddr), from the access point's list
static bool captiveMacOf(uint32_t ip, uint8_t* mac, uint32_t now)
{
    for (int pass = 0; pass < 2; pass++)
    {
        for (uint8_t i = 0; i < captive.cacheN; i++)
            if (captive.cache[i].ip == ip) { memcpy(mac, captive.cache[i].mac, 6); return true; }
        if (pass == 1 || (captive.cacheMs != 0 && now - captive.cacheMs < 200)) return false;   // read just now
        wifi_sta_list_t l;
        esp_netif_pair_mac_ip_t p[ESP_WIFI_MAX_CONN_NUM];
        captive.cacheN = 0;
        captive.cacheMs = msStamp(now);
        if (esp_wifi_ap_get_sta_list(&l) != ESP_OK || l.num <= 0) return false;
        const int n = l.num < ESP_WIFI_MAX_CONN_NUM ? l.num : ESP_WIFI_MAX_CONN_NUM;
        for (int i = 0; i < n; i++) { memcpy(p[i].mac, l.sta[i].mac, 6); p[i].ip.addr = 0; }
        esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (!ap || esp_netif_dhcps_get_clients_by_mac(ap, n, p) != ESP_OK) return false;
        for (int i = 0; i < n; i++)
        {
            if (p[i].ip.addr == 0) continue;
            captive.cache[captive.cacheN].ip = p[i].ip.addr;
            memcpy(captive.cache[captive.cacheN].mac, p[i].mac, 6);
            captive.cacheN++;
        }
    }
    return false;
}

// This device pressed "Done" (ip as in a sockaddr)
static bool captiveDone(uint32_t ip, uint32_t now)
{
    if (captive.cacheMs != 0 && now - captive.cacheMs >= CAPTIVE_MAC_MS) captive.cacheN = 0;   // read the list again
    uint8_t m[6];
    if (!captiveMacOf(ip, m, now)) return false;
    for (uint8_t i = 0; i < captive.n; i++) if (!memcmp(captive.mac[i], m, 6)) return true;
    return false;
}

// "Done on this device": false when the device is not on our network (or the access point does not know it)
static bool captiveMarkDone(uint32_t ip, uint32_t now)
{
    captive.cacheN = 0;                                        // a device that just joined: read the list again
    captive.cacheMs = 0;
    uint8_t m[6];
    if (!captiveMacOf(ip, m, now)) return false;
    for (uint8_t i = 0; i < captive.n; i++) if (!memcmp(captive.mac[i], m, 6)) return true;
    memcpy(captive.mac[captive.next], m, 6);
    captive.next = (captive.next + 1) % CAPTIVE_DEVICES;
    if (captive.n < CAPTIVE_DEVICES) captive.n++;
    captiveSave();
    logEvent("Sign-in page: done on the device %02X:%02X:%02X:%02X:%02X:%02X (it gets no more sign-in notice)", m[0], m[1], m[2], m[3], m[4], m[5]);
    return true;
}

static void captiveStart()
{
    captiveLoad();
    captive.sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (captive.sock < 0) { logEvent("Sign-in page: the DNS server did NOT start (no socket)"); return; }
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(53);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(captive.sock, (struct sockaddr*)&a, sizeof(a)) < 0)
    {
        lwip_close(captive.sock);
        captive.sock = -1;
        logEvent("Sign-in page: the DNS server did NOT start (port 53 taken)");
    }
}

// Our DNS server: a few questions per loop pass, never waits. Only for devices on our own network.
static void captivePoll(uint32_t now)
{
    if (captive.sock < 0) return;
    static uint8_t b[512];
    for (int k = 0; k < 8; k++)
    {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int n = lwip_recvfrom(captive.sock, b, sizeof(b) - 16, MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
        if (n <= 0) break;
        const uint32_t ip = from.sin_addr.s_addr;
        if (n < 12 || !onApNet((const uint8_t*)&ip)) continue;          // a question, from our own network only
        if ((b[2] & 0x80) || (b[2] & 0x78) || b[4] != 0 || b[5] == 0) continue;   // a query, standard, with a question
        int at = 12;                                                    // the first question's name, then its type and class
        while (at < n && b[at] != 0 && (b[at] & 0xC0) == 0) at += b[at] + 1;
        if (at >= n || b[at] != 0 || at + 5 > n) continue;
        at++;
        const uint16_t type = (uint16_t)(b[at] << 8 | b[at + 1]), cls = (uint16_t)(b[at + 2] << 8 | b[at + 3]);
        at += 4;                                                        // the answer: our header, that one question, ...
        b[2] = 0x84 | (b[2] & 0x01);                                    // an answer, authoritative, "recursion desired" as asked
        b[4] = 0; b[5] = 1;                                             // one question
        b[6] = b[7] = b[8] = b[9] = b[10] = b[11] = 0;
        if (captiveDone(ip, now)) { b[3] = 3; captive.refused++; }      // "no such name": the device says "no internet"
        else
        {
            b[3] = 0;
            captive.asked++;
            if (type == 1 && cls == 1)                                  // an address: ours
            {
                const uint8_t a[16] = { 0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, CAPTIVE_TTL_S, 0, 4, apNet[0], apNet[1], apNet[2], 1 };
                memcpy(b + at, a, sizeof(a));
                at += sizeof(a);
                b[7] = 1;
            }
        }
        struct sockaddr_in to = from;
        lwip_sendto(captive.sock, b, at, MSG_DONTWAIT, (struct sockaddr*)&to, sizeof(to));
    }
}

// ---------------------------------------------------------------- WiFi
static bool     staWasUp = false;
static int      apClientsWas = -1;
static uint32_t rebootAtMs = 0;
static String   staIpStr = "";

static const char* resetReasonText()
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:  return "power on";                      // the module was switched on
        case ESP_RST_SW:       return "restart requested";             // page, update, new WiFi settings, USB command
        case ESP_RST_PANIC:    return "program error";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:      return "program stopped (watchdog)";
        case ESP_RST_BROWNOUT: return "low voltage (brownout)";
        case ESP_RST_DEEPSLEEP:return "deep sleep";
        case ESP_RST_EXT:      return "RESET button";
        default:               return "other";
    }
}

static const char* staState()
{
    if (!cfg.staOn || cfg.staSsid.length() == 0) return "off";
    if (staSame) return "same";                        // left: it has the numbers of our own network (wifiWatch)
    if (bridge.staPaused) return "wait";               // not searched for while a program runs over the WiFi bridge
    return WiFi.status() == WL_CONNECTED ? "ok" : "con";
}

static void wifiStart()
{
    WiFi.persistent(false);
    WiFi.setHostname(cfg.host.c_str());
    const bool sta = cfg.staOn && cfg.staSsid.length() > 0;
    WiFi.mode(sta ? WIFI_AP_STA : WIFI_AP);
    if (cfg.bridge) memcpy(apNet, cfg.modNet, 3);    // the module's numbers (above, apNet); the devices get .2 to .12
    snprintf(apIpText, sizeof(apIpText), "%u.%u.%u.1", apNet[0], apNet[1], apNet[2]);
    WiFi.softAPConfig(apIp(), apIp(), IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str(), 1, 0, AP_STATIONS);      // (channel 1 and the name shown, as the core's own)
    if (sta)
    {
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
    }
    if (captive.sock < 0) captiveStart();          // every name -> this page (the sign-in page), until "Done" on a device
    if (MDNS.begin(cfg.host.c_str()))
    {
        MDNS.setInstanceName(cfg.apSsid);                   // the name in AgOpenWeb's list: the own network's, unique to this XIAO
        MDNS.addService("http", "tcp", 80);                 // the page, for the AgOpenWeb fork's "Search the network"
        MDNS.addService("aowcard", "tcp", 80);              // the official AgOpenWeb add-on card announcement (Docs/ADD_ON_CARDS.md)
        MDNS.addServiceTxt("aowcard", "tcp", "v", "1");     //   TXT v=1 is required, path says where the page is
        MDNS.addServiceTxt("aowcard", "tcp", "path", "/");
    }
    logEvent("WiFi: own network '%s' at %s%s", cfg.apSsid.c_str(), apIpText, sta ? ", joining the other network" : "");
}

// State changes of the other network and of the clients on ours: events
static void wifiWatch()
{
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && cfg.bridge && !staSame)
    {
        // Our own network has the module's numbers (wifiStart); an other network with the same ones (a tractor's router
        // at 192.168.5.1) would leave the XIAO two ways to every address there, and the program's packets could go out
        // the wrong one. Seen as the XIAO gets its address there: left, and not joined again while the bridge is on.
        IPAddress me = WiFi.localIP();
        const uint8_t m[3] = { me[0], me[1], me[2] };
        if (onApNet(m))
        {
            staSame = true;
            up = false;
            WiFi.setAutoReconnect(false);
            WiFi.disconnect();
            logEvent("WiFi: left '%s': it has the numbers of the XIAO's own network (%u.%u.%u.x, the module's: WiFi bridge on). Not joined while the bridge is on",
                     cfg.staSsid.c_str(), m[0], m[1], m[2]);
        }
    }
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

// The module's (Teensy's) numbers, from its status ("ip", PGN 141): kept, so that with the bridge on our own network
// has them from the start (wifiStart). When they change with the bridge on (the module's subnet changed from a page),
// the XIAO restarts, and its network follows: the tablet and the modules on it join it again.
static uint32_t modNetSets = 0;
static void modNetWatch(uint32_t now)
{
    if (teensy.statusSets == modNetSets) return;
    const char* key = "\"ip\":\"";
    const size_t kn = strlen(key);
    int at = -1;
    for (size_t i = 0; i + kn < teensy.statusLen; i++) if (!memcmp(teensy.status + i, key, kn)) { at = (int)(i + kn); break; }
    if (at < 0) { modNetSets = teensy.statusSets; return; }
    char t[16];
    size_t k = 0;
    while (k < sizeof(t) - 1 && at + k < teensy.statusLen && teensy.status[at + k] != '"') { t[k] = teensy.status[at + k]; k++; }
    t[k] = '\0';
    unsigned a, b, c, d;
    if (sscanf(t, "%u.%u.%u.%u", &a, &b, &c, &d) != 4 || a > 255 || b > 255 || c > 255 || d > 255) { modNetSets = teensy.statusSets; return; }
    const uint8_t n[3] = { (uint8_t)a, (uint8_t)b, (uint8_t)c };
    if (!netNumbersOk(n) || !memcmp(n, cfg.modNet, 3)) { modNetSets = teensy.statusSets; return; }
    if (cfg.bridge && bridgeSteering(now)) return;              // never under a tablet that steers (checked again next time)
    modNetSets = teensy.statusSets;
    memcpy(cfg.modNet, n, 3);
    settingsSave();
    if (!cfg.bridge) return;
    logEvent("The module is now on %u.%u.%u.x: the XIAO restarts so that its own network follows it (WiFi bridge on)", n[0], n[1], n[2]);
    if (!rebootAtMs) rebootAtMs = now + 1500;
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
    j.str("apIp", apIpText);
    j.i("cl", WiFi.softAPgetStationNum());
    if (cfg.staOn && cfg.staSsid.length())
    {
        j.str("sta", cfg.staSsid.c_str());
        j.str("st", staState());
        if (WiFi.status() == WL_CONNECTED) { j.str("ip", WiFi.localIP().toString().c_str()); j.i("rssi", WiFi.RSSI()); }
    }
    j.num("t", temperatureRead(), 1);         // 4 Nov 2026: the chip's temperature (its own sensor), degrees C
    j.i("br", bridge.sIn >= 0 ? 1 : 0);       // 22 Nov 2026: the WiFi bridge is switched on
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

// Requests that come in on our own network for any other host name (phones checking for internet) go
// to our page. Never on the other network: there the router may give the XIAO its own name
// (xiao.home, ...), and our own address does not exist on that network.
static bool captiveRedirect()
{
    if (!(server.client().localIP() == WiFi.softAPIP())) return false;
    if (captiveDone((uint32_t)server.client().remoteIP(), millis())) return false;   // pressed "Done": no sign-in page for it
    String h = server.hostHeader();
    h.toLowerCase();
    if (h.length() == 0 || h == apIpText || h == cfg.host + ".local") return false;
    server.sendHeader("Location", String("http://") + apIpText + "/", true);
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
    j.obj("memLow");                           // the lowest point under MEM_TELL since the start, and what was going on (24 Nov 2026)
    j.u("free", memLow.lowest);                // 0: never that short
    j.i("ago", memLow.ms ? (long)((now - memLow.ms) / 1000u) : -1);   // (noted before now: never ahead of it)
    j.u("freeThen", memLow.freeThen);
    j.u("block", memLow.block);                // the largest piece of free memory then
    j.str("what", memLow.what);
    j.close();
    j.num("temp", temperatureRead(), 1);       // 4 Nov 2026: the chip's temperature, degrees C
    j.u("sketch", sketchSize);
    j.u("sketchFree", sketchFree);

    j.obj("wifi");
    j.str("ap", cfg.apSsid.c_str());
    j.str("apIp", apIpText);                   // 192.168.4.1; with the WiFi bridge on, the module's numbers (22 Nov 2026)
    j.i("apCl", WiFi.softAPgetStationNum());
    j.i("ch", WiFi.channel());
    j.str("host", cfg.host.c_str());
    j.bo("staOn", cfg.staOn && cfg.staSsid.length() > 0);
    j.str("sta", cfg.staSsid.c_str());
    j.str("st", staState());
    if (WiFi.status() == WL_CONNECTED) { j.str("ip", WiFi.localIP().toString().c_str()); j.i("rssi", WiFi.RSSI()); }
    j.close();

    j.obj("bridge");                           // the WiFi bridge (22 Nov 2026)
    j.bo("on", bridge.sIn >= 0);
    j.bo("teensy", teensy.bridgeKnown(now));   // the Teensy's firmware knows the bridge
    j.i("state", teensy.bridgeKnown(now) ? teensy.bridgeState : -1);   // 0 free, 1 a program on the cable has the module, 2 ours has it
    if (bridge.progIp) { char t[20]; ipText(t, sizeof(t), bridge.progIp); j.str("prog", t); }
    else j.str("prog", "");
    j.bo("steer", bridgeSteering(now));
    j.u("in", bridge.in);
    j.u("out", bridge.out);
    j.num("rtcmKB", bridge.rtcm / 1024.0, 1);
    j.u("lost", bridgeLost());
    j.u("other", bridge.other);
    j.arr("mods");                             // the other modules on this network (sprayer, seed drill)
    for (const BridgeModule& m : bridge.mod)
    {
        if (m.ip == 0) continue;
        char t[20];
        ipText(t, sizeof(t), m.ip);
        j.open();
        j.str("ip", t);
        j.u("to", m.toIt);
        j.u("from", m.fromIt);
        j.close();
    }
    j.closeArr();
    j.u("modTeensy", bridge.modTeensy);        // their packets passed to the Teensy for its Machine tab (PGN 152) ...
    j.u("modLink", bridge.modLink);            // ... and those not passed: no room on the link
    j.close();

    {
        const bool onAp = server.client().localIP() == WiFi.softAPIP();
        j.obj("captive");                      // the sign-in page (23 Nov 2026): this browser's device, and how many are done
        j.bo("ap", onAp);
        j.bo("done", onAp && captiveDone((uint32_t)server.client().remoteIP(), now));
        j.u("n", captive.n);
        j.close();
    }
    j.obj("tpage");                            // the module's page through the XIAO (Nov 2026): http://<apIp>:8080/
    const char* tn = tpageNot(now);
    j.bo("ready", tn == nullptr);
    j.str("why", tn ? tn : "");
    j.u("req", tpage.req);
    j.u("refused", tpage.refused);
    j.u("broken", tpage.broken);
    j.u("lowMem", tpage.lowMem);               // the times it waited for memory (TPAGE_HEAP_LOW)
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
    {   // sections 1-64 as 16 hex digits (section 1 = the lowest bit), as the Teensy's Machine tab: 1-16 from PGN 254, the
        // rest from a PGN 229 of the last 2 s; secN = the highest section asked for since the start
        uint64_t sec = teensy.p254.sections;
        if (teensy.ms229 && now - teensy.ms229 < 2000) sec |= teensy.sec229 & ~(uint64_t)0xFFFF;
        if (teensy.secTop < 64 && sec >> teensy.secTop) teensy.secTop = 64 - __builtin_clzll(sec);
        char hex[20];
        snprintf(hex, sizeof(hex), "%08lx%08lx", (unsigned long)(sec >> 32), (unsigned long)(sec & 0xFFFFFFFF));
        j.str("sec64", hex);
        j.u("secN", teensy.secTop);
    }
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
    j.bo("bridge", cfg.bridge);
    j.bo("apPassOwn", cfg.apPass != DEFAULT_AP_PASS);       // the WiFi bridge asks for a password of one's own
    char mn[16];
    snprintf(mn, sizeof(mn), "%u.%u.%u", cfg.modNet[0], cfg.modNet[1], cfg.modNet[2]);
    j.str("modNet", mn);                                    // the module's numbers: our own network's with the bridge on
    j.raw("}");
    sendJson(j);
}

// Things that restart the XIAO or take its radio away are refused while the program on our WiFi steers the
// tractor through us (the WiFi bridge): the steering would stop in the middle of the field.
static bool bridgeLetsUs(const char* what)
{
    if (!bridgeSteering(millis())) return true;
    char m[150];
    snprintf(m, sizeof(m), "%s: not now - autosteer is on in the program that reaches the module through this XIAO. Switch autosteer off first.", what);
    sendError(409, m);
    return false;
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
    if (!bridgeLetsUs("Save")) return;
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
    if (server.hasArg("bridge")) s.bridge = server.arg("bridge") == "1";      // a page of before 22 Nov 2026 sends none: kept
    if (s.bridge && s.apPass == DEFAULT_AP_PASS)
    {
        sendError(400, "WiFi bridge: first give the XIAO's network a password of your own (the field above) - whoever is on this network can then steer the tractor");
        return;
    }
    cfg = s;
    settingsSave();
    logEvent("Settings saved: own network '%s', other network %s, name %s.local, WiFi bridge %s - restarting", cfg.apSsid.c_str(),
          cfg.staOn ? cfg.staSsid.c_str() : "off", cfg.host.c_str(), cfg.bridge ? "on" : "off");
    server.send(200, "application/json", "{\"ok\":true}");
    rebootAtMs = millis() + 1500;
}

static int16_t scanState = -2;            // -2 not started, -1 running

static void apiScan()
{
    Json j(jbuf, sizeof(jbuf));
    j.raw("{");
    const int16_t n = WiFi.scanComplete();
    if (bridgeBusy(millis()) && n != WIFI_SCAN_RUNNING && scanState != -1)
    {
        // a search takes the radio to every channel in turn, away from the program that has the module (the WiFi bridge)
        j.bo("busy", false);
        j.str("note", "Not searching now: a program is running through this XIAO's WiFi. Type the network's name, or search when the program is closed.");
        j.arr("list");
        j.closeArr();
    }
    else if (n == WIFI_SCAN_RUNNING) { j.bo("busy", true); }
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
static size_t  updBytes = 0;            // size of the program received (Update.progress() reads 0 once Update.end() has run)

// How long an upload may wait for the next data from the browser (3 Nov 2026). The web server gives up after 5 s
// (HTTP_MAX_SEND_WAIT), and a weak Wi-Fi link can pause longer than that: the upload was then cut off with nothing
// in Messages and only "The connection failed" on the page.
#define UPLOAD_WAIT_MS 20000

// Why an update would be refused before its first byte is taken, or nullptr: the XIAO's own program (toTeensy false) or
// the module's firmware over the link (true). Asked at the start of the upload, and by POST /api/update-check, which the
// page sends before it sends (or downloads) the file: a wrong password, autosteer on or a silent Teensy is then told at
// once in the install box, not after the whole file has gone up (24 Nov 2026).
static const char* updateRefusal(bool toTeensy)
{
    if (server.arg("pw") != cfg.apPass) return "wrong password";
    if (bridgeSteering(millis())) return "not now: autosteer is on in the program that reaches the module through this XIAO - switch autosteer off first";
    if (toTeensy && !teensy.teensyTalking(millis())) return "no data from the module: is the Teensy on and the link working?";
    return nullptr;
}

// POST /api/update-check, pw= and for=teensy (the module's firmware) or nothing (the XIAO's own): would the update start?
// Nothing is changed; a refusal is in Messages as the upload's own would be.
static void apiUpdateCheck()
{
    const bool toTeensy = server.arg("for") == "teensy";
    const char* why = updateRefusal(toTeensy);
    if (!why) { server.send(200, "application/json", "{\"ok\":true}"); return; }
    logEvent("%s REFUSED: %s", toTeensy ? "Module firmware update" : "Program update", why);
    sendError(strcmp(why, "wrong password") == 0 ? 403 : 409, why);
}

static void updateUpload()
{
    HTTPUpload& up = server.upload();
    linkPoll(millis());                     // keep saying hello to the Teensy while the file comes in (and the WiFi bridge going)
    if (up.status == UPLOAD_FILE_START)
    {
        updOk = false;
        updErr = "";
        server.client().setTimeout(UPLOAD_WAIT_MS);
        if (const char* why = updateRefusal(false)) { updErr = why; return; }
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
            updBytes = up.totalSize;
            if (Update.end(true)) updOk = true;
            else updErr = Update.errorString();
        }
        else Update.abort();
    }
    else if (up.status == UPLOAD_FILE_ABORTED)                      // the browser's connection dropped or stalled
    {
        Update.abort();
        if (updErr != "wrong password")
            logEvent("Program update STOPPED after %u kB: the browser's connection dropped (weak Wi-Fi?), old program kept",
                     (unsigned)(up.totalSize / 1024));
        updErr = "upload aborted";
    }
}

static void updateDone()
{
    if (updOk)
    {
        logEvent("Program update: %u bytes checked, restarting", (unsigned)updBytes);
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

// POST /api/subnet, subnet=192.168.5 (2026-10-24): the module's Ethernet subnet, sent to the Teensy (PGN 144),
// which saves it and restarts with the new IP - the way to reach a module that sits on another network than
// the device, since this page is on WiFi whatever the module's IP is.
static void apiSubnet()
{
    if (!passwordOk()) return;
    unsigned a = 0, b = 0, c = 0;
    char tail;
    const String v = server.arg("subnet");
    if (sscanf(v.c_str(), "%u.%u.%u%c", &a, &b, &c, &tail) != 3 || a == 0 || a == 127 || a > 223 || b > 255 || c > 255)
    {
        sendError(400, "subnet: three numbers, like 192.168.5 (the first 1 to 223, not 127)");
        return;
    }
    if (!teensy.teensyTalking(millis())) { sendError(409, "no data from the module: is the Teensy on and the link working?"); return; }
    teensy.sendSubnet((uint8_t)a, (uint8_t)b, (uint8_t)c);
    logEvent("Module subnet %u.%u.%u sent to the Teensy from the page: it saves it and restarts", a, b, c);
    server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------- the module's (Teensy's) firmware, over the link
// 2 Nov 2026. For a module whose own page cannot be reached (its Ethernet, its network, a bug in the page): the
// .hex comes from the browser as our own update does (POST /api/teensy-update, the file in a form) and goes on
// to the Teensy over the link in blocks of LINK_FW_BLOCK bytes, each answered by the Teensy before the next
// (PGN 145, zXiao.ino); the Teensy checks the file as it does one from its own page and installs it. About a
// minute for a 2 MB .hex; the browser's upload goes at the link's pace. POST /api/teensy-revert asks the Teensy
// to put back the previous firmware it keeps in its flash (nothing is sent).
static String   tfwErr;
static bool     tfwOk = false;
static uint32_t tfwSize = 0;             // bytes the Teensy checked and is installing
static uint16_t tfwSeq = 0;              // the last block number sent

// Wait up to ms for the Teensy's answer to command cmd, block seq: 1 = ok, 0 = refused (tfwErr says why),
// -1 = no answer (tfwErr = none). The link keeps being read (and the hello sent) meanwhile.
static int tfwWait(uint8_t cmd, uint16_t seq, uint32_t ms, const char* none)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < ms)
    {
        teensy.poll(millis());
        stillHere();
        if (teensy.fwAck)
        {
            teensy.fwAck = false;
            if (teensy.fwCmd == cmd && teensy.fwSeq == seq)
            {
                if (teensy.fwCode == 0) return 1;
                tfwErr = teensy.fwText[0] ? teensy.fwText : "refused by the module";
                return 0;
            }
        }
        delay(1);
    }
    tfwErr = none;
    return -1;
}

static const char* TFW_NO_ANSWER = "no answer from the module: is the Teensy on, with a firmware of 2 Nov 2026 or later (install that one from its own page or by USB once)?";

static void teensyUpdateUpload()
{
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START)
    {
        tfwOk = false;
        tfwErr = "";
        tfwSeq = 0;
        tfwSize = 0;
        server.client().setTimeout(UPLOAD_WAIT_MS);
        if (const char* why = updateRefusal(true)) { tfwErr = why; return; }
        const uint32_t size = strtoul(server.arg("size").c_str(), nullptr, 10);
        logEvent("Module firmware: sending %s (%lu kB) to the Teensy over the link", up.filename.c_str(), (unsigned long)(size / 1024));
        teensy.fwAck = false;
        teensy.fwBegin(server.arg("force") == "1", size, up.filename.c_str());
        tfwWait(1, 0, 5000, TFW_NO_ANSWER);
    }
    else if (up.status == UPLOAD_FILE_WRITE)
    {
        if (tfwErr.length()) { linkPoll(millis()); return; }        // refused: the rest of the file is read and dropped, the link and the bridge go on
        for (size_t off = 0; off < up.currentSize && tfwErr.length() == 0; off += LINK_FW_BLOCK)
        {
            const size_t left = up.currentSize - off;
            const uint8_t n = (uint8_t)(left < LINK_FW_BLOCK ? left : LINK_FW_BLOCK);
            tfwSeq++;
            for (int attempt = 0; attempt < 2; attempt++)          // once more when no answer came (a lost frame or answer)
            {
                teensy.fwAck = false;
                teensy.fwBlock(tfwSeq, up.buf + off, n);
                const int r = tfwWait(2, tfwSeq, 3000, "the module stopped answering during the transfer (link error) - try again");
                if (r >= 0) break;
                if (attempt == 0) tfwErr = "";
            }
        }
    }
    else if (up.status == UPLOAD_FILE_END)
    {
        if (tfwErr.length()) return;
        teensy.fwAck = false;
        teensy.fwSend(3);
        if (tfwWait(3, 0, 15000, "no answer from the module after the last block") == 1)
        {
            tfwOk = true;
            tfwSize = strtoul(teensy.fwText, nullptr, 10);
        }
    }
    else if (up.status == UPLOAD_FILE_ABORTED)                      // the browser's connection dropped or stalled
    {
        if (tfwErr.length() == 0)
        {
            teensy.fwSend(4);
            logEvent("Module firmware update STOPPED after %u kB: the browser's connection dropped (weak Wi-Fi?), firmware kept",
                     (unsigned)(up.totalSize / 1024));
            tfwErr = "upload aborted";
        }
    }
}

static void teensyUpdateDone()
{
    if (tfwOk)
    {
        logEvent("Module firmware: %lu bytes checked by the Teensy, it is installing and restarting", (unsigned long)tfwSize);
        char b[64];
        snprintf(b, sizeof(b), "{\"ok\":true,\"size\":%lu}", (unsigned long)tfwSize);
        server.send(200, "application/json", b);
    }
    else
    {
        if (tfwErr.length() == 0) tfwErr = "no file";
        logEvent("Module firmware update REFUSED: %s", tfwErr.c_str());
        sendError(tfwErr == "wrong password" ? 403 : 400, tfwErr.c_str());
    }
}

// POST /api/teensy-revert: the Teensy puts back the previous firmware it keeps (PGN 145, command 5)
static void apiTeensyRevert()
{
    if (!passwordOk()) return;
    if (!teensy.teensyTalking(millis())) { sendError(409, "no data from the module: is the Teensy on and the link working?"); return; }
    teensy.fwAck = false;
    teensy.fwSend(5);
    if (tfwWait(5, 0, 10000, TFW_NO_ANSWER) == 1)
    {
        logEvent("Module firmware: the Teensy goes back to its previous firmware and restarts");
        server.send(200, "application/json", "{\"ok\":true}");
    }
    else
    {
        logEvent("Going back to the previous module firmware REFUSED: %s", tfwErr.c_str());
        sendError(409, tfwErr.c_str());
    }
}

// The sign-in page: "done" = no more sign-in notice on the device of this browser (on our own network only; no
// password: it changes that device only); "forget" = every device gets the sign-in page again (password)
static void apiCaptive()
{
    const uint32_t now = millis();
    if (server.arg("forget") == "1")
    {
        if (!passwordOk()) return;
        captive.n = captive.next = 0;
        captiveSave();
        logEvent("Sign-in page: every device gets it again");
        server.send(200, "application/json", "{\"ok\":true}");
        return;
    }
    if (!(server.client().localIP() == WiFi.softAPIP())) { sendError(400, "only for a device on the XIAO's own WiFi"); return; }
    if (!captiveMarkDone((uint32_t)server.client().remoteIP(), now)) { sendError(400, "the XIAO does not find this device among those on its WiFi (a fixed address?)"); return; }
    server.send(200, "application/json", "{\"ok\":true}");
}

static void apiRestart()
{
    if (!passwordOk()) return;
    if (!bridgeLetsUs("Restart")) return;
    logEvent("Restart requested from the page");
    server.send(200, "application/json", "{\"ok\":true}");
    rebootAtMs = millis() + 1000;
}

// The page's ETag: its build date (13 Nov 2026, as the Teensy page). The page is served with "Cache-Control:
// no-cache", so the browser keeps it but asks every time ("If-None-Match"); the same build answers "304 Not
// Modified" (a few hundred bytes), a new build sends the page. So the 53 kB page only travels over the WiFi once
// per build and browser, not at every visit.
static char pageEtag[40];
static void pageEtagSetup()
{
    const char* b = __DATE__ " " __TIME__;           // e.g. Nov  3 2026 14:20:11 -> "Nov-3-2026-14:20:11" (no spaces in an ETag)
    size_t k = 0;
    pageEtag[k++] = '"';
    for (const char* p = b; *p && k < sizeof(pageEtag) - 2; p++)
    {
        if (*p == ' ') { if (pageEtag[k - 1] != '-') pageEtag[k++] = '-'; }
        else pageEtag[k++] = *p;
    }
    pageEtag[k++] = '"';
    pageEtag[k] = '\0';
}

static void webSetup()
{
    pageEtagSetup();
    static const char* keep[] = {"If-None-Match"};
    server.collectHeaders(keep, 1);
    server.on("/", HTTP_GET, []() {
        if (captiveRedirect()) return;
        server.sendHeader("ETag", pageEtag);
        server.sendHeader("Cache-Control", "no-cache");
        if (server.header("If-None-Match").indexOf(pageEtag) >= 0) { server.send(304); return; }
        server.send_P(200, "text/html; charset=utf-8", XIAO_PAGE);
    });
    server.on("/api/state", HTTP_GET, apiState);
    server.on("/api/teensy", HTTP_GET, apiTeensy);
    server.on("/api/log", HTTP_GET, apiLog);
    server.on("/api/settings", HTTP_GET, apiSettingsGet);
    server.on("/api/settings", HTTP_POST, apiSettingsPost);
    server.on("/api/scan", HTTP_GET, apiScan);
    server.on("/api/restart", HTTP_POST, apiRestart);
    server.on("/api/captive", HTTP_POST, apiCaptive);
    server.on("/api/subnet", HTTP_POST, apiSubnet);
    server.on("/api/teensy-update", HTTP_POST, teensyUpdateDone, teensyUpdateUpload);   // the Teensy's firmware over the link
    server.on("/api/teensy-revert", HTTP_POST, apiTeensyRevert);
    server.on("/api/update-check", HTTP_POST, apiUpdateCheck);                         // before an update: would it start?
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
    Serial.printf("Own WiFi '%s' at %s, %d client(s)\n", cfg.apSsid.c_str(), apIpText, WiFi.softAPgetStationNum());
    if (cfg.staOn) Serial.printf("Other network '%s': %s %s\n", cfg.staSsid.c_str(), staState(), WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "");
    Serial.printf("Teensy: %s, %lu frames, %lu bad checksums, status %ld s ago\n", teensy.teensyTalking(millis()) ? "talking" : "SILENT",
                  (unsigned long)teensy.goodFrames, (unsigned long)teensy.crcErr, teensy.statusMs ? (long)((millis() - teensy.statusMs) / 1000) : -1L);
    if (bridge.sIn < 0) Serial.println("WiFi bridge: off");
    else
    {
        char t[20] = "none";
        if (bridge.progIp) ipText(t, sizeof(t), bridge.progIp);
        Serial.printf("WiFi bridge: on, Teensy %s, program %s, %lu packets to the module, %lu to the program, %lu lost, %d other module(s) on this network\n",
                      !teensy.bridgeKnown(millis()) ? "does NOT know it (older firmware?)" : teensy.bridgeState == LINK_BRIDGE_WIFI ? "gives it the module"
                      : teensy.bridgeState == LINK_BRIDGE_CABLE ? "has a program on its cable" : "free", t,
                      (unsigned long)bridge.in, (unsigned long)bridge.out, (unsigned long)bridgeLost(), bridgeModules());
    }
    Serial.println("Commands: status | wifi-reset | restart");
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
        if (!strcmp(cmd, "status") || !strcmp(cmd, "estado")) printState();             // "estado", "reiniciar": the old names still work
        else if (!strcmp(cmd, "wifi-reset"))
        {
            settingsReset();
            logEvent("WiFi settings back to the defaults (USB command) - restarting");
            delay(300);
            ESP.restart();
        }
        else if (!strcmp(cmd, "restart") || !strcmp(cmd, "reiniciar")) { delay(100); ESP.restart(); }
        else if (cmd[0]) Serial.println("Commands: status | wifi-reset | restart");
    }
}

// ---------------------------------------------------------------- setup / loop
void setup()
{
    linkLock = xSemaphoreCreateRecursiveMutex();              // the loop's from here on (LinkWebServer::handleClient lets go of it)
    if (linkLock) xSemaphoreTakeRecursive(linkLock, portMAX_DELAY);
    Serial.begin(115200);                                     // USB-C (bench / panel port)
    TeensyPort.setRxBufferSize(4096);                         // a busy web page must not lose the Teensy's data
    TeensyPort.setTxBufferSize(1024);                         // the WiFi bridge: a write never waits for the link (bridgePoll asks for room first)
    TeensyPort.begin(LINK_BAUD, SERIAL_8N1, PIN_FROM_TEENSY, PIN_TO_TEENSY);
    TeensyPort.setRxFIFOFull(64);                             // see README: room for the WiFi's busy moments
    teensy.poll(millis());                                      // first hello at once
    settingsLoad();
    sketchSize = ESP.getSketchSize();
    sketchFree = ESP.getFreeSketchSpace();
    logEvent("%s started (%s)", XIAO_FW_NAME, resetReasonText());
    wifiStart();
    bridgeStart();
    webSetup();
    tpageStart();                                             // the module's page through the XIAO (port 8080)
    esp_task_wdt_config_t w = {15000, 0, true};               // restart if the program hangs for 15 s
    esp_task_wdt_reconfigure(&w);
    if (esp_task_wdt_add_user("program", &wdtProgram) != ESP_OK) { wdtProgram = nullptr; enableLoopWDT(); }
    stillHere();
    if (!linkLock || xTaskCreateUniversal(linkTask, "link", LINK_TASK_STACK, nullptr, 2, nullptr, ARDUINO_RUNNING_CORE) != pdPASS)
        logEvent("The link task did NOT start: a slow browser can hold up the link to the Teensy");
}

void loop()
{
    static uint32_t infoMs = 0, watchMs = 0;

    stillHere();
    linkPoll(millis());
    captivePoll(millis());
    server.handleClient();
    usbCommands();

    // The time is read here, after the web server: while that waited for a browser the link task ran, and its time
    // stamps (the Teensy's "who has the module", the program's last packet) are later than a time read before it.
    // "earlier time - stamp" reads four billion milliseconds: for that pass the program looked long gone, and the
    // search for the other network started again with the tablet steering.
    const uint32_t now = millis();
    if (now - watchMs >= 500) { watchMs = now; wifiWatch(); bridgeStaWatch(now); modNetWatch(now); }
    if (now - infoMs >= LINK_INFO_MS || infoMs == 0) { infoMs = msStamp(now); sendInfo(); }
    if (rebootAtMs && (int32_t)(now - rebootAtMs) >= 0) { delay(50); ESP.restart(); }
    delay(1);                                                 // let the WiFi have the core
}
