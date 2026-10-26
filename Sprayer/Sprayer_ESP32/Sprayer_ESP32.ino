/* Sprayer_ESP32.ino - sprayer section control for AgOpenWeb / AgOpenGPS on an ESP32 8-relay board.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Successor of Sprayer_Nano v0.3 (same folder). The machine-module messages follow the official
   AgOpenGPS Machine_UDP_v5 module by Brian Tee (AgOpenGPS-Official/Boards, GPL v3).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Sprayer_ESP32.ino  v0.6 (2026-10-26), not yet tested on the sprayer.
//   v0.6: all 8 relays are sections 1-8 by default (the boom has 5 now, up to 8 later).
//   v0.5: the program subnet can also be set on the page (for routers that are not 192.168.5.x).
//
// Board: "ESP32_Relay_X8" (303E32DC810): ESP32-WROOM-32E, 8 relays, 5-30 V DC in, LM2596 on board.
// The ESP32 is the standard AgOpenGPS "machine module" on the tractor's Wi-Fi network, IP <subnet>.123,
// and it has its own Wi-Fi network and web page for setup and for watching the sprayer.
//
// What it does:
//   - listens on UDP port 8888 for the program (AgOpenWeb or AgOpenGPS + AgIO):
//       239 machine data (sections, hydraulic lift, tramline, speed), 229 the 64-section version,
//       238 machine config (relay polarity, lift times), 236 pin functions (which relay does what),
//       201 subnet, 200 hello, 202 scan. The config, pin functions and subnet are kept in flash.
//   - answers on port 9999 (broadcast) like the official module: 123 to the hello, 203 to the scan,
//       237 every 200 ms (sign of life), plus 160 once a second with the sprayer readings
//       (pressure, flow, litres, master switch, sections open) for the AgOpenWeb fork's sprayer strip.
//   - no message from the program for 2 s (Wi-Fi gone, tablet frozen): every section closes.
//   - own Wi-Fi network "Sprayer-xxxx" (password "agopengps" until changed), always on: the page at
//       http://192.168.4.1/. On the tractor network: http://<subnet>.123/ or http://sprayer.local/
//   - the page shows pressure, flow, L applied, sections asked for and open, the link to the program,
//       and has the Wi-Fi settings and program subnet, the sensor calibration, a program update (.bin)
//       and a restart.
//   - IO0 button held 5 s while running: Wi-Fi settings back to the defaults (forgotten password).
//   - serial monitor (115200): "status", "wifi-reset", "total-reset", "restart".
//
// Arduino IDE: board package "esp32" by Espressif Systems, board "ESP32 Dev Module", default partition
// scheme (it has room for updates over Wi-Fi). No other libraries needed.
// First upload over the 6-pin header with a USB-serial adapter (3.3 V): see README.md.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_system.h"
#include "sPage.h"

#define FW_NAME             "Sprayer_ESP32"
#define FW_VERSION          "0.6"

// ---------------------------------------------------------------- pins (ESP32_Relay_X8 board)
const uint8_t RELAY_PIN[8] = { 32, 33, 25, 26, 27, 14, 12, 13 };   // relays 1..8, on when HIGH (checked 2026-10-25)
#define NUM_RELAYS          8
#define PRESSURE_PIN        34      // ADC1, input only: pressure sensor signal through the 10k / 15k divider
#define FLOW_PIN            35      // input only: flow meter signal through the 10k / 15k divider
#define MASTER_PIN          39      // "SVN", input only: optional master switch sense (12 V through 47k / 10k)
#define BUTTON_PIN          0       // the IO0 button: held 5 s = Wi-Fi settings back to the defaults
#define LED_PIN             23      // the board's programmable LED (set to -1 if it is not on this pin)

// ---------------------------------------------------------------- settings you may want to change
#define USE_MASTER_SWITCH   0       // 1 = MASTER_PIN reads the master switch. 0 = the switch cuts the valves' 12 V in the wiring
#define NO_DATA_TIMEOUT_MS  2000    // no message from the program for this long: close every section
#define DEFAULT_AP_PASS     "agopengps"
#define DEFAULT_HOST        "sprayer"
#define LISTEN_PORT         8888
#define SEND_PORT           9999
#define SPRAYER_PGN         0xA0    // 160: sprayer readings for the AgOpenWeb fork (Docs/PGN.md there)

// Output functions (PGN 236), as the official Machine_UDP_v5 maps them:
//   0 nothing, 1-16 section 1-16, 17 hydraulic lower, 18 hydraulic raise, 19 tramline right,
//   20 tramline left, 21 geo stop
#define FN_HYD_LOWER        17
#define FN_HYD_RAISE        18
#define FN_TRAM_RIGHT       19
#define FN_TRAM_LEFT        20
#define FN_GEO_STOP         21

// ---------------------------------------------------------------- settings kept in flash
struct Settings
{
    uint8_t subnet[3] = { 192, 168, 5 };    // the program's subnet (PGN 201); the module is <subnet>.123
    String  apSsid, apPass;                 // the module's own Wi-Fi network
    bool    staOn = false;                  // also join the tractor's Wi-Fi network (where AgOpenWeb is)
    String  staSsid, staPass;
    String  host;                           // http://<host>.local/ on the tractor network
    // machine config from the program (PGN 238)
    uint8_t raiseTime = 2, lowerTime = 4, hydEnable = 0, invertOutputs = 0;
    uint8_t user1 = 0, user2 = 0, user3 = 0, user4 = 0;
    // output functions from the program (PGN 236): relays 1-8 = sections 1-8 until the program says otherwise.
    // AgOpenWeb sends its own pin config at start (its default is pins 1-6 = sections 1-6, 7-8 = none), so
    // for 8 sections set pins 7 and 8 to Section 7 and Section 8 in AgOpenWeb (Machine, Pin config).
    uint8_t pin[24] = { 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // sensor calibration
    float   pulsesPerLitre = 450.0f;        // YF-S201: about 450. Calibrate with a bucket (README)
    float   pressureMaxBar = 10.0f;         // 0-10 bar sensor, 0.5 V = 0 bar, 4.5 V = full scale
    float   pressureDivider = 0.6f;         // the divider on the sensor signal: 15k / (10k + 15k)
} cfg;

// ---------------------------------------------------------------- state
WiFiUDP     udp;
WebServer   server(80);
DNSServer   dns;
Preferences prefs;

uint16_t sectionsReq = 0;       // sections the program asks for, section 1 = bit 0 (PGN 239 / 229)
uint16_t sectionsOn  = 0;       // sections the relays really have open
uint8_t  relayBits   = 0;       // outputs on, relay 1 = bit 0
uint8_t  uTurn = 0, hydLift = 0, tramline = 0, geoStop = 0;
float    gpsKmh = 0;
bool     isRaise = false, isLower = false;
uint32_t last239Ms = 0, lastHelloMs = 0;
bool     have239 = false, haveHello = false;
bool     linkOk = false;        // a PGN 239 arrived in the last NO_DATA_TIMEOUT_MS
bool     masterOn = true;
uint32_t count239 = 0, countHello = 0, countCrcErr = 0, countOther = 0;

float    pressureBar = 0, sensorVolts = 0;
float    flowLpm = 0, totalLitres = 0;
uint32_t totalPulses = 0;

bool     staWasUp = false;
uint32_t restartAtMs = 0;       // the web page asked for a restart
char     eventLog[12][96];      // the last events, for the page and the serial monitor
uint8_t  eventNext = 0;

// ---------------------------------------------------------------- prototypes (functions in the z*.ino files)
void settingsLoad();
void settingsSave();
void settingsResetWifi();
void wifiStart();
void wifiWatch();
void webStart();
void udpStart();
void udpPoll();
void sendSignOfLife();
void sendReadings();
void outputsInit();
void outputsAllOff();
void hydTick();
void applyOutputs();
void sensorsInit();
void readSensors();
void updateFlow();
void logEvent(const char* fmt, ...);
String defaultApSsid();
String moduleName();

// ---------------------------------------------------------------- small helpers
// AgOpenGPS checksum: sum of the bytes from the 3rd up to the one before the checksum
uint8_t pgnChecksum(const uint8_t* msg, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < length; i++) sum += msg[i];
    return sum;
}

bool staUp() { return cfg.staOn && WiFi.status() == WL_CONNECTED; }

void logEvent(const char* fmt, ...)
{
    char* b = eventLog[eventNext];
    const uint32_t s = millis() / 1000;
    int n = snprintf(b, sizeof(eventLog[0]), "%lu:%02lu ", (unsigned long)(s / 60), (unsigned long)(s % 60));
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b + n, sizeof(eventLog[0]) - n, fmt, ap);
    va_end(ap);
    Serial.println(b);
    eventNext = (eventNext + 1) % 12;
}

static void ledSet(bool on)
{
#if LED_PIN >= 0
    digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}

// LED: steady = on the tractor network and talking to the program; slow blink = on the network,
// no program; fast blink = not on the tractor network
static void ledTick()
{
    const uint32_t ms = millis();
    if (staUp() && linkOk)      ledSet(true);
    else if (staUp())           ledSet((ms / 500) % 2);
    else                        ledSet((ms / 125) % 2);
}

static void printStatus()
{
    Serial.printf("%s v%s  up %lu s  heap %u\n", FW_NAME, FW_VERSION, (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap());
    Serial.printf("Own network %s at 192.168.4.1, %d device(s)\n", cfg.apSsid.c_str(), WiFi.softAPgetStationNum());
    if (cfg.staOn) Serial.printf("Tractor network %s: %s, IP %s\n", cfg.staSsid.c_str(), staUp() ? "joined" : "not joined", WiFi.localIP().toString().c_str());
    else Serial.println("Tractor network: not set (web page > Wi-Fi)");
    Serial.printf("Program: %s | sections asked %u open %u | %.1f bar | %.1f L/min | %.1f L | master %s\n",
                  linkOk ? "talking" : "silent", sectionsReq, sectionsOn, pressureBar, flowLpm, totalLitres, masterOn ? "on" : "off");
}

static void serialCommands()
{
    static char line[32];
    static uint8_t n = 0;
    while (Serial.available())
    {
        const char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (n < sizeof(line) - 1) line[n++] = c; continue; }
        line[n] = 0; n = 0;
        if      (!strcmp(line, "status"))      printStatus();
        else if (!strcmp(line, "restart"))     { logEvent("restart (serial)"); restartAtMs = millis() + 200; }
        else if (!strcmp(line, "wifi-reset"))  { settingsResetWifi(); logEvent("Wi-Fi settings back to the defaults (serial)"); restartAtMs = millis() + 200; }
        else if (!strcmp(line, "total-reset")) { totalLitres = 0; totalPulses = 0; logEvent("total set to 0 (serial)"); }
        else if (n == 0 && line[0]) Serial.println("commands: status, wifi-reset, total-reset, restart");
    }
}

// The IO0 button held for 5 s while running: Wi-Fi settings back to the defaults
static void buttonTick()
{
    static uint32_t downSince = 0;
    if (digitalRead(BUTTON_PIN) == HIGH) { downSince = 0; return; }
    if (downSince == 0) { downSince = millis(); return; }
    if (millis() - downSince > 5000 && restartAtMs == 0)
    {
        settingsResetWifi();
        logEvent("IO0 held 5 s: Wi-Fi settings back to the defaults, restarting");
        for (int i = 0; i < 10; i++) { ledSet(i % 2); delay(100); }
        restartAtMs = millis() + 200;
    }
}

// ---------------------------------------------------------------- setup and loop
void setup()
{
    outputsInit();                  // relays off straight away (valves closed)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
#if LED_PIN >= 0
    pinMode(LED_PIN, OUTPUT);
#endif
    Serial.begin(115200);
    delay(100);
    Serial.printf("\n%s v%s\n", FW_NAME, FW_VERSION);

    settingsLoad();
    sensorsInit();
    wifiStart();
    udpStart();
    webStart();
    logEvent("ready: module IP %u.%u.%u.123 on the tractor network, page at 192.168.4.1", cfg.subnet[0], cfg.subnet[1], cfg.subnet[2]);
}

void loop()
{
    static uint32_t last50 = 0, last200 = 0, last1000 = 0;
    const uint32_t now = millis();

    udpPoll();                      // messages from the program
    server.handleClient();          // the web page
    dns.processNextRequest();       // captive page on the own network
    serialCommands();
    buttonTick();

    if (now - last50 >= 50)         // 20 times a second
    {
        last50 = now;
        readSensors();
        applyOutputs();
        ledTick();
    }

    if (now - last200 >= 200)       // 5 times a second, like the official module
    {
        last200 = now;
        hydTick();
        sendSignOfLife();
    }

    if (now - last1000 >= 1000)     // once a second
    {
        last1000 = now;
        updateFlow();
        sendReadings();
        wifiWatch();
    }

    if (restartAtMs && (int32_t)(now - restartAtMs) >= 0)
    {
        outputsAllOff();
        delay(100);
        ESP.restart();
    }
}
