/* Sprayer_ESP32.ino - sprayer section control for AgOpenWeb / AgOpenGPS on an ESP32 8-relay board.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Successor of Sprayer_Nano v0.3 (same folder). The machine-module messages follow the official
   AgOpenGPS Machine_UDP_v5 module by Brian Tee (AgOpenGPS-Official/Boards, GPL v3).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Sprayer_ESP32.ino  v0.8 (2026-10-27), not yet tested on the sprayer or the seed drill.
//   v0.8: more than 16 sections (PGN 229, up to 64) and the KinCony KC868-A32 board (32 relays, network
//         cable): BOARD_KC868_A32, for the 19-section seed drill. Relays can follow the sections in a row
//         ("first section" on the page), the module's IP can end in another number than 123 (a second module).
//   v0.7.2: the page says which sensors the machine has (pressure, flow, both or none). With none (a seed
//           drill) the module sends no PGN 160, and its page, AgOpenWeb and the AiO page show no sprayer readings.
//   v0.7.1: sensor dividers from a common resistor kit (no 15k): pressure 10k / 10k (0.5, the new
//           default on the page), flow 10k / 22k.
//   v0.7: also runs on a plain ESP32 dev module with relay modules (up to 16 relays): BOARD_RELAY_X8 0.
//   v0.6: all 8 relays are sections 1-8 by default (the boom has 5 now, up to 8 later).
//   v0.5: the program subnet can also be set on the page (for routers that are not 192.168.5.x).
//
// Board: "ESP32_Relay_X8" (303E32DC810): ESP32-WROOM-32E, 8 relays, 5-30 V DC in, LM2596 on board.
//   Or any ESP32 dev module with relay modules, up to 16 relays (BOARD_DEVKIT below).
//   Or the KinCony KC868-A32: ESP32, 32 relays, network cable, 12 V (BOARD_KC868_A32 below).
// The ESP32 is the standard AgOpenGPS "machine module" on the tractor's network, IP <subnet>.123,
// and it has its own Wi-Fi network and web page for setup and for watching the sprayer (or the seed drill).
//
// What it does:
//   - listens on UDP port 8888 for the program (AgOpenWeb or AgOpenGPS + AgIO):
//       239 machine data (sections 1-16, hydraulic lift, tramline, speed), 229 sections 1-64 (sent by
//       AgOpenWeb when the tool has more than 16 sections),
//       238 machine config (relay polarity, lift times), 236 pin functions (which relay does what),
//       201 subnet, 200 hello, 202 scan. The config, pin functions and subnet are kept in flash.
//   - answers on port 9999 (broadcast) like the official module: 123 to the hello, 203 to the scan,
//       237 every 200 ms (sign of life), plus 160 once a second with the sprayer readings
//       (pressure, flow, litres, master switch, sections open) for the AgOpenWeb fork's sprayer strip;
//       a machine without sensors (set on the page: a seed drill) sends no 160.
//   - no message from the program for 2 s (Wi-Fi or cable gone, tablet frozen): every section closes.
//   - own Wi-Fi network "Sprayer-xxxx" (password "agopengps" until changed), always on: the page at
//       http://192.168.4.1/. On the tractor network: http://<subnet>.123/ or http://sprayer.local/
//       (on the KC868-A32 "Seeder-xxxx" and http://seeder.local/, see MODULE_KIND)
//   - the page shows pressure, flow, L applied (only for the sensors fitted), sections asked for and open,
//       the link to the program, and has the Wi-Fi settings and program subnet, the sensors fitted and their
//       calibration, a program update (.bin) and a restart.
//   - IO0 button held 5 s while running: Wi-Fi settings back to the defaults (forgotten password).
//   - serial monitor (115200): "status", "wifi-reset", "total-reset", "restart".
//
// Arduino IDE: board package "esp32" by Espressif Systems, board "ESP32 Dev Module", default partition
// scheme (it has room for updates over Wi-Fi). No other libraries needed.
// First upload over the 6-pin header with a USB-serial adapter (3.3 V): see README.md.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_system.h"
#include "sPage.h"

#define FW_NAME             "Sprayer_ESP32"
#define FW_VERSION          "0.8"

// ---------------------------------------------------------------- board and pins
// BOARD says which board this is (change the line below, or compile with -DBOARD=...):
//   BOARD_RELAY_X8   the ESP32_Relay_X8 board: 8 relays on it, Wi-Fi
//   BOARD_DEVKIT     any ESP32 dev module (30 or 38 pins) with one or two relay modules wired to the pins in
//                    RELAY_PIN below: relay 1 is the first pin. Keep the list as long as the relays you have
//                    (8 or 16)
//   BOARD_KC868_A32  the KinCony KC868-A32: 32 relays on it (through four PCF8574 chips), network cable
//                    (or Wi-Fi), 12 V
#define BOARD_RELAY_X8      1
#define BOARD_DEVKIT        2
#define BOARD_KC868_A32     3
#ifndef BOARD
#define BOARD               BOARD_RELAY_X8
#endif

#if BOARD == BOARD_RELAY_X8
#define BOARD_NAME          "ESP32_Relay_X8"
const uint8_t RELAY_PIN[] = { 32, 33, 25, 26, 27, 14, 12, 13 };   // relays 1..8 (checked 2026-10-25)
#define NUM_RELAYS          8
#define RELAY_ON_HIGH       1       // this board's relays close when the pin is HIGH
#define LED_PIN             23      // the board's programmable LED (set to -1 if it is not on this pin)
#define USE_ETHERNET        0
#elif BOARD == BOARD_DEVKIT
#define BOARD_NAME          "ESP32 dev module"
// Free at boot and safe as outputs on a dev module (0, 2, 12, 15 are left out: they set the boot mode, and
// 6-11 go to the flash). The sensors stay on 34, 35 and 39, which are input-only pins.
const uint8_t RELAY_PIN[] = { 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 };   // relays 1..16
#define NUM_RELAYS          ((uint8_t)(sizeof(RELAY_PIN) / sizeof(RELAY_PIN[0])))
#define RELAY_ON_HIGH       0       // the usual relay modules (optocoupler inputs) close when the pin is LOW
#define LED_PIN             2       // the dev module's blue LED
#define USE_ETHERNET        0
#elif BOARD == BOARD_KC868_A32
// The relays hang on I2C bus A (SDA GPIO 15, SCL GPIO 13): PCF8574 0x24 = relays 1-8, 0x25 = 9-16,
// 0x21 = 17-24, 0x22 = 25-32, bit 0 = the first relay of the eight. A PCF8574 pin LOW closes the relay.
// The network chip is a LAN8720 (MDC GPIO 23, MDIO GPIO 18, clock out on GPIO 17, address 0).
// (KinCony "KC868-A32 ESP32 I/O pin define", kincony.com/forum, thread 1771.)
// The four analog inputs (0-5 V) are GPIO 39 (A1), 34 (A2), 36 (A3) and 35 (A4).
#define BOARD_NAME          "KC868-A32"
#define NUM_RELAYS          32
#define RELAY_ON_HIGH       0
#define LED_PIN             -1      // no LED for the program on this board
#define RELAY_SDA           15
#define RELAY_SCL           13
const uint8_t RELAY_CHIP[4] = { 0x24, 0x25, 0x21, 0x22 };
#define USE_ETHERNET        1
#define LAN_PHY_ADDR        0
#define LAN_POWER           -1      // no power pin for the network chip
#define LAN_MDC             23
#define LAN_MDIO            18
#else
#error "BOARD: choose BOARD_RELAY_X8, BOARD_DEVKIT or BOARD_KC868_A32"
#endif

#if USE_ETHERNET
#include <ETH.h>
#endif

// The name the module goes by: its own Wi-Fi network (Sprayer-xxxx), http://sprayer.local/ and the page's title
#ifndef MODULE_KIND
#if BOARD == BOARD_KC868_A32
#define MODULE_KIND         "Seeder"        // Renato's 19-section seed drill
#define DEFAULT_HOST        "seeder"
#else
#define MODULE_KIND         "Sprayer"
#define DEFAULT_HOST        "sprayer"
#endif
#endif
#ifndef DEFAULT_HOST
#define DEFAULT_HOST        "machine"
#endif

// Relay 1 follows this section and the next relays the next sections (0 = the relays follow the program's
// pin config, PGN 236). The program's pin config only knows sections 1-16, so a board with more relays
// than that starts at section 1. Can be changed on the page.
#define DEFAULT_FIRST_SECTION (NUM_RELAYS > 16 ? 1 : 0)

// Sensors. On the KC868-A32, GPIO 34 and 35 are its analog inputs A2 and A4 (0-5 V, the board has its own
// divider on them): the seed drill has no sensors (cfg.sensors = 0 below), so nothing is wired there.
#define PRESSURE_PIN        34      // ADC1, input only: pressure sensor signal through the 10k / 10k divider
#define FLOW_PIN            35      // input only: flow meter signal through the 10k / 22k divider
#define MASTER_PIN          39      // "SVN", input only: optional master switch sense (12 V through 47k / 10k)
#define BUTTON_PIN          0       // the IO0 button: held 5 s = Wi-Fi settings back to the defaults

// Sensors fitted (cfg.sensors, set on the page): a sprayer has both, a seed drill none
#define SENSOR_PRESSURE     1
#define SENSOR_FLOW         2
#if BOARD == BOARD_KC868_A32
#define DEFAULT_SENSORS     0                               // the seed drill
#else
#define DEFAULT_SENSORS     (SENSOR_PRESSURE | SENSOR_FLOW) // the sprayer
#endif

// ---------------------------------------------------------------- settings you may want to change
#define USE_MASTER_SWITCH   0       // 1 = MASTER_PIN reads the master switch. 0 = the switch cuts the valves' 12 V in the wiring
#define NO_DATA_TIMEOUT_MS  2000    // no message from the program for this long: close every section
#define DEFAULT_AP_PASS     "agopengps"
#define MAX_SECTIONS        64      // AgOpenWeb has up to 64 sections (PGN 229)
#define LISTEN_PORT         8888
#define SEND_PORT           9999
#define SPRAYER_PGN         0xA0    // 160: sprayer readings for the AgOpenWeb fork (Docs/PGN.md there)

// Output functions in the program's pin config (PGN 236), as the official Machine_UDP_v5 maps them:
//   0 nothing, 1-16 section 1-16, 17 hydraulic lower, 18 hydraulic raise, 19 tramline right,
//   20 tramline left, 21 geo stop
#define PIN_HYD_LOWER       17
#define PIN_GEO_STOP        21
// What a relay does inside the module (relayFn(), the page): 0 nothing, 1-64 that section, FN_OTHER + 1-5 the
// program's other functions (hydraulic lower, hydraulic raise, tramline right, tramline left, geo stop)
#define FN_OTHER            100

// ---------------------------------------------------------------- settings kept in flash
struct Settings
{
    uint8_t subnet[3] = { 192, 168, 5 };    // the program's subnet (PGN 201); the module is <subnet>.<ipLast>
    uint8_t ipLast = 123;                   // 123 = the machine module; another number for a second module
    uint8_t firstSection = DEFAULT_FIRST_SECTION;   // relay 1 = this section, then in a row (0 = pin config)
    String  apSsid, apPass;                 // the module's own Wi-Fi network
    bool    staOn = false;                  // also join the tractor's Wi-Fi network (where AgOpenWeb is); on a
                                            // board with a network cable, Wi-Fi instead of the cable
    String  staSsid, staPass;
    String  host;                           // http://<host>.local/ on the tractor network
    // machine config from the program (PGN 238)
    uint8_t raiseTime = 2, lowerTime = 4, hydEnable = 0, invertOutputs = 0;
    uint8_t user1 = 0, user2 = 0, user3 = 0, user4 = 0;
    // output functions from the program (PGN 236): relays 1-8 = sections 1-8 until the program says otherwise.
    // AgOpenWeb sends its own pin config at start (its default is pins 1-6 = sections 1-6, 7-8 = none), so
    // for 8 sections set pins 7 and 8 to Section 7 and Section 8 in AgOpenWeb (Machine, Pin config).
    uint8_t pin[24] = { 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // sensors fitted and their calibration
    uint8_t sensors = DEFAULT_SENSORS;      // none (a seed drill): no PGN 160, no readings on the page
    float   pulsesPerLitre = 450.0f;        // YF-S201: about 450. Calibrate with a bucket (README)
    float   pressureMaxBar = 10.0f;         // 0-10 bar sensor, 0.5 V = 0 bar, 4.5 V = full scale
    float   pressureDivider = 0.5f;         // the divider on the pressure signal: 10k / (10k + 10k)
} cfg;

// ---------------------------------------------------------------- state
WiFiUDP     udp;
WebServer   server(80);
DNSServer   dns;
Preferences prefs;

uint64_t sectionsReq = 0;       // sections the program asks for, section 1 = bit 0 (PGN 239, 17-64 from PGN 229)
uint64_t sectionsOn  = 0;       // sections the relays really have open
uint32_t relayBits   = 0;       // outputs on, relay 1 = bit 0
uint16_t sections239 = 0;       // sections 1-16 as the last PGN 239 (or 229) gave them
uint64_t sections229 = 0;       // sections 1-64 as the last PGN 229 gave them
uint8_t  secTop = 0;            // the highest section asked for since the start (the page shows that many)
uint8_t  uTurn = 0, hydLift = 0, tramline = 0, geoStop = 0;
float    gpsKmh = 0;
bool     isRaise = false, isLower = false;
uint32_t last239Ms = 0, last229Ms = 0, lastHelloMs = 0;
bool     have239 = false, have229 = false, haveHello = false;
bool     linkOk = false;        // a PGN 239 arrived in the last NO_DATA_TIMEOUT_MS
bool     masterOn = true;
uint32_t count239 = 0, count229 = 0, countHello = 0, countCrcErr = 0, countOther = 0;

float    pressureBar = 0, sensorVolts = 0;
float    flowLpm = 0, totalLitres = 0;
uint32_t totalPulses = 0;

bool     staWasUp = false, ethWasUp = false;
uint32_t restartAtMs = 0;       // the web page asked for a restart
char     eventLog[12][96];      // the last events, for the page and the serial monitor
uint8_t  eventNext = 0;

// ---------------------------------------------------------------- prototypes (functions in the z*.ino files)
void settingsLoad();
void settingsSave();
void settingsResetWifi();
void wifiStart();
void wifiWatch();
bool ethUp();
IPAddress moduleIp();
void webStart();
void udpStart();
void udpPoll();
void sendSignOfLife();
void sendReadings();
void outputsInit();
void outputsAllOff();
void hydTick();
void applyOutputs();
uint8_t relayFn(uint8_t i);
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
bool netUp() { return staUp() || ethUp(); }       // on the tractor network, over Wi-Fi or the cable

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
    if (netUp() && linkOk)      ledSet(true);
    else if (netUp())           ledSet((ms / 500) % 2);
    else                        ledSet((ms / 125) % 2);
}

static void printStatus()
{
    Serial.printf("%s v%s  up %lu s  heap %u\n", FW_NAME, FW_VERSION, (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap());
    Serial.printf("Own network %s at 192.168.4.1, %d device(s)\n", cfg.apSsid.c_str(), WiFi.softAPgetStationNum());
    if (cfg.staOn) Serial.printf("Tractor network %s: %s, IP %s\n", cfg.staSsid.c_str(), staUp() ? "joined" : "not joined", WiFi.localIP().toString().c_str());
    else if (USE_ETHERNET) Serial.printf("Network cable: %s, IP %s\n", ethUp() ? "connected" : "not connected", moduleIp().toString().c_str());
    else Serial.println("Tractor network: not set (web page > Wi-Fi)");
    Serial.printf("Program: %s | sections asked %08lX%08lX open %08lX%08lX | %.1f bar | %.1f L/min | %.1f L | master %s\n",
                  linkOk ? "talking" : "silent", (unsigned long)(sectionsReq >> 32), (unsigned long)sectionsReq,
                  (unsigned long)(sectionsOn >> 32), (unsigned long)sectionsOn, pressureBar, flowLpm, totalLitres, masterOn ? "on" : "off");
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
    outputsInit();                  // relays off straight away (valves closed, seed rows shut)
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
    logEvent("ready: module IP %u.%u.%u.%u on the tractor network, page at 192.168.4.1", cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], cfg.ipLast);
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
