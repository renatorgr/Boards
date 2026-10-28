/* Seeder_ESP32.ino - seed drill section control for AgOpenWeb / AgOpenGPS on the Relay24_ESP32 board (24 relays)
   or a KinCony KC868-A32 (32 relays).
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Made from Sprayer_ESP32 v0.8 (folder Sprayer of the same repository): the seed drill has its own folder.
   The machine-module messages follow the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3).
   2026-10-28 (Claude): v0.2: the Relay24_ESP32 board (folder Relay24_ESP32) is the default board.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Seeder_ESP32.ino  v0.2 (2026-10-28), not yet run on a board.
//   v0.2: the Relay24_ESP32 board of this repository (BOARD_RELAY24, the default): 24 DPDT relays through
//         74HC595 shift registers, one per actuator, and an RTL8201F network chip. The KC868-A32 stays
//         as BOARD_KC868_A32.
//   v0.1.1: PGN 237 in the layout of the AgOpenGPS AiO v26 firmware (sections, lift, tramline, geo stop,
//         status), the sections above 16 after it (bytes 13-18).
//   v0.1: the seed drill part of Sprayer_ESP32 v0.8 on its own: the KC868-A32 only, no pressure or flow
//         sensor, no PGN 160.
//
// Boards (BOARD below):
//   - Relay24_ESP32 (folder Relay24_ESP32 of this repository): an ESP32 DevKitC V4 plugged into a board with
//     24 DPDT relays, one per section, that turn the linear actuator's polarity round themselves, and a
//     network socket (RTL8201F), 12 V in.
//   - KinCony KC868-A32: ESP32, 32 relays (10 A, COM / NO / NC) through four PCF8574 chips, a network
//     socket (LAN8720), 32 inputs, 4 analog inputs, 12 V in. Relay n drives the 12 V DPDT relay that runs
//     the linear actuator of section n (README.md).
// For the 19-section no-till seed drill.
// The ESP32 is the standard AgOpenGPS "machine module" on the tractor's network, IP <subnet>.123, through
// the network cable, and it has its own Wi-Fi network and web page for setup and for watching the seed drill.
//
// What it does:
//   - listens on UDP port 8888 for the program (AgOpenWeb or AgOpenGPS + AgIO):
//       239 machine data (sections 1-16, hydraulic lift, tramline, speed), 229 sections 1-64 (sent by
//       AgOpenWeb when the tool has more than 16 sections),
//       238 machine config (relay polarity, lift times), 236 pin functions (which relay does what),
//       201 subnet, 200 hello, 202 scan. The config, pin functions and subnet are kept in flash.
//   - relay 1 follows section 1, relay 2 section 2 and so on (the page can change the first section, or hand
//       the relays back to the program's pin config, which only knows sections 1-16).
//   - answers on port 9999 (broadcast) like the official module: 123 to the hello, 203 to the scan,
//       237 every 200 ms (what the outputs do, as the AiO v26 sends it, plus the sections above 16: the AiO
//       Teensy's Machine tab shows them all).
//   - no message from the program for 2 s (cable or Wi-Fi gone, tablet frozen): every section closes.
//   - own Wi-Fi network "Seeder-xxxx" (password "agopengps" until changed), always on: the page at
//       http://192.168.4.1/. On the tractor network: http://<subnet>.123/ or http://seeder.local/
//   - the page shows the sections asked for and open, the relays, the link to the program and the network,
//       and has the network settings and program subnet, the first section the relays follow, a program
//       update (.bin) and a restart.
//   - IO0 (BOOT) button held 5 s while running: Wi-Fi settings back to the defaults (forgotten password).
//       On the Relay24_ESP32 only while the network cable is not in use (GPIO 0 carries its clock).
//   - serial monitor (115200): "status", "wifi-reset", "restart".
//
// Arduino IDE: board package "esp32" by Espressif Systems (version 2 or 3), board "ESP32 Dev Module",
// default partition scheme (it has room for updates over the network). No other libraries needed.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ETH.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "dPage.h"

#define FW_NAME             "Seeder_ESP32"
#define FW_VERSION          "0.2"

// ---------------------------------------------------------------- board and pins
// BOARD says which board this is (change the line below, or compile with -DBOARD=...):
//   BOARD_RELAY24     the Relay24_ESP32 board of this repository: 24 DPDT relays, network cable
//   BOARD_KC868_A32   the KinCony KC868-A32: 32 relays, each switching an external DPDT relay, network cable
#define BOARD_RELAY24       1
#define BOARD_KC868_A32     2
#ifndef BOARD
#define BOARD               BOARD_RELAY24
#endif

#if BOARD == BOARD_RELAY24
// The relays hang on three 74HC595 shift registers in a chain (data GPIO 13, clock GPIO 16, latch GPIO 32)
// through ULN2803A drivers: U1 = relays 1-8, U2 = 9-16, U3 = 17-24, output QA of each = its first relay.
// A HIGH output closes the relay. Output enable (GPIO 4, active low) has a pull-up on the board, so every
// relay stays off until the program has shifted in zeros and pulls it low.
// The network chip is an RTL8201F (address 0, MDC GPIO 23, MDIO GPIO 18). It runs on its own crystal and
// sends the 50 MHz network clock into GPIO 0; a pull-down on its reset (GPIO 33) keeps it quiet until the
// program starts it, so the ESP32 still boots and takes uploads normally. (Relay24_ESP32/README.md.)
#define BOARD_NAME          "Relay24_ESP32"
#define NUM_RELAYS          24
#define RELAY_ON_HIGH       1       // the relay closes when its 74HC595 output is HIGH
#define SR_DATA             13      // 74HC595 SER
#define SR_CLOCK            16      // 74HC595 SRCLK
#define SR_LATCH            32      // 74HC595 RCLK
#define SR_ENABLE           4       // 74HC595 OE, LOW = outputs on
#define LAN_PHY             ETH_PHY_RTL8201
#define LAN_PHY_ADDR        0
#define LAN_POWER           33      // the network chip's reset: LOW = held in reset
#define LAN_MDC             23
#define LAN_MDIO            18
#define LAN_CLOCK           ETH_CLOCK_GPIO0_IN
#define BUTTON_PIN          0       // the DevKitC's BOOT button
#define BUTTON_ON_LAN_CLOCK 1       // GPIO 0 is also the network clock (buttonTick)
#elif BOARD == BOARD_KC868_A32
// The relays hang on I2C bus A (SDA GPIO 15, SCL GPIO 13): PCF8574 0x24 = relays 1-8, 0x25 = 9-16,
// 0x21 = 17-24, 0x22 = 25-32, bit 0 = the first relay of the eight. A PCF8574 pin LOW closes the relay.
// The network chip is a LAN8720 (MDC GPIO 23, MDIO GPIO 18, clock out on GPIO 17, address 0).
// (KinCony "KC868-A32 ESP32 I/O pin define", kincony.com/forum, thread 1771.)
#define BOARD_NAME          "KC868-A32"
#define NUM_RELAYS          32
#define RELAY_ON_HIGH       0       // the relay closes when its PCF8574 pin is LOW
#define RELAY_SDA           15
#define RELAY_SCL           13
const uint8_t RELAY_CHIP[4] = { 0x24, 0x25, 0x21, 0x22 };
#define LAN_PHY             ETH_PHY_LAN8720
#define LAN_PHY_ADDR        0
#define LAN_POWER           -1      // no power pin for the network chip
#define LAN_MDC             23
#define LAN_MDIO            18
#define LAN_CLOCK           ETH_CLOCK_GPIO17_OUT
#define BUTTON_PIN          0       // the IO0 (download) button
#define BUTTON_ON_LAN_CLOCK 0
#else
#error "BOARD: choose BOARD_RELAY24 or BOARD_KC868_A32"
#endif

// ---------------------------------------------------------------- settings you may want to change
#define NO_DATA_TIMEOUT_MS  2000    // no message from the program for this long: close every section
#define DEFAULT_AP_PASS     "agopengps"
#define DEFAULT_HOST        "seeder"
#define MAX_SECTIONS        64      // AgOpenWeb has up to 64 sections (PGN 229)
#define LISTEN_PORT         8888
#define SEND_PORT           9999

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
    uint8_t firstSection = 1;               // relay 1 = this section, then in a row (0 = the program's pin config)
    String  apSsid, apPass;                 // the module's own Wi-Fi network
    bool    staOn = false;                  // the tractor's Wi-Fi network instead of the network cable
    String  staSsid, staPass;
    String  host;                           // http://<host>.local/ on the tractor network
    // machine config from the program (PGN 238)
    uint8_t raiseTime = 2, lowerTime = 4, hydEnable = 0, invertOutputs = 0;
    uint8_t user1 = 0, user2 = 0, user3 = 0, user4 = 0;
    // output functions from the program (PGN 236), used only when firstSection is 0
    uint8_t pin[24] = { 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
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
bool     linkOk = false;        // a PGN 239 or 229 arrived in the last NO_DATA_TIMEOUT_MS
uint32_t count239 = 0, count229 = 0, countHello = 0, countCrcErr = 0, countOther = 0;

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
bool ethTried();
void ethHoldReset();
IPAddress moduleIp();
void webStart();
void udpStart();
void udpPoll();
void sendSignOfLife();
void outputsInit();
void outputsAllOff();
void hydTick();
void applyOutputs();
uint8_t relayFn(uint8_t i);
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
bool netUp() { return staUp() || ethUp(); }       // on the tractor network, over the cable or Wi-Fi

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

static void printStatus()
{
    Serial.printf("%s v%s  up %lu s  heap %u\n", FW_NAME, FW_VERSION, (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap());
    Serial.printf("Own network %s at 192.168.4.1, %d device(s)\n", cfg.apSsid.c_str(), WiFi.softAPgetStationNum());
    if (cfg.staOn) Serial.printf("Tractor Wi-Fi %s: %s, IP %s\n", cfg.staSsid.c_str(), staUp() ? "joined" : "not joined", WiFi.localIP().toString().c_str());
    else Serial.printf("Network cable: %s, IP %s\n", ethUp() ? "connected" : "no link", moduleIp().toString().c_str());
    Serial.printf("Program: %s | sections asked %08lX%08lX open %08lX%08lX | relays %08lX\n",
                  linkOk ? "talking" : "silent", (unsigned long)(sectionsReq >> 32), (unsigned long)sectionsReq,
                  (unsigned long)(sectionsOn >> 32), (unsigned long)sectionsOn, (unsigned long)relayBits);
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
        else if (line[0]) Serial.println("commands: status, wifi-reset, restart");
    }
}

// The IO0 button held for 5 s while running: Wi-Fi settings back to the defaults
static void buttonTick()
{
    static uint32_t downSince = 0;
    if (BUTTON_ON_LAN_CLOCK && ethTried()) return;          // GPIO 0 carries the network clock: not a button now
    if (digitalRead(BUTTON_PIN) == HIGH) { downSince = 0; return; }
    if (downSince == 0) { downSince = millis(); return; }
    if (millis() - downSince > 5000 && restartAtMs == 0)
    {
        settingsResetWifi();
        logEvent("IO0 held 5 s: Wi-Fi settings back to the defaults, restarting");
        restartAtMs = millis() + 200;
    }
}

// ---------------------------------------------------------------- setup and loop
void setup()
{
    outputsInit();                  // relays off straight away (every row shut)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);
    delay(100);
    Serial.printf("\n%s v%s\n", FW_NAME, FW_VERSION);

    settingsLoad();
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
        applyOutputs();
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
        wifiWatch();
    }

    if (restartAtMs && (int32_t)(now - restartAtMs) >= 0)
    {
        outputsAllOff();
        ethHoldReset();             // the network chip quiet again, as at power-on
        delay(100);
        ESP.restart();
    }
}
