/* fake_core.h - a pretend ESP32 + Arduino core, so the real Sprayer_ESP32 / Seeder_ESP32 code runs on a PC.
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Only what the two programs use is here. Everything a call changes lives in a fake::Device (pins, flash,
// network, web requests, serial text); the harness owns the devices and looks at them. fake::cur is the
// device whose code runs now: the harness sets it before it calls setup(), loop() or an interrupt.
// The bodies are in fake_core.cpp, which is linked into the harness program; the firmware itself is
// built as a shared library and loaded again at each restart, so its variables start fresh as on a board.
#pragma once

#include <cstdint>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <functional>
#include <algorithm>
#include <type_traits>

// ---------------------------------------------------------------- Arduino basics
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define PROGMEM
#define IRAM_ATTR
#define F(s) (s)
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define digitalPinToInterrupt(p) (p)
#define ESP_ARDUINO_VERSION_MAJOR 3

using std::isfinite;
using std::isnan;
using std::isinf;

template <class A, class B> inline auto min(A a, B b) -> typename std::common_type<A, B>::type { return a < b ? a : b; }
template <class A, class B> inline auto max(A a, B b) -> typename std::common_type<A, B>::type { return a > b ? a : b; }

uint32_t millis();
uint32_t micros();
void     delay(uint32_t ms);
void     pinMode(uint8_t pin, uint8_t mode);
void     digitalWrite(uint8_t pin, uint8_t level);
int      digitalRead(uint8_t pin);
uint32_t analogReadMilliVolts(uint8_t pin);
uint16_t analogRead(uint8_t pin);
enum adc_attenuation_t { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };
void     analogSetPinAttenuation(uint8_t pin, adc_attenuation_t a);
void     attachInterrupt(uint8_t pin, void (*isr)(), int mode);
inline void noInterrupts() {}
inline void interrupts() {}

// ---------------------------------------------------------------- String (the Arduino one, on std::string)
class String
{
public:
    String() {}
    String(const char* s) : s_(s ? s : "") {}
    String(const std::string& s) : s_(s) {}
    String(char c) : s_(1, c) {}
    String(int v) : s_(std::to_string(v)) {}
    String(unsigned int v) : s_(std::to_string(v)) {}
    String(long v) : s_(std::to_string(v)) {}
    String(unsigned long v) : s_(std::to_string(v)) {}
    String(long long v) : s_(std::to_string(v)) {}
    String(unsigned long long v) : s_(std::to_string(v)) {}
    String(unsigned char v) : s_(std::to_string((unsigned)v)) {}
    String(float v, unsigned int decimals = 2) { fromFloat(v, decimals); }
    String(double v, unsigned int decimals = 2) { fromFloat(v, decimals); }

    size_t      length() const { return s_.size(); }
    const char* c_str() const { return s_.c_str(); }
    char        operator[](size_t i) const { return i < s_.size() ? s_[i] : 0; }
    char&       operator[](size_t i) { return s_[i]; }
    bool        reserve(size_t n) { s_.reserve(n); return true; }
    float       toFloat() const { return (float)atof(s_.c_str()); }
    long        toInt() const { return atol(s_.c_str()); }
    void        toLowerCase() { for (auto& c : s_) c = (char)tolower((unsigned char)c); }
    bool        operator==(const String& o) const { return s_ == o.s_; }
    bool        operator!=(const String& o) const { return s_ != o.s_; }
    bool        operator==(const char* o) const { return s_ == (o ? o : ""); }
    bool        operator!=(const char* o) const { return !(*this == o); }
    String&     operator+=(const String& o) { s_ += o.s_; return *this; }
    String&     operator+=(const char* o) { s_ += o ? o : ""; return *this; }
    String&     operator+=(char c) { s_ += c; return *this; }
    const std::string& str() const { return s_; }

    friend String operator+(const String& a, const String& b) { return String(a.s_ + b.s_); }
    friend String operator+(const String& a, const char* b) { return String(a.s_ + (b ? b : "")); }
    friend String operator+(const char* a, const String& b) { return String(std::string(a ? a : "") + b.s_); }
    friend String operator+(const String& a, char c) { return String(a.s_ + c); }

private:
    void fromFloat(double v, unsigned int d)
    {
        char b[64];
        snprintf(b, sizeof(b), "%.*f", (int)d, v);
        s_ = b;
    }
    std::string s_;
};

// ---------------------------------------------------------------- IPAddress
class IPAddress
{
public:
    IPAddress() : a_{ 0, 0, 0, 0 } {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : a_{ a, b, c, d } {}
    uint8_t  operator[](int i) const { return a_[i & 3]; }
    uint8_t& operator[](int i) { return a_[i & 3]; }
    bool     operator==(const IPAddress& o) const { return memcmp(a_, o.a_, 4) == 0; }
    bool     operator!=(const IPAddress& o) const { return !(*this == o); }
    String   toString() const
    {
        char b[20];
        snprintf(b, sizeof(b), "%u.%u.%u.%u", a_[0], a_[1], a_[2], a_[3]);
        return String(b);
    }
private:
    uint8_t a_[4];
};

// ---------------------------------------------------------------- Serial
class HardwareSerial
{
public:
    void   begin(unsigned long baud);
    int    available();
    int    read();
    size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    size_t print(const char* s);
    size_t print(const String& s) { return print(s.c_str()); }
    size_t println(const char* s = "");
    size_t println(const String& s) { return println(s.c_str()); }
};
extern HardwareSerial Serial;

// ---------------------------------------------------------------- ESP
class EspClass
{
public:
    uint64_t getEfuseMac();
    uint32_t getFreeHeap() { return 180000; }
    [[noreturn]] void restart();
};
extern EspClass ESP;

namespace fake { struct Device; }

// ---------------------------------------------------------------- Wi-Fi
#define WIFI_OFF 0
#define WIFI_STA 1
#define WIFI_AP 2
#define WIFI_AP_STA 3
typedef int wifi_mode_t;
enum wl_status_t { WL_IDLE_STATUS = 0, WL_NO_SSID_AVAIL = 1, WL_SCAN_COMPLETED = 2, WL_CONNECTED = 3,
                   WL_CONNECT_FAILED = 4, WL_CONNECTION_LOST = 5, WL_DISCONNECTED = 6 };
enum wifi_auth_mode_t { WIFI_AUTH_OPEN = 0, WIFI_AUTH_WPA2_PSK = 3 };
// Wi-Fi events, as the core gives them to WiFi.onEvent() (only those the programs use); the reasons are ESP-IDF's
enum arduino_event_id_t { ARDUINO_EVENT_WIFI_STA_CONNECTED = 4, ARDUINO_EVENT_WIFI_STA_DISCONNECTED = 5,
                          ARDUINO_EVENT_WIFI_STA_GOT_IP = 7, ARDUINO_EVENT_MAX = 99 };
typedef arduino_event_id_t WiFiEvent_t;
struct wifi_event_sta_disconnected_t { uint8_t ssid[32]; uint8_t ssid_len; uint8_t bssid[6]; uint8_t reason; int8_t rssi; };
union arduino_event_info_t { wifi_event_sta_disconnected_t wifi_sta_disconnected; };
typedef arduino_event_info_t WiFiEventInfo_t;
typedef std::function<void(arduino_event_id_t, arduino_event_info_t)> WiFiEventFuncCb;
enum wifi_err_reason_t { WIFI_REASON_UNSPECIFIED = 1, WIFI_REASON_AUTH_EXPIRE = 2, WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT = 15,
                         WIFI_REASON_BEACON_TIMEOUT = 200, WIFI_REASON_NO_AP_FOUND = 201, WIFI_REASON_AUTH_FAIL = 202,
                         WIFI_REASON_ASSOC_FAIL = 203, WIFI_REASON_HANDSHAKE_TIMEOUT = 204, WIFI_REASON_CONNECTION_FAIL = 205 };
#define WIFI_SCAN_RUNNING (-1)
#define WIFI_SCAN_FAILED (-2)

class WiFiClass
{
public:
    void        persistent(bool) {}
    bool        setHostname(const char*) { return true; }
    bool        mode(int m);
    bool        softAPConfig(IPAddress ip, IPAddress gw, IPAddress mask);
    bool        softAP(const char* ssid, const char* pass);
    IPAddress   softAPIP();
    int         softAPgetStationNum();
    bool        config(IPAddress ip, IPAddress gw, IPAddress mask, IPAddress dns1 = IPAddress());
    bool        setAutoReconnect(bool) { return true; }
    int         onEvent(WiFiEventFuncCb cb, arduino_event_id_t event = ARDUINO_EVENT_MAX);
    wl_status_t begin(const char* ssid, const char* pass);
    wl_status_t status();
    IPAddress   localIP();
    int8_t      RSSI();
    int16_t     scanNetworks(bool async = false);
    int16_t     scanComplete();
    void        scanDelete();
    String      SSID(int i);
    int32_t     RSSI(int i);
    int32_t     channel(int i);
    wifi_auth_mode_t encryptionType(int i);
};
extern WiFiClass WiFi;

class WiFiUDP
{
public:
    WiFiUDP();
    uint8_t   begin(uint16_t port);
    int       parsePacket();
    int       read(uint8_t* buf, size_t len);
    IPAddress remoteIP();
    int       beginPacket(IPAddress ip, uint16_t port);
    size_t    write(const uint8_t* buf, size_t len);
    int       endPacket();
private:
    std::vector<uint8_t> out_, in_;
    IPAddress outIp_, inIp_;
    uint16_t  outPort_ = 0;
    bool      haveIn_ = false;
};

class WiFiClient
{
public:
    IPAddress localIP();
};

// ---------------------------------------------------------------- network cable
enum eth_phy_type_t { ETH_PHY_LAN8720, ETH_PHY_TLK110, ETH_PHY_RTL8201, ETH_PHY_DP83848, ETH_PHY_KSZ8041, ETH_PHY_KSZ8081 };
enum eth_clock_mode_t { ETH_CLOCK_GPIO0_IN, ETH_CLOCK_GPIO0_OUT, ETH_CLOCK_GPIO16_OUT, ETH_CLOCK_GPIO17_OUT };
class ETHClass
{
public:
    bool     begin(eth_phy_type_t type, int32_t phy_addr, int mdc, int mdio, int power, eth_clock_mode_t clock_mode);
    bool     config(IPAddress ip, IPAddress gw, IPAddress mask, IPAddress dns1 = IPAddress());
    bool     linkUp();
    uint8_t  linkSpeed() { return 100; }
};
extern ETHClass ETH;

// ---------------------------------------------------------------- web server
enum HTTPMethod { HTTP_ANY, HTTP_GET, HTTP_HEAD, HTTP_POST, HTTP_PUT, HTTP_PATCH, HTTP_DELETE, HTTP_OPTIONS };
enum HTTPUploadStatus { UPLOAD_FILE_START, UPLOAD_FILE_WRITE, UPLOAD_FILE_END, UPLOAD_FILE_ABORTED };
#define HTTP_UPLOAD_BUFLEN 1436
struct HTTPUpload
{
    HTTPUploadStatus status;
    String filename, name, type;
    size_t totalSize = 0, currentSize = 0;
    uint8_t buf[HTTP_UPLOAD_BUFLEN];
};

class WebServer
{
public:
    typedef std::function<void()> Handler;
    explicit WebServer(int port = 80);
    ~WebServer();
    void        on(const char* uri, HTTPMethod method, Handler fn);
    void        on(const char* uri, HTTPMethod method, Handler fn, Handler ufn);
    void        onNotFound(Handler fn) { notFound_ = fn; }
    void        begin() {}
    void        handleClient();
    String      arg(const char* name);
    String      arg(const String& name) { return arg(name.c_str()); }
    bool        hasArg(const char* name);
    void        send(int code, const char* type, const String& body);
    void        send(int code, const char* type, const char* body) { send(code, type, String(body)); }
    void        send_P(int code, const char* type, const char* body) { send(code, type, String(body)); }
    void        sendHeader(const String& name, const String& value, bool first = false);
    HTTPUpload& upload() { return upload_; }
    WiFiClient  client() { return WiFiClient(); }
    String      hostHeader();

    struct Route { std::string uri; HTTPMethod method; Handler fn, ufn; };
    std::vector<Route> routes_;
    Handler    notFound_;
    HTTPUpload upload_;
};

// ---------------------------------------------------------------- DNS, mDNS
enum class DNSReplyCode { NoError = 0, FormError, ServerFailure, NonExistentDomain };
class DNSServer
{
public:
    void setErrorReplyCode(DNSReplyCode) {}
    bool start(uint16_t, const char*, IPAddress) { return true; }
    void processNextRequest() {}
};
class MDNSResponder
{
public:
    bool begin(const char* host);
    void setInstanceName(const String& name);
    bool addService(const char* service, const char* proto, uint16_t port);
    bool addServiceTxt(const char* service, const char* proto, const char* key, const char* value);
};
extern MDNSResponder MDNS;

// ---------------------------------------------------------------- flash (Preferences), program update
class Preferences
{
public:
    bool   begin(const char* ns, bool readOnly = false);
    void   end();
    uint8_t getUChar(const char* key, uint8_t def = 0);
    size_t  putUChar(const char* key, uint8_t v);
    bool    getBool(const char* key, bool def = false);
    size_t  putBool(const char* key, bool v);
    float   getFloat(const char* key, float def = NAN);
    size_t  putFloat(const char* key, float v);
    String  getString(const char* key, const String& def = String());
    size_t  putString(const char* key, const String& v);
    size_t  getBytesLength(const char* key);
    size_t  getBytes(const char* key, void* buf, size_t maxLen);
    size_t  putBytes(const char* key, const void* buf, size_t len);
private:
    std::string ns_;
    bool open_ = false, readOnly_ = false;
};

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF
class UpdateClass
{
public:
    bool   begin(size_t size);
    size_t write(uint8_t* data, size_t len);
    bool   end(bool evenIfRemaining = false);
    void   abort();
    bool   hasError();
};
extern UpdateClass Update;

// ---------------------------------------------------------------- I2C (the KC868-A32's PCF8574 chips)
class TwoWire
{
public:
    bool    begin(int sda, int scl, uint32_t freq = 100000);
    void    beginTransmission(uint8_t addr);
    size_t  write(uint8_t b);
    uint8_t endTransmission(bool stop = true);
private:
    uint8_t addr_ = 0;
    std::vector<uint8_t> buf_;
};
extern TwoWire Wire;

// ---------------------------------------------------------------- ESP-IDF bits
typedef int gpio_num_t;
int gpio_set_level(gpio_num_t pin, uint32_t level);

// ---------------------------------------------------------------- the pretend board and its surroundings
namespace fake
{
struct Restart {};                          // ESP.restart(): caught by the harness, which loads the program again

struct Packet
{
    uint64_t ms = 0;                        // the harness clock when it was sent
    IPAddress src, dst;
    uint16_t srcPort = 0, dstPort = 0;
    std::vector<uint8_t> data;
};

struct PrefValue { char type = 0; std::vector<uint8_t> bytes; };

struct HttpRequest
{
    HTTPMethod method = HTTP_GET;
    std::string uri, host;
    std::map<std::string, std::string> args;
    bool fromAp = true;                     // asked on the module's own network (else the tractor network)
    std::vector<uint8_t> file;              // an upload (POST /api/update)
    std::string fileName;
};
struct HttpResponse
{
    bool done = false;
    int code = 0;
    std::string type, body;
    std::map<std::string, std::string> headers;
};

// What the tractor network looks like (set by the harness, shared by every device)
struct Network
{
    bool        wifiOn = true;              // the tractor's Wi-Fi router is on
    std::string ssid = "Tractor", pass = "tractor123";
    uint8_t     subnet[3] = { 192, 168, 5 };
    bool        dhcp = true;
};
extern Network net;
extern uint64_t clockMs;                    // the harness clock

struct Device
{
    std::string label;
    uint64_t mac = 0x0000A1B2C3D4E5F6ull;
    uint64_t bootAt = 0;                    // clockMs at the last start
    uint32_t millisOffset = 0;              // millis() = clockMs - bootAt + millisOffset (to try the 49-day wrap)

    // pins
    uint8_t  mode[40] = {};
    uint8_t  level[40] = {};
    uint8_t  inputLevel[40];                // what digitalRead gives on an input (HIGH = button not pressed)
    uint32_t analogMv[40] = {};
    void   (*isr[40])() = {};
    std::function<void(uint8_t pin, uint8_t level)> onWrite;   // the harness's board model (74HC595 chain)
    uint32_t pinWrites = 0;

    // I2C chips: address -> the byte last written; only those present answer
    std::map<uint8_t, uint8_t> i2c;
    std::map<uint8_t, bool>    i2cPresent;

    // flash
    std::map<std::string, std::map<std::string, PrefValue>> prefs;
    uint32_t flashWrites = 0;               // putX calls that changed something (flash wear)

    // Wi-Fi and the cable
    int         wifiMode = WIFI_OFF;
    std::string apSsid, apPass;
    IPAddress   apIp { 192, 168, 4, 1 };    // the own network's address (softAPConfig; the ESP32's own is 192.168.4.1)
    bool        apOn = false;
    int         apClients = 1;              // the phone on the module's own network
    bool        staBegun = false;
    std::string staSsid, staPass;
    uint64_t    staBeginAt = 0;
    std::vector<std::pair<arduino_event_id_t, WiFiEventFuncCb>> wifiEvents;   // WiFi.onEvent() (cleared before the program goes)
    bool        staLinked = false;          // the last event said joined
    uint64_t    staEventAt = 0;             // the last "disconnected" event (one every 3 s while it tries)
    bool        staStatic = false;
    IPAddress   staIp, staGw, staMask;
    int         scanState = -2;             // -2 none, -1 running
    uint64_t    scanAt = 0;
    bool        ethChip = false;            // the board has a network chip that starts
    eth_phy_type_t ethChipType = ETH_PHY_RTL8201;
    bool        cableIn = false;            // a network cable to the tractor's switch
    bool        ethBegun = false, ethOk = false;
    IPAddress   ethIp;
    bool        mdnsOn = false;
    std::string mdnsHost, mdnsInstance;

    // UDP
    uint16_t    udpPort = 0;
    std::deque<Packet> rx;                  // to the module's port, waiting for parsePacket()
    std::vector<Packet> tx;                 // sent by the module (the harness takes them away)

    // web
    WebServer*  web = nullptr;
    HttpRequest req;
    bool        reqPending = false;
    HttpResponse resp;

    // program update
    bool        updBegun = false, updError = false;
    size_t      updBytes = 0;
    std::vector<uint8_t> updImage;
    bool        updDone = false;            // a whole image was accepted

    // serial
    std::string serialOut, serialIn;

    // set by ESP.restart()
    uint32_t restarts = 0;

    Device() { for (auto& l : inputLevel) l = HIGH; }

    // helpers for the harness
    bool      staUp() const;                // joined the tractor's Wi-Fi
    void      tickWifi();                   // the Wi-Fi events the core would give the program (before each loop())
    bool      onTractorNet() const;         // Wi-Fi joined or cable up
    IPAddress tractorIp() const;            // its IP on the tractor network (0.0.0.0 when not there)
    bool      sameNumbers() const;          // the own network and the tractor's Wi-Fi have the same first three numbers:
                                            // the chip then sends what is for the tractor network out on its own one
};
extern Device* cur;
}
