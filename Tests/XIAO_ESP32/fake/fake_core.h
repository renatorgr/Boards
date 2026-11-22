/* fake_core.h - a pretend ESP32 + Arduino core, so the real XIAO program (v4.5_XIAO_ESP32) runs on a PC.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The web server follows what WebServer.cpp / Parsing.cpp of the ESP32 Arduino core 3.3.12 do (Espressif, LGPL 2.1),
   the update follows its Updater.cpp, the serial port its esp32-hal-uart.c: the same waits, limits and order.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Only what the XIAO program uses is here. Everything a call changes lives in a fake::Device (flash, WiFi, the
// serial port to the Teensy, UDP sockets, browser connections, the watchdog); the harness owns the device and looks
// at it. Time is the harness clock (fake::clockMs): it moves only in fake::tick(), which delay() and yield() call,
// and every millisecond the world around the XIAO takes one step (fake::onMs). So a wait inside the program is a
// wait in pretend time too: the Teensy goes on sending, the tablet goes on sending, the watchdog goes on counting.
// The bodies are in fake_core.cpp, linked into the harness program; the XIAO program itself is built as a shared
// library and loaded again at each restart, so its variables start fresh as on a board.
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
#include <memory>
#include <functional>
#include <algorithm>

// ---------------------------------------------------------------- Arduino basics
typedef uint8_t byte;
typedef bool boolean;
#define PROGMEM
#define PGM_P const char*
#define F(s) (s)
#define SERIAL_8N1 0x800001c
#define ARDUINO_USB_CDC_ON_BOOT 1
#define ESP_ARDUINO_VERSION_MAJOR 3
#ifdef XIAO_KIT_S3
#define D6 43
#define D7 44
#else
#define D6 21
#define D7 20
#endif

using std::isfinite;
using std::isnan;
using std::isinf;

uint32_t millis();
uint32_t micros();
void     delay(uint32_t ms);
void     yield();
float    temperatureRead();
void     enableLoopWDT();
void     feedLoopWDT();

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

    size_t      length() const { return s_.size(); }
    const char* c_str() const { return s_.c_str(); }
    char        operator[](size_t i) const { return i < s_.size() ? s_[i] : 0; }
    char&       operator[](size_t i) { return s_[i]; }
    long        toInt() const { return atol(s_.c_str()); }
    void        toLowerCase() { for (auto& c : s_) c = (char)tolower((unsigned char)c); }
    int         indexOf(const char* t) const { const size_t p = s_.find(t ? t : ""); return p == std::string::npos ? -1 : (int)p; }
    int         indexOf(const String& t) const { return indexOf(t.c_str()); }
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

private:
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
    uint32_t raw() const { uint32_t v; memcpy(&v, a_, 4); return v; }      // as in a sockaddr: the first number in the low byte
    static IPAddress fromRaw(uint32_t v) { IPAddress i; memcpy(i.a_, &v, 4); return i; }
    String   toString() const
    {
        char b[20];
        snprintf(b, sizeof(b), "%u.%u.%u.%u", a_[0], a_[1], a_[2], a_[3]);
        return String(b);
    }
private:
    uint8_t a_[4];
};

// ---------------------------------------------------------------- serial ports: Serial = the USB-C port, Serial1 = the UART to the Teensy
class HardwareSerial
{
public:
    explicit HardwareSerial(int n) : n_(n) {}
    void   begin(unsigned long baud, uint32_t config = SERIAL_8N1, int rxPin = -1, int txPin = -1);
    size_t setRxBufferSize(size_t n);
    size_t setTxBufferSize(size_t n);
    bool   setRxFIFOFull(uint8_t n);
    int    available();
    int    read();
    int    availableForWrite();
    size_t write(const uint8_t* b, size_t n);
    size_t write(uint8_t b) { return write(&b, 1); }
    void   flush();
    size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    size_t print(const char* s);
    size_t print(const String& s) { return print(s.c_str()); }
    size_t println(const char* s = "");
    size_t println(const String& s) { return println(s.c_str()); }
    explicit operator bool() const;         // the USB port: a computer has it open
private:
    int n_;
};
extern HardwareSerial Serial, Serial1;

// ---------------------------------------------------------------- ESP, reset reasons, the task watchdog
class EspClass
{
public:
    uint64_t getEfuseMac();
    uint32_t getFreeHeap();
    uint32_t getMinFreeHeap();
    uint32_t getSketchSize();
    uint32_t getFreeSketchSpace();
    [[noreturn]] void restart();
};
extern EspClass ESP;

enum esp_reset_reason_t { ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW, ESP_RST_PANIC, ESP_RST_INT_WDT,
                          ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO };
esp_reset_reason_t esp_reset_reason();

struct esp_task_wdt_config_t { uint32_t timeout_ms; uint32_t idle_core_mask; bool trigger_panic; };
int esp_task_wdt_reconfigure(const esp_task_wdt_config_t* c);

// the header of a program file (.bin), as ESP-IDF's esp_app_format.h
enum esp_chip_id_t : uint16_t { ESP_CHIP_ID_ESP32 = 0x0000, ESP_CHIP_ID_ESP32S2 = 0x0002, ESP_CHIP_ID_ESP32C3 = 0x0005,
                                ESP_CHIP_ID_ESP32S3 = 0x0009, ESP_CHIP_ID_INVALID = 0xFFFF };
#define ESP_IMAGE_HEADER_MAGIC 0xE9
struct __attribute__((packed)) esp_image_header_t
{
    uint8_t  magic, segment_count, spi_mode, spi_speed_size;
    uint32_t entry_addr;
    uint8_t  wp_pin, spi_pin_drv[3];
    esp_chip_id_t chip_id;
    uint8_t  min_chip_rev;
    uint16_t min_chip_rev_full, max_chip_rev_full;
    uint8_t  reserved[4], hash_appended;
};
static_assert(sizeof(esp_image_header_t) == 24, "the program file header is 24 bytes");

// ---------------------------------------------------------------- WiFi
#define WIFI_OFF 0
#define WIFI_STA 1
#define WIFI_AP 2
#define WIFI_AP_STA 3
enum wl_status_t { WL_IDLE_STATUS = 0, WL_NO_SSID_AVAIL = 1, WL_SCAN_COMPLETED = 2, WL_CONNECTED = 3,
                   WL_CONNECT_FAILED = 4, WL_CONNECTION_LOST = 5, WL_DISCONNECTED = 6 };
enum wifi_auth_mode_t { WIFI_AUTH_OPEN = 0, WIFI_AUTH_WPA2_PSK = 3 };
#define WIFI_SCAN_RUNNING (-1)
#define WIFI_SCAN_FAILED (-2)

class WiFiClass
{
public:
    void        persistent(bool) {}
    bool        setHostname(const char* h);
    bool        mode(int m);
    bool        softAPConfig(IPAddress ip, IPAddress gw, IPAddress mask);
    bool        softAP(const char* ssid, const char* pass);
    IPAddress   softAPIP();
    int         softAPgetStationNum();
    bool        setAutoReconnect(bool on);
    wl_status_t begin(const char* ssid, const char* pass);
    bool        disconnect();
    wl_status_t status();
    IPAddress   localIP();
    int8_t      RSSI();
    int32_t     channel();
    int16_t     scanNetworks(bool async = false);
    int16_t     scanComplete();
    void        scanDelete();
    String      SSID(int i);
    int32_t     RSSI(int i);
    int32_t     channel(int i);
    wifi_auth_mode_t encryptionType(int i);
};
extern WiFiClass WiFi;

namespace fake { struct Conn; struct ConnHandle; struct Device; }

// ---------------------------------------------------------------- a browser's connection, the listening port
class NetworkClient
{
public:
    NetworkClient() {}
    explicit NetworkClient(std::shared_ptr<fake::ConnHandle> h) : h_(h) {}
    explicit operator bool() const { return (bool)h_; }
    bool      connected();
    int       available();
    int       read();
    int       peek();
    size_t    readBytes(char* buf, size_t n);       // as Stream: waits up to the timeout for each byte
    size_t    write(const uint8_t* b, size_t n);    // waits until the connection has taken it (10 s at most)
    size_t    print(const char* s) { return write((const uint8_t*)s, strlen(s)); }
    void      setTimeout(unsigned long ms);
    unsigned long getTimeout() const;
    void      clear();                              // what the browser sent and was not read is dropped
    void      stop();
    int       fd() const;
    IPAddress localIP();
    IPAddress remoteIP();
private:
    fake::Conn* conn() const;
    std::shared_ptr<fake::ConnHandle> h_;       // the socket: closed when the last copy of the client goes
};

class NetworkServer
{
public:
    bool          hasClient();
    NetworkClient accept();
};

// ---------------------------------------------------------------- web server (as the core's WebServer, the parts the program uses)
enum HTTPMethod { HTTP_ANY, HTTP_GET, HTTP_HEAD, HTTP_POST, HTTP_PUT, HTTP_PATCH, HTTP_DELETE, HTTP_OPTIONS };
enum HTTPUploadStatus { UPLOAD_FILE_START, UPLOAD_FILE_WRITE, UPLOAD_FILE_END, UPLOAD_FILE_ABORTED };
enum HTTPClientStatus { HC_NONE, HC_WAIT_READ, HC_WAIT_CLOSE };
#define HTTP_UPLOAD_BUFLEN      1436
#define HTTP_MAX_DATA_WAIT      5000    // ms to wait for the client to send the request
#define HTTP_MAX_POST_WAIT      5000    // ms to wait for POST data to arrive
#define HTTP_MAX_SEND_WAIT      5000    // ms to wait for data chunk to be ACKed
#define HTTP_MAX_CLOSE_WAIT     5000    // ms to wait for the client to close the connection
#define WEBSERVER_MAX_URI_LEN   2048
#define WEBSERVER_MAX_LINE_LEN  4096
#define WEBSERVER_MAX_LINE_WAIT 5000    // ms for a single protocol line to arrive complete
#define WEBSERVER_MAX_HEADER_WAIT 10000 // ms for the request line and all headers together
struct HTTPUpload
{
    HTTPUploadStatus status = UPLOAD_FILE_START;
    String filename, name, type;
    size_t totalSize = 0, currentSize = 0;
    uint8_t buf[HTTP_UPLOAD_BUFLEN];
};

class WebServer
{
public:
    typedef std::function<void()> Handler;
    explicit WebServer(int port = 80);
    virtual ~WebServer();
    void        on(const char* uri, HTTPMethod method, Handler fn);
    void        on(const char* uri, HTTPMethod method, Handler fn, Handler ufn);
    void        onNotFound(Handler fn) { notFound_ = fn; }
    void        begin();
    virtual void handleClient();
    void        collectHeaders(const char* keys[], size_t n);
    String      header(const char* name);
    String      arg(const char* name);
    String      arg(const String& name) { return arg(name.c_str()); }
    bool        hasArg(const char* name);
    String      hostHeader() { return String(host_); }
    void        send(int code, const char* type = nullptr, const String& body = String());
    void        send(int code, const char* type, const char* body) { send(code, type, String(body)); }
    void        send_P(int code, PGM_P type, PGM_P body);
    void        send_P(int code, PGM_P type, PGM_P body, size_t len);
    void        sendHeader(const String& name, const String& value, bool first = false);
    HTTPUpload& upload() { return upload_; }
    NetworkClient& client() { return _currentClient; }

protected:
    virtual size_t _currentClientWrite(const char* b, size_t l) { return _currentClient.write((const uint8_t*)b, l); }
    virtual size_t _currentClientWrite_P(PGM_P b, size_t l) { return _currentClient.write((const uint8_t*)b, l); }
    NetworkServer    _server;
    NetworkClient    _currentClient;
    HTTPClientStatus _currentStatus = HC_NONE;
    unsigned long    _statusChange = 0;

private:
    struct Route { std::string uri; HTTPMethod method; Handler fn, ufn; };
    enum class Line { Ok, TooLong, TimedOut };
    typedef std::vector<std::pair<std::string, std::string>> Pairs;
    Line  readLine(std::string& line, size_t maxLength, unsigned long phaseStart = 0, unsigned long phaseBudget = 0);
    void  errorResponse(const char* status);
    bool  parseRequest();
    bool  parseForm(const std::string& boundary);
    bool  uploadAborted();
    int   uploadReadByte();
    void  uploadWriteByte(uint8_t b);
    void  uploadCall();
    void  parseArguments(const std::string& data);
    void  collectHeader(const std::string& name, const std::string& value);
    void  prepareHeader(std::string& out, int code, const char* type, size_t len);
    void  handleRequest();
    std::vector<Route> routes_;
    Handler     notFound_;
    Route*      route_ = nullptr;           // the handler of the request being read (null: none)
    HTTPUpload  upload_;
    bool        haveUpload_ = false;
    HTTPMethod  method_ = HTTP_ANY;
    int         version_ = 1;
    std::string uri_, host_;
    Pairs       args_, postArgs_, respHeaders_, headers_;   // headers_: the ones asked for with collectHeaders()
};

// ---------------------------------------------------------------- DNS, mDNS
enum class DNSReplyCode { NoError = 0, FormError, ServerFailure, NonExistentDomain };
class DNSServer
{
public:
    void setErrorReplyCode(DNSReplyCode) {}
    bool start(uint16_t port, const char* name, IPAddress ip);
    void processNextRequest();
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
    bool   clear();
    bool   getBool(const char* key, bool def = false);
    size_t putBool(const char* key, bool v);
    String getString(const char* key, const String& def = String());
    size_t putString(const char* key, const String& v);
private:
    std::string ns_;
    bool open_ = false, readOnly_ = false;
};

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF
class UpdateClass
{
public:
    bool        begin(size_t size);
    size_t      write(uint8_t* data, size_t len);
    bool        end(bool evenIfRemaining = false);
    void        abort();
    bool        hasError();
    size_t      progress();
    const char* errorString();
};
extern UpdateClass Update;

// ---------------------------------------------------------------- the pretend board and its surroundings
namespace fake
{
struct Restart { esp_reset_reason_t reason; };      // ESP.restart() or the watchdog: caught by the harness, which loads the program again

struct Datagram
{
    uint64_t ms = 0;                        // the harness clock when it was sent
    uint32_t src = 0, dst = 0;              // as in a sockaddr (IPAddress::raw)
    uint16_t srcPort = 0, dstPort = 0;
    std::vector<uint8_t> data;
};

struct UdpSocket
{
    bool     open = false;
    uint16_t port = 0;
    std::deque<Datagram> rx;                // waiting for recvfrom(); lwIP keeps UDP_RX_QUEUE at most
    uint32_t rxDropped = 0;                 // arrived with the queue full
};
#define UDP_RX_QUEUE 6                      // CONFIG_LWIP_UDP_RECVMBOX_SIZE of the ESP32 Arduino core
#define UDP_FD_BASE  60
#define TCP_SND_BUF  5744                   // what a connection's send buffer holds (4 segments)
#define TCP_BACKLOG  4                      // connections that may wait to be accepted (NetworkServer's listen())
#define UART_FIFO    128                    // the chip's own transmit / receive bytes
#define UART_TX_ITEM 24                     // room the serial driver takes for itself with every write (two ring items)
#define UART_PER_MS  46.08                  // 460800 baud, 10 bits a byte

// One connection from a browser to port 80
struct Conn
{
    int       fd = -1;
    bool      accepted = false;             // the program took it from the listening port
    bool      fromAp = true;                // on the XIAO's own network (else the other network it joined)
    IPAddress peer;
    std::deque<uint8_t> rx;                 // from the browser, readable now
    std::string tx;                         // everything the program sent
    size_t    inFlight = 0;                 // sent, still in the connection's buffer
    size_t    takePerMs = 4000;             // what the browser takes per millisecond (0: it takes nothing)
    bool      peerClosed = false;           // the browser closed (or its link dropped for good)
    bool      serverClosed = false;         // the XIAO closed it (or restarted)
    bool      resetByServer = false;        // ... with data from the browser unread, or by a restart: what was not yet
                                            // taken by the browser is lost (the browser sees "connection reset")
    unsigned long timeoutMs = 1000;         // Stream's default
    uint64_t  openedMs = 0, closedMs = 0;
    size_t    delivered() const { return tx.size() - inFlight; }      // what the browser has by now
};
struct ConnHandle
{
    std::shared_ptr<Conn> c;
    explicit ConnHandle(std::shared_ptr<Conn> x) : c(x) {}
    ~ConnHandle();
};

struct PrefValue { char type = 0; std::string bytes; };

// The other WiFi network around (a farm router, a phone's hotspot): set by the harness
struct Router
{
    bool        on = true;
    std::string ssid = "Farm", pass = "farm12345";
    uint8_t     subnet[3] = { 192, 168, 1 };
    int         channel = 6;
};

struct Device
{
    uint64_t mac = 0x0000A1B2C3D4E5F6ull;
    bool     s3 = false;                    // XIAO ESP32S3 (else C3): the harness sets it, the program is built for it
    uint64_t bootAt = 0;                    // clockMs at the last start
    uint32_t millisOffset = 0;              // millis() = clockMs - bootAt + millisOffset (to try the 49-day wrap)
    bool     running = false;               // the program's code is running now (setup() or loop())
    esp_reset_reason_t resetReason = ESP_RST_POWERON;
    float    tempC = 41.5f;

    // the task watchdog (the loop task is fed before each loop() and by feedLoopWDT())
    bool     wdtOn = false;
    uint32_t wdtTimeoutMs = 5000;
    uint64_t wdtFedAt = 0;
    uint32_t wdtLongestMs = 0;              // the longest time without a feed

    // flash
    std::map<std::string, std::map<std::string, PrefValue>> prefs;
    uint32_t flashWrites = 0;

    // WiFi
    int         wifiMode = WIFI_OFF;
    std::string hostName;
    std::string apSsid, apPass;
    bool        apOn = false;
    int         apStations = 0;             // devices on the XIAO's own network (set by the harness)
    bool        staBegun = false, staAuto = true, staJoined = false;
    std::string staSsid, staPass;
    uint64_t    staTryAt = 0;               // when the search for the other network (re)started
    uint32_t    staBegins = 0, staLost = 0;
    int         scanState = -2;             // -2 none, -1 running
    uint64_t    scanAt = 0;
    uint32_t    scans = 0;
    bool        webListening = false;       // port 80 is open
    bool        mdnsOn = false;
    std::string mdnsHost, mdnsInstance;
    std::vector<std::string> mdnsServices;
    bool        dnsOn = false;
    Router      router;

    // the serial port to the Teensy (UART1): 460800 baud = 46 bytes a millisecond each way, moved by the harness
    unsigned long uartBaud = 0;
    size_t      uartRxCap = 256, uartTxCap = 0;
    std::deque<uint8_t> uartRx;             // from the Teensy, not read yet
    std::deque<uint8_t> uartTx;             // to the Teensy: in the transmit buffer (a ring of whole writes)
    std::deque<uint8_t> uartFifo;           // ... and in the chip's own 128 bytes, next on the wire
    size_t      uartTxUsed = 0;             // room taken in the transmit buffer (with the driver's own bytes per write)
    std::deque<std::pair<size_t, size_t>> uartTxItems;   // (bytes left of this write, its room in the buffer)
    uint32_t    uartRxLost = 0;             // bytes from the Teensy that found the receive buffer full
    uint32_t    uartWriteWaitMs = 0;        // time the program spent waiting in a write (the buffer was full)

    // UDP sockets (lwIP), by number
    std::vector<UdpSocket> udp;
    std::vector<Datagram> udpOut;           // sent by the program (the harness takes them away)
    bool        udpJam = false;             // the WiFi takes no datagram now (its queue is full): sendto() fails
    uint32_t    udpSendFails = 0;

    // browsers: connections waiting to be accepted, and every connection (the harness looks at them)
    std::deque<std::shared_ptr<Conn>> pending;
    std::vector<std::shared_ptr<Conn>> conns;
    int         nextFd = 10;

    // program update
    size_t      updSize = 0;                // > 0: an update is running (the core's _size)
    int         updError = 0;               // 0 = none (the numbers are the core's UPDATE_ERROR_*)
    std::vector<uint8_t> updBuf, updImage;  // the sector being filled, what was written to the flash
    size_t      updProgress = 0, updPartition = 0x140000;
    bool        updDone = false;            // a whole image was accepted: the next start runs it
    bool        updDoneIsApp = false;       // ... and it is an application (not a bootloader with one behind it)
    uint32_t    updInstalled = 0;           // images that started running
    uint32_t    updInstalledNotApp = 0;     // ... that were no application

    // the USB-C port
    bool        usbOpen = false;            // a computer has the port open
    std::string usbOut, usbIn;

    uint32_t    restarts = 0;

    bool      staUp() const { return staJoined; }       // joined the other network
    bool      staSearching() const;         // trying to join it (the radio leaves the XIAO's own channel to look)
    bool      scanning() const { return scanState == -1; }
    size_t    udpOpen() const { size_t n = 0; for (auto& s : udp) n += s.open; return n; }     // UDP sockets the program has open
    IPAddress staIp() const { return IPAddress(router.subnet[0], router.subnet[1], router.subnet[2], 50); }

    // ---- what the harness does to the board
    void      step();                       // one millisecond of the board's own surroundings: the other network, browsers' connections
    size_t    uartWire(uint8_t* out, size_t max);       // bytes that left for the Teensy on the wire now (at most max)
    void      uartFeed(const uint8_t* b, size_t n);     // bytes from the Teensy arriving on the wire
    bool      udpIn(const Datagram& d);     // a datagram arrives from the WiFi: false = no socket took it (not open, or its queue full)
    std::shared_ptr<Conn> connect(bool fromAp, IPAddress peer);    // a browser opens a connection to port 80 (null: not taken)
    void      stopped(esp_reset_reason_t why);          // the program stopped (a restart, or the power went): RAM and radio state go
};
extern Device* cur;
extern uint64_t clockMs;
extern std::function<void()> onMs;          // the world around the XIAO: one step, every millisecond
void tick(uint32_t ms = 1);                 // pretend time passes (the watchdog may bite: throws Restart)
}
