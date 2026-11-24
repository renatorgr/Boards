/* fake_core.cpp - the pretend ESP32 + Arduino core (fake_core.h).
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The web server follows WebServer.cpp / Parsing.cpp of the ESP32 Arduino core 3.3.12 (Espressif, LGPL 2.1), the
   update its Updater.cpp, the serial port its esp32-hal-uart.c and ESP-IDF's driver: the same waits, limits and order.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include <stdexcept>
#include "fake_core.h"
#include <algorithm>
#include "lwip/sockets.h"

namespace fake
{
Device* cur = nullptr;
uint64_t clockMs = 100000;      // the bench was switched on a while ago
uint32_t freeHeap = 182000;
uint32_t minFreeHeap = 151000;
std::function<void()> onMs;

// The program's other tasks: each one whose sleep is over runs until it sleeps again (vTaskDelay throws TaskSleep).
// The loop stands still meanwhile; a wait inside a task moves the clock but gives no other task a turn.
static void runTasks(Device& d)
{
    for (size_t i = 0; i < d.tasks.size(); i++)
    {
        if (clockMs < d.tasks[i].wakeAt) continue;
        d.inTask = (int)i;
        d.tasks[i].turns++;
        try
        {
            d.tasks[i].fn(d.tasks[i].arg);
            d.inTask = -1;
            throw Restart{ ESP_RST_PANIC };             // a task function that returns: the chip stops there
        }
        catch (TaskSleep& s) { d.tasks[i].wakeAt = clockMs + (s.ticks ? s.ticks : 1); }
        catch (...) { d.inTask = -1; throw; }
        d.inTask = -1;
    }
}

// Pretend time: the only place it moves. The world takes a step every millisecond, the program's other tasks get
// their turn, then the task watchdog looks at its clocks: the loop task's (the core feeds it before each loop()
// pass, the program with feedLoopWDT()) and the program's own entry (esp_task_wdt_reset_user).
void tick(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        clockMs++;
        if (onMs) onMs();
        Device* d = cur;
        if (d && d->inTask < 0 && !d->tasks.empty()) runTasks(*d);
        if (d && d->running)
        {
            uint32_t gap = 0;
            if (d->wdtOn) gap = (uint32_t)(clockMs - d->wdtFedAt);
            if (d->wdtUserOn) gap = std::max(gap, (uint32_t)(clockMs - d->wdtUserFedAt));
            if (gap > d->wdtLongestMs) d->wdtLongestMs = gap;
            if (gap >= d->wdtTimeoutMs && (d->wdtOn || d->wdtUserOn)) throw Restart{ ESP_RST_TASK_WDT };
        }
    }
}

ConnHandle::~ConnHandle()
{
    if (c->serverClosed) return;
    c->serverClosed = true;
    c->closedMs = clockMs;
    if (!c->rx.empty()) c->resetByServer = true;       // closing with unread input resets the connection (lwIP)
    c->rx.clear();
}

bool Device::staSearching() const
{
    return (wifiMode & WIFI_STA) && staBegun && !staJoined && (staAuto || clockMs - staTryAt < 10000);
}

// The other network, and what browsers have taken from their connections
void Device::step()
{
    const bool there = router.on && staSsid == router.ssid && staPass == router.pass;
    if (staJoined && (!there || !staBegun || !(wifiMode & WIFI_STA)))
    {
        staJoined = false;
        staLost++;
        staTryAt = clockMs;
    }
    else if (!staJoined && (wifiMode & WIFI_STA) && staBegun && there && staSearching() && clockMs - staTryAt >= 1500) staJoined = true;
    if (scanState == -1 && clockMs - scanAt >= 2500) scanState = 0;      // done: scanComplete() gives the list
    for (auto& c : conns)
    {
        if (c->inFlight == 0) continue;
        if (c->resetByServer) { c->tx.resize(c->delivered()); c->inFlight = 0; continue; }
        c->inFlight -= std::min(c->inFlight, c->takePerMs);
    }
}

// The transmit side of the serial port, as ESP-IDF's driver: the program's writes sit whole in a ring buffer, the
// chip's own 128 bytes are filled from it when they run low, and a write's room in the ring is free again once its
// last byte has moved on.
size_t Device::uartWire(uint8_t* out, size_t max)
{
    size_t n = 0;
    auto refill = [this]() {
        while (uartFifo.size() < UART_FIFO && !uartTx.empty())
        {
            uartFifo.push_back(uartTx.front());
            uartTx.pop_front();
            if (!uartTxItems.empty() && --uartTxItems.front().first == 0)
            {
                uartTxUsed -= uartTxItems.front().second;
                uartTxItems.pop_front();
            }
        }
    };
    while (n < max)
    {
        if (uartFifo.empty()) { refill(); if (uartFifo.empty()) break; }
        out[n++] = uartFifo.front();
        uartFifo.pop_front();
        if (uartFifo.size() <= 10) refill();
    }
    return n;
}

void Device::uartFeed(const uint8_t* b, size_t n)
{
    if (uartBaud == 0) return;                      // the port is not open (the program is not running): nobody listens
    for (size_t i = 0; i < n; i++)
    {
        if (uartRx.size() >= uartRxCap + UART_FIFO) { uartRxLost++; continue; }
        uartRx.push_back(b[i]);
    }
}

bool Device::udpIn(const Datagram& d)
{
    const IPAddress from = IPAddress::fromRaw(d.src);
    if (from[0] == apIp[0] && from[1] == apIp[1] && from[2] == apIp[2] && std::find(apPeers.begin(), apPeers.end(), d.src) == apPeers.end()) apPeers.push_back(d.src);
    for (auto& s : udp)
    {
        if (!s.open || s.port != d.dstPort) continue;
        if (s.rx.size() >= UDP_RX_QUEUE) { s.rxDropped++; return false; }
        s.rx.push_back(d);
        return true;
    }
    return false;
}

std::shared_ptr<Conn> Device::connect(bool fromAp, IPAddress peer, uint16_t port)
{
    const bool listening = port == 80 ? webListening : std::find(listenPorts.begin(), listenPorts.end(), port) != listenPorts.end();
    size_t waiting = 0;
    for (auto& p : pending) waiting += p->port == port;
    if (!listening || waiting >= TCP_BACKLOG) return nullptr;
    if (fromAp ? !apOn : !staJoined) return nullptr;
    if (fromAp && std::find(apPeers.begin(), apPeers.end(), peer.raw()) == apPeers.end()) apPeers.push_back(peer.raw());
    auto c = std::make_shared<Conn>();
    c->fd = nextFd++;
    c->fromAp = fromAp;
    c->port = port;
    c->peer = peer;
    c->openedMs = clockMs;
    pending.push_back(c);
    conns.push_back(c);
    return c;
}

void Device::stopped(esp_reset_reason_t why)
{
    resetReason = why;
    running = false;
    wdtOn = wdtUserOn = false;
    minFreeHeap = freeHeap < 151000 ? freeHeap : 151000;   // the allocator's lowest point starts again with the program
    tasks.clear();
    locks.clear();
    inTask = -1;
    wifiMode = WIFI_OFF;
    apOn = false;
    apIp = IPAddress(192, 168, 4, 1);
    staBegun = staJoined = false;
    staAuto = true;
    scanState = -2;
    mdnsOn = dnsOn = webListening = false;
    listenPorts.clear();
    mdnsServices.clear();
    uartBaud = 0;
    uartRxCap = 256;
    uartTxCap = 0;
    uartRx.clear(); uartTx.clear(); uartFifo.clear(); uartTxItems.clear();
    uartTxUsed = 0;
    for (auto& s : udp) { s.open = false; s.rx.clear(); }
    for (auto& c : conns)
        if (!c->serverClosed) { c->serverClosed = true; c->resetByServer = true; c->closedMs = clockMs; c->tx.resize(c->delivered()); c->inFlight = 0; c->rx.clear(); }
    pending.clear();
    updSize = 0; updError = 0; updBuf.clear(); updImage.clear(); updProgress = 0;
    usbIn.clear();
}
}

using fake::cur;

// ---------------------------------------------------------------- time
uint32_t millis() { return (uint32_t)(fake::clockMs - cur->bootAt + cur->millisOffset); }
uint32_t micros() { return millis() * 1000u; }
void delay(uint32_t ms) { fake::tick(ms); }
void yield() { fake::tick(1); }             // a loop that waits with yield() spends time: a millisecond a turn here
float temperatureRead() { return cur->tempC; }
void enableLoopWDT() { cur->wdtOn = true; cur->wdtFedAt = fake::clockMs; }
void feedLoopWDT() { cur->wdtFedAt = fake::clockMs; }
int esp_task_wdt_reconfigure(const esp_task_wdt_config_t* c) { cur->wdtTimeoutMs = c->timeout_ms; return 0; }
int esp_task_wdt_add_user(const char*, esp_task_wdt_user_handle_t* handle)
{
    cur->wdtUserOn = true;
    cur->wdtUserFedAt = fake::clockMs;
    *handle = (esp_task_wdt_user_handle_t)cur;
    return ESP_OK;
}
int esp_task_wdt_reset_user(esp_task_wdt_user_handle_t) { cur->wdtUserFedAt = fake::clockMs; return ESP_OK; }

// ---------------------------------------------------------------- FreeRTOS: a second task and a lock
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex()
{
    cur->locks.emplace_back(new fake::Lock());
    return cur->locks.back().get();
}

BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t lock, TickType_t wait)
{
    const int me = cur->inTask;
    for (TickType_t waited = 0;; waited++)
    {
        if (lock->depth == 0 || lock->owner == me)
        {
            lock->owner = me;
            lock->depth++;
            return pdTRUE;
        }
        if (wait == 0 || (wait != portMAX_DELAY && waited >= wait)) return pdFALSE;
        if (me >= 0) throw std::logic_error("pretend core: a task waits for a lock the loop holds - not played here (a task takes a lock with no wait)");
        fake::tick(1);                              // the loop waits: the tasks get their turns
    }
}

BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t lock)
{
    if (lock->depth == 0 || lock->owner != cur->inTask) { cur->lockErrors++; return pdFALSE; }
    lock->depth--;
    return pdTRUE;
}

BaseType_t xTaskCreateUniversal(TaskFunction_t fn, const char* name, uint32_t stack, void* arg, UBaseType_t priority, TaskHandle_t* handle, BaseType_t core)
{
    fake::Task t;
    t.fn = fn; t.arg = arg; t.name = name ? name : ""; t.stack = stack; t.priority = priority; t.core = core;
    t.wakeAt = fake::clockMs;
    cur->tasks.push_back(t);
    if (handle) *handle = (TaskHandle_t)(uintptr_t)cur->tasks.size();
    return pdPASS;
}

void vTaskDelay(TickType_t ticks)
{
    if (cur->inTask >= 0) throw fake::TaskSleep{ ticks };
    fake::tick(ticks);
}
esp_reset_reason_t esp_reset_reason() { return cur->resetReason; }

// ---------------------------------------------------------------- ESP
EspClass ESP;
uint64_t EspClass::getEfuseMac() { return cur->mac; }
uint32_t EspClass::getFreeHeap() { return fake::freeHeap; }
// The lowest point is the allocator's own: it never goes back up, and never stays above what is free now
uint32_t EspClass::getMinFreeHeap() { if (fake::freeHeap < fake::minFreeHeap) fake::minFreeHeap = fake::freeHeap; return fake::minFreeHeap; }
uint32_t EspClass::getMaxAllocHeap() { return fake::freeHeap * 3 / 4; }   // the largest piece: some of the free memory is in small pieces
uint32_t EspClass::getSketchSize() { return 1167872; }
uint32_t EspClass::getFreeSketchSpace() { return (uint32_t)cur->updPartition; }
void EspClass::restart() { throw fake::Restart{ ESP_RST_SW }; }

// ---------------------------------------------------------------- serial ports
HardwareSerial Serial(0), Serial1(1);

void HardwareSerial::begin(unsigned long baud, uint32_t, int, int)
{
    if (n_ == 0) return;
    cur->uartBaud = baud;
    cur->uartRx.clear();
}
size_t HardwareSerial::setRxBufferSize(size_t n)
{
    if (n_ == 0 || cur->uartBaud) return 0;         // only before begin()
    cur->uartRxCap = n <= UART_FIFO ? UART_FIFO + 1 : n;
    return cur->uartRxCap;
}
size_t HardwareSerial::setTxBufferSize(size_t n)
{
    if (n_ == 0 || cur->uartBaud) return 0;
    cur->uartTxCap = (n > 0 && n <= UART_FIFO) ? UART_FIFO + 1 : n;
    return cur->uartTxCap;
}
bool HardwareSerial::setRxFIFOFull(uint8_t) { return true; }
int HardwareSerial::available() { return (int)(n_ == 0 ? cur->usbIn.size() : cur->uartRx.size()); }
int HardwareSerial::read()
{
    if (n_ == 0)
    {
        if (cur->usbIn.empty()) return -1;
        const uint8_t c = (uint8_t)cur->usbIn[0];
        cur->usbIn.erase(0, 1);
        return c;
    }
    if (cur->uartRx.empty()) return -1;
    const uint8_t c = cur->uartRx.front();
    cur->uartRx.pop_front();
    return c;
}
// As uartAvailableForWrite(): the room in the transmit ring (the longest write it takes now, never over half the
// ring), or - when the ring says 0 - the room in the chip's own 128 bytes.
int HardwareSerial::availableForWrite()
{
    if (n_ == 0) return 256;
    if (cur->uartTxCap == 0) return (int)(UART_FIFO - cur->uartFifo.size());
    const size_t free = cur->uartTxCap > cur->uartTxUsed ? cur->uartTxCap - cur->uartTxUsed : 0;
    size_t room = free > 8 ? free - 8 : 0;
    const size_t biggest = cur->uartTxCap / 2 - 8;
    if (room > biggest) room = biggest;
    return (int)(room ? room : UART_FIFO - cur->uartFifo.size());
}
// As uart_write_bytes(): never refuses; with no room it waits for the wire (pretend time passes meanwhile)
size_t HardwareSerial::write(const uint8_t* b, size_t n)
{
    if (n_ == 0)
    {
        if (cur->usbOpen) cur->usbOut.append((const char*)b, n);
        return n;
    }
    if (cur->uartBaud == 0 || n == 0) return 0;
    if (cur->uartTxCap == 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            while (cur->uartFifo.size() >= UART_FIFO) { cur->uartWriteWaitMs++; fake::tick(1); }
            cur->uartFifo.push_back(b[i]);
        }
        return n;
    }
    const size_t room = UART_TX_ITEM + ((n + 3) & ~(size_t)3);
    while (cur->uartTxUsed + room > cur->uartTxCap) { cur->uartWriteWaitMs++; fake::tick(1); }
    cur->uartTxUsed += room;
    cur->uartTxItems.push_back({ n, room });
    cur->uartTx.insert(cur->uartTx.end(), b, b + n);
    return n;
}
void HardwareSerial::flush() { if (n_ == 1) while (!cur->uartTx.empty() || !cur->uartFifo.empty()) fake::tick(1); }
size_t HardwareSerial::printf(const char* fmt, ...)
{
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    return write((const uint8_t*)b, (size_t)std::min(n < 0 ? 0 : n, (int)sizeof(b) - 1));
}
size_t HardwareSerial::print(const char* s) { return write((const uint8_t*)s, strlen(s)); }
size_t HardwareSerial::println(const char* s) { return print(s) + write((const uint8_t*)"\r\n", 2); }
HardwareSerial::operator bool() const { return n_ == 0 ? cur->usbOpen : cur->uartBaud != 0; }

// ---------------------------------------------------------------- WiFi
WiFiClass WiFi;
bool WiFiClass::setHostname(const char* h) { cur->hostName = h ? h : ""; return true; }
bool WiFiClass::mode(int m) { cur->wifiMode = m; if (!(m & WIFI_AP)) cur->apOn = false; return true; }
bool WiFiClass::softAPConfig(IPAddress ip, IPAddress, IPAddress) { cur->apIp = ip; return true; }
bool WiFiClass::softAP(const char* ssid, const char* pass, int, int, int maxStations)
{
    if (!ssid || !*ssid || strlen(ssid) > 32) return false;
    if (pass && *pass && (strlen(pass) < 8 || strlen(pass) > 63)) return false;     // as the core: refused, no network
    cur->apMaxStations = maxStations;
    cur->apSsid = ssid;
    cur->apPass = pass ? pass : "";
    cur->apOn = (cur->wifiMode & WIFI_AP) != 0;
    return cur->apOn;
}
IPAddress WiFiClass::softAPIP() { return cur->apIp; }
int WiFiClass::softAPgetStationNum() { return cur->apOn ? cur->apStations : 0; }
bool WiFiClass::setAutoReconnect(bool on) { cur->staAuto = on; return true; }
wl_status_t WiFiClass::begin(const char* ssid, const char* pass)
{
    cur->staSsid = ssid ? ssid : "";
    cur->staPass = pass ? pass : "";
    cur->staBegun = true;
    cur->staJoined = false;
    cur->staTryAt = fake::clockMs;
    cur->staBegins++;
    return WL_DISCONNECTED;
}
bool WiFiClass::disconnect()
{
    if (cur->staJoined) cur->staLost++;
    cur->staBegun = cur->staJoined = false;
    return true;
}
wl_status_t WiFiClass::status()
{
    if (cur->staJoined) return WL_CONNECTED;
    if (!cur->staBegun) return WL_IDLE_STATUS;
    return cur->router.on && cur->staSsid == cur->router.ssid ? WL_DISCONNECTED : WL_NO_SSID_AVAIL;
}
IPAddress WiFiClass::localIP() { return cur->staJoined ? cur->staIp() : IPAddress(); }
int8_t WiFiClass::RSSI() { return cur->staJoined ? -61 : 0; }
int32_t WiFiClass::channel() { return cur->staJoined ? cur->router.channel : 1; }
int16_t WiFiClass::scanNetworks(bool)
{
    cur->scanState = -1;
    cur->scanAt = fake::clockMs;
    cur->scans++;
    return WIFI_SCAN_RUNNING;
}
// what a search finds: the other network (twice: two access points of it), a neighbour, one with a hidden name
static int scanCount() { return cur->router.on ? 4 : 2; }
int16_t WiFiClass::scanComplete() { return cur->scanState == -2 ? WIFI_SCAN_FAILED : cur->scanState == -1 ? WIFI_SCAN_RUNNING : (int16_t)scanCount(); }
void WiFiClass::scanDelete() { cur->scanState = -2; }
String WiFiClass::SSID(int i)
{
    if (!cur->router.on) i += 2;
    return i == 0 || i == 1 ? String(cur->router.ssid) : i == 2 ? String("Neighbour \"5G\"") : String("");
}
int32_t WiFiClass::RSSI(int i) { return -50 - 9 * i; }
int32_t WiFiClass::channel(int i) { return i < 2 && cur->router.on ? cur->router.channel : 11; }
wifi_auth_mode_t WiFiClass::encryptionType(int i) { return (!cur->router.on ? i + 2 : i) == 2 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK; }

// ---------------------------------------------------------------- DNS, mDNS
bool DNSServer::start(uint16_t, const char*, IPAddress) { cur->dnsOn = true; return true; }
void DNSServer::processNextRequest() {}
MDNSResponder MDNS;
bool MDNSResponder::begin(const char* host) { cur->mdnsOn = true; cur->mdnsHost = host ? host : ""; return true; }
void MDNSResponder::setInstanceName(const String& name) { cur->mdnsInstance = name.str(); }
bool MDNSResponder::addService(const char* service, const char* proto, uint16_t port)
{
    cur->mdnsServices.push_back(std::string("_") + service + "._" + proto + ":" + std::to_string(port));
    return true;
}
bool MDNSResponder::addServiceTxt(const char* service, const char* proto, const char* key, const char* value)
{
    cur->mdnsServices.push_back(std::string("_") + service + "._" + proto + " " + key + "=" + value);
    return true;
}

// ---------------------------------------------------------------- flash
bool Preferences::begin(const char* ns, bool readOnly) { ns_ = ns; readOnly_ = readOnly; open_ = true; return true; }
void Preferences::end() { open_ = false; }
bool Preferences::clear()
{
    if (!open_ || readOnly_) return false;
    cur->prefs[ns_].clear();
    cur->flashWrites++;
    return true;
}
bool Preferences::getBool(const char* key, bool def)
{
    auto& m = cur->prefs[ns_];
    auto it = m.find(key);
    return open_ && it != m.end() && it->second.type == 'b' && it->second.bytes.size() == 1 ? it->second.bytes[0] != 0 : def;
}
size_t Preferences::putBool(const char* key, bool v)
{
    if (!open_ || readOnly_) return 0;
    cur->prefs[ns_][key] = { 'b', std::string(1, v ? 1 : 0) };
    cur->flashWrites++;
    return 1;
}
String Preferences::getString(const char* key, const String& def)
{
    auto& m = cur->prefs[ns_];
    auto it = m.find(key);
    return open_ && it != m.end() && it->second.type == 's' ? String(it->second.bytes) : def;
}
size_t Preferences::putString(const char* key, const String& v)
{
    if (!open_ || readOnly_) return 0;
    cur->prefs[ns_][key] = { 's', v.str() };
    cur->flashWrites++;
    return v.length();
}

// ---------------------------------------------------------------- program update (as the core's Updater.cpp)
// The file is gathered a flash sector (4096 bytes) at a time, so progress() moves in sectors; the first sector must
// start with the program-file mark; end() checks the whole image as the chip's own start-up code would (the header,
// the chip it is for, every segment, the checksum) and only then makes it the one to start.
UpdateClass Update;
enum { UE_OK = 0, UE_READ = 3, UE_SPACE = 4, UE_SIZE = 5, UE_ACTIVATE = 9, UE_ABORT = 12, UE_DECRYPT = 13 };
static void updReset(int err)
{
    cur->updBuf.clear();
    cur->updImage.clear();
    cur->updProgress = 0;
    cur->updSize = 0;
    cur->updError = err;
}
static bool updWriteBuffer()
{
    // a first byte that is not the mark: the core takes the file for an encrypted one, and no key is set
    if (cur->updProgress == 0 && (cur->updBuf.empty() || cur->updBuf[0] != ESP_IMAGE_HEADER_MAGIC)) { updReset(UE_DECRYPT); return false; }
    cur->updImage.insert(cur->updImage.end(), cur->updBuf.begin(), cur->updBuf.end());
    cur->updProgress += cur->updBuf.size();
    cur->updBuf.clear();
    return true;
}
// esp_image_verify(): header, chip, segments, checksum (the SHA-256 after it is not computed here: only its room)
static bool imageValid(const std::vector<uint8_t>& im, uint16_t chip)
{
    if (im.size() < sizeof(esp_image_header_t)) return false;
    esp_image_header_t h;
    memcpy(&h, im.data(), sizeof(h));
    if (h.magic != ESP_IMAGE_HEADER_MAGIC || h.segment_count == 0 || h.segment_count > 16 || h.chip_id != chip) return false;
    size_t at = sizeof(h);
    uint8_t sum = 0xEF;
    for (int s = 0; s < h.segment_count; s++)
    {
        if (at + 8 > im.size()) return false;
        uint32_t len;
        memcpy(&len, im.data() + at + 4, 4);
        at += 8;
        if (len > im.size() - at) return false;
        for (uint32_t i = 0; i < len; i++) sum ^= im[at + i];
        at += len;
    }
    at = (at + 16) & ~(size_t)15;               // padded to 16 bytes, the checksum in the last one
    if (at > im.size() || im[at - 1] != sum) return false;
    if (h.hash_appended && at + 32 > im.size()) return false;
    return true;
}
bool UpdateClass::begin(size_t size)
{
    if (cur->updSize > 0) return false;         // one is running
    updReset(UE_OK);
    if (size == 0) { cur->updError = UE_SIZE; return false; }
    if (size == UPDATE_SIZE_UNKNOWN) size = cur->updPartition;
    else if (size > cur->updPartition) { cur->updError = UE_SIZE; return false; }
    cur->updSize = size;
    return true;
}
size_t UpdateClass::write(uint8_t* data, size_t len)
{
    if (hasError() || cur->updSize == 0) return 0;
    if (len > cur->updSize - cur->updProgress) { updReset(UE_SPACE); return 0; }
    size_t left = len;
    while (cur->updBuf.size() + left > 4096)
    {
        const size_t toBuff = 4096 - cur->updBuf.size();
        cur->updBuf.insert(cur->updBuf.end(), data + (len - left), data + (len - left) + toBuff);
        if (!updWriteBuffer()) return len - left;
        left -= toBuff;
    }
    cur->updBuf.insert(cur->updBuf.end(), data + (len - left), data + len);
    if (cur->updBuf.size() == cur->updSize - cur->updProgress && !updWriteBuffer()) return len - left;
    return len;
}
bool UpdateClass::end(bool evenIfRemaining)
{
    if (hasError() || cur->updSize == 0) return false;
    if (cur->updProgress != cur->updSize && !evenIfRemaining) { updReset(UE_ABORT); return false; }
    if (evenIfRemaining)
    {
        if (!cur->updBuf.empty() && !updWriteBuffer()) return false;
        cur->updSize = cur->updProgress;
    }
    if (cur->updImage.empty() || cur->updImage[0] != ESP_IMAGE_HEADER_MAGIC) { updReset(UE_READ); return false; }
    if (!imageValid(cur->updImage, (uint16_t)(cur->s3 ? ESP_CHIP_ID_ESP32S3 : ESP_CHIP_ID_ESP32C3))) { updReset(UE_ACTIVATE); return false; }
    // an application carries its description 32 bytes in (esp_app_desc_t, magic ABCD5432); a bootloader does not
    uint32_t desc = 0;
    if (cur->updImage.size() >= 0x24) memcpy(&desc, cur->updImage.data() + 0x20, 4);
    cur->updDone = true;
    cur->updDoneIsApp = desc == 0xABCD5432u;
    updReset(UE_OK);
    return true;
}
void UpdateClass::abort() { updReset(UE_ABORT); }
bool UpdateClass::hasError() { return cur->updError != UE_OK; }
size_t UpdateClass::progress() { return cur->updProgress; }
const char* UpdateClass::errorString()
{
    switch (cur->updError)
    {
    case UE_OK:       return "No Error";
    case UE_READ:     return "Flash Read Failed";
    case UE_SPACE:    return "Not Enough Space";
    case UE_SIZE:     return "Bad Size Given";
    case UE_ACTIVATE: return "Could Not Activate The Firmware";
    case UE_ABORT:    return "Aborted";
    case UE_DECRYPT:  return "Decryption error";
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------- UDP sockets (lwIP), send() on a browser's connection
static fake::UdpSocket* udpOf(int s)
{
    const int i = s - UDP_FD_BASE;
    return i >= 0 && i < (int)cur->udp.size() && cur->udp[i].open ? &cur->udp[i] : nullptr;
}
int lwip_socket(int, int, int)
{
    for (size_t i = 0; i < cur->udp.size(); i++)
        if (!cur->udp[i].open) { cur->udp[i] = fake::UdpSocket(); cur->udp[i].open = true; return UDP_FD_BASE + (int)i; }
    if (cur->udp.size() >= 10) { errno = ENFILE; return -1; }      // CONFIG_LWIP_MAX_SOCKETS
    cur->udp.emplace_back();
    cur->udp.back().open = true;
    return UDP_FD_BASE + (int)cur->udp.size() - 1;
}
int lwip_setsockopt(int s, int, int, const void*, socklen_t) { return udpOf(s) ? 0 : -1; }
int lwip_bind(int s, const struct sockaddr* name, socklen_t)
{
    fake::UdpSocket* u = udpOf(s);
    if (!u) { errno = EBADF; return -1; }
    const struct sockaddr_in* a = (const struct sockaddr_in*)name;
    u->port = htons(a->sin_port);
    return 0;
}
int lwip_recvfrom(int s, void* mem, size_t len, int, struct sockaddr* from, socklen_t* fromlen)
{
    fake::UdpSocket* u = udpOf(s);
    if (!u) { errno = EBADF; return -1; }
    if (u->rx.empty()) { errno = EAGAIN; return -1; }
    const fake::Datagram d = u->rx.front();
    u->rx.pop_front();
    const size_t n = std::min(len, d.data.size());
    if (n) memcpy(mem, d.data.data(), n);
    if (from)
    {
        struct sockaddr_in a;
        memset(&a, 0, sizeof(a));
        a.sin_len = sizeof(a);
        a.sin_family = AF_INET;
        a.sin_port = htons(d.srcPort);
        a.sin_addr.s_addr = d.src;
        memcpy(from, &a, sizeof(a));
        if (fromlen) *fromlen = sizeof(a);
    }
    return (int)n;
}
int lwip_sendto(int s, const void* data, size_t size, int, const struct sockaddr* to, socklen_t)
{
    fake::UdpSocket* u = udpOf(s);
    if (!u) { errno = EBADF; return -1; }
    if (cur->udpJam || !cur->apOn) { cur->udpSendFails++; errno = ENOMEM; return -1; }
    const struct sockaddr_in* a = (const struct sockaddr_in*)to;
    fake::Datagram d;
    d.ms = fake::clockMs;
    d.src = cur->apIp.raw();
    d.srcPort = u->port;
    d.dst = a->sin_addr.s_addr;
    d.dstPort = htons(a->sin_port);
    d.data.assign((const uint8_t*)data, (const uint8_t*)data + size);
    cur->udpOut.push_back(d);
    return (int)size;
}
int lwip_close(int s)
{
    fake::UdpSocket* u = udpOf(s);
    if (!u) return -1;
    u->open = false;
    u->rx.clear();
    return 0;
}
// send() with MSG_DONTWAIT on a browser's connection: what its send buffer takes now
int send(int s, const void* data, size_t size, int)
{
    fake::Conn* c = nullptr;
    for (auto& x : cur->conns) if (x->fd == s && !x->serverClosed) c = x.get();
    if (!c) { errno = EBADF; return -1; }
    if (c->peerClosed) { errno = ECONNRESET; return -1; }
    const size_t room = TCP_SND_BUF > c->inFlight ? TCP_SND_BUF - c->inFlight : 0;
    if (room == 0) { errno = EAGAIN; return -1; }
    const size_t n = std::min(room, size);
    c->tx.append((const char*)data, n);
    c->inFlight += n;
    return (int)n;
}

// ---------------------------------------------------------------- a browser's connection
fake::Conn* NetworkClient::conn() const { return h_ && !h_->c->serverClosed ? h_->c.get() : nullptr; }
bool NetworkClient::connected()
{
    fake::Conn* c = conn();
    return c && !(c->peerClosed && c->rx.empty());
}
int NetworkClient::available() { fake::Conn* c = conn(); return c ? (int)c->rx.size() : 0; }
int NetworkClient::read()
{
    fake::Conn* c = conn();
    if (!c || c->rx.empty()) return -1;
    const uint8_t b = c->rx.front();
    c->rx.pop_front();
    return b;
}
int NetworkClient::peek() { fake::Conn* c = conn(); return c && !c->rx.empty() ? c->rx.front() : -1; }
int NetworkClient::read(uint8_t* buf, size_t n)
{
    fake::Conn* c = conn();
    if (!c || c->rx.empty()) return -1;
    size_t k = 0;
    while (k < n && !c->rx.empty()) { buf[k++] = c->rx.front(); c->rx.pop_front(); }
    return (int)k;
}
size_t NetworkClient::readBytes(char* buf, size_t n)
{
    size_t k = 0;
    while (k < n)
    {
        int b = read();
        const uint32_t t0 = millis();
        while (b < 0 && millis() - t0 < getTimeout()) { fake::tick(1); b = read(); }      // Stream::timedRead: a busy wait
        if (b < 0) break;
        buf[k++] = (char)b;
    }
    return k;
}
size_t NetworkClient::write(const uint8_t* b, size_t n)
{
    size_t done = 0;
    const uint64_t t0 = fake::clockMs;
    while (done < n && fake::clockMs - t0 < 10000)
    {
        fake::Conn* c = conn();
        if (!c) break;
        const int r = ::send(c->fd, b + done, n - done, MSG_DONTWAIT);
        if (r > 0) { done += r; continue; }
        if (errno != EAGAIN) break;
        fake::tick(1);
    }
    return done;
}
void NetworkClient::setTimeout(unsigned long ms) { fake::Conn* c = conn(); if (c) c->timeoutMs = ms; }
unsigned long NetworkClient::getTimeout() const { fake::Conn* c = conn(); return c ? c->timeoutMs : 1000; }
void NetworkClient::clear() { fake::Conn* c = conn(); if (c) c->rx.clear(); }
void NetworkClient::stop() { h_.reset(); }
int NetworkClient::fd() const { fake::Conn* c = conn(); return c ? c->fd : -1; }
IPAddress NetworkClient::localIP() { fake::Conn* c = conn(); return !c ? IPAddress() : c->fromAp ? cur->apIp : cur->staIp(); }
IPAddress NetworkClient::remoteIP() { fake::Conn* c = conn(); return c ? c->peer : IPAddress(); }

void NetworkServer::begin()
{
    if (port_ == 80) cur->webListening = true;
    else if (std::find(cur->listenPorts.begin(), cur->listenPorts.end(), port_) == cur->listenPorts.end()) cur->listenPorts.push_back(port_);
}
bool NetworkServer::hasClient()
{
    for (auto& c : cur->pending) if (c->port == port_) return true;
    return false;
}
NetworkClient NetworkServer::accept()
{
    for (auto it = cur->pending.begin(); it != cur->pending.end(); ++it)
    {
        if ((*it)->port != port_) continue;
        std::shared_ptr<fake::Conn> c = *it;
        cur->pending.erase(it);
        c->accepted = true;
        return NetworkClient(std::make_shared<fake::ConnHandle>(c));
    }
    return NetworkClient();
}

// ---------------------------------------------------------------- web server (as the core's WebServer.cpp / Parsing.cpp)
static bool sameNoCase(const std::string& a, const char* b)
{
    if (a.size() != strlen(b)) return false;
    for (size_t i = 0; i < a.size(); i++) if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return false;
    return true;
}
static bool startsWith(const std::string& a, const std::string& b) { return a.compare(0, b.size(), b) == 0; }
static std::string trimmed(const std::string& s)
{
    size_t a = 0, b = s.size();
    while (a < b && isspace((unsigned char)s[a])) a++;
    while (b > a && isspace((unsigned char)s[b - 1])) b--;
    return s.substr(a, b - a);
}
static std::string subFrom(const std::string& s, size_t from) { return from >= s.size() ? std::string() : s.substr(from); }
static std::string urlDecode(const std::string& text)
{
    std::string out;
    char temp[] = "0x00";
    const size_t len = text.size();
    size_t i = 0;
    while (i < len)
    {
        char c = text[i++];
        if (c == '%' && i + 1 < len)
        {
            temp[2] = text[i++];
            temp[3] = text[i++];
            c = (char)strtol(temp, nullptr, 16);
        }
        else if (c == '+') c = ' ';
        out += c;
    }
    return out;
}
static const char* codeText(int code)
{
    switch (code)
    {
    case 200: return "OK";
    case 204: return "No Content";
    case 302: return "Found";
    case 304: return "Not Modified";
    case 400: return "Bad Request";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 408: return "Request Time-out";
    case 409: return "Conflict";
    case 414: return "Request-URI Too Large";
    case 500: return "Internal Server Error";
    }
    return "";
}

WebServer::WebServer(int) {}
WebServer::~WebServer() {}
void WebServer::on(const char* uri, HTTPMethod method, Handler fn) { routes_.push_back({ uri, method, fn, nullptr }); }
void WebServer::on(const char* uri, HTTPMethod method, Handler fn, Handler ufn) { routes_.push_back({ uri, method, fn, ufn }); }
void WebServer::begin() { cur->webListening = true; }
void WebServer::collectHeaders(const char* keys[], size_t n)
{
    headers_.clear();
    headers_.push_back({ "Authorization", "" });
    headers_.push_back({ "If-None-Match", "" });
    for (size_t i = 0; i < n; i++) headers_.push_back({ keys[i], "" });
}
void WebServer::collectHeader(const std::string& name, const std::string& value)
{
    for (auto& h : headers_) if (sameNoCase(name, h.first.c_str())) { h.second = value; return; }
}
String WebServer::header(const char* name)
{
    for (auto& h : headers_) if (sameNoCase(h.first, name)) return String(h.second);
    return String();
}
String WebServer::arg(const char* name)
{
    for (auto& a : postArgs_) if (a.first == name) return String(a.second);
    for (auto& a : args_) if (a.first == name) return String(a.second);
    return String();
}
bool WebServer::hasArg(const char* name)
{
    for (auto& a : postArgs_) if (a.first == name) return true;
    for (auto& a : args_) if (a.first == name) return true;
    return false;
}

void WebServer::handleClient()
{
    if (_currentStatus == HC_NONE)
    {
        _currentClient = _server.accept();
        if (!_currentClient) { delay(1); return; }
        _currentStatus = HC_WAIT_READ;
        _statusChange = millis();
    }
    bool keep = false, callYield = false;
    if (_currentClient.connected())
    {
        if (_currentStatus == HC_WAIT_READ)
        {
            if (_currentClient.available())
            {
                _currentClient.setTimeout(HTTP_MAX_SEND_WAIT);
                if (parseRequest())
                {
                    respHeaders_.clear();
                    handleRequest();
                }
            }
            else
            {
                if (millis() - _statusChange <= HTTP_MAX_DATA_WAIT) keep = true;
                callYield = true;
            }
        }
        else if (_currentStatus == HC_WAIT_CLOSE && millis() - _statusChange <= HTTP_MAX_CLOSE_WAIT) keep = callYield = true;
    }
    if (!keep)
    {
        _currentClient = NetworkClient();
        _currentStatus = HC_NONE;
        haveUpload_ = false;
    }
    if (callYield) yield();
}

// One line of the request, without its CR LF: bounded in length, in the time one line may take and (phaseBudget) in
// the time the whole head may take. It waits, turning in place, for bytes that have not arrived.
WebServer::Line WebServer::readLine(std::string& line, size_t maxLength, unsigned long phaseStart, unsigned long phaseBudget)
{
    line.clear();
    const unsigned long idleTimeout = _currentClient.getTimeout();
    const unsigned long lineStart = millis();
    unsigned long lastActivity = lineStart;
    while (millis() - lastActivity < idleTimeout)
    {
        if (millis() - lineStart >= WEBSERVER_MAX_LINE_WAIT) return Line::TimedOut;
        if (phaseBudget && millis() - phaseStart >= phaseBudget) return Line::TimedOut;
        const int c = _currentClient.read();
        if (c < 0) { yield(); continue; }
        lastActivity = millis();
        if (c == '\r')
        {
            while (millis() - lastActivity < idleTimeout)
            {
                const int next = _currentClient.peek();
                if (next < 0) { yield(); continue; }
                if (next == '\n') _currentClient.read();
                break;
            }
            return Line::Ok;
        }
        if (line.size() >= maxLength) return Line::TooLong;
        line += (char)c;
    }
    return Line::TimedOut;
}

// The answer to a request dropped part-way; what the browser already sent is read away first (for up to 50 ms of
// silence), because closing with unread input would reset the connection and the answer with it.
void WebServer::errorResponse(const char* status)
{
    const std::string r = std::string("HTTP/1.1 ") + status + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    _currentClient.write((const uint8_t*)r.data(), r.size());
    size_t drained = 0;
    unsigned long lastData = millis();
    while (drained < 2 * WEBSERVER_MAX_LINE_LEN && millis() - lastData < 50)
    {
        if (_currentClient.available() <= 0) { yield(); continue; }
        _currentClient.read();
        drained++;
        lastData = millis();
    }
}

void WebServer::parseArguments(const std::string& data)
{
    args_.clear();
    size_t pos = 0;
    while (pos <= data.size() && args_.size() < 256)
    {
        const size_t eq = data.find('=', pos), amp = data.find('&', pos);
        if (eq == std::string::npos || (amp != std::string::npos && eq > amp))
        {
            if (amp == std::string::npos) break;
            pos = amp + 1;
            continue;
        }
        args_.push_back({ urlDecode(data.substr(pos, eq - pos)), urlDecode(amp == std::string::npos ? data.substr(eq + 1) : data.substr(eq + 1, amp - eq - 1)) });
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
}

bool WebServer::parseRequest()
{
    postArgs_.clear();
    haveUpload_ = false;
    const unsigned long headStart = millis();
    std::string req;
    switch (readLine(req, WEBSERVER_MAX_LINE_LEN, headStart, WEBSERVER_MAX_HEADER_WAIT))
    {
    case Line::TooLong:  errorResponse("414 URI Too Long"); return false;
    case Line::TimedOut: errorResponse("408 Request Timeout"); return false;
    case Line::Ok:       break;
    }
    for (auto& h : headers_) h.second.clear();
    const size_t a = req.find(' ');
    const size_t b = a == std::string::npos ? a : req.find(' ', a + 1);
    if (a == std::string::npos || b == std::string::npos) return false;
    const std::string methodStr = req.substr(0, a);
    std::string url = req.substr(a + 1, b - a - 1), search;
    version_ = atoi(subFrom(req, b + 8).c_str());
    const size_t q = url.find('?');
    if (q != std::string::npos) { search = url.substr(q + 1); url = url.substr(0, q); }
    uri_ = url;
    host_.clear();
    if (url.size() + search.size() > WEBSERVER_MAX_URI_LEN) { errorResponse("414 URI Too Long"); return false; }
    static const struct { const char* name; HTTPMethod m; } METHODS[] = {
        { "GET", HTTP_GET }, { "HEAD", HTTP_HEAD }, { "POST", HTTP_POST }, { "PUT", HTTP_PUT }, { "PATCH", HTTP_PATCH },
        { "DELETE", HTTP_DELETE }, { "OPTIONS", HTTP_OPTIONS } };
    method_ = HTTP_ANY;
    for (auto& m : METHODS) if (methodStr == m.name) method_ = m.m;
    if (method_ == HTTP_ANY) return false;
    route_ = nullptr;
    for (auto& r : routes_) if ((r.method == HTTP_ANY || r.method == method_) && r.uri == uri_) { route_ = &r; break; }

    if (method_ == HTTP_POST || method_ == HTTP_PUT || method_ == HTTP_PATCH || method_ == HTTP_DELETE)
    {
        std::string boundary;
        bool isForm = false, isEncoded = false;
        long contentLength = 0;
        while (true)
        {
            switch (readLine(req, WEBSERVER_MAX_LINE_LEN, headStart, WEBSERVER_MAX_HEADER_WAIT))
            {
            case Line::TooLong:  errorResponse("431 Request Header Fields Too Large"); return false;
            case Line::TimedOut: errorResponse("408 Request Timeout"); return false;
            case Line::Ok:       break;
            }
            if (req.empty()) break;
            const size_t div = req.find(':');
            if (div == std::string::npos) break;
            const std::string name = req.substr(0, div), value = trimmed(req.substr(div + 1));
            collectHeader(name, value);
            if (sameNoCase(name, "Content-Type"))
            {
                if (startsWith(value, "text/plain")) isForm = false;
                else if (startsWith(value, "application/x-www-form-urlencoded")) { isForm = false; isEncoded = true; }
                else if (startsWith(value, "multipart/"))
                {
                    boundary = subFrom(value, value.find('=') + 1);
                    boundary.erase(std::remove(boundary.begin(), boundary.end(), '"'), boundary.end());
                    if (boundary.size() > 70) return false;
                    isForm = true;
                }
            }
            else if (sameNoCase(name, "Content-Length")) contentLength = atol(value.c_str());
            else if (sameNoCase(name, "Host")) host_ = value;
        }
        if (!isForm)
        {
            // the body: waits up to HTTP_MAX_POST_WAIT for each piece of it to arrive
            std::string plain;
            while ((long)plain.size() < contentLength)
            {
                int tries = HTTP_MAX_POST_WAIT;
                while (!_currentClient.available() && tries--) delay(1);
                size_t got = (size_t)_currentClient.available();
                if (!got) break;
                while (got--) plain += (char)_currentClient.read();     // (the core takes all that arrived, also past the length)
            }
            if ((long)plain.size() < contentLength) return false;
            if (contentLength > 0)
            {
                if (isEncoded) { if (!search.empty()) search += '&'; search += plain; }
                parseArguments(search);
                if (!isEncoded) args_.push_back({ "plain", plain });
            }
            else parseArguments(search);
        }
        else
        {
            parseArguments(search);
            if (!parseForm(boundary)) return false;
        }
    }
    else
    {
        while (true)
        {
            switch (readLine(req, WEBSERVER_MAX_LINE_LEN, headStart, WEBSERVER_MAX_HEADER_WAIT))
            {
            case Line::TooLong:  errorResponse("431 Request Header Fields Too Large"); return false;
            case Line::TimedOut: errorResponse("408 Request Timeout"); return false;
            case Line::Ok:       break;
            }
            if (req.empty()) break;
            const size_t div = req.find(':');
            if (div == std::string::npos) break;
            const std::string name = req.substr(0, div), value = subFrom(req, div + 2);
            collectHeader(name, value);
            if (sameNoCase(name, "Host")) host_ = value;
        }
        parseArguments(search);
    }
    _currentClient.clear();
    return true;
}

void WebServer::uploadCall() { if (route_ && route_->ufn) route_->ufn(); }
bool WebServer::uploadAborted()
{
    if (haveUpload_)
    {
        upload_.status = UPLOAD_FILE_ABORTED;
        uploadCall();
    }
    postArgs_.clear();
    return false;
}
void WebServer::uploadWriteByte(uint8_t b)
{
    if (upload_.currentSize == HTTP_UPLOAD_BUFLEN)
    {
        uploadCall();
        upload_.totalSize += upload_.currentSize;
        upload_.currentSize = 0;
    }
    upload_.buf[upload_.currentSize++] = b;
}
// One byte of the file. When none is there it waits, 2 ms at a time, for as long as the connection's timeout says
// (the program sets it), and gives up at once when the browser closed.
int WebServer::uploadReadByte()
{
    int res = _currentClient.read();
    if (res < 0)
    {
        const unsigned long start = millis();
        const long timeoutMs = (long)_currentClient.getTimeout();
        bool timedOut = false;
        for (;;)
        {
            if (!_currentClient.connected()) return -1;
            while (!timedOut && !_currentClient.available() && _currentClient.connected())
            {
                delay(2);
                timedOut = (long)(millis() - start) >= timeoutMs;
            }
            res = _currentClient.read();
            if (res >= 0) return res;
            timedOut = (long)(millis() - start) >= timeoutMs;
            if (timedOut) return res;
        }
    }
    return res;
}

bool WebServer::parseForm(const std::string& boundary)
{
    std::string line;
    int retry = 0;
    do
    {
        if (readLine(line, WEBSERVER_MAX_LINE_LEN) != Line::Ok) return false;
        ++retry;
    } while (line.empty() && retry < 3);
    if (line != "--" + boundary) return false;
    postArgs_.clear();
    size_t skipped = 0;
    bool inEmptyRun = false;
    uint32_t emptyRunStart = 0;
    while (true)
    {
        std::string argName, argValue, argType, argFilename;
        bool argIsFile = false;
        if (!_currentClient.connected() && !_currentClient.available()) return uploadAborted();
        if (readLine(line, WEBSERVER_MAX_LINE_LEN) != Line::Ok) return uploadAborted();
        if (line.empty())
        {
            if (!inEmptyRun) { inEmptyRun = true; emptyRunStart = millis(); }
            else if (millis() - emptyRunStart > HTTP_MAX_POST_WAIT) return uploadAborted();
            if (++skipped > 64) return uploadAborted();
            continue;
        }
        inEmptyRun = false;
        if (line.size() > 19 && sameNoCase(line.substr(0, 19), "Content-Disposition"))
        {
            skipped = 0;
            size_t nameStart = line.find('=');
            if (nameStart == std::string::npos) continue;
            argName = subFrom(line, nameStart + 2);
            nameStart = argName.find('=');
            if (nameStart == std::string::npos) argName = argName.substr(0, argName.empty() ? 0 : argName.size() - 1);
            else
            {
                argFilename = nameStart + 2 < argName.size() ? argName.substr(nameStart + 2, argName.size() - 1 - (nameStart + 2)) : std::string();
                argName = argName.substr(0, argName.find('"'));
                argIsFile = true;
            }
            argType = "text/plain";
            if (readLine(line, WEBSERVER_MAX_LINE_LEN) != Line::Ok) return uploadAborted();
            while (!line.empty())
            {
                if (line.size() > 12 && sameNoCase(line.substr(0, 12), "Content-Type")) argType = subFrom(line, line.find(':') + 2);
                if (readLine(line, WEBSERVER_MAX_LINE_LEN) != Line::Ok) return uploadAborted();
            }
            if (!argIsFile)
            {
                while (true)
                {
                    if (readLine(line, 16384) != Line::Ok) return uploadAborted();
                    if (startsWith(line, "--" + boundary)) break;
                    if (!_currentClient.connected() && !_currentClient.available() && line.empty()) return uploadAborted();
                    if (!argValue.empty()) argValue += "\n";
                    argValue += line;
                }
                postArgs_.push_back({ argName, argValue });
                if (line == "--" + boundary + "--") break;
                if (postArgs_.size() >= 32) return uploadAborted();
            }
            else
            {
                haveUpload_ = true;
                upload_.status = UPLOAD_FILE_START;
                upload_.name = String(argName);
                upload_.filename = String(argFilename);
                upload_.type = String(argType);
                upload_.totalSize = 0;
                upload_.currentSize = 0;
                uploadCall();
                upload_.status = UPLOAD_FILE_WRITE;
                const std::string fast = "\r\n--" + boundary;
                size_t ptr = 0;
                while (true)
                {
                    const int ret = uploadReadByte();
                    if (ret < 0) return uploadAborted();
                    const char in = (char)ret;
                    if (in == fast[ptr])
                    {
                        if (++ptr == fast.size()) break;
                    }
                    else
                    {
                        for (size_t i = 0; i < ptr; i++) uploadWriteByte((uint8_t)fast[i]);
                        if (in == fast[0]) ptr = 1;
                        else { uploadWriteByte((uint8_t)in); ptr = 0; }
                    }
                }
                uploadCall();
                upload_.totalSize += upload_.currentSize;
                upload_.status = UPLOAD_FILE_END;
                uploadCall();
                if (!_currentClient.connected()) return uploadAborted();
                if (readLine(line, WEBSERVER_MAX_LINE_LEN) != Line::Ok) return uploadAborted();
                if (line == "--") break;
                continue;
            }
        }
        else if (line == "--" + boundary + "--") break;
        else if (++skipped > 64) return uploadAborted();
    }
    for (auto& a : args_) if (postArgs_.size() < 32) postArgs_.push_back(a);
    args_ = postArgs_;
    postArgs_.clear();
    return true;
}

void WebServer::sendHeader(const String& name, const String& value, bool first)
{
    if (name.str().find_first_of("\r\n") != std::string::npos || value.str().find_first_of("\r\n") != std::string::npos) return;
    if (first) respHeaders_.insert(respHeaders_.begin(), { name.str(), value.str() });
    else respHeaders_.push_back({ name.str(), value.str() });
}
void WebServer::prepareHeader(std::string& out, int code, const char* type, size_t len)
{
    out += "HTTP/1." + std::to_string(version_) + " " + std::to_string(code) + " " + codeText(code) + "\r\n";
    sendHeader("Content-Type", type ? type : "text/html", true);
    sendHeader("Content-Length", String((unsigned long)len));
    sendHeader("Connection", "close");
    for (auto& h : respHeaders_) out += h.first + ": " + h.second + "\r\n";
    out += "\r\n";
}
void WebServer::send(int code, const char* type, const String& body)
{
    std::string head;
    prepareHeader(head, code, type, body.length());
    _currentClientWrite(head.data(), head.size());
    if (body.length()) _currentClientWrite(body.c_str(), body.length());
}
void WebServer::send_P(int code, PGM_P type, PGM_P body)
{
    const size_t len = body ? strlen(body) : 0;
    std::string head;
    prepareHeader(head, code, type, len);
    _currentClientWrite(head.data(), head.size());
    _currentClientWrite_P(body, len);
}
void WebServer::send_P(int code, PGM_P type, PGM_P body, size_t len)
{
    std::string head;
    prepareHeader(head, code, type, len);
    _currentClientWrite(head.data(), head.size());
    _currentClientWrite_P(body, len);
}
void WebServer::handleRequest()
{
    if (route_) route_->fn();
    else if (notFound_) notFound_();
    else send(404, "text/html", String("Not found: ") + String(uri_));
    uri_.clear();
}

// ---------------------------------------------------------------- the access point's devices (esp_wifi.h, esp_netif.h)
#include "esp_wifi.h"
#include "esp_netif.h"
esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t* sta)
{
    memset(sta, 0, sizeof(*sta));
    if (!fake::cur->apOn) return -1;
    for (uint32_t ip : fake::cur->apPeers)
    {
        if (sta->num >= ESP_WIFI_MAX_CONN_NUM) break;
        wifi_sta_info_t& i = sta->sta[sta->num++];
        i.mac[0] = 0x02;
        i.mac[5] = IPAddress::fromRaw(ip)[3];
    }
    return ESP_OK;
}
static int netifDummy;
esp_netif_t* esp_netif_get_handle_from_ifkey(const char*) { return (esp_netif_t*)&netifDummy; }
esp_err_t esp_netif_dhcps_get_clients_by_mac(esp_netif_t*, int num, esp_netif_pair_mac_ip_t* p)
{
    for (int i = 0; i < num; i++)
    {
        p[i].ip.addr = 0;
        for (uint32_t ip : fake::cur->apPeers)
            if (p[i].mac[0] == 0x02 && IPAddress::fromRaw(ip)[3] == p[i].mac[5]) p[i].ip.addr = ip;
    }
    return ESP_OK;
}
