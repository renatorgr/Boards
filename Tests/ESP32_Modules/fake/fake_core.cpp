/* fake_core.cpp - the bodies of the pretend ESP32 + Arduino core (fake_core.h).
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Linked into the harness program (not into the firmware library), so what the firmware does to its
// pretend board stays in fake::Device when the firmware is loaded again after a restart.
#include "fake_core.h"

namespace fake
{
Network  net;
uint64_t clockMs = 1000;
Device*  cur = nullptr;

bool Device::staUp() const
{
    if (!staBegun || !net.wifiOn || !(wifiMode & WIFI_STA)) return false;
    if (staSsid != net.ssid || staPass != net.pass) return false;
    return clockMs - staBeginAt >= 1500;    // about 1.5 s to join
}

bool Device::onTractorNet() const { return staUp() || (ethOk && cableIn); }

bool Device::sameNumbers() const
{
    if (!apOn || !staUp() || (ethOk && cableIn)) return false;
    const IPAddress t = tractorIp();
    return t[0] == apIp[0] && t[1] == apIp[1] && t[2] == apIp[2];
}

IPAddress Device::tractorIp() const
{
    if (ethOk && cableIn) return ethIp;
    if (!staUp()) return IPAddress();
    if (staStatic) return staIp;
    // DHCP from the router: an address in the router's subnet
    return IPAddress(net.subnet[0], net.subnet[1], net.subnet[2], (uint8_t)(100 + (mac & 0x3F)));
}
}

using fake::cur;

// ---------------------------------------------------------------- Arduino basics
uint32_t millis() { return (uint32_t)(fake::clockMs - cur->bootAt) + cur->millisOffset; }
uint32_t micros() { return millis() * 1000u; }
void delay(uint32_t ms) { fake::clockMs += ms; }

void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= 40) return;
    cur->mode[pin] = mode;
}

void digitalWrite(uint8_t pin, uint8_t level)
{
    if (pin >= 40) return;
    cur->level[pin] = level ? HIGH : LOW;
    cur->pinWrites++;
    if (cur->onWrite) cur->onWrite(pin, cur->level[pin]);
}

int digitalRead(uint8_t pin)
{
    if (pin >= 40) return LOW;
    if (cur->mode[pin] == OUTPUT) return cur->level[pin];
    return cur->inputLevel[pin];
}

uint32_t analogReadMilliVolts(uint8_t pin) { return pin < 40 ? cur->analogMv[pin] : 0; }
uint16_t analogRead(uint8_t pin) { return (uint16_t)std::min<uint32_t>(4095, analogReadMilliVolts(pin) * 4095 / 3100); }
void analogSetPinAttenuation(uint8_t, adc_attenuation_t) {}
void attachInterrupt(uint8_t pin, void (*isr)(), int) { if (pin < 40) cur->isr[pin] = isr; }

int gpio_set_level(gpio_num_t pin, uint32_t level)
{
    digitalWrite((uint8_t)pin, level ? HIGH : LOW);
    return 0;
}

// ---------------------------------------------------------------- Serial
HardwareSerial Serial;
void HardwareSerial::begin(unsigned long) {}
int HardwareSerial::available() { return (int)cur->serialIn.size(); }
int HardwareSerial::read()
{
    if (cur->serialIn.empty()) return -1;
    const int c = (uint8_t)cur->serialIn[0];
    cur->serialIn.erase(0, 1);
    return c;
}
size_t HardwareSerial::printf(const char* fmt, ...)
{
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    cur->serialOut += b;
    return n < 0 ? 0 : (size_t)n;
}
size_t HardwareSerial::print(const char* s) { cur->serialOut += s; return strlen(s); }
size_t HardwareSerial::println(const char* s) { cur->serialOut += s; cur->serialOut += "\n"; return strlen(s) + 1; }

// ---------------------------------------------------------------- ESP
EspClass ESP;
uint64_t EspClass::getEfuseMac() { return cur->mac; }
esp_reset_reason_t esp_reset_reason() { return cur->resetReason; }
TaskHandle_t xTaskGetCurrentTaskHandleForCore(int core)
{
    return core >= 0 && core < 2 && cur->coreTask[core][0] ? (TaskHandle_t)cur->coreTask[core] : nullptr;
}
char* pcTaskGetName(TaskHandle_t task) { return (char*)task; }
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{
    if (cpu != 0) return ESP_FAIL;          // only core 0's is kept (the sprayer's)
    cur->idleHook = cb;
    return ESP_OK;
}

// ---------------------------------------------------------------- the crash record
esp_err_t esp_core_dump_image_check(void)
{
    return cur->crash.state == 1 ? ESP_OK : cur->crash.state == 2 ? ESP_ERR_INVALID_CRC : ESP_ERR_INVALID_SIZE;
}
esp_err_t esp_core_dump_image_get(size_t* out_addr, size_t* out_size)
{
    if (!cur->crash.state) return ESP_ERR_INVALID_SIZE;
    *out_addr = 0x3F0000;
    *out_size = cur->crash.size;
    return ESP_OK;
}
esp_err_t esp_core_dump_get_summary(esp_core_dump_summary_t* s)
{
    if (cur->crash.state != 1) return ESP_ERR_INVALID_CRC;
    const fake::Device::Crash& c = cur->crash;
    memset(s, 0, sizeof(*s));
    snprintf(s->exc_task, sizeof(s->exc_task), "%s", c.task.c_str());
    s->exc_pc = c.pc;
    s->exc_bt_info.depth = (uint32_t)std::min<size_t>(c.bt.size(), 16);
    for (uint32_t i = 0; i < s->exc_bt_info.depth; i++) s->exc_bt_info.bt[i] = c.bt[i];
    snprintf((char*)s->app_elf_sha256, sizeof(s->app_elf_sha256), "%s", c.prog.c_str());
    s->ex_info.exc_cause = 28;              // LoadProhibited, when the record has no words
    return ESP_OK;
}
esp_err_t esp_core_dump_get_panic_reason(char* buf, size_t size)
{
    if (cur->crash.state != 1 || cur->crash.reason.empty()) return ESP_ERR_NOT_FOUND;
    snprintf(buf, size, "%s", cur->crash.reason.c_str());
    return ESP_OK;
}
int esp_app_get_elf_sha256(char* dst, size_t size)
{
    return snprintf(dst, size, "%s", cur->appSha.c_str());
}
void EspClass::restart()
{
    cur->restarts++;
    cur->resetReason = ESP_RST_SW;
    throw fake::Restart();
}

// ---------------------------------------------------------------- Wi-Fi
WiFiClass WiFi;
bool WiFiClass::mode(int m) { cur->wifiMode = m; return true; }
bool WiFiClass::softAPConfig(IPAddress ip, IPAddress, IPAddress) { cur->apIp = ip; return true; }
bool WiFiClass::softAP(const char* ssid, const char* pass)
{
    // the ESP32 refuses a password of 1-7 characters (and then has no own network at all)
    const size_t n = pass ? strlen(pass) : 0;
    if (n > 0 && n < 8) { cur->apOn = false; return false; }
    if (!ssid || !ssid[0] || strlen(ssid) > 32) { cur->apOn = false; return false; }
    cur->apSsid = ssid;
    cur->apPass = pass ? pass : "";
    cur->apOn = true;
    return true;
}
IPAddress WiFiClass::softAPIP() { return cur->apIp; }
int WiFiClass::softAPgetStationNum() { return cur->apOn ? cur->apClients : 0; }
bool WiFiClass::config(IPAddress ip, IPAddress gw, IPAddress mask, IPAddress)
{
    cur->staStatic = true;
    cur->staIp = ip;
    cur->staGw = gw;
    cur->staMask = mask;
    return true;
}
wl_status_t WiFiClass::begin(const char* ssid, const char* pass)
{
    cur->staBegun = true;
    cur->staSsid = ssid ? ssid : "";
    cur->staPass = pass ? pass : "";
    cur->staBeginAt = fake::clockMs;
    return WL_DISCONNECTED;
}
int WiFiClass::onEvent(WiFiEventFuncCb cb, arduino_event_id_t event)
{
    cur->wifiEvents.push_back({ event, cb });
    return (int)cur->wifiEvents.size();
}
// As the core does: "got IP" once joined, "disconnected" with ESP-IDF's reason when the link goes, and again every few
// seconds while it keeps trying (no network of that name: 201; a wrong WPA2 password usually ends the 4-way handshake: 15)
void fake::Device::tickWifi()
{
    if (wifiEvents.empty() || !staBegun || !(wifiMode & WIFI_STA)) { staLinked = false; return; }
    auto fire = [this](arduino_event_id_t e, uint8_t reason) {
        arduino_event_info_t info = {};
        info.wifi_sta_disconnected.reason = reason;
        const auto list = wifiEvents;
        for (auto& [ev, cb] : list) if (ev == ARDUINO_EVENT_MAX || ev == e) cb(e, info);
    };
    const bool up = staUp();
    if (up && !staLinked) { staLinked = true; fire(ARDUINO_EVENT_WIFI_STA_GOT_IP, 0); return; }
    if (up) return;
    if (staLinked) { staLinked = false; staEventAt = fake::clockMs; fire(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, WIFI_REASON_BEACON_TIMEOUT); return; }
    if (fake::clockMs - staBeginAt < 1500 || (staEventAt && fake::clockMs - staEventAt < 3000)) return;
    staEventAt = fake::clockMs;
    const bool found = fake::net.wifiOn && staSsid == fake::net.ssid;
    fire(ARDUINO_EVENT_WIFI_STA_DISCONNECTED, found ? WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT : WIFI_REASON_NO_AP_FOUND);
}
wl_status_t WiFiClass::status()
{
    if (cur->staUp()) return WL_CONNECTED;
    if (!cur->staBegun) return WL_IDLE_STATUS;
    if (!fake::net.wifiOn || cur->staSsid != fake::net.ssid) return WL_NO_SSID_AVAIL;
    if (cur->staPass != fake::net.pass) return WL_CONNECT_FAILED;
    return WL_DISCONNECTED;
}
IPAddress WiFiClass::localIP() { return cur->staUp() ? cur->tractorIp() : IPAddress(); }
int8_t WiFiClass::RSSI() { return cur->staUp() ? -58 : 0; }
int16_t WiFiClass::scanNetworks(bool)
{
    cur->scanState = -1;
    cur->scanAt = fake::clockMs;
    return WIFI_SCAN_RUNNING;
}
int16_t WiFiClass::scanComplete()
{
    if (cur->scanState != -1) return WIFI_SCAN_FAILED;
    if (fake::clockMs - cur->scanAt < 2500) return WIFI_SCAN_RUNNING;
    return fake::net.wifiOn ? 3 : 2;
}
void WiFiClass::scanDelete() { cur->scanState = -2; }
// What the module hears: the tractor's network (when it is on) and a neighbour's, twice (the page lists it once)
String WiFiClass::SSID(int i)
{
    if (fake::net.wifiOn && i == 0) return String(fake::net.ssid);
    return String("Neighbour");
}
int32_t WiFiClass::RSSI(int i) { return -50 - 10 * i; }
int32_t WiFiClass::channel(int i) { return 1 + 5 * i; }
wifi_auth_mode_t WiFiClass::encryptionType(int) { return WIFI_AUTH_WPA2_PSK; }

// ---------------------------------------------------------------- UDP
WiFiUDP::WiFiUDP() {}
uint8_t WiFiUDP::begin(uint16_t port) { cur->udpPort = port; return 1; }
int WiFiUDP::parsePacket()
{
    if (cur->rx.empty()) { haveIn_ = false; return 0; }
    fake::Packet p = cur->rx.front();
    cur->rx.pop_front();
    in_ = p.data;
    inIp_ = p.src;
    haveIn_ = true;
    return (int)in_.size();
}
int WiFiUDP::read(uint8_t* buf, size_t len)
{
    if (!haveIn_) return 0;
    const size_t n = std::min(len, in_.size());
    memcpy(buf, in_.data(), n);
    in_.erase(in_.begin(), in_.begin() + n);
    return (int)n;
}
IPAddress WiFiUDP::remoteIP() { return inIp_; }
int WiFiUDP::beginPacket(IPAddress ip, uint16_t port)
{
    out_.clear();
    outIp_ = ip;
    outPort_ = port;
    return 1;
}
size_t WiFiUDP::write(const uint8_t* buf, size_t len)
{
    out_.insert(out_.end(), buf, buf + len);
    return len;
}
int WiFiUDP::endPacket()
{
    if (!cur->onTractorNet()) return 0;     // no route: lwIP drops it
    if (cur->sameNumbers()) return 1;       // two networks with the same numbers: it goes out on the own one, where nobody listens
    fake::Packet p;
    p.ms = fake::clockMs;
    p.src = cur->tractorIp();
    p.srcPort = cur->udpPort;
    p.dst = outIp_;
    p.dstPort = outPort_;
    p.data = out_;
    cur->tx.push_back(p);
    return 1;
}

IPAddress WiFiClient::localIP() { return cur->req.fromAp ? cur->apIp : cur->tractorIp(); }

// ---------------------------------------------------------------- the network cable
ETHClass ETH;
bool ETHClass::begin(eth_phy_type_t type, int32_t, int, int, int, eth_clock_mode_t)
{
    cur->ethBegun = true;
    cur->ethOk = cur->ethChip && type == cur->ethChipType;
    return cur->ethOk;
}
bool ETHClass::config(IPAddress ip, IPAddress, IPAddress, IPAddress)
{
    cur->ethIp = ip;
    return true;
}
bool ETHClass::linkUp() { return cur->ethOk && cur->cableIn; }

// ---------------------------------------------------------------- web server
WebServer::WebServer(int) { if (cur) cur->web = this; }
WebServer::~WebServer() { if (cur && cur->web == this) cur->web = nullptr; }
void WebServer::on(const char* uri, HTTPMethod method, Handler fn) { routes_.push_back({ uri, method, fn, nullptr }); }
void WebServer::on(const char* uri, HTTPMethod method, Handler fn, Handler ufn) { routes_.push_back({ uri, method, fn, ufn }); }

void WebServer::handleClient()
{
    if (!cur->reqPending) return;
    cur->reqPending = false;
    cur->resp = fake::HttpResponse();
    for (auto& r : routes_)
    {
        if (r.uri != cur->req.uri || (r.method != HTTP_ANY && r.method != cur->req.method)) continue;
        if (r.ufn)
        {
            const std::vector<uint8_t>& f = cur->req.file;
            upload_.filename = String(cur->req.fileName.c_str());
            upload_.totalSize = 0;
            upload_.currentSize = 0;
            upload_.status = UPLOAD_FILE_START;
            r.ufn();
            for (size_t at = 0; at < f.size(); at += HTTP_UPLOAD_BUFLEN)
            {
                const size_t n = std::min<size_t>(HTTP_UPLOAD_BUFLEN, f.size() - at);
                memcpy(upload_.buf, f.data() + at, n);
                upload_.currentSize = n;
                upload_.totalSize += n;
                upload_.status = UPLOAD_FILE_WRITE;
                r.ufn();
            }
            upload_.currentSize = 0;
            upload_.status = UPLOAD_FILE_END;
            r.ufn();
        }
        r.fn();
        return;
    }
    if (notFound_) notFound_();
}

String WebServer::arg(const char* name)
{
    auto it = cur->req.args.find(name);
    return it == cur->req.args.end() ? String() : String(it->second.c_str());
}
bool WebServer::hasArg(const char* name) { return cur->req.args.count(name) > 0; }
void WebServer::send(int code, const char* type, const String& body)
{
    cur->resp.done = true;
    cur->resp.code = code;
    cur->resp.type = type ? type : "";
    cur->resp.body = body.str();
}
void WebServer::sendHeader(const String& name, const String& value, bool) { cur->resp.headers[name.str()] = value.str(); }
String WebServer::hostHeader() { return String(cur->req.host.c_str()); }

// ---------------------------------------------------------------- mDNS
MDNSResponder MDNS;
bool MDNSResponder::begin(const char* host) { cur->mdnsOn = true; cur->mdnsHost = host; return true; }
void MDNSResponder::setInstanceName(const String& name) { cur->mdnsInstance = name.str(); }
bool MDNSResponder::addService(const char*, const char*, uint16_t) { return true; }
bool MDNSResponder::addServiceTxt(const char*, const char*, const char*, const char*) { return true; }

// ---------------------------------------------------------------- Preferences (the ESP32's NVS)
bool Preferences::begin(const char* ns, bool readOnly)
{
    if (!ns || strlen(ns) > 15) return false;
    ns_ = ns;
    open_ = true;
    readOnly_ = readOnly;
    return true;
}
void Preferences::end() { open_ = false; }

namespace
{
const fake::PrefValue* prefGet(const std::string& ns, const char* key, char type)
{
    auto n = cur->prefs.find(ns);
    if (n == cur->prefs.end()) return nullptr;
    auto k = n->second.find(key);
    if (k == n->second.end() || k->second.type != type) return nullptr;   // NVS: another type reads as missing
    return &k->second;
}
size_t prefPut(const std::string& ns, bool open, bool readOnly, const char* key, char type, const void* p, size_t n)
{
    if (!open || readOnly || !key || strlen(key) > 15) return 0;
    fake::PrefValue v;
    v.type = type;
    v.bytes.assign((const uint8_t*)p, (const uint8_t*)p + n);
    auto& slot = cur->prefs[ns][key];
    if (slot.type != v.type || slot.bytes != v.bytes) cur->flashWrites++;
    slot = v;
    return n;
}
}

uint8_t Preferences::getUChar(const char* key, uint8_t def)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'u') : nullptr;
    return v && v->bytes.size() == 1 ? v->bytes[0] : def;
}
size_t Preferences::putUChar(const char* key, uint8_t x) { return prefPut(ns_, open_, readOnly_, key, 'u', &x, 1); }
uint32_t Preferences::getUInt(const char* key, uint32_t def)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'u') : nullptr;
    if (!v || v->bytes.size() != sizeof(uint32_t)) return def;
    uint32_t x;
    memcpy(&x, v->bytes.data(), sizeof(x));
    return x;
}
size_t Preferences::putUInt(const char* key, uint32_t x) { return prefPut(ns_, open_, readOnly_, key, 'u', &x, sizeof(x)); }
bool Preferences::getBool(const char* key, bool def)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'u') : nullptr;  // stored as a byte by the ESP32 core too
    return v && v->bytes.size() == 1 ? v->bytes[0] != 0 : def;
}
size_t Preferences::putBool(const char* key, bool x) { const uint8_t b = x; return prefPut(ns_, open_, readOnly_, key, 'u', &b, 1); }
float Preferences::getFloat(const char* key, float def)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'b') : nullptr;  // the core keeps a float as a blob
    if (!v || v->bytes.size() != sizeof(float)) return def;
    float f;
    memcpy(&f, v->bytes.data(), sizeof(f));
    return f;
}
size_t Preferences::putFloat(const char* key, float x) { return prefPut(ns_, open_, readOnly_, key, 'b', &x, sizeof(x)); }
String Preferences::getString(const char* key, const String& def)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 's') : nullptr;
    return v ? String(std::string(v->bytes.begin(), v->bytes.end())) : def;
}
size_t Preferences::putString(const char* key, const String& x) { return prefPut(ns_, open_, readOnly_, key, 's', x.c_str(), x.length()); }
size_t Preferences::getBytesLength(const char* key)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'b') : nullptr;
    return v ? v->bytes.size() : 0;
}
size_t Preferences::getBytes(const char* key, void* buf, size_t maxLen)
{
    const fake::PrefValue* v = open_ ? prefGet(ns_, key, 'b') : nullptr;
    if (!v || v->bytes.size() > maxLen) return 0;
    memcpy(buf, v->bytes.data(), v->bytes.size());
    return v->bytes.size();
}
size_t Preferences::putBytes(const char* key, const void* buf, size_t len) { return prefPut(ns_, open_, readOnly_, key, 'b', buf, len); }

// ---------------------------------------------------------------- program update
UpdateClass Update;
bool UpdateClass::begin(size_t)
{
    cur->updBegun = true;
    cur->updError = false;
    cur->updBytes = 0;
    cur->updImage.clear();
    cur->updDone = false;
    return true;
}
size_t UpdateClass::write(uint8_t* data, size_t len)
{
    if (!cur->updBegun || cur->updError) return 0;
    // as the ESP32 core: an app image starts with the magic byte 0xE9 (a .merged.bin starts with the bootloader's padding)
    if (cur->updBytes == 0 && len > 0 && data[0] != 0xE9) { cur->updError = true; return 0; }
    cur->updImage.insert(cur->updImage.end(), data, data + len);
    cur->updBytes += len;
    return len;
}
bool UpdateClass::end(bool)
{
    if (!cur->updBegun || cur->updError || cur->updBytes == 0) { cur->updError = true; return false; }
    cur->updDone = true;
    cur->updBegun = false;
    return true;
}
void UpdateClass::abort()
{
    cur->updBegun = false;
    cur->updError = true;                   // as the core: hasError() after abort()
}
bool UpdateClass::hasError() { return cur->updError; }

// ---------------------------------------------------------------- I2C
TwoWire Wire;
bool TwoWire::begin(int, int, uint32_t) { return true; }
void TwoWire::beginTransmission(uint8_t addr) { addr_ = addr; buf_.clear(); }
size_t TwoWire::write(uint8_t b) { buf_.push_back(b); return 1; }
uint8_t TwoWire::endTransmission(bool)
{
    auto p = cur->i2cPresent.find(addr_);
    if (p == cur->i2cPresent.end() || !p->second) return 2;   // no answer to the address
    if (!buf_.empty()) cur->i2c[addr_] = buf_.back();         // a PCF8574 keeps the last byte
    return 0;
}
