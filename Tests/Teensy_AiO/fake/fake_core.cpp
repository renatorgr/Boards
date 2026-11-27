/* fake_core.cpp - the bodies of the pretend Teensy 4.1 core (fake_core.h and the library headers next to it).
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "fake_core.h"
#include "Wire.h"
#include "EEPROM.h"
#include "FlexCAN_T4.h"
#include "NativeEthernet.h"
#include "NativeEthernetUdp.h"
#include <set>

namespace fake
{
Device*  cur = nullptr;
uint64_t clockUs = 1000000;
std::function<void(uint64_t ms)> worldTick;
std::function<void()> wdtWarn;
static Regs theRegs;
Regs* regs() { return &theRegs; }
static bool inTick = false;

void advance(uint64_t us)
{
    const uint64_t to = clockUs + us;
    while (clockUs < to)
    {
        // up to the next millisecond boundary, then the world's tick for it
        const uint64_t next = (clockUs / 1000 + 1) * 1000;
        if (next > to) { clockUs = to; break; }
        clockUs = next;
        if (cur && cur->wdtOn)
        {
            const uint64_t since = clockUs - cur->wdtFedUs;
            if (!cur->wdtWarned && cur->wdtWarnMs && since >= (uint64_t)(cur->wdtTimeoutMs - cur->wdtWarnMs) * 1000)
            {
                cur->wdtWarned = true;
                if (cur->wdtIsr) cur->wdtIsr();
            }
            if (since >= (uint64_t)cur->wdtTimeoutMs * 1000)
            {
                cur->wdtResets++;
                throw Restart{ SRC_SRSR_WDOG_RST_B };
            }
        }
        if (worldTick && !inTick)
        {
            inTick = true;
            try { worldTick(clockUs / 1000); } catch (...) { inTick = false; throw; }
            inTick = false;
        }
    }
}

static void wdtFeedNow()
{
    cur->wdtFedUs = clockUs;
    cur->wdtWarned = false;
}

void resetRegs(uint32_t srsr)
{
    theRegs = Regs();
    theRegs.srsr.v = srsr;
    theRegs.srsr.onWrite = [](uint32_t old, uint32_t x) { theRegs.srsr.v = old & ~x; };   // write 1 to clear
    theRegs.aircr.onWrite = [](uint32_t, uint32_t x)
    {
        if (x == 0x05FA0004u) { cur->restartsAsked++; throw Restart{ SRC_SRSR_LOCKUP_SYSRESETREQ }; }
    };
    static uint16_t lastWsr = 0;
    theRegs.wsr.onWrite = [](uint32_t, uint32_t x)
    {
        if (x == 0xAAAA && lastWsr == 0x5555) wdtFeedNow();
        lastWsr = (uint16_t)x;
    };
    theRegs.wcr.onWrite = [](uint32_t old, uint32_t x)
    {
        if ((old & WDOG_WCR_WDE) && !(x & WDOG_WCR_WDE)) theRegs.wcr.v |= WDOG_WCR_WDE;   // WDE cannot be cleared
        cur->wdtTimeoutMs = (((theRegs.wcr.v >> 8) & 0xFF) + 1) * 500;
        if ((theRegs.wcr.v & WDOG_WCR_WDE) && !cur->wdtOn) { cur->wdtOn = true; wdtFeedNow(); }
    };
    theRegs.wicr.onWrite = [](uint32_t, uint32_t x)
    {
        cur->wdtWarnMs = (x & 0x8000) ? (x & 0xFF) * 500 : 0;
    };
    theRegs.mac0.v = 0x00A13F2Au;
    theRegs.mac1.v = 0x0004u;
    if (cur)
    {
        cur->wdtOn = false;
        cur->wdtWarned = false;
        cur->wdtTimeoutMs = cur->wdtWarnMs = 0;
        cur->wdtIsr = nullptr;
    }
}
}
using fake::cur;

// ---------------------------------------------------------------- time
static uint32_t nowMs() { return (uint32_t)((fake::clockUs - cur->bootUs) / 1000) + cur->millisOffset; }
uint32_t millis() { fake::advance(1); return nowMs(); }
uint32_t micros() { fake::advance(1); return (uint32_t)(fake::clockUs - cur->bootUs) + cur->millisOffset * 1000u; }
void delay(uint32_t ms) { fake::advance((uint64_t)ms * 1000 + 1); }
void delayMicroseconds(uint32_t us) { fake::advance(us + 1); }
void delayNanoseconds(uint32_t ns) { fake::advance(ns / 1000 + 1); }
void yield() { fake::advance(1); }

// ---------------------------------------------------------------- pins
void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= NUM_PINS) return;
    cur->mode[pin] = mode;
}
void digitalWrite(uint8_t pin, uint8_t level)
{
    if (pin >= NUM_PINS) return;
    level = level ? HIGH : LOW;
    const bool changed = cur->level[pin] != level || cur->pwm[pin] != -1;
    cur->level[pin] = level;
    cur->pwm[pin] = -1;
    if (changed && cur->onPin) cur->onPin(pin);
}
void digitalToggle(uint8_t pin) { if (pin < NUM_PINS) digitalWrite(pin, !cur->level[pin]); }
uint8_t digitalRead(uint8_t pin)
{
    if (pin >= NUM_PINS) return LOW;
    if (cur->mode[pin] == OUTPUT) return cur->level[pin];
    return cur->inputLevel[pin];
}
static unsigned readBits = 10, writeBits = 8;
int analogRead(uint8_t pin)
{
    fake::advance(15);                                      // a conversion takes about 15 us
    if (pin >= NUM_PINS) return 0;
    const uint32_t v10 = cur->analogIn[pin] > 1023 ? 1023 : cur->analogIn[pin];
    return readBits >= 10 ? (int)(v10 << (readBits - 10)) : (int)(v10 >> (10 - readBits));
}
void analogWrite(uint8_t pin, int value)
{
    if (pin >= NUM_PINS) return;
    const int maxv = (1 << writeBits) - 1;
    value = value < 0 ? 0 : value > maxv ? maxv : value;
    if (writeBits != 8) value = (int)((long)value * 255 / maxv);
    const bool changed = cur->pwm[pin] != value;
    cur->pwm[pin] = value;
    cur->level[pin] = value > 0;
    if (changed && cur->onPin) cur->onPin(pin);
}
void analogWriteFrequency(uint8_t pin, float f) { if (pin < NUM_PINS) cur->pwmFreq[pin] = f; }
void analogWriteResolution(unsigned int bits) { writeBits = bits; }
void analogReadResolution(unsigned int bits) { readBits = bits; }
void analogReadAveraging(unsigned int) {}
void tone(uint8_t pin, uint16_t f, uint32_t) { if (pin < NUM_PINS) cur->toneHz[pin] = f; }
void noTone(uint8_t pin) { if (pin < NUM_PINS) cur->toneHz[pin] = 0; }
void attachInterrupt(uint8_t, void (*)(), int) {}
void detachInterrupt(uint8_t) {}
static uint32_t rnd = 12345;
long random(long howbig) { rnd = rnd * 1103515245u + 12345u; return howbig > 0 ? (long)((rnd >> 8) % (uint32_t)howbig) : 0; }
long random(long lo, long hi) { return hi > lo ? lo + random(hi - lo) : lo; }
void randomSeed(unsigned long s) { rnd = (uint32_t)s; }
char* dtostrf(double val, int width, unsigned int prec, char* buf) { sprintf(buf, "%*.*f", width, (int)prec, val); return buf; }

PortD PORTD;
// The core's PORTD emulation (avr_emulation.h): |= writes HIGH to pins 0-7 whose bit is set, &= writes LOW to the
// pins whose bit is clear, whatever the pins were before
PortD& PortD::operator|=(unsigned long m)
{
    for (int b = 0; b < 8; b++) if (m & (1u << b)) digitalWrite((uint8_t)b, HIGH);
    v = (uint8_t)(v | m);
    return *this;
}
PortD& PortD::operator&=(unsigned long m)
{
    for (int b = 0; b < 8; b++) if (!(m & (1u << b))) digitalWrite((uint8_t)b, LOW);
    v = (uint8_t)(v & m);
    return *this;
}

// ---------------------------------------------------------------- the chip
void (*_VectorsRam[176])() = {};
void attachInterruptVector(IRQ_NUMBER_t irq, void (*fn)())
{
    if (irq == IRQ_WDOG1) cur->wdtIsr = fn;
}
void NVIC_ENABLE_IRQ(int) {}
void NVIC_DISABLE_IRQ(int) {}
extern "C" uint32_t set_arm_clock(uint32_t f) { return f; }
float tempmonGetTemp() { return cur->cpuTempC; }
extern "C" void* fake_teensy_ram2(uint32_t addr) { return cur->ram2 + ((addr - 0x2027FF00u) & 0xFF); }
// The flash starts on a 4 KB boundary, as the real one (0x60000000): FlasherX and zOTA.ino round addresses to sectors
extern "C" uint32_t fake_teensy_flash() { return ((uint32_t)(uintptr_t)cur->flash.data() + 4095u) & ~4095u; }
extern "C" uint32_t fake_teensy_image_len() { return cur->imageLen; }
unsigned long _flashimagelen = 0;
unsigned long _heap_end = 0, _ebss = 0;
char* __brkval = (char*)&_heap_end;

static uint8_t* flashAt(uint32_t addr, uint32_t len)
{
    const uint32_t base = fake_teensy_flash();
    if (addr < base || addr + len > base + 0x800000u) return nullptr;
    return (uint8_t*)(uintptr_t)addr;
}
extern "C" void eepromemu_flash_write(void* addr, const void* data, uint32_t len)
{
    fake::advance(len / 2 + 10);                            // a page program: about 0.5 ms per 256 bytes
    uint8_t* p = flashAt((uint32_t)(uintptr_t)addr, len);
    if (!p) return;
    for (uint32_t i = 0; i < len; i++) p[i] &= ((const uint8_t*)data)[i];   // NOR flash: only 1 -> 0
}
extern "C" void eepromemu_flash_erase_sector(void* addr)
{
    fake::advance(30000);                                   // a 4 KB sector erase: about 30 ms (interrupts off)
    const uint32_t a = (uint32_t)(uintptr_t)addr & ~0xFFFu;
    uint8_t* p = flashAt(a, 4096);
    if (p) memset(p, 0xFF, 4096);
}
extern "C" void eepromemu_flash_erase_32K_block(void* addr) { for (int i = 0; i < 8; i++) eepromemu_flash_erase_sector((char*)addr + i * 4096); }
extern "C" void eepromemu_flash_erase_64K_block(void* addr) { for (int i = 0; i < 16; i++) eepromemu_flash_erase_sector((char*)addr + i * 4096); }

// FlasherX (FlashTxx.c is not built: these work on the pretend flash; zOTA.ino includes FlashTxx.h as C)
extern "C" {
uint32_t flash_keep_start = 0, flash_keep_end = 0;
int flash_sector_not_erased(uint32_t address)
{
    const uint8_t* p = flashAt(address & ~0xFFFu, 4096);
    if (!p) return 1;
    for (int i = 0; i < 4096; i++) if (p[i] != 0xFF) return 1;
    return 0;
}
int flash_erase_block(uint32_t address, uint32_t size)
{
    for (uint32_t a = address & ~0xFFFu; a < address + size; a += 4096) eepromemu_flash_erase_sector((void*)(uintptr_t)a);
    return 0;
}
int flash_write_block(uint32_t addr, char* data, uint32_t count)
{
    eepromemu_flash_write((void*)(uintptr_t)addr, data, count);
    return 0;
}
int check_flash_id(uint32_t buffer, uint32_t size)
{
    const uint8_t* p = flashAt(buffer, size);
    if (!p) return 0;
    static const char id[] = "fw_teensy41";
    const size_t n = sizeof(id) - 1;
    for (uint32_t i = 0; i + n <= size; i++) if (!memcmp(p + i, id, n)) return 1;
    return 0;
}
int firmware_buffer_init(uint32_t* addr, uint32_t* size) { *addr = fake_teensy_flash() + 0x400000; *size = 0x200000; return 1; }
void firmware_buffer_free(uint32_t, uint32_t) {}
void flash_move(uint32_t dst, uint32_t src, uint32_t size)
{
    uint8_t* d = flashAt(dst, size);
    uint8_t* s = flashAt(src, size);
    if (d && s) memmove(d, s, size);
    // the rest of the flash up to the EEPROM area is erased, except the keep window
    const uint32_t base = fake_teensy_flash();
    const uint32_t end = base + 0x800000 - 64 * 4096;
    for (uint32_t a = (dst + size + 4095) & ~0xFFFu; a < end; a += 4096)
    {
        if (a >= flash_keep_start && a < flash_keep_end) continue;
        uint8_t* p = flashAt(a, 4096);
        if (p) memset(p, 0xFF, 4096);
    }
    cur->imageLen = size;
    cur->restartsAsked++;
    throw fake::Restart{ SRC_SRSR_LOCKUP_SYSRESETREQ };
}
}

CrashReportClass CrashReport;
CrashReportClass::operator bool() { return cur->crashed; }
void CrashReportClass::clear()
{
    cur->crashed = false;
    memset(cur->ram2 + 0x80, 0, 0x40);
    SRC_SRSR.v = 0;
    SRC_GPR5.v = 0;
}
void CrashReportClass::breadcrumb(unsigned n, uint32_t v)
{
    if (n < 1 || n > 6) return;
    auto* bc = (crashreport_breadcrumbs_struct*)(cur->ram2 + 0xC0);
    bc->bitmask |= 1u << (n - 1);
    bc->value[n - 1] = v;
}
SPIClass SPI;

// ---------------------------------------------------------------- serial ports
HardwareSerial Serial(0), Serial1(1), Serial2(2), Serial3(3), Serial4(4), Serial5(5), Serial6(6), Serial7(7), Serial8(8);
fake::SerialPort& HardwareSerial::port() const { return cur->serial[n_]; }
void HardwareSerial::begin(uint32_t baud, uint16_t)
{
    auto& p = port();
    p.baud = baud;
    p.begins++;
    if (n_) { p.rx.clear(); }
}
void HardwareSerial::end() { port().baud = 0; }
int HardwareSerial::available() { fake::advance(1); return (int)port().rx.size(); }
int HardwareSerial::read()
{
    auto& p = port();
    if (p.rx.empty()) return -1;
    const uint8_t b = p.rx.front();
    p.rx.pop_front();
    return b;
}
int HardwareSerial::peek() { auto& p = port(); return p.rx.empty() ? -1 : p.rx.front(); }
size_t HardwareSerial::write(uint8_t b) { return write(&b, 1); }
size_t HardwareSerial::write(const uint8_t* b, size_t n)
{
    auto& p = port();
    if (n_ == 0) { p.tx.insert(p.tx.end(), b, b + n); if (p.tx.size() > 65536) p.tx.erase(p.tx.begin(), p.tx.end() - 32768); return n; }
    if (!p.baud) return 0;
    // the core's write waits while the transmit buffer is full (the world's tick sends bytes at the baud rate)
    for (size_t i = 0; i < n; i++)
    {
        uint64_t waited = 0;
        while (p.tx.size() >= p.txCap && waited < 5000000) { fake::advance(20); waited += 20; }
        p.tx.push_back(b[i]);
    }
    return n;
}
int HardwareSerial::availableForWrite()
{
    auto& p = port();
    if (n_ == 0) return 64;
    return p.tx.size() >= p.txCap ? 0 : (int)(p.txCap - p.tx.size());
}
void HardwareSerial::addMemoryForRead(void*, size_t n) { port().rxCap = 64 + n; }
void HardwareSerial::addMemoryForWrite(void*, size_t n) { port().txCap = 40 + n; }
void HardwareSerial::clear() { port().rx.clear(); }

// ---------------------------------------------------------------- I2C
TwoWire Wire(0), Wire1(1), Wire2(2);
uint8_t TwoWire::endTransmission(bool)
{
    fake::advance(20 + out_.size() * 25);                   // 400 kHz: about 25 us a byte
    auto it = cur->i2c[bus_].find(addr_);
    if (!on_ || it == cur->i2c[bus_].end() || !it->second) return 2;   // no answer to the address
    it->second->write(out_.data(), out_.size());
    out_.clear();
    return 0;
}
uint8_t TwoWire::requestFrom(uint8_t addr, uint8_t n, bool)
{
    fake::advance(20 + n * 25);
    in_.clear();
    at_ = 0;
    auto it = cur->i2c[bus_].find(addr);
    if (!on_ || it == cur->i2c[bus_].end() || !it->second) return 0;
    in_.resize(n);
    const size_t got = it->second->read(in_.data(), n);
    in_.resize(got);
    return (uint8_t)got;
}

// ---------------------------------------------------------------- EEPROM
EEPROMClass EEPROM;
void EEPROMClass::write(int a, uint8_t v)
{
    if (a < 0 || a >= length()) return;
    if (cur->eeprom[a] == v) return;
    if (cur->eepromWritesLeft == 0) return;            // the power is gone (a scenario's cut)
    if (cur->eepromWritesLeft > 0) cur->eepromWritesLeft--;
    cur->eepromWrites++;
    cur->eeprom[a] = v;
}

// ---------------------------------------------------------------- CAN3
void fakeCanBegin(uint32_t baud) { cur->canBegun = true; if (baud) cur->canBaud = baud; }
bool fakeCanRead(CAN_message_t& m)
{
    if (!cur->canBegun || cur->canRx.empty()) return false;
    const fake::CanFrame f = cur->canRx.front();
    cur->canRx.pop_front();
    m = CAN_message_t();
    m.id = f.id;
    m.flags.extended = f.ext;
    m.len = f.len;
    memcpy(m.buf, f.buf, 8);
    m.bus = 3;
    return true;
}
int fakeCanWrite(const CAN_message_t& m)
{
    if (!cur->canBegun) return 0;
    fake::CanFrame f;
    f.id = m.id;
    f.ext = m.flags.extended;
    f.len = m.len;
    memcpy(f.buf, m.buf, 8);
    cur->canTx.push_back(f);
    if (cur->canTx.size() > 4096) cur->canTx.erase(cur->canTx.begin(), cur->canTx.begin() + 2048);
    return 1;
}

// ---------------------------------------------------------------- Ethernet
EthernetClass Ethernet;
IntervalTimer EthernetClass::_fnet_poll;
MDNSClass MDNS;
static int udpMarker;                                       // what a UDP socket looks like in socket_ptr[]
static int listenMarker;

int EthernetClass::begin(uint8_t*, unsigned long timeout, unsigned long)
{
    for (auto& s : socket_ptr) s = nullptr;
    if (!cur->ethChip) return 0;
    cur->ethBegun = true;
    fake::advance((uint64_t)timeout * 1000 + 50000);       // the PHY start, then DHCP waits (0 = none)
    _fnet_poll.begin(fnet_poll, 1000);
    return 0;                                               // no DHCP answer
}
void EthernetClass::begin(uint8_t* mac, IPAddress ip) { begin(mac, 0); setLocalIP(ip); }
EthernetLinkStatus EthernetClass::linkStatus() { return !cur->ethBegun ? Unknown : cur->cableIn ? LinkON : LinkOFF; }
EthernetHardwareStatus EthernetClass::hardwareStatus() { return cur->ethChip ? EthernetTeensy41 : EthernetNoHardware; }
IPAddress EthernetClass::localIP() { return cur->ip; }
void EthernetClass::setLocalIP(const IPAddress ip) { cur->ip = ip; }

void fnet_poll() { fake::advance(2); }
void* fnet_netif_get_default() { return cur->ethBegun ? (void*)&Ethernet : nullptr; }
fnet_socket_t fnet_socket(int, int type, int) { return (cur->ethBegun && type == SOCK_STREAM) ? (fnet_socket_t)&listenMarker : FNET_NULL; }
int fnet_socket_bind(fnet_socket_t, const fnet_sockaddr*, fnet_size_t) { return FNET_OK; }
int fnet_socket_setopt(fnet_socket_t, int, int, const void*, fnet_size_t) { return FNET_OK; }
int fnet_socket_listen(fnet_socket_t, int) { cur->webListening = true; return FNET_OK; }
fnet_socket_t fnet_socket_accept(fnet_socket_t s, fnet_sockaddr*, fnet_size_t* len)
{
    if (s != (fnet_socket_t)&listenMarker || !len || *len < 16 || cur->listenQueue.empty() || !cur->cableIn) return FNET_NULL;
    fake::TcpConn* c = cur->listenQueue.front();
    cur->listenQueue.pop_front();
    c->accepted = true;
    return (fnet_socket_t)c;
}
int fnet_socket_close(fnet_socket_t s)
{
    if (s == (fnet_socket_t)&listenMarker) { cur->webListening = false; return FNET_OK; }
    for (auto& c : cur->conns) if ((fnet_socket_t)c.get() == s) c->moduleClosed = true;
    return FNET_OK;
}

fake::TcpConn* EthernetClient::conn()
{
    if (sock_ >= MAX_SOCK_NUM) return nullptr;
    void* p = Ethernet.socket_ptr[sock_];
    for (auto& c : cur->conns) if (c.get() == p) return c.get();
    return nullptr;
}
EthernetClient::operator bool() { return conn() != nullptr; }
uint8_t EthernetClient::connected()
{
    auto* c = conn();
    if (!c || c->moduleClosed) return 0;
    return !c->browserClosed || !c->toModule.empty();
}
int EthernetClient::available() { auto* c = conn(); return c ? (int)c->toModule.size() : 0; }
int EthernetClient::read()
{
    auto* c = conn();
    if (!c || c->toModule.empty()) return -1;
    const uint8_t b = c->toModule.front();
    c->toModule.pop_front();
    return b;
}
int EthernetClient::read(uint8_t* buf, size_t n)
{
    auto* c = conn();
    if (!c) return -1;
    size_t k = 0;
    while (k < n && !c->toModule.empty()) { buf[k++] = c->toModule.front(); c->toModule.pop_front(); }
    return (int)k;
}
size_t EthernetClient::write(const uint8_t* buf, size_t n)
{
    auto* c = conn();
    if (!c || c->moduleClosed || c->browserClosed) return 0;
    fake::advance(1 + n / 64);                              // copying into the stack's buffer
    const size_t k = std::min(n, c->window);
    c->fromModule.append((const char*)buf, k);
    c->window -= k;
    return k;
}
int EthernetClient::availableForWrite() { auto* c = conn(); return (c && !c->browserClosed) ? (int)c->window : 0; }
void EthernetClient::stop()
{
    auto* c = conn();
    if (c) c->moduleClosed = true;
    if (sock_ < MAX_SOCK_NUM && Ethernet.socket_ptr[sock_] != (fnet_socket_t)&udpMarker) Ethernet.socket_ptr[sock_] = nullptr;
}

int MDNSClass::begin(const char* host, int) { cur->mdnsOn = true; cur->mdnsHost = host ? host : ""; return 1; }
void MDNSClass::setServiceName(const char*) {}
int MDNSClass::addService(const char*, uint16_t, const fnet_mdns_txt_key_t* (*)()) { return 1; }

uint8_t EthernetUDP::begin(uint16_t port)
{
    if (!cur->ethBegun) return 0;
    for (auto& s : Ethernet.socket_ptr)
        if (s == nullptr)
        {
            s = (fnet_socket_t)&udpMarker;
            port_ = port;
            cur->udpIn[port];                               // the port is open from now on
            return 1;
        }
    return 0;
}
void EthernetUDP::stop() { cur->udpIn.erase(port_); port_ = 0; }
int EthernetUDP::parsePacket()
{
    fake::advance(1);
    in_.clear();
    at_ = 0;
    if (!port_ || !cur->cableIn) return 0;
    auto it = cur->udpIn.find(port_);
    if (it == cur->udpIn.end() || it->second.empty()) return 0;
    const fake::UdpPacket p = it->second.front();
    it->second.pop_front();
    in_ = p.data;
    inIp_ = p.src;
    inPort_ = p.srcPort;
    return (int)in_.size();
}
int EthernetUDP::read(uint8_t* buf, size_t n)
{
    size_t k = 0;
    while (k < n && at_ < in_.size()) buf[k++] = in_[at_++];
    return (int)k;
}
int EthernetUDP::beginPacket(IPAddress ip, uint16_t port)
{
    out_.clear();
    outIp_ = ip;
    outPort_ = port;
    return cur->ethBegun ? 1 : 0;
}
int EthernetUDP::endPacket()
{
    fake::advance(5 + out_.size() / 32);
    if (!cur->ethBegun) return 0;
    if (cur->cableIn)
    {
        fake::UdpPacket p;
        p.us = fake::clockUs;
        p.src = cur->ip;
        p.srcPort = port_;
        p.dst = outIp_;
        p.dstPort = outPort_;
        p.data = out_;
        cur->udpOut.push_back(p);
        if (cur->udpOut.size() > 20000) cur->udpOut.erase(cur->udpOut.begin(), cur->udpOut.begin() + 10000);
    }
    out_.clear();
    return 1;
}

namespace fake
{
// a fresh start of the program: what the core and the libraries hold goes back to how a reset leaves it
void bootReset()
{
    for (auto& s : Ethernet.socket_ptr) s = nullptr;
    PORTD.v = 0;
    readBits = 10;
    writeBits = 8;
}
}
