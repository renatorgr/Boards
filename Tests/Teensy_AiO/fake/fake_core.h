/* fake_core.h - a pretend Teensy 4.1 + Teensyduino core, so the real v4.5_AiO_WEB firmware runs on a PC.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Only what the firmware uses is here. Everything a call changes lives in a fake::Device (pins, serial ports,
// I2C chips, CAN, EEPROM, flash, network, the watchdog); the harness owns the device and the world around it.
// Time: fake::clockUs is the pretend clock. Every millis()/micros() call costs a microsecond (so a loop that
// waits for the clock ends), delay() costs what it asks, and each millisecond the clock crosses runs the
// world's tick (GPS bytes, Keya heartbeats, AgOpenWeb's packets arrive while the firmware waits, as they do).
// The watchdog is checked there too: not fed for 1.5 s = its warning interrupt, 2 s = a restart.
// The firmware is built as a shared library and loaded again at each start, so its variables start fresh.
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
#include <type_traits>
#include <strings.h>

#define ARDUINO_TEENSY41 1
#define __IMXRT1062__ 1
#define TEENSYDUINO 158
#define ARDUINO 10819
#define F_CPU 450000000
#define F_CPU_ACTUAL 450000000u

// ---------------------------------------------------------------- Arduino basics
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define OUTPUT_OPENDRAIN 4
#define INPUT_DISABLE 5
#define RISING 3
#define FALLING 2
#define CHANGE 4
#define LSBFIRST 0
#define MSBFIRST 1
#define PROGMEM
#define FLASHMEM
#define DMAMEM
#define EXTMEM
#define FASTRUN
#define PSTR(s) (s)
#define F(s) (s)
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define pgm_read_word(p) (*(const uint16_t*)(p))
#define pgm_read_dword(p) (*(const uint32_t*)(p))
#define memcpy_P memcpy
#define strlen_P strlen
#define strcpy_P strcpy
#define strncpy_P strncpy
#define strcmp_P strcmp
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))
#define digitalPinToInterrupt(p) (p)
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define TWO_PI 6.283185307179586476925286766559
#define HALF_PI 1.5707963267948966192313216916398
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Teensy 4.1 analog pins
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#define A8 22
#define A9 23
#define A10 24
#define A11 25
#define A12 26
#define A13 27
#define A14 38
#define A15 39
#define A16 40
#define A17 41
#define LED_BUILTIN 13
#define NUM_PINS 64

using std::isfinite;
using std::isnan;
using std::isinf;
using std::abs;

template <class A, class B> inline auto min(A a, B b) -> typename std::common_type<A, B>::type { return a < b ? a : b; }
template <class A, class B> inline auto max(A a, B b) -> typename std::common_type<A, B>::type { return a > b ? a : b; }
inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return in_max == in_min ? out_min : (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t millis();
uint32_t micros();
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
void     delayNanoseconds(uint32_t ns);
void     yield();
void     pinMode(uint8_t pin, uint8_t mode);
void     digitalWrite(uint8_t pin, uint8_t level);
uint8_t  digitalRead(uint8_t pin);
#define  digitalWriteFast digitalWrite
#define  digitalReadFast digitalRead
void     digitalToggle(uint8_t pin);
int      analogRead(uint8_t pin);
void     analogWrite(uint8_t pin, int value);
void     analogWriteFrequency(uint8_t pin, float frequency);
void     analogWriteResolution(unsigned int bits);
void     analogReadResolution(unsigned int bits);
void     analogReadAveraging(unsigned int num);
void     tone(uint8_t pin, uint16_t frequency, uint32_t duration = 0);
void     noTone(uint8_t pin);
void     attachInterrupt(uint8_t pin, void (*isr)(), int mode);
void     detachInterrupt(uint8_t pin);
inline void noInterrupts() {}
inline void interrupts() {}
inline void __disable_irq() {}
inline void __enable_irq() {}
char*    dtostrf(double val, int width, unsigned int prec, char* buf);
long     random(long howbig);
long     random(long howsmall, long howbig);
void     randomSeed(unsigned long seed);

// the core's millisecond counter (read only by the firmware)
#define systick_millis_count (millis())

// AVR port emulation: PORTD bits 0-7 are digital pins 0-7 (the Cytron direction, pin 4)
struct PortD
{
    uint8_t v = 0;
    PortD& operator|=(unsigned long m);
    PortD& operator&=(unsigned long m);
    operator uint8_t() const { return v; }
};
extern PortD PORTD;

class elapsedMillis
{
public:
    elapsedMillis() { ms_ = millis(); }
    elapsedMillis(unsigned long v) { ms_ = millis() - v; }
    operator unsigned long() const { return millis() - ms_; }
    elapsedMillis& operator=(unsigned long v) { ms_ = millis() - v; return *this; }
    elapsedMillis& operator-=(unsigned long v) { ms_ += v; return *this; }
    elapsedMillis& operator+=(unsigned long v) { ms_ -= v; return *this; }
private:
    unsigned long ms_;
};
class elapsedMicros
{
public:
    elapsedMicros() { us_ = micros(); }
    elapsedMicros(unsigned long v) { us_ = micros() - v; }
    operator unsigned long() const { return micros() - us_; }
    elapsedMicros& operator=(unsigned long v) { us_ = micros() - v; return *this; }
private:
    unsigned long us_;
};

class IntervalTimer
{
public:
    bool begin(void (*fn)(), unsigned long us) { fn_ = fn; us_ = us; on_ = true; return true; }
    bool begin(void (*fn)(), int us) { return begin(fn, (unsigned long)us); }
    bool begin(void (*fn)(), float us) { return begin(fn, (unsigned long)us); }
    void end() { on_ = false; }
    void priority(uint8_t) {}
    bool running() const { return on_; }
private:
    void (*fn_)() = nullptr;
    unsigned long us_ = 0;
    bool on_ = false;
};

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
    bool        operator==(const String& o) const { return s_ == o.s_; }
    bool        operator!=(const String& o) const { return s_ != o.s_; }
    bool        operator==(const char* o) const { return s_ == (o ? o : ""); }
    String&     operator=(const char* o) { s_ = o ? o : ""; return *this; }
    String&     operator+=(const String& o) { s_ += o.s_; return *this; }
    String&     operator+=(const char* o) { s_ += o ? o : ""; return *this; }
    String&     operator+=(char c) { s_ += c; return *this; }
    friend String operator+(const String& a, const String& b) { return String(a.s_ + b.s_); }
private:
    void fromFloat(double v, unsigned int d) { char b[64]; snprintf(b, sizeof(b), "%.*f", (int)d, v); s_ = b; }
    std::string s_;
};

// ---------------------------------------------------------------- Print / Stream
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
class Print
{
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t* b, size_t n) { size_t k = 0; while (n--) k += write(*b++); return k; }
    size_t write(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
    size_t write(const char* s, size_t n) { return write((const uint8_t*)s, n); }
    virtual int availableForWrite() { return 0; }
    virtual void flush() {}
    size_t print(const char* s) { return write(s); }
    size_t print(const String& s) { return write(s.c_str()); }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(int v, int base = DEC) { return printNum((long)v, base); }
    size_t print(unsigned int v, int base = DEC) { return printNum((unsigned long)v, base); }
    size_t print(long v, int base = DEC) { return printNum(v, base); }
    size_t print(unsigned long v, int base = DEC) { return printNum(v, base); }
    size_t print(unsigned char v, int base = DEC) { return printNum((unsigned long)v, base); }
    size_t print(double v, int digits = 2) { char b[64]; snprintf(b, sizeof(b), "%.*f", digits, v); return write(b); }
    template <class T> size_t println(T v) { size_t n = print(v); return n + write("\r\n"); }
    template <class T> size_t println(T v, int fmt) { size_t n = print(v, fmt); return n + write("\r\n"); }
    size_t println() { return write("\r\n"); }
    size_t printf(const char* f, ...) __attribute__((format(printf, 2, 3)))
    {
        char b[1024];
        va_list ap;
        va_start(ap, f);
        const int n = vsnprintf(b, sizeof(b), f, ap);
        va_end(ap);
        return write((const uint8_t*)b, n < 0 ? 0 : (size_t)std::min(n, (int)sizeof(b) - 1));
    }
private:
    size_t printNum(long v, int base)
    {
        char b[40];
        if (base == HEX) snprintf(b, sizeof(b), "%lX", v);
        else if (base == OCT) snprintf(b, sizeof(b), "%lo", v);
        else snprintf(b, sizeof(b), "%ld", v);
        return write(b);
    }
    size_t printNum(unsigned long v, int base)
    {
        char b[40];
        if (base == HEX) snprintf(b, sizeof(b), "%lX", v);
        else if (base == OCT) snprintf(b, sizeof(b), "%lo", v);
        else snprintf(b, sizeof(b), "%lu", v);
        return write(b);
    }
};
class Stream : public Print
{
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    void setTimeout(unsigned long) {}
};

namespace fake { struct SerialPort; }

// The Teensy's serial ports (Serial1..Serial8) and its USB serial (Serial)
class HardwareSerial : public Stream
{
public:
    explicit HardwareSerial(int n) : n_(n) {}
    void   begin(uint32_t baud, uint16_t format = 0);
    void   end();
    int    available() override;
    int    read() override;
    int    peek() override;
    size_t write(uint8_t b) override;
    size_t write(const uint8_t* b, size_t n) override;
    using Print::write;
    int    availableForWrite() override;
    void   flush() override {}
    void   addMemoryForRead(void* buf, size_t n);
    void   addMemoryForWrite(void* buf, size_t n);
    void   clear();
    operator bool() const { return true; }
    fake::SerialPort& port() const;
private:
    int n_;
};
typedef HardwareSerial HardwareSerialIMXRT;
typedef HardwareSerial usb_serial_class;
extern HardwareSerial Serial, Serial1, Serial2, Serial3, Serial4, Serial5, Serial6, Serial7, Serial8;
#define SERIAL_8N1 0

// ---------------------------------------------------------------- IPAddress
class IPAddress
{
public:
    IPAddress() : a_{ 0, 0, 0, 0 } {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : a_{ a, b, c, d } {}
    IPAddress(const uint8_t* p) { memcpy(a_, p, 4); }
    IPAddress(uint32_t v) { memcpy(a_, &v, 4); }
    uint8_t  operator[](int i) const { return a_[i & 3]; }
    uint8_t& operator[](int i) { return a_[i & 3]; }
    bool     operator==(const IPAddress& o) const { return memcmp(a_, o.a_, 4) == 0; }
    bool     operator!=(const IPAddress& o) const { return !(*this == o); }
    operator uint32_t() const { uint32_t v; memcpy(&v, a_, 4); return v; }
    std::string str() const { char b[20]; snprintf(b, sizeof(b), "%u.%u.%u.%u", a_[0], a_[1], a_[2], a_[3]); return b; }
private:
    uint8_t a_[4];
};

// ---------------------------------------------------------------- the chip: registers the firmware touches
// A register is a value that can do something when written (a restart, a watchdog feed)
struct Reg32
{
    uint32_t v = 0;
    std::function<void(uint32_t old, uint32_t x)> onWrite;     // may change v (write-1-to-clear, a feed sequence)
    Reg32& operator=(uint32_t x) { const uint32_t old = v; v = x; if (onWrite) onWrite(old, x); return *this; }
    Reg32& operator=(const Reg32& o) { return *this = o.v; }
    Reg32& operator|=(uint32_t x) { return *this = v | x; }
    Reg32& operator&=(uint32_t x) { return *this = v & x; }
    operator uint32_t() const { return v; }
};
namespace fake { struct Regs; extern Regs* regs(); }
struct fake::Regs
{
    Reg32 srsr, gpr5, aircr, wcr, wsr, wicr, wmcr, ccgr3, mac0, mac1, cfg0, cfg1;
};
#define SRC_SRSR (fake::regs()->srsr)
#define SRC_GPR5 (fake::regs()->gpr5)
#define SCB_AIRCR (fake::regs()->aircr)
#define WDOG1_WCR (fake::regs()->wcr)
#define WDOG1_WSR (fake::regs()->wsr)
#define WDOG1_WICR (fake::regs()->wicr)
#define WDOG1_WMCR (fake::regs()->wmcr)
#define CCM_CCGR3 (fake::regs()->ccgr3)
#define HW_OCOTP_MAC0 (fake::regs()->mac0)
#define HW_OCOTP_MAC1 (fake::regs()->mac1)
#define HW_OCOTP_CFG0 (fake::regs()->cfg0)
#define HW_OCOTP_CFG1 (fake::regs()->cfg1)
#define CCM_CCGR_ON 3
#define CCM_CCGR3_WDOG1(n) ((uint32_t)(((n) & 0x03) << 16))
#define SRC_SRSR_IPP_RESET_B         ((uint32_t)(1 << 0))
#define SRC_SRSR_LOCKUP_SYSRESETREQ  ((uint32_t)(1 << 1))
#define SRC_SRSR_CSU_RESET_B         ((uint32_t)(1 << 2))
#define SRC_SRSR_IPP_USER_RESET_B    ((uint32_t)(1 << 3))
#define SRC_SRSR_WDOG_RST_B          ((uint32_t)(1 << 4))
#define SRC_SRSR_JTAG_RST_B          ((uint32_t)(1 << 5))
#define SRC_SRSR_JTAG_SW_RST         ((uint32_t)(1 << 6))
#define SRC_SRSR_WDOG3_RST_B         ((uint32_t)(1 << 7))
#define SRC_SRSR_TEMPSENSE_RST_B     ((uint32_t)(1 << 8))
#define WDOG_WCR_WDZST  ((uint16_t)(1 << 0))
#define WDOG_WCR_WDBG   ((uint16_t)(1 << 1))
#define WDOG_WCR_WDE    ((uint16_t)(1 << 2))
#define WDOG_WCR_WDT    ((uint16_t)(1 << 3))
#define WDOG_WCR_SRS    ((uint16_t)(1 << 4))
#define WDOG_WCR_WDA    ((uint16_t)(1 << 5))
#define WDOG_WCR_SRE    ((uint16_t)(1 << 6))
#define WDOG_WCR_WDW    ((uint16_t)(1 << 7))
#define WDOG_WCR_WT(n)  ((uint16_t)(((n) & 0xFF) << 8))

enum IRQ_NUMBER_t { IRQ_LPUART1 = 20, IRQ_LPUART2, IRQ_LPUART3, IRQ_LPUART4, IRQ_LPUART5, IRQ_LPUART6, IRQ_LPUART7, IRQ_LPUART8,
                    IRQ_CAN1 = 36, IRQ_CAN2 = 37, IRQ_CAN3 = 154, IRQ_WDOG1 = 92, IRQ_PIT = 122, IRQ_ENET = 114, IRQ_USB1 = 113, IRQ_GPT1 = 100 };
void NVIC_ENABLE_IRQ(int irq);
void NVIC_DISABLE_IRQ(int irq);
#define NVIC_SET_PRIORITY(irq, prio) ((void)(irq), (void)(prio))
#define NVIC_IS_ENABLED(irq) (0)
void attachInterruptVector(IRQ_NUMBER_t irq, void (*fn)());
extern void (*_VectorsRam[176])();
inline void arm_dcache_flush(void*, uint32_t) {}
inline void arm_dcache_delete(void*, uint32_t) {}
inline void arm_dcache_flush_delete(void*, uint32_t) {}
extern "C" uint32_t set_arm_clock(uint32_t frequency);
float tempmonGetTemp();
extern "C" void* fake_teensy_ram2(uint32_t addr);       // the fixed RAM2 addresses (prep.py)
extern "C" uint32_t fake_teensy_flash();                // FLASH_BASE_ADDR (prep.py, FlashTxx.h)
extern "C" uint32_t fake_teensy_image_len();            // the running firmware's size (prep.py, zOTA.ino OTA_IMAGE_LEN)
extern "C" void eepromemu_flash_write(void* addr, const void* data, uint32_t len);
extern "C" void eepromemu_flash_erase_sector(void* addr);
extern "C" unsigned long _flashimagelen;
extern "C" unsigned long _heap_end, _ebss;
extern "C" char* __brkval;

// The core's crash record (CrashReport, at the top of RAM2) and its types
struct arm_fault_info_struct
{
    uint32_t len, ipsr, cfsr, hfsr, mmfar, bfar, ret, xpsr, temp, time, crc;
};
struct crashreport_breadcrumbs_struct
{
    uint32_t bitmask, value[6], checksum;
};
class CrashReportClass
{
public:
    operator bool();
    void clear();
    void breadcrumb(unsigned n, uint32_t v);
    size_t printTo(Print& p) const { return p.print("no crash"); }
};
extern CrashReportClass CrashReport;

// ---------------------------------------------------------------- the pretend Teensy
namespace fake
{
struct Restart { uint32_t srsr; };      // a reset: thrown by SCB_AIRCR and by the watchdog, caught by the harness

struct SerialPort
{
    int      n = 0;
    uint32_t baud = 0;                  // what the firmware set (0 = not begun)
    size_t   rxCap = 64;                // the core's 64 bytes + addMemoryForRead
    size_t   txCap = 40;                // the core's 40 bytes + addMemoryForWrite (availableForWrite)
    std::deque<uint8_t> rx;             // bytes that arrived, waiting for read()
    uint32_t rxDropped = 0;             // bytes lost because rx was full
    std::vector<uint8_t> tx;            // what the firmware wrote (the harness takes it)
    uint32_t begins = 0;                // begin() calls (a baud search shows here)
    void put(uint8_t b) { if (rx.size() >= rxCap) { rxDropped++; return; } rx.push_back(b); }
};

struct I2cChip
{
    virtual ~I2cChip() {}
    virtual void write(const uint8_t* d, size_t n) = 0;     // one transmission (beginTransmission .. endTransmission)
    virtual size_t read(uint8_t* d, size_t n) = 0;          // requestFrom
};

struct CanFrame { uint32_t id = 0; bool ext = false; uint8_t len = 0; uint8_t buf[8] = {}; };

struct UdpPacket
{
    uint64_t us = 0;
    IPAddress src, dst;
    uint16_t srcPort = 0, dstPort = 0;
    std::vector<uint8_t> data;
};

struct TcpConn                          // one browser connection to the web page
{
    std::deque<uint8_t> toModule;       // the request (the harness writes it)
    std::string fromModule;             // the answer (the firmware writes it)
    size_t window = 8192;               // what the module may still write before the browser reads (availableForWrite)
    bool browserClosed = false, moduleClosed = false, accepted = false;
};

struct Device
{
    uint64_t bootUs = 0;                // clockUs at the last start
    uint32_t millisOffset = 0;          // millis() = (clockUs - bootUs) / 1000 + millisOffset (to try the 49-day wrap)

    // pins
    uint8_t  mode[NUM_PINS] = {};
    uint8_t  level[NUM_PINS] = {};      // output level written
    uint8_t  inputLevel[NUM_PINS];      // what digitalRead gives on an input (HIGH = switch open, the pull-up)
    uint16_t analogIn[NUM_PINS] = {};   // analogRead value (10 bits)
    int      pwm[NUM_PINS] = {};        // analogWrite value (-1 = digitalWrite since)
    float    pwmFreq[NUM_PINS] = {};
    uint32_t toneHz[NUM_PINS] = {};
    std::function<void(uint8_t pin)> onPin;   // the harness's board model sees every output change

    // serial ports: 0 = USB, 1..8 = Serial1..Serial8
    SerialPort serial[9];

    // I2C: bus 0 = Wire, 1 = Wire1; only the chips present answer
    std::map<uint8_t, I2cChip*> i2c[3];

    // CAN3
    bool     canBegun = false;
    uint32_t canBaud = 0;
    std::deque<CanFrame> canRx;
    std::vector<CanFrame> canTx;

    // EEPROM (4284 bytes, the Teensy 4.1's emulation) and the 8 MB flash, kept across restarts
    std::vector<uint8_t> eeprom = std::vector<uint8_t>(4284, 0xFF);
    uint32_t eepromWrites = 0;          // bytes that changed (flash wear)
    std::vector<uint8_t> flash = std::vector<uint8_t>(0x800000 + 4096, 0xFF);   // 8 MB from a 4 KB boundary inside (fake_teensy_flash)
    uint8_t  ram2[256] = {};            // 0x2027FF00..0x2027FFFF: the core's crash record
    uint32_t imageLen = 0;              // the running firmware's size in the flash (the repository's .hex, loaded by the harness)
    bool     crashed = false;           // the core's crash record holds a crash (CrashReport)

    // reset status for the next start, the watchdog
    uint32_t nextSrsr = SRC_SRSR_IPP_RESET_B;
    bool     wdtOn = false;
    uint32_t wdtTimeoutMs = 0, wdtWarnMs = 0;
    uint64_t wdtFedUs = 0;
    bool     wdtWarned = false;
    void   (*wdtIsr)() = nullptr;
    uint32_t wdtResets = 0;

    // Ethernet
    bool     ethChip = true;            // the Teensy 4.1's network chip is there
    bool     cableIn = true;            // a cable to the tractor's switch
    bool     ethBegun = false;
    IPAddress ip;
    std::map<uint16_t, std::deque<UdpPacket>> udpIn;    // per open port
    std::vector<UdpPacket> udpOut;
    uint32_t udpInDropped = 0;
    bool     webListening = false;
    std::deque<TcpConn*> listenQueue;
    std::vector<std::unique_ptr<TcpConn>> conns;
    bool     mdnsOn = false;
    std::string mdnsHost;

    // the chip's temperature
    float    cpuTempC = 45;

    // what happened, for the harness
    uint64_t lastLoopStartUs = 0;
    uint32_t restartsAsked = 0;

    Device() { for (auto& l : inputLevel) l = HIGH; }
};
extern Device* cur;
extern uint64_t clockUs;                // the pretend clock
extern std::function<void(uint64_t ms)> worldTick;   // the world around the board, once per millisecond
extern std::function<void()> wdtWarn;   // the watchdog's warning interrupt (the harness calls wdtWarnC)
void advance(uint64_t us);              // move the clock on (runs worldTick and the watchdog)
void resetRegs(uint32_t srsr);          // the registers as after a reset with this reason
void bootReset();                       // the core and the libraries as a reset leaves them
}

// ---------------------------------------------------------------- SPI (only for the BNO085 library to build)
#define SPI_MODE3 3
class SPISettings
{
public:
    SPISettings() {}
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass
{
public:
    void begin() {}
    void beginTransaction(SPISettings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t) { return 0xFF; }
};
extern SPIClass SPI;
