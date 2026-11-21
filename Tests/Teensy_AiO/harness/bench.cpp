/* bench.cpp - the test bench (bench.h).
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bench.h"
#include "checks.h"
#include <cmath>
#include <dlfcn.h>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace bench
{
// ---------------------------------------------------------------- the installations
static const KitInfo KITS[] = {
    { Kit::PumaCytron, "um982-cytron",  "UM982 + TM171, valve through the Cytron, steer button",  Gnss::UM982, Imu::TM171,  Act::Cytron,  2 },
    { Kit::F9pIbt2,    "f9p-ibt2",      "F9P + BNO085 (RVC), motor through an IBT2, no switch",    Gnss::F9P,   Imu::BnoRvc, Act::IBT2,    0 },
    { Kit::Um982Keya,  "um982-keya",    "UM982 dual (no IMU), Keya motor on CAN, steer switch",    Gnss::UM982, Imu::None,   Act::Keya,    1 },
    { Kit::F9pDanfoss, "f9p-danfoss",   "F9P + TM171, Danfoss valve, no switch",                   Gnss::F9P,   Imu::TM171,  Act::Danfoss, 0 },
};
const KitInfo& info(Kit k) { for (auto& i : KITS) if (i.id == k) return i; return KITS[0]; }
std::vector<Kit> allKits() { return { Kit::PumaCytron, Kit::F9pIbt2, Kit::Um982Keya, Kit::F9pDanfoss }; }
std::vector<Kit> valveKits() { return { Kit::PumaCytron, Kit::F9pIbt2, Kit::F9pDanfoss }; }

// the board's pins (Autosteer.ino and the AiO v4.5 schematic)
#define PIN_PWM1   2        // Cytron PWM, IBT2 LPWM, Danfoss signal
#define PIN_PWM2   3        // Cytron: the valve relay / lock valve; IBT2 RPWM; Danfoss: the PVE's power relay
#define PIN_DIR    4        // Cytron DIR, IBT2 enable
#define PIN_STEER  32
#define PIN_WORK   34
#define PIN_REMOTE 37

std::vector<uint8_t> pgn(uint8_t number, const std::vector<uint8_t>& data, uint8_t src)
{
    std::vector<uint8_t> p = { 0x80, 0x81, src, number, (uint8_t)data.size() };
    p.insert(p.end(), data.begin(), data.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < p.size(); i++) sum += p[i];
    p.push_back(sum);
    return p;
}

std::string fmt1(float v)
{
    char b[32];
    snprintf(b, sizeof(b), "%.1f", v);
    return b;
}

static float frand(uint32_t& s)
{
    s = s * 1103515245u + 12345u;
    return ((s >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f;      // -1 .. 1
}

// ---------------------------------------------------------------- the module
static std::string libDir()
{
    const char* d = getenv("TEENSY_LIB_DIR");
    return d && d[0] ? d : "build";
}

// The running firmware in the pretend flash: the repository's .hex (as a Teensy that was given it over USB), so
// the update keeps a real previous firmware and can go back to it. Returns the image's size, 0 without a .hex.
static uint32_t loadHex(fake::Device& d)
{
    const char* path = getenv("TEENSY_HEX");
    if (!path) return 0;
    std::ifstream f(path);
    if (!f) return 0;
    uint8_t* const flash = (uint8_t*)(((uintptr_t)d.flash.data() + 4095u) & ~(uintptr_t)4095u);   // as fake_teensy_flash()
    uint32_t base = 0, top = 0;
    std::string l;
    auto hx = [&](size_t i) { return (uint32_t)std::stoul(l.substr(i, 2), nullptr, 16); };
    while (std::getline(f, l))
    {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (l.size() < 11 || l[0] != ':') continue;
        const uint32_t n = hx(1), addr = hx(3) << 8 | hx(5), type = hx(7);
        if (type == 4) base = (hx(9) << 24) | (hx(11) << 16);
        else if (type == 0)
            for (uint32_t i = 0; i < n; i++)
            {
                const uint32_t a = base + addr + i;
                if (a < 0x60000000u || a >= 0x60800000u) continue;
                flash[a - 0x60000000u] = (uint8_t)hx(9 + 2 * i);
                top = std::max(top, a - 0x60000000u + 1);
            }
    }
    return top;
}

Aio::Aio()
{
    static int copies = 0;
    soPath_ = libDir() + "/run_" + std::to_string(getpid()) + "_" + std::to_string(copies++) + ".so";
    std::ifstream src(libDir() + "/aio.so", std::ios::binary);
    if (!src) throw std::runtime_error("firmware library missing: " + libDir() + "/aio.so (run ./run.sh)");
    std::ofstream dst(soPath_, std::ios::binary);
    dst << src.rdbuf();
    dst.close();
    fake::cur = &dev;
    // the running firmware in the flash: the repository's .hex (zOTA.ino reads its size as OTA_IMAGE_LEN)
    dev.imageLen = loadHex(dev);
    if (!dev.imageLen) dev.imageLen = 420000;
}

Aio::~Aio()
{
    unload();
    unlink(soPath_.c_str());
}

void Aio::load()
{
    fake::cur = &dev;
    fake::resetRegs(dev.nextSrsr);
    dev.nextSrsr = SRC_SRSR_IPP_RESET_B;
    fake::bootReset();
    dev.bootUs = fake::clockUs;
    so_ = dlopen(soPath_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!so_) throw std::runtime_error(std::string("dlopen: ") + dlerror());
    setup_ = (void (*)())dlsym(so_, "_Z5setupv");
    loop_ = (void (*)())dlsym(so_, "_Z4loopv");
    faultHookC_ = (void (*)(const uint32_t*))dlsym(so_, "faultHookC");
    wdtWarnC_ = (void (*)(const uint32_t*))dlsym(so_, "wdtWarnC");
    if (!setup_ || !loop_) throw std::runtime_error("setup/loop not found in the firmware library");
    boots++;
    hung = false;
}

// A reset or the power gone: the program and its RAM go, the pins float (the drivers see nothing: off)
void Aio::unload()
{
    if (!so_) return;
    dlclose(so_);
    so_ = nullptr;
    setup_ = loop_ = nullptr;
    faultHookC_ = wdtWarnC_ = nullptr;
    for (int p = 0; p < NUM_PINS; p++) { dev.mode[p] = INPUT; dev.level[p] = LOW; dev.pwm[p] = 0; dev.toneHz[p] = 0; }
    dev.wdtOn = false;
    dev.wdtIsr = nullptr;
    dev.ethBegun = false;
    dev.webListening = false;
    dev.listenQueue.clear();
    for (auto& c : dev.conns) c->moduleClosed = true;
    dev.udpIn.clear();
    dev.mdnsOn = false;
    dev.canBegun = false;
    dev.canRx.clear();
    dev.canTx.clear();
    for (auto& s : dev.serial) { s.baud = 0; s.rx.clear(); s.tx.clear(); s.rxCap = 64; s.txCap = 40; }
}

// setup(), again after a restart that happens inside it
void Aio::boot()
{
    for (int tries = 0; tries < 20; tries++)
    {
        load();
        char why[64];
        snprintf(why, sizeof(why), "srsr 0x%X at %llu ms", (unsigned)fake::regs()->srsr.v, (unsigned long long)(fake::clockUs / 1000));
        bootReasons.push_back(why);
        try { setup_(); return; }
        catch (fake::Restart& r) { unload(); dev.nextSrsr = r.srsr; }
    }
    throw std::runtime_error("the module restarts over and over inside setup()");
}

void Aio::powerOn()
{
    if (so_) return;
    dev.nextSrsr = SRC_SRSR_IPP_RESET_B;
    boot();
}

void Aio::powerOff()
{
    unload();
}

void Aio::step()
{
    if (!so_) { fake::advance(1000); return; }
    const uint64_t t0 = fake::clockUs;
    try
    {
        if (hung) fake::advance(loopCostUs);          // stopped: only the clock and the watchdog go on
        else loop_();
    }
    catch (fake::Restart& r)
    {
        unload();
        dev.nextSrsr = r.srsr;
        boot();
        return;
    }
    const uint64_t took = fake::clockUs - t0;
    if (!hung && took > loopMaxUs) loopMaxUs = took;
    if (took < loopCostUs)
    {
        try { fake::advance(loopCostUs - took); }
        catch (fake::Restart& r) { unload(); dev.nextSrsr = r.srsr; boot(); }
    }
}

// A fault in the firmware: the crash hook runs (outputs off, the Keya told), the core records the crash and waits
// with every interrupt off (the watchdog warning cannot run) until the watchdog resets it, or 8 s
void Aio::crash()
{
    if (!so_) return;
    if (faultHookC_) faultHookC_(nullptr);
    dev.crashed = true;
    auto* fi = (arm_fault_info_struct*)(dev.ram2 + 0x80);
    fi->len = sizeof(arm_fault_info_struct);
    fi->cfsr = 0x00008200;              // a precise bus fault with a valid address
    fi->bfar = 0xDEADBEE0;
    fi->ret = 0x0000ABCD;
    dev.wdtIsr = nullptr;
    const uint64_t t0 = fake::clockUs;
    try
    {
        while (fake::clockUs - t0 < 8000000ull) fake::advance(1000);
        throw fake::Restart{ SRC_SRSR_LOCKUP_SYSRESETREQ };
    }
    catch (fake::Restart& r)
    {
        unload();
        dev.nextSrsr = r.srsr;
        boot();
    }
}

std::string Aio::logLine(const std::string& text, size_t from) const
{
    const size_t at = log.find(text, from);
    if (at == std::string::npos) return "";
    const size_t b = log.rfind('\n', at);
    const size_t e = log.find('\n', at);
    return log.substr(b == std::string::npos ? 0 : b + 1, (e == std::string::npos ? log.size() : e) - (b == std::string::npos ? 0 : b + 1));
}

// ---------------------------------------------------------------- ADS1115
void Ads1115::write(const uint8_t* d, size_t n)
{
    if (n < 1) return;
    pointer = d[0] & 3;
    if (n >= 3 && pointer == 1)
    {
        config = (uint16_t)(d[1] << 8 | d[2]);
        if (config & 0x8000)
        {
            const int mux = (config >> 12) & 7, pga = (config >> 9) & 7, dr = (config >> 5) & 7;
            static const float FS[8] = { 6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f, 0.256f, 0.256f };
            static const int SPS[8] = { 8, 16, 32, 64, 128, 250, 475, 860 };
            float v;
            switch (mux)
            {
            case 0: v = ain[0] - ain[1]; break;
            case 1: v = ain[0] - ain[3]; break;
            case 2: v = ain[1] - ain[3]; break;
            case 3: v = ain[2] - ain[3]; break;
            default: v = ain[mux - 4]; break;
            }
            long code = lroundf(v / FS[pga] * 32768.0f);
            if (code > 32767) code = 32767;
            if (code < -32768) code = -32768;
            pending = (int16_t)code;
            readyUs = fake::clockUs + 1000000ull / SPS[dr] + 50;
            converting = true;
        }
    }
}

void Ads1115::tick()
{
    if (converting && fake::clockUs >= readyUs) { result = pending; converting = false; }
}

size_t Ads1115::read(uint8_t* d, size_t n)
{
    tick();
    const uint16_t v = pointer == 0 ? (uint16_t)result : pointer == 1 ? (uint16_t)((config & 0x7FFF) | (converting ? 0 : 0x8000)) : 0;
    if (n > 0) d[0] = (uint8_t)(v >> 8);
    if (n > 1) d[1] = (uint8_t)v;
    return n < 2 ? n : 2;
}

float Was::volts(float wheelDeg, uint32_t& seed) const
{
    if (shortToGround) return 0.0f;
    if (unplugged) return floatV + 0.01f * frand(seed);
    float v = centreV + (mountedBackwards ? -1.0f : 1.0f) * vPerDeg * wheelDeg + noiseV * frand(seed);
    return v < 0 ? 0 : v > 5.0f ? 5.0f : v;
}

// ---------------------------------------------------------------- the program
void Program::send(const std::vector<uint8_t>& pkt, uint16_t port, IPAddress dst)
{
    fake::UdpPacket p;
    p.us = fake::clockUs;
    p.src = ip;
    p.srcPort = 9999;
    p.dst = dst == IPAddress() ? IPAddress(ip[0], ip[1], ip[2], 255) : dst;
    p.dstPort = port;
    p.data = pkt;
    out.push_back(p);
}

void Program::send252()
{
    send(pgn(252, { ss.kp, ss.highPwm, ss.lowPwm, ss.minPwm, ss.counts, (uint8_t)ss.wasOffset, (uint8_t)((uint16_t)ss.wasOffset >> 8), ss.ackerman }));
}

void Program::send251()
{
    const uint8_t s0 = (sc.invertWas << 0) | (sc.relayActiveHigh << 1) | (sc.motorDir << 2) | (sc.singleWas << 3) | (sc.cytron << 4) |
                       (sc.steerSwitch << 5) | (sc.steerButton << 6) | (sc.encoder << 7);
    const uint8_t s1 = (sc.danfoss << 0) | (sc.pressure << 1) | (sc.current << 2) | (sc.yAxis << 3);
    send(pgn(251, { s0, sc.pulseCountMax, sc.minKmhX10, s1, 0, 0, 0, 0 }));
}

void Program::sendSubnet(uint8_t a, uint8_t b, uint8_t c)
{
    send(pgn(201, { 201, 201, a, b, c }), 8888, IPAddress(255, 255, 255, 255));
}

void Program::scan()
{
    send(pgn(202, { 202, 202, 5 }), 8888, IPAddress(255, 255, 255, 255));
}

std::vector<uint8_t> Program::packet254(float kmh) const
{
    const uint16_t sp = (uint16_t)lroundf(kmh * 10.0f);
    const int16_t sa = (int16_t)lroundf(setDeg * 100.0f);
    return pgn(254, { (uint8_t)sp, (uint8_t)(sp >> 8), (uint8_t)(engage ? 1 : 0), (uint8_t)sa, (uint8_t)((uint16_t)sa >> 8),
                      (uint8_t)(engage ? 127 : 255), 0, 0 });
}

std::vector<Heard> Program::heardPgn(uint8_t number, uint64_t sinceMs) const
{
    std::vector<Heard> r;
    for (auto& h : heard) if (h.ms >= sinceMs && h.pgn() == number) r.push_back(h);
    return r;
}

bool Program::screenHas(const std::string& text, size_t from) const
{
    for (size_t i = from; i < screen.size(); i++) if (screen[i].find(text) != std::string::npos) return true;
    return false;
}

// ---------------------------------------------------------------- the world
static World* theWorld = nullptr;

World::World(Kit kit) : ki(info(kit))
{
    theWorld = this;
    rx.kind = ki.gnss;
    rx.on = ki.gnss != Gnss::None;
    rx.hpr = ki.gnss == Gnss::UM982;
    imu.kind = ki.imu;
    imu.on = ki.imu != Imu::None;
    keya.fitted = ki.act == Act::Keya;
    tractor.lockValveFitted = ki.act == Act::Cytron || ki.act == Act::Danfoss;
    if (ki.act == Act::IBT2 || ki.act == Act::Keya) tractor.deadPwm = 15;
    aio.dev.i2c[1][0x48] = &ads;
    fake::worldTick = [this](uint64_t ms) { tick(ms); };
    fake::wdtWarn = nullptr;
    setUpProgram();
}

World::~World()
{
    // TEENSY_DUMP_LOG=<folder>: the module's log of each scenario run into <folder>/<kit>-<pid>.log (to look into one)
    if (const char* dir = getenv("TEENSY_DUMP_LOG"))
    {
        std::ofstream f(std::string(dir) + "/" + ki.key + "-" + std::to_string(getpid()) + ".log");
        f << aio.log;
    }
    fake::worldTick = nullptr;
    theWorld = nullptr;
}

void World::setUpProgram()
{
    // the careful user: the WAS zeroed (offset), counts measured, the right driver, the switch mode that is wired
    const float rawCentre = was.centreV / (6.144f / 16384.0f);
    aow.ss = SteerSettings();
    aow.ss.counts = (uint8_t)lroundf(was.vPerDeg / (6.144f / 16384.0f));
    aow.ss.wasOffset = (int16_t)lroundf(6805.0f - rawCentre);
    aow.sc = SteerConfig();
    aow.sc.cytron = ki.act != Act::IBT2;
    aow.sc.danfoss = ki.act == Act::Danfoss;
    aow.sc.steerSwitch = ki.steerMode == 1;
    aow.sc.steerButton = ki.steerMode == 2;
    if (ki.act == Act::Danfoss) aow.ss.highPwm = 250;
}

bool World::relayOn() const
{
    // pin 3 drives the valve relay: on when the firmware drives it to the "steering" level for the configuration
    // the hardware is wired for (Invert relays off: HIGH)
    const auto& d = aio.dev;
    if (!aio.on() || d.mode[PIN_PWM2] != OUTPUT || d.pwm[PIN_PWM2] > 0) return false;
    return d.level[PIN_PWM2] == HIGH;
}

static int duty(const fake::Device& d, int pin)
{
    if (d.pwm[pin] > 0) return d.pwm[pin];
    if (d.pwm[pin] < 0 && d.mode[pin] == OUTPUT && d.level[pin]) return 255;
    return 0;
}

bool World::outputActive() const
{
    if (!aio.on()) return keya.fitted && keya.enabled && keya.speedCmd != 0;
    const auto& d = aio.dev;
    switch (ki.act)
    {
    case Act::Cytron:  return relayOn() || duty(d, PIN_PWM1) > 0;
    case Act::Danfoss: return relayOn();
    case Act::IBT2:    return d.mode[PIN_DIR] == OUTPUT && d.level[PIN_DIR] && (duty(d, PIN_PWM1) > 0 || duty(d, PIN_PWM2) > 0);
    case Act::Keya:    return keya.enabled && keya.speedCmd != 0;
    }
    return false;
}

// An IBT2 driven as an IBT2 keeps its enable (pin 4) on while it steers, with both PWM at 0 when the wheels are
// where they should be. Driven as a Cytron (the module's setting, from AgOpenWeb or its defaults), pin 4 is the
// direction and pin 3 the relay, written as a plain output: only the outputs count then.
bool World::engaged() const
{
    if (outputActive()) return true;
    const auto& d = aio.dev;
    return ki.act == Act::IBT2 && aio.on() && d.pwm[PIN_PWM2] >= 0 && d.mode[PIN_DIR] == OUTPUT && d.level[PIN_DIR];
}

void World::tickSteering(uint64_t)
{
    Tractor& t = tractor;
    const auto& d = aio.dev;
    float dps = 0;
    const float span = 255.0f - t.deadPwm;
    switch (ki.act)
    {
    case Act::Cytron:
    {
        const bool lock = !t.lockValveFitted || relayOn();
        const int p = aio.on() ? duty(d, PIN_PWM1) : 0;
        if (lock && p > t.deadPwm) dps = t.maxRateDps * (p - t.deadPwm) / span * (d.level[PIN_DIR] ? -1.0f : 1.0f);
        break;
    }
    case Act::IBT2:
    {
        const bool en = aio.on() && d.mode[PIN_DIR] == OUTPUT && d.level[PIN_DIR];
        const int net = aio.on() ? duty(d, PIN_PWM1) - duty(d, PIN_PWM2) : 0;
        if (en && abs(net) > t.deadPwm) dps = t.maxRateDps * (abs(net) - t.deadPwm) / span * (net > 0 ? -1.0f : 1.0f);
        break;
    }
    case Act::Danfoss:
    {
        const bool power = !t.lockValveFitted || relayOn();
        const float dt = aio.on() ? duty(d, PIN_PWM1) / 255.0f : 0;
        if (power && dt >= 0.1f && dt <= 0.9f)            // outside 10-90 % the PVE sees a fault and stays in neutral
        {
            float flow = (dt - 0.5f) / 0.25f;
            flow = flow > 1 ? 1 : flow < -1 ? -1 : flow;
            if (fabsf(flow) > 0.04f) dps = -t.maxRateDps * flow;
        }
        break;
    }
    case Act::Keya:
        if (keya.fitted && keya.powered && keya.enabled) dps = -t.maxRateDps * 1.2f * keya.speedCmd / 1000.0f;
        break;
    }
    if (t.wiredBackwards) dps = -dps;
    t.moduleDps = dps;
    float w = t.wheelDeg + (dps + t.handDps) * 0.001f;
    if (w > t.lockDeg) w = t.lockDeg;
    if (w < -t.lockDeg) w = -t.lockDeg;
    t.wheelDeg = w;
    // the tractor drives
    const float v = t.kmh / 3.6f;
    const float h = t.headingDeg * (float)M_PI / 180.0f;
    t.headingDeg += (float)(v / t.wheelbase * tanf(t.wheelDeg * (float)M_PI / 180.0f) * 0.001 * 180.0 / M_PI);
    if (t.headingDeg >= 360) t.headingDeg -= 360;
    if (t.headingDeg < 0) t.headingDeg += 360;
    t.x += v * sinf(h) * 0.001f;
    t.y += v * cosf(h) * 0.001f;
    t.travelM += fabsf(v) * 0.001f;
    // the WAS and the ADS1115
    ads.ain[0] = was.volts(t.wheelDeg, seed_);
    ads.ain[1] = was.noLowWire ? 4.9f : 0.0f;
    ads.tick();
}

// ---------------------------------------------------------------- the receiver
static std::string nmeaSum(const std::string& body)
{
    uint8_t s = 0;
    for (char c : body) s ^= (uint8_t)c;
    char b[8];
    snprintf(b, sizeof(b), "*%02X\r\n", s);
    return "$" + body + b;
}

static std::string latLon(double deg, bool lat)
{
    const double a = fabs(deg);
    const int d = (int)a;
    const double m = (a - d) * 60.0;
    char b[40];
    snprintf(b, sizeof(b), lat ? "%02d%011.8f" : "%03d%011.8f", d, m);
    return b;
}

void World::tickGnss(uint64_t ms)
{
    Receiver& r = rx;
    if (r.on && r.rateHz > 0 && ms - r.lastEpochMs >= (uint64_t)(1000 / r.rateHz))
    {
        r.lastEpochMs = ms;
        r.epochs++;
        const double lat0 = 39.5, lon0 = -8.0;
        const double lat = lat0 + tractor.y / 111320.0, lon = lon0 + tractor.x / (111320.0 * cos(lat0 * M_PI / 180.0));
        const uint64_t tod = 36000000ull + ms;                    // 10:00:00 + the bench's time
        char t[16];
        snprintf(t, sizeof(t), "%02u%02u%02u.%02u", (unsigned)(tod / 3600000 % 24), (unsigned)(tod / 60000 % 60), (unsigned)(tod / 1000 % 60),
                 (unsigned)(tod / 10 % 100));
        const char* talker = r.kind == Gnss::UM982 ? "GN" : "GN";
        char b[200];
        std::string s;
        if (r.garbage)
            for (int i = 0; i < 180; i++) s += (char)(seed_ = seed_ * 1103515245u + 12345u, (seed_ >> 16) & 0xFF);
        else
        {
            snprintf(b, sizeof(b), "%sGGA,%s,%s,N,%s,W,%c,%02d,0.6,120.000,M,50.000,M,%s,0000", talker, t, latLon(lat, true).c_str(),
                     latLon(lon, false).c_str(), r.fix, r.fix == '0' ? 0 : r.sats, r.fix >= '2' ? "1.0" : "");
            s += nmeaSum(b);
            const double kn = tractor.kmh / 1.852;
            snprintf(b, sizeof(b), "%sVTG,%.2f,T,,M,%.3f,N,%.3f,K,%c", talker, tractor.headingDeg, kn, tractor.kmh, r.fix == '0' ? 'N' : 'D');
            s += nmeaSum(b);
            if (r.kind == Gnss::UM982 && r.hpr)
            {
                snprintf(b, sizeof(b), "GNHPR,%s,%.2f,%.2f,%.2f,%d,%d,1.0,0000", t, tractor.headingDeg, tractor.rollDeg, 0.0, r.fix == '4' ? 4 : 5, r.sats);
                s += nmeaSum(b);
            }
        }
        for (char c : s) r.wire.push_back((uint8_t)c);
        if (r.wire.size() > 20000) r.wire.erase(r.wire.begin(), r.wire.begin() + 10000);
    }
    // bytes on the wire at the baud rate
    r.credit += r.baud / 10000.0;
    auto& port = aio.dev.serial[r.port];
    while (r.credit >= 1 && !r.wire.empty())
    {
        r.credit -= 1;
        uint8_t c = r.wire.front();
        r.wire.pop_front();
        if (!aio.on() || port.baud == 0) continue;
        if (port.baud != r.baud) c = (uint8_t)(c * 7 + 0x35);  // the wrong speed: framing errors, other bytes
        port.put(c);
    }
    if (r.wire.empty()) r.credit = 0;
}

// ---------------------------------------------------------------- the IMU
static uint16_t modbusCrc(const uint8_t* p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++)
    {
        crc ^= p[i];
        for (int k = 0; k < 8; k++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void tmFrame(std::deque<uint8_t>& w, uint8_t fn, const std::vector<uint8_t>& content)
{
    std::vector<uint8_t> f = { 0xAA, 0x55, (uint8_t)(4 + content.size()) };
    const uint32_t info = fn | (107u << 10) | (1u << 21);     // from the TM171 (ID 107)
    f.push_back((uint8_t)info); f.push_back((uint8_t)(info >> 8)); f.push_back((uint8_t)(info >> 16)); f.push_back((uint8_t)(info >> 24));
    f.insert(f.end(), content.begin(), content.end());
    const uint16_t crc = modbusCrc(f.data() + 2, f.size() - 2);
    f.push_back((uint8_t)crc);
    f.push_back((uint8_t)(crc >> 8));
    for (uint8_t b : f) w.push_back(b);
}

static void putF(std::vector<uint8_t>& v, float f)
{
    uint8_t b[4];
    memcpy(b, &f, 4);
    v.insert(v.end(), b, b + 4);
}

void World::tickImu(uint64_t ms)
{
    ImuDev& m = imu;
    if (m.on && m.kind != Imu::None)
    {
        float roll = tractor.rollDeg + 0.05f * frand(seed_), yaw = tractor.headingDeg + 0.05f * frand(seed_);
        if (m.frozen) { roll = m.frozenRoll; yaw = m.frozenYaw; }
        else { m.frozenRoll = roll; m.frozenYaw = yaw; }
        if (m.kind == Imu::TM171 && ms - m.lastMs >= 40)
        {
            m.lastMs = ms;
            std::vector<uint8_t> c;
            putF(c, (float)(ms % 100000));                        // its time stamp
            putF(c, roll);
            putF(c, 0.3f);
            putF(c, yaw > 180 ? yaw - 360 : yaw);
            tmFrame(m.wire, 35, c);
        }
        if (m.kind == Imu::TM171 && ms - m.lastStatusMs >= 1000)
        {
            m.lastStatusMs = ms;
            std::vector<uint8_t> c;
            putF(c, 0);
            putF(c, 31.5f);
            c.push_back(0); c.push_back(0); c.push_back(3); c.push_back(0);
            tmFrame(m.wire, 22, c);
        }
        if (m.kind == Imu::BnoRvc && ms - m.lastMs >= 10)
        {
            m.lastMs = ms;
            const int16_t y = (int16_t)lroundf((yaw > 180 ? yaw - 360 : yaw) * 100), p = (int16_t)lroundf(roll * 100), r = 30;
            std::vector<uint8_t> f = { 0xAA, 0xAA, m.rvcIndex++, (uint8_t)y, (uint8_t)(y >> 8), (uint8_t)p, (uint8_t)(p >> 8),
                                       (uint8_t)r, (uint8_t)(r >> 8), 0, 0, 0, 0, 0xE8, 0x03, 0, 0, 0 };
            uint8_t sum = 0;
            for (size_t k = 2; k < f.size(); k++) sum += f[k];
            f.push_back(sum);
            for (uint8_t b : f) m.wire.push_back(b);
        }
        if (m.wire.size() > 8000) m.wire.erase(m.wire.begin(), m.wire.begin() + 4000);
    }
    m.credit += m.baud / 10000.0;
    auto& port = aio.dev.serial[5];
    while (m.credit >= 1 && !m.wire.empty())
    {
        m.credit -= 1;
        uint8_t c = m.wire.front();
        m.wire.pop_front();
        if (!aio.on() || port.baud == 0) continue;
        if (port.baud != m.baud) c = (uint8_t)(c * 7 + 0x35);
        port.put(c);
    }
    if (m.wire.empty()) m.credit = 0;
}

// ---------------------------------------------------------------- the Keya
void World::tickKeya(uint64_t ms)
{
    Keya& k = keya;
    auto& d = aio.dev;
    if (!k.fitted) { d.canTx.clear(); return; }
    // the commands the module sent
    for (const auto& f : d.canTx)
    {
        if (!k.powered || !k.canWired || f.id != 0x06000001 || !f.ext) continue;
        if (f.buf[0] == 0x23 && f.buf[1] == 0x0D && f.buf[2] == 0x20) { if (!k.enabled) k.enables++; k.enabled = true; k.lastCmdMs = ms; }
        else if (f.buf[0] == 0x23 && f.buf[1] == 0x0C && f.buf[2] == 0x20) { k.enabled = false; k.speedCmd = 0; }
        else if (f.buf[0] == 0x23 && f.buf[1] == 0x00 && f.buf[2] == 0x20) { k.speedCmd = (int16_t)(f.buf[4] << 8 | f.buf[5]); k.lastCmdMs = ms; k.speedCmds++; }
    }
    d.canTx.clear();
    if (!k.powered) { k.enabled = false; k.speedCmd = 0; return; }
    if (k.enabled && ms - k.lastCmdMs > k.cmdTimeoutMs) { k.enabled = false; k.speedCmd = 0; }   // its own safety stop
    const float rpmLike = k.enabled ? k.speedCmd / 10.0f : 0;
    k.motorDeg += tractor.moduleDps * 0.001f * 15.0f + tractor.handDps * 0.001f * 15.0f;   // the steering wheel turns ~15x the wheels
    k.amps = k.enabled ? 0.5f + fabsf(k.speedCmd) / 1000.0f * 3.0f + (tractor.handDps != 0 ? 4.0f : 0) : 0.2f;
    if (ms - k.lastBeatMs >= 20)
    {
        k.lastBeatMs = ms;
        if (!k.canWired || !d.canBegun || d.canBaud != 250000 || !aio.on()) return;
        const int16_t pos = (int16_t)(-lroundf(k.motorDeg * 10)), spd = (int16_t)lroundf(tractor.handDps != 0 ? 0 : rpmLike), cur = (int16_t)lroundf(k.amps);
        fake::CanFrame f;
        f.id = 0x07000001;
        f.ext = true;
        f.len = 8;
        f.buf[0] = (uint8_t)(pos >> 8); f.buf[1] = (uint8_t)pos;
        f.buf[2] = (uint8_t)(spd >> 8); f.buf[3] = (uint8_t)spd;
        f.buf[4] = (uint8_t)(cur >> 8); f.buf[5] = (uint8_t)cur;
        f.buf[6] = (uint8_t)(k.faults >> 8); f.buf[7] = (uint8_t)((k.faults & 0xFE) | (k.enabled ? 0 : 1));
        if (d.canRx.size() < 256) d.canRx.push_back(f);
    }
}

// ---------------------------------------------------------------- the network
static bool reaches(const IPAddress& dst, const IPAddress& host)
{
    if (dst == IPAddress(255, 255, 255, 255)) return true;
    if (dst[0] != host[0] || dst[1] != host[1] || dst[2] != host[2]) return false;
    return dst[3] == 255 || dst[3] == host[3];
}

void World::deliverToModule(const fake::UdpPacket& p)
{
    auto& d = aio.dev;
    if (!aio.on() || !cableIn || !tabletOnNet || !d.ethBegun || !reaches(p.dst, d.ip)) return;
    if (aow.dropEvery && ++aow.sentToModule % aow.dropEvery == 0) return;
    auto it = d.udpIn.find(p.dstPort);
    if (it == d.udpIn.end()) return;
    if (it->second.size() >= 64) { d.udpInDropped++; return; }
    it->second.push_back(p);
    // what the module has been asked (for the rules)
    const auto& x = p.data;
    if (p.dstPort == 8888 && x.size() >= 13 && x[0] == 0x80 && x[1] == 0x81 && x[2] == 0x7F && x[3] == 254)
    {
        const float kmh = (x[5] | x[6] << 8) * 0.1f;
        if ((x[7] & 1) && kmh >= 0.1f) lastAskMs = now();
        else lastStopMs = now();
        setAtModule = (int16_t)(x[8] | x[9] << 8) * 0.01f;
    }
}

void World::tickNetwork(uint64_t ms)
{
    Program& a = aow;
    if (a.on)
    {
        if (a.sendHello && ms - a.lastHelloMs >= 1000)
        {
            a.lastHelloMs = ms;
            std::vector<uint8_t> h = { 0x80, 0x81, 0x7F, 200, 3, 56, 0, 0 };
            uint8_t ck = 0;
            for (size_t i = 2; i < h.size(); i++) ck += h[i];
            h.push_back(a.agio ? 0x47 : ck);
            a.send(h);
        }
        if (a.send254 && ms - a.last254Ms >= a.period254)
        {
            a.last254Ms = ms;
            if (a.guide && a.engage)
            {
                // pure pursuit to the line x = lineX, northwards
                const float ld = std::max(3.0f, tractor.kmh / 3.6f * 2.0f);
                const float alpha = atan2f(a.lineX - tractor.x, ld) - tractor.headingDeg * (float)M_PI / 180.0f;
                float st = atanf(2.0f * tractor.wheelbase * sinf(alpha) / ld) * 180.0f / (float)M_PI;
                a.setDeg = st > 35 ? 35 : st < -35 ? -35 : st;
            }
            a.send(a.packet254(a.kmhOverride >= 0 ? a.kmhOverride : tractor.kmh));
        }
    }
    while (!a.out.empty())
    {
        deliverToModule(a.out.front());
        a.out.pop_front();
    }
    // from the module
    auto& d = aio.dev;
    for (const auto& p : d.udpOut)
    {
        if (!cableIn || !tabletOnNet || !a.on || !reaches(p.dst, a.ip) || p.dstPort != 9999) continue;
        if (!p.data.empty() && p.data[0] == '$')
        {
            std::string s(p.data.begin(), p.data.end());
            size_t at = 0;
            while (at < s.size())
            {
                size_t e = s.find("\r\n", at);
                if (e == std::string::npos) e = s.size();
                const std::string l = s.substr(at, e - at);
                at = e + 2;
                if (l.size() < 4 || l[0] != '$') continue;
                a.nmeaLines++;
                a.lastNmeaMs = ms;
                const size_t star = l.rfind('*');
                uint8_t sum = 0;
                for (size_t i = 1; i < star && star != std::string::npos; i++) sum ^= (uint8_t)l[i];
                if (star == std::string::npos || star + 3 > l.size() || strtol(l.substr(star + 1, 2).c_str(), nullptr, 16) != sum) a.nmeaBadSum++;
                if (l.find("GGA,") != std::string::npos) a.nmeaGga++;
                if (a.nmea.size() < 400) a.nmea.push_back(l);
            }
            continue;
        }
        Heard h { ms, p.srcPort, p.data };
        if (h.pgn() == 221 && p.data.size() >= 8)
            a.screen.push_back(std::string(p.data.begin() + 7, p.data.end() - 1));
        if (a.heard.size() > 40000) a.heard.erase(a.heard.begin(), a.heard.begin() + 20000);
        a.heard.push_back(h);
    }
    d.udpOut.clear();
    // the browser reads what the module wrote (the socket's window opens again)
    for (auto& c : d.conns) if (!c->browserClosed && c->window < 8192) c->window = std::min<size_t>(8192, c->window + 4000);
    // the module's other serial ports drain at their baud rate; the receiver reads what comes to it at its own speed
    for (int i = 1; i <= 8; i++)
    {
        auto& s = d.serial[i];
        if (!s.baud || s.tx.empty()) continue;
        size_t n = std::max<size_t>(1, s.baud / 10000);
        n = std::min(n, s.tx.size());
        if (i == rx.port && rx.on && s.baud == rx.baud)
        {
            rx.bytesFromModule += n;
            rx.fromModule.append(s.tx.begin(), s.tx.begin() + n);
            // a command that sets its port to 460800: "CONFIG COM1 460800" (UM982), UBX-CFG-VALSET with 460800 (u-blox)
            static const char ubx460800[4] = { 0x00, 0x08, 0x07, 0x00 };
            if (rx.baud != 460800 && (rx.fromModule.find("460800") != std::string::npos ||
                                      rx.fromModule.find(std::string(ubx460800, 4)) != std::string::npos))
            {
                rx.baud = 460800;
                rx.baudChanges++;
                rx.fromModule.clear();
            }
            if (rx.fromModule.size() > 4096) rx.fromModule.erase(0, 2048);
        }
        s.tx.erase(s.tx.begin(), s.tx.begin() + n);
    }
    // the USB serial: the log
    if (!d.serial[0].tx.empty())
    {
        aio.log.append(d.serial[0].tx.begin(), d.serial[0].tx.end());
        d.serial[0].tx.clear();
    }
}

// ---------------------------------------------------------------- the rules
Rule& World::rule(const std::string& key, const std::string& what)
{
    for (auto& r : rules) if (r.key == key) return r;
    rules.push_back({ key, what, 0, 0, "" });
    return rules.back();
}

void World::violate(const std::string& key, const std::string& what, const std::string& detail)
{
    Rule& r = rule(key, what);
    if (!r.violations) { r.firstMs = now(); r.firstDetail = detail; }
    r.violations++;
}

void World::expectOff(const std::string& why, uint32_t graceMs)
{
    offWhy_ = why;
    offFrom_ = now() + graceMs;
}

void World::tickRules(uint64_t ms)
{
    const bool active = outputActive();
    if (engaged()) engagedMs++;
    if (active && !wasActive_) activeSinceMs_ = ms;
    wasActive_ = active;
    rule("steer-only-when-asked", "Steering outputs only while AgOpenWeb asks (within its 2.5 s watchdog)");
    rule("right-way", "The module steers towards the asked angle, never away from it");
    if (ruleSteerOnlyWhenAsked && active)
    {
        // asked: the last steering packet asked to steer, at most 2.5 s ago (+ a steering loop and the packet's way);
        // a packet that says stop is acted on within 60 ms
        const bool askedNow = lastAskMs && lastAskMs >= lastStopMs && ms - lastAskMs <= 2500 + 80;
        const bool stopping = lastStopMs > lastAskMs && ms - lastStopMs <= 60 && lastAskMs && lastStopMs - lastAskMs <= 2580;
        bool switchOpen = false;
        if (aow.sc.steerSwitch && ki.steerMode == 1 && !sw.steerClosed) switchOpen = true;
        if ((!askedNow && !stopping) || (switchOpen && ms - switchOpenedMs > 60))
            violate("steer-only-when-asked", "Steering outputs only while AgOpenWeb asks (within its 2.5 s watchdog)",
                    chk::fmt("outputs on at %llu ms: %s%s", (unsigned long long)ms,
                             !lastAskMs ? "AgOpenWeb never asked to steer"
                                        : chk::fmt("last ask %llu ms ago, last stop %llu ms ago", (unsigned long long)(ms - lastAskMs),
                                                   (unsigned long long)(lastStopMs ? ms - lastStopMs : 0)).c_str(),
                             switchOpen ? ", steer switch open" : ""));
    }
    if (!offWhy_.empty() && active && ms >= offFrom_)
        violate("off-after-fault", "Steering off after a fault or a stop",
                chk::fmt("outputs still on %llu ms after: %s", (unsigned long long)(ms - offFrom_), offWhy_.c_str()));
    // the right way: pushing the wheels while they are more than 2 deg from the asked angle, away from it, for 80 ms
    if (ruleRightWay && tractor.moduleDps != 0 && ms - activeSinceMs_ > 100)
    {
        const float err = tractor.wheelDeg - setAtModule;
        const int dir = tractor.moduleDps > 0 ? 1 : -1;
        if (fabsf(err) > 2.0f && (err > 0) == (dir > 0))
        {
            if (!wrongDir_) { wrongDir_ = dir; wrongSinceMs_ = ms; wrongFrom_ = tractor.wheelDeg; }
            if (ms - wrongSinceMs_ > 80 && fabsf(tractor.wheelDeg - wrongFrom_) > maxWrongWayDeg) maxWrongWayDeg = fabsf(tractor.wheelDeg - wrongFrom_);
            if (ms - wrongSinceMs_ == 81)
                violate("right-way", "The module steers towards the asked angle, never away from it",
                        chk::fmt("wheels at %.1f deg, asked %.1f deg, pushed further away for 80 ms", tractor.wheelDeg, setAtModule));
        }
        else wrongDir_ = 0;
    }
    else wrongDir_ = 0;
    if (active && ms - activeSinceMs_ > 3000 && lastAskMs && ms - lastAskMs < 200)
    {
        const float e = fabsf(tractor.wheelDeg - setAtModule);
        if (e > maxAbsErrDeg) maxAbsErrDeg = e;
    }
}

void World::tick(uint64_t ms)
{
    tickSteering(ms);
    tickKeya(ms);
    tickGnss(ms);
    tickImu(ms);
    // the switches (closed = LOW), the sensors
    auto& d = aio.dev;
    d.inputLevel[PIN_STEER] = sw.steerClosed ? LOW : HIGH;
    d.inputLevel[PIN_WORK] = sw.workClosed ? LOW : HIGH;
    d.inputLevel[PIN_REMOTE] = sw.remoteClosed ? LOW : HIGH;
    if (sw.steerClosed != lastSteerClosed_) { lastSteerClosed_ = sw.steerClosed; if (!sw.steerClosed) switchOpenedMs = ms; }
    tickNetwork(ms);
    tickRules(ms);
    for (auto& w : watchers) w();
}

void World::run(uint32_t ms)
{
    const uint64_t end = fake::clockUs + (uint64_t)ms * 1000;
    while (fake::clockUs < end) aio.step();
}

void World::runUntil(const std::function<bool()>& done, uint32_t maxMs)
{
    const uint64_t end = fake::clockUs + (uint64_t)maxMs * 1000;
    while (fake::clockUs < end && !done()) aio.step();
}

void World::startUp(uint32_t ms)
{
    aio.powerOn();
    run(300);
    aow.sendSettings();
    run(ms > 300 ? ms - 300 : 0);
}

// ---------------------------------------------------------------- the browser
Http World::httpRaw(const std::string& request, uint32_t timeoutMs)
{
    Http h;
    auto& d = aio.dev;
    if (!aio.on() || !cableIn || !tabletOnNet) { run(std::min<uint32_t>(timeoutMs, 200)); return h; }
    d.conns.push_back(std::make_unique<fake::TcpConn>());
    fake::TcpConn* c = d.conns.back().get();
    for (char ch : request) c->toModule.push_back((uint8_t)ch);
    if (d.webListening) d.listenQueue.push_back(c);
    const uint64_t t0 = fake::clockUs;
    runUntil([&] { return c->moduleClosed; }, timeoutMs);
    h.ms = (uint32_t)((fake::clockUs - t0) / 1000);
    c->browserClosed = true;
    const std::string& r = c->fromModule;
    const size_t he = r.find("\r\n\r\n");
    if (r.compare(0, 5, "HTTP/") == 0 && he != std::string::npos)
    {
        h.code = atoi(r.c_str() + 9);
        h.head = r.substr(0, he);
        h.body = r.substr(he + 4);
        if (h.body.size() && (h.body[0] == '{' || h.body[0] == '[')) h.json = Json::parse(h.body, h.jsonOk);
    }
    return h;
}

Http World::http(const std::string& method, const std::string& path, const std::string& body,
                 const std::vector<std::string>& headers, uint32_t timeoutMs)
{
    std::string r = method + " " + path + " HTTP/1.1\r\nHost: 192.168.5.126\r\n";
    for (auto& x : headers) r += x + "\r\n";
    if (method == "POST") r += "Content-Type: application/x-www-form-urlencoded\r\nContent-Length: " + std::to_string(body.size()) + "\r\n";
    r += "Connection: close\r\n\r\n" + body;
    return httpRaw(r, timeoutMs);
}

Json World::status()
{
    Http h = http("GET", "/api/status");
    if (h.code != 200 || !h.jsonOk)
    {
        badStatus++;
        lastBadStatus = h.code ? std::to_string(h.code) + " " + h.body.substr(0, 300) : "no answer";
        return Json();
    }
    return h.json;
}

// A firmware file sent to /api/ota the way the page sends it: the body is the .hex, bytesPerMs as the link allows
void World::upload(const std::vector<uint8_t>& file, Http& h, const std::string& query, uint32_t bytesPerMs)
{
    auto& d = aio.dev;
    h = Http();
    if (!aio.on() || !cableIn) return;
    std::string head = "POST /api/ota" + query + " HTTP/1.1\r\nHost: 192.168.5.126\r\nContent-Type: application/octet-stream\r\nContent-Length: " +
                       std::to_string(file.size()) + "\r\nConnection: close\r\n\r\n";
    d.conns.push_back(std::make_unique<fake::TcpConn>());
    fake::TcpConn* c = d.conns.back().get();
    for (char ch : head) c->toModule.push_back((uint8_t)ch);
    if (d.webListening) d.listenQueue.push_back(c);
    size_t at = 0;
    const uint32_t bootsBefore = aio.boots;
    const uint64_t t0 = fake::clockUs;
    watchers.push_back([&, c] {
        uint32_t n = bytesPerMs;
        while (n-- && at < file.size() && c->toModule.size() < 16384 && !c->moduleClosed) c->toModule.push_back(file[at++]);
    });
    runUntil([&] { return c->moduleClosed || aio.boots != bootsBefore; }, 120000);
    watchers.pop_back();
    h.ms = (uint32_t)((fake::clockUs - t0) / 1000);
    c->browserClosed = true;
    const std::string& r = c->fromModule;
    const size_t he = r.find("\r\n\r\n");
    if (r.compare(0, 5, "HTTP/") == 0 && he != std::string::npos)
    {
        h.code = atoi(r.c_str() + 9);
        h.head = r.substr(0, he);
        h.body = r.substr(he + 4);
        if (h.body.size() && h.body[0] == '{') h.json = Json::parse(h.body, h.jsonOk);
    }
}

void reportRules(World& w)
{
    for (auto& r : w.rules)
    {
        if (r.key == "right-way" && !w.ruleRightWay) continue;
        if (r.key == "steer-only-when-asked" && !w.ruleSteerOnlyWhenAsked) continue;
        chk::must(r.key, r.violations == 0, r.what,
                  r.violations ? chk::fmt("%u ms of it, first at %llu ms: %s", r.violations, (unsigned long long)r.firstMs, r.firstDetail.c_str()) : "");
    }
    chk::must("no-watchdog-reset", w.aio.dev.wdtResets == 0 || w.expectWatchdog, "No watchdog restart",
              chk::fmt("%u watchdog restarts", w.aio.dev.wdtResets));
    chk::must("status-json", w.badStatus == 0, "The web page's /api/status is valid JSON", w.lastBadStatus);
}
}
