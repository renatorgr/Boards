/* bench.cpp - the test bench (bench.h).
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bench.h"
#include <dlfcn.h>
#include <unistd.h>
#include <fstream>
#include <stdexcept>

namespace bench
{
// ---------------------------------------------------------------- the boards (facts from each board, not from the firmware)
static const BoardInfo BOARDS[] = {
    { Board::SprayerX8,     "sprayer-x8",     "ESP32_Relay_X8",   true,  8,  "sprayer_1" },
    { Board::SprayerDevkit, "sprayer-devkit", "ESP32 dev module", true,  16, "sprayer_2" },
    { Board::SeederRelay24, "seeder-relay24", "Relay24_ESP32",    false, 24, "seeder_1" },
    { Board::SeederKC868,   "seeder-kc868",   "KC868-A32",        false, 32, "seeder_2" },
};
const BoardInfo& info(Board b) { for (auto& i : BOARDS) if (i.id == b) return i; return BOARDS[0]; }
std::vector<Board> sprayerBoards() { return { Board::SprayerX8, Board::SprayerDevkit }; }
std::vector<Board> seederBoards() { return { Board::SeederRelay24, Board::SeederKC868 }; }
std::vector<Board> allBoards() { return { Board::SprayerX8, Board::SprayerDevkit, Board::SeederRelay24, Board::SeederKC868 }; }

// ESP32_Relay_X8: relays 1-8 on these pins, a HIGH pin closes the relay (the board's transistors)
static const std::vector<uint8_t> X8_PINS = { 32, 33, 25, 26, 27, 14, 12, 13 };
// a dev module with two 8-relay modules (optocoupler inputs: a LOW pin closes the relay), wired as the README says
static const std::vector<uint8_t> DEVKIT_PINS = { 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 };
static const std::vector<uint8_t> NO_PINS;
// Relay24_ESP32: 74HC595 chain SER 13, SRCLK 16, RCLK 32, OE 4 (pull-up on the board); a HIGH output closes the relay
// through the ULN2803; relay 1 = U1 QA = the last bit shifted in
#define SR_DATA 13
#define SR_CLOCK 16
#define SR_LATCH 32
#define SR_OE 4
// KC868-A32: PCF8574 chips 0x24 (relays 1-8), 0x25 (9-16), 0x21 (17-24), 0x22 (25-32); a LOW pin closes the relay
static const uint8_t KC_CHIPS[4] = { 0x24, 0x25, 0x21, 0x22 };

const std::vector<uint8_t>& Module::relayPins(Board b)
{
    if (b == Board::SprayerX8) return X8_PINS;
    if (b == Board::SprayerDevkit) return DEVKIT_PINS;
    return NO_PINS;
}

std::vector<uint8_t> pgn(uint8_t number, const std::vector<uint8_t>& data, uint8_t src)
{
    std::vector<uint8_t> p = { 0x80, 0x81, src, number, (uint8_t)data.size() };
    p.insert(p.end(), data.begin(), data.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < p.size(); i++) sum += p[i];
    p.push_back(sum);
    return p;
}

std::string hex(uint64_t v)
{
    char b[24];
    snprintf(b, sizeof(b), "%llx", (unsigned long long)v);
    return b;
}

// ---------------------------------------------------------------- a module
static std::string libDir()
{
    const char* d = getenv("MODULE_LIB_DIR");
    return d && d[0] ? d : "build";
}

Module::Module(Board b, const std::string& l, uint64_t mac) : board(b), bi(info(b)), label(l)
{
    dev.label = l;
    dev.mac = mac;
    // a copy of the firmware library for this module, so two modules of the same kind can run side by side
    static int copies = 0;
    soPath_ = libDir() + "/run_" + std::to_string(getpid()) + "_" + std::to_string(copies++) + ".so";
    std::ifstream src(libDir() + "/" + bi.lib + ".so", std::ios::binary);
    if (!src) throw std::runtime_error(std::string("firmware library missing: ") + libDir() + "/" + bi.lib + ".so (run ./run.sh)");
    std::ofstream dst(soPath_, std::ios::binary);
    dst << src.rdbuf();
    dst.close();

    if (b == Board::SeederRelay24 || b == Board::SeederKC868)
    {
        dev.ethChip = true;
        dev.ethChipType = b == Board::SeederRelay24 ? ETH_PHY_RTL8201 : ETH_PHY_LAN8720;
    }
    if (b == Board::SeederKC868)
        for (uint8_t a : KC_CHIPS) { dev.i2cPresent[a] = true; dev.i2c[a] = 0xFF; }     // PCF8574 at power-on: pins high
    srShift_ = srOut_ = 0xA5A5A5;           // a 74HC595 wakes up with anything in it (OE held off by the pull-up)
    dev.onWrite = [this](uint8_t pin, uint8_t level) {
        if (board != Board::SeederRelay24) return;
        if (pin == SR_CLOCK && level == HIGH && lastClock_ == LOW) srShift_ = ((srShift_ << 1) | dev.level[SR_DATA]) & 0xFFFFFF;
        if (pin == SR_LATCH && level == HIGH && lastLatch_ == LOW) srOut_ = srShift_;
        if (pin == SR_CLOCK) lastClock_ = level;
        if (pin == SR_LATCH) lastLatch_ = level;
    };
}

Module::~Module()
{
    unload();
    unlink(soPath_.c_str());
}

void Module::load()
{
    fake::Device* before = fake::cur;
    fake::cur = &dev;
    dev.bootAt = fake::clockMs;
    so_ = dlopen(soPath_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!so_) throw std::runtime_error(std::string("dlopen: ") + dlerror());
    setup_ = (void (*)())dlsym(so_, "_Z5setupv");
    loop_ = (void (*)())dlsym(so_, "_Z4loopv");
    if (!setup_ || !loop_) throw std::runtime_error("setup/loop not found in the firmware library");
    boots++;
    callSetup();
    fake::cur = before;
}

void Module::callSetup()
{
    try { setup_(); }
    catch (fake::Restart&) { unload(); load(); }
}

// Power gone (or the restart): the program and everything in RAM go, the pins float, the chips' outputs drop
void Module::unload()
{
    if (!so_) return;
    fake::Device* before = fake::cur;
    fake::cur = &dev;
    dev.wifiEvents.clear();                 // the callbacks are the program's code: gone with it
    dlclose(so_);
    fake::cur = before;
    so_ = nullptr;
    setup_ = loop_ = nullptr;
    for (int p = 0; p < 40; p++) { dev.mode[p] = INPUT; dev.level[p] = LOW; dev.isr[p] = nullptr; }
    dev.web = nullptr;
    dev.reqPending = false;
    dev.wifiMode = WIFI_OFF;
    dev.apOn = false;
    dev.staBegun = false;
    dev.staLinked = false;
    dev.staEventAt = 0;
    dev.staStatic = false;
    dev.ethBegun = dev.ethOk = false;
    dev.mdnsOn = false;
    dev.udpPort = 0;
    dev.rx.clear();
    dev.tx.clear();
    dev.scanState = -2;
    dev.updBegun = false;
}

void Module::powerOn()
{
    if (so_) return;
    load();
}

void Module::powerOff()
{
    unload();
    if (board == Board::SeederKC868) for (uint8_t a : KC_CHIPS) dev.i2c[a] = 0xFF;
    srShift_ = srOut_ = 0x5A5A5A;
}

void Module::step()
{
    if (!so_) return;
    fake::Device* before = fake::cur;
    fake::cur = &dev;
    dev.tickWifi();
    try { loop_(); }
    catch (fake::Restart&)
    {
        // ESP.restart(): the chip resets; the 74HC595s keep their bits but OE floats up to "off"
        unload();
        load();
    }
    fake::cur = before;
}

uint32_t Module::relays() const
{
    uint32_t r = 0;
    switch (board)
    {
    case Board::SprayerX8:
        for (size_t i = 0; i < X8_PINS.size(); i++)
            if (dev.mode[X8_PINS[i]] == OUTPUT && dev.level[X8_PINS[i]] == HIGH) r |= 1u << i;
        break;
    case Board::SprayerDevkit:
        for (size_t i = 0; i < DEVKIT_PINS.size(); i++)
            if (dev.mode[DEVKIT_PINS[i]] == OUTPUT && dev.level[DEVKIT_PINS[i]] == LOW) r |= 1u << i;
        break;
    case Board::SeederRelay24:
        if (dev.mode[SR_OE] == OUTPUT && dev.level[SR_OE] == LOW) r = srOut_;    // else the pull-up keeps the outputs off
        break;
    case Board::SeederKC868:
        for (int c = 0; c < 4; c++)
        {
            auto p = dev.i2cPresent.find(KC_CHIPS[c]);
            if (p == dev.i2cPresent.end() || !p->second) continue;
            auto v = dev.i2c.find(KC_CHIPS[c]);
            const uint8_t b = v == dev.i2c.end() ? 0xFF : v->second;
            for (int k = 0; k < 8; k++) if (!((b >> k) & 1)) r |= 1u << (c * 8 + k);
        }
        break;
    }
    return r;
}

Http Module::http(HTTPMethod m, const std::string& uri, const std::map<std::string, std::string>& args, bool fromAp,
                  const std::vector<uint8_t>* file, const std::string& fileName)
{
    Http h;
    if (!so_ || !dev.web) return h;
    dev.req = fake::HttpRequest();
    dev.req.method = m;
    dev.req.uri = uri;
    dev.req.args = args;
    dev.req.fromAp = fromAp;
    dev.req.host = fromAp ? dev.apIp.toString().str() : dev.tractorIp().toString().str();
    if (file) { dev.req.file = *file; dev.req.fileName = fileName; }
    dev.reqPending = true;
    step();                                 // loop() -> server.handleClient()
    if (!dev.resp.done) return h;
    h.code = dev.resp.code;
    h.body = dev.resp.body;
    if (dev.resp.type == "application/json") h.json = Json::parse(h.body, h.jsonOk);
    return h;
}

Json Module::status()
{
    Http h = http(HTTP_GET, "/api/status");
    if (h.code == 200 && !h.jsonOk) { badStatus++; lastBadStatus = h.body; }
    return h.json;
}

// ---------------------------------------------------------------- AgOpenWeb
void Aow::send(const std::vector<uint8_t>& pkt, IPAddress dst, uint16_t port)
{
    fake::Packet p;
    p.ms = fake::clockMs;
    p.src = ip;
    p.srcPort = 9999;
    p.dst = dst == IPAddress() ? IPAddress(ip[0], ip[1], ip[2], 255) : dst;
    p.dstPort = port;
    p.data = pkt;
    out.push_back(p);
}

void Aow::sendMachineConfig(uint8_t raise, uint8_t lower, uint8_t hydEnable, bool invert)
{
    send(pgn(238, { raise, lower, hydEnable, (uint8_t)(invert ? 1 : 0), 0, 0, 0, 0 }));
}

void Aow::sendPinConfig(const std::vector<uint8_t>& pins)
{
    std::vector<uint8_t> d(24, 0);
    for (size_t i = 0; i < pins.size() && i < 24; i++) d[i] = pins[i];
    send(pgn(236, d));
}

void Aow::sendWidths()
{
    std::vector<uint8_t> d;
    for (int i = 0; i < 16; i++) { d.push_back(widthCm[i] & 0xFF); d.push_back(widthCm[i] >> 8); }
    d.push_back((uint8_t)toolSections);
    send(pgn(235, d));
}

void Aow::sendSubnet(uint8_t a, uint8_t b, uint8_t c) { send(pgn(201, { 201, 201, a, b, c }), IPAddress(255, 255, 255, 255)); }
void Aow::sendScan() { send(pgn(202, { 202, 202, 5 }), IPAddress(255, 255, 255, 255)); }

void Aow::startup(bool invert, const std::vector<uint8_t>& pins)
{
    sendMachineConfig(2, 4, 0, invert);
    std::vector<uint8_t> p = pins;
    if (p.empty()) for (int i = 0; i < 24; i++) p.push_back(i < toolSections && i < 16 ? (uint8_t)(i + 1) : 0);
    sendPinConfig(p);
    sendWidths();
}

std::vector<Heard> Aow::heardPgn(uint8_t number, uint64_t sinceMs) const
{
    std::vector<Heard> r;
    for (auto& h : heard) if (h.pgn() == number && h.ms >= sinceMs) r.push_back(h);
    return r;
}

// ---------------------------------------------------------------- the sprayer
std::vector<bool> SprayerRig::valveOpen() const
{
    std::vector<bool> open(valves, false);
    const uint32_t r = m->relays();
    std::vector<bool> energised(valves, false);
    for (size_t i = 0; i < relayValve.size(); i++)
        if (relayValve[i] >= 0 && relayValve[i] < valves && ((r >> i) & 1)) energised[relayValve[i]] = true;
    for (int v = 0; v < valves; v++)
    {
        open[v] = ncWiring ? !energised[v] : energised[v];
        if (v < (int)stuckOpen.size() && stuckOpen[v]) open[v] = true;
    }
    return open;
}

void SprayerRig::tick()
{
    fake::Device& d = m->dev;
    const std::vector<bool> open = valveOpen();
    bar = pumpOn ? pumpBar : 0;
    float flow = 0;
    for (int v = 0; v < valves; v++)
        if (open[v]) flow += (v < (int)nozzles.size() ? nozzles[v] : 6) * nozzleLpm * sqrtf(bar / chartBar);
    lpm = flow;

    const float sensorV = std::min(4.5f, std::max(0.5f, 0.5f + 4.0f * bar / sensorMaxBar));
    const uint32_t pressureMv = (uint32_t)std::min(3100.0f, sensorV * divider * 1000.0f);
    noise = noise * 1103515245u + 12345u;
    const uint32_t floating = (noise >> 16) % 150;      // an input with nothing on it
    const double pulsesPerMs = flowFitted ? lpm * meterPpl / 60000.0 : 0;
    const uint32_t meterMv = pulsesPerMs > 0 ? 1700 : 0;  // the pulse train read as a level: about half of 3.4 V

    if (!swapped)
    {
        d.analogMv[34] = pressureFitted ? pressureMv : floating;
        if (flowFitted && d.isr[35])
        {
            pulseAcc += pulsesPerMs;
            fake::Device* before = fake::cur;
            fake::cur = &d;
            while (pulseAcc >= 1) { d.isr[35](); pulseAcc -= 1; }
            fake::cur = before;
        }
    }
    else
    {
        // the flow meter on the pressure input and the pressure sensor on the flow input: a steady voltage, no edges
        d.analogMv[34] = flowFitted ? meterMv : floating;
    }
}

// ---------------------------------------------------------------- the world
World::World()
{
    fake::clockMs = 1000;
    fake::net = fake::Network();
}

World::~World()
{
    rig.reset();
    mods.clear();
}

Module& World::add(Board b, const std::string& label)
{
    const uint64_t mac = 0x0000A1B2C3D40000ull + ((uint64_t)(mods.size() + 1) << 40) + (mods.size() + 1) * 0x0101;
    mods.emplace_back(new Module(b, label, mac));
    return *mods.back();
}

SprayerRig& World::addRig(Module& m)
{
    rig.reset(new SprayerRig());
    rig->m = &m;
    rig->valves = 8;
    for (int i = 0; i < m.bi.relays; i++) rig->relayValve.push_back(i < 8 ? i : -1);
    rig->nozzles.assign(8, 6);
    rig->stuckOpen.assign(8, false);
    return *rig;
}

static uint32_t key(IPAddress a) { return (uint32_t)a[0] << 24 | (uint32_t)a[1] << 16 | (uint32_t)a[2] << 8 | a[3]; }

static bool reachesModule(const fake::Packet& p, const Module& m)
{
    if (!m.on() || !m.dev.onTractorNet() || m.dev.udpPort != p.dstPort) return false;
    const IPAddress me = m.dev.tractorIp();
    if (p.dst == IPAddress(255, 255, 255, 255) || p.dst == me) return true;
    return p.dst == IPAddress(me[0], me[1], me[2], 255);        // its own subnet's broadcast only
}

void World::run(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t++)
    {
        // AgOpenWeb: 239 (and 229) every 100 ms, a hello every second
        if (aow.on && fake::clockMs - aow.last239 >= 100)
        {
            aow.last239 = fake::clockMs;
            const uint8_t spd = (uint8_t)std::min(255.0f, aow.kmh * 10.0f + 0.5f);
            aow.send(pgn(239, { aow.uTurn, spd, aow.hydLift, aow.tram, aow.geo, 0,
                                (uint8_t)(aow.sections & 0xFF), (uint8_t)((aow.sections >> 8) & 0xFF) }));
            if (aow.toolSections > 16 && aow.send229)
            {
                std::vector<uint8_t> d;
                for (int b = 0; b < 8; b++) d.push_back((uint8_t)(aow.sections >> (8 * b)));
                aow.send(pgn(229, d));
            }
        }
        if (aow.on && fake::clockMs - aow.lastHello >= 1000)
        {
            aow.lastHello = fake::clockMs;
            aow.send(pgn(200, { 56, 0, 0 }));
        }
        // the tablet's messages reach the modules (a switched-off tablet sends nothing at all)
        while (!aow.out.empty())
        {
            const fake::Packet p = aow.out.front();
            aow.out.pop_front();
            if (!aow.on) continue;
            if (!xiao.on) { for (auto& m : mods) if (reachesModule(p, *m)) m->dev.rx.push_back(p); continue; }
            // On the XIAO's network: its copy at once to every module it knows, from its own address (not the subnet,
            // the scan, the steering's own) ...
            const uint8_t number = p.data.size() > 3 ? p.data[3] : 0;
            const bool forModules = number != 201 && number != 202 && number != 254 && number != 252 && number != 251;
            if (xiao.bridge && p.dstPort == 8888 && forModules)
                for (auto& m : mods)
                {
                    if (!m->on() || !m->dev.staUp() || !xiao.known.count(key(m->dev.tractorIp()))) continue;
                    fake::Packet c = p;
                    c.src = XiaoAp::ip();
                    c.dst = m->dev.tractorIp();
                    m->dev.rx.push_back(c);
                    xiao.copies++;
                }
            // ... and the tablet's own broadcast later, when the access point lets it out (or never)
            if (xiao.loseOneIn && ++xiao.lateCount % xiao.loseOneIn == 0) continue;
            xiao.late.push_back({ fake::clockMs + xiao.lateMs, p });
        }
        while (!xiao.late.empty() && xiao.late.front().first <= fake::clockMs)
        {
            const fake::Packet p = xiao.late.front().second;
            xiao.late.pop_front();
            for (auto& m : mods) if (reachesModule(p, *m)) m->dev.rx.push_back(p);
        }
        for (auto it = xiao.known.begin(); it != xiao.known.end();)
            it = fake::clockMs - it->second >= 3000 ? xiao.known.erase(it) : std::next(it);
        if (rig) rig->tick();
        for (auto& m : mods) m->step();
        // what the modules sent reaches the tablet (port 9999, broadcast of its subnet, everyone's, or its own IP)
        for (auto& m : mods)
        {
            for (auto& p : m->dev.tx)
            {
                if (p.dstPort != 9999) continue;
                const bool toAll = p.dst == IPAddress(255, 255, 255, 255) || p.dst == IPAddress(aow.ip[0], aow.ip[1], aow.ip[2], 255);
                if (xiao.on)
                {
                    // On the XIAO's network: it hears what is sent to it and to everyone, notes the module, and passes
                    // the message on to the tablet. What went to everyone would reach the tablet again later, straight
                    // from the module: counted, not listened to.
                    if (!m->dev.staUp()) continue;
                    if (xiao.bridge && (toAll || p.dst == XiaoAp::ip()))
                    {
                        xiao.known[key(p.src)] = fake::clockMs;
                        if (aow.on) { aow.heard.push_back({ p.ms, XiaoAp::ip(), p.data }); xiao.passedOn++; }
                    }
                    if (toAll) xiao.toEveryone++;
                    else if (p.dst == aow.ip && aow.on) aow.heard.push_back({ p.ms, p.src, p.data });
                    continue;
                }
                if (aow.on && (toAll || p.dst == aow.ip)) aow.heard.push_back({ p.ms, p.src, p.data });
            }
            m->dev.tx.clear();
        }
        for (auto& w : watchers) w();
        fake::clockMs++;
    }
}

void World::useXiao()
{
    xiao.on = true;
    fake::net.ssid = "XIAO-3F2A";
    fake::net.pass = "trator-2026";
    fake::net.subnet[0] = 192; fake::net.subnet[1] = 168; fake::net.subnet[2] = 4;
    fake::net.dhcp = true;
    aow.ip = IPAddress(192, 168, 4, 2);
}

void joinTractorWifi(World& w, Module& m, const std::string& pass, const std::string& subnet, int ipLast)
{
    const std::string apName = m.dev.apSsid.empty() ? std::string(m.bi.sprayer ? "Sprayer" : "Seeder") : m.dev.apSsid;
    m.http(HTTP_POST, "/api/wifi", { { "pw", "agopengps" }, { "apSsid", apName }, { "apPass", "" }, { "staOn", "1" },
                                     { "staSsid", fake::net.ssid }, { "staPass", pass }, { "host", m.bi.sprayer ? "sprayer" : "seeder" },
                                     { "subnet", subnet }, { "ipLast", std::to_string(ipLast) } });
    w.run(3000);                            // the restart, then joining the network
}
}
