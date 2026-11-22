/* bench.cpp - the test bench (bench.h): the XIAO, the browsers, the programs on the tablets, the rules.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "bench.h"
#include <dlfcn.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace bench
{
static const KitInfo KITS[] = {
    { Kit::C3, "c3", "XIAO ESP32C3", "xiao_c3", 5, "Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin/v4.5_XIAO_ESP32C3.bin" },
    { Kit::S3, "s3", "XIAO ESP32S3", "xiao_s3", 9, "Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin/v4.5_XIAO_ESP32S3.bin" },
};
const KitInfo& info(Kit k) { return KITS[k == Kit::S3 ? 1 : 0]; }
std::vector<Kit> bothKits() { return { Kit::C3, Kit::S3 }; }
std::vector<Kit> oneKit() { return { Kit::C3 }; }

static World* theWorld = nullptr;
World* world() { return theWorld; }

std::vector<uint8_t> packet(uint8_t pgn, const std::vector<uint8_t>& data, uint8_t src)
{
    std::vector<uint8_t> p = { 0x80, 0x81, src, pgn, (uint8_t)data.size() };
    p.insert(p.end(), data.begin(), data.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < p.size(); i++) sum += p[i];
    p.push_back(sum);
    return p;
}

std::vector<uint8_t> repoFile(const std::string& path)
{
    const char* r = getenv("BOARDS_REPO");
    const std::string full = std::string(r && r[0] ? r : "../..") + "/" + path;
    std::ifstream f(full, std::ios::binary);
    if (!f) throw std::runtime_error("file missing: " + full);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string s = ss.str();
    return std::vector<uint8_t>(s.begin(), s.end());
}

// A program file as the Arduino IDE exports it, in small: the 24-byte header, two segments (the first starts with
// the application's description, as every ESP32 application), then the checksum in the last byte of a 16-byte row.
// No byte in the segments is 0xE9 (the mark a program file starts with), so a scenario decides where one is.
void sealImage(std::vector<uint8_t>& im)
{
    size_t at = 24;
    uint8_t sum = 0xEF;
    for (int s = 0; s < im[1]; s++)
    {
        uint32_t len;
        memcpy(&len, im.data() + at + 4, 4);
        at += 8;
        for (uint32_t i = 0; i < len; i++) sum ^= im[at + i];
        at += len;
    }
    at = (at + 16) & ~(size_t)15;
    im[at - 1] = sum;
}
std::vector<uint8_t> makeImage(uint16_t chip, size_t size, uint32_t seed)
{
    size = (size + 15) & ~(size_t)15;
    std::vector<uint8_t> im(size, 0);
    const uint8_t head[24] = { 0xE9, 2, 0x02, 0x2F, 0x20, 0x00, 0x38, 0x42, 0xEE, 0, 0, 0, (uint8_t)chip, (uint8_t)(chip >> 8), 0, 0, 0, 0xFF, 0xFF, 0, 0, 0, 0, 0 };
    memcpy(im.data(), head, 24);
    const uint32_t total = (uint32_t)size - 24 - 16 - 16;      // two segment headers, the checksum row
    const uint32_t len1 = (total / 2) & ~3u, len2 = (total - len1) & ~3u;
    uint32_t x = seed * 2654435761u + 1;
    auto fill = [&](size_t at, uint32_t len) {
        for (uint32_t i = 0; i < len; i++)
        {
            x = x * 1664525u + 1013904223u;
            const uint8_t b = (uint8_t)(x >> 24);
            im[at + i] = b == 0xE9 ? 0xE8 : b;
        }
    };
    const uint32_t a1 = 0x3C0A0020u, a2 = 0x42000020u;
    memcpy(im.data() + 24, &a1, 4);
    memcpy(im.data() + 28, &len1, 4);
    fill(32, len1);
    const uint32_t desc = 0xABCD5432u;
    memcpy(im.data() + 32, &desc, 4);
    memcpy(im.data() + 32 + len1, &a2, 4);
    memcpy(im.data() + 36 + len1, &len2, 4);
    fill(40 + len1, len2);
    sealImage(im);
    return im;
}

std::string fmt(const char* f, ...)
{
    char b[2048];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof(b), f, ap);
    va_end(ap);
    return b;
}
bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

// ---------------------------------------------------------------- the XIAO
static std::string libDir()
{
    const char* d = getenv("XIAO_LIB_DIR");
    return d && d[0] ? d : "build";
}

Xiao::~Xiao() { unload(); }

void Xiao::load()
{
    fake::cur = &dev;
    dev.s3 = kit == Kit::S3;
    dev.bootAt = fake::clockMs;
    if (dev.updDone)                        // an update was accepted: this start runs it
    {
        dev.updDone = false;
        dev.updInstalled++;
        if (!dev.updDoneIsApp) dev.updInstalledNotApp++;
    }
    const std::string path = libDir() + "/" + info(kit).lib + ".so";
    so_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!so_) throw std::runtime_error(std::string("dlopen: ") + dlerror() + " (run ./run.sh)");
    setup_ = (void (*)())dlsym(so_, "_Z5setupv");
    loop_ = (void (*)())dlsym(so_, "_Z4loopv");
    if (!setup_ || !loop_) throw std::runtime_error("setup/loop not found in the program library");
    boots++;
    on = true;
    dev.running = true;
    try { setup_(); }
    catch (fake::Restart& r) { restarted(r.reason); return; }
    dev.running = false;
}

// The program goes: its variables with it (the library is loaded again at the next start)
void Xiao::unload()
{
    if (!so_) return;
    dlclose(so_);
    so_ = nullptr;
    setup_ = loop_ = nullptr;
}

void Xiao::restarted(esp_reset_reason_t why)
{
    unload();
    dev.stopped(why);
    dev.restarts++;
    restarts.push_back(why);
    on = false;
    backAt = fake::clockMs + 350;           // the chip's own start-up
}

void Xiao::powerOn()
{
    if (on) return;
    dev.resetReason = ESP_RST_POWERON;
    backAt = 0;
    load();
}

void Xiao::powerOff()
{
    unload();
    dev.stopped(ESP_RST_POWERON);
    on = false;
    backAt = 0;
}

void Xiao::pass()
{
    if (!on)
    {
        if (backAt && fake::clockMs >= backAt) { backAt = 0; load(); }
        else fake::tick(1);
        return;
    }
    const uint64_t t0 = fake::clockMs;
    dev.wdtFedAt = t0;                      // the core feeds the loop's watchdog before each pass
    dev.running = true;
    try
    {
        loop_();
        dev.running = false;
        if (fake::clockMs == t0) fake::tick(1);     // (the program's other tasks may run in here too)
    }
    catch (fake::Restart& r) { restarted(r.reason); return; }
    if (fake::clockMs - t0 > longestPassMs) longestPassMs = (uint32_t)(fake::clockMs - t0);
}

// ---------------------------------------------------------------- a program on a tablet
IPAddress Program::ip() const
{
    const fake::Device& d = world()->dev;
    return otherNet && !forged ? IPAddress(d.router.subnet[0], d.router.subnet[1], d.router.subnet[2], ipLast) : IPAddress(192, 168, 4, ipLast);
}

void Program::send(uint16_t port, const std::vector<uint8_t>& data, bool toAll)
{
    World& w = *world();
    fake::Datagram d;
    d.ms = fake::clockMs;
    d.src = ip().raw();
    d.srcPort = 9999;
    const IPAddress me = ip();
    if (otherNet && forged) d.dst = w.dev.staIp().raw();        // (sent straight to the XIAO's address on that network)
    else d.dst = toAll ? IPAddress(me[0], me[1], me[2], 255).raw() : (otherNet ? w.dev.staIp() : IPAddress(192, 168, 4, 1)).raw();
    d.dstPort = port;
    d.data = data;
    const bool link = w.xiao.on && (otherNet ? w.dev.staJoined : w.apUsable());
    const bool taken = link && w.dev.udpIn(d);
    if (port == 8888)
    {
        if (taken && !otherNet)
        {
            w.progLog.push_back({ fake::clockMs, data });
            while (w.progLog.size() > 20000) { w.progLog.pop_front(); w.progLogBase++; }
        }
        sent.push_back({ fake::clockMs, data, taken });
        nSent++;
        if (!taken) nNotTaken++;
        while (matchFrom > 64) { sent.pop_front(); matchFrom--; }
        while (sent.size() > 60000) { sent.pop_front(); if (matchFrom) matchFrom--; }
    }
    else if (taken) rtcmSent.insert(rtcmSent.end(), data.begin(), data.end());
}

// What AgOpenWeb sends a module: its hello every second, PGN 254 a hundred times a second, the machine data ten
// times a second, and its corrections. The steer angle in PGN 254 counts up, so every one is different and the
// bench can tell which arrived.
void Program::step()
{
    if (!on) return;
    const uint64_t now = fake::clockMs;
    if (now % 1000 == (uint64_t)ipLast * 7 % 1000) send(8888, packet(200, { 56, 0, 0 }), true);
    if (now % every254Ms == ipLast % every254Ms)
    {
        seq++;
        send(8888, packet(254, { 72, 0, (uint8_t)(engaged ? 1 : 0), (uint8_t)seq, (uint8_t)(seq >> 8), 0, 0xFF, 0x00 }), true);
    }
    for (uint32_t k = 0; k < extraPerMs; k++)
    {
        seq++;
        send(8888, packet(254, { 72, 0, (uint8_t)(engaged ? 1 : 0), (uint8_t)seq, (uint8_t)(seq >> 8), 0, 0xFF, 0x00 }), true);
    }
    if (now % 100 == (uint64_t)ipLast % 100) send(8888, packet(239, { 0, 72, 1, 0, 0, 0, 0xFF, 0x00 }), true);
    if (rtcmEveryMs && now % rtcmEveryMs == 0)
    {
        std::vector<uint8_t> r(rtcmBytes);
        for (size_t i = 0; i < r.size(); i++) r[i] = (uint8_t)(i * 31 + seq + (i >> 3));
        r[0] = 0xD3;
        send(2233, r, false);
    }
}

void Program::receive(const fake::Datagram& d)
{
    if (d.src != IPAddress(192, 168, 4, 1).raw()) { gotMachineDirect++; return; }   // straight from a module that sent to everyone
    got++;
    lastGotMs = fake::clockMs;
    if (!d.data.empty() && d.data[0] == '$') gotNmea++;
    else if (d.data.size() >= 6 && d.data[0] == 0x80 && d.data[1] == 0x81)
    {
        if (d.data[2] == 0x7B && d.srcPort == 9999) { gotMachine++; lastMachineMs = fake::clockMs; }
        else if (d.data[3] == 253) got253++;
        else if (d.data[3] == 126 || d.data[3] == 121) gotHello++;
        else gotOther++;
    }
    else gotOther++;
}

// ---------------------------------------------------------------- a section-control module on the XIAO's network
bool Machine::throughXiao() const { return lastGotMs != 0 && fake::clockMs - lastGotMs < 3000; }

void Machine::send(const std::vector<uint8_t>& data)
{
    World& w = *world();
    const bool toAll = !throughXiao();
    fake::Datagram d;
    d.ms = fake::clockMs;
    d.src = ip().raw();
    d.srcPort = 8888;
    d.dst = (toAll ? IPAddress(192, 168, 4, 255) : IPAddress(192, 168, 4, 1)).raw();
    d.dstPort = 9999;
    d.data = data;
    sent++;
    if (!w.xiao.on || !w.apUsable()) return;
    if (w.dev.udpIn(d))
    {
        heard++;
        sentLog.push_back(data);
        while (sentMatched > 64) { sentLog.pop_front(); sentMatched--; }
        while (sentLog.size() > 4000) { sentLog.pop_front(); if (sentMatched) sentMatched--; }
    }
    if (toAll) for (Program* p : { &w.tab, &w.tab2 }) if (p->on) p->receive(d);    // the access point passes it on to the other devices
}

// PGN 237 every 200 ms (a counter in it, so every one is different), program or not
void Machine::step()
{
    if (!on) return;
    for (uint32_t k = 0; k < extraPerMs; k++)
    {
        seq++;
        send(packet(237, { (uint8_t)seq, (uint8_t)(seq >> 8), 0, 0, 0, 1, ipLast, 0 }, 0x7B));
    }
    if (fake::clockMs % 200 != (uint64_t)ipLast % 200) return;
    seq++;
    send(packet(237, { (uint8_t)seq, (uint8_t)(seq >> 8), 0, 0, 0, 1, ipLast, 0 }, 0x7B));
}

void Machine::receive(const fake::Datagram& d, uint32_t delayMs)
{
    got++;
    lastGotMs = fake::clockMs;
    if (delayMs > worstDelayMs) worstDelayMs = delayMs;
    if (d.data.size() < 6) return;
    if (d.data[3] == 239) got239++;
    else if (d.data[3] == 201 || d.data[3] == 202) gotSubnetOrScan++;
    else if (d.data[3] == 254 || d.data[3] == 252 || d.data[3] == 251) gotSteer++;
    else if (d.data[3] == 200)
    {
        gotHello++;
        send(packet(123, { (uint8_t)seq, (uint8_t)(seq >> 8), ipLast, 0, 0 }, 0x7B));
    }
}

// ---------------------------------------------------------------- a browser's requests
std::string enc(const std::string& s)
{
    std::string o;
    for (unsigned char c : s)
    {
        if (isalnum(c) || strchr("-_.!~*'()", c)) o += (char)c;
        else o += fmt("%%%02X", c);
    }
    return o;
}
static const char* AGENT = "User-Agent: Mozilla/5.0 (Linux; Android 14; SM-X210) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36\r\n"
                           "Accept-Encoding: gzip, deflate\r\nAccept-Language: pt-PT,pt;q=0.9,en;q=0.8\r\n";
std::string GET(const std::string& path, const std::string& host, const std::string& extra)
{
    return "GET " + path + " HTTP/1.1\r\nHost: " + host + "\r\nConnection: keep-alive\r\n" + AGENT + "Accept: */*\r\nReferer: http://" + host + "/\r\n" + extra + "\r\n";
}
std::string POST(const std::string& path, const std::vector<std::pair<std::string, std::string>>& form, const std::string& host)
{
    std::string body;
    for (auto& f : form) body += (body.empty() ? "" : "&") + enc(f.first) + "=" + enc(f.second);
    return "POST " + path + " HTTP/1.1\r\nHost: " + host + "\r\nConnection: keep-alive\r\nContent-Length: " + std::to_string(body.size()) + "\r\n" + AGENT +
           "Content-Type: application/x-www-form-urlencoded\r\nAccept: */*\r\nOrigin: http://" + host + "\r\nReferer: http://" + host + "/\r\n\r\n" + body;
}
std::string UPLOAD(const std::string& pathAndQuery, const std::string& fileName, const std::vector<uint8_t>& file, const std::string& host)
{
    const std::string boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    std::string body = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"" + fileName + "\"\r\nContent-Type: application/octet-stream\r\n\r\n";
    body.append((const char*)file.data(), file.size());
    body += "\r\n--" + boundary + "--\r\n";
    return "POST " + pathAndQuery + " HTTP/1.1\r\nHost: " + host + "\r\nConnection: keep-alive\r\nContent-Length: " + std::to_string(body.size()) + "\r\n" + AGENT +
           "Content-Type: multipart/form-data; boundary=" + boundary + "\r\nAccept: */*\r\nOrigin: http://" + host + "\r\nReferer: http://" + host + "/\r\n\r\n" + body;
}

// ---------------------------------------------------------------- the world
World::World(Kit k) : dev(xiao.dev)
{
    xiao.kit = k;
    dev.s3 = k == Kit::S3;
    dev.usbOpen = true;                     // a serial monitor on the USB-C port: every event of the XIAO is printed there
    knownSsid = fmt("XIAO-%02X%02X", (unsigned)((dev.mac >> 32) & 0xFF), (unsigned)((dev.mac >> 40) & 0xFF));
    tab.ipLast = 2;
    tab2.ipLast = 3;
    for (size_t i = 0; i < mod.size(); i++) mod[i].ipLast = (uint8_t)(123 + i);
    far.ipLast = 77;
    far.otherNet = true;
    fake::cur = &dev;
    theWorld = this;
    fake::onMs = [this]() { step(); };
    teensy.bootAt = fake::clockMs;
    helloWatchFrom_ = fake::clockMs;
}

World::~World()
{
    fake::onMs = nullptr;
    theWorld = nullptr;
}

void World::start()
{
    xiao.powerOn();
    run(3000);
}

void World::run(uint32_t ms)
{
    const uint64_t until = fake::clockMs + ms;
    while (fake::clockMs < until) xiao.pass();
}

bool World::runUntil(const std::function<bool()>& cond, uint32_t maxMs)
{
    const uint64_t until = fake::clockMs + maxMs;
    while (fake::clockMs < until)
    {
        if (cond()) return true;
        xiao.pass();
    }
    return cond();
}

void World::flash(const char* key, const std::string& v) { dev.prefs["xiao"][key] = { 's', v }; }
void World::flash(const char* key, bool v) { dev.prefs["xiao"][key] = { 'b', std::string(1, v ? 1 : 0) }; }
bool World::hasFlash(const char* key) const { auto n = dev.prefs.find("xiao"); return n != dev.prefs.end() && n->second.count(key); }
std::string World::flashStr(const char* key) const { return hasFlash(key) ? dev.prefs.at("xiao").at(key).bytes : std::string(); }
bool World::flashBool(const char* key) const { return hasFlash(key) && dev.prefs.at("xiao").at(key).bytes == std::string(1, 1); }
void World::bridgeSaved(const std::string& pass)
{
    flash("apPass", pass);
    flash("bridge", true);
    knownPass = pass;
}
bool World::apUsable() const { return apUpSince_ && fake::clockMs - apUpSince_ >= 2000; }
void World::usb(const std::string& line) { dev.usbIn += line + "\n"; }
bool World::said(const std::string& part) const { return has(dev.usbOut, part); }
std::string World::hostFor(const Link& l) const { return l.fromAp ? "192.168.4.1" : std::string(dev.staIp().toString().c_str()); }

uint8_t World::noisy(uint8_t b)
{
    if (noise > 0 && rnd() % 1000000 < (uint32_t)(noise * 1000000)) b ^= (uint8_t)(1u << (rnd() % 8));
    return b;
}

// The serial link: 46 bytes a millisecond each way, the Teensy's part in between
void World::wires()
{
    uint8_t buf[64];
    accOut_ += UART_PER_MS;
    size_t n = (size_t)accOut_;
    accOut_ -= n;
    if (xiao.on)
    {
        const size_t k = dev.uartWire(buf, n);
        if (!wireCut && !cutToTeensy) for (size_t i = 0; i < k; i++) teensy.wireIn(noisy(buf[i]));
    }
    teensy.step();
    accIn_ += UART_PER_MS;
    n = std::min((size_t)accIn_, teensy.tx.size());
    accIn_ -= (size_t)accIn_;
    for (size_t i = 0; i < n; i++) { buf[i] = noisy(teensy.tx.front()); teensy.tx.pop_front(); }
    if (!wireCut && !cutToXiao && xiao.on && n) dev.uartFeed(buf, n);
    if (!wireJunk.empty() && xiao.on)
    {
        for (size_t i = 0; i < 46; i++) buf[i] = (uint8_t)wireJunk[junkAt_++ % wireJunk.size()];
        dev.uartFeed(buf, 46);
    }
}

// R1: what reached the Teensy through the bridge (PGN 146: the sender's IP, then its datagram)
void World::bridgeFrame(const uint8_t* d, uint8_t n)
{
    auto broke = [this](const std::string& what) { if (rules.passed.size() < 20) rules.passed.push_back(fmt("at %llu ms: %s", (unsigned long long)fake::clockMs, what.c_str())); };
    const IPAddress from(d[0], d[1], d[2], d[3]);
    const std::string who = from.toString().c_str();
    if (!bridgeAtBoot_) { broke("a packet from " + who + " was passed to the Teensy with the WiFi bridge switched off"); return; }
    if (!teensy.knowsBridge || fake::clockMs - teensy.toldAt > 3200) broke("a packet from " + who + " was passed to a Teensy that does not say it knows the bridge");
    if (n < 4 + 6 || d[4] != 0x80 || d[5] != 0x81) { broke("something that is no AgOpenGPS packet was passed to the Teensy (from " + who + ")"); return; }
    if (teensy.told == 1 && fake::clockMs - teensy.toldSince > 200 && d[4 + 3] != 200)
        broke(fmt("PGN %u from %s was passed while a program on the cable has the module (only the hello may pass)", d[4 + 3], who.c_str()));
    Program* p = nullptr;
    for (Program* q : { &tab, &tab2, &far }) if (q->ip() == from && (q->on || !q->sent.empty())) { p = q; break; }
    if (!p) { broke("a packet was passed to the Teensy from " + who + ", and no program has that address"); return; }
    if (p->otherNet) { broke("a packet from the program on the OTHER network (" + who + ") was passed to the Teensy"); return; }
    const std::vector<uint8_t> data(d + 4, d + n);
    size_t k = p->matchFrom;
    while (k < p->sent.size() && p->sent[k].data != data) k++;
    if (k == p->sent.size()) { broke("a packet was passed to the Teensy that the program at " + who + " never sent (or it came again, or out of order)"); return; }
    p->nNotPassed += (uint32_t)(k - p->matchFrom);
    p->matchFrom = k + 1;
    p->nPassed++;
    const uint32_t delay = (uint32_t)(fake::clockMs - p->sent[k].ms);
    if (delay > p->worstDelayMs) p->worstDelayMs = delay;
    if (data[3] == 254 && data.size() >= 8 && (data[7] & 1)) steerSeen();
}

// R2: what the XIAO sent on the WiFi
void World::datagramsOut()
{
    auto broke = [this](const std::string& what) { if (rules.dest.size() < 20) rules.dest.push_back(fmt("at %llu ms: %s", (unsigned long long)fake::clockMs, what.c_str())); };
    for (const fake::Datagram& d : dev.udpOut)
    {
        const IPAddress to = IPAddress::fromRaw(d.dst);
        const std::string where = fmt("%s:%u", to.toString().c_str(), d.dstPort);
        Program* p = nullptr;
        for (Program* q : { &tab, &tab2 }) if (q->ip() == to) p = q;
        Machine* m = nullptr;
        for (Machine& q : mod) if (q.ip() == to && (q.on || q.sent)) m = &q;
        if (to[3] == 255 || to[0] != 192 || to[1] != 168 || to[2] != 4) broke("a datagram went to " + where + " (to everyone, or off the XIAO's own network)");
        else if (m && d.dstPort == 8888)
        {
            // for a section-control module: one of the program's own, each once and in order, never the subnet or the scan
            if (m->logAt < progLogBase) m->logAt = progLogBase;
            size_t k = (size_t)(m->logAt - progLogBase);
            while (k < progLog.size() && progLog[k].data != d.data) k++;
            if (k == progLog.size()) broke("a datagram went to the module at " + where + " that the program never sent (or it came again, or out of order)");
            else
            {
                m->logAt = progLogBase + k + 1;
                if (m->on && apUsable()) m->receive(d, (uint32_t)(fake::clockMs - progLog[k].ms));
            }
            continue;
        }
        else if (!p || d.dstPort != 9999) broke("a datagram went to " + where + ", where no program is");
        if (d.srcPort == 9999)
        {
            // from the socket of the other modules: something one of them sent, each once and in order
            bool found = false;
            for (Machine& q : mod)
            {
                size_t k = q.sentMatched;
                while (k < q.sentLog.size() && q.sentLog[k] != d.data) k++;
                if (k == q.sentLog.size()) continue;
                q.sentMatched = k + 1;
                found = true;
                break;
            }
            if (!found) broke("a datagram went to " + where + " that no module sent (or it came again, or out of order)");
            if (p && p->on && apUsable()) p->receive(d);
            continue;
        }
        size_t k = teensy.outMatched;
        while (k < teensy.outLog.size() && teensy.outLog[k] != d.data) k++;
        if (k == teensy.outLog.size()) broke("a datagram went to " + where + " that the Teensy never sent (or it came again, or out of order)");
        else teensy.outMatched = k + 1;
        while (teensy.outMatched > 64) { teensy.outLog.pop_front(); teensy.outMatched--; }
        if (p && p->on && apUsable()) p->receive(d);
    }
    dev.udpOut.clear();
}

int World::ask(const std::string& req, const Link& l, const std::string& what)
{
    auto e = std::make_unique<Exchange>();
    e->req = req;
    e->link = l;
    e->what = what.empty() ? req.substr(0, req.find(" HTTP/")) : what;
    e->startMs = fake::clockMs;
    e->reply.askedMs = fake::clockMs;
    exchanges_.push_back(std::move(e));
    return (int)exchanges_.size() - 1;
}

Reply World::wait(int id)
{
    Exchange& e = *exchanges_[id];
    runUntil([&]() { return e.reply.done || e.reply.failed; }, e.link.patienceMs + 5000);
    return e.reply;
}

Reply World::http(const std::string& req, const Link& l) { return wait(ask(req, l)); }

JsonDoc World::state() { return get("/api/state").json; }

// A whole answer is with the browser (or the XIAO closed the connection): take it apart, and look at its form
void World::finishReply(Exchange& e, bool closed)
{
    Reply& r = e.reply;
    const std::string all = e.conn->tx.substr(0, e.conn->delivered());
    auto bad = [&](const std::string& what) { if (rules.answers.size() < 20) rules.answers.push_back(e.what + ": " + what); };
    const size_t headEnd = all.find("\r\n\r\n");
    r.endMs = fake::clockMs;
    if (headEnd == std::string::npos)
    {
        r.failed = true;
        r.why = all.empty() ? "the XIAO closed the connection without an answer" : "the answer was cut short in its header";
        return;
    }
    std::istringstream head(all.substr(0, headEnd));
    std::string line;
    std::getline(head, line);
    if (line.size() < 12 || line.compare(0, 7, "HTTP/1.") != 0) bad("the first line of the answer is not HTTP: " + line.substr(0, 40));
    else r.code = atoi(line.c_str() + 9);
    while (std::getline(head, line))
    {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const size_t c = line.find(':');
        if (c == std::string::npos) { bad("a header line without a colon: " + line.substr(0, 40)); continue; }
        std::string name = line.substr(0, c), value = line.substr(c + 1);
        for (auto& ch : name) ch = (char)tolower((unsigned char)ch);
        while (!value.empty() && value[0] == ' ') value.erase(0, 1);
        r.headers[name] = value;
    }
    r.body = all.substr(headEnd + 4);
    if (!r.headers.count("content-length")) bad("no Content-Length");
    else
    {
        const size_t want = strtoul(r.headers["content-length"].c_str(), nullptr, 10);
        if (r.body.size() < want)
        {
            r.failed = true;
            r.why = fmt("the answer was cut short: %u of %u bytes", (unsigned)r.body.size(), (unsigned)want);
            if (!e.gone && closed && !e.conn->resetByServer) bad(r.why);
            return;
        }
        if (r.body.size() > want) bad(fmt("%u bytes more than its Content-Length says", (unsigned)(r.body.size() - want)));
    }
    if (r.headers.count("content-type") && r.headers["content-type"].compare(0, 16, "application/json") == 0)
    {
        r.json = JsonDoc::parse(r.body, r.jsonOk);
        if (!r.jsonOk) bad("the answer is not valid JSON: " + r.body.substr(0, 160));
    }
    r.done = true;
    if (r.tookMs() > rules.worstAnswerMs && e.link.upPerMs >= 1000 && e.link.stallAt == SIZE_MAX) rules.worstAnswerMs = r.tookMs();
}

void World::browsers()
{
    const uint64_t now = fake::clockMs;
    for (auto& up : exchanges_)
    {
        Exchange& e = *up;
        Reply& r = e.reply;
        if (r.done || r.failed) continue;
        if (now - e.startMs > e.link.patienceMs)
        {
            r.failed = true;
            r.why = fmt("no answer in %u s", e.link.patienceMs / 1000);
            r.endMs = now;
            if (e.conn) e.conn->peerClosed = true;      // the browser gives up
            continue;
        }
        if (!e.conn)
        {
            if (now < e.nextTryMs) continue;
            const bool reach = xiao.on && (e.link.fromAp ? apUsable() : dev.staJoined);
            const IPAddress me = e.link.fromAp ? IPAddress(192, 168, 4, e.link.ipLast) : IPAddress(dev.router.subnet[0], dev.router.subnet[1], dev.router.subnet[2], e.link.ipLast);
            if (reach) e.conn = dev.connect(e.link.fromAp, me);
            if (!e.conn) { e.nextTryMs = now + 1000; continue; }     // tries again, as a browser does
        }
        fake::Conn& c = *e.conn;
        c.takePerMs = e.gone ? 0 : e.link.downPerMs;
        if (!e.gone && c.delivered() >= e.link.deafAt) { e.gone = true; c.takePerMs = 0; }
        if (c.serverClosed && c.resetByServer && c.inFlight == 0)
        {
            // what the browser had not taken yet is lost with the reset; a whole answer may still be there
            const std::string all = c.tx.substr(0, c.delivered());
            const size_t headEnd = all.find("\r\n\r\n");
            size_t want = SIZE_MAX;
            if (headEnd != std::string::npos)
            {
                const size_t cl = all.find("Content-Length: ");
                if (cl != std::string::npos && cl < headEnd) want = strtoul(all.c_str() + cl + 16, nullptr, 10);
            }
            if (headEnd != std::string::npos && all.size() - headEnd - 4 >= want) { finishReply(e, true); continue; }
            r.failed = true;
            r.why = "connection reset by the XIAO";
            r.endMs = now;
            continue;
        }
        // its request, at the link's pace (and never more than the XIAO's receive window holds)
        if (!e.gone && !e.link.neverAsks && now - e.startMs >= e.link.idleFirstMs && e.sent < e.req.size() && !c.serverClosed)
        {
            if (!e.stalled && e.sent >= e.link.stallAt) { e.stalled = true; e.stallUntil = now + e.link.stallMs; }
            if (e.sent >= e.link.vanishAt) e.gone = true;
            else if (e.sent >= e.link.closeAt)
            {
                c.peerClosed = true;
                r.failed = true;
                r.why = "the browser closed the connection";
                r.endMs = now;
                continue;
            }
            else if (now >= e.stallUntil)
            {
                size_t n = std::min(e.link.upPerMs, e.req.size() - e.sent);
                if (e.sent >= e.link.dripAt) n = (now - e.startMs) % e.link.dripEveryMs == 0 ? std::min(n, e.link.dripBytes) : 0;
                size_t stopAt = e.sent < e.link.dripAt ? e.link.dripAt : SIZE_MAX;
                if (!e.stalled) stopAt = std::min(stopAt, e.link.stallAt);
                stopAt = std::min(stopAt, std::min(e.link.vanishAt, e.link.closeAt));
                if (stopAt > e.sent) n = std::min(n, stopAt - e.sent);
                else n = stopAt == SIZE_MAX ? n : 0;
                const size_t window = c.rx.size() < TCP_SND_BUF ? TCP_SND_BUF - c.rx.size() : 0;
                n = std::min(n, window);
                c.rx.insert(c.rx.end(), e.req.begin() + e.sent, e.req.begin() + e.sent + n);
                e.sent += n;
            }
        }
        // its answer
        if (c.serverClosed && c.inFlight == 0) { finishReply(e, true); continue; }
        if (c.inFlight == 0 && !c.tx.empty())
        {
            const size_t headEnd = c.tx.find("\r\n\r\n");
            if (headEnd == std::string::npos) continue;
            const size_t cl = c.tx.find("Content-Length: ");
            if (cl == std::string::npos || cl > headEnd) continue;
            if (c.tx.size() - headEnd - 4 >= strtoul(c.tx.c_str() + cl + 16, nullptr, 10)) finishReply(e, false);
        }
    }
}

void World::step()
{
    const uint64_t now = fake::clockMs;
    if (xiao.boots != bootsSeen_)
    {
        bootsSeen_ = xiao.boots;
        const std::string pass = hasFlash("apPass") ? flashStr("apPass") : "agopengps";
        bridgeAtBoot_ = flashBool("bridge") && pass != "agopengps" && pass.size() >= 8 && pass.size() <= 63;
        helloWatchFrom_ = now;
    }
    if (teensy.on != teensyWasOn_ || wireCut != wireWasCut_ || cutToTeensy != toTeensyWasCut_)
    {
        teensyWasOn_ = teensy.on;
        wireWasCut_ = wireCut;
        toTeensyWasCut_ = cutToTeensy;
        helloWatchFrom_ = now;
    }
    if (!xiao.on) helloWatchFrom_ = now;
    for (size_t i = 0; i < later_.size(); i++)
        if (later_[i].second && now >= later_[i].first) { auto fn = later_[i].second; later_[i].second = nullptr; fn(); }
    dev.step();
    const bool up = xiao.on && dev.apOn && dev.apSsid == knownSsid && dev.apPass == knownPass;
    if (!up) apUpSince_ = 0;
    else if (!apUpSince_) apUpSince_ = now;
    int machines = 0;
    for (const Machine& m : mod) machines += m.on;
    dev.apStations = apUsable() ? std::min(dev.apMaxStations, phones + (tab.on ? 1 : 0) + (tab2.on ? 1 : 0) + machines) : 0;
    wires();
    tab.step();
    tab2.step();
    far.step();
    for (Machine& m : mod) m.step();
    datagramsOut();
    for (Program* p : { &tab, &tab2 })
        if (p->machineWatch && fake::clockMs - p->lastMachineMs > p->worstMachineGapMs) p->worstMachineGapMs = (uint32_t)(fake::clockMs - p->lastMachineMs);
    browsers();
    // R3: the hello (every 2 s) while both run and the wires are whole
    if (xiao.on && teensy.on && !wireCut && !cutToTeensy && teensy.gps2On)
    {
        const uint32_t gap = (uint32_t)(now - std::max(helloLastMs_, helloWatchFrom_));
        if (gap > rules.worstHelloGapMs) rules.worstHelloGapMs = gap;
    }
    // R6: steering data at the Teensy while the tablet steers through the bridge
    if (steerWatch_)
    {
        const uint32_t gap = (uint32_t)(now - steerLastMs_);
        if (gap > rules.worstSteerGapMs) rules.worstSteerGapMs = gap;
    }
}
}
