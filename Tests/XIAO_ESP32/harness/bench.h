/* bench.h - the test bench: the real XIAO program on a pretend XIAO, with a pretend Teensy on its serial link,
   programs (AgOpenWeb / AgOpenGPS) on tablets, browsers on its page and another WiFi network around.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The pretend Teensy follows zXiao.ino of the Teensy firmware AiO_WEB (its side of the link): the same frames,
   limits and timings.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One World per scenario. Pretend time moves a millisecond at a time and only inside the XIAO program's own waits
// (fake_core.h); every millisecond World::step() moves the bytes on the serial link (46 a millisecond each way, as
// 460800 baud), lets the pretend Teensy and the programs do their part, feeds the browsers' requests in and takes
// their answers out. While it does so it watches the rules that must hold whatever the user does (World::Rules).
#pragma once
#include "fake_core.h"
#include "json.h"
#include <array>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bench
{
enum class Kit { C3, S3 };
struct KitInfo
{
    Kit id;
    const char* key;                // on the command line and in known-bugs.txt
    const char* name;
    const char* lib;                // the program built for it: build/<lib>.so
    uint16_t chip;                  // the chip number in a program file's header
    const char* bin;                // its compiled program in the repository (v4.5_XIAO_ESP32_bin)
};
const KitInfo& info(Kit k);
std::vector<Kit> bothKits();
std::vector<Kit> oneKit();          // the XIAO ESP32C3 (the one on the tractors)

// 80 81 <source> <pgn> <length> <data> <sum>: an AgOpenGPS packet, and a frame on the link
std::vector<uint8_t> packet(uint8_t pgn, const std::vector<uint8_t>& data, uint8_t src = 0x7F);
std::vector<uint8_t> repoFile(const std::string& path);                 // a file of the repository (BOARDS_REPO)
std::vector<uint8_t> makeImage(uint16_t chip, size_t size, uint32_t seed = 1);   // a small valid program file (.bin)
void sealImage(std::vector<uint8_t>& image);        // its checksum again, after a byte in it was changed
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
bool has(const std::string& text, const std::string& part);

// ---------------------------------------------------------------- the XIAO
struct Xiao
{
    Kit kit = Kit::C3;
    fake::Device dev;
    bool on = false;                // powered and its program running
    uint64_t backAt = 0;            // it restarted: the program starts again at this time
    int boots = 0;
    std::vector<esp_reset_reason_t> restarts;    // every restart the program made itself, with its reason
    uint32_t longestPassMs = 0;     // the longest single loop() pass

    ~Xiao();
    void powerOn();
    void powerOff();
    void pass();                    // one loop() pass (or a millisecond, when it is off)

private:
    void load();
    void unload();
    void restarted(esp_reset_reason_t why);
    void* so_ = nullptr;
    void (*setup_)() = nullptr;
    void (*loop_)() = nullptr;
};

// ---------------------------------------------------------------- the pretend Teensy (zXiao.ino's side of the link)
struct World;
struct Teensy
{
    // ---- what the scenario sets
    bool on = true;
    bool gps2On = true;             // the GPS2 port is set to "XIAO" in the Teensy's settings
    bool knowsBridge = true;        // firmware of 22 Nov 2026 or later: says the bridge's state (PGN 149)
    bool knowsFw = true;            // firmware of 2 Nov 2026 or later: takes an update over the link (PGN 145)
    bool engaged = false;           // autosteer is engaged (it then refuses an update, a subnet)
    bool moving = false;
    bool cable = false;             // a program is talking on its network cable
    bool gnss = true;               // it has a position: four sentences for the program ten times a second
    bool sendsStatus = true;
    size_t statusBytes = 1800;      // the size of its status (the real one is 1.3 to 2.5 kB)
    std::vector<uint8_t> fwExpect;  // an update over the link is good only when it is this file, whole
    int fwLoseAnswer = 0;           // lose the answer to this block number once (0: none)
    int fwStopAt = 0;               // stop answering from this block number on (0: never)

    // ---- what it saw and did
    struct Line { uint32_t seq, ms; std::string text; };
    std::deque<Line> lines;         // its log (the last 60 lines, as the real one)
    uint32_t nextSeq = 1;
    std::vector<std::string> allLines;      // every log line since the bench started
    std::vector<std::string> xiaoSaid;      // the XIAO's text lines it logged ("[xiao] > ...")
    std::string info;               // the XIAO's last info (PGN 143)
    uint32_t infoRx = 0, hellos = 0, crcErr = 0, cut = 0, beforeHello = 0, textDropped = 0;
    uint32_t netIn = 0, netIgnored = 0, netOut = 0, netOutDrop = 0, toCable = 0, old146 = 0;
    uint32_t statusSent = 0, logSent = 0, silentEvents = 0, restarts = 0, wifi201 = 0;
    uint32_t steerOffs = 0;         // its steering watchdog cut the steering: no PGN 254 for 2.5 s with autosteer asked for
    std::vector<uint8_t> rtcm;      // corrections written to its receiver
    std::vector<std::array<uint8_t, 3>> subnets;    // subnets it accepted (it then restarts)
    std::array<uint8_t, 3> net = { 192, 168, 5 };  // its own network's first three numbers (its status says <net>.126)
    std::vector<std::string> subnetRefused;
    uint32_t fwDone = 0, fwRefused = 0, reverts = 0;
    std::string fwWhy;
    std::string lastStatus;         // the last status it sent whole
    uint32_t statusN = 0;           // ... and its number ("n" in it)
    uint8_t brState = 0;            // 0 free, 1 the cable has the module, 2 the program on the XIAO's WiFi has it
    uint8_t told = 255;             // the state it last told the XIAO (PGN 149; 255: none yet) ...
    uint64_t toldSince = 0, toldAt = 0;     // ... since when it is that one, and when it last said so
    bool detected = false;          // the XIAO said hello in the last 10 s
    bool steering() const { return steerOn_; }      // autosteer is on: steering data keeps coming
    bool engagedNow() const { return engaged || steerOn_; }
    uint64_t bootAt = 0;

    void restart(uint32_t offMs = 2500);    // it restarts (a new subnet, an update, a power dip)
    void log(const std::string& text);
    bool logged(const std::string& part) const;
    uint32_t ms() const { return (uint32_t)(fake::clockMs - bootAt); }

    // ---- used by the world
    void step();
    void wireIn(uint8_t b) { if (on) in_.push_back(b); }
    std::deque<uint8_t> tx;         // its transmit buffer (2048 bytes)
    std::deque<std::vector<uint8_t>> outLog;   // every datagram it handed to the XIAO for the program, in order
    size_t outMatched = 0;

private:
    int room() const { return 2048 - (int)tx.size(); }
    void writeFrame(uint8_t pgn, const uint8_t* d, uint8_t n);
    bool sendOwn(uint8_t pgn, const uint8_t* d, uint8_t n);
    void netOutSend(const std::vector<uint8_t>& d);
    void hello();
    void textByte(uint8_t b);
    void textFlush();
    int byte(uint8_t b);
    int scan();
    void shift(size_t n);
    void takeCmd(const uint8_t* d, uint8_t n);
    void takeFw(const uint8_t* d, uint8_t n);
    void fwAnswer(uint8_t cmd, uint16_t seq, uint8_t code, const char* text);
    void fwStop(uint8_t cmd, uint16_t seq, const char* why);
    void takeNet(const uint8_t* d, uint8_t n);
    void programPacket(const uint8_t* d, int n);
    void bridgeTick();
    void txTick();
    void boot();

    uint64_t backAt_ = 0, lastHelloMs_ = 0, xfLastMs_ = 0, xRxMs_ = 0, textWinMs_ = 0, brStateMs_ = 0, brCableMs_ = 0, brWifiMs_ = 0, brIgnoreMs_ = 0;
    uint64_t xsLastMs_ = 0, fwLastMs_ = 0, last254Ms_ = 0, cableMs_ = 0, gnssMs_ = 0;
    int helloPos_ = 0, textWinN_ = 0;
    std::vector<uint8_t> xf_;
    std::string text_;
    bool textBad_ = false, steerOn_ = false;
    uint8_t brIp_[4] = { 0, 0, 0, 0 };
    uint32_t xlNext_ = 1, n254_ = 0, cableSeq_ = 0, epoch_ = 0;
    std::string xsBuf_;
    size_t xsPos_ = 0;
    uint8_t xsSet_ = 0, xsPart_ = 0, xsParts_ = 0;
    bool fwPending_ = false, fwActive_ = false, fwForce_ = false;
    uint16_t fwExpectSeq_ = 0;
    uint32_t fwSize_ = 0;
    std::vector<uint8_t> fwGot_;
    std::deque<uint8_t> in_;
};

// ---------------------------------------------------------------- a program (AgOpenWeb / AgOpenGPS) on a tablet
struct Program
{
    // ---- what the scenario sets
    bool on = false;                // running, and sending to the module
    bool engaged = false;           // autosteer asked for (the bit in PGN 254)
    bool otherNet = false;          // it is on the other network, not on the XIAO's own
    bool forged = false;            // ... and its datagrams say they come from the XIAO's network all the same
    uint8_t ipLast = 2;             // <the XIAO's network>.<ipLast>: 192.168.4.x, or the module's with the bridge on (or the other network's)
    uint32_t every254Ms = 10;       // AgOpenWeb sends PGN 254 a hundred times a second
    uint32_t rtcmEveryMs = 0;       // corrections (NTRIP) this often (0: none) ...
    size_t rtcmBytes = 256;         // ... of this size (AgOpenWeb and AgIO send them 256 bytes at a time, 25 ms apart at the closest)
    uint32_t extraPerMs = 0;        // a flood: this many more datagrams every millisecond

    // ---- what happened
    struct Sent { uint64_t ms; std::vector<uint8_t> data; bool taken; };
    std::deque<Sent> sent;          // its datagrams for the module, in order (trimmed as they are matched)
    size_t matchFrom = 0;
    uint32_t nSent = 0, nPassed = 0, nNotPassed = 0, nNotTaken = 0;     // not taken: the XIAO's socket did not take it (closed, or full)
    uint32_t worstDelayMs = 0;      // from the tablet to the Teensy
    std::vector<uint8_t> rtcmSent;
    uint32_t got = 0, got253 = 0, gotNmea = 0, gotHello = 0, gotOther = 0;
    uint32_t gotMachine = 0;        // from the other modules (sprayer, seed drill) through the XIAO: PGN 237, 123
    uint32_t gotMachineDirect = 0;  // ... and heard straight from a module that sent to everyone
    uint32_t worstMachineGapMs = 0; // the longest time without one through the XIAO, while machineWatch
    bool machineWatch = false;
    uint64_t lastMachineMs = 0;
    uint64_t lastGotMs = 0;
    uint16_t seq = 0;

    IPAddress ip() const;
    void step();
    void receive(const fake::Datagram& d);
    void send(uint16_t port, const std::vector<uint8_t>& data, bool toAll);     // one datagram, now (8888: for the module, 2233: corrections)
};

// ---------------------------------------------------------------- a section-control module on the XIAO's network
// A sprayer or seed drill module (Sprayer_ESP32, Seeder_ESP32) that joined the XIAO's network, in small: PGN 237
// every 200 ms whether a program runs or not, PGN 123 for every hello, and the rule of the real one: while the
// program's packets come from the XIAO's own address, what it sends goes to that address only (else to everyone).
struct Machine
{
    // ---- what the scenario sets
    bool on = false;
    uint8_t ipLast = 123;           // <the XIAO's network>.<ipLast>
    uint32_t extraPerMs = 0;        // gone wrong: this many more datagrams every millisecond

    // ---- what happened
    uint32_t got = 0, got239 = 0, gotHello = 0, gotSubnetOrScan = 0, gotSteer = 0;  // the program's datagrams that reached it through the XIAO
    uint32_t worstDelayMs = 0;      // from the tablet to this module
    uint32_t sent = 0, heard = 0;   // its own datagrams, and those of them the XIAO's socket took
    uint64_t lastGotMs = 0;         // the last one from the XIAO (0: none yet)
    std::deque<std::vector<uint8_t>> sentLog;       // what the XIAO's socket took, in order (trimmed as it is matched)
    size_t sentMatched = 0;
    uint64_t logAt = 0;             // the next entry of the programs' log it may get (World::progLog)
    uint16_t seq = 0;

    IPAddress ip() const;
    bool throughXiao() const;       // it answers the XIAO's address only now
    void step();
    void receive(const fake::Datagram& d, uint32_t delayMs);
    void send(const std::vector<uint8_t>& data);
};

// ---------------------------------------------------------------- a browser on the XIAO's page
struct Link
{
    bool fromAp = true;             // on the XIAO's own network (else: on the other network it joined)
    uint8_t ipLast = 20;
    size_t upPerMs = 2000;          // bytes it sends a millisecond (a good link)
    size_t downPerMs = 2000;        // bytes it takes a millisecond
    size_t stallAt = SIZE_MAX;      // after this many bytes of the request: nothing for stallMs (a lost packet, a weak link)
    uint32_t stallMs = 0;
    size_t dripAt = SIZE_MAX;       // after this many bytes of the request: only dripBytes get through every dripEveryMs
    size_t dripBytes = 1;
    uint32_t dripEveryMs = 1000;
    size_t vanishAt = SIZE_MAX;     // after this many bytes of the request the link is gone, without a word
    size_t closeAt = SIZE_MAX;      // after this many bytes of the request the browser closes (the tab was closed)
    size_t deafAt = SIZE_MAX;       // after this many bytes of the answer it takes nothing more (gone, without a word)
    uint32_t idleFirstMs = 0;       // it connects, and asks only this much later (a spare connection)
    bool neverAsks = false;         // ... or never
    uint32_t patienceMs = 60000;    // how long it waits for the answer
};
struct Reply
{
    bool done = false;              // a whole answer arrived
    bool failed = false;            // none did: not connected, reset, cut short, no answer in time
    std::string why;
    int code = 0;
    std::map<std::string, std::string> headers;     // names in lower case
    std::string body;
    JsonDoc json;                   // the body, when it is JSON
    bool jsonOk = false;
    uint64_t askedMs = 0, endMs = 0;
    uint32_t tookMs() const { return (uint32_t)(endMs - askedMs); }
    std::string error() const { return json["error"].str(); }
    std::string hdr(const std::string& name) const { auto it = headers.find(name); return it == headers.end() ? std::string() : it->second; }
};
struct Exchange
{
    std::string req, what;
    Link link;
    std::shared_ptr<fake::Conn> conn;
    size_t sent = 0;
    uint64_t startMs = 0, nextTryMs = 0, stallUntil = 0;
    bool stalled = false, gone = false;
    Reply reply;
};
std::string enc(const std::string& s);              // as the page's encodeURIComponent
std::string GET(const std::string& path, const std::string& host = "192.168.4.1", const std::string& extraHeaders = "");
std::string POST(const std::string& path, const std::vector<std::pair<std::string, std::string>>& form, const std::string& host = "192.168.4.1");
std::string UPLOAD(const std::string& pathAndQuery, const std::string& fileName, const std::vector<uint8_t>& file, const std::string& host = "192.168.4.1");

// ---------------------------------------------------------------- everything together
struct World
{
    Xiao xiao;
    Teensy teensy;
    Program tab, tab2, far;         // a tablet on the XIAO's network, a second one, a program on the other network
    std::array<Machine, 5> mod;     // section-control modules on the XIAO's network: .123 and up (all off at first)
    fake::Device& dev;

    // the WiFi name and password the user's devices have for the XIAO's network (they join only when both are right)
    std::string knownSsid = "XIAO-D4C3", knownPass = "agopengps";
    int phones = 1;                 // phones on the XIAO's network besides the tablets
    double noise = 0;               // the chance that a byte on the serial link arrives wrong (each way)
    bool wireCut = false;           // the link's wires are not connected
    bool cutToXiao = false;         // only the wire from the Teensy to the XIAO is broken ...
    bool cutToTeensy = false;       // ... or only the one from the XIAO to the Teensy
    std::string wireJunk;           // not a Teensy on the port: something that keeps sending this, again and again

    // ---- the rules that hold whatever the user does: every break is written down here
    struct Rules
    {
        std::vector<std::string> passed;        // R1 something reached the Teensy through the bridge that should not have
        std::vector<std::string> dest;          // R2 a datagram left the XIAO for someone else than the program, or one nobody sent
        std::vector<std::string> answers;       // R5 an answer to a browser that is not well formed
        uint32_t worstHelloGapMs = 0;           // R3 the longest time without a hello while the XIAO ran
        uint32_t worstSteerGapMs = 0;           // R6 the longest time without steering data at the Teensy while the tablet steered
        uint32_t worstAnswerMs = 0;
    } rules;
    int restartsExpected = 0;       // restarts the user asked for (a save, an update, the restart button, a USB command)
    int watchdogExpected = 0;       // restarts by the watchdog that the scenario is about (the page stuck for over a minute)

    explicit World(Kit k);
    ~World();
    void start();                   // power on, and let it settle for a second
    void run(uint32_t ms);
    bool runUntil(const std::function<bool()>& cond, uint32_t maxMs);
    void flash(const char* key, const std::string& v);      // a setting in the XIAO's flash, as saved earlier
    void flash(const char* key, const char* v) { flash(key, std::string(v)); }
    void flash(const char* key, bool v);
    std::string flashStr(const char* key) const;
    bool flashBool(const char* key) const;
    bool hasFlash(const char* key) const;
    void bridgeSaved(const std::string& pass = "trator-2026");    // the user switched the bridge on earlier (own password)
    bool apUsable() const;          // the user's devices are on the XIAO's network now
    void usb(const std::string& line);      // typed in the USB serial monitor
    bool said(const std::string& part) const;       // the XIAO wrote this in its events (Messages tab, USB)
    void noteUserRestart() { restartsExpected++; }
    // Something to do ms from now. A scenario's own code does not run while the program is inside one long loop()
    // pass (the web server waiting for a phone): what must happen in the middle of such a wait goes here.
    void later(uint32_t ms, std::function<void()> fn) { later_.push_back({ fake::clockMs + ms, fn }); }

    int ask(const std::string& req, const Link& l = Link(), const std::string& what = "");
    Reply& reply(int id) { return exchanges_[id]->reply; }
    Reply wait(int id);
    Reply http(const std::string& req, const Link& l = Link());
    Reply get(const std::string& path, const Link& l = Link()) { return http(GET(path, hostFor(l)), l); }
    Reply post(const std::string& path, const std::vector<std::pair<std::string, std::string>>& form, const Link& l = Link()) { return http(POST(path, form, hostFor(l)), l); }
    Reply upload(const std::string& pathAndQuery, const std::string& fileName, const std::vector<uint8_t>& file, const Link& l = Link()) { return http(UPLOAD(pathAndQuery, fileName, file, hostFor(l)), l); }
    JsonDoc state();                // GET /api/state
    std::string hostFor(const Link& l) const;

    // ---- used by the Teensy, the programs and the modules
    struct ProgSent { uint64_t ms; std::vector<uint8_t> data; };
    std::deque<ProgSent> progLog;   // what the programs on the XIAO's network sent for the module and its socket took, in order
    uint64_t progLogBase = 0;       // the number of its first entry (the log is trimmed)
    void step();
    void bridgeFrame(const uint8_t* d, uint8_t n);          // a PGN 146 arrived at the Teensy: is it allowed?
    void steerSeen() { steerLastMs_ = fake::clockMs; }
    uint8_t noisy(uint8_t b);
    uint64_t helloLastMs_ = 0, helloWatchFrom_ = 0, steerLastMs_ = 0, apUpSince_ = 0;
    bool steerWatch_ = false;       // the tablet steers through the bridge now: the gaps in its data at the Teensy count
    bool bridgeAtBoot_ = false;     // the bridge was switched on (with a password of one's own) when the XIAO last started

private:
    void wires();
    void datagramsOut();
    void browsers();
    void finishReply(Exchange& e, bool closed);
    std::vector<std::unique_ptr<Exchange>> exchanges_;
    std::vector<std::pair<uint64_t, std::function<void()>>> later_;
    int bootsSeen_ = -1;
    bool teensyWasOn_ = true, wireWasCut_ = false, toTeensyWasCut_ = false;
    double accOut_ = 0, accIn_ = 0;
    size_t junkAt_ = 0;
    uint32_t rng_ = 12345;
    uint32_t rnd() { rng_ = rng_ * 1664525u + 1013904223u; return rng_ >> 8; }
};
World* world();
IPAddress apAt(uint8_t last);       // <the XIAO's own network>.<last>
}
