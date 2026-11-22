/* bench.h - the test bench: the modules (the real firmware on a pretend board), AgOpenWeb on the tablet, the
   tractor network and the sprayer behind a sprayer module.
   Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The bench runs in steps of 1 ms of pretend time: AgOpenWeb sends what the real program sends (PGN 239 ten
// times a second, 229 with it on a tool of more than 16 sections, a hello every second), the sprayer turns the
// open valves into pressure and flow-meter pulses, every module runs loop() once, and what the modules send
// reaches the tablet when the network lets it through. The relays are read from the board itself (pins, the
// 74HC595 chain, the PCF8574 chips), never from the firmware's own variables.
#pragma once
#include "../fake/fake_core.h"
#include "json.h"
#include <memory>

namespace bench
{
enum class Board { SprayerX8, SprayerDevkit, SeederRelay24, SeederKC868 };

struct BoardInfo
{
    Board       id;
    const char* key;            // short name in the report
    const char* name;           // what the firmware calls the board (/api/status "board")
    bool        sprayer;        // Sprayer_ESP32 (else Seeder_ESP32)
    int         relays;
    const char* lib;            // the firmware built for this board (build/<lib>.so)
};
const BoardInfo& info(Board b);
std::vector<Board> sprayerBoards();
std::vector<Board> seederBoards();
std::vector<Board> allBoards();

// An AgOpenGPS message: 80 81 src pgn len data crc
std::vector<uint8_t> pgn(uint8_t number, const std::vector<uint8_t>& data, uint8_t src = 0x7F);

struct Http
{
    int code = 0;               // 0 = no answer (module off)
    std::string body;
    Json json;
    bool jsonOk = false;
};

class Module
{
public:
    Module(Board b, const std::string& label, uint64_t mac);
    ~Module();
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;

    Board            board;
    const BoardInfo& bi;
    std::string      label;
    fake::Device     dev;
    uint32_t         boots = 0;

    void powerOn();
    void powerOff();
    void resetBy(esp_reset_reason_t why);   // the chip resets by itself (a brownout, a crash, the watchdog) and starts again
    bool on() const { return so_ != nullptr; }
    void step();                // loop() once; a restart the program asks for loads it again
    uint32_t relays() const;    // relays whose coil is on, relay 1 = bit 0

    Http http(HTTPMethod m, const std::string& uri, const std::map<std::string, std::string>& args = {},
              bool fromAp = true, const std::vector<uint8_t>* file = nullptr, const std::string& fileName = "");
    Json status();              // GET /api/status; a broken JSON answer is counted in badStatus
    int  badStatus = 0;
    std::string lastBadStatus;

    // the board's own pins (for the relay model and the sprayer)
    static const std::vector<uint8_t>& relayPins(Board b);

private:
    void load();
    void unload();
    void callSetup();
    void* so_ = nullptr;
    void (*setup_)() = nullptr;
    void (*loop_)() = nullptr;
    std::string soPath_;
    uint32_t srShift_ = 0, srOut_ = 0;      // the Relay24's 74HC595 chain: shifting in, latched to the outputs
    uint8_t  lastClock_ = 0, lastLatch_ = 0;
};

struct Heard
{
    uint64_t ms;
    IPAddress src;
    std::vector<uint8_t> data;
    uint8_t pgn() const { return data.size() > 3 ? data[3] : 0; }
};

// AgOpenWeb on the tablet
struct Aow
{
    bool      on = true;                    // running, on the tractor network
    IPAddress ip { 192, 168, 5, 10 };
    uint64_t  sections = 0;                 // the sections it asks for, section 1 = bit 0
    int       toolSections = 8;             // over 16: PGN 229 goes with every 239
    float     kmh = 8.0f;
    uint8_t   hydLift = 0, tram = 0, geo = 0, uTurn = 0;
    uint16_t  widthCm[16] = { 300, 300, 300, 300, 300, 300, 300, 300 };
    bool      send229 = true;               // false: an older program that never sends PGN 229
    std::vector<Heard> heard;               // what reached the tablet on UDP 9999

    void send(const std::vector<uint8_t>& pkt, IPAddress dst = IPAddress(), uint16_t port = 8888);
    void sendMachineConfig(uint8_t raise, uint8_t lower, uint8_t hydEnable, bool invert);
    void sendPinConfig(const std::vector<uint8_t>& pins);
    void sendWidths();
    void sendSubnet(uint8_t a, uint8_t b, uint8_t c);
    void sendScan();
    void startup(bool invert = false, const std::vector<uint8_t>& pins = {});   // 238, 236, 235 as at the program's start
    std::vector<Heard> heardPgn(uint8_t number, uint64_t sinceMs = 0) const;

    uint64_t last239 = 0, lastHello = 0;
    std::deque<fake::Packet> out;           // sent, not delivered yet
};

// The sprayer behind a Sprayer_ESP32 module: valves, pump, nozzles, pressure sensor, flow meter
struct SprayerRig
{
    Module* m = nullptr;
    int valves = 8;
    std::vector<int> relayValve;            // relay i drives valve relayValve[i] (-1: nothing); default relay i -> valve i
    bool ncWiring = false;                  // the valves open while their relay is OFF
    std::vector<int> nozzles;               // per valve (default 6: 3 m at 0.5 m)
    std::vector<bool> stuckOpen;            // a valve that never closes
    float nozzleLpm = 1.6f, chartBar = 3.0f;    // the real nozzles: L/min at chart pressure
    bool  pumpOn = true;
    float pumpBar = 3.0f;
    bool  pressureFitted = true, flowFitted = true, swapped = false;
    float sensorMaxBar = 10.0f, divider = 0.5f, meterPpl = 450.0f;   // the real sensor, divider and meter

    float bar = 0, lpm = 0;
    double pulseAcc = 0;
    uint32_t noise = 12345;
    std::vector<bool> valveOpen() const;
    void tick();                            // 1 ms
};

// The AiO board's XIAO on a tractor with no router, its WiFi bridge on: the tablet and the modules are on its
// network (192.168.4.x, the XIAO at .1). It gives every module it knows its own copy of each message of the tablet,
// at once and from its own address; the tablet's own broadcast reaches the modules too, but late (an access point
// holds a broadcast for its next beacon) or not at all. What a module sends reaches the tablet through the XIAO.
// A module is known from what it sends, and forgotten 3 s after the last.
struct XiaoAp
{
    bool     on = false;                    // the tractor network is the XIAO's (World::useXiao())
    bool     bridge = true;                 // its WiFi bridge is switched on
    uint32_t lateMs = 150;                  // how late the tablet's own broadcast reaches a module
    int      loseOneIn = 0;                 // of those broadcasts, one in this many never arrives (0: all arrive)
    std::map<uint32_t, uint64_t> known;     // the modules it knows: address -> when last heard
    std::deque<std::pair<uint64_t, fake::Packet>> late;     // broadcasts on their way
    uint32_t copies = 0;                    // copies given to modules
    uint32_t passedOn = 0;                  // modules' messages passed on to the tablet
    uint32_t toEveryone = 0;                // modules' messages that went to everyone (not to the XIAO only)
    uint32_t lateCount = 0;
    static IPAddress ip() { return IPAddress(192, 168, 4, 1); }
};

class World
{
public:
    World();
    ~World();
    Aow aow;
    XiaoAp xiao;
    void useXiao();                         // no router: the tractor network is the XIAO's (name XIAO-3F2A, password trator-2026)
    std::vector<std::unique_ptr<Module>> mods;
    std::unique_ptr<SprayerRig> rig;
    std::vector<std::function<void()>> watchers;    // run every ms (rules that must hold all the time)

    Module&     add(Board b, const std::string& label = "module");
    SprayerRig& addRig(Module& m);
    void        run(uint32_t ms);
    uint64_t    now() const { return fake::clockMs; }
};

// The careful user's set-up of the network on the module's page: the tractor's Wi-Fi (the sprayer; the seed
// drill uses its cable unless told), then the restart that saving does
void joinTractorWifi(World& w, Module& m, const std::string& pass = "tractor123", const std::string& subnet = "192.168.5",
                     int ipLast = 123);
// Sections 1..n as a bit mask
inline uint64_t first(int n) { return n >= 64 ? ~0ull : ((1ull << n) - 1); }
std::string hex(uint64_t v);
}
