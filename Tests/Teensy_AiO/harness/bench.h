/* bench.h - the test bench: the AiO module (the real Teensy firmware on a pretend Teensy 4.1), the tractor with its
   steering, the receiver, the IMU, the wheel angle sensor, the Keya motor, AgOpenWeb on the tablet and a browser.
   Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The bench runs on the pretend clock (fake::clockUs). The firmware's loop() runs over and over; every
// millisecond of pretend time (also inside a loop() that waits) the world takes one step: the steering moves the
// wheels by what the pins / the CAN bus tell it, the tractor drives, the receiver and the IMU send their bytes at
// their baud rate, the Keya sends its heartbeat, AgOpenWeb sends its PGNs, the browser reads, and the safety rules
// are checked. What the module does is read from its pins and its CAN frames, never from the firmware's variables.
#pragma once
#include "../fake/fake_core.h"
#include "json.h"
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bench
{
enum class Gnss { None, UM982, F9P };
enum class Imu { None, TM171, BnoRvc };
enum class Act { Cytron, IBT2, Danfoss, Keya };

// An installation: what is fitted to the AiO board and how AgOpenWeb is set up for it when all is right
enum class Kit { PumaCytron, F9pIbt2, Um982Keya, F9pDanfoss };
struct KitInfo
{
    Kit         id;
    const char* key;            // short name in the report
    const char* name;
    Gnss        gnss;
    Imu         imu;
    Act         act;
    uint8_t     steerMode;      // 0 none (the screen button), 1 switch, 2 button
};
const KitInfo& info(Kit k);
std::vector<Kit> allKits();
std::vector<Kit> valveKits();   // the kits steered through the board's pins (not the Keya)

// An AgOpenGPS message: 80 81 src pgn len data crc
std::vector<uint8_t> pgn(uint8_t number, const std::vector<uint8_t>& data, uint8_t src = 0x7F);

// AgOpenWeb's steer settings (PGN 252) and steer configuration (PGN 251)
struct SteerSettings
{
    uint8_t kp = 60, highPwm = 150, lowPwm = 30, minPwm = 25, counts = 120;
    int16_t wasOffset = 0;
    uint8_t ackerman = 100;
};
struct SteerConfig
{
    bool invertWas = false, relayActiveHigh = false, motorDir = false, singleWas = true, cytron = true;
    bool steerSwitch = false, steerButton = false, encoder = false;
    uint8_t pulseCountMax = 3, minKmhX10 = 5;
    bool danfoss = false, pressure = false, current = false, yAxis = false;
};

struct Http
{
    int code = 0;               // 0 = no answer
    std::string head, body;
    Json json;
    bool jsonOk = false;
    uint32_t ms = 0;            // how long the answer took (pretend time)
};

struct Heard
{
    uint64_t ms;
    uint16_t srcPort;
    std::vector<uint8_t> data;
    uint8_t pgn() const { return data.size() > 3 && data[0] == 0x80 && data[1] == 0x81 ? data[3] : 0; }
};

// ---------------------------------------------------------------- the AiO module: the firmware on a pretend Teensy
class Aio
{
public:
    Aio();
    ~Aio();
    Aio(const Aio&) = delete;
    Aio& operator=(const Aio&) = delete;

    fake::Device dev;
    uint32_t     boots = 0;
    std::string  log;               // everything the firmware wrote on the USB serial (its [boot] / [status] lines)
    std::vector<std::string> bootReasons;
    uint64_t     loopMaxUs = 0;     // the longest loop() call since resetLoopMax()
    uint64_t     loopCostUs = 200;  // the time one pass of loop() takes at least (the real Teensy: 5000 passes a second)
    bool         hung = false;      // the firmware stopped (a scenario: hang()): no loop() runs, the clock goes on

    void powerOn();
    void powerOff();
    bool on() const { return so_ != nullptr; }
    void step();                    // loop() once; a restart (software, watchdog, update) loads the firmware again
    void crash();                   // a fault: the crash hook runs, then the core's handler waits for the watchdog
    void resetLoopMax() { loopMaxUs = 0; }
    size_t logMark() const { return log.size(); }
    bool logHas(const std::string& text, size_t from = 0) const { return log.find(text, from) != std::string::npos; }
    std::string logLine(const std::string& text, size_t from = 0) const;   // the first line holding text

private:
    void load();
    void unload();
    void boot();
    void* so_ = nullptr;
    void (*setup_)() = nullptr;
    void (*loop_)() = nullptr;
    void (*faultHookC_)(const uint32_t*) = nullptr;
    void (*wdtWarnC_)(const uint32_t*) = nullptr;
    std::string soPath_;
};

// ---------------------------------------------------------------- the world around it
// ADS1115 on Wire1 (0x48): the wheel angle sensor's voltage, converted as the chip does (16 bits, its PGA, its rate)
struct Ads1115 : fake::I2cChip
{
    float ain[4] = { 2.5f, 0, 0, 0 };   // volts on AIN0..AIN3 (set by the world every ms)
    uint8_t  pointer = 0;
    uint16_t config = 0x8583;
    int16_t  result = 0, pending = 0;
    uint64_t readyUs = 0;
    bool     converting = false;
    void write(const uint8_t* d, size_t n) override;
    size_t read(uint8_t* d, size_t n) override;
    void tick();
};

// The tractor and its steering
struct Tractor
{
    float x = 0, y = -20, headingDeg = 0;   // metres east / north of the field's origin, degrees from north
    float kmh = 0;                          // ground speed (the scenario sets it)
    float wheelDeg = 0;                     // front wheel angle, + = right
    float lockDeg = 40;                     // the wheels stop at +-lockDeg
    float wheelbase = 3.0f;
    float rollDeg = 0;
    float maxRateDps = 30;                  // the steering's top speed, degrees a second
    int   deadPwm = 20;                     // PWM under this does not move the wheels (valve / motor)
    bool  wiredBackwards = false;           // the valve coils / motor wires swapped: the wheels go the other way
    bool  lockValveFitted = true;           // Cytron / Danfoss: pin 3 drives a valve (or the PVE's power) that must be on
    float handDps = 0;                      // the driver turns the wheel (degrees a second), whatever the module does
    float moduleDps = 0;                    // what the module's output does to the wheels now (degrees a second)
    float travelM = 0;
};

// The wheel angle sensor (WAS): 5 V supply, 0.045 V per degree around 2.5 V, on ADS1115 AIN0
struct Was
{
    float centreV = 2.5f, vPerDeg = 0.045f;
    bool  mountedBackwards = false;     // the voltage falls when the wheels turn right
    bool  unplugged = false;            // the plug out: the input floats at floatV
    float floatV = 3.93f;               // where an open input settles (a Puma, 19 Nov 2026: 3.93 V)
    bool  shortToGround = false;        // the signal wire cut and touching the frame: 0 V
    bool  noLowWire = false;            // differential input and nothing on WAS_L (pin 3): it floats near 5 V
    float noiseV = 0.002f;
    float volts(float wheelDeg, uint32_t& seed) const;
};

struct Receiver
{
    Gnss     kind = Gnss::UM982;
    bool     on = true;                 // powered and sending
    uint32_t baud = 460800;             // its port's speed (the module listens at 460800)
    int      port = 7;                  // the Teensy serial port it is wired to (7 = the GPS port; 2 = the GPS2 port)
    char     fix = '4';                 // GGA quality: '4' RTK fixed, '5' float, '1' GPS, '0' none
    int      sats = 32;
    bool     hpr = true;                // UM982: the dual-antenna heading (HPR)
    int      rateHz = 10;
    bool     garbage = false;           // sends noise instead of NMEA
    std::deque<uint8_t> wire;           // bytes on their way at the baud rate
    double   credit = 0;
    uint64_t lastEpochMs = 0;
    uint32_t epochs = 0;
    std::string fromModule;             // what the module sent it (commands, RTCM), the last bytes
    uint32_t bytesFromModule = 0;
    uint32_t baudChanges = 0;           // the module set it to 460800 (CONFIG COM1 460800 / UBX-CFG-VALSET)
};

struct ImuDev
{
    Imu      kind = Imu::TM171;
    bool     on = true;
    uint32_t baud = 115200;
    bool     frozen = false;            // keeps sending the same values
    std::deque<uint8_t> wire;
    double   credit = 0;
    uint64_t lastMs = 0, lastStatusMs = 0;
    uint8_t  rvcIndex = 0;
    float    frozenRoll = 0, frozenYaw = 0;
};

struct Keya
{
    bool     fitted = false;
    bool     powered = true;            // its 12 V
    bool     canWired = true;           // CAN H / L right and plugged in
    bool     enabled = false;
    int16_t  speedCmd = 0;              // the last speed command (+-1000 = full)
    uint64_t lastCmdMs = 0;             // its own safety: no command for cmdTimeoutMs = it stops
    uint32_t cmdTimeoutMs = 500;        // (assumed; the Keya's own value is not documented)
    uint16_t faults = 0;                // byte 6 << 8 | byte 7 of its heartbeat
    float    motorDeg = 0;              // its position (degrees of the steering wheel)
    float    amps = 0;
    uint64_t lastBeatMs = 0;
    uint32_t enables = 0, speedCmds = 0;
};

struct Switches
{
    bool steerClosed = false;           // the steer switch / button (pin 32): closed = LOW
    bool workClosed = false;            // pin 34
    bool remoteClosed = false;          // pin 37
};

// AgOpenWeb (or AgOpenGPS) on the tablet
struct Program
{
    bool      on = true;
    bool      agio = false;             // AgOpenGPS / AgIO (its hello ends in 0x47) instead of AgOpenWeb
    IPAddress ip { 192, 168, 5, 10 };
    bool      engage = false;           // its AutoSteer button
    bool      guide = true;             // follow the line x = lineX (else: setDeg as the scenario says)
    float     lineX = 0;
    float     setDeg = 0;               // the steer angle it asks for
    float     kmhOverride = -1;         // the speed it sends (default: the tractor's)
    uint32_t  period254 = 100;          // ms between steering packets
    bool      send254 = true, sendHello = true;
    SteerSettings ss;
    SteerConfig   sc;
    std::vector<Heard> heard;           // what reached the tablet on UDP 9999 from the module
    std::vector<std::string> screen;    // PGN 221 texts shown on its screen
    std::vector<std::string> nmea;      // NMEA lines from the module (kept short)
    uint32_t  nmeaLines = 0, nmeaBadSum = 0, nmeaGga = 0;
    uint64_t  lastNmeaMs = 0;
    uint64_t  last254Ms = 0, lastHelloMs = 0;
    bool      lastEngageSent = false;   // AgOpenWeb holds an unchanged steering packet back for 30 ms after its AutoSteer
    uint64_t  holdUntilMs = 0;          // button changed (the firmware reads the change every 25 ms; matters at 100 a second)
    uint32_t  dropEvery = 0;            // the network loses every Nth packet to the module (0 = none)
    uint32_t  sentToModule = 0;
    std::deque<fake::UdpPacket> out;    // sent, not delivered yet

    void send(const std::vector<uint8_t>& pkt, uint16_t port = 8888, IPAddress dst = IPAddress());
    void send252();
    void send251();
    void sendSettings() { send252(); send251(); }
    void sendSubnet(uint8_t a, uint8_t b, uint8_t c);
    void scan();
    std::vector<uint8_t> packet254(float kmh) const;
    std::vector<Heard> heardPgn(uint8_t number, uint64_t sinceMs = 0) const;
    bool screenHas(const std::string& text, size_t from = 0) const;
};

// The XIAO on the GPS2 port (Serial2, 460800 baud), as its program does it (Firmware/.../v4.5_XIAO_ESP32): the hello
// every 2 s and, with the WiFi bridge switched on in its page, the packets of a program on its own WiFi to and from
// the module. The bridge's rules of the real program: nothing to a module that does not say it knows the bridge
// (PGN 149, at most 3 s old); only the program's hello while a program on the network cable has the module.
struct Xiao
{
    bool     fitted = false;            // plugged in and running
    bool     bridge = false;            // the WiFi bridge switched on
    bool     forwardsAll = false;       // a XIAO that keeps none of the rules (the module must keep its own)
    std::deque<uint8_t> wire;           // bytes on their way to the module
    double   credit = 0;
    uint64_t lastHelloMs = 0;
    bool     helloSent = false;
    std::vector<uint8_t> in;            // bytes from the module, not a whole frame yet
    uint8_t  state = 255;               // what the module says (PGN 149): 0 free, 1 a program on the cable has it, 2 the
                                        // program on the WiFi has it; 255 = it has not said (for 3 s)
    uint64_t stateMs = 0;
    uint32_t frames = 0, badSum = 0, states = 0;
    uint32_t toModule = 0;              // program packets handed to the module (PGN 146)
    uint32_t toProgram = 0;             // datagrams for the program from the module (PGN 148)
    uint32_t rtcmToModule = 0;          // bytes of corrections handed to the module (PGN 147)
    uint32_t dropped = 0;               // not sent: its transmit buffer (1024 bytes) was full
    std::map<uint8_t, uint32_t> pgns;   // frames from the module, by PGN
    uint8_t  version = 0;               // the bridge's version the module says (PGN 149): 2 = it serves its page through the XIAO
    // The module's page through the XIAO (port 8080 of its WiFi, PGN 150 / 151), as the XIAO program: one browser at a
    // time; it tells the module how much of the answer it holds (the window) and how much the browser took
    struct Web
    {
        bool        on = false;         // a request open
        uint8_t     sid = 0;
        std::string req;                // the browser's request
        size_t      reqSent = 0;        // ... handed to the module
        bool        opened = false;     // "open" sent
        size_t      window = 6144;      // answer bytes the XIAO holds for the browser
        std::string held;               // ... held now, not taken by the browser yet
        std::string got;                // what the browser has
        uint32_t    taken = 0, ackedAt = 0;     // answer bytes the browser took; the count last sent to the module
        bool        ended = false;      // the module said the end
        uint32_t    takePerMs = 4000;   // the browser's speed (0: it takes nothing)
        uint32_t    overWindow = 0, badOffset = 0;
        int         dropPieceAt = -1;   // a request piece lost on the link: the one that starts at this offset
    } web;
    uint32_t webSessions = 0;
    void frame(uint8_t pgn, const uint8_t* data, size_t n);
};

// A rule the module must keep all the time, checked every millisecond
struct Rule
{
    std::string key, what;
    uint32_t violations = 0;
    uint64_t firstMs = 0;
    std::string firstDetail;
};

class World
{
public:
    explicit World(Kit kit);
    ~World();
    const KitInfo& ki;
    Aio      aio;
    Tractor  tractor;
    Was      was;
    Ads1115  ads;
    bool     adsFitted = true;
    Receiver rx;
    ImuDev   imu;
    Keya     keya;
    Switches sw;
    Program  aow;
    Xiao     xiao;                      // on the GPS2 port (not fitted unless a scenario says so)
    Program  tab;                       // AgOpenWeb on a tablet with no network cable, on the XIAO's own WiFi (off unless...)
    uint32_t netTo9999 = 0;             // datagrams the module put on the network cable for the program's port
    bool     cableIn = true;            // the module's network cable
    bool     tabletOnNet = true;

    // the module's steering, read from its pins / CAN frames
    bool     outputActive() const;      // anything that can move the wheels: a PWM, the valve relay, the Keya enabled
    bool     engaged() const;           // the module steers: as above, or an IBT2's enable on (its PWM is 0 on the line)
    float    outputDps() const { return tractor.moduleDps; }   // what it does to the wheels
    bool     relayOn() const;           // pin 3 as the valve / PVE power (Cytron, Danfoss)
    uint64_t now() const { return fake::clockUs / 1000; }

    // the safety rules (on by default): outputs only while the program asks, the right way
    std::vector<Rule> rules;
    bool     ruleSteerOnlyWhenAsked = true;
    bool     ruleRightWay = true;
    // "from now on the outputs must be off within graceMs" (a WAS fault, a switch off...): expectOff() .. expectOffEnd()
    void     expectOff(const std::string& why, uint32_t graceMs);
    void     expectOffEnd() { offWhy_.clear(); }
    uint64_t lastAskMs = 0;             // the last steering packet that asked to steer (delivered to the module)
    uint64_t lastStopMs = 0;            // the last one that did not (or the program's silence starts)
    float    setAtModule = 0;           // the steer angle in the last steering packet the module got
    uint64_t switchOpenedMs = 0;        // when the steer switch was last opened
    bool     expectWatchdog = false;    // a scenario that makes the watchdog restart the module on purpose
    float    maxWrongWayDeg = 0;        // how far the wheels went the wrong way under the module's push
    uint64_t engagedMs = 0;             // time engaged()
    float    maxAbsErrDeg = 0;          // |wheel - asked| while active, after the first 2 s of each engage
    std::vector<std::function<void()>> watchers;

    void     run(uint32_t ms);          // the module's loop and the world for ms of pretend time
    void     runUntil(const std::function<bool()>& done, uint32_t maxMs);
    void     startUp(uint32_t ms = 8000);   // power on, wait for set-up, the program's hello and settings
    Http     http(const std::string& method, const std::string& path, const std::string& body = "",
                  const std::vector<std::string>& headers = {}, uint32_t timeoutMs = 6000);
    Http     httpRaw(const std::string& request, uint32_t timeoutMs = 6000);
    // the same through the XIAO (a phone on its WiFi, http://<the XIAO>:8080/); at the timeout the phone goes (the XIAO
    // tells the module so, unless sayGone is false: a XIAO that just stops)
    Http     httpXiao(const std::string& method, const std::string& path, const std::string& body = "",
                      const std::vector<std::string>& headers = {}, uint32_t timeoutMs = 15000);
    Http     httpXiaoRaw(const std::string& request, uint32_t timeoutMs = 15000, bool sayGone = true);
    Json     status();                  // GET /api/status
    int      badStatus = 0;
    std::string lastBadStatus;
    void     upload(const std::vector<uint8_t>& file, Http& out, const std::string& query = "", uint32_t bytesPerMs = 200);

    // the careful user's AgOpenWeb settings for this kit
    void     setUpProgram();

private:
    void tick(uint64_t ms);
    void tickSteering(uint64_t ms);
    void tickGnss(uint64_t ms);
    void tickImu(uint64_t ms);
    void tickKeya(uint64_t ms);
    void tickNetwork(uint64_t ms);
    void tickXiao(uint64_t ms);
    void xiaoHears(const uint8_t* b, size_t n, uint64_t ms);
    void tickRules(uint64_t ms);
    void deliverToModule(const fake::UdpPacket& p);
    void noteAsk(const std::vector<uint8_t>& x);
    void programStep(Program& a, uint64_t ms);
    void programHears(Program& a, const std::vector<uint8_t>& data, uint16_t srcPort, uint64_t ms);
    std::string offWhy_;
    uint64_t offFrom_ = 0;
    uint32_t seed_ = 12345;
    uint64_t lastProgramStep_ = 0;
    bool     wasActive_ = false;
    uint64_t activeSinceMs_ = 0;
    float    wrongFrom_ = 0;
    int      wrongDir_ = 0;
    uint64_t wrongSinceMs_ = 0;
    bool     lastSteerClosed_ = false;
    Rule&    rule(const std::string& key, const std::string& what);
    void     violate(const std::string& key, const std::string& what, const std::string& detail);
};

// The rules' results as checks (call at the end of a scenario)
void reportRules(World& w);
std::string fmt1(float v);
}
