/* zMachine.ino - the machine module (IP .123) as seen from the network, for the web page's Machine tab.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   25 Oct 2026: any standard machine module is recognised, the official AgOpenGPS Machine_UDP_v5 included
   (its hello reply has no checksum, its PGN 237 carries the program's source byte); the sections come from
   the hello reply when there is no sprayer PGN 160.
   26 Oct 2026: sections 9-16 of the sprayer PGN 160 (byte 15, Sprayer_ESP32 v0.7); the highest section
   seen since the start is kept, so the page keeps all its squares while every section is off.
   27 Oct 2026: up to 64 sections: those the program asks for from its PGN 229 (Autosteer.ino ReceiveUdp,
   sent with every 239 when the tool has more than 16 sections), those open from the seed drill module's
   PGN 237 (Seeder_ESP32, sections 17-64 in bytes 10-15).
   27 Oct 2026: the sections of every machine module, whatever it is: PGN 237 from 0x7B in the AgOpenGPS AiO v26
   layout (sections 1-16 the module really drives; 17-64 in bytes 13-18 when it sends them) before the hello
   reply (the AiO v26 leaves its sections at 0 there) and PGN 160; several modules on the network (each its own
   IP) are kept apart and their sections put together.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zMachine.ino
// The machine module (source 0x7B = 123, IP .123) answers the program's hello (PGN 200) with PGN 123, the
// sections it has on in two bytes, and sends its own PGNs to UDP 9999, broadcast to the whole 192.168.5.x
// network. The Teensy listens on 9999 too (Eth_udpModules, zEthernet.ino) and keeps only what comes from
// the machine module; everything else on 9999 (the other modules, the Teensy's own sentences) is read and
// dropped. Nothing is sent and nothing changes in the steering: it is only for the web page (Machine tab,
// shown once the module has been heard) and the [machine] messages.
//
// These modules are known, and anything that talks like them works:
//   - the official AgOpenGPS Machine_UDP_v5 (Arduino Nano + ENC28J60). Its hello reply leaves the checksum
//     byte at 71, never computed, so it is taken without a checksum. Every 200 ms it also sends a PGN 237
//     with fixed content and the program's source byte 0x7F (a leftover of the old code): here it only
//     counts as a sign of life. With "Invert relay" on in the program, its hello reply carries the output
//     levels, so the sections read reversed.
//   - the machine part of the AgOpenGPS AiO v26 firmware (Firmware_Teensy_AiO_26, MachineProcessor): hello
//     reply with its sections left at 0, and PGN 237 from 0x7B fifty times a second with what it drives
//     (layout below).
//   - the sprayer module "Sprayer_Nano" (same hardware as the official one): hello reply with a right
//     checksum, and PGN 160.
//   - our ESP32 modules, "Sprayer_ESP32" (PGN 237 and 160) and "Seeder_ESP32" (PGN 237 with sections 17-64).
//
// The program asks for sections 1-16 in PGN 254 (relay, relayHi; Autosteer.ino) and, on a tool with more
// than 16 sections, for sections 1-64 in PGN 229 (bytes 5-12, section 1 = byte 5 bit 0), which it sends to
// UDP 8888 with every 239. The hello reply and PGN 160 carry sections 1-16 only.
//
// What a module has open comes, by its IP, from its PGN 237 when it sent one in the last MACH_237_MS, else
// from its PGN 160, else from its hello reply. Several boards can answer as the machine module (each with its
// own IP, say one for sections 1-16 and one for 17-32): up to MACH_MODS of them are kept apart, and the tab
// shows the sections of all those heard lately together.
//
// Sprayer PGN 160 (0xA0), once a second, 16 bytes (17 with sections 9-16):
//   80 81 7B A0 0A | pressure (cbar, 2) | flow (cL/min, 2) | total (dL, 4) | main switch (1) | sections (1) | CRC
//   80 81 7B A0 0B | ... the same ... | sections 1-8 (1) | sections 9-16 (1) | CRC   (Sprayer_ESP32 v0.7)
//   (little-endian; CRC = sum of the bytes from the 3rd to the one before the CRC, as in AgOpenGPS)
//
// PGN 237 (0xED) from 0x7B, as the AiO v26 sends it (the official Machine_UDP_v5's 237 has 8 bytes of fixed
// content and the source 0x7F instead):
//   80 81 7B ED 08 | sections 1-16 (2) | lift (1: 0 off, 1 down, 2 up) | tramline (1) | geo stop (1) |
//                    status (1: bit 0 = the program's PGN 239 still arrives) | 0 | 0 | CRC
//   80 81 7B ED 0E | ... the same 8 bytes ... | sections 17-64 (6) | CRC   (Seeder_ESP32)

#define MACH_SRC          0x7B   // the machine module's source byte (IP .123)
#define MACH_PGN_HELLO    123    // the module's reply to the program's hello (PGN 200): sections on, 2 bytes
#define MACH_PGN_LIFE     237    // the official module's periodic PGN (source 0x7F, fixed content); the seed drill's (0x7B)
#define MACH_PGN_SPRAYER  0xA0   // 160
#define MACH_PKTS_LOOP    16     // packets read from UDP 9999 per loop at most
#define MACH_SILENCE_MS   5000   // silent after this long (as STATUS_SILENCE_MS, which zStatus.ino defines after this file)
#define MACH_229_MS       2000   // sections 17-64 asked for: only from a PGN 229 younger than this
#define MACH_237_MS       2000   // a module that sent PGN 237 this lately: its sections from that, not the hello reply
#define MACH_MODS         4      // machine modules (IPs) kept apart

static struct
{
    uint32_t lastMs;             // last packet of any kind from the machine module (0: never)
    uint32_t pkts, bad;          // its packets, those with a wrong checksum
    int16_t  lastPgn;
    uint8_t  ip[4];
    bool     here;               // heard in the last MACH_SILENCE_MS (for the [machine] lines)

    uint32_t helloMs, helloN;    // last hello reply (0: never), how many
    bool     helloNoCrc;         // the hello reply came without a checksum (the official module)
    uint32_t secMs;              // last word on the sections, from the hello reply, PGN 160 or 237 (0: never)
    uint8_t  secPgn;             // which of them: 123, 160 or 237
    uint64_t sections;           // bit 0 = section 1, those open on all the modules heard lately
    uint8_t  secTop;             // the highest section asked for or reported since the start (0: none yet)
    struct
    {
        uint32_t ip;             // the module's IP (0: free)
        uint32_t ms, ms237;      // its last word on the sections, its last PGN 237 (0: never)
        uint64_t sections;       // what it has open
    } mod[MACH_MODS];

    uint32_t ms229;              // last PGN 229 from the program (0: never)
    uint64_t asked229;           // its sections 1-64

    uint32_t sprayMs, sprayN;    // last sprayer PGN 160 (0: never), how many
    uint16_t pressCbar, flowCLmin;
    uint32_t totalDl;
    bool     mainOn;
} mach;

// PGN 229 from the program, from ReceiveUdp (Autosteer.ino): sections 1-64 in bytes 5-12
void machineNote229(const uint8_t* d, int len)
{
    if (len < 14 || d[4] < 8) return;
    uint64_t s = 0;
    for (uint8_t b = 0; b < 8; b++) s |= (uint64_t)d[5 + b] << (8 * b);
    mach.asked229 = s;
    const uint32_t now = millis();
    mach.ms229 = now ? now : 1;
}

// The sections the program asks for: 1-16 from PGN 254, 17-64 from a recent PGN 229
static uint64_t machineAsked(uint32_t now)
{
    uint64_t a = (uint16_t)(relay | relayHi << 8);
    if (mach.ms229 && now - mach.ms229 < MACH_229_MS) a |= mach.asked229 & ~(uint64_t)0xFFFF;
    return a;
}

// A module's sections, by its IP: from PGN 237 (all), or from the hello reply / PGN 160 (sections 1-16, only
// when the module sent no PGN 237 lately). Then the sections of all the modules heard lately put together.
static void machineSections(uint32_t ip, uint8_t pgn, uint64_t sections, uint32_t now)
{
    uint8_t m = MACH_MODS;
    for (uint8_t i = 0; i < MACH_MODS && m == MACH_MODS; i++)
        if (mach.mod[i].ip == ip) m = i;
    if (m == MACH_MODS)                                       // a new module: a free entry, or the one silent longest
    {
        m = 0;
        for (uint8_t i = 1; i < MACH_MODS; i++)
            if (!mach.mod[i].ip || (mach.mod[m].ip && now - mach.mod[i].ms > now - mach.mod[m].ms)) m = i;
        mach.mod[m].ip = ip;
        mach.mod[m].ms237 = 0;
        mach.mod[m].sections = 0;
    }
    if (pgn == MACH_PGN_LIFE) mach.mod[m].ms237 = now;
    else if (mach.mod[m].ms237 && now - mach.mod[m].ms237 < MACH_237_MS) return;
    mach.mod[m].sections = sections;
    mach.mod[m].ms = now;
    mach.secMs = now;
    mach.secPgn = pgn;

    uint64_t all = 0;
    for (uint8_t i = 0; i < MACH_MODS; i++)
        if (mach.mod[i].ip && now - mach.mod[i].ms < MACH_SILENCE_MS) all |= mach.mod[i].sections;
    mach.sections = all;
}

// Sections 1-64 as 16 hex digits, section 1 = the lowest bit ("000000000007ffff" = sections 1-19)
static void machineHex(Json& j, const char* k, uint64_t v)
{
    char t[20];
    snprintf(t, sizeof(t), "%08lx%08lx", (unsigned long)(v >> 32), (unsigned long)(v & 0xFFFFFFFF));
    j.str(k, t);
}

// Called from loop(), once Ethernet is running
void machineLoop()
{
#ifdef ARDUINO_TEENSY41
    if (!Ethernet_running) return;
    const uint32_t now = millis();
    for (uint8_t k = 0; k < MACH_PKTS_LOOP; k++)
    {
        const int len = Eth_udpModules.parsePacket();
        if (len <= 0) break;
        uint8_t d[64];
        const int got = Eth_udpModules.read(d, sizeof(d));   // the rest of a longer packet is dropped
        if (got < 5 || d[0] != 0x80 || d[1] != 0x81) continue;
        const bool fromModule = d[2] == MACH_SRC;
        const bool lifeSign   = d[2] == 0x7F && d[3] == MACH_PGN_LIFE;   // the official module's PGN 237
        if (!fromModule && !lifeSign) continue;

        const int n = got < len ? got : len;
        mach.pkts++;
        if (n < d[4] + 6) { mach.bad++; continue; }
        uint8_t sum = 0;
        for (int b = 2; b < n - 1; b++) sum += d[b];
        const bool hello = fromModule && d[3] == MACH_PGN_HELLO;
        if (sum != d[n - 1])
        {
            if (!hello) { mach.bad++; continue; }
            mach.helloNoCrc = true;                           // the official module never computes it
        }
        else if (hello) mach.helloNoCrc = false;

        const IPAddress ip = Eth_udpModules.remoteIP();
        for (uint8_t b = 0; b < 4; b++) mach.ip[b] = ip[b];
        mach.lastMs = now ? now : 1;
        mach.lastPgn = d[3];

        const uint32_t from = (uint32_t)ip[0] << 24 | (uint32_t)ip[1] << 16 | (uint32_t)ip[2] << 8 | ip[3];
        if (hello && d[4] >= 2)
        {
            machineSections(from, MACH_PGN_HELLO, (uint16_t)(d[5] | d[6] << 8), mach.lastMs);
            mach.helloMs  = mach.lastMs;
            mach.helloN++;
        }
        else if (fromModule && d[3] == MACH_PGN_SPRAYER && d[4] >= 10)
        {
            mach.pressCbar = (uint16_t)(d[5] | d[6] << 8);
            mach.flowCLmin = (uint16_t)(d[7] | d[8] << 8);
            mach.totalDl   = (uint32_t)d[9] | (uint32_t)d[10] << 8 | (uint32_t)d[11] << 16 | (uint32_t)d[12] << 24;
            mach.mainOn    = d[13] != 0;
            machineSections(from, MACH_PGN_SPRAYER, d[4] >= 11 ? (uint16_t)(d[14] | d[15] << 8) : d[14], mach.lastMs);
            mach.sprayMs   = mach.lastMs;
            mach.sprayN++;
        }
        else if (fromModule && d[3] == MACH_PGN_LIFE && d[4] >= 2)
        {
            uint64_t s = (uint16_t)(d[5] | d[6] << 8);
            if (d[4] >= 14)
                for (uint8_t b = 0; b < 6; b++) s |= (uint64_t)d[13 + b] << (16 + 8 * b);
            machineSections(from, MACH_PGN_LIFE, s, mach.lastMs);
        }
    }

    // the highest section used since the start: the page keeps that many squares while all are off
    const uint64_t used = machineAsked(now) | mach.sections;
    if (mach.secTop < 64 && used >> mach.secTop) mach.secTop = 64 - __builtin_clzll(used);

    const bool here = mach.lastMs != 0 && now - mach.lastMs < MACH_SILENCE_MS;
    if (here != mach.here)
    {
        mach.here = here;
        if (here) StatusLog.line("[machine] machine module %u.%u.%u.%u heard on UDP 9999 (web page: Machine tab)",
                                 mach.ip[0], mach.ip[1], mach.ip[2], mach.ip[3]);
        else      StatusLog.line("[machine] machine module silent for %u s", (unsigned)(MACH_SILENCE_MS / 1000));
    }
#endif
}

// The "mach" object of /api/status (zWeb.ino): only once the module has been heard, so the page
// shows the Machine tab only then
void machineJson(Json& j, uint32_t now)
{
    if (!mach.lastMs) return;
    j.obj("mach");
    // the IPs of the modules whose sections are shown (with the last word on the sections), else the last one heard
    char ip[MACH_MODS * 17 + 4] = "";
    for (uint8_t i = 0; i < MACH_MODS; i++)
    {
        const uint32_t a = mach.mod[i].ip;
        if (!a || mach.secMs - mach.mod[i].ms >= MACH_SILENCE_MS) continue;
        const size_t n = strlen(ip);
        snprintf(ip + n, sizeof(ip) - n, "%s%lu.%lu.%lu.%lu", n ? ", " : "",
                 (unsigned long)(a >> 24), (unsigned long)(a >> 16 & 255), (unsigned long)(a >> 8 & 255), (unsigned long)(a & 255));
    }
    if (!ip[0]) snprintf(ip, sizeof(ip), "%u.%u.%u.%u", mach.ip[0], mach.ip[1], mach.ip[2], mach.ip[3]);
    j.str("ip", ip);
    j.bo("here", now - mach.lastMs < MACH_SILENCE_MS);
    j.i("ago", (long)((now - mach.lastMs) / 1000));
    j.u("pkts", mach.pkts);
    j.u("bad", mach.bad);
    j.i("lastPgn", mach.lastPgn);
    machineHex(j, "cmd", machineAsked(now));          // the sections the program asks for (PGN 254 and 229)
    j.i("helloAgo", mach.helloMs ? (long)((now - mach.helloMs) / 1000) : -1);
    j.u("helloN", mach.helloN);
    j.bo("helloNoCrc", mach.helloNoCrc);
    j.i("secAgo", mach.secMs ? (long)((now - mach.secMs) / 1000) : -1);
    machineHex(j, "sec", mach.sections);
    j.u("secN", mach.secTop);
    j.u("secPgn", mach.secPgn);
    j.i("sprayAgo", mach.sprayMs ? (long)((now - mach.sprayMs) / 1000) : -1);
    if (mach.sprayMs)
    {
        j.u("sprayN", mach.sprayN);
        j.num("bar", mach.pressCbar / 100.0, 2);
        j.num("lmin", mach.flowCLmin / 100.0, 2);
        j.num("total", mach.totalDl / 10.0, 1);
        j.bo("main", mach.mainOn);
    }
    j.close();
}
