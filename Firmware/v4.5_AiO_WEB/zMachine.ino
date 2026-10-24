/* zMachine.ino - the machine module (IP .123) as seen from the network, for the web page's Machine tab.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zMachine.ino
// The machine module (source 0x7B = 123, e.g. the sprayer Arduino Nano "Pulverizador_Nano") answers the
// program's hello and sends its own PGNs to UDP 9999, broadcast to the whole 192.168.5.x network. The
// Teensy listens on 9999 too (Eth_udpModules, zEthernet.ino) and keeps only what comes from the machine
// module; everything else on 9999 (the other modules, the Teensy's own sentences) is read and dropped.
// Nothing is sent and nothing changes in the steering: it is only for the web page (Machine tab, shown
// once the module has been heard) and the [machine] messages.
//
// Sprayer PGN 160 (0xA0), once a second, 16 bytes:
//   80 81 7B A0 0A | pressure (cbar, 2) | flow (cL/min, 2) | total (dL, 4) | main switch (1) | sections (1) | CRC
//   (little-endian; CRC = sum of the bytes from the 3rd to the one before the CRC, as in AgOpenGPS)

#define MACH_SRC          0x7B   // the machine module's source byte (IP .123)
#define MACH_PGN_SPRAYER  0xA0   // 160
#define MACH_PKTS_LOOP    16     // packets read from UDP 9999 per loop at most
#define MACH_SILENCE_MS   5000   // silent after this long (as STATUS_SILENCE_MS, which zStatus.ino defines after this file)

static struct
{
    uint32_t lastMs;             // last packet of any kind from the machine module (0: never)
    uint32_t pkts, bad;          // its packets, those with a wrong checksum
    int16_t  lastPgn;
    uint8_t  ip[4];
    bool     here;               // heard in the last MACH_SILENCE_MS (for the [machine] lines)

    uint32_t sprayMs, sprayN;    // last sprayer PGN 160 (0: never), how many
    uint16_t pressCbar, flowCLmin;
    uint32_t totalDl;
    bool     mainOn;
    uint8_t  sections;           // bit 0 = section 1, as the Nano has its relays
} mach;

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
        if (got < 5 || d[0] != 0x80 || d[1] != 0x81 || d[2] != MACH_SRC) continue;

        const int n = got < len ? got : len;
        uint8_t sum = 0;
        for (int b = 2; b < n - 1; b++) sum += d[b];
        mach.pkts++;
        if (n < d[4] + 6 || sum != d[n - 1]) { mach.bad++; continue; }

        const IPAddress ip = Eth_udpModules.remoteIP();
        for (uint8_t b = 0; b < 4; b++) mach.ip[b] = ip[b];
        mach.lastMs = now ? now : 1;
        mach.lastPgn = d[3];

        if (d[3] == MACH_PGN_SPRAYER && d[4] >= 10)
        {
            mach.pressCbar = (uint16_t)(d[5] | d[6] << 8);
            mach.flowCLmin = (uint16_t)(d[7] | d[8] << 8);
            mach.totalDl   = (uint32_t)d[9] | (uint32_t)d[10] << 8 | (uint32_t)d[11] << 16 | (uint32_t)d[12] << 24;
            mach.mainOn    = d[13] != 0;
            mach.sections  = d[14];
            mach.sprayMs   = mach.lastMs;
            mach.sprayN++;
        }
    }

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
    char ip[20];
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u", mach.ip[0], mach.ip[1], mach.ip[2], mach.ip[3]);
    j.str("ip", ip);
    j.bo("here", now - mach.lastMs < MACH_SILENCE_MS);
    j.i("ago", (long)((now - mach.lastMs) / 1000));
    j.u("pkts", mach.pkts);
    j.u("bad", mach.bad);
    j.i("lastPgn", mach.lastPgn);
    j.u("cmd", relay);                              // the sections the program asks for (PGN 254, Autosteer.ino)
    j.i("sprayAgo", mach.sprayMs ? (long)((now - mach.sprayMs) / 1000) : -1);
    if (mach.sprayMs)
    {
        j.u("sprayN", mach.sprayN);
        j.num("bar", mach.pressCbar / 100.0, 2);
        j.num("lmin", mach.flowCLmin / 100.0, 2);
        j.num("total", mach.totalDl / 10.0, 1);
        j.bo("main", mach.mainOn);
        j.u("sec", mach.sections);
    }
    j.close();
}
