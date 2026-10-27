/* zPgn.ino - the UDP messages (PGNs) to and from AgOpenWeb / AgOpenGPS.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Message layouts after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3) and Docs/PGN.md of AgOpenWeb.
   2026-10-27 (Claude): no PGN 160 from a machine without sensors (a seed drill).
   2026-10-27 (Claude): PGN 229 read in full (sections 1-64); messages also go out over the network cable.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Received on port 8888, source 0x7F (the program):
//   239 machine data (sections 1-16), 229 sections 1-64 (bytes 5-12, 8 per byte, section 1 = byte 5 bit 0;
//   sent with every 239 when the tool has more than 16 sections), 238 machine config, 236 output functions,
//   201 subnet, 200 hello, 202 scan
// Sent to <subnet>.255 port 9999, source 0x7B (machine module), checksum = sum of bytes 2..n-2:
//   123 hello reply, 203 scan reply (to 255.255.255.255), 237 sign of life every 200 ms (these carry
//   sections 1-16 and relays 1-16, as in the official layouts),
//   160 sprayer readings once a second (sections 1-8 in byte 14, 9-16 in byte 15), only when the machine has
//       a pressure sensor or a flow meter (page setting)

static uint8_t rx[300];

void udpStart()
{
    udp.begin(LISTEN_PORT);
}

static void sendPgn(uint8_t* msg, size_t length, IPAddress to)
{
    if (!netUp()) return;                                   // nobody to talk to yet (no tractor Wi-Fi, no cable)
    msg[length - 1] = pgnChecksum(msg, length);
    udp.beginPacket(to, SEND_PORT);
    udp.write(msg, length);
    udp.endPacket();
}

static IPAddress broadcastIp()
{
    return IPAddress(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 255);
}

// Every 200 ms, PGN 237 "from machine": the official module sends it as a sign of life (with
// dummy content); here it carries what the relays are doing
void sendSignOfLife()
{
    uint8_t msg[] = { 0x80, 0x81, 0x7B, 237, 8,
                      lowByte(sectionsOn), highByte(sectionsOn), lowByte(relayBits), highByte(relayBits),
                      masterOn, 0, 0, 0, 0 };
    sendPgn(msg, sizeof(msg), broadcastIp());
}

// Once a second, PGN 160: the sprayer readings for the AgOpenWeb fork's sprayer strip. Sections 1-8 in
// byte 14 as the fork reads them; sections 9-16 in byte 15 (the fork and the Teensy accept the longer
// message and still read byte 14). A machine without sensors (a seed drill) has nothing to report: no PGN 160,
// so the program and the AiO page show no sprayer readings; its sections still go out in the hello reply
void sendReadings()
{
    if (!cfg.sensors) return;
    const uint16_t p = pressureBar * 100;                   // hundredths of a bar
    const uint16_t f = flowLpm * 100;                       // hundredths of a L/min
    const uint32_t t = totalLitres * 10;                    // tenths of a litre
    uint8_t msg[] = { 0x80, 0x81, 0x7B, SPRAYER_PGN, 11,
                      lowByte(p), highByte(p),
                      lowByte(f), highByte(f),
                      (uint8_t)t, (uint8_t)(t >> 8), (uint8_t)(t >> 16), (uint8_t)(t >> 24),
                      masterOn, lowByte(sectionsOn), highByte(sectionsOn),
                      0 };
    sendPgn(msg, sizeof(msg), broadcastIp());
}

static void handlePgn(uint8_t pgn, const uint8_t* d, uint8_t dataLength, IPAddress from)
{
    switch (pgn)
    {
    case 239:                                               // machine data, 10 times a second
        if (dataLength < 8) return;
        uTurn    = d[5];
        gpsKmh   = d[6] / 10.0f;                            // km/h x 10 in one byte
        hydLift  = d[7];                                    // 1 lower, 2 raise
        tramline = d[8];                                    // bit 0 right, bit 1 left
        geoStop  = d[9];
        sections239 = d[11] | (uint16_t)d[12] << 8;         // sections 1-8, 9-16
        last239Ms = millis();
        have239 = true;
        count239++;
        applyOutputs();                                     // no need to wait for the next 50 ms
        break;

    case 229:                                               // sections 1-64, with every 239 on a tool of 17+ sections
    {
        if (dataLength < 8) return;
        uint64_t s = 0;
        for (uint8_t b = 0; b < 8; b++) s |= (uint64_t)d[5 + b] << (8 * b);
        sections229 = s;
        sections239 = (uint16_t)s;                          // the same sections 1-16 as in the 239
        last229Ms = last239Ms = millis();
        have229 = have239 = true;
        count229++;
        applyOutputs();
        break;
    }

    case 200:                                               // hello from the program: answer as a machine module
    {
        lastHelloMs = millis();
        haveHello = true;
        countHello++;
        uint8_t reply[] = { 0x80, 0x81, 0x7B, 123, 5, lowByte(sectionsOn), highByte(sectionsOn), 0, 0, 0, 0 };
        sendPgn(reply, sizeof(reply), broadcastIp());
        break;
    }

    case 238:                                               // machine config, at start-up and on profile load
        if (dataLength < 8) return;
        cfg.raiseTime     = d[5];
        cfg.lowerTime     = d[6];
        cfg.hydEnable     = d[7];
        cfg.invertOutputs = bitRead(d[8], 0);
        cfg.user1 = d[9];
        cfg.user2 = d[10];
        cfg.user3 = d[11];
        cfg.user4 = d[12];
        settingsSave();
        logEvent("machine config from the program: lift %s, raise %u s, lower %u s, outputs %s",
                 cfg.hydEnable ? "on" : "off", cfg.raiseTime, cfg.lowerTime, cfg.invertOutputs ? "INVERTED (relay active high ticked)" : "normal");
        break;

    case 236:                                               // output functions, 24 entries
        if (dataLength < 24) return;
        for (uint8_t i = 0; i < 24; i++) cfg.pin[i] = d[5 + i] <= PIN_GEO_STOP ? d[5 + i] : 0;
        settingsSave();
        logEvent("output functions from the program: relays %u %u %u %u %u %u %u %u%s",
                 cfg.pin[0], cfg.pin[1], cfg.pin[2], cfg.pin[3], cfg.pin[4], cfg.pin[5], cfg.pin[6], cfg.pin[7],
                 cfg.firstSection ? " (not used: the relays follow the sections in a row)" : "");
        break;

    case 201:                                               // subnet: save and restart with the new IP
        if (dataLength == 5 && d[5] == 201 && d[6] == 201)
        {
            cfg.subnet[0] = d[7];
            cfg.subnet[1] = d[8];
            cfg.subnet[2] = d[9];
            settingsSave();
            logEvent("subnet from the program: %u.%u.%u, restarting", d[7], d[8], d[9]);
            restartAtMs = millis() + 300;
        }
        break;

    case 202:                                               // scan: tell the program who and where we are
        if (dataLength == 3 && d[5] == 202 && d[6] == 202)
        {
            uint8_t reply[] = { 0x80, 0x81, 0x7B, 203, 7,
                                cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], cfg.ipLast,
                                from[0], from[1], from[2], 0 };
            sendPgn(reply, sizeof(reply), IPAddress(255, 255, 255, 255));
        }
        break;

    default:
        countOther++;
        break;
    }
}

void udpPoll()
{
    for (int guard = 0; guard < 10 && udp.parsePacket() > 0; guard++)
    {
        const int len = udp.read(rx, sizeof(rx));
        if (len < 6 || rx[0] != 0x80 || rx[1] != 0x81) continue;    // not an AgOpenGPS message
        if (rx[2] != 0x7F) continue;                                 // only the program talks to a machine module
        const uint8_t pgn = rx[3], dataLength = rx[4];
        if (len < dataLength + 6) continue;                          // cut short
        if (pgnChecksum(rx, dataLength + 6) != rx[dataLength + 5]) { countCrcErr++; continue; }
        handlePgn(pgn, rx, dataLength, udp.remoteIP());
    }
}
