/* zPgn.ino - the UDP messages (PGNs) to and from AgOpenWeb / AgOpenGPS.
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr. Made from Sprayer_ESP32 v0.8.
   Message layouts after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3) and Docs/PGN.md of AgOpenWeb.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Received on port 8888, source 0x7F (the program):
//   239 machine data (sections 1-16), 229 sections 1-64 (bytes 5-12, 8 per byte, section 1 = byte 5 bit 0;
//   sent with every 239 when the tool has more than 16 sections), 238 machine config, 236 output functions,
//   201 subnet, 200 hello, 202 scan
// Sent to <subnet>.255 port 9999, source 0x7B (machine module), checksum = sum of bytes 2..n-2:
//   123 hello reply (sections 1-16 open, as in the official layout), 203 scan reply (to 255.255.255.255),
//   237 every 200 ms, as the AgOpenGPS AiO v26 sends it, with the sections above 16 after it:
//       80 81 7B ED 0E | sections 1-16 open (2) | lift (0 off, 1 down, 2 up) | tramline | geo stop | status |
//                        0 | 0 | sections 17-64 open (6) | CRC
//   (the official module sends it with 8 bytes of fixed content; the AiO Teensy's Machine tab reads all the
//   sections from it). No PGN 160: the seed drill has no sensors.
//   221 hardware message, the text the program shows on its screen for a few seconds (AgOpenGPS layout, as the
//       AiO Teensy sends it): seconds to show, colour (0 warning, 1 normal), text of 60 bytes at most. Sent when a
//       section has no relay (noRelayTick).

static uint8_t rx[300];

void udpStart()
{
    udp.begin(LISTEN_PORT);
}

static void sendPgn(uint8_t* msg, size_t length, IPAddress to)
{
    if (!netUp()) return;                                   // nobody to talk to yet (no tractor Wi-Fi, no cable)
    msg[length - 1] = pgnChecksum(msg, length);
    udp.beginPacket(viaGateway ? gatewayIp() : to, SEND_PORT);     // through the XIAO: to it only ("xiao-network" in the main file)
    udp.write(msg, length);
    udp.endPacket();
}

static IPAddress broadcastIp()
{
    return IPAddress(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 255);
}

// ==== shared "no-relay": the same in Sprayer_ESP32 and Seeder_ESP32 (change both; check_shared_blocks.py compares them)
// PGN 221: { 0x80, 0x81, 0x7B, 221, n + 2, seconds to show, colour (0 = warning), text of n bytes (60 at most), CRC }
static void sendHardwareMessage(const char* text, uint8_t seconds, bool warning)
{
    uint8_t msg[8 + 60] = { 0x80, 0x81, 0x7B, HARDWARE_MESSAGE_PGN };
    size_t n = strlen(text);
    if (n > 60) n = 60;
    msg[4] = (uint8_t)(n + 2);
    msg[5] = seconds;
    msg[6] = warning ? 0 : 1;
    memcpy(msg + 7, text, n);
    sendPgn(msg, 8 + n, broadcastIp());
}

// Sections as a short list, section 1 = bit 0: "7-8", "2, 9-12"
static void sectionList(uint64_t mask, char* out, size_t size)
{
    out[0] = '\0';
    for (uint8_t i = 0; i < 64; i++)
    {
        if (!((mask >> i) & 1)) continue;
        uint8_t j = i;
        while (j < 63 && ((mask >> (j + 1)) & 1)) j++;
        char v[12];
        if (j == i) snprintf(v, sizeof(v), "%s%u", out[0] ? ", " : "", (unsigned)(i + 1));
        else        snprintf(v, sizeof(v), "%s%u-%u", out[0] ? ", " : "", (unsigned)(i + 1), (unsigned)(j + 1));
        strlcat(out, v, size);
        i = j;
    }
}

// Once a second: the sections this module should drive that no relay follows (noRelaySections(), each module's own
// rule) go in noRelay for the page. The log says it when they change; the program's screen (PGN 221) when they
// change, when the program comes back, and every NO_RELAY_REPEAT_S while the program asks for one of them.
void noRelayTick()
{
    static uint64_t told = 0;
    static uint32_t toldMs = 0;
    static bool wasLink = false;
    const uint32_t now = millis();
    const uint64_t miss = noRelaySections();
    const bool changed = miss != told, back = linkOk && !wasLink, one = __builtin_popcountll(miss) == 1;
    wasLink = linkOk;
    told = miss;
    noRelay = miss;
    char list[48], text[61];
    sectionList(miss, list, sizeof(list));
    if (changed)
    {
        if (miss) logEvent("%s %s: no relay follows %s (%s)", one ? "section" : "sections", list, one ? "it" : "them", noRelayWhere());
        else logEvent("every section has a relay again");
    }
    if (!linkOk) return;
    if (miss && (changed || back || ((sectionsReq & miss) && now - toldMs >= NO_RELAY_REPEAT_S * 1000UL)))
    {
        snprintf(text, sizeof(text), "%s: %s %s %s no relay (%s)", SCREEN_NAME, one ? "section" : "sections", list,
                 one ? "has" : "have", noRelayWhere());
        sendHardwareMessage(text, 10, true);
        toldMs = now;
    }
    else if (!miss && changed)
    {
        snprintf(text, sizeof(text), "%s: every section has a relay again", SCREEN_NAME);
        sendHardwareMessage(text, 3, false);
    }
}
// ==== end shared "no-relay"

// ==== shared "invert-warning": the same in Sprayer_ESP32 and Seeder_ESP32 (change both; check_shared_blocks.py compares them)
// Once a second: the outputs inverted ("relay active high" ticked in the program, PGN 238) on a board whose relays close
// one way only (POLARITY_FIXED): every relay closes with nothing asked. Said on the program's screen when it comes on,
// when the program comes back, and every INVERT_REPEAT_S while it lasts (the log already says INVERTED).
void invertTick()
{
#if POLARITY_FIXED
    static bool     told = false, wasLink = false;
    static uint32_t toldMs = 0;
    const uint32_t now = millis();
    const bool back = linkOk && !wasLink;
    wasLink = linkOk;
    if (!cfg.invertOutputs)
    {
        if (told && linkOk) sendHardwareMessage(SCREEN_NAME ": outputs normal again", 3, false);
        told = false;
        return;
    }
    if (!linkOk || (told && !back && now - toldMs < INVERT_REPEAT_S * 1000UL)) return;
    sendHardwareMessage(SCREEN_NAME ": outputs inverted, untick 'relay active high'", INVERT_REPEAT_S + 2, true);
    told = true;
    toldMs = now;
#endif
}
// ==== end shared "invert-warning"

// Every 200 ms, PGN 237 "from machine", in the layout of the AgOpenGPS AiO v26 firmware (MachineProcessor):
// what the outputs really do (sections, lift, tramline, geo stop; all off while the program is silent) and
// a status byte, bit 0 = the program's PGN 239 still arrives. The official Machine_UDP_v5 sends dummy content.
// Bytes 13-18: the open sections above 16 (a reader of the 8-byte layout stops before them).
void sendSignOfLife()
{
    const uint64_t s = sectionsOn;
    const uint8_t lift = linkOk && isLower ? 1 : linkOk && isRaise ? 2 : 0;
    uint8_t msg[] = { 0x80, 0x81, 0x7B, 237, 14,
                      (uint8_t)s, (uint8_t)(s >> 8), lift, (uint8_t)(linkOk ? tramline & 3 : 0),
                      (uint8_t)(linkOk && geoStop ? 1 : 0), (uint8_t)(linkOk ? 1 : 0), 0, 0,
                      (uint8_t)(s >> 16), (uint8_t)(s >> 24), (uint8_t)(s >> 32), (uint8_t)(s >> 40),
                      (uint8_t)(s >> 48), (uint8_t)(s >> 56),
                      0 };
    sendPgn(msg, sizeof(msg), broadcastIp());
}

// ==== shared "pgn-common-cases": the same in Sprayer_ESP32 and Seeder_ESP32 (change both; check_shared_blocks.py compares them)
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
// ==== end shared "pgn-common-cases"

    case 236:                                               // output functions, 24 entries
        if (dataLength < 24) return;
        for (uint8_t i = 0; i < 24; i++) cfg.pin[i] = d[5 + i] <= PIN_GEO_STOP ? d[5 + i] : 0;
        settingsSave();
        logEvent("output functions from the program: relays %u %u %u %u %u %u %u %u%s",
                 cfg.pin[0], cfg.pin[1], cfg.pin[2], cfg.pin[3], cfg.pin[4], cfg.pin[5], cfg.pin[6], cfg.pin[7],
                 cfg.firstSection ? " (not used: the relays follow the sections in a row)" : "");
        break;

    // ==== shared "pgn-subnet-receive": the same in Sprayer_ESP32 and Seeder_ESP32 (change both; check_shared_blocks.py compares them)
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
    if (viaGateway && millis() - viaGatewayMs >= GATEWAY_HOLD_MS)
    {
        viaGateway = false;
        logEvent("program packets no longer from %s: answering everyone again", gatewayIp().toString().c_str());
    }
    for (int guard = 0; guard < 10 && udp.parsePacket() > 0; guard++)
    {
        const int len = udp.read(rx, sizeof(rx));
        if (len < 6 || rx[0] != 0x80 || rx[1] != 0x81) continue;    // not an AgOpenGPS message
        if (rx[2] != 0x7F) continue;                                 // only the program talks to a machine module
        const uint8_t pgn = rx[3], dataLength = rx[4];
        if (len < dataLength + 6) continue;                          // cut short
        if (pgnChecksum(rx, dataLength + 6) != rx[dataLength + 5]) { countCrcErr++; continue; }
        const IPAddress from = udp.remoteIP();
        if (staUp() && from == gatewayIp())                          // the XIAO's copy (the "xiao-network" notes in the main file)
        {
            viaGatewayMs = millis();
            if (!viaGateway)
            {
                viaGateway = true;
                logEvent("the program's packets come from %s (XIAO WiFi bridge): answering it only", from.toString().c_str());
            }
        }
        else if (viaGateway) { countBeside++; continue; }            // the same packet again, late: the copy was here first
        handlePgn(pgn, rx, dataLength, from);
    }
}
    // ==== end shared "pgn-subnet-receive"
