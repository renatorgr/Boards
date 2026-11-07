/* zPgn.ino - the UDP messages (PGNs) to and from AgOpenWeb / AgOpenGPS.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Message layouts after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3) and Docs/PGN.md of AgOpenWeb.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Received on port 8888, source 0x7F (the program):
//   239 machine data (sections 1-16), 229 sections 1-64 (bytes 5-12, 8 per byte, section 1 = byte 5 bit 0;
//   sent with every 239 when the tool has more than 16 sections), 238 machine config, 236 output functions,
//   235 section dimensions (widths of sections 1-16 in cm, uint16, bytes 5-36; number of sections in byte 37;
//   sent at the program's start and when the tool's sections change), 201 subnet, 200 hello, 202 scan
// Sent to <subnet>.255 port 9999, source 0x7B (machine module), checksum = sum of bytes 2..n-2:
//   123 hello reply (sections 1-16 open), 203 scan reply (to 255.255.255.255),
//   237 every 200 ms, as the AgOpenGPS AiO v26 sends it:
//       80 81 7B ED 08 | sections 1-16 open (2) | lift (0 off, 1 down, 2 up) | tramline | geo stop | status | 0 | 0 | CRC
//   160 sprayer readings once a second (sections 1-8 in byte 14, 9-16 in byte 15), only when the machine has
//       a pressure sensor or a flow meter (page setting)
//   161 ISOBUS process data once a second, right after the 160 (and only with it): 8 bytes per value, as the
//       ISO 11783-10 Process Data Message (the "ISOBUS Data Everywhere" proposal of the AgOpenGPS ISOBUS group):
//       byte 0 = command 3 ("value") in bits 0-3 + element number bits 0-3 in bits 4-7, byte 1 = element number
//       bits 4-11, bytes 2-3 = DDI, bytes 4-7 = value, int32; all little-endian. Element 0 = the sprayer, 1 = its boom.
//       A value that is not known (no sensor, stopped, no target) is left out of that message.
//         DDI   194 actual pressure, 0.1 Pa (bar x 1 000 000)       element 1, pressure sensor fitted
//               37 actual flow, mm3/s (L/min x 16 667)                element 1, flow meter fitted
//              351 total volume, ml                                   element 0, flow meter fitted
//              141 actual work state: master switch, 1 on, 0 off      element 1
//              161 actual condensed work state 1-16: 2 bits per section (01 open, 00 closed, 11 not there)  element 1
//                2 actual rate, 0.01 mm3/m2 (L/ha x 10 000)           element 1, while moving with the widths known
//                1 target rate, same unit                             element 1, when a target is set
//              193 the pressure that gives the target now, 0.1 Pa     element 1
//              195 minimum pressure, 196 maximum pressure, 0.1 Pa     element 1, when set
//            57344 (proprietary range) sprayer alarms: the ALARM_* bits, always sent  element 1
//   221 hardware message, the text the program shows on its screen for a few seconds (AgOpenGPS layout, as the
//       AiO Teensy sends it): seconds to show, colour (0 warning, 1 normal), text of 60 bytes at most. Sent when an
//       alarm comes on or goes off, and every ALARM_REPEAT_S while one is on. The official AgOpenWeb reads it (161 only
//       the fork).

static uint8_t rx[300];

void udpStart()
{
    udp.begin(LISTEN_PORT);
}

static void sendPgn(uint8_t* msg, size_t length, IPAddress to)
{
    if (!staUp()) return;                                   // nobody to talk to yet
    msg[length - 1] = pgnChecksum(msg, length);
    udp.beginPacket(to, SEND_PORT);
    udp.write(msg, length);
    udp.endPacket();
}

static IPAddress broadcastIp()
{
    return IPAddress(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 255);
}

// Every 200 ms, PGN 237 "from machine", in the layout of the AgOpenGPS AiO v26 firmware (MachineProcessor):
// what the outputs really do (sections, lift, tramline, geo stop; all off while the program is silent) and
// a status byte, bit 0 = the program's PGN 239 still arrives. The official Machine_UDP_v5 sends dummy content.
void sendSignOfLife()
{
    const uint8_t lift = linkOk && isLower ? 1 : linkOk && isRaise ? 2 : 0;
    uint8_t msg[] = { 0x80, 0x81, 0x7B, 237, 8,
                      lowByte(sectionsOn), highByte(sectionsOn), lift, (uint8_t)(linkOk ? tramline & 3 : 0),
                      (uint8_t)(linkOk && geoStop ? 1 : 0), (uint8_t)(linkOk ? 1 : 0), 0, 0, 0 };
    sendPgn(msg, sizeof(msg), broadcastIp());
}

// Once a second, PGN 160: the sprayer readings for the AgOpenWeb fork's sprayer strip. Sections 1-8 in
// byte 14 as the fork reads them; sections 9-16 in byte 15 (the fork and the Teensy accept the longer
// message and still read byte 14). With no sensor fitted there is nothing to report: no PGN 160,
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

// One process data value (8 bytes) into msg at n, n moved on
static void pdValue(uint8_t* msg, size_t& n, uint16_t element, uint16_t ddi, int32_t value)
{
    msg[n++] = 3 | (uint8_t)((element & 0x0F) << 4);        // command 3 = value
    msg[n++] = (uint8_t)(element >> 4);
    msg[n++] = lowByte(ddi);
    msg[n++] = highByte(ddi);
    for (uint8_t b = 0; b < 4; b++) msg[n++] = (uint8_t)((uint32_t)value >> (8 * b));
}

static int32_t barToPd(float bar) { return (int32_t)lroundf(bar * 1000000.0f); }       // 0.1 Pa
static int32_t lhaToPd(float lha) { return (int32_t)lroundf(lha * 10000.0f); }         // 0.01 mm3/m2

// The sections the boom has: those of the program's tool (PGN 235), else the highest section a relay follows
static uint8_t boomSections()
{
    if (cfg.toolSections) return cfg.toolSections > MAX_TOOL_SECTIONS ? MAX_TOOL_SECTIONS : cfg.toolSections;
    uint8_t top = 0;
    for (uint8_t i = 0; i < NUM_RELAYS; i++) { const uint8_t f = relayFn(i); if (f >= 1 && f <= MAX_TOOL_SECTIONS && f > top) top = f; }
    return top;
}

// Once a second, PGN 161: the sprayer's values as ISOBUS process data (layout above). Sent with the 160.
void sendProcessData()
{
    if (!cfg.sensors) return;
    uint8_t msg[5 + 8 * 13 + 1] = { 0x80, 0x81, 0x7B, PROCESS_DATA_PGN, 0 };
    size_t n = 5;
    if (cfg.sensors & SENSOR_PRESSURE) pdValue(msg, n, 1, 194, barToPd(pressureBar));
    if (cfg.sensors & SENSOR_FLOW)
    {
        pdValue(msg, n, 1, 37, (int32_t)lroundf(flowLpm * 1000000.0f / 60.0f));
        pdValue(msg, n, 0, 351, (int32_t)lroundf(totalLitres * 1000.0f));
    }
    pdValue(msg, n, 1, 141, masterOn ? 1 : 0);
    uint32_t condensed = 0;
    const uint8_t boom = boomSections();
    for (uint8_t i = 0; i < 16; i++)
        condensed |= (uint32_t)(i >= boom ? 3 : ((sectionsOn >> i) & 1) ? 1 : 0) << (2 * i);
    pdValue(msg, n, 1, 161, (int32_t)condensed);
    if (rateLha >= 0) pdValue(msg, n, 1, 2, lhaToPd(rateLha));
    if (cfg.targetRate > 0) pdValue(msg, n, 1, 1, lhaToPd(cfg.targetRate));
    if (needBar > 0) pdValue(msg, n, 1, 193, barToPd(needBar));
    if (cfg.minBar > 0) pdValue(msg, n, 1, 195, barToPd(cfg.minBar));
    if (cfg.maxBar > 0) pdValue(msg, n, 1, 196, barToPd(cfg.maxBar));
    pdValue(msg, n, 1, 57344, sprayAlarms);
    msg[4] = n - 5;
    sendPgn(msg, n + 1, broadcastIp());
}

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

// Once a second, after the alarms of rateTick(): the alarms as a PGN 221 message when they change (one normal-colour
// note when the last one goes off), and again every ALARM_REPEAT_S while one is on, so the program's screen keeps it.
void sendAlarmMessage()
{
    static uint8_t  lastSent = 0;                           // the alarms the last message told
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if (sprayAlarms == lastSent && (!sprayAlarms || now - lastMs < ALARM_REPEAT_S * 1000UL)) return;
    if (!sprayAlarms) sendHardwareMessage("Sprayer: alarms over", 3, false);
    else
    {
        char text[61] = "Sprayer:", v[32];
        if (sprayAlarms & ALARM_PRESSURE_LOW)  { snprintf(v, sizeof(v), " low pressure %.1f bar,", pressureBar); strlcat(text, v, sizeof(text)); }
        if (sprayAlarms & ALARM_PRESSURE_HIGH) { snprintf(v, sizeof(v), " high pressure %.1f bar,", pressureBar); strlcat(text, v, sizeof(text)); }
        if (sprayAlarms & ALARM_FLOW_LOW)      strlcat(text, " flow too low,", sizeof(text));
        if (sprayAlarms & ALARM_FLOW_HIGH)     strlcat(text, " flow too high,", sizeof(text));
        if (sprayAlarms & ALARM_FLOW_CLOSED)   strlcat(text, " flow with the sections off,", sizeof(text));
        const size_t n = strlen(text);
        if (n && text[n - 1] == ',') text[n - 1] = '\0';
        sendHardwareMessage(text, ALARM_REPEAT_S + 2, true);
    }
    lastSent = sprayAlarms;
    lastMs = now;
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
        logEvent("output functions from the program: relays %u %u %u %u %u %u %u %u",
                 cfg.pin[0], cfg.pin[1], cfg.pin[2], cfg.pin[3], cfg.pin[4], cfg.pin[5], cfg.pin[6], cfg.pin[7]);
        break;

    case 235:                                               // section dimensions: the widths for the L/ha
    {
        if (dataLength < 33) return;
        uint16_t cm[MAX_TOOL_SECTIONS];
        bool same = d[37] == cfg.toolSections;
        for (uint8_t i = 0; i < MAX_TOOL_SECTIONS; i++)
        {
            cm[i] = d[5 + 2 * i] | (uint16_t)d[6 + 2 * i] << 8;
            if (cm[i] != cfg.sectionCm[i]) same = false;
        }
        count235++;
        if (same) break;                                    // nothing new: the flash is left alone
        memcpy(cfg.sectionCm, cm, sizeof(cm));
        cfg.toolSections = d[37];
        settingsSave();
        logEvent("sections from the program: %u, %u %u %u %u %u %u %u %u cm", cfg.toolSections,
                 cm[0], cm[1], cm[2], cm[3], cm[4], cm[5], cm[6], cm[7]);
        break;
    }

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
