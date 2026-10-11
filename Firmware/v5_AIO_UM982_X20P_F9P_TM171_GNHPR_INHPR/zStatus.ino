// zStatus.ino
// Short status messages on the USB serial monitor, printed ONLY when something changes
// (so it can stay on in the tractor). Prefix "[status]".
//   - Ethernet cable / link and the module's IP address
//   - AgOpenWeb talking to the module (first packet, and silence)
//   - RTCM corrections arriving from AgOpenWeb (NTRIP)
//   - GPS: first position, fix quality changes, no GGA / VTG from the receiver
//   - Dual antenna heading (UM982 HPR) available or lost; single-antenna receiver (no HPR)
//     Without dual heading: WHY (UM982 HPR QF 0 / HPR not arriving with its GGA / empty
//     heading) and the last HPR line exactly as the receiver sent it
//   - Receiver sentences with a bad checksum; Teensy reading the receiver too slowly
//   - TM171 data arriving or lost
// A state must hold for STATUS_STABLE_MS before it is reported, so a fix or dual state
// that flickers at 10 Hz does not flood the monitor.

#define STATUS_MESSAGES     1       // 0 = no [status] lines
#define STATUS_STABLE_MS    1000    // a new state is reported after it has held this long
#define STATUS_SILENCE_MS   5000    // AgOpenWeb / RTCM / GGA considered gone after this long

static uint32_t aogLastMs  = 0;     // last UDP packet from AgOpenWeb (port 8888)
static uint8_t  aogIp[4]   = {0, 0, 0, 0};
static uint32_t rtcmLastMs = 0;     // last RTCM packet from AgOpenWeb (port 2233)
static uint32_t rtcmBytes  = 0;

// Called from ReceiveUdp() for every packet from AgOpenWeb
void statusNoteAogPacket(IPAddress ip)
{
    aogLastMs = millis();
    aogIp[0] = ip[0]; aogIp[1] = ip[1]; aogIp[2] = ip[2]; aogIp[3] = ip[3];
}

// Called from udpNtrip() for every correction packet
void statusNoteRtcm(unsigned int len)
{
    rtcmLastMs = millis();
    rtcmBytes += len;
}

// A value that is reported only after it has been stable for STATUS_STABLE_MS
struct StableState
{
    int      reported  = -999;      // last value printed
    int      candidate = -999;
    uint32_t sinceMs   = 0;

    bool update(int v)              // true when v should be printed now
    {
        if (v != candidate) { candidate = v; sinceMs = millis(); }
        if (candidate != reported && millis() - sinceMs >= STATUS_STABLE_MS)
        {
            reported = candidate;
            return true;
        }
        return false;
    }
};

static const char* fixName(int q)
{
    switch (q)
    {
        case 0:  return "no fix";
        case 1:  return "GPS single";
        case 2:  return "DGPS";
        case 4:  return "RTK fixed";
        case 5:  return "RTK float";
        default: return "other";
    }
}

static void printIp(const uint8_t* ip)
{
    Serial.print(ip[0]); Serial.print('.'); Serial.print(ip[1]); Serial.print('.');
    Serial.print(ip[2]); Serial.print('.'); Serial.print(ip[3]);
}

// Back to "hhmmss.ss" from utcCs() (the time field x 100; -1 -> "none")
static void csToText(int32_t cs, char* out, size_t n)
{
    if (cs < 0) { snprintf(out, n, "none"); return; }
    snprintf(out, n, "%06ld.%02ld", (long)(cs / 100), (long)(cs % 100));
}

// Every 5 s: the sentence types the receiver sent in that window, with their rate, e.g.
// "[status] GPS: the receiver sends (last 5 s): GNGGA 10.0 Hz, GNHPR 10.0 Hz, GNVTG 10.0 Hz".
// Printed only when the set of types changes. Counted before the parser (zHandlers.ino).
static void statusRxTypes()
{
    static char reportedSet[160] = "";
    // sort by name (tiny table)
    for (uint8_t a = 0; a < rxTypeCount; a++)
        for (uint8_t b = a + 1; b < rxTypeCount; b++)
            if (strcmp(rxTypes[b].name, rxTypes[a].name) < 0) { RxType t = rxTypes[a]; rxTypes[a] = rxTypes[b]; rxTypes[b] = t; }

    char set[160] = "", line[300] = "";
    for (uint8_t k = 0; k < rxTypeCount; k++)
    {
        char item[40];
        snprintf(item, sizeof(item), "%s%s", k ? "," : "", rxTypes[k].name);
        strncat(set, item, sizeof(set) - strlen(set) - 1);
        snprintf(item, sizeof(item), "%s%s %.1f Hz", k ? ", " : "", rxTypes[k].name, rxTypes[k].count / (STATUS_SILENCE_MS / 1000.0));
        strncat(line, item, sizeof(line) - strlen(line) - 1);
    }
    if (rxTypeOthers)
    {
        char item[40];
        snprintf(item, sizeof(item), " + %u lines of other types", (unsigned)rxTypeOthers);
        strncat(line, item, sizeof(line) - strlen(line) - 1);
    }
    rxTypeCount  = 0;
    rxTypeOthers = 0;

    if (set[0] == '\0' || !strcmp(set, reportedSet)) return;   // nothing / same set as reported
    strncpy(reportedSet, set, sizeof(reportedSet) - 1);
    Serial.print("[status] GPS: the receiver sends (last 5 s): ");
    Serial.println(line);
}

// While there is no dual heading: say WHY (the most frequent reason over the last 5 s window),
// with the last HPR exactly as the receiver sent it. Printed when the state starts, and again
// when the reason changes.
static int noDualWhyReported = -1;
static void statusNoDualWhy(bool force)
{
    int why = epochNoDual;                         // fallback: the last epoch's reason
    uint32_t best = 0;
    for (int r = NODUAL_NOT_IN_TIME; r < NODUAL_COUNT; r++)
        if (noDualCount[r] > best) { best = noDualCount[r]; why = r; }
    if (why < NODUAL_NOT_IN_TIME) return;          // nothing to explain (dual OK / no HPR at all)
    if (!force && why == noDualWhyReported) return;
    noDualWhyReported = why;

    char b[340];
    if (why == NODUAL_QF)
        snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading ($GNTHS mode V) - the UM982 sends HPR with QF %d: "
                               "it has no heading solution from its two antennas (antennas, cables, UM982 heading setup)", hprQf);
    else if (why == NODUAL_NOT_IN_TIME)
    {
        const uint32_t now = millis();
        char tH[16], tG[16];
        csToText(hprUtcCs, tH, sizeof(tH));
        csToText(ggaUtcCs, tG, sizeof(tG));
        if (hprLateAtMs != 0 && now - hprLateAtMs < 2000)
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading ($GNTHS mode V) - the HPR arrives too late, %lu ms after its GGA "
                                   "(the firmware waits %d ms; %lu late HPRs so far). Please send this line",
                     (unsigned long)hprLateMs, EPOCH_WAIT_MS, (unsigned long)hprLateCount);
        else if (now - hprMs > 500)
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading ($GNTHS mode V) - the receiver stopped sending HPR (last one %lu ms ago)",
                     (unsigned long)(now - hprMs));
        else
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading ($GNTHS mode V) - HPRs arrive but their time (%s) does not match "
                                   "the GGA time (%s). Please send this line", tH, tG);
    }
    else
        snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading ($GNTHS mode V) - HPR QF %d but its heading field is empty or out of range", hprQf);
    Serial.println(b);
    Serial.print("[status]   last HPR from the receiver: ");
    Serial.println(hprRawLast[0] ? hprRawLast : "(none)");
}

// Called from loop()
void statusUpdate()
{
#if STATUS_MESSAGES
    static StableState link, aog, rtcm, ggaSeen, vtgSeen, fix, dual, tm171;
    static bool firstPosition = true;
    static bool everGga = false;
    const uint32_t now = millis();

    // ---- Ethernet cable / IP
    if (Ethernet_running)
    {
        const int up = (Ethernet.linkStatus() == LinkON) ? 1 : 0;
        if (link.update(up))
        {
            if (up)
            {
                Serial.print("[status] Ethernet: link up, module IP ");
                Serial.print(Ethernet.localIP());
                Serial.print(", sending to ");
                Serial.print(Eth_ipDestination);
                Serial.print(":");
                Serial.println(portDestination);
            }
            else Serial.println("[status] Ethernet: cable unplugged / no link");
        }
    }

    // ---- AgOpenWeb
    const int aogOn = (aogLastMs != 0 && now - aogLastMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (aog.update(aogOn))
    {
        if (aogOn) { Serial.print("[status] AgOpenWeb connected, IP "); printIp(aogIp); Serial.println(); }
        else if (aogLastMs != 0) Serial.println("[status] AgOpenWeb silent for 5 s");
    }

    // ---- RTCM corrections
    const int rtcmOn = (rtcmLastMs != 0 && now - rtcmLastMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (rtcm.update(rtcmOn))
    {
        if (rtcmOn) Serial.println("[status] RTCM corrections arriving from AgOpenWeb (NTRIP)");
        else if (rtcmLastMs != 0) Serial.println("[status] RTCM corrections stopped 5 s ago");
    }

    // ---- GGA from the receiver
    const int ggaOn = (ggaMs != 0 && now - ggaMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (ggaOn) everGga = true;
    if (ggaSeen.update(ggaOn) && !ggaOn && everGga)
        Serial.println("[status] GPS: no GGA from the receiver for 5 s");

    // ---- Fix quality and first position
    if (ggaOn)
    {
        const int q = atoi(fixQuality);
        if (fix.update(q))
        {
            Serial.print("[status] GPS: ");
            Serial.print(fixName(q));
            Serial.print(" (fix "); Serial.print(q);
            Serial.print(", "); Serial.print(numSats); Serial.print(" satellites");
            if (ageDGPS[0] != '\0') { Serial.print(", correction age "); Serial.print(ageDGPS); Serial.print(" s"); }
            Serial.println(")");
        }
        if (firstPosition && q > 0 && latitude[0] != '\0')
        {
            firstPosition = false;
            Serial.print("[status] GPS: first position ");
            Serial.print(nmeaToDeg(latitude, latNS[0]), 7); Serial.print(", ");
            Serial.print(nmeaToDeg(longitude, lonEW[0]), 7);
            Serial.print(", altitude "); Serial.print(altitude); Serial.println(" m");
        }
    }

    // ---- VTG (speed) from the receiver
    if (ggaOn)
    {
        const int vtgOn = (vtgMs != 0 && now - vtgMs < STATUS_SILENCE_MS) ? 1 : 0;
        if (vtgSeen.update(vtgOn))
        {
            if (vtgOn) Serial.println("[status] GPS: VTG arriving, speed from the receiver");
            else       Serial.println("[status] GPS: no VTG from the receiver, speed computed from the positions");
        }
    }

    // ---- Dual antenna heading (UM982 HPR -> $GNTHS) vs TM171 heading ($INHPR)
    if (ggaOn)
    {
        int d;
        const bool hprSeen = hprMs != 0 && (now - hprMs) < RECEIVER_SEEN_MS;
        if (!hprSeen) d = 0;                       // no HPR: single-antenna receiver (or HPR not configured)
        else d = epochDual ? 2 : 1;                // 2 dual heading, 1 HPR present but no dual heading
        if (dual.update(d))
        {
            if (d == 2)
            {
                Serial.print("[status] Dual: dual-antenna heading OK (HPR QF ");
                Serial.print(epochHprQf);
                Serial.println(epochHprQf == 4 ? " fixed), sent as $GNTHS" : " float), sent as $GNTHS");
            }
            else if (d == 1)
            {
                Serial.println("[status] Dual: no dual-antenna heading ($GNTHS mode V), AgOpenWeb uses the TM171 heading ($INHPR)");
                statusNoDualWhy(true);
            }
            else
            {
                Serial.println("[status] Single antenna: no HPR from the receiver (F9P / X20P, or HPR off), AgOpenWeb uses the TM171 heading ($INHPR) - set Dual GPS off");
                if (hprRawMs != 0 && now - hprRawMs < RECEIVER_SEEN_MS)
                {
                    // HPR lines ARE coming in, but none reached HPR_Handler: the parser rejects them
                    Serial.print("[status]   but HPR lines are arriving and none is accepted (bad format?) - please send this line: ");
                    Serial.println(hprRawLast);
                }
                else if (hprRawLast[0])
                {
                    char b[80];
                    snprintf(b, sizeof(b), "[status]   the last HPR line arrived %lu s ago: ", (unsigned long)((now - hprRawMs) / 1000));
                    Serial.print(b);
                    Serial.println(hprRawLast);
                }
            }
        }
    }

    // ---- Every 5 s: reason for no dual heading (if it changed), bad checksums, serial backlog
    static uint32_t windowMs = 0;
    static uint32_t crcAtWindow = 0;
    static bool     crcBad = false, slowRead = false;
    if (now - windowMs >= STATUS_SILENCE_MS)
    {
        windowMs = now;
        if (dual.reported == 1) statusNoDualWhy(false);
        for (int r = 0; r < NODUAL_COUNT; r++) noDualCount[r] = 0;

        // What the receiver sends (every sentence type, before the parser). Printed when the set changes.
        statusRxTypes();

        const uint32_t crcNew = nmeaCrcErrors - crcAtWindow;
        crcAtWindow = nmeaCrcErrors;
        if (crcNew > 0)
        {
            char b[160];
            snprintf(b, sizeof(b), "[status] GPS: %lu sentences from the receiver with a bad checksum in the last 5 s (bytes lost or corrupted on the serial line)", (unsigned long)crcNew);
            Serial.println(b);
            crcBad = true;
        }
        else if (crcBad)
        {
            Serial.println("[status] GPS: no more bad checksums from the receiver");
            crcBad = false;
        }

        // More than one whole epoch (GGA + VTG + HPR, about 210 bytes) waiting in the serial buffer
        const uint16_t peak = gnssRxPeak;
        gnssRxPeak = 0;
        if (peak >= 256 && !slowRead)
        {
            char b[160];
            snprintf(b, sizeof(b), "[status] GPS: the Teensy is reading the receiver late (up to %u bytes waiting in the serial buffer)", (unsigned)peak);
            Serial.println(b);
            slowRead = true;
        }
        else if (peak < 256 && slowRead)
        {
            Serial.println("[status] GPS: receiver data read on time again");
            slowRead = false;
        }
    }

    // ---- TM171
    const int tmOn = (TM171DataSeen && TM171lastData < TM171_FRESH_MS) ? 1 : 0;
    if (tm171.update(tmOn))
    {
        if (tmOn) Serial.println("[status] TM171: data OK (roll, and heading without dual antenna)");
        else      Serial.println("[status] TM171: NO DATA - no roll, no IMU heading ($INHPR QF 0)");
    }
#endif
}
