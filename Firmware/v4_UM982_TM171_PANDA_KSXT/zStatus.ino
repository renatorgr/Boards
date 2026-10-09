// zStatus.ino
// Short status messages on the USB serial monitor, printed ONLY when something changes
// (so it can stay on in the tractor). Prefix "[status]".
//   - Ethernet cable / link and the module's IP address
//   - AgOpenWeb talking to the module (first packet, and silence)
//   - RTCM corrections arriving from AgOpenWeb (NTRIP)
//   - GPS: first position, fix quality changes, no GGA from the UM982
//   - Dual antenna (KSXT) available or lost
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

// Called from loop()
void statusUpdate()
{
#if STATUS_MESSAGES
    static StableState link, aog, rtcm, ggaSeen, fix, dual;
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

    // ---- GGA from the UM982
    const int ggaOn = (ggaMs != 0 && now - ggaMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (ggaOn) everGga = true;
    if (ggaSeen.update(ggaOn) && !ggaOn && everGga)
        Serial.println("[status] GPS: no GGA from the UM982 for 5 s");

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

    // ---- Dual antenna (KSXT) vs single antenna ($PANDA)
    if (ggaOn)
    {
        int d;
        if (ksxtRxCount == 0) d = 0;               // no KSXT at all
        else d = sendKsxt ? 2 : 1;                 // 2 dual, 1 KSXT present but no dual heading
        if (dual.update(d))
        {
            if (d == 2)
            {
                Serial.print("[status] Dual: dual-antenna heading OK (");
                Serial.print(ksxtLastQuality == 3 ? "RTK fixed" : "RTK float");
                Serial.println("), sending $KSXT");
            }
            else if (d == 1) Serial.println("[status] Dual: no dual-antenna heading, sending $PANDA with TM171 heading");
            else             Serial.println("[status] Dual: no KSXT from the UM982, sending $PANDA with TM171 heading");
        }
    }
#endif
}
