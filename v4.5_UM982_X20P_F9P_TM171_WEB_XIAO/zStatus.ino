/* zStatus.ino - [status] messages and which program talks to the module.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zStatus.ino
// Short status messages, printed ONLY when something changes (so they can stay on in the
// tractor). Prefix "[status]". Every line goes to the USB serial monitor and to a log kept in
// RAM that the web page shows (zWeb.ino), so the box does not have to be opened to read them.
//   - Ethernet cable / link and the module's IP address
//   - AgOpenWeb or AgOpenGPS talking to the module (first packet, and silence); which one,
//     from its hello, and the automatic output mode that follows it
//   - RTCM corrections arriving from AgOpenWeb (NTRIP); and from the radio (XBee), with
//     bad checksums and bytes that are not RTCM 3 (zRtcm.h)
//   - GPS: first position, fix quality changes, no GGA / VTG from the receiver
//   - Dual antenna heading (UM982 HPR) available or lost; single-antenna receiver (no HPR)
//     Without dual heading: WHY (UM982 HPR QF 0 / HPR not arriving with its GGA / empty
//     heading) and the last HPR line exactly as the receiver sent it
//   - Receiver sentences with a bad checksum; Teensy reading the receiver too slowly
//   - IMU (TM171 / BNO085) data arriving or lost; the receiver model (zReceiver.ino)
// A state must hold for STATUS_STABLE_MS before it is reported, so a fix or dual state
// that flickers at 10 Hz does not flood the monitor.

#include "zStatusLog.h"     // the log class (included first by the main .ino)

#define STATUS_STABLE_MS    1000    // a new state is reported after it has held this long
#define STATUS_SILENCE_MS   5000    // AgOpenWeb / RTCM / GGA considered gone after this long


StatusLogClass StatusLog;      // see zStatusLog.h

// Sentence names in the messages, by output mode (zSettings.ino; automatic: by the program)
static const char* dualSentence() { return outClassicNow() ? "$PAOGI" : "$GNTHS"; }
// Where the TM171 heading goes: classic = $PAOGI with a dual-antenna receiver (AgOpenGPS stays on
// "Dual"), $PANDA with a single-antenna one (AgOpenGPS on "Fix"); standard = $INHPR.
static const char* imuSentence()  { return outClassicNow() ? (receiverIsDual() ? "$PAOGI" : "$PANDA") : "$INHPR"; }
static const char* noDualTag()
{
    static char t[40];
    if (!outClassicNow()) return "($GNTHS mode V)";
    snprintf(t, sizeof(t), "(%s heading in $PAOGI)", imuName());   // TM171 or BNO085
    return t;
}

static uint32_t aogLastMs  = 0;     // last UDP packet from AgOpenWeb (port 8888)
static uint8_t  aogIp[4]   = {0, 0, 0, 0};
// the corrections, counted as RTCM 3 messages (zRtcm.h): from the program (NTRIP, UDP 2233)
// and from the radio (XBee LR on Serial3)
RtcmCounter ntripRtcm, radioRtcm;

// Called from ReceiveUdp() for every packet from AgOpenWeb
void statusNoteAogPacket(IPAddress ip)
{
    aogLastMs = millis();
    aogIp[0] = ip[0]; aogIp[1] = ip[1]; aogIp[2] = ip[2]; aogIp[3] = ip[3];
}

// Called from udpNtrip() for every piece of a correction packet
void statusNoteRtcm(const uint8_t* d, uint16_t len)
{
    ntripRtcm.feed(d, len, millis());
}

// called from loop() with the bytes from the radio (they go on to the receiver)
void statusNoteRadio(const uint8_t* d, uint16_t len)
{
    radioRtcm.feed(d, len, millis());
}

// every 5 s, for one source of corrections: bad checksums, bytes with no RTCM message in them
static void statusRtcmWindow(RtcmCounter& r, const char* who, bool& badShown, bool& junkShown)
{
    r.window(STATUS_SILENCE_MS / 1000);
    if (r.newBad > 0)
    {
        StatusLog.line("[status] %s: %lu RTCM messages with a bad checksum in the last 5 s, %lu good (bytes lost or changed on the way)",
                       who, (unsigned long)r.newBad, (unsigned long)r.newGood);
        badShown = true;
    }
    else if (badShown && r.newGood > 0)
    {
        StatusLog.line("[status] %s: no more RTCM messages with a bad checksum", who);
        badShown = false;
    }
    const bool junk = r.newBytes >= 200 && r.newGood == 0;
    if (junk && !junkShown)
        StatusLog.line("[status] %s: %lu bytes in the last 5 s but no whole RTCM message (baud rate? not RTCM 3?)", who, (unsigned long)r.newBytes);
    junkShown = junk;
}

// ---- which program talks to the module, told apart by its hello (PGN 200, once a second
// to UDP 8888). Both send 80 81 7F C8 03 38 00 00 and a last byte:
//   AgIO (AgOpenGPS): 0x47, a fixed byte that AgIO never computes (AgIO UDP.designer.cs);
//   AgOpenWeb: the real checksum, 0x82 (PgnBuilder.BuildHelloPacket).
// The automatic output mode follows it: standard sentences for AgOpenWeb, classic for AgOpenGPS.
// Classic also before the first hello and for any other hello, because AgOpenWeb reads $PAOGI /
// $PANDA too, but AgOpenGPS does not read $GNTHS / $INHPR.
#define HOST_SWITCH_HELLOS  3       // hellos in a row from the other program before following it
#define HOST_GONE_MS        5000    // after this long without its hello the program is gone: the
                                    // next program is followed at its first hello
#define HOST_BOTH_MS        3000    // hellos from both programs within this time: two programs

static uint8_t  hostNow       = HOST_NONE;      // the program followed (last one seen)
static uint8_t  hostCand      = HOST_NONE;      // another program saying hello...
static uint8_t  hostCandN     = 0;              // ...this many times in a row
static uint32_t hostLastMs[4] = {0, 0, 0, 0};   // last hello of each kind (HostProgram)
static uint32_t hostHellos    = 0;
static uint8_t  hostIp[4]     = {0, 0, 0, 0};   // where the followed program's hello came from

static const char* hostName(uint8_t h)
{
    switch (h)
    {
        case HOST_AGOPENWEB: return "AgOpenWeb";
        case HOST_AGOPENGPS: return "AgOpenGPS";
        case HOST_OTHER:     return "another program";
        default:             return "AgOpenWeb / AgOpenGPS";
    }
}

uint8_t hostProgram() { return hostNow; }

bool outClassicNow()
{
    if (cfg.outputMode == OUT_CLASSIC)  return true;
    if (cfg.outputMode == OUT_STANDARD) return false;
    return hostNow != HOST_AGOPENWEB;            // automatic
}

// Hellos from AgOpenWeb and from AgOpenGPS in the last HOST_BOTH_MS
static bool hostBoth()
{
    const uint32_t now = millis();
    return hostLastMs[HOST_AGOPENWEB] != 0 && now - hostLastMs[HOST_AGOPENWEB] < HOST_BOTH_MS &&
           hostLastMs[HOST_AGOPENGPS] != 0 && now - hostLastMs[HOST_AGOPENGPS] < HOST_BOTH_MS;
}

static uint8_t helloKind(const uint8_t* d, uint16_t len)
{
    if (len != 9 || d[4] != 3) return HOST_OTHER;
    if (d[8] == 0x47) return HOST_AGOPENGPS;
    uint8_t ck = 0;
    for (uint8_t i = 2; i < 8; i++) ck += d[i];
    return d[8] == ck ? HOST_AGOPENWEB : HOST_OTHER;
}

// Called from ReceiveUdp() for every hello (PGN 200): d = the packet, len = its length
void statusNoteHello(const uint8_t* d, uint16_t len, IPAddress ip)
{
    const uint32_t now = millis();
    const uint8_t  k   = helloKind(d, len);
    const bool     goneBefore = hostNow == HOST_NONE || now - hostLastMs[hostNow] >= HOST_GONE_MS;
    hostLastMs[k] = now;
    hostHellos++;
    if (k == hostNow) { hostCandN = 0; hostIp[0] = ip[0]; hostIp[1] = ip[1]; hostIp[2] = ip[2]; hostIp[3] = ip[3]; return; }
    if (k == HOST_OTHER && hostNow != HOST_NONE && !goneBefore) return;   // keep following the known program
    if (k != hostCand) { hostCand = k; hostCandN = 0; }
    if (++hostCandN < (goneBefore ? 1 : HOST_SWITCH_HELLOS)) return;

    const bool wasClassic = outClassicNow();
    hostNow   = k;
    hostCandN = 0;
    hostIp[0] = ip[0]; hostIp[1] = ip[1]; hostIp[2] = ip[2]; hostIp[3] = ip[3];
    char ipText[20];
    snprintf(ipText, sizeof(ipText), "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
    const char* how = k == HOST_AGOPENGPS ? "AgIO hello" : k == HOST_AGOPENWEB ? "hello" : "hello not from AgIO nor AgOpenWeb";
    if (cfg.outputMode == OUT_AUTO)
        StatusLog.line("[status] Program: %s (%s from %s). Automatic mode: %s%s", hostName(k), how, ipText,
                       outClassicNow() ? "classic sentences ($PAOGI with the UM982, $PANDA with F9P / X20P)"
                                       : "standard sentences ($GNGGA $GNVTG $GNTHS $INHPR)",
                       outClassicNow() == wasClassic ? "" : " from now on");
    else if (k == HOST_AGOPENGPS && cfg.outputMode == OUT_STANDARD)
        StatusLog.line("[status] Program: AgOpenGPS (%s from %s), but the output mode is AgOpenWeb: AgOpenGPS does not read "
                       "$GNTHS / $INHPR - choose Automatic or AgOpenGPS on the web page", how, ipText);
    else
        StatusLog.line("[status] Program: %s (%s from %s), output mode %s (set on the web page)", hostName(k), how, ipText,
                       cfg.outputMode == OUT_CLASSIC ? "AgOpenGPS (classic)" : "AgOpenWeb (standard)");
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
    StatusLog.print(ip[0]); StatusLog.print('.'); StatusLog.print(ip[1]); StatusLog.print('.');
    StatusLog.print(ip[2]); StatusLog.print('.'); StatusLog.print(ip[3]);
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
char rxTypesLast[300] = "";        // the last 5 s summary (web page)

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
        char item[48];
        snprintf(item, sizeof(item), "%s%.15s", k ? "," : "", rxTypes[k].name);
        strncat(set, item, sizeof(set) - strlen(set) - 1);
        const unsigned tenths = (unsigned)rxTypes[k].count * 10u / (STATUS_SILENCE_MS / 1000u);   // Hz x 10 (whole numbers, no compiler warning)
        snprintf(item, sizeof(item), "%s%.15s %u.%u Hz", k ? ", " : "", rxTypes[k].name, tenths / 10u, tenths % 10u);
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
    memcpy(rxTypesLast, line, sizeof(rxTypesLast));          // for the web page, every window (same size, ends in '\0')

    if (set[0] == '\0' || !strcmp(set, reportedSet)) return;   // nothing / same set as reported
    memcpy(reportedSet, set, sizeof(reportedSet));           // same size, ends in '\0'
    StatusLog.print("[status] GPS: the receiver sends (last 5 s): ");
    StatusLog.println(line);
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
        snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading %s - the UM982 sends HPR with QF %d: "
                               "it has no heading solution from its two antennas (antennas, cables, UM982 heading setup)", noDualTag(), hprQf);
    else if (why == NODUAL_NOT_IN_TIME)
    {
        const uint32_t now = millis();
        char tH[16], tG[16];
        csToText(hprUtcCs, tH, sizeof(tH));
        csToText(ggaUtcCs, tG, sizeof(tG));
        if (hprLateAtMs != 0 && now - hprLateAtMs < 2000)
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading %s - the HPR arrives too late, %lu ms after its GGA "
                                   "(the firmware waits %d ms; %lu late HPRs so far). Please send this line",
                     noDualTag(), (unsigned long)hprLateMs, cfg.epochWaitMs, (unsigned long)hprLateCount);
        else if (now - hprMs > 500)
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading %s - the receiver stopped sending HPR (last one %lu ms ago)",
                     noDualTag(), (unsigned long)(now - hprMs));
        else
            snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading %s - HPRs arrive but their time (%s) does not match "
                                   "the GGA time (%s). Please send this line", noDualTag(), tH, tG);
    }
    else
        snprintf(b, sizeof(b), "[status] Dual: no dual-antenna heading %s - HPR QF %d but its heading field is empty or out of range", noDualTag(), hprQf);
    StatusLog.println(b);
    StatusLog.print("[status]   last HPR from the receiver: ");
    StatusLog.println(hprRawLast[0] ? hprRawLast : "(none)");
}

// Called from loop()
void statusUpdate()
{
    static StableState link, aog, rtcm, radio, ggaSeen, vtgSeen, fix, dual, tm171, both;
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
                StatusLog.print("[status] Ethernet: link up, module IP ");
                StatusLog.print(Ethernet.localIP());
                StatusLog.print(", sending to ");
                StatusLog.print(Eth_ipDestination);
                StatusLog.print(":");
                StatusLog.println(portDestination);
            }
            else StatusLog.println("[status] Ethernet: cable unplugged / no link");
        }
    }

    // ---- AgOpenWeb / AgOpenGPS (named after its hello)
    const int aogOn = (aogLastMs != 0 && now - aogLastMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (aog.update(aogOn))
    {
        if (aogOn) { StatusLog.print("[status] "); StatusLog.print(hostName(hostNow)); StatusLog.print(" connected, IP "); printIp(aogIp); StatusLog.println(); }
        else if (aogLastMs != 0) { StatusLog.print("[status] "); StatusLog.print(hostName(hostNow)); StatusLog.println(" silent for 5 s"); }
    }
    if (both.update(hostBoth() ? 1 : 0) && hostBoth())
        StatusLog.line("[status] Hellos from AgOpenWeb AND AgOpenGPS: two programs talk to the module - close one (the output follows %s)",
                       hostName(hostNow));

    // ---- RTCM corrections from the program (NTRIP) and from the radio
    const int rtcmOn = ntripRtcm.arriving(now, STATUS_SILENCE_MS) ? 1 : 0;
    if (rtcm.update(rtcmOn))
    {
        if (rtcmOn) StatusLog.line("[status] RTCM corrections arriving from %s (NTRIP)", hostName(hostNow));
        else if (ntripRtcm.lastMs != 0) StatusLog.println("[status] RTCM corrections (NTRIP) stopped 5 s ago");
    }
    const int radioOn = radioRtcm.arriving(now, STATUS_SILENCE_MS) ? 1 : 0;
    if (radio.update(radioOn))
    {
        if (radioOn) StatusLog.println("[status] Radio (XBee): RTCM corrections arriving");
        else if (radioRtcm.lastMs != 0) StatusLog.println("[status] Radio (XBee): corrections stopped 5 s ago");
    }

    // ---- GGA from the receiver
    const int ggaOn = (ggaMs != 0 && now - ggaMs < STATUS_SILENCE_MS) ? 1 : 0;
    if (ggaOn) everGga = true;
    if (ggaSeen.update(ggaOn) && !ggaOn && everGga)
        StatusLog.println("[status] GPS: no GGA from the receiver for 5 s");

    // ---- Fix quality and first position
    if (ggaOn)
    {
        const int q = atoi(fixQuality);
        if (fix.update(q))
        {
            StatusLog.print("[status] GPS: ");
            StatusLog.print(fixName(q));
            StatusLog.print(" (fix "); StatusLog.print(q);
            StatusLog.print(", "); StatusLog.print(numSats); StatusLog.print(" satellites");
            if (ageDGPS[0] != '\0') { StatusLog.print(", correction age "); StatusLog.print(ageDGPS); StatusLog.print(" s"); }
            StatusLog.println(")");
        }
        if (firstPosition && q > 0 && latitude[0] != '\0')
        {
            firstPosition = false;
            StatusLog.print("[status] GPS: first position ");
            StatusLog.print(nmeaToDeg(latitude, latNS[0]), 7); StatusLog.print(", ");
            StatusLog.print(nmeaToDeg(longitude, lonEW[0]), 7);
            StatusLog.print(", altitude "); StatusLog.print(altitude); StatusLog.println(" m");
        }
    }

    // ---- VTG (speed) from the receiver
    if (ggaOn)
    {
        const int vtgOn = (vtgMs != 0 && now - vtgMs < STATUS_SILENCE_MS) ? 1 : 0;
        if (vtgSeen.update(vtgOn))
        {
            if (vtgOn) StatusLog.println("[status] GPS: VTG arriving, speed from the receiver");
            else       StatusLog.println("[status] GPS: no VTG from the receiver, speed computed from the positions");
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
                StatusLog.print("[status] Dual: dual-antenna heading OK (HPR QF ");
                StatusLog.print(epochHprQf);
                StatusLog.print(epochHprQf == 4 ? " fixed), sent as " : " float), sent as ");
                StatusLog.println(dualSentence());
            }
            else if (d == 1)
            {
                if (outClassicNow())
                {
                    if (tmHeadingUsable())
                        StatusLog.line("[status] Dual: no dual-antenna heading, AgOpenGPS gets the %s heading in $PAOGI (%s)", imuName(),
                                       offsetFrom == 'C' ? "aligned to the GPS course" : "aligned to the dual heading, corrected by the GPS course");
                    else
                        StatusLog.line("[status] Dual: no dual-antenna heading, AgOpenGPS gets the GPS course in $PAOGI while moving (%s %s)", imuName(),
                                       !epochTmOk ? "lost" : !yawSignLocked ? "turning direction not known yet - set it on the Settings page or drive a few curves"
                                                                            : "not aligned yet - drive straight above 3 km/h");
                }
                else
                    StatusLog.line("[status] Dual: no dual-antenna heading %s, AgOpenWeb uses the %s heading (%s)", noDualTag(), imuName(), imuSentence());
                statusNoDualWhy(true);
            }
            else
            {
                if (!outClassicNow())
                    StatusLog.line("[status] Single antenna: no HPR from the receiver (%s, or HPR off), AgOpenWeb uses the %s heading (%s) - set Dual GPS off",
                                   rxShortName("F9P / X20P"), imuName(), imuSentence());
                else if (receiverIsDual())
                    StatusLog.line("[status] Dual: the UM982 stopped sending HPR, AgOpenGPS gets the %s heading ($PAOGI) - check the UM982 output (GPHPR 0.1)", imuName());
                else
                    StatusLog.line("[status] Single antenna: no HPR from the receiver (%s), sending $PANDA with the %s heading - set the AgOpenGPS heading source to Fix",
                                   rxShortName("F9P / X20P"), imuName());
                if (hprRawMs != 0 && now - hprRawMs < RECEIVER_SEEN_MS)
                {
                    // HPR lines ARE coming in, but none reached HPR_Handler: the parser rejects them
                    StatusLog.print("[status]   but HPR lines are arriving and none is accepted (bad format?) - please send this line: ");
                    StatusLog.println(hprRawLast);
                }
                else if (hprRawLast[0])
                {
                    char b[80];
                    snprintf(b, sizeof(b), "[status]   the last HPR line arrived %lu s ago: ", (unsigned long)((now - hprRawMs) / 1000));
                    StatusLog.print(b);
                    StatusLog.println(hprRawLast);
                }
            }
        }
    }

    // ---- IMU heading: yaw direction learned, aligned without dual heading, glitches compensated
    {
        static bool     signReported  = false;
        static bool     courseAligned = false;
        static uint32_t jumpsReported = 0, reseedsReported = 0;
        if (cfg.tm171YawSign == 0 && yawSignLocked && !signReported)
        {
            signReported = true;
            StatusLog.line("[status] %s: yaw direction learned from the %s: %s (it can be set on the Settings page to skip learning)", imuName(),
                           signSrc == 'C' ? "GPS course" : "dual heading", yawSign > 0 ? "+1" : "-1");
        }
        if (offsetSeeded && offsetFrom == 'C' && !courseAligned)
        {
            courseAligned = true;
            StatusLog.line("[status] %s: heading aligned to the GPS course (no dual heading yet; forward travel assumed)", imuName());
        }
        if (tmJumpCount != jumpsReported)
        {
            jumpsReported = tmJumpCount;
            StatusLog.line("[status] %s: yaw jumped more than %d deg in 100 ms, compensated (%lu so far)", imuName(), (int)TM171_JUMP_DEG, (unsigned long)tmJumpCount);
        }
        if (courseReseedCount != reseedsReported)
        {
            reseedsReported = courseReseedCount;
            StatusLog.line("[status] %s: heading did not match the GPS course for 3 s, re-aligned to it (forward assumed; %lu so far)", imuName(), (unsigned long)courseReseedCount);
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

        // the corrections: rate, bad checksums
        static bool ntripBad = false, ntripJunk = false, radioBad = false, radioJunk = false;
        statusRtcmWindow(ntripRtcm, "NTRIP", ntripBad, ntripJunk);
        statusRtcmWindow(radioRtcm, "Radio (XBee)", radioBad, radioJunk);

        const uint32_t crcNew = nmeaCrcErrors - crcAtWindow;
        crcAtWindow = nmeaCrcErrors;
        if (crcNew > 0)
        {
            char b[160];
            snprintf(b, sizeof(b), "[status] GPS: %lu sentences from the receiver with a bad checksum in the last 5 s (bytes lost or corrupted on the serial line)", (unsigned long)crcNew);
            StatusLog.println(b);
            crcBad = true;
        }
        else if (crcBad)
        {
            StatusLog.println("[status] GPS: no more bad checksums from the receiver");
            crcBad = false;
        }

        // More than one whole epoch (GGA + VTG + HPR, about 210 bytes) waiting in the serial buffer
        const uint16_t peak = gnssRxPeak;
        gnssRxPeak = 0;
        if (peak >= 256 && !slowRead)
        {
            char b[160];
            snprintf(b, sizeof(b), "[status] GPS: the Teensy is reading the receiver late (up to %u bytes waiting in the serial buffer)", (unsigned)peak);
            StatusLog.println(b);
            slowRead = true;
        }
        else if (peak < 256 && slowRead)
        {
            StatusLog.println("[status] GPS: receiver data read on time again");
            slowRead = false;
        }
    }

    // ---- IMU (TM171 or BNO085, zImu.ino)
    const int tmOn = (TM171DataSeen && TM171lastData < TM171_FRESH_MS) ? 1 : 0;
    if (tm171.update(tmOn))
    {
        if (tmOn) StatusLog.line("[status] %s: data OK (roll, and heading without dual antenna)", imuName());
        else      StatusLog.line("[status] %s: NO DATA - no roll, no IMU heading (%s)", imuName(), outClassicNow() ? "$PANDA heading 65535" : "$INHPR QF 0");
    }
}
