/* zImplement.ino - implement mode: the module as a GPS on an implement (a trailed seed drill), not a steer module.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// README "Implement mode". A second AiO on the implement, on the tractor's network, measures where the implement really
// goes (its antenna, its roll) for a logger on the tablet, while the program steers with the tractor's module only. So
// this module must never look like a second tractor GPS or steer module to the program:
//  - its positions go to UDP 9990 (IMPLEMENT_PORT), never to the program's 9999: sendLine (zHandlers.ino);
//  - nothing else goes to the program: progSend and SendUdp (Autosteer.ino), the XIAO's frames (zXiao.ino);
//  - it answers no hello (PGN 200) and no scan (202), and takes no steering (254), steer settings (252, 251) or
//    geometry (247): programPacket (Autosteer.ino). Only the subnet (201), so it stays on the program's network;
//  - no steering at all (Autosteer_running false, autosteerSetup), IP .130 (zEthernet.ino), mDNS aio-implement.local;
//  - one sentence per fix ($PANDA with an F9P / X20P, $PAOGI with a UM982 or a dual F9P: outClassicNow, zStatus.ino),
//    so the logger writes one line per fix, with the roll in it.
// They go to the program's computer (the address its hellos or its corrections come from, heard in the last 5 s),
// where the logger runs; with no program heard, to everyone on the subnet (x.x.x.255).

#define IMPL_PROGRAM_GONE_MS 5000

static uint8_t     implProgIp[4] = { 0, 0, 0, 0 };
static uint32_t    implProgMs    = 0;       // msStamp: the program's computer last heard (0 = never)
static const char* implProgFrom  = "";      // what it was heard by: "hello" or "corrections"
static uint32_t    implSent      = 0;       // datagrams sent to UDP 9990 since start-up
static uint32_t    implSecMs     = 0;       // the second being counted (msStamp)
static uint16_t    implSecN      = 0;       // ...datagrams in it so far
static uint16_t    implRate      = 0;       // datagrams in the last whole second
static int8_t      implUnicast   = -1;      // the last one went to: 1 the program's computer, 0 everyone, -1 none yet

// The program's computer: from its hello (Autosteer.ino, programPacket) and from its corrections (zUdpNtrip.ino)
void implementNoteProgram(const uint8_t* ip, const char* what)
{
    if (!implementMode()) return;
    memcpy(implProgIp, ip, 4);
    implProgMs   = msStamp(millis());
    implProgFrom = what;
}

static bool implProgramHeard(uint32_t now) { return implProgMs != 0 && now - implProgMs < IMPL_PROGRAM_GONE_MS; }

// One sentence (with its CR LF) of the epoch, from sendLine (zHandlers.ino); never through the XIAO: the implement's
// module is on the cable (the XIAO bridge takes corrections only from its own WiFi, README "Implement mode")
void implementSend(const char* line, size_t n)
{
    if (!Ethernet_running) return;
    const uint32_t now = millis();
    const bool uni = implProgramHeard(now);
    IPAddress dst = uni ? IPAddress(implProgIp[0], implProgIp[1], implProgIp[2], implProgIp[3]) : Eth_ipDestination;
    if ((int8_t)uni != implUnicast)
    {
        implUnicast = uni ? 1 : 0;
        if (uni) StatusLog.line("[status] Implement: positions to %u.%u.%u.%u:%u, the program's computer (its %s)",
                                dst[0], dst[1], dst[2], dst[3], IMPLEMENT_PORT, implProgFrom);
        else     StatusLog.line("[status] Implement: positions to everyone, %u.%u.%u.%u:%u (no program heard for %u s)",
                                dst[0], dst[1], dst[2], dst[3], IMPLEMENT_PORT, IMPL_PROGRAM_GONE_MS / 1000);
    }
    Eth_udpPAOGI.beginPacket(dst, IMPLEMENT_PORT);
    Eth_udpPAOGI.write(line, n);
    Eth_udpPAOGI.endPacket();
    implSent++;
    if (implSecMs == 0 || now - implSecMs >= 1000)
    {
        implRate  = (implSecMs != 0 && now - implSecMs < 2000) ? implSecN : 0;
        implSecMs = msStamp(now);
        implSecN  = 0;
    }
    implSecN++;
}

// For /api/status (zWeb.ino) and the XIAO page (PGN 141)
FLASHMEM void implementJson(Json& j, uint32_t now)
{
    j.obj("impl");
    j.bo("on", implementMode());                      // this start runs as an implement's GPS
    j.bo("next", roleSaved == ROLE_IMPLEMENT);       // the role saved, taken at the next start
    j.u("port", IMPLEMENT_PORT);
    j.u("sent", implSent);
    j.u("rate", (implSecMs != 0 && now - implSecMs < 2000) ? implRate : 0);
    char ip[20] = "";
    if (implUnicast == 1) snprintf(ip, sizeof(ip), "%u.%u.%u.%u", implProgIp[0], implProgIp[1], implProgIp[2], implProgIp[3]);
    else if (implUnicast == 0) snprintf(ip, sizeof(ip), "%u.%u.%u.255", Eth_myip[0], Eth_myip[1], Eth_myip[2]);
    j.str("to", ip);                                  // where the last one went ("" none yet)
    j.str("by", implUnicast == 1 ? implProgFrom : "");
    j.close();
}
