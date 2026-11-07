/* zEthernet.ino - Ethernet start-up and the module's IP address.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in chriskinal's AOG_Teensy_UM982
   and Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Changes 2026 by Claude (Anthropic) for renatorgr: two messages; the network stack polled from loop()
   (netPollFromLoop, netPoll, netDelay - see zNet.h); 8 KB socket buffers; one [boot] line instead of
   the start-up prints (the link and the IP: "[status] Ethernet: link up ...", zStatus.ino);
   2026-10-24: the subnet changed from the XIAO page (netSetSubnet; the card on the module's own page was
   taken out again the same day: a mistake there would leave the page unreachable).
   2026-11-02: the module announces its web page by mDNS (http://aio.local/, DNS-SD "_http._tcp"), so a browser
   finds it by name and AgOpenWeb's "Search the network" (web cards) lists it at once (mdnsStart).
   2026-11-02, later: the socket buffer size comes from zNet.h (NET_SOCKET_SIZE), shared with the web server's
   own listening socket (zWeb.ino).
   2026-11-07: the page is also announced as an AgOpenWeb add-on card (DNS-SD "_aowcard._tcp", TXT v=1 and path, the
   official Docs/ADD_ON_CARDS.md), under an instance name unique to this module ("AiO" + four digits of the Teensy's
   own serial id, mdnsName: the Ethernet MAC is the same on every AiO board).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
void EthernetStart()
{
#ifdef ARDUINO_TEENSY41
  // start the Ethernet connection (its [boot] line at the end; the link and the module IP: the
  // "[status] Ethernet: link up ..." line, zStatus.ino, also when the cable is plugged in later)

  // Socket buffers of 8 KB (the library's are 2 KB): an answer to the web page goes into the network
  // stack at once and is sent while loop() runs on. With 2 KB, loop() waited for the browser's ACKs, up to
  // 130 ms per answer, and the GPS serial buffer filled up ("reading the receiver late"). 8 sockets x
  // 2 x 8 KB + a 96 KB stack heap, in RAM2.
  Ethernet.setStackHeap(96 * 1024);
  Ethernet.setSocketSize(NET_SOCKET_SIZE);    // 8 KB (zNet.h; the web server's listening socket uses the same)

  // try to congifure using IP:
  Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    StatusLog.println("[boot] Ethernet: no Ethernet hardware - GPS over USB only");

    return;
  }


//grab the ip from EEPROM
  Eth_myip[0] = networkAddress.ipOne;
  Eth_myip[1] = networkAddress.ipTwo;
  Eth_myip[2] = networkAddress.ipThree;
  if (Autosteer_running) 
  {
    Eth_myip[3] = 126;  //126 is steer module, with or without GPS
  }
  else
  {
    Eth_myip[3] = 120;  //120 is GPS only module
  }

  Ethernet.setLocalIP(Eth_myip);  // Change IP address to IP set by user

  Ethernet_running = true;
  mdnsStart();                    // http://aio.local/ (after the IP: the name is announced with it)

  Eth_ipDestination[0] = Eth_myip[0];
  Eth_ipDestination[1] = Eth_myip[1];
  Eth_ipDestination[2] = Eth_myip[2];
  Eth_ipDestination[3] = 255;

  // the UDP ports: GNSS out (from portMy), NTRIP in, steering in and out, the machine module in (9999)
  const bool gOk = Eth_udpPAOGI.begin(portMy);
  const bool nOk = Eth_udpNtrip.begin(AOGNtripPort);
  const bool aOk = Eth_udpAutoSteer.begin(AOGAutoSteerPort);
  const bool mOk = Eth_udpModules.begin(portDestination);    // the machine module's PGNs (zMachine.ino)
  StatusLog.line("[boot] Ethernet: UDP ports %u GNSS out%s, %u NTRIP in%s, %u steering in / out%s, %u machine module in%s",
                 portMy, gOk ? "" : " (FAILED)", AOGNtripPort, nOk ? "" : " (FAILED)", AOGAutoSteerPort, aOk ? "" : " (FAILED)",
                 portDestination, mOk ? "" : " (FAILED)");
 #endif
}

// ---------------------------------------------------------------- mDNS: the page by name
// The module answers for the name MDNS_HOST.local (http://aio.local/ in a browser on the same network) and
// announces its web page twice over DNS-SD, as the XIAO, sprayer and seed drill programs do: as "_http._tcp",
// which the AgOpenWeb fork's web cards ("Search the network") list in a second, by name, without sweeping the
// addresses; and as "_aowcard._tcp", the official AgOpenWeb add-on card announcement (Docs/ADD_ON_CARDS.md:
// TXT v=1 is required, path says where the page is). The instance name of both is what the operator will see in
// AgOpenWeb's list, so it has to tell this module from another one on the same network: "AiO" and four digits
// of the Teensy's own serial id (mdnsName; the Ethernet MAC is the same on every AiO board). It is FNET's own
// responder (fnet_mdns, built into NativeEthernet), run from loop() by netPoll() like the rest of the stack; it
// answers queries sent from any port by unicast (RFC 6762 6.7), which is how AgOpenWeb asks. A name clash on
// the network is settled by FNET (it adds a number to the name).
#define MDNS_HOST "aio"
char mdnsName[12] = "";                         // "AiO 3F2A": the instance name of both services (the Connections card, zWeb.ino)

#ifdef ARDUINO_TEENSY41
static const fnet_mdns_txt_key_t* aowcardTxt()  // the TXT record of "_aowcard._tcp"
{
  static const fnet_mdns_txt_key_t txt[] = { { (char*)"v", (char*)"1" }, { (char*)"path", (char*)"/" }, { 0, 0 } };
  return txt;
}
#endif

static void mdnsStart()
{
#ifdef ARDUINO_TEENSY41
  snprintf(mdnsName, sizeof(mdnsName), "AiO %04X", (unsigned)(HW_OCOTP_MAC0 & 0xFFFF));
  MDNS.begin(MDNS_HOST, 2);                     // the host name and room for two services
  MDNS.setServiceName(mdnsName);                // NativeEthernet: one instance name for every service
  MDNS.addService("_http._tcp", 80);            // the web page, for the fork's "Search the network"
  MDNS.addService("_aowcard._tcp", 80, aowcardTxt);   // the official add-on card announcement
  StatusLog.line("[boot] mDNS: the page is also http://%s.local/, announced as \"%s\" (_http._tcp and _aowcard._tcp: AgOpenWeb's web cards find it by name)", MDNS_HOST, mdnsName);
#endif
}

// ---------------------------------------------------------------- the network stack, from loop()
// (zNet.h) netPollOn: the library's 1 ms timer is stopped and fnet_poll() is called here, by loop()
// (netPoll) and by the code that waits for the network (netDelay, the web page's waits in zWeb.ino).
static bool     netPollOn = false;
static uint32_t netPollUs = 0;

void netPollFromLoop()
{
#ifdef ARDUINO_TEENSY41
  netPollTimer(NetPollTimerTag())->end();       // no more fnet_poll() in the timer interrupt
  netPollOn = fnet_netif_get_default() != 0;    // FNET started (Ethernet.begin() worked)
  netPollUs = micros();
  if (netPollOn) fnet_poll();
#endif
}

void netPoll()
{
#ifdef ARDUINO_TEENSY41
  if (!netPollOn) return;
  const uint32_t now = micros();
  if (now - netPollUs < 1000) return;           // once a millisecond, as the library's timer did
  netPollUs = now;
  fnet_poll();
#endif
}

void netDelay(uint32_t ms)
{
  const uint32_t t0 = millis();
  while (millis() - t0 < ms)
  {
    netPoll();
    delayMicroseconds(100);
  }
}

// ---------------------------------------------------------------- the module's subnet
// The module's IP is <subnet>.126 (or .120 without autosteer), the subnet saved in the EEPROM (address 60,
// with the steer settings; 192.168.5 from new). AgIO and AgOpenWeb change it with PGN 201 (Autosteer.ino);
// this does the same from the XIAO page (PGN 144, zXiao.ino), which is on WiFi and so is reached whatever
// the module's IP is. Not from the module's own page: a mistake there would leave it unreachable. Saved here; the caller answers and then restarts the Teensy (the new IP
// is taken at start-up, EthernetStart). Returns an error text, or nullptr when saved.
const char* netSetSubnet(uint8_t a, uint8_t b, uint8_t c, const char* who)
{
  if (a == 0 || a == 127 || a > 223)                return "invalid subnet (the first number: 1 to 223, not 127)";
  if (watchdogTimer < WATCHDOG_THRESHOLD)           return "not while autosteer is engaged";
  if (a == networkAddress.ipOne && b == networkAddress.ipTwo && c == networkAddress.ipThree)
                                                    return "the module is on that subnet already";
  const uint8_t last = Eth_myip[3] ? Eth_myip[3] : (Autosteer_running ? 126 : 120);
  StatusLog.line("[status] subnet %u.%u.%u -> %u.%u.%u from %s: saved, restarting as %u.%u.%u.%u",
                 networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, a, b, c, who, a, b, c, last);
  networkAddress.ipOne = a;
  networkAddress.ipTwo = b;
  networkAddress.ipThree = c;
#ifdef ARDUINO_TEENSY41
  EEPROM.put(60, networkAddress);
#endif
  return nullptr;
}
