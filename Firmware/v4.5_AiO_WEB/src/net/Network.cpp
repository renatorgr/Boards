/* Network.cpp - Ethernet start-up, the module's IP address, the UDP sockets, the network stack from loop().
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in chriskinal's AOG_Teensy_UM982
   and Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Changes 2026 by Claude (Anthropic): two messages; the network stack polled from loop()
   (netPollFromLoop, netPoll, netDelay - see Net.h); 8 KB socket buffers; one [boot] line instead of
   the start-up prints (the link and the IP: "[status] Ethernet: link up ...", zStatus.ino); the
   network globals moved here from the main .ino (Network.h), EthernetStart(steerModule).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Network.cpp  (was zEthernet.ino)

#include <Arduino.h>
#include "Network.h"
#include "Net.h"                        // the network stack runs from loop(), not from an interrupt
#include "../system/StatusLog.h"        // [boot] lines (zStatus.ino)

// ---------------------------------------------------------------- addresses and sockets (Network.h)
#ifdef ARDUINO_TEENSY41
ConfigIP networkAddress;

byte Eth_myip[4] = {0, 0, 0, 0};
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy           = 5120;
unsigned int AOGNtripPort     = 2233;
unsigned int AOGAutoSteerPort = 8888;
unsigned int portDestination  = 9999;
char Eth_NTRIP_packetBuffer[512];

EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpAutoSteer;
EthernetUDP Eth_udpModules;            // UDP 9999 in: the machine module's PGNs (zMachine.ino)

IPAddress Eth_ipDestination;
#endif

bool Ethernet_running  = false;

// ---------------------------------------------------------------- start-up
void EthernetStart(bool steerModule)
{
#ifdef ARDUINO_TEENSY41
  // start the Ethernet connection (its [boot] line at the end; the link and the module IP: the
  // "[status] Ethernet: link up ..." line, zStatus.ino, also when the cable is plugged in later)

  // Socket buffers of 8 KB (the library's are 2 KB): an answer to the web page goes into the network
  // stack at once and is sent while loop() runs on. With 2 KB, loop() waited for the browser's ACKs, up to
  // 130 ms per answer, and the GPS serial buffer filled up ("reading the receiver late"). 8 sockets x
  // 2 x 8 KB + a 96 KB stack heap, in RAM2.
  Ethernet.setStackHeap(96 * 1024);
  Ethernet.setSocketSize(8 * 1024);

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
  if (steerModule)
  {
    Eth_myip[3] = 126;  //126 is steer module, with or without GPS
  }
  else
  {
    Eth_myip[3] = 120;  //120 is GPS only module
  }

  Ethernet.setLocalIP(Eth_myip);  // Change IP address to IP set by user

  Ethernet_running = true;

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

// ---------------------------------------------------------------- the network stack, from loop()
// (Net.h) netPollOn: the library's 1 ms timer is stopped and fnet_poll() is called here, by loop()
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
