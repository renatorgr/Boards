/* Network.h - Ethernet: the module's IP address, the UDP ports and sockets (Network.cpp).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   The names come from the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in chriskinal's
   AOG_Teensy_UM982; they were globals of the main .ino until 2026.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Network.h
// Included at the top of the main .ino, so every .ino file can use the sockets and the addresses.
#ifndef NETWORK_H
#define NETWORK_H

#include <stdint.h>

#ifdef ARDUINO_TEENSY41
#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// The first three numbers of the IP address: from the EEPROM, changed by the program (PGN 201,
// Autosteer.ino). The last one is set by EthernetStart().
struct ConfigIP {
    uint8_t ipOne   = 192;
    uint8_t ipTwo   = 168;
    uint8_t ipThree = 5;
};
extern ConfigIP networkAddress;

extern byte Eth_myip[4];                // the module's IP address
extern byte mac[6];

extern unsigned int portMy;             // 5120: GNSS out (from this port)
extern unsigned int AOGNtripPort;       // 2233: NTRIP in
extern unsigned int AOGAutoSteerPort;   // 8888: steering in / out
extern unsigned int portDestination;    // 9999: to the program, and the machine module in
extern char Eth_NTRIP_packetBuffer[512];

extern EthernetUDP Eth_udpPAOGI;
extern EthernetUDP Eth_udpNtrip;
extern EthernetUDP Eth_udpAutoSteer;
extern EthernetUDP Eth_udpModules;      // UDP 9999 in: the machine module's PGNs (zMachine.ino)

extern IPAddress Eth_ipDestination;     // x.x.x.255: the program (broadcast)
#endif

extern bool Ethernet_running;           // Ethernet hardware found, sockets open

// Start Ethernet and open the UDP ports. steerModule: IP .126 (steer module), else .120 (GPS only)
void EthernetStart(bool steerModule);

// The network stack, from loop() (Net.h)
void netPollFromLoop();                 // after EthernetStart(): the library's timer stops
void netPoll();                         // fnet_poll(), once a millisecond at most
void netDelay(uint32_t ms);             // delay() that keeps the network going

#endif
