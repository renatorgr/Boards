/* zEthernet.ino - Ethernet start-up and the module's IP address.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 Micro board
   (renatorgr, github.com/renatorgr/Boards).
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in chriskinal's AOG_Teensy_UM982
   and Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Small change 2026 by Claude (Anthropic): two messages.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
void EthernetStart()
{
#ifdef ARDUINO_TEENSY41
  // start the Ethernet connection:
  Serial.println("Initializing ethernet with static IP address");

  // try to congifure using IP:
  Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found. GPS via USB only.");

    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet cable is not connected - Starting ethernet anyway.");
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
  Serial.println("\r\nEthernet status OK");
  Serial.print("IP set Manually: ");
  Serial.println(Ethernet.localIP());

  Ethernet_running = true;

  Eth_ipDestination[0] = Eth_myip[0];
  Eth_ipDestination[1] = Eth_myip[1];
  Eth_ipDestination[2] = Eth_myip[2];
  Eth_ipDestination[3] = 255;

  Serial.print("\r\nEthernet IP of module: "); Serial.println(Ethernet.localIP());
  Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
  Serial.print("All data sending to port: "); Serial.println(portDestination);

  // init UPD Port sending to AOG
  if (Eth_udpPAOGI.begin(portMy))
  {
    Serial.print("Ethernet GNSS UDP sending from port: ");
    Serial.println(portMy);
  }

  // init UPD Port getting NTRIP from AOG
  if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
  {
    Serial.print("Ethernet NTRIP UDP listening to port: ");
    Serial.println(AOGNtripPort);
  }

  // init UPD Port getting AutoSteer data from AOG
  if (Eth_udpAutoSteer.begin(AOGAutoSteerPort)) // AOGAutoSteerPortipPort
  {
    Serial.print("Ethernet AutoSteer UDP listening to & send from port: ");
    Serial.println(AOGAutoSteerPort);
  }
 #endif
}
