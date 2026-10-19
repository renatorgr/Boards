/* zUdpNtrip.ino - RTCM corrections from the program (UDP 2233) to the receiver.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in chriskinal's AOG_Teensy_UM982
   and Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Changes 2026 by Claude (Anthropic): datagrams read in pieces of the buffer size, [status] count;
   the RTCM messages counted (zRtcm.h, zStatus.ino).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
void udpNtrip()
{
#ifdef ARDUINO_TEENSY41
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpNtrip.parsePacket();

  if (packetLength > 0)
  {
    // Read the datagram in pieces of at most the buffer size, so a datagram larger than
    // Eth_NTRIP_packetBuffer (512) is forwarded whole and never written past the buffer.
    // (AgOpenWeb sends 256-byte datagrams; other senders may send more.)
    unsigned int total = 0;
    while (total < packetLength)
    {
      const int n = Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, sizeof(Eth_NTRIP_packetBuffer));
      if (n <= 0) break;
      SerialGPS->write(Eth_NTRIP_packetBuffer, n);
      statusNoteRtcm((const uint8_t*)Eth_NTRIP_packetBuffer, (uint16_t)n);   // [status] messages, web page (zStatus.ino)
      total += n;
    }
  }
 #endif
}
