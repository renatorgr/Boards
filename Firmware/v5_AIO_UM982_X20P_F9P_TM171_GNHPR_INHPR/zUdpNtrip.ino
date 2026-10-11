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
      total += n;
    }
    statusNoteRtcm(total);                                           // [status] messages (zStatus.ino)
  }
 #endif
}
