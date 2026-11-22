// NativeEthernetUdp.h - UDP sockets on fake::Device (Teensy test harness, Tests/Teensy_AiO).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "NativeEthernet.h"

class EthernetUDP
{
public:
    uint8_t   begin(uint16_t port);
    void      stop();
    int       parsePacket();
    int       available() { return (int)(in_.size() - at_); }
    int       read() { return at_ < in_.size() ? in_[at_++] : -1; }
    int       read(uint8_t* buf, size_t n);
    int       read(char* buf, size_t n) { return read((uint8_t*)buf, n); }
    int       peek() { return at_ < in_.size() ? in_[at_] : -1; }
    void      flush() {}
    IPAddress remoteIP() { return inIp_; }
    uint16_t  remotePort() { return inPort_; }
    int       beginPacket(IPAddress ip, uint16_t port);
    size_t    write(uint8_t b) { out_.push_back(b); return 1; }
    size_t    write(const uint8_t* buf, size_t n) { out_.insert(out_.end(), buf, buf + n); return n; }
    size_t    write(const char* s) { return write((const uint8_t*)s, strlen(s)); }
    size_t    write(const char* s, size_t n) { return write((const uint8_t*)s, n); }
    int       endPacket();
private:
    uint16_t port_ = 0;
    std::vector<uint8_t> out_, in_;
    size_t at_ = 0;
    IPAddress outIp_, inIp_;
    uint16_t outPort_ = 0, inPort_ = 0;
};
