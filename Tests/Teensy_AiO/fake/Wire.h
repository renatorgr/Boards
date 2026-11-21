// Wire.h - the Teensy's I2C ports (Wire, Wire1, Wire2) on the pretend chips of fake::Device (Teensy test harness,
// Tests/Teensy_AiO). SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fake_core.h"

class TwoWire : public Stream
{
public:
    explicit TwoWire(int bus) : bus_(bus) {}
    void    begin() { on_ = true; }
    void    end() { on_ = false; }
    void    setClock(uint32_t) {}
    void    setSDA(uint8_t) {}
    void    setSCL(uint8_t) {}
    void    beginTransmission(uint8_t addr) { addr_ = addr; out_.clear(); }
    void    beginTransmission(int addr) { beginTransmission((uint8_t)addr); }
    size_t  write(uint8_t b) override { out_.push_back(b); return 1; }
    size_t  write(const uint8_t* b, size_t n) override { out_.insert(out_.end(), b, b + n); return n; }
    size_t  write(int b) { return write((uint8_t)b); }
    size_t  write(unsigned int b) { return write((uint8_t)b); }
    size_t  write(long b) { return write((uint8_t)b); }
    using Print::write;
    uint8_t endTransmission(bool stop = true);
    uint8_t requestFrom(uint8_t addr, uint8_t n, bool stop = true);
    uint8_t requestFrom(int addr, int n) { return requestFrom((uint8_t)addr, (uint8_t)n); }
    uint8_t requestFrom(uint8_t addr, uint8_t n, uint8_t stop) { return requestFrom(addr, n, (bool)stop); }
    int     available() override { return (int)(in_.size() - at_); }
    int     read() override { return at_ < in_.size() ? in_[at_++] : -1; }
    int     peek() override { return at_ < in_.size() ? in_[at_] : -1; }
private:
    int bus_;
    bool on_ = false;
    uint8_t addr_ = 0;
    std::vector<uint8_t> out_, in_;
    size_t at_ = 0;
};
extern TwoWire Wire, Wire1, Wire2;
