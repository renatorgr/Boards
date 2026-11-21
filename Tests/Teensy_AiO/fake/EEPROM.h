// EEPROM.h - the Teensy 4.1's EEPROM emulation (4284 bytes) on fake::Device::eeprom (Teensy test harness,
// Tests/Teensy_AiO). SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fake_core.h"

class EEPROMClass
{
public:
    uint8_t read(int a) const { return (a >= 0 && a < length()) ? fake::cur->eeprom[a] : 0; }
    void    write(int a, uint8_t v);
    void    update(int a, uint8_t v) { if (read(a) != v) write(a, v); }
    int     length() const { return (int)fake::cur->eeprom.size(); }
    template <class T> T& get(int a, T& t) const
    {
        uint8_t* p = (uint8_t*)&t;
        for (size_t i = 0; i < sizeof(T); i++) p[i] = read(a + (int)i);
        return t;
    }
    template <class T> const T& put(int a, const T& t)
    {
        const uint8_t* p = (const uint8_t*)&t;
        for (size_t i = 0; i < sizeof(T); i++) update(a + (int)i, p[i]);
        return t;
    }
};
extern EEPROMClass EEPROM;
