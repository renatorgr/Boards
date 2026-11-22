// FlexCAN_T4.h - the Teensy 4's CAN ports on fake::Device (only CAN3 is wired: the Keya) (Teensy test harness,
// Tests/Teensy_AiO). SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fake_core.h"

enum CAN_DEV_TABLE { CAN0 = 0, CAN1 = 1, CAN2 = 2, CAN3 = 3 };
enum FLEXCAN_RXQUEUE_TABLE { RX_SIZE_2 = 2, RX_SIZE_4 = 4, RX_SIZE_8 = 8, RX_SIZE_16 = 16, RX_SIZE_32 = 32, RX_SIZE_64 = 64,
                             RX_SIZE_128 = 128, RX_SIZE_256 = 256, RX_SIZE_512 = 512, RX_SIZE_1024 = 1024 };
enum FLEXCAN_TXQUEUE_TABLE { TX_SIZE_2 = 2, TX_SIZE_4 = 4, TX_SIZE_8 = 8, TX_SIZE_16 = 16, TX_SIZE_32 = 32, TX_SIZE_64 = 64,
                             TX_SIZE_128 = 128, TX_SIZE_256 = 256, TX_SIZE_512 = 512, TX_SIZE_1024 = 1024 };
typedef struct CAN_message_t
{
    uint32_t id = 0;
    uint16_t timestamp = 0;
    uint8_t  idhit = 0;
    struct { bool extended = 0; bool remote = 0; bool overrun = 0; bool reserved = 0; } flags;
    uint8_t  len = 8;
    uint8_t  buf[8] = { 0 };
    int8_t   mb = 0;
    uint8_t  bus = 0;
    bool     seq = 0;
} CAN_message_t;

bool fakeCanRead(CAN_message_t& m);
int  fakeCanWrite(const CAN_message_t& m);
void fakeCanBegin(uint32_t baud);

template <CAN_DEV_TABLE B, FLEXCAN_RXQUEUE_TABLE R = RX_SIZE_16, FLEXCAN_TXQUEUE_TABLE T = TX_SIZE_16>
class FlexCAN_T4
{
public:
    void begin() { fakeCanBegin(0); }
    void setBaudRate(uint32_t baud) { fakeCanBegin(baud); }
    void setMaxMB(uint8_t) {}
    void enableFIFO(bool = 1) {}
    void enableFIFOInterrupt(bool = 1) {}
    void setFIFOFilter(int) {}
    void events() {}
    int  read(CAN_message_t& m) { return B == CAN3 ? fakeCanRead(m) : 0; }
    int  write(const CAN_message_t& m) { return B == CAN3 ? fakeCanWrite(m) : 0; }
};
#define ACCEPT_ALL 0
#define REJECT_ALL 1
