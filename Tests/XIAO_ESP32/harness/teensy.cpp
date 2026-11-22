/* teensy.cpp - the pretend Teensy: its side of the serial link with the XIAO.
   Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   It follows zXiao.ino of the Teensy firmware AiO_WEB (in this repository, GPL v3): the same frames, limits, timings
   and refusals, without the Teensy's own work behind them (GNSS, steering, Ethernet).
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The numbers below are zXiao.ino's (XIAO_HELLO_TIMEOUT_MS, XIAO_TX_RESERVE, BRIDGE_CABLE_MS, ...). When that file
// changes what goes over the link, this one follows.
#include "bench.h"

namespace bench
{
static const char HELLO[] = "ESP32-hello";
enum { SRC_TEENSY = 0x7E, PGN_STATUS = 141, PGN_LOG = 142, PGN_INFO = 143, PGN_CMD = 144, PGN_FW = 145, PGN_NET_IN = 146,
       PGN_RTCM = 147, PGN_NET_OUT = 148, PGN_BRIDGE = 149 };
enum { HELLO_TIMEOUT_MS = 10000, GAP_MS = 50, FRAMES_PER_LOOP = 8, TEXT_MAX = 160, TEXT_PER_10S = 10, STATUS_MS = 1000,
       LOG_PER_LOOP = 4, TX_RESERVE = 256, TX_BUFFER = 2048, TX_IDLE_WIFI = 128, CABLE_MS = 3000, GONE_MS = 3000,
       STATE_MS = 1000, FW_TIMEOUT_MS = 15000, LOG_LINES = 60, STEER_WATCHDOG_MS = 2500 };

void Teensy::log(const std::string& text)
{
    lines.push_back({ nextSeq++, ms(), text.substr(0, 200) });
    if (lines.size() > LOG_LINES) lines.pop_front();
    allLines.push_back(text);
}
bool Teensy::logged(const std::string& part) const
{
    for (auto& l : allLines) if (has(l, part)) return true;
    return false;
}

void Teensy::restart(uint32_t offMs)
{
    on = false;
    backAt_ = fake::clockMs + offMs;
    restarts++;
    tx.clear();
    in_.clear();
    xf_.clear();
    text_.clear();
    lines.clear();
    nextSeq = xlNext_ = 1;
    detected = textBad_ = steerOn_ = fwPending_ = fwActive_ = false;
    brState = 0;
    told = 255;
    helloPos_ = 0;
    xsBuf_.clear();
    brCableMs_ = brWifiMs_ = brIgnoreMs_ = 0;
}
void Teensy::boot()
{
    on = true;
    backAt_ = 0;
    bootAt = fake::clockMs;
    log("[status] module started");
}

void Teensy::writeFrame(uint8_t pgn, const uint8_t* d, uint8_t n)
{
    uint8_t sum = (uint8_t)(SRC_TEENSY + pgn + n);
    tx.push_back(0x80); tx.push_back(0x81); tx.push_back(SRC_TEENSY); tx.push_back(pgn); tx.push_back(n);
    for (uint8_t i = 0; i < n; i++) { tx.push_back(d[i]); sum += d[i]; }
    tx.push_back(sum);
}
// the Teensy's own frames (status, log): only with room to spare, so the program's packets never wait for them
bool Teensy::sendOwn(uint8_t pgn, const uint8_t* d, uint8_t n)
{
    if (brState == 2 ? room() < TX_BUFFER - TX_IDLE_WIFI : room() < (int)n + 6 + TX_RESERVE) return false;
    writeFrame(pgn, d, n);
    return true;
}
// one datagram for the program on the XIAO's WiFi (PGN 148)
void Teensy::netOutSend(const std::vector<uint8_t>& d)
{
    if (brState != 2) return;               // (it goes out on the network cable instead)
    if (d.empty() || d.size() > 255 || room() < (int)d.size() + 6) { netOutDrop++; return; }
    writeFrame(PGN_NET_OUT, d.data(), (uint8_t)d.size());
    netOut++;
    outLog.push_back(d);
}

void Teensy::hello()
{
    hellos++;
    lastHelloMs_ = fake::clockMs;
    world()->helloLastMs_ = fake::clockMs;
    if (!detected)
    {
        detected = true;
        xlNext_ = nextSeq > LOG_LINES ? nextSeq - LOG_LINES : 1;
        xsBuf_.clear();
        xsLastMs_ = fake::clockMs - STATUS_MS;
        log(gps2On ? "[xiao] XIAO said hello on Serial2: AgOpenWeb packets (UDP 8888) now go to it, its PGNs go to UDP 9999"
                   : "[xiao] XIAO said hello on Serial2, but the GPS2 port is off in the settings: ignored");
    }
}

void Teensy::textFlush()
{
    if (!text_.empty() && !textBad_ && gps2On && detected)
    {
        const uint64_t now = fake::clockMs;
        if (now - textWinMs_ > 10000) { textWinMs_ = now; textWinN_ = 0; }
        if (textWinN_ < TEXT_PER_10S)
        {
            textWinN_++;
            xiaoSaid.push_back(text_);
            log("[xiao] > " + text_);
        }
        else textDropped++;
    }
    text_.clear();
    textBad_ = false;
}
void Teensy::textByte(uint8_t b)
{
    if (b == '\n') { textFlush(); return; }
    if (b == '\r') return;
    if (b < 0x20 || b > 0x7E) { textBad_ = true; return; }
    if (text_.size() < TEXT_MAX) text_ += (char)b;
}

void Teensy::shift(size_t n)
{
    while (n < xf_.size() && xf_[n] != 0x80) n++;
    xf_.erase(xf_.begin(), xf_.begin() + std::min(n, xf_.size()));
}

void Teensy::takeCmd(const uint8_t* d, uint8_t n)
{
    if (n >= 4 && d[0] == 1)
    {
        const char* err = (d[1] == 0 || d[1] == 127 || d[1] > 223) ? "the first number must be 1 to 223, and not 127"
                          : engagedNow() ? "not while autosteer is engaged" : nullptr;
        if (err)
        {
            subnetRefused.push_back(err);
            log(fmt("[xiao] subnet %u.%u.%u refused: %s", d[1], d[2], d[3], err));
            return;
        }
        subnets.push_back({ d[1], d[2], d[3] });
        restart();                          // it saves the subnet and restarts, as after PGN 201
        return;
    }
    log(fmt("[xiao] unknown command from the XIAO: PGN 144, command %u, %u bytes", n ? d[0] : 0, n));
}

void Teensy::fwAnswer(uint8_t cmd, uint16_t seq, uint8_t code, const char* text)
{
    uint8_t d[4 + 160];
    d[0] = cmd; d[1] = (uint8_t)seq; d[2] = (uint8_t)(seq >> 8); d[3] = code;
    size_t t = text ? strlen(text) : 0;
    if (t > sizeof(d) - 4) t = sizeof(d) - 4;
    if (t) memcpy(d + 4, text, t);
    writeFrame(PGN_FW, d, (uint8_t)(4 + t));
}
void Teensy::fwStop(uint8_t cmd, uint16_t seq, const char* why)
{
    fwAnswer(cmd, seq, 1, why);
    fwActive_ = false;
    fwRefused++;
    fwWhy = why;
    log(fmt("[xiao] firmware update REFUSED: %s (old firmware kept)", why));
}

// A firmware update for the Teensy from the XIAO page (PGN 145): begin, blocks answered one by one, end
void Teensy::takeFw(const uint8_t* d, uint8_t n)
{
    if (n < 1) return;
    const uint8_t cmd = d[0];
    const uint16_t seq = n >= 3 ? (uint16_t)(d[1] | (d[2] << 8)) : 0;
    const char* refusal = engagedNow() ? "autosteer is engaged - disengage it first"
                          : moving ? "the tractor is moving - stop first (on a bench, switch simulator mode on)" : nullptr;
    if (cmd == 1)
    {
        if (fwActive_ || fwPending_) { fwAnswer(1, 0, 1, "an update is already running"); return; }
        uint32_t size = 0;
        if (n >= 6) memcpy(&size, d + 2, 4);
        const char* why = n < 6 ? "bad begin frame" : refusal;
        if (!why && (size < 1000 || size > 8000000)) why = "no file, or file size not plausible";
        if (why)
        {
            fwRefused++;
            fwWhy = why;
            log(fmt("[xiao] firmware update from the XIAO page REFUSED: %s (old firmware kept)", why));
            fwAnswer(1, 0, 1, why);
            return;
        }
        fwForce_ = (d[1] & 1) != 0;
        fwSize_ = size;
        fwExpectSeq_ = 1;
        fwGot_.clear();
        fwPending_ = true;
        return;
    }
    if (cmd == 5)
    {
        if (refusal)
        {
            log(fmt("[xiao] going back to the previous firmware REFUSED: %s", refusal));
            fwAnswer(5, 0, 1, refusal);
            return;
        }
        reverts++;
        fwAnswer(5, 0, 0, "");
        while (!tx.empty()) { world()->dev.uartFeed(&tx.front(), 1); tx.pop_front(); }     // Serial2.flush(), then it installs
        restart(4000);
        return;
    }
    if (!fwActive_)
    {
        if (cmd == 2 || cmd == 3) fwAnswer(cmd, seq, 1, "no update running");
        return;
    }
    fwLastMs_ = fake::clockMs;
    if (cmd == 2)
    {
        if (fwStopAt && seq >= fwStopAt) return;                        // (the harness: it stops answering here)
        if (seq + 1 == fwExpectSeq_) { fwAnswer(2, seq, 0, ""); return; }   // the last block again: its answer was lost
        if (seq != fwExpectSeq_) { fwStop(2, seq, "blocks out of order (link error) - try again"); return; }
        fwGot_.insert(fwGot_.end(), d + 3, d + n);
        fwExpectSeq_++;
        if (fwLoseAnswer == seq) { fwLoseAnswer = 0; return; }           // (the harness: this answer is lost on the wire)
        fwAnswer(2, seq, 0, "");
        return;
    }
    if (cmd == 3)
    {
        if (fwGot_.size() != fwSize_ || (!fwExpect.empty() && fwGot_ != fwExpect)) { fwStop(3, 0, "the file did not arrive whole (checksum)"); return; }
        log(fmt("[xiao] firmware update: %lu bytes checked, installing and restarting", (unsigned long)fwSize_));
        fwAnswer(3, 0, 0, std::to_string(fwSize_).c_str());
        fwDone++;
        fwActive_ = false;
        while (!tx.empty()) { world()->dev.uartFeed(&tx.front(), 1); tx.pop_front(); }
        restart(5000);
        return;
    }
    if (cmd == 4) fwStop(4, seq, "cancelled from the XIAO page");
}

// A packet of the program, as one on UDP 8888: what the Teensy answers goes back through the XIAO
void Teensy::programPacket(const uint8_t* d, int n)
{
    if (n < 6 || d[0] != 0x80 || d[1] != 0x81 || n < 6 + d[4]) return;
    const uint8_t pgn = d[3];
    if (pgn == 200)
    {
        netOutSend(packet(126, { 0, 0, 0, 0, 0 }, 126));
        netOutSend(packet(121, { 0, 0, 0, 0, 0 }, 121));
    }
    else if (pgn == 254 && d[4] >= 8)
    {
        n254_++;
        if (d[5 + 2] & 1) { steerOn_ = true; last254Ms_ = fake::clockMs; }
        else steerOn_ = false;
        netOutSend(packet(253, { d[5 + 3], d[5 + 4], (uint8_t)n254_, (uint8_t)(n254_ >> 8), 0, 0, (uint8_t)(steerOn_ ? 0 : 1), 60 }, 126));
        if (n254_ % 4 == 0) netOutSend(packet(250, { (uint8_t)n254_, 0, 0, 0, 0, 0, 0, 0 }, 126));
    }
    else if (pgn == 201)
    {
        wifi201++;
        log("[status] subnet change (PGN 201) from the program on the XIAO's WiFi: not done");
    }
}

void Teensy::takeNet(const uint8_t* d, uint8_t n)
{
    if (n < 4 + 5 || fwActive_) return;
    world()->bridgeFrame(d, n);
    const uint64_t now = fake::clockMs;
    if (brCableMs_ != 0 && now - brCableMs_ < CABLE_MS)
    {
        netIgnored++;
        if (brIgnoreMs_ == 0 || now - brIgnoreMs_ >= 30000)
        {
            brIgnoreMs_ = now;
            log(fmt("[xiao] WiFi bridge: a program on the XIAO's WiFi (%u.%u.%u.%u) is calling, but one is talking on the network cable: the cable has the module, the WiFi one is not answered", d[0], d[1], d[2], d[3]));
        }
        return;
    }
    if (brState != 2)
    {
        brState = 2;
        log(fmt("[xiao] WiFi bridge: the program at %u.%u.%u.%u is on the XIAO's WiFi - GPS and steering go to it through the XIAO, nothing for the program goes out on the network", d[0], d[1], d[2], d[3]));
    }
    memcpy(brIp_, d, 4);
    brWifiMs_ = now;
    netIn++;
    programPacket(d + 4, (int)n - 4);
}

int Teensy::scan()
{
    int sent = 0;
    while (!xf_.empty())
    {
        if (xf_[0] != 0x80 || (xf_.size() >= 2 && xf_[1] != 0x81)) { shift(1); continue; }
        if (xf_.size() < 5) break;
        const size_t need = 6 + xf_[4];
        if (xf_.size() < need) break;
        uint8_t sum = 0;
        for (size_t i = 2; i < need - 1; i++) sum += xf_[i];
        if (sum == xf_[need - 1])
        {
            const uint8_t pgn = xf_[3], len = xf_[4];
            const std::vector<uint8_t> data(xf_.begin() + 5, xf_.begin() + 5 + len);
            if (!detected) beforeHello++;
            else if (pgn == PGN_INFO) { info.assign(data.begin(), data.end()); infoRx++; }
            else if (pgn == PGN_CMD) takeCmd(data.data(), len);
            else if (pgn == PGN_FW && knowsFw) takeFw(data.data(), len);
            else if (pgn == PGN_NET_IN && knowsBridge) { takeNet(data.data(), len); sent++; }
            else if (pgn == PGN_RTCM && knowsBridge) { if (len && brState == 2 && !fwActive_) rtcm.insert(rtcm.end(), data.begin(), data.end()); }
            else
            {
                // any other PGN goes out on the network cable to UDP 9999, as an older firmware does with all of them
                toCable++;
                if (pgn == PGN_NET_IN || pgn == PGN_RTCM)
                {
                    old146++;
                    if (pgn == PGN_NET_IN) world()->bridgeFrame(data.data(), len);
                }
                sent++;
            }
            if (!on) return sent;           // that frame restarted it
            shift(need);
        }
        else
        {
            crcErr++;
            shift(1);
        }
    }
    return sent;
}

int Teensy::byte(uint8_t b)
{
    if (b == (uint8_t)HELLO[helloPos_])
    {
        if (++helloPos_ == (int)sizeof(HELLO) - 1)
        {
            helloPos_ = 0;
            hello();
            text_.clear();
            textBad_ = false;
            return 0;
        }
    }
    else helloPos_ = b == (uint8_t)HELLO[0] ? 1 : 0;
    if (!gps2On) return 0;
    if (xf_.empty() && b != 0x80) { textByte(b); return 0; }
    if (xf_.empty()) { text_.clear(); textBad_ = false; }
    xf_.push_back(b);
    xfLastMs_ = fake::clockMs;
    return scan();
}

void Teensy::bridgeTick()
{
    const uint64_t now = fake::clockMs;
    const bool here = detected && gps2On;
    if (brState == 2 && (!here || now - brWifiMs_ >= GONE_MS))
    {
        brState = 0;
        log(fmt("[xiao] WiFi bridge: %s - the module is free again, GPS goes out on the network", here ? "nothing from the program on the XIAO's WiFi for 3 s" : "the XIAO is gone"));
    }
    else if (brState == 1 && now - brCableMs_ >= CABLE_MS) brState = 0;
    if (!here) { told = 255; return; }
    if ((brState != told || now - brStateMs_ >= STATE_MS) && room() >= 3 + 6)
    {
        const uint8_t d[3] = { 1, brState, 0 };
        writeFrame(PGN_BRIDGE, d, 3);
        if (brState != told) toldSince = now;
        told = brState;
        brStateMs_ = toldAt = now;
    }
}

// Status (once a second, in pieces of 240 bytes) and log lines to the XIAO
void Teensy::txTick()
{
    const uint64_t now = fake::clockMs;
    if (!detected || !gps2On) { xsBuf_.clear(); return; }
    for (int k = 0; k < LOG_PER_LOOP && xlNext_ < nextSeq; k++)
    {
        const Line* l = nullptr;
        for (auto& x : lines) if (x.seq == xlNext_) l = &x;
        if (!l) { xlNext_++; continue; }
        uint8_t d[8 + 200];
        const size_t t = std::min(l->text.size(), sizeof(d) - 8);
        memcpy(d, &l->seq, 4);
        memcpy(d + 4, &l->ms, 4);
        memcpy(d + 8, l->text.data(), t);
        if (!sendOwn(PGN_LOG, d, (uint8_t)(8 + t))) return;
        xlNext_++;
        logSent++;
    }
    if (xsBuf_.empty() && sendsStatus && now - xsLastMs_ >= STATUS_MS)
    {
        xsLastMs_ = now;
        statusN++;
        xsBuf_ = fmt("{\"n\":%lu,\"up\":%lu,\"fw\":\"v4.5_AiO_WEB\",\"gnss\":{\"fix\":4,\"sats\":28,\"age\":0.8},\"steer\":{\"on\":%s,\"angle\":-1.25},\"note\":\"",
                     (unsigned long)statusN, (unsigned long)(ms() / 1000), steerOn_ ? "true" : "false");
        while (xsBuf_.size() + 3 < statusBytes) xsBuf_ += (char)('a' + xsBuf_.size() % 26);
        xsBuf_ += "\"}";
        xsPos_ = 0;
        xsSet_++;
        xsPart_ = 0;
        xsParts_ = (uint8_t)((xsBuf_.size() + 239) / 240);
    }
    while (!xsBuf_.empty() && xsPart_ < xsParts_)
    {
        const size_t k = std::min<size_t>(xsBuf_.size() - xsPos_, 240);
        uint8_t d[3 + 240];
        d[0] = xsSet_; d[1] = xsPart_; d[2] = xsParts_;
        memcpy(d + 3, xsBuf_.data() + xsPos_, k);
        if (!sendOwn(PGN_STATUS, d, (uint8_t)(3 + k))) return;
        xsPos_ += k;
        xsPart_++;
    }
    if (!xsBuf_.empty() && xsPart_ >= xsParts_)
    {
        lastStatus = xsBuf_;
        xsBuf_.clear();
        statusSent++;
    }
}

// One pass of the Teensy's loop (it runs far faster than once a millisecond; nothing here depends on that)
void Teensy::step()
{
    const uint64_t now = fake::clockMs;
    if (!on)
    {
        in_.clear();
        if (backAt_ && now >= backAt_) boot();
        return;
    }
    if (detected && now - lastHelloMs_ > HELLO_TIMEOUT_MS)
    {
        detected = false;
        silentEvents++;
        log("[xiao] XIAO silent for 10 s (no hello): stopped sending it the AgOpenWeb packets");
    }
    int frames = 0;
    if (!xf_.empty() && now - xfLastMs_ > GAP_MS && in_.empty())
    {
        cut++;
        shift(1);
        frames += scan();
        xfLastMs_ = now;
    }
    if ((!text_.empty() || textBad_) && now - xRxMs_ > 500 && in_.empty()) { text_.clear(); textBad_ = false; }
    int budget = 2048;
    while (on && budget-- > 0 && (fwActive_ || frames < FRAMES_PER_LOOP) && !in_.empty())
    {
        const uint8_t b = in_.front();
        in_.pop_front();
        frames += byte(b);
        xRxMs_ = now;
    }
    if (!on) return;
    if (fwPending_)
    {
        fwPending_ = false;
        fwActive_ = true;
        fwLastMs_ = now;
        steerOn_ = false;
        log(fmt("[xiao] firmware update from the XIAO page: receiving a .hex file (%lu bytes) over the XIAO link, steering off", (unsigned long)fwSize_));
        fwAnswer(1, 0, 0, "");
    }
    if (fwActive_)                          // the update has the Teensy's loop: nothing else is sent meanwhile
    {
        if (now - fwLastMs_ > FW_TIMEOUT_MS) fwStop(2, fwExpectSeq_, "upload stopped (timeout): nothing from the XIAO for 15 s");
        return;
    }
    if (steerOn_ && now - last254Ms_ > STEER_WATCHDOG_MS)
    {
        steerOn_ = false;
        steerOffs++;
        log("[status] autosteer OFF: no steering data from the program for 2.5 s");
    }
    if (cable && now - cableMs_ >= 10)       // a program on the network cable: its PGN 254 a hundred times a second
    {
        cableMs_ = now;
        brCableMs_ = now;
        if (brState == 2) log(fmt("[xiao] WiFi bridge: a program is talking on the network cable - the cable has the module, the program on the XIAO's WiFi (%u.%u.%u.%u) is no longer answered", brIp_[0], brIp_[1], brIp_[2], brIp_[3]));
        brState = 1;
        cableSeq_++;
        const std::vector<uint8_t> p = packet(254, { 50, 0, 0, (uint8_t)cableSeq_, (uint8_t)(cableSeq_ >> 8), 0, 0x0F, 0 });
        if (detected && gps2On && room() >= (int)p.size()) tx.insert(tx.end(), p.begin(), p.end());   // copied to the XIAO, for its page
    }
    if (knowsBridge) bridgeTick();
    txTick();
    if (gnss && brState == 2 && now - gnssMs_ >= 100)       // four sentences for the program, ten times a second
    {
        gnssMs_ = now;
        epoch_++;
        const char* f[4] = { "$PANDA,1205%02u.%u0,3842.1234567,N,00907.7654321,W,4,28,0.6,123.45,0.8,7.2,270.15,0.52,-0.31,0.05*5A\r\n",
                             "$GNGGA,1205%02u.%u0,3842.1234567,N,00907.7654321,W,4,28,0.6,123.45,M,50.1,M,0.8,0123*6C\r\n",
                             "$GNVTG,270.15,T,,M,3.89,N,7.20,K,D*%02u%u\r\n", "$PAOGI,%02u%u,4,28*11\r\n" };
        for (auto s : f)
        {
            const std::string t = fmt(s, (epoch_ / 10) % 60, epoch_ % 10);
            netOutSend(std::vector<uint8_t>(t.begin(), t.end()));
        }
    }
}
}
