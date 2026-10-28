/* zOutputs.ino - the relays: sections, hydraulic lift, tramline, geo stop.
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr. Made from Sprayer_ESP32 v0.8.
   Hydraulic lift timing after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3).
   2026-10-28 (Claude): v0.2: the Relay24_ESP32's relays (74HC595 shift registers) next to the KC868-A32's.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Each relay has a function (relayFn): a section, the hydraulic lift, a tramline side or the geo stop.
//   - cfg.firstSection 1-64: relay 1 is that section and the next relays the next sections, in a row
//     (the 19-section seed drill has more sections than the program's pin config knows).
//   - cfg.firstSection 0: what the program gives each relay (PGN 236, kept in cfg.pin): sections 1-16 and
//     the other functions, for the first 24 relays.
// Everything is off while the program is silent.
//
// RELAY_ON_HIGH (Seeder_ESP32.ino) says how the board's relays close: on a HIGH 74HC595 output on the
// Relay24_ESP32, on a LOW PCF8574 pin on the KC868-A32.
// cfg.invertOutputs (PGN 238, byte 8 bit 0; the official module calls it isRelayActiveHigh) turns that
// round: 0 = as built, 1 = inverted.
// The program sends this setting at start-up and when the machine profile is loaded, so the
// "relay active high" box in its machine settings normally stays unticked.

// The pin level for a function that is on or off, after the board's polarity and the program's setting
static int relayLevel(bool on)
{
    const bool drive = on != (cfg.invertOutputs != 0);
    return (drive == (RELAY_ON_HIGH != 0)) ? HIGH : LOW;
}

static uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

// What relay i (0 = relay 1) does: 0 nothing, 1-64 that section, FN_OTHER + 1-5 lift lower, lift raise,
// tramline right, tramline left, geo stop
uint8_t relayFn(uint8_t i)
{
    if (cfg.firstSection)
    {
        const uint16_t s = cfg.firstSection + i;
        return s <= MAX_SECTIONS ? s : 0;
    }
    if (i >= 24) return 0;
    const uint8_t p = cfg.pin[i];
    if (p >= PIN_HYD_LOWER && p <= PIN_GEO_STOP) return FN_OTHER + (p - PIN_HYD_LOWER + 1);
    return p <= 16 ? p : 0;
}

#if BOARD == BOARD_RELAY24
// The Relay24_ESP32's relays: 24 bits shifted into the chain of three 74HC595 chips, relay 24 first (it ends
// up at the far end, U3 output QH) and relay 1 last (U1 output QA), then one latch pulse moves them all to the
// outputs at once. Written every time: a bit upset by noise on the wires is put right within 50 ms.
static void relaysWrite(uint32_t bits)
{
    for (int8_t i = NUM_RELAYS - 1; i >= 0; i--)
    {
        digitalWrite(SR_DATA, relayLevel((bits >> i) & 1));
        digitalWrite(SR_CLOCK, HIGH);
        digitalWrite(SR_CLOCK, LOW);
    }
    digitalWrite(SR_LATCH, HIGH);
    digitalWrite(SR_LATCH, LOW);
}

void outputsInit()
{
    pinMode(SR_DATA, OUTPUT);
    pinMode(SR_CLOCK, OUTPUT);
    pinMode(SR_LATCH, OUTPUT);
    digitalWrite(SR_CLOCK, LOW);
    digitalWrite(SR_LATCH, LOW);
    relaysWrite(0);                                         // every relay off in the chips first,
    pinMode(SR_ENABLE, OUTPUT);
    digitalWrite(SR_ENABLE, LOW);                           // then their outputs on (the board's pull-up held them off)
}

#elif BOARD == BOARD_KC868_A32
// The KC868-A32's relays: eight on each PCF8574 chip, one byte per chip (bit 0 = its first relay). A chip
// is only written when its byte changes, and once a second anyway (a chip that missed a write catches up).
static int16_t chipSent[4] = { -1, -1, -1, -1 };
static bool    chipOk[4] = { true, true, true, true };
static uint32_t chipRefreshMs = 0;

static void relaysWrite(uint32_t bits)
{
    const bool refresh = millis() - chipRefreshMs >= 1000;
    if (refresh) chipRefreshMs = millis();
    for (uint8_t c = 0; c < 4; c++)
    {
        uint8_t b = 0;
        for (uint8_t k = 0; k < 8; k++) if (relayLevel((bits >> (c * 8 + k)) & 1) == HIGH) b |= 1 << k;
        if (b == chipSent[c] && !refresh) continue;
        Wire.beginTransmission(RELAY_CHIP[c]);
        Wire.write(b);
        const bool ok = Wire.endTransmission() == 0;
        if (ok != chipOk[c]) logEvent(ok ? "relays %u-%u answer again" : "relays %u-%u: no answer from their chip (I2C 0x%02X)",
                                      c * 8 + 1, c * 8 + 8, RELAY_CHIP[c]);
        chipOk[c] = ok;
        chipSent[c] = ok ? b : -1;
    }
}

void outputsInit()
{
    Wire.begin(RELAY_SDA, RELAY_SCL, 100000);
    chipRefreshMs = millis();
    for (uint8_t c = 0; c < 4; c++)                         // quietly: the serial port is not open yet
    {
        Wire.beginTransmission(RELAY_CHIP[c]);
        Wire.write(relayLevel(false) == HIGH ? 0xFF : 0x00);
        chipOk[c] = Wire.endTransmission() == 0;
    }
}
#endif

// Everything off, whatever the polarity setting says (used before a restart or a program update)
void outputsAllOff()
{
    relaysWrite(0);
    relayBits = 0;
    sectionsOn = 0;
}

// Every 200 ms: the hydraulic lift pulses, as the official module does them (raise/lower time in s)
void hydTick()
{
    if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
    {
        lastTrigger = hydLift;
        lowerTimer = 0;
        raiseTimer = 0;
        if (hydLift == 1) lowerTimer = cfg.lowerTime * 5;      // 5 ticks per second
        if (hydLift == 2) raiseTimer = cfg.raiseTime * 5;
    }
    if (raiseTimer) { raiseTimer--; lowerTimer = 0; }
    if (lowerTimer) lowerTimer--;

    // anything odd, or the program silent: no lift movement
    if ((hydLift != 1 && hydLift != 2) || !linkOk)
    {
        lowerTimer = 0;
        raiseTimer = 0;
        lastTrigger = 0;
    }
    isLower = cfg.hydEnable && lowerTimer;
    isRaise = cfg.hydEnable && raiseTimer;
}

// Every 50 ms and right after a PGN 239 or 229: the relays follow the program
void applyOutputs()
{
    const uint32_t now = millis();
    linkOk = have239 && (now - last239Ms) < NO_DATA_TIMEOUT_MS;

    // sections 17-64 come in PGN 229, which the program sends with every 239 when the tool has more than 16
    const bool fresh229 = have229 && (now - last229Ms) < NO_DATA_TIMEOUT_MS;
    sectionsReq = sections239 | (fresh229 ? sections229 & ~(uint64_t)0xFFFF : 0);
    if (sectionsReq)
    {
        const uint8_t top = 64 - __builtin_clzll(sectionsReq);
        if (top > secTop) secTop = top;
    }

    const uint64_t req = linkOk ? sectionsReq : 0;
    const bool other[6] = { false,                          // other[f - FN_OTHER]
                            linkOk && isLower, linkOk && isRaise,
                            linkOk && (tramline & 1), linkOk && (tramline & 2),
                            linkOk && geoStop != 0 };
    uint32_t bits = 0;
    uint64_t open = 0;
    for (uint8_t i = 0; i < NUM_RELAYS; i++)
    {
        const uint8_t f = relayFn(i);
        bool on = false;
        if (f >= 1 && f <= MAX_SECTIONS) on = (req >> (f - 1)) & 1;
        else if (f > FN_OTHER && f <= FN_OTHER + 5) on = other[f - FN_OTHER];
        if (!on) continue;
        bits |= (uint32_t)1 << i;
        if (f <= MAX_SECTIONS) open |= (uint64_t)1 << (f - 1);
    }
    relaysWrite(bits);
    relayBits = bits;
    sectionsOn = open;
}
