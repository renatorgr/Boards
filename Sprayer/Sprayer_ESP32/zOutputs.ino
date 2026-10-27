/* zOutputs.ino - the relays: sections, hydraulic lift, tramline, geo stop.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Hydraulic lift timing after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3).
   2026-10-27 (Claude): up to 64 sections (PGN 229); the seed drill's KC868-A32 moved to Seeder/Seeder_ESP32.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Each relay has a function from the program (PGN 236, kept in cfg.pin): a section, the hydraulic
// lift, a tramline side or the geo stop. Everything is off while the program is silent.
//
// RELAY_ON_HIGH (Sprayer_ESP32.ino) says how the relays of the chosen board close: on a HIGH pin
// (ESP32_Relay_X8) or on a LOW pin (the usual relay modules). cfg.invertOutputs (PGN 238, byte 8
// bit 0; the official module calls it isRelayActiveHigh) turns that round: 0 = as built, 1 = inverted.
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
    if (i >= 24) return 0;
    const uint8_t p = cfg.pin[i];
    if (p >= PIN_HYD_LOWER && p <= PIN_GEO_STOP) return FN_OTHER + (p - PIN_HYD_LOWER + 1);
    return p <= 16 ? p : 0;
}

static void relaysWrite(uint32_t bits)
{
    for (uint8_t i = 0; i < NUM_RELAYS; i++) digitalWrite(RELAY_PIN[i], relayLevel((bits >> i) & 1));
}

void outputsInit()
{
    for (uint8_t i = 0; i < NUM_RELAYS; i++)
    {
        digitalWrite(RELAY_PIN[i], relayLevel(false));
        pinMode(RELAY_PIN[i], OUTPUT);
    }
}

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

    const uint64_t req = (linkOk && masterOn) ? sectionsReq : 0;
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
