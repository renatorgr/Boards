/* zOutputs.ino - the relays: sections, hydraulic lift, tramline, geo stop.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Hydraulic lift timing after the official AgOpenGPS Machine_UDP_v5 module by Brian Tee
   (AgOpenGPS-Official/Boards, GPL v3).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Each relay has a function from the program (PGN 236, kept in cfg.pin): a section, the hydraulic
// lift, a tramline side or the geo stop. Everything is off while the program is silent.
//
// cfg.invertOutputs (PGN 238, byte 8 bit 0; the official module calls it isRelayActiveHigh):
//   0 = the pin is HIGH while the function is on - right for this board, whose relays switch on HIGH
//   1 = the pin is LOW while the function is on (relay boards that switch on LOW)
// The program sends this setting at start-up and when the machine profile is loaded, so the
// "relay active high" box in its machine settings must stay unticked for this board.

static uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

void outputsInit()
{
    for (uint8_t i = 0; i < NUM_RELAYS; i++)
    {
        digitalWrite(RELAY_PIN[i], LOW);
        pinMode(RELAY_PIN[i], OUTPUT);
    }
}

// Everything off, whatever the polarity setting says (used before a restart or a program update)
void outputsAllOff()
{
    for (uint8_t i = 0; i < NUM_RELAYS; i++) digitalWrite(RELAY_PIN[i], cfg.invertOutputs ? HIGH : LOW);
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

// Every 50 ms and right after a PGN 239: the relays follow the program
void applyOutputs()
{
    linkOk = have239 && (millis() - last239Ms) < NO_DATA_TIMEOUT_MS;

    bool state[FN_GEO_STOP];                                // state[f - 1] = function f on
    const uint16_t req = (linkOk && masterOn) ? sectionsReq : 0;
    for (uint8_t s = 0; s < 16; s++) state[s] = bitRead(req, s);
    state[FN_HYD_LOWER - 1] = linkOk && isLower;
    state[FN_HYD_RAISE - 1] = linkOk && isRaise;
    state[FN_TRAM_RIGHT - 1] = linkOk && bitRead(tramline, 0);
    state[FN_TRAM_LEFT - 1]  = linkOk && bitRead(tramline, 1);
    state[FN_GEO_STOP - 1]   = linkOk && geoStop != 0;

    uint8_t bits = 0;
    uint16_t open = 0;
    for (uint8_t i = 0; i < NUM_RELAYS; i++)
    {
        const uint8_t f = cfg.pin[i];
        const bool on = f >= 1 && f <= FN_GEO_STOP && state[f - 1];
        digitalWrite(RELAY_PIN[i], (on != (cfg.invertOutputs != 0)) ? HIGH : LOW);
        if (on) bitSet(bits, i);
        if (on && f <= 16) bitSet(open, f - 1);
    }
    relayBits = bits;
    sectionsOn = open;
}
