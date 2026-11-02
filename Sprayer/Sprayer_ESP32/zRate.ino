/* zRate.ino - application rate (L/ha), the pressure for the target rate, and the sprayer alarms.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The sums and the alarm rules are those of SprayerService and the sprayer strip in the AgOpenWeb fork
   (renatorgr/AgOpenWeb, branch pulverizador), so the module and the program agree.
   2026-11-02 (Claude): new in v0.10.0.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Once a second, after the flow of the last second is known:
//   open width   = the widths (PGN 235) of the sections the relays have open
//   open nozzles = per open section, its width / nozzle spacing, rounded
//   L/ha         = L/min x 600 / (km/h x open width in m); not below 1 km/h (stopped) or without the widths
//   expected     = open nozzles x nozzle flow x sqrt(pressure / chart pressure): flow grows with the square root
//                  of the pressure
//   needed bar   = the pressure that gives the target: chart pressure x (L/min needed / (nozzles x nozzle flow))^2,
//                  or from the flow and pressure measured when the nozzle flow is 0
//   hint km/h    = when the needed pressure is above the maximum (or below the minimum): the speed that gives
//                  the target at that limit, km/h x sqrt(limit / needed)
// An alarm comes on once it has lasted ALARM_CONFIRM_S seconds, and the flow is not checked for as long again
// after the sections change, while the valves open and the flow meter catches up.

#define MIN_KMH_FOR_RATE    1.0f    // below this the tractor counts as stopped: no L/ha
#define NO_FLOW_LPM         0.5f    // less than this counts as no flow (the flow meter's noise)

static uint8_t  alarmHeld[5];       // seconds in a row per alarm bit
static uint64_t lastOpen = ~(uint64_t)0;
static uint8_t  sinceChange = 0;

static bool hasPressure() { return cfg.sensors & SENSOR_PRESSURE; }
static bool hasFlow()     { return cfg.sensors & SENSOR_FLOW; }

void rateTick()
{
    // the open sections' width and nozzles
    float width = 0;
    uint16_t nozzles = 0;
    for (uint8_t i = 0; i < MAX_TOOL_SECTIONS && i < cfg.toolSections; i++)
    {
        if (!((sectionsOn >> i) & 1)) continue;
        const float w = cfg.sectionCm[i] / 100.0f;
        width += w;
        if (cfg.nozzleSpacing > 0) nozzles += (uint16_t)lroundf(w / cfg.nozzleSpacing);
    }
    openWidthM = width;
    openNozzles = nozzles > 255 ? 255 : nozzles;

    const float p = hasPressure() ? pressureBar : 0;
    const bool chart = cfg.nozzleFlow > 0 && cfg.nozzleBar > 0;
    expectedLpm = chart && hasPressure() && nozzles ? nozzles * cfg.nozzleFlow * sqrtf(p / cfg.nozzleBar) : -1;

    // actual rate, and the pressure that gives the target at this speed
    const float kmh = linkOk ? gpsKmh : 0;
    const bool moving = kmh >= MIN_KMH_FOR_RATE && width > 0;
    rateLha = moving && hasFlow() ? flowLpm * 600.0f / (kmh * width) : -1;
    needBar = -1;
    hintKmh = -1;
    if (moving && cfg.targetRate > 0)
    {
        const float needLpm = cfg.targetRate * kmh * width / 600.0f;
        if (chart && nozzles) needBar = cfg.nozzleBar * powf(needLpm / (nozzles * cfg.nozzleFlow), 2);
        else if (hasFlow() && hasPressure() && flowLpm > NO_FLOW_LPM && p > 0) needBar = p * powf(needLpm / flowLpm, 2);
        if (needBar > 0 && cfg.maxBar > 0 && needBar > cfg.maxBar) hintKmh = kmh * sqrtf(cfg.maxBar / needBar);
        else if (needBar > 0 && cfg.minBar > 0 && needBar < cfg.minBar) hintKmh = kmh * sqrtf(cfg.minBar / needBar);
    }

    // alarms
    if (sectionsOn != lastOpen) sinceChange = 0;
    else if (sinceChange < 255) sinceChange++;
    lastOpen = sectionsOn;
    const bool settled = sinceChange >= ALARM_CONFIRM_S;
    const bool spraying = sectionsOn != 0;
    const float tol = (cfg.flowTolerance < 1 ? 1 : cfg.flowTolerance) / 100.0f;
    uint8_t raw = 0;
    if (hasPressure() && spraying && cfg.minBar > 0 && p < cfg.minBar) raw |= ALARM_PRESSURE_LOW;
    if (hasPressure() && cfg.maxBar > 0 && p > cfg.maxBar) raw |= ALARM_PRESSURE_HIGH;
    if (hasFlow() && spraying && settled && expectedLpm > NO_FLOW_LPM)
    {
        if (flowLpm < expectedLpm * (1 - tol)) raw |= ALARM_FLOW_LOW;
        else if (flowLpm > expectedLpm * (1 + tol)) raw |= ALARM_FLOW_HIGH;
    }
    if (hasFlow() && !spraying && settled && flowLpm > NO_FLOW_LPM) raw |= ALARM_FLOW_CLOSED;

    uint8_t on = 0;
    for (uint8_t b = 0; b < 5; b++)
    {
        const uint8_t bit = 1 << b;
        alarmHeld[b] = (raw & bit) ? (alarmHeld[b] < 255 ? alarmHeld[b] + 1 : 255) : 0;
        if (alarmHeld[b] >= ALARM_CONFIRM_S) on |= bit;
    }
    if (on & ~sprayAlarms) logEvent("alarm: %s%s%s%s%s", on & ALARM_PRESSURE_LOW & ~sprayAlarms ? "low pressure " : "",
                                    on & ALARM_PRESSURE_HIGH & ~sprayAlarms ? "high pressure " : "",
                                    on & ALARM_FLOW_LOW & ~sprayAlarms ? "flow too low " : "",
                                    on & ALARM_FLOW_HIGH & ~sprayAlarms ? "flow too high " : "",
                                    on & ALARM_FLOW_CLOSED & ~sprayAlarms ? "flow with the sections off" : "");
    sprayAlarms = on;
}
