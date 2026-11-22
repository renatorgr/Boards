/* zSensors.ino - pressure sensor, flow meter (and the litres applied, kept in flash), optional master switch.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The ESP32 pins take 3.3 V at most, so both 5 V sensors go through a divider (10k in series, then to GND):
//   pressure sensor 0.5-4.5 V, 10k to GND (0.5)  ->  0.25-2.25 V on GPIO 34 (ADC, where it is most accurate)
//   flow meter pulses 0-5 V, 22k to GND (0.69)    ->  0-3.4 V on GPIO 35 (counted by an interrupt; the high
//       level stays above the ESP32's 2.5 V threshold even if the meter has its own pull-up)
// cfg.pressureDivider is that 0.5; change it on the web page if other resistors are used.
// A sensor the page says is not fitted (cfg.sensors) reads 0: without the sensor and
// its divider the pin floats and would show made-up pressure or pulses.

static volatile uint32_t flowPulses = 0;

static void IRAM_ATTR flowIsr() { flowPulses++; }

void sensorsInit()
{
    analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);        // up to about 3.1 V
    pinMode(FLOW_PIN, INPUT);                               // the divider holds the pin, no pull-up needed
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowIsr, FALLING);
#if USE_MASTER_SWITCH
    analogSetPinAttenuation(MASTER_PIN, ADC_11db);
#endif
}

// 20 times a second
void readSensors()
{
    if (cfg.sensors & SENSOR_PRESSURE)
    {
        const uint32_t mv = analogReadMilliVolts(PRESSURE_PIN);
        sensorVolts = mv / 1000.0f / cfg.pressureDivider;  // back to the sensor's own volts
        float bar = (sensorVolts - 0.5f) / 4.0f * cfg.pressureMaxBar;
        if (bar < 0) bar = 0;
        pressureBar += (bar - pressureBar) * 0.1f;          // smoothed so it does not jump
    }
    else
    {
        sensorVolts = 0;
        pressureBar = 0;
    }

#if USE_MASTER_SWITCH
    masterOn = analogReadMilliVolts(MASTER_PIN) > 1000;     // about 6 V at the switch with 47k / 10k
#else
    masterOn = true;                                        // the master switch cuts the valves' 12 V in the wiring
#endif
}

// Once a second: a pressure sensor ticked on the page that reads under 0.3 V three seconds in a row is not there
// (unplugged, a cut wire, no 5 V): a 0.5-4.5 V sensor gives 0.5 V at 0 bar. The log says it when it changes; the alarm
// message on the program's screen and the page say "pressure sensor unplugged?" instead of "low pressure 0.0 bar".
void sensorsTick()
{
    static uint8_t lowFor = 0;
    const bool low = (cfg.sensors & SENSOR_PRESSURE) && sensorVolts < 0.3f;
    lowFor = !low ? 0 : lowFor < 3 ? lowFor + 1 : 3;
    const bool unplugged = lowFor >= 3;
    if (unplugged == pressureUnplugged) return;
    pressureUnplugged = unplugged;
    if (unplugged) logEvent("pressure sensor reads %.2f V: unplugged? (a 0.5-4.5 V sensor gives 0.5 V at 0 bar)", sensorVolts);
    else logEvent("pressure sensor reads %.2f V again", sensorVolts);
}

// Once a second
void updateFlow()
{
    noInterrupts();
    uint32_t n = flowPulses;
    flowPulses = 0;
    interrupts();
    if (!(cfg.sensors & SENSOR_FLOW)) n = 0;

    const float litres = n / cfg.pulsesPerLitre;            // litres in this second
    flowLpm = litres * 60.0f;
    totalLitres += litres;
    totalPulses += n;
}

// The litres applied (and the flow pulses) are kept in flash, in their own namespace so the settings are not written
// again each time: the flow meter's count used to start from 0 at every start, so a module that restarted by itself
// in the middle of a job (a drop of the supply voltage) lost the job's litres. Saved every TOTAL_SAVE_S while they
// change, when they are set to 0, and before a restart that is asked for; a power cut loses TOTAL_SAVE_S at most.
// The litres go in as whole centilitres: two 32-bit numbers take two entries of the flash's table, where the core keeps
// a float as a blob (three). About 2900 saves on a day of 8 h spraying; the flash's wear levelling spreads them over its
// pages, so it lasts decades.
static float    savedLitres = 0;
static uint32_t savedPulses = 0;
static uint32_t savedAtMs = 0;

void totalLoad()
{
    prefs.begin("total", true);
    totalLitres = prefs.getUInt("cl", 0) / 100.0f;
    totalPulses = prefs.getUInt("pulses", 0);
    prefs.end();
    savedLitres = totalLitres;
    savedPulses = totalPulses;
    savedAtMs = millis();
}

void totalSave()
{
    if (totalLitres == savedLitres && totalPulses == savedPulses) return;   // nothing new: the flash is left alone
    const float cl = totalLitres * 100.0f;
    prefs.begin("total", false);
    prefs.putUInt("cl", cl > 0 && cl < 4.0e9f ? (uint32_t)lroundf(cl) : 0);
    prefs.putUInt("pulses", totalPulses);
    prefs.end();
    savedLitres = totalLitres;
    savedPulses = totalPulses;
    savedAtMs = millis();
}

// Once a second
void totalTick()
{
    if (millis() - savedAtMs >= TOTAL_SAVE_S * 1000UL) totalSave();
}
