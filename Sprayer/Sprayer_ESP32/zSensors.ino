/* zSensors.ino - pressure sensor, flow meter, optional master switch.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The ESP32 pins take 3.3 V at most, so both 5 V sensors go through a 10k / 15k divider (0.6):
//   pressure sensor 0.5-4.5 V  ->  0.3-2.7 V on GPIO 34 (ADC, calibrated by the ESP32 itself)
//   flow meter pulses 0-5 V    ->  0-3 V on GPIO 35 (counted by an interrupt)
// cfg.pressureDivider is that 0.6; change it on the web page if other resistors are used.

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
    const uint32_t mv = analogReadMilliVolts(PRESSURE_PIN);
    sensorVolts = mv / 1000.0f / cfg.pressureDivider;      // back to the sensor's own volts
    float bar = (sensorVolts - 0.5f) / 4.0f * cfg.pressureMaxBar;
    if (bar < 0) bar = 0;
    pressureBar += (bar - pressureBar) * 0.1f;              // smoothed so it does not jump

#if USE_MASTER_SWITCH
    masterOn = analogReadMilliVolts(MASTER_PIN) > 1000;     // about 6 V at the switch with 47k / 10k
#else
    masterOn = true;                                        // the master switch cuts the valves' 12 V in the wiring
#endif
}

// Once a second
void updateFlow()
{
    noInterrupts();
    const uint32_t n = flowPulses;
    flowPulses = 0;
    interrupts();

    const float litres = n / cfg.pulsesPerLitre;            // litres in this second
    flowLpm = litres * 60.0f;
    totalLitres += litres;
    totalPulses += n;
}
