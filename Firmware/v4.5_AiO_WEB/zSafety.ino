/* zSafety.ino - autosteer safety: WAS check, why autosteer went off, messages on the program's
   screen (PGN 221), hardware watchdog, why the Teensy started.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr (AiO_WEB: the angle and speed causes,
   wasVoltsOk, the crash hook and the longer crash line, the simulator mode on the program's screen,
   the kickout values in %, the loop time for the report).
   The message format (PGN 221) is the one AgOpenGPS reads (UDPComm, case 221) and AgOpenWeb
   reads the same way ("AiO Board Msgs").
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zSafety.ino
//   - WAS check: the wheel angle sensor voltage must stay between 0.2 and 4.8 V. Outside (wire broken,
//     short circuit, plug out, sensor dead) the WAS reads an end stop and the valve would be driven
//     hard the other way: autosteer is turned off and blocked, like a kickout (only the module's valve
//     / motor output: the tractor's own steering, the steering wheel, is not touched). It can be
//     switched off on the web page for a bench without a WAS (the wheel angle then goes to the
//     program as 0 while the WAS reads out of range).
//     Differential input (A/D converter "Differential" in the program): -0.5 to 4.8 V. Below -0.5 V
//     WAS_H is under WAS_L, which happens when nothing is wired to WAS_L (AmpSeal pin 3, it floats near
//     5 V): a 3-wire sensor with the program set to Differential. The reading would be a large angle to
//     one side (-98 deg for -3.67 V), so this is a fault with the hint to choose Single.
//     A plug out or a broken wire can float inside the range: the angle then jumps faster than wheels
//     turn (blocked at once), or reads still while the module pushes at its Max Limit (autosteer off).
//     After a jump, the first push of the next engage is a short test push the angle must follow.
//   - Why autosteer went off: switch, button, pressure / current / encoder kickout, WAS, the program
//     silent. Shown on the program's screen (PGN 221), in the [status] messages and on the web page.
//   - Hardware watchdog (WDOG1): the Teensy restarts if the loop stops for 2 s. 0.5 s before, an
//     interrupt switches the autosteer outputs (valve / motor) off and notes where the firmware was stuck.
//   - Crash (a fault): before the Teensy core's crash handler, faultHook switches the autosteer outputs
//     off at once and notes the code that called the crashed function.
//   - Why the Teensy started (power, restart, watchdog, crash), at boot and on the web page.
// Autosteer.ino calls wasCheck(), wasJumpCheck(), steerStuckCheck(), wasTestFailed(), wasTestIdle(), steerKick(),
// steerOnNote() and sensorViewNote() from its 40 Hz steering loop, and calcSteeringPID() (AutosteerPID.ino)
// wasTestPush(); loop() calls wdtFeed().

#include "zSafety.h"      // SteerOff, BootWhy (included first by the main .ino)
#include "zVersion.h"     // FW_NAME (in the crash and watchdog lines)

// ---------------------------------------------------------------- PGN 221: text on the program's screen
// { 0x80, 0x81, source, 221, text length + 2, seconds to show, 0 = warning colour, text, CRC }
uint32_t hwMsgCount = 0;
char     hwMsgLast[64] = "";

void hwMessage(const char* text, uint8_t seconds, bool warning)
{
    uint8_t b[8 + 60];
    size_t n = strlen(text);
    if (n > 60) n = 60;
    b[0] = 0x80; b[1] = 0x81; b[2] = 0x7E; b[3] = 221;
    b[4] = (uint8_t)(n + 2);
    b[5] = seconds;
    b[6] = warning ? 0 : 1;
    memcpy(b + 7, text, n);
    uint8_t ck = 0;
    for (size_t i = 2; i < 7 + n; i++) ck += b[i];
    b[7 + n] = ck;
    progSend(b, (uint8_t)(8 + n));                 // on the network, or to the program on the XIAO's WiFi (Autosteer.ino)
    hwMsgCount++;
    strncpy(hwMsgLast, text, sizeof(hwMsgLast) - 1);
    hwMsgLast[sizeof(hwMsgLast) - 1] = '\0';
}

// ---------------------------------------------------------------- WAS check
#define WAS_V_PER_COUNT  (6.144f / 16384.0f)   // ADS1115 at +-6.144 V full scale, reading >> 1 (Autosteer.ino)
#define WAS_LOW_V        0.20f
#define WAS_HIGH_V       4.80f
#define WAS_DIFF_LOW_V  -0.50f                 // differential input: WAS_H under WAS_L = nothing on WAS_L (AmpSeal pin 3)
#define WAS_BAD_MS       75                    // out of range for 75 ms (4 readings in a row at 40 Hz, 8 at 100 Hz) = fault
#define WAS_GOOD_MS      1000                  // back in range for 1 s = fault over

int16_t  wasRaw    = 0;          // ADS1115 reading >> 1, before the offset (about 2722 per volt)
float    wasVolts  = 0;
bool     wasOut    = false;      // out of range now
bool     wasBlock  = false;      // autosteer blocked by it (check on)
uint32_t wasFaults = 0;
static uint32_t wasBadMs = 0, wasGoodMs = 0;   // since when out of range / in range (0 = not now)

// The time a timer started, for timers where 0 means "not running": never 0 and never ahead of now.
// (millis() | 1 is one millisecond ahead on an even millisecond; "now - timer" is unsigned, so in the same
// loop pass it read four billion and the time looked over at once: until 17 Nov 2026 the kickout delay of
// Autosteer.ino and the two times of the WAS check below did not count on an even millisecond.)
uint32_t msStamp(uint32_t now)
{
    return now != 0 ? now : 0xFFFFFFFFu;       // (0xFFFFFFFF is the millisecond before 0)
}

// Called with every WAS reading (40 or 100 Hz, Autosteer.ino). True: autosteer must stay off.
bool wasCheck(int16_t raw)
{
    wasRaw   = raw;
    wasVolts = raw * WAS_V_PER_COUNT;
    const bool single = steerConfig.SingleInputWAS;
    const bool bad = single ? (wasVolts < WAS_LOW_V || wasVolts > WAS_HIGH_V)
                            : (wasVolts < WAS_DIFF_LOW_V || wasVolts > WAS_HIGH_V);
    // Differential input and WAS_H well under WAS_L: nothing on WAS_L (AmpSeal pin 3), it floats near 5 V.
    // That is a 3-wire sensor (signal on pin 2, ground on pin 4) with the program's A/D converter set to
    // Differential: the hint names the setting to change (Single), or the other wiring (ground to pin 3 too).
    const bool noLow = !single && wasVolts < WAS_DIFF_LOW_V;
    const uint32_t now = millis();
    if (bad) { wasGoodMs = 0; if (!wasBadMs) wasBadMs = msStamp(now); }
    else     { wasBadMs = 0;  if (!wasGoodMs) wasGoodMs = msStamp(now); }

    if (!wasOut && wasBadMs && now - wasBadMs >= WAS_BAD_MS)
    {
        wasOut = true;
        wasFaults++;
        if (!cfg.wasCheckOff)
        {
            wasBlock = true;
            char m[64];
            if (noLow)
            {
                StatusLog.line("[status] WAS: %.2f V, negative on the differential input: nothing on WAS_L (AmpSeal pin 3)? Choose Single in the program (A/D converter) or wire the sensor's ground to pin 3 - autosteer blocked",
                               wasVolts);
                snprintf(m, sizeof(m), "WAS %.2f V differential: pin 3 empty? Choose Single", wasVolts);
            }
            else
            {
                StatusLog.line("[status] WAS: %.2f V, out of the %.1f-%.1f V range (wire, plug or sensor?) - autosteer blocked",
                               wasVolts, single ? WAS_LOW_V : WAS_DIFF_LOW_V, WAS_HIGH_V);
                snprintf(m, sizeof(m), "WAS fault (%.2f V): autosteer blocked", wasVolts);
            }
            hwMessage(m, 8, true);
        }
        else
            StatusLog.line("[status] WAS: %.2f V, out of range - the WAS check is off (bench), autosteer not blocked, wheel angle sent as 0 deg", wasVolts);
    }
    else if (wasOut && !wasBlock && !cfg.wasCheckOff)              // the check was switched on during a fault
    {
        wasBlock = true;
        StatusLog.line("[status] WAS: %.2f V, out of range - autosteer blocked", wasVolts);
    }
    else if (wasOut && wasGoodMs && now - wasGoodMs >= WAS_GOOD_MS)
    {
        wasOut = false;
        const bool switchStillOn = steerConfig.SteerSwitch && digitalRead(STEERSW_PIN) == LOW;
        StatusLog.line("[status] WAS: %.2f V, back in range%s", wasVolts,
                       !wasBlock ? "" : switchStillOn ? " - turn the steer switch off and on again to engage autosteer" : " - autosteer can be engaged again");
    }
    // The block ends with the reading back in range and, with a steer switch, the switch seen off
    // (so autosteer does not come back by itself while the switch is still on).
    if (wasBlock && !wasOut && (!steerConfig.SteerSwitch || digitalRead(STEERSW_PIN) == HIGH)) wasBlock = false;
    if (cfg.wasCheckOff) wasBlock = false;
    return wasBlock;
}

// ---------------------------------------------------------------- WAS jump, and wheels that do not answer
// A WAS plug out or a broken signal wire is not always out of range: the open input floats, and it can settle
// anywhere inside 0.2-4.8 V (a Puma, 19 Nov 2026: 3.93 V with the plug out, an angle of 29 deg, no fault). The
// voltage check above cannot tell that from a sensor; the angle can, in two ways:
//   - a jump: the angle changes faster than wheels turn, as the input runs to its floating voltage when the
//     plug comes out (or back to the sensor's when it goes in). Autosteer is blocked at once, also standing
//     still with autosteer off, and the program's screen says it. The block ends 3 s after the last jump, but
//     the plug may still be out then: the first push of the next engage is a short, gentle test push, and the
//     module steers on only when the angle follows it (wasTestPush). Else autosteer goes off again, and the
//     next engage tests again. Only the wheels answering a push ends it: a still or drifting reading does not.
//   - wheels that do not follow: the module pushes at its Max Limit and the angle does not move towards the
//     set point. A floating input reads a still angle while the wheels turn: autosteer goes off.
// Both belong to the WAS check: off with it (a bench without a WAS), and not used with a Keya as the WAS.
#define WAS_JUMP_DEG      20.0f   // a change this large...
#define WAS_JUMP_WIN_MS   150     // ...within this time (133 deg/s) is a jump: wheels turn 25-40 deg/s under
                                  //    autosteer, about 80 with a fast hand on a quick steering...
#define WAS_JUMP_KEEP_MS  50      // ...when it lasts this long (one bad reading comes back: not a jump)
#define WAS_JUMP_HOLD_MS  3000    // the block ends this long after the last jump (with a steer switch, once it is off)
#define WAS_HIST_N        20      // readings kept: more than WAS_JUMP_WIN_MS at 100 Hz
#define STUCK_MS          500     // pushing hard this long, the wheels...
#define STUCK_FIRST_MS    800     // (from the start of a push: the valve or motor takes up to 0.3 s to react)
#define STUCK_DEG         1.0f    // ...must move this far towards the set point
#define STUCK_PWM_PCT     90      // pushing hard: the PID's output at 90 % of the Max Limit (highPWM) or more
#define WAS_TEST_PCT      33      // the test push after a jump: Min To Move + a third of the way to the Max Limit...
#define WAS_TEST_MS       700     // ...for at most this long (a valve or motor takes up to 0.3 s to react)...
#define WAS_TEST_DEG      1.0f    // ...and the angle must move this far the pushed way, two readings in a row (as STUCK_DEG)

bool     wasJumpBlock = false;    // autosteer blocked by a jump
uint32_t wasJumps     = 0;        // jumps since start-up
float    wasJumpDeg   = 0;        // the last one, degrees
bool     stuckBlock   = false;    // autosteer turned off by wheels that did not follow, a steer switch still on
bool     wasTestNeed  = false;    // a jump seen, and the wheels not yet seen to answer a test push since
static uint32_t wasJumpMs = 0;    // when the last jump was seen
static int16_t  histRaw[WAS_HIST_N];
static uint32_t histMs[WAS_HIST_N];
static uint8_t  histHead = 0, histCount = 0;   // the next place to write, readings kept
static uint32_t candMs = 0;       // a change of WAS_JUMP_DEG seen, waiting WAS_JUMP_KEEP_MS (0 = none)
static int16_t  candFrom = 0;     // the reading it jumped from...
static int8_t   candSide = 0;     // ...and which way
static float    stepPeak[2] = {0, 0};   // the largest change within WAS_JUMP_WIN_MS: this 10 s window and the one before
static uint32_t stepPeakMs = 0;
// the test push after a jump (wasTestPush below)
static bool     testOn   = false; // a test push under way (from the first push of an engage)
static uint32_t testUsed = 0;     // ms it has pushed so far (a pause below the minimum speed does not count)
static uint32_t testLast = 0;     // the last loop that pushed (0 = paused, or not started)
static float    testFrom = 0;     // the wheel angle when it started
static int8_t   testDir  = 0;     // the way the angle must go: +1 / -1
static int16_t  testPwm  = 0;     // the push, without its sign
static uint8_t  testSeen = 0;     // readings in a row at WAS_TEST_DEG or more that way
static bool     testFail = false; // the angle did not follow: autosteer off (wasTestFailed)

static float wasCpd()             // the program's counts per degree (PGN 252)
{
    return steerSettings.steerSensorCounts >= 1.0f ? steerSettings.steerSensorCounts : 1.0f;
}

// The largest change of the angle within WAS_JUMP_WIN_MS in the last 10 to 20 s (web page: how fast a plug
// pulled out moves the reading, against what the wheels do)
float wasStepPeak() { return stepPeak[0] > stepPeak[1] ? stepPeak[0] : stepPeak[1]; }

// The texts of the jump check (rare: in flash, not in RAM1). what: 0 a jump blocks autosteer, 1 another jump
// while it blocks, 2 the block is over.
FLASHMEM void wasJumpSay(uint8_t what, int16_t from, int16_t to)
{
    const float v0 = from * WAS_V_PER_COUNT, v1 = to * WAS_V_PER_COUNT;
    if (what == 0)
    {
        StatusLog.line("[status] WAS: the angle jumped %.1f deg, more than %.0f in %.2f s (%.2f V to %.2f V): faster than wheels turn - WAS plug, wire or sensor? - autosteer blocked",
                       wasJumpDeg, WAS_JUMP_DEG, WAS_JUMP_WIN_MS / 1000.0f, v0, v1);
        char m[64];
        snprintf(m, sizeof(m), "WAS jumped %d deg: plug or wire? Autosteer blocked", (int)(wasJumpDeg + 0.5f));
        hwMessage(m, 8, true);
    }
    else if (what == 1)
        StatusLog.line("[status] WAS: the angle jumped again, %.1f deg (%.2f V to %.2f V) - autosteer stays blocked", wasJumpDeg, v0, v1);
    else
        StatusLog.line("[status] WAS: no jump for %u s - autosteer can be engaged again: its first push is a short test, and it steers on only if the wheel angle follows",
                       (unsigned)(WAS_JUMP_HOLD_MS / 1000));
}

// Called with every WAS reading after wasCheck() (40 or 100 Hz, Autosteer.ino). True: autosteer must stay off.
bool wasJumpCheck(int16_t raw)
{
    const uint32_t now = millis();
    const float cpd = wasCpd();
    if (cfg.wasCheckOff || wasBadMs) { histCount = 0; candMs = 0; }   // the check off, or the voltage out of range (wasCheck's)
    else
    {
        // the reading furthest from this one within WAS_JUMP_WIN_MS
        int16_t farRaw = raw;
        for (uint8_t k = 0; k < histCount; k++)
        {
            const uint8_t i = (uint8_t)((histHead + WAS_HIST_N - 1 - k) % WAS_HIST_N);
            if (now - histMs[i] > WAS_JUMP_WIN_MS) break;
            if (abs(histRaw[i] - raw) > abs(farRaw - raw)) farRaw = histRaw[i];
        }
        const float stepDeg = abs(farRaw - raw) / cpd;
        if (stepDeg > stepPeak[0]) stepPeak[0] = stepDeg;
        if (now - stepPeakMs >= 10000) { stepPeakMs = now; stepPeak[1] = stepPeak[0]; stepPeak[0] = stepDeg; }

        if (candMs)
        {
            // a jump stays: the reading keeps that far from where it jumped from, on the same side
            const bool stays = (raw - candFrom) * candSide > 0 && abs(raw - candFrom) / cpd >= WAS_JUMP_DEG;
            if (!stays)
            {
                candMs = 0;
                histCount = 0;            // a bad reading: forget it, or the next readings would "jump" from it
            }
            else if (now - candMs >= WAS_JUMP_KEEP_MS)
            {
                candMs = 0;
                histCount = 0;            // the next jump is measured from here
                wasJumpDeg = abs(raw - candFrom) / cpd;
                wasJumps++;
                wasJumpSay(wasJumpBlock ? 1 : 0, candFrom, raw);
                wasJumpBlock = true;
                wasTestNeed = true;       // and after the block, the wheels must answer a test push (wasTestPush)...
                testOn = false;           // ...from the next engage: a test under way does not count what the jump moved
                wasJumpMs = msStamp(now);
            }
        }
        else if (stepDeg >= WAS_JUMP_DEG)
        {
            candMs = msStamp(now);
            candFrom = farRaw;
            candSide = raw > farRaw ? 1 : -1;
        }
        histRaw[histHead] = raw;
        histMs[histHead] = now;
        histHead = (uint8_t)((histHead + 1) % WAS_HIST_N);
        if (histCount < WAS_HIST_N) histCount++;
    }
    // The block ends WAS_JUMP_HOLD_MS after the last jump and, with a steer switch, once the switch is seen off
    // (as the WAS check's). The plug may still be out then: wasTestNeed stays, so the next engage starts with a
    // test push (wasTestPush below), and autosteer goes off again when the angle does not follow it.
    if (wasJumpBlock && now - wasJumpMs >= WAS_JUMP_HOLD_MS && (!steerConfig.SteerSwitch || digitalRead(STEERSW_PIN) == HIGH))
    {
        wasJumpBlock = false;
        wasJumpSay(2, 0, 0);
    }
    if (cfg.wasCheckOff) { wasJumpBlock = false; wasTestNeed = false; }
    return wasJumpBlock;
}

// ---------------------------------------------------------------- the test push after a jump
// The texts of the test push (rare: in flash). ok: the angle followed.
FLASHMEM void wasTestSay(bool ok, float moved, uint32_t ms)
{
    if (ok)
    {
        StatusLog.line("[status] WAS: the wheel angle followed the test push (PWM %d: %.1f deg in %.2f s) - the WAS reads the wheels again, autosteer steers on",
                       testPwm, moved, ms / 1000.0f);
        hwMessage("WAS OK: the wheels followed the test push", 4, false);
    }
    else
        StatusLog.line("[status] WAS: test push PWM %d for %.1f s and the wheel angle moved %.1f deg the pushed way (%.1f needed): the WAS does not see the wheels - plug or wire? - autosteer off; the next engage tests again",
                       testPwm, ms / 1000.0f, moved, WAS_TEST_DEG);
}

// Called by the PID (AutosteerPID.ino calcSteeringPID) every steering loop while autosteer steers, with its output
// (+ = the angle must go down, before the motor direction) and the wheel angle and set point in degrees. While a
// jump is not cleared (wasTestNeed), the PID's output is replaced by the test push: gentle (a third of the way from
// Min To Move to the Max Limit), the PID's way, for at most WAS_TEST_MS. When the angle moves WAS_TEST_DEG that
// way, the test is over and the PID steers from then on; else the output goes to 0 and wasTestFailed() turns
// autosteer off. A test with the plug still out moves the real wheels a few degrees at most.
int16_t wasTestPush(int16_t pwm, float angle, float setPoint)
{
    if (wasTestNeed && (cfg.wasCheckOff || (isKeya && !steerConfig.SingleInputWAS))) wasTestNeed = false;   // no jump check then
    if (!wasTestNeed) { testOn = false; return pwm; }
    // the PID asks for nothing (below the minimum steering speed it does not push): no push, the test pauses. The
    // last loop's push lasted until now and counts (else a PID output that flickers to 0 every other loop would
    // push for ever); the readings in a row start again, so both come while it pushes
    if (pwm == 0)
    {
        if (testLast) testUsed += millis() - testLast;
        testLast = 0;
        testSeen = 0;
        return 0;
    }
    const uint32_t now = millis();
    if (!testOn)
    {
        const int hi = steerSettings.highPWM, lo = steerSettings.minPWM;
        int t = lo + (hi > lo ? (hi - lo) * WAS_TEST_PCT / 100 : 0);
        if (t > hi) t = hi;                               // never above the Max Limit (Min To Move above it)
        testPwm = (int16_t)t;
        // the PID's way; but when the program asks past its maximum wheel angle (PGN 247; 35 deg without it) and the
        // wheels are already well over to that side, they may stand at the stop (the steer wizard's Maximum Steering
        // Angle step): then towards straight ahead. "Well over": 10 deg short of it, and at least half of it (a small
        // maximum angle in the program would make any angle that side count)
        const float lockDeg = aowMaxSteerDeg ? aowMaxSteerDeg + 2.0f : 35.0f;
        const bool atStop = fabsf(setPoint) > lockDeg && angle * setPoint > 0 && fabsf(angle) > fmaxf(lockDeg - 10.0f, lockDeg * 0.5f);
        testDir = atStop ? (angle > 0 ? -1 : 1) : (pwm > 0 ? -1 : 1);
        testFrom = angle;
        testSeen = 0;
        testUsed = 0;
        testLast = 0;
        testOn = true;
    }
    if (testLast) testUsed += now - testLast;             // the time the last loop's push lasted
    testLast = msStamp(now);
    const float moved = (angle - testFrom) * testDir;
    // the angle moved the pushed way, and not by a jump still being judged (wasJumpCheck: a plug going in)
    testSeen = (moved >= WAS_TEST_DEG && !candMs) ? (uint8_t)(testSeen + 1) : 0;
    if (testSeen >= 2)                                    // it follows: the PID steers from this loop on
    {
        wasTestNeed = false;
        testOn = false;
        wasTestSay(true, moved, testUsed);
        return pwm;
    }
    if (testUsed >= WAS_TEST_MS)                          // it did not follow: output 0, autosteer off (wasTestFailed)
    {
        testOn = false;
        testFail = true;
        wasTestSay(false, moved, testUsed);
        return 0;
    }
    return testDir < 0 ? testPwm : (int16_t)-testPwm;    // + = the angle must go down
}

// Called by the steering loop (Autosteer.ino) while autosteer is off: a test push under way is dropped, and the
// next engage starts it again from its own angle
void wasTestIdle()
{
    testOn = false;
    testFail = false;
}

// Called every steering loop after the PID (Autosteer.ino). True once when the angle did not follow the test push:
// autosteer off now (the cause is SO_WAS_TEST); with a steer switch, it stays off until the switch is seen off (as
// after wheels that did not follow, steerStuckBlocked), and the next engage tests again (wasTestNeed stays).
bool wasTestFailed()
{
    if (!testFail) return false;
    testFail = false;
    steerKick(SO_WAS_TEST, testPwm);
    stuckBlock = steerConfig.SteerSwitch;
    return true;
}

// The text when the wheels did not follow (rare: in flash)
FLASHMEM void steerStuckSay(int pwm, uint32_t ms, float from, float to, float setPoint)
{
    StatusLog.line("[status] Autosteer: PWM %d for %.1f s and the wheel angle went from %.1f to %.1f deg (asked %.1f): the wheels did not follow. WAS plug or wire, a hand on the wheel or the wheels at a stop? - autosteer off",
                   pwm, ms / 1000.0f, from, to, setPoint);
}

// Called every steering loop after the PID (Autosteer.ino). on: steering now, with a WAS; pwm: the PID's output
// before the motor direction (+ = the angle must go down); angle and setPoint in degrees, as the program sees
// them. True: autosteer off now (the cause is SO_STUCK).
bool steerStuckCheck(bool on, int16_t pwm, float angle, float setPoint)
{
    static uint32_t pushMs = 0;    // since when the module pushes hard one way (0 = not now)
    static float    pushFrom = 0;  // the wheel angle then
    static int8_t   pushDir = 0;   // the way the angle must go: +1 / -1
    static bool     pushFirst = false;   // the first stretch of this push (the steering's reaction time in it)
    const uint32_t now = millis();
    const int hi = steerSettings.highPWM, lo = steerSettings.minPWM;
    // Pushing hard: the PID at its Max Limit, a large error. Not judged:
    //  - a Max Limit hardly above Min To Move (the wheels may only creep at it);
    //  - a set point past the program's maximum wheel angle (PGN 247; 35 deg when the program does not send it):
    //    the program drives the wheels to a stop on purpose (the steer wizard's Maximum Steering Angle step asks
    //    for 60 deg and waits for them to stand still there);
    //  - the WAS check off (a bench without a WAS: the angle reads still).
    const float lockDeg = aowMaxSteerDeg ? aowMaxSteerDeg + 2.0f : 35.0f;
    const bool hard = on && !cfg.wasCheckOff && hi >= lo + 10 && abs(pwm) * 100 >= hi * STUCK_PWM_PCT && fabsf(setPoint) <= lockDeg;
    if (!hard) { pushMs = 0; return false; }
    const int8_t dir = pwm > 0 ? -1 : 1;
    if (!pushMs || dir != pushDir) { pushMs = msStamp(now); pushFrom = angle; pushDir = dir; pushFirst = true; return false; }
    if ((angle - pushFrom) * dir >= STUCK_DEG) { pushMs = msStamp(now); pushFrom = angle; pushFirst = false; return false; }   // they follow: the next stretch from here
    const uint32_t waitMs = pushFirst ? STUCK_FIRST_MS : STUCK_MS;
    if (now - pushMs < waitMs) return false;
    pushMs = 0;
    steerStuckSay(abs(pwm), waitMs, pushFrom, angle, setPoint);
    steerKick(SO_STUCK, abs(pwm));
    stuckBlock = steerConfig.SteerSwitch;   // a steer switch would turn it on again at once: off until it is seen off
    return true;
}

// Called at the top of the steering loop (Autosteer.ino): a steer switch still on after steerStuckCheck()
// turned autosteer off keeps it off, until the switch is seen off.
bool steerStuckBlocked()
{
    if (stuckBlock && (!steerConfig.SteerSwitch || digitalRead(STEERSW_PIN) == HIGH)) stuckBlock = false;
    return stuckBlock;
}

// The program asks to steer while the WAS check (voltage or jump) or a lost Keya blocks autosteer: say why again on the
// program's screen. wasCheck() and keyaCheck() say it once, when the fault starts, and that is often
// before the program runs (a bench board with no WAS reads about 4.9 V from power-on); after that the
// module just does not steer, and the program only shows its AutoSteer button red. Once each time the
// program's button goes on (PGN 254 status bit 0 from 0 to 1), at most every 3 s. Not when the block
// starts while the program already asks: wasCheck() / keyaCheck() have just said it.
// A fault that is over (the WAS back in range, the Keya answering) blocks on only until a steer switch
// left on is turned off: then that is what it says, not the fault. With several blocks every one gets its
// line in Messages; the program's screen has room for one: a fault still there (Keya lost, WAS jump held, WAS
// out) before one that is over, and of two still there the WAS. A steer switch left on after the wheels did
// not follow (steerStuckCheck) is said the same way as a fault that is over.
FLASHMEM void blockedSay()
{
    static uint32_t saidMs = 0;
    const uint32_t now = millis();
    if (!wasBlock && !keyaBlock && !wasJumpBlock && !stuckBlock) return;
    if (saidMs && now - saidMs < 3000) return;
    saidMs = msStamp(now);
    // A fault still there (live) wins the screen's line over a "back again, switch off and on" line.
    char m[64] = "";
    bool live = false;
    if (stuckBlock)
    {
        snprintf(m, sizeof(m), "Wheels did not follow: steer switch off and on");
        StatusLog.line("[status] The program asks to steer: autosteer stays off, the wheels did not follow the last time - turn the steer switch off and on again (check the WAS plug first)");
    }
    if (keyaBlock)
    {
        if (keyaLost)
        {
            snprintf(m, sizeof(m), "Keya not responding (CAN cable?): autosteer off");
            live = true;
            StatusLog.line("[status] The program asks to steer: autosteer blocked, the Keya motor does not answer");
        }
        else
        {
            if (!live) snprintf(m, sizeof(m), "Keya answers again: steer switch off and on");
            StatusLog.line("[status] The program asks to steer: the Keya motor answers again - turn the steer switch off and on again to engage autosteer");
        }
    }
    if (wasJumpBlock)
    {
        if (now - wasJumpMs < WAS_JUMP_HOLD_MS)
        {
            snprintf(m, sizeof(m), "WAS jumped %d deg: plug or wire? Autosteer blocked", (int)(wasJumpDeg + 0.5f));
            live = true;
            StatusLog.line("[status] The program asks to steer: autosteer blocked, the WAS angle jumped %.1f deg (plug or wire?)", wasJumpDeg);
        }
        else
        {
            if (!live) snprintf(m, sizeof(m), "WAS jump over: steer switch off and on");
            StatusLog.line("[status] The program asks to steer: the WAS jump is over - turn the steer switch off and on again to engage autosteer");
        }
    }
    if (wasBlock)
    {
        if (!wasOut)
        {
            if (!live) snprintf(m, sizeof(m), "WAS in range again: steer switch off and on");
            StatusLog.line("[status] The program asks to steer: the WAS is back in range (%.2f V) - turn the steer switch off and on again to engage autosteer", wasVolts);
        }
        else
        {
            if (!steerConfig.SingleInputWAS && wasVolts < WAS_DIFF_LOW_V)
                snprintf(m, sizeof(m), "WAS %.2f V differential: pin 3 empty? Choose Single", wasVolts);
            else
                snprintf(m, sizeof(m), "WAS fault (%.2f V): autosteer blocked", wasVolts);
            StatusLog.line("[status] The program asks to steer: autosteer blocked by the WAS check (%.2f V)", wasVolts);
        }
    }
    hwMessage(m, 8, true);
}

// Called from the steering loop (40 or 100 Hz, Autosteer.ino), after wasCheck() and keyaCheck()
void blockedAskNote()
{
    static bool asked = false;
    const bool ask = (guidanceStatus & 1) != 0;
    if (ask && !asked && (wasBlock || keyaBlock || wasJumpBlock || stuckBlock)) blockedSay();
    asked = ask;
}

// WAS check off (bench) and the WAS out of range: the wheel angle goes to the program as 0
// degrees (no WAS on the bench reads about 0 V = full lock to one side on the program's screen).
// With the check on, a WAS fault keeps its real reading: autosteer is blocked and the program
// is told why.
bool wasBenchZero()
{
    return cfg.wasCheckOff && wasOut;
}

// AiO_WEB: the last WAS reading was inside the voltage range. The +-50 deg wheel angle cut of
// Autosteer.ino only acts then: a voltage out of range is the WAS check's (above: 3 loops against
// glitches, its own message, and not at all with the check off on the bench).
bool wasVoltsOk()
{
    return wasBadMs == 0;
}

// ---------------------------------------------------------------- pressure / current value for the page
float sensorView = 0;                  // the kickout sensor's value (0-255, as sent to the program), or the pressure input
static float    sensorPeak[2] = {0, 0}; // highest value in this 5 s window and in the one before
static uint32_t sensorPeakMs = 0;
static float    pressFilt = 0;

float pressureFiltered(float sample)  // the pressure input with the same filter as the kickout (Autosteer.ino)
{
    pressFilt = pressFilt * kickKeepPressure + sample * (1 - kickKeepPressure);   // 0.6 / 0.4 per 25 ms (Autosteer.ino steerLoopApply)
    return pressFilt;
}

void sensorViewNote(float v)
{
    sensorView = v;
    if (v > sensorPeak[0]) sensorPeak[0] = v;
    const uint32_t now = millis();
    if (now - sensorPeakMs >= 5000) { sensorPeakMs = now; sensorPeak[1] = sensorPeak[0]; sensorPeak[0] = v; }
}

float sensorPeakNow() { return sensorPeak[0] > sensorPeak[1] ? sensorPeak[0] : sensorPeak[1]; }

// ---------------------------------------------------------------- why autosteer went off
static uint8_t  kickCause = SO_NONE;   // the last kickout (pressure, current, encoder)...
static uint32_t kickMs    = 0;         // ...when...
static int      kickValue = 0;         // ...and its value
uint8_t  offCause = SO_NONE;           // the last time autosteer went off: why...
uint32_t offMs    = 0;                 // ...when...
int      offValue = 0;                 // ...with the value (kickouts)
uint32_t offModuleCount = 0;           // times the module itself turned it off
uint32_t pgn254Ms = 0, pgn252Ms = 0, pgn251Ms = 0;   // last steering data / settings / configuration
static bool steerWasOn = false;

void steerKick(uint8_t cause, int value) { kickCause = cause; kickMs = millis(); kickValue = value; }
void steerNote254() { pgn254Ms = millis(); }
void steerNote252() { pgn252Ms = millis(); }
void steerNote251() { pgn251Ms = millis(); }

const char* offName(uint8_t c)          // for the web page
{
    switch (c)
    {
        case SO_SWITCH:   return "switch";
        case SO_BUTTON:   return "button";
        case SO_PRESSURE: return "pressure";
        case SO_CURRENT:  return "current";
        case SO_ENCODER:  return "encoder";
        case SO_WAS:      return "was";
        case SO_SILENT:   return "silent";
        case SO_PROGRAM:  return "program";
        case SO_SPEED:    return "speed";
        case SO_OTHER:    return "other";
        case SO_KEYA:     return "keya";
        case SO_KEYA_FAULT: return "keyafault";
        case SO_ANGLE:    return "angle";
        case SO_FAST:     return "fast";
        case SO_KEYA_LOST: return "keyalost";
        case SO_WAS_JUMP: return "wasjump";
        case SO_STUCK:    return "stuck";
        case SO_WAS_TEST: return "wastest";
        default:          return "";
    }
}

// The kickout sensor's 0-255 value (and PulseCountMax, the trip point) in %, as AgOpenWeb / AgOpenGPS show them
static int pct255(int v) { return (v * 100 + 127) / 255; }

static void offText(char* b, size_t n, uint8_t c, int v)
{
    const int lim = pct255(steerConfig.PulseCountMax);
    switch (c)
    {
        case SO_SWITCH:   snprintf(b, n, "steer switch"); break;
        case SO_BUTTON:   snprintf(b, n, "steer button"); break;
        case SO_PRESSURE: snprintf(b, n, "pressure kickout (%d %%, limit %d %%)", pct255(v), lim); break;
        case SO_CURRENT:  // a Keya: its own current, in amps (0-255 = 0-8 A, KEYA_AMPS_X in Autosteer.ino)
            if (isKeya) snprintf(b, n, "Keya current kickout (%.1f A, limit %.1f A)", v / KEYA_AMPS_X, steerConfig.PulseCountMax / KEYA_AMPS_X);
            else        snprintf(b, n, "current kickout (%d %%, limit %d %%)", pct255(v), lim);
            break;
        case SO_ENCODER:  snprintf(b, n, "wheel encoder kickout (%d pulses)", v); break;
        case SO_WAS:      snprintf(b, n, "WAS fault (%.2f V)", wasVolts); break;
        case SO_SILENT:   snprintf(b, n, "no steering data from the program for 2.5 s"); break;
        case SO_PROGRAM:  snprintf(b, n, "by the program (its screen says why)"); break;
        case SO_SPEED:    snprintf(b, n, "speed below 0.1 km/h"); break;
        case SO_KEYA:     snprintf(b, n, "Keya kickout: wheel turned by hand (%d rpm)", v); break;
        case SO_KEYA_FAULT: snprintf(b, n, "Keya motor fault (0x%04X)", v & 0xFFFF); break;
        case SO_ANGLE:    snprintf(b, n, "wheel angle %d deg, beyond +-50", v); break;
        case SO_FAST:     snprintf(b, n, "speed %d km/h, above %u", v, (unsigned)cfg.steerMaxKmh); break;
        case SO_KEYA_LOST: snprintf(b, n, "Keya not responding (no heartbeat for 0.5 s)"); break;
        case SO_WAS_JUMP: snprintf(b, n, "WAS jump (%d deg, plug or wire?)", v); break;
        case SO_STUCK:    snprintf(b, n, "wheels did not follow (PWM %d): WAS plug?", v); break;
        case SO_WAS_TEST: snprintf(b, n, "WAS did not follow the test push: plug?"); break;
        default:          snprintf(b, n, "other"); break;
    }
}

// Called at the end of every steering loop (40 or 100 Hz) with "steering now"
void steerOnNote(bool on)
{
    if (on == steerWasOn) return;
    steerWasOn = on;
    const uint32_t now = millis();
    if (on) { kickCause = SO_NONE; return; }

    uint8_t c;
    int v = 0;
    if (wasBlock)                                           c = SO_WAS;
    else if (keyaBlock)                                     c = SO_KEYA_LOST;
    else if (wasJumpBlock)                                  { c = SO_WAS_JUMP; v = (int)(wasJumpDeg + 0.5f); }
    else if (kickCause != SO_NONE && now - kickMs < 500)    { c = kickCause; v = kickValue; }
    else if (pgn254Ms == 0 || now - pgn254Ms > 1000)        c = SO_SILENT;
    else if (steerSwitch == 1 && steerConfig.SteerSwitch)   c = SO_SWITCH;
    else if (steerSwitch == 1 && steerConfig.SteerButton)   c = steerOffByProgram ? SO_PROGRAM : SO_BUTTON;
    else if (!(guidanceStatus & 1) || steerSwitch == 1)     c = SO_PROGRAM;   // no switch / button: it follows the program's button
    else if (gpsSpeed < 0.1f)                               c = SO_SPEED;
    else                                                    c = SO_OTHER;
    offCause = c; offMs = now; offValue = v;
    if (c == SO_PROGRAM)                                    // its button, outside the boundary, speed limits, RTK lost:
    {                                                       // the program's screen says which; the module only sees its bit go to 0
        StatusLog.line("[status] Autosteer OFF: by the program (its screen says why: its button, boundary, speed, RTK)");
        return;
    }
    if (!SO_FROM_MODULE(c)) return;                         // the program knows why

    offModuleCount++;
    char t[72];
    offText(t, sizeof(t), c, v);
    StatusLog.line("[status] Autosteer OFF: %s", t);
    if (c == SO_WAS || c == SO_KEYA_LOST || c == SO_WAS_JUMP) return;   // wasCheck() / keyaCheck() / wasJumpCheck() have put it on the program's screen
    char m[96];
    snprintf(m, sizeof(m), "Autosteer off: %s", t);
    const bool byHand = c == SO_SWITCH || c == SO_BUTTON;
    hwMessage(m, byHand ? 3 : 6, !byHand);
}

// ---------------------------------------------------------------- why the Teensy started
// The i.MX RT reset status (SRC_SRSR) and, after a crash, the Teensy core's crash record
// (CrashReport, at the top of RAM2). After a watchdog reset, the warning interrupt below has left
// in the core's breadcrumbs: a mark, the loop step and the address of the code that was running.
#define BC_MARK      0x57D0600Du        // breadcrumb 4: "the watchdog warning interrupt wrote 5 and 6"
#define BC_FAULT     0xFA017C0Du        // breadcrumb 1: "the crash hook wrote 2 and 3" (faultHookC)
uint8_t  bootWhy     = BOOT_UNKNOWN;
uint32_t bootSrsr    = 0;
uint32_t bootFaultPc = 0, bootCfsr = 0; // crash: the faulting code, the fault status
uint32_t bootFaultLr = 0;               // crash: the code that called the crashed function (0: unknown)
uint32_t bootFaultAddr = 0;             // crash: the memory address it could not use (MMFAR / BFAR)
bool     bootFaultAddrOk = false;
uint32_t bootStuckPc = 0;               // watchdog: the code that was running
uint8_t  bootStep    = 0;               // watchdog: the loop step

volatile uint8_t loopStep = 0;          // set by loop() before each part (see stepName)

const char* stepName(uint8_t s)
{
    switch (s)
    {
        case 1:  return "Keya CAN";
        case 2:  return "USB serial";
        case 3:  return "receiver";
        case 4:  return "NTRIP";
        case 5:  return "radio";
        case 6:  return "IMU";
        case 7:  return "GPS2 port (second F9P / XIAO)";
        case 8:  return "GNSS output";
        case 9:  return "status";
        case 10: return "web page";
        case 11: return "steering";
        case 12: return "LEDs";
        case 13: return "GPS receiver";
        case 14: return "Ethernet stack";
        case 15: return "machine module";
        case 20: return "firmware update";
        default: return "?";
    }
}

const char* bootName(uint8_t w)         // for the web page
{
    switch (w)
    {
        case BOOT_POWER:     return "power";
        case BOOT_UPLOAD:    return "upload";
        case BOOT_RESTART:   return "restart";
        case BOOT_WATCHDOG:  return "watchdog";
        case BOOT_CRASH:     return "crash";
        case BOOT_CRASH_WDT: return "crash-wdt";
        case BOOT_TEMP:      return "temp";
        case BOOT_OTHER:     return "other";
        default:             return "unknown";
    }
}

// First thing in setup(): read and clear the reset status
void bootCapture()
{
    bootSrsr = SRC_SRSR;
    const uint32_t gpr5 = SRC_GPR5;
    const bool crash = (bool)CrashReport;
    if (crash)
    {
        const struct arm_fault_info_struct* fi = (const struct arm_fault_info_struct*)0x2027FF80;
        bootFaultPc = fi->ret;
        bootCfsr    = fi->cfsr;
        if (fi->cfsr & 0x80u)          { bootFaultAddr = fi->mmfar; bootFaultAddrOk = true; }   // MMARVALID
        else if (fi->cfsr & 0x8000u)   { bootFaultAddr = fi->bfar;  bootFaultAddrOk = true; }   // BFARVALID
    }
    volatile struct crashreport_breadcrumbs_struct* bc = (struct crashreport_breadcrumbs_struct*)0x2027FFC0;
    const bool marked = (bootSrsr & SRC_SRSR_WDOG_RST_B) && (bc->bitmask & 0x38) == 0x38 && bc->value[3] == BC_MARK;
    if (marked) { bootStep = (uint8_t)bc->value[4]; bootStuckPc = bc->value[5]; }
    // the crash hook: the caller, and the first faulting code (the core's record holds the last fault,
    // which is in the hook if the hook itself failed)
    if (crash && (bc->bitmask & 0x07) == 0x07 && bc->value[0] == BC_FAULT) { bootFaultLr = bc->value[1]; bootFaultPc = bc->value[2]; }
    bc->bitmask &= ~0x3Fu;                                       // breadcrumbs 1 to 6 used
    arm_dcache_flush((void*)bc, sizeof(struct crashreport_breadcrumbs_struct));
    if (crash) CrashReport.clear();                             // also clears SRC_SRSR and SRC_GPR5
    SRC_SRSR = SRC_SRSR;                                         // write-1-to-clear: the next start sees only its own reason
    SRC_GPR5 = 0;

    if (crash)                                                   bootWhy = (bootSrsr & SRC_SRSR_WDOG_RST_B) ? BOOT_CRASH_WDT : BOOT_CRASH;
    else if (bootSrsr & (SRC_SRSR_WDOG_RST_B | SRC_SRSR_WDOG3_RST_B)) bootWhy = BOOT_WATCHDOG;
    else if (bootSrsr & SRC_SRSR_TEMPSENSE_RST_B)                bootWhy = BOOT_TEMP;
    else if (bootSrsr & SRC_SRSR_LOCKUP_SYSRESETREQ)             bootWhy = gpr5 == 0x0BAD00F1 ? BOOT_CRASH : BOOT_RESTART;
    else if (bootSrsr & (SRC_SRSR_JTAG_RST_B | SRC_SRSR_JTAG_SW_RST)) bootWhy = BOOT_UPLOAD;
    else if (bootSrsr & (SRC_SRSR_IPP_RESET_B | SRC_SRSR_IPP_USER_RESET_B)) bootWhy = BOOT_POWER;
    else                                                         bootWhy = BOOT_OTHER;
}

// After the [boot] line: the reason in the log
void bootReport()
{
    char stuck[80] = "";
    if (bootStep) snprintf(stuck, sizeof(stuck), ", in the %s part of the loop at 0x%08lX", stepName(bootStep), (unsigned long)bootStuckPc);
    // the crash: the caller, the fault status and the address it could not use; then the firmware and
    // the Teensyduino it was built with (the addresses only mean something with that build)
    char from[32] = "", addr[24] = "", built[64];
    if (bootFaultLr)     snprintf(from, sizeof(from), ", called from 0x%08lX", (unsigned long)(bootFaultLr & ~1u));
    if (bootFaultAddrOk) snprintf(addr, sizeof(addr), ", address 0x%08lX", (unsigned long)bootFaultAddr);
#ifdef TEENSYDUINO
    snprintf(built, sizeof(built), "%s, Teensyduino %d.%02d", FW_NAME, TEENSYDUINO / 100, TEENSYDUINO % 100);
#else
    snprintf(built, sizeof(built), "%s", FW_NAME);
#endif
    switch (bootWhy)
    {
        case BOOT_POWER:     StatusLog.line("[boot] Start: power on (or the Teensy's button, or a USB upload)"); break;
        case BOOT_UPLOAD:    StatusLog.line("[boot] Start: after programming (JTAG reset)"); break;
        case BOOT_RESTART:   StatusLog.line("[boot] Start: restart by software (web page, firmware update, or the program's subnet change)"); break;
        case BOOT_WATCHDOG:  StatusLog.line("[boot] Start: WATCHDOG - the firmware stopped for 2 s%s (%s). Please send this line", stuck, built); break;
        case BOOT_CRASH:
        case BOOT_CRASH_WDT: StatusLog.line("[boot] Start: CRASH at 0x%08lX%s (fault 0x%08lX%s), restarted%s (%s). Please send this line",
                                            (unsigned long)bootFaultPc, from, (unsigned long)bootCfsr, addr,
                                            bootWhy == BOOT_CRASH_WDT ? " by the watchdog" : "", built); break;
        case BOOT_TEMP:      StatusLog.line("[boot] Start: the Teensy got too hot (temperature monitor)"); break;
        default:             StatusLog.line("[boot] Start: reason unknown (reset status 0x%lX)", (unsigned long)bootSrsr); break;
    }
}

// ---------------------------------------------------------------- hardware watchdog (WDOG1)
// Timeout (WT + 1) x 0.5 s. Not stopped while a debugger halts the CPU (WDBG). The reset is the
// chip's own (wdog_rst, WDT bit left 0). WDE cannot be cleared once set: switching the watchdog off
// on the web page takes effect at the next start.
#define WDT_WT            3            // 2 s
#define WDT_WICT          1            // warning interrupt 0.5 s before the reset
#define WDOG_WICR_WIE     ((uint16_t)(1 << 15))
#define WDOG_WICR_WTIS    ((uint16_t)(1 << 14))
#define WDOG_WICR_WICT(n) ((uint16_t)((n) & 0xFF))
bool wdtRunning = false;

void wdtFeed()
{
    if (!wdtRunning) return;
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

// ---------------------------------------------------------------- loop time (the report, web page Firmware tab)
// The time between two starts of loop(): the longest since start-up and in the last 5 s window (a firmware
// update or a slow browser show up here too). Since 3 Nov 2026 also where the time goes: loop() calls
// loopAt(step) before each of its parts (stepName), and for each part the report gets the average and the
// longest time per loop in the last whole 5 s window, with the loops per second (loopHz). On the board with
// two F9Ps and a BNO085 the loop ran 850 times a second against 24 000 on the bench board, nobody knew why.
uint32_t loopCount = 0, loopMaxUs = 0, loopMax5Us = 0, loopHz = 0;
uint32_t steerLoopHz = 0;                                       // the steering loops per second (steerLoopN, Autosteer.ino)
uint32_t loopStepAvgUs[LOOP_STEPS] = {}, loopStepMaxUs[LOOP_STEPS] = {};
static uint32_t loopLastUs = 0, loopWinMaxUs = 0, loopWinMs = 0, loopWinCount = 0, stepT0Us = 0;
static uint32_t stepSumUs[LOOP_STEPS] = {}, stepWinMaxUs[LOOP_STEPS] = {};

// AiO_WEB 4 Nov 2026: the chip's temperature from the Teensy 4.1's own sensor (tempmonGetTemp(), Teensyduino), once a
// second. Shown with the clock (F_CPU_ACTUAL, the IDE's CPU Speed) on the Firmware tab and kept in the report, with the
// highest value since start-up. From CPU_HOT_C a warning: the chip is rated to 95 C, where Teensyduino's own protection
// restarts it, so 80 C means "look at the box" (sun, no air), not "stop".
#define CPU_HOT_C        80.0f
#define CPU_HOT_CLEAR_C  75.0f
float cpuTemp = 0, cpuTempMax = 0;
bool  cpuHot = false;
static uint32_t cpuTempMs = 0;

static void cpuTempTick(uint32_t ms)
{
    if (ms - cpuTempMs < 1000) return;
    cpuTempMs = ms;
    cpuTemp = tempmonGetTemp();
    if (cpuTemp > cpuTempMax) cpuTempMax = cpuTemp;
    if (!cpuHot && cpuTemp >= CPU_HOT_C)
    {
        cpuHot = true;
        StatusLog.line("[status] Teensy at %.0f C (%lu MHz): hot - check the module's box (sun, air); the chip restarts itself at about 95 C",
                       (double)cpuTemp, (unsigned long)(F_CPU_ACTUAL / 1000000));
        hwMessage("Teensy hot: check the module's box", 10, true);
    }
    else if (cpuHot && cpuTemp <= CPU_HOT_CLEAR_C)
    {
        cpuHot = false;
        StatusLog.line("[status] Teensy at %.0f C: cooled down", (double)cpuTemp);
    }
}

// The part that was running (loopStep) ends now: its time goes to its bucket
static void stepAccount(uint32_t us)
{
    const uint8_t p = loopStep < LOOP_STEPS ? loopStep : 0;
    const uint32_t d = us - stepT0Us;
    stepSumUs[p] += d;
    if (d > stepWinMaxUs[p]) stepWinMaxUs[p] = d;
    stepT0Us = us;
}

void loopAt(uint8_t step)
{
    stepAccount(micros());
    loopStep = step;
}

void loopTick()
{
    const uint32_t us = micros();
    if (loopCount)
    {
        const uint32_t d = us - loopLastUs;
        if (d > loopMaxUs)    loopMaxUs = d;
        if (d > loopWinMaxUs) loopWinMaxUs = d;
        stepAccount(us);                                    // the last part of the previous loop (the LEDs)
    }
    else stepT0Us = us;
    loopLastUs = us;
    loopCount++;
    loopWinCount++;
    const uint32_t ms = millis();
    cpuTempTick(ms);
    if (ms - loopWinMs >= 5000)
    {
        loopHz = loopWinCount * 1000UL / (ms - loopWinMs);
        steerLoopHz = steerLoopN * 1000UL / (ms - loopWinMs);
        steerLoopN = 0;
        for (uint8_t k = 0; k < LOOP_STEPS; k++)
        {
            loopStepAvgUs[k] = stepSumUs[k] / loopWinCount;
            loopStepMaxUs[k] = stepWinMaxUs[k];
            stepSumUs[k] = 0;
            stepWinMaxUs[k] = 0;
        }
        loopWinMs = ms; loopMax5Us = loopWinMaxUs; loopWinMaxUs = 0; loopWinCount = 0;
    }
}

// Valve / motor driver outputs off at once (Cytron: PWM 0 and the valve relay off; IBT2: both PWM 0
// and the enable low), from the watchdog warning and the crash hook. The Keya is told on the CAN bus
// (not from the watchdog warning: it could interrupt the loop in the middle of a CAN write).
static void steerPinsOff()
{
    if (isKeya) return;
    analogWrite(PWM1_LPWM, 0);
    if (steerConfig.CytronDriver) digitalWrite(PWM2_RPWM, steerConfig.IsRelayActiveHigh ? 1 : 0);
    else { analogWrite(PWM2_RPWM, 0); digitalWrite(DIR1_RL_ENABLE, 0); }
}

// The warning interrupt: the loop has not fed the watchdog for 1.5 s. Autosteer outputs off now,
// note where the firmware was (the reset follows 0.5 s later).
extern "C" __attribute__((used)) void wdtWarnC(const uint32_t* frame)
{
    steerPinsOff();
    CrashReport.breadcrumb(4, BC_MARK);
    CrashReport.breadcrumb(5, loopStep);
    CrashReport.breadcrumb(6, frame ? frame[6] : 0);     // the stacked return address = the code that was running
    WDOG1_WICR = WDOG_WICR_WIE | WDOG_WICR_WTIS | WDOG_WICR_WICT(WDT_WICT);   // flag cleared (WIE, WICT stay)
    NVIC_DISABLE_IRQ(IRQ_WDOG1);
}

// Entry: pass the stacked frame (main or process stack) to wdtWarnC
#if defined(__arm__)
static void wdtWarnIsr() __attribute__((naked));
static void wdtWarnIsr()
{
    asm volatile(
        "tst lr, #4                    \n"
        "ite eq                        \n"
        "mrseq r0, msp                 \n"
        "mrsne r0, psp                 \n"
        "movw r1, #:lower16:wdtWarnC   \n"
        "movt r1, #:upper16:wdtWarnC   \n"
        "bx r1                         \n");
}
#else
static void wdtWarnIsr() { wdtWarnC(nullptr); }      // (test build on a PC: no stacked frame)
#endif

// ---------------------------------------------------------------- crash (fault): outputs off, the caller noted
// The Teensy core's crash handler (unused_interrupt_vector, startup.c) records the crash and then waits
// (8 s, or until the watchdog resets) with every interrupt off: the watchdog warning above cannot run
// and the valve / motor outputs would keep their last value until the reset. faultHook goes first on a
// HardFault, MemManage, BusFault or UsageFault: outputs off, and in the core's breadcrumbs (they stay
// over the reset) the code that called the crashed function (the stacked LR) and the faulting code
// (the stacked PC); then the core's handler, with the stack as the fault left it.
static volatile uint8_t faultHookRuns = 0;

extern "C" __attribute__((used)) void faultHookC(const uint32_t* frame)
{
    if (faultHookRuns++) return;                          // once: a fault in here goes straight to the core's handler
    CrashReport.breadcrumb(2, frame ? frame[5] : 0);      // stacked LR
    CrashReport.breadcrumb(3, frame ? frame[6] : 0);      // stacked PC
    CrashReport.breadcrumb(1, BC_FAULT);
    steerPinsOff();
    if (isKeya) disableKeyaSteer();                       // speed 0 and disable, on the CAN bus
}

#if defined(__arm__)
// (_ebss and unused_interrupt_vector: the core's linker script and startup.c, named only in the asm)
static void faultHook() __attribute__((naked));
static void faultHook()
{
    // r0 = the stacked frame (main or process stack). The C part only runs with 512 bytes of stack
    // left above the end of RAM1's variables (a stack overflow goes straight to the core's handler).
    // The stack and LR (EXC_RETURN) are as they came when the core's handler starts.
    asm volatile(
        "tst lr, #4                              \n"
        "ite eq                                  \n"
        "mrseq r0, msp                           \n"
        "mrsne r0, psp                           \n"
        "movw r1, #:lower16:_ebss                \n"
        "movt r1, #:upper16:_ebss                \n"
        "add r1, r1, #512                        \n"
        "mov r2, sp                              \n"
        "cmp r2, r1                              \n"
        "blo 1f                                  \n"
        "push {r0, lr}                           \n"
        "movw r1, #:lower16:faultHookC           \n"
        "movt r1, #:upper16:faultHookC           \n"
        "blx r1                                  \n"
        "pop {r0, lr}                            \n"
        "1:                                      \n"
        "movw r1, #:lower16:unused_interrupt_vector \n"
        "movt r1, #:upper16:unused_interrupt_vector \n"
        "bx r1                                   \n");
}

// First thing in setup(), after bootCapture(): the four fault vectors (HardFault, MemManage, BusFault,
// UsageFault) go to faultHook; every other unused vector stays on the core's handler.
void faultHookInstall()
{
    for (int v = 3; v <= 6; v++) _VectorsRam[v] = faultHook;
    asm volatile("dsb" ::: "memory");
}
#else
void faultHookInstall() {}                                // (test build on a PC)
#endif

// End of setup(), and when it is switched on on the web page
void wdtStart()
{
    if (wdtRunning || cfg.wdtOff) return;
    CCM_CCGR3 |= CCM_CCGR3_WDOG1(CCM_CCGR_ON);
    WDOG1_WMCR = 0;                                                  // power-down counter off
    WDOG1_WICR = WDOG_WICR_WIE | WDOG_WICR_WICT(WDT_WICT);
    attachInterruptVector(IRQ_WDOG1, wdtWarnIsr);
    NVIC_SET_PRIORITY(IRQ_WDOG1, 0);
    NVIC_ENABLE_IRQ(IRQ_WDOG1);
    const uint16_t wcr = WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WDBG | WDOG_WCR_WT(WDT_WT);   // SRS, WDA: 0 would reset
    WDOG1_WCR = wcr;
    WDOG1_WCR = wcr | WDOG_WCR_WDE;
    wdtRunning = true;
    wdtFeed();
    StatusLog.line("[boot] Watchdog on: the Teensy restarts if the firmware stops for 2 s");
}

// Before copying a firmware update over the running one (zOTA.ino): the longest timeout, 128 s
void wdtLongTimeout()
{
    if (!wdtRunning) return;
    NVIC_DISABLE_IRQ(IRQ_WDOG1);
    WDOG1_WCR = (uint16_t)((WDOG1_WCR & 0x00FF) | WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WT(0xFF));
    wdtFeed();
}

// ---------------------------------------------------------------- the program says hello
// When a program starts talking to the module (first hello, or after 5 s without one): what it
// should know, on its screen. Once per start: a watchdog / crash restart. Every time: the WAS
// check switched off, the simulator mode on (zHandlers.ino).
static uint32_t helloLastMs = 0;
static bool     bootMsgSent = false;

void safetyNoteHello()
{
    const uint32_t now = millis();
    const bool fresh = helloLastMs == 0 || now - helloLastMs > 5000;
    helloLastMs = now;
    if (!fresh) return;
    char m[64] = "";
    if (!bootMsgSent)
    {
        bootMsgSent = true;
        if (bootWhy == BOOT_WATCHDOG)                             snprintf(m, sizeof(m), "Module restarted by its watchdog");
        else if (bootWhy == BOOT_CRASH || bootWhy == BOOT_CRASH_WDT) snprintf(m, sizeof(m), "Module restarted after a crash");
    }
    if (cfg.wasCheckOff) strncat(m, m[0] ? " - WAS check OFF" : "WAS check OFF (bench setting)", sizeof(m) - strlen(m) - 1);
    if (cfg.simMode)     strncat(m, m[0] ? " - simulator mode" : "Simulator mode: the GPS is not sent", sizeof(m) - strlen(m) - 1);
    if (m[0]) hwMessage(m, 10, true);
}
