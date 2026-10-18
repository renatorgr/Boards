/* zSafety.ino - autosteer safety: WAS check, why autosteer went off, messages on the program's
   screen (PGN 221), hardware watchdog, why the Teensy started.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 Micro board
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr (v7.5).
   The message format (PGN 221) is the one AgOpenGPS reads (UDPComm, case 221) and AgOpenWeb
   reads the same way ("AiO Board Msgs").
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zSafety.ino
//   - WAS check: the wheel angle sensor voltage must stay between 0.2 and 4.8 V. Outside (wire broken,
//     short circuit, plug out, sensor dead) the WAS reads an end stop and the valve would be driven
//     hard the other way: autosteer is turned off and blocked, like a kickout (only the module's valve
//     / motor output: the tractor's own steering, the steering wheel, is not touched). It can be
//     switched off on the web page for a bench without a WAS.
//   - Why autosteer went off: switch, button, pressure / current / encoder kickout, WAS, the program
//     silent. Shown on the program's screen (PGN 221), in the [status] messages and on the web page.
//   - Hardware watchdog (WDOG1): the Teensy restarts if the loop stops for 2 s. 0.5 s before, an
//     interrupt switches the autosteer outputs (valve / motor) off and notes where the firmware was stuck.
//   - Why the Teensy started (power, restart, watchdog, crash), at boot and on the web page.
// Autosteer.ino calls wasCheck(), steerKick(), steerOnNote() and sensorViewNote() from its 40 Hz
// steering loop; loop() calls wdtFeed().

#include "zSafety.h"      // SteerOff, BootWhy (included first by the main .ino)

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
    if (Ethernet_running) SendUdp(b, (uint8_t)(8 + n), Eth_ipDestination, portDestination);
    hwMsgCount++;
    strncpy(hwMsgLast, text, sizeof(hwMsgLast) - 1);
    hwMsgLast[sizeof(hwMsgLast) - 1] = '\0';
}

// ---------------------------------------------------------------- WAS check
#define WAS_V_PER_COUNT  (6.144f / 16384.0f)   // ADS1115 at +-6.144 V full scale, reading >> 1 (Autosteer.ino)
#define WAS_LOW_V        0.20f
#define WAS_HIGH_V       4.80f
#define WAS_BAD_LOOPS    4                     // out of range 4 steering loops in a row (100 ms) = fault
#define WAS_GOOD_LOOPS   40                    // back in range for 1 s = fault over

int16_t  wasRaw    = 0;          // ADS1115 reading >> 1, before the offset (about 2722 per volt)
float    wasVolts  = 0;
bool     wasOut    = false;      // out of range now
bool     wasBlock  = false;      // autosteer blocked by it (check on)
uint32_t wasFaults = 0;
static uint8_t wasBadN = 0, wasGoodN = 0;

// Called with every WAS reading (40 Hz). True: autosteer must stay off.
bool wasCheck(int16_t raw)
{
    wasRaw   = raw;
    wasVolts = raw * WAS_V_PER_COUNT;
    const bool single = steerConfig.SingleInputWAS;
    const bool bad = single ? (wasVolts < WAS_LOW_V || wasVolts > WAS_HIGH_V)
                            : (wasVolts < -WAS_HIGH_V || wasVolts > WAS_HIGH_V);   // differential: only the ends
    if (bad) { wasGoodN = 0; if (wasBadN < 255) wasBadN++; }
    else     { wasBadN = 0;  if (wasGoodN < 255) wasGoodN++; }

    if (!wasOut && wasBadN >= WAS_BAD_LOOPS)
    {
        wasOut = true;
        wasFaults++;
        if (!cfg.wasCheckOff)
        {
            wasBlock = true;
            StatusLog.line("[status] WAS: %.2f V, out of the %.1f-%.1f V range (wire, plug or sensor?) - autosteer blocked",
                           wasVolts, single ? WAS_LOW_V : -WAS_HIGH_V, WAS_HIGH_V);
            char m[64];
            snprintf(m, sizeof(m), "WAS fault (%.2f V): autosteer blocked", wasVolts);
            hwMessage(m, 8, true);
        }
        else
            StatusLog.line("[status] WAS: %.2f V, out of range - the WAS check is off (bench), autosteer not blocked", wasVolts);
    }
    else if (wasOut && !wasBlock && !cfg.wasCheckOff)              // the check was switched on during a fault
    {
        wasBlock = true;
        StatusLog.line("[status] WAS: %.2f V, out of range - autosteer blocked", wasVolts);
    }
    else if (wasOut && wasGoodN >= WAS_GOOD_LOOPS)
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

// ---------------------------------------------------------------- pressure / current value for the page
float sensorView = 0;                  // the kickout sensor's value (0-255, as sent to the program), or the pressure input
static float    sensorPeak[2] = {0, 0}; // highest value in this 5 s window and in the one before
static uint32_t sensorPeakMs = 0;
static float    pressFilt = 0;

float pressureFiltered(float sample)  // the pressure input with the same filter as the kickout (Autosteer.ino)
{
    pressFilt = pressFilt * 0.6f + sample * 0.4f;
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
        default:          return "";
    }
}

static void offText(char* b, size_t n, uint8_t c, int v)
{
    switch (c)
    {
        case SO_SWITCH:   snprintf(b, n, "steer switch"); break;
        case SO_BUTTON:   snprintf(b, n, "steer button"); break;
        case SO_PRESSURE: snprintf(b, n, "pressure kickout (%d, limit %d)", v, (int)steerConfig.PulseCountMax); break;
        case SO_CURRENT:  snprintf(b, n, "current kickout (%d, limit %d)", v, (int)steerConfig.PulseCountMax); break;
        case SO_ENCODER:  snprintf(b, n, "wheel encoder kickout (%d pulses)", v); break;
        case SO_WAS:      snprintf(b, n, "WAS fault (%.2f V)", wasVolts); break;
        case SO_SILENT:   snprintf(b, n, "no steering data from the program for 2.5 s"); break;
        case SO_PROGRAM:  snprintf(b, n, "by the program"); break;
        case SO_SPEED:    snprintf(b, n, "speed below 0.1 km/h"); break;
        default:          snprintf(b, n, "other"); break;
    }
}

// Called at the end of every steering loop (40 Hz) with "steering now"
void steerOnNote(bool on)
{
    if (on == steerWasOn) return;
    steerWasOn = on;
    const uint32_t now = millis();
    if (on) { kickCause = SO_NONE; return; }

    uint8_t c;
    int v = 0;
    if (wasBlock)                                           c = SO_WAS;
    else if (kickCause != SO_NONE && now - kickMs < 500)    { c = kickCause; v = kickValue; }
    else if (pgn254Ms == 0 || now - pgn254Ms > 1000)        c = SO_SILENT;
    else if (steerSwitch == 1 && steerConfig.SteerSwitch)   c = SO_SWITCH;
    else if (steerSwitch == 1 && steerConfig.SteerButton)   c = SO_BUTTON;
    else if (!(guidanceStatus & 1) || steerSwitch == 1)     c = SO_PROGRAM;   // no switch / button: it follows the program's button
    else if (gpsSpeed < 0.1f)                               c = SO_SPEED;
    else                                                    c = SO_OTHER;
    offCause = c; offMs = now; offValue = v;
    if (!SO_FROM_MODULE(c)) return;                         // the program knows why

    offModuleCount++;
    char t[72];
    offText(t, sizeof(t), c, v);
    StatusLog.line("[status] Autosteer OFF: %s", t);
    if (c == SO_WAS) return;                                // wasCheck() has put it on the program's screen
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
uint8_t  bootWhy     = BOOT_UNKNOWN;
uint32_t bootSrsr    = 0;
uint32_t bootFaultPc = 0, bootCfsr = 0; // crash: the faulting code, the fault status
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
        case 6:  return "TM171";
        case 7:  return "XIAO";
        case 8:  return "GNSS output";
        case 9:  return "status";
        case 10: return "web page";
        case 11: return "steering";
        case 12: return "Ethernet link";
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
    }
    volatile struct crashreport_breadcrumbs_struct* bc = (struct crashreport_breadcrumbs_struct*)0x2027FFC0;
    const bool marked = (bootSrsr & SRC_SRSR_WDOG_RST_B) && (bc->bitmask & 0x38) == 0x38 && bc->value[3] == BC_MARK;
    if (marked) { bootStep = (uint8_t)bc->value[4]; bootStuckPc = bc->value[5]; }
    bc->bitmask &= ~0x38u;                                       // breadcrumbs 4, 5, 6 used
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
    switch (bootWhy)
    {
        case BOOT_POWER:     StatusLog.line("[boot] Start: power on (or the Teensy's button, or a USB upload)"); break;
        case BOOT_UPLOAD:    StatusLog.line("[boot] Start: after programming (JTAG reset)"); break;
        case BOOT_RESTART:   StatusLog.line("[boot] Start: restart by software (web page, firmware update, or the program's subnet change)"); break;
        case BOOT_WATCHDOG:  StatusLog.line("[boot] Start: WATCHDOG - the firmware stopped for 2 s%s. Please send this line", stuck); break;
        case BOOT_CRASH:     StatusLog.line("[boot] Start: CRASH - the firmware stopped at 0x%08lX (fault status 0x%08lX), restarted. Please send this line",
                                            (unsigned long)bootFaultPc, (unsigned long)bootCfsr); break;
        case BOOT_CRASH_WDT: StatusLog.line("[boot] Start: CRASH - the firmware stopped at 0x%08lX (fault status 0x%08lX), restarted by the watchdog. Please send this line",
                                            (unsigned long)bootFaultPc, (unsigned long)bootCfsr); break;
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

// The warning interrupt: the loop has not fed the watchdog for 1.5 s. Autosteer outputs off now,
// note where the firmware was (the reset follows 0.5 s later).
extern "C" __attribute__((used)) void wdtWarnC(const uint32_t* frame)
{
    if (!isKeya)
    {
        analogWrite(PWM1_LPWM, 0);
        if (steerConfig.CytronDriver) digitalWrite(PWM2_RPWM, steerConfig.IsRelayActiveHigh ? 1 : 0);
        else { analogWrite(PWM2_RPWM, 0); digitalWrite(DIR1_RL_ENABLE, 0); }
    }
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
// check switched off.
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
    if (m[0]) hwMessage(m, 10, true);
}
