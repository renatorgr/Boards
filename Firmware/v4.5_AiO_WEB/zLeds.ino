/* zLeds.ino - the three LEDs on the front of the board, with the meanings of the AiO 26 firmware.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 24 Oct 2026 by Claude (Anthropic) for renatorgr, after the LED_Status_Guide of
   AgOpenGPS-Official/Firmware_Teensy_AiO_26 (GPL v3).
   25 Oct 2026: the in-between state (orange) is shown as red and green in turn: through the diffused light
   windows of a panel (Renato's RayCorp case) orange cannot be told from red.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The AiO v4.5 board has three two-colour LEDs (a red and a green chip each; both on = orange), one
// Teensy pin per colour. The AiO 26 board has four RGB LEDs, so its scheme is followed as far as two
// colours allow: its blue pulse becomes a short dark flick, and its fourth LED (IMU) has no equivalent
// here (the IMU state is on the web page and in the program's message, PGN 221).
// Orange (both chips on) looks like plain red through the diffused light window of an enclosure panel, so
// the orange state below is shown as red and green in turn, LED_MID_MS each (0 = steady orange).
//
//   PWR/ETH  red     no Ethernet link
//            orange  link, but no program talking to the module (nothing on UDP 8888 for 5 s)
//            green   AgOpenWeb or AgOpenGPS talking
//   GPS      off     no GGA from the receiver for 10 s
//            red     GGA without a fix
//            orange  fix, but not RTK fixed (GPS, DGPS, RTK float)
//            green   RTK fixed
//            flick   corrections (RTCM) arriving from NTRIP or the radio: a short dark flick every second
//   STEER    red     fault: ADS1115 (WAS) not found, WAS voltage out of range, or Keya heartbeat lost
//            orange  ready, autosteer off
//            green   autosteer on
//            flick   the steer button or switch was used: a short dark flick
//
// Before this file the LEDs were written from five places with other meanings (GPS: green = dual heading,
// red = IMU heading, red blinking = IMU lost; PWR/ETH: red = no link, green = link). Now only ledsUpdate()
// writes them, 20 times a second from loop(); the other files only report their state.
// The Teensy's own LED (pin 13) still blinks with every GGA (zHandlers.ino).

#define LED_OFF     0
#define LED_RED     1
#define LED_GREEN   2
#define LED_ORANGE  3
#define LED_TICK_MS       50      // the LEDs are decided this often
#define LED_MID_MS        500     // the orange state shows as red and green in turn, this long each (0 = steady orange)
#define LED_FLICK_MS      120     // how long a dark flick lasts
#define LED_RTCM_EVERY_MS 1000    // one flick per second while corrections arrive
#define LED_RTCM_GONE_MS  2000    // no RTCM byte for this long: no more flicks
#define LED_GGA_GONE_MS   10000   // no GGA for this long: GPS LED off

static uint32_t ledTickMs = 0;
static uint32_t ledGpsFlickMs = 0;     // when the GPS LED's dark flick started (0 = none)
static uint32_t ledSteerFlickMs = 0;   // the same for the STEER LED
static uint8_t  ledSteerSwPrev = 255;  // steerSwitch last seen (255 = not yet)

static void ledPair(uint8_t redPin, uint8_t greenPin, uint8_t colour)
{
    digitalWrite(redPin,   (colour & LED_RED)   ? HIGH : LOW);
    digitalWrite(greenPin, (colour & LED_GREEN) ? HIGH : LOW);
}

// The colour an LED really shows: orange becomes red and green in turn (see LED_MID_MS)
static uint8_t ledShown(uint8_t colour, uint32_t now)
{
#if LED_MID_MS > 0
    if (colour == LED_ORANGE) return ((now / LED_MID_MS) & 1) ? LED_RED : LED_GREEN;
#endif
    return colour;
}

// The steer button was pressed (Autosteer.ino, button mode): a dark flick on the STEER LED
void ledSteerPressed() { ledSteerFlickMs = millis() | 1; }

// From loop(), every pass
void ledsUpdate()
{
    const uint32_t now = millis();
    // the steer switch (switch mode: every change is the driver's hand; in button mode Autosteer.ino calls ledSteerPressed)
    if (steerConfig.SteerSwitch == 1 && ledSteerSwPrev != 255 && steerSwitch != ledSteerSwPrev) ledSteerFlickMs = now | 1;
    ledSteerSwPrev = steerSwitch;
    if (now - ledTickMs < LED_TICK_MS) return;
    ledTickMs = now;

    // PWR/ETH
    uint8_t c;
    if (Ethernet.linkStatus() != LinkON) c = LED_RED;
    else if (statusProgramTalking(now))  c = LED_GREEN;
    else                                 c = LED_ORANGE;
    ledPair(Power_on_LED, Ethernet_Active_LED, ledShown(c, now));

    // GPS: the fix quality of the last GGA (field 6: 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float)
    if (gpsReadyTime == 0 || now - gpsReadyTime > LED_GGA_GONE_MS) c = LED_OFF;
    else
    {
        const char q = fixQuality[0];
        c = (q == '0' || q == '\0') ? LED_RED : (q == '4') ? LED_GREEN : LED_ORANGE;
    }
    const uint32_t rtcmMs = statusRtcmLastMs();
    if (rtcmMs && now - rtcmMs < LED_RTCM_GONE_MS && now - ledGpsFlickMs >= LED_RTCM_EVERY_MS) ledGpsFlickMs = now | 1;
    if (ledGpsFlickMs && now - ledGpsFlickMs < LED_FLICK_MS) c = LED_OFF;
    ledPair(GPSRED_LED, GPSGREEN_LED, ledShown(c, now));

    // STEER
    if (!Autosteer_running || wasOut || (isKeya && keyaLost)) c = LED_RED;
    else if (watchdogTimer < WATCHDOG_THRESHOLD)              c = LED_GREEN;
    else                                                      c = LED_ORANGE;
    if (ledSteerFlickMs && now - ledSteerFlickMs < LED_FLICK_MS) c = LED_OFF;
    ledPair(AUTOSTEER_STANDBY_LED, AUTOSTEER_ACTIVE_LED, ledShown(c, now));
}
