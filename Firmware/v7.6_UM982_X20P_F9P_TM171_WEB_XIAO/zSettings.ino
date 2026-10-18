/* zSettings.ino - settings from the web page, kept in the EEPROM.
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zSettings.ino
// Settings that can be changed from the web page (zWeb.ino) without reprogramming the Teensy.
// Stored in the Teensy's EEPROM at SETTINGS_EEPROM_ADDR, away from the AgOpenGPS steer settings
// (addresses 0-63, Autosteer.ino). Firmware updates from the web page keep the EEPROM.
//
// The defaults come from the #defines at the top of zHandlers.ino. A blank or damaged EEPROM
// block (first start, other firmware) gives the defaults.

#include "zSettings.h"   // the WebSettings struct (included first by the main .ino)

WebSettings cfg;                  // the settings in use
bool        cfgFromEeprom = false;
bool        settingsAutoSetNow = false;  // v7.4: this start switched older settings to the automatic mode

static void settingsDefaults(WebSettings& s)
{
    memset(&s, 0, sizeof(s));
    s.magic              = SETTINGS_MAGIC;
    s.version            = SETTINGS_VERSION;
    s.outputMode         = OUT_AUTO;      // v7.4 (was OUT_STANDARD)
    s.classicNoDual      = 0;
    s.hprAcceptFloat     = HPR_ACCEPT_FLOAT;
    s.epochWaitMs        = EPOCH_WAIT_MS;
    s.tm171SwapRollPitch = TM171_SWAP_ROLL_PITCH;
    s.tm171YawSign       = TM171_YAW_SIGN;
    s.xiaoOff            = 0;         // XIAO bridge on: it does nothing until a XIAO says hello
    s.autoSet            = 1;
    s.wasCheckOff        = 0;         // v7.5: WAS check on
    s.wdtOff             = 0;         // v7.5: hardware watchdog on
}

static uint8_t settingsChecksum(const WebSettings& s)
{
    const uint8_t* p = (const uint8_t*)&s;
    uint8_t sum = 0x5A;
    for (size_t i = 0; i < offsetof(WebSettings, checksum); i++) sum = (uint8_t)((sum << 1) | (sum >> 7)) ^ p[i];
    return sum;
}

// Values out of range are replaced by the default (never trust what is in the EEPROM).
static void settingsSanitize(WebSettings& s)
{
    WebSettings d;
    settingsDefaults(d);
    if (s.outputMode > OUT_AUTO)                                     s.outputMode = d.outputMode;
    if (s.classicNoDual > 1)                                         s.classicNoDual = d.classicNoDual;
    if (s.hprAcceptFloat > 1)                                        s.hprAcceptFloat = d.hprAcceptFloat;
    if (s.epochWaitMs < 20 || s.epochWaitMs > 100)                   s.epochWaitMs = d.epochWaitMs;
    if (s.tm171SwapRollPitch > 1)                                    s.tm171SwapRollPitch = d.tm171SwapRollPitch;
    if (s.tm171YawSign != 0 && s.tm171YawSign != 1 && s.tm171YawSign != -1) s.tm171YawSign = d.tm171YawSign;
    if (s.xiaoOff > 1)                                               s.xiaoOff = d.xiaoOff;
    if (s.autoSet > 1)                                               s.autoSet = 1;
    if (s.wasCheckOff > 1)                                           s.wasCheckOff = d.wasCheckOff;   // v7.5
    if (s.wdtOff > 1)                                                s.wdtOff = d.wdtOff;             // v7.5
}

// Side effects of the settings on the running firmware. yawChanged: the TM171 yaw direction
// setting changed (or start-up) - learn / align it again.
void settingsApply(bool yawChanged)
{
    if (!yawChanged) return;
    // TM171 yaw direction: forced, or learned again from the dual heading
    yawSign       = (cfg.tm171YawSign == 0) ? 1.0f : (float)cfg.tm171YawSign;
    yawSignLocked = (cfg.tm171YawSign != 0);
    signHaveLast  = false;
    signCorr      = 0;
    signMag       = 0;
    signSrc       = '-';
    offsetSeeded  = false;            // re-align the TM171 heading (next dual epoch, or the GPS course)
    offsetFrom    = '-';
}

void settingsLoad()
{
    WebSettings s;
    EEPROM.get(SETTINGS_EEPROM_ADDR, s);
    if (s.magic == SETTINGS_MAGIC && s.version == SETTINGS_VERSION && s.checksum == settingsChecksum(s))
    {
        settingsSanitize(s);
        cfg = s;
        cfgFromEeprom = true;
        // v7.4: settings saved by 7.3 or older - the output mode becomes automatic, once (it can be
        // set back to AgOpenWeb or AgOpenGPS on the web page, and stays so)
        if (!cfg.autoSet)
        {
            cfg.outputMode = OUT_AUTO;
            cfg.autoSet    = 1;
            settingsSave();
            settingsAutoSetNow = true;
        }
    }
    else
    {
        settingsDefaults(cfg);
        cfgFromEeprom = false;
    }
    settingsApply(true);
}

void settingsSave()
{
    cfg.magic    = SETTINGS_MAGIC;
    cfg.version  = SETTINGS_VERSION;
    cfg.checksum = settingsChecksum(cfg);
    EEPROM.put(SETTINGS_EEPROM_ADDR, cfg);
    cfgFromEeprom = true;
}
