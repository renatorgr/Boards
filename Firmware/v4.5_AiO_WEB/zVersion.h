/* zVersion.h - the firmware version and name.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zVersion.h
// The firmware version, in one place. When it changes, the sketch folder and the main .ino get the
// same name as FW_NAME (e.g. v4.5_AiO_WEB/v4.5_AiO_WEB.ino).
// FW_NAME and FW_VERSION show on the web page (Firmware) and in the [boot] / [web] lines.
#ifndef ZVERSION_H
#define ZVERSION_H

#define FW_VERSION       "4.5"
#define FW_NAME          "v" FW_VERSION "_AiO_WEB"

#endif
