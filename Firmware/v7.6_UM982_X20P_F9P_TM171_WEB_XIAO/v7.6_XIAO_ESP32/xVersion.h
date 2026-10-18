/* xVersion.h - the program version and the board it is built for.
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware
   UM982_X20P_F9P_TM171_WEB_XIAO (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// xVersion.h
// The XIAO program's version, in one place. It goes with the Teensy firmware of the same number
// (v7.6_UM982_X20P_F9P_TM171_WEB_XIAO): when it changes, the sketch folder and the main .ino get the
// new number too (v7.6_XIAO_ESP32).
// The same program runs on the XIAO ESP32C3 and the XIAO ESP32S3 (same pins, same size): the name it
// shows says which one it was compiled for.
#pragma once
#include "sdkconfig.h"

#define XIAO_FW_VERSION  "7.6"

#if CONFIG_IDF_TARGET_ESP32C3
#define XIAO_CHIP        "ESP32C3"
#define XIAO_CHIP_NICE   "ESP32-C3"
#elif CONFIG_IDF_TARGET_ESP32S3
#define XIAO_CHIP        "ESP32S3"
#define XIAO_CHIP_NICE   "ESP32-S3"
#else
#error "Arduino IDE: Tools > Board: XIAO_ESP32C3 or XIAO_ESP32S3"
#endif

#define XIAO_FW_NAME     "v" XIAO_FW_VERSION "_XIAO_" XIAO_CHIP
