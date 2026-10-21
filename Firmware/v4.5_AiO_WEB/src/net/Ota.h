/* Ota.h - firmware update from the web page (Ota.cpp).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Ota.h
// What the web server (zWeb.ino) calls to receive, check and install a new firmware.
#ifndef OTA_H
#define OTA_H

#include <stddef.h>
#include <stdint.h>

// Kept in the image (printed in /api/status) so a new firmware of this family can be recognised
extern const char otaMarker[];    // OTA_MARKER
extern const char otaFlashId[];   // "fw_teensy41"

// Can an update be stored now? nullptr = yes, else why not
const char* otaBegin();
// Feed the bytes of the .hex file as they arrive. nullptr = fine so far, else the error
const char* otaFeed(const uint8_t* d, size_t n);
// After the last byte: every check. nullptr when the image may be installed; *size = image size in bytes.
// force = install even without OTA_MARKER (another firmware)
const char* otaFinish(bool force, uint32_t* size);
// Copy the checked image over the running firmware and restart. Does not return.
void otaInstall(uint32_t size);

// One hex digit to its value, -1 if it is not one (also decodes %XX in web forms, zWeb.ino)
int hexNib(char c);

#endif
