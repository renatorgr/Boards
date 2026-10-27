# CLAUDE.md

This file guides Claude Code (claude.ai/code) when working in this repository.

## What is here

Hardware and firmware for the **AgOpenGPS AiO v4.5** autosteer boards (Micro and Standard), made for Renato's
tractors, sprayer and no-till seed drill. Renato is a farmer and a beginner programmer: explain changes simply, and give him
text to paste into GitHub as a raw markdown code block.

| Folder | What | More |
|--------|------|------|
| `Firmware/v4.5_AiO_WEB/` | Teensy 4.1 firmware (Arduino IDE sketch): GNSS, IMU, autosteer, web page, OTA | its `CLAUDE.md` and `README.md` |
| `Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32/` | XIAO ESP32C3 / S3 program for the GPS2 port (own WiFi page) | its `README.md` |
| `Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32_bin/` | the XIAO program compiled | its `README.md` |
| `Firmware/simulator/` | the web pages with a pretend module, published on GitHub Pages by an Action | its `README.md` |
| `Seeder/Seeder_ESP32/` | seed drill section controller, KinCony KC868-A32 (32 relays, network cable) | its `README.md` |
| `Sprayer/Sprayer_ESP32/`, `Sprayer/Sprayer_Nano/` | sprayer section controller (ESP32 relay board; the older Nano) | their `README.md` |
| `AiO_Micro45_XIAO_C3/`, `AiO_Std45_XIAO_C3/` | KiCad adapter boards XIAO → GPS2 socket, with JLCPCB files | their `README.md` |

## Rules for every change

- Code, comments, web pages and READMEs in **English**.
- Keep each file's header: purpose, credits, what changed and by whom, GPL v3 / SPDX line.
- The Teensy firmware **cannot be compiled in Claude's cloud sessions** (pjrc.com, arduino.cc and the PlatformIO
  registry are blocked). Say so in the pull request; Renato compiles before merging. The ESP32 programs
  (`Seeder/Seeder_ESP32`, `Sprayer/Sprayer_ESP32`) can: arduino-cli with the esp32 core fetched from GitHub.
- Do not rename `Firmware/vX.Y_AiO_WEB/` folders or the files in them: the module's web page and the XIAO
  page find new versions on GitHub by those names (see the firmware `CLAUDE.md`, "Common Gotchas").
- The firmware steers a tractor: safety code (WAS check, cut-offs, Keya heartbeat, watchdog) must keep working.
- Pull requests: one topic each, a short plain description of what changes for Renato and how to test it.

## Boards (KiCad)

KiCad 7 installs in the cloud with `apt` (`kicad`, `librsvg2-bin` for PNG renders); `pcbnew` Python works for
edits and DRC. JLCPCB sites are not reachable from the cloud: part stock must be checked by Renato.

## License

The firmware is GPL v3 (`Firmware/v4.5_AiO_WEB/COPYING`). The boards follow the licence of the AgOpenGPS
boards they adapt to.
